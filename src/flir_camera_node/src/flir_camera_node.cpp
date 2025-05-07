#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

#include <opencv2/opencv.hpp>

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include <thread>
#include <dynamic_reconfigure/server.h>
#include <flir_camera/FlirConfig.h>
#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include <xmlrpcpp/XmlRpcValue.h>
#include <stdexcept>
#include <atomic>

// ---------- helper for GenApi enumerations ----------
namespace
{
    inline void setEnum(Spinnaker::GenApi::INodeMap &nm,
                        const char *enumName,
                        const char *entryName)
    {
        using namespace Spinnaker::GenApi;
        CEnumerationPtr e = nm.GetNode(enumName);
        if (!e || !IsWritable(e))
            throw std::runtime_error(std::string("Enum '") + enumName + "' not writable");
        CEnumEntryPtr v = e->GetEntryByName(entryName);
        if (!IsReadable(v))
            throw std::runtime_error(std::string("Entry '") + entryName + "' not readable");
        e->SetIntValue(v->GetValue());
    }
} // anonymous namespace

// ----------------------------------------------------
//  CameraHandler declaration + definition in one block
// ----------------------------------------------------
struct CamConfig
{
    std::string serial;
    std::string name;
    double framerate;
    int width, height;
    bool primary;
};

class CameraHandler
{
public:
    CameraHandler(Spinnaker::CameraPtr cam,
                  ros::NodeHandle &nh,
                  const CamConfig &cfg);
    void spin();
    void stop();
    ~CameraHandler();

private:
    // Dynamic reconfigure callback
    void reconfigureCallback(flir_camera::FlirConfig &config, uint32_t level);

    Spinnaker::CameraPtr cam_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    ros::Publisher info_pub_;
    camera_info_manager::CameraInfoManager cinfo_;
    std::string topic_;
    CamConfig cfg_;
    std::atomic<bool> running_{true};
    std::thread worker_;

    // Dynamic reconfigure server
    dynamic_reconfigure::Server<flir_camera::FlirConfig> dyn_server_;
    dynamic_reconfigure::Server<flir_camera::FlirConfig>::CallbackType dyn_cb_;
};
{
public:
    CameraHandler(Spinnaker::CameraPtr cam,
                  ros::NodeHandle & nh,
                  const CamConfig &cfg);
    void spin();
    void stop();
    ~CameraHandler();

private:
    Spinnaker::CameraPtr cam_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    ros::Publisher info_pub_;
    camera_info_manager::CameraInfoManager cinfo_;
    std::string topic_;
    CamConfig cfg_;
    std::atomic<bool> running_{true};
    std::thread worker_;
};

// -------- constructor with exception handling --------
CameraHandler::CameraHandler(Spinnaker::CameraPtr cam,
                             ros::NodeHandle &nh,
                             const CamConfig &cfg)
try
    : cam_(cam), it_(nh), cfg_(cfg), cinfo_(nh, cfg.name)
{
    // 1: Init camera so GenApi nodes become available
    cam_->Init();

    auto &nodemap = cam_->GetNodeMap();
    using namespace Spinnaker::GenApi;

    // 2: Apply resolution
    {
        auto ptrW = nodemap.GetNode("Width");
        auto ptrH = nodemap.GetNode("Height");
        if (ptrW && ptrH && IsWritable(ptrW) && IsWritable(ptrH))
        {
            ptrW->SetValue(cfg_.width);
            ptrH->SetValue(cfg_.height);
        }
    }

    // 3: Frame-rate
    {
        auto ptrRateEnable = nodemap.GetNode("AcquisitionFrameRateEnable");
        auto ptrRate = nodemap.GetNode("AcquisitionFrameRate");
        if (ptrRateEnable && ptrRate && IsWritable(ptrRateEnable))
        {
            ptrRateEnable->SetValue(true);
            ptrRate->SetValue(cfg_.framerate);
        }
    }

    // 4: Pixel format → Mono8
    {
        auto pixelFmt = nodemap.GetNode("PixelFormat");
        if (IsWritable(pixelFmt))
        {
            auto mono8 = pixelFmt->GetEntryByName("Mono8");
            if (IsAvailable(mono8))
                pixelFmt->SetIntValue(mono8->GetValue());
        }
    }

    // ** Enable chunk timestamp for accurate hardware sync **
    {
        auto chunkMode = nodemap.GetNode("ChunkModeActive");
        if (chunkMode && IsWritable(chunkMode))
            chunkMode->SetValue(true);

        auto chunkSel = nodemap.GetNode("ChunkSelector");
        if (chunkSel && IsWritable(chunkSel))
        {
            auto tsEntry = chunkSel->GetEntryByName("Timestamp");
            if (IsAvailable(tsEntry))
                chunkSel->SetIntValue(tsEntry->GetValue());
        }

        auto chunkEnable = nodemap.GetNode("ChunkEnable");
        if (chunkEnable && IsWritable(chunkEnable))
            chunkEnable->SetValue(true);
    }

    // ** Setup dynamic reconfigure callback **
    dyn_cb_ = boost::bind(&CameraHandler::reconfigureCallback, this, _1, _2);
    dyn_server_.setCallback(dyn_cb_);

    // 5: Hardware sync
    if (cfg_.primary)
    {
        setEnum(nodemap, "LineSelector", "Line3");
        setEnum(nodemap, "LineMode", "Output");
        setEnum(nodemap, "LineSource", "ExposureActive");
        ROS_INFO("Camera %s: configured as PRIMARY sync on Line3",
                 cfg_.name.c_str());
    }
    else
    {
        setEnum(nodemap, "TriggerSelector", "FrameStart");
        setEnum(nodemap, "TriggerSource", "Line3");
        setEnum(nodemap, "TriggerMode", "On");
        ROS_INFO("Camera %s: configured as SECONDARY sync (external trigger)",
                 cfg_.name.c_str());
    }

    // 6: Begin acquisition
    cam_->BeginAcquisition();

    // 7: ROS publishers
    topic_ = "/camera/" + cfg_.name + "/image_raw";
    pub_ = it_.advertise(topic_, 5);
    info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("/camera/" + cfg_.name + "/camera_info", 5);

    ROS_INFO("Started '%s' @ %dx%d @ %.1f Hz → %s",
             cfg_.name.c_str(),
             cfg_.width, cfg_.height, cfg_.framerate,
             topic_.c_str());

    // Launch worker thread
    worker_ = std::thread(&CameraHandler::spin, this);
}
catch (const Spinnaker::Exception &ex)
{
    ROS_FATAL("Spinnaker exception initializing camera %s: %s",
              cfg.name.c_str(), ex.what());
    throw;
}

// -------- worker thread with timeout & exception safety ------
void CameraHandler::spin()
{
    try
    {
        while (ros::ok() && running_)
        {
            Spinnaker::ImagePtr img;
            try
            {
                img = cam_->GetNextImage(1000); // timeout in ms
            }
            catch (const Spinnaker::TimeoutException &)
            {
                continue;
            }

            if (img->IsIncomplete())
            {
                img->Release();
                continue;
            }

            // Build ROS Image message directly (single copy)
            sensor_msgs::Image msg;
            msg.header.frame_id = cfg_.name;
            uint64_t t_ns = img->GetTimeStamp();
            msg.header.stamp.fromNSec(t_ns);
            msg.height = img->GetHeight();
            msg.width = img->GetWidth();
            msg.encoding = sensor_msgs::image_encodings::MONO8;
            msg.step = msg.width; // mono8 → 1 byte/pixel
            size_t sz = msg.height * msg.step;
            msg.data.assign(static_cast<uint8_t *>(img->GetData()),
                            static_cast<uint8_t *>(img->GetData()) + sz);

            pub_.publish(msg);

            // Publish CameraInfo
            sensor_msgs::CameraInfo info = cinfo_.getCameraInfo();
            info.header = msg.header;
            info_pub_.publish(info);

            img->Release();
        }
    }
    catch (const Spinnaker::Exception &ex)
    {
        ROS_ERROR("Exception in CameraHandler::spin for %s: %s",
                  cfg_.name.c_str(), ex.what());
    }
}
}

// -------- dynamic reconfigure callback --------
void CameraHandler::reconfigureCallback(flir_camera::FlirConfig &config, uint32_t level)
{
    auto &nm = cam_->GetNodeMap();
    using namespace Spinnaker::GenApi;
    // Enable and set decimation
    setEnum(nm, "DecimationMode", "On");
    {
        auto decH = nm.GetNode("DecimationHorizontal");
        if (decH && IsWritable(decH))
            decH->SetValue(config.decimation_x);
        auto decV = nm.GetNode("DecimationVertical");
        if (decV && IsWritable(decV))
            decV->SetValue(config.decimation_y);
    }
    // Exposure (manual)
    {
        auto expAuto = nm.GetNode("ExposureAuto");
        if (expAuto && IsWritable(expAuto))
            expAuto->SetIntValue(
                CEnumerationPtr(expAuto)->GetEntryByName("Off")->GetValue());
        auto expTime = nm.GetNode("ExposureTime");
        if (expTime && IsWritable(expTime))
            expTime->SetValue(config.exposure);
    }
    // Gain (manual)
    {
        auto gainAuto = nm.GetNode("GainAuto");
        if (gainAuto && IsWritable(gainAuto))
            gainAuto->SetIntValue(
                CEnumerationPtr(gainAuto)->GetEntryByName("Off")->GetValue());
        auto gainNode = nm.GetNode("Gain");
        if (gainNode && IsWritable(gainNode))
            gainNode->SetValue(config.gain);
    }
    // Frame rate
    {
        auto frEnable = nm.GetNode("AcquisitionFrameRateEnable");
        auto frVal = nm.GetNode("AcquisitionFrameRate");
        if (frEnable && frVal && IsWritable(frEnable))
        {
            frEnable->SetValue(true);
            frVal->SetValue(config.framerate);
        }
    }
    ROS_INFO("Reconfigure: decH=%d decV=%d exp=%.1f gain=%.1f fps=%.1f",
             config.decimation_x,
             config.decimation_y,
             config.exposure,
             config.gain,
             config.framerate);
}

// -------- stop acquisition cleanly ------------------------
void CameraHandler::stop()
{
    running_ = false;
    try
    {
        cam_->EndAcquisition();
        cam_->DeInit();
    }
    catch (const Spinnaker::Exception &ex)
    {
        ROS_ERROR("Error stopping camera %s: %s",
                  cfg_.name.c_str(), ex.what());
    }
}

// -------- destructor --------------------------------------
CameraHandler::~CameraHandler()
{
    stop();
    if (worker_.joinable())
        worker_.join();
}

// ----------------------------------------------------
//  main()
// ----------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "flir_camera_node");
    ros::NodeHandle nh("~");

    // 1) YAML → vector<CamConfig>
    XmlRpc::XmlRpcValue cam_list;
    if (!nh.getParam("camera_config/cameras", cam_list) ||
        cam_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_FATAL("camera_config/cameras param not found or is not an array");
        return 1;
    }

    auto require = [&](XmlRpc::XmlRpcValue &v, const char *key) -> XmlRpc::XmlRpcValue &
    {
        if (!v.hasMember(key))
            throw std::runtime_error(std::string("Missing YAML key: ") + key);
        return v[key];
    };

    std::unordered_map<std::string, CamConfig> config_map;
    int primary_count = 0;
    for (int i = 0; i < cam_list.size(); ++i)
    {
        auto &e = cam_list[i];
        CamConfig c;
        c.serial = static_cast<std::string>(require(e, "serial"));
        c.name = static_cast<std::string>(require(e, "name"));
        c.framerate = static_cast<double>(require(e, "framerate"));
        c.width = static_cast<int>(require(e, "width"));
        c.height = static_cast<int>(require(e, "height"));
        if (!e.hasMember("primary"))
            throw std::runtime_error("Missing YAML key: primary");
        c.primary = static_cast<bool>(e["primary"]);
        if (c.primary)
            ++primary_count;

        config_map[c.serial] = c;
        ROS_INFO("YAML cfg: %s → serial=%s, %dx%d@%.1f Hz (primary=%s)",
                 c.name.c_str(), c.serial.c_str(),
                 c.width, c.height, c.framerate,
                 c.primary ? "true" : "false");
    }

    if (primary_count != 1)
    {
        ROS_FATAL("Exactly one camera must have primary: true (found %d)", primary_count);
        return 1;
    }

    // 2) Enumerate cameras and spawn handlers
    auto system = Spinnaker::System::GetInstance();
    auto camList = system->GetCameras();
    unsigned int total = camList.GetSize();
    ROS_INFO("SDK reports %u cameras connected", total);

    std::vector<std::shared_ptr<CameraHandler>> handlers;
    handlers.reserve(total);
    for (unsigned i = 0; i < total; ++i)
    {
        auto cam = camList.GetByIndex(i);
        using namespace Spinnaker::GenApi;
        auto strPtr = cam->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");
        std::string serial = (strPtr && IsReadable(strPtr))
                                 ? std::string(strPtr->GetValue())
                                 : std::string();

        auto it = config_map.find(serial);
        if (it != config_map.end())
        {
            try
            {
                handlers.emplace_back(
                    std::make_shared<CameraHandler>(cam, nh, it->second));
            }
            catch (const std::exception &ex)
            {
                ROS_FATAL("Failed to initialize camera %s: %s",
                          it->second.name.c_str(), ex.what());
            }
        }
    }

    if (handlers.empty())
    {
        ROS_FATAL("No configured cameras matched! Exiting.");
        return 1;
    }

    ros::spin();

    // 3) Clean shutdown
    for (auto &h : handlers)
        h->stop();

    camList.Clear();
    system->ReleaseInstance();
    return 0;
}

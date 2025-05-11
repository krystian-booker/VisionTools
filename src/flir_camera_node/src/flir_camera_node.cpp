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
#include <mutex>
#include <cstring>
#include <dynamic_reconfigure/server.h>
#include <flir_camera_node/FlirConfig.h>
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
}

// ----------------------------------------------------
//  Role enumeration
// ----------------------------------------------------
enum class Role
{
    PRIMARY,
    SECONDARY,
    SOLO
};

// ----------------------------------------------------
//  CameraHandler declaration + definition
// ----------------------------------------------------
struct CamConfig
{
    std::string serial;
    std::string name;
    double framerate;
    Role role;
    std::string video_mode;
    double exposure;
    double gain;
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
    void reconfigureCallback(flir_camera_node::FlirConfig &config, uint32_t level);

    Spinnaker::CameraPtr cam_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    ros::Publisher info_pub_;
    ros::NodeHandle cam_nh_;
    camera_info_manager::CameraInfoManager cinfo_;
    std::string topic_;
    CamConfig cfg_;
    std::atomic<bool> running_{true};

    // Thread-safety for GenApi and image acquisition
    std::mutex cam_mutex_;

    // Timestamp base mapping
    ros::Time base_time_;
    bool base_time_initialized_{false};

    // Pre-allocated messages
    sensor_msgs::ImagePtr img_msg_ptr_;
    sensor_msgs::CameraInfoPtr info_msg_ptr_;
    std::thread worker_;

    // Dynamic reconfigure server
    dynamic_reconfigure::Server<flir_camera_node::FlirConfig> dyn_server_;
    dynamic_reconfigure::Server<flir_camera_node::FlirConfig>::CallbackType dyn_cb_;
};

// -------- constructor with exception handling --------
CameraHandler::CameraHandler(Spinnaker::CameraPtr cam,
                             ros::NodeHandle &nh,
                             const CamConfig &cfg)
try
    : cam_(cam),
      it_(nh),
      cfg_(cfg),
      cam_nh_(nh, cfg.name),
      cinfo_(cam_nh_, cfg.name),
      dyn_server_(cam_nh_)
{
    // Init camera so GenApi nodes become available
    cam_->Init();
    auto &nodemap = cam_->GetNodeMap();
    using namespace Spinnaker::GenApi;

    // Video mode
    {
        setEnum(nodemap, "VideoMode", cfg_.video_mode.c_str());
    }

    // Frame-rate
    {
        CBooleanPtr ptrRateEnable = nodemap.GetNode("AcquisitionFrameRateEnable");
        CFloatPtr ptrRate = nodemap.GetNode("AcquisitionFrameRate");
        if (ptrRateEnable && ptrRate && IsWritable(ptrRateEnable))
        {
            ptrRateEnable->SetValue(true);
            ptrRate->SetValue(cfg_.framerate);
        }
    }

    // Exposure
    {
        using namespace Spinnaker::GenApi;
        CEnumerationPtr expAuto = nodemap.GetNode("ExposureAuto");
        if (expAuto && IsWritable(expAuto))
            expAuto->SetIntValue(
                CEnumEntryPtr(expAuto->GetEntryByName("Off"))->GetValue());
        CFloatPtr expTime = nodemap.GetNode("ExposureTime");
        if (expTime && IsWritable(expTime))
            expTime->SetValue(cfg_.exposure);
    }

    // Gain
    {
        using namespace Spinnaker::GenApi;
        CEnumerationPtr gainAuto = nodemap.GetNode("GainAuto");
        if (gainAuto && IsWritable(gainAuto))
            gainAuto->SetIntValue(
                CEnumEntryPtr(gainAuto->GetEntryByName("Off"))->GetValue());
        CFloatPtr gainNode = nodemap.GetNode("Gain");
        if (gainNode && IsWritable(gainNode))
            gainNode->SetValue(cfg_.gain);
    }

    // Pixel format → Mono8
    {
        CEnumerationPtr pixelFmt = nodemap.GetNode("PixelFormat");
        if (pixelFmt && IsWritable(pixelFmt))
        {
            CEnumEntryPtr mono8 = pixelFmt->GetEntryByName("Mono8");
            if (mono8 && IsReadable(mono8))
                pixelFmt->SetIntValue(mono8->GetValue());
        }
    }

    // Enable chunk timestamp
    {
        CBooleanPtr chunkMode = nodemap.GetNode("ChunkModeActive");
        if (chunkMode && IsWritable(chunkMode))
            chunkMode->SetValue(true);

        CEnumerationPtr chunkSel = nodemap.GetNode("ChunkSelector");
        if (chunkSel && IsWritable(chunkSel))
        {
            CEnumEntryPtr tsEntry = chunkSel->GetEntryByName("Timestamp");
            if (tsEntry && IsReadable(tsEntry))
                chunkSel->SetIntValue(tsEntry->GetValue());
        }

        CBooleanPtr chunkEnable = nodemap.GetNode("ChunkEnable");
        if (chunkEnable && IsWritable(chunkEnable))
            chunkEnable->SetValue(true);
    }

    // Dynamic reconfigure
    dyn_cb_ = boost::bind(&CameraHandler::reconfigureCallback, this, _1, _2);
    dyn_server_.setCallback(dyn_cb_);

    // Hardware sync / mode based on role
    switch (cfg_.role)
    {
    case Role::PRIMARY:
        setEnum(nodemap, "LineSelector", "Line3");
        setEnum(nodemap, "LineMode", "Output");
        setEnum(nodemap, "LineSource", "ExposureActive");
        ROS_INFO("Camera %s: configured as PRIMARY sync on Line3",
                 cfg_.name.c_str());
        break;

    case Role::SECONDARY:
        setEnum(nodemap, "TriggerSelector", "FrameStart");
        setEnum(nodemap, "TriggerSource", "Line3");
        setEnum(nodemap, "TriggerMode", "On");
        ROS_INFO("Camera %s: configured as SECONDARY sync (external trigger)",
                 cfg_.name.c_str());
        break;

    case Role::SOLO:
        // No hardware sync
        ROS_INFO("Camera %s: configured as SOLO (independent)",
                 cfg_.name.c_str());
        break;
    }

    cam_->BeginAcquisition();

    // ROS publishers
    topic_ = "/camera/" + cfg_.name + "/image_raw";
    pub_ = it_.advertise(topic_, 1);
    info_pub_ = cam_nh_.advertise<sensor_msgs::CameraInfo>(
        "camera_info", 1);

    // pre-allocate message buffers
    img_msg_ptr_ = boost::make_shared<sensor_msgs::Image>();
    info_msg_ptr_ = boost::make_shared<sensor_msgs::CameraInfo>(
        cinfo_.getCameraInfo());

    ROS_INFO("Started '%s' @%.1f Hz → %s",
             cfg_.name.c_str(),
             cfg_.framerate,
             topic_.c_str());

    // launch worker thread
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
            {
                std::lock_guard<std::mutex> lk(cam_mutex_);
                img = cam_->GetNextImage(1000); // timeout in ms
            }

            if (!img || img->IsIncomplete())
            {
                if (img)
                    img->Release();
                continue;
            }

            // Initialize base_time_ on first image
            uint64_t t_ns = img->GetTimeStamp();
            if (!base_time_initialized_)
            {
                base_time_ = ros::Time::now() - ros::Duration().fromNSec(t_ns);
                base_time_initialized_ = true;
            }

            // Fill and publish Image
            img_msg_ptr_->header.frame_id = cfg_.name;
            img_msg_ptr_->header.stamp =
                base_time_ + ros::Duration().fromNSec(t_ns);
            img_msg_ptr_->height = img->GetHeight();
            img_msg_ptr_->width = img->GetWidth();
            img_msg_ptr_->encoding = sensor_msgs::image_encodings::MONO8;
            img_msg_ptr_->step = img_msg_ptr_->width;
            size_t sz = img_msg_ptr_->height * img_msg_ptr_->step;
            img_msg_ptr_->data.resize(sz);
            std::memcpy(img_msg_ptr_->data.data(), img->GetData(), sz);
            pub_.publish(img_msg_ptr_);

            // Publish cached CameraInfo (only header changes)
            info_msg_ptr_->header = img_msg_ptr_->header;
            info_pub_.publish(info_msg_ptr_);

            img->Release();
        }
    }
    catch (const Spinnaker::Exception &ex)
    {
        ROS_ERROR("Exception in CameraHandler::spin for %s: %s",
                  cfg_.name.c_str(), ex.what());
    }
}

// -------- dynamic reconfigure callback --------
void CameraHandler::reconfigureCallback(flir_camera_node::FlirConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure CB: will set exp=%.1f gain=%.1f fps=%.1f",
        config.exposure, config.gain, config.framerate);
        
    std::lock_guard<std::mutex> lk(cam_mutex_);
    auto &nm = cam_->GetNodeMap();
    using namespace Spinnaker::GenApi;

    // Exposure (manual)
    {
        CEnumerationPtr expAuto = nm.GetNode("ExposureAuto");
        if (expAuto && IsWritable(expAuto))
            expAuto->SetIntValue(
                CEnumEntryPtr(expAuto->GetEntryByName("Off"))->GetValue());
        CFloatPtr expTime = nm.GetNode("ExposureTime");
        if (expTime && IsWritable(expTime))
            expTime->SetValue(config.exposure);
    }

    // Gain (manual)
    {
        CEnumerationPtr gainAuto = nm.GetNode("GainAuto");
        if (gainAuto && IsWritable(gainAuto))
            gainAuto->SetIntValue(
                CEnumEntryPtr(gainAuto->GetEntryByName("Off"))->GetValue());
        CFloatPtr gainNode = nm.GetNode("Gain");
        if (gainNode && IsWritable(gainNode))
            gainNode->SetValue(config.gain);
    }

    // Frame rate
    {
        CBooleanPtr frEnable = nm.GetNode("AcquisitionFrameRateEnable");
        CFloatPtr frVal = nm.GetNode("AcquisitionFrameRate");
        if (frEnable && frVal && IsWritable(frEnable))
        {
            frEnable->SetValue(true);
            frVal->SetValue(config.framerate);
        }
    }

    ROS_INFO("Reconfigure: exp=%.1f gain=%.1f fps=%.1f",
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flir_camera_node_node");
    ros::NodeHandle nh("~");

    auto readDouble = [&](XmlRpc::XmlRpcValue &v,
                          const std::string &key,
                          double default_val) -> double
    {
        if (!v.hasMember(key))
            return default_val;

        XmlRpc::XmlRpcValue &x = v[key];
        switch (x.getType())
        {
        case XmlRpc::XmlRpcValue::TypeDouble:
            return static_cast<double>(x);
        case XmlRpc::XmlRpcValue::TypeInt:
            return static_cast<int>(x);
        default:
            throw std::runtime_error(
                "Param '" + key + "' has wrong type (must be int or double)");
        }
    };

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
    int secondary_count = 0;

    for (int i = 0; i < cam_list.size(); ++i)
    {
        auto &e = cam_list[i];
        CamConfig c;
        c.serial = static_cast<std::string>(require(e, "serial"));
        c.name = static_cast<std::string>(require(e, "name"));
        c.framerate = readDouble(e, "framerate", 30.0);
        c.exposure = readDouble(e, "exposure", 10000.0);
        c.gain = readDouble(e, "gain", 1.0);

        // Video Mode
        if (e.hasMember("video_mode"))
            c.video_mode = static_cast<std::string>(e["video_mode"]);
        else
            c.video_mode = "Mode0";

        // Shutter mode
        std::string role_str = static_cast<std::string>(require(e, "role"));
        if (role_str == "primary")
            c.role = Role::PRIMARY;
        else if (role_str == "secondary")
            c.role = Role::SECONDARY;
        else if (role_str == "solo")
            c.role = Role::SOLO;
        else
            throw std::runtime_error("Invalid role: " + role_str);

        if (c.role == Role::PRIMARY)
            ++primary_count;
        else if (c.role == Role::SECONDARY)
            ++secondary_count;

        // Store config
        config_map[c.serial] = c;

        // Print config
        ROS_INFO(
            "YAML cfg: %s → serial=%s @%.1f Hz (role=%s, video_mode=%s, exposure=%.1fus, gain=%.1f)",
            c.name.c_str(),
            c.serial.c_str(),
            c.framerate,
            role_str.c_str(),
            c.video_mode.c_str(),
            c.exposure,
            c.gain);
    }

    // only enforce “exactly one primary” if we have any secondaries
    if (secondary_count > 0 && primary_count != 1)
    {
        ROS_FATAL("When secondaries are present, exactly one camera must be primary (found %d)", primary_count);
        return 1;
    }

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
        CStringPtr strPtr = cam->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");
        std::string serial = "";
        if (strPtr && Spinnaker::GenApi::IsReadable(strPtr))
        {
            serial = strPtr->GetValue();
        }

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

    for (auto &h : handlers)
        h->stop();

    camList.Clear();
    system->ReleaseInstance();
    return 0;
}

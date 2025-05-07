#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include <thread>
#include <vector>
#include <memory>
#include <string>
#include <map>
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
            throw std::runtime_error(std::string("Enum ‘") + enumName + "’ not writable");
        CEnumEntryPtr v = e->GetEntryByName(entryName);
        if (!IsReadable(v))
            throw std::runtime_error(std::string("Entry ‘") + entryName + "’ not readable");
        e->SetIntValue(v->GetValue());
    }
} // anonymous namespace
// ----------------------------------------------------

// ----------------------------------------------------
//  CameraHandler declaration + definition in one block
// ----------------------------------------------------
struct CamConfig
{
    std::string serial;
    std::string name;
    double framerate;
    int width, height;
    std::string sync; // "primary" or "secondary"
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
    Spinnaker::CameraPtr cam_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    std::string topic_;
    CamConfig cfg_;
    std::atomic<bool> running_{true};
};

// -------- constructor with exception handling --------
CameraHandler::CameraHandler(Spinnaker::CameraPtr cam,
                             ros::NodeHandle &nh,
                             const CamConfig &cfg)
try
    : cam_(cam), it_(nh), cfg_(cfg)
{
    // 1: Init camera so GenApi nodes become available
    cam_->Init();

    // 2: Apply resolution
    auto &nodemap = cam_->GetNodeMap();
    using namespace Spinnaker::GenApi;
    auto ptrW = nodemap.GetNode("Width");
    auto ptrH = nodemap.GetNode("Height");
    if (ptrW && ptrH && IsWritable(ptrW) && IsWritable(ptrH))
    {
        ptrW->SetValue(cfg_.width);
        ptrH->SetValue(cfg_.height);
    }

    // 3: Frame-rate
    auto ptrRateEnable = nodemap.GetNode("AcquisitionFrameRateEnable");
    auto ptrRate = nodemap.GetNode("AcquisitionFrameRate");
    if (ptrRateEnable && ptrRate && IsWritable(ptrRateEnable))
    {
        ptrRateEnable->SetValue(true);
        ptrRate->SetValue(cfg_.framerate);
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

    // 5: Hardware sync
    if (cfg_.sync == "primary")
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

    // 7: ROS publisher
    topic_ = "/camera/" + cfg_.name + "/image_raw";
    pub_ = it_.advertise(topic_, 5);
    ROS_INFO("Started '%s' @ %dx%d @ %.1f Hz → %s",
             cfg_.name.c_str(),
             cfg_.width, cfg_.height, cfg_.framerate,
             topic_.c_str());
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

            cv::Mat frame(img->GetHeight(), img->GetWidth(),
                          CV_8UC1, img->GetData());
            // deep copy to own the buffer
            auto msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame.clone())
                           .toImageMsg();
            msg->header.stamp = ros::Time::now();
            pub_.publish(msg);

            img->Release();
        }
    }
    catch (const Spinnaker::Exception &ex)
    {
        ROS_ERROR("Exception in CameraHandler::spin for %s: %s",
                  cfg_.name.c_str(), ex.what());
    }
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
    if (running_)
    {
        try
        {
            cam_->EndAcquisition();
            cam_->DeInit();
        }
        catch (const Spinnaker::Exception &ex)
        {
            ROS_ERROR("Error deinitializing camera %s: %s",
                      cfg_.name.c_str(), ex.what());
        }
    }
}

// ----------------------------------------------------
//  main()
// ----------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "flir_camera_node");
    ros::NodeHandle nh("~");

    // 1) YAML → map<serial, CamConfig>
    XmlRpc::XmlRpcValue cam_list;
    if (!nh.getParam("camera_config/cameras", cam_list) ||
        cam_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_FATAL("camera_config/cameras param not found or is not an array");
        return 1;
    }

    std::unordered_map<std::string, CamConfig> config_map;
    for (int i = 0; i < cam_list.size(); ++i)
    {
        auto &e = cam_list[i];
        CamConfig c;
        c.serial = static_cast<std::string>(e["serial"]);
        c.name = static_cast<std::string>(e["name"]);
        c.framerate = static_cast<double>(e["framerate"]);
        c.width = static_cast<int>(e["width"]);
        c.height = static_cast<int>(e["height"]);
        c.sync = static_cast<std::string>(e["sync"]);
        config_map[c.serial] = c;
        ROS_INFO("YAML cfg: %s → serial=%s, %dx%d@%.1f Hz (%s)",
                 c.name.c_str(), c.serial.c_str(),
                 c.width, c.height, c.framerate,
                 c.sync.c_str());
    }

    // 2) Enumerate cameras and spawn handlers
    auto system = Spinnaker::System::GetInstance();
    auto camList = system->GetCameras();
    unsigned int total = camList.GetSize();
    ROS_INFO("SDK reports %u cameras connected", total);

    std::vector<std::shared_ptr<CameraHandler>> handlers;
    std::vector<std::thread> threads;
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
                auto handler = std::make_shared<CameraHandler>(cam, nh, it->second);
                handlers.push_back(handler);
                threads.emplace_back(&CameraHandler::spin, handler);
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
    for (auto &t : threads)
        if (t.joinable())
            t.join();

    camList.Clear();
    system->ReleaseInstance();
    return 0;
}

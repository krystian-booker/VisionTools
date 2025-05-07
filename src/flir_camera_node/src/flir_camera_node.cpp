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
#include <xmlrpcpp/XmlRpcValue.h>
#include <stdexcept>

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
    ~CameraHandler();

private:
    Spinnaker::CameraPtr cam_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    std::string topic_;
    CamConfig cfg_;
};

// -------- constructor --------------------------------
CameraHandler::CameraHandler(Spinnaker::CameraPtr cam,
                             ros::NodeHandle &nh,
                             const CamConfig &cfg)
    : cam_(cam), it_(nh), cfg_(cfg)
{
    // 1: Init camera so GenApi nodes become available
    cam_->Init();

    // 2: Apply resolution
    auto &nodemap = cam_->GetNodeMap();
    using namespace Spinnaker::GenApi;
    CIntegerPtr ptrW = nodemap.GetNode("Width");
    CIntegerPtr ptrH = nodemap.GetNode("Height");
    if (ptrW && ptrH && IsWritable(ptrW) && IsWritable(ptrH))
    {
        ptrW->SetValue(cfg_.width);
        ptrH->SetValue(cfg_.height);
    }

    // 3: Frame‑rate
    CBooleanPtr ptrRateEnable = nodemap.GetNode("AcquisitionFrameRateEnable");
    CFloatPtr ptrRate = nodemap.GetNode("AcquisitionFrameRate");
    if (ptrRateEnable && ptrRate && IsWritable(ptrRateEnable))
    {
        ptrRateEnable->SetValue(true);
        ptrRate->SetValue(cfg_.framerate);
    }

    // 4: Pixel format → Mono8
    {
        CEnumerationPtr pixelFmt = nodemap.GetNode("PixelFormat");
        if (IsWritable(pixelFmt))
        {
            CEnumEntryPtr mono8 = pixelFmt->GetEntryByName("Mono8");
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
    pub_ = it_.advertise(topic_, 1);
    ROS_INFO("Started '%s' @ %dx%d @ %.1f Hz → %s",
             cfg_.name.c_str(),
             cfg_.width, cfg_.height, cfg_.framerate,
             topic_.c_str());
}

// -------- worker thread ------------------------------
void CameraHandler::spin()
{
    while (ros::ok())
    {
        auto img = cam_->GetNextImage();
        if (img->IsIncomplete())
        {
            img->Release();
            continue;
        }

        cv::Mat frame(img->GetHeight(), img->GetWidth(),
                      CV_8UC1, img->GetData());

        auto msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame)
                       .toImageMsg();
        msg->header.stamp = ros::Time::now();
        pub_.publish(msg);

        img->Release();
    }
}

// -------- destructor ---------------------------------
CameraHandler::~CameraHandler()
{
    cam_->EndAcquisition();
    cam_->DeInit();
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
    nh.getParam("camera_config/cameras", cam_list);
    std::vector<CamConfig> configs;
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
        configs.push_back(c);
        ROS_INFO("YAML cfg: %s → serial=%s, %dx%d@%.1f Hz (%s)",
                 c.name.c_str(), c.serial.c_str(),
                 c.width, c.height, c.framerate,
                 c.sync.c_str());
    }

    // 2) Enumerate cameras and spawn handlers
    auto system = Spinnaker::System::GetInstance();
    auto camList = system->GetCameras();
    unsigned int total = camList.GetSize();
    ROS_INFO("SDK reports %u cameras connected", total);

    std::vector<std::thread> threads;
    for (auto &cfg : configs)
    {
        for (unsigned i = 0; i < total; ++i)
        {
            auto cam = camList.GetByIndex(i);

            using namespace Spinnaker::GenApi;
            CStringPtr strPtr =
                cam->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");
                std::string serial =
                (strPtr && IsReadable(strPtr))
                    ? std::string(strPtr->GetValue()) 
                    : std::string();
            
            if (serial == cfg.serial)
            {
                auto handler = std::make_shared<CameraHandler>(cam, nh, cfg);
                threads.emplace_back(&CameraHandler::spin, handler);
                break;
            }
        }
    }

    if (threads.empty())
    {
        ROS_FATAL("No configured cameras matched! Exiting.");
        return 1;
    }

    ros::spin();
    for (auto &t : threads)
        if (t.joinable())
            t.join();

    camList.Clear();
    system->ReleaseInstance();
    return 0;
}

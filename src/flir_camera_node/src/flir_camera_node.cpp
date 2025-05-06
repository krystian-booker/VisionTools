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

// Simple struct to hold YAML params
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
                  const CamConfig &cfg)
        : cam_(cam), it_(nh), cfg_(cfg)
    {
        // 1: Init camera so GenApi nodes become available
        cam_->Init();

        // 2: Apply resolution
        auto nodemap = cam_->GetNodeMap();
        Spinnaker::GenApi::CIntegerPtr ptrW = nodemap.GetNode("Width");
        Spinnaker::GenApi::CIntegerPtr ptrH = nodemap.GetNode("Height");
        if (ptrW && ptrH && ptrW->IsWritable() && ptrH->IsWritable())
        {
            ptrW->SetValue(cfg_.width);
            ptrH->SetValue(cfg_.height);
        }

        // 3: Apply frame rate (disable auto and set manual)
        Spinnaker::GenApi::CBooleanPtr ptrRateEnable =
            nodemap.GetNode("AcquisitionFrameRateEnable");
        Spinnaker::GenApi::CFloatPtr ptrRate =
            nodemap.GetNode("AcquisitionFrameRate");
        if (ptrRateEnable && ptrRate && ptrRateEnable->IsWritable())
        {
            ptrRateEnable->SetValue(true);
            ptrRate->SetValue(cfg_.framerate);
        }

        // 4: Pixel format → Mono8
        Spinnaker::GenApi::CEnumerationPtr ptrPixel =
            nodemap.GetNode("PixelFormat");
        if (ptrPixel && ptrPixel->IsWritable())
        {
            auto entryMono8 = ptrPixel->GetEntryByName("Mono8");
            if (entryMono8 && entryMono8->IsAvailable())
                ptrPixel->SetIntValue(entryMono8->GetValue());
        }

        // 5: Hardware‐sync setup
        if (cfg_.sync == "primary")
        {
            // master: output a pulse on Line0 each frame
            auto lineSel = nodemap.GetNode("LineSelector")
                               ->GetCurrentEntry();
            nodemap.GetNode("LineSelector")
                ->SetIntValue(nodemap.GetNode("LineSelector")
                                  ->GetEntryByName("Line3")
                                  ->GetValue());
            nodemap.GetNode("LineMode")
                ->SetIntValue(nodemap.GetNode("LineMode")
                                  ->GetEntryByName("Output")
                                  ->GetValue());
            nodemap.GetNode("LineSource")
                ->SetIntValue(nodemap.GetNode("LineSource")
                                  ->GetEntryByName("ExposureActive")
                                  ->GetValue());
            ROS_INFO("Camera %s: configured as PRIMARY sync on Line3",
                     cfg_.name.c_str());
        }
        else
        {
            // slave: use external trigger on Line0
            nodemap.GetNode("TriggerSelector")
                ->SetIntValue(nodemap.GetNode("TriggerSelector")
                                  ->GetEntryByName("FrameStart")
                                  ->GetValue());
            nodemap.GetNode("TriggerSource")
                ->SetIntValue(nodemap.GetNode("TriggerSource")
                                  ->GetEntryByName("Line3")
                                  ->GetValue());
            nodemap.GetNode("TriggerMode")
                ->SetIntValue(nodemap.GetNode("TriggerMode")
                                  ->GetEntryByName("On")
                                  ->GetValue());
            ROS_INFO("Camera %s: configured as SECONDARY sync (external trigger)",
                     cfg_.name.c_str());
        }

        // 6: Begin acquisition
        cam_->BeginAcquisition();

        // 7: Setup ROS publisher
        topic_ = "/camera/" + cfg_.name + "/image_raw";
        pub_ = it_.advertise(topic_, 1);
        ROS_INFO("Started '%s' @ %dx%d @ %.1fHz → topic '%s'",
                 cfg_.name.c_str(),
                 cfg_.width, cfg_.height,
                 cfg_.framerate,
                 topic_.c_str());
    }

    void spin()
    {
        while (ros::ok())
        {
            auto img = cam_->GetNextImage();
            if (img->IsIncomplete())
            {
                img->Release();
                continue;
            }

            // Already Mono8 from GenApi
            int w = img->GetWidth(), h = img->GetHeight();
            cv::Mat frame(h, w, CV_8UC1, img->GetData());

            auto msg = cv_bridge::CvImage(
                           std_msgs::Header(), "mono8", frame)
                           .toImageMsg();
            msg->header.stamp = ros::Time::now();
            pub_.publish(msg);

            img->Release();
        }
    }

    ~CameraHandler()
    {
        cam_->EndAcquisition();
        cam_->DeInit();
    }

private:
    Spinnaker::CameraPtr cam_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    std::string topic_;
    CamConfig cfg_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flir_camera_node");
    ros::NodeHandle nh("~");

    // —– Read YAML array into vector<CamConfig>
    XmlRpc::XmlRpcValue cam_list;
    nh.getParam("camera_config/cameras", cam_list);
    std::vector<CamConfig> configs;
    for (int i = 0; i < cam_list.size(); ++i)
    {
        auto &e = cam_list[i];
        CamConfig c;
        c.serial = (std::string)e["serial"];
        c.name = (std::string)e["name"];
        c.framerate = (double)e["framerate"];
        c.width = (int)e["width"];
        c.height = (int)e["height"];
        c.sync = (std::string)e["sync"];
        configs.push_back(c);
        ROS_INFO("YAML cfg: %s → serial=%s, %dx%d@%.1fHz (%s)",
                 c.name.c_str(), c.serial.c_str(),
                 c.width, c.height, c.framerate,
                 c.sync.c_str());
    }

    // —– Enumerate all FLIR cameras, match by serial, spawn handlers
    auto system = Spinnaker::System::GetInstance();
    auto camList = system->GetCameras();
    unsigned int total = camList.GetSize();
    ROS_INFO("SDK reports %u cameras connected", total);

    std::vector<std::thread> threads;
    for (auto &cfg : configs)
    {
        // find this serial in the actual list
        for (unsigned int i = 0; i < total; ++i)
        {
            auto cam = camList.GetByIndex(i);
            // read its serial number
            auto ptr = cam->GetTLDeviceNodeMap()
                           .GetNode("DeviceSerialNumber");
            std::string s = (ptr && ptr->IsReadable())
                                ? ptr->GetValue()
                                : "";
            if (s == cfg.serial)
            {
                // create handler + thread
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

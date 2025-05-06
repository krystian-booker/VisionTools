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

class CameraHandler
{
public:
    CameraHandler(Spinnaker::CameraPtr cam,
                  ros::NodeHandle &nh,
                  const std::string &topic_root)
        : cam_(cam), it_(nh), topic_root_(topic_root)
    {
        // Initialize and start acquisition
        cam_->Init();
        cam_->BeginAcquisition();
        pub_ = it_.advertise(topic_root_ + "/image_raw", 1);
        ROS_INFO("Camera on topic '%s/image_raw' started", topic_root_.c_str());
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

            // Convert to MONO8 for a true monochrome camera
            auto cvImg = img->Convert(
                Spinnaker::PixelFormat_Mono8,
                Spinnaker::ColorProcessingAlgorithm_HQ_LINEAR);

            int w = cvImg->GetWidth(), h = cvImg->GetHeight();
            cv::Mat frame(h, w, CV_8UC1, cvImg->GetData());

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
    std::string topic_root_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flir_camera_node");
    ros::NodeHandle nh("~"); // private handle to read params

    // --- 1) Load YAML camera config ---
    XmlRpc::XmlRpcValue cam_list;
    if (!nh.getParam("camera_config/cameras", cam_list) ||
        cam_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_FATAL("Parameter 'camera_config/cameras' missing or not an array");
        return 1;
    }

    std::map<std::string, std::string> config;
    for (int i = 0; i < cam_list.size(); ++i)
    {
        auto &entry = cam_list[i];
        if (entry.hasMember("serial") && entry.hasMember("name"))
        {
            std::string serial = static_cast<std::string>(entry["serial"]);
            std::string name = static_cast<std::string>(entry["name"]);
            config[serial] = name;
            ROS_INFO("Configured camera serial=%s as '%s'",
                     serial.c_str(), name.c_str());
        }
        else
        {
            ROS_WARN("Skipping invalid camera entry %d", i);
        }
    }

    // --- 2) Enumerate and filter actual FLIR cameras ---
    auto system = Spinnaker::System::GetInstance();
    auto camList = system->GetCameras();
    unsigned int found = camList.GetSize();
    ROS_INFO("FLIR SDK reports %u camera(s) connected", found);

    std::vector<std::unique_ptr<CameraHandler>> handlers;
    std::vector<std::thread> threads;
    for (unsigned int i = 0; i < found; ++i)
    {
        auto cam = camList.GetByIndex(i);
        // read its serial
        std::string serial = "unknown";
        auto ptrSerial = cam->GetTLDeviceNodeMap()
                             .GetNode("DeviceSerialNumber");
        if (Spinnaker::GenApi::IsAvailable(ptrSerial) &&
            Spinnaker::GenApi::IsReadable(ptrSerial))
        {
            serial = ptrSerial->GetValue();
        }

        auto it = config.find(serial);
        if (it == config.end())
        {
            ROS_WARN("Camera with serial %s not in config; skipping", serial.c_str());
            continue;
        }

        // Create handler with topic_root = "/camera/<name>"
        std::string topic_root = "/camera/" + it->second;
        handlers.emplace_back(
            new CameraHandler(cam, nh, topic_root));
    }

    if (handlers.empty())
    {
        ROS_FATAL("No configured cameras were found. Exiting.");
        camList.Clear();
        system->ReleaseInstance();
        return 1;
    }

    // --- 3) Start a thread per handler ---
    for (auto &h : handlers)
        threads.emplace_back(&CameraHandler::spin, h.get());

    ros::spin();

    // Cleanup
    for (auto &t : threads)
        if (t.joinable())
            t.join();

    camList.Clear();
    system->ReleaseInstance();
    return 0;
}

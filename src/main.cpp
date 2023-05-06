#include <cstdint>
#include <iostream>

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include "rclcpp/rclcpp.hpp"
#include "opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

namespace flir_vision_ros2 {

  class FlirVision: public rclcpp::Node {
  public:
    explicit FlirVision(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()):
    Node("flir_vision", options),
    pub_cam_grey_(image_transport::create_publisher(this, "/flir_grey")),
    pub_cam_heat_(image_transport::create_publisher(this, "/flir_heat")) {
      timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(10),
        std::bind(&FlirVision::connectCamera, this));
      connectCamera();
    }

    ~FlirVision() {
    }

  private:
    void connectCamera() {
      SystemPtr system;
      system = System::GetInstance();
      CameraList camList = system -> GetCameras();

      const unsigned int numCameras = camList.GetSize();

      RCLCPP_INFO_STREAM_ONCE(get_logger(), "Number of cameras detected: " << numCameras);

      if (numCameras == 0) {
        camList.Clear();
        system -> ReleaseInstance();

        RCLCPP_WARN(get_logger(), "Flir A320 not detected, checking again in 10 seconds");
        return;
      }

      int result = 0;
      bool cam_with_mac = false;

      std::string mac_address = "";
      std::string mac_address_w_colons = "";
      if (!has_parameter("mac_address")) declare_parameter("mac_address", mac_address);
      get_parameter("mac_address", mac_address);
      std::string hexString;
      if (mac_address != "") {
        mac_address_w_colons = mac_address;
        mac_address.erase(std::remove(mac_address.begin(), mac_address.end(), ':'), mac_address.end());
        std::stringstream ss;
        ss << "0x" << std::hex << std::setw(12) << std::setfill('0') << std::stoull(mac_address, nullptr, 16);
        hexString = ss.str();
      }

      // Run example on each camera
      for (unsigned int i = 0; i < numCameras; i++) {
        pCam = camList.GetByIndex(i);
        
        if (mac_address != "") {
          INodeMap & nodeMapTLDevice = pCam -> GetTLDeviceNodeMap();
          try {
            FeatureList_t features;
            const CCategoryPtr category = nodeMapTLDevice.GetNode("DeviceInformation");
            if (IsReadable(category)) {
              category -> GetFeatures(features);

              for (auto it = features.begin(); it != features.end(); ++it) {
                const CNodePtr pfeatureNode = * it;
                CValuePtr pValue = static_cast < CValuePtr > (pfeatureNode);
                if (pfeatureNode -> GetName() == "GevDeviceMACAddress") {
                  if (IsReadable(pValue)) {
                    if (areEqualHexStrings(pValue -> ToString().c_str(), hexString)) {
                      cam_with_mac = true;
                      break;
                    }
                  }
                }
              }
            } else {
              RCLCPP_WARN(get_logger(), "Device control information not available.");
            }
          } catch (Spinnaker::Exception & e) {
            RCLCPP_ERROR(get_logger(), e.what());
          }
        } else cam_with_mac = true;

        if (!cam_with_mac) {
          pCam = nullptr;
          camList.Clear();
          system -> ReleaseInstance();

          RCLCPP_WARN_STREAM(get_logger(), "Device with  mac address " << mac_address_w_colons << " not found. Checking in 10 seconds.");
          return;
        }

        timer_->reset();
        result = result | RunSingleCamera(pCam);
      }

      pCam = nullptr;
      camList.Clear();
      system -> ReleaseInstance();
      return;
    }

    int AcquireImages(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice) {
      int result = 0;

      try {
        CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        if (!IsReadable(ptrAcquisitionMode) ||
          !IsWritable(ptrAcquisitionMode)) {
          RCLCPP_WARN(get_logger(), "Unable to set acquisition mode to continuous (enum retrieval). Aborting...");
          return 0;
        }

        // Retrieve entry node from enumeration node
        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode -> GetEntryByName("Continuous");
        if (!IsReadable(ptrAcquisitionModeContinuous)) {
          RCLCPP_WARN(get_logger(), "Unable to get or set acquisition mode to continuous (entry retrieval). Aborting...");
          return 0;
        }

        // Retrieve integer value from entry node
        const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous -> GetValue();

        // Set integer value from entry node as new value of enumeration node
        ptrAcquisitionMode -> SetIntValue(acquisitionModeContinuous);

        pCam -> BeginAcquisition();

        ImageProcessor processor;
        processor.SetColorProcessing(SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);

        while (rclcpp::ok()) {
          try {
            ImagePtr pResultImage = pCam -> GetNextImage(1000);

            if (pResultImage -> IsIncomplete()) {
              // Retrieve and print the image status description
              RCLCPP_WARN_STREAM(get_logger(), "Image incomplete: " << Image::GetImageStatusDescription(pResultImage -> GetImageStatus()) << "...");

            } else {
              ImagePtr convertedImage = processor.Convert(pResultImage, PixelFormat_Mono8);

              cv::Mat opencvImage = cv::Mat(convertedImage -> GetHeight(), convertedImage -> GetWidth(), CV_8UC1, convertedImage -> GetData());
              cv::Mat heatMap;
              cv::normalize(opencvImage, heatMap, 0, 255, cv::NORM_MINMAX);
              cv::applyColorMap(heatMap, heatMap, cv::COLORMAP_JET);

              std_msgs::msg::Header header;
              header.frame_id = "flir";
              header.stamp = get_clock()->now();

              sensor_msgs::msg::Image::SharedPtr grey_msg = cv_bridge::CvImage(header, "mono8", opencvImage).toImageMsg();
              pub_cam_grey_.publish(*grey_msg.get());

              sensor_msgs::msg::Image::SharedPtr heat_msg = cv_bridge::CvImage(header, "bgr8", heatMap).toImageMsg();
              pub_cam_heat_.publish(*heat_msg.get());
            }

            pResultImage -> Release();

          } catch (Spinnaker::Exception & e) {
            RCLCPP_ERROR_STREAM(get_logger(), "" << e.what());
            return 0;
          }
        }

        pCam -> EndAcquisition();
      } catch (Spinnaker::Exception & e) {
        timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(10),
          std::bind(&FlirVision::connectCamera, this));
        RCLCPP_ERROR_STREAM(get_logger(), "" << e.what());
        return 0;
      }

      return 0;
    }

    int RunSingleCamera(CameraPtr pCam) {
      int result;
      int err = 0;

      try {
        // Retrieve TL device nodemap and print device information
        INodeMap & nodeMapTLDevice = pCam -> GetTLDeviceNodeMap();

        result = PrintDeviceInfo(nodeMapTLDevice);

        // Initialize camera
        pCam -> Init();

        // Retrieve GenICam nodemap
        INodeMap & nodeMap = pCam -> GetNodeMap();

        // Configure custom image settings
        err = ConfigureCustomImageSettings(nodeMap);
        if (err < 0) {
          return err;
        }

        // Configure heartbeat for GEV camera
        #ifdef _DEBUG
        result = result | DisableGVCPHeartbeat(pCam);
        #else
        result = result | ResetGVCPHeartbeat(pCam);
        #endif

        // Acquire images
        result = result | AcquireImages(pCam, nodeMap, nodeMapTLDevice);

        #ifdef _DEBUG
        // Reset heartbeat for GEV camera
        result = result | ResetGVCPHeartbeat(pCam);
        #endif

        // Deinitialize camera
        pCam -> DeInit();
      } catch (Spinnaker::Exception & e) {
        RCLCPP_ERROR_STREAM(get_logger(), "" << e.what());
        return 0;
      }

      return result;
    }

    int PrintDeviceInfo(INodeMap & nodeMap) {
      int result = 0;
      RCLCPP_INFO(get_logger(), "*** DEVICE INFORMATION ***");

      try {
        FeatureList_t features;
        const CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
        if (IsReadable(category)) {
          category -> GetFeatures(features);

          for (auto it = features.begin(); it != features.end(); ++it) {
            const CNodePtr pfeatureNode = * it;
            CValuePtr pValue = static_cast < CValuePtr > (pfeatureNode);
            RCLCPP_INFO_STREAM(get_logger(), pfeatureNode -> GetName() << " : "
              << (IsReadable(pValue) ? pValue -> ToString() : "Node not readable"));
          }
        } else {
          RCLCPP_WARN(get_logger(), "Device control information not available.");
        }
      } catch (Spinnaker::Exception & e) {
        RCLCPP_ERROR(get_logger(), e.what());
        return 0;
      }

      return result;
    }

    int ConfigureCustomImageSettings(INodeMap & nodeMap) {
      int result = 0;

      RCLCPP_INFO(get_logger(), "*** CONFIGURING CUSTOM IMAGE SETTINGS ***");

      try {
        CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");
        if (IsReadable(ptrPixelFormat) && IsWritable(ptrPixelFormat)) {
          // Retrieve the desired entry node from the enumeration node
          CEnumEntryPtr ptrPixelFormatMono8 = ptrPixelFormat -> GetEntryByName("Mono16");
          if (IsReadable(ptrPixelFormatMono8)) {
            // Retrieve the integer value from the entry node
            int64_t pixelFormatMono8 = ptrPixelFormatMono8 -> GetValue();

            // Set integer as new value for enumeration node
            ptrPixelFormat -> SetIntValue(pixelFormatMono8);

            RCLCPP_INFO_STREAM(get_logger(), "Pixel format set to " << ptrPixelFormat -> GetCurrentEntry() -> GetSymbolic());
          } else {
            RCLCPP_WARN(get_logger(), "Pixel format mono 8 not readable...");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Pixel format not readable or writable...");
        }
      } catch (Spinnaker::Exception & e) {
        RCLCPP_ERROR(get_logger(), e.what());
        return 0;
      }

      return result;
    }

    // Disables or enables heartbeat on GEV cameras so debugging does not incur timeout errors
    int ConfigureGVCPHeartbeat(CameraPtr pCam, bool enable) {
      // Retrieve TL device nodemap
      INodeMap & nodeMapTLDevice = pCam -> GetTLDeviceNodeMap();

      // Retrieve GenICam nodemap
      INodeMap & nodeMap = pCam -> GetNodeMap();

      CEnumerationPtr ptrDeviceType = nodeMapTLDevice.GetNode("DeviceType");
      if (!IsReadable(ptrDeviceType)) {
        return 0;
      }

      if (ptrDeviceType -> GetIntValue() != DeviceType_GigEVision) {
        return 0;
      }

      if (enable) {
        RCLCPP_INFO(get_logger(), "Resetting heartbeat...");
      } else {
        RCLCPP_INFO(get_logger(), "Disabling heartbeat...");
      }

      CBooleanPtr ptrDeviceHeartbeat = nodeMap.GetNode("GevGVCPHeartbeatDisable");
      if (!IsWritable(ptrDeviceHeartbeat)) {
        RCLCPP_INFO(get_logger(), "Unable to configure heartbeat. Continuing with execution as this may be non-fatal...");
      } else {
        ptrDeviceHeartbeat -> SetValue(enable);

        if (!enable) {
          RCLCPP_WARN(get_logger(), "WARNING: Heartbeat has been disabled for the rest of this example run.");
        } else {
          RCLCPP_INFO(get_logger(), "Heartbeat has been reset.");
        }
      }

      return 0;
    }

    int ResetGVCPHeartbeat(CameraPtr pCam) {
      return ConfigureGVCPHeartbeat(pCam, true);
    }

    int DisableGVCPHeartbeat(CameraPtr pCam) {
      return ConfigureGVCPHeartbeat(pCam, false);
    }

    bool areEqualHexStrings(const std::string& s1, const std::string& s2) {
        // Convert the strings to unsigned long long integers
        unsigned long long n1 = std::stoull(s1, nullptr, 16);
        unsigned long long n2 = std::stoull(s2, nullptr, 16);
        return n1 == n2;
    }

    CameraPtr pCam = nullptr;

    const image_transport::Publisher pub_cam_grey_;
    const image_transport::Publisher pub_cam_heat_;
    sensor_msgs::msg::Image::SharedPtr img_msg;

    rclcpp::TimerBase::SharedPtr timer_;
  };

} // namespace flir_vision_ros2

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(flir_vision_ros2::FlirVision)
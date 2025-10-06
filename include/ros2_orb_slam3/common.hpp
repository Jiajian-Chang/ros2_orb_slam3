// Include file 
#ifndef COMMON_HPP  // Header guard to prevent multiple inclusions
#define COMMON_HPP

// C++ includes
#include <iostream> // The iostream library is an object-oriented library that provides input and output functionality using streams
#include <algorithm> // The header <algorithm> defines a collection of functions especially designed to be used on ranges of elements.
#include <fstream> // Input/output stream class to operate on files.
#include <chrono> // c++ timekeeper library
#include <vector> // vectors are sequence containers representing arrays that can change in size.
#include <queue>
#include <thread> // class to represent individual threads of execution.
#include <mutex> // A mutex is a lockable object that is designed to signal when critical sections of code need exclusive access, preventing other threads with the same protection from executing concurrently and access the same memory locations.
#include <cstdlib> // to find home directory

#include <cstring>
#include <sstream> // String stream processing functionalities
#include <map>
#include <limits>

//* ROS2 includes
//* std_msgs in ROS 2 https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
#include "rclcpp/rclcpp.hpp"

// #include "your_custom_msg_interface/msg/custom_msg_field.hpp" // Example of adding in a custom message
#include <std_msgs/msg/header.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sensor_msgs/msg/image.hpp"
using std::placeholders::_1; //* TODO why this is suggested in official tutorial

// Include Eigen
// Quick reference: https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
#include <Eigen/Dense> // Includes Core, Geometry, LU, Cholesky, SVD, QR, and Eigenvalues header file

// Include cv-bridge
#include <cv_bridge/cv_bridge.h>

// Include OpenCV computer vision library
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp> // Image processing tools
#include <opencv2/highgui/highgui.hpp> // GUI tools
#include <opencv2/core/eigen.hpp>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

//* ORB SLAM 3 includes
#include "System.h" //* Also imports the ORB_SLAM3 namespace

//* Gobal defs
#define pass (void)0 // Python's equivalent of "pass" i.e. no operation


//* Node specific definitions
class MonocularMode : public rclcpp::Node
{   
    //* This slam node inherits from both rclcpp and ORB_SLAM3::System classes
    //* public keyword needs to come before the class constructor and anything else
    public:
    std::string experimentConfig = ""; // String to receive settings sent by the python driver
    double timeStep; // Timestep data received from the python node
    std::string receivedConfig = "";

    //* Class constructor
    MonocularMode(); // Constructor 

    ~MonocularMode(); // Destructor
        
    private:
        
        // Class internal variables
        std::string homeDir = "";
        std::string packagePath = "ros2_test/src/ros2_orb_slam3/"; //! Change to match path to your workspace
        std::string OPENCV_WINDOW = ""; // Set during initialization
        std::string nodeName = ""; // Name of this node
        std::string vocFilePath = ""; // Path to ORB vocabulary provided by DBoW2 package
        std::string settingsFilePath = ""; // Path to settings file provided by ORB_SLAM3 package
        bool bSettingsFromPython = false; // Flag set once when experiment setting from python node is received
        
        std::string subexperimentconfigName = ""; // Subscription topic name
        std::string pubconfigackName = ""; // Publisher topic name
        std::string subImgMsgName = ""; // Topic to subscribe to receive RGB images from a python node
        std::string subTimestepMsgName = ""; // Topic to subscribe to receive the timestep related to the 

        //* Definitions of publisher and subscribers
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr expConfig_subscription_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr configAck_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImgMsg_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subTimestepMsg_subscription_;

        //* ORB_SLAM3 related variables
        ORB_SLAM3::System* pAgent; // pointer to a ORB SLAM3 object
        ORB_SLAM3::System::eSensor sensorType;
        bool enablePangolinWindow = false; // Shows Pangolin window output
        bool enableOpenCVWindow = false; // Shows OpenCV window output

        //* ROS callbacks
        void experimentSetting_callback(const std_msgs::msg::String& msg); // Callback to process settings sent over by Python node
        void Timestep_callback(const std_msgs::msg::Float64& time_msg); // Callback to process the timestep for this image
        void Img_callback(const sensor_msgs::msg::Image& msg); // Callback to process RGB image and semantic matrix sent by Python node
        
        //* Helper functions
        // ORB_SLAM3::eigenMatXf convertToEigenMat(const std_msgs::msg::Float32MultiArray& msg); // Helper method, converts semantic matrix eigenMatXf, a Eigen 4x4 float matrix
        void initializeVSLAM(std::string& configString); //* Method to bind an initialized VSLAM framework to this node


};


//* Stereo node specific definitions
class StereoMode : public rclcpp::Node
{
    public:
    StereoMode();
    ~StereoMode();
    void initializeSubscriptions();

    private:
        // Internal state
        std::string nodeName = "";
        std::string vocFilePath = "";
        std::string settingsFilePath = ""; // Hardcoded to a Stereo config YAML
        bool headless_ = false;
        
        // Frames
        std::string mapFrame = "map";
        std::string cameraFrame = "camera_link";

        // Topics
        std::string leftImageTopic = "camera/left";
        std::string rightImageTopic = "camera/right";

        // message_filters subscribers and sync
        message_filters::Subscriber<sensor_msgs::msg::Image> left_filter_;
        message_filters::Subscriber<sensor_msgs::msg::Image> right_filter_;
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
        int approx_sync_queue_size_ = 10;

        // ORB-SLAM3
        ORB_SLAM3::System* pAgent = nullptr;
        ORB_SLAM3::System::eSensor sensorType;

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        nav_msgs::msg::Path keyframe_path_;
        rclcpp::TimerBase::SharedPtr init_timer_;

        // Callback
        void stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr left,
                            const sensor_msgs::msg::Image::ConstSharedPtr right);

        // Helpers
        void initializeVSLAM();
        void publishOutputs(const Sophus::SE3f& Tcw, const rclcpp::Time& stamp);
};

#endif
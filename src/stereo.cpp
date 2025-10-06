#include "ros2_orb_slam3/common.hpp"
#include <rmw/qos_profiles.h>

StereoMode::StereoMode() : Node("stereo_node_cpp")
{
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 STEREO NODE STARTED");

    // Parameters (optional overrides)
    this->declare_parameter("node_name_arg", "stereo_slam_cpp");
    this->declare_parameter("left_topic", leftImageTopic);
    this->declare_parameter("right_topic", rightImageTopic);
    this->declare_parameter("voc_path", "");
    this->declare_parameter("settings_path", "");
    this->declare_parameter("map_frame", mapFrame);
    this->declare_parameter("camera_frame", cameraFrame);
    this->declare_parameter("approx_sync_queue", approx_sync_queue_size_);
    this->declare_parameter("headless", false);

    nodeName = this->get_parameter("node_name_arg").as_string();
    leftImageTopic = this->get_parameter("left_topic").as_string();
    rightImageTopic = this->get_parameter("right_topic").as_string();
    mapFrame = this->get_parameter("map_frame").as_string();
    cameraFrame = this->get_parameter("camera_frame").as_string();
    approx_sync_queue_size_ = this->get_parameter("approx_sync_queue").as_int();
    headless_ = this->get_parameter("headless").as_bool();

    // Require absolute paths provided via params
    std::string voc_param = this->get_parameter("voc_path").as_string();
    std::string settings_param = this->get_parameter("settings_path").as_string();

    vocFilePath = voc_param;
    settingsFilePath = settings_param;

    if (vocFilePath.empty() || vocFilePath.front() != '/')
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid voc_path. Provide an absolute path to ORBvoc.txt(.bin)");
        rclcpp::shutdown();
        return;
    }
    if (settingsFilePath.empty() || settingsFilePath.front() != '/')
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid settings_path. Provide an absolute path to a Stereo YAML");
        rclcpp::shutdown();
        return;
    }

    if (nodeName != this->get_name())
        RCLCPP_WARN(this->get_logger(), "node_name_arg (%s) does not match actual node name (%s)", nodeName.c_str(), this->get_name());

    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "left_topic %s", leftImageTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "right_topic %s", rightImageTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "map_frame %s, camera_frame %s", mapFrame.c_str(), cameraFrame.c_str());
    RCLCPP_INFO(this->get_logger(), "voc: %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "settings: %s", settingsFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "headless: %s", headless_ ? "true" : "false");

    initializeVSLAM();

    // Publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/camera_pose", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/keyframe_path", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::Int32>("/tracking_state", 10);
    map_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_points", 1);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    keyframe_path_.header.frame_id = mapFrame;

    // Defer subscriptions to avoid shared_from_this during construction
    init_timer_ = this->create_wall_timer(std::chrono::milliseconds(0), [this]() {
        this->initializeSubscriptions();
        this->init_timer_->cancel();
    });
}

void StereoMode::initializeSubscriptions()
{
    auto node_shared = std::static_pointer_cast<rclcpp::Node>(shared_from_this());

    left_filter_.subscribe(node_shared, leftImageTopic, rmw_qos_profile_sensor_data);
    right_filter_.subscribe(node_shared, rightImageTopic, rmw_qos_profile_sensor_data);

    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(approx_sync_queue_size_), left_filter_, right_filter_);
    sync_->registerCallback(std::bind(&StereoMode::stereoCallback, this, std::placeholders::_1, std::placeholders::_2));
}

StereoMode::~StereoMode()
{
    if (pAgent)
    {
        pAgent->Shutdown();
    }
    pass;
}

void StereoMode::initializeVSLAM()
{
    if (vocFilePath.empty() || settingsFilePath.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "VOC or settings path empty");
        rclcpp::shutdown();
        return;
    }

    sensorType = ORB_SLAM3::System::STEREO;
    bool enablePangolinWindowLocal = !headless_;
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindowLocal);
    std::cout << "StereoMode node initialized" << std::endl;
}

void StereoMode::stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr left,
                                const sensor_msgs::msg::Image::ConstSharedPtr right)
{
    if (!pAgent) return;

    cv_bridge::CvImagePtr left_ptr;
    cv_bridge::CvImagePtr right_ptr;
    try
    {
        left_ptr = cv_bridge::toCvCopy(*left);
        right_ptr = cv_bridge::toCvCopy(*right);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error converting stereo images");
        return;
    }

    // Use left stamp as time (seconds)
    const rclcpp::Time stamp(left->header.stamp);
    const double t_sec = static_cast<double>(stamp.nanoseconds()) * 1e-9;

    // Track stereo and get Tcw
    Sophus::SE3f Tcw = pAgent->TrackStereo(left_ptr->image, right_ptr->image, t_sec);

    publishOutputs(Tcw, stamp);
}

void StereoMode::publishOutputs(const Sophus::SE3f& Tcw, const rclcpp::Time& stamp)
{
    // Tracking state (placeholder)
    std_msgs::msg::Int32 state_msg;
    state_msg.data = 2; // OK
    state_pub_->publish(state_msg);

    // Convert to Twc (camera in world) and transform NED -> ENU
    Sophus::SE3f Twc_ned = Tcw.inverse();
    const Eigen::Matrix3f C_enu_ned = (Eigen::Matrix3f() << 0, 1, 0,
                                                           1, 0, 0,
                                                           0, 0, -1).finished();
    Eigen::Matrix3f R_enu = C_enu_ned * Twc_ned.rotationMatrix();
    Eigen::Vector3f t_enu = C_enu_ned * Twc_ned.translation();

    Eigen::Quaternionf q(R_enu);
    q.normalize();

    // PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = mapFrame;
    pose_msg.pose.position.x = t_enu.x();
    pose_msg.pose.position.y = t_enu.y();
    pose_msg.pose.position.z = t_enu.z();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    pose_pub_->publish(pose_msg);

    // Path
    keyframe_path_.header.stamp = stamp;
    keyframe_path_.header.frame_id = mapFrame;
    keyframe_path_.poses.push_back(pose_msg);
    path_pub_->publish(keyframe_path_);

    // TF map -> camera_frame (ENU)
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = mapFrame;
    tf_msg.child_frame_id = cameraFrame;
    tf_msg.transform.translation.x = t_enu.x();
    tf_msg.transform.translation.y = t_enu.y();
    tf_msg.transform.translation.z = t_enu.z();
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf_msg);

    // Map points placeholder
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = mapFrame;
    map_points_pub_->publish(cloud_msg);
} 
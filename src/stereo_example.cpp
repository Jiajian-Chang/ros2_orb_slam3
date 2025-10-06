/*
* Stereo entrypoint similar to mono_example.cpp
*/

//* Import all necessary modules
#include "ros2_orb_slam3/common.hpp"

//* main
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoMode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 
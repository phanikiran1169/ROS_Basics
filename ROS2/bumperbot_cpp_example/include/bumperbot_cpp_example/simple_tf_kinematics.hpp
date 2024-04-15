#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>

class SimpleTfKinematics : public rclcpp::Node
{
public:
    SimpleTfKinematics(const std::string& name);
private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped static_transform_stamped_;

};
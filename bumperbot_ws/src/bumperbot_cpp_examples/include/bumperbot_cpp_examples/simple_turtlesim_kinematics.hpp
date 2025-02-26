#ifndef SIMPLE_TURTLESIM_KINEMATICS_HPP
#define SIMPLE_TURTLESIM_KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>


class SimpleTurtlesimKinematics : public rclcpp::Node
{
public:
    SimpleTurtlesimKinematics(const std::string &name, double x, double y, double theta);
    void update(double linear_velocity, double angular_velocity);
    double get_x() const;
    double get_y() const;
    double get_theta() const;

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_sub_;

    turtlesim::msg::Pose last_turtle1_pose_;
    turtlesim::msg::Pose last_turtle2_pose_;

    double x_;
    double y_;
    double theta_;

    void turtule1PoseCallback(const turtlesim::msg::Pose & pose);

    void turtle2PoseCallback(const turtlesim::msg::Pose & pose);
   
};

#endif
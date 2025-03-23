#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>
#include <memory>
#include <vector>


using namespace std::placeholders;

class TrajectoryVisualization : public rclcpp::Node
{
public:
  TrajectoryVisualization() : Node("trajectory_visualization")
  {
    declare_parameter<std::string>("odom_topic", "bumperbot_controller/odom");
    std::string odom_topic = get_parameter("odom_topic").as_string();
    declare_parameter<std::string>("odom_noisy_topic", "bumperbot_controller/odom_noisy");
    std::string odom_noisy_topic = get_parameter("odom_noisy_topic").as_string();
    declare_parameter<std::string>("odom_filtered_topic", "odometry/filtered");
    std::string odom_filtered_topic = get_parameter("odom_filtered_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Initializing with odom_topic: %s, odom_noisy_topic: %s and odom_filtered_topic: %s", odom_topic.c_str(), odom_noisy_topic.c_str(), odom_filtered_topic.c_str()); 

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, std::bind(&TrajectoryVisualization::odometryCallback, this, _1)
    );
    odom_noisy_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_noisy_topic, 10, std::bind(&TrajectoryVisualization::odometryNoisyCallback, this, _1)
    );
    odom_filtered_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_filtered_topic, 10, std::bind(&TrajectoryVisualization::odometryFilteredCallback, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s, odom_noisy_topic: %s and odom_filtered_topic: %s", odom_topic.c_str(), odom_noisy_topic.c_str(), odom_filtered_topic.c_str());
      
    trajectory_pub_ = create_publisher<nav_msgs::msg::Path>("bumperbot_utils/trajectory", 10);
    trajectory_noisy_pub_ = create_publisher<nav_msgs::msg::Path>("bumperbot_utils/trajectory_noisy", 10);
    trajectory_filtered_pub_ = create_publisher<nav_msgs::msg::Path>("bumperbot_utils/trajectory_filtered", 10);

    RCLCPP_INFO(this->get_logger(), "Published trajectory");
    RCLCPP_INFO(this->get_logger(), "Published noisy trajectory");
    RCLCPP_INFO(this->get_logger(), "Published filtered trajectory");
  }
  
private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_noisy_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_filtered_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_noisy_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_filtered_pub_;

  nav_msgs::msg::Path trajectory_;
  nav_msgs::msg::Path trajectory_noisy_;
  nav_msgs::msg::Path trajectory_filtered_;

  void odometryCallback(const nav_msgs::msg::Odometry &msg)
  {
    trajectory_.header.frame_id = msg.header.frame_id;
    geometry_msgs::msg::PoseStamped curr_pose;
    curr_pose.header.frame_id = msg.header.frame_id;
    curr_pose.header.stamp = msg.header.stamp;
    curr_pose.pose = msg.pose.pose;
    trajectory_.poses.push_back(curr_pose);
    
    trajectory_pub_->publish(trajectory_);
  }

  void odometryNoisyCallback(const nav_msgs::msg::Odometry &msg)
  {
    trajectory_noisy_.header.frame_id = msg.header.frame_id;
    geometry_msgs::msg::PoseStamped curr_pose;
    curr_pose.header.frame_id = msg.header.frame_id;
    curr_pose.header.stamp = msg.header.stamp;
    curr_pose.pose = msg.pose.pose;
    trajectory_noisy_.poses.push_back(curr_pose);
    
    trajectory_noisy_pub_->publish(trajectory_noisy_);
  }

  void odometryFilteredCallback(const nav_msgs::msg::Odometry &msg)
  {
    trajectory_filtered_.header.frame_id = msg.header.frame_id;
    geometry_msgs::msg::PoseStamped curr_pose;
    curr_pose.header.frame_id = msg.header.frame_id;
    curr_pose.header.stamp = msg.header.stamp;
    curr_pose.pose = msg.pose.pose;
    trajectory_filtered_.poses.push_back(curr_pose);
    
    trajectory_filtered_pub_->publish(trajectory_filtered_);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryVisualization>());
  rclcpp::shutdown();
  return 0;
}
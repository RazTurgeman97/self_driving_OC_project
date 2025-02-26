#include "bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp"

using std::placeholders::_1;


SimpleTurtlesimKinematics::SimpleTurtlesimKinematics(const std::string &name, double x, double y, double theta)
    : Node(name), x_(x), y_(y), theta_(theta)
{
    turtle1_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10, std::bind(&SimpleTurtlesimKinematics::turtule1PoseCallback, this, _1));

    turtle2_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle2/pose", 10, std::bind(&SimpleTurtlesimKinematics::turtle2PoseCallback, this, _1));
}

void SimpleTurtlesimKinematics::turtule1PoseCallback(const turtlesim::msg::Pose &pose)
{
    last_turtle1_pose_ = pose;
}

void SimpleTurtlesimKinematics::turtle2PoseCallback(const turtlesim::msg::Pose &pose)
{
    last_turtle2_pose_ = pose;

    float Tx = last_turtle2_pose_.x - last_turtle1_pose_.x;
    float Ty = last_turtle2_pose_.y - last_turtle1_pose_.y;
    float distance = std::sqrt(Tx * Tx + Ty * Ty);

    float theta_rad = last_turtle2_pose_.theta - last_turtle1_pose_.theta;
    float theta_deg = theta_rad * 180 / M_PI;

    RCLCPP_INFO_STREAM(get_logger(),
        "\nTranslation Vector turtle1 -> turtle2 \n" <<
        "Tx: " << Tx << "\n" <<
        "Ty: " << Ty << "\n" <<
        "Distance between turtles: " << distance << "\n" <<
        "Rotation Matrix turtle1 -> turtle2 \n" <<
        "theta (rad): " << theta_rad << "\n" <<
        "theta (deg): " << theta_deg << "\n" <<
        "|R11         R12| : |" << std::cos(theta_rad) << "\t" << -std::sin(theta_rad) << "|\n" <<
        "|R21         R22| : |" << std::sin(theta_rad) << "\t" << std::cos(theta_rad) << "|\n"
    );

    // RCLCPP_INFO(get_logger(), "\nDistance between turtles: %f", distance);
}

void SimpleTurtlesimKinematics::update(double linear_velocity, double angular_velocity)
{
    x_ += linear_velocity * std::cos(theta_);
    y_ += linear_velocity * std::sin(theta_);
    theta_ += angular_velocity;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTurtlesimKinematics>("simple_turtlesim_kinematics", 0, 0, 0);
    rclcpp::spin(node);

    // rclcpp::Rate rate(0.01);
    // while (rclcpp::ok())
    // {
    //     node->update(1, 0);
    //     rclcpp::spin_some(node);
    //     rate.sleep();
    // }

    rclcpp::shutdown();
    return 0;
}
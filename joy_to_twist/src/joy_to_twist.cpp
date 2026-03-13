#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoyToTwist : public rclcpp::Node
{
public:
    JoyToTwist() : Node("joy_to_twist")
    {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoyToTwist::joyCallback, this, std::placeholders::_1));

        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);


        // 参数
        linear_scale_ = this->declare_parameter("linear_scale", 0.5);
        angular_scale_ = this->declare_parameter("angular_scale", 1.0);
        deadband_ = this->declare_parameter("deadband", 0.05);

        RCLCPP_INFO(this->get_logger(), "joy_to_twist started");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    double linear_scale_;
    double angular_scale_;
    double deadband_;

    double apply_deadband(double v)
    {
        if (std::abs(v) < deadband_)
            return 0.0;
        return v;
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->buttons.size() <= 5)
            return;

        // enable button
        if (msg->buttons[5] == 0)
            return;

        if (msg->axes.size() < 2)
            return;

        double linear = apply_deadband(msg->axes[1]);
        double angular = apply_deadband(msg->axes[0]);

        geometry_msgs::msg::Twist twist;

        twist.linear.x = linear * linear_scale_;
        twist.angular.z = angular * angular_scale_;

        twist_pub_->publish(twist);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToTwist>());
    rclcpp::shutdown();
}

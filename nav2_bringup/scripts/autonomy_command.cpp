#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cg_msgs/msg/actuator_command.hpp>
#include <algorithm>  // For std::clamp

class CmdVelToActuator : public rclcpp::Node {
public:
    CmdVelToActuator() : Node("cmd_vel_to_actuator"), max_linear_speed(0.5), max_steering_angle(0.5), tool_height_(50.0) {
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&CmdVelToActuator::cmd_vel_callback, this, std::placeholders::_1));

        tool_height_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/tool_height", 10, std::bind(&CmdVelToActuator::tool_height_callback, this, std::placeholders::_1));

        actuator_pub_ = this->create_publisher<cg_msgs::msg::ActuatorCommand>("/actuator_cmd", 10);
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto actuator_msg = cg_msgs::msg::ActuatorCommand();
        actuator_msg.header.stamp = this->get_clock()->now();

        // Convert linear velocity to wheel velocity
        actuator_msg.wheel_velocity = (msg->linear.x / max_linear_speed) * 100.0;
        actuator_msg.wheel_velocity = std::clamp(actuator_msg.wheel_velocity, -15.0, 15.0);

        // Convert angular velocity to steer position
        actuator_msg.steer_position = -(msg->angular.z / max_steering_angle) * 100.0;
        actuator_msg.steer_position = std::clamp(actuator_msg.steer_position, -90.0, 90.0);

        // Include the latest tool height (assumed to be already in [0, 100] percent)
        actuator_msg.tool_position = std::clamp(tool_height_, 0.0, 100.0);

        // Publish the message
        actuator_pub_->publish(actuator_msg);
        RCLCPP_INFO(this->get_logger(), "Published ActuatorCommand: wheel_velocity=%.2f, steer_position=%.2f, tool_position=%.2f",
                    actuator_msg.wheel_velocity, actuator_msg.steer_position, actuator_msg.tool_position);
    }

    void tool_height_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        tool_height_ = msg->data;
        RCLCPP_DEBUG(this->get_logger(), "Received tool height: %.2f", tool_height_);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tool_height_sub_;
    rclcpp::Publisher<cg_msgs::msg::ActuatorCommand>::SharedPtr actuator_pub_;

    double tool_height_;
    const double max_linear_speed;
    const double max_steering_angle;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelToActuator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class AckermannSteeringController : public rclcpp::Node {
public:
    AckermannSteeringController()
    : Node("ackermann_steering_controller") {
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&AckermannSteeringController::cmd_vel_callback, this, std::placeholders::_1));
        ackermann_cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/ackermann_cmd_vel", 10);
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 여기서 Ackermann 조향 알고리즘을 적용하여 처리된 명령을 생성합니다.
        auto ackermann_cmd = geometry_msgs::msg::Twist();
        ackermann_cmd.linear.x = msg->linear.x;
        ackermann_cmd.angular.z = msg->angular.z;
        
        // 처리된 명령을 발행합니다.
        ackermann_cmd_vel_publisher_->publish(ackermann_cmd);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ackermann_cmd_vel_publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AckermannSteeringController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
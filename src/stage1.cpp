// balancing the robot

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Controller : public rclcpp::Node
{
public:
  Controller() : Node("controller"),
                 kp_(10), ki_(30), kd_(0.0), integral_(0.0), prev_error_(0.0), setpoint_(0.0)
  {
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu_plugin/out", 10, std::bind(&Controller::imuCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&Controller::timerCallback, this));
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    //RCLCPP_INFO(this->get_logger(), "Pitch = '%.2f'",pitch);
    double error = setpoint_ - pitch; // Assuming pitch=0 is balanced
    integral_ += error * 0.005; // Assuming the timer callback is 100ms, so dt=0.1s
    double derivative = (error - prev_error_) / 0.005;
    output_ = kp_ * error + ki_ * integral_ + kd_ * derivative;
    prev_error_ = error;
  }

  void timerCallback()
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = -output_; // Initially, don't move forward or backward
    twist.angular.z = 0; // Use PID output to adjust angular velocity
    cmd_vel_publisher_->publish(twist);
    RCLCPP_INFO(this->get_logger(), "Publishing velocity command: linear.x = '%.2f', angular.z = '%.2f'", twist.linear.x, twist.angular.z);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  double kp_, ki_, kd_, integral_, prev_error_, setpoint_, output_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Controller>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

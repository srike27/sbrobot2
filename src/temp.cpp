#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Controller : public rclcpp::Node
{
public:
  Controller() : Node("controller"),
                 kp_speed(10), ki_speed(30), kd_speed(0.0), integral_speed(0.0), prev_error_speed(0.0), setpoint_(0.0)
  {
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    tele_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "tele_vel", 10, std::bind(&Controller::teleVelCallback, this, std::placeholders::_1));
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu_plugin/out", 10, std::bind(&Controller::imuCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&Controller::timerCallback, this));
    vel_rxed = false;
    kp_setpoint = 0.05;
    ki_setpoint = 0;
    kd_setpoint = 0.001;
    integral_setpoint = 0;
    prev_error_setpoint = 0;
    last_call_time_ = this->now();
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if(!vel_rxed){
      setpoint_ = 0;
    }
    tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);
    
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    //RCLCPP_INFO(this->get_logger(), "Pitch = '%.2f'",pitch);
    double error = setpoint_ - pitch; // Assuming pitch=0 is balanced
    integral_speed += error * 0.005; // Assuming the timer callback is 100ms, so dt=0.1s
    double derivative = (error - prev_error_speed) / 0.005;
    set_lin_x = kp_speed * error + ki_speed * integral_speed + kd_speed * derivative;
    prev_error_speed = error;
    vel_rxed = false;
  }

  void timerCallback()
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = -set_lin_x; // Initially, don't move forward or backward
    twist.angular.z = command_ang_z; // Use PID output to adjust angular velocity
    cmd_vel_publisher_->publish(twist);
    RCLCPP_INFO(this->get_logger(), "Publishing velocity command: linear.x = '%.2f', angular.z = '%.2f'", twist.linear.x, twist.angular.z);
  }

  void teleVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    rclcpp::Time current_time = this->now();
    delta_time = (current_time - last_call_time_).seconds();
    last_call_time_ = current_time;
    
    vel_rxed = true;
    command_lin_x = msg->linear.x;
    command_ang_z = msg->angular.z;
    double error = command_lin_x - set_lin_x;
    integral_setpoint += error*delta_time;
    double derivative = (error - prev_error_setpoint)/delta_time;
    setpoint_ = kp_setpoint*error + ki_setpoint*integral_setpoint + kd_setpoint*derivative;
    prev_error_setpoint = error;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr tele_vel_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  double kp_speed, ki_speed, kd_speed, integral_speed, prev_error_speed, setpoint_;
  double kp_setpoint, ki_setpoint, kd_setpoint, integral_setpoint, prev_error_setpoint;
  double roll, pitch, yaw;
  bool vel_rxed;
  double command_lin_x, command_ang_z;
  double set_lin_x;
  double delta_time;
  rclcpp::Time last_call_time_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Controller>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

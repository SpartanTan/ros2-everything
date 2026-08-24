#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
      : Node("Yaw_pubisher"), count_(0)
    {
        q.setRPY(0, 0, 0);
        subscription_ =
          this->create_subscription<sensor_msgs::msg::Imu>(
            imuTopicName_, 10,
            std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {

        if (imuTopicName_ == "/eufs/imu") {
            tf2::Quaternion q;
            tf2::convert(msg->orientation, q);
            double roll, pitch, yaw;
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            RCLCPP_INFO(this->get_logger(), "Raw Quaternion: %f %f %f %f", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
            RCLCPP_INFO(this->get_logger(), "Quaternion q: %f %f %f %f", q.x(), q.y(), q.z(), q.w());
            yaw = yaw * 180 / M_PI;
            RCLCPP_INFO(this->get_logger(), "Yaw: %f", yaw);

        } else {
            RCLCPP_INFO(this->get_logger(), "Yaw: %f", msg->orientation.z); 
        }
    }

    std::string imuTopicName_ = "/imu";
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    size_t count_;
    tf2::Quaternion q;
    double yaw = 0.0;
    geometry_msgs::msg::Quaternion q_msg;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}

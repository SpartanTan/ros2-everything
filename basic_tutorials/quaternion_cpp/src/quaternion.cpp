#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
      : Node("quaternion_publisher"), count_(0)
    {
        q.setRPY(0, 0, 0);
        publisher_ = this->create_publisher<geometry_msgs::msg::Quaternion>("quaternion_topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        yaw += 0.1;
        q.setRPY(0, 0, yaw);
        q_msg = tf2::toMsg(q);
        tf2::Quaternion q_from_msg;
        tf2::convert(q_msg, q_from_msg); 
        

        RCLCPP_INFO(this->get_logger(), "Origin q: %f %f %f %f",
            q.x(), q.y(), q.z(), q.w());
        RCLCPP_INFO(this->get_logger(), "Q_from_msg: %f %f %f %f",
            q_from_msg.x(), q_from_msg.y(), q_from_msg.z(), q_from_msg.w());

        // recover yaw
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        RCLCPP_INFO(this->get_logger(), "RPY: %f %f %f", roll, pitch, yaw);

        publisher_->publish(q_msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr publisher_;
    size_t count_;
    tf2::Quaternion q;
    double yaw = 0.0;
    geometry_msgs::msg::Quaternion q_msg;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
    FrameListener()
      : Node("spawner_test"),
        turtle_spawning_service_ready_(false),
        turtle_spawned_(false)
    {
        target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle1");
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 1);

        timer_ = this->create_wall_timer(1s, std::bind(&FrameListener::on_timer_, this));
        spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");
    }

private:
    void on_timer_()
    {
        std::string fromFrameRel = "world";//target_frame_.c_str();
        std::string toFrameRel = "turtle2";
        if (turtle_spawning_service_ready_) {
            RCLCPP_INFO(this->get_logger(), "Spawn turtle 2 success");
            geometry_msgs::msg::TransformStamped t;

            // check if can find the tarnsform
            try {
                t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);

                RCLCPP_INFO(
                    this->get_logger(), "x: %f, y: %f",t.transform.translation.x, t.transform.translation.y);


            } catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(
                    this->get_logger(), "Could not transform %s to %s: %s",
                    toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
                return;
            }

            geometry_msgs::msg::Twist msg;
            static const double scaleRtationRate = 1.0;
            msg.angular.z = scaleRtationRate * atan2(
                t.transform.translation.y,
                t.transform.translation.x);

            static const double scaleFowardSpeed = 0.5;
            msg.linear.x = scaleFowardSpeed *
              sqrt(pow(t.transform.translation.x, 2) + pow(t.transform.translation.y, 2));
            publisher_->publish(msg);

        } else {
            if (spawner_->service_is_ready()) {
                RCLCPP_INFO(this->get_logger(), "Service is ready");
                auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
                request->x = 4.0;
                request->y = 2.0;
                request->theta = 0.0;
                request->name = "turtle2";

                using ServiceResponseFuture = rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
                auto response_received_callback = [this](ServiceResponseFuture future) {
                      auto result = future.get();
                      if (strcmp(result->name.c_str(), "turtle2") == 0) {
                          turtle_spawning_service_ready_ = true;
                      } else {
                          RCLCPP_ERROR(this->get_logger(), "Service callback result mismatch");
                      }
                  };
                auto result = spawner_->async_send_request(request, response_received_callback);
            } else {
                RCLCPP_INFO(this->get_logger(), "Service is not ready");
            }
        }
    }


    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    bool turtle_spawning_service_ready_;
    bool turtle_spawned_;

    std::string target_frame_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameListener>());
    rclcpp::shutdown();
}

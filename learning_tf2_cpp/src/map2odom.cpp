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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void transformTF2ToMsg(
    const tf2::Transform & tf2, geometry_msgs::msg::TransformStamped & msg,
    rclcpp::Time stamp, const std::string & frame_id,
    const std::string & child_frame_id)
{
    // transformTF2ToMsg(tf2, msg.transform);
    msg.transform.translation.x = tf2.getOrigin().x();
    msg.transform.translation.y = tf2.getOrigin().y();
    msg.transform.translation.z = tf2.getOrigin().z();
    msg.transform.rotation.x = tf2.getRotation().x();
    msg.transform.rotation.y = tf2.getRotation().y();
    msg.transform.rotation.z = tf2.getRotation().z();
    msg.transform.rotation.w = tf2.getRotation().w();

    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.child_frame_id = child_frame_id;
}

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
    FrameListener()
      : Node("turtle_tf2_frame_listener"),
        turtle_spawning_service_ready_(false),
        turtle_spawned_(false)
    {
        target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle1");
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 1);
        timer_ = this->create_wall_timer(1s, std::bind(&FrameListener::on_timer_, this));
    }

private:
    void on_timer_()
    {
        std::string fromFrameRel = "odom";
        std::string toFrameRel = "car";

        geometry_msgs::msg::TransformStamped map2odom;
        geometry_msgs::msg::TransformStamped map2car;


        if (turtle_spawning_service_ready_) {
            if (turtle_spawned_) {
                geometry_msgs::msg::TransformStamped t;

                try {
                    t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
                    
                    geometry_msgs::msg::TransformStamped odom2car;
                    odom2car = t;

                    tf2::Stamped<tf2::Transform> odom2car_transform;
                    tf2::Stamped<tf2::Transform> map2car_transform;
                    tf2::Transform map2odom_transform;

                    // tf2::fromMsg<geometry_msgs::msg::TransformStamped, tf2::Stamped<tf2::Transform>>(odom2car, odom2car_transform);

                    // tf2::fromMsg<geometry_msgs::msg::TransformStamped, tf2::Stamped<tf2::Transform>>(map2car, map2car_transform);

                    tf2::fromMsg(odom2car, odom2car_transform);

                    tf2::fromMsg(map2car, map2car_transform);
                    map2odom_transform = map2car_transform.inverseTimes(odom2car_transform);

                    transformTF2ToMsg(map2odom_transform, map2odom, this->now(), "map", "odom");

                } catch (const tf2::TransformException & ex) {
                    RCLCPP_INFO(
                        this->get_logger(), "Could not transform %s to %s: %s",
                        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
                    return;
                }
                RCLCPP_INFO(
                    this->get_logger(), "x: %f, y: %f", t.transform.translation.x,
                    t.transform.translation.y);
                geometry_msgs::msg::Twist msg;

                static const double scaleRotationRate = 1.0;
                msg.angular.z = scaleRotationRate * atan2(
                    t.transform.translation.y,
                    t.transform.translation.x);

                static const double scaleForwardSpeed = 0.5;
                msg.linear.x = scaleForwardSpeed * sqrt(
                    pow(t.transform.translation.y, 2) +
                    pow(t.transform.translation.x, 2));
                publisher_->publish(msg);
            } else {
                RCLCPP_INFO(this->get_logger(), "Successfully spawned");
                turtle_spawned_ = true;
            }
        } else {
            if (spawner_->service_is_ready()) {
                auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
                request->x = 4.0;
                request->y = 2.0;
                request->theta = 0.0;
                request->name = "turtle2";

                using ServiceResponseFuture = rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
                auto response_rceived_callback = [this](ServiceResponseFuture future) {
                      auto result = future.get();
                      if (strcmp(result->name.c_str(), "turtle2") == 0) {
                          turtle_spawning_service_ready_ = true;
                      } else {
                          RCLCPP_ERROR(this->get_logger(), "Service callback result mismatch");
                      }
                  };
                auto result = spawner_->async_send_request(request, response_rceived_callback);
            } else {
                RCLCPP_INFO(this->get_logger(), "Service is not ready");
            }
        }
    }
    bool turtle_spawning_service_ready_;
    bool turtle_spawned_;

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameListener>());
    rclcpp::shutdown();

    return 0;
}

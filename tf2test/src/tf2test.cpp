#include <memory>
#include <chrono>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class TFPublisher : public rclcpp::Node
{
public:
    TFPublisher()
      : Node("TFPublisher"), count_(0)
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&TFPublisher::timer_callback, this));

    }

private:
    void timer_callback()
    {
        count_ = count_ + 0.2;
        pub_tf();
        RCLCPP_INFO_STREAM(this->get_logger(), "countttt: " << count_ << std::endl);
    }

    void pub_tf()
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "car";

        t.transform.translation.x = 1.0;
        t.transform.translation.y = count_;
        t.transform.translation.z = 0.0;

        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(t);
    }
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    float count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFPublisher>());
    rclcpp::shutdown();
    return 0;
}

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
      : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("visual_point_n_line", 10);
        markerArrayPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "marker_array", 1);
        markerPub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "marker_points", 1);

        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {

        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
        visualization();
    }

    void visualization()
    {
        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/map";
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = this->now();
        points.ns = line_strip.ns = line_list.ns = "points_and_lines";
        points.action = line_strip.action = line_list.action = points.ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w =
          1.0;

        points.id = 0;
        line_strip.id = 1;
        line_list.id = 2;

        points.type = points.POINTS;
        line_strip.type = line_strip.LINE_STRIP;
        line_list.type = line_list.LINE_LIST;

        points.scale.x = 0.2;
        points.scale.y = 0.2;

        line_strip.scale.x = 0.1;
        line_list.scale.x = 0.1;

        points.color.g = 1.0;
        points.color.a = 1.0;

        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        line_list.color.r = 1.0;
        line_list.color.a = 1.0;

        for (int i = 0; i < 100; i++) {
            float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
            float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

            geometry_msgs::msg::Point p;
            p.x = (int)i - 50;
            p.y = y;
            p.z = z;

            points.points.push_back(p);
            line_strip.points.push_back(p);

            line_list.points.push_back(p);
            p.z += 1.0;
            line_list.points.push_back(p);
        }
        markerPub_->publish(points);
        markerPub_->publish(line_strip);
        markerPub_->publish(line_list);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerArrayPub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub_;
    visualization_msgs::msg::Marker points, line_strip, line_list;
    float f = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}

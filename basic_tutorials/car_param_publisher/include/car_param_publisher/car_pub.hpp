#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

class CarParameter : public rclcpp::Node
{
public:
    CarParameter();

    void carParamPrint();
    void init_parameters();

private:

    float weight{200.0}, front_ratio{7.2}, rear_ratio{7.3};
    std_msgs::msg::Float32MultiArray carParams_;      
    // float carParams[3]={weight,front_ratio,rear_ratio};
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr car_param_publisher;
    size_t count_;
    std_msgs::msg::String msg = std_msgs::msg::String();
};

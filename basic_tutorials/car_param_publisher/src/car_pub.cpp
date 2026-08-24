#include "car_param_publisher/car_pub.hpp"

using namespace std::chrono_literals;

CarParameter::CarParameter()
  : Node("car_param_node"), count_(0)
{
    init_parameters();
    RCLCPP_INFO(this->get_logger(), "Creating car param publisher");
    car_param_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("car_param", 1);
    // carParams.data[0] = weight;
    // carParams.data[1] = front_ratio;
    // carParams.data[2] = rear_ratio;
    timer_ = this->create_wall_timer(500ms, std::bind(&CarParameter::carParamPrint, this));

}

void CarParameter::carParamPrint()
{
    // RCLCPP_INFO(get_logger(), "Publishing parameters for the %d time", count_++);

    // carParams.data[0] = weight;
    // std::vector<float> tt = {1,2,3};
    this->get_parameter("car_weight", weight);
    this->get_parameter("front_ratio", front_ratio);
    this->get_parameter("rear_ratio", rear_ratio);
    
    carParams_.data = std::vector<float> {weight, front_ratio, rear_ratio};
    // RCLCPP_INFO(get_logger(), "car param: %f", carParams_.data[0]);
    // RCLCPP_INFO_STREAM(
    //     get_logger(),
    //     "weight = " << carParams_.data[0] << " front_ratio = " << carParams_.data[1] << " rear_ratio = " <<
    //         carParams_.data[2]);
    RCLCPP_INFO_STREAM(
        get_logger(),
        "weight = " << weight << " front_ratio = " << front_ratio << " rear_ratio = " << rear_ratio);
    car_param_publisher->publish(carParams_);
}

void CarParameter::init_parameters()
{
    this->declare_parameter("car_weight", 200.0);
    this->declare_parameter("front_ratio",0.45);
    this->declare_parameter("rear_ratio", 0.33);

    this->get_parameter("car_weight", weight);
    this->get_parameter("front_ratio", front_ratio);
    this->get_parameter("rear_ratio", rear_ratio);
}

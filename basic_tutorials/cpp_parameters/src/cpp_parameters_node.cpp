#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>

using namespace std::chrono_literals;

class ParametersClass : public rclcpp::Node
{
public:
    ParametersClass()
      : Node("parameter_node")
    {
        this->declare_parameter<std::string>("my_parameter", "world");
        auto testoutput = this->declare_parameter<float>("car_weight", 255);
        RCLCPP_INFO(this->get_logger(),"testoutput is %f", testoutput);
        // timer_ = this->create_wall_timer(100ms, std::bind(&ParametersClass::respond, this));
    }
    void respond()
    {
        this->get_parameter("my_parameter", parameter_string_);
        this->get_parameter("car_weight", car_weight);
        RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());
        RCLCPP_INFO(this->get_logger(), "car weight is %f", car_weight);
    }

private:
    std::string parameter_string_;
    rclcpp::TimerBase::SharedPtr timer_;
    float car_weight;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParametersClass>());
    rclcpp::shutdown();
    return 0;
}

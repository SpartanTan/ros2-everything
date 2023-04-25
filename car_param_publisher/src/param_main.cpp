#include "car_param_publisher/car_pub.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarParameter>());
    rclcpp::shutdown();

    return 0;
}
#include <string>
#include <chrono>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main()
{
    std::string mcShareDirectory = ament_index_cpp::get_package_share_directory(
        "rclcpp_test");
    
    std::cout << mcShareDirectory << std::endl;
    return 0;
}

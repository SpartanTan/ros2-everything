#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

using AddTwoInts = example_interfaces::srv::AddTwoInts;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
}
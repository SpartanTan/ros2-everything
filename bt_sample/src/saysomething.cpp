#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace BT;

class SaySomething : public SyncActionNode
{
public:
SaySomething(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config)
{}

static PortsList providedPorts()
{
    return {InputPort<std::string>("message") };
}

NodeStatus tick() override
{
    Expected<std::string> msg = getInput<std::string>("message");
    if(!msg)
    {
        throw BT::RuntimeError("missing required input [message]: ",  msg.error() );
    }

    std::cout << "Robot says: " << msg.value() << std::endl;
    return NodeStatus::SUCCESS;
}

};


class ThinkWhatToSay : public SyncActionNode
{
public:
ThinkWhatToSay(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config)
{}

static PortsList providedPorts()
{
    return { OutputPort<std::string>("text") };
}

NodeStatus tick() override
{
    setOutput("text", "The answer is 42." );
    return NodeStatus::SUCCESS;
}
};

int main()
{
    BehaviorTreeFactory factory;
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");
    
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("bt_sample");
    auto tree = factory.createTreeFromFile(pkg_share_dir + "/trees/tree2.xml");
    
    tree.tickWhileRunning();

    return 0;
}
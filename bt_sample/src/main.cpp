#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

class ApproachObject : public BT::SyncActionNode
{
public:
ApproachObject(const std::string& name) : BT::SyncActionNode(name, {})
{}

BT::NodeStatus tick() override
{
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return BT::NodeStatus::FAILURE;
}

};

BT::NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

class GripperInterface
{
public:
GripperInterface(): _open(true) {}

BT::NodeStatus open()
{
    _open = true;
    std::cout << "GripperInterface::open" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus close()
{
    std::cout << "GripperInterface::close" << std::endl;
    _open = false;
    return BT::NodeStatus::SUCCESS;
}

private:
bool _open;
};


int main()
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ApproachObject>("ApproachObject");
    factory.registerSimpleCondition("CheckBattery", [&](BT::TreeNode&) {return CheckBattery(); });

    GripperInterface gripper;

    factory.registerSimpleAction("OpenGripper", [&](BT::TreeNode&) { return gripper.open(); });
    factory.registerSimpleAction("CloseGripper", [&](BT::TreeNode&) { return gripper.close(); });

    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("bt_sample");
    auto tree = factory.createTreeFromFile(pkg_share_dir + "/trees/tree1.xml");

    tree.tickWhileRunning();

    return 0;
}
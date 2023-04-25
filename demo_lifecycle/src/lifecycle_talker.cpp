#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
    /// LifecycleTalker constructor
    /**
     * The lifecycletalker/lifecyclenode constructor has the same
     * arguments a regular node.
     */
    explicit LifecycleTalker(const std::string & node_name, bool intra_process_comms = false)
      : rclcpp_lifecycle::LifecycleNode(node_name,
          rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {}

    void publish()
    {
        static size_t count = 0;
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Lifecycle HelloWorld #" + std::to_string(++count);

        if (!pub_->is_activated()) {
            RCLCPP_INFO(
                get_logger(),
                "Lifecycle publisher is currently inactive. Messages are not published.");
        } else {
            RCLCPP_INFO(
                get_logger(), "Lifecycle publisher is active. Publishing: [%s]", msg->data.c_str());
        }

        // We independently from the current state call publish on the lifecycle
        // publisher.
        // Only if the publisher is in an active state, the message transfer is
        // enabled and the message actually published.
        pub_->publish(std::move(msg));
    }

    /// Transition callback for state configuring
    /**
     * on_configure callback is being called when the lifecycle node
     * enters the "configuring" state.
     * Depending on the return value of this function, the state machine
     * either invokes a transition to the "inactive" state or stays
     * in "unconfigured".
     * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
     * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
     * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &)
    {
        pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&LifecycleTalker::publish, this));

        RCLCPP_INFO(get_logger(), "on_configure() is called.");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /// Transition callback for state activating
    /**
     * on_activate callback is being called when the lifecycle node
     * enters the "activating" state.
     * Depending on the return value of this function, the state machine
     * either invokes a transition to the "active" state or stays
     * in "inactive".
     * TRANSITION_CALLBACK_SUCCESS transitions to "active"
     * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
     * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state)
    {
        // The parent class method automatically transition on managed entities
        // (currently, LifecyclePublisher).
        // pub_->on_activate() could also be called manually here.
        // Overriding this method is optional, a lot of times the default is enough.
        LifecycleNode::on_activate(state);

        RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

        // Let's sleep for 2 seconds.
        // We emulate we are doing important
        // work in the activating phase.
        std::this_thread::sleep_for(2s);

        // We return a success and hence invoke the transition to the next
        // step: "active".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the "inactive" state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::
               SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state)
    {
        // The parent class method automatically transition on managed entities
        // (currently, LifecyclePublisher).
        // pub_->on_deactivate() could also be called manually here.
        // Overriding this method is optional, a lot of times the default is enough.
        LifecycleNode::on_deactivate(state);

        RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

        // We return a success and hence invoke the transition to the next
        // step: "inactive".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the "active" state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &)
    {
        // In our cleanup phase, we release the shared pointers to the
        // timer and publisher. These entities are no longer available
        // and our node is "clean".
        timer_.reset();
        pub_.reset();

        RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

        // We return a success and hence invoke the transition to the next
        // step: "unconfigured".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the "inactive" state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & state)
    {
        // In our shutdown phase, we release the shared pointers to the
        // timer and publisher. These entities are no longer available
        // and our node is "clean".
        timer_.reset();
        pub_.reset();

        RCUTILS_LOG_INFO_NAMED(
            get_name(),
            "on shutdown is called from state %s.",
            state.label().c_str());

        // We return a success and hence invoke the transition to the next
        // step: "finalized".
        // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
        // would stay in the current state.
        // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
        // this callback, the state machine transitions to state "errorprocessing".
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    // std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
};


int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<LifecycleTalker> lc_node =
      std::make_shared<LifecycleTalker>("lc_talker");

    exe.add_node(lc_node->get_node_base_interface());
    exe.spin();
}

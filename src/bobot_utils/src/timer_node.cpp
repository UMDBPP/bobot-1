// |***************************************************************|
// |* (c) Copyright 2023                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <bobot_msgs/msg/bobot_timer.hpp> // No idea why it doesnt build as "BobotTimer.hpp" but we ball I gues

// C++ specific libraries
#include <atomic>
#include <string>

class BobotTimerNode : public rclcpp_lifecycle::LifecycleNode
{

public:

    // Constructor -  needs a node name and a boolean that indicates if we are using intra_process_commms (?))
    // TODO: is this something we need to fill, or will on__configure take care of this?
    explicit BobotTimerNode(const std::string &node_name, bool intra_process_comms = false) : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {}


    void publish_time_count()
    {
        auto timer_msg = std::make_unique<bobot_msgs::msg::BobotTimer>();
    }
    // Callback function for on_configure, a  lifecycle based method (believed to be inhertied)
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&)
    {
        // timer_publish = this->create_publisher
    }
        
    ~BobotTimerNode(); // Deconstructor also needs nothing

private:

    // BIG INFO BLOCKS ARE DIRECTLY FROM ROS
/*
    We hold an instance of a lifecycle publisher. This lifecycle publisher can 
    be activated or deactivated regarding on which state the lifecycle node is in.
    By default, a lifecycle publisher is inactive by creation and has to be
    activated to publish messages into the ROS world.
*/
    // This is a regular ROS publisher, BUT it follows the rules of the lifecycle management, as in it won't do JACK until you set a certain state
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<bobot_msgs::msg::BobotTimer>> timer_publisher_;

/*
    We hold an instance of a timer which periodically triggers the publish function.
    As for the beta version, this is a regular timer. In a future version, a
    lifecycle timer will be created which obeys the same lifecycle management as the
    lifecycle publisher.
*/  
    /// Same as above, this is a ros timer that regulates the rate at which the publisher publishhes data. Should follow lifecycle management (someho?)
    std::shared_ptr<rclcpp::TimerBase> pubbing_rate;


    uint64_t  time_val;

    




        
};


int main(int argc, char* argv[])
{

    return(0);
}
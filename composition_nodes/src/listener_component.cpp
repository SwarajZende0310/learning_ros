#include "composition_nodes/listener_component.hpp"

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace composition
{
    Listener::Listener(const rclcpp::NodeOptions &options): Node("listener",options)
    {
        auto callback = [this](std_msgs::msg::String::ConstSharedPtr msg)-> void
                        {
                            RCLCPP_INFO(this->get_logger(),"I heard: [%s]",  msg->data.c_str());
                            std::flush(std::cout);
                        };
        sub_ = create_subscription<std_msgs::msg::String>("chatter",10,callback);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Listener)
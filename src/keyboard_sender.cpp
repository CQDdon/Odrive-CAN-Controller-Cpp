#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>

class KeyboardInputNode : public rclcpp::Node {
public:
    KeyboardInputNode() : Node("keyboard_sender_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("encoded_data", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                         std::bind(&KeyboardInputNode::publishEncodedData, this));
    }

private:
    void publishEncodedData() {
        // Prompt user for input
        std::string user_input;
        std::cout << "Enter input (ID(hex),MODE,DATA(float)): ";
        std::getline(std::cin, user_input);

        auto message = std_msgs::msg::String();
        message.data = user_input;

        RCLCPP_INFO(this->get_logger(), "Data sent : %s", user_input.c_str());
        publisher_->publish(message);

        // Clear user input to save memory
        user_input.clear();
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardInputNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

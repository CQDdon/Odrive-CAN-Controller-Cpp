#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <cstring>

class KeyboardInputNode : public rclcpp::Node {
public:
    KeyboardInputNode() : Node("keyboard_sender_node") {
        publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("encoded_data", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&KeyboardInputNode::publishEncodedData, this));
    }

private:
    void publishEncodedData() {
        // Prompt user for input
        std::string user_input;
        std::cout << "Enter input (ID(hex),MODE,DATA(float)): ";
        std::getline(std::cin, user_input);

        // Parse user input
        std::stringstream ss(user_input);
        std::string id_str, mode_str, data_str;
        if (!std::getline(ss, id_str, ',') || !std::getline(ss, mode_str, ',') || !std::getline(ss, data_str)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid input format");
            return;
        }

        // Convert ID from hex string to integer
        int input_id;
        std::stringstream(id_str) >> std::hex >> input_id;
        if (input_id > 0x3F) { // 6-bit limit
            RCLCPP_ERROR(this->get_logger(), "Input ID exceeds 6-bit limit for 11-bit encoding");
            return;
        }

        // Determine mode and corresponding hexadecimal
        int mode_hex;
        if (mode_str == "vel") mode_hex = 0x0D;
        else if (mode_str == "ang") mode_hex = 0x0C;
        else {
            RCLCPP_ERROR(this->get_logger(), "Invalid mode: %s", mode_str.c_str());
            return;
        }

        // Encode ID: ID_input << 5 | mode_hex
        int encoded_id = ((input_id << 5) | mode_hex) & 0x7FF;

        // Convert data from string to float
        float data_float = std::stof(data_str);

        // Convert float to IEEE 754 format (little-endian)
        uint8_t data_bytes[4];
        std::memcpy(data_bytes, &data_float, sizeof(data_float));

        // Prepare message
        std_msgs::msg::ByteMultiArray message;
        uint8_t encoded_id_low = encoded_id & 0xFF;
        uint8_t encoded_id_high = (encoded_id >> 8) & 0x07; // 3 bits

        message.data.push_back(encoded_id_high);
        message.data.push_back(encoded_id_low);
        for (int i = 0; i < 4; ++i) {
            message.data.push_back(data_bytes[i]);
        }

        // Log and publish the message
        std::ostringstream hex_data_array;
        hex_data_array << std::hex << std::setfill('0');
        for (uint8_t b : data_bytes) {
            hex_data_array << "0x" << std::setw(2) << (int)b << " ";
        }
        RCLCPP_INFO(this->get_logger(), "Encoded ID: 0x%X, Data: %s", encoded_id, hex_data_array.str().c_str());

        publisher_->publish(message);

        // Clear user input to save memory
        user_input.clear();
    }

    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardInputNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

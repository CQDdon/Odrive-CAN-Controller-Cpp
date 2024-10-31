#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> // For ByteMultiArray messages
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>
#include <array>
#include <iostream>
#include <vector>

using namespace std;

/*
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
*/

class CANUSBNode : public rclcpp::Node {
public:
    CANUSBNode() : Node("odrive_sender_node") {
        // Create socket for CAN communication
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error creating socket");
            return;
        }

        // Bind the socket to can0 interface
        struct ifreq ifr;
        strncpy(ifr.ifr_name, "vcan0", IFNAMSIZ);
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error binding to can0");
            return;
        }

        struct sockaddr_can addr;
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error binding socket to can0 interface");
            return;
        }

        // Subscription to listen to `encoded_data` topic
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "encoded_data", 10, bind(&CANUSBNode::listener_callback, this, placeholders::_1)
        );
    }

    ~CANUSBNode() {
        if (can_socket_ >= 0) {
            close(can_socket_);
        }
    }

private:
    int can_socket_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

    vector<uint8_t> pad_vector(const vector<uint8_t>& data, size_t target_length = 8, uint8_t pad_value = 0) {
        vector<uint8_t> padded_data = data;
        if (padded_data.size() < target_length) {
            padded_data.resize(target_length, pad_value);
        } else if (padded_data.size() > target_length) {
            padded_data.resize(target_length);
        }
        return padded_data;
    }

    void listener_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data.size() != 6) {
            RCLCPP_ERROR(this->get_logger(), "Invalid data length; expected 6 bytes (2 ID and 4 data bytes)");
            return;
        }

        // Extract ID and data bytes from the message
        can_frame frame;
        frame.can_id = msg->data[0] << 8 | msg->data[1];
        auto data_bytes = vector<uint8_t>(msg->data.begin() + 2, msg->data.end());
        data_bytes = pad_vector(data_bytes);

        memcpy(frame.data, data_bytes.data(), 8);
        frame.can_dlc = 8;

        // Print the data in hex format
        stringstream ss;
        ss << hex << (int)frame.can_id << "#";
        for (int i = 0; i < frame.can_dlc; ++i) {
            ss << hex << (int)frame.data[i];
        }
        

        // Send CAN message
        if (write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
            RCLCPP_ERROR(this->get_logger(), "Message NOT sent");
        } else {
            RCLCPP_INFO(this->get_logger(), "Message sent on can0: ");
            cout << ss.str() << endl;
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = make_shared<CANUSBNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
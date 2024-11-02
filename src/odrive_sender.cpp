#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> 
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

    void pad_array(const uint8_t* data, size_t data_length, uint8_t* padded_data, size_t target_length = 8, uint8_t pad_value = 0) {
        // Copy data to padded_data
        std::memcpy(padded_data, data, std::min(data_length, target_length));

        // Pad with pad_value if needed
        if (data_length < target_length) {
            std::fill(padded_data + data_length, padded_data + target_length, pad_value);
        }
    }

    void listener_callback(const std_msgs::msg::String::SharedPtr msg) {

        // Parse data from publisher
        std::stringstream ss(msg->data);
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
        bool vel_flag = 0;
        bool ang_flag = 0;
        if (mode_str == "vel") {
            mode_hex = 0x0D;
            vel_flag = 1;
            ang_flag = 0;
        }
        else if (mode_str == "ang") {
            mode_hex = 0x0C;
            ang_flag = 1;
            vel_flag = 0;
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

        // Extract ID and data bytes from the message
        can_frame frame;
        frame.can_id = encoded_id;

        uint8_t data_send[8];
        pad_array(data_bytes, 4, data_send);

        memcpy(frame.data, data_send, 8);
        frame.can_dlc = 8;

        // Print the data in hex format
        stringstream sts;
        sts << hex << (int)frame.can_id << "#";
        for (int i = 0; i < frame.can_dlc; ++i) {
            sts << hex << (int)frame.data[i];
        }
        

        // Send CAN message
        if (write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
            RCLCPP_ERROR(this->get_logger(), "Message NOT sent");
        } else {
            RCLCPP_INFO(this->get_logger(), "Message sent on can0: ");
            cout << sts.str() << endl;
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

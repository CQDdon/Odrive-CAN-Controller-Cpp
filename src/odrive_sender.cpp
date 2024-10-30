// Since this code was wrote stupidly without any understanding, please check this code carefully (better using chatGPT to regenerate it)

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp> // For ByteMultiArray messages
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
        strncpy(ifr.ifr_name, "can0", IFNAMSIZ);
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
        subscriber_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
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
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscriber_;

    vector<uint8_t> pad_vector(const vector<uint8_t>& data, size_t target_length = 8, uint8_t pad_value = 0) {
        vector<uint8_t> padded_data = data;
        if (padded_data.size() < target_length) {
            padded_data.resize(target_length, pad_value);
        } else if (padded_data.size() > target_length) {
            padded_data.resize(target_length);
        }
        return padded_data;
    }

    void listener_callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
        if (msg->data.size() != 5) {
            RCLCPP_ERROR(this->get_logger(), "Invalid data length; expected 5 bytes (1 ID and 4 data bytes)");
            return;
        }

        // Extract ID and data bytes from the message
        can_frame frame;
        frame.can_id = msg->data[0];
        auto data_bytes = vector<uint8_t>(msg->data.begin() + 1, msg->data.end());
        data_bytes = pad_vector(data_bytes);

        memcpy(frame.data, data_bytes.data(), 8);
        frame.can_dlc = 8;

        // Print the data in hex format
        stringstream ss;
        ss << hex << (int)frame.can_id << "#";
        for (int i = 0; i < frame.can_dlc; ++i) {
            ss << hex << (int)frame.data[i];
        }
        cout << ss.str() << endl;

        // Send CAN message
        if (write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
            RCLCPP_ERROR(this->get_logger(), "Message NOT sent");
        } else {
            RCLCPP_INFO(this->get_logger(), "Message sent on can0");
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

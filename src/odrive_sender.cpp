// Since this code was wrote stupidly without any understanding, please check this code carefully (better using chatGPT to regenerate it)

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

class odriveSender : public rclcpp::Node
{
private:
  void topic_callback(const std_msgs::msg::Byte & msg) const
  {
    /*
    Code here if you trust me unconditional
    */
  }
  rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr subscription_;

public:
  odriveSender()
  : Node("odrive_sender")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Byte>(
      "topic", 10, std::bind(&odriveSender::topic_callback, this, _1));
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<odriveSender>());
  rclcpp::shutdown();
  return 0;
}
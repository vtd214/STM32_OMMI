#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class AppController : public rclcpp::Node
{
public:
  AppController()
  : Node("app_controller")
  {
    subscription_ =
      this->create_subscription<std_msgs::msg::String>(
        "velocity",
        10,
        std::bind(
          &AppController::topic_callback,
          this,
          _1));
  }

private:
  void topic_callback(
    const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(
      this->get_logger(),
      "I heard: '%s'",
      msg.data.c_str());
  }

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AppController>());
  rclcpp::shutdown();
  return 0;
}

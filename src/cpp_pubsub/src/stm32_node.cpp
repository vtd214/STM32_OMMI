#include "chrono"
#include "functional"
#include "memory"
#include "string"

// ROS libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Topic: data type ==> send: string, int 

using namespace std::chrono_literals;

class Stm32Node : public rclcpp::Node {
    public:
        Stm32Node() : Node("stm32_node") {
            publisher_ = this->create_publisher<std_msgs::msg::String>("velocity", 10);
            timer_ = this->create_wall_timer(500ms, std::bind(&Stm32Node::TimerCallback, this));
        }


    private:
        void TimerCallback() {
            auto msg = std_msgs::msg::String();
            msg.data = "Velocity: " + std::to_string(velocity);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
            publisher_->publish(msg);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        int velocity = 5;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Stm32Node>());

    rclcpp::shutdown();
    return 0;
}
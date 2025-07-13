// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/bool.hpp>

// class SyncNode : public rclcpp::Node {
// public:
//     SyncNode();

// private:
//     void timer_callback();

//     std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> drone_subs;
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr proceed_pub;
//     rclcpp::TimerBase::SharedPtr timer;
//     std::vector<bool> drone_reached;
//     size_t nb_drones;
// };
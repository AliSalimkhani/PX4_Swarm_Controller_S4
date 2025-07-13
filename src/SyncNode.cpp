// #include "SyncNode.hpp"

// SyncNode::SyncNode() : Node("sync_node") {
//     this->declare_parameter<int>("nb_drones");
//     nb_drones = static_cast<size_t>(this->get_parameter("nb_drones").as_int());
//     drone_reached.assign(nb_drones, false);

//     for (size_t i = 0; i < nb_drones; ++i) {
//         std::string topic = "/px4_" + std::to_string(i + 1) + "/waypoint_reached";
//         drone_subs.push_back(this->create_subscription<std_msgs::msg::Bool>(
//             topic, 10, [this, i](const std_msgs::msg::Bool::SharedPtr msg) {
//                 drone_reached[i] = msg->data;
//             }));
//     }

//     proceed_pub = this->create_publisher<std_msgs::msg::Bool>("/sync/proceed", 10);
//     timer = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { timer_callback(); });
// }

// void SyncNode::timer_callback() {
//     if (std::all_of(drone_reached.begin(), drone_reached.end(), [](bool v) { return v; })) {
//         std_msgs::msg::Bool msg;
//         msg.data = true;
//         proceed_pub->publish(msg);
//     }
// }

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<SyncNode>());
//     rclcpp::shutdown();
//     return 0;
// }
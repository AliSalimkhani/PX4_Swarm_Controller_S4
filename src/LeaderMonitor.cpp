// File: src/LeaderMonitor.cpp
// Description: This file contains the corrected code for the LeaderMonitor node.
// The main issue was using the message's internal timestamp (from the drone's clock)
// instead of the reception time (from the ROS clock) to check for leader liveness.
// This caused incorrect timeout calculations.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <random> // برای تولید اعداد تصادفی جهت انتخاب لیدر جدید

class LeaderMonitor : public rclcpp::Node {
private:
    int nb_drones;
    int current_leader_id;
    int current_wp_idx = 0;
    std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr> subs;
    std::vector<rclcpp::Time> last_msg_time;
    std::vector<int> leader_miss_count;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr leader_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr waypoint_pub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr waypoint_sub;
    rclcpp::TimerBase::SharedPtr timer;

public:
    LeaderMonitor() : Node("leader_monitor") {
        this->declare_parameter<int>("nb_drones", 3);
        this->get_parameter("nb_drones", nb_drones);
        RCLCPP_INFO(this->get_logger(), "Monitoring %d drones", nb_drones);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
        qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
        qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        qos_profile.depth = 20;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

        leader_pub = this->create_publisher<std_msgs::msg::Int32>("/simulation/new_leader_id", 10);
        waypoint_pub = this->create_publisher<std_msgs::msg::Int32>("/simulation/current_waypoint_idx", 10);
        waypoint_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/simulation/current_waypoint_idx", 10, std::bind(&LeaderMonitor::waypoint_callback, this, std::placeholders::_1));

        last_msg_time.assign(nb_drones, this->now());
        leader_miss_count.assign(nb_drones, 0);
        current_leader_id = 0;

        for (int i = 0; i < nb_drones; i++) {
            std::string topic = "/px4_" + std::to_string(i + 1) + "/fmu/out/vehicle_local_position_agent";
            subs.push_back(this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                topic, qos,
                [this, i](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                    // --- FIX START ---
                    // The original code used `rclcpp::Time(msg->timestamp * 1000)`, which is incorrect
                    // because msg->timestamp is from the drone's internal clock, not the ROS clock.
                    // We must use the time of message reception (`this->now()`) for accurate liveness checking.
                    last_msg_time[i] = this->now();
                    // --- FIX END ---
                    leader_miss_count[i] = 0;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "LeaderMonitor: Received position from drone %d (via agent): x=%.2f, y=%.2f, z=%.2f",
                                i + 1, msg->x, msg->y, msg->z);
                }));
        }

        std_msgs::msg::Int32 leader_msg;
        leader_msg.data = current_leader_id;
        leader_pub->publish(leader_msg);
        RCLCPP_INFO(this->get_logger(), "Initial leader assigned: %d", current_leader_id + 1);

        timer = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&LeaderMonitor::check_leader_alive, this));
    }

private:
    void waypoint_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_wp_idx = msg->data;
    }

    void check_leader_alive() {
        auto now_time = this->now();
        auto elapsed = (now_time - last_msg_time[current_leader_id]).seconds();
        RCLCPP_DEBUG(this->get_logger(), "Elapsed time since last message from leader %d: %.2f seconds",
                     current_leader_id + 1, elapsed);

        if (elapsed > 2.0) {
            leader_miss_count[current_leader_id]++;
            RCLCPP_WARN(this->get_logger(), "Leader %d missed update, miss count: %d",
                        current_leader_id + 1, leader_miss_count[current_leader_id]);

            if (leader_miss_count[current_leader_id] >= 3) {
                RCLCPP_ERROR(this->get_logger(), "Leader %d lost, reassigning...", current_leader_id + 1);
                int new_leader = pick_new_leader();
                if (new_leader >= 0) {
                    current_leader_id = new_leader;
                    leader_miss_count[new_leader] = 0;
                    std_msgs::msg::Int32 leader_msg;
                    leader_msg.data = new_leader;
                    leader_pub->publish(leader_msg);
                    RCLCPP_INFO(this->get_logger(), "New leader assigned: %d", new_leader + 1);

                    std_msgs::msg::Int32 wp_msg;
                    wp_msg.data = current_wp_idx;
                    waypoint_pub->publish(wp_msg);
                    RCLCPP_INFO(this->get_logger(), "Published current waypoint index %d for new leader %d.", current_wp_idx, new_leader + 1);
                } else {
                    RCLCPP_FATAL(this->get_logger(), "No active drones available to become leader!");
                }
            }
        } else {
            leader_miss_count[current_leader_id] = 0;
        }
    }

    int pick_new_leader() {
        std::vector<int> available_drones;
        double availability_timeout = 10.0; // Drones that sent a message within the last 10s are eligible

        for (int i = 0; i < nb_drones; i++) {
            if (i != current_leader_id && (this->now() - last_msg_time[i]).seconds() < availability_timeout) {
                available_drones.push_back(i);
            }
        }

        if (available_drones.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No active drones available to be the new leader!");
            return -1;
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, available_drones.size() - 1);
        return available_drones[dis(gen)];
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LeaderMonitor>());
    rclcpp::shutdown();
    return 0;
}

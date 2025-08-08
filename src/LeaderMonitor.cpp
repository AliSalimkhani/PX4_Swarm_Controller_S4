// File: src/LeaderMonitor.cpp
// Description: This file contains the corrected code for the LeaderMonitor node.
// The main issue was using the message's internal timestamp (from the drone's clock)
// instead of the reception time (from the ROS clock) to check for leader liveness.
// This caused incorrect timeout calculations.
// This version adds a ROS2 service to allow external disarming of the current leader drone,
// and extends it to initiate a landing sequence and then transfer leadership.
// The leader election logic is changed from random selection to selecting the closest
// available drone to the old leader's last known position.
// It now also handles ties by randomly selecting among drones that are equally closest.
// Crucially, when 'disarm_leader' is called, the leader election uses the drone's position
// at the moment the service was invoked as the reference point.
// This version includes enhanced logging for leader election debugging.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <random> // For generating random numbers
#include <std_srvs/srv/trigger.hpp> // For the new disarm service
#include <px4_msgs/srv/vehicle_command.hpp> // For sending vehicle commands (land, disarm)
#include <px4_msgs/msg/vehicle_command.hpp> // For the VehicleCommand message type
#include <limits> // For std::numeric_limits
#include <cmath> // For std::hypot
#include <vector> // For std::vector
#include <algorithm> // For std::shuffle

class LeaderMonitor : public rclcpp::Node {
private:
    int nb_drones;
    int current_leader_id;
    int current_wp_idx = 0;
    std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr> subs;
    std::vector<rclcpp::Time> last_msg_time;
    std::vector<int> leader_miss_count;
    // Member variable to store the last known position of each drone
    std::vector<px4_msgs::msg::VehicleLocalPosition> last_positions; 
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr leader_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr waypoint_pub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr waypoint_sub;
    rclcpp::TimerBase::SharedPtr timer;

    // Member variable for the disarm service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_service_;

    // Temporary variables to store the old leader's position when disarm service is called
    // This ensures the reference point for election is fixed at the time of service invocation.
    px4_msgs::msg::VehicleLocalPosition temp_old_leader_pos_for_election_;
    bool use_temp_leader_pos_for_election_ = false;

public:
    LeaderMonitor() : Node("leader_monitor") {
        this->declare_parameter<int>("nb_drones", 3);
        this->get_parameter("nb_drones", nb_drones);
        RCLCPP_INFO(this->get_logger(), "Monitoring %d drones", nb_drones);

        // Define QoS profile for subscriptions to PX4 topics
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
        qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
        qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        qos_profile.depth = 20;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

        // Initialize publishers and subscribers
        leader_pub = this->create_publisher<std_msgs::msg::Int32>("/simulation/new_leader_id", 10);
        waypoint_pub = this->create_publisher<std_msgs::msg::Int32>("/simulation/current_waypoint_idx", 10);
        waypoint_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/simulation/current_waypoint_idx", 10, std::bind(&LeaderMonitor::waypoint_callback, this, std::placeholders::_1));

        // Initialize last message times and miss counts for all drones
        last_msg_time.assign(nb_drones, this->now());
        leader_miss_count.assign(nb_drones, 0);
        // Initialize the last_positions vector with the correct size
        last_positions.resize(nb_drones); 
        current_leader_id = 0; // Start with drone 0 as the initial leader

        // Create subscriptions for each drone's local position
        for (int i = 0; i < nb_drones; i++) {
            std::string topic = "/px4_" + std::to_string(i + 1) + "/fmu/out/vehicle_local_position_agent";
            subs.push_back(this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                topic, qos,
                [this, i](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                    last_msg_time[i] = this->now(); // Use ROS reception time for liveness
                    last_positions[i] = *msg; // Store the last received position
                    leader_miss_count[i] = 0; // Reset miss count on successful message reception
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "LeaderMonitor: Received position from drone %d (via agent): x=%.2f, y=%.2f, z=%.2f",
                                i + 1, msg->x, msg->y, msg->z);
                }));
        }

        // Publish initial leader ID
        std_msgs::msg::Int32 leader_msg;
        leader_msg.data = current_leader_id;
        leader_pub->publish(leader_msg);
        RCLCPP_INFO(this->get_logger(), "Initial leader assigned: %d", current_leader_id + 1);

        // Create a timer to periodically check leader liveness
        timer = this->create_wall_timer(
            std::chrono::milliseconds(1000), // Check every 1 second
            std::bind(&LeaderMonitor::check_leader_alive, this));

        // Create the new disarm service
        disarm_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/disarm_leader",
            std::bind(&LeaderMonitor::disarm_leader_callback, this, std::placeholders::_1, std::placeholders::_2));
            
        RCLCPP_INFO(this->get_logger(), "LeaderMonitor is ready. Use '/disarm_leader' service to disarm the current leader.");
    }

private:
    // Callback for current waypoint index updates
    void waypoint_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_wp_idx = msg->data;
        RCLCPP_DEBUG(this->get_logger(), "Received current waypoint index: %d", current_wp_idx);
    }

    // Function to check if the current leader is alive
    void check_leader_alive() {
        auto now_time = this->now();
        // Calculate elapsed time since the last message from the current leader
        auto elapsed = (now_time - last_msg_time[current_leader_id]).seconds();
        RCLCPP_DEBUG(this->get_logger(), "Elapsed time since last message from leader %d: %.2f seconds",
                     current_leader_id + 1, elapsed);

        // If no message received from the leader for more than 2 seconds
        if (elapsed > 2.0) {
            leader_miss_count[current_leader_id]++;
            RCLCPP_WARN(this->get_logger(), "Leader %d missed update, miss count: %d",
                        current_leader_id + 1, leader_miss_count[current_leader_id]);

            // If the leader has missed 3 consecutive updates, reassign leader
            if (leader_miss_count[current_leader_id] >= 3) {
                RCLCPP_ERROR(this->get_logger(), "Leader %d lost, attempting to reassign leader...", current_leader_id + 1);
                int new_leader = pick_new_leader(); // Try to find a new leader
                if (new_leader >= 0) {
                    current_leader_id = new_leader;
                    leader_miss_count[new_leader] = 0; // Reset miss count for the new leader
                    std_msgs::msg::Int32 leader_msg;
                    leader_msg.data = new_leader;
                    leader_pub->publish(leader_msg); // Publish the new leader ID
                    RCLCPP_INFO(this->get_logger(), "New leader assigned: %d", new_leader + 1);

                    // Publish the current waypoint index for the new leader to take over
                    std_msgs::msg::Int32 wp_msg;
                    wp_msg.data = current_wp_idx;
                    waypoint_pub->publish(wp_msg);
                    RCLCPP_INFO(this->get_logger(), "Published current waypoint index %d for new leader %d.", current_wp_idx, new_leader + 1);
                } else {
                    RCLCPP_FATAL(this->get_logger(), "No active drones available to become leader! System might be in a critical state.");
                }
            }
        } else {
            // If leader is alive, reset its miss count
            leader_miss_count[current_leader_id] = 0;
        }
    }

    // Function to pick a new leader from available drones based on proximity to a reference position.
    // If multiple drones are equally closest, one is chosen randomly from that subset.
    int pick_new_leader() {
        std::vector<int> available_drones;
        double availability_timeout = 10.0; // Drones that sent a message within the last 10s are eligible

        // Find all available drones (not the current leader and recently active)
        for (int i = 0; i < nb_drones; i++) {
            if (i != current_leader_id && (this->now() - last_msg_time[i]).seconds() < availability_timeout) {
                available_drones.push_back(i);
            } else {
                RCLCPP_INFO(this->get_logger(), "Drone %d is NOT available. Current leader: %d, Elapsed time: %.2f",
                            i + 1, current_leader_id + 1, (this->now() - last_msg_time[i]).seconds());
            }
        }

        if (available_drones.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No active drones available to be the new leader!");
            return -1; // Indicate no suitable leader found
        }

        std::vector<int> closest_candidates;
        double min_distance = std::numeric_limits<double>::max(); // Initialize with a very large value

        // Determine the reference position for distance calculation
        const px4_msgs::msg::VehicleLocalPosition* reference_pos_ptr;
        if (use_temp_leader_pos_for_election_) {
            reference_pos_ptr = &temp_old_leader_pos_for_election_;
            RCLCPP_INFO(this->get_logger(), "Using temporary old leader position for election: x=%.2f, y=%.2f, z=%.2f",
                        reference_pos_ptr->x, reference_pos_ptr->y, reference_pos_ptr->z);
            // Reset the flag immediately after using the temporary position
            use_temp_leader_pos_for_election_ = false; 
        } else {
            // Default behavior: use the last known position of the current (failed) leader
            reference_pos_ptr = &last_positions[current_leader_id];
            RCLCPP_INFO(this->get_logger(), "Using current leader's last known position for election: x=%.2f, y=%.2f, z=%.2f",
                        reference_pos_ptr->x, reference_pos_ptr->y, reference_pos_ptr->z);
        }
        const auto& old_leader_pos = *reference_pos_ptr;

        // Iterate through available drones to find the closest one(s)
        RCLCPP_INFO(this->get_logger(), "Evaluating %zu available candidates:", available_drones.size());
        for (int id : available_drones) {
            const auto& candidate_pos = last_positions[id];
            // Calculate Euclidean distance in 3D space
            double distance = std::hypot(candidate_pos.x - old_leader_pos.x,
                                         candidate_pos.y - old_leader_pos.y,
                                         candidate_pos.z - old_leader_pos.z);

            RCLCPP_INFO(this->get_logger(), "  Candidate Drone %d (x=%.2f, y=%.2f, z=%.2f) distance to old leader: %.2f",
                        id + 1, candidate_pos.x, candidate_pos.y, candidate_pos.z, distance);

            // If a new minimum distance is found, clear previous candidates and add this one
            if (distance < min_distance) { 
                min_distance = distance;
                closest_candidates.clear(); // Clear previous candidates as a new minimum is found
                closest_candidates.push_back(id);
            } 
            // If the distance is equal to the current minimum, add this drone as another candidate
            else if (distance == min_distance) {
                closest_candidates.push_back(id);
            }
        }

        // If no candidates found (shouldn't happen if available_drones is not empty, but for safety)
        if (closest_candidates.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No closest candidates found despite available drones. This indicates a logic error.");
            return -1;
        }

        // Randomly select one leader from the closest candidates
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, closest_candidates.size() - 1);
        int new_leader = closest_candidates[dis(gen)];

        RCLCPP_INFO(this->get_logger(), "Selected new leader %d based on closest proximity (distance: %.2f) to old leader %d. Chosen from %zu candidates.",
                    new_leader + 1, min_distance, current_leader_id + 1, closest_candidates.size());
        return new_leader;
    }

    // Callback function for the /disarm_leader service
    void disarm_leader_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // Suppress unused parameter warning
        RCLCPP_WARN(this->get_logger(), "Disarm service called for leader %d. Initiating land and leader transfer.", current_leader_id + 1);

        // Store the ID of the current leader before forcing the transfer
        int old_leader_id = current_leader_id;

        // NEW: Capture the old leader's position at the moment the service is invoked
        // This position will be used as the reference for electing the new leader.
        // Ensure last_positions[old_leader_id] is valid before accessing
        if (old_leader_id >= 0 && old_leader_id < nb_drones) {
            temp_old_leader_pos_for_election_ = last_positions[old_leader_id];
            use_temp_leader_pos_for_election_ = true; // Activate the flag to use this temporary position
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid old_leader_id %d for capturing position.", old_leader_id);
            response->success = false;
            response->message = "Invalid leader ID. Cannot initiate land and transfer.";
            return;
        }


        // 1. Send LAND command to the current leader
        std::string service_name = "/px4_" + std::to_string(old_leader_id + 1) + "/fmu/vehicle_command";
        auto client = this->create_client<px4_msgs::srv::VehicleCommand>(service_name);

        if (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Vehicle command service for drone %d not available. Cannot send LAND command.", old_leader_id + 1);
            response->success = false;
            response->message = "Vehicle command service not available for leader " + std::to_string(old_leader_id + 1) + ". Cannot land.";
            // IMPORTANT: If service is not available, reset the flag to prevent using stale temp data later.
            use_temp_leader_pos_for_election_ = false; 
            return;
        }

        // Create the LAND request
        using VehicleCommand = px4_msgs::msg::VehicleCommand;
        auto land_request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
        land_request->request.command = VehicleCommand::VEHICLE_CMD_NAV_LAND; // Command for landing
        land_request->request.param1 = 0.0f; // No specific landing target, land at current position
        land_request->request.param2 = 0.0f; // Reserved
        land_request->request.param3 = 0.0f; // Reserved
        land_request->request.param4 = 0.0f; // Reserved
        land_request->request.param5 = 0.0f; // Latitude (ignored if param1=0)
        land_request->request.param6 = 0.0f; // Longitude (ignored if param1=0)
        land_request->request.param7 = 0.0f; // Altitude (ignored if param1=0)
        land_request->request.target_system = 1;
        land_request->request.target_component = 1;
        land_request->request.from_external = true;
        land_request->request.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        // Send the LAND command asynchronously
        client->async_send_request(land_request);
        RCLCPP_INFO(this->get_logger(), "LAND command sent to current leader drone %d.", old_leader_id + 1);

        // Optional: Add a delay to allow the drone to start landing before potentially disarming or changing leader.
        // NOTE: This is a blocking delay. For real-time critical systems,
        // a state machine or monitoring drone's landing status would be better.
        // For simulation, a short delay might be acceptable.
        rclcpp::sleep_for(std::chrono::seconds(5)); // Wait 5 seconds for landing to begin

        // 2. Re-trigger leader election logic to transfer leadership
        // Force trigger leader loss for the old leader
        leader_miss_count[old_leader_id] = 3; 
        this->check_leader_alive(); // This will detect the "lost" leader and pick a new one

        // 3. Send DISARM command to the *original* leader (old_leader_id)
        //    This ensures the drone that was just the leader explicitly disarms.
        //    We need to create a new client for the old leader if the current_leader_id has changed.
        //    Using the same 'client' object as it's still valid for the same drone ID.
        //    If 'old_leader_id' is different from 'current_leader_id' after check_leader_alive(),
        //    then 'client' is still pointing to the old leader's service.
        
        auto disarm_request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
        disarm_request->request.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        disarm_request->request.param1 = 0.0f; // 0.0f means DISARM
        disarm_request->request.target_system = 1;
        disarm_request->request.target_component = 1;
        disarm_request->request.from_external = true;
        disarm_request->request.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        
        client->async_send_request(disarm_request); 
        RCLCPP_INFO(this->get_logger(), "Explicit DISARM command sent to old leader drone %d.", old_leader_id + 1);

        // Response reflects the initiation of the process, not necessarily its completion
        response->success = true;
        response->message = "LAND command sent to old leader " + std::to_string(old_leader_id + 1) + ". Initiated leader transfer.";
    }
};

int main(int argc, char *argv[]) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    // Create and spin the LeaderMonitor node
    rclcpp::spin(std::make_shared<LeaderMonitor>());
    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}

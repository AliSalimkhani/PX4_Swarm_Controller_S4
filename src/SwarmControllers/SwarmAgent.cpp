#include "SwarmControllers/SwarmAgent.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>  
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>

namespace Controller {

SwarmAgent::SwarmAgent(const rclcpp::NodeOptions &options)
: Node("swarm_agent", options),
  leader_initialized_(false),
  initial_takeoff_complete_(false),
  reliable_qos_(rclcpp::QoSInitialization(rmw_qos_profile_default.history, rmw_qos_profile_default.depth), rmw_qos_profile_default),
  sensor_qos_(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 10), rmw_qos_profile_sensor_data)
{
    const std::string name_space{this->get_namespace()};

    // --- Parameters ---
    this->declare_parameter<bool>("is_leader", false);
    this->declare_parameter<int>("drone_id", 0);
    this->declare_parameter<std::string>("wp_path", "");
    this->declare_parameter<double>("x_init", 0.0);
    this->declare_parameter<double>("y_init", 0.0);
    this->declare_parameter<std::vector<double>>("gains", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("x_formation", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("y_formation", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("z_formation", std::vector<double>{});

    std::string json_config_path;
    try {
        std::string package_path = ament_index_cpp::get_package_share_directory("px4_swarm_controller");
        json_config_path = package_path + "/config/control_config.json";
        RCLCPP_INFO(this->get_logger(), "Using control_config.json at: %s", json_config_path.c_str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to locate package directory: %s", e.what());
        return;
    }

    // --- Load config from JSON ---
    std::ifstream ifs(json_config_path);
    if (ifs.is_open()) {
        try {
            nlohmann::json j;
            ifs >> j;
            if (j.contains("neighborhood") && j["neighborhood"].contains("params")) {
                const auto& params = j["neighborhood"]["params"];
                if (params.contains("x_formation")) this->set_parameter(rclcpp::Parameter("x_formation", params["x_formation"].get<std::vector<double>>()));
                if (params.contains("y_formation")) this->set_parameter(rclcpp::Parameter("y_formation", params["y_formation"].get<std::vector<double>>()));
                if (params.contains("z_formation")) this->set_parameter(rclcpp::Parameter("z_formation", params["z_formation"].get<std::vector<double>>()));
            }
            if (j.contains("controller") && j["controller"].contains("params")) {
                if (j["controller"]["params"].contains("gains")) this->set_parameter(rclcpp::Parameter("gains", j["controller"]["params"]["gains"].get<std::vector<double>>()));
            }
        } catch (const nlohmann::json::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON parsing error: %s", e.what());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Could not open JSON config file: %s", json_config_path.c_str());
    }

    // --- Get all parameters ---
    this->get_parameter("is_leader", is_leader_);
    this->get_parameter("drone_id", my_id_);
    this->get_parameter("wp_path", wp_path_);
    this->get_parameter("x_formation", x_formation_);
    this->get_parameter("y_formation", y_formation_);
    this->get_parameter("z_formation", z_formation_);
    this->get_parameter("x_init", x_init_);
    this->get_parameter("y_init", y_init_);
    
    std::vector<double> gains_val;
    this->get_parameter("gains", gains_val);
    if (gains_val.size() >= 9) {
        pid_ax_.setKp(gains_val[0]); pid_ax_.setKi(gains_val[1]); pid_ax_.setKd(gains_val[2]);
        pid_ay_.setKp(gains_val[3]); pid_ay_.setKi(gains_val[4]); pid_ay_.setKd(gains_val[5]);
        pid_az_.setKp(gains_val[6]); pid_az_.setKi(gains_val[7]); pid_az_.setKd(gains_val[8]);
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid number of gains provided (%zu, expected 9).", gains_val.size());
    }

    // --- Initial State ---
    current_leader_id_ = 0;
    if (is_leader_) {
        current_state_ = DroneState::LEADER;
        current_leader_id_ = my_id_;
        RCLCPP_INFO(this->get_logger(), "Drone %d starting as LEADER.", my_id_ + 1);
    } else {
        current_state_ = DroneState::FOLLOWER;
        RCLCPP_INFO(this->get_logger(), "Drone %d starting as FOLLOWER.", my_id_ + 1);
    }
    
    // --- Waypoints ---
    if (!wp_path_.empty()) {
        try {
            waypoints_config_ = YAML::LoadFile(wp_path_);
            wp_threshold_ = waypoints_config_["threshold"].as<double>();
            wp_threshold_angle_ = waypoints_config_["threshold_angle"].as<double>();

            if (current_state_ == DroneState::LEADER) {
                writeWP(wp_idx_);
                leader_initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "Initial WP set for leader in constructor at index %zu", wp_idx_);
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load/parse waypoint file: %s", e.what());
        }
    } else if (is_leader_) {
        RCLCPP_WARN(this->get_logger(), "Leader has no waypoint path. It will hover.");
    }

    // --- ROS Comms ---
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(name_space + "/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(name_space + "/fmu/in/trajectory_setpoint", 10);
    waypoint_idx_pub_ = this->create_publisher<std_msgs::msg::Int32>("/simulation/current_waypoint_idx", 10);
    self_position_publisher_ = this->create_publisher<VehicleLocalPosition>(name_space + "/fmu/out/vehicle_local_position_agent", reliable_qos_);
    leader_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("/simulation/new_leader_id", 10, std::bind(&SwarmAgent::leader_id_callback, this, std::placeholders::_1));
    pose_subscriber_ = this->create_subscription<VehicleLocalPosition>(name_space + "/fmu/out/vehicle_local_position", sensor_qos_, std::bind(&SwarmAgent::pose_subscriber_callback, this, std::placeholders::_1));
    waypoint_idx_sub_ = this->create_subscription<std_msgs::msg::Int32>("/simulation/current_waypoint_idx", 10, std::bind(&SwarmAgent::waypoint_idx_callback, this, std::placeholders::_1));
    if (current_state_ == DroneState::FOLLOWER) {
        std::string leader_topic = "/px4_" + std::to_string(current_leader_id_ + 1) + "/fmu/out/vehicle_local_position_agent";
        leader_actual_position_subscriber_ = this->create_subscription<VehicleLocalPosition>(leader_topic, reliable_qos_, std::bind(&SwarmAgent::leader_actual_position_callback, this, std::placeholders::_1));
    }
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&SwarmAgent::timer_callback, this));
}

void SwarmAgent::timer_callback() {
    switch (current_state_) {
        case DroneState::LEADER: leader_logic(); break;
        case DroneState::FOLLOWER: follower_logic(); break;
    }
}

void SwarmAgent::leader_id_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    int new_leader_id = msg->data;
    if (my_id_ == new_leader_id) {
        if (current_state_ != DroneState::LEADER) {
            RCLCPP_WARN(this->get_logger(), "Drone %d PROMOTED TO LEADER!", my_id_ + 1);
            current_state_ = DroneState::LEADER;
            current_leader_id_ = my_id_;

            leader_initialized_ = false;
            initial_takeoff_complete_ = true; // A new leader doesn't need to do the initial takeoff sequence.

            // No longer need to calculate a recentering offset.
            // The leader will now publish its position relative to the formation center.

            if (leader_actual_position_subscriber_) leader_actual_position_subscriber_.reset();
            
            pid_ax_.reset_integral(); pid_ay_.reset_integral(); pid_az_.reset_integral();
        }
    } else {
        if (current_state_ != DroneState::FOLLOWER || current_leader_id_ != new_leader_id) {
            current_state_ = DroneState::FOLLOWER;
            current_leader_id_ = new_leader_id;
            if (leader_actual_position_subscriber_) leader_actual_position_subscriber_.reset();
            std::string leader_topic = "/px4_" + std::to_string(current_leader_id_ + 1) + "/fmu/out/vehicle_local_position_agent";
            leader_actual_position_subscriber_ = this->create_subscription<VehicleLocalPosition>(leader_topic, reliable_qos_, std::bind(&SwarmAgent::leader_actual_position_callback, this, std::placeholders::_1));
            leader_pose_received_ = false;
        }
    }
}

void SwarmAgent::pose_subscriber_callback(const VehicleLocalPosition::SharedPtr msg) {
    my_current_pose_ = *msg;
    pose_received_ = true;

    // If this drone is the leader, it must publish its position as the formation center.
    // It does this by subtracting its own formation offset from its actual position.
    // Followers will then add their own offset to this broadcasted center point.
    if (current_state_ == DroneState::LEADER) {
        VehicleLocalPosition formation_center_pose = my_current_pose_;
        float offset_n = 0.0f, offset_e = 0.0f, offset_d = 0.0f;

        if (my_id_ >= 0 && static_cast<size_t>(my_id_) < x_formation_.size()) {
            // FIX: Make mapping consistent
            offset_e = static_cast<float>(x_formation_[my_id_]); // Correct: x_formation -> East
            offset_n = static_cast<float>(y_formation_[my_id_]); // Correct: y_formation -> North
            offset_d = static_cast<float>(z_formation_[my_id_]);
        }
        
        formation_center_pose.x -= offset_n; // North
        formation_center_pose.y -= offset_e; // East
        formation_center_pose.z -= offset_d; // Down

        self_position_publisher_->publish(formation_center_pose);
    } else {
        // Followers publish their true position. This is mainly for the LeaderMonitor
        // and doesn't affect the swarm formation logic itself.
        self_position_publisher_->publish(my_current_pose_);
    }
}

void SwarmAgent::waypoint_idx_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (wp_idx_ != static_cast<size_t>(msg->data)) {
        wp_idx_ = msg->data;
        if (current_state_ == DroneState::LEADER) writeWP(wp_idx_);
    }
}

void SwarmAgent::leader_logic() {
    if (waypoints_config_.IsNull()) {
        RCLCPP_WARN(this->get_logger(), "Leader logic skipped: waypoints not loaded");
        return;
    }

    if (!pose_received_) {
        RCLCPP_WARN(this->get_logger(), "Pose not yet received for Leader %d", my_id_ + 1);
        return;
    }

    if (!leader_initialized_) {
        RCLCPP_INFO(this->get_logger(), "Leader %d initializing: writing waypoint index %zu", my_id_ + 1, wp_idx_);
        writeWP(wp_idx_);
        leader_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Leader %d setting initial waypoint to index %zu.", my_id_ + 1, wp_idx_);
    }

    // Initial takeoff sequence for the original leader
    if (!initial_takeoff_complete_ && pose_received_) {
        TrajectorySetpoint takeoff_setpoint{};
        takeoff_setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        // FIX #2: Takeoff target is now relative to the current position's Z value,
        // which makes it robust against variations in the initial spawning height.
        takeoff_setpoint.position = {my_current_pose_.x, my_current_pose_.y, my_current_pose_.z - 1.0f}; 
        takeoff_setpoint.yaw = 0.0;

        RCLCPP_INFO(this->get_logger(), "Leader %d initial pose: x=%.2f, y=%.2f, z=%.2f", 
                    my_id_ + 1, my_current_pose_.x, my_current_pose_.y, my_current_pose_.z);

        RCLCPP_INFO(this->get_logger(), "Leader %d takeoff setpoint: x=%.2f, y=%.2f, z=%.2f", 
                    my_id_ + 1, takeoff_setpoint.position[0], takeoff_setpoint.position[1], takeoff_setpoint.position[2]);

        double dist = std::hypot(my_current_pose_.x - takeoff_setpoint.position[0],
                                 my_current_pose_.y - takeoff_setpoint.position[1],
                                 my_current_pose_.z - takeoff_setpoint.position[2]);

        RCLCPP_INFO(this->get_logger(), "Leader %d takeoff distance: %.2f", my_id_ + 1, dist);

        if (dist < wp_threshold_) {
            initial_takeoff_complete_ = true;
            RCLCPP_INFO(this->get_logger(), "Leader %d completed initial takeoff.", my_id_ + 1);
        }

        publish_offboard_control_mode(CONTROL::POSITION);
        trajectory_setpoint_publisher_->publish(takeoff_setpoint);
        return;
    }

    if (pose_received_ && initial_takeoff_complete_) {
        double dist = std::hypot(my_current_pose_.x - leader_waypoint_.position[0],
                                 my_current_pose_.y - leader_waypoint_.position[1],
                                 my_current_pose_.z - leader_waypoint_.position[2]);

        double yaw_dist = std::abs(std::fmod(my_current_pose_.heading - leader_waypoint_.yaw + M_PI, 2 * M_PI) - M_PI);

        RCLCPP_INFO(this->get_logger(), "Leader %d distance to waypoint: %.2f, yaw diff: %.2f",
                    my_id_ + 1, dist, yaw_dist);

        if (dist < wp_threshold_ && yaw_dist < wp_threshold_angle_) {
            wp_idx_++;
            size_t wp_size = waypoints_config_["wp"][std::string("/px4_1")].size();
            if (wp_idx_ >= wp_size) wp_idx_ = 0;

            RCLCPP_INFO(this->get_logger(), "Leader %d reached waypoint, moving to index %zu", my_id_ + 1, wp_idx_);
            writeWP(wp_idx_);

            std_msgs::msg::Int32 wp_msg;
            wp_msg.data = wp_idx_;
            waypoint_idx_pub_->publish(wp_msg);
        }

        publish_offboard_control_mode(CONTROL::POSITION);
        trajectory_setpoint_publisher_->publish(leader_waypoint_);
    }
}


// In SwarmAgent::writeWP
void SwarmAgent::writeWP(const size_t idx) {
    // ... (previous code) ...
    float offset_n = 0.0f, offset_e = 0.0f, offset_d = 0.0f;
    if (my_id_ >= 0 && static_cast<size_t>(my_id_) < x_formation_.size()) {
        // FIX: Make mapping consistent
        offset_e = static_cast<float>(x_formation_[my_id_]); // Correct: x_formation -> East
        offset_n = static_cast<float>(y_formation_[my_id_]); // Correct: y_formation -> North
        offset_d = static_cast<float>(z_formation_[my_id_]);
    }

    leader_waypoint_.position = {
        static_cast<float>(coord(idx, "y") + offset_n), // North
        static_cast<float>(coord(idx, "x") + offset_e), // East
        static_cast<float>(coord(idx, "z") + offset_d)  // Down
    };
    leader_waypoint_.yaw = static_cast<float>(coord(idx, "yaw"));
}

double SwarmAgent::coord(const size_t idx, const std::string &var) {
    return waypoints_config_["wp"]["/px4_1"][idx][var].as<double>();
}

void SwarmAgent::leader_actual_position_callback(const VehicleLocalPosition::SharedPtr msg) {
    leader_actual_position_ = *msg;
    leader_pose_received_ = true;
}

void SwarmAgent::follower_logic() {
    TrajectorySetpoint setpoint{};
    setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    if (pose_received_ && leader_pose_received_) {
        publish_offboard_control_mode(CONTROL::POSITION);

        float offset_n = 0.0f, offset_e = 0.0f, offset_d = 0.0f;
        if (my_id_ >= 0 && static_cast<size_t>(my_id_) < x_formation_.size()) {
            offset_e = static_cast<float>(x_formation_[my_id_]);
            offset_n = static_cast<float>(y_formation_[my_id_]);
            offset_d = static_cast<float>(z_formation_[my_id_]);
        }
        
        setpoint.position = {
            leader_actual_position_.x + 0.0,
            leader_actual_position_.y + 0.0,
            leader_actual_position_.z + offset_d + -2.0
        };
        setpoint.yaw = leader_actual_position_.heading;
        
    } else {
        publish_offboard_control_mode(CONTROL::POSITION);
        setpoint.position = {my_current_pose_.x, my_current_pose_.y, my_current_pose_.z};
        setpoint.yaw = my_current_pose_.heading; // Keep current orientation
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Follower %d holding position due to missing leader pose.", my_id_ + 1);
    }

    trajectory_setpoint_publisher_->publish(setpoint);
}

void SwarmAgent::publish_offboard_control_mode(CONTROL control_mode) {
    OffboardControlMode msg{};
    msg.position = (control_mode == CONTROL::POSITION);
    msg.velocity = (control_mode == CONTROL::VELOCITY);
    msg.acceleration = (control_mode == CONTROL::ACCELERATION);
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

} // namespace Controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Controller::SwarmAgent)

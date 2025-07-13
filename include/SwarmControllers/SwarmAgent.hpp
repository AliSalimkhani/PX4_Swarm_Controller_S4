#pragma once

#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "PID.hpp" // Make sure this file exists and its path is correct
#include <custom_msgs/msg/weighted_topology_neighbors.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/int32.hpp>
#include <limits>
#include <chrono>
#include <vector> // Required for std::vector, used for formation
#include <string> // Required for std::string

using namespace std::chrono_literals;

namespace Controller {

// Definition of possible drone states
enum class DroneState {
    LEADER,
    FOLLOWER
};

// Definition of control modes for publish_offboard_control_mode
enum class CONTROL {
    POSITION,
    VELOCITY,
    ACCELERATION,
    ATTITUDE,
    RATES
};


class SwarmAgent : public rclcpp::Node {
public:
    explicit SwarmAgent(const rclcpp::NodeOptions &options);

private:
    // Typedefs for convenience
    using WeightedTopologyNeighbors = custom_msgs::msg::WeightedTopologyNeighbors;
    using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
    using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;
    using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
    using PID_Controller = PID::PID<float>; // Renamed to avoid conflict with PID namespace
    using PoseTwist = Eigen::Vector<float, 6>;
    using Neighborhood = Eigen::Matrix<float, 6, Eigen::Dynamic>;

    // Main functions
    void timer_callback();
    void leader_id_callback(const std_msgs::msg::Int32::SharedPtr msg);

    // Functions related to leader state
    void leader_logic();
    void pose_subscriber_callback(const VehicleLocalPosition::SharedPtr msg);
    void waypoint_idx_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void writeWP(size_t idx);
    double coord(size_t idx, const std::string &var);

    // Functions related to follower state
    void follower_logic();
    void neighbors_callback(const WeightedTopologyNeighbors::SharedPtr &neighbors);
    void neighbors_to_matrix(const WeightedTopologyNeighbors &neighbors);
    void leader_actual_position_callback(const VehicleLocalPosition::SharedPtr msg);

    // Common functions
    void publish_offboard_control_mode(CONTROL control_mode);

    // State and ID variables
    DroneState current_state_;
    int my_id_;
    int current_leader_id_;
    bool is_leader_; // ADDED: Declared as member variable
    bool leader_initialized_{false};
      bool initial_takeoff_complete_{false};

    // Common ROS variables
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr leader_subscriber_;

    // NEW: Publisher for own position, to be consumed by LeaderMonitor
    rclcpp::Publisher<VehicleLocalPosition>::SharedPtr self_position_publisher_;

    // Leader-related variables
    VehicleLocalPosition my_current_pose_; // Current position of the drone itself (leader and follower)
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr waypoint_idx_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr waypoint_idx_sub_;
    TrajectorySetpoint leader_waypoint_;
    size_t wp_idx_{0u};
    YAML::Node waypoints_config_;
    double wp_threshold_{};
    double wp_threshold_angle_{};
    double x_init_{};
    double y_init_{};
    bool pose_received_{false}; // Whether own drone's position has been received.
    std::string wp_path_; // ADDED: Declared as member variable

    // Follower-related variables
    rclcpp::Subscription<WeightedTopologyNeighbors>::SharedPtr neighbors_subscriber_;
    Neighborhood neighborhood_matrix_;
    bool is_neighborhood_empty_{true};
    PID_Controller pid_ax_{3., 2u}, pid_ay_{3., 2u}, pid_az_{3., 2u}; // Using PID_Controller
    float command_tp_{0.0};
    std::vector<float> default_pose_{0., 0., -5.};

    // New variables for leader-following capability
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr leader_actual_position_subscriber_; // Subscriber for leader's actual position
    VehicleLocalPosition leader_actual_position_; // Variable to store leader's actual position
    bool leader_pose_received_{false}; // Flag to check if leader's position has been received.

    // QoS profiles as member variables
    rclcpp::QoS reliable_qos_;
    rclcpp::QoS sensor_qos_;

    // --- ADDITIONS FOR FORMATION ---
    std::vector<double> x_formation_;
    std::vector<double> y_formation_;
    std::vector<double> z_formation_;
    // --- END ADDITIONS FOR FORMATION ---

    // --- ADDITIONS FOR LEADER PROMOTION RECENTERING ---
    // These offsets will be calculated when a new leader is promoted.
    // They represent the shift needed to make the new leader's current location
    // (minus its formation offset) become the effective origin for the mission path.
    double formation_recenter_offset_x_{0.0};
    double formation_recenter_offset_y_{0.0};
    double formation_recenter_offset_z_{0.0};
    // --- END ADDITIONS FOR LEADER PROMOTION RECENTERING ---
};

} // namespace Controller
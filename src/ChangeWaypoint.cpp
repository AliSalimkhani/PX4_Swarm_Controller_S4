/**
 * @author Arthur Astier
 */

 #include "ChangeWaypoint.hpp"
 #include <std_msgs/msg/int32.hpp>  // برای دریافت leader id و wp_idx
 
 /**
  * @brief Constructor for the ChangeWaypoint class.
  */
 ChangeWaypoint::ChangeWaypoint() : Node("waypoint") {
     const std::string name_space{this->get_namespace()};
 
     this->declare_parameter<std::string>("wp_path");
     this->declare_parameter<double>("x_init");
     this->declare_parameter<double>("y_init");
     this->declare_parameter<int>("drone_id", 0);  // شناسه ربات فعلی
 
     const auto wp_path{this->get_parameter("wp_path").as_string()};
     x_init = this->get_parameter("x_init").as_double();
     y_init = this->get_parameter("y_init").as_double();
     this->get_parameter("drone_id", my_id);
 
     offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(
             name_space + "/fmu/in/offboard_control_mode", 10);
     trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(
             name_space + "/fmu/in/trajectory_setpoint", 10);
 
     // Position subscriber
     rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
     auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
     pose_subscriber = this->create_subscription<VehicleLocalPosition>(
             name_space + "/fmu/out/vehicle_local_position", qos,
             [this](const typename VehicleLocalPosition::SharedPtr msg) {
                 pose_subscriber_callback(msg);
             });
 
     // Subscriber برای لیدر جدید
     leader_subscriber = this->create_subscription<std_msgs::msg::Int32>(
         "/simulation/new_leader_id", 10,
         [this](const std_msgs::msg::Int32::SharedPtr msg) {
             current_leader_id = msg->data;
             if (my_id == current_leader_id) {
                 RCLCPP_INFO(this->get_logger(), "Drone %d is now the leader!", my_id + 1);
                 writeWP(wp_idx); // اطمینان از تنظیم waypoint فعلی
             }
         });
 
     // Subscriber برای دریافت wp_idx
     waypoint_idx_sub = this->create_subscription<std_msgs::msg::Int32>(
         "/simulation/current_waypoint_idx", 10,
         [this](const std_msgs::msg::Int32::SharedPtr msg) {
             if (my_id == current_leader_id) {
                 wp_idx = msg->data;
                 writeWP(wp_idx);
                 RCLCPP_INFO(this->get_logger(), "Leader %d updated waypoint index to %zu", my_id + 1, wp_idx);
             }
         });
 
     // Publisher برای انتشار wp_idx
     waypoint_idx_pub = this->create_publisher<std_msgs::msg::Int32>("/simulation/current_waypoint_idx", 10);
 
     const auto node{YAML::LoadFile(wp_path)};
     threshold = node["threshold"].as<double>();
     threshold_angle = node["threshold_angle"].as<double>();
     waypoints = node["wp"][name_space];
 
     RCLCPP_INFO(this->get_logger(), "WP Threshold: %f", threshold);
     RCLCPP_INFO(this->get_logger(), "WP Angle Threshold: %f", threshold_angle);
     RCLCPP_INFO(this->get_logger(), "Found %ld waypoints", std::size(waypoints));
 
     writeWP(wp_idx);  // مقداردهی اولیه waypoint
 }
 
 /**
  * @brief Publish the offboard control mode.
  */
 void ChangeWaypoint::publish_offboard_control_mode() {
     OffboardControlMode msg{};
     msg.position = true;
     msg.velocity = false;
     msg.acceleration = false;
     msg.attitude = false;
     msg.body_rate = false;
     msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
     offboard_control_mode_publisher_->publish(msg);
 }
 
 /**
  * @brief Callback function for the vehicle local position subscriber.
  */
 void ChangeWaypoint::pose_subscriber_callback(const VehicleLocalPosition::SharedPtr &pose) {
     // فقط اگر لیدر هستیم ادامه بده
     if (my_id != current_leader_id) return;
 
     double theta_d = coord(wp_idx, "yaw");
 
     double distance{std::hypot(pose->x - waypoint.position[0],
                                pose->y - waypoint.position[1],
                                pose->z - waypoint.position[2])};
 
     double current_theta{pose->heading};
 
     if ((distance < threshold) &&
         (std::abs(std::fmod(current_theta - theta_d + M_PI, 2 * M_PI) - M_PI) < threshold_angle)) {
         wp_idx++;
         if (wp_idx == std::size(waypoints)) {
             wp_idx = 0;
         }
         writeWP(wp_idx);
 
         // انتشار wp_idx جدید
         std_msgs::msg::Int32 wp_msg;
         wp_msg.data = wp_idx;
         waypoint_idx_pub->publish(wp_msg);
         RCLCPP_INFO(this->get_logger(), "Leader %d published new waypoint index: %zu", my_id + 1, wp_idx);
     }
 
     publish_offboard_control_mode();
     trajectory_setpoint_publisher_->publish(waypoint);
 }
 
 /**
  * @brief Get coordinate value from waypoints.
  */
 double ChangeWaypoint::coord(const size_t idx, const std::string &var) {
     return waypoints[idx][var].as<double>();
 }
 
 /**
  * @brief Write waypoint based on index.
  */
 void ChangeWaypoint::writeWP(const std::size_t idx) {
     waypoint.timestamp = static_cast<uint64_t>(this->get_clock()->now().seconds());
     waypoint.position = {static_cast<float>(coord(idx, "y") - y_init),
                          static_cast<float>(coord(idx, "x") - x_init),
                          static_cast<float>(coord(idx, "z"))};
     waypoint.yaw = static_cast<float>(coord(idx, "yaw"));
 }
 
 /**
  * @brief Main function
  */
 int main(int argc, char **argv) {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<ChangeWaypoint>());
     rclcpp::shutdown();
     return 0;
 }
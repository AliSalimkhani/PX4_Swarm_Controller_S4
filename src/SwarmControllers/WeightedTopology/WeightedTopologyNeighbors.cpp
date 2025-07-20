/**
 * @brief WeightedTopologyNeighbors class for calculating weighted topology neighbors based on vehicle positions.
 * @details This class extends the NearestNeighbors class template to calculate weighted topology neighbors.
 * It provides functionality to process neighbor positions, enrich neighborhoods, and handle weighted topology neighbors.
 * @author Arthur Astier
 */

 #include "SwarmControllers/WeightedTopology/WeightedTopologyNeighbors.hpp"
 #include <std_msgs/msg/int32.hpp>  // برای دریافت ID لیدر جدید
 
 /**
  * @brief Constructor for WeightedTopologyNeighbors class.
  */
 Neighborhood::WeightedTopologyNeighbors::WeightedTopologyNeighbors()
         : NearestNeighbors<custom_msgs::msg::WeightedTopologyNeighbors>() {
 
     // پارامترهای ورودی از launch file
     this->declare_parameter<std::vector<bool>>("leaders");
     this->declare_parameter<std::vector<double>>("x_formation");
     this->declare_parameter<std::vector<double>>("y_formation");
     this->declare_parameter<std::vector<double>>("z_formation");
 
     const auto x_formation{this->get_parameter("x_formation").as_double_array()};
     const auto y_formation{this->get_parameter("y_formation").as_double_array()};
     const auto z_formation{this->get_parameter("z_formation").as_double_array()};
     vectors_to_Vector3d(x_formation, y_formation, z_formation);
 
     leaders = this->get_parameter("leaders").as_bool_array();
 
     prcs = PRCS::Constant(nb_drones, nb_drones);
     prcs_neighborhood = PRCS::Constant(nb_drones, nb_drones);
 
     // 📥 Subscriber برای دریافت لیدر جدید
     leader_subscriber = this->create_subscription<std_msgs::msg::Int32>(
         "/simulation/new_leader_id", 10,
         [this](const std_msgs::msg::Int32::SharedPtr msg) {
             int new_leader = msg->data;
             for (size_t i = 0; i < leaders.size(); ++i) {
                 leaders[i] = (i == static_cast<size_t>(new_leader));
             }
             RCLCPP_INFO(this->get_logger(), "New leader assigned: Drone %zu", static_cast<size_t>(new_leader) + 1);
         }
     );
 }
 
 /**
  * @brief Converts vectors representing x, y, and z coordinates to Eigen Vector3d objects.
  */
 void Neighborhood::WeightedTopologyNeighbors::vectors_to_Vector3d(const std::vector<double> &x, const std::vector<double> &y,
                                                                   const std::vector<double> &z) {
     if ((x.size(), y.size(), z.size()) == (nb_drones, nb_drones, nb_drones)) {
         formation.reserve(nb_drones);
         std::transform(std::begin(x), std::end(x), std::begin(formation), [&, idx = 0u](const auto aX) mutable {
             Vector3d offset{aX, y[idx], z[idx]};
             ++idx;
             return offset;
         });
     } else {
         RCLCPP_INFO(this->get_logger(), "Drone formation provided is not correct !");
     }
 }
 
 /**
  * @brief Processes the position of a neighbor drone.
  */
 void Neighborhood::WeightedTopologyNeighbors::process_neighbor_position(const std::size_t drone_idx,
                                                                          const std::size_t neighbor_idx,
                                                                          const VehicleLocalPosition &position,
                                                                          VehicleLocalPosition neighbor_position,
                                                                          WeightedTopologyNeighborsMsg &neighborhood) {
     neighbor_position.x = position.x - neighbor_position.x - static_cast<float>(formation[drone_idx].x() - formation[neighbor_idx].x());
     neighbor_position.y = position.y - neighbor_position.y - static_cast<float>(formation[drone_idx].y() - formation[neighbor_idx].y());
     neighbor_position.z = position.z - neighbor_position.z - static_cast<float>(formation[drone_idx].z() - formation[neighbor_idx].z());
 
     neighbor_position.vx = position.vx - neighbor_position.vx;
     neighbor_position.vy = position.vy - neighbor_position.vy;
     neighbor_position.vz = position.vz - neighbor_position.vz;
 
     neighborhood.neighbors_position.emplace_back(neighbor_position);
     neighborhood.neighbors_ids.emplace_back(neighbor_idx);
     prcs_neighborhood[neighbor_idx] = prcs[neighbor_idx];
 }
 
 /**
  * @brief Processes the neighborhood based on the positions of the neighbor drones.
  */
 void Neighborhood::WeightedTopologyNeighbors::process_neighborhood(const std::size_t drone_idx,
                                                                     WeightedTopologyNeighborsMsg &neighborhood) {
     if (!std::empty(neighborhood.neighbors_position)) {
         if (leaders[drone_idx]) {
             prcs[drone_idx] = 1;
         } else {
             const auto min_prc_neighborhood{prcs_neighborhood.minCoeff() + 1};
             prcs[drone_idx] = static_cast<std::size_t>(std::copysign(1u, min_prc_neighborhood)) *
                               std::min(min_prc_neighborhood, nb_drones);
             prcs_neighborhood.setConstant(nb_drones);
         }
     }
 }
 
 /**
  * @brief Enriches the neighborhood message with PRC-based weights.
  */
 void Neighborhood::WeightedTopologyNeighbors::enrich_neighborhood(WeightedTopologyNeighborsMsg &neighborhood) {
     const auto sum_updated_prcs_neighborhood{
         std::accumulate(std::begin(neighborhood.neighbors_ids), std::end(neighborhood.neighbors_ids),
                         0.0,
                         [this](const double sum, const auto id) {
                             return sum + 1. / static_cast<double>(prcs[id]);
                         })
     };
 
     Weights weights;
     weights.reserve(nb_drones);
     std::for_each(std::begin(neighborhood.neighbors_ids), std::end(neighborhood.neighbors_ids),
                   [this, &weights, sum_updated_prcs_neighborhood](const auto id) {
                       weights.emplace_back((1. / static_cast<double>(prcs[id])) /
                                            static_cast<double>(sum_updated_prcs_neighborhood));
                   });
     neighborhood.set__weights(weights);
 }
 
 /**
  * @brief Main function
  */
 int main(int argc, char *argv[]) {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<Neighborhood::WeightedTopologyNeighbors>());
     rclcpp::shutdown();
     return 0;
 }
 
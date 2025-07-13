/**
 * @brief WeightedTopologyNeighbors class for calculating weighted topology neighbors based on vehicle positions.
 * @details This class extends the NearestNeighbors class template to calculate weighted topology neighbors.
 * It provides functionality to process neighbor positions, enrich neighborhoods, and handle weighted topology neighbors.
 * @author Astier Arthur
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "NearestNeighbors.hpp"
#include <custom_msgs/msg/weighted_topology_neighbors.hpp>
#include <eigen3/Eigen/Eigen>
#include <std_msgs/msg/int32.hpp>  // برای تشخیص لیدر جدید

namespace Neighborhood {
    class WeightedTopologyNeighbors : public NearestNeighbors<custom_msgs::msg::WeightedTopologyNeighbors> {
        using WeightedTopologyNeighborsMsg = custom_msgs::msg::WeightedTopologyNeighbors;
        using PRCS = Eigen::Vector<std::size_t, Eigen::Dynamic>;
        using Weights = std::vector<double>;
        using Vector3d = Eigen::Vector3d;
        using Formation = std::vector<Vector3d>;

    public:
        /**
         * @brief Constructor for WeightedTopologyNeighbors class.
         */
        WeightedTopologyNeighbors();

    private:
        /**
         * @brief Converts vectors representing x, y, and z coordinates to Eigen Vector3d objects.
         */
        void vectors_to_Vector3d(const std::vector<double> &x, const std::vector<double> &y,
                                 const std::vector<double> &z);

        /**
         * @brief Processes the position of a neighbor drone.
         */
        void process_neighbor_position(const std::size_t drone_idx, const std::size_t neighbor_idx,
                                       const VehicleLocalPosition &position,
                                       VehicleLocalPosition neighbor_position,
                                       WeightedTopologyNeighborsMsg &neighborhood) override;

        /**
         * @brief Processes the neighborhood based on the positions of the neighbor drones.
         */
        void process_neighborhood(const std::size_t drone_idx, WeightedTopologyNeighborsMsg &neighborhood) override;

        /**
         * @brief Enriches the neighborhood message with weights calculated based on the PRCS (priority based on conflict state) of each neighbor.
         */
        void enrich_neighborhood(WeightedTopologyNeighborsMsg &neighborhood) override;

    private:
        std::vector<bool> leaders;                ///< نشان می‌دهد هر پهپاد لیدر هست یا نه
        PRCS prcs;                                ///< بردار PRC اصلی
        PRCS prcs_neighborhood;                   ///< بردار موقت برای محاسبه neighborhood
        Formation formation;                      ///< فرمیشن سه‌بعدی برای هر پهپاد

        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr leader_subscriber; ///< Subscriber برای لیدر جدید
    };
}

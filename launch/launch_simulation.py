#!/usr/bin/env python3

__author__ = "Arthur Astier"

import os
import json
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def parse_swarm_config(config_file):
    """
    Parses the swarm configuration from the provided JSON file.
    This is the correct version for your project.
    """
    model_counts = {}
    swarm_config = config_file["swarm"] # 
    for key, item in swarm_config.items():
        model = item["model"] # 
        model_counts[model] = model_counts.get(model, 0) + 1
    script = ""
    for key, item in model_counts.items():
        script += key + ":" + str(item) + ","
    script = script[:-1]
    initial_poses_string = "\""
    initial_poses_dict = dict()
    for idx, item in enumerate(swarm_config.values()):
        initial_pose = item["initial_pose"] # 
        initial_poses_dict["px4_" + str(idx + 1)] = initial_pose # 
        initial_poses_string += str(initial_pose["x"]) + "," + str(initial_pose["y"]) + "|"
    initial_poses_string = initial_poses_string[:-1] + "\""
    is_leaders = [item["is_leader"] for item in swarm_config.values()] # 
    return len(swarm_config), script, initial_poses_string, initial_poses_dict, is_leaders, config_file["trajectory"]


def generate_launch_description():
    ld = LaunchDescription()
    package_dir = get_package_share_directory('px4_swarm_controller')

    with open(os.path.join(package_dir, 'config', 'swarm_config.json'), 'r') as swarm_file:
        swarm_config_data = json.load(swarm_file)
    # The line below now calls the one and only correct version of parse_swarm_config
    nb_drones, script, initial_poses, initial_poses_dict, is_leaders, trajectory = parse_swarm_config(swarm_config_data) # 

    with open(os.path.join(package_dir, 'config', 'control_config.json'), 'r') as control_file:
        control_config = json.load(control_file)

    neighborhood = control_config["neighborhood"]
    neighbors_exe, neighbors_distance, neighbors_params = neighborhood["neighbors_exe"], \
        neighborhood["neighbor_distance"], neighborhood["params"]

    controller_info = control_config["controller"]
    controller_params = controller_info["params"]
    is_leader_follower_control = controller_info["leader_follower"]

    if is_leader_follower_control:
        neighbors_params = {"leaders": is_leaders, **neighbors_params}
    else:
        is_leaders = [False for _ in is_leaders]

    composable_node_descriptions_list = []
    swarm_container = ComposableNodeContainer(
        name='px4_swarm_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_node_descriptions_list,
        output='screen',
    )
    ld.add_action(swarm_container)

    ld.add_action(
        Node(
            package='px4_swarm_controller',
            executable='simulation_node.py',
            name='simulation_node',
            # Note: The original parse_swarm_config produces a string for initial_poses, not a list.
            # This assumes simulation_node.py can handle the string format "|".
            parameters=[{'script': script, 'initial_pose': initial_poses}]
        )
    )
    xs_init = []
    ys_init = []

    for i, ((namespace, initial_pose), aleader) in enumerate(zip(initial_poses_dict.items(), is_leaders)):
        xs_init.append(initial_pose["y"])
        ys_init.append(initial_pose["x"])
        composable_node_descriptions_list.append(
            ComposableNode(
                package='px4_swarm_controller',
                plugin='Controller::SwarmAgent',
                name=f'swarm_agent_{i+1}',
                namespace=namespace,
                parameters=[
                    {"is_leader": aleader},
                    {"drone_id": i},
                    {"wp_path": os.path.join(package_dir, "config", "Trajectories", trajectory)},
                    {"x_init": initial_pose["x"], "y_init": initial_pose["y"]},
                    controller_params
                ],
            )
        )

    # The code you added for the LeaderMonitor is correct.
    try:
        initial_leader_id = is_leaders.index(True) # 
    except ValueError:
        initial_leader_id = 0
        # In a real ROS2 launch file, you would use a log message.
        print("WARN: No initial leader set in swarm_config.json, defaulting to drone 0.")

    ld.add_action(
        Node(
            package='px4_swarm_controller',
            executable='leader_monitor',
            name='leader_monitor',
            namespace='simulation',
            output='screen',
            parameters=[
                {"nb_drones": nb_drones}, # 
                {"leader_id": initial_leader_id} # 
            ]
        )
    )

    ld.add_action(
        Node(
            package='px4_swarm_controller',
            executable=neighbors_exe,
            name='nearest_neighbors',
            namespace='simulation',
            parameters=[
                {"nb_drones": nb_drones, "neighbor_distance": neighbors_distance,
                 "x_init": xs_init, "y_init": ys_init, **neighbors_params}]
        ))

    ld.add_action(
        Node(
            package='px4_swarm_controller',
            executable='arming',
            name='arming',
            namespace='simulation',
            parameters=[{"nb_drones": nb_drones}]
        ))

    return ld
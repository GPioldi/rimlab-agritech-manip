from launch import LaunchDescription
from launch_ros.actions import Node
import os
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    ld = LaunchDescription()

    # DEPOSIT CONFIGURATIONS:
    # left: [4.5718, -2.1777, 2.2249, 0.1426, 1.8128, 3.2097],
    # center: [3.4493, -1.9110, 2.1788, -0.2667, 1.9227, 3.0986],
    # right: [1.3803, -1.9258, 2.2249, -0.3885, 1.8128, 3.2096],  
    #
    # field: [-0.05, -0.52, -2.34, -0.97, 1.74, 2.98]

    tomato_pick_param={
        "tomato_position_topic": "/tomato_position", # topic published by tomato detector
        "cycle_time_ms": 50,                         # period of main control loop
        "world_frame_id": "world",                   # frame_id of world
        "wrist_frame_id": "wrist_3_link",            # frame_id of robot wrist
        "camera_frame_id": "camera_infra_optical_frame",      # frame_id of eye-in-hand camera
        "gripper_frame_id": "wrist_3_link",      # frame_id of eye-in-hand camera
        "position_wrist_camera": [0.009, -0.08245, 0.09533],   # position of camera frame w.r.t. wrist
        "rot_rpy_wrist_camera": [0.0, 0.0, 180.0],             # orientation of camera frame w.r.t. wrist (angles in deg)
        #"joints_home": [1.57, -2.036, 2.22, -0.14, 1.57, 3.10],         # joint values at robot home configuration 
        #"joints_deposit":   [-0.05, -0.52, -2.34, -0.97, 1.74, 2.98],
        "joints_home": [-0.0026, -0.862, -2.2731, 0.0068, 1.5067, 3.0997],         # joint values at robot home configuration 
        "joints_deposit":   [0.03, -1.9647, -1.83, 0.7389, 1.48, 3.0995],

        "association_distance": 0.10,    # tolerance on target observation for association by the tracker
        "hit_num_min": 6,                # minimum number of hits by tracker to classify valid targets
        "miss_num_max": 10,              # maximum number of consecutive misses to remove a valid target from tracking

        "gripper_width_open": 74,
        "gripper_width_closed": 30,

        "observation_distance": 0.30,    # distance between the target center and the viewpoint at observation
        "grasp_distance_far": 0.30,    # distance between the target center and the viewpoint at observation
        "grasp_distance_close": 0.13,    # distance between the target center and the viewpoint at observation
        #"approach_dir": [0.0, 0.4, 0.68],
        "approach_dir": [-0.8, 0.0, 0.20],
        "approach_dir_pitch1_deg": -20.0,
        "approach_dir_pitch2_deg": -5.0,
    }

    moveit_config_pkg_name = "ur10e_agritech"
    moveit_config = MoveItConfigsBuilder(moveit_config_pkg_name).to_moveit_configs()

    tomato_pick = Node(
        package='agritech_manip',
        executable='tomato_pick',
        output="screen",
        parameters=[
            tomato_pick_param,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics
            ], 
        name='tomato_pick',
        #prefix=['gdb -ex=r --args'],
    )
    ld.add_action(tomato_pick)

    return ld

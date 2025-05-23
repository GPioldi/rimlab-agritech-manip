from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import time
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ld = LaunchDescription()

    rbt_state_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # os.path.join(get_package_share_directory('ur_ros_driver'), 
            #              '/home/rimlab/ros2_ws/src/ros2-rimlab/manip/ur10e/ur_ros_driver/launch/robot_state_receiver.launch.py')
            os.path.join(FindPackageShare('ur_ros_driver').find('ur_ros_driver'), 'launch', 'robot_state_receiver.launch.py')
        )
    )
    ld.add_action(rbt_state_receiver)

    scene_path = os.path.join(FindPackageShare('ur10e_test').find('ur10e_test'), 'scene_geometry', 'rimlab_pal3_whit_plant.scene')

    scene_publisher = Node(
        package='moveit_ros_planning',
        executable='moveit_publish_scene_from_text',
        #arguments=[os.path.join(FindPackageShare('ur10e_test').find('ur10e_test'), 'scene_geometry', 'rimlab_pal3_ whit_plant.scene')],
        arguments=['--scene ', scene_path],
        name='moveit_publish_scene_from_text',
    )
    ld.add_action(scene_publisher)

    # return LaunchDescription([
    DeclareLaunchArgument(
        'enable_rgbd',
        default_value='true'
    ),
    DeclareLaunchArgument(
        'enable_sync',
        default_value='true'
    ),
    DeclareLaunchArgument(
        'align_depth.enable',
        default_value='true'
    ),
    DeclareLaunchArgument(
        'enable_color',
        default_value='true'
    ),
    DeclareLaunchArgument(
        'enable_depth',
        default_value='true'
    ),
    DeclareLaunchArgument(
        'pointcloud.enable',
        default_value='true'
    )
    # ])
    realsense2_path = os.path.join(FindPackageShare('agritech_manip').find('agritech_manip'), 'launch', 'realsense2_d405.launch.py')
    camera_realsense2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            realsense2_path), 
        )
    
    ld.add_action(camera_realsense2)

    tomato_msgsync = Node(
        package='tomato_detection',
        executable='msgs_synch.py',
        name='msgs_synch',
    )
    ld.add_action(tomato_msgsync)

    tomato_det = Node(
        package='tomato_detection',
        executable='tomato_detector.py',
        name='tomato_detector',
    )
    ld.add_action(tomato_det)

    moveit_config = MoveItConfigsBuilder("ur10e_d405", package_name="ur10e_d405_moveit_config").to_moveit_configs()

    # cmd_server = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('ur_ros_driver'), 
    #                       '/home/rimlab/ros2_ws/src/ros2-rimlab/manip/ur10e/ur_ros_driver/launch/command_server.launch.py')
    #     )
    # )
    # ld.add_action(cmd_server)

    # tomato_apprch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('ur10e_test'), 
    #                      '/home/rimlab/ros2_ws/src/ros2-rimlab/vision/agritech/ur10e_test/launch/tomato_approach.launch.py')
    #     )
    # )
    # ld.add_action(tomato_apprch)

    # tomato_pick = Node(
    #     package='agritech_manip',
    #     executable='tomato_pick',
    #     #arguments=['--scene ', scene_path],
    #     name='tomato_pick',
    # )
    # ld.add_action(tomato_pick)

    return ld
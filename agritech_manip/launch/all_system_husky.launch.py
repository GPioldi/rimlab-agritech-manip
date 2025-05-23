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
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    # Call external launch file with nodes to read the robot state 
    # rbt_state_receiver = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         # os.path.join(get_package_share_directory('ur_ros_driver'), 
    #         #              '/home/rimlab/ros2_ws/src/ros2-rimlab/manip/ur10e/ur_ros_driver/launch/robot_state_receiver.launch.py')
    #         os.path.join(FindPackageShare('ur_ros_driver').find('ur_ros_driver'), 'launch', 'robot_state_receiver.launch.py')
    #     )
    # )
    rbt_state_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('ur_ros_rtde').find('ur_ros_rtde'), 'launch', 'robot_state_receiver_husky.launch.py')
        )
    )
    ld.add_action(rbt_state_receiver)

    # Node to publish the scene description in the planner
    scene_path = os.path.join(FindPackageShare('agritech_manip').find('agritech_manip'), 'scene_geometry', 'husky_manip_mobile.scene')
    scene_publisher = Node(
        package='moveit_ros_planning',
        executable='moveit_publish_scene_from_text',
        #arguments=[os.path.join(FindPackageShare('ur10e_test').find('ur10e_test'), 'scene_geometry', 'rimlab_pal3_ whit_plant.scene')],
        arguments=['--scene ', scene_path],
        name='moveit_publish_scene_from_text',
    )
    ld.add_action(scene_publisher)

    camera_realsense2 = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(FindPackageShare('agritech_manip').find('agritech_manip'), 'launch', 'low_my_realsense.launch.xml')
        )
    )
    ld.add_action(camera_realsense2)

    tomato_det = Node(
        package='tomato_detection',
        executable='tomato_detector.py',
        name='tomato_detector',
    )
    ld.add_action(tomato_det)

    cmd_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('ur_ros_rtde').find('ur_ros_rtde'), 'launch', 'command_server_pal3.launch.py')
        )
    )
    ld.add_action(cmd_server)

    return ld
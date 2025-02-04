import os
import xacro
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    package_path = get_package_share_directory("robot_arm")

    robot_arm_macro = os.path.join(package_path, "urdf", "robot_arm.xacro")
    rviz_config = os.path.join(package_path, "config", "config.rviz")
    gazebo_world = os.path.join(package_path, "worlds", "robot_world.sdf")
    ign_gazebo_world = os.path.join(package_path, "worlds", "robot_world_ign.sdf")
    robot_description = xacro.process_file(robot_arm_macro).toxml()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            ),
            ExecuteProcess(
                cmd=["rviz2", "-d", rviz_config],
                output="screen",
            ),
            # ExecuteProcess(
            #     cmd=[
            #         "gazebo",
            #         "--ros-args",
            #         "--world",
            #         gazebo_world,
            #     ],
            #     output="screen",
            # ),
            # ExecuteProcess(
            #     cmd=[
            #         "gazebo",
            #         "--ros-args",
            #         "-s",
            #         "libgazebo_ros_factory.so",
            #         "--world",
            #         gazebo_world,
            #     ],
            #     output="screen",
            # ),
            # Start Gazebo Fortress (ignition gazebo)
            ExecuteProcess(
                cmd=[
                    "ign",
                    "gazebo",
                    ign_gazebo_world,
                    "--verbose",
                    "4"
                ],
                output="screen",
            ),
            # Node(
            #     package="gazebo_ros",
            #     executable="spawn_entity.py",
            #     arguments=["-entity", "robot_arm", "-topic", "robot_description"],
            #     output="screen",
            # ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "ros_gz_sim",
                    "create",
                    "-topic",
                    "/robot_description",
                    "-name",
                    "robot_arm",
                ],
                output="screen",
            ),
        ]
    )

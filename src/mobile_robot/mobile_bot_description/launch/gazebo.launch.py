from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable

import os
import xacro

def generate_launch_description():
    ld = LaunchDescription()

    #package path
    package_path = get_package_share_directory("mobile_bot_description")

    #file configuration
    xacro_file = os.path.join(
        package_path,
        "urdf",
        "mobile_robot.xacro"
    )

    #rviz config.
    rviz_config = os.path.join(
        package_path,
        "rviz",
        "mobile_robot.rviz"
    )
    #control path
    ros2control = os.path.join(
        package_path,
        "config",
        "control.yaml"
    )

    world_file = os.path.join(
        package_path,
        "worlds",
        "potential_field.sdf"
    )

    # description of robot
    robot_description_config = xacro.process_file(xacro_file)

    robot_description = {
        "robot_description" : robot_description_config.toxml()
    }

    # gazebo launch description
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("ros_gz_sim"),
            "launch",
            "gz_sim.launch.py"
        )
    )

    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments = {
            "gz_args" : "-r " + world_file,
            "on_exit_shutdown": "true"
        }.items()
    )

    spawnModelNodeGazebo = Node(
        package="ros_gz_sim",
        executable="create",    
        arguments=[
            "-topic", "robot_description",
            "-name", "mobile_robot",
            "-x", "0",
            "-y", "0",
            "-z", "0.5",
        ],
        output = "screen"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable = "robot_state_publisher",
        output = "screen",
        parameters=[robot_description, {"use_sim_time": True}]
    )

    bridge_parms = os.path.join(
        package_path,
        "parameters",
        "bridge_parameter.yaml"
    )

    start_gz_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{
            "config_file" : bridge_parms
        }]
    )

    rviz = Node(
        package="rviz2",
        executable = "rviz2",
        output = "screen",
        arguments=["-d", rviz_config]
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    ros2_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--param-file", ros2control]
    )

    share_path = os.path.dirname(package_path)

    ld.add_action(
        SetEnvironmentVariable(
            name= "GZ_SIM_RESOURCE_PATH",
            value=[
                EnvironmentVariable("GZ_SIM_RESOURCE_PATH"),
                ":",
                share_path,
            ],
        )
    )
    ld.add_action(robot_state_publisher)
    ld.add_action(gazeboLaunch)
    ld.add_action(spawnModelNodeGazebo)
    ld.add_action(start_gz_ros_bridge_cmd)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(ros2_controller_spawner)
    ld.add_action(rviz)

    return ld
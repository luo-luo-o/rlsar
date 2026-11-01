# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rname = LaunchConfiguration("rname")
    gui = LaunchConfiguration("gui")

    wname = "stairs"
    robot_name = ParameterValue(Command(["echo -n ", rname]), value_type=str)
    ros_namespace = ParameterValue(Command(["echo -n ", "/", rname, "_gazebo"]), value_type=str)
    gazebo_model_name = ParameterValue(
        PythonExpression(["'robot_model' if '", rname, "' == 'dog' else '", rname, "_gazebo'"]),
        value_type=str,
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            Command(["echo -n ", Command(["ros2 pkg prefix ", rname, "_description"])]),
            "/share/", rname, "_description/xacro/robot.xacro"
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            # "verbose": "true",
            # "pause": "true",  # Not Available
            "world": os.path.join(get_package_share_directory("rl_sar"), "worlds", wname + ".world"),
            "gui": gui,
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "robot_model",
            "-z", PythonExpression(["'0.265' if '", rname, "' == 'dog' else '1.0'"]),
        ],
        output="screen",
    )

    set_initial_joints = Node(
        package="dog_description",
        executable="set_initial_joint_positions.py",
        arguments=["--model-name", "robot_model"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", rname, "' == 'dog'"])),
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    robot_joint_controller_node = Node(
        package="controller_manager",
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        arguments=["robot_joint_controller"],
        output="screen",
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.1,
            'autorepeat_rate': 0.0,
        }],
    )

    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        parameters=[{
            "robot_name": robot_name,
            "gazebo_model_name": gazebo_model_name,
        }],
    )

    def after_dog_spawn(event, context):
        if event.returncode != 0:
            return [LogInfo(msg="Dog spawn failed; controllers will not start"),
                    EmitEvent(event=Shutdown(reason="Dog spawn failed"))]
        return [set_initial_joints]

    def after_dog_initialization(event, context):
        if event.returncode != 0:
            return [LogInfo(msg="Dog joint initialization failed; controllers will not start"),
                    EmitEvent(event=Shutdown(reason="Dog joint initialization failed"))]
        return [joint_state_broadcaster_node, robot_joint_controller_node]

    start_initialization = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=after_dog_spawn),
        condition=IfCondition(PythonExpression(["'", rname, "' == 'dog'"])),
    )
    start_dog_controllers = RegisterEventHandler(
        OnProcessExit(target_action=set_initial_joints, on_exit=after_dog_initialization),
        condition=IfCondition(PythonExpression(["'", rname, "' == 'dog'"])),
    )
    start_joint_state_broadcaster_other = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[joint_state_broadcaster_node]),
        condition=UnlessCondition(PythonExpression(["'", rname, "' == 'dog'"])),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rname",
            description="Robot name (e.g., a1, go2)",
            default_value=TextSubstitution(text=""),
        ),
        DeclareLaunchArgument(
            "gui",
            description="Launch Gazebo client GUI",
            default_value=TextSubstitution(text="true"),
        ),
        start_initialization,
        start_dog_controllers,
        start_joint_state_broadcaster_other,
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        joy_node,
        param_node,
    ])

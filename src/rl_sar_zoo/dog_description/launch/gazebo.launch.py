import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import EmitEvent, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    package_share = get_package_share_directory("dog_description")
    gazebo_share = get_package_share_directory("gazebo_ros")
    robot_description = xacro.process_file(
        os.path.join(package_share, "xacro", "robot.xacro")
    ).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={"gui": gui}.items(),
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description", "-entity", "dog", "-z", "0.265"],
        output="screen",
    )
    set_initial_joints = Node(
        package="dog_description",
        executable="set_initial_joint_positions.py",
        arguments=["--model-name", "dog"],
        output="screen",
    )
    executable = "spawner.py" if os.environ.get("ROS_DISTRO") == "foxy" else "spawner"
    joint_state = Node(
        package="controller_manager",
        executable=executable,
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    robot_controller = Node(
        package="controller_manager",
        executable=executable,
        arguments=["robot_joint_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    def after_spawn(event, context):
        if event.returncode != 0:
            return [LogInfo(msg="Dog spawn failed; controllers will not start"),
                    EmitEvent(event=Shutdown(reason="Dog spawn failed"))]
        return [set_initial_joints]

    def after_initialization(event, context):
        if event.returncode != 0:
            return [LogInfo(msg="Dog joint initialization failed; controllers will not start"),
                    EmitEvent(event=Shutdown(reason="Dog joint initialization failed"))]
        return [joint_state, robot_controller]

    initialize_joints = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=after_spawn)
    )
    start_controllers = RegisterEventHandler(
        OnProcessExit(target_action=set_initial_joints, on_exit=after_initialization)
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            description="Launch Gazebo client GUI",
            default_value="true",
        ),
        initialize_joints,
        start_controllers,
        gazebo,
        state_publisher,
        spawn,
    ])

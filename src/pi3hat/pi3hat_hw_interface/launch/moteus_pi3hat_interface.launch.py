from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command,LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    urdf_name_arg = DeclareLaunchArgument("urdf_file",default_value="omnicar.xacro")
    ld.add_action(urdf_name_arg)
    conf_name_arg = DeclareLaunchArgument("conf_file",default_value="omnicar.yaml")
    ld.add_action(conf_name_arg)

    robot_model_path = PathJoinSubstitution([
        FindPackageShare("pi3hat_hw_interface"),
        "urdf",LaunchConfiguration("urdf_file")
        ])
    robot_description = ParameterValue(
        Command(["xacro ", robot_model_path]),
        value_type=str
    )

    controller_path = PathJoinSubstitution([
        FindPackageShare("pi3hat_hw_interface"),
        "config",LaunchConfiguration("conf_file")
        ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description},controller_path],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    omni_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_controller"],
    )

    ld.add_action(control_node)
    ld.add_action(jsb_spawner)
    ld.add_action(omni_spawner)
    return ld

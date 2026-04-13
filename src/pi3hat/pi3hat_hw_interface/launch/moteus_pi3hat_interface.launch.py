from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_name_arg = DeclareLaunchArgument('urdf_file', default_value='omnicar.xacro')
    conf_name_arg = DeclareLaunchArgument('conf_file', default_value='omnicar.yaml')
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
    )

    robot_model_path = PathJoinSubstitution(
        [FindPackageShare('pi3hat_hw_interface'), 'urdf', LaunchConfiguration('urdf_file')]
    )
    robot_description = ParameterValue(Command(['xacro ', robot_model_path]), value_type=str)

    controller_path = PathJoinSubstitution(
        [FindPackageShare('pi3hat_hw_interface'), 'config', LaunchConfiguration('conf_file')]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_path],
    )

    state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['state_broadcaster'],
    )

    omni_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omni_controller'],
    )

    rosbag_record = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record_bag')),
        cmd=[
            'ros2',
            'bag',
            'record',
            '/distributor_state_broadcaster/distributors_state',
            '/distributor_state_broadcaster/transition_event',
            '/events/write_split',
            '/ik_controller/base_pose',
            '/joy',
            '/joy/set_feedback',
            '/nuc_heartbeat',
            '/omni_controller/direct_wheels_cmd',
            '/omni_controller/joints_command',
            '/omni_controller/joints_state',
            '/omni_controller/legs_cmd',
            '/omni_controller/performance',
            '/omni_controller/safety_state',
            '/omni_controller/transition_event',
            '/omni_controller/twist_cmd',
            '/state_broadcaster/joints_state',
            '/state_broadcaster/performance_indexes',
            '/state_broadcaster/transition_event',
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            urdf_name_arg,
            conf_name_arg,
            record_bag_arg,
            control_node,
            state_broadcaster_spawner,
            omni_controller_spawner,
            rosbag_record,
        ]
    )

from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):

    moveit_configs = MoveItConfigsBuilder(
        context=context,
        controllers_name='fake_controllers',
        dof=5,
        robot_type='xarm',
        add_gripper=True,
        ros2_control_plugin='uf_robot_hardware/UFRobotFakeSystemHardware',
    ).to_moveit_configs()

    pick_place_node = Node(
        package='xarm_pick_place',
        executable='pick_place',
        output='screen',
        parameters=[
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            moveit_configs.robot_description_kinematics,
            moveit_configs.joint_limits,
        ],
    )

    return [
        TimerAction(
            period=5.0,
            actions=[pick_place_node]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
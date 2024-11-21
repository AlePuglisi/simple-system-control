from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    packages_name = "system_description"
    xacro_file_name = "simple_system.xacro"
    rviz_file_name = "system_rviz.rviz"

    #initialize sim_time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(packages_name), "urdf", xacro_file_name]
            ),
        ]
    )

    system_controllers = PathJoinSubstitution(
        [
            FindPackageShare("system_control"),
            "config",
            "system_controller.yaml",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(packages_name), "rviz", rviz_file_name]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        output='screen'
    )

    # system_command_interface_node = Node(
    #     package='system_command_interface',
    #     executable='system_command_interface',
    #     name='system_command_interface',
    #     output='screen'
    # )

    gazebo_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description',
                   '-entity', 'simple_system'],
        output='screen'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'),
            '/gazebo.launch.py'])
        #launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )

    controller_node = Node(
        package='system_control',
        executable='system_control',
        name = 'system_control',
        output='screen',
    )

    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[{'robot_description': robot_description},
    #                 system_controllers]
    # )

    # delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["effort_controller"],
    )
    
    # delayed_effort_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager,
    #         on_start=[effort_controller_spawner],
    #     )
    # )   

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # delayed_joint_broad_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager,
    #         on_start=[joint_state_broadcaster_spawner],
    #     )
    # ) 

    return LaunchDescription([
        gazebo,
        # nodes:
        joy_node,
        # system_command_interface_node,
        rviz_node, 
        robot_state_pub_node, 
        gazebo_spawner,
        effort_controller_spawner,
        joint_state_broadcaster_spawner,
        controller_node,
        # delayed_controller_manager,
        # delayed_joint_broad_spawner,
        # delayed_effort_controller_spawner,
    ])
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Step 1: Find robot description file in the installed package
    pkg = get_package_share_directory('diffbot_ekf')
    xacro_file = os.path.join(pkg, 'urdf', 'diffbot.urdf.xacro')

    # Step 2: Run xacro to turn the .xacro file into plain URDF text
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Step 3: Start Gazebo
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    gazebo_file = os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
    gazebo = IncludeLaunchDescription( 
        PythonLaunchDescriptionSource(gazebo_file),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items()
    )

    # Step 4: robot_state_publisher: publishes the URDF and the link positions
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    # Step 5: Put robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # Spawns 5mm onto ground (base_link is 0.10m above wheel bottom)
        arguments=['-topic', 'robot_description', '-entity', 'diffbot', '-z', '0.105'],
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show the Gazebo window (false = run without it)'),
        gazebo,
        rsp,
        spawn_entity,
    ])
import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Set your actual package name here
    package_name = 'bangla_bot'

    # Path to the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    modelFileRelativePath = 'description/robot.urdf.xacro'

    # this is the absolute path to the model
    pathModelFile = os.path.join(get_package_share_directory(package_name), modelFileRelativePath)

    # get the robot description from the xacro model file
    robotDescription = xacro.process_file(pathModelFile).toxml()


    # GZ Sim (from ros_gz_sim)
    custom_world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'random_hall.sdf')
    if os.path.exists(custom_world_path):
        world_to_use = custom_world_path
        print(f"[INFO] Using custom world: {world_to_use}")
    else:
        world_to_use = 'empty.sdf'
        print(f"[WARN] Custom world not found. Using default: {world_to_use}")

    # Gazebo launch
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={
            'gz_args': f'-r -v4 {world_to_use}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn robot into GZ Sim
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot',
            '-topic', 'robot_description'
        ],
        output='screen',
    )

    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True
        }]
    )

    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ],
        output='screen',
    )


    return LaunchDescription([
        rsp,
        gazeboLaunch ,
        nodeRobotStatePublisher,
        spawnModelNodeGazebo,
        start_gazebo_ros_bridge_cmd
    ])

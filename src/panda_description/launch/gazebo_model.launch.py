import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # this name has to match the robot name in the Xacro file
    robotXacroName='car'
    # this is the name of our package, at the same time this is the name of the
    # folder that will be used to define the paths
    namePackage = 'moveit_resources_panda_description'
    # this is a relative path to the xacro file defining the model
    modelFileRelativePath = 'urdf/car.xacro'
    # this is a relative path to the Gazebo world file
    # worldFileRelativePath = 'world/arena4.world'
    # this is the absolute path to the model
    pathModelFile = os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)
    # this is the absolute path to the world model
    # pathWorldFile = os.path.join(get_package_share_directory(namePackage),worldFileRelativePath)
    # get the robot description from the xacro model file
    robotDescription = xacro.process_file(pathModelFile).toxml()
    # this is the launch file from the gazebo_ros package
    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),'launch', 'gazebo.launch.py'))
    # this is the launch description
    gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch)
    # here, we create a gazebo_ros Node
    spawnModelNode = Node(package='gazebo_ros', executable='spawn_entity.py',
    arguments=['-topic', 'robot_description', '-entity', robotXacroName],output='screen')

    # Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
        'use_sim_time': True}]
    )
    # Load controllers using ros2 control spawn, but delay their activation
    load_joint_state_broadcaster = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
                output='screen'
            )
        ]
    )

    load_arm_controller = TimerAction(
        period=7.0,  # Delay in seconds, ensure this happens after joint_state_broadcaster
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'pusher_controller'],
                output='screen'
            )
        ]
    )

    # Launch MoveIt 2 (robot_state_publisher + move_group + RViz)
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('config'), '/launch/move_group.launch.py']),  # Replace with actual MoveIt package name
    )

    # Launch RViz with MoveIt 2 configuration
    rviz_config_file = FindPackageShare('config')\
        .find('config') + '/config/moveit.rviz'  # Replace with actual path

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # here we create an empty launch description object
    launchDescriptionObject = LaunchDescription()
    # we add gazeboLaunch
    launchDescriptionObject.add_action(gazeboLaunch)

    # we add the two nodes
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    # launchDescriptionObject.add_action(load_joint_state_broadcaster)
    # launchDescriptionObject.add_action(load_arm_controller)
    # launchDescriptionObject.add_action(rviz)
    # launchDescriptionObject.add_action(move_group_launch)
    return launchDescriptionObject
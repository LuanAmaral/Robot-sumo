import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Launch arguments
ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='robot_1', description='Name of the robot to spawn'),
    DeclareLaunchArgument('x', default_value='1', description='Initial x position of the robot'),
    DeclareLaunchArgument('y', default_value='0', description='Initial y position of the robot'),
    DeclareLaunchArgument('z', default_value='0.3', description='Initial z position of the robot'),
    DeclareLaunchArgument('yaw', default_value='0', description='Initial yaw of the robot'),
]

def launch_setup(context):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    
    # Get the path to the robot_sumo package
    robot_sumo_dir = get_package_share_directory('robot_sumo')

    # Get the path to the robot.urdf file
    robot_urdf = os.path.join(robot_sumo_dir, 'worlds', 'robot.urdf')

    # Create the node to spawn the robot in Ignition Gazebo
    spawn_robot_node = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-name', LaunchConfiguration('robot_name'), 
                     '-x', LaunchConfiguration('x'),
                     '-y', LaunchConfiguration('y'),
                     '-z', LaunchConfiguration('z'),
                     '-Y', LaunchConfiguration('yaw'),
                     '-file', robot_urdf],
    )
    
    # Launch bridge between ROS 2 and Ignition Gazebo
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/model/'
                   +robot_name+
                   '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
        output='screen',
    )
    
    pose_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/model/'
                   +robot_name+
                   '/pose@geometry_msgs/msg/Pose@ignition.msgs.Pose'],
        output='screen',
    )
    
    return [spawn_robot_node, gazebo_bridge, pose_bridge]
    

def generate_launch_description():
    # Create the launch description and add the spawn_robot_node
    opc_func = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(opc_func)
    
    return ld

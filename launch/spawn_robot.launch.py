import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

#create a ros2 launch file to spawn the robot in the ignition gazebo world
def generate_launch_description():
    # Get the path to the robot_sumo package
    robot_sumo_dir = get_package_share_directory('robot_sumo')

    # Get the path to the robot.urdf file
    robot_urdf = os.path.join(robot_sumo_dir, 'worlds', 'robot.urdf')

    # Create the node to spawn the robot in Ignition Gazebo
    spawn_robot_node = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-name', 'robot_1', '-x', '1', '-y', '0', '-z', '0.3', '-file', robot_urdf],
        output='screen'
    )
    
    # Launch bridge between ROS 2 and Ignition Gazebo
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/model/robot_1/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
        output='screen',
        namespace='robot'
    )

    # Create the launch description and add the spawn_robot_node
    ld = LaunchDescription()
    ld.add_action(spawn_robot_node)
    ld.add_action(gazebo_bridge)

    return ld

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('model')
    urdf_path = os.path.join(pkg, 'resource', 'model.urdf')
    world_path = os.path.join(pkg, 'worlds', 'maze.sdf')
    textures_path = os.path.join(pkg, 'worlds', 'textures')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    robot_desc = robot_desc.replace(
        'model://model/worlds/textures/',
        f'file://{textures_path}/'
    )

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', '/robot_description',
                '-name', 'robot',
                '-x', '0', '-y', '4', '-z', '0.1',
                '-Y', '-1.5708'
            ],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            ],
            output='screen'
        ),

    ])

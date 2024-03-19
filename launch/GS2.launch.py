from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros_driver_node',
            name='ydlidar_lidar_publisher',
            output='screen',
            parameters=[
                {'port': '/dev/ydlidar'},
                {'frame_id': 'laser_frame'},
                {'baudrate': 921600},
                {'lidar_type': 3},
                {'device_type': 0},
                {'sample_rate': 4},
                {'abnormal_check_count': 4},
                {'resolution_fixed': True},
                {'auto_reconnect': True},
                {'reversion': False},
                {'inverted': False},
                {'isSingleChannel': False},
                {'intensity': True},
                {'support_motor_dtr': False},
                {'invalid_range_is_inf': False},
                {'point_cloud_preservative': False},
                {'angle_min': -180.0},
                {'angle_max': 180.0},
                {'range_min': 0.025},
                {'range_max': 16.0},
                {'frequency': 10.0}
            ]
        ),
        Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
                    )
    ])

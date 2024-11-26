from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
     
    return LaunchDescription([
        Node(
            package='cam2lidar_fusion', 
            executable='cam2lidar_fusion', 
            name='fusion',   
            output='screen',      
            parameters=[{
                "lidar_mode": "3d",
                'cam_topic': '/front_camera/image_raw',
                'cam_info_topic': '/front_camera/camera_info',
                'lidar_topic': '/cloud',
                'fused_topic': '/fused/points',
                'fused_cam_topic': '/fused/camera',
                'fused_cam_pub': True,
                'fused_pub': True,
                'fused_cam_veiw': True,
                'camera_positon': [0.0, 0.0, -0.49],
                'camera_rpy': [4.71, 4.71, 3.14],
                'lidar_positon': [0.0, 0.0, 0.0],
                'lidar_rpy': [0.0, 0.0, 0.0],
            }]
        ),
    ])

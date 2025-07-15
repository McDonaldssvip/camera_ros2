from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    declare_cam_id = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='camera ID'
    )
    declare_cam_width = DeclareLaunchArgument(
        'camera_width',
        default_value='640',
        description='camera frame width'
    )
    declare_cam_height = DeclareLaunchArgument(
        'camera_height',
        default_value='480',
        description='camera frame height'
    )
    declare_cam_fps = DeclareLaunchArgument(
        'camera_fps',
        default_value='30',
        description='camera frame rate (fps)'
    )

    camera_node = Node(
        package='camera',
        executable='camera_node',
        name='camera_high',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'width':     LaunchConfiguration('camera_width'),
            'height':    LaunchConfiguration('camera_height'),
            'fps':       LaunchConfiguration('camera_fps'),
        }],
    )

    return LaunchDescription([
        declare_cam_id,
        declare_cam_width,
        declare_cam_height,
        declare_cam_fps,
        camera_node,
    ])

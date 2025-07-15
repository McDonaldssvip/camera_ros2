from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    declare_cam1_id = DeclareLaunchArgument(
        'camera1_id',
        default_value='0',
        description='High-res camera ID'
    )
    declare_cam1_width = DeclareLaunchArgument(
        'camera1_width',
        default_value='640',
        description='High-res camera frame width'
    )
    declare_cam1_height = DeclareLaunchArgument(
        'camera1_height',
        default_value='480',
        description='High-res camera frame height'
    )
    declare_cam1_fps = DeclareLaunchArgument(
        'camera1_fps',
        default_value='30',
        description='High-res camera frame rate (fps)'
    )

    declare_cam2_id = DeclareLaunchArgument(
        'camera2_id',
        default_value='2',
        description='Wrist camera ID'
    )
    declare_cam2_width = DeclareLaunchArgument(
        'camera2_width',
        default_value='640',
        description='Wrist camera frame width'
    )
    declare_cam2_height = DeclareLaunchArgument(
        'camera2_height',
        default_value='480',
        description='Wrist camera frame height'
    )
    declare_cam2_fps = DeclareLaunchArgument(
        'camera2_fps',
        default_value='30',
        description='Wrist camera frame rate (fps)'
    )

    camera1_node = Node(
        package='camera',
        executable='camera_node',
        name='camera_high',
        parameters=[{
            'camera_id': LaunchConfiguration('camera1_id'),
            'width':     LaunchConfiguration('camera1_width'),
            'height':    LaunchConfiguration('camera1_height'),
            'fps':       LaunchConfiguration('camera1_fps'),
        }],
    )

    camera2_node = Node(
        package='camera',
        executable='camera_node',
        name='camera_wrist',
        parameters=[{
            'camera_id': LaunchConfiguration('camera2_id'),
            'width':     LaunchConfiguration('camera2_width'),
            'height':    LaunchConfiguration('camera2_height'),
            'fps':       LaunchConfiguration('camera2_fps'),
        }],
    )

    return LaunchDescription([

        declare_cam1_id,
        declare_cam1_width,
        declare_cam1_height,
        declare_cam1_fps,

        declare_cam2_id,
        declare_cam2_width,
        declare_cam2_height,
        declare_cam2_fps,

        camera1_node,
        camera2_node,
    ])

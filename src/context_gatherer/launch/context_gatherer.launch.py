from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_topic = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Source image topic for context snapshots',
    )

    odometry_topic = DeclareLaunchArgument(
        'odometry_topic',
        default_value='/odometry/global',
        description='Odometry topic providing robot pose/velocity',
    )

    map_topic = DeclareLaunchArgument(
        'map_topic',
        default_value='/map',
        description='Occupancy grid topic for environment map',
    )

    geo_topic = DeclareLaunchArgument(
        'geo_topic',
        default_value='/gps/geo',
        description='Geodetic position topic',
    )

    image_transport = DeclareLaunchArgument(
        'image_transport',
        default_value='raw',
        description='Preferred image transport hint',
    )

    data_timeout = DeclareLaunchArgument(
        'data_timeout_sec',
        default_value='5.0',
        description='Maximum staleness (seconds) accepted for sensor data',
    )

    jpeg_quality = DeclareLaunchArgument(
        'image_jpeg_quality',
        default_value='70',
        description='JPEG quality used to encode preview thumbnails (1-100)',
    )

    preview_width = DeclareLaunchArgument(
        'image_target_width',
        default_value='320',
        description='Optional width for preview scaling (0 to keep original)',
    )

    preview_height = DeclareLaunchArgument(
        'image_target_height',
        default_value='0',
        description='Optional height for preview scaling (0 to keep aspect ratio)',
    )

    node = Node(
        package='context_gatherer',
        executable='context_gatherer_node',
        name='context_gatherer',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'map_topic': LaunchConfiguration('map_topic'),
            'geo_topic': LaunchConfiguration('geo_topic'),
            'image_transport': LaunchConfiguration('image_transport'),
            'data_timeout_sec': LaunchConfiguration('data_timeout_sec'),
            'image_jpeg_quality': LaunchConfiguration('image_jpeg_quality'),
            'image_target_width': LaunchConfiguration('image_target_width'),
            'image_target_height': LaunchConfiguration('image_target_height'),
        }],
    )

    return LaunchDescription([
        camera_topic,
        odometry_topic,
        map_topic,
        geo_topic,
        image_transport,
        data_timeout,
        jpeg_quality,
        preview_width,
        preview_height,
        node,
    ])

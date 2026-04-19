from launch import LaunchDescription

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    camera_container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='left_rectify_node',
                remappings=[('image', '/left_camera/image_raw'),
                            ('camera_info', '/left_camera/camera_info'),
                            ('image_rect', '/left_camera/image_rect')]
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='right_rectify_node',
                remappings=[('image', '/right_camera/image_raw'),
                            ('camera_info', '/right_camera/camera_info'),
                            ('image_rect', '/right_camera/image_rect')]
            ),
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::DisparityNode',
                name='disparity_node',
                remappings=[('left/image_rect', '/left_camera/image_rect'),
                            ('right/image_rect', '/right_camera/image_rect'),
                            ('disparity', '/camera/disparity')]
            ),
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::PointCloudNode',
                name='point_cloud_node',
                remappings=[('left/image_rect_color', '/left_camera/image_rect'),
                            ('right/camera_info', '/right_camera/camera_info'),
                            ('disparity', '/camera/disparity'),
                            ('points2', '/camera/points')]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        camera_container,
    ])
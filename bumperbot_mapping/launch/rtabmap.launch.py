import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    bumperbot_mapping_pkg = get_package_share_directory("bumperbot_mapping")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    rtabmap_container = ComposableNodeContainer(
        name='rtabmap_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
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
                package='rtabmap_odom',
                plugin='rtabmap_odom::StereoOdometry',
                name='stereo_odometry',
                parameters=[
                    os.path.join(
                        bumperbot_mapping_pkg,
                        "config",
                        "rtabmap.yaml"),
                    {"use_sim_time": use_sim_time}
                ],
                remappings=[
                    ("left/image_rect", "/left_camera/image_rect"),
                    ("right/image_rect", "/right_camera/image_rect"),
                    ("left/camera_info", "/left_camera/camera_info"),
                    ("right/camera_info", "/right_camera/camera_info"),
                    ("odom", "/camera/odom"),
                    ("imu", "/imu/filtered")
                ],
            ),
            ComposableNode(
                package='rtabmap_slam',
                plugin='rtabmap_slam::CoreWrapper',
                name='rtabmap',
                parameters=[
                    os.path.join(
                        bumperbot_mapping_pkg,
                        "config",
                        "rtabmap.yaml"),
                    {"use_sim_time": use_sim_time}
                ],
                remappings=[
                    ("left/image_rect", "/left_camera/image_rect"),
                    ("right/image_rect", "/right_camera/image_rect"),
                    ("left/camera_info", "/left_camera/camera_info"),
                    ("right/camera_info", "/right_camera/camera_info"),
                    ("imu", "/imu/filtered"),
                ],
            ),
        ],
        output='screen',
    )

    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        output="screen",
        parameters=[
            {"subscribe_stereo": True,
             "subscribe_odom_info": True,
             "approx_sync": False,
             "wait_imu_to_init": False,
             "use_sim_time": use_sim_time}
        ],
        remappings=[
            ("imu", "/imu/out"),
            ("left/image_rect", "/left_camera/image_rect"),
            ("right/image_rect", "/right_camera/image_rect"),
            ("left/camera_info", "/left_camera/camera_info"),
            ("right/camera_info", "/right_camera/camera_info"),
        ],
    )

    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        output="screen",
        parameters=[{"use_mag": False, 
                     "world_frame":"enu", 
                     "publish_tf": False,
                     "use_sim_time": use_sim_time}],
        remappings=[("imu/data_raw", "/imu/out"),
                    ("imu/data", "/imu/filtered")]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rtabmap_container,
        # rtabmap_viz,
        imu_filter,
    ])
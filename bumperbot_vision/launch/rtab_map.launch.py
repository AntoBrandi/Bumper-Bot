import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    bumperbot_vision_pkg = get_package_share_directory("bumperbot_vision")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    rtabmap_odom = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        output="screen",
        emulate_tty=True,
        parameters=[
            os.path.join(
                bumperbot_vision_pkg,
                "config",
                "rtabmap.yaml"),
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
            ("imu", "/imu/out"),
            ("depth/image", "/rgbd_camera/depth/image_raw"),
            ("rgb/image", "/rgbd_camera/image_raw"),
            ("rgb/camera_info", "/rgbd_camera/camera_info"),
        ],
    )

    rtabmap_slam = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        output="screen",
        parameters=[
            os.path.join(
                bumperbot_vision_pkg,
                "config",
                "rtabmap.yaml"),
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
            ("imu", "/imu/out"),
            ("depth/image", "/rgbd_camera/depth/image_raw"),
            ("rgb/image", "/rgbd_camera/image_raw"),
            ("rgb/camera_info", "/rgbd_camera/camera_info"),
        ],
    )

    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        output="screen",
        parameters=[
            {"subscribe_depth": True,
             "subscribe_odom_info": True,
             "approx_sync": False,
             "wait_imu_to_init": True,
             "use_sim_time": use_sim_time}
        ],
        remappings=[
            ("imu", "/imu/out"),
            ("depth/image", "/rgbd_camera/depth/image_raw"),
            ("rgb/image", "/rgbd_camera/image_raw"),
            ("rgb/camera_info", "/rgbd_camera/camera_info"),
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
        remappings=[("imu/data_raw", "/camera/imu")]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rtabmap_odom,
        rtabmap_slam,
        # rtabmap_viz,
        imu_filter,
    ])
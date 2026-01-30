import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Locate the turtlebot4_gz_bringup launch file
    try:
        tb4_pkg_share = get_package_share_directory('turtlebot4_gz_bringup')
    except Exception:
        tb4_pkg_share = ''

    tb4_launch = os.path.join(tb4_pkg_share, 'launch', 'turtlebot4_gz.launch.py')

    include_tb4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb4_launch),
        launch_arguments={
            'x': '-7.8',
            'y': '3',
            'z': '0.01',
            'yaw': '3.1416',
        }.items()
    )

    # Start vision detector node
    vision_detector = ExecuteProcess(
        cmd=['ros2', 'run', 'tb4_gz_rqt_launch', 'vision_detector_node'],
        output='screen',
    )

    # Start rqt_image_view for vision detections
    rqt_vision = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/vision/detections'],
        output='screen',
    )

    # Set initial pose after 5 seconds delay (wait for localization to start)
    set_initial_pose = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once', '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}'
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        include_tb4,
        vision_detector,
        rqt_vision,
        set_initial_pose,
    ])

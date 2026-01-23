import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
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
    )

    # Start rqt_image_view as a separate process so it opens the GUI
    rqt = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
        output='screen',
    )

    return LaunchDescription([
        include_tb4,
        rqt,
    ])

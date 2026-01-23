# tb4_gz_rqt_launch

Small helper package that includes TurtleBot4 Gazebo bringup and starts `rqt_image_view`.

Usage:

- Install system deps (replace <distro> with `jazzy`):
  sudo apt update && sudo apt install ros-<distro>-rqt-image-view ros-<distro>-turtlebot4-gz-bringup

- Build and source:
  colcon build --symlink-install
  source install/setup.bash

- Launch:
  ros2 launch tb4_gz_rqt_launch tb4_gz_rqt_launch.launch.py

Notes:
- Ensure you have a working display (GUI) for `rqt_image_view` (export DISPLAY or use X-forwarding).
- Verify the camera topic (e.g., `/camera/image_raw`) after spawn and select it in `rqt_image_view`.

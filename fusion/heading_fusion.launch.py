"""Heading sensor-fusion EKF (robot_localization).

Fuses OAK-D BMI270 gyro-rate (base) + rtabmap VO yaw + rf2o LiDAR yaw into one
low-drift heading, published as the odom -> base_footprint transform.

Deploys as loose files (like ros2_bridge.py), launched in-container:
    ros2 launch /path/to/heading_fusion.launch.py
ekf_heading.yaml must sit next to this file.

Assumes the source nodes are already up: /oak/imu/data + /vo_odom via
oak_ros_mode.sh, /odom_rf2o via the rf2o launch.

VERIFY-GATES before trusting this for nav (the live turn-test, when ribs allow):
  1. Gyro base is /imu/data_raw (ESP32 raw gyro), NOT /oak/imu/data (OAK IMU streams
     zero data, cause unresolved). /imu/data_raw tested clean under LIGHT motor load;
     still verify under real DRIVING load (wheels on ground) — log /cmd_vel +
     /imu/data_raw together. Confirm its frame_id is in the TF tree.
  2. /vo_odom covariance balloons (~9999) on the 0,0,0,0 dropout frames, so the EKF
     down-weights them. If it reports small covariance on a null frame, add explicit
     rejection — do not feed a zero quaternion to the filter.
  3. Disable the CURRENT odom -> base_footprint owner: it is base_node's pub_odom_tf
     (default true, verified live), NOT rf2o -- rf2o already ships publish_tf:=False.
     Relaunch bringup_lidar with pub_odom_tf:=false so this EKF owns the edge. Only
     flip it WITH the EKF running, else nothing publishes odom -> base_footprint.
     (The "duplicate" /rf2o_laser_odometry is a harmless ghost DDS participant left by
     pkill -9 shutdowns -- one real process, one publisher. A container restart clears
     it; shut rf2o down with -INT/-TERM, not -9, to stop minting them.)
  4. Turn a known 90 deg: confirm fused yaw tracks it, holds when stationary, rides
     through a VO dropout on the gyro alone, and check SCALE (rf2o read ~107 for ~90
     on the bench — calibrate against a known angle).
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(os.path.dirname(__file__), 'ekf_heading.yaml')
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[cfg],
        ),
    ])

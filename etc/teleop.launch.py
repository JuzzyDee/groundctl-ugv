"""teleop.launch.py — joy_node + teleop_twist_joy with our button map.

Reads /dev/input/js0 (the Waveshare dongle controller), publishes Joy on
/joy, converts to Twist on /cmd_vel_joy. Deadman on LB, turbo on RB, A as
emergency. Params in /tmp/teleop_params.yaml (scp'd from etc/ on the Mac).

Launch inside the ugv_jp6 container:

    docker exec -d ugv_jp6 bash -c \
      'source /opt/ros/humble/setup.bash && \
       source /home/ws/ugv_ws/install/setup.bash && \
       ros2 launch /tmp/teleop.launch.py > /tmp/teleop.log 2>&1'

Verify:
    docker exec ugv_jp6 bash -c 'source /opt/ros/humble/setup.bash && ros2 topic echo /cmd_vel_joy'

Hold LB, push the left stick forward → linear.x should go positive.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = "/tmp/teleop_params.yaml"

    return LaunchDescription([
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_node",
            parameters=[params_file],
            # No remap — teleop_node's default publish topic is /cmd_vel, which
            # ugv_driver subscribes to directly. Waveshare stack doesn't actually
            # run twist_mux despite what the bringup docs suggest, so we publish
            # straight to the driver's input. Future: when Haiku also drives,
            # install twist_mux properly and route both sources through it with
            # joystick at higher priority.
            output="screen",
        ),
    ])

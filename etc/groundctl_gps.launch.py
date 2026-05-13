"""
groundctl_gps.launch.py — combined GPS stack for the rover.

Brings up three nodes in one launch:
  1. ntrip_client      — TLS-NTRIP to AUSCORS, publishes /ntrip_client/rtcm
  2. ublox_dgnss       — talks to ZED-F9R via libusb, forwards RTCM corrections,
                         publishes /ubx_nav_hp_pos_llh + other UBX topics
  3. nav_sat_fix_hp    — converts UBX-NAV-HPPOSLLH → sensor_msgs/NavSatFix on /fix

Why this exists instead of just chaining the upstream ublox_dgnss launches:
  - Loads creds from /home/ws/.groundctl.env in Python (clean, no shell escaping)
  - Wraps username/password in ParameterValue(value_type=str) to bypass the
    YAML parser. Without this, NTRIP passwords containing ':', '#', '@', etc.
    fail with "Unable to parse the value of parameter password as yaml".

Source the env file before launching:
  set -a; source /home/ws/.groundctl.env; set +a
  ros2 launch /tmp/groundctl_gps.launch.py
"""
import os
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ntrip_user = os.environ.get('AUSCORS_NTRIP_USER', '')
    ntrip_pass = os.environ.get('AUSCORS_NTRIP_PASS', '')
    ntrip_host = os.environ.get('AUSCORS_NTRIP_HOST', 'ntrip.data.gnss.ga.gov.au')
    ntrip_port = int(os.environ.get('AUSCORS_NTRIP_PORT', '443'))
    ntrip_mount = os.environ.get('AUSCORS_MOUNTPOINT', 'RSBY00AUS0')

    if not ntrip_user or not ntrip_pass:
        raise RuntimeError(
            "AUSCORS_NTRIP_USER/AUSCORS_NTRIP_PASS not set. "
            "Source /home/ws/.groundctl.env before launching."
        )

    ntrip_params = [{
        'use_https': True,
        'host': ntrip_host,
        'port': ntrip_port,
        'mountpoint': ntrip_mount,
        'username': ParameterValue(ntrip_user, value_type=str),
        'password': ParameterValue(ntrip_pass, value_type=str),
        'maxage_conn': 30,
    }]

    ntrip_container = ComposableNodeContainer(
        name='ntrip_client_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='ntrip_client_node',
                plugin='ublox_dgnss::NTRIPClientNode',
                name='ntrip_client',
                parameters=ntrip_params,
            )
        ],
    )

    # Mirrors upstream ublox_rover_hpposllh_navsatfix.launch.py defaults,
    # with DEVICE_FAMILY overridden to F9R for sensor-fusion-aware behaviour.
    dgnss_params = [
        {'DEVICE_FAMILY': 'F9R'},
        {'DEVICE_SERIAL_STRING': ''},
        {'FRAME_ID': 'gps'},
        {'CFG_USBOUTPROT_NMEA': False},
        {'CFG_RATE_MEAS': 10},
        {'CFG_RATE_NAV': 100},
        {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 1},
        {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 5},
        {'CFG_MSGOUT_UBX_NAV_COV_USB': 1},
        {'CFG_MSGOUT_UBX_NAV_PVT_USB': 1},
        {'CFG_MSGOUT_UBX_NAV_SAT_USB': 5},
        {'CFG_MSGOUT_UBX_NAV_SIG_USB': 5},
        {'CFG_MSGOUT_UBX_RXM_RTCM_USB': 1},
        # Sensor fusion installation config (task #28).
        # Auto-mount-alignment: F9R figures out its own orientation from
        # observed driving dynamics (left/right turns). Skips the need to
        # manually compute Euler angles from the chip's mounted orientation.
        {'CFG_SFIMU_AUTO_MNTALG_ENA': True},
        # IMU centre → antenna phase centre, rover body frame (X fwd, Y left,
        # Z up), cm signed. Antenna is directly above the chip in the housing,
        # slightly forward, no port/starboard offset.
        {'CFG_SFIMU_IMU2ANT_LA_X': 2},
        {'CFG_SFIMU_IMU2ANT_LA_Y': 0},
        {'CFG_SFIMU_IMU2ANT_LA_Z': 7},
        # IMU centre → vehicle reference point (centre of rear wheel pair).
        # Mount is on rear-right of the deck, forward of the rear axle, so VRP
        # is behind/left/below the IMU.
        {'CFG_SFODO_IMU2VRP_LA_X': -7},
        {'CFG_SFODO_IMU2VRP_LA_Y': 9},
        {'CFG_SFODO_IMU2VRP_LA_Z': -10},
    ]

    dgnss_container = ComposableNodeContainer(
        name='ublox_dgnss_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_dgnss_node',
                plugin='ublox_dgnss::UbloxDGNSSNode',
                name='ublox_dgnss',
                parameters=dgnss_params,
            )
        ],
    )

    navsatfix_container = ComposableNodeContainer(
        name='ublox_nav_sat_fix_hp_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_nav_sat_fix_hp_node',
                plugin='ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode',
                name='ublox_nav_sat_fix_hp',
            )
        ],
    )

    return launch.LaunchDescription([
        ntrip_container,
        dgnss_container,
        navsatfix_container,
    ])

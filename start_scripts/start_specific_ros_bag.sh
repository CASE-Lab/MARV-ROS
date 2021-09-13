#!/bin/bash

# Topics to log
specific_ros_bag_topics="
/marv/sys/status/logging_marker
 /marv/sys/status/current_scenario
 /marv/sys/status/state_12V_auto
 /marv/sys/log/log1_ncu
 /marv/sys/log/log1_pdu
 /marv/sys/log/log1_tcu
 /marv/sys/log/log2_tcu
 /marv/sys/status/heartbeat_acu
 /marv/sys/status/heartbeat_pdu
 /marv/sys/status/heartbeat_rcu
 /marv/sys/status/heartbeat_tcu
 /marv/sys/status/heartbeat_ncu
 /marv/sys/status/heartbeat_ucu
 /marv/sys/status/heartbeat_ocu
 /marv/sys/ctrl/scenario_state
 /marv/sys/ctrl/cmd_steering
 /sbg/ekf_euler
 /sbg/ekf_nav
 /sbg/utc_time
 /sbg/imu_data
 /marv/nav/sbg_ref_pos
 /marv/nav/sbg_ref_pos_state
 /marv/nav/sbg_pose
 /marv/nav/sbg_ekf_status
 /marv/nav/sbg_current_pos
 /marv/nav/sbg_velocity_magnitude
 /marv/nav/sbg_velocity
 /marv/nav/sbg_acceleration
 /marv/nav/sbg_angular_velocity
"

if [ ! -d "/logs_specific" ] 
then
    mkdir logs_specific
fi
cd logs_specific

ec=0; while (($ec < 2)); do echo "starting..."; sleep 1; ros2 bag record $specific_ros_bag_topics -b 2500000; (($?==2)) && break; ((ec+=1)); done

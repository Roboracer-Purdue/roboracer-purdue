LIST OF PACKAGES
- IV2026 Roboracer code

LIST OF CHANGES
*** Added temporary multiplier of * 1.2 to speed in publish_drive(), should be implemented otherwise
+ Added reference to waypoint csv by path (Adding to shared in setup.py, adding script on top of the original code to load default path, and apply default path in declare_all_parameter())
+ IV_2026_SIM_clean.png is the grayscale version of the original image suitable for gym
+ Generated a new waypoints "IV26_gen_rl.csv"
+ Generated a new waypoints "IV26_gen_rl2.csv", with 'Black Patches' buffer on sharp turn
+ Generated a new waypoints "IV26_gen_rl3.csv", The same as rl2 but with faster max speed

* Changed default obstacle_stop_distance from 0.85 to 0.30
* Changed default obstacle_slow_distance from 1.8 to 0.75
* Changed default wall_min_distance from 0.25 to 0.12
* Changed default path_corridor_width from 0.45 to 0.25
* Changed default collision_radius from 0.40 to 0.22
* Changed default opponent_detect_distance from 3.0 to 1.4
* Changed default safe_follow_distance from 1.0 to 0.45
* Changed default frenet_max_offset from 0.35 to 0.20
* Changed default steering_smoothing_alpha from 0.50 to 0.10
* Changed default accel_limit from 0.8 to 2.5

1. To run this on sim, use the following command to subscribe to edom instead of amcl
'''
ros2 run race_day_controller f1tenth_race_day \
  --ros-args \
  -p use_amcl_pose:=false \
  -p odom_topic:=/ego_racecar/odom \
  -p scan_topic:=/scan \
  -p drive_topic:=/drive \
==OPTIONAL==
  -p waypoint_file:=/absolute/path/to/waypoints.csv
  -p max_speed:=4.0
'''

2. 
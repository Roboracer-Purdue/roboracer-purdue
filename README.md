Car 2 Stack, are some useful commands < > means you should change that position to one that matches your directory. 

ssh into car 

ssh arc@<ip_address>

Password: ORIN11    or orin11



Launching f1tenth stack 

ros2 launch f1tenth_stack bringup_launch.py


Running slam tool box 

ros2 launch slam_toolbox online_async_launch.py   use_sim_time:=false   slam_params_file:=/home/arc/f1tenth_ws/src/config/slam_params.yaml

saving a map 

ros2 run nav2_map_server map_saver_cli -f ~/f1tenth_ws/maps/<map name> -t /map --ros-args -p map_subscribe_transient_local:=true


running localization (AMCL)

ros2 launch f1tenth_localization amcl_localization.launch.py map:=/home/arc/f1tenth_ws/maps/<roboracer_track/racetrack.yaml>


Overlay way points on map 

ros2 run trajectory_tools overlay_waypoints_on_map \
  --ros-args \
  -p waypoints:=/home/arc/f1tenth_ws/waypoints/raw/<raceline.csv> \
  -p map_yaml:=/home/arc/f1tenth_ws/maps/<roboracer_track/racetrack.yaml>


Run pure pursuit

ros2 launch pure_pursuit pure_pursuit.launch.py


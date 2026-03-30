# Particle Filter
### Installation
Check out installation guide on course excel 
"[Penn] T05 Running SLAM and Particle Filter"
### What does PF do?
Initialize a map server as specified in the config/localize.yaml
Read the scan and odom message and use complex AF math to compute 
the most probable position of the car and announce it to topic
"/pf/viz/inferred_pose"

Example:
header:
  stamp:
    sec: 1774885573
    nanosec: 359000999
  frame_id: /map
pose:
  position:
    x: -10.161210007916358
    y: 8.517119661138675
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.9999957118964682
    w: 0.002928513048587313
---

Note that PF does initialize its map server, to use it in conjunction
with the sim, you must disable the sim's map server and nav2.

### Running PF
ros2 launch particle_filter localize_launch.py

### Running PF with Sim
I have included gym_noserver.py, this can be placed directly in 
f1_tenth_gym launch folder, make sure to colcon before use

ros2 launch f1tenth_gym_ros gym_noserver.py
ros2 launch particle_filter localize_launch.py





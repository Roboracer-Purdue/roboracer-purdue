# F1TENTH Sim — Run Cheatsheet

Quick command reference for daily use. Each block is one terminal.
Workspace root assumed at: `~/Desktop/roboracer-purdue/Meghaj_roboracer_ws`

---

## Every new terminal starts with this
```bash
source /opt/ros/humble/setup.bash                 # load ROS 2 Humble
cd ~/Desktop/roboracer-purdue/Meghaj_roboracer_ws # go to the workspace
source install/local_setup.bash                   # load this workspace's packages
```

---

## 1. Launch the simulator  (Terminal 1)
```bash
cd ~/Desktop/roboracer-purdue/Meghaj_roboracer_ws/one_car_gym_roboracer # go to the gym scripts
ros2 launch f1tenth_gym_ros gym_bridge_launch.py   # starts bridge + map + RViz
```
Wait for RViz to open with the map and car. Leave this running.

---

## 2. Drive manually  (Terminal 2)
```bash
python3 ~/key_drive.py     # keyboard teleop: w/s = speed, a/d = steer, space = stop
```
Keep this terminal focused while driving, or keys won't register.

---

## 3. Run your algorithm (Assumes Pure Pursuit) (Terminal 3)
```bash
python3 ~/path/to/pure_pursuit.py     # your controller: follows the raceline
```
(Swap in the logger or velocity profiler here when that's the task instead.)

---

## Log a new raceline (instead of step 3)
```bash
python3 ~/path/to/waypoint_logger.py   # records x,y,yaw,speed as you drive
# drive a full lap with key_drive.py, then Ctrl-C to save the CSV
```

---

## Generate a velocity profile from a logged lap
```bash
python3 ~/path/to/velocity_profile.py <in_lap.csv> <out_profiled.csv>
# adds curvature-based speeds + braking zones to the raceline
```

---

## Quick health checks (any spare terminal)
```bash
ros2 node list                     # /bridge should be present
ros2 lifecycle get /map_server     # should say 'active'
ros2 topic echo /ego_racecar/odom --once   # car pose is publishing
ros2 topic echo /scan --once               # LiDAR is publishing
```

---

## Only after editing code/config/launch  (NOT every run)
```bash
cd ~/Desktop/roboracer-purdue/Meghaj_roboracer_ws
colcon build                       # rebuild changed packages
source install/local_setup.bash    # reload
```
> Editing a launch file or `sim.yaml` requires a rebuild — launch reads from
> `install/`, not `src/`. Editing a plain `python3 script.py` you run directly
> does NOT need a rebuild.

---

## If RViz hangs / no window (VM only, one-time)
```bash
echo 'export QT_QPA_PLATFORM=xcb' >> ~/.bashrc
echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc
source ~/.bashrc
```

---

## Typical session, start to finish
```bash
# T1: sim
source /opt/ros/humble/setup.bash && cd ~/Desktop/roboracer-purdue/Meghaj_roboracer_ws && source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# T2: drive
source /opt/ros/humble/setup.bash
python3 ~/key_drive.py

# T3: algorithm
source /opt/ros/humble/setup.bash && cd ~/Desktop/roboracer-purdue/Meghaj_roboracer_ws && source install/local_setup.bash
python3 ~/path/to/pure_pursuit.py
```

---

## Back up your work to the repo (git)

Repo root: `~/Desktop/roboracer-purdue`  (the workspace lives inside it)

### Everyday: save and push your changes
```bash
cd ~/Desktop/roboracer-purdue     # go to the REPO root (not the workspace)
git status                        # see what changed (build/install/log should NOT appear)
git add -A                        # stage all changes
git commit -m "describe what you changed"   # snapshot it locally
git push                          # send it up to GitHub
```

### Pull the latest before you start working
```bash
cd ~/Desktop/roboracer-purdue
git pull                          # grab any changes from GitHub first
```

### One-time setup (only if this repo isn't connected yet)
```bash
cd ~/Desktop/roboracer-purdue
git init                                  # start git here (skip if already a repo)
git remote add origin <your-repo-URL>     # link to GitHub (skip if already linked)
git add -A
git commit -m "initial workspace backup"
git branch -M main
git push -u origin main                   # first push sets the upstream
```

> Before the first commit, confirm `.gitignore` is at the repo root and
> `git status` shows **no** `build/`, `install/`, `log/`, or `__pycache__`
> entries. If it does, the ignore file is missing or misnamed.

# F1TENTH Gym Simulator — Team Setup Guide (ROS 2 Humble in a VM)

A reproducible guide to get **every team member** an identical, working F1TENTH
`gym_bridge` simulator running on **Ubuntu 22.04 + ROS 2 Humble**, inside a virtual
machine — regardless of whether the host laptop is **macOS or Windows**.

**Key principle:** once you're inside the Ubuntu guest, the sim install is *identical*
for everyone (Sections 2–12). The only host-specific part is **provisioning the VM**
(Section 1). The RViz display fix in **Section 7 is universal** — every VM needs it,
on every hypervisor.

---

## 0. What you're installing

Two pieces work together:

| Component | What it is | Where it lives |
|-----------|-----------|----------------|
| **`f1tenth_gym`** | The Python physics/LiDAR simulator backend. Registers the `f110_gym:f110-v0` Gym environment. | Installed into **system Python** via `pip` |
| **`f1tenth_gym_ros`** | The ROS 2 bridge: wraps the gym in a node, publishes `/scan`, `/ego_racecar/odom`, TF, and the map; subscribes to `/drive` and `/cmd_vel`. | A package inside a colcon workspace |

The bridge node (`gym_bridge`) is the only thing that creates the `map` TF frame and
publishes the odometry that drives the sim — if it isn't alive, RViz shows
"Frame [map] does not exist" and nothing moves.

---

## 1. Provision your VM (the only host-specific step)

Everyone runs the **same Ubuntu 22.04 desktop** guest. How you create that guest
depends on your host.

### 1.1 Pick a hypervisor for your host

| Host OS | Recommended hypervisor(s) | Guest CPU arch |
|---------|---------------------------|----------------|
| **macOS — Apple Silicon (M1–M4)** | Parallels Desktop (smoothest), UTM (free, QEMU-based), VMware Fusion | **ARM64 (aarch64)** |
| **macOS — Intel** | Parallels Desktop, VMware Fusion, VirtualBox | x86_64 (amd64) |
| **Windows 10/11** | VMware Workstation Player, VirtualBox, Hyper-V (Pro editions) | x86_64 (amd64) |

> Licensing changes over time — Parallels is paid; VMware Fusion/Workstation and
> VirtualBox have free personal-use tiers at time of writing. Verify current terms.

### 1.2 Get the matching Ubuntu 22.04 image

- **Apple Silicon hosts → ARM64 (aarch64)** Ubuntu 22.04 desktop ISO.
- **Everything else → AMD64 (x86_64)** Ubuntu 22.04 desktop ISO.

**Why the arch matters:** ROS 2 Humble apt packages exist for *both* arm64 and amd64,
so ROS is fine either way. The `f1tenth_gym` Python deps (`numpy`, `scipy`, `numba`)
also ship wheels for both — but on ARM64 a wheel is occasionally missing and pip will
build from source, which needs `build-essential python3-dev` installed (Section 2 covers
this). Functionally the sim behaves identically on both arches.

### 1.3 VM resource allocation (recommended)

The sim runs physics + RViz + your own nodes simultaneously, so give the guest headroom:

| Resource | Minimum | Recommended |
|----------|---------|-------------|
| CPU cores | 2 | **4+** |
| RAM | 4 GB | **8 GB** |
| Disk | 30 GB | **40 GB+** |
| 3D acceleration | — | **Enabled** (in the VM's display/graphics settings) |

### 1.4 Install your hypervisor's guest tools, then reboot

Guest tools provide display integration, clipboard, and 3D support. **Without them,
RViz is far more likely to hang.** Each hypervisor has its own package — they all do the
same job:

| Hypervisor | How to install guest tools |
|------------|----------------------------|
| **Parallels** | Menu → *Actions → Install Parallels Tools*, run the installer in Ubuntu, reboot |
| **VMware (Fusion/Workstation)** | `sudo apt install open-vm-tools open-vm-tools-desktop`, reboot |
| **VirtualBox** | *Devices → Insert Guest Additions CD…*, run it (or `sudo apt install virtualbox-guest-utils virtualbox-guest-x11`), reboot |
| **UTM / QEMU** | `sudo apt install spice-vdagent qemu-guest-agent`, reboot |
| **Hyper-V** | Enhanced session + `sudo apt install linux-azure` (xrdp-based); 3D is limited — rely on Section 7 |

> Even with guest tools installed, **apply the Section 7 display fix** — it's what
> actually makes RViz reliable across all of these.

### 1.5 Do NOT use conda inside the guest

ROS 2 Humble is built against the guest's **system Python 3.10**. An active conda
environment causes `rclpy`/`numpy` ABI mismatches and import failures. Keep conda
deactivated:
```bash
conda deactivate                                  # until no (base)/env name in prompt
conda config --set auto_activate_base false       # stop auto-activation
```

> **WSL2 note (Windows):** WSL2 is a lightweight VM and *can* run this stack via WSLg,
> but its display path differs from a full desktop VM. For team uniformity, use a full
> Ubuntu 22.04 desktop VM as above so everyone's environment is identical.

---

## 2. System (apt) dependencies

Identical on every guest (arm64 or amd64):
```bash
sudo apt update
sudo apt install -y \
    python3-pip git build-essential python3-dev \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-ackermann-msgs \
    ros-humble-xacro \
    ros-humble-teleop-twist-keyboard
```
`build-essential python3-dev` are included so that, on ARM64, any Python dep without a
prebuilt wheel can compile. The bridge launch needs `nav2_map_server` +
`nav2_lifecycle_manager` (navigation2), `xacro`, and `ackermann_msgs`.

---

## 3. Install the F1TENTH gym (Python backend)

```bash
cd ~/Desktop
git clone https://github.com/f1tenth/f1tenth_gym.git
cd f1tenth_gym
pip3 install -e .
```
`-e` installs it editable into **system Python**. Verify the env registers:
```bash
python3 -c "import gym; gym.make('f110_gym:f110-v0', map='maps/levine', map_ext='.png', num_agents=1); print('gym OK')"
```
> Errors about `transforms3d`, `numba`, or `coverage` → **Section 4**.

---

## 4. Python dependencies that bite (apply these)

Two failures that crash the bridge in `__init__`, both fixed with `pip`:

**a) `ModuleNotFoundError: No module named 'transforms3d'`**
```bash
pip3 install transforms3d
```

**b) `AttributeError: module 'coverage' has no attribute 'types'`** (numba/coverage clash)
```bash
pip3 install --upgrade coverage
# if it persists, the sim doesn't need coverage at all:
pip3 uninstall -y coverage
```

> These hit native installs because the upstream project is normally containerized with
> pinned deps. On a VM they fall to system Python, which may have incompatible versions.

---

## 5. Create the workspace and build the bridge

```bash
mkdir -p ~/Desktop/sim_ws/src
cd ~/Desktop/sim_ws/src
git clone https://github.com/f1tenth/f1tenth_gym_ros.git

cd ~/Desktop/sim_ws
rosdep install --from-paths src --ignore-src -r -y   # catch-all for ROS deps
colcon build
source install/local_setup.bash
```
Re-`source install/local_setup.bash` in **every** new terminal that runs sim nodes, and
**rebuild (`colcon build`) after editing any config or launch file** — launch reads from
`install/share`, not `src`.

---

## 6. Configuration — `sim.yaml`

File: `~/Desktop/sim_ws/src/f1tenth_gym_ros/config/sim.yaml`. The map parameters must be
(note: `map_path` is the **bare name, no extension**):
```yaml
    # map parameters
    map_path: '/home/<USER>/Desktop/sim_ws/src/f1tenth_gym_ros/maps/levine'
    map_img_ext: '.png'
```
The launch file builds the map_server path as `map_path + '.yaml'` → `levine.yaml`. If
`map_path` ends in `.png` (or the launch concatenates `map_img_ext`) you get a bogus
`levine.png.yaml` and the map fails to load — see troubleshooting.

**Custom map:** drop `<name>.png` + `<name>.yaml` (same basename) in `maps/`, point
`map_path` at `.../maps/<name>`, and set the start pose (`sx`, `sy`, `stheta`) to a point
**on the track** — the default `(0,0)` may be far off your map.

---

## 7. Universal VM display fix for RViz  ← every VM needs this

In **any** virtual machine — Parallels, VMware, VirtualBox, UTM, Hyper-V — RViz (a Qt +
OpenGL app) tends to **hang with no window** because of (1) the Ubuntu Wayland session
and (2) the hypervisor's limited virtual-GPU OpenGL. The fix is the same everywhere:
force Qt onto X11 and use software rendering. Add both to `~/.bashrc`:
```bash
echo 'export QT_QPA_PLATFORM=xcb' >> ~/.bashrc
echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc
source ~/.bashrc
```
- `QT_QPA_PLATFORM=xcb` → Qt uses X11 instead of hanging on Wayland negotiation.
- `LIBGL_ALWAYS_SOFTWARE=1` → renders on the CPU (llvmpipe), bypassing whatever virtual
  GPU the hypervisor exposes. For the flat 2D F1TENTH scene this is plenty fast.

This is **hypervisor-independent** because the Wayland issue is a guest-OS (Ubuntu GNOME)
behavior and software rendering sidesteps every virtual GPU equally. It is the single
most important step for getting RViz to appear in a VM.

---

## 8. Run the simulator

Each step in its **own terminal**, all with ROS sourced.

**Terminal 1 — simulator (bridge + map + RViz):**
```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/sim_ws && source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
Wait for the RViz window to open showing the map and the car.

---

## 9. Drive the car

The car spawns frozen until a drive command arrives.

### Option A — stock keyboard teleop
```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Publishes `Twist` to `/cmd_vel`. **Movement keys** (`i` forward, `,` reverse, `j`/`l`
steer, `k`/space stop) — speed keys (`q`/`z`) only change the multiplier. The terminal
must stay **focused** to capture keys. The bridge turns `/cmd_vel` steering into binary
±0.3 — fine for testing, poor for smooth racelines.

### Option B — custom teleop (smoother, recommended for logging racelines)
Publishes `AckermannDriveStamped` to `/drive` with **proportional** steering. Save as
`~/key_drive.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import sys, termios, tty, select

HELP = """
Drive:  w/s = speed +/-   a/d = steer left/right
        space = stop      q = quit
"""

class KeyDrive(Node):
    def __init__(self):
        super().__init__('key_drive')
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.speed = 0.0
        self.steer = 0.0

    def send(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steer
        self.pub.publish(msg)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    r, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if r else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rclpy.init()
    node = KeyDrive()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        while True:
            k = get_key(settings)
            if   k == 'w': node.speed += 0.5
            elif k == 's': node.speed -= 0.5
            elif k == 'a': node.steer += 0.05
            elif k == 'd': node.steer -= 0.05
            elif k == ' ': node.speed = 0.0; node.steer = 0.0
            elif k == 'q': break
            node.steer = max(-0.4, min(0.4, node.steer))   # real steering range
            node.speed = max(-2.0, min(7.0, node.speed))
            node.send()
            print(f'\rspeed={node.speed:+.2f}  steer={node.steer:+.2f}   ', end='')
    finally:
        node.speed = 0.0; node.steer = 0.0; node.send()    # stop on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run it (terminal focused):
```bash
source /opt/ros/humble/setup.bash
python3 ~/key_drive.py
```

---

## 10. Verify everything is alive

```bash
ros2 node list
# expect: /bridge  /ego_robot_state_publisher  /lifecycle_manager_localization
#         /map_server  /rviz

ros2 lifecycle get /map_server             # -> active
ros2 topic echo /ego_racecar/odom --once   # pose data flowing
ros2 topic echo /scan --once               # 1080 LiDAR ranges
```
If `/bridge` is missing it crashed in `__init__` — check Section 4 deps and Section 6
config, then run it standalone to see the traceback:
```bash
ros2 run f1tenth_gym_ros gym_bridge
```

---

## 11. Troubleshooting reference

| Symptom | Cause | Fix |
|---------|-------|-----|
| RViz hangs, no window (any hypervisor) | Wayland + virtual-GPU OpenGL in the VM | `QT_QPA_PLATFORM=xcb` + `LIBGL_ALWAYS_SOFTWARE=1` (Section 7) |
| RViz still flaky after Section 7 | Guest tools not installed / 3D accel off | Install guest tools (Section 1.4), enable 3D accel, reboot |
| `pip install` builds for ages or fails on ARM64 | No prebuilt wheel for aarch64 | Ensure `build-essential python3-dev` installed (Section 2), retry |
| `ModuleNotFoundError: transforms3d` | Missing Python dep | `pip3 install transforms3d` |
| `AttributeError: module 'coverage' has no attribute 'types'` | numba/coverage version clash | `pip3 install --upgrade coverage` (or uninstall it) |
| `Failed to load map yaml file: .../levine.png.yaml` | `map_path` includes `.png`, or launch concatenates `map_img_ext` | `map_path` = bare name; launch uses `map_path + '.yaml'` (Section 6) |
| `IsADirectoryError` on file open | Opening a folder instead of a file inside it | Open the full file path, not the folder |
| `FileExistsError` from `os.makedirs(..., exist_ok=True)` | A file already exists with the folder's name | Delete the stray file, re-run |
| RViz: "Frame [map] does not exist", RobotModel red | `/bridge` crashed → no `map` TF | Fix the bridge crash (deps/config); confirm `/bridge` in node list |
| `ros2 run` reports no executable | Missing `console_scripts` in `setup.py`, or not rebuilt | Add entry point, `colcon build`, re-source |
| Conda active during build/run | ABI/import mismatch vs system Python 3.10 | `conda deactivate`; disable auto-activate (Section 1.5) |
| Teleop keys do nothing | Terminal not focused (raw stdin reads focused window only) | Click into the teleop terminal; keep it focused |
| Car connected to `/cmd_vel` but won't move | Pressing speed keys (`q`/`z`) which command 0 velocity | Press **movement** keys (`i`/`w`); raise speed first |
| Sim sluggish / low FPS | VM under-resourced | Raise CPU cores / RAM (Section 1.3) |

---

## 12. Quick reference — full startup

```bash
# Terminal 1: sim
source /opt/ros/humble/setup.bash
cd ~/Desktop/sim_ws && source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# Terminal 2: drive
source /opt/ros/humble/setup.bash
python3 ~/key_drive.py

# Terminal 3: your nodes (logger, pure pursuit, ...)
source /opt/ros/humble/setup.bash
python3 ~/your_node.py
```
Order matters only in that **Terminal 1 must come up first** so the topics exist.

---

## Appendix — per-member setup checklist

Hand this to each teammate; everyone ends at the same working state:

- [ ] Hypervisor installed for my host (Section 1.1)
- [ ] Ubuntu 22.04 desktop VM created with correct arch (1.2) and resources (1.3)
- [ ] Guest tools installed + rebooted (1.4)
- [ ] ROS 2 Humble installed; conda disabled (1.5)
- [ ] apt deps installed (Section 2)
- [ ] `f1tenth_gym` pip-installed, import check passes (Section 3)
- [ ] `transforms3d` installed; coverage fix applied if needed (Section 4)
- [ ] Workspace built with `colcon build` (Section 5)
- [ ] `sim.yaml` map params correct (Section 6)
- [ ] Display fix in `~/.bashrc` (Section 7)
- [ ] `ros2 launch ... gym_bridge_launch.py` shows map + car in RViz (Section 8)
- [ ] `key_drive.py` drives the car (Section 9)
- [ ] `ros2 node list` shows `/bridge`; odom + scan flowing (Section 10)

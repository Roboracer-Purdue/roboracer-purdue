# Patchy Roboracer Sim Workspace

Autonomous F1TENTH racing simulator for new members. Clone it, run one setup
script, and you're driving a simulated car and building autonomy on top of it.

**Target:** Ubuntu 22.04 (in a VM is fine) • ROS 2 Humble

---

## Quick start

**Step 1 — Create your own branch (on GitHub, before cloning).**
This repo's `main` is the template only, and `testing_new_ws` is the shared
starting point everyone builds from. Create your own branch off of
`testing_new_ws` (not `main`) before you clone — e.g. via the GitHub UI's
branch dropdown, or from another machine:
```bash
git fetch origin
git checkout testing_new_ws
git checkout -b <your-branch-name>       # e.g. Meghaj_Kabra_ws
git push -u origin <your-branch-name>
```

**Step 2 — Clone the repo and check out your branch.**
```bash
git clone -b <your-branch-name> <REPO_URL> roboracer-template
cd roboracer-template
```
This clones starting from `testing_new_ws`'s contents, on your own branch —
so what you see locally matches your branch, not `main`.

**Step 3 — Commit and push only to your own branch.**
```bash
git add .
git commit -m "your message"
git push origin <your-branch-name>
```
Never push directly to `main` or `testing_new_ws` — those stay as shared
reference points. All of your work stays isolated on your own branch.

Now run setup:

```bash
chmod +x setup.sh
./setup.sh                    # installs ROS 2 Humble + everything, ~15-20 min
```

> **Note — adding lab templates (lab1, lab2, etc.) as submodules:**
> Before adding any lab template as a submodule, **fork it to your own GitHub
> account first**, then add *your fork's* URL — not the original roboracer
> template URL. A submodule is a separate repo; commits you make inside it
> push to whatever URL it points to. If you point it at the shared upstream
> template, your work (and anyone else's, if they push) lands in the shared
> repo instead of staying in your own workspace. Forking first keeps your
> work isolated and safe:
> ```bash
> git submodule add git@github.com:<your-username>/<lab-template-repo>.git <path>
> ```

Open a **new terminal** when it finishes (so the environment loads), then verify:

```bash
python3 -c "import f110_gym; print('gym OK')"
```

If that prints `gym OK`, you're set up.

---

## Run the sim

Three terminals. Each new terminal already has ROS + the workspace sourced
(setup.sh added that to your `~/.bashrc`).

```bash
# Terminal 1 — the simulator (map + car in RViz)
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# Terminal 2 — drive it manually
python3 scripts/key_drive.py          # w/s = speed, a/d = steer, space = stop

# Terminal 3 — run an algorithm (your controller goes here)
python3 scripts/<your_node>.py
```

Keep Terminal 2 focused while driving or the keys won't register.

---

## What's in here

```
roboracer-template/
├── setup.sh                 One-command install (ROS + deps + backend + build)
├── src/f1tenth_gym_ros/     The ROS 2 sim bridge (map, car, sensors)
├── scripts/                 Ready-to-use tools:
│   ├── key_drive.py           keyboard teleop
│   ├── waypoint_logger.py     record a raceline as you drive
│   └── velocity_profile.py    add speeds to a recorded raceline
├── docs/                    Full setup guide + command cheatsheet
└── f1tenth_gym/             Physics backend (created by setup.sh, not in git)
```

---

## The typical workflow

1. **Drive a lap** with `key_drive.py` while running `waypoint_logger.py` — this
   records a raceline (`x, y, yaw, speed`) to a CSV.
2. **Profile it** with `velocity_profile.py` — computes safe cornering speeds.
3. **Follow it** — write a controller (e.g. pure pursuit) that reads the CSV and
   publishes drive commands. This is the part you build.

The sim publishes `/scan` (LiDAR) and `/ego_racecar/odom` (pose) and listens on
`/drive`. Your nodes subscribe to those and publish drive commands — the same
interface a real car uses, so code you write here transfers to hardware.

---

## Common issues

| Problem | Fix |
|---------|-----|
| `./setup.sh` won't run — "permission denied" | `chmod +x setup.sh` |
| setup.sh stops on a conda/venv error | `conda deactivate` (or `deactivate`), rerun |
| RViz opens no window / hangs | Already handled by setup.sh; open a fresh terminal so the display fix loads |
| `import f110_gym` fails | Rerun `cd f1tenth_gym && pip3 install -e .` (not inside a venv) |
| Launch: map file not found | Rebuild: `colcon build && source install/local_setup.bash` |
| Keys do nothing while driving | Click the teleop terminal to focus it |

More detail in `docs/`.

---

## After editing code

Editing a `scripts/*.py` you run directly → just rerun it, no build needed.

Editing anything under `src/` (launch files, config, the bridge) → rebuild:

```bash
colcon build
source install/local_setup.bash
```

---

## Switching maps

Drop `<name>.png` + `<name>.yaml` (same name) into
`src/f1tenth_gym_ros/maps/`, set `map_path: '<name>'` in
`src/f1tenth_gym_ros/config/sim.yaml`, then `colcon build`. No file paths to edit —
the launch resolves the map by name.

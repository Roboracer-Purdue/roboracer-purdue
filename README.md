<div align="center">

# Roboracer @ Purdue

**Autonomous racing club · Purdue University in Indianapolis**

*We build 1/10-scale race cars that perceive, plan, and drive themselves — no driver, no remote control, just code on the racing line.*

[![Competition](https://img.shields.io/badge/Roboracer_IV_2026-P10_·_Detroit-CFB991?style=for-the-badge&labelColor=0B0A08)](https://roboracer.ai)
[![Stack](https://img.shields.io/badge/ROS_2-Jetson_Orin-CFB991?style=for-the-badge&labelColor=0B0A08)](#the-cars)
[![Join](https://img.shields.io/badge/Join_us-BoilerLink-CFB991?style=for-the-badge&labelColor=0B0A08)](https://boilerlink.purdue.edu/organization/arcindy)

<img src="assets/grid-iv2026.jpg" alt="The full grid at Roboracer IV 2026 in Detroit" width="100%">

</div>

## Who we are

Roboracer @ Purdue is a student-founded, student-run team that competes on the Roboracer (F1TENTH-class) autonomous racing platform. Our cars carry a complete self-driving stack — LiDAR perception, mapping and localization, trajectory planning, and low-level control — running onboard at race pace.

Members work across the whole problem: writing and tuning planners, building SLAM maps of new tracks, profiling speed through corners, designing and maintaining the vehicles themselves, and calling strategy on race day. The club is undergraduate-led, advised by Dr. Lingxi Li, and open to every major and experience level.

## Highlights

🏁 **P10 at Roboracer IV 2026 (Detroit)** — In our first season on the international stage, we qualified and raced to a top-10 finish as the **only all-undergraduate team on the grid**, competing against graduate programs from Carnegie Mellon, Penn, UIC, and more.

<div align="center">
<img src="assets/board-iv2026.jpg" alt="Board members with Car 2 at the Roboracer IV 2026 venue" width="70%">
</div>

## The cars

Our fleet of 1/10-scale vehicles shares a common platform:

| | |
|---|---|
| **Platform** | Roboracer (F1TENTH-class), 1/10 scale |
| **Compute** | NVIDIA Jetson Orin |
| **Middleware** | ROS 2 on Ubuntu |
| **Sensing** | 2D scanning LiDAR + odometry |
| **Drivetrain** | Brushless motor with VESC controller |
| **Planning** | Pure Pursuit + Frenet Corridor Planner |
| **Fallback** | Disparity-extender reactive avoidance |
| **Mapping** | SLAM-built track maps, hand-refined |

The software is our own — waypoint logging and editing tools, velocity profiling, and a planner that blends a global racing line with local Frenet-frame corridors, backed by a reactive layer for whatever the race throws at us.

## Leadership

| Name | Role |
|---|---|
| Meghaj | President · Co-founder |
| Maninder Kaur | Vice President · Public Relations · Co-founder |
| Andrew Messiha | Treasurer · Co-founder |
| Prajwal Vijay Kumar | Co-founder |
| Jeerapat "Patchy" Suanthong | Head of Autonomy |
| John Orina | Autonomy Mentor |
| Nilay Thakkar | Hardware |
| Dr. Lingxi Li | Faculty Advisor · Co-founder |

## Get involved

No experience required — just the willingness to learn fast. Whether you want to write code that races, tune a controller until it stops spinning out, or help build the next car, there's a seat for you.

- 🔧 **Join the club:** [BoilerLink — Roboracer @ Purdue](https://boilerlink.purdue.edu/organization/arcindy)
- 📸 **Follow along:** [@purdue_roboracer](https://www.instagram.com/purdue_roboracer/) on Instagram
- 💻 **Our code:** [github.com/Roboracer-Purdue](https://github.com/Roboracer-Purdue)

## Acknowledgments

Thanks to Dr. Lingxi Li for advising the team, to Purdue University in Indianapolis for supporting student motorsport at 1/10 scale, and to the Roboracer community for building the platform and the grid we race on.

<div align="center">

**Boiler up. Hammer down.** 🔨

</div>

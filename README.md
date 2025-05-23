# Development of a Control Software for Coordinating a Robotic Manipulator for Fruit Harvesting

> **Bachelor Thesis – Department of Engineering & Architecture – University of Parma**  
> Author: *Giuliano Pioldi* – Academic year 2023‑24  
> Supervisors: **Prof. Ing. Ph.D. Dario Lodi Rizzini**, **Dr. Ing. Ph.D. Cand. Alessio Saccuti**  
> Laboratory: **RIMLAB – Robotics and Intelligent Machines Laboratory**

---

## Table of Contents
1. [Project Overview](#project-overview)
2. [Hardware Platform](#hardware-platform)
3. [Software Architecture](#software-architecture)
4. [Installation](#installation)
5. [Quick Start](#quick-start)
6. [Repository Structure](#repository-structure)
7. [Images & Media](#images--media)
8. [Roadmap & Future Work](#roadmap--future-work)
9. [How to Contribute](#how-to-contribute)
10. [Acknowledgments](#acknowledgments)
11. [License](#license)

---

## Project Overview
This repository contains the **control software stack** developed for my experimental thesis *“Development of a Control Software for Coordinating a Robotic Manipulator for Fruit Harvesting.”*  
The goal is to **integrate, coordinate and control** a **UR10e** 6‑DOF manipulator mounted on a **Clearpath Husky** unmanned ground vehicle (UGV) to autonomously detect, reach, grasp and pick tomato fruits in greenhouse and open‑field scenarios.

### Key achievements
- **Unified launch pipeline** `launch/all_systems_tutti.py` that spawns *all* system components in simulation & real robots, including laboratory test‑bed geometries and the Husky mobile base.  
- **Tomato‑picking state machine** (`src/tomato_pick/tomato_pick_node.cpp`) that  
  1. Detects candidate fruits using RGB‑D perception;  
  2. Plans arm motion & approach trajectory;  
  3. Monitors **torque feedback** from UR10e internal current sensors to estimate resisting traction force;  
  4. Decides successful detachment and triggers retraction.  
- Modular ROS packages (tested on **ROS Noetic**, Ubuntu 20.04) and portable to ROS 2.  
- Extensive lab and field validation (see [Images & Media](#images--media)).

---

## Hardware Platform

| Sub‑System       | Model                                  | Function                                                       |
| ---------------- | -------------------------------------- | -------------------------------------------------------------- |
| **Manipulator**  | Universal Robots **UR10e**             | 6‑DOF arm, internal current sensors used for torque estimation |
| **End‑Effector** | Soft‑robotic gripper                   | Delicate tomato grasping                                       |
| **Mobile Base**  | **Clearpath Husky A200**               | All‑terrain UGV carrying the arm                               |
| **Perception**   | Intel **RealSense D435i** RGB‑D camera | Fruit localisation & depth sensing                             |
|                  | **Ouster‑64** 3‑D LiDAR (optional)     | Environment mapping & navigation                               |

---

## Software Architecture
```
+--------------------------------------------------------------+
|                            ROS Master                        |
+----------------------+---------------------+-----------------+
| Navigation Stack     | Perception Stack    | Manipulation    |
| (move_base / Nav2)   | (vision_msgs)       | (MoveIt! 2)     |
+----------------------+---------------------+-----------------+
|      Husky driver    | RealSense driver    | UR driver       |
+----------------------+---------------------+-----------------+
                        | torque_monitors  |
                        +------------------+
```
*Figure – High‑level ROS graph.*

- **Launch orchestration**: `launch/all_systems_tutti.py` — spawns Gazebo/MoveIt! simulation, RViz, real drivers and auxiliary nodes with one command.  
- **State machine**: implemented with **SMACC2‑style** pattern in C++ for ROS1 compatibility.  
- **Torque feedback**: raw joint currents → estimated joint torques → threshold logic to detect fruit detachment.

For a deep‑dive, consult the thesis (see `docs/thesis/` when available).

---

## Installation
```bash
# 1. Clone
mkdir -p ~/agritech_ws/src && cd ~/agritech_ws/src
git clone https://github.com/GPioldi/rimlab-agritech-manip.git

# 2. Install dependencies (ROS Noetic example)
cd ~/agritech_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
catkin_make    # or colcon build --symlink-install (ROS2)

# 4. Source workspace
source devel/setup.bash
```
> **Note:** For real‑robot execution you will also need the official **UR ROS Driver** and **Husky Base** packages from Clearpath.

---

## Quick Start
### Simulation
```bash
roslaunch launch/all_systems_tutti.py sim:=true rviz:=true
```
### Real Robot (lab)
```bash
ROS_MASTER_URI=http://husky:11311 \
roslaunch launch/all_systems_tutti.py sim:=false husky_hostname:=husky ur_ip:=192.168.131.10
```
### Tomato‑Picking Demo Only
```bash
rosrun tomato_pick tomato_pick_node
```

---

## Repository Structure
Below is the **actual file‑tree** (excluding `.svn` metadata) extracted from the uploaded archive. This layout is kept inside the repository root `rimlab-agritech-manip/`.

```text
rimlab-agritech-manip/
├── agritech_manip/                    # ↳ main ROS package
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── Istruzioni.txt                 # quick Italian notes / TODOs
│   ├── include/
│   │   └── agritech_manip/
│   │       ├── robot_dispatcher.h
│   │       ├── static_object_tracker.h
│   │       ├── tomato_pick_node.h
│   │       └── transform.h
│   ├── src/
│   │   ├── robot_dispatcher.cpp
│   │   ├── static_object_tracker.cpp
│   │   ├── test_static_object_tracker.cpp
│   │   ├── tomato_pick.cpp            # library
│   │   ├── tomato_pick_node.cpp       # executable node (state‑machine)
│   │   └── transform.cpp
│   ├── launch/
│   │   ├── all_system.launch.py       # alternative setup
│   │   ├── all_system_husky.launch.py # 🏞️ field robot stack
│   │   ├── all_system_pal3.launch.py  # 🏠 lab environment
│   │   ├── camera_only.launch.py
│   │   ├── tomato_pick_alone.launch.py
│   │   ├── tomato_pick_husky.launch.py
│   │   ├── my_realsense.launch.xml
│   │   ├── low_my_realsense.launch.xml
│   │   └── realsense2_d405_before_update_20240629.launch.py
│   └── scene_geometry/
│       └── husky_manip_mobile.scene   # Gazebo object to load mobile base + arm
│
├── docs/
│   ├── images/                        # → README figures
│   │   ├── ManipulatorConfiguration.jpeg
│   │   ├── HuskyConfiguration.jpeg
│   │   ├── RobotParts.jpeg
│   │   ├── LabExperiment_1.jpeg
│   │   ├── LabExperiment_2.jpeg
│   │   └── OnFieldExperiment.jpeg
│   └── videos/                        # (to be filled)
├── LICENSE                            # MIT
└── README.md                          # this file
```
*If you later split code into more ROS packages (e.g. `tomato_pick`), please update this list.*

---

## Images & Media
| Image | Caption |
|-------|---------|
| ![Manipulator configuration](docs/images/ManipulatorConfiguration.jpeg) | **Bench set‑up of the UR10e manipulator.** 6‑DOF arm clamped on a rigid frame inside RIMLAB for kinematic calibration and controller tuning. |
| ![Husky configuration](docs/images/HuskyConfiguration.jpeg) | **UR10e on Husky UGV.** Complete mobile manipulation platform; note the RealSense D435i on the wrist and the soft gripper. |
| ![Robot parts](docs/images/RobotParts.jpeg) | **Joint and link naming scheme.** Helpful when referencing torque or position feedback in code and during troubleshooting. |
| ![Lab experiment 1](docs/images/LabExperiment_1.jpeg) | **Lab trial – approach phase.** The arm aligns the end‑effector with a ripe tomato detected by the vision stack. |
| ![Lab experiment 2](docs/images/LabExperiment_2.jpeg) | **Lab trial – detachment detection.** Snapshot taken as the torque threshold indicates successful fruit pick. |
| ![On‑field experiment](docs/images/OnFieldExperiment.jpeg) | **First on‑field test.** Husky navigates a tomato row in an outdoor plot; the manipulator harvests under natural lighting. |

> **Video footage** will be uploaded in `docs/videos/` and embedded in the Wiki.

---

## Roadmap & Future Work
| Status | Item                             | Description                                                                                                                 |
| ------ | -------------------------------- | --------------------------------------------------------------------------------------------------------------------------- |
| 🔄 WIP | **Extensive field trials**       | Benchmark performance in real greenhouse rows under varying lighting, canopy density and fruit‑ripeness conditions.        |
| 🚀     | **Torque‑threshold auto‑tuning** | Online adaptation of detachment thresholds to minimise missed picks and stem damage.                                        |
| ⏱️     | **Cycle‑time optimisation**      | Parallelise perception & motion planning to cut pick‑and‑place time below 5 s per fruit.                                    |
| 🌐     | **SLAM + autonomous navigation** | Row‑coverage planning with LiDAR & RGB‑D fusion (Nav2 integration).                                                         |
| 🍓     | **Multi‑fruit generalisation**   | Extend perception & gripper parameters to strawberries & bell peppers.                                                      |
| 📦     | **Dataset release**              | Publish RGB‑D + joint‑torque logs under CC‑BY for community benchmarking.                                                   |
| 💬     | **Community ideas welcome!**     | **Any suggestion? Write me! 😊** Open an issue or ping me on GitHub — we love feedback!                                     |

---

## How to Contribute
1. **Fork** the repo and create your branch: `git checkout -b feature/foo_bar`  
2. **Commit** your changes: `git commit -am 'Add some foo_bar'`  
3. **Push** to the branch: `git push origin feature/foo_bar`  
4. Create a **Pull Request** with a concise description.  

Please run `catkin_lint` and `clang-format` before submitting a PR.

---

## Acknowledgments
This project has been made possible thanks to the invaluable support of **RIMLAB – Robotics and Intelligent Machines Laboratory** (University of Parma) and the mentorship of **Prof. Ing. Ph.D. Dario Lodi Rizzini** and **Dr. Ing. Ph.D. Cand. Alessio Saccuti**.  
Additional thanks go to colleagues who provided hardware assembly help, debugging assistance and fruitful discussions.

---

## License
Distributed under the **MIT License**. See [`LICENSE`](LICENSE) for more information.

---

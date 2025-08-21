# ViNT ROS 2 Simulation (Gazebo Harmonic)

Author: Abdoullah Ichou

This folder contains ROS 2 nodes and a helper script to run ViNT with Gazebo Harmonic.

## Prerequisites
- ROS 2 Jazzy Jalisco (source your ROS 2 setup)
- Gazebo Harmonic (gz sim)
- joy (ROS 2) package for joystick: `ros2 run joy joy_node`
- Python deps: torch, numpy, pillow, pyyaml (installed in your environment)
- Optional: tmux (for the helper script)

## Key files
- `navigate_ros2.py` — ViNT navigation node (subscribes to camera, publishes waypoints and goal flag)
- `pd_controller_ros2.py` — PD controller (consumes `/waypoint`, publishes `/cmd_vel`)
- `joy_teleop_ros2.py` — Joystick teleop (publishes `/cmd_vel` when deadman held)
- `navigate_ros2.sh` — tmux helper to launch Gazebo, joy, ViNT, and controller together
- `create_topomap_ros2.py` — Save camera frames to build a topomap
- `evaluate_bag_ros2.py` — Compute time-to-goal, path length, and control effort from a ros2 bag

Default topics used:
- Camera: `/camera/image_raw`
- Waypoint: `/waypoint`
- Velocity: `/cmd_vel`
- Reached goal: `/topoplan/reached_goal`

Topomap and model configs:
- Topomap images under `../topomaps/images/<dir>`
- Model configs in `../config/models.yaml`; robot limits in `../config/robot.yaml`

## Option A — Run everything with the tmux helper
```bash
cd deployment/src
chmod +x navigate_ros2.sh
./navigate_ros2.sh \
  --world /absolute/path/to/your.world \
  --model vint \
  --dir topomap \
  --goal-node -1 \
  --image-topic /camera/image_raw \
  --waypoint-topic /waypoint
```
Panes:
1) Gazebo Harmonic (`gz sim`)
2) `ros2 run joy joy_node`
3) `python3 navigate_ros2.py` (ViNT)
4) `python3 pd_controller_ros2.py` (PD controller)

## Option B — Run processes manually
- Gazebo Harmonic:
```bash
gz sim -v 4 /absolute/path/to/your.world
```
- Joy node:
```bash
ros2 run joy joy_node
```
- ViNT node:
```bash
cd deployment/src
python3 navigate_ros2.py --model vint --dir topomap --goal-node -1 \
  --image-topic /camera/image_raw --waypoint-topic /waypoint
```
- PD controller:
```bash
cd deployment/src
python3 pd_controller_ros2.py
```

## Visualization
- RViz 2:
```bash
rviz2
```
Add displays for Image (`/camera/image_raw`), Odometry (`/odom`), TF, and monitor `/cmd_vel`.

## Record and compare runs
- Record:
```bash
ros2 bag record /camera/image_raw /odom /cmd_vel /topoplan/reached_goal -o run_vint_A
# Repeat for another model/checkpoint -> run_vint_B
```
- Compare metrics (suggested): time-to-goal (from `/topoplan/reached_goal`), path length (from `/odom`), control effort (|`/cmd_vel`|). Parse both bags and write a small CSV to compare runs.

### Quick evaluator usage
After recording a bag (directory with metadata.yaml inside), run:
```bash
cd deployment/src
python3 evaluate_bag_ros2.py --bag /path/to/run_vint_A --model-tag vint_A --csv-out eval_results.csv
python3 evaluate_bag_ros2.py --bag /path/to/run_vint_B --model-tag vint_B --csv-out eval_results.csv
```
This prints metrics to the console and appends rows to `eval_results.csv`.

## Notes & Troubleshooting
- If your camera/odom are in Gazebo Transport, add `ros_gz_bridge` mappings so ROS 2 topics exist.
- If your camera topic differs, pass `--image-topic` to `navigate_ros2.py`.
- `robot.yaml` in this repo references ROS 1 cmd_vel_mux; ROS 2 nodes here fall back to publishing on `/cmd_vel`.

## What you still need that’s not in this repo
- A robot model for Gazebo Harmonic and ROS 2 (URDF/SDF) that provides:
  - A camera sensor producing a ROS 2 image topic (default: `/camera/image_raw`). If using Gazebo Transport only, use `ros_gz_bridge` to create the ROS 2 topic.
  - A mobile base controlled via `/cmd_vel` (geometry_msgs/Twist).
- A world file for Gazebo Harmonic (`.sdf`/`.world`) with an environment to navigate.
- A topomap directory: a sequence of images (0.png, 1.png, …) collected along a path in your world.

### Choosing a robot and world
- Start with a ROS 2-ready mobile robot (e.g., TurtleBot4) and its sample world, or any robot that exposes a camera and accepts `/cmd_vel`.
- Ensure the camera topic name matches what you pass to `navigate_ros2.py` (default `/camera/image_raw`).

### Creating a topomap (ROS 2)
```bash
cd deployment/src
python3 create_topomap_ros2.py --dir topomap --dt 1.0 --image-topic /camera/image_raw
```
Drive the robot through the intended route (with joystick or other teleop). Images will be saved under `../topomaps/images/topomap/` as 0.png, 1.png, …

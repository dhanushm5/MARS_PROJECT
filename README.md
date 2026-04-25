# MARS_PROJECT

## Overview
This repository contains a single automation script, `graphslam_final.sh`, that runs a complete GraphSLAM demo in simulation. The script launches an Ignition Gazebo world with a Turtlebot3 Waffle–equivalent model equipped with RGB‑D camera and 2D LiDAR, starts RTAB‑Map, RViz2, automated exploration, and captures before/after loop‑closure map snapshots.

## Key Features
- Builds a custom Gazebo world and robot model on the fly.
- Bridges sensor topics and TF between Ignition Gazebo and ROS 2.
- Runs RTAB‑Map in RGB‑D + LiDAR mode.
- Auto‑drives a repeatable exploration path and records trajectory.
- Captures **before** and **after** loop‑closure maps and generates a comparison image.
- Saves maps, trajectory plot, and RTAB‑Map database automatically.

## Requirements
You need a ROS 2 Humble environment with Ignition Gazebo and RTAB‑Map tooling installed.

**Required software**
- Ubuntu + ROS 2 Humble (`/opt/ros/humble`)
- Ignition Gazebo (`ign gazebo` CLI)
- `ros_gz_bridge`
- `rtabmap` and `rtabmap_launch`
- `rviz2`
- Python 3

**Python packages**
- `numpy`
- `matplotlib`

> The script attempts to install `numpy<2` and `matplotlib` via `pip` if missing.
**GUI**
- An X server is required for Gazebo/RViz.
- The script sets `DISPLAY=:1` and `LIBGL_ALWAYS_SOFTWARE=1` for software rendering.

## Quick Start
1. Ensure ROS 2 Humble is installed and sourced.
2. Verify Ignition Gazebo and ROS packages are available.
3. Run the script:
   ```bash
   bash graphslam_final.sh
   ```
4. Results are saved to `~/graphslam_ws/results`.

## What the Script Does
1. **Gazebo**: starts simulation with a custom indoor world.
2. **Bridge + TF**: bridges `/scan`, `/camera/*`, `/odom`, `/tf`, `/clock` and sets static transforms.
3. **RTAB‑Map**: launches RGB‑D + LiDAR GraphSLAM.
4. **RViz2**: loads a preconfigured visualization.
5. **Map Snapshot Node**: captures before/after loop‑closure maps.
6. **Auto‑Drive + Trajectory**: drives a fixed path and records trajectory.

## Output Files
All outputs are written to `~/graphslam_ws/results`:
- `map_before_loop_closure.png`
- `map_after_loop_closure.png`
- `loop_closure_comparison.png`
- `trajectory_plot.png`
- `graphslam_map.pgm` + `graphslam_map.yaml`
- `rtabmap_final.db`
- `trajectory_raw.json`

Logs are written to: `~/graphslam_ws/run.log`

## Configuration Notes
- The world and robot model are generated inside the script.
  - World: `~/graphslam_ws/worlds/world.sdf`
  - Model: `~/graphslam_ws/models/tb3_waffle_sim/`
- To customize the path, map capture timing, or sensors, edit `graphslam_final.sh`.

## Troubleshooting
- **ROS 2 not found**: ensure `/opt/ros/humble/setup.bash` exists and is sourced.
- **No GUI / black window**: verify your X server and `DISPLAY` settings.
- **Missing topics**: confirm `ros_gz_bridge`, RTAB‑Map, and Ignition Gazebo are installed and compatible.
- **Inspect logs**: check `~/graphslam_ws/run.log` for errors.

## Stopping the Demo
Use `Ctrl+C` to stop. The script also cleans up background processes on exit.

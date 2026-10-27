# DJI RoboMaster TT 2D SLAM Prototype

A ROS 2 (Humble) 2D SLAM system for the **DJI RoboMaster TT (Tello Talent) / Tello EDU** drone using monocular metric depth estimation and `slam_toolbox`.

The drone captures front monocular camera video over UDP, estimates metric depth using **Depth Anything V2**, projects the depth into a 3D point cloud, converts it into a virtual 2D laser scan, fuses onboard IMU/odometry using **`robot_localization`**, and builds real-time occupancy grid maps using **`slam_toolbox`**.

---

## Architecture Overview

```
                      +-----------------------------+
                      | DJI RoboMaster TT / Tello   |
                      | (Wi-Fi UDP 8889 / 11111)    |
                      +--------------+--------------+
                                     |
               +---------------------+---------------------+
               | H.264 Video Stream                        | Drone State / IMU
               v                                           v
    +----------------------+                    +----------------------+
    |   rmtt_driver_node   |                    |   rmtt_driver_node   |
    |  (PyAV Video Decode) |                    |  (/flight_data, IMU) |
    +----------+-----------+                    +----------+-----------+
               | /camera/image_raw                         | /tello/odom, /tello/imu
               v                                           v
    +----------------------+                    +----------------------+
    | depth_to_cloud_node  |                    |     ekf_node         |
    | (Depth Anything V2)  |                    | (robot_localization) |
    +----------+-----------+                    +----------+-----------+
               | /cloud_obstacles                          | /odometry/filtered
               v                                           | (odom -> base_stabilized)
    +----------------------------+                         |
    | pointcloud_to_laserscan    |                         |
    +--------------+-------------+                         |
                   | /scan                                 |
                   +-------------------+-------------------+
                                       |
                                       v
                         +---------------------------+
                         |       slam_toolbox        |
                         | (map -> odom transform &  |
                         |  /map occupancy grid)     |
                         +---------------------------+
```

---

## Features

- **Monocular Depth to 2D LiDAR**: Zero-shot metric depth estimation with Hugging Face's Depth Anything V2 (`Depth-Anything-V2-Metric-Indoor-Small-hf`), projected into a stabilized horizontal virtual scan.
- **Sensor Fusion**: Robot localization EKF fuses drone velocities and attitude to maintain smooth `odom -> base_stabilized` transforms.
- **Real-Time SLAM**: 2D occupancy mapping powered by `slam_toolbox` async node.
- **Dual-Mode Teleoperation**: Keyboard teleop node supporting both continuous flight mode and discrete step motion mode.
- **Bench Cooling Mode**: Allows motor-assisted cooling while running and testing SLAM on a desk.
- **Fail-Safe Emergency Stop**: Automatic emergency motor cutoff on ROS 2 launch shutdown.
- **Containerized**: Fully configured Docker environment with CUDA 12.1, PyTorch, and ROS 2 Humble.

---

## Prerequisites

- **Host OS**: Linux (Ubuntu 20.04 or 22.04 recommended)
- **GPU**: NVIDIA GPU with proprietary drivers
- **Software**: Docker and [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- **Hardware**: DJI RoboMaster TT or Tello EDU drone

---

## Quick Start

### 1. Host Network & Latency Setup

Connect your host machine's Wi-Fi to the drone's access point (e.g., `RMTT-XXXXXX` or `TELLO-XXXXXX`), then run the host preparation script:

```bash
chmod +x host_setup.sh
sudo ./host_setup.sh
```

This script:
- Disables Wi-Fi power-saving mode on the connected wireless interface.
- Prevents Bluetooth coexistence interference.
- Configures X11 passthrough permissions for Docker GUI applications (RViz2).

---

### 2. Build and Start the Docker Container

Build the Docker image and launch the container with GPU and host networking:

```bash
chmod +x docker/run_container.sh
./docker/run_container.sh
```

---

### 3. Build the ROS 2 Workspace (Inside Container)

Once inside the container:

```bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

### 4. Launch the SLAM Stack

Run the unified bringup launch file:

```bash
# Standard flight mode
ros2 launch rmtt_slam_bringup prototype_slam.launch.py

# Or bench testing mode (runs motors at low idle for cooling without takeoff)
ros2 launch rmtt_slam_bringup prototype_slam.launch.py bench_cooling_mode:=true
```

---

### 5. Teleoperation

Open a second terminal into the running container:

```bash
docker exec -it rmtt_proto bash
source /root/ros2_ws/install/setup.bash
ros2 run rmtt_slam_bringup rmtt_teleop_node
```

#### Keyboard Controls

| Key | Action |
| :--- | :--- |
| **`T`** | Takeoff |
| **`G`** | Land |
| **`SPACE`** | Emergency motor stop (immediate cutoff) |
| **`W` / `S`** | Forward / Backward (+X / -X) |
| **`A` / `D`** | Left / Right (+Y / -Y) |
| **`I` / `K`** | Altitude Up / Down (+Z / -Z) |
| **`J` / `L`** | Rotate Left / Right (Yaw) |
| **`C`** | Switch to Continuous flight mode |
| **`V`** | Switch to Step flight mode |
| **`1` – `5`** | Speed / Step size presets (20% – 100%) |
| **`X`** | Hover / Stop motion |
| **`M` / `N`** | Motor On (cooling) / Motor Off |

---

## Visualizing in RViz2

From within the container (or on the host with ROS 2 Humble installed):

```bash
rviz2
```

Configure:
- **Fixed Frame**: `map`
- **Map**: Topic `/map`
- **LaserScan**: Topic `/scan`
- **PointCloud2**: Topic `/cloud_obstacles`
- **Image**: Topic `/camera/image_raw`

---

## Repository Structure

```
.
├── docker/
│   ├── Dockerfile             # Ubuntu 22.04 + CUDA 12.1 + ROS 2 Humble + PyTorch
│   └── run_container.sh       # Container launch wrapper
├── host_setup.sh              # Wi-Fi latency & X11 host configuration
├── ros2_ws/
│   └── src/
│       └── rmtt_slam_bringup/
│           ├── config/
│           │   ├── ekf.yaml                   # robot_localization filter config
│           │   └── slam_toolbox_params.yaml   # slam_toolbox mapping config
│           ├── launch/
│           │   └── prototype_slam.launch.py   # Unified system launch file
│           ├── rmtt_slam_bringup/
│           │   ├── rmtt_driver_node.py        # Tello SDK & video receiver
│           │   ├── depth_to_cloud_node.py     # Depth Anything V2 inference & cloud publisher
│           │   └── rmtt_teleop_node.py        # Dual-mode keyboard controller
│           ├── package.xml
│           ├── setup.cfg
│           └── setup.py
├── .gitignore
├── LICENSE
└── README.md
```

---

## License

This project is licensed under the [Apache License 2.0](LICENSE).

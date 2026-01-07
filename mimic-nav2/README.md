# Mimic Nav2 Navigation Package

**Autonomous navigation for Mimic mecanum robot using Nav2 and ZED2 camera**

## Overview

This package provides autonomous navigation capabilities for the Mimic mecanum robot using:
- **Nav2** stack for path planning and control
- **ZED2 camera** for visual-inertial odometry and obstacle detection
- **DWB controller** with holonomic/mecanum support
- **Depth-to-laserscan** conversion for 2D costmap generation

## Package Structure

```
mimic-nav2/
├── config/          # Nav2 parameter files
├── launch/          # Launch files
├── urdf/            # Robot description with sensors
├── rviz/            # RViz configurations
├── maps/            # Pre-made maps (for map-based navigation)
├── scripts/         # Utility scripts
└── README.md
```

## Prerequisites

- ROS2 Humble
- Nav2 stack installed
- ZED SDK and ROS2 wrapper
- mimic-teleop package (robot driver)

## Installation

Run the installation script:
```bash
cd /home/projects/projects/mimic-teleop
./install_nav2_jetson.sh
```

Or install manually:
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## Quick Start

### 1. Launch Robot with Navigation (All-in-One)
```bash
ros2 launch mimic-nav2 bringup.launch.py
```

This starts:
- Robot driver (twist_to_serial)
- ZED camera with positional tracking
- Depth-to-laserscan conversion
- Nav2 stack
- RViz with Nav2 panel

### 2. Set Navigation Goal in RViz
- Click "2D Goal Pose" button
- Click on map and drag to set goal orientation
- Robot will autonomously navigate to the goal

## Launch Files

### bringup.launch.py (Recommended)
Full system launch - robot + localization + navigation
```bash
ros2 launch mimic-nav2 bringup.launch.py
```

### localization.launch.py
ZED camera and sensor processing only
```bash
ros2 launch mimic-nav2 localization.launch.py
```

### navigation.launch.py
Nav2 stack only (requires localization running separately)
```bash
ros2 launch mimic-nav2 navigation.launch.py
```

### robot_bringup.launch.py
Robot driver only (ESP32 communication)
```bash
ros2 launch mimic-nav2 robot_bringup.launch.py
```

## Configuration

### Key Parameters

**Nav2 Controller (config/nav2_params.yaml)**
- `controller_server.FollowPath.plugin`: DWB local planner
- `controller_server.FollowPath.holonomic`: **true** (mecanum!)
- Max velocities tuned for ESP32 limits

**ZED Camera (config/zed.yaml)**
- Positional tracking enabled
- Publishes `odom → base_link` transform
- Depth range configured for indoor navigation

**Costmaps**
- Global: 50m x 50m (adjustable)
- Local: 5m x 5m rolling window
- Inflation radius: 0.3m

## TF Tree Structure

```
map (optional, for map-based nav)
 └── odom (from ZED positional tracking)
      └── base_link
           ├── camera_link (ZED2 mount)
           ├── laser_frame (virtual lidar from depth)
           ├── front_left_wheel
           ├── front_right_wheel
           ├── rear_left_wheel
           └── rear_right_wheel
```

## Topics

### Subscribed
- `/cmd_vel` - Velocity commands from Nav2 → robot

### Published
- `/scan` - 2D laser scan (from ZED depth)
- `/odom` - Odometry (wheel encoders + ZED)
- `/zed2/zed_node/odom` - ZED visual odometry
- `/global_costmap/costmap` - Global costmap
- `/local_costmap/costmap` - Local costmap

## Future: Distributed Setup (Dev Machine Offload)

For computationally intensive Nav2 processing, you can run:
- **Jetson**: Robot driver + ZED + sensors
- **Dev Machine**: Nav2 stack + RViz

Setup:
1. Configure ROS_DOMAIN_ID (same on both machines)
2. Jetson: `ros2 launch mimic-nav2 robot_bringup.launch.py`
3. Dev Machine: `ros2 launch mimic-nav2 navigation_remote.launch.py`

See `install_nav2_dev.sh` for dev machine setup.

## Troubleshooting

### Robot doesn't move
- Check ESP32 serial connection: `ls /dev/ttyUSB*`
- Verify twist_to_serial node: `ros2 node list`
- Check cmd_vel messages: `ros2 topic echo /cmd_vel`

### ZED tracking fails
- Ensure good lighting
- Check ZED is publishing: `ros2 topic list | grep zed`
- Verify TF tree: `ros2 run tf2_tools view_frames`

### Nav2 costmap errors
- Check scan data: `ros2 topic echo /scan`
- Verify depth-to-laserscan: `ros2 node info /zed_depth_to_laserscan`

### TF transform errors
- Verify only ONE source publishes `odom → base_link`
- Check: `ros2 run tf2_ros tf2_echo odom base_link`

## Development

### Building
```bash
cd /home/projects/projects/mimic-teleop
colcon build --packages-select mimic-nav2 --symlink-install
source install/setup.bash
```

### Testing Nav2 without robot
Use Nav2 simulation or bag files:
```bash
ros2 bag play <your_sensor_data>.bag
ros2 launch mimic-nav2 navigation.launch.py
```

## Credits

- Based on Articulated Robotics Nav2 tutorial
- DWB controller configuration adapted for mecanum drive
- ZED integration for mapless navigation

## License

Apache-2.0

---

**The Mimic Robotics** | January 2026

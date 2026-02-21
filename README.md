# Cleaning Robot Workspace

## Overview
This project simulates and controls a cleaning robot in a **ROS2 Humble** environment. It is designed to demonstrate autonomous mapping, navigation, and exploration using industry-standard tools.

### Key Features
- **Autonomous Navigation**: Uses **Nav2 (Navigation 2)** for reliable path planning and obstacle avoidance.
- **SLAM**: Generates 2D occupancy maps using **SLAM Toolbox**.
- **Auto-Exploration**: Uses `m-explore-ros2` to autonomously detect frontiers and explore unknown environments.
- **Web Interface**: A responsive web-based commander to mark custom waypoints (e.g., "Kitchen", "Table") and send the robot to them.
- **Ignition Gazebo Simulation**: Realistic physics simulation with a custom robot model requiring `ros_gz`.

---

## 1. Prerequisites & Dependencies

### System Requirements
- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS Distribution**: ROS2 Humble Hawksbill

### Required Dependencies
Install the primary ROS2 packages:
```bash
sudo apt update
sudo apt install -y \
    ros-humble-desktop-full \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-rosbridge-server \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-teleop-twist-keyboard \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge
```

### Optional Dependencies
If you plan to use a joystick or priority multiplexing:
```bash
sudo apt install -y ros-humble-twist-mux ros-humble-joy ros-humble-teleop-twist-joy
```

---

## 2. Installation and Build

1.  **Create a Workspace**:
    ```bash
    mkdir -p ~/rover_ws/src
    cd ~/rover_ws/src
    ```

2.  **Clone the Repository**:
    Paste the contents of this folder into `src` (or git clone if applicable).

3.  **Install Dependencies (rosdep)**:
    Navigate to the workspace root and install any missing dependencies:
    ```bash
    cd ~/rover_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

4.  **Build the Workspace**:
    ```bash
    colcon build --symlink-install
    ```
    *Note: `--symlink-install` allows you to edit Python scripts and launch files without rebuilding.*

5.  **Source the Workspace**:
    ```bash
    source install/setup.bash
    ```
    *(Recommended: Add this line to your `~/.bashrc`)*

---

## 3. Running the Simulation

All commands assume you have sourced the workspace: `source ~/rover_ws/install/setup.bash`

### A. View Robot Model (URDF Check)
To visualize just the robot description in RViz without physics:
```bash
ros2 launch cleaning_robot_description view_bot.launch.py
```

### B. Autonomous Exploration (SLAM + Auto-Explore)
This is the main demo. It launches Gazebo, Nav2, SLAM Toolbox, and the Exploration nodes. The robot will start mapping the "Cafe" world automatically.

```bash
ros2 launch cleaning_robot_bringup full_system.launch.py slam:=true explore:=true world_file:=cafe.world
```

### C. Manual Mapping (SLAM Only)
Use this if you want to drive the robot yourself to build the map.

1.  **Launch System**:
    ```bash
    ros2 launch cleaning_robot_bringup full_system.launch.py slam:=true explore:=false world_file:=cafe.world
    ```

2.  **Drive Robot** (in a new terminal):
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
    *Controls: `i` (forward), `,` (back), `j` (left), `l` (right), `k` (stop)*

---

## 4. Saving the Map
Once you are satisfied with the generated map:

1.  **Save the Map**:
    ```bash
    ros2 run nav2_map_server map_saver_cli -f ~/rover_ws/src/cleaning_robot_navigation/maps/my_map
    ```
    *This creates `my_map.yaml` and `my_map.pgm`.*

---

## 5. Navigation Mode
Use this mode to navigate a pre-saved map.

1.  **Stop Previous Runs** (Ctrl+C).
2.  **Launch Navigation**:
    ```bash
    ros2 launch cleaning_robot_bringup full_system.launch.py slam:=false localization:=true map:=~/rover_ws/src/cleaning_robot_navigation/maps/cafe_map.yaml
    ```
    *(Ensure the map path is correct)*

3.  **Send Goals**:
    - **RViz**: Use the "Nav2 Goal" button at the top.
    - **Terminal**:
      ```bash
      ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: -0.5, z: 0.0}, orientation: {w: 1.0}}}}"
      ```

---

## 6. Web Commander Interface
The project includes a custom Web UI to control the robot and manage waypoints.

1.  **Access the UI**:
    Open your browser (on the same machine) and go to:
    [http://localhost:5000](http://localhost:5000)

2.  **Features**:
    - **Live Map**: Shows the robot's position and the costmap.
    - **Add Waypoints**: Click anywhere on the map to name and save a location (e.g., "Kitchen").
    - **Smart Snapping**: If you click on an obstacle, the system automatically finds the nearest safe point.
    - **Navigate**: Click "Go" next to any saved waypoint to send the robot there.

---

## 7. Troubleshooting

- **Robot model missing in Gazebo?**
  Ensure you installed `ros-humble-ros-gz` and `ros-humble-ros-gz-bridge`.
  
- **"Command not found" error?**
  Make sure you ran `source install/setup.bash` in *every* new terminal.

- **Web UI not connecting?**
  Ensure `rosbridge_server` is running (it starts automatically with `full_system.launch.py`). Check the browser console (F12) for errors.

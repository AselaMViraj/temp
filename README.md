# 🧹 Cleaning Robot – Real Hardware SLAM & Navigation Guide

This guide explains how to:

* Build the ROS2 workspace
* Perform SLAM (manual exploration)
* Perform SLAM (auto exploration)
* Save the generated map
* Navigate using a saved map

It is written for users who may be new to ROS2.

---

# 📦 System Requirements

* Ubuntu 22.04
* ROS2 (Humble or compatible version)
* Real robot hardware connected and powered
* Workspace: `~/Desktop/rover_ws`

---

# 🚀 1. Build the Workspace

This must be done **before running the system**.

## Step 1 — Open Terminal

```bash
cd ~/Desktop/rover_ws
```

## Step 2 — Build the Workspace

```bash
colcon build
```

Wait until the build completes successfully.

## Step 3 — Source the Workspace

⚠️ This must be done in **every new terminal**.

```bash
source install/setup.bash
```

---

# 🗺 2. SLAM – Manual Exploration Mode

Use this mode to manually drive the robot and create a map.

---

## Terminal 1 — Start the Real System

```bash
cd ~/Desktop/rover_ws
source install/setup.bash

ros2 launch cleaning_robot_bringup real_system.launch.py slam:=true explore:=false
```

Do NOT close this terminal while mapping.

---

## Terminal 2 — Control the Robot

Open a **new terminal**:

```bash
cd ~/Desktop/rover_ws
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Keyboard Controls

| Key | Action        |
| --- | ------------- |
| i   | Move Forward  |
| ,   | Move Backward |
| j   | Turn Left     |
| l   | Turn Right    |

Drive the robot slowly and cover the entire environment.

---

## Stop Manual Exploration

1. Stop keyboard control (Terminal 2):

```
Ctrl + C
```

2. Keep Terminal 1 running until the map is saved.

---

# 💾 3. Save the Map

Open a **new terminal**:

```bash
cd ~/Desktop/rover_ws
source install/setup.bash

ros2 run nav2_map_server map_saver_cli -f ~/Desktop/rover_ws/src/cleaning_robot_navigation/maps/my_map
```

This will generate:

```
my_map.pgm
my_map.yaml
```

After saving the map:

* Go back to Terminal 1
* Press `Ctrl + C` to stop SLAM

---

# 🤖 4. SLAM – Auto Exploration Mode

If you want the robot to explore automatically:

```bash
cd ~/Desktop/rover_ws
source install/setup.bash

ros2 launch cleaning_robot_bringup real_system.launch.py slam:=true explore:=true
```

After exploration finishes:

1. Open a new terminal
2. Save the map using the same map saver command
3. Stop the running SLAM system with:

```
Ctrl + C
```

---

# 🧭 5. Static Navigation Mode (Using Saved Map)

After saving your map:

```bash
cd ~/Desktop/rover_ws
source install/setup.bash

ros2 launch cleaning_robot_bringup real_system.launch.py slam:=false localization:=true map:=/home/idea8/Desktop/rover_ws/src/cleaning_robot_navigation/maps/my_map.yaml
```

This will:

* Load the saved map
* Start localization
* Enable navigation

You can now send navigation goals via RViz or your navigation interface.

---

# ⚠️ Important Notes

* Always run `source install/setup.bash` in every new terminal
* Never close the main launch terminal while the robot is running
* Always save the map before stopping SLAM
* Use `Ctrl + C` to safely stop ROS2 processes
* Run `colcon build` after making code changes

---

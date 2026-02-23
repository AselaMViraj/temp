# 🧹 Cleaning Robot

# Real Hardware SLAM & Navigation Guide

### (Jetson Orin Nano via SSH + Remote Visualization)

This guide explains how to operate the real robot where:

* 🧠 ROS2 runs on **Jetson Orin Nano**
* 💻 You connect using **SSH from your laptop**
* 🗺 SLAM + Navigation run on Jetson
* 🖥 RViz can run on your laptop
* 🌐 Web interface can mark navigation points

Workspace location on Jetson:

```
~/cleaning_bot/rover_ws_2/temp
```

---

# 🏗 System Architecture

| Device           | Role                    |
| ---------------- | ----------------------- |
| Jetson Orin Nano | Runs ROS2 + Nav2 + SLAM |
| Laptop           | SSH control + RViz      |
| Robot            | Sensors, Motors, LiDAR  |

---

# 🔐 1. Connect to Jetson (From Laptop)

## Find Jetson IP (On Jetson)

```bash
ip a
```

Look for something like:

```
192.168.X.XXX
```

## Connect from Laptop

```bash
ssh idea8@<JETSON_IP>
```

Example:

```bash
ssh idea8@192.168.1.45
```

All ROS commands must be executed inside SSH session.

---

# 🏗 2. Build Workspace (On Jetson via SSH)

```bash
cd ~/cleaning_bot/rover_ws_2/temp
colcon build
source install/setup.bash
```

⚠ Always run `source install/setup.bash` in every new SSH terminal.

---

# 🗺 3. SLAM – Manual Exploration

## SSH Terminal 1 – Start SLAM

```bash
ssh idea8@<JETSON_IP>

cd ~/cleaning_bot/rover_ws_2/temp
source install/setup.bash

ros2 launch cleaning_robot_bringup real_system.launch.py slam:=true explore:=false
```

Do NOT close this terminal.

---

## SSH Terminal 2 – Teleop Control

```bash
ssh idea8@<JETSON_IP>

cd ~/cleaning_bot/rover_ws_2/temp
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Controls

| Key | Action   |
| --- | -------- |
| i   | Forward  |
| ,   | Backward |
| j   | Left     |
| l   | Right    |

Explore the full area slowly.

---

## Stop Teleop

```
Ctrl + C
```

Keep SLAM running.

---

# 💾 4. Save the Map

Open new SSH terminal:

```bash
ssh idea8@<JETSON_IP>

cd ~/cleaning_bot/rover_ws_2/temp
source install/setup.bash

ros2 run nav2_map_server map_saver_cli -f ~/cleaning_bot/rover_ws_2/temp/cleaning_robot_navigation/maps/my_map
```

After saving:

Go to SLAM terminal → press:

```
Ctrl + C
```

---

# 🤖 5. SLAM – Auto Exploration

```bash
ssh idea8@<JETSON_IP>

cd ~/cleaning_bot/rover_ws_2/temp
source install/setup.bash

ros2 launch cleaning_robot_bringup real_system.launch.py slam:=true explore:=true
```

After exploration:

1. Save map
2. Stop SLAM with `Ctrl + C`

---

# 🧭 6. Static Navigation Mode (Using Saved Map)

```bash
ssh idea8@<JETSON_IP>

cd ~/cleaning_bot/rover_ws_2/temp
source install/setup.bash

ros2 launch cleaning_robot_bringup real_system.launch.py \
slam:=false localization:=true \
map:=/home/idea8/cleaning_bot/rover_ws_2/temp/cleaning_robot_navigation/maps/my_map.yaml
```

Now navigation stack is active.

---

# 🎯 7. Command the Robot

You have two ways:

---

## Option 1 – Using RViz (Recommended)

1. Open RViz
2. Click **"Nav2 Goal"**
3. Click a location on the map

Robot will move to that location.

---

## Option 2 – Send Goal from Terminal

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: -0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

You can change:

* `x`
* `y`

To move robot to any coordinate.

---

# 🌐 8. Mark Places Using Web Interface

The system includes a browser-based place marking interface.

## Open in Browser

Same device:

```
http://localhost:5000
```

From another device:

```
http://<JETSON_IP>:5000
```

---

## Features

### 🗺 Tap on Map

* Adds a new place (e.g., "Kitchen", "Table")

### 📌 Saved Places

* Shows all saved locations
* Click **Go** → Robot navigates automatically

---

# 🖥 9. Visualize on Laptop (Remote RViz)

The workspace already includes:

```
cleaning_bot_description/view_remote.launch.py
```

---

## Run on Laptop

Make sure:

* Laptop and Jetson are on same network
* Same ROS2 version
* Same `ROS_DOMAIN_ID`

On laptop:

```bash
cd ~/cleaning_bot/rover_ws_2/temp
source install/setup.bash

ros2 launch cleaning_bot_description view_remote.launch.py
```
(change the path according to your directory)
---

## Add Topics in RViz

Click **Add** → Add necessary topics:

Recommended:

* Map
* LaserScan
* TF
* RobotModel
* Path
* Costmap
* Pose

---

# ⚠️ Important Rules

* Always SSH before running ROS commands
* Always source workspace in every terminal
* Save map BEFORE stopping SLAM
* Never close main launch terminal while robot is running
* Use `Ctrl + C` for safe shutdown

---

# 🧠 Complete Workflow Summary

1. SSH into Jetson
2. Build workspace
3. Launch SLAM
4. Explore area
5. Save map
6. Stop SLAM
7. Launch static navigation
8. Send goals (RViz / terminal / web UI)

---


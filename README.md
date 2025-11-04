# DAT160 Search & Rescue Robot Competition

Quick reference guide for running the competition simulation.

**Competition:** 
https://frdedynamics.github.io/hvl_robotics_website/courses/dat160/competition  

**Team Repo:** 
https://github.com/668807/multi_robot_challenge_25

## 🚀 Quick Start (Daily Use)

### 1. Launch Simulation
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch multi_robot_challenge_23 rescue_robots_w1.launch.py
```
**Wait 20 seconds for robots to spawn.**

### 2. Launch Scoring System
```bash
source ~/ros2_ws/install/setup.bash
ros2 run scoring scoring
```
**Wait for "Initial Positions received" message.**

### 3. View Cameras (Optional)
```bash
# Robot 0 camera
ros2 run rqt_image_view rqt_image_view
# Select: /tb3_0/camera/image_raw

# Robot 1 camera (new terminal)
ros2 run rqt_image_view rqt_image_view  
# Select: /tb3_1/camera/image_raw
```

### 4. Manual Control (Testing)
```bash
# Control tb3_0 with WASD
ros2 run turtlebot3_teleop teleop_keyboard --ros-args --remap /cmd_vel:=/tb3_0/cmd_vel

# Control tb3_1 (new terminal)
ros2 run turtlebot3_teleop teleop_keyboard --ros-args --remap /cmd_vel:=/tb3_1/cmd_vel
```
**Controls:** W (forward), S (backward), A (left), D (right), X (stop)

---

## 📋 One-Time Setup

### Copy Required Packages
```bash
cd ~/ros2_ws/src

# Copy from official repo (ros2_students_25-master)
cp -r ~/Downloads/ros2_students_25-master/multi_robot_challenge_23 .
cp -r ~/Downloads/ros2_students_25-master/multi_robot_scoring .
cp -r ~/Downloads/ros2_students_25-master/robot_teleop .
cp -r ~/Downloads/ros2_students_25-master/robot_vision .
cp -r ~/Downloads/ros2_students_25-master/ros2_aruco .

# Clone team repo
git clone https://github.com/668807/DAT160_S-R_Project.git
```

### Fix World Files (CRITICAL for Scoring)
```bash
cd ~/ros2_ws/src/multi_robot_challenge_23/worlds
cp ~/ros2_ws/src/multi_robot_scoring/worlds/*.world .
```

### Build Workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 🎯 Key Topics

**IMPORTANT:** Use these topics (map coordinates):
- `/tb3_X/marker_id` - Marker ID (0-4)
- `/tb3_X/marker_map_pose` - Position in map frame ✅

**DO NOT USE** (camera coordinates):
- `/tb3_X/aruco_markers` ❌
- `/tb3_X/aruco_poses` ❌

**Other useful topics:**
- `/tb3_X/cmd_vel` - Control robot
- `/tb3_X/scan` - LiDAR data
- `/tb3_X/odom` - Odometry
- `/tb3_X/camera/image_raw` - Camera

---

## 🏆 Scoring System

### Point Values
- Marker 0, 1, 3 (small fires): **100 pt each**
- Marker 2 (human): **100 pt**
- Marker 4 (big fire): **100 pt** + **300 pt bonus** (when both robots meet there)
- **Time penalty:** Start at 600, lose 1 pt/second

### Report a Marker
```bash
ros2 service call /set_marker_position scoring_interfaces/srv/SetMarkerPosition \
  "{marker_id: 4, marker_position: {x: 4.8, y: -0.9, z: 0.25}}"
```

Get position from `/tb3_X/marker_map_pose` when marker is detected.

---

## 🔧 Common Commands

```bash
# List topics
ros2 topic list

# Echo a topic
ros2 topic echo /tb3_0/marker_id

# List services
ros2 service list

# List nodes
ros2 node list

# Rebuild after code changes
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## ⚠️ Troubleshooting

**Problem:** Scoring says "service not available"  
**Fix:** World files missing plugin. See setup section.

**Problem:** Missing `/tb3_0/marker_id` topics  
**Fix:** Restart simulation, wait 30 seconds for full init.

**Problem:** `ros2 topic list` shows nothing  
**Fix:** Run `source ~/ros2_ws/install/setup.bash` in that terminal.

**Problem:** Camera blank in rqt  
**Fix:** Use `ros2 run rqt_image_view rqt_image_view` instead.

---

## 📁 Project Structure

```
~/ros2_ws/src/
├── DAT160_SnR_Project/          # Your custom code
│   ├── sr_perception/
│   ├── sr_navigation/
│   ├── sr_coordination/
│   └── sr_bringup/
├── multi_robot_challenge_23/    # Competition worlds
├── multi_robot_scoring/         # Scoring system
└── ros2_aruco/                  # ArUco detection
```

---

## 🌍 Different Worlds

```bash
ros2 launch multi_robot_challenge_23 rescue_robots_w1.launch.py  # World 1
ros2 launch multi_robot_challenge_23 rescue_robots_w2.launch.py  # World 2
ros2 launch multi_robot_challenge_23 rescue_robots_w3.launch.py  # World 3
ros2 launch multi_robot_challenge_23 rescue_robots_w4.launch.py  # World 4
ros2 launch multi_robot_challenge_23 rescue_robots_w5.launch.py  # World 5
```

---

## Team Members

- [Add names here]

## Git commands
```bash
cd ~/ros2_ws/src/DAT160_SnR_Project

# Check what files changed
git status

# Add all changed files
git add .

# Commit with a message
git commit -m "Comment"

# Pull latest changes from team first (avoid conflicts)
git pull

# Push your changes
git push
```

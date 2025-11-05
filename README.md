# DAT160 Search & Rescue Robot Competition

Quick reference guide for running the competition simulation.

**Competition:** 
https://frdedynamics.github.io/hvl_robotics_website/courses/dat160/competition  

**Team Repo:** 
https://github.com/668807/multi_robot_challenge_25

## Quick Start

### 1. Launch Gazebo and map
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch multi_robot_challenge_23 rescue_robots_w1.launch.py
```
**Wait 20 seconds for robots to spawn.**

### 2. NEW terminal; launch marker recognition node
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch multi_robot_challenge_23 aruco_recognition.launch.py namespace:=tb3_0
```

### 3. NEW terminal; launch scoring node
```bash
ros2 run scoring scoring
```

### 4. NEW terminal; launch marker detection node
```bash
ros2 run multi_robot_challenge_23 marker_detection --ros-args -p namespace:=tb3_0
```

### 5. View Cameras (Optional)
```bash
# Robot 0 camera
ros2 run rqt_image_view rqt_image_view
# Select: /tb3_0/camera/image_raw

# Robot 1 camera (new terminal)
ros2 run rqt_image_view rqt_image_view  
# Select: /tb3_1/camera/image_raw
```

### 6. Manual Control (Testing)
```bash
# Control tb3_0 with WASD
ros2 run turtlebot3_teleop teleop_keyboard --ros-args --remap /cmd_vel:=/tb3_0/cmd_vel

# Control tb3_1 (new terminal)
ros2 run turtlebot3_teleop teleop_keyboard --ros-args --remap /cmd_vel:=/tb3_1/cmd_vel
```
**Controls:** W (forward), S (backward), A (left), D (right), X (stop)

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

## Team Members

- [Add names here]

## Git commands
```bash
cd ~/ros2_ws/src/multi_robot_challenge_25

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

## Bash kommandoer
```bash
ros2 launch multi_robot_challenge_23 rescue_robots_w1.launch.py

ros2 launch multi_robot_challenge_23 aruco_recognition.launch.py namespace:=tb3_0

ros2 run scoring scoring

ros2 run multi_robot_challenge_23 marker_detection --ros-args -p namespace:=tb3_0

ros2 run multi_robot_challenge_23 wall_follower --ros-args -r __ns:=/tb3_0

ros2 service call /tb3_0/wall_follower_enable std_srvs/srv/SetBool "{data: true}"
```



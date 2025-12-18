# lane_follow
A ROS2 package that allows a robot to follow a lane by using a camera to detect the lane and a proportional control node to adjust the robot's speed and steering angle.

## Dependencies
- ROS2 Jazzy
- opencv-python
- turtlebot3_gazebo
- ros_gz_sim
- cv_bridge

Make sure you have all dependencies installed and that the ros2 environment is sourced before continuing.
## Installation
1. **Clone this repository in your ROS2 workspace:** 
```bash
git clone https://github.umn.edu/hassa844/csci-4551-project.git
```
2. **Build the package:**
```bash
cd ~/csci-4551-project
colcon build
source install/setup.bash
```
## Running the simulation
Launch all components using the launch file:
```bash
ros2 launch lane_following_turtlebot3 lane_following.launch.py
```

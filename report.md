# Project Report

## Introduction/Motivation

Autonomous navigation is a core challenge in robotics with real-world applications in self-driving cars, delivery robots, and warehouse automation. This project focuses on building a lane-following system for a simulated TurtleBot3 robot. The goal is to show how a robot can use its camera sensor to see the environment and convert that visual data into movement commands.

The main motivation behind this project is to apply the "Sense-Plan-Act" cycle that is central to autonomous robotics. Lane following is a good starting point because it covers the basics of robotics in a controlled setting. The robot needs to sense the lane using its camera, figure out where it is relative to the lane center, and then adjust its steering to stay on track.

We decided to use the Gazebo simulator along with ROS2 for this project. Working in simulation allows us to test our code safely without the risk of damaging actual hardware. It also makes it easier to reset the environment and tweak settings during development. We chose the TurtleBot3 (waffle_pi model) because it is commonly used in robotics education and comes with a built-in camera sensor.

This project ties directly into several topics covered in our CSCI 4551 Robotics course:

- **OpenCV and Image Filtering**: We use OpenCV to process camera images. This includes converting color spaces (BGR to HSV), applying threshold filters to isolate lane markings, and using morphological operations to clean up noise in the image.

- **Sensors and Locomotion**: The robot relies on its camera sensor to perceive the lane. Based on that sensor input, the system sends velocity commands to control the robot's movement and steering.

- **Motion Planning**: While this project uses a simple proportional controller rather than full path planning, it demonstrates the basic idea of converting sensor feedback into planned motion commands.

- **State Estimation**: The vision system estimates the robot's position relative to the lane center by calculating a normalized error value. This error serves as a simple form of state estimation that the controller uses to make decisions.

- **ROS2**: The project is built using ROS2's publish-subscribe architecture. We have separate nodes for vision processing and motor control that communicate through ROS2 topics.

By working on this project, we get practical experience connecting perception to action, which is the foundation for more advanced topics like SLAM and full autonomous navigation.

## Methodology

### System Architecture

The project uses a two-node ROS2 architecture where each node handles a specific task. The first node (vision node) processes camera images to detect the lane, and the second node (controller node) uses that information to steer the robot. These nodes communicate through a ROS2 topic called `/lane_error`.

![System Architecture Diagram - Vision Node publishes lane error, Controller Node subscribes and sends velocity commands]

### Vision Node

The vision node is responsible for detecting the lane and calculating how far off-center the robot is. Here is how the image processing pipeline works:

1. **Image Acquisition**: The node subscribes to the `/camera/image_raw` topic to receive live camera feed from the TurtleBot3's onboard camera. We use the `cv_bridge` library to convert ROS Image messages into OpenCV format.

2. **Region of Interest (ROI)**: Instead of processing the entire image, we only look at the bottom 40% of the frame. This is where the lane markings are most visible and relevant for steering decisions.

3. **Color Space Conversion**: The image is converted from BGR to HSV color space. HSV makes it easier to filter colors because it separates color information (hue) from brightness (value).

4. **Threshold Filtering**: We apply a threshold to isolate white lane markings. The HSV range used is:
   - Hue: 0 to 180 (any color)
   - Saturation: 0 to 50 (low saturation for white)
   - Value: 200 to 255 (high brightness)

5. **Morphological Operations**: A morphological "opening" operation is applied using a 5x5 kernel. This removes small noise pixels from the binary mask while preserving the lane shape.

6. **Centroid Calculation**: Using OpenCV's `moments()` function, we find the center of the detected white pixels. This gives us the x-coordinate of where the lane appears in the image.

7. **Error Calculation**: The lane error is calculated as a normalized value:
   ```
   error = (lane_center_x - image_center_x) / image_center_x
   ```
   This gives a value between -1 and 1, where 0 means the robot is centered, negative means the lane is to the left, and positive means it is to the right.

8. **Error Persistence**: If the lane is temporarily lost (no white pixels detected), the node keeps publishing the last known error value. This prevents the robot from making sudden erratic movements.

The vision node publishes the error value to the `/lane_error` topic at 5 Hz.

### Controller Node

The controller node takes the lane error and converts it into movement commands for the robot. It uses a simple proportional (P) controller:

1. **Subscribing to Error**: The node listens to the `/lane_error` topic to receive the current lane position error.

2. **Velocity Commands**: For each error value received, the controller calculates:
   - **Linear velocity**: Fixed at 0.2 m/s (constant forward speed)
   - **Angular velocity**: Calculated as `-Kp * error`, where Kp = 0.5

3. **Publishing Commands**: The velocity commands are published as `TwistStamped` messages to the `/cmd_vel` topic, which the TurtleBot3 listens to for movement instructions.

The negative sign in the angular velocity formula ensures that when the lane is detected to the right (positive error), the robot turns right to correct, and vice versa.

### Simulation Environment

The simulation runs in Gazebo using a custom track world. Key components include:

- **Track Model**: A custom 3D track mesh created in Blender and exported as a `.dae` file. The track features white lane markings on a dark surface.
- **TurtleBot3 Waffle Pi**: This robot model includes an RGB camera sensor needed for lane detection.
- **Physics Engine**: ODE physics engine running at 1000 Hz for realistic robot movement.
- **Spawn Position**: The robot starts at coordinates (3.0, 4.0) on the track.

### Launch System

A single launch file (`launch_project.py`) starts the entire system:
1. Sets the TurtleBot3 model to `waffle_pi`
2. Configures Gazebo resource paths for models
3. Launches the Gazebo simulator with the custom track world
4. Spawns the TurtleBot3 in the simulation
5. Starts both the vision node and controller node

### Work Distribution

| Team Member | Contributions |
|-------------|---------------|
| [Member 1]  | [Tasks completed] |
| [Member 2]  | [Tasks completed] |
| [Member 3]  | [Tasks completed] |

## Results
- quantitative/qualitative,you need to include screenshots, graphs, and other visuals to demonstrate results
## Conclusion
- A breif conclusion
# Project Report

## Introduction/Motivation

Autonomous navigation is a core challenge in robotics with real-world applications in self-driving cars, delivery robots, and warehouse automation. Our project focuses on building a lane-following system for a simulated turtlebot robot. We hope to show how a robot can use its camera sensor to the environment and convert that visual data into movement commands. Our project is relevant to the course because it incorporates what we have learned about ROS, computer vision, and the "sense-plan-act" cycle that is central to robotics.

## Methodology

Our system is built using ROS2 and runs in the Gazebo simulator. We use the TurtleBot3 (waffle_pi model) because it comes with a camera sensor. The project is split into two main parts: a vision node that detects the lane, and a controller node that steers the robot.

### Vision Node

The vision node handles all the image processing. It receives camera images and figures out where the lane is. Here is how it works:

1. **Get the image**: The node subscribes to the camera topic and uses cv_bridge to convert the ROS image into an OpenCV format we can work with.

2. **Focus on the road**: We crop the image to only look at the bottom 40%. This is where the lane markings actually appear, so there is no point processing the sky or background.

3. **Convert to HSV**: We convert the image from BGR to HSV color space. This makes it easier to filter out specific colors regardless of lighting conditions.

4. **Filter for white**: We apply a threshold to find white pixels. White has low saturation (0-50) and high brightness (200-255). Everything else gets filtered out.

5. **Clean up noise**: We use a morphological opening operation to remove small dots and noise from the image while keeping the lane shape intact.

6. **Find the lane center**: Using OpenCV's moments function, we calculate the center point of all the white pixels. This tells us where the lane is in the image.

7. **Calculate the error**: We compare where the lane center is versus where the image center is. The result is a number between -1 and 1 that tells us how far off-center the robot is.

If the lane is temporarily lost, the node keeps using the last known error value so the robot does not make sudden jerky movements.

### Controller Node

The controller node is simpler. It listens for the lane error and converts it into steering commands:

- **Forward speed**: Always set to 0.2 m/s
- **Steering**: Calculated as the error multiplied by -0.5 (the proportional gain)

When the error is positive (lane is to the right), the robot turns right. When the error is negative (lane is to the left), the robot turns left. The robot publishes these velocity commands to the `/cmd_vel` topic.

### Simulation Setup

We built a custom track in Blender with white lane markings on a dark surface. The track is loaded into Gazebo along with the TurtleBot3. A launch file starts everything at once: it opens Gazebo, spawns the robot on the track, and starts both nodes.

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
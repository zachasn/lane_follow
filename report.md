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
- How did you do what you did?
- Explain the core details of your project
- Provide a brief breakdown of work distribution -- that is, which project member did what part

## Results
- quantitative/qualitative,you need to include screenshots, graphs, and other visuals to demonstrate results
## Conclusion
- A breif conclusion
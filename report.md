# Project Report

## Introduction/Motivation

Autonomous navigation is one of the most fundamental challenges in robotics, with applications ranging from self-driving vehicles to warehouse automation. This project implements a lane-following system for a simulated TurtleBot3 robot, demonstrating how a robot can perceive its environment through computer vision and translate visual information into real-time motion control.

The motivation for this project stems from the desire to understand and implement the core "Sense-Plan-Act" paradigm that underlies most autonomous systems. Lane following serves as an ideal introductory problem because it encapsulates essential robotics concepts in a constrained, well-defined environment. The robot must continuously sense lane markings using its onboard camera, process this visual data to determine its position relative to the lane center, and act by adjusting its steering to maintain proper alignment.

We chose to work in simulation using Gazebo and ROS2 for several practical reasons. Simulation provides a safe environment to develop and test algorithms without risking damage to physical hardware. It also allows for rapid iteration and debugging, as the environment can be easily reset and parameters can be quickly adjusted. The TurtleBot3 platform was selected because it is a well-documented, widely-used robot in educational settings and comes equipped with the sensors needed for this task.

This project is directly relevant to CSCI 4551 (Robotics) as it integrates multiple core concepts covered in the course:

- **Computer Vision**: We apply OpenCV-based image processing techniques including color space conversion (BGR to HSV), thresholding, morphological operations, and moment-based centroid detection to identify lane markings.

- **Locomotion and Control**: The system implements a proportional controller that converts perceived lane error into appropriate steering commands, demonstrating closed-loop feedback control.

- **ROS2 Middleware**: The project architecture uses ROS2's publish-subscribe communication pattern with separate vision and controller nodes, showcasing modular robotic system design.

- **Simulation**: Working with Gazebo and custom world files demonstrates how simulation environments are used in robotics development pipelines.

By completing this project, we gain hands-on experience with the complete pipeline from perception to actuation, preparing us to tackle more complex autonomous systems in the future.

## Methodology
- How did you do what you did?
- Explain the core details of your project
- Provide a brief breakdown of work distribution -- that is, which project member did what part

## Results
- quantitative/qualitative,you need to include screenshots, graphs, and other visuals to demonstrate results
## Conclusion
- A breif conclusion
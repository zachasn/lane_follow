# 1. CSCI 4551 Project Presentation
## Lane-Following Turtlebot in Simulation
- Bilal Mohamed, Senior, CS Major
- Abdisamed Abdullahi, Junior, CS Major
- Zachariah Hassan, Senior, CS Major

# 2. Problem & Relevancy
- Problem: Autonomous navigation is challenging, especially in dynamic environments. It requires perceiving environment with a camera and converting that data into motion commands for the robot.
- Relevancy: Uses the following concepts studied in CSCI 4551:
    - Computer Vision: Processing camera images to detect lanes.
    - Locomotion: Implementing control strategies for robot movement.
    - Gazebo Simulation: Testing algorithms in a simulated environment.
    - It also fulfills:
        - Sense: the robot senses its environment using a camera.
        - Plan: the robot plans its path based on lane detection.
        - Act: the robot executes movement after perceiving the lane.

# 3. How we plan to complete the project
- Use a turtlebot3 robot in ROS2/Gazebo simulation and spawn the robot in a road environment with a marked lane.
- Detect the lane using OpenCV filtering and use a proportional controller to follow the lane.
- Publish velocity commands to the robot based on lane position.
- Test all components together in simulation.

# 4. project load Division
**Abdisamed: Control & Integration**
- Implement the control node (P-controller)
- Convert lane-error into steering commands
- Help integrate the final pipeline

**Bilal: Vision & Lane Detection**
- Subscribe to camera feed
- Use OpenCV filtering to detect lane center
- Publish lane-error data for the controller

**Zachariah: Simulation & Setup**
- Create the road environment
- Set up ROS2/Gazebo simulation environment
- Create launch files to run the simulation

# 5. What we hope to acomplish
- A working ROS2 package that allows a turtlebot3 to follow a lane in simulation using
    - Camera-based lane detection
    - Proportional control for steering and speed adjustment
- A video demo showcasing the robot successfully following the lane in simulation.
- A final report documenting our methodology, results, and learnings from the project.

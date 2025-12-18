import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # ======= Environment variables =======
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_share = get_package_share_directory('lane_follower_pkg')
    turtlebot_gazebo = get_package_share_directory('turtlebot3_gazebo')
    x_pose = LaunchConfiguration('x_pose', default='3.0')
    y_pose = LaunchConfiguration('y_pose', default='4.0')
    z_pose = LaunchConfiguration('z_pose', default='0.02')# stops robot from clipping into the road mesh
    world_file = os.path.join(pkg_share, 'world', 'track.sdf')
    turtlebot_model = SetEnvironmentVariable(
        'TURTLEBOT3_MODEL',
        'waffle_pi' # waffle_pi comes with a camera
    )
    set_env_vars_turtlebot = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(turtlebot_gazebo, 'models')
    )
    set_env_vars_pkg = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_share, 'models')
    )
   
    # ======= Gazebo setup =======
    start_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_file], 'on_exit_shutdown': 'true'}.items()
    )
   
    # ======= Spawn turtlebot  =======
    spawn_turtlebot = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
             os.path.join(turtlebot_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
         launch_arguments={'x_pose': x_pose, 'y_pose': y_pose, 'z_pose': z_pose}.items()
    )
    
    # ======= Vision Node =======
    vision_node = Node(
        package='lane_follower_pkg',
        executable='vision_node',
        prefix=['xterm -e ']
    )
    # ======= Control Node =======
    control_node = Node(
        package='lane_follower_pkg',
        executable='controller_node',
        prefix=['xterm -e ']
    )
    
    # rqt_img = Node(
    #     package='rqt_image_view',
    #     executable='rqt_image_view',
    #     prefix=['xterm -e ']
    # )
    # rqt_graph = Node(
    #     package='rqt_graph',
    #     executable='rqt_graph',
    #     prefix=['xterm -e ']
    # )
    return LaunchDescription([
        turtlebot_model,
        set_env_vars_turtlebot,
        set_env_vars_pkg,
        start_gz_sim,
        spawn_turtlebot,
        vision_node,
        control_node
    ])
   
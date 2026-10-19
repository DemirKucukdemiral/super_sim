import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("super_sim_dynamics")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    # Models and World paths
    models_path = os.path.join(pkg_share, "models")
    world_file = os.path.join(pkg_share, "worlds", "car_world.sdf")
    urdf_file = os.path.join(models_path, "my_car", "car.urdf")
    bridge_config = os.path.join(pkg_share, "config", "gz_bridge.yaml")

    # Launch Arguments
    gui_arg = DeclareLaunchArgument("gui", default_value="true", description="Launch Gazebo GUI window")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="false", description="Launch RViz2 visualizer")
    powertrain_arg = DeclareLaunchArgument("powertrain", default_value="EV", description="Powertrain: EV or ICE")
    terrain_arg = DeclareLaunchArgument("terrain", default_value="FLAT", description="Terrain mode: HILLS, FLAT, ROLLER_COASTER")

    autonomous_arg = DeclareLaunchArgument("autonomous", default_value="false", description="Start the controller adapter (do not run teleop simultaneously)")
    controller_arg = DeclareLaunchArgument("controller", default_value="super_sim_dynamics.controllers:StopController", description="Importable Python module:Class implementing step(observation, dt)")

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    powertrain = LaunchConfiguration("powertrain")
    terrain = LaunchConfiguration("terrain")

    # Set Gazebo Resource Path
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=f"{models_path}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    )

    # 1. Gazebo Simulation (SDF includes locked chase camera behind vehicle)
    gz_args_gui = f"-r {world_file}"
    gz_args_headless = f"-r -s {world_file}"

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args_gui}.items(),
        condition=IfCondition(gui)
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args_headless}.items(),
        condition=UnlessCondition(gui)
    )

    # 2. ROS-Gazebo Bridge (LiDAR, Camera RGB & Depth, IMU, Clock)
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen"
    )

    # 3. Robot State Publisher with car.urdf
    with open(urdf_file, "r") as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen"
    )

    # 4. 14-DoF Vehicle Dynamics & Powertrain Node with Dynamic Terrain Modeling
    vehicle_node = Node(
        package="super_sim_dynamics",
        executable="vehicle_node.py",
        name="vehicle_14dof_node",
        parameters=[os.path.join(pkg_share, "config", "vehicle_params_14dof.yaml"), {
            "powertrain": powertrain,
            "terrain": terrain,
        }],
        output="screen"
    )

    # 5. Optional RViz2 Visualizer
    rviz_config = os.path.join(pkg_share, "config", "super_sim.rviz")
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(rviz),
        output="screen"
    )

    autonomy_node = Node(
        package="super_sim_dynamics", executable="autonomy_node.py",
        parameters=[{"controller": LaunchConfiguration("controller")}],
        condition=IfCondition(LaunchConfiguration("autonomous")), output="screen"
    )

    return LaunchDescription([
        autonomous_arg, controller_arg, autonomy_node,
        gui_arg,
        rviz_arg,
        powertrain_arg,
        terrain_arg,
        set_gz_resource_path,
        gz_sim_gui,
        gz_sim_headless,
        gz_bridge,
        robot_state_publisher,
        vehicle_node,
        rviz2
    ])

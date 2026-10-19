# Super Sim

A ROS 2 Jazzy / Gazebo Harmonic vehicle simulator with a Python 14-DoF
vehicle model, city world, keyboard driving, and bridged camera, LiDAR and IMU.
The current development environment is Ubuntu in the `ugrdv-humble` OrbStack
machine (the machine name is historical; the installed ROS distribution is Jazzy).

## Start here

Inside the Linux machine, from this folder:

```bash
./build.bash
./run_sim.bash
```

In another terminal:

```bash
cd ~/super_sim
./run_teleop.bash
```

Stop with Ctrl+C in the launch terminal. `./kill_sim.bash` sends Ctrl+C only to
Super Sim launch processes owned by your user; it does not kill other ROS stacks.

From macOS, enter Linux first with `orb -m ugrdv-humble`.
Launchers resolve their own location, so the project can be moved or cloned
under another user without editing absolute paths.

### Prerequisites on a new machine

Use Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, its Python transport/message
bindings (`gz.transport13` and `gz.msgs10`), colcon, rosdep and pytest.
With ROS installed and rosdep initialized:

```bash
rosdep install --from-paths ros2_ws/src --ignore-src -r -y
./build.bash
```

`ROS_DISTRO` defaults to `jazzy`. Other distributions have not been tested.

### Useful launch options

```bash
./run_sim.bash gui:=false            # No Gazebo GUI (sensors may still need rendering)
./run_sim.bash rviz:=true            # Also open RViz
./run_sim.bash powertrain:=ICE       # Default is EV
./run_sim.bash terrain:=HILLS        # Default is FLAT
```

Terrain options change the analytical physics surface; they do not automatically
rebuild Gazebo's visible world. The simulation is a research model, not a validated
vehicle or a real-car deployment stack.

## Add your autonomous code

The physics engine stays separate from the algorithm. Choose one of two paths:

1. **A small Python controller:** implement `step(observation, dt)` and return a
   `DriveCommand`. The adapter handles ROS messages, timing, and stale odometry.
2. **An existing ROS 2 stack:** subscribe to the sensor/odometry topics below and
   publish actuator commands directly. You do not need the Python adapter.

See [Adding autonomy](docs/AUTONOMY.md) for a complete example.

```bash
# Starts the adapter with a default controller that holds the brakes.
./run_sim.bash autonomous:=true

# Low-speed straight-line example, for a clear simulated road only.
./run_sim.bash autonomous:=true controller:=super_sim_dynamics.controllers:CruiseController
```

Run only one command-producing stack at a time. The vehicle currently accepts
both `/cmd_vel` and direct drive messages; there is no command multiplexer.
Do not run teleop alongside an autonomous controller.

## Interfaces

| Topic | ROS message | Meaning |
|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | Model pose and body-frame velocity |
| `/my_car/ground_truth/odom` | `nav_msgs/msg/Odometry` | Ground-truth model odometry |
| `/my_car/drive_request` | `std_msgs/msg/Float32MultiArray` | `[throttle, brake, steering_rad, handbrake]` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Forward target speed and yaw rate; negative speed requests braking, not reverse |
| `/my_car/lidar/points` | `sensor_msgs/msg/PointCloud2` | Gazebo LiDAR |
| `/my_car/camera/image` | `sensor_msgs/msg/Image` | RGB camera |
| `/my_car/camera/depth_image` | `sensor_msgs/msg/Image` | Depth camera |
| `/my_car/imu` | `sensor_msgs/msg/Imu` | Gazebo IMU |
| `/my_car/vehicle_status` | `std_msgs/msg/String` | JSON telemetry |

Direct drive: throttle is clamped to `[-1, 1]`, brake to `[0, 1]`, and steering
to the vehicle's maximum angle. Brake or handbrake overrides throttle.
Malformed/non-finite direct commands apply the brakes. A three-value message
is accepted and releases the handbrake. Positive steering turns left.

The dynamics loop uses a fixed time step with a wall-clock timer. It is not
synchronized to Gazebo's `/clock`; pausing Gazebo does not pause that loop.
The autonomous adapter therefore uses wall time as well. Sensor timing and
real-time factor should be considered when evaluating perception algorithms.

## Project map

```text
models/                       Vehicle meshes, URDF, and Gazebo models
worlds/                       Gazebo world and collision bounds
ros2_ws/src/super_sim_dynamics/
  launch/                     Launch composition
  config/                     Runtime ROS parameters, bridge mappings, RViz
  super_sim_dynamics/
    vehicle_*model.py         Physics and physical parameter dataclasses
    vehicle_node.py           ROS/Gazebo adapter for the physics
    tire_model.py             Tire forces
    engine_model.py           Powertrain
    braking_model.py          Brakes and ABS
    terrain_model.py          Analytical terrain
    teleop_node.py            Keyboard controls
    autonomy.py               Observation and DriveCommand contract
    autonomy_node.py          ROS adapter for user controllers
    controllers.py            Stop and cruise examples
  test/                       Physics and controller-contract tests
docs/                         Extension and GitHub guides
```

Edit `Vehicle14DoFParams` and the tire/engine/brake parameter dataclasses to tune
physical properties. `config/vehicle_params_14dof.yaml` contains only supported
runtime ROS settings; launch arguments take precedence. Rebuild after changing
launch configuration or adding installed files.

## Check your changes

```bash
./run_tests.bash -q         # No ROS/Gazebo needed; requires Python + pytest
./run_benchmark.bash       # Numerical exploration; not a certification test
```

[VEHICLE_DYNAMICS.md](VEHICLE_DYNAMICS.md) contains the longer modelling notes.
Historical descriptions are preserved in [the previous README](docs/README.previous.md);
its removed mission commands and claims should not be used as current setup instructions.

## Put this folder on GitHub

See [GitHub setup](docs/GITHUB.md). Generated build/install/log folders, caches,
backup files, and recorded runs are excluded by `.gitignore`.

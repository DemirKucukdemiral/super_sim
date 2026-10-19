# Adding an autonomous controller

## Python adapter

Put this in `ros2_ws/src/super_sim_dynamics/super_sim_dynamics/my_controller.py`:

```python
from super_sim_dynamics.autonomy import DriveCommand

class MyController:
    def __init__(self):
        # Load your path/model or initialize controller state here.
        pass

    def step(self, observation, dt):
        # observation: x, y, yaw, speed, yaw_rate
        # units: metres, seconds, radians; speed is body-forward velocity
        # dt is elapsed wall time since the last controller tick.
        # Replace this with your planning/control calculation.
        return DriveCommand(brake=1.0)
```

Then run:

```bash
./build.bash
./run_sim.bash autonomous:=true controller:=super_sim_dynamics.my_controller:MyController
```

The class must take no constructor arguments and implement `step(observation, dt)`.
Keep `step` short enough to run at 20 Hz. Internal state can live on `self`.
Separate expensive perception/planning into its own ROS nodes; communicate via
ROS topics rather than blocking the controller loop.

The adapter clamps outputs, applies braking while odometry is unavailable or
older than 0.5 s, and latches into braking if your controller raises an exception
or returns invalid output. Fix the problem and restart the node to recover.
These checks do not provide obstacle avoidance or make arbitrary algorithms safe.

To run the adapter separately and customize timing:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run super_sim_dynamics autonomy_node.py --ros-args \
  -p controller:=super_sim_dynamics.my_controller:MyController \
  -p rate_hz:=20.0 -p odom_timeout_s:=0.5
```

## Existing ROS packages, perception and planning

Add your ROS packages under `ros2_ws/src/`, then build the whole workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`build.bash` builds only the simulator package. Use your package's own launch file
for a larger autonomous stack. Run Super Sim without `autonomous:=true` if your
stack already publishes commands.

Publish `std_msgs/msg/Float32MultiArray` on `/my_car/drive_request`:

```python
msg.data = [0.15, 0.0, 0.05, 0.0]  # throttle, brake, steering radians, handbrake
```

Send commands regularly (e.g. 20 Hz). The vehicle's existing command watchdog
reduces throttle after 0.5 s without input; this is distinct from the adapter's
full-brake stale-odometry response. Only one publisher should own vehicle commands.

For sensor subscriptions, use a QoS compatible with the bridge (typically
`rclpy.qos.qos_profile_sensor_data`). Check actual publisher QoS with
`ros2 topic info --verbose TOPIC`. `/odom` is ground truth, not a localization
estimate; use your own estimator when evaluating realistic autonomy.

The simulator currently uses a single vehicle and fixed `/my_car/...` topic names.
Changing `car_name` alone does not provide multi-vehicle namespacing.

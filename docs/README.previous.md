# Super Sim Autonomous Vehicle Simulation (14-DoF Dynamics & Sensor Stack)

A high-fidelity autonomous vehicle simulation platform built inside Gazebo Harmonic and ROS 2 Jazzy, featuring the user custom `my_car` 3D model, a full **14 Degrees of Freedom (14-DoF)** vehicle dynamics engine, high-performance powertrain (EV & ICE with LSD), hydraulic braking model with closed-loop **Anti-Lock Braking System (ABS)**, 3D LiDAR, RGB-D Camera, and IMU.

---

## 1. System Architecture

```
super_sim/
├── models/
│   └── my_car/
│       ├── model.config          # Model metadata
│       ├── model.sdf             # Gazebo Harmonic SDF with sensors & joint controllers
│       ├── car.urdf              # ROS 2 URDF (REP-120 compliant) for TF & RViz2
│       └── meshes/               # Centered & aligned vehicle meshes (5.1m L x 2.0m W x 1.6m H)
│           ├── car_body_aligned.dae
│           ├── wheel_fl_aligned.dae
│           ├── wheel_fr_aligned.dae
│           ├── wheel_rl_aligned.dae
│           └── wheel_rr_aligned.dae
├── worlds/
│   └── car_world.sdf             # World with 61 city buildings (physical collisions), avenues & cross streets
├── ros2_ws/
│   └── src/
│       └── super_sim_dynamics/   # ROS 2 14-DoF Dynamics package
│           ├── config/
│           │   ├── vehicle_params_14dof.yaml  # Tunable vehicle parameters
│           │   ├── gz_bridge.yaml             # ROS-Gazebo bridge mappings
│           │   └── super_sim.rviz             # Pre-configured RViz2 layout
│           ├── launch/
│           │   └── super_sim.launch.py        # Master launch file
│           ├── super_sim_dynamics/
│           │   ├── tire_model.py              # Pacejka 96 Magic Formula + combined slip
│           │   ├── engine_model.py            # EV motor & ICE + Salisbury LSD
│           │   ├── braking_model.py           # Hydraulic caliper torques + ABS controller
│           │   ├── vehicle_14dof_model.py     # 14-DoF differential equations & kinematics
│           │   ├── vehicle_node.py            # Main 100 Hz simulation node & Gazebo sync
│           │   ├── autonomous_controller.py   # Pure pursuit & automated test missions
│           │   └── teleop_node.py             # Terminal teleop HUD & mission dispatcher
│           └── test/
│               └── test_14dof_physics.py      # Automated physics & stability test suite
├── run_sim.bash                  # Quick launcher for Gazebo & ROS 2 stack
├── run_teleop.bash               # Quick launcher for terminal HUD teleop
└── run_tests.bash                # Quick launcher for pytest physics suite
```

---

## 2. Mathematical Modeling (14 Degrees of Freedom)

The simulation integrates a complete 14-DoF non-linear multi-body dynamic system:

### A. Sprung Mass Chassis (6 DoF)
- **Surge ($u$)**: Longitudinal velocity along body x-axis.
- **Sway ($v$)**: Lateral velocity along body y-axis.
- **Heave ($w$)**: Vertical velocity along body z-axis.
- **Roll ($p$)**: Angular velocity about body x-axis.
- **Pitch ($q$)**: Angular velocity about body y-axis.
- **Yaw ($r$)**: Angular velocity about body z-axis.

Differential Equations of Motion:
$$\begin{aligned}
m_s (\dot{u} - v r + w q) &= \sum F_{x,b} - F_{\text{drag}} - m_s g \sin\theta \\
m_s (\dot{v} - w p + u r) &= \sum F_{y,b} + m_s g \cos\theta \sin\phi \\
m_s (\dot{w} - u q + v p) &= \sum \Delta F_{\text{susp}} - F_{\text{downforce}} \\
I_{xx} \dot{p} + (I_{zz} - I_{yy}) q r &= M_x \\
I_{yy} \dot{q} + (I_{xx} - I_{zz}) p r &= M_y \\
I_{zz} \dot{r} + (I_{yy} - I_{xx}) p q &= M_z
\end{aligned}$$

### B. Suspension & Unsprung Masses (4 DoF)
Four independent corners ($FL, FR, RL, RR$):
- Corner deflection: $\Delta z_i = z_{u,i} - z_{c,i}$.
- Front Anti-Roll Bar: $F_{\text{arb},f} = K_{\text{arb},f} (\Delta z_0 - \Delta z_1)$.
- Rear Anti-Roll Bar: $F_{\text{arb},r} = K_{\text{arb},r} (\Delta z_2 - \Delta z_3)$.
- Unsprung vertical acceleration:
  $$m_{u,i} \ddot{z}_{u,i} = F_{t,z,i} - F_{\text{susp},i}$$

### C. Wheel Spin Dynamics (4 DoF)
Rotational dynamics for each individual wheel with spin velocity $\omega_i$:
$$I_w \dot{\omega}_i = T_{\text{drive},i} - T_{\text{brake},i} \operatorname{sgn}(\omega_i) - F_{x,i} R_w - T_{\text{rr},i}$$

### D. Tire Friction & Magic Formula Model
- Pure longitudinal ($F_{x0}$) and lateral ($F_{y0}$) forces computed using the **Pacejka 96 Magic Formula**:
  $$F_0 = D \sin\left(C \arctan\left(B \Phi - E (B \Phi - \arctan(B \Phi))\right)\right)$$
- Coupled via a **Friction Ellipse**:
  $$\sigma = \sqrt{\left(\frac{\kappa}{\kappa_{\text{peak}}}\right)^2 + \left(\frac{\alpha}{\alpha_{\text{peak}}}\right)^2}$$
- Enforces the physical grip boundary: $\sqrt{F_x^2 + F_y^2} \le \mu F_z$.
- Smooth low-speed regularization eliminates numerical zero-speed singularities during stops and launches.

---

## 3. Engine & Braking Models

### Powertrain Model
- **EV Mode**: 240 kW motor delivering up to 450 Nm of instantaneous torque, with constant-power scaling above base speed (5200 RPM) up to 14,500 RPM, including regenerative braking.
- **ICE Mode**: Realistic torque-RPM power curve (peaking at 380 Nm @ 4400 RPM), idle governor (850 RPM), redline limiter (7000 RPM), 6-speed transmission, and automatic shifting.
- **Drivetrain & Differential**: Salisbury Limited Slip Differential (LSD) dynamically biasing torque between left and right wheels based on differential wheel speed during cornering.

### Braking System & ABS Controller
- Master cylinder pressure builds with hydraulic line lag ($\tau = 25\text{ ms}$).
- Dynamic front/rear brake bias (65% front, 35% rear).
- **Active ABS**: Monitors wheel slip $\kappa_i = \frac{R_w \omega_i - v_{x,w,i}}{\max(|v_{x,w,i}|, 0.5)}$. When negative slip exceeds $-16\%$, the ABS state machine enters `DUMP` phase (reducing pressure by 60%), then `HOLD`, then `RAMP`, preventing wheel lockup and maintaining steering control.

---

## 4. Sensors & Perception

All sensors are modeled physically on `my_car` and bridged to standard ROS 2 topics:
1. **3D LiDAR (`gpu_lidar`)**:
   - 32 vertical rings, 1080 horizontal samples (360° coverage).
   - Range: 0.3 m to 100.0 m @ 15 Hz.
   - Topic: `/my_car/lidar/points` (`sensor_msgs/msg/PointCloud2`).
2. **RGB-D Camera (`rgbd_camera`)**:
   - Forward-facing on windshield/roof (1280x720 @ 30 Hz, 90° FOV).
   - Topics:
     - RGB Image: `/my_car/camera/image` (`sensor_msgs/msg/Image`).
     - Depth Image: `/my_car/camera/depth_image` (`sensor_msgs/msg/Image`).
     - Camera Info: `/my_car/camera/camera_info` (`sensor_msgs/msg/CameraInfo`).
3. **IMU Sensor (`imu`)**:
   - 6-axis gyroscope and accelerometer with Gaussian noise.
   - Topic: `/my_car/imu` (`sensor_msgs/msg/Imu`).
4. **Vehicle Telemetry**:
   - Full 6-DOF odometry: `/odom` (`nav_msgs/msg/Odometry`).
   - Wheel speeds (RPM & rad/s): `/my_car/wheel_speeds` (`sensor_msgs/msg/JointState`).
   - Steering angle: `/my_car/steering_angle` (`std_msgs/msg/Float32`).
   - JSON Status: `/my_car/vehicle_status` (`std_msgs/msg/String`).

---

---

## 5. Urban City Map & Building Collision Dynamics

The simulation features a comprehensive urban environment designed for realistic city driving and perception testing:

### A. Road Network & City Grid
- **Scale**: $440\text{ m} \times 240\text{ m}$ street network across 28 city blocks.
- **5 East-West Avenues**: Located at $Y = -120, -60, 0, 60, 120$ m. The main Central Avenue at $Y = 0$ provides a 14 m wide roadway with a 22 m clearance canyon between building facades.
- **6 North-South Cross Streets**: Located at $X = -110, -50, 20, 90, 160, 220$ m, creating realistic 4-way intersections and urban turns.
- **Pure Urban Styling**: All racetrack cones, race apex markers, and track rumble curbs have been completely removed in favor of asphalt avenues, sidewalks, and crosswalk markings.

### B. Building Architecture & Solid Collisions
- **61 Distinct Buildings**: Ranging from 12 m to 65 m in height with varied architectural finishes (Granite, Sandstone, White Plaza, Red Brick, Bronze Glass, Steel Office) and rooftop mechanical penthouses.
- **Dual-Layer Physical Collisions**:
  1. **Gazebo Harmonic Physical Collisions**: Every building has an explicit `<collision>` geometry box with high contact stiffness ($K_p = 10^8$) and surface friction ($\mu = 1.2$), enabling realistic LiDAR reflections, depth camera returns, and Gazebo contact physics.
  2. **14-DoF Analytical Collision Engine**: The vehicle dynamics node integrates building bounding boxes (`city_buildings.json`) at 100 Hz ($<3\,\mu\text{s}$ check). When contact occurs, the vehicle is pushed outside the wall boundary and subjected to inelastic restitution ($u \leftarrow -0.15u$, $v \leftarrow 0$, $r \leftarrow 0$), preventing any wall penetration in odometry, TF, and kinematics.

## 6. Quick Start Guide

### Step 1: Enter OrbStack Container
```bash
orb -m ugrdv-humble
cd ~/super_sim
```

### Step 2: Run Full Simulation (Gazebo GUI + 14-DoF Physics + Sensors)
```bash
./run_sim.bash
```
*Optional flags:*
- `gui:=false`: Run Gazebo in headless mode for high-performance CPU simulation.
- `rviz:=true`: Launch RViz2 pre-configured with 3D LiDAR point cloud and RGB-D camera feeds.
- `autonomous:=true mode:=ACCEL_TEST`: Run automatic 0-100 km/h acceleration test.
- `autonomous:=true mode:=BRAKE_TEST`: Run automatic ABS emergency braking test.
- `autonomous:=true mode:=SLALOM_TEST`: Run automated slalom cone weaving test.
- `powertrain:=ICE`: Switch powertrain from EV (default) to Internal Combustion Engine.

### Step 3: Interactive Terminal Teleoperation HUD
In a separate terminal window:
```bash
orb -m ugrdv-humble
cd ~/super_sim
./run_teleop.bash
```

### Step 4: Run Physics & Model Test Suite
```bash
cd ~/super_sim
./run_tests.bash
```

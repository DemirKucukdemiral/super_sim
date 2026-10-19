#!/usr/bin/env python3
"""
Main ROS 2 Vehicle Node for 14-DoF Dynamics, Engine, Braking & Sensor Stack.
Synchronizes with Gazebo Harmonic via gz-transport.
"""

import json
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from super_sim_dynamics.autonomy import DriveCommand

import gz.transport13 as gzt
from gz.msgs10.double_pb2 import Double
from gz.msgs10.cameratrack_pb2 import CameraTrack

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32, String, Float32MultiArray
from tf2_ros import TransformBroadcaster

from super_sim_dynamics.vehicle_14dof_model import (
    Vehicle14DoFParams, Vehicle14DoFState, step_14dof, calc_ackermann_angles
)
from super_sim_dynamics.engine_model import PowertrainType, DrivetrainLayout
from super_sim_dynamics.terrain_model import TerrainModel, TerrainType


RPM_TO_RAD_S = math.pi / 30.0

WHEEL_SPIN_TOPICS = {
    "fl": "my_car/spin_fl_cmd",
    "fr": "my_car/spin_fr_cmd",
    "rl": "my_car/spin_rl_cmd",
    "rr": "my_car/spin_rr_cmd",
}

STEER_TOPICS = {
    "fl": "my_car/steer_fl_cmd",
    "fr": "my_car/steer_fr_cmd",
}


class Vehicle14DoFNode(Node):
    def __init__(self):
        super().__init__("vehicle_14dof_node")

        # Parameters
        self.declare_parameter("use_gazebo", True)
        self.declare_parameter("update_rate_hz", 100.0)
        self.declare_parameter("car_name", "my_car")
        self.declare_parameter("powertrain", "EV")
        self.declare_parameter("drivetrain", "AWD")
        self.declare_parameter("abs_enabled", True)
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("spawn_yaw", 0.0)

        self.use_gazebo = self.get_parameter("use_gazebo").value
        self.update_rate_hz = self.get_parameter("update_rate_hz").value
        self.car_name = self.get_parameter("car_name").value
        powertrain_str = self.get_parameter("powertrain").value
        drivetrain_str = self.get_parameter("drivetrain").value
        abs_enabled = self.get_parameter("abs_enabled").value

        # Initialize Terrain Model (Flat vs Hilly terrain)
        self.declare_parameter("terrain", "FLAT")
        terrain_str = str(self.get_parameter("terrain").value).upper()
        try:
            self.terrain = TerrainModel(TerrainType[terrain_str])
        except KeyError:
            self.terrain = TerrainModel(TerrainType.FLAT)

        # Load City Building Obstacles for Collision Resolution
        self._city_buildings = []
        city_bldg_path = Path(get_package_share_directory("super_sim_dynamics")) / "worlds/city_buildings.json"
        if city_bldg_path.exists():
            try:
                with open(city_bldg_path, "r") as f:
                    self._city_buildings = json.load(f)
                self.get_logger().info(f"Loaded {len(self._city_buildings)} building obstacles for collision handling.")
            except Exception as e:
                self.get_logger().warning(f"Failed to load building obstacles: {e}")

        self.create_subscription(String, "/my_car/set_terrain_type", self._on_set_terrain, 10)

        # Initialize Physics Parameters
        self.params = Vehicle14DoFParams()
        if powertrain_str.upper() == "ICE":
            self.params.engine_params.powertrain_type = PowertrainType.ICE
        else:
            self.params.engine_params.powertrain_type = PowertrainType.EV

        if drivetrain_str.upper() == "RWD":
            self.params.engine_params.drivetrain_layout = DrivetrainLayout.RWD
        elif drivetrain_str.upper() == "FWD":
            self.params.engine_params.drivetrain_layout = DrivetrainLayout.FWD
        else:
            self.params.engine_params.drivetrain_layout = DrivetrainLayout.AWD

        self.params.brake_params.abs_enabled = abs_enabled

        # Initial State
        sx = self.get_parameter("spawn_x").value
        sy = self.get_parameter("spawn_y").value
        syaw = self.get_parameter("spawn_yaw").value
        self.state = Vehicle14DoFState(
            x=sx, y=sy, z=self.params.nominal_ride_height, yaw=syaw
        )

        # Control Inputs
        self.throttle_cmd = 0.0
        self.brake_cmd = 0.0
        self.steer_cmd = 0.0
        self.handbrake_cmd = False
        self.last_cmd_time = self.get_clock().now()
        self.command_timeout_s = 0.5

        # Gazebo Transport Node & Fast Non-Blocking Actuator Publishers
        self.gz_node = gzt.Node()
        self._wheel_spin_pubs = {
            k: self.gz_node.advertise(t, Double) for k, t in WHEEL_SPIN_TOPICS.items()
        }
        self._steer_pubs = {
            k: self.gz_node.advertise(t, Double) for k, t in STEER_TOPICS.items()
        }
        self._gui_track_pub = self.gz_node.advertise("/gui/track", CameraTrack)

        # ROS 2 Subscriptions
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._on_initial_pose, 10)
        self.create_subscription(Float32MultiArray, "/my_car/drive_request", self._on_drive_request, 10)

        # ROS 2 Publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.gt_odom_pub = self.create_publisher(Odometry, "/my_car/ground_truth/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/my_car/imu_dynamics", 10)
        self.steering_pub = self.create_publisher(Float32, "/my_car/steering_angle", 10)
        self.wheel_speed_pub = self.create_publisher(JointState, "/my_car/wheel_speeds", 10)
        self.status_pub = self.create_publisher(String, "/my_car/vehicle_status", 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # 100 Hz Physics Stepping Timer
        self.dt = 1.0 / self.update_rate_hz
        self.timer = self.create_timer(self.dt, self._on_timer_tick)

        # Automatic Gazebo Third-Person Camera Lock behind my_car
        self._cam_lock_attempts = 0
        self._cam_lock_timer = self.create_timer(0.5, self._lock_gazebo_camera)

        self.get_logger().info(
            f"Vehicle 14-DoF Node initialized. Powertrain: {self.params.engine_params.powertrain_type.value}, "
            f"Drivetrain: {self.params.engine_params.drivetrain_layout.value}, ABS: {abs_enabled}"
        )

    def _lock_gazebo_camera(self):
        """Commands Gazebo GUI camera to rigidly follow and track behind my_car."""
        if not self.use_gazebo or self._cam_lock_attempts > 24:
            if self._cam_lock_timer:
                self._cam_lock_timer.cancel()
            return

        self._cam_lock_attempts += 1
        try:
            from gz.msgs10.cameratrack_pb2 import CameraTrack
            from gz.msgs10.stringmsg_pb2 import StringMsg
            from gz.msgs10.vector3d_pb2 import Vector3d
            from gz.msgs10.boolean_pb2 import Boolean

            # 1. Native Gazebo Track & Follow (mode: FOLLOW, follow_target + track_target with P=1.0)
            # This causes the camera to tether to the car's local coordinate frame and rotate with vehicle yaw.
            track_msg = CameraTrack()
            track_msg.track_mode = CameraTrack.FOLLOW
            track_msg.follow_target.name = self.car_name
            track_msg.track_target.name = self.car_name
            track_msg.follow_offset.x = -5.8
            track_msg.follow_offset.y = 0.0
            track_msg.follow_offset.z = 2.1
            track_msg.track_offset.x = 2.0
            track_msg.track_offset.y = 0.0
            track_msg.track_offset.z = 0.5
            track_msg.follow_pgain = 1.0
            track_msg.track_pgain = 1.0
            self._gui_track_pub.publish(track_msg)

            # 2. Also invoke service as fallback
            req_target = StringMsg()
            req_target.data = self.car_name
            self.gz_node.request("/gui/follow", req_target, StringMsg, Boolean, 40)

            req_offset = Vector3d()
            req_offset.x = -5.8
            req_offset.y = 0.0
            req_offset.z = 2.1
            self.gz_node.request("/gui/follow/offset", req_offset, Vector3d, Boolean, 40)

            if self._cam_lock_attempts == 1:
                self.get_logger().info(f"Gazebo chase camera locked rigidly behind {self.car_name} (yaw-locked, P=1.0).")
        except Exception:
            pass

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Converts standard geometry_msgs/Twist to vehicle throttle, brake, and steering.
        Steering and Throttle/Brake are completely decoupled."""
        target_vx = msg.linear.x
        target_omega = msg.angular.z
        curr_vx = self.state.u

        # 1. Steering ONLY: strictly controls front wheels based on target_omega
        vx_ref = max(1.0, abs(curr_vx))
        delta = math.atan2(self.params.wheelbase * target_omega, vx_ref)
        max_steer_speed = self.params.max_steer_angle_rad / (1.0 + 0.007 * (curr_vx ** 2))
        self.steer_cmd = max(-max_steer_speed, min(max_steer_speed, delta))

        # 2. Throttle & Brake ONLY: strictly controls engine/brake, NEVER touched by steering
        if abs(target_vx) < 1e-4:
            self.throttle_cmd = 0.0
            self.brake_cmd = 0.05 if abs(curr_vx) > 0.2 else 0.0
        elif target_vx > 0.0:
            if curr_vx < target_vx:
                self.throttle_cmd = min(1.0, max(0.0, (target_vx - curr_vx) * 0.30))
                self.brake_cmd = 0.0
            else:
                self.throttle_cmd = 0.0
                self.brake_cmd = min(1.0, (curr_vx - target_vx) * 0.30)
        else:
            self.throttle_cmd = 0.0
            self.brake_cmd = min(1.0, abs(target_vx) * 0.40)

        self.last_cmd_time = self.get_clock().now()

    def _on_drive_request(self, msg: Float32MultiArray) -> None:
        """Direct drive input: [throttle (0..1), brake (0..1), steering_rad, handbrake (0/1)]."""
        try:
            command = DriveCommand.from_array(msg.data).limited(self.params.max_steer_angle_rad)
        except (ValueError, TypeError):
            command = DriveCommand.stop()
            self.get_logger().warning("Invalid drive request; applying brakes.")
        self.throttle_cmd = command.throttle
        self.brake_cmd = command.brake
        self.steer_cmd = command.steering
        self.handbrake_cmd = command.handbrake
        self.last_cmd_time = self.get_clock().now()

    def _on_set_terrain(self, msg: String):
        try:
            t_type = TerrainType[msg.data.strip().upper()]
            self.terrain.set_type(t_type)
            self.get_logger().info(f"Terrain type updated to: {t_type.value}")
        except KeyError:
            self.get_logger().warning(f"Unknown terrain type: {msg.data}")

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Resets vehicle state to specified pose."""
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(o.z, o.w)
        self.state = Vehicle14DoFState(
            x=p.x, y=p.y, z=self.params.nominal_ride_height, yaw=yaw
        )
        self.get_logger().info(f"Vehicle reset to x={p.x:.2f}, y={p.y:.2f}, yaw={yaw:.2f}")

    def _sync_with_gazebo(self) -> None:
        """Updates Gazebo wheel joint velocity commands and front steering angle asynchronously."""
        if not self.use_gazebo:
            return

        # 1. Drive Wheel Spin Joints (asynchronous non-blocking topic publish)
        for key, w in (
            ("fl", self.state.omega[0]),
            ("fr", self.state.omega[1]),
            ("rl", self.state.omega[2]),
            ("rr", self.state.omega[3])
        ):
            self._wheel_spin_pubs[key].publish(Double(data=float(w)))

        # 2. Drive Front Steering Joints (asynchronous non-blocking topic publish)
        delta_fl, delta_fr = calc_ackermann_angles(self.params, self.state.steer_angle)
        self._steer_pubs["fl"].publish(Double(data=float(delta_fl)))
        self._steer_pubs["fr"].publish(Double(data=float(delta_fr)))

    def _publish_telemetry(self) -> None:
        now = self.get_clock().now().to_msg()

        # 1. TF: map -> odom -> base_link
        t_odom = TransformStamped()
        t_odom.header.stamp = now
        t_odom.header.frame_id = "map"
        t_odom.child_frame_id = "odom"
        t_odom.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_odom)

        t_base = TransformStamped()
        t_base.header.stamp = now
        t_base.header.frame_id = "odom"
        t_base.child_frame_id = "base_link"
        t_base.transform.translation.x = self.state.x
        t_base.transform.translation.y = self.state.y
        t_base.transform.translation.z = self.state.z

        cr = math.cos(self.state.roll * 0.5)
        sr = math.sin(self.state.roll * 0.5)
        cp = math.cos(self.state.pitch * 0.5)
        sp = math.sin(self.state.pitch * 0.5)
        cy = math.cos(self.state.yaw * 0.5)
        sy = math.sin(self.state.yaw * 0.5)

        t_base.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t_base.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t_base.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t_base.transform.rotation.z = cr * cp * sy - sr * sp * cy
        self.tf_broadcaster.sendTransform(t_base)

        # 2. Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.state.x
        odom.pose.pose.position.y = self.state.y
        odom.pose.pose.position.z = self.state.z
        odom.pose.pose.orientation = t_base.transform.rotation
        odom.twist.twist.linear.x = self.state.u
        odom.twist.twist.linear.y = self.state.v
        odom.twist.twist.linear.z = self.state.w
        odom.twist.twist.angular.x = self.state.p
        odom.twist.twist.angular.y = self.state.q
        odom.twist.twist.angular.z = self.state.r
        self.odom_pub.publish(odom)

        # 3. IMU
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "base_link"
        imu.orientation = t_base.transform.rotation
        imu.angular_velocity.x = self.state.p
        imu.angular_velocity.y = self.state.q
        imu.angular_velocity.z = self.state.r
        g = 9.81
        imu.linear_acceleration.x = (sum(self.state.tire_fx) / self.params.sprung_mass)
        imu.linear_acceleration.y = (sum(self.state.tire_fy) / self.params.sprung_mass)
        imu.linear_acceleration.z = g * math.cos(self.state.pitch) * math.cos(self.state.roll)
        self.imu_pub.publish(imu)

        # 4. Wheel Speeds
        js = JointState()
        js.header.stamp = now
        js.name = ["wheel_front_left_spin", "wheel_front_right_spin", "wheel_rear_left_spin", "wheel_rear_right_spin"]
        js.velocity = [float(w) for w in self.state.omega]
        self.wheel_speed_pub.publish(js)

        # 5. Steering Angle
        self.steering_pub.publish(Float32(data=float(self.state.steer_angle)))

        # 6. Detailed Telemetry Status
        status_dict = {
            "speed_kmh": round(self.state.u * 3.6, 2),
            "engine_rpm": round(self.state.engine_state.rpm, 1),
            "gear": self.state.engine_state.current_gear,
            "throttle_pct": round(self.state.engine_state.actual_throttle * 100.0, 1),
            "brake_line_bar": round(self.state.brake_state.master_cylinder_pressure_bar, 1),
            "abs_active": self.state.brake_state.abs_active,
            "roll_deg": round(math.degrees(self.state.roll), 2),
            "pitch_deg": round(math.degrees(self.state.pitch), 2),
            "yaw_rate_deg_s": round(math.degrees(self.state.r), 2),
            "altitude_z": round(self.state.z, 2),
            "grade_pct": round(math.tan(self.state.pitch) * 100.0, 1),
            "terrain": self.terrain.terrain_type.value,
            "slips": [round(s, 3) for s in self.state.wheel_slips],
            "normal_loads_N": [round(f, 1) for f in self.state.normal_forces]
        }
        self.status_pub.publish(String(data=json.dumps(status_dict)))

    def _resolve_building_collisions(self) -> bool:
        """Checks for penetration between the vehicle bounding box and city buildings,
        resolving overlaps by pushing the vehicle back and applying inelastic restitution."""
        if not self._city_buildings:
            return False

        hl = 2.4  # Half vehicle length (m)
        hw = 1.0  # Half vehicle width (m)
        cos_y = math.cos(self.state.yaw)
        sin_y = math.sin(self.state.yaw)
        dx = hl * abs(cos_y) + hw * abs(sin_y)
        dy = hl * abs(sin_y) + hw * abs(cos_y)

        x = self.state.x
        y = self.state.y
        collided = False

        # Check up to 2 iterations to resolve multiple building or corner overlaps
        for _ in range(2):
            iter_col = False
            for b in self._city_buildings:
                car_xmin, car_xmax = x - dx, x + dx
                car_ymin, car_ymax = y - dy, y + dy
                if car_xmax > b["x_min"] and car_xmin < b["x_max"] and car_ymin > b["y_min"] and car_ymin < b["y_max"]:
                    ov_w = car_xmax - b["x_min"]
                    ov_e = b["x_max"] - car_xmin
                    ov_s = car_ymax - b["y_min"]
                    ov_n = b["y_max"] - car_ymin
                    min_ov = min(ov_w, ov_e, ov_s, ov_n)

                    if min_ov == ov_w:
                        x -= (ov_w + 0.05)
                    elif min_ov == ov_e:
                        x += (ov_e + 0.05)
                    elif min_ov == ov_s:
                        y -= (ov_s + 0.05)
                    else:
                        y += (ov_n + 0.05)
                    iter_col = True
                    collided = True
            if not iter_col:
                break

        if collided:
            self.state.x = x
            self.state.y = y
            # Inelastic collision: bounce back slightly and zero lateral/rotational momentum
            self.state.u = -0.15 * self.state.u
            self.state.v = 0.0
            self.state.w = 0.0
            self.state.r = 0.0
            self.state.p = 0.0
            self.state.q = 0.0
            self.state.omega = [0.0, 0.0, 0.0, 0.0]

        return collided

    def _on_timer_tick(self) -> None:
        # Check command timeout
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if time_since_cmd > self.command_timeout_s:
            self.throttle_cmd = 0.0
            self.brake_cmd = 0.05 if abs(self.state.u) > 0.5 else 0.0

        # Query dynamic ground elevation at 4 tire patches
        road_heights = self.terrain.get_wheel_heights(
            self.state.x, self.state.y, self.state.yaw,
            a=self.params.dist_cg_front,
            b=self.params.dist_cg_rear,
            wf=self.params.track_width_front * 0.5,
            wr=self.params.track_width_rear * 0.5
        )

        # Step 14-DoF Physics with full 3D terrain interaction
        self.state = step_14dof(
            self.params,
            self.state,
            self.throttle_cmd,
            self.brake_cmd,
            self.steer_cmd,
            self.handbrake_cmd,
            self.dt,
            road_heights=road_heights
        )

        # Check and resolve building obstacle collisions
        self._resolve_building_collisions()

        # Publish ROS 2 topics & synchronize Gazebo
        self._publish_telemetry()
        self._sync_with_gazebo()


def main(args=None):
    rclpy.init(args=args)
    node = Vehicle14DoFNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

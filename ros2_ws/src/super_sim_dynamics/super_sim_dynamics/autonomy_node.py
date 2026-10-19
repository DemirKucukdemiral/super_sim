#!/usr/bin/env python3
"""ROS adapter: odometry -> controller -> existing vehicle actuator topic."""
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from super_sim_dynamics.autonomy import Observation, DriveCommand, load_controller


class AutonomyNode(Node):
    def __init__(self):
        super().__init__("super_sim_autonomy")
        self.declare_parameter("controller", "super_sim_dynamics.controllers:StopController")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("odom_timeout_s", 0.5)
        rate = float(self.get_parameter("rate_hz").value)
        self.timeout = float(self.get_parameter("odom_timeout_s").value)
        if not math.isfinite(rate) or rate <= 0 or not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("rate_hz and odom_timeout_s must be finite and positive")
        self.controller = load_controller(self.get_parameter("controller").value)
        self.observation = None
        self.received_at = 0.0
        self.previous_tick = time.monotonic()
        self.failed = False
        self.publisher = self.create_publisher(Float32MultiArray, "/my_car/drive_request", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_timer(1.0 / rate, self.tick)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        values = (p.x, p.y, yaw, msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        if all(math.isfinite(v) for v in values):
            self.observation = Observation(*values)
            self.received_at = time.monotonic()
        else:
            self.observation = None

    def tick(self):
        now = time.monotonic()
        dt, self.previous_tick = now - self.previous_tick, now
        command = DriveCommand.stop()
        if not self.failed and self.observation is not None and now - self.received_at <= self.timeout:
            try:
                command = self.controller.step(self.observation, dt).limited()
            except Exception as error:
                self.failed = True
                self.get_logger().error(f"Controller stopped: {error}. Restart the node after fixing it.")
        self.publisher.publish(Float32MultiArray(data=command.as_array()))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AutonomyNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publisher.publish(Float32MultiArray(data=DriveCommand.stop().as_array()))
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

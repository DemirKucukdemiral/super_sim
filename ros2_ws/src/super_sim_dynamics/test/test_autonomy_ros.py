"""Adapter tests; skipped when ROS is unavailable, run inside the sourced ROS environment."""
import math
from unittest.mock import Mock
import pytest

rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry
from super_sim_dynamics.autonomy import DriveCommand
from super_sim_dynamics.autonomy_node import AutonomyNode


@pytest.fixture
def node():
    rclpy.init()
    instance = AutonomyNode()
    instance.publisher = Mock()
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def command(node):
    return node.publisher.publish.call_args.args[0].data.tolist()


def test_missing_and_stale_odometry_brakes(node):
    node.tick()
    assert command(node) == [0., 1., 0., 0.]
    node.controller = Mock()
    node.controller.step.return_value = DriveCommand(throttle=.2)
    msg = Odometry()
    msg.pose.pose.orientation.w = 1.
    node.on_odom(msg)
    node.tick()
    assert command(node)[0] == pytest.approx(.2)
    node.received_at -= 1.
    node.tick()
    assert command(node) == [0., 1., 0., 0.]
    assert node.controller.step.call_count == 1


def test_controller_failure_latches_until_restart(node):
    node.on_odom(Odometry())
    node.controller = Mock()
    node.controller.step.side_effect = RuntimeError('test failure')
    node.tick()
    node.tick()
    assert node.failed
    assert node.controller.step.call_count == 1
    assert command(node) == [0., 1., 0., 0.]


def test_odometry_fields_and_invalid_data(node):
    msg = Odometry()
    msg.pose.pose.position.x = 4.
    msg.pose.pose.orientation.z = math.sin(.5)
    msg.pose.pose.orientation.w = math.cos(.5)
    msg.twist.twist.linear.x = 3.
    node.on_odom(msg)
    assert node.observation.yaw == pytest.approx(1.)
    assert node.observation.speed == 3.
    msg.pose.pose.position.x = math.nan
    node.on_odom(msg)
    node.tick()
    assert node.observation is None
    assert command(node) == [0., 1., 0., 0.]

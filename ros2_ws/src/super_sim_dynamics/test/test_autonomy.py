import math
import pytest
from super_sim_dynamics.autonomy import DriveCommand, Observation, load_controller


@pytest.mark.parametrize("values", [[], [1, 2], [0, 0, math.nan], [math.inf, 0, 0], [0, 0, 0, math.nan]])
def test_rejects_malformed_commands(values):
    with pytest.raises(ValueError):
        DriveCommand.from_array(values)


def test_limits_actuators_and_brake_overrides_throttle():
    assert DriveCommand(2, 0, -9).limited().as_array() == [1., 0., -.65, 0.]
    assert DriveCommand(1, 2, 9).limited().as_array() == [0., 1., .65, 0.]
    assert DriveCommand(1, 0, 0, True).limited().throttle == 0


def test_three_value_command_releases_handbrake():
    assert not DriveCommand.from_array([0, 0, 0]).handbrake


def test_nonfinite_plugin_output_is_rejected():
    with pytest.raises(ValueError):
        DriveCommand(steering=math.nan).limited()


def test_default_plugin_holds_vehicle():
    controller = load_controller("super_sim_dynamics.controllers:StopController")
    assert controller.step(Observation(0, 0, 0, 0, 0), .05) == DriveCommand.stop()


def test_cruise_plugin_brakes_above_target():
    controller = load_controller("super_sim_dynamics.controllers:CruiseController")
    command = controller.step(Observation(0, 0, 0, 5, 0), .05).limited()
    assert command.brake > 0 and command.throttle == 0


def test_bad_plugin_spec_has_clear_error():
    with pytest.raises(ValueError, match="module:Class"):
        load_controller("missing_separator")

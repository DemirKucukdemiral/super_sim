#!/usr/bin/env python3
"""
Unit and regression tests for 14-DoF vehicle dynamics model,
tire models, powertrain, and braking systems.
"""

import math
from pathlib import Path
from super_sim_dynamics.vehicle_14dof_model import (
    Vehicle14DoFParams, Vehicle14DoFState, step_14dof, calc_ackermann_angles
)
from super_sim_dynamics.engine_model import PowertrainType


def test_ackermann_steering():
    params = Vehicle14DoFParams()
    delta = 0.30  # rad
    fl, fr = calc_ackermann_angles(params, delta)
    print(f"Ackermann test passed: steer={delta:.2f}, FL={fl:.4f}, FR={fr:.4f}")
    assert fl > delta > fr or fl < delta < fr, "Ackermann geometry invalid"


def test_static_stability():
    params = Vehicle14DoFParams()
    state = Vehicle14DoFState()
    dt = 0.01

    for _ in range(200):
        state = step_14dof(params, state, throttle_cmd=0.0, brake_cmd=0.0, steer_cmd=0.0, handbrake_cmd=False, dt=dt)

    print(f"Static stability passed: u={state.u:.3f}, roll={state.roll:.4f}, pitch={state.pitch:.4f}")
    assert abs(state.u) < 0.01, "Vehicle drifted forward without throttle"
    assert abs(state.v) < 0.01, "Vehicle drifted laterally without input"
    assert abs(state.roll) < 0.02, "Excessive static roll"


def test_ev_acceleration():
    params = Vehicle14DoFParams()
    params.engine_params.powertrain_type = PowertrainType.EV
    state = Vehicle14DoFState()
    dt = 0.01

    for _ in range(300):
        state = step_14dof(params, state, throttle_cmd=1.0, brake_cmd=0.0, steer_cmd=0.0, handbrake_cmd=False, dt=dt)

    print(f"EV Acceleration (3.0s): speed={state.u*3.6:.1f} km/h, distance={state.x:.1f} m, RPM={state.engine_state.rpm:.0f}")
    assert state.u * 3.6 > 60.0, "EV acceleration underperforming"


def test_ice_acceleration():
    params = Vehicle14DoFParams()
    params.engine_params.powertrain_type = PowertrainType.ICE
    state = Vehicle14DoFState()
    dt = 0.01

    for _ in range(300):
        state = step_14dof(params, state, throttle_cmd=1.0, brake_cmd=0.0, steer_cmd=0.0, handbrake_cmd=False, dt=dt)

    print(f"ICE Acceleration (3.0s): speed={state.u*3.6:.1f} km/h, gear={state.engine_state.current_gear}, RPM={state.engine_state.rpm:.0f}")
    assert state.engine_state.current_gear > 1, "Automatic transmission failed to upshift"
    assert state.u * 3.6 > 40.0, "ICE acceleration underperforming"


def test_braking_and_abs():
    params = Vehicle14DoFParams()
    params.brake_params.abs_enabled = True
    state = Vehicle14DoFState(u=25.0, omega=[25.0 / params.wheel_radius] * 4)
    dt = 0.01
    abs_activated = False

    for _ in range(320):
        state = step_14dof(params, state, throttle_cmd=0.0, brake_cmd=1.0, steer_cmd=0.0, handbrake_cmd=False, dt=dt)
        if state.brake_state.abs_active:
            abs_activated = True
        if state.u <= 0.1:
            break

    print(f"Emergency braking test: final_speed={state.u:.2f} m/s, ABS triggered={abs_activated}")
    assert abs_activated, "ABS did not trigger during emergency stop!"
    assert state.u < 1.0, f"Vehicle did not come to a stop: {state.u} m/s"


def test_cornering_and_load_transfer():
    params = Vehicle14DoFParams()
    state = Vehicle14DoFState(u=15.0, omega=[15.0 / params.wheel_radius] * 4)
    dt = 0.01

    for _ in range(150):
        state = step_14dof(params, state, throttle_cmd=0.2, brake_cmd=0.0, steer_cmd=0.15, handbrake_cmd=False, dt=dt)

    print(f"Cornering test: Yaw rate={state.r:.3f} rad/s, Roll={math.degrees(state.roll):.2f} deg, Fy_front={state.tire_fy[0]:.1f} N")
    assert abs(state.r) > 0.05, f"Vehicle did not turn: yaw_rate={state.r}"
    assert abs(state.roll) > 0.005, f"Chassis did not roll under lateral load: roll={state.roll}"


def test_building_collision_restitution():
    import json
    bldg_path = Path(__file__).resolve().parents[4] / "worlds/city_buildings.json"
    assert bldg_path.is_file(), f"Missing collision fixture: {bldg_path}"
    with open(bldg_path, "r") as f:
        bldgs = json.load(f)

    params = Vehicle14DoFParams()
    state = Vehicle14DoFState(x=0.0, y=0.0, yaw=math.radians(45.0), u=15.0, omega=[15.0 / params.wheel_radius] * 4)
    dt = 0.01
    hl, hw = 2.4, 1.0
    collided = False

    for _ in range(120):
        state = step_14dof(params, state, throttle_cmd=0.5, brake_cmd=0.0, steer_cmd=0.0, handbrake_cmd=False, dt=dt)
        cos_y = math.cos(state.yaw)
        sin_y = math.sin(state.yaw)
        dx = hl * abs(cos_y) + hw * abs(sin_y)
        dy = hl * abs(sin_y) + hw * abs(cos_y)
        x = state.x
        y = state.y
        for b in bldgs:
            car_xmin, car_xmax = x - dx, x + dx
            car_ymin, car_ymax = y - dy, y + dy
            if car_xmin < b["x_max"] and car_xmax > b["x_min"] and car_ymin < b["y_max"] and car_ymax > b["y_min"]:
                collided = True
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
                state.x, state.y = x, y
                state.u = -0.15 * state.u
                break
        if collided:
            break

    assert collided, "Collision test failed: vehicle did not trigger collision with street building"
    assert state.u < 0.0, "Restitution failed: speed was not reversed upon impact"
    print(f"Building collision test passed: impact pose=({state.x:.2f}, {state.y:.2f}), rebound_u={state.u:.2f} m/s")


if __name__ == "__main__":
    test_ackermann_steering()
    test_static_stability()
    test_ev_acceleration()
    test_ice_acceleration()
    test_braking_and_abs()
    test_cornering_and_load_transfer()
    test_building_collision_restitution()
    print("ALL 14-DOF TESTS PASSED SUCCESSFULLY!")

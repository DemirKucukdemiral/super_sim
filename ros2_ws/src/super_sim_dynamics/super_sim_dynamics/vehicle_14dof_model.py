"""
Full 14 Degrees of Freedom (14-DoF) Vehicle Dynamics Model.
- 6-DoF Sprung Mass (Chassis 3D translation & rotation: Surge, Sway, Heave, Roll, Pitch, Yaw)
- 4-DoF Unsprung Masses (Independent 4-corner suspension heave with Anti-Roll Bars)
- 4-DoF Wheel Rotational Dynamics (continuously integrated wheel spin, Pacejka + friction ellipse)

Conventions (see VEHICLE_DYNAMICS.md):
- Body frame: +X forward, +Y left, +Z up (right-handed).
- Roll phi > 0: right side down. Pitch theta > 0: nose down. Yaw psi > 0: counter-clockwise.
"""

import math
from dataclasses import dataclass, field
from typing import Optional
from super_sim_dynamics.tire_model import (
    TireParams, calc_pure_longitudinal_force, calc_pure_lateral_force,
    calc_longitudinal_slip_stiffness, friction_ellipse_scale
)
from super_sim_dynamics.engine_model import EngineParams, EngineState, update_engine
from super_sim_dynamics.braking_model import BrakeParams, BrakeState, update_braking


@dataclass
class Vehicle14DoFParams:
    # Mass and Inertia
    sprung_mass: float = 1550.0             # Chassis mass [kg]
    unsprung_mass_front: float = 38.0       # Unsprung mass per front corner [kg]
    unsprung_mass_rear: float = 36.0        # Unsprung mass per rear corner [kg]
    Ixx: float = 520.0                      # Roll inertia [kg*m^2]
    Iyy: float = 2400.0                     # Pitch inertia [kg*m^2]
    Izz: float = 3500.0                     # Yaw inertia [kg*m^2]
    Ixz: float = 0.0                        # Cross-product inertia [kg*m^2] (unused, principal axes assumed)

    # Geometry
    wheelbase: float = 3.0832               # Total wheelbase L = a + b [m]
    dist_cg_front: float = 1.5416           # Distance from CG to front axle a [m]
    dist_cg_rear: float = 1.5416            # Distance from CG to rear axle b [m]
    track_width_front: float = 1.8565       # Front track width Wf [m]
    track_width_rear: float = 1.8565        # Rear track width Wr [m]
    cg_height_above_ground: float = 0.52    # CG height above ground h_cg [m]
    wheel_radius: float = 0.56985           # Wheel unloaded radius R_w [m]
    wheel_inertia: float = 1.80             # Wheel rotational inertia I_w [kg*m^2]
    suspension_mount_z: float = 0.0         # Chassis suspension hardpoint z relative to body origin z_s^B [m]
    nominal_ride_height: float = 0.52    # Design height z_0 of the body reference point [m]

    # Suspension Parameters (Corner stiffness & damping)
    k_spring_front: float = 45000.0         # Front suspension spring rate [N/m]
    k_spring_rear: float = 42000.0          # Rear suspension spring rate [N/m]
    c_bump_front: float = 3200.0            # Front bump damping coefficient [N*s/m]
    c_rebound_front: float = 4800.0         # Front rebound damping coefficient [N*s/m]
    c_bump_rear: float = 2800.0             # Rear bump damping coefficient [N*s/m]
    c_rebound_rear: float = 4200.0          # Rear rebound damping coefficient [N*s/m]
    k_arb_front: float = 12000.0            # Front Anti-Roll Bar stiffness [N/m]
    k_arb_rear: float = 8000.0              # Rear Anti-Roll Bar stiffness [N/m]
    tire_vertical_stiffness: float = 180000.0 # Tire radial vertical spring rate [N/m]
    tire_vertical_damping: float = 1800.0   # Tire radial damping [N*s/m]

    # Aerodynamics
    air_density: float = 1.225              # Air density [kg/m^3]
    drag_coeff: float = 0.32                # Cd
    frontal_area: float = 2.25              # A [m^2]
    downforce_coeff: float = 0.45           # C_L (negative lift)
    aero_front_balance: float = 0.48        # beta_aero: fraction of downforce acting at front axle
    aero_drag_height: float = 0.0           # Height of drag line of action above CG z_aero [m]

    # Steering
    max_steer_angle_rad: float = 0.65       # Maximum front steering lock (~37 deg)
    steer_time_constant: float = 0.08       # Steering actuator lag [s]

    # Traction control (drive-torque reduction on excessive positive slip)
    traction_control_enabled: bool = True
    tc_slip_target: float = 0.10            # kappa above which drive torque is cut
    tc_slip_band: float = 0.10              # slip band over which torque falls to tc_min_scale
    tc_min_scale: float = 0.10              # minimum fraction of drive torque retained

    # Numerics
    slip_velocity_regularization: float = 0.5  # v_reg used in slip denominators [m/s]

    # Sub-models
    tire_params: TireParams = field(default_factory=TireParams)
    engine_params: EngineParams = field(default_factory=EngineParams)
    brake_params: BrakeParams = field(default_factory=BrakeParams)


def _static_equilibrium(params: Vehicle14DoFParams) -> tuple[list[float], list[float]]:
    """Returns (z_u0, delta_z_nom) for the design ride height on flat ground."""
    g = 9.81
    a, b = params.dist_cg_front, params.dist_cg_rear
    ms_f = (b / (a + b)) * params.sprung_mass * 0.5
    ms_r = (a / (a + b)) * params.sprung_mass * 0.5
    kt = params.tire_vertical_stiffness
    dz_t_f = (ms_f + params.unsprung_mass_front) * g / kt
    dz_t_r = (ms_r + params.unsprung_mass_rear) * g / kt
    Rw = params.wheel_radius
    zu0 = [Rw - dz_t_f, Rw - dz_t_f, Rw - dz_t_r, Rw - dz_t_r]
    zc0 = params.nominal_ride_height + params.suspension_mount_z
    return zu0, [zu0[i] - zc0 for i in range(4)]


_DEFAULT_ZU0, _ = _static_equilibrium(Vehicle14DoFParams())


@dataclass
class Vehicle14DoFState:
    # 6-DoF Sprung Mass States: Position and Euler Angles
    x: float = 0.0                          # World X position [m]
    y: float = 0.0                          # World Y position [m]
    z: float = 0.52                         # World Z position of body reference point [m]
    roll: float = 0.0                       # Roll angle phi [rad] (right side down is positive)
    pitch: float = 0.0                      # Pitch angle theta [rad] (nose down is positive)
    yaw: float = 0.0                        # Yaw angle psi [rad]

    # 6-DoF Sprung Mass Velocities in Body Frame
    u: float = 0.0                          # Longitudinal velocity [m/s]
    v: float = 0.0                          # Lateral velocity [m/s]
    w: float = 0.0                          # Vertical heave velocity [m/s]
    p: float = 0.0                          # Roll rate [rad/s]
    q: float = 0.0                          # Pitch rate [rad/s]
    r: float = 0.0                          # Yaw rate [rad/s]

    # 4-DoF Unsprung Suspension Mass States (FL, FR, RL, RR)
    # Wheel hub world elevations [m] (default: static equilibrium on flat ground)
    z_u: list[float] = field(default_factory=lambda: list(_DEFAULT_ZU0))
    # Wheel hub vertical velocities [m/s]
    dz_u: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])

    # 4-DoF Wheel Rotational States (FL, FR, RL, RR) [rad/s]
    omega: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])

    # Steering state
    steer_angle: float = 0.0                # Actual steering angle [rad]

    # Sub-model internal states
    engine_state: EngineState = field(default_factory=EngineState)
    brake_state: BrakeState = field(default_factory=BrakeState)

    # Road heights under the 4 tires at the previous outer step (for z_road rate)
    road_z_prev: Optional[list[float]] = None

    # Telemetry data updated every step
    wheel_slips: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    wheel_slip_angles: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    normal_forces: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    tire_fx: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    tire_fy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    suspension_deflections: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])


def calc_ackermann_angles(params: Vehicle14DoFParams, delta: float) -> tuple[float, float]:
    """Calculates left and right front wheel steer angles based on Ackermann geometry."""
    if abs(delta) < 1e-5:
        return delta, delta
    L = params.wheelbase
    W = params.track_width_front
    tan_d = math.tan(delta)
    delta_fl = math.atan((L * tan_d) / (L - 0.5 * W * tan_d))
    delta_fr = math.atan((L * tan_d) / (L + 0.5 * W * tan_d))
    return delta_fl, delta_fr


def step_14dof(
    params: Vehicle14DoFParams,
    state: Vehicle14DoFState,
    throttle_cmd: float,
    brake_cmd: float,
    steer_cmd: float,
    handbrake_cmd: bool,
    dt: float,
    road_heights: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
) -> Vehicle14DoFState:
    """Advances the 14-DoF vehicle dynamics model forward by time dt."""
    num_substeps = 4
    sub_dt = dt / num_substeps

    road_new = list(road_heights)
    road_old = list(state.road_z_prev) if state.road_z_prev is not None else list(road_new)
    # Rate of change of terrain height sampled under each moving tire
    dz_road = [(road_new[i] - road_old[i]) / dt for i in range(4)] if dt > 0.0 else [0.0] * 4

    current_state = state
    for k in range(num_substeps):
        frac = (k + 1) / num_substeps
        road_sub = tuple(road_old[i] + frac * (road_new[i] - road_old[i]) for i in range(4))
        current_state = _integrate_single_substep(
            params, current_state, throttle_cmd, brake_cmd, steer_cmd, handbrake_cmd,
            sub_dt, road_sub, dz_road
        )
    current_state.road_z_prev = road_new
    return current_state


def _integrate_single_substep(
    params: Vehicle14DoFParams,
    s: Vehicle14DoFState,
    throttle_cmd: float,
    brake_cmd: float,
    steer_cmd: float,
    handbrake_cmd: bool,
    dt: float,
    road_z: tuple[float, float, float, float],
    dz_road: list[float]
) -> Vehicle14DoFState:
    g = 9.81
    a = params.dist_cg_front
    b = params.dist_cg_rear
    wf = params.track_width_front * 0.5
    wr = params.track_width_rear * 0.5
    Rw = params.wheel_radius
    zs = params.suspension_mount_z
    m_s = params.sprung_mass
    mu_f = params.unsprung_mass_front
    mu_r = params.unsprung_mass_rear
    m_total = m_s + 2.0 * mu_f + 2.0 * mu_r
    m_u = [mu_f, mu_f, mu_r, mu_r]
    tp = params.tire_params
    v_reg = params.slip_velocity_regularization

    # 1. Steering actuator dynamics: delta_sat = clamp(delta_cmd, +/-delta_max), delta_{k+1} = delta_k + (1 - exp(-dt/tau))*(delta_sat - delta_k)
    delta_sat = max(-params.max_steer_angle_rad, min(params.max_steer_angle_rad, steer_cmd))
    alpha_steer = 1.0 - math.exp(-dt / params.steer_time_constant)
    new_steer = s.steer_angle + alpha_steer * (delta_sat - s.steer_angle)
    delta_fl, delta_fr = calc_ackermann_angles(params, new_steer)
    delta_wheels = [delta_fl, delta_fr, 0.0, 0.0]

    # 2. Orientation, Euler-angle rates and world vertical velocity
    phi, th = s.roll, s.pitch
    sphi, cphi = math.sin(phi), math.cos(phi)
    sth, cth = math.sin(th), math.cos(th)
    cth_safe = max(0.01, cth)
    phi_dot = s.p + (s.q * sphi + s.r * cphi) * (sth / cth_safe)
    theta_dot = s.q * cphi - s.r * sphi
    z_dot = -sth * s.u + cth * sphi * s.v + cth * cphi * s.w    # third row of R_WB

    # 3. Suspension hardpoint kinematics (exact, from p_c = p_cg + R_WB r_i)
    hp_x = [a, a, -b, -b]
    hp_y = [wf, -wf, wr, -wr]
    z_c = [0.0] * 4
    dz_c = [0.0] * 4
    for i in range(4):
        xi, yi = hp_x[i], hp_y[i]
        z_c[i] = s.z - xi * sth + yi * sphi * cth + zs * cth * cphi
        dz_c[i] = (z_dot
                   - xi * cth * theta_dot
                   + yi * (cphi * cth * phi_dot - sphi * sth * theta_dot)
                   - zs * (sth * cphi * theta_dot + cth * sphi * phi_dot))

    # 4. Suspension deflection relative to design ride height, preload, damping, ARB
    _, dz_nom = _static_equilibrium(params)
    delta_z = [(s.z_u[i] - z_c[i]) - dz_nom[i] for i in range(4)]      # > 0: jounce
    delta_dz = [s.dz_u[i] - dz_c[i] for i in range(4)]

    ms_f = (b / (a + b)) * m_s * 0.5
    ms_r = (a / (a + b)) * m_s * 0.5
    preload = [ms_f * g, ms_f * g, ms_r * g, ms_r * g]
    k_s = [params.k_spring_front, params.k_spring_front, params.k_spring_rear, params.k_spring_rear]
    c_bump = [params.c_bump_front, params.c_bump_front, params.c_bump_rear, params.c_bump_rear]
    c_reb = [params.c_rebound_front, params.c_rebound_front, params.c_rebound_rear, params.c_rebound_rear]

    # ARB: positive roll (right down) -> dz_FL < dz_FR -> F_arb < 0 -> unload left, load right (restoring)
    f_arb_front = params.k_arb_front * (delta_z[0] - delta_z[1])
    f_arb_rear = params.k_arb_rear * (delta_z[2] - delta_z[3])
    arb = [+f_arb_front, -f_arb_front, +f_arb_rear, -f_arb_rear]

    f_susp = [0.0] * 4
    for i in range(4):
        c_i = c_bump[i] if delta_dz[i] >= 0.0 else c_reb[i]
        f_susp[i] = preload[i] + k_s[i] * delta_z[i] + c_i * delta_dz[i] + arb[i]

    # 5. Aerodynamics (forces on the sprung mass)
    v2 = s.u * s.u + s.v * s.v
    F_drag = 0.5 * params.air_density * params.drag_coeff * params.frontal_area * v2
    F_downforce = 0.5 * params.air_density * params.downforce_coeff * params.frontal_area * v2
    F_aero_f = params.aero_front_balance * F_downforce
    F_aero_r = (1.0 - params.aero_front_balance) * F_downforce

    # 6. Tire vertical contact force = normal load (unilateral, cannot pull the wheel down)
    kt = params.tire_vertical_stiffness
    ct = params.tire_vertical_damping
    Fz = [0.0] * 4
    for i in range(4):
        dz_tire = (road_z[i] + Rw) - s.z_u[i]
        if dz_tire > 0.0:
            d_dz_tire = dz_road[i] - s.dz_u[i]
            Fz[i] = max(0.0, kt * dz_tire + ct * d_dz_tire)

    # 7. Wheel hub kinematics and slip angles
    vx_b = [s.u - wf * s.r, s.u + wf * s.r, s.u - wr * s.r, s.u + wr * s.r]
    vy_b = [s.v + a * s.r, s.v + a * s.r, s.v - b * s.r, s.v - b * s.r]
    vx_wheel = [0.0] * 4
    slip_angles = [0.0] * 4
    for i in range(4):
        cd, sd = math.cos(delta_wheels[i]), math.sin(delta_wheels[i])
        vx_wh = vx_b[i] * cd + vy_b[i] * sd
        vy_wh = -vx_b[i] * sd + vy_b[i] * cd
        vx_wheel[i] = vx_wh
        slip_angles[i] = -math.atan2(vy_wh, max(abs(vx_wh), v_reg))

    # 8. Powertrain
    wheel_rpms = tuple(abs(w) * (30.0 / math.pi) for w in s.omega)
    new_engine_state, T_drive_t = update_engine(
        params.engine_params, s.engine_state, throttle_cmd, wheel_rpms, dt
    )
    T_drive = list(T_drive_t)
    if params.traction_control_enabled:
        for i in range(4):
            k_prev = s.wheel_slips[i]
            if T_drive[i] * k_prev > 0.0 and abs(k_prev) > params.tc_slip_target:
                cut = 1.0 - (abs(k_prev) - params.tc_slip_target) / params.tc_slip_band
                T_drive[i] *= max(params.tc_min_scale, min(1.0, cut))

    # 9. Brakes & ABS (uses slip ratios from the previous substep)
    new_brake_state, T_brake_t = update_braking(
        params.brake_params, s.brake_state, brake_cmd, handbrake_cmd,
        tuple(s.wheel_slips), wheel_rpms, abs(s.u), dt
    )
    T_brake = list(T_brake_t)

    # 10. Tire forces (Pacejka + friction ellipse) and continuous wheel spin integration
    #     Linearly-implicit Euler in omega: stable for the stiff tire/wheel mode.
    Fx_w = [0.0] * 4
    Fy_w = [0.0] * 4
    new_omega = [0.0] * 4
    slips = [0.0] * 4
    Iw = params.wheel_inertia
    for i in range(4):
        if Fz[i] < tp.eps_F:
            # Explicit airborne guard: normal load below threshold yields zero tire contact forces
            Fx_w[i] = 0.0
            Fy_w[i] = 0.0
            w0 = s.omega[i]
            w_free = w0 + dt * T_drive[i] / Iw
            impulse = dt * T_brake[i] / Iw
            new_omega[i] = 0.0 if abs(w_free) <= impulse else w_free - math.copysign(impulse, w_free)
            slips[i] = 0.0
            continue

        vx = vx_wheel[i]
        v_den = max(abs(vx), v_reg)
        w0 = s.omega[i]
        kappa0 = (Rw * w0 - vx) / v_den

        Fx0 = calc_pure_longitudinal_force(tp, Fz[i], kappa0)
        Fy0 = calc_pure_lateral_force(tp, Fz[i], slip_angles[i])
        lam = friction_ellipse_scale(tp, Fz[i], Fx0, Fy0)

        # dFx/d(omega) of the (scaled) tire force, clipped >= 0 for unconditional stability
        k_w = lam * max(0.0, calc_longitudinal_slip_stiffness(tp, Fz[i], kappa0)) * Rw / v_den
        denom = Iw + dt * Rw * k_w

        # Drive torque + tire reaction (implicit)
        w_free = w0 + dt * (T_drive[i] - Rw * lam * Fx0) / denom
        # Resistive torques (brake + rolling resistance) oppose rotation and cannot reverse it
        T_rr = (tp.c_rr0 + tp.c_rr1 * vx * vx) * Fz[i] * Rw
        impulse = dt * (T_brake[i] + T_rr) / denom
        if abs(w_free) <= impulse:
            w1 = 0.0
        else:
            w1 = w_free - math.copysign(impulse, w_free)

        Fx = lam * Fx0 + k_w * (w1 - w0)
        Fy = lam * Fy0
        # Re-enforce friction ellipse after the linearised update
        lam2 = friction_ellipse_scale(tp, Fz[i], Fx, Fy)
        Fx_w[i] = lam2 * Fx
        Fy_w[i] = lam2 * Fy
        new_omega[i] = w1
        slips[i] = max(-1.0, min(1.0, (Rw * w1 - vx) / v_den))

    # 11. Unsprung mass heave
    new_z_u = [0.0] * 4
    new_dz_u = [0.0] * 4
    for i in range(4):
        ddz_u = (Fz[i] - f_susp[i] - m_u[i] * g) / m_u[i]
        new_dz_u[i] = s.dz_u[i] + ddz_u * dt
        new_z_u[i] = s.z_u[i] + new_dz_u[i] * dt

    # 12. Tire forces into body frame
    Fx_b = [0.0] * 4
    Fy_b = [0.0] * 4
    for i in range(4):
        cd, sd = math.cos(delta_wheels[i]), math.sin(delta_wheels[i])
        Fx_b[i] = Fx_w[i] * cd - Fy_w[i] * sd
        Fy_b[i] = Fx_w[i] * sd + Fy_w[i] * cd

    sum_Fx_tires = sum(Fx_b)
    sum_Fy_tires = sum(Fy_b)
    drag_x = -math.copysign(F_drag, s.u) if abs(s.u) > 1e-6 else 0.0

    # 13. Chassis translational dynamics
    # Horizontal: unsprung masses move with the body -> total mass. Vertical: sprung mass only.
    du = (sum_Fx_tires + drag_x) / m_total + s.v * s.r - s.w * s.q + g * sth
    dv = sum_Fy_tires / m_total - s.u * s.r + s.w * s.p - g * cth * sphi
    dw = (sum(f_susp) - F_downforce) / m_s + s.u * s.q - s.v * s.p - g * cth * cphi

    # 14. Chassis rotational dynamics (moments about CG, M = r x F)
    hcg = params.cg_height_above_ground
    Mx = (wf * (f_susp[0] - f_susp[1]) + wr * (f_susp[2] - f_susp[3])
          + hcg * sum_Fy_tires)
    My = (-a * (f_susp[0] + f_susp[1]) + b * (f_susp[2] + f_susp[3])
          - hcg * sum_Fx_tires
          + a * F_aero_f - b * F_aero_r
          + params.aero_drag_height * drag_x)
    Mz = (a * (Fy_b[0] + Fy_b[1]) - b * (Fy_b[2] + Fy_b[3])
          + wf * (Fx_b[1] - Fx_b[0]) + wr * (Fx_b[3] - Fx_b[2]))

    dp = (Mx - (params.Izz - params.Iyy) * s.q * s.r) / params.Ixx
    dq = (My - (params.Ixx - params.Izz) * s.p * s.r) / params.Iyy
    dr = (Mz - (params.Iyy - params.Ixx) * s.p * s.q) / params.Izz

    new_u = s.u + du * dt
    new_v = s.v + dv * dt
    new_w = s.w + dw * dt
    new_p = s.p + dp * dt
    new_q = s.q + dq * dt
    new_r = s.r + dr * dt

    # Numerical low-speed deadband (standstill numerical stabilization, not physical static friction)
    # Applied when vehicle is stopped with brakes engaged or on near-level ground to prevent limit-cycle chatter.
    v_body = math.hypot(s.u, s.v)
    has_braking = (max(T_brake) > 50.0) or handbrake_cmd
    if v_body < 0.08 and max(abs(t) for t in T_drive) < 10.0 and (has_braking or abs(sth) < 0.01):
        new_u = 0.0
        new_v = 0.0
        new_r = 0.0

    # 15. Kinematics (semi-implicit Euler with updated rates)
    d_phi = new_p + (new_q * sphi + new_r * cphi) * (sth / cth_safe)
    d_theta = new_q * cphi - new_r * sphi
    d_psi = (new_q * sphi + new_r * cphi) / cth_safe

    new_roll = s.roll + d_phi * dt
    new_pitch = s.pitch + d_theta * dt
    new_yaw = s.yaw + d_psi * dt

    cy, sy = math.cos(new_yaw), math.sin(new_yaw)
    cp, sp = math.cos(new_pitch), math.sin(new_pitch)
    cr, sr = math.cos(new_roll), math.sin(new_roll)

    dx_world = (cy * cp) * new_u + (cy * sp * sr - sy * cr) * new_v + (cy * sp * cr + sy * sr) * new_w
    dy_world = (sy * cp) * new_u + (sy * sp * sr + cy * cr) * new_v + (sy * sp * cr - cy * sr) * new_w
    dz_world = (-sp) * new_u + (cp * sr) * new_v + (cp * cr) * new_w

    return Vehicle14DoFState(
        x=s.x + dx_world * dt, y=s.y + dy_world * dt, z=s.z + dz_world * dt,
        roll=new_roll, pitch=new_pitch, yaw=new_yaw,
        u=new_u, v=new_v, w=new_w, p=new_p, q=new_q, r=new_r,
        z_u=new_z_u, dz_u=new_dz_u, omega=new_omega, steer_angle=new_steer,
        engine_state=new_engine_state, brake_state=new_brake_state,
        road_z_prev=s.road_z_prev,
        wheel_slips=slips, wheel_slip_angles=slip_angles, normal_forces=Fz,
        tire_fx=Fx_w, tire_fy=Fy_w, suspension_deflections=delta_z
    )

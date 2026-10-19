"""
Powertrain and Drivetrain Model for Electric Autonomous Vehicle.
Electric Motor (EV) with single-speed reduction gearing, overspeed cut,
and smooth speed-sensitive Limited-Slip Differentials (LSD).
"""

import math
from dataclasses import dataclass, field
from enum import Enum


class PowertrainType(Enum):
    EV = "EV"
    ICE = "ICE"


class DrivetrainLayout(Enum):
    AWD = "AWD"
    RWD = "RWD"
    FWD = "FWD"


@dataclass
class EngineParams:
    powertrain_type: PowertrainType = PowertrainType.EV
    drivetrain_layout: DrivetrainLayout = DrivetrainLayout.AWD
    
    # EV Motor Specifications (Total Vehicle Output)
    ev_max_torque: float = 950.0          # Peak total motor torque [N*m] across axles
    ev_max_power: float = 650000.0        # Peak total motor power [W] (650 kW / ~870 hp)
    ev_base_rpm: float = 6500.0           # Base speed where constant power starts [RPM]
    ev_max_rpm: float = 20000.0           # Maximum motor speed / overspeed cutoff [RPM]
    ev_gear_ratio: float = 10.5           # High-efficiency reduction gear ratio G_ev
    ev_regen_max_torque: float = 320.0    # Peak regenerative braking torque [N*m]
    
    # ICE Engine Specifications (Fallback / Alternative)
    ice_idle_rpm: float = 850.0           # Idle speed [RPM]
    ice_redline_rpm: float = 8000.0       # Redline engine speed [RPM]
    ice_peak_torque: float = 680.0        # Peak engine torque [N*m] @ 4600 RPM
    ice_peak_power_rpm: float = 7200.0    # Engine RPM for peak horsepower
    ice_inertia: float = 0.18             # Flywheel & engine rotating inertia [kg*m^2]
    ice_gear_ratios: list[float] = field(default_factory=lambda: [-3.40, 0.0, 3.82, 2.20, 1.52, 1.16, 0.91, 0.76])
    ice_final_drive: float = 4.10         # Final drive ratio
    
    # Differential & Efficiency
    driveline_efficiency: float = 0.96    # Mechanical driveline efficiency eta
    lsd_bias_ratio: float = 0.40          # Limited Slip Differential locking bias lambda_lsd
    lsd_smoothing_gain: float = 0.5       # Speed-sensitive LSD smoothing factor k_lsd [s/rad]
    awd_front_split: float = 0.50         # Front/Rear torque split for AWD (50% front, 50% rear)
    
    # Response Time Constants
    throttle_tau: float = 0.01            # Throttle actuator first-order lag [s]


@dataclass
class EngineState:
    rpm: float = 850.0                    # Powertrain equivalent motor / engine RPM
    current_gear: int = 1                 # Current transmission gear (-1: Rev, 0: N, 1..6)
    actual_throttle: float = 0.0          # Lagged throttle [0.0 .. 1.0]
    output_shaft_torque: float = 0.0      # Torque delivered to driveshaft [N*m]
    is_rev_limiting: bool = False         # Rev limiter / overspeed cut active


def get_ev_available_torque(params: EngineParams, motor_rpm: float) -> float:
    """
    Calculates maximum available total motor torque at given equivalent motor RPM:
    - Constant torque regime: T = T_max for RPM <= RPM_base
    - Constant power regime:  T = min(T_max, P_max / Omega_m) for RPM_base < RPM < RPM_max
    - Overspeed branch:       T = 0 for RPM >= RPM_max (cutoff)
    """
    rpm = max(0.0, abs(motor_rpm))
    if rpm <= params.ev_base_rpm:
        return params.ev_max_torque
    elif rpm < params.ev_max_rpm:
        omega = rpm * (2.0 * math.pi / 60.0)
        return min(params.ev_max_torque, params.ev_max_power / max(1.0, omega))
    else:
        # Overspeed fuel / power cutoff branch
        return 0.0


def get_ice_torque_curve(params: EngineParams, engine_rpm: float) -> float:
    """Calculates ICE engine brake torque vs RPM."""
    if engine_rpm < params.ice_idle_rpm * 0.8:
        return 50.0
    if engine_rpm >= params.ice_redline_rpm:
        return 0.0
    rpm_norm = (engine_rpm - 4400.0) / 3000.0
    factor = max(0.65, 1.0 - 0.45 * (rpm_norm ** 2))
    return params.ice_peak_torque * factor


def update_engine(
    params: EngineParams,
    state: EngineState,
    throttle_cmd: float,
    wheel_speeds_rpm: tuple[float, float, float, float],
    dt: float
) -> tuple[EngineState, tuple[float, float, float, float]]:
    """
    Advances engine/powertrain state and computes drive torque to each wheel:
    (T_fl, T_fr, T_rl, T_rr) [N*m].
    """
    throttle_cmd = max(-1.0, min(1.0, throttle_cmd))
    
    # 1. Throttle first-order lag: dot(alpha) = (alpha_cmd - alpha) / tau
    alpha = 1.0 - math.exp(-dt / params.throttle_tau)
    new_throttle = state.actual_throttle + alpha * (throttle_cmd - state.actual_throttle)
    
    fl_rpm, fr_rpm, rl_rpm, rr_rpm = wheel_speeds_rpm
    avg_front_rpm = (fl_rpm + fr_rpm) * 0.5
    avg_rear_rpm = (rl_rpm + rr_rpm) * 0.5
    
    if params.powertrain_type == PowertrainType.EV:
        # Motor speeds per axle: Omega_m,f = G_ev * (w_FL + w_FR)/2, Omega_m,r = G_ev * (w_RL + w_RR)/2
        motor_rpm_f = abs(avg_front_rpm) * params.ev_gear_ratio
        motor_rpm_r = abs(avg_rear_rpm) * params.ev_gear_ratio
        motor_rpm = (motor_rpm_f + motor_rpm_r) * 0.5 if params.drivetrain_layout == DrivetrainLayout.AWD else motor_rpm_r
        
        avail_torque = get_ev_available_torque(params, motor_rpm)
        motor_torque = new_throttle * avail_torque
        total_axle_torque = motor_torque * params.ev_gear_ratio * params.driveline_efficiency
        new_rpm = motor_rpm
        rev_limit = motor_rpm >= params.ev_max_rpm
        new_gear = -1 if new_throttle < -0.01 else 1
        
    else:
        # ICE implementation
        if new_throttle < -0.01:
            new_gear = -1
            gear_ratio = params.ice_gear_ratios[0]
            total_ratio = abs(gear_ratio) * params.ice_final_drive
            target_engine_rpm = max(params.ice_idle_rpm, abs(avg_rear_rpm) * total_ratio)
            new_rpm = target_engine_rpm
            rev_limit = new_rpm >= params.ice_redline_rpm
            eng_torque = get_ice_torque_curve(params, new_rpm) * abs(new_throttle)
            total_axle_torque = -eng_torque * total_ratio * params.driveline_efficiency
        else:
            new_gear = max(1, state.current_gear)
            gear_idx = new_gear + 1
            gear_ratio = params.ice_gear_ratios[gear_idx] if 0 <= gear_idx < len(params.ice_gear_ratios) else 1.0
            total_ratio = abs(gear_ratio) * params.ice_final_drive
            target_engine_rpm = max(params.ice_idle_rpm, abs(avg_rear_rpm) * total_ratio)
            new_rpm = target_engine_rpm
            rev_limit = new_rpm >= params.ice_redline_rpm
            if new_rpm > 5800.0 and new_gear < 6:
                new_gear += 1
            elif new_rpm < 2200.0 and new_gear > 1:
                new_gear -= 1
            eng_torque = get_ice_torque_curve(params, new_rpm) * new_throttle
            total_axle_torque = eng_torque * total_ratio * params.driveline_efficiency
    
    # 2. Distribute torque across axles (AWD 50:50 default)
    if params.drivetrain_layout == DrivetrainLayout.AWD:
        front_axle_torque = total_axle_torque * params.awd_front_split
        rear_axle_torque = total_axle_torque * (1.0 - params.awd_front_split)
    elif params.drivetrain_layout == DrivetrainLayout.RWD:
        front_axle_torque = 0.0
        rear_axle_torque = total_axle_torque
    else:
        front_axle_torque = total_axle_torque
        rear_axle_torque = 0.0
        
    # 3. Smooth Speed-Sensitive Limited Slip Differential (LSD)
    # T_left/right = T_axle * (0.5 +/- lambda_lsd * tanh(k_lsd * Delta_Omega))
    d_omega_f = (fr_rpm - fl_rpm) * (math.pi / 30.0)
    lsd_f = math.tanh(d_omega_f * params.lsd_smoothing_gain) * params.lsd_bias_ratio
    T_fl = front_axle_torque * (0.5 + lsd_f)
    T_fr = front_axle_torque * (0.5 - lsd_f)
    
    d_omega_r = (rr_rpm - rl_rpm) * (math.pi / 30.0)
    lsd_r = math.tanh(d_omega_r * params.lsd_smoothing_gain) * params.lsd_bias_ratio
    T_rl = rear_axle_torque * (0.5 + lsd_r)
    T_rr = rear_axle_torque * (0.5 - lsd_r)
    
    new_state = EngineState(
        rpm=new_rpm,
        current_gear=new_gear,
        actual_throttle=new_throttle,
        output_shaft_torque=total_axle_torque,
        is_rev_limiting=rev_limit
    )
    
    return new_state, (T_fl, T_fr, T_rl, T_rr)

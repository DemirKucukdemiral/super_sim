"""
Hydraulic Braking System with Anti-Lock Braking System (ABS) Controller.
Prevents wheel lockup, maintains steerability, and optimizes braking deceleration.
"""

import math
from dataclasses import dataclass, field
from enum import Enum


class ABSState(Enum):
    APPLY = "APPLY"
    DUMP = "DUMP"
    HOLD = "HOLD"
    RAMP = "RAMP"


@dataclass
class BrakeParams:
    max_line_pressure_bar: float = 120.0       # Max hydraulic line pressure [bar]
    hydraulic_tau: float = 0.025               # Hydraulic line build-up time constant [s]
    front_bias: float = 0.65                   # Front axle braking proportion (65% front, 35% rear)
    
    # Caliper & Disc Geometry
    piston_area_front: float = 0.0042          # Front brake caliper piston area [m^2]
    piston_area_rear: float = 0.0028           # Rear brake caliper piston area [m^2]
    effective_rotor_radius_front: float = 0.170 # Front disc rotor effective radius [m]
    effective_rotor_radius_rear: float = 0.155  # Rear disc rotor effective radius [m]
    brake_pad_mu: float = 0.45                 # Brake pad friction coefficient
    
    # ABS Controller Settings
    abs_enabled: bool = True
    slip_lockup_threshold: float = -0.16       # Slip threshold triggering ABS (~16% negative slip)
    slip_recovery_threshold: float = -0.07     # Slip threshold where wheel has recovered
    dump_pressure_fraction: float = 0.40       # Retain 40% pressure during DUMP phase (dump 60%)
    ramp_rate_bar_per_s: float = 300.0         # Pressure re-apply ramp rate [bar/s]


@dataclass
class BrakeState:
    master_cylinder_pressure_bar: float = 0.0
    caliper_pressure_bar: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])  # FL, FR, RL, RR
    abs_modes: list[ABSState] = field(default_factory=lambda: [ABSState.APPLY]*4)
    abs_active: bool = False
    handbrake_engaged: bool = False


def update_braking(
    params: BrakeParams,
    state: BrakeState,
    brake_pedal: float,              # Driver command [0.0 .. 1.0]
    handbrake: bool,                 # Handbrake / EBS command
    wheel_slips: tuple[float, float, float, float],    # (kappa_fl, kappa_fr, kappa_rl, kappa_rr)
    wheel_speeds_rpm: tuple[float, float, float, float],
    vehicle_speed_ms: float,
    dt: float
) -> tuple[BrakeState, tuple[float, float, float, float]]:
    """
    Simulates hydraulic braking and ABS modulation.
    Returns:
        (new_state, (Tb_fl, Tb_fr, Tb_rl, Tb_rr)): Updated state and braking torque per wheel [N*m].
    """
    brake_pedal = max(0.0, min(1.0, brake_pedal))
    target_pressure = brake_pedal * params.max_line_pressure_bar
    
    # Master cylinder pressure lag
    alpha = 1.0 - math.exp(-dt / params.hydraulic_tau)
    new_master_pressure = state.master_cylinder_pressure_bar + alpha * (target_pressure - state.master_cylinder_pressure_bar)
    
    # Base demands based on front/rear bias (in bar)
    demand_f = new_master_pressure * params.front_bias
    demand_r = new_master_pressure * (1.0 - params.front_bias)
    wheel_demands = [demand_f, demand_f, demand_r, demand_r]
    
    new_pressures = list(state.caliper_pressure_bar)
    new_abs_modes = list(state.abs_modes)
    any_abs_active = False
    
    # Minimum speed for ABS intervention (under 3 m/s (~11 km/h), let vehicle come to a complete stop)
    abs_allowed = params.abs_enabled and (vehicle_speed_ms > 3.0) and (not handbrake)
    
    for i in range(4):
        slip = wheel_slips[i]
        curr_p = new_pressures[i]
        dem_p = wheel_demands[i]
        mode = new_abs_modes[i]
        
        if abs_allowed and dem_p > 5.0:
            if slip < params.slip_lockup_threshold:
                # Wheel is entering excessive slip / lockup -> DUMP pressure
                mode = ABSState.DUMP
                curr_p = dem_p * params.dump_pressure_fraction
                any_abs_active = True
                
            elif mode == ABSState.DUMP:
                if slip > params.slip_recovery_threshold:
                    # Wheel has regained spin speed -> HOLD pressure at optimal grip
                    mode = ABSState.HOLD
                    any_abs_active = True
                else:
                    curr_p = dem_p * params.dump_pressure_fraction
                    any_abs_active = True
                    
            elif mode == ABSState.HOLD:
                # Step up pressure gradually
                mode = ABSState.RAMP
                any_abs_active = True
                
            elif mode == ABSState.RAMP:
                # Ramp pressure back up towards demand
                curr_p = min(dem_p, curr_p + params.ramp_rate_bar_per_s * dt)
                if curr_p >= dem_p:
                    mode = ABSState.APPLY
                else:
                    any_abs_active = True
            else:
                curr_p = dem_p
        else:
            mode = ABSState.APPLY
            curr_p = dem_p
            
        new_pressures[i] = curr_p
        new_abs_modes[i] = mode
    
    # Handbrake engagement: clamps rear brakes to max pressure
    if handbrake:
        new_pressures[2] = params.max_line_pressure_bar
        new_pressures[3] = params.max_line_pressure_bar
        
    # Convert pressure [bar] to caliper braking torque [N*m]
    # 1 bar = 100,000 Pa (N/m^2)
    # Torque = 2 * (P * A) * mu * r_eff (for 2 pads per rotor)
    BAR_TO_PA = 100000.0
    
    # Front calipers
    K_front = 2.0 * BAR_TO_PA * params.piston_area_front * params.brake_pad_mu * params.effective_rotor_radius_front
    Tb_fl = new_pressures[0] * K_front
    Tb_fr = new_pressures[1] * K_front
    
    # Rear calipers
    K_rear = 2.0 * BAR_TO_PA * params.piston_area_rear * params.brake_pad_mu * params.effective_rotor_radius_rear
    Tb_rl = new_pressures[2] * K_rear
    Tb_rr = new_pressures[3] * K_rear
    
    new_state = BrakeState(
        master_cylinder_pressure_bar=new_master_pressure,
        caliper_pressure_bar=new_pressures,
        abs_modes=new_abs_modes,
        abs_active=any_abs_active,
        handbrake_engaged=handbrake
    )
    
    return new_state, (Tb_fl, Tb_fr, Tb_rl, Tb_rr)

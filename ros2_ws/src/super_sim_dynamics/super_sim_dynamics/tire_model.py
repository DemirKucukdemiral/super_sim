"""
Simplified Magic Formula Tire Model with Combined Slip and Friction Ellipse.
Pacejka sine-arctan formulation with smooth low-speed regularization to eliminate
zero-speed singularity and explicit airborne guards to prevent 0/0 evaluation.
"""

import math
from dataclasses import dataclass


@dataclass
class TireParams:
    # Friction coefficients
    mu_x: float = 1.30         # Peak longitudinal friction coefficient
    mu_y: float = 1.35         # Peak lateral friction coefficient
    
    # Pacejka Longitudinal (Fx0) coefficients
    Bx: float = 10.0           # Stiffness factor
    Cx: float = 1.65           # Shape factor
    Ex: float = -0.50          # Curvature factor
    
    # Pacejka Lateral (Fy0) coefficients
    By: float = 12.0           # Stiffness factor
    Cy: float = 1.30           # Shape factor
    Ey: float = -0.70          # Curvature factor
    
    # Optimal slip points
    kappa_peak: float = 0.12   # Reference peak longitudinal slip
    alpha_peak: float = 0.09   # Reference peak slip angle [rad]
    
    # Rolling resistance coefficients
    c_rr0: float = 0.012       # Static rolling resistance coefficient
    c_rr1: float = 1.5e-5      # Dynamic rolling resistance coefficient (s^2/m^2)
    
    # Wheel geometry
    wheel_radius: float = 0.56985  # Effective loaded radius [m]
    eps_F: float = 1e-3        # Airborne / zero normal load threshold [N]


def calc_pure_longitudinal_force(params: TireParams, Fz: float, kappa: float) -> float:
    """Simplified Magic Formula for pure longitudinal tire force Fx0 [N]."""
    if Fz < params.eps_F:
        return 0.0
    Dx = params.mu_x * Fz
    Bx = params.Bx
    Cx = params.Cx
    Ex = params.Ex
    phi = (1.0 - Ex) * kappa + (Ex / Bx) * math.atan(Bx * kappa)
    return Dx * math.sin(Cx * math.atan(Bx * phi))


def calc_pure_lateral_force(params: TireParams, Fz: float, alpha: float) -> float:
    """Simplified Magic Formula for pure lateral tire force Fy0 [N]."""
    if Fz < params.eps_F:
        return 0.0
    Dy = params.mu_y * Fz
    By = params.By
    Cy = params.Cy
    Ey = params.Ey
    phi = (1.0 - Ey) * alpha + (Ey / By) * math.atan(By * alpha)
    return Dy * math.sin(Cy * math.atan(By * phi))


def calc_longitudinal_slip_stiffness(params: TireParams, Fz: float, kappa: float, h: float = 1e-4) -> float:
    """Local slope dFx0/dkappa [N] of pure longitudinal Magic Formula (central difference)."""
    if Fz < params.eps_F:
        return 0.0
    return (calc_pure_longitudinal_force(params, Fz, kappa + h)
            - calc_pure_longitudinal_force(params, Fz, kappa - h)) / (2.0 * h)


def friction_ellipse_scale(params: TireParams, Fz: float, Fx: float, Fy: float) -> float:
    """
    Friction-ellipse saturation factor lambda.
    Gamma = sqrt((Fx/(mu_x Fz))^2 + (Fy/(mu_y Fz))^2); lambda = 1 if Gamma <= 1 else 1/Gamma.
    Explicit guard: Fz < eps_F implies wheel is airborne -> lambda = 0.0 (prevents 0/0 singularity).
    """
    if Fz < params.eps_F:
        return 0.0
    gamma = math.hypot(Fx / (params.mu_x * Fz), Fy / (params.mu_y * Fz))
    return 1.0 if gamma <= 1.0 else 1.0 / gamma


def calc_combined_tire_forces(
    params: TireParams,
    Fz: float,
    kappa: float,
    alpha: float,
    vx_wheel: float,
    rw_omega: float = 0.0
) -> tuple[float, float, float]:
    """
    Combined longitudinal/lateral tire forces via friction ellipse scaling.
    Guarded against airborne wheels (Fz < eps_F -> Fx = Fy = 0).
    """
    if Fz < params.eps_F:
        return 0.0, 0.0, 0.0
    Fx0 = calc_pure_longitudinal_force(params, Fz, kappa)
    Fy0 = calc_pure_lateral_force(params, Fz, alpha)
    lam = friction_ellipse_scale(params, Fz, Fx0, Fy0)
    Trr = (params.c_rr0 + params.c_rr1 * vx_wheel * vx_wheel) * Fz * params.wheel_radius
    return lam * Fx0, lam * Fy0, Trr

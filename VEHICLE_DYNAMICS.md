# Super Sim: 14 Degrees of Freedom (14-DoF) Vehicle Dynamics Specification

This document provides the coordinate conventions, kinematics, governing equations, and benchmark evaluations of the **Super Sim 14-DoF Vehicle Dynamics Model** (ROS 2 Jazzy, Gazebo Harmonic). Every equation below is implemented directly in `super_sim_dynamics/vehicle_14dof_model.py`, `tire_model.py`, `engine_model.py`, and `braking_model.py`.

---

## 1. System Overview & State Space Representation

- **Sprung mass (chassis)**: 6 DoF — surge $u$, sway $v$, heave $w$, roll rate $p$, pitch rate $q$, yaw rate $r$.
- **Unsprung masses**: 4 DoF — vertical heave of each wheel hub (FL, FR, RL, RR).
- **Wheel spin**: 4 DoF — rotational speed of each wheel.
- **Steering actuator**: 1 additional first-order state $\delta$.

$$\mathbf{x} = \begin{bmatrix}
x, y, z & \text{World position of body reference point } [m] \\
\phi, \theta, \psi & \text{Euler angles (roll, pitch, yaw) } [rad] \\
u, v, w & \text{Body-frame translational velocities } [m/s] \\
p, q, r & \text{Body-frame angular rates } [rad/s] \\
z_{u,i} & \text{Wheel hub world elevations } [m] \\
\dot{z}_{u,i} & \text{Wheel hub vertical velocities } [m/s] \\
\omega_i & \text{Wheel spin speeds } [rad/s] \\
\delta & \text{Front steering angle } [rad]
\end{bmatrix}^T \in \mathbb{R}^{25}$$

(24 mechanical states + 1 steering-actuator state.) Corner indices: FL $=0$, FR $=1$, RL $=2$, RR $=3$.

---

## 2. Coordinate Frames & Transformations

1. **World frame** $(X_W, Y_W, Z_W)$: fixed to the terrain, $\mathbf{g}_W = [0, 0, -g]^T$, $g = 9.81\,m/s^2$.
2. **Body frame** $(X_B, Y_B, Z_B)$, right-handed, origin at the sprung-mass CG:
   - $+X_B$ forward ($u$), $+Y_B$ left ($v$), $+Z_B$ up ($w$).
   - **Roll** $\phi$: rotation about $+X_B$ (rate $p$). By the right-hand rule $+Y_B$ rotates toward $+Z_B$, so **positive roll = left side up / right side down**.
   - **Pitch** $\theta$: rotation about $+Y_B$ (rate $q$). $+X_B$ rotates toward $-Z_B$, so **positive pitch = nose down**.
   - **Yaw** $\psi$: rotation about $+Z_B$ (rate $r$), counter-clockwise positive.
   - Positive moments $M_x, M_y, M_z$ drive positive $\dot p, \dot q, \dot r$: $M_y > 0$ is nose-down, $M_x > 0$ is right-side-down.
   - **CG elevation & nominal ride height**: The body reference point is located at the sprung-mass CG. On flat ground at design ride height, the nominal world elevation of the CG is $z_0 = h_{\text{cg}} = 0.52\text{ m}$, ensuring that the geometric CG height above the ground plane and the effective vertical moment arm for tire contact patch forces $\sum F_x, \sum F_y$ are identically consistent ($z_0 = h_{\text{cg}}$).

### 2.1 Rotation Matrix & Gravity Projection

3-2-1 (yaw-pitch-roll) DCM from body to world:

$$R_{WB} = \begin{bmatrix}
\cos\theta \cos\psi & \sin\phi \sin\theta \cos\psi - \cos\phi \sin\psi & \cos\phi \sin\theta \cos\psi + \sin\phi \sin\psi \\
\cos\theta \sin\psi & \sin\phi \sin\theta \sin\psi + \cos\phi \cos\psi & \cos\phi \sin\theta \sin\psi - \sin\phi \cos\psi \\
-\sin\theta & \sin\phi \cos\theta & \cos\phi \cos\theta
\end{bmatrix}$$

The first column (body $+X_B$ in world) has vertical component $-\sin\theta$: positive $\theta$ points the nose down, consistent with Section 2.

$$\mathbf{g}_B = R_{WB}^T \mathbf{g}_W = \begin{bmatrix} +g \sin\theta \\ -g \cos\theta \sin\phi \\ -g \cos\theta \cos\phi \end{bmatrix}$$

Nose-down ($\theta > 0$) means pointing downhill, so gravity accelerates the vehicle forward.

---

## 3. 6-DoF Sprung Mass (Chassis) Equations of Motion

### 3.1 Translational Equations

The unsprung masses translate horizontally with the body, so the horizontal equations use the total mass $m_{\text{tot}} = m_s + \sum_i m_{u,i}$; the heave equation uses the sprung mass $m_s$ only (the unsprung heave has its own equations, Section 4.3):

$$\dot{u} = \frac{\sum_i F_{x,b,i} - \operatorname{sgn}(u) F_{\text{drag}}}{m_{\text{tot}}} + v r - w q + g \sin\theta$$

$$\dot{v} = \frac{\sum_i F_{y,b,i}}{m_{\text{tot}}} - u r + w p - g \cos\theta \sin\phi$$

$$\dot{w} = \frac{\sum_i F_{\text{susp},i} - F_{\text{downforce}}}{m_s} + u q - v p - g \cos\theta \cos\phi$$

At rest on flat ground $\sum_i F_{\text{susp},i} = m_s g$, so $\dot w = 0$.

### 3.2 Rotational Equations (principal axes)

$$\dot{p} = \frac{M_x - (I_{zz} - I_{yy}) q r}{I_{xx}}, \qquad
\dot{q} = \frac{M_y - (I_{xx} - I_{zz}) p r}{I_{yy}}, \qquad
\dot{r} = \frac{M_z - (I_{yy} - I_{xx}) p q}{I_{zz}}$$

All moments follow from $\mathbf{M} = \mathbf{r} \times \mathbf{F}$ about the CG. Suspension forces act upward at the hardpoints $\mathbf{r}_i = [x_i, y_i, z^B_{s,i}]^T$. Tire horizontal forces act at the contact patches, $z = -h_{\text{cg}}$ below the CG. Aerodynamic downforce acts on the sprung mass at the axles.

$$M_x = w_f \left(F_{\text{susp},\text{FL}} - F_{\text{susp},\text{FR}}\right) + w_r \left(F_{\text{susp},\text{RL}} - F_{\text{susp},\text{RR}}\right) + h_{\text{cg}} \sum_i F_{y,b,i}$$

$$M_y = -a \left(F_{\text{susp},\text{FL}} + F_{\text{susp},\text{FR}}\right) + b \left(F_{\text{susp},\text{RL}} + F_{\text{susp},\text{RR}}\right) - h_{\text{cg}} \sum_i F_{x,b,i} + \underbrace{a F_{\text{aero},f} - b F_{\text{aero},r}}_{M_{y,\text{aero}}} - z_{\text{aero}} \operatorname{sgn}(u) F_{\text{drag}}$$

$$M_z = a \left(F_{y,b,\text{FL}} + F_{y,b,\text{FR}}\right) - b \left(F_{y,b,\text{RL}} + F_{y,b,\text{RR}}\right) + w_f \left(F_{x,b,\text{FR}} - F_{x,b,\text{FL}}\right) + w_r \left(F_{x,b,\text{RR}} - F_{x,b,\text{RL}}\right)$$

#### Sign derivations
- **Roll**: upward force at $y = +w_f$: $[0, w_f, 0] \times [0, 0, F] = (+w_f F, 0, 0)$. Lateral tire force at $z = -h_{\text{cg}}$: $[0, 0, -h_{\text{cg}}] \times [0, F_y, 0] = (+h_{\text{cg}} F_y, 0, 0)$. In a **left** turn the tires push the car left ($\sum F_y > 0$), so $M_x > 0$ and the body rolls right-side-down, i.e. toward the outside of the turn.
- **Pitch**: upward front force $[a,0,0]\times[0,0,F] = (0, -aF, 0)$, so it lifts the nose. Upward rear force gives $+bF$. Tire traction at $z=-h_{\text{cg}}$: $[0,0,-h_{\text{cg}}]\times[F_x,0,0] = (0, -h_{\text{cg}}F_x, 0)$. Acceleration therefore causes squat (nose up) and braking causes dive.
- **Aero pitch**: downforce at the front axle, $[a,0,0]\times[0,0,-F_{\text{aero},f}] = (0, +aF_{\text{aero},f}, 0)$ (nose down). At the rear it gives $-bF_{\text{aero},r}$. Because of this term the aero balance $\beta_{\text{aero}}$ has a physical effect.
- **Drag**: drag acts at the aerodynamic centre of pressure, at height $z_{\text{aero}}$ above the CG, **not** at the contact patch. It is therefore excluded from the $-h_{\text{cg}}$ term. $[0,0,z_{\text{aero}}]\times[-D,0,0] = (0, -z_{\text{aero}}D, 0)$. The default is $z_{\text{aero}} = 0$.

### 3.3 Kinematics

$$\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} =
\begin{bmatrix}
1 & \sin\phi \tan\theta & \cos\phi \tan\theta \\
0 & \cos\phi & -\sin\phi \\
0 & \frac{\sin\phi}{\cos\theta} & \frac{\cos\phi}{\cos\theta}
\end{bmatrix}
\begin{bmatrix} p \\ q \\ r \end{bmatrix},
\qquad
\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{z} \end{bmatrix} = R_{WB} \begin{bmatrix} u \\ v \\ w \end{bmatrix}$$

---

## 4. Unsprung Masses & Independent Suspension

### 4.1 Hardpoint Geometry

Body-frame hardpoints: $\mathbf{r}_{\text{FL}} = [a, w_f, z^B_{s}]^T$, $\mathbf{r}_{\text{FR}} = [a, -w_f, z^B_{s}]^T$, $\mathbf{r}_{\text{RL}} = [-b, w_r, z^B_{s}]^T$, $\mathbf{r}_{\text{RR}} = [-b, -w_r, z^B_{s}]^T$.

From $\mathbf{p}_{c,i} = \mathbf{p}_{\text{CG}} + R_{WB}\mathbf{r}_i$ (third row of $R_{WB}$):

$$z_{c,i} = z - x_i \sin\theta + y_i \sin\phi\cos\theta + z^B_{s} \cos\theta\cos\phi$$

$$\dot{z}_{c,i} = \dot z - x_i \cos\theta\,\dot\theta + y_i\left(\cos\phi\cos\theta\,\dot\phi - \sin\phi\sin\theta\,\dot\theta\right) - z^B_s\left(\sin\theta\cos\phi\,\dot\theta + \cos\theta\sin\phi\,\dot\phi\right)$$

where $\dot z$ is the **world** vertical velocity (third row of $R_{WB}[u,v,w]^T$) and $\dot\phi, \dot\theta$ are the Euler rates of Section 3.3.

Small-angle approximation:

$$z_{c,\text{FL/FR}} \simeq z - a\theta \pm w_f\phi + z^B_s, \qquad z_{c,\text{RL/RR}} \simeq z + b\theta \pm w_r\phi + z^B_s$$

### 4.2 Ride-Height Reference, Preload, Damping & Anti-Roll Bars

Static sprung load per corner: $m_{s,f} = \frac{b}{a+b}\frac{m_s}{2}$, $m_{s,r} = \frac{a}{a+b}\frac{m_s}{2}$, giving preloads $F_{\text{preload},f} = m_{s,f}g$ and $F_{\text{preload},r} = m_{s,r}g$.

Design ride height on flat ground: the body reference point is at $z_0$, the hardpoints are at $z^0_{c,i} = z_0 + z^B_s$, and the hubs are at $z^0_{u,i} = R_w - \Delta z^0_{\text{tire},i}$ with $\Delta z^0_{\text{tire},i} = (m_{s,i} + m_{u,i})g / k_{t,z}$.

Suspension deflection is measured **relative to the design ride height**:

$$\Delta z_i = (z_{u,i} - z_{c,i}) - (z^0_{u,i} - z^0_{c,i}), \qquad \Delta\dot z_i = \dot z_{u,i} - \dot z_{c,i}$$

$\Delta z_i > 0$ is jounce. At design ride height $\Delta z_i = \Delta\dot z_i = 0$, so $F_{\text{susp},i} = F_{\text{preload},i}$ exactly.

Damping: $c_i = c_{\text{bump}}$ if $\Delta\dot z_i \ge 0$, otherwise $c_{\text{rebound}}$.

Anti-roll bars:

$$F_{\text{ARB},f} = k_{\text{arb},f}\left(\Delta z_{\text{FL}} - \Delta z_{\text{FR}}\right), \qquad F_{\text{ARB},r} = k_{\text{arb},r}\left(\Delta z_{\text{RL}} - \Delta z_{\text{RR}}\right)$$

Net upward suspension force on the chassis:

$$F_{\text{susp},\text{FL}} = F_{\text{preload},f} + k_{s,f}\Delta z_{\text{FL}} + c_{\text{FL}}\Delta\dot z_{\text{FL}} + F_{\text{ARB},f}$$
$$F_{\text{susp},\text{FR}} = F_{\text{preload},f} + k_{s,f}\Delta z_{\text{FR}} + c_{\text{FR}}\Delta\dot z_{\text{FR}} - F_{\text{ARB},f}$$
$$F_{\text{susp},\text{RL}} = F_{\text{preload},r} + k_{s,r}\Delta z_{\text{RL}} + c_{\text{RL}}\Delta\dot z_{\text{RL}} + F_{\text{ARB},r}$$
$$F_{\text{susp},\text{RR}} = F_{\text{preload},r} + k_{s,r}\Delta z_{\text{RR}} + c_{\text{RR}}\Delta\dot z_{\text{RR}} - F_{\text{ARB},r}$$

**Restoring check**: when $\phi > 0$ (right side down), $z_{c,\text{FL}}$ rises and $z_{c,\text{FR}}$ falls. So $\Delta z_{\text{FL}} < \Delta z_{\text{FR}}$ and $F_{\text{ARB},f} < 0$. The left force decreases and the right force increases, giving $M_x = w_f(F_{\text{FL}} - F_{\text{FR}}) < 0$, which opposes the roll. Roll stiffness per axle is $K_\phi = w^2(2k_s + 4k_{\text{arb}})$.

### 4.3 Tire Vertical Force & Unsprung Heave

The road height under a moving wheel, $z_{\text{road},i}(t) = z_{\text{road}}(x_i(t), y_i(t))$, varies in time even on stationary terrain. With

$$\Delta z_{\text{tire},i} = (z_{\text{road},i} + R_w) - z_{u,i}, \qquad \Delta\dot z_{\text{tire},i} = \dot z_{\text{road},i} - \dot z_{u,i},$$

the contact force is unilateral, because the road cannot pull the tire down:

$$F_{\text{tire},z,i} = \begin{cases} \max\left(0,\; k_{t,z}\Delta z_{\text{tire},i} + c_{t,z}\Delta\dot z_{\text{tire},i}\right), & \Delta z_{\text{tire},i} > 0 \\ 0, & \text{airborne} \end{cases}$$

$\dot z_{\text{road},i}$ is the finite difference of the sampled road heights over each control step. The road height itself is linearly interpolated across the integration substeps.

$$m_{u,i}\ddot z_{u,i} = F_{\text{tire},z,i} - F_{\text{susp},i} - m_{u,i} g$$

---

## 5. Normal Tire Loads

The normal load used by the tire model **is** the contact force:

$$F_{z,i} \equiv F_{\text{tire},z,i}$$

Under quasi-steady wheel heave ($\ddot z_{u,i} \approx 0$), the unsprung equation gives directly

$$F_{z,i} \approx F_{\text{susp},i} + m_{u,i} g.$$

There is no separate downforce term. Aerodynamic downforce acts on the sprung mass, compresses the suspension, and so reaches the tires through $F_{\text{susp}}$. Globally, in steady state, $\sum_i F_{\text{susp},i} = m_s g + F_{\text{downforce}}$, therefore

$$\sum_i F_{z,i} = \sum_i F_{\text{susp},i} + \sum_i m_{u,i} g = m_{\text{tot}}\, g + F_{\text{downforce}},$$

with no double counting. The front/rear split of the downforce is set by $M_{y,\text{aero}}$ (Section 3.2):

$$F_{\text{downforce}} = \tfrac12 \rho_{\text{air}} C_L A (u^2 + v^2), \quad F_{\text{aero},f} = \beta_{\text{aero}} F_{\text{downforce}}, \quad F_{\text{aero},r} = (1-\beta_{\text{aero}}) F_{\text{downforce}}, \quad F_{\text{drag}} = \tfrac12 \rho_{\text{air}} C_d A (u^2+v^2)$$

---

## 6. Steering & Wheel Kinematics

**Actuator lag & saturation**: The continuous steering rate is:

$$\dot\delta = \frac{1}{\tau_{\text{steer}}}\left(\operatorname{clamp}(\delta_{\text{cmd}}, -\delta_{\max}, \delta_{\max}) - \delta\right)$$

The exact discrete update for a zero-order hold command saturates the input prior to integration:

$$\delta_{\text{sat}} = \operatorname{clamp}(\delta_{\text{cmd}}, -\delta_{\max}, \delta_{\max}), \qquad \delta_{k+1} = \delta_k + (1 - e^{-\Delta t / \tau_{\text{steer}}})(\delta_{\text{sat}} - \delta_k)$$

**Ackermann geometry**:

$$\delta_{\text{FL}} = \arctan\frac{L\tan\delta}{L - w_f\tan\delta}, \qquad \delta_{\text{FR}} = \arctan\frac{L\tan\delta}{L + w_f\tan\delta}, \qquad \delta_{\text{RL}} = \delta_{\text{RR}} = 0$$

**Hub velocities** ($\mathbf v_{b,i} = \mathbf v_B + \boldsymbol\omega_B \times \mathbf r_i$, planar projection):
FL: $(u - w_f r,\ v + a r)$; FR: $(u + w_f r,\ v + a r)$; RL: $(u - w_r r,\ v - b r)$; RR: $(u + w_r r,\ v - b r)$. Rotated into each wheel frame by $\delta_i$:

$$v_{x,w,i} = v_{x,b,i}\cos\delta_i + v_{y,b,i}\sin\delta_i, \qquad v_{y,w,i} = -v_{x,b,i}\sin\delta_i + v_{y,b,i}\cos\delta_i$$

**Slips** (with $v_{\text{reg}} = 0.5\,m/s$ to eliminate zero-speed division singularities):

$$\alpha_i = -\arctan\frac{v_{y,w,i}}{\max(|v_{x,w,i}|, v_{\text{reg}})}, \qquad \kappa_i = \frac{R_w\omega_i - v_{x,w,i}}{\max(|v_{x,w,i}|, v_{\text{reg}})}$$

---

## 7. Simplified Magic Formula Tire Model + Friction Ellipse

Pure-slip formulation based on Pacejka sine-arctan functional form ($s = \kappa$ or $\alpha$):

$$F_0(s) = D\sin\left(C\arctan(B\varphi)\right), \qquad \varphi = (1-E)s + \frac{E}{B}\arctan(Bs)$$

with peak forces $D_x = \mu_x F_z$ and $D_y = \mu_y F_z$. $F_{x0}$ and $F_{y0}$ are odd functions, preserving the signs of $\kappa$ and $\alpha$.

> [!NOTE]
> This formulation is a **Simplified Magic Formula model** utilizing Pacejka curve shapes ($B, C, D, E$) with friction-ellipse combined-slip coupling, distinct from the complete 80+ parameter MF96/MF2002 standard (which additionally includes tire load degressivity, camber thrust, inflation pressure sensitivity, ply steer, and conicity).

**Friction ellipse & airborne wheel protection**:
To avoid $0/0$ numerical evaluation when a wheel is airborne or fully unloaded ($F_z < \epsilon_F$, where $\epsilon_F = 1.0\times 10^{-3}\text{ N}$):

$$F_z < \epsilon_F \implies F_{x,w} = 0, \quad F_{y,w} = 0, \quad \lambda = 0$$

When contact is established ($F_z \ge \epsilon_F$), pure forces are unattenuated inside the friction envelope and scaled only when combined demand exceeds the envelope:

$$\Gamma = \sqrt{\left(\frac{F_{x0}}{\mu_x F_z}\right)^2 + \left(\frac{F_{y0}}{\mu_y F_z}\right)^2}, \qquad \lambda = \begin{cases} 1, & \Gamma \le 1 \\ 1/\Gamma, & \Gamma > 1 \end{cases}, \qquad F_{x,w} = \lambda F_{x0}, \quad F_{y,w} = \lambda F_{y0}$$

**Wheel to body frame**: $F_{x,b,i} = F_{x,w,i}\cos\delta_i - F_{y,w,i}\sin\delta_i$, $\quad F_{y,b,i} = F_{x,w,i}\sin\delta_i + F_{y,w,i}\cos\delta_i$.

---

## 8. Wheel Spin Dynamics

$$I_w\dot\omega_i = T_{\text{drive},i} - F_{x,w,i}R_w - \left(T_{\text{brake},i} + T_{\text{rr},i}\right)\operatorname{sgn}(\omega_i), \qquad T_{\text{rr},i} = (c_{\text{rr}0} + c_{\text{rr}1}v_{x,w,i}^2)F_{z,i}R_w$$

This equation is **integrated continuously for every wheel**. There is no grip/slip regime switching and no algebraic overwrite of $\omega_i$.

**Integration scheme.** The tire-wheel mode is stiff (time constant $\tau \approx I_w v / (C_\kappa R_w^2) \sim 30\,\mu\text{s} - 2\,\text{ms}$, below the $2.5\,\text{ms}$ substep). The update is therefore linearly implicit in $\omega$:

$$k_\omega = \lambda\,\max\!\left(0, \frac{\partial F_{x0}}{\partial\kappa}\right)\frac{R_w}{\max(|v_{x,w}|, v_{\text{reg}})}, \qquad
\omega^{\text{free}} = \omega^n + \frac{\Delta t\,(T_{\text{drive}} - R_w\lambda F_{x0}(\kappa^n))}{I_w + \Delta t R_w k_\omega}$$

Resistive torques (brake + rolling resistance) act as Coulomb friction that **cannot reverse the spin direction**:

$$J = \frac{\Delta t\,(T_{\text{brake}} + T_{\text{rr}})}{I_w + \Delta t R_w k_\omega}, \qquad \omega^{n+1} = \begin{cases} 0, & |\omega^{\text{free}}| \le J \ \text{(wheel held / locked)} \\ \omega^{\text{free}} - J\operatorname{sgn}(\omega^{\text{free}}), & \text{otherwise} \end{cases}$$

The force sent to the chassis is $F_{x,w} = \lambda F_{x0}(\kappa^n) + k_\omega(\omega^{n+1} - \omega^n)$, re-clamped to the friction ellipse.

---

## 9. Powertrain, Traction Control & Limited-Slip Differentials

### 9.1 Electric Powertrain Architecture
- **Total output ratings**: $T_{\max} = 950\text{ N}\cdot\text{m}$ peak torque, $P_{\max} = 650\text{ kW}$ peak power, representing total vehicle capability across both axles.
- **AWD configuration**: 50:50 torque distribution between front and rear axles. Single reduction ratio $G_{\text{ev}} = 10.5$, driveline efficiency $\eta = 0.96$.
- **Equivalent motor speed**: Defined by axle wheel speeds:

$$\Omega_{m,f} = G_{\text{ev}} \frac{\omega_{\text{FL}} + \omega_{\text{FR}}}{2}, \qquad \Omega_{m,r} = G_{\text{ev}} \frac{\omega_{\text{RL}} + \omega_{\text{RR}}}{2}, \qquad \Omega_m = \frac{\Omega_{m,f} + \Omega_{m,r}}{2}$$

- **Operating regimes & overspeed cut**:

$$T_m = \begin{cases}
T_{\max} \alpha_{\text{throttle}}, & \Omega_m \le \Omega_{\text{base}} \quad (\text{constant torque regime}) \\
\min\left(T_{\max}, \frac{P_{\max}}{\Omega_m}\right) \alpha_{\text{throttle}}, & \Omega_{\text{base}} < \Omega_m < \Omega_{\max} \quad (\text{constant power regime}) \\
0, & \Omega_m \ge \Omega_{\max} \quad (\text{overspeed rev-limit cutoff})
\end{cases}$$

Driveshaft torque before axle split: $T_{\text{axle,tot}} = T_m G_{\text{ev}}\eta$, subject to first-order throttle lag $\tau_{\text{throttle}} = 0.01\text{ s}$.

### 9.2 Smooth Speed-Sensitive Limited-Slip Differential (LSD)

Rather than a discontinuous multi-plate mechanical Salisbury clutch pack, cross-axle torque biasing is modeled using a smooth speed-difference function:

$$\Delta\Omega = \omega_{\text{right}} - \omega_{\text{left}}, \qquad T_{\text{left/right}} = T_{\text{axle}}\left(0.5 \pm \lambda_{\text{lsd}}\tanh(k_{\text{lsd}}\Delta\Omega)\right)$$

where $k_{\text{lsd}} = 0.5\,\text{s/rad}$ and $\lambda_{\text{lsd}} = 0.40$. If the right wheel slips faster ($\Delta\Omega > 0$), drive torque shifts continuously to the slower, gripping left wheel.

### 9.3 Traction Control
The drive torque on each wheel is trimmed when slip ratio exceeds target:

$$T_{\text{drive},i} \leftarrow T_{\text{drive},i}\cdot\operatorname{clamp}\!\left(1 - \frac{|\kappa_i| - \kappa_{\text{tc}}}{\Delta\kappa_{\text{tc}}},\ s_{\min},\ 1\right) \quad \text{if } T_{\text{drive},i}\kappa_i > 0 \text{ and } |\kappa_i| > \kappa_{\text{tc}}$$

with $\kappa_{\text{tc}} = 0.10$, $\Delta\kappa_{\text{tc}} = 0.10$, $s_{\min} = 0.10$.

---

## 10. Hydraulic Brakes & ABS

1. Line pressure: $\tau_{\text{hyd}}\dot P + P = P_{\max}\alpha_{\text{brake}}$, with $65\%$ front brake bias.
2. Caliper torque: $T_{\text{brake},i} = 2P_i A_{\text{piston}}\mu_{\text{pad}} r_{\text{rotor}}$.
3. ABS state machine per wheel (APPLY / DUMP / HOLD / RAMP) with slip thresholds $\kappa_{\text{lock}} = -0.16$ and $\kappa_{\text{rec}} = -0.07$, active above $3\,\text{m/s}$.

---

## 11. Numerical Implementation Notes

- Control rate 100 Hz with 4 integration substeps per step ($\Delta t = 2.5\,\text{ms}$). Semi-implicit Euler for chassis and unsprung states; linearly implicit Euler for wheel spin.
- No artificial damping on $p, q, r, w$; roll/pitch angles are unconstrained. Damping arises solely from physical dampers, tires, and anti-roll bars.
- **Numerical low-speed deadband (standstill stabilization)**: At near-zero speeds ($\sqrt{u^2+v^2} < 0.08\,\text{m/s}$), in order to avoid numerical limit-cycle chatter from regularised slip denominators without requiring an algebraic loop for static Coulomb stick-slip friction, a numerical velocity deadband is applied when drive torque is minimal ($T_{\text{drive}} < 10\,\text{N}\cdot\text{m}$) and braking is active (or on near-level terrain $|\sin\theta| < 0.01$). This is a computational stabilization deadband, not a physical tire-carcass static stiction model. On steeper slopes without brakes applied, gravitational downhill acceleration naturally breaks this deadband and allows the vehicle to roll.

---

## 12. Vehicle Model Parameters

> [!NOTE]
> **Wheel radius $R_w = 0.56985\,m$** is larger than a typical road tire ($\approx 0.33$–$0.36\,m$) to match the Gazebo URDF collision cylinders and wheel meshes. Gear ratio and motor torque are sized to deliver realistic tractive force $F_x = T/R_w$.
> **Design ride height $z_0 = 0.52\,m$** matches the CG height $h_{\text{cg}} = 0.52\,m$ above ground on flat terrain.

| Category | Parameter | Symbol | Value | Unit |
|---|---|---|---|---|
| **Chassis** | Sprung mass | $m_s$ | 1550.0 | kg |
| | Unsprung mass per corner (front / rear) | $m_{u,f}, m_{u,r}$ | 38.0 / 36.0 | kg |
| | Total mass | $m_{\text{tot}}$ | 1698.0 | kg |
| | Wheelbase | $L$ | 3.0832 | m |
| | CG to front / rear axle | $a, b$ | 1.5416 / 1.5416 | m |
| | Track width (front / rear) | $2w_f, 2w_r$ | 1.8565 / 1.8565 | m |
| | CG height above ground | $h_{\text{cg}}$ | 0.52 | m |
| | Suspension hardpoint height rel. body origin | $z^B_s$ | 0.0 | m |
| | Design ride height of body reference (CG) | $z_0$ | 0.52 | m |
| | Roll / pitch / yaw inertia | $I_{xx}, I_{yy}, I_{zz}$ | 520 / 2400 / 3500 | kg·m² |
| **Wheel & Tire** | Wheel radius | $R_w$ | 0.56985 | m |
| | Wheel inertia | $I_w$ | 1.80 | kg·m² |
| | Peak friction (long. / lat.) | $\mu_x, \mu_y$ | 1.30 / 1.35 | – |
| | Simplified Pacejka longitudinal | $B_x, C_x, E_x$ | 10.0, 1.65, −0.50 | – |
| | Simplified Pacejka lateral | $B_y, C_y, E_y$ | 12.0, 1.30, −0.70 | – |
| | Tire vertical stiffness / damping | $k_{t,z}, c_{t,z}$ | 180 000 / 1800 | N/m, N·s/m |
| | Rolling resistance | $c_{\text{rr}0}, c_{\text{rr}1}$ | 0.012, 1.5×10⁻⁵ | –, s²/m² |
| | Slip regularisation speed | $v_{\text{reg}}$ | 0.5 | m/s |
| | Airborne threshold | $\epsilon_F$ | 0.001 | N |
| **Suspension** | Spring rate (front / rear) | $k_{s,f}, k_{s,r}$ | 45 000 / 42 000 | N/m |
| | Bump / rebound damping, front | $c_{\text{bump},f}, c_{\text{reb},f}$ | 3200 / 4800 | N·s/m |
| | Bump / rebound damping, rear | $c_{\text{bump},r}, c_{\text{reb},r}$ | 2800 / 4200 | N·s/m |
| | ARB stiffness (front / rear) | $k_{\text{arb},f}, k_{\text{arb},r}$ | 12 000 / 8000 | N/m |
| **Steering** | Max steer angle | $\delta_{\max}$ | 0.65 (≈37°) | rad |
| | Actuator time constant | $\tau_{\text{steer}}$ | 0.08 | s |
| **Powertrain** | Motor peak torque / power (total vehicle) | $T_{\max}, P_{\max}$ | 950 N·m / 650 kW | – |
| | Base / max motor speed | $\text{RPM}_{\text{base}}, \text{RPM}_{\max}$ | 6500 / 20 000 | rpm |
| | Reduction ratio / efficiency | $G_{\text{ev}}, \eta$ | 10.5 / 0.96 | – |
| | AWD split (front / rear) | – | 0.50 / 0.50 | – |
| | LSD bias / smoothing | $\lambda_{\text{lsd}}, k_{\text{lsd}}$ | 0.40 / 0.50 | –, s/rad |
| | Throttle time constant | $\tau_{\text{throttle}}$ | 0.01 | s |
| | TC slip target / band / min scale | $\kappa_{\text{tc}}, \Delta\kappa_{\text{tc}}, s_{\min}$ | 0.10 / 0.10 / 0.10 | – |
| **Braking** | Max line pressure / time constant | $P_{\max}, \tau_{\text{hyd}}$ | 120 bar / 0.025 s | – |
| | Front brake bias | – | 0.65 | – |
| | Piston area (front / rear) | $A_{p,f}, A_{p,r}$ | 0.0042 / 0.0028 | m² |
| | Rotor effective radius (front / rear) | $r_f, r_r$ | 0.170 / 0.155 | m |
| | Pad friction | $\mu_{\text{pad}}$ | 0.45 | – |
| | ABS lock / recover slip | $\kappa_{\text{lock}}, \kappa_{\text{rec}}$ | −0.16 / −0.07 | – |
| **Aerodynamics** | Drag coefficient / frontal area | $C_d, A$ | 0.32 / 2.25 | –, m² |
| | Downforce coefficient | $C_L$ | 0.45 | – |
| | Aero balance (front fraction) | $\beta_{\text{aero}}$ | 0.48 | – |
| | Drag line height above CG | $z_{\text{aero}}$ | 0.0 | m |
| | Air density | $\rho_{\text{air}}$ | 1.225 | kg/m³ |

---

## 13. Benchmark Comparison Using ISO-Defined Manoeuvres and Literature Metrics

To verify dynamic behavior, the 14-DoF model is simulated through standard automotive test procedures defined by international standards (**ISO 4138:2021**, **ISO 7401:2011**, **ISO 3888-2:2025**) and compared against analytical formulations and empirical data reported in classical vehicle dynamics literature:
- **Gillespie, T. D.**, *Fundamentals of Vehicle Dynamics*, SAE International, 1992.
- **Milliken, W. F., & Milliken, D. L.**, *Race Car Vehicle Dynamics*, SAE International, 1995.
- **Pacejka, H. B.**, *Tire and Vehicle Dynamics*, Butterworth-Heinemann / SAE, 2002.

> [!NOTE]
> ISO 4138 and ISO 7401 define open-loop test procedures and data-processing methods for measuring steady-state and transient responses; ISO 3888-2 defines track geometry for severe obstacle avoidance. These standards specify how tests are run rather than universal pass/fail thresholds. The numerical ranges tabulated below represent typical literature reference values for production passenger cars and performance vehicles.

### 13.1 Validation Matrix Overview

| Manoeuvre / Metric | Standard / Test Method | Literature Comparison Range & Source | Analytical Formulation | 14-DoF Simulation Result | Observations |
|---|---|---|---|---|---|
| **Understeer Gradient ($K_{us}$)** | **ISO 4138:2021** (Constant-Radius Skid-Pad, $R = 50\text{ m}$) | $+1.5^\circ \text{ to } +3.5^\circ / g$ (Passenger cars)<br>$+0.3^\circ \text{ to } +1.2^\circ / g$ (Sports EV)<br>*(Gillespie 1992, Ch. 6, p. 206)* | $K_{us,\text{lin}} = \frac{W_f}{C_f} - \frac{W_r}{C_r}$ (Linear bicycle model) | **$+0.47^\circ / g$** (Moderate range $0.19g - 0.48g$)<br>**$+0.61^\circ / g$** (Overall regression $0.035g - 0.88g$) | Positive (understeer) throughout. Steepens progressively at high $a_y$. |
| **Yaw Rate Step Response Time ($t_{r,90\%}$)** | **ISO 7401:2011** (Step Steer at 80 km/h) | $0.15 \text{ to } 0.35 \text{ s}$ (Passenger cars)<br>*(Milliken 1995, Ch. 6, Table 6.1)* | $t_r \approx \frac{1}{\omega_n} \arctan\left(\frac{\sqrt{1-\zeta^2}}{\zeta}\right)$ | **$0.320 \text{ s}$** | Falls within typical passenger car response range. |
| **Yaw Rate Overshoot ($M_p$)** | **ISO 7401:2011** (Step Steer at 80 km/h) | $0\% \text{ to } 25\%$<br>*(Milliken 1995, Ch. 6, p. 182)* | $M_p = \exp\left(-\frac{\pi \zeta}{\sqrt{1-\zeta^2}}\right)$ | **$0.0\%$** (Monotonic settling) | Monotonic response; directional settling without secondary oscillatory swing. |
| **Quasi-Static Yaw Rate Gain ($r/\delta$)** | **ISO 7401:2011** (Step Steer steady state at 80 km/h) | Close agreement with linear bicycle model gain | $\frac{r}{\delta} = \frac{V/L}{1 + \frac{K_{us} V^2}{g L}} = 6.35\text{ (deg/s)/deg}$ (using $K_{us} = +0.47^\circ/g$) | **$6.73\text{ (deg/s)/deg}$** | Strong agreement ($94.4\%$) with theoretical 2-DoF bicycle model. |
| **Steady-State Roll Gradient ($\frac{d\phi}{da_y}$)** | Constant-radius sweep ($a_y \le 0.6g$) | $3.0^\circ \text{ to } 4.5^\circ / g$ (Production sedans)<br>*(Gillespie 1992, Ch. 7, p. 250)* | $\frac{m_s h_{\text{cg}}}{K_{\phi,\text{eff}} - m_s g h_{\text{cg}}} = 2.94^\circ / g$ (Series suspension + tire compliance) | **$3.24^\circ / g$** | Consistent with series compliance model and literature window. |
| **Steady-State Pitch Gradient ($\frac{d\theta}{da_x}$)** | Straight-line braking dive test | $1.0^\circ \text{ to } 1.8^\circ / g$<br>*(Gillespie 1992, Ch. 8, p. 285)* | $\frac{m_s h_{\text{cg}}}{K_\theta} \frac{180}{\pi} g = 1.10^\circ / g$ | **$1.09^\circ / g$** ($+1.06^\circ$ pitch at $0.97g$ braking) | Within $1\%$ of closed-form pitch stiffness prediction. |
| **Severe Lane Change (Moose Test Track)** | **ISO 3888-2:2025** (Obstacle Avoidance Track at 72 km/h) | Vehicle remains controllable without wheel liftoff ($F_z > 0$) | Multi-body dynamic load transfer limit | **$1.00g$ Peak $a_y$**; Inside tire minimum load **$1802.6\text{ N}$** (Zero wheel liftoff); residual $r = 0.0007^\circ/\text{s}$ | Dynamic load transfer $\Delta F_z \approx 2372\text{ N}$ matches theoretical prediction; car recovers stably. |

---

### 13.2 ISO 4138 Steady-State Circular Test ($R = 50\text{ m}$)

The constant-radius test was executed across 7 discrete forward speeds from $15\text{ km/h}$ to $75\text{ km/h}$ on a $50\text{ m}$ circle. The kinematic Ackermann steer angle is:

$$\delta_{\text{ack}} = \arctan\left(\frac{L}{R}\right) = \arctan\left(\frac{3.0832}{50}\right) = 3.529^\circ$$

| Speed $V$ | Lateral Accel $a_y$ | Steer Angle $\delta$ | $\delta - \delta_{\text{ack}}$ | Roll Angle $\phi$ |
|---|---|---|---|---|
| $15\text{ km/h}$ | $0.035\,g$ | $3.566^\circ$ | $+0.037^\circ$ | $0.11^\circ$ |
| $25\text{ km/h}$ | $0.098\,g$ | $3.603^\circ$ | $+0.074^\circ$ | $0.32^\circ$ |
| $35\text{ km/h}$ | $0.193\,g$ | $3.649^\circ$ | $+0.121^\circ$ | $0.62^\circ$ |
| $45\text{ km/h}$ | $0.318\,g$ | $3.707^\circ$ | $+0.178^\circ$ | $1.03^\circ$ |
| $55\text{ km/h}$ | $0.475\,g$ | $3.782^\circ$ | $+0.254^\circ$ | $1.54^\circ$ |
| $65\text{ km/h}$ | $0.662\,g$ | $3.895^\circ$ | $+0.366^\circ$ | $2.15^\circ$ |
| $75\text{ km/h}$ | $0.876\,g$ | $4.108^\circ$ | $+0.580^\circ$ | $2.84^\circ$ |

#### Observations on Understeer Mechanics:
1. **Linear regime ($0.19g \le a_y \le 0.48g$)**: Linear regression across moderate speeds yields $K_{us} = +0.47^\circ / g$.
2. **Limit envelope ($0.035g \le a_y \le 0.88g$)**: Linear regression across all 7 speeds yields $K_{us} = +0.61^\circ / g$, with an end-to-end secant slope of $+0.65^\circ / g$.
3. **Understeer Mechanism Attribution**: In physical vehicles, roll-stiffness distribution induces understeer through tire normal load sensitivity (degressivity of cornering stiffness with increasing normal load). However, because the current tire model uses $D = \mu F_z$ with fixed $B, C, E$, tire force scales linearly with normal load; consequently, **load-transfer distribution alone cannot generate cornering stiffness loss in this model**. The measured progressive understeer in simulation instead arises from:
   - Friction-ellipse coupling with the 50:50 AWD longitudinal drive forces.
   - Front Ackermann steering geometry divergence between inner and outer wheels.
   - Dynamic roll and pitch attitude altering chassis slip angles.

---

### 13.3 Severe Obstacle Avoidance & Wheel Load Transfer (ISO 3888-2 Track)

During the $72\text{ km/h}$ double lane change manoeuvre, individual corner tire normal loads were sampled continuously throughout the transient:
- **Static per-wheel loads**: Front $F_{z,f0} = 4174.2\text{ N}$, Rear $F_{z,r0} = 4154.5\text{ N}$.
- **Theoretical steady load transfer at $1.0g$**:

$$\Delta F_z \approx m_{\text{tot}} g \cdot \frac{h_{\text{cg}}}{2w} = 1698 \times 9.81 \times \frac{0.52}{1.8565} \approx 4666\text{ N} \quad (\sim 2333\text{ N per corner})$$

- **Sampled dynamic minimum normal load**: **$F_{z,\min} = 1802.6\text{ N}$** (occurring on the inside front wheel at peak reversal).
- **Dynamic load transfer**: $\Delta F_z = 4174.2 - 1802.6 = 2371.6\text{ N}$ ($56.8\%$ reduction on the inside tire), closely matching theoretical load transfer.
- **Stability**: Because the minimum normal force remains well above zero ($F_z \ge 1802.6\text{ N} > 0$), there is zero wheel liftoff or rollover tendency, and the vehicle exits the manoeuvre with near-zero residual yaw rate ($r = 0.0007^\circ/\text{s}$).

---

### 13.4 Reproducibility

The benchmark test harness is fully reproducible in the simulation environment:
```bash
cd ~/super_sim
./run_benchmark.bash
```
or via:
```bash
python3 /home/demirkucukdemiral/super_sim/benchmark_iso_validation.py
```

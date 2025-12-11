# 3 DOF Robotic Arm MATLAB Simulation

This repository contains a MATLAB simulation of a 3-DOF robotic arm based off the "anthropomorphic arm" found in Siciliano's *Robotics: Modelling, Planning and Control*. It was developed for my ME500 final project and uses the kinematic and dynamic derivations from the textbook for forward/inverse kinematics, Jacobian computation, and trajectory planning.

## Quick Start

```matlab
derive_arm_equations
run_sim
```

## Features

- Full rigid body dynamics (mass, inertia, gravity, Coriolis)
- Joint space and operational space control
- Inverse dynamics (computed torque) and PD control
- Cubic trajectory generation
- 3D animation and tracking error plots

## Configuration

Edit `run_sim.m` to change settings:

**Control modes:**

```matlab
params.CONTROL_SPACE = 'JOINT';           % or 'OPERATIONAL'
params.USE_INVERSE_DYNAMICS = true;       % or false for PD control
params.USE_TRAJECTORY = true;             % or false for step input
```

**Control gains:**

```matlab
omega_n = 10;     % Natural frequency (rad/s)
zeta = 1;         % Damping ratio
```

**Robot parameters:**

```matlab
params.m1 = 1.0;  params.m2 = 5.0;  params.m3 = 3.0;  % Masses (kg)
params.a2 = 0.5;  params.a3 = 0.5;                     % Link lengths (m)
```

**Target positions:**

```matlab
target_pos = {[0.5; 0.5; 0.5], [0.0; -0.6; 0.4], [-0.5; 0.5; 0.2]};
```

## Project Structure

- `run_sim.m` - Main simulation script
- `f_arm_dynamics.m` - System dynamics (ODE function)
- `get_*_control.m` - Control laws
- `get_*_kinematics.m` - Forward/inverse kinematics
- `get_*_traj.m` - Trajectory generation
- `generated_functions/` - Auto-generated dynamics matrices (B, C, G, J)
- `visualization/` - Animation and plotting

## Notes

- Requires MATLAB with Symbolic Math Toolbox (for `derive_arm_equations.m` only)
- Max workspace reach: 1.0m (a2 + a3)
- To modify link lengths, re-run `derive_arm_equations.m`

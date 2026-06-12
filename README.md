# ECE 171A Project: Vehicle Steering Control
Car simulation project

## Files
- main_project.m       : Main script. Run this to reproduce all results.
- generate_reference.m : Generates constant-speed reference trajectory 
                         from a parametric curve.
- visualize_tracking.m : 3D driving visualization 
                         (requires Automated Driving Toolbox).

## How to Run
1. Open MATLAB
2. Add all files to the MATLAB path
3. Run main_project.m

## Control Design Overview

### Plant G(s)
The vehicle is modeled as a linear state-space system with two states:
sideslip angle β and yaw rate ψ̇. The input is steering angle u [rad]
and the output is lateral acceleration ÿ_S [m/s²] at the front bumper
sensor. The transfer function G(s) is obtained by converting this
state-space model using tf(ss(A,B,C,D)).

### PID Controller (Kp=1, Ki=0.5, Kd=0.1)
The proportional term provides immediate corrective steering in response
to lateral acceleration error. The integral term eliminates steady-state
error — without it, the vehicle settles ~0.6% off the reference path.
The derivative term dampens oscillations during transient response.
Closed-loop is stable with all poles in the left half plane.

### Lead-Lag Controller (K=5, lead zero=3, lead pole=15, 
###                       lag zero=0.5, lag pole=0.05)
The lead component adds a zero closer to the origin than its pole,
pulling root locus branches leftward and improving transient damping.
The lag component boosts low-frequency gain to reduce steady-state
error, playing a similar role to the integrator in PID but through
pole-zero placement rather than integral action.

### Reference Trajectory
A circular path (R=100m) is parameterized as mathematical function
handles for position, slope, and curvature. generate_reference.m
drives along this circle at v=25 m/s and samples the required lateral
acceleration ddy_ref every millisecond — this is the ideal signal the
controller must track.

### Simulation
lsim simulates the closed-loop transfer function TF responding to
ddy_ref over time, producing ddy_actual — the lateral acceleration
the real vehicle would generate. This is then integrated to recover
the actual x,y trajectory in the global frame for comparison against
the reference path.

## Notes
- visualize_tracking.m requires the Automated Driving Toolbox.
  It is commented out in main_project.m if toolbox is unavailable.
- Reference trajectory can be switched to lemniscate by uncommenting
  the lemniscate struct and updating the generate_reference call.
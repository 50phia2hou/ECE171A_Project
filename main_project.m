%% Vehicle Model
% Constants
v = 25; M = 1573; J = 2873;
lf = 1.10; lr = 1.58;
cf = 80000; cr = 80000;
mu = 0.9; ds = 1.96;

% State Space Variables
A = [ -mu*(cf+cr)/(M*v),        -1 + mu*(cr*lr - cf*lf)/(M*v^2);
       mu*(cr*lr - cf*lf)/J,    -mu*(cf*lf^2 + cr*lr^2)/(J*v)   ];

B = [ mu*cf/(M*v);
      mu*cf*lf/J  ];

C = [ mu*(ds*(cr*lr - cf*lf)/J - (cr+cf)/M),  mu/v*(cr*lr - cf*lf)/M ];

D = mu*(ds*cf*lf/J + cf/M);

% System Model
sys = ss(A, B, C, D);

G = tf(sys);
%% Plots of Plant
pzmap(G)
% Open-loop observations: 2 poles 2 zeroes all negative real part with imaginary parts

% Open-loop plant is stable with oscillatory dynamics (complex poles/zeros).
% This is a well-behaved starting point.
% Negative real parts mean the plant doesn't naturally diverge, but the imaginary parts mean it oscillates.
% Therefore, the controller needs to damp that out and track the reference accurately.

bode(G)

step(G)


%% P Control Design
% Design the controller here
Kp = 1; 
F_p = pid(Kp);

%% Closed-Loop Transfer Function
TF_p = feedback(F_p*G, 1);
%% Plots of P Controller
% Check stability and performance
pzmap(TF_p)

% Pole-zero plot observation: both poles and zeroes have negative real
% parts, but zero overlays on pole.

% Having a zero sitting exactly on top of a pole effectively removes
% that mode from the transfer function. This simplifies the system but
% also means that mode is uncontrollable/unobservable.
% Feedback cannot be influenced.

step(TF_p)
stepinfo(TF_p)

[Gm, Pm, Wgm, Wpm] = margin(F*G)
bw = bandwidth(TF)
% Infinity Gain/Phase margin means that the plot will never be unstable no
% matter how large the gain/phase addition is.


%% PID Control Design
% Design the controller here
Kp = 1; Ki = 0.5; Kd = 0.1;
F = pid(Kp, Ki, Kd);
TF = feedback(F*G, 1);

%% Closed-Loop Transfer Function
TF = feedback(F*G, 1);

%% Plots of PID Controller
% Check stability and performance
pzmap(TF)

% Pole-zero plot observation: both poles and zeroes have negative real
% parts, but zero overlays on pole.

% Having a zero sitting exactly on top of a pole effectively removes
% that mode from the transfer function. This simplifies the system but
% also means that mode is uncontrollable/unobservable.
% This means feedback cannot be influenced.

% infinite margins indicate unconditional stability but suggest the 
% loop gain is low, and that performance improvement came from adding 
% the integrator (Ki) rather than increasing Kp.

step(TF)
stepinfo(TF)
[Gm, Pm, Wgm, Wpm] = margin(F*G)
bw = bandwidth(TF)


%% Lead-Lag Control Design
% Lead: improves transient response (pull poles left)
% Lag: improves steady-state error (adds low-freq gain)

z_lead = 3;    p_lead = 15;   % lead: zero closer to origin than pole
z_lag  = 0.5;  p_lag  = 0.05; % lag: zero further from origin than pole
K = 5;

F_ll = K * tf([1 z_lead],[1 p_lead]) * tf([1 z_lag],[1 p_lag]);

TF_ll = feedback(F_ll*G, 1);

% Analysis
rlocus(F_ll*G)
step(TF_ll)
stepinfo(TF_ll)
[Gm_ll, Pm_ll, Wgm_ll, Wpm_ll] = margin(F_ll*G)
bw_ll = bandwidth(TF_ll)

%% Comparative Plot
figure();
subplot(1,3,1);
rlocus(G);
title('Plant Only');

subplot(1,3,2);
rlocus(F*G);         % PID
title('PID');

subplot(1,3,3);
rlocus(F_ll*G);      % lead-lag
title('Lead-Lag');

%% Reference Trajectory 
v = 25;
dt = 1e-3;
T = 10;

% Circle definition
R = 100;
circle.rx   = @(u)  R*cos(u);
circle.ry   = @(u)  R*sin(u);
circle.drx  = @(u) -R*sin(u);
circle.dry  = @(u)  R*cos(u);
circle.ddrx = @(u) -R*cos(u);
circle.ddry = @(u) -R*sin(u);

% generate_reference with circle
[rx,ry,drx,dry,ddrx,ddry,rho,ddy_ref,psi_ref] = generate_reference(circle,v,dt,T);
ts = (0:dt:T).';

%% Simulation
ddy_actual = lsim(TF, ddy_ref, ts);
rho_actual = ddy_actual / v^2;
psi_actual = psi_ref(1) + cumsum(v * rho_actual * dt);
x_actual   = rx(1) + cumsum(v * cos(psi_actual) * dt);
y_actual   = ry(1) + cumsum(v * sin(psi_actual) * dt);

waypoints = [x_actual, y_actual, zeros(size(x_actual))];
yaw = rad2deg(psi_actual);

%% Plot The Tracking Error
figure();
subplot(2,1,1);
plot(ts, ddy_ref, 'r--', 'DisplayName', 'Reference', 'LineWidth', 1.5); 
hold on;
plot(ts, ddy_actual, 'b-', 'DisplayName', 'Actual', 'LineWidth', 1.0);
hold off;
xlabel('Time [s]');
ylabel('Lateral Acceleration [m/s^2]');
legend('Location','best');
grid on;

subplot(2,1,2);
plot(rx, ry, 'r--', 'DisplayName', 'Reference', 'LineWidth', 1.5);
hold on; 
plot(x_actual, y_actual, 'b-', 'DisplayName', 'Actual', 'LineWidth', 1.0);
hold off; 
xlabel('x [m]'); ylabel('y [m]');
legend('Location','best'); 
grid on;


%% Visualize The Tracking - requires installation of new toolbox

% license('test', 'Automated_Driving_Toolbox')
% visualize_tracking(rx, ry, waypoints, yaw, v, dt);
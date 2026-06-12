function [rx,ry,drx,dry,ddrx,ddry,rho,ddy,psi] = generate_reference(curve,v,dt,T)
%GENERATE_REFERENCE Generate a constant-speed planar trajectory reference.
%
% This function samples a planar parametric curve at approximately constant
% arc-length spacing corresponding to constant travel speed v and sampling
% interval dt.
%
% The curve is parameterized by an arbitrary scalar parameter u:
%
%   r(u) = [x(u); y(u)]
%
% The function computes the arc-length parameterization numerically and
% returns trajectory quantities sampled at uniform time intervals:
%
%   t_k = k*dt,    k = 0,...,N
%
% assuming constant forward speed v.
%
% INPUTS
% -------
% curve : struct or object containing function handles
%     curve.rx(u)    : x-position
%     curve.ry(u)    : y-position
%     curve.drx(u)   : dx/du
%     curve.dry(u)   : dy/du
%     curve.ddrx(u)  : d(dx/du)/du
%     curve.ddry(u)  : d(dy/du)/du
%
% v : scalar
%     Constant traversal speed along the curve [m/s].
%
% dt : scalar
%     Sampling time step [s].
%
% T : scalar
%     Total trajectory duration [s].
%
% OUTPUTS
% --------
% rx, ry : vectors
%     Sampled x- and y-coordinates of the trajectory.
%
% drx, dry : vectors
%     First derivatives of the curve with respect to parameter u.
%
% ddrx, ddry : vectors
%     Second derivatives of the curve with respect to parameter u.
%
% rho : vector
%     Signed curvature of the planar curve:
%
%         rho = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
%
% ddy : vector
%     Lateral acceleration assuming constant speed:
%
%         ddy = v^2 * rho
%
% psi : vector
%     Unwrapped heading angle:
%
%         psi = atan2(dy/du, dx/du)
%
% NOTES
% -----
% The curve parameter u is NOT assumed to correspond to arc length or time.
% The function numerically computes the arc-length mapping and inverts it
% using interpolation to achieve approximately constant-speed traversal.
%
% The parameter domain is adaptively expanded until the curve length
% exceeds the required travel distance:
%
%     S_target = v*T
%
arguments (Input)
    curve
    v
    dt
    T
end

arguments (Output)
    rx
    ry
    drx
    dry
    ddrx
    ddry
    rho
    ddy
    psi
end

% Desired total arc length
S_target = v*T;

% Adaptive discretization of curve parameter u
du = 1e-3;
Umax = 2.0;
while true
    u_grid = (0:du:Umax).';
    vx = curve.drx(u_grid);
    vy = curve.dry(u_grid);
    speed = hypot(vx, vy);
    S = cumtrapz(u_grid, speed);
    if S(end) >= S_target
        break;
    end
    Umax = 2 * Umax;
end

% Desired arc-length samples
ts = (0:dt:T).';
s = v*ts;
u = interp1(S, u_grid, s, 'pchip');

rx  = curve.rx(u);
ry  = curve.ry(u);
drx = curve.drx(u);
dry = curve.dry(u);
ddrx = curve.ddrx(u);
ddry = curve.ddry(u);
rho = (drx .* ddry - dry .* ddrx) ./ (drx.^2 + dry.^2).^(3/2); % Signed curvature
ddy = v^2 * rho; % Lateral acceleration
psi = unwrap(atan2(dry, drx)); % Heading angle

end


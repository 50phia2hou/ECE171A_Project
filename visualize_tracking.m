function visualize_tracking(rx, ry, waypoints, yaw, v, dt)
N = length(rx);
sampleTime = 0.05;
ds = round(sampleTime / dt);
T_horizon = 2.0;
nHorizon = round(T_horizon / dt);

scenario = drivingScenario('SampleTime', sampleTime);
roadStep = max(1, round(2 / (v*dt)));
roadIdx = unique([1:roadStep:N, N]);
road(scenario, [rx(roadIdx), ry(roadIdx), zeros(length(roadIdx),1)]);

ego = vehicle(scenario, 'ClassID', 1);
wp = waypoints(1:ds:end, :);
smoothTrajectory(ego, wp, v*ones(size(wp,1),1), 'Yaw', yaw(1:ds:end));

plot(scenario);

chasePlot(ego, 'ViewHeight', 10, 'ViewPitch', 20, 'ViewLocation', [-15 0]);
ax = gca; 
hold(ax, 'on');
Ref = plot3(ax, NaN, NaN, NaN, 'r-', 'LineWidth', 3);
step = 0;
while advance(scenario)
    step = step + 1;
    i1 = min(step * ds, N);
    i2 = min(i1 + nHorizon, N);
    set(Ref, 'XData', rx(i1:i2), 'YData', ry(i1:i2), ...
        'ZData', 0.2*ones(i2-i1+1, 1));
    pause(0.05);
end
end


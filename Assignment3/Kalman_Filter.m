%Kalman Filter
%Implement the Kalman filter/predictor and estimate the velocity and acceleration from
%noisy position measurements (see .mat file)
addpath('../simulink/');
addpath('../method/');

Ts = 0.001;
simTime = 10;
time = 0:Ts:simTime;

% Simulate and get the position timeseries
out = sim('../simulink/signalsKF.slx', simTime);
x = out.position_GaussianNoise.signals.values;
%x = out.position_QuantizationNoise.Data;
dx = out.velocity.signals.values;
ddx = out.acceleration.signals.values;


%% Kalman filter

[~, f_dx, f_ddx] = KF(x, time, false);

%% Steady state Kalman filter
[~, ssf_dx, ssf_ddx] = KF(x, time, true);

%% Kalman predictor

[~, p_dx, p_ddx] = KP(x, time, false);

%% Steady state Kalman predictor

[~, ssp_dx, ssp_ddx] = KP(x, time, true);

%% Plot the results

figure(); sgtitle("Position signal with Gaussian noise");

subplot(321); title("Kalman filter/predictor"); grid on; hold on;
plot(time, x, 'LineWidth', 2); ylabel("Position [m or rad]");

subplot(322); title("Steady state Kalman filter/predictor"); grid on; hold on;
plot(time, x, 'LineWidth', 2); ylabel("Position [m or rad]");

subplot(323); xlabel("Time [s]"); ylabel("Velocity [m/s or rad/s]"); grid on; hold on;
plot(time, dx, 'LineWidth', 2); hold on; plot(time, f_dx, 'LineWidth', 2); hold on; plot(time, p_dx, 'LineWidth', 2); legend("Actual", "Filter", "Predictor");

subplot(325);  grid on; xlabel("Time [s]"); ylabel("Acceleration [m/s^2 or rad/s^2]"); hold on;
plot(time, ddx, 'LineWidth', 2); hold on; plot(time, f_ddx, 'LineWidth', 2); hold on; plot(time, p_ddx, 'LineWidth', 2); legend("Actual", "Filter", "Predictor");

subplot(324); grid on; ylabel("Velocity [m/s or rad/s]"); hold on;
plot(time, dx, 'LineWidth', 2); hold on; plot(time, ssf_dx, 'LineWidth', 2); hold on; plot(time, ssp_dx, 'LineWidth', 2); legend("Actual", "Filter", "Predictor");

subplot(326); grid on; xlabel("Time [s]"); ylabel("Acceleration [m/s^2 or rad/s^2]"); hold on;
plot(time, ddx, 'LineWidth', 2); hold on; plot(time, ssf_ddx, 'LineWidth', 2); hold on; plot(time, ssp_ddx, 'LineWidth', 2); legend("Actual", "Filter", "Predictor");
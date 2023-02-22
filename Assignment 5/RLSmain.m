% Assignment 5: Parameter identification with LS and RLS
%SETUP

close all;
clear all;

addpath('../simulink/')
addpath('../method/')

% Load the DC motor's parameters
DCmotor_parameters;
% Run the simulink model
out = sim('DCmotor_maxon_Pcontrol', 5);
position = out.positions.Data;
voltages = out.voltages.Data;
time = out.positions.Time;
actual_velocity = out.actual_vel.Data;
actual_acceleration = out.actual_acc.Data;

%% Get velocity and acceleration with Kalman smoother

[theta, omega, dOmega] = KS(position);

%% Least Square estimation 
X(:,1) = dOmega; X(:,2) = omega;
Y(:,1) = voltages;
%vettore di parametri che stiamo stimando 
beta_LS = inv(X.'*X)*X.' * Y;
%stima delle tensioni usando i parametri di beta_LS
y_LS = X*beta_LS;
fprintf("LS estimation: k=%.4f, tau=%.4f\n", 1/beta_LS(2), beta_LS(1)/beta_LS(2));

%% Recursive Least Square estimation 
%per tenere conto dei valori più recenti o più vecchi 
lambda1 = 0.8;
lambda2 = 0.975;
lambda3 = 0.999;
[y_RLS1, ~] = RLS(X, Y, lambda1);
[y_RLS2, beta_RLS] = RLS(X, Y, lambda2);
[y_RLS3, ~] = RLS(X, Y, lambda3);
%disp da aggiungere
fprintf("RLS estimation: k=%.4f, tau=%.4f\n", 1/beta_RLS(2), beta_RLS(1)/beta_RLS(2));

%% Plots
figure; plot(time, Y, 'LineWidth', 2); hold on;
plot(time, y_LS, 'LineWidth', 2);
plot(time, y_RLS1, 'LineWidth', 2);
plot(time, y_RLS2, 'LineWidth', 2);
plot(time, y_RLS3, 'LineWidth', 2);
legend("Actual", "LS", "RLS - \lambda=0.8", "RLS - \lambda=0.975", "RLS - \lambda=0.999"); grid on;
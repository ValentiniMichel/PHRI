% Assignment 6
% Implement the Scattering-based bilateral teleoperation architecture for the
%   1. Force-Position case
%   2. Position-Position case
% Compare positions, velocities, forces, commands in free motion and in contact
% Create another simulink model and (a) add the measurement noise to the
%position/force signals, and (b) estimate velocities from positions
clear all;
close all;
clc;

% assignment 6 delay
s = tf('s');

%master robot impedance
Mm = 0.5;
Dm = 10;

%slave robot impedance
Ms = 2;
Ds = 20;

Bm = 20*0.8;
Km = 10;

%slave robot controller 
Bs = 4*Bm;
Ks = 4*Km;

%operator impedance params
Jh_i = 0;
Bh_i = 1;
Kh_i = 1;

%operator controller
Dh = 20*0.8;
Kh = 10;

%input force
A_int = 1;

%env params
Be = 100;
Ke = 200;
xe = 0.4;

Ts = 0.001;

delayT = 10;
b = 1;
Fip = 10;
Fc = 0.5;
%Frequency filter
Ff = 0.001;

A = [1 Ts;
     0 1];

B = [Ts^2/2;Ts];

x0 = [0 0];

C = [1 0];

q_m = 10000000;

R = 1;

Q_m = q_m*B*B';
q_s = 10000000;
Q_s = q_s*B*B';
posNoiseVariance = 0;
forceNoiseVariance = 0;

open('../simulink/scattering_FP.slx');
%sim('../simulink/scattering_FP.slx');

%% in contact
xe = 0.9;
sim('../simulink/scattering_FP.slx');

%% with noise
posNoiseVariance = 0.000000000000001;
forceNoiseVariance = 0.00000000000001;
xe = 0.7;

out = sim('../simulink/scattering_FP.slx');
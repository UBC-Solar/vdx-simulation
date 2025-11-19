%% Rough Rocker Calculations

clc; format short g
x=1;y=2;z=3;

g = 9.81;

run("mainHardpointForces.m")
close;
clc;

trakWidth = trackWidth/1000; % m

k_spring = 87563.5; % 500 lb/in from shock

% Method 1

f_tp = 1800; % N

m = f_tp/g; %kg

f1 = 2; % Hz

k_ride1 = (2*pi*f1)^2 * m % N/m

k_roll = k_ride1*trakWidth^2/2 % (N m)/rad


% Method 2

delta_f = f_tp - wS_FL*g

travel = 0.0508;

k_ride2 = delta_f/travel

f2 = 1/(2*pi) * sqrt(k_ride2/m)


% Motion Ratios

MR1 = sqrt(k_ride1/k_spring)

MR2 = sqrt(k_ride2/k_spring)



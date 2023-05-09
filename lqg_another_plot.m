clear all;
close all;
clc;

% Define the system matrices
A = [0 1 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 0;
     0 0 0 0 0 1;
     0 0 0 0 0 0];

Ixx = 1;
Iyy = 1;
Izz = 1;

B = [0 0 0;
     1/Ixx 0 0;
     0 0 0;
     0 1/Iyy 0;
     0 0 0;
     0 0 1/Izz];

C = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];

D = zeros(6, 3);

% Define the weighting matrices
Q = diag([1, 1, 1, 1, 1, 1]);
R = eye(3);
N = zeros(6, 3);

% Construct the LQG state-space model
sys = ss(A, B, C, D);

% Design the LQG controller
[K, ~, ~] = lqry(sys, Q, R, N);

% Define the initial state and time span
x0 = [0.1; 0.1; 0.1; 0.1; 0.1; 0.1];
tspan = linspace(0, 20, 1000);

% Define the LQG controller function
lqg_controller = @(t, x) -K * x;

% Simulate the system with the LQG controller
[t, x] = ode45(@(t, x) A * x + B * lqg_controller(t, x), tspan, x0);

% Define the control inputs function
control_inputs = @(t, x) -K * x';

% Interpolate the state trajectory
x_traj = interp1(t, x, tspan, 'linear', 'extrap');

% Compute control inputs
u = zeros(length(tspan), 3);
for i = 1:length(tspan)
    u(i, :) = control_inputs(tspan(i), x_traj(i, :));
end

% Plot the results
figure;
plot(tspan, u(:, 1), tspan, u(:, 2), tspan, u(:, 3));
title('Control Inputs');
xlabel('Time (s)');
ylabel('Control Input');
legend('Thrust', 'Torque 1', 'Torque 2');

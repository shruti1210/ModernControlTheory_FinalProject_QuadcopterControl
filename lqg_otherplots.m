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
control_inputs = @(t, x) -K * x;

% Simulate the system with the LQG controller and get the control inputs
[t, x] = ode45(@(t, x) A * x + B * control_inputs(t, x), tspan, x0);
% Simulate the system with the LQG controller and get the control inputs


% Compute the error for each state variable
desired_state = [0; 0; 0; 0; 0; 0];
error = x - repmat(desired_state', length(t), 1);

% Plot the results
figure;
plot(t, error(:, 1), t, error(:, 2), t, error(:, 3), t, error(:, 4), t, error(:, 5), t, error(:, 6));
title('Error');
xlabel('Time (s)');
ylabel('Error');
legend('Error in phi', 'Error in dphi', 'Error in theta', 'Error in dtheta', 'Error in psi', 'Error in dpsi');

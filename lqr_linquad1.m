clear all;
close all;
clc;

A = [0 1 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 0;
     0 0 0 0 0 1;
     0 0 0 0 0 0];

Ixx = 10;
Iyy = 10;
Izz = 10;

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

D = zeros(6,3);

% Define the weighting matrices
%Q = diag([1, 1, 1, 1, 1, 1]) + C'*Qf*C;
Q = diag([1, 1, 1, 1, 1, 1]);
R = diag([10, 10, 10]);

% Compute the optimal control gain matrix K
[P, ~, ~] = dare(A, B, Q, R);
K = -inv(R)*B'*P;

% Define the initial state and time span
x0 = [0.1; 0.1; 0.1; 0.1; 0.1; 0.1];
tspan = linspace(0, 20, 1000);

% Define the LQR controller function
lqr_controller = @(t,x) (A - B*K)*x;

% Simulate the system with the LQR controller
[t, x] = ode45(lqr_controller, tspan, x0);

% Plot the results
figure;
plot(t, x(:, 1), t, x(:, 2), t, x(:, 3), t, x(:, 4), t, x(:, 5), t, x(:, 6));
title('LQR Controller Response');
xlabel('Time (s)');
ylabel('State');
legend('phi', 'dphi', 'theta', 'dtheta', 'psi', 'dpsi');

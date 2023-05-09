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

% Plot the results
figure;
plot(t, x(:, 1), t, x(:, 2), t, x(:, 3), t, x(:, 4), t, x(:, 5), t, x(:, 6));
title('LQG Controller Response');
xlabel('Time (s)');
ylabel('State');
legend('phi', 'dphi', 'theta', 'dtheta', 'psi', 'dpsi');

% % Animation
% figure;
% hold on;
% grid on;
% axis equal;
% axis([-1 1 -1 1 -1 1]);
% 
% % Define the faces for each side of the quadcopter body
% faces = [1 2 3 4; 4 3 7 8; 8 7 6 5; 5 6 2 1; 2 6 7 3; 1 4 8 5];
% 
% for i = 1:length(t)
%     phi = x(i, 1);
%     theta = x(i, 3);
%     psi = x(i, 5);
%     
%     % Define the quadcopter geometry (modify as needed)
%     arm_length = 0.1;
%     body_length = 0.2;
%     body_width = 0.2;
%     body_height = 0.1;
%     
%     % Define the vertices of the quadcopter body
%     body_vertices = [
%         -body_length/2 -body_width/2 -body_height/2;
%         -body_length/2 body_width/2 -body_height/2;
%         body_length/2 body_width/2 -body_height/2;
%         body_length/2 -body_width/2 -body_height/2;
%         -body_length/2 -body_width/2 body_height/2;
%         -body_length/2 body_width/2 body_height/2;
%         body_length/2 body_width/2 body_height/2;
%         body_length/2 -body_width/2 body_height/2
%     ];
%     
%     % Define the vertices of the quadcopter arms
%     arm_vertices = [
%         -arm_length/2 -body_width/2 -body_height/2;
%         -arm_length/2 body_width/2 -body_height/2;
%         arm_length/2 body_width/2 -body_height/2;
%         arm_length/2 -body_width/2 -body_height/2;
%         -arm_length/2 -body_width/2 0;
%         -arm_length/2 body_width/2 0;
%         arm_length/2 body_width/2 0;
%         arm_length/2 -body_width/2 0
%     ];
%     
%     % Rotate the body vertices based on the orientation angles
%     R = [
%         cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi);
%         cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi), sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi);
%         -cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)
%     ];
%     rotated_body_vertices = R * body_vertices';
%     rotated_body_vertices = rotated_body_vertices';
%     
%     % Plot the quadcopter body
%     patch('Faces', faces, 'Vertices', rotated_body_vertices, 'FaceColor', 'blue');
%     
%     % Plot the quadcopter arms
%     for j = 1:4
%         rotated_arm_vertices = R * arm_vertices';
%         rotated_arm_vertices = rotated_arm_vertices';
%         translated_arm_vertices = rotated_arm_vertices + rotated_body_vertices(j, :);
%         patch('Vertices', translated_arm_vertices, 'Faces', [1 2 3 4], 'FaceColor', 'red');
%     end
%     
%     drawnow;
% end
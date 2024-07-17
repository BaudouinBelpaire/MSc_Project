% Forward displacement kinematics and force analysis of a 4-DOF robot
clear all
close all
clc

tol = 0.1;
z_target_arr = [-7,-17];

L0= 12.5;
L1= 0;
L2= 2;
L3= 0;
L4 = 14;
L5 = 0;
L6 = 0;
L7 = 0;
L8 = 14;

Alpha0 = 0;
Alpha1 = 0;
Alpha2 = pi/2;
Alpha3 = -pi/2;
Alpha4 = 0;
Alpha5 = 0;
Alpha6 = pi/2;
Alpha7 = -pi/2;
Alpha8 = 0;

d1= 0;
d2= -5;
d3= 0;
d4= -2;
d5= 0;
d6= -2;
d7= 0;
d8 = 2;
d9 = 0;

% Input joint variables
% Theta1, Theta2, Theta3
Theta1= 0; %axis e
Theta2= 0;
Theta3= -pi/4; %axis d
Theta4= 0;
Theta5= 0;
Theta6= 0; 
Theta7 = -pi/2 - Theta3; %axis c
Theta8 = 0;
Theta9 = 0; %axis b

rad_step = 0.01;
for j=1:2
Theta3_arr = -pi/2:rad_step:pi/2;
z_target = z_target_arr(1,j);
for i=1:size(Theta3_arr,2)

Theta3 = Theta3_arr(1,i);
Theta7 = -pi/2 - Theta3;

DH_params = [
        Alpha0, L0, d1, Theta1;
        Alpha1, L1, d2, Theta2;
        Alpha2, L2, d3, Theta3;
        Alpha3, L3, d4, Theta4;
        Alpha4, L4, d5, Theta5;
        Alpha5, L5, d6, Theta6;
        Alpha6, L6, d7, Theta7;
        Alpha7, L7, d8, Theta8;
        Alpha8, L8, d9, Theta9;
 ];

% Initialize transformation matrix
T = eye(4);
    
% Compute the position of each joint
for i = 1:size(DH_params,1)
    Alpha = DH_params(i, 1);
    L = DH_params(i, 2);
    d = DH_params(i, 3);
    Theta = DH_params(i, 4);
        
    % Transformation matrix for current joint
    Tx=[1,0,0,L;0,1,0,0;0,0,1,0;0,0,0,1];
    Rx=[1,0,0,0;0,cos(Alpha),-sin(Alpha),0;0,sin(Alpha),cos(Alpha),0;0,0,0,1];
    Rz=[cos(Theta),-sin(Theta),0,0;sin(Theta),cos(Theta),0,0;0,0,1,0;0,0,0,1];
    Tz=[1,0,0,0;0,1,0,0;0,0,1,d;0,0,0,1];
    T = T * Tx*Rx*Rz*Tz;
    
end
err = abs(z_target-T(3,4));
if(err<tol)
    err
    Theta3, Theta7
    break
end 
end 

% Initialize transformation matrix
T = eye(4);
    
% Initialize arrays to store joint positions
x = zeros(1, 9);
y = zeros(1, 9);
z = zeros(1, 9);

% Compute the position of each joint
for i = 1:size(DH_params,1)
    Alpha = DH_params(i, 1);
    L = DH_params(i, 2);
    d = DH_params(i, 3);
    Theta = DH_params(i, 4);
        
    % Transformation matrix for current joint
    Tx=[1,0,0,L;0,1,0,0;0,0,1,0;0,0,0,1];
    Rx=[1,0,0,0;0,cos(Alpha),-sin(Alpha),0;0,sin(Alpha),cos(Alpha),0;0,0,0,1];
    Rz=[cos(Theta),-sin(Theta),0,0;sin(Theta),cos(Theta),0,0;0,0,1,0;0,0,0,1];
    Tz=[1,0,0,0;0,1,0,0;0,0,1,d;0,0,0,1];
    T = T * Tx*Rx*Rz*Tz;
        
    % Store the position of the current joint
    x(i+1) = T(1, 4);
    y(i+1) = T(2, 4);
    z(i+1) = T(3, 4);
end

% Plot the robot
figure;
plot3(x, y, z, 'o-', 'LineWidth', 2, 'MarkerSize', 10, 'MarkerFaceColor', 'r');
hold on;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('4-DOF Robot Visualization');
axis equal;
end

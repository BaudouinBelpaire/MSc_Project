% Forward displacement kinematics and force analysis of a 4-DOF robot
clear all 
close all
clc

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
    T = T * Tx*Rx*Rz*Tz
        
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

% Define the HT matrix for transaltion along the X-axis
Tx01=[1,0,0,L0;0,1,0,0;0,0,1,0;0,0,0,1];
Tx12=[1,0,0,L1;0,1,0,0;0,0,1,0;0,0,0,1];
Tx23=[1,0,0,L2;0,1,0,0;0,0,1,0;0,0,0,1];
Tx34=[1,0,0,L3;0,1,0,0;0,0,1,0;0,0,0,1];
Tx45=[1,0,0,L4;0,1,0,0;0,0,1,0;0,0,0,1];
Tx56=[1,0,0,L5;0,1,0,0;0,0,1,0;0,0,0,1];
Tx67=[1,0,0,L6;0,1,0,0;0,0,1,0;0,0,0,1];
Tx78=[1,0,0,L7;0,1,0,0;0,0,1,0;0,0,0,1];
% Define the HT matrix for Rotaltion along the X-axis
Rx01=[1,0,0,0;0,cos(Alpha0),-sin(Alpha0),0;0,sin(Alpha0),cos(Alpha0),0;0,0,0,1];
Rx12=[1,0,0,0;0,cos(Alpha1),-sin(Alpha1),0;0,sin(Alpha1),cos(Alpha1),0;0,0,0,1];
Rx23=[1,0,0,0;0,cos(Alpha2),-sin(Alpha2),0;0,sin(Alpha2),cos(Alpha2),0;0,0,0,1];
Rx34=[1,0,0,0;0,cos(Alpha3),-sin(Alpha3),0;0,sin(Alpha3),cos(Alpha3),0;0,0,0,1];
Rx45=[1,0,0,0;0,cos(Alpha4),-sin(Alpha4),0;0,sin(Alpha4),cos(Alpha4),0;0,0,0,1];
Rx56=[1,0,0,0;0,cos(Alpha5),-sin(Alpha5),0;0,sin(Alpha5),cos(Alpha5),0;0,0,0,1];
Rx67=[1,0,0,0;0,cos(Alpha6),-sin(Alpha6),0;0,sin(Alpha6),cos(Alpha6),0;0,0,0,1];
Rx78=[1,0,0,0;0,cos(Alpha7),-sin(Alpha7),0;0,sin(Alpha7),cos(Alpha7),0;0,0,0,1];
% Define the HT matrix for Rotation along the Z-axis
Rz01=[cos(Theta1),-sin(Theta1),0,0;sin(Theta1),cos(Theta1),0,0;0,0,1,0;0,0,0,1];
Rz12=[cos(Theta2),-sin(Theta2),0,0;sin(Theta2),cos(Theta2),0,0;0,0,1,0;0,0,0,1];
Rz23=[cos(Theta3),-sin(Theta3),0,0;sin(Theta3),cos(Theta3),0,0;0,0,1,0;0,0,0,1];
Rz34=[cos(Theta4),-sin(Theta4),0,0;sin(Theta4),cos(Theta4),0,0;0,0,1,0;0,0,0,1];
Rz45=[cos(Theta5),-sin(Theta5),0,0;sin(Theta5),cos(Theta5),0,0;0,0,1,0;0,0,0,1];
Rz56=[cos(Theta6),-sin(Theta6),0,0;sin(Theta6),cos(Theta6),0,0;0,0,1,0;0,0,0,1];
Rz67=[cos(Theta7),-sin(Theta7),0,0;sin(Theta7),cos(Theta7),0,0;0,0,1,0;0,0,0,1];
Rz78=[cos(Theta8),-sin(Theta8),0,0;sin(Theta8),cos(Theta8),0,0;0,0,1,0;0,0,0,1];
% Define the HT matrix for Translation along the Z-axis
Tz01=[1,0,0,0;0,1,0,0;0,0,1,d1;0,0,0,1];
Tz12=[1,0,0,0;0,1,0,0;0,0,1,d2;0,0,0,1];
Tz23=[1,0,0,0;0,1,0,0;0,0,1,d3;0,0,0,1];
Tz34=[1,0,0,0;0,1,0,0;0,0,1,d4;0,0,0,1];
Tz45=[1,0,0,0;0,1,0,0;0,0,1,d5;0,0,0,1];
Tz56=[1,0,0,0;0,1,0,0;0,0,1,d6;0,0,0,1];
Tz67=[1,0,0,0;0,1,0,0;0,0,1,d7;0,0,0,1];
Tz78=[1,0,0,0;0,1,0,0;0,0,1,d8;0,0,0,1];
% Define the HT matrix for each link
T01=Tx01*Rx01*Rz01*Tz01;
T12=Tx12*Rx12*Rz12*Tz12;
T23=Tx23*Rx23*Rz23*Tz23;
T34=Tx34*Rx34*Rz34*Tz34;
T45=Tx45*Rx45*Rz45*Tz45;
T56=Tx56*Rx56*Rz56*Tz56;
T67=Tx67*Rx67*Rz67*Tz67;
T78=Tx78*Rx78*Rz78*Tz78;
% Forward Kinematics
T01
T02=T01*T12
T03=T02*T23
T04=T01*T12*T23*T34
T05=T04*T45
T06=T05*T56
T07=T06*T67
T08=T07*T78
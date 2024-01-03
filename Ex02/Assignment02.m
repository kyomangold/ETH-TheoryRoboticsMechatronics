%%  TRM Assignment 02

clear;
close all;
clc;
 
 
%% for system 1
P1 = [1 0 0 1]';
P2 = [0 1 0 1]';
P3 = [0 0 1 1]';

g0_1 = [1 0 0 0;
          0 1 0 5;
          0 0 1 0;
          0 0 0 1];
theta1 = 0:0.1:4;

v1 = [0 0 1 0 0 0]';
xi1 = twist(v1);

P1_traj = [];
P2_traj = [];
P3_traj = [];

for i = 1:length(theta1)
    g_theta = twistexp(xi1, theta1(i))*g0_1;
    P1_traj = [P1_traj g_theta*P1];
    P2_traj = [P2_traj g_theta*P2];
    P3_traj = [P3_traj g_theta*P3];
end

figure, 
sgtitle('System 1')
subplot(2,2,1)
plot(theta1, P1_traj(1,:), 'b', 'LineWidth',2);
hold on
plot(theta1, P2_traj(1,:), 'g', 'LineWidth',2);
plot(theta1, P3_traj(1,:), 'r', 'LineWidth',2);
grid on
xlabel('\theta')
ylabel('X')
legend('[1 0 0]','[0 1 0]','[0 0 1]')

subplot(2,2,2)
plot(theta1, P1_traj(2,:), 'b', 'LineWidth',2);
hold on
plot(theta1, P2_traj(2,:), 'g', 'LineWidth',2);
plot(theta1, P3_traj(2,:), 'r', 'LineWidth',2);
grid on
xlabel('\theta')
ylabel('Y')
legend('[1 0 0]','[0 1 0]','[0 0 1]')

subplot(2,2,3)
plot(theta1, P1_traj(3,:), 'b', 'LineWidth',2);
hold on
plot(theta1, P2_traj(3,:), 'g', 'LineWidth',2);
plot(theta1, P3_traj(3,:), 'r', 'LineWidth',2);
grid on
xlabel('\theta')
ylabel('Z')
legend('[1 0 0]','[0 1 0]','[0 0 1]')

subplot(2,2,4)
plot3(P1_traj(1,:), P1_traj(2,:), P1_traj(3,:), 'b', 'LineWidth',2);
hold on
plot3(P2_traj(1,:), P2_traj(2,:), P2_traj(3,:), 'g', 'LineWidth',2);
plot3(P3_traj(1,:), P3_traj(2,:), P3_traj(3,:), 'r', 'LineWidth',2);
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
legend('[1 0 0]','[0 1 0]','[0 0 1]')


%% for system 2
P1 = [1 0 0 1]';
P2 = [0 1 0 1]';
P3 = [0 0 1 1]';

g0_2 = [0 0 1 2;
          0 -1 0 3;
          1 0 0 0;
          0 0 0 1];
theta2 = 0:pi/20:pi/2;

v2 = createtwist([1 0 0], [2 3 0]);
xi2 = twist(v2);

P1_traj = [];
P2_traj = [];
P3_traj = [];

for i = 1:length(theta2)
    g_theta = twistexp(xi2, theta2(i))*g0_2;
    P1_traj = [P1_traj g_theta*P1];
    P2_traj = [P2_traj g_theta*P2];
    P3_traj = [P3_traj g_theta*P3];
end

figure, 
sgtitle('System 2')
subplot(2,2,1)
plot(theta2, P1_traj(1,:), 'b', 'LineWidth',2);
hold on
plot(theta2, P2_traj(1,:), 'g', 'LineWidth',2);
plot(theta2, P3_traj(1,:), 'r', 'LineWidth',2);
grid on
xlabel('\theta')
ylabel('X')
xlim([0 pi/2])
legend('[1 0 0]','[0 1 0]','[0 0 1]')

subplot(2,2,2)
plot(theta2, P1_traj(2,:), 'b', 'LineWidth',2);
hold on
plot(theta2, P2_traj(2,:), 'g', 'LineWidth',2);
plot(theta2, P3_traj(2,:), 'r', 'LineWidth',2);
grid on
xlabel('\theta')
ylabel('Y')
xlim([0 pi/2])
legend('[1 0 0]','[0 1 0]','[0 0 1]')

subplot(2,2,3)
plot(theta2, P1_traj(3,:), 'b', 'LineWidth',2);
hold on
plot(theta2, P2_traj(3,:), 'g', 'LineWidth',2);
plot(theta2, P3_traj(3,:), 'r', 'LineWidth',2);
grid on
xlabel('\theta')
ylabel('Z')
xlim([0 pi/2])
legend('[1 0 0]','[0 1 0]','[0 0 1]')

subplot(2,2,4)
plot3(P1_traj(1,:), P1_traj(2,:), P1_traj(3,:), 'b', 'LineWidth',2);
hold on
plot3(P2_traj(1,:), P2_traj(2,:), P2_traj(3,:), 'g', 'LineWidth',2);
plot3(P3_traj(1,:), P3_traj(2,:), P3_traj(3,:), 'r', 'LineWidth',2);
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
legend('[1 0 0]','[0 1 0]','[0 0 1]')


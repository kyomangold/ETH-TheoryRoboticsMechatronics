%% TRM Assignment 03

clear all
close all
clc


%% definitions for both systems

theta1 = -pi/4:pi/36:pi/2; % theta range of joint 1
theta2 = -pi/2:pi/36:pi/2; % theta range of joint 2
omega1 = [1 0 0]';         % omega of joint 1
omega2 = [1 0 0]';         % omega of joint 2
n = length(theta1)*length(theta2); % number of theta configurations


%% System 1

disp('System 1');

g_0t = transl(0,8,5) % hom. transformation g_0t(0)
q1 = [0 0 5]';
q2 = [0 5 5]';

v1 = createtwist(omega1, q1); % twist coordinates (6x1), joint 1
v2 = createtwist(omega2, q2); % twist coordinates (6x1), joint 2
xi1 = twist(v1) % twist (4x4), joint 1
xi2 = twist(v2) % twist (4x4), joint 2

g_b0 = eye(4) % hom. transformation g_b0(0)

y_sys1 = zeros(n,1);
z_sys1 = zeros(n,1);

index = 0;
for i = 1:length(theta1),
    for j = 1:length(theta2),
        index = index + 1;
        % calculating g(theta)
        g_theta = g_b0*twistexp(xi1,theta1(i))*twistexp(xi2,theta2(j))*g_0t;
        v_temp = g_theta*[0; 0; 0; 1]; % applying g(theta) on origin of frame t
        % extracting y- and z-component
        y_sys1(index) = v_temp(2);
        z_sys1(index) = v_temp(3);
    end
end



%% System 2

disp('System 2');

g_0t = eye(4) % hom. transformation g_0t(0)
q1 = [0 -8 0]';
q2 = [0 -3 0]';

v1 = createtwist(omega1, q1); % twist coordinates (6x1), joint 1
v2 = createtwist(omega2, q2); % twist coordinates (6x1), joint 2
xi1 = twist(v1) % twist (4x4), joint 1
xi2 = twist(v2) % twist (4x4), joint 2

g_b0 = transl(0,8,5) % hom. transformation g_b0(0)

y_sys2 = zeros(n,1);
z_sys2 = zeros(n,1);

index = 0;
for i = 1:length(theta1),
    for j = 1:length(theta2),
        index = index + 1;
        % calculating g(theta)
        g_theta = g_b0*twistexp(xi1,theta1(i))*twistexp(xi2,theta2(j))*g_0t;
        v_temp = g_theta*[0; 0; 0; 1]; % applying g(theta) on origin of frame t
        % extracting y- and z-component
        y_sys2(index) = v_temp(2);
        z_sys2(index) = v_temp(3);
    end
end


%% plot the workspace
figure(1);
plot(y_sys1,z_sys1,'ko',y_sys2,z_sys2,'kx');
xlabel('y_b');
ylabel('z_b');
axis equal;
grid on;
legend('System 1','System 2');
title('Screw Theory');



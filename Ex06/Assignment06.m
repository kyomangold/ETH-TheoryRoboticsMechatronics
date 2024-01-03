%% TRM Assignment 06 

clear all
close all
clc
 
%% Define the constants

n    =  10;    % number of points in circular trajectory
beta =  0.8;   % gain to control algorithm's speed of convergence
 
error_criteria =  0.01; % convergence criteria on error

a1 = 3; % length of link 1
a2 = 4; % length of link 2
r  = 2; % radius of circular trajectory
center = [5; 0]; % center of circular trajectory


%% Generate circular trajectory with n points

theta = linspace(0,2*pi,n+1); % get the angles for n equally spaced points


% x,y components on circular trajectory
Circle(:,1) = center(1) + r*cos(theta(2:end));   % x
Circle(:,2) = center(2) + r*sin(theta(2:end));  % y
 
 
%% Initialize robot in zero configuration
 
X_a = [a1+a2;0]; % zero-position of end-effector
theta1 = 0;   % zero configuration
theta2 = 0;   % zero configuration
 
m = 1; % index for iteration

X_a_pt = [];
S_hist = [];
manipulability = [];
%% Implement numerical inverse kinematics
 
joints = zeros(2,3); % joint parameters 

X_a_hist = X_a; % x,y position of end effector
theta1_hist = theta1;
theta2_hist = theta2;

for i = 1:n
  % for each point on the circle
  X_des = [Circle(i,1); Circle(i,2)]; % desired position from circle
  error =   X_des - X_a % error between desired and actual position
  
  while norm(error) > error_criteria  % until we converge on the point
    joints(:,2) = a1*[cos(theta1); sin(theta1)];
    joints(:,3) = X_a;
    
    % plot the current configuration
    figure(1),
    clf
    plot(Circle(:,1), Circle(:,2), 'ro', X_a_hist(1,:), X_a_hist(2,:), 'k-');
    hold on;
    plot(joints(1,:), joints(2,:), 'r-o', 'LineWidth', 2);
    axis equal;
    xlabel('X');
    ylabel('Y');
    axis([-(a1+a2) (a1+a2) -(a1+a2) (a1+a2)]);
    pause(0.01)
    
    % compute Jacobian
    J = [-a1*sin(theta1)-a2*sin(theta1+theta2)	-a2*sin(theta1+theta2);
         a1*cos(theta1)+a2*cos(theta1+theta2)	a2*cos(theta1+theta2)]; %derive Jacobian by hand, first setup position equations for position x,y
    DeltaV = beta*error; % compute desired direction to move
    DeltaTheta = pinv(J)*DeltaV; % compute best choice achievable

    % new thetas
    theta1 = theta1 + DeltaTheta(1);
    theta2 = theta2 + DeltaTheta(2);
    % new position
    X_a = [ a1*cos(theta1)+a2*cos(theta1+theta2);
           a1*sin(theta1)+a2*sin(theta1+theta2)]; % x,y position of end-effector 
    X_a_hist = [X_a_hist X_a];      
    theta1_hist = [theta1_hist theta1];
    theta2_hist = [theta2_hist theta2];

    % new error
    error = X_des - X_a;
    
    
  end

  % calculate manipulability and save axes of manip. ellipsoid
  
  [U,S,V] = svd(J); % singular value decomposition 
  manipulability = [manipulability prod(diag(S))];
  S_hist = [S_hist diag(S)]; %singular values
  U_hist(1:4, i) = [U(:,1); U(:,2)];

end
 
 
%% make plots
title_label = sprintf('n = %3i, \\beta = %0.2f', n, beta);

% desired points as red circles, actual trajectory as a black line
figure,
title(title_label);
subplot(2,2,[1,3])
plot(Circle(:,1), Circle(:,2), 'ro', X_a_hist(1,:), X_a_hist(2,:), 'k-');
hold on;
axis equal;
xlabel('X');
ylabel('Y');
grid on; 


% change of thetas at each iteration
subplot(2,2,2), plot(theta1_hist);
xlabel('Iterations');
ylabel('\theta_1');
subplot(2,2,4), plot(theta2_hist);
xlabel('Iterations');
ylabel('\theta_2');

%plot manipulability measure W 
figure;plot(1:n,manipulability)
grid on
xlabel('Number of points');
ylabel('Manipulability: \sigma_1 * \sigma_2');


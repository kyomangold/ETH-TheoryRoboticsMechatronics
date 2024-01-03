%% TRM Assignment 07

clear all;
close all;
clc;


%% Define variables 

syms th1 th2 th3 th4 th5 th6 l0 l1 real

x = [1 0 0]'; y = [0 1 0]'; z = [0 0 1]';

w1 = z; w2 = -x; v3 = y; w3 = [0 0 0]'; 
w4 = z; w5 = -x; w6 = y;

q1 = [0 0 l0]';
q2 = [0 0 l0]';
q3 = [0 0 l0]';
q4 = [0 l1 l0]';
q5 = [0 l1 l0]';
q6 = [0 l1 l0]';


%% Compute twist and matrix exponential

xi1 = [-cross(w1,q1); w1];
xi2 = [-cross(w2,q2); w2];
xi3 = [v3; w3];
xi4 = [-cross(w4,q4); w4];
xi5 = [-cross(w5,q5); w5];
xi6 = [-cross(w6,q6); w6];

ex1 = twistexp(xi1,th1);
ex2 = twistexp(xi2,th2);
ex3 = twistexp(xi3,th3);
ex4 = twistexp(xi4,th4);
ex5 = twistexp(xi5,th5);
ex6 = twistexp(xi6,th6);

%% (1) Compute Jacobian 

% different adjoint for each joint 
Ad1 = [ex1(1:3,1:3) skew(ex1(1:3,4))*ex1(1:3,1:3); zeros(3) ex1(1:3,1:3)];
ex = ex1*ex2; % 
Ad2 = [ex(1:3,1:3) skew(ex(1:3,4))*ex(1:3,1:3); zeros(3) ex(1:3,1:3)];
ex = ex*ex3;
Ad3 = [ex(1:3,1:3) skew(ex(1:3,4))*ex(1:3,1:3); zeros(3) ex(1:3,1:3)];
ex = ex*ex4;
Ad4 = [ex(1:3,1:3) skew(ex(1:3,4))*ex(1:3,1:3); zeros(3) ex(1:3,1:3)];
ex = ex*ex5;
Ad5 = [ex(1:3,1:3) skew(ex(1:3,4))*ex(1:3,1:3); zeros(3) ex(1:3,1:3)];

% final Jacobian
J = simplify([xi1 Ad1*xi2 Ad2*xi3 Ad3*xi4 Ad4*xi5 Ad5*xi6])


%% (2) Compute all singular configuration of the robotic system

% determinant is zero in singularity configuration
detJ = simplify(det(J));
singularity = solve(detJ==0,[th1, th2, th3, th4 th5 th6]);

Number_of_singularities = length(singularity.th1)
% check each configuration, which can be reached in allowed boundaries
% --> the 3rd configuration
% --> the 2nd configuration with theta2 = -pi/2

%% (3) Compute the null space and spatial velocity in the possible singular configuration. 

% substitute l0 and l1
Jnum = subs(J, l0, 0.2);
Jnum = simplify(subs(Jnum, l1, 0.5));

% substitute thetas with values for the 3rd singularity configuration 
th1 = singularity.th1(3);
th2 = singularity.th2(3);
th3 = singularity.th3(3);
th4 = singularity.th4(3);
th5 = singularity.th5(3);
th6 = singularity.th6(3);
Joint_parameters = [th1 th2 th3 th4 th5 th6]
JSingular1 = subs(Jnum); % Jacobian of singular configuration

% substitute thetas with values for the 2nd singularity configuration 
th1 = singularity.th1(2);
th2 = -singularity.th2(2);
th3 = singularity.th3(2);
th4 = singularity.th4(2);
th5 = singularity.th5(2);
th6 = singularity.th6(2);
Joint_parameters = [th1 th2 th3 th4 th5 th6]
JSingular2 = subs(Jnum); % Jacobian of singular configuration

%% (3a) Nullspace of singular configuration

nullspace = null(JSingular1)
nullspace = null(JSingular2)
% if entries are non-zero -> corresponding joints are linearly dependent 
% 3rd singularity --> joint 4 and 6 are rotating around z-axis in this configuration
% 2nd singularity --> joint 1 and 6 are rotating around z-axis in this configuration

%% (3b) Spatial Velocity: checking the velocity also gives clues about lost DoFs

syms t1 t2 t3 t4 t5 t6 real % theta dot 
velocity_in_singular_config1 = JSingular1*[t1 t2 t3 t4 t5 t6]' 
% In this configuration, the robot cannot produce any angular velocity
% around the y-axis anymore
velocity_in_singular_config2 = JSingular2*[t1 t2 t3 t4 t5 t6]' 
% In this configuration, the robot can produce all components of the spatial velocity. 
% In this case only the x componenent of the body velocity would be = 0. 


%% (4) Determine the minimum workspace velocity that can be produced by a unit joint velocity vector

% define and substitute all joint parameters 
th1 = 0;
th2 = pi/4;
th3 = 0.05;
th4 = pi/3;
th5 = 0;
th6 = 0;
J_min_WS_velocity =subs(Jnum);

% The minimum singular value of the Jacobian corresponds to the minimum 
% workspace velocity that can be produced by a unit joint velocity vector
% (see page 128 "A mathematical description of robotic manipulaiton" for
% fore details) 
[U,S,V] = svd(double(J_min_WS_velocity));
% diag(S) are singular values
min_velocity = min(diag(S)) % the magnitude of the min WS velocity
direction_minWS_velocity = U(:,end)*min(diag(S)) % twist (direction) of minimum WS velocity



%% Assignement 7 (2)Compute the joint forces in the general configuration 

% Define the force variables
syms Fx Fy Fz tx ty tz real 

% The coint torques can be computed by multipling the transposed Jacobian
% with the end effector force
FEE = [Fx Fy Fz tx ty tz]';

FEE
% tau describes the torques which have to applied in the invidual joints in
% order to sustain the EE force
tau = simplify(J'*FEE);

tau


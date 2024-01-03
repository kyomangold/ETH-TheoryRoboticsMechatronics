%%  TRM Assignment 01

clear;
close all;
clc;

%% 1.g)
S1 = importdata('sensor1.txt', '');
S2 = importdata('sensor2.txt', '');

% commands to run in terminal >> fitdist(S1, 'normal') and fitdist(S2, 'normal')

%% 1.h) 

% i
d_rt = 1;% [m] distance robot – table 
h_table = 1; % [m] height, table
s_table = 1; % [m] side, table
s_cube = 0.2; % [m] side, cube
d_tc = 2; % [m] distance table – camera

%h01 = transl(0, d_rt, h_table)
%h12 = transl(-(s_table+s_cube)/2, (s_table-s_cube)/2, 0)
h23 = transl(s_cube/2, s_cube/2,d_tc) * rotyh(pi) * rotzh(pi/2);

%h02 = h01 * h12; 
%h03 = h02 * h23;



% iv
%r23 = rot(h23);
%t23 = transl(h23);
%t32 = -r23'*t23(1:3);
%h32 = [r23' t32; 0 0 0 1]
% or more computationally intensive: 
h32 = inv(h23)

% extract xc, yc, and zc
xc = t32(1,1); 
yc = t32(2,1);
zc = t32(3,1);

f = 0.1; % [m] focal length 
% from part b):
xs = f * xc/zc;
ys = f * yc/zc;

px_size = 1e-5; %[m] pixel size 
ccd_size = 1024; %[px] ccd size 
% % form part c) 
xs_px = xs/px_size + ccd_size/2
ys_px = ys/px_size + ccd_size/2

% v
h02a = h02*rotzh(25*pi/180)*transl(0.06,0,0)*rotyh(pi/4);
h32a = inv(h03)*h02a;
z2_prime = h32a(1:4,3)



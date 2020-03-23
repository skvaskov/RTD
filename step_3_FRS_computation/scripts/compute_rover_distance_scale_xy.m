%% description
% This script computes the distance scaling needed to compute an FRS for
% the Rover FRS.
%
% Author: Sean Vaskov
% Created: 06 March 2020

clear;close
%% user parameters
% uncomment one of the following lines to load the relevant error function
% data; we'll compute the FRS for that error function

% load timing info
load('rover_timing.mat')

% load error info
load('rover_xy_error_functions_T1.5_v0_1.0_to_2.0_degx3_degy3.mat')

% define semialg set restricting yawrate commands based on heading
% hold for all sets
hZ0{1} = @(z,k) k(1,:)-(-w0_des_min/psi_end_max*k(2,:)+w0_des_min);
hZ0{2} = @(z,k) (w0_des_max/-psi_end_min*k(2,:)+w0_des_max) - k(1,:);

%constraints to limit velocity to 0.5 m/s at 2 m/s
hZ0{3} = @(z,k) -0.5*k(3,:) + 1.5 - k(1,:);
hZ0{4} = @(z,k) -0.5*k(3,:) + 1.5 + k(1,:);

% define range of initial conditions
Z0_range = [[0;0;0],[0;0;0]];

% define range of parameters
K_range = [[w0_des_min;psi_end_min;v_des_min],...
           [w0_des_max;psi_end_max;v_des_max]];


%% automated from here
psi0_min = -psi_end_max;
psi0_max = -psi_end_min;

%trajectory producing dynamics
f_traj = @(t,z,k) rover_trajectory_producing_model(t,z,k,t_f);

% build g matrix
g = [g_x;g_y;0];

g_traj = msspoly_to_fun(g,{t,z,k});


[zscale,zoffset] =get_state_scaling_factors(f_traj,Z0_range,'K_range',K_range,'hZ0',hZ0,'T',T,'n_scale',sqrt(2)/2,'g',g_traj,'plotting',true);

%% save output
scaling_filename = ['rover_FRS_xy_scaling_T',num2str(T,'%0.1f'),...
    '_v0_',num2str(v0_min,'%0.1f'),'_to_',num2str(v0_max,'%0.1f'),'_',date,'.mat'] ;
save(scaling_filename,'zscale','zoffset','T','*_min','*_max') ;

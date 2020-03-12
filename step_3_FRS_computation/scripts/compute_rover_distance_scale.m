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

 filename = 'rover_pos_error_functions_v0_1.0_to_2.0.mat' ;


%% automated from here
% load timing info
load('rover_timing.mat')

% load error info
load(filename)

% create the agent
A = RoverAWD ;

% define range of initial conditions
Z0_range = [[0;0;0],[0;0;0]];
% define range of parameters
K_range = [[w0_des_min;psi_end_min;v_des_min],...
          [w0_des_max;psi_end_max;v_des_max]];

% define semialg set restricting yawrate commands based on heading
hZ0{1} = @(z,k) k(1,:)-(-w0_des_min/psi_end_max*k(2,:)+w0_des_min);
hZ0{2} = @(z,k) (w0_des_max/-psi_end_min*k(2,:)+w0_des_max) - k(1,:);


%trajectory producing dynamics
f_traj = @(t,z,k) rover_trajectory_producing_model(t,z,k,t_f);

% build g matrix
g = [g_v_cos,g_v_sin;-g_vy_sin,g_vy_cos;0,0];

g_traj = msspoly_to_fun(g,{t,z,k});


[zscale,zoffset] =get_state_scaling_factors(f_traj,Z0_range,'K_range',K_range,'hZ0',hZ0,'T',t_f,'n_scale',0.6,'g',g_traj,'plotting',true);

%% save output
filename = ['rover_FRS_scaling_v0_',...
    num2str(v0_min,'%0.1f'),'_to_',...
    num2str(v0_max,'%0.1f'),'.mat'] ;
save(filename,'zscale','zoffset') ;

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

% load error info (we will loop through all of the error files. get scaling
% factors for each and save a mat file with the scaling factors for each)

error_files_directory = '~/MATLAB/IJRR_bridging_the_gap/step_2_error_function/data/rover';

plotting = false;


%% automated from here

folder_info = dir(error_files_directory);

for i = 3:length(folder_info)
    
    compute_scaling_factors([folder_info(i).folder,'/',folder_info(i).name],plotting)
    
end

function compute_scaling_factors(filename,plotting)

load(filename)
%set conditions for initial heading in the global road frame
%(note that this will be used later when we "mirror" the frs)
psi0_min = -pi;
psi0_max = 0;

% define semialg set restricting yawrate commands based on heading
% hold for all sets
hZ0{1} = @(z,k) k(1,:)-(1/0.5*k(2,:)-1);

L = [min(A.footprint_vertices(1,:)) max(A.footprint_vertices(2,:))];
W = [min(A.footprint_vertices(2,:)) max(A.footprint_vertices(2,:))];
% define range of initial conditions
Z0_range = [[L(1);W(1);0],[L(2);W(2);0]];

% define range of parameters
K_range = [[w0_des_min;psi_end_min;v_des_min],...
           [w0_des_max;psi_end_max;v_des_max]];

%trajectory producing dynamics
f_traj = @(t,z,k) rover_trajectory_producing_model(t,z,k,t_f);

% build g matrix
g = [g_x,0;0,g_y;0,0];

g_traj = msspoly_to_fun(g,{t,z,k});


[zscale,zoffset] = get_state_scaling_factors(f_traj,Z0_range,'K_range',K_range,'hZ0',hZ0,'T',T,'n_scale',sqrt(2)/2,'g',g_traj,'plotting',plotting);

%% save output
scaling_filename = ['rover_FRS_xy_scaling_T',num2str(T,'%0.1f'),...
    '_v0_',num2str(v0_min,'%0.1f'),'_to_',num2str(v0_max,'%0.1f'),...
    '_delta0_',num2str(delta0_min,'%0.2f'),'_to_',num2str(delta0_max,'%0.2f'),'.mat'] ;

save(scaling_filename,'zscale','zoffset','T','*_min','*_max') ;

end

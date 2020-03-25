%% description
% This script generates a symbolic trajectory for the segway, and saves a
% function that returns that symbolic trajectory as a 5-by-N array, given
% an intial yaw rate and velocity. The trajectory has an initial state of
% (x,y,h,w,v) = (0,0,0,w_0,v_0).
%
% Author: Shreyas Kousik
% Created: 24 Mar 2020
% Updated: nah
%
%% user parameters
% trajectory timing
dt_traj = 0.01 ;
t_traj = 0.25 ;

% values for validation
w_0_val = 0 ;
v_0_val = 0 ;
w_des_val = -1 ;
v_des_val = 0.25 ;

% save symbolic traj
save_flag = false ;
data_filename = 'segway_symbolic_traj_data.mat' ;
traj_filename = 'segway_symbolic_traj.m' ;

%% automated from here
% create sym vars
syms w_des v_des w_0 v_0 real

% create time vector and initial condition
T = 0:dt_traj:t_traj ;
z_0 = [zeros(3,1) ; w_0 ; v_0] ;

%% create symbolic trajectory
N_T = length(T) ;
Z_sym = [z_0, nan(5,N_T-1)] ;

tic
for idx = 2:N_T
    % euler-integrate to get new state
    z_old = Z_sym(:,idx-1) ;
    z_new = z_old + dt_traj.*segway_symbolic_model(z_old,[w_des;v_des]) ;
    Z_sym(:,idx) = z_new ;
end
toc

% generate matlab function from symbolic dynamics
if save_flag
    Z_fn = matlabFunction(Z_sym,'Vars',{'w_0','v_0','w_des','v_des'},...
        'File',traj_filename) ;
else
    Z_fn = matlabFunction(Z_sym,'Vars',{'w_0','v_0','w_des','v_des'}) ;
end

%% save the timing info
if save_flag
    save(data_filename,'dt_traj','t_traj','traj_filename') ;
end

%% integrate dynamics with ode45 to compare
% ode45
z_0_val = double(subs(z_0,[w_0;v_0],[w_0_val;v_0_val])) ;
u_val = [w_des_val ; v_des_val] ;
[~,Z_ode_val] = ode45(@(t,z) segway_symbolic_model(z,u_val),T,z_0_val) ;
Z_ode_val = Z_ode_val';

% symbolic
Z_sym_val = Z_fn(w_0_val,v_0_val,w_des_val,v_des_val) ;

%% compare timing
disp('ODE45 timing:')
timeit(@() ode45(@(t,z) segway_symbolic_model(z,u_val),T,z_0_val))

disp('Symbolic timing:')
timeit(@() Z_fn(w_0_val,v_0_val,w_des_val,v_des_val))

%% plotting
figure(1) ; clf ; hold on ; axis equal ;

plot_path(Z_ode_val(1:2,:),'b--')
plot_path(Z_sym_val(1:2,:),'r--')
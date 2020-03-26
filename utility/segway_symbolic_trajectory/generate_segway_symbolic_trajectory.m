%% description
% This script generates a symbolic trajectory for the segway, and saves a
% function that returns that symbolic trajectory as a 5-by-N array, given
% an intial yaw rate and velocity. The trajectory has an initial state of
% (x,y,h,w,v) = (0,0,0,w_0,v_0).
%
% This takes about 1 -- 2 s to precompute the symbolic trajectory with
% RK4 integration. Generating a MATLAB function handle from the symbolic
% trajectory takes about 2 s. Saving a MATLAB file of that function takes
% a few minutes.
%
% Author: Shreyas Kousik
% Created: 24 Mar 2020
% Updated: 25 Mar 2020
%
%% user parameters
% trajectory timing
dt_traj = 0.01 ;
t_traj = 0.25 ;

% values for validation
w_0_val = 1 ;
v_0_val = 1 ;
w_des_val = 0 ;
v_des_val = 1.25 ;

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

disp('Computing symbolic trajectory')
tic
for idx = 2:N_T
    % get old state
    z_old = Z_sym(:,idx-1) ;
    
    % RK4 to get new state
    h = dt_traj ;
    k_1 = h*segway_symbolic_model(z_old,[w_des;v_des]) ;
    k_2 = h*segway_symbolic_model(z_old + k_1/2,[w_des;v_des]) ;
    k_3 = h*segway_symbolic_model(z_old + k_2/2,[w_des;v_des]) ;
    k_4 = h*segway_symbolic_model(z_old + k_3,[w_des;v_des]) ;
    
    z_new = z_old + (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4) ;
    
    % save new state
    Z_sym(:,idx) = z_new ;
end
toc

% generate matlab function from symbolic dynamics
if save_flag
    disp('Saving trajectory function as MATLAB file')
    tic
    Z_fn = matlabFunction(Z_sym,'Vars',{'w_0','v_0','w_des','v_des'},...
        'File',traj_filename) ;
    toc
else
    disp('Generating trajectory function')
    tic
    Z_fn = matlabFunction(Z_sym,'Vars',{'w_0','v_0','w_des','v_des'}) ;
    toc
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
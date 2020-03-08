%% description
close all
clear
%
% This script computes the tracking error function "g" for the rover.
% See the paper "Bridging the Gap Between Safety and Real-Time Performance
% in Receding-Horizon Trajectory Design for Mobile Robots" for an
% explanation of the error function in Section 2.2.2. In particular, see
% Assumption 10 that defines the tracking error function.
%
% The paper is available here: https://arxiv.org/abs/1809.06746
%
% Author: Sean Vaskov
% Created: 06 March 2020

degree = 3;

%% user parameters
% initial condition bounds (recall that the state is (x,y,h,v), but the
% robot's dynamics in SE(2) are position/translation invariant)
v0_min = 1.0 ; % m/s
v0_max = 2.0 ; % m/s

% command bounds
w0_min = -1.0 ; % rad/s
w0_max =  1.0 ; % rad/s

v_des_min = 1;
v_des_max = 2;

w0_des_min = -1;
w0_des_max = 1;

psi_end_min = -0.5; %rad
psi_end_max = 0.5; %rad

delta_v = 1.0 ; % m/s

% number of samples in v0, w, and v
N_samples = 4 ;

% timing
t_sample = 0.01 ;


%% automated from here
% create roveragent
A = RoverAWD ;
l = A.wheelbase;
lr = A.rear_axel_to_center_of_mass;

%convert yawrate to wheelangle to save
delta0_min = min( A.yawrate_to_wheelangle([v0_max,v0_max,v0_min,v0_min],[w0_max,w0_min,w0_max,w0_min]));
delta0_max = max( A.yawrate_to_wheelangle([v0_max,v0_max,v0_min,v0_min],[w0_max,w0_min,w0_max,w0_min]));

% create initial condition vector
v0_vec = linspace(v0_min,v0_max,N_samples) ;

% create yaw commands
w0_vec = linspace(w0_min,w0_max,N_samples) ;

% create psi0 commands
psiend_vec = linspace(psi_end_min,psi_end_max,N_samples);


% load timing
try
    disp('Loading r RTD planner timing info.')
    timing_info = load('rover_timing.mat') ;
    t_plan = timing_info.t_plan ;
    t_stop = timing_info.t_stop ;
    t_f = timing_info.t_f ;
catch
    disp('Could not find timing MAT file. Setting defaults!')
    t_plan = 0.5 ;
    t_stop = 2 ;
    t_f = 2 ;
end

% initialize time vectors for saving tracking error; we use two to separate
% out the braking portion of the trajectory
T_data = unique([0:t_sample:t_f,t_f]) ;

% initialize arrays for saving tracking error; note there will be one row
% for every (v0,w_des,v_des) combination, so there are N_samples^3 rows
N_total = N_samples^5;
v_data = nan(N_total,length(T_data)) ;
w_data = nan(N_total,length(T_data)) ;
vy_data = nan(N_total,length(T_data)) ;
%% tracking error computation loop
err_idx = 1 ;

tic
% for each initial condition...
for v0 = v0_vec
    for w0 = w0_vec
        
        % get approximate initial wheel angle from yawrate
        delta0 = A.yawrate_to_wheelangle(v0,w0);

        % create the feasible speed commands from the initial condition
        v_vec = linspace(max(v0 - delta_v,v_des_min), min(v0 + delta_v,v_des_max), N_samples) ;
        
        % for each yaw and speed command...
        for v_des = v_vec
            for psi_end = psiend_vec
                
                % create the initial condition
                z0 = [0;0;0;v0;delta0] ; % (x,y,h,v,delta)
                
                %create feasible initial yawrate commands from initial
                %condition
                w0_des_min_temp = max(w0_des_min, -w0_des_min/psi_end_max*psi_end+w0_des_min);
                w0_des_max_temp = min(w0_des_max, w0_des_max/-psi_end_min*psi_end+w0_des_max);
                
                w0_des_vec = linspace(w0_des_min_temp,w0_des_max_temp, N_samples);
                
                for w0_des = w0_des_vec
                    
                    % create the desired trajectory
                    [T_go,U_go,Z_go] = make_rover_desired_trajectory(t_f,w0_des,psi_end,v_des) ;
                    
                    
                    % reset the robot
                    A.reset(z0)
                    
                    % track the desired trajectory
                    A.move(t_f,T_go,U_go,Z_go) ;
                    
                    % get the executed position trajectory
                    T = A.time ;

                    % compute the error before t_plan
                    z_1 = match_trajectories(T_go,T,A.state) ;
                    
                    % save data
                    ve_1 = z_1(4,:)-v_des;
                    
                    %estimate yawrate from velocity
                    w_go = v_des*tan(U_go(2,:))/l;
                    vy_go = lr*w_go;
                    
                    w_1 = A.wheelangle_to_yawrate(z_1(4,:),z_1(5,:)); 
                    vy_1 = A.wheelangle_to_lateral_veloctity(z_1(4,:),z_1(5,:));
                    
                    
                    w_data(err_idx,:) = abs(w_1-w_go);
                    v_data(err_idx,:) = abs(ve_1);
                    vy_data(err_idx,:) = abs(vy_1-vy_go);
                    % increment counter
                    err_idx = err_idx + 1 ;
                    
%                     cla 
%                     plot(z_1(1,:),z_1(2,:))
%                     hold on
%                     plot(Z_go(1,:),Z_go(2,:))
%                     pause
%                     
                    if mod(err_idx,10) == 0
                        disp(['Iteration ',num2str(err_idx),' out of ',num2str(N_samples^5)])
                    end
                end
            end
            
        end
    end
end
toc



%% fit g with a polynomial
% get the max of the error
v_err = max(v_data,[],1) ;
w_err = max(w_data,[],1) ;
vy_err = max(vy_data,[],1);

% fit polynomials to the max data
g_v_coeffs = polyfit(T_data,v_err,degree) ;
g_vy_coeffs = polyfit(T_data,vy_err,degree) ;
g_w_coeffs = polyfit(T_data,w_err,degree) ;


%% evaluate polynomial for plotting
g_v_plot = polyval(g_v_coeffs,T_data ) ;
g_vy_plot = polyval(g_vy_coeffs,T_data);
g_w_plot = polyval(g_w_coeffs,T_data ) ;

%% reconfigure polynomials to be greater than all data points
g_v_coeffs(end) = g_v_coeffs(end) + max(g_v_plot-v_err);
g_v_plot = polyval(g_v_coeffs,T_data ) ;

g_vy_coeffs(end) = g_vy_coeffs(end) + max(g_vy_plot-vy_err);
g_vy_plot = polyval(g_vy_coeffs,T_data ) ;

g_w_coeffs(end) = g_w_coeffs(end) + max(g_w_plot-w_err);
g_w_plot = polyval(g_w_coeffs,T_data ) ;


%% save data
filename = ['rover_error_functions_v0_',...
            num2str(v0_min,'%0.1f'),'_to_',...
            num2str(v0_max,'%0.1f'),'.mat'] ;
save(filename,'g_v_coeffs','g_vy_coeffs','g_w_coeffs','delta0_min','delta0_max','psi_end_min','psi_end_max',...
     'delta_v','v0_min','v0_max','w0_des_min','w0_des_max','v_des_min','v_des_max') ;

%% plotting
figure(1) ;

% plot v error
subplot(3,1,1) ; hold on ;
plot(T_data,v_data','b:')
g_v_handle = plot(T_data,g_v_plot,'r-','LineWidth',1.5) ;
title('error in velocity')
xlabel('time [s]')
ylabel('velocity error [m]')
legend(g_v_handle,' g_v(t) dt','Location','NorthWest')
set(gca,'FontSize',15)

% plot vy error
subplot(3,1,2) ; hold on ;
plot(T_data,vy_data','b:')
g_vy_handle = plot(T_data,g_vy_plot,'r-','LineWidth',1.5) ;
title('error in lateral velocity')
xlabel('time [s]')
ylabel('velocity error [m]')
legend(g_vy_handle,' g_vy(t) dt','Location','NorthWest')
set(gca,'FontSize',15)

% plot w error
subplot(3,1,3) ; hold on ;
plot(T_data,w_data','b:')
g_w_handle = plot(T_data,g_w_plot,'r-','LineWidth',1.5) ;
title('error in yawrate')
xlabel('time [s]')
ylabel('yawrate error [m]')
legend(g_w_handle,'g_w(t) dt','Location','NorthWest')
set(gca,'FontSize',15)
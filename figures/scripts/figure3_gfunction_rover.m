%% description
close all
clear
%
% This script loads a file with tracking error functions for the rover. The
% user then selects a trajectory parameter k. The script then samples the
% high fidelity model from the range of initial conditions. Then plots the
% trajectory data and tracking error function evaluated at k
%

% Author: Sean Vaskov
% Created: 16 April 2020


% number of samples in v0, w, and v
N_samples = 10;


%tracking error file
load('rover_xy_error_functions_T1.5_v0_1.5_to_2.2_delta0_-0.05_to_0.05_degx3_degy3.mat');

% v_des = randRange(v_des_min,v_des_max);
v_des =  1.9368;
% psi_end = randRange(psi_end_min,psi_end_max);
psi_end = 0.3961;
% w0_des  = randRange(max(-1,-1+2*psi_end),w0_des_max);
w0_des = 0.9511;

save_pdf_flag = true;

t_sample = 0.01;

% line color
g_color = [0,0.75,0.25];

%% automated from here
% create roveragent
RoverLLC = rover_PD_LLC('yaw_gain',5,'yaw_rate_gain',0.5);
A = RoverAWD('LLC',RoverLLC,'max_speed',3) ;
l = A.wheelbase;
lr = A.rear_axel_to_center_of_mass;

% create initial condition vectors
delta0_vec = linspace(delta0_min,delta0_max,N_samples) ;
v0_vec = linspace(v0_min,v0_max,N_samples) ;


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
T_data = unique([0:t_sample:T,T]) ;

% initialize arrays for saving tracking error; note there will be one row
% for every (v0,w_des,v_des) combination, so there are N_samples^3 rows
N_total = N_samples^2;

e_x_max_data = nan(N_total,length(T_data)) ;
e_x_min_data = nan(N_total,length(T_data)) ;
e_y_max_data = nan(N_total,length(T_data)) ;
e_y_min_data = nan(N_total,length(T_data)) ;



%% tracking error computation loop
err_idx = 1 ;

tic
% for each initial condition...
for v0 = v0_vec
    for delta0 = delta0_vec
        
        
        
        % create the initial condition
        z0 = [0;0;0;v0;delta0] ; % (x,y,h,v,delta)
        
        
        % create the desired trajectory
        [T_ref,U_ref,Z_ref] = make_rover_desired_trajectory(t_f,w0_des,psi_end,v_des) ;
        
        % reset the robot
        A.reset(z0)
        
        % track the desired trajectory
        A.move(t_f,T_ref,U_ref,Z_ref) ;
        
        % compute the error before t_plan
        z_act = match_trajectories(T_data,A.time,A.state) ;
        U_ref = match_trajectories(T_data,T_ref,U_ref);
        Z_ref = match_trajectories(T_data,T_ref,Z_ref);
        
        %get corners of footprint
        R_1 = rotation_matrix_2D(z_act(3,:));
        R_ref = rotation_matrix_2D(Z_ref(3,:));
        
        V1 = repmat(A.footprint_vertices(:,1),[1 length(T_data)]);
        V2 = repmat(A.footprint_vertices(:,2),[1 length(T_data)]);
        V3 = repmat(A.footprint_vertices(:,3),[1 length(T_data)]);
        V4 = repmat(A.footprint_vertices(:,4),[1 length(T_data)]);
        
        V1_act = reshape(R_1*(V1(:)),[2 length(T_data)])+z_act(A.position_indices,:);
        V2_act = reshape(R_1*(V2(:)),[2 length(T_data)])+z_act(A.position_indices,:);
        V3_act = reshape(R_1*(V3(:)),[2 length(T_data)])+z_act(A.position_indices,:);
        V4_act = reshape(R_1*(V4(:)),[2 length(T_data)])+z_act(A.position_indices,:);
        
        V1_ref = reshape(R_ref*(V1(:)),[2 length(T_data)])+Z_ref(A.position_indices,:);
        V2_ref = reshape(R_ref*(V2(:)),[2 length(T_data)])+Z_ref(A.position_indices,:);
        V3_ref = reshape(R_ref*(V3(:)),[2 length(T_data)])+Z_ref(A.position_indices,:);
        V4_ref = reshape(R_ref*(V4(:)),[2 length(T_data)])+Z_ref(A.position_indices,:);
        
        
        %store position error data
        e_x_max_data(err_idx,:) = max([z_act(A.position_indices(1),:)-Z_ref(A.position_indices(1),:);...
            V1_act(1,:)-V1_ref(1,:);V2_act(1,:)-V2_ref(1,:);V3_act(1,:)-V3_ref(1,:);V4_act(1,:)-V4_ref(1,:)]);
        
        e_x_min_data(err_idx,:) = min([z_act(A.position_indices(1),:)-Z_ref(A.position_indices(1),:);...
            V1_act(1,:)-V1_ref(1,:);V2_act(1,:)-V2_ref(1,:);V3_act(1,:)-V3_ref(1,:);V4_act(1,:)-V4_ref(1,:)]);
        
        e_y_max_data(err_idx,:) = max([z_act(A.position_indices(2),:)-Z_ref(A.position_indices(2),:);...
            V1_act(2,:)-V1_ref(2,:);V2_act(2,:)-V2_ref(2,:);V3_act(2,:)-V3_ref(2,:);V4_act(2,:)-V4_ref(2,:)]);
        
        e_y_min_data(err_idx,:) = min([z_act(A.position_indices(2),:)-Z_ref(A.position_indices(2),:);...
            V1_act(2,:)-V1_ref(2,:);V2_act(2,:)-V2_ref(2,:);V3_act(2,:)-V3_ref(2,:);V4_act(2,:)-V4_ref(2,:)]);
    
        
        % increment counter
        err_idx = err_idx + 1 ;
        
        
        if mod(err_idx,10) == 0
            disp(['Iteration ',num2str(err_idx),' out of ',num2str(N_samples^2)])
        end
    end
end
            

toc

x_err = max(abs(e_x_max_data),abs(e_x_min_data));
y_err = max(abs(e_y_max_data),abs(e_y_min_data));
 

%% plotting
g_xt = subs(g_x,k,[w0_des;psi_end;v_des]);
g_yt = subs(g_y,k,[w0_des;psi_end;v_des]);


% evaluate polynomial for plotting at random parameter
int_g_x_vals = msubs(integral(g_xt,t),t,T_data) ;
int_g_y_vals = msubs(integral(g_yt,t),t,T_data) ;

fha = figure(1) ; clf ;

% plot x error
hold on ;
plot(T_data,x_err','b.')
g_x_handle =  plot(T_data,int_g_x_vals,'-','Color',g_color,'LineWidth',2.0) ;
xlabel('time [s]')
ylabel('x error [m]')
legend(g_x_handle,'\int g_x(t,k) dt','Location','NorthWest')
set(gca,'FontSize',15)
box on
pbaspect([2 1 1])

fhb = figure(2) ; clf ;
% plot y error
hold on ;
plot(T_data,y_err','b.')
g_y_handle = plot(T_data,int_g_y_vals,'-','Color',g_color,'LineWidth',2.0) ;
xlabel('time [s]')
ylabel('y error [m]')
legend(g_y_handle,'\int g_y(t,k) dt','Location','NorthWest')
box on
set(gca,'FontSize',15)
pbaspect([2 1 1])
%% save_figures
if save_pdf_flag
    save_figure_to_pdf(fha,'rover_gx.pdf')
    save_figure_to_pdf(fhb,'rover_gy.pdf')
end

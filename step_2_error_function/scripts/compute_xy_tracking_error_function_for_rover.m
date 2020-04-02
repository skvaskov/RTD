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
% Created: 24 March 2020
g_degree_x = 3;
g_degree_y = 3;

% number of samples in v0, w, and v
N_samples = 5;

% timing
t_sample = 0.01 ;

plotting = false;
%% user parameters
% initial condition bounds (recall that the state is (x,y,h,v), but the
% robot's dynamics in SE(2) are position/translation invariant)
%time horizon
T = 1.5;

%velocity initial and desired bounds
v0_min = 1.0 ; % m/s
v0_max = 2.0 ; % m/s

v_des_min = 1.0;
v_des_max = 2.0;

%script will loopthrough all of these combos, fit error functions for each
%then save as separate files

delta0_combos = [-0.5, -0.3,  -0.15, -0.05, 0.05, 0.15, 0.3;...
                 -0.3, -0.15, -0.05,  0.05, 0.15, 0.3,  0.5];


% command bounds
w0_des_min = -1.0;
w0_des_max =  1.0;

psi_end_min = 0; %rad
psi_end_max = 0.5; %rad





%% automated from here

for dcom = delta0_combos
    
delta0_min = dcom(1);
delta0_max = dcom(2);
    
% create roveragent
RoverLLC = rover_PD_LLC('yaw_gain',5,'yaw_rate_gain',0.5);
A = RoverAWD('LLC',RoverLLC,'max_speed',3) ;
l = A.wheelbase;
lr = A.rear_axel_to_center_of_mass;

% create initial condition vector for velocity
v0_vec = linspace(v0_min,v0_max,N_samples) ;

% create initial condition vector for wheelangle
delta0_vec = linspace(delta0_min,delta0_max,N_samples) ;

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
T_data = unique([0:t_sample:T,T]) ;

% initialize arrays for saving tracking error; note there will be one row
% for every (v0,w_des,v_des) combination, so there are N_samples^3 rows
N_total = N_samples^5;

e_x_max_data = nan(N_total,length(T_data)) ;
e_x_min_data = nan(N_total,length(T_data)) ;
e_y_max_data = nan(N_total,length(T_data)) ;
e_y_min_data = nan(N_total,length(T_data)) ;

e_dxdt_max_data = nan(N_total,length(T_data)) ;
e_dxdt_min_data = nan(N_total,length(T_data)) ;
e_dydt_max_data = nan(N_total,length(T_data)) ;
e_dydt_min_data = nan(N_total,length(T_data)) ;

%parameters we want g to be a function of
psi_ref_data = nan(N_total,length(T_data));
v_des_data = nan(N_total,length(T_data));
w0_des_data = nan(N_total,length(T_data));
psi_end_data = nan(N_total,length(T_data));


%% tracking error computation loop
err_idx = 1 ;

tic
% for each initial condition...
for v0 = v0_vec
    for delta0 = delta0_vec
        
        % create the feasible speed commands from the initial condition
        v_vec = linspace(v_des_min,v_des_max, N_samples) ;
        
        % for each yaw and speed command...
        for v_des = v_vec
            for psi_end = psiend_vec
                
                % create the initial condition
                z0 = [0;0;0;v0;delta0] ; % (x,y,h,v,delta)
                
                %create feasible initial yawrate commands from initial heading
                w0_des_min_temp = max(w0_des_min, 1/0.5*psi_end-1);
                w0_des_max_temp = min(w0_des_max, 1/0.5*psi_end+1);
                
                w0_des_vec = linspace(w0_des_min_temp,w0_des_max_temp, N_samples);
                
                for w0_des = w0_des_vec
                    
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
                                               
                    %store state error data
                    psi_ref_data(err_idx,:) = Z_ref(3,:);
                    
                    %store state and parameter data
                    v_des_data(err_idx,:) = v_des*ones(length(T_data),1);
                    w0_des_data(err_idx,:) =  w0_des*ones(length(T_data),1);
                    psi_end_data(err_idx,:) = psi_end*ones(length(T_data),1);
                    
                    % increment counter
                    err_idx = err_idx + 1 ;
                    

                    if mod(err_idx,10) == 0
                        disp(['Iteration ',num2str(err_idx),' out of ',num2str(N_samples^5)])
                    end
                end
            end
            
        end
    end
end
toc

e_x_data = max(abs(e_x_max_data),abs(e_x_min_data));
e_y_data = max(abs(e_y_max_data),abs(e_y_min_data));
%take derivatives of error data

e_dxdt_data = get_5point_derivative(e_x_data,T_data);

e_dydt_data = get_5point_derivative(e_y_data,T_data);




 % reset the robot
 A.reset(z0)
%% fit g with a polynomial
% get the max of the error

t = msspoly('t',1);
z = msspoly('z',3);
k = msspoly('k',3);

K_data_col = [w0_des_data(:)';psi_end_data(:)';v_des_data(:)'];
T_data_col = repmat(T_data,[N_total,1]);
T_data_col = T_data_col(:)';

% fit polynomials to the max data
g_x =  fit_bounding_polynomial_from_samples(e_dxdt_data(:)', [T_data_col;K_data_col] ,[t;k],g_degree_x);

g_y =  fit_bounding_polynomial_from_samples(e_dydt_data(:)', [T_data_col;K_data_col] ,[t;k],g_degree_y);


%% save data

filename = ['rover_xy_error_functions_T',num2str(T,'%0.1f'),...
             '_v0_',num2str(v0_min,'%0.1f'),'_to_',num2str(v0_max,'%0.1f'),...
             '_delta0_', num2str(delta0_min,'%0.2f'),'_to_',num2str(delta0_max,'%0.2f'),...
            '_degx',num2str(g_degree_x),'_degy',num2str(g_degree_y),'.mat'] ;
        
save(filename,'A','T','t_f','t','z','k','g_*','*_min','*_max') ;


 
 
 %% plotting
 if plotting
 N_plot = 5;
 
 plot_idxs = randi(N_total,[N_plot,1]);
 plot_colors = rand([N_plot,3]);
 figure 
 for i = 1:N_plot

hold on

K_data_plot = [w0_des_data(plot_idxs(i),:);psi_end_data(plot_idxs(i),:);v_des_data(plot_idxs(i),:)];

% evaluate polynomial for plotting at random parameter
int_g_x_plot = msubs(integral(g_x,t),[t;z(3);k],[T_data;psi_ref_data(plot_idxs(i),:);K_data_plot]) ;
int_g_y_plot = msubs(integral(g_y,t),[t;z(3);k],[T_data;psi_ref_data(plot_idxs(i),:);K_data_plot]) ;



% plot v error
subplot(2,1,1) ; hold on ;
plot(T_data,e_x_data(plot_idxs(i),:),'*','Color',plot_colors(i,:))
plot(T_data,int_g_x_plot,'Color',plot_colors(i,:),'LineWidth',1.0) ;


subplot(2,1,2) ; hold on ;
plot(T_data,e_y_data(plot_idxs(i),:),'*','Color',plot_colors(i,:))
plot(T_data,int_g_y_plot,'Color',plot_colors(i,:),'LineWidth',1.0) ;



 end
 
 % plot v error
subplot(2,1,1) ; hold on ;
xlabel('time [s]')
ylabel('x error [m/s]')
set(gca,'FontSize',12)

subplot(2,1,2) ; hold on ;
xlabel('time [s]')
ylabel('y error [m/s]')
set(gca,'FontSize',12)
 end
 
end

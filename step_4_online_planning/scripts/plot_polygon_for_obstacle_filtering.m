%% description
close all
clear
%
% This script generates samples of the robots motion then plots a polygon
% specified by reachable_set_poly. This polygon should contain the entire
% reachable set of the robot for all trajectory parameters, and thus can be
% used to filter obstacles
%
% Author: Sean Vaskov
% Created: 22 March 2020

%  polygon to bound rechable set
reachable_set_poly = [-0.3, 0.2,   1.75,  3.5,  3.5 ,  1.75, 0.2,-0.3,-0.3;...
                      -0.2,-0.3,-2,-2,2 , 2,0.3,0.2,-0.2];

%% user parameters
% initial condition bounds (recall that the state is (x,y,h,v), but the
% robot's dynamics in SE(2) are position/translation invariant)
v0_min = 0.0 ; % m/s
v0_max = 2.25 ; % m/s

% command bounds
delta0_min = -0.5 ; % rad/s
delta0_max =  0.5; % rad/s

v_des_min = 0.5;
v_des_max = 2.0;

w0_des_min = -1;
w0_des_max = 1;

restrict_yaw_rate_based_on_velocity = false;

psi_end_min = -0.5; %rad
psi_end_max = 0.5; %rad

diff_v = 1.0 ; % m/s

% number of samples in v0, w, and v
N_samples = 3 ;

% timing
t_sample = 0.01 ;

%time horizon
T = 1.5;



%% automated from here
% create roveragent
RoverLLC = rover_PD_LLC('yaw_gain',5,'yaw_rate_gain',0.5);
A = RoverAWD('LLC',RoverLLC,'max_speed',3) ;
l = A.wheelbase;
lr = A.rear_axel_to_center_of_mass;

% create initial condition vector
v0_vec = linspace(v0_min,v0_max,N_samples) ;

% create yaw commands
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

x_data = nan(N_total,length(T_data)) ;
y_data = nan(N_total,length(T_data)) ;
psi_data = nan(N_total,length(T_data)) ;


%% tracking error computation loop
err_idx = 1 ;

tic
% for each initial condition...
for v0 = v0_vec
    for delta0 = delta0_vec
        

        % create the feasible speed commands from the initial condition
        v_vec = linspace(max(v0 - diff_v,v_des_min), min(v0 + diff_v,v_des_max), N_samples) ;
        
        % for each yaw and speed command...
        for v_des = v_vec
            for psi_end = psiend_vec
                
                % create the initial condition
                z0 = [0;0;0;v0;delta0] ; % (x,y,h,v,delta)
                
                %create feasible initial yawrate commands from initial
                %condition
                w0_des_min_temp = max(w0_des_min, 0.5*psi_end-1);
                w0_des_max_temp = min(w0_des_max, 0.5*psi_end+1);
                
                w0_des_vec = linspace(w0_des_min_temp,w0_des_max_temp, N_samples);
                
                for w0_des = w0_des_vec
                    
                    % create the desired trajectory
                    [T_go,U_go,Z_go] = make_rover_desired_trajectory(t_f,w0_des,psi_end,v_des) ;
                    
                    % reset the robot
                    A.reset(z0)
                    
                    % track the desired trajectory
                    A.move(t_f,T_go,U_go,Z_go) ;
                    

                    % compute the error before t_plan
                    z_1 = match_trajectories(T_data,A.time,A.state) ;
                    
                    x_data(err_idx,:)   = z_1(A.position_indices(1),:);
                    y_data(err_idx,:)   = z_1(A.position_indices(2),:);
                    psi_data(err_idx,:) = z_1(A.heading_index,:);
                    
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


%% plot
x_data = x_data(:)';
y_data = y_data(:)';
psi_data = psi_data(:)';

plot(reachable_set_poly(1,:),reachable_set_poly(2,:), 'LineWidth',1.0,'Color',[0 0.8, 0.2])
hold on

plot(x_data,y_data,'b.');

R = rotation_matrix_2D(psi_data);

V1 = repmat(A.footprint_vertices(:,1),[1 length(x_data)]);
V2 = repmat(A.footprint_vertices(:,2),[1 length(x_data)]);
V3 = repmat(A.footprint_vertices(:,3),[1 length(x_data)]);
V4 = repmat(A.footprint_vertices(:,4),[1 length(x_data)]);

V1 = reshape(R*(V1(:)),[2 length(x_data)]);
V2 = reshape(R*(V2(:)),[2 length(x_data)]);
V3 = reshape(R*(V3(:)),[2 length(x_data)]);
V4 = reshape(R*(V4(:)),[2 length(x_data)]);


plot(x_data+V1(1,:),y_data+V1(2,:),'b.')
plot(x_data+V2(1,:),y_data+V2(2,:),'b.')
plot(x_data+V3(1,:),y_data+V3(2,:),'b.')
plot(x_data+V4(1,:),y_data+V4(2,:),'b.')


legend({'polygon','data'})

xlabel('x (m)')
ylabel('y (m)')








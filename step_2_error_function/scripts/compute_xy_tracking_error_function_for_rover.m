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
g_degree_x = 3;
g_degree_y = 3;

%% user parameters
% initial condition bounds (recall that the state is (x,y,h,v), but the
% robot's dynamics in SE(2) are position/translation invariant)
v0_min = 0.0 ; % m/s
v0_max = 1.0 ; % m/s

% command bounds
w0_min = -1.0 ; % rad/s
w0_max =  1.0 ; % rad/s

v_des_min = 0.5;
v_des_max = 1.25;

%set w0_des_min to 0 since we will be mirroring
w0_des_min = 0;
w0_des_max = 1;

restrict_yaw_rate_based_on_velocity = false;

psi_end_min = -0.5; %rad
psi_end_max = 0.5; %rad

diff_v = 1.0 ; % m/s

% number of samples in v0, w, and v
N_samples = 5 ;

% timing
t_sample = 0.01 ;

%time horizon
T = 1.5;


%% automated from here
% create roveragent
RoverLLC = rover_PD_LLC('yaw_gain',4,'yaw_rate_gain',1);
A = RoverAWD('LLC',RoverLLC,'max_speed',3) ;
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
T_data = unique([0:t_sample:T,T]) ;

% initialize arrays for saving tracking error; note there will be one row
% for every (v0,w_des,v_des) combination, so there are N_samples^3 rows
N_total = N_samples^5;

e_x_data = nan(N_total,length(T_data)) ;
e_y_data = nan(N_total,length(T_data)) ;

psi_ref_data = nan(N_total,length(T_data));

v_des_data = nan(N_total,length(T_data));
w0_des_data = nan(N_total,length(T_data));
psi_end_data = nan(N_total,length(T_data));


%% tracking error computation loop
err_idx = 1 ;

tic
% for each initial condition...
for v0 = v0_vec
    for w0 = w0_vec
        
        % get approximate initial wheel angle from yawrate
        delta0 = A.yawrate_to_wheelangle(v0,w0);

        % create the feasible speed commands from the initial condition
        v_vec = linspace(max(v0 - diff_v,v_des_min), min(v0 + diff_v,v_des_max), N_samples) ;
        
        % for each yaw and speed command...
        for v_des = v_vec
            for psi_end = psiend_vec
                
                % create the initial condition
                z0 = [0;0;0;v0;delta0] ; % (x,y,h,v,delta)
                
                %create feasible initial yawrate commands from initial heading
                w0_des_min_temp = max(w0_des_min, -w0_des_min/psi_end_max*psi_end+w0_des_min);
                w0_des_max_temp = min(w0_des_max, w0_des_max/-psi_end_min*psi_end+w0_des_max);
                
                if restrict_yaw_rate_based_on_velocity
                    %create feasible initial yawarate commands from velocity
                    %command
                    w0_des_min_temp = max(w0_des_min_temp,  0.5 * v_des - 1.5);
                    w0_des_max_temp = min(w0_des_max_temp, -0.5 * v_des + 1.5);
                end
                
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
                    U_go = match_trajectories(T_data,T_go,U_go);
                    Z_go = match_trajectories(T_data,T_go,Z_go);
                    
                    %estimate yawrate from velocity
                    w_go = v_des*tan(U_go(2,:))/l;
                    
                    vy_cos_go = lr*w_go.*(1-Z_go(3,:).^2/2);
                    vy_sin_go = lr*w_go.*(Z_go(3,:)-Z_go(3,:).^3/6);
                    
                    v_cos_go = v_des*(1-Z_go(3,:).^2/2);
                    v_sin_go = v_des*(Z_go(3,:)-Z_go(3,:).^3/6);
                    
                    vy_1 = A.wheelangle_to_lateral_veloctity(z_1(4,:),z_1(5,:));
                    
                    %store velocity error data
                    e_x_data(err_idx,:) = (z_1(4,:).*cos(z_1(3,:))-vy_1.*sin(z_1(3,:)))- (v_cos_go-vy_sin_go);
                    e_y_data(err_idx,:) = (z_1(4,:).*sin(z_1(3,:))+vy_1.*cos(z_1(3,:)))- (v_sin_go+vy_cos_go);
                    
                    %store state error data
                    psi_ref_data(err_idx,:) = Z_go(3,:);
                    
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


 % reset the robot
 A.reset(z0)
%% fit g with a polynomial
% get the max of the error
x_err_col = abs(e_x_data(:)');
y_err_col = abs(e_y_data(:)');


t = msspoly('t',1);
z = msspoly('z',3);
k = msspoly('k',3);

K_data_col = [w0_des_data(:)';psi_end_data(:)';v_des_data(:)'];
T_data_col = repmat(T_data,[N_total,1]);
T_data_col = T_data_col(:)';

% fit polynomials to the max data
g_x =  fit_bounding_polynomial_from_samples(x_err_col, [T_data_col;psi_ref_data(:)';K_data_col] ,[t;z(3);k],g_degree_x);
g_y =  fit_bounding_polynomial_from_samples(y_err_col, [T_data_col;psi_ref_data(:)';K_data_col] ,[t;z(3);k],g_degree_y);


%% save data

filename = ['rover_xy_error_functions_T',num2str(T,'%0.1f'),'_v0_',...
            num2str(v0_min,'%0.1f'),'_to_',num2str(v0_max,'%0.1f')...
            '_degx',num2str(g_degree_x),'_degy',num2str(g_degree_y),'.mat'] ;
save(filename,'A','T','t_f','t','z','k','g_x','g_y','delta0_min','delta0_max','diff_v','*_min','*_max') ;


 
 
 %% plotting
 N_plot = 10;
 
 plot_idxs = randi(N_total,[N_plot,1]);
 plot_colors = rand([N_plot,3]);
 
 for i = 1:N_plot
figure(1) ;
hold on

K_data_plot = [w0_des_data(plot_idxs(i),:);psi_end_data(plot_idxs(i),:);v_des_data(plot_idxs(i),:)];

% evaluate polynomial for plotting at random parameter
g_x_plot = msubs(g_x,[t;z(3);k],[T_data;psi_ref_data(plot_idxs(i),:);K_data_plot]) ;
g_y_plot = msubs(g_y,[t;z(3);k],[T_data;psi_ref_data(plot_idxs(i),:);K_data_plot]) ;


% plot v error
subplot(2,1,1) ; hold on ;
plot(T_data,abs(e_x_data(plot_idxs(i),:)),'*','Color',plot_colors(i,:))
plot(T_data,g_x_plot,'Color',plot_colors(i,:),'LineWidth',1.0) ;

subplot(2,1,2) ; hold on ;
plot(T_data,abs(e_y_data(plot_idxs(i),:)),'*','Color',plot_colors(i,:))
plot(T_data,g_y_plot,'Color',plot_colors(i,:),'LineWidth',1.0) ;


 end
 
 % plot v error
subplot(2,1,1) ; hold on ;
xlabel('time [s]')
ylabel('dx/dt error [m/s]')
set(gca,'FontSize',12)

subplot(2,1,2) ; hold on ;
xlabel('time [s]')
ylabel('dy/dy error [m/s]')
set(gca,'FontSize',12)


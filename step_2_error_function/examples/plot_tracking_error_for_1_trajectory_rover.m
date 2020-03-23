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
% Created: 13 March 2020

%% user parameters
% initial condition bounds (recall that the state is (x,y,h,v), but the
% robot's dynamics in SE(2) are position/translation invariant)
v0_min = 1.0 ; % m/s
v0_max = 2.0 ; % m/s

% command bounds
w0_min = -1.0 ; % rad/s
w0_max =  1.0 ; % rad/s

v_des = 2;

w0_des = 1;

psi_end = 0.0; %rad

delta_v = 1.0 ; % m/s

% number of samples in v0, w, and v
N_samples = 10 ;

% timing
t_sample = 0.01 ;


%% agent
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

w0_vec = linspace(w0_min,w0_max,N_samples) ;



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
Z_data = cell(N_samples^2,1);

%% tracking error computation loop
% create the desired trajectory
[T_go,U_go,Z_go] = make_rover_desired_trajectory(t_f,w0_des,psi_end,v_des) ;

tic
err_idx = 1;
% for each initial condition...
for v0 = v0_vec
    for w0 = w0_vec
        
        % get approximate initial wheel angle from yawrate
        delta0 = A.yawrate_to_wheelangle(v0,w0);

     

 
                
                % create the initial condition
                z0 = [0;0;0;v0;delta0] ; % (x,y,h,v,delta)
              
              
                    
                    
                    % reset the robot
                    A.reset(z0)
                    
                    % track the desired trajectory
                    A.move(t_f,T_go,U_go,Z_go) ;
                    
                    % get the executed position trajectory
                    T = A.time ;

                    % compute the error before t_plan
                    Z_data{err_idx} = match_trajectories(T_go,T,A.state) ;
          
                    if mod(err_idx,10) == 0
                        disp(['Iteration ',num2str(err_idx),' out of ',num2str(N_samples^2)])
                    end
                    err_idx = err_idx + 1;
                    
    end
end


toc

T_data = repmat(T_go,[1 N_samples^2]);
Z_data = cat(2,Z_data{:});



 
 %% plotting

subplot(2,2,1)
plot(Z_data(1,:),Z_data(2,:),'b.')
hold on
plot(Z_go(1,:),Z_go(2,:),'r','LineWidth',1.0)

subplot(2,2,2)
plot(T_data,Z_data(3,:),'b.')
hold on
plot(T_go,Z_go(3,:),'r','LineWidth',1.0)


subplot(2,2,3)
plot(T_data,Z_data(4,:),'b.')
hold on
plot(T_go,Z_go(4,:),'r','LineWidth',1.0)


subplot(2,2,4)
plot(T_data,Z_data(5,:),'b.')
hold on
plot(T_go,Z_go(5,:),'r','LineWidth',1.0)



subplot(2,2,1) ; hold on ;
xlabel('x [m]')
ylabel('y [m]')
set(gca,'FontSize',12)

subplot(2,2,2) ; hold on ;
xlabel('time [s]')
ylabel('\psi [rad]')
set(gca,'FontSize',12)

% plot vy error
subplot(2,2,3) ; hold on ;
xlabel('time [s]')
ylabel('v [m/s]')
set(gca,'FontSize',12)

subplot(2,2,4) ; hold on ;
xlabel('time [s]')
ylabel('\delta [rad]')
set(gca,'FontSize',12)


%% description
% This script computes the distance scaling needed to compute an FRS for
% the segway.
%
% Author: Shreyas Kousik
% Created: 10 Mar 2020
% Updated: 12 Mar 2020
%
%% user parameters
% uncomment one of the following lines to load the relevant error function
% data; we'll compute the FRS for that error function

% filename = 'segway_error_functions_v_0_0.0_to_0.5.mat' ;
% filename = 'segway_error_functions_v_0_0.5_to_1.0.mat' ;
% filename = 'segway_error_functions_v_0_1.0_to_1.5.mat' ;

% set this to true to save the output
save_data_flag = false ;

%% automated from here
% load timing info
load('segway_timing.mat')

% load error info
load(filename)

% set max speed
max_speed = 1.5 ; % we'll just hard code this for now

% create the agent
A = segway_agent() ;

% create the range of command inputs and initial conditions
w_range = [0, w_max] ;
v_range = [-delta_v, delta_v] ;
w_0_range = [0, w_max] ;
v_0_range = [v_0_min, v_0_max] ;

% initialize time and distance scales
time_scale = get_t_f_from_v_0_for_segway(v_0_min) ;
distance_scale_x = 0 ;
distance_scale_y = 0 ;

% get the stopping time for the given speed range
t_f = get_t_f_from_v_0_for_segway(v_0_min) ;

%% find distance scale
% iterate through initial conditions and commanded yaw rates and speeds

% for each initial yaw rate...
for w_0 = w_0_range
    % set desired omega range based on current omega
    w_des_range = [w_0 - delta_w, w_0 + delta_w] ;
    
    % for each initial speed...
    for v_0 = v_0_range
        
        % for each commanded yaw rate...
        for w_des_cur = w_des_range
            % make sure the commanded yaw rate obeys the max/min bunds
            w_des = bound_values(w_des_cur,w_max) ;
            
            % get the range of speeds to test
            v_des_range = v_0 + v_range ;
            
            % for each commanded speed...
            for v_des_cur = v_des_range
                % make sure we're not above the max speed
                v_des = bound_values(v_des_cur,[0, max_speed]) ;
                
                % reset the agent
                z0 = [0;0;0;w_0;v_0] ;
                A.reset(z0) ;
                
                % create the trajectory to realize
                [T,U,Z] = make_segway_desired_trajectory(t_f,w_des,v_des) ;
                
                % run the agent
                A.move(T(end),T,U,Z) ;
                
                % find the distance traveled
                distance_scale_x = max(distance_scale_x, max(A.state(A.position_indices(1),:))) ;
                distance_scale_y = max(distance_scale_y, 2*max(A.state(A.position_indices(2),:))) ;
            end
        end
    end
end

% find the distance scale
if v_0_min == 0.0
    distance_scale = max(distance_scale_x,distance_scale_y) + 3*A.footprint ;
else
    distance_scale = max(distance_scale_x,distance_scale_y) + 2*A.footprint ;
end
disp(['Distance scale found: ',num2str(distance_scale,'%0.2f')])

% set the (x0,y0) offset so that x0 lies at -0.5, since the dynamics are
% scaled to travel a distance of around 1
initial_x = -0.5 ;
initial_y = 0 ;

%% save output
if save_data_flag
    filename = ['segway_FRS_scaling_v_0_',...
        num2str(v_0_min,'%0.1f'),'_to_',...
        num2str(v_0_max,'%0.1f'),'.mat'] ;
    
    disp(['Saving data to ',filename])
    
    save(filename,'time_scale','distance_scale',...
        'distance_scale_x','distance_scale_y',...
        'max_speed',...
        'initial_x','initial_y') ;
end
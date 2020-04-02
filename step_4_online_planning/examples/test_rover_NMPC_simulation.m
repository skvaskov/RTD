clear
close all
%% description
% This script runs a simulation with the Rover in the simulator
% framework, using RTD to plan online.
%
% Author: Sean Vaskov
% Created: 12 March 2020
%
%% user parameters
% world
obstacle_size_bounds = [0.3,0.7;0.2,0.4] ; 
obstacle_long_spacing = [2,5];
obstacle_long_spacing_mean = 3;
obstacle_long_spacing_std_dev = 1.0;
obstacle_lat_spacing = [-0.1 0.1];
obstacle_rotation_bounds = [-1,1]*2*pi/180;
N_obstacles = 3  ;
road_length = 30;
lane_width = 0.6;
bound_space = 0.4;

% planner
buffer = 0.1 ; % m
t_plan = 10 ; % if t_plan = t_move, then real time planning is enforced
t_move = 0.5 ;
T_min = 1.5; %enough time to plan braking trajectory from 2 m/s
T_max = 5;
desired_speed = 2;
speed_weight = 1;
lateral_weight = 1;
longitudinal_weight = 1;
steering_weight = 1;
lookahead_distance = 4;
max_heading = 0.6;


plot_HLP_flag = true;

run ~/MATLAB/GPOPS-II/gpopsMatlabPathSetup.m

% simulation
verbose_level = 5 ;
max_sim_time = 1000 ;
max_sim_iterations = 1000 ;

%% automated from here
A = RoverAWD();

W = two_lane_road_static('N_obstacles',N_obstacles,'bound_space',0.4,...
                     'verbose',verbose_level, 'obstacle_size_bounds',obstacle_size_bounds,...
                     'lane_width',lane_width,'road_length',road_length,...
                     'obstacle_long_spacing',obstacle_long_spacing,...
                     'obstacle_long_spacing_mean',obstacle_long_spacing_mean,...
                     'obstacle_long_spacing_std_dev',obstacle_long_spacing_std_dev,...
                     'obstacle_lat_spacing',obstacle_lat_spacing,'obstacle_rotation_bounds',obstacle_rotation_bounds) ;

P = rover_GPOPS_planner('plot_HLP_flag',plot_HLP_flag','HLP',lane_HLP,'timeout',t_plan,'t_plan',t_plan,'t_move',t_move,'T_min',T_min,'T_max',T_max,...
                        'speed_weight',speed_weight,'steering_weight',steering_weight,'lateral_weight',lateral_weight,...
                         'longitudinal_weight',longitudinal_weight,'desired_speed',desired_speed,'lookahead_distance',lookahead_distance,...
                         'max_heading',0.6,'buffer',buffer);
                 
                 
S = simulator(A,W,P,'allow_replan_errors',true,'verbose',verbose_level,...
              'max_sim_time',max_sim_time,...
              'max_sim_iterations',max_sim_iterations) ;

%% run simulation
S.run() ;
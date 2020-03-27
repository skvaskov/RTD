clear all;
close all;
%% description
% This script generates 1000 random static_box_world objects for the
% simulator framework, and saves them as a .mat file
%
% Author: Sean Vaskov
% Created: 26 Mar 2020
% Updated: -
%
%% user parameters
N_worlds = 1000;
% world parameters
obstacle_size_bounds = [0.3,0.7;0.2,0.4] ; 
obstacle_long_spacing = [0,8];
obstacle_long_spacing_mean = 5;
obstacle_long_spacing_std_dev = 1.0;
obstacle_lat_spacing = [-0.1 0.1];
obstacle_rotation_bounds = [-1,1]*2*pi/180;
N_obstacles = 3  ;
road_length = 30;
lane_width = 0.6;
bound_space = 0.4;

% file i/o
save_flag = true ;
worlds_filename = 'rover_simulation_worlds.mat' ;

%% automated from here
disp('Making new worlds')

W_all = cell(1,N_worlds) ;

for w_idx = 1:N_worlds

    W_all{w_idx} = two_lane_road_static('N_obstacles',N_obstacles,'bound_space',0.4,...
                      'obstacle_size_bounds',obstacle_size_bounds,...
                     'lane_width',lane_width,'road_length',road_length,...
                     'obstacle_long_spacing',obstacle_long_spacing,...
                     'obstacle_long_spacing_mean',obstacle_long_spacing_mean,...
                     'obstacle_long_spacing_std_dev',obstacle_long_spacing_std_dev,...
                     'obstacle_lat_spacing',obstacle_lat_spacing,'obstacle_rotation_bounds',obstacle_rotation_bounds);
end


if save_flag
    disp(['Saving worlds to ',worlds_filename])
    save(worlds_filename, 'W_all','N_worlds')
end
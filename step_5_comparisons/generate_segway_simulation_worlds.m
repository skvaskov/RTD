%% description
% This script generates 1000 random static_box_world objects for the
% simulator framework, and saves them as a .mat file
%
% Author: Shreyas Kousik
% Created: 25 Mar 2020
% Updated: -
%
%% user parameters
% world parameters
N_worlds = 1000 ;
N_obstacle_bounds = [5,15] ;
bounds = [-4,5,-2.5,2.5] ;
goal_radius = 0.5 ;

% file i/o
save_flag = true ;
worlds_filename = 'segway_simulation_worlds.mat' ;

%% automated from here
disp('Making new worlds')

W_all = cell(1,N_worlds) ;

for w_idx = 1:N_worlds
    N_obstacles = rand_int(N_obstacle_bounds(1),N_obstacle_bounds(2)) ;
    
    W_all{w_idx} = static_box_world('bounds',bounds,...
        'N_obstacles',N_obstacles,...
        'goal_radius',goal_radius,...
        'obstacle_size_bounds',[0.3, 0.3]) ;
end


if save_flag
    disp(['Saving worlds to ',worlds_filename])
    save(worlds_filename, 'W_all','N_worlds')
end
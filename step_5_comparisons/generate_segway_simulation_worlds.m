%% description
% This script generates 1000 random static_box_world objects for the
% simulator framework, and saves them as a .mat file
%
% Author: Shreyas Kousik
% Created: 25 Mar 2020
% Updated: 27 Mar 2020
%
%% user parameters
% world parameters
bounds = [-4,5,-2.5,2.5] ;
goal_radius = 0.5 ;
buffer = 0.5 ; % prevents start and goal being too close to world bounds
N_obstacles_bounds = [1,10] ; % [min,max] N_obstacles per world
N_worlds_per_N_obs = 100 ;

% file i/o
save_flag = true ;
worlds_filename = 'segway_simulation_worlds.mat' ;

%% automated from here
disp('Making new worlds')

N_obstacles_vec = N_obstacles_bounds(1):N_obstacles_bounds(2) ;
N_worlds = N_worlds_per_N_obs*length(N_obstacles_vec) ;
W_all = cell(1,N_worlds) ;

world_idx = 1 ;
for N_obstacles = N_obstacles_vec
    for idx = 1:N_worlds_per_N_obs
        W_all{world_idx} = static_box_world('bounds',bounds,...
            'buffer',buffer,...
            'N_obstacles',N_obstacles,...
            'goal_radius',goal_radius,...
            'obstacle_size_bounds',[0.3, 0.3]) ;
        world_idx = world_idx + 1 ;
    end
end


if save_flag
    disp(['Saving worlds to ',worlds_filename])
    save(worlds_filename, 'W_all','N_worlds')
end
%% description
% This script inspects the results of experiment 1 for the RRT planner, to
% determine what an appropriate buffer size is to balance between safety
% and performance (i.e., not crashing, vs. reaching the goal)
%
% NOTE, to run this script, you should be in the following directory:
%   IJRR_bridging_the_gap/step_5_comparisons/experiment_1/segway_data
%
% Author: Shreyas Kousik
% Created: 25 Mar 2020
% Updated: -
%
%% automated from here
summary_save_filename_header = 'segway_RRT_experiment_1_summary' ;
f = dir(pwd) ;

goal = [] ;
collision = [] ;

for idx = 1:length(f)
    n = f(idx).name ;
    if startsWith(n,summary_save_filename_header)
        load(f(idx).name)
        
        goal_temp = [] ;
        collision_temp = [] ;
        for p_idx = 1:length(summary)
            goal_temp = [goal_temp ; summary(p_idx).goal_check] ;
            collision_temp = [collision_temp ; summary(p_idx).collision_check] ;
        end
        
        goal = [goal, goal_temp] ;
        collision = [collision, collision_temp] ;
    end
end

%% plotting
sum(goal,2)
sum(collision,2)
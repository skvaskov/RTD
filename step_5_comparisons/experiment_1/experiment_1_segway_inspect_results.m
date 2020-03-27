%% description
% This script inspects the results of experiment 1 for the RRT planner, to
% determine what an appropriate buffer size is to balance between safety
% and performance (i.e., not crashing, vs. reaching the goal)
%
% This takes about 5 seconds to load the data from 500 files
%
% NOTE, to run this script, you should be in the following directory:
%   IJRR_bridging_the_gap/step_5_comparisons/experiment_1/segway_data
%
% Author: Shreyas Kousik
% Created: 25 Mar 2020
% Updated: -
%
%% automated from here
% load each summary file and get the number of goals and collisions
summary_save_filename_header = 'segway_experiment_1_summary' ;
f = dir(pwd) ;

goal = [] ;
collision = [] ;

disp('Loading data')
tic
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
toc

% sanity check
if isempty(goal)
    error(['Please navigate to the directory where the experiment data is, ',...
        'and make the variable sure summary_save_filename_header is named ',...
        'to match your data .mat files.'])
end

%% get total number of goals and collisions
goal_result = sum(goal,2) ;
collision_result = sum(collision,2) ;
N_worlds_str = num2str(size(goal,2)) ;

disp(' ')
disp('--------------------------------------------------------------------')
disp('EXPERIMENT 1 RESULTS')
disp(' ')
disp(['Number of worlds: ',N_worlds_str])
disp(['Number of planners: ',num2str(length(summary))])
disp(' ')

for p_idx = 1:length(summary)
    disp(['Planner ',num2str(p_idx),': ',summary(p_idx).planner_name])
    disp(['    Goals:        ',...
        num2str(goal_result(p_idx)),' / ',N_worlds_str]) ;
    disp(['    Collisions:   ',...
        num2str(collision_result(p_idx)),' / ',N_worlds_str]) ;
    disp(' ')
end
disp('--------------------------------------------------------------------')
disp(' ')
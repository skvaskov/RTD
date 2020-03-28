function varargout = inspect_experiment_results(directory)
% inspect_experiment_results()
% inspect_experiment_results(directory)
% [goals, collisions, planners] = inspect_experiment_results(directory)
%
% This function loads all of the experiment results .mat files in a given
% directory and compiles the number of goals reached and number of
% collisions experienced by each planner. If no directory is provided, it
% attempts to load everything from the present working directory.
%
% Author: Shreyas Kousik
% Created: 25 Mar 2020
% Updated: 27 Mar 2020

if nargin < 1
    f = dir(pwd) ;
else
    f = dir(directory) ;
end

% load each summary file and get the number of goals and collisions
N_files = length(f) ;
N_report = floor(N_files/10) ;

goal = [] ;
collision = [] ;

disp('Loading data')
tic
for idx = 1:N_files
    n = f(idx).name ;
    if contains(n,'summary')
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
    
    % report loading file progress
    if mod(idx,N_report) == 0
        disp([num2str(100*idx/N_files,'%0.0f'),' % complete'])
    end
end
toc

% sanity check
if isempty(goal)
    error('Please navigate to a directory containing experiment data!')
end

% get total number of goals and collisions
goal_result = sum(goal,2) ;
collision_result = sum(collision,2) ;
N_worlds_str = num2str(size(goal,2)) ;

% get planner names
planner_names = {} ;
for p_idx = 1:length(summary)
    planner_names{p_idx} = summary(p_idx).planner_name ;
end

% display results
disp(' ')
disp('--------------------------------------------------------------------')
disp('RESULTS')
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

% set output args
varargout = {goal_result, collision_result, planner_names} ;
varargout = varargout(1:nargout) ;
classdef world < handle
% Class: world
%
% This world superclass contains generic properties and methods to be used
% in the simulator framework. It is 2-D by default.
%
% PROPERTIES
%     dimension             specifies 2-D or 3-D world; default is 2
%
%     start                 (x,y) or (x,y,z) agent start position
%
%     goal                  (x,y) or (x,y,z) desired final location
%
%     goal_radius           radius about the goal; if the agent's center of
%                           mass is within goal_radius of the goal in the
%                           2-norm, then the agent has reached the goal
%
%     bounds                2-D bounds written as [xmin,xmax,ymin,ymax],
%                           or 3-D bounds [xmin,xmax,ymin,ymax,zmin,zmax]
%
%     N_obstacles           number of obstacles (integer)
%
%     obstacles             the obstacles in the world; by default, this is
%                           a 2-by-N array defining polygons by their
%                           vertices, and separated by columns of nans
%
%     current_time          the current time in the world, updated using
%                           agent_info the world does a collision check
%
%     verbose               verbosity level of display; 0 is silent, 1 is
%                           mildly talkative, and 2 or greater is more
%                           talkative
%
%     plot_data             a structure containing the data output of, for
%                           example, the plot function; data can be updated
%                           in the world.plot method instead of re-plotting
%                           entirely, which makes for smooth animations
%
% METHODS
%     world                 constructor method
%
%     setup                 creates start, goal, and obstacles; since
%                           obstacle configurations are typically
%                           randomized, this is usually called each time
%                           before running a simulation to present all of
%                           the simulator2D planners with a new world
%
%     reset                 resets the current time index to 1 (used when
%                           the agent is reset to the start location,
%                           usually); is usually overwritten by subclasses
%
%     get_world_info        returns obstacles within the sensor window of
%                           the agent by default; should be overwritten by
%                           subclasses to be more situation-specific
%
%     goal_check            checks if the agent is within goal_radius of
%                           the goal
%
%     collision_check       checks if the agent has collided with any
%                           obstacles; should be overwritten by subclasses
%                           to be obstacle-specific
%
%     vdisp                 verbose display; prints messages to screen that
%                           if they have a low enough verbosity level

    properties (Access = public)
        dimension = 2 ;
        start = [] ;
        goal = [] ;
        goal_radius = 1 ;
        bounds = Inf*[-1 1 -1 1] ;
        N_obstacles = [] ;
        obstacles = [] ;
        collision_check_time_discretization = 0.005 ;
        animation_time_discretization = 0.01 ;
        current_time = 0 ;
        verbose = 0 ;
        name = 'world' ;
        plot_data
    end
    
    methods
    %% constructor
        function W = world(varargin)
            W = parse_args(W,varargin{:}) ;
        end
        
    %% setup
        function setup(W)
        % Method: W.setup()
        %
        % Create a random start and goal point. Subclasses should override
        % this with a situation-specific setup function.
            
            % generate default start and goal locations
            if isempty(W.start)
                W.start = zeros(W.dimension,1) ;
            end
            
            if isempty(W.goal)
                W.goal = zeros(W.dimension,1) ;
            end
            
            % reset world time index
            W.current_time = 0 ;
        end
        
    %% reset
        function reset(W)
        % Method: W.reset()
        %
        % Reset the world's current time index to 1. This is generic and
        % can be overwritten in a subclass. This allows a world to reset
        % its timing without creating new start/goal/obstacles, which is
        % the point of the world.setup method.
            W.current_time = 0 ;
        end
        
    %% get world info for planning
        function I = get_world_info(W,~,~)
        % Method: I = get_world_info(agent_info,planner_info)
        %
        % Return all of a world's obstacles, and the obstacle type, in a
        % structure. This is generic and should be overwritten by specific
        % methods that, e.g., check which obstacles are within the robot's
        % sensor radius, or model occlusions.
            I.obstacles = W.obstacles ;
            I.start = W.start ;
            I.goal = W.goal ;
            I.dimension = W.dimension ;
            I.bounds = W.bounds ;
        end
        
    %% goal check
        function out = goal_check(W,agent_info)
        % Method: out = goal_check(agent_info)
        %
        % Checks if the agent's center of mass is within W.goal_radius of
        % W.goal in the 2-norm.
            z = agent_info.position ;
            dz = z - repmat(W.goal,1,size(z,2)) ;
            out = min(vecnorm(dz,2)) <= W.goal_radius ;
        end
        
    %% collision check
        function out = collision_check(W,agent_info,check_full_traj)
        % out = W.collision_check(agent_info)
        % out = W.collision_check(agent_info,check_full_traj)
        %
        % Run a collision check given the agent's information object. The
        % second input states whether or not to check the agent's entire
        % trajectory; by default this is false, and the world only checks
        % the duration of the trajectory that is new since the last
        % collision check was run. The output is false if there are no
        % collisions.
        
            % whether or not to check full agent trajectory (default is no)
            if nargin < 3
                check_full_traj = false ;
            end
            
            if check_full_traj
                W.vdisp('Checking entire trajectory (this might take a while)',3)
                t_start = 0 ;
            else    
                t_start = W.current_time ;
            end
            
            % create time vector for checking
            t_agent = agent_info.time(end) ;
            t_check = t_start:W.collision_check_time_discretization:t_agent ;
            
            if isempty(t_check) || t_check(end) ~= t_agent
                t_check = [t_check, t_agent] ;
            end
            
            % get agent trajectory interpolated to time
            z_agent = match_trajectories(t_check,agent_info.time,agent_info.state) ;
            
            % run collision check
            W.vdisp('Running collision check!',3)
            out = false ; % optimism!
            t_idx = 1 ;
            while ~out && t_idx <= length(t_check)
                z = z_agent(:,t_idx) ;
                out = W.collision_check_single_state(agent_info,z) ;
                t_idx = t_idx + 1 ;
            end
            
            if out
                W.vdisp(['Collision detected at t = ',num2str(t_check(t_idx))],1)
            else
                W.vdisp('No collisions detected',3)
            end
            
            % update world time
            W.current_time = t_agent ;
        end
        
        function out = collision_check_single_state(W,agent_info,state)
        % out = W.collision_check_single_state(agent_info,state)
        %
        % Perform a collision check for the provided state, and return true
        % if it is in collision. By default, this just returns false.
            out = false ;
        end
       
    %% plotting
        function plot(W)
            if ~any(isinf(W.bounds))
                axis(W.bounds)
            end
            
            % TO DO: plot start and goal, obstacles, and world bounds
            
%             % plot start and goal
%             xcirc = cos(linspace(0,2*pi)) ;
%             ycirc = sin(linspace(0,2*pi)) ;
%             
%             plot(W.start(1), W.start(2), 'ko')
%             plot(W.goal(1), W.goal(2), 'kx')
%             r = W.goal_radius ;
%             plot(r*xcirc + W.goal(1), r*ycirc + W.goal(2), 'k:')
        end
        
        function plot_at_time(W,t,color)
            if nargin < 3
                color = [1 0 0] ;
            end
            
            W.vdisp('Plotting at a specific time is undefined.',1)
        end
        
        function animate(W,time_interval_to_animate)
            % W.animate()
            % W.animate([t_start, t_finish])
            %
            % Animates the given world. By default, the animation is from
            % duration t = 0 to t = W.current_time, and discretized by the
            % animation_time_discretization property. If the second input
            % is given as a time interval, then that time interval is
            % animated.
            
            if nargin < 2
                T = [0, W.current_time] ;
            else
                T = time_interval_to_animate ;
            end
            
            if diff(T) > 0
                for tidx = T(1):W.animation_time_discretization:T(end)
                    W.plot_at_time(tidx)
                    pause(W.animation_time_discretization) ;
                end
            else
                plot(W)
            end
        end
        
%% utility
        function vdisp(W,s,l)
        % Display a string s if the message's verbose level l is greater
        % than or equal to the planner's verbose level.
            if nargin < 3
                l = 1 ;
            end
            if W.verbose >= l
                if ischar(s)
                    disp(['    W: ',s])
                else
                    disp('    W: String not provided!')
                end
            end
        end
    end
end

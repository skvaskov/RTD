classdef high_level_planner < handle
    % Class: high_level_planner
    %
    % For use with simulator. The high-level planner is responsible for
    % creating coarse plans for an agent to navigate a world. This class is
    % mean to be called by a planner object in its replan method while planning
    % online; it is not called directly by simulator. It turns out that having
    % a coarse planner that generates, e.g., waypoints, is really useful for
    % enabling a mid-level trajectory planner to do its thing.
    
    %% properties
    properties
        dimension = 2 ;
        default_lookahead_distance = 1 ;
        waypoint_reached_radius = 1 ;
        waypoints = [] ;
        N_waypoints = 0 ;
        current_waypoint = [] ;
        current_waypoint_index = 1 ;
        waypoints_include_heading = false
        verbose = 0 ;
        plot_data = [] ;
        waypoint_color = [1 0.5 0] ;
        waypoint_marker = '^' ;
        waypoint_line_style = '--' ;
        goal = [1;0] ;
        bounds ;
    end
    
    methods
        %% constructor
        function HLP = high_level_planner(varargin)
            HLP = parse_args(HLP,varargin{:}) ;
            HLP.dimension = length(HLP.goal) ;
            
            % set default plot data
            HLP.plot_data.waypoints = [] ;
            HLP.plot_data.current_waypoint = [] ;
        end
        
        %% setup
        function setup(HLP,agent_info,world_info)
            HLP.dimension = world_info.dimension ;
            HLP.goal = world_info.goal ;
            HLP.bounds = world_info.bounds ;
        end
        
        %% get waypoint
        function waypoint = get_waypoint(HLP,~,~,~)
            % waypoint = get_waypoint(agent_info,world_info,lookahead_distance)
            %
            % This is the function to be called by the planner replan method. It
            % should return a waypoint, which is usually an (x,y) position or an
            % (x,y,h) pose in SE^2.
            warning('The getWaypoint method is undefined!')
            waypoint = [] ;
        end
        
        %% plot
        function plot(HLP)
            % plot(HLP)
            %
            % Plots all current waypoints as a line, and plots the current waypoint
            % with a marker (default is a triangle). This method plots in 2D or 3D
            % as appropriate. Note, this plot method should be called by the
            % planner in its plot method, since it is not called by simulator.
            
            plot_format = HLP.waypoint_line_style ;
            wps = HLP.waypoints ;
            
            % check for plot
            plot_up = check_if_plot_is_available(HLP,'waypoints') ;
            
            if ~isempty(wps)
                if plot_up
                    HLP.plot_data.waypoints.XData = wps(1,:) ;
                    HLP.plot_data.waypoints.YData = wps(2,:) ;
                    
                    if HLP.dimension > 2
                        HLP.plot_data.waypoints.ZData = wps(3,:) ;
                    end
                else
                    switch HLP.dimension
                        case 2
                            HLP.plot_data.waypoints = plot(wps(1,:),wps(2,:),...
                                plot_format,'Color',HLP.waypoint_color) ;
                        case 3
                            HLP.plot_data.waypoints = plot2(wps(1,:),wps(2,:),wps(3,:),...
                                plot_format,'Color',HLP.waypoint_color) ;
                    end
                end
            end
            
            wp_cur = HLP.current_waypoint ;
            
            if ~isempty(wp_cur)
                if plot_up
                    HLP.plot_data.current_waypoint.XData = wp_cur(1) ;
                    HLP.plot_data.current_waypoint.YData = wp_cur(2) ;
                    
                    if HLP.dimension > 2
                        HLP.plot_data.current_waypoint.ZData = wp_cur(3) ;
                    end
                else
                    switch HLP.dimension
                        case 2
                            HLP.plot_data.current_waypoint = plot(wp_cur(1),wp_cur(2),...
                                HLP.waypoint_marker,'Color',HLP.waypoint_color) ;
                        case 3
                            HLP.plot_data.current_waypoint = plot3(wp_cur(1),wp_cur(2),wp_cur(3),...
                                HLP.waypoint_marker,'Color',HLP.waypoint_color) ;
                    end
                end
            end
        end
        
        function plot_at_time(HLP,t)
            % plot_at_time(HLP,t)
            %
            % Plot the HLP at the given time t. By default, this is just the same
            % as the HLP.plot method.
            plot(HLP) ;
        end
        
        %% display info
        function vdisp(HLP,s,l)
            % Display a string s if the message's verbose level l is greater
            % than or equal to the waypoint planner's verbose level.
            if nargin < 3
                l = 1 ;
            end
            if HLP.verbose >= l
                if ischar(s)
                    disp(['        HLP: ',s])
                else
                    disp('        HLP: String not provided!')
                end
            end
        end
    end
end
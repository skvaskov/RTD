classdef planner < handle
    properties
        % generic planner properties
        name
        bounds % world bounds plus planner-specific buffer
        buffer % minimum amount to buffer obstacles, given by the world
        HLP % high level planner
        current_plan ;
        current_obstacles ;
        verbose = 0 ;
        timeout = 1 ; % time allowed for "replan" function to execute
        t_plan = 1 ; % same as timeout; just for notational purposes
        t_move = 1 ;% amount of time the planner expects the agent to move
        info % information structure to keep a log when planning
        plot_data % data for current plot
        plot_HLP_flag = false ;
    end
    
    methods
        %% constructor
        function P = planner(varargin)
            P = parse_args(P,varargin{:}) ;
            
            P.plot_data.current_plan = [] ;
        end
        
        %% setup
        function setup(P,~,~)
            % P.setup(agent_info,world_info)
            %
            % ADD COMMENTS
            P.vdisp('Planner setup function is undefined!')
        end
        
        %% replan
        function [T_nom,U_nom,Z_nom] = replan(~,~,~)
            % [T_nom,U_nom,Z_nom] = P.replan(agent_info,world_info)
            %
            % ADD COMMENTS
            warning('Planner replan function is undefined!')
            
            if isempty(P.HLP)
                warning('The high level planner is also undefined!')
            end
            
            T_nom = [] ; U_nom = [] ; Z_nom = [] ;
        end
        
        %% plotting
        function plot(P,color)
            if nargin < 2
                color = [0 0 1] ;
            end
            
            hold_check = ~ishold ;
            if hold_check
                hold on
            end
            
            if check_if_plot_is_available(P,'current_plan')
                P.vdisp('Updating plot',5) ;
                P.plot_data.current_plan.XData = P.current_plan(1,:) ;
                P.plot_data.current_plan.YData = P.current_plan(2,:) ;
            else
                P.vdisp('Plotting new data',5)
                
                % plot anticipated trajectory from planner
                try
                    if ~isempty(P.current_plan)
                        P.plot_data.current_plan = plot(P.current_plan(1,:),P.current_plan(2,:),...
                            ':','LineWidth',2,'Color',color) ;
                    end
                catch
                    P.vdisp('Unable to plot current plan!',5)
                end
            end
            
            % plot waypoints
            if P.plot_HLP_flag
                P.HLP.plot() ;
            end
            
            if hold_check
                hold off
            end
        end
        
        function plot_at_time(P,t,color)
            if nargin < 2
                color = [0 0 1] ;
            end
            
            P.vdisp('Plotting at a specific time is undefined.',1)
        end
        
        %% utility
        function vdisp(P,s,l)
            % Display a string s if the message's verbose level l is greater
            % than or equal to the planner's verbose level.
            if nargin < 3
                l = 1 ;
            end
            if P.verbose >= l
                if ischar(s)
                    disp(['    P: ',s])
                else
                    disp('    P: String not provided!')
                end
            end
        end
    end
end
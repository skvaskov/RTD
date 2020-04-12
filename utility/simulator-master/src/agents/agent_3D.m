classdef agent_3D < agent
methods
    %% constructor
    function A = agent_3D(varargin)
        n_states = 3 ;
        position_indices = 1:3 ;
        A@agent('n_states',n_states,...
            'position_indices',position_indices,...
            varargin{:},'dimension',3) ;
        A.reset() ;
    end
    
    %% reset
    function reset(A,state)
        % method: reset(state)
        %
        % Zeros out the agent's states and inputs, and sets its time to
        % zero. By default, the state input can be a point in R^3, i.e. a
        % point (x,y,z), or it can be A.n_states-by-1.
        if nargin < 2
            reset@agent(A) ;
        else
            reset@agent(A,state) ;
        end
    end
    
    %% plotting
    function plot(A,color)
        if nargin < 2
            color = [0 0 1] ;
        end
        
        % get data to plot
        xyz = A.state(A.position_indices,:) ;
        
        % check whether or not to create new plot data
        if check_if_plot_is_available(A,'trajectory')
            A.vdisp('Updating plot',5) ;
            A.plot_data.trajectory.XData = xyz(1,:) ;
            A.plot_data.trajectory.YData = xyz(2,:) ;
            A.plot_data.trajectory.ZData = xyz(3,:) ;
        else
            A.vdisp('Plotting new data',5)
            
            % plot trajectory
            trajectory_data = plot3(xyz(1,:),xyz(2,:),xyz(3,:),'Color',color) ;
            
            % update plot data
            A.plot_data.trajectory = trajectory_data ;
        end
    end
    
    function plot_at_time(A,t,color)
        if nargin < 3
            color = [0 0 1] ;
        end
        
        % get state at time t
        z_t = match_trajectories(t,A.time,A.state) ;
        xyz = z_t(A.position_indices);

        % check if a figure is up; if so, create a new figure,
        % otherwise update the existing data
        if check_if_plot_is_available(A,'trajectory')
            A.plot_data.trajectory.XData = xyz(1) ;
            A.plot_data.trajectory.YData = xyz(2) ;
            A.plot_data.trajectory.ZData = xyz(3) ;
        else
            % plot trajectory
            trajectory_data = plot3(xyz(1),xyz(2),xyz(3),'o','Color',color) ;
            A.plot_data.trajectory = trajectory_data ;
        end
    end
    
    function lims = get_axis_lims(A)
        z = A.state(A.position_indices,:) ;
        xmin = min(z(1,:)) - A.animation_plot_buffer ;
        xmax = max(z(1,:)) + A.animation_plot_buffer ;
        ymin = min(z(2,:)) - A.animation_plot_buffer ;
        ymax = max(z(2,:)) + A.animation_plot_buffer ;
        zmin = min(z(3,:)) - A.animation_plot_buffer ;
        zmax = max(z(3,:)) + A.animation_plot_buffer ; 
        lims = [xmin xmax ymin ymax zmin zmax] ;
    end
end
end
classdef agent < handle
% Class: agent
%
% Generic point-mass agent with zero dynamics as default. Other agents
% should inherit this as a superclass.

    properties
        % state space model properties
        state = [] ;
        time = [] ;
        input = [] ;
        input_time = [] ;
        dimension = 2 ;
        n_states = 2 ;
        n_inputs = 0 ;
        position_indices = [1 2] ;
        
        % sensing
        sensor_radius = 1 ;
        
        % low-level controller
        LLC = [] ;
        
        % collision-checking
        collision_check_input_data
        
        % plotting
        plot_data = struct('trajectory',[]) ;
        plot_input_data
        
        % animation
        animation_plot_buffer = 1 ; % meter
        animation_time_discretization  = 0.05 ;
        animation_playback_rate = 1 ;
        animation_set_axes_flag = false ;
        animation_set_view_flag = false ;
        animation_view = 2 ;
        animation_gif_filename = 'agent_animation.gif' ;
        
        % user-friendly properties
        name = 'agent' ;
        verbose = 0 ;
    end
    
    methods
    %% constructor
        function A = agent(varargin)
        % A = agent('property1',value1,'property2',value2,...)
        % A = agent(verbose_level,'property1',value1,'property2',value2,...)
        %
        % This method creates an agent object. It takes in string/argument
        % pairs, where the strings are the names of the agent default
        % properties.
            if mod(length(varargin),2) == 1
                verbose_level = varargin{1} ;
                varargin = varargin(2:end) ;
                A.verbose = verbose_level ;
            end
            
            A = parse_args(A,varargin{:}) ;
            
            A.reset() ;
        end
        
        %% reset
        function reset(A,state)
            % method: reset(state)
            %
            % Zeros out the agent's states and inputs, and sets its time to
            % zero. The state input can be in R^2, i.e. a point (x,y), or
            % it can be A.n_states-by-1.
            
            if nargin < 2
                state = zeros(A.n_states,1) ;
            end
            
            A.state = zeros(A.n_states,1) ;
            if length(state) == 2
                A.state(A.position_indices) = state(1:2) ;
            else
                N = length(state) ;
                A.state = zeros(A.n_states,1) ;
                A.state(1:N) = state ;
            end
            A.time = 0 ;
            A.input = zeros(A.n_inputs,1) ; % this forces the input size to
                                            % match the time vector size
            A.input_time = 0 ;
        end
        
        %% get agent info
        function agent_info = get_agent_info(A)
            agent_info.dimension = A.dimension ;
            agent_info.state = A.state ;
            agent_info.position = A.state(A.position_indices,:) ;
            agent_info.position_indices = A.position_indices ;
            agent_info.time = A.time ;
            agent_info.sensor_radius = A.sensor_radius ;
            agent_info.n_inputs = A.n_inputs ;
            agent_info.n_states = A.n_states ;
        end
        
        %% move
        function move(A,t_move,T_ref,U_ref,Z_ref)
            % method: move(t_move,T_ref,U_ref,Z_ref)
            %
            % Moves the agent for the duration t_move using the nominal
            % inputs U_ref and nominal trajectory Z_ref that are indexed by
            % the nominal time T_ref.
            %
            % This method assumes that the input is zero-order hold, and
            % the input corresponding to the last time index is zero; this
            % is why the last old input is discarded when the input is
            % updated. Similarly, this method assumes that the nominal time
            % starts at 0.
            
            A.vdisp('Moving!',5)
            
            % set up default reference trajectory
            if nargin < 5
                Z_ref = [] ;
            end
            
            % get the time, input, and reference trajectory to use for
            % moving the agent
            [T_used,U_used,Z_used] = A.move_setup(t_move,T_ref,U_ref,Z_ref) ;
            
            % get the current state
            zcur = A.state(:,end) ;
            
            % call the ode solver to simulate agent
            [tout,zout] = A.integrator(@(t,z) A.dynamics(t,z,T_ref,U_ref,Z_ref),...
                                       [0 t_move], zcur) ;
            
            A.commit_move_data(tout,zout,T_used,U_used,Z_used) ;
        end
        
        function [T,U,Z] = move_setup(A,t_move,T_ref,U_ref,Z_ref)
            % method: [T,U,Z] = move_setup(A,t_move,T_ref,U_ref,Z_ref)
            %
            % Given an amount of time t_move for the agent to move, and a
            % reference time vector, output a time vector to actually use
            % during the call to move the agent.
            
            % timing setup
            if T_ref(end) < t_move
                A.vdisp(['Provided input time vector is shorter than the ',...
                        'desired motion time! The agent will still be ',...
                        'moved for the duration t_move.'])
            end

            % make sure the reference time is a row vector
            T_ref = T_ref(:)' ;
            
            % make sure the reference time is unique, and only get those
            % rows of U_ref and Z_ref
            [T_ref,unique_idxs,~] = unique(T_ref,'stable') ;
            U_ref = U_ref(:,unique_idxs) ;
            
            
            % get the amount of time to actually move the agent
            tlog = T_ref <= t_move ;
            T = T_ref(tlog) ;
            if T(end) < t_move
                T = [T, t_move] ;
            end
            
            % interpolate the reference input and trajectory to pass to the
            % agent's move method
            if nargin < 5 || isempty(Z_ref)
                U = match_trajectories(T,T_ref,U_ref) ;
                Z = [] ;
            else
                Z_ref = Z_ref(:,unique_idxs) ;
                [U,Z] = match_trajectories(T,T_ref,U_ref,T_ref,Z_ref) ;
            end
        end
        
        function commit_move_data(A,T_state,Z_state,T_used,U_used,~)
            % method: commit_move_data(T_state,Z_state,T_input,U_input,Z_input)
            %
            % After moving the agent, commit the new state and input
            % trajectories, and associated time vectors, to the agent's
            % state, time, input, and input_time properties.
            
            A.state = [A.state, Z_state(:,2:end)] ;
            A.time = [A.time, A.time(end) + T_state(2:end)] ;
            A.input_time = [A.input_time, A.input_time(end) + T_used(2:end)] ;
            A.input = [A.input, U_used(:,1:end-1)] ;
        end
        
        %% stop
        function stop(A,t)
        % method: stop(t)
        %
        % By default, calls the move function with zero input for duration
        % t seconds; should be overwritten in a subclass.
        
            A.move(t,[0 t], zeros(A.n_inputs,2)) ;
        end
        
        %% default dynamics
        function zd = dynamics(~,~,z,~,~)
        % dzdt = dynamics(t,z,T,U,Z)
        %
        % If z is the state, then this function returns the time derivative
        % of z. It takes in the current time t, state z, and time vector T
        % with the same length as the input vector U and desired trajectory
        % vector Z.
            zd = zeros(size(z)) ;
        end        
        
        %% default integrator
        function [tout,zout] = integrator(~,fun,tspan,z0)
            [tout,zout] = ode45(@(t,z) fun(t,z),tspan,z0(:)) ;
            tout = tout' ;
            zout = zout' ;
        end
        
        %% verbose display
        function vdisp(A,s,l)
        % Display a string s if the message's verbose level l is greater
        % than or equal to the planner's verbose level.
            if nargin < 3
                l = 1 ;
            end
            if A.verbose >= l
                if ischar(s)
                    disp(['    A: ',s])
                else
                    disp('    A: String not provided!')
                end
            end
        end
        
        %% plotting
        function plot(A,color)
            if nargin < 2
                color = [0 0 1] ;
            end
            
            % set up data to plot
            xy = A.state(A.position_indices,:) ;
            
            hold_check = ~ishold ;
            if hold_check
                hold on
            end
            
            % check if a figure is up; if so, create a new figure,
            % otherwise update the existing data
            if check_if_plot_is_available(A,'trajectory')
                A.vdisp('Updating plot',5)
                
                A.plot_data.trajectory.XData = xy(1,:) ;
                A.plot_data.trajectory.YData = xy(2,:) ;
            else
                A.vdisp('Plotting new data',5)
                
                % plot trajectory
                trajectory_data = plot(xy(1,:),xy(2,:),'Color',color) ;
                A.plot_data.trajectory = trajectory_data ;
            end
            
            if hold_check
                hold off
            end
        end
        
        function plot_at_time(A,t,color)
            if nargin < 3
                color = [0 0 1] ;
            end
            
            % get state at time t
            z_t = match_trajectories(t,A.time,A.state) ;
            xy = z_t(A.position_indices);
            
            % check if a figure is up; if so, create a new figure,
            % otherwise update the existing data
            if check_if_plot_is_available(A,'trajectory')
                A.plot_data.trajectory.XData = xy(1) ;
                A.plot_data.trajectory.YData = xy(2) ;
                
                if A.plot_sensor_radius
                    A.plot_data.sensor_radius.XData = xcirc ;
                    A.plot_data.sensor_radius.YData = ycirc ;
                end
            else
                % plot trajectory
                trajectory_data = plot(xy(1),xy(2),'o','Color',color) ;
                A.plot_data.trajectory = trajectory_data ;

                % plot sensor radius
                if A.plot_sensor_radius
                    hold on
                    sensor_radius_data = plot(xcirc, ycirc,'--','Color',[0.5 0.5 1]) ;
                    A.plot_data.sensor_radius = sensor_radius_data ;
                    hold off
                end
            end
        end
        
        function animate(A,save_gif,time_interval)
        % method: animate(save_gif)
        %
        % Given the agent's executed trajectory, animate it for the
        % duration given by A.time. The time between animated frames is
        % given by A.animation_time_discretization.

            if nargin < 2
                save_gif = false ;
                start_gif = false ;
            else
                start_gif = true ;
                filename = A.gif_setup() ;
            end
            
            if nargin < 3
                time_interval = [A.time(1), A.time(end)] ;
            end

            % get timing info
            t_vec = time_interval(1):A.animation_time_discretization:time_interval(end) ;
            frame_rate = A.animation_time_discretization / A.animation_playback_rate ;

            % get axis limits
            lims = A.get_axis_lims() ;

            for t_idx = t_vec
                % create plot
                A.plot_at_time(t_idx)

                if A.animation_set_axes_flag
                    axis equal
                    axis(lims)
                end
                
                if A.animation_set_view_flag
                    view(A.animation_view)
                end

                % create gif
                if save_gif
                    % get current figure
                    fh = get(groot,'CurrentFigure') ;
                    frame = getframe(fh) ;
                    im = frame2im(frame);
                    [imind,cm] = rgb2ind(im,256);

                    if start_gif
                        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,...
                                'DelayTime',frame_rate) ;
                        start_gif = false ;
                    else 
                        imwrite(imind,cm,filename,'gif','WriteMode','append',...
                                'DelayTime',frame_rate) ;
                    end
                else
                    pause(frame_rate)
                end
            end
        end
        
        function lims = get_axis_lims(A)
            z = A.state(A.position_indices,:) ;
            xmin = min(z(1,:)) - A.animation_plot_buffer ;
            xmax = max(z(1,:)) + A.animation_plot_buffer ;
            ymin = min(z(2,:)) - A.animation_plot_buffer ;
            ymax = max(z(2,:)) + A.animation_plot_buffer ;
            lims = [xmin xmax ymin ymax] ;
        end
    
        function filename = gif_setup(A)       
            filename = A.animation_gif_filename ;

            dir_content = dir(pwd) ;
            filenames   = {dir_content.name} ;
            file_check  = any(cellfun(@(x) strcmp(filename,x),filenames)) ;
            filename_new = filename ;
            cur_int = 1 ;

            while file_check
                filename_new = [filename(1:end-4),'_',num2str(cur_int),filename(end-3:end)] ;
                file_check  = any(cellfun(@(x) strcmp(filename_new,x),filenames)) ;
                cur_int = cur_int + 1 ;
            end

            filename = filename_new ;
        end
    end
end
classdef rover_RRT_planner < planner
    properties        
	% implementation-specific properties
        current_nodes_thr % each column is a node in the full state space
        current_nodes_brk
        
        dt_edge=0.01 % RRT time discretization of each edge (trajectory between nodes)
        T_edge=0.5 % RRT duration of each edge
        iter_max=1e4 % RRT iteration max
        node_min=4 % RRT node


        max_dist=10 %maximum length of trajectories
        T_min=1.5 % lower bound on timing, can be used to encourage safety
        T_max=3% upper bound on timing, encourages robot to go faster
        plot_current_nodes % plots the current tree if available

        upper_boundary
        lower_boundary

        stop_flag
        braking_flag
        current
        agent_state

        
        %braking
        T_old
        U_old
        Z_old

        max_heading = 0.6;
        
        lookahead_distance = 4;
        
        desired_speed = 2;
        max_speed = 2;
        min_speed = 0.5;
        max_wheelangle = 0.5;
        
        obstacle_buffer = [0,0;0,0];
        
        rrt_dynamics
        
        current_waypoint
        
        control_weights = [0 2]' ;
        y_weight = 1;
        x_weight = 1;
   
    end
    
    methods
    %% constructor
    function P = rover_RRT_planner(varargin)
          % set default parameters
        t_plan = 0.5 ;
        timeout = 0.5;
        t_move = 0.5 ;
        buffer = 0;
        
        % parse arguments
        P@planner('t_plan',t_plan,'timeout',timeout,'t_move',t_move,'buffer',buffer,varargin{:})
       
    end
    
    %% setup
    function setup(P,agent_info,world_info)
        
        % get world bounds
        P.upper_boundary = world_info.road_upper_boundary;
        P.lower_boundary = world_info.road_lower_boundary;
        
        % initialize old state
        P.T_old = 0 ;
        P.U_old = [0 ; 0] ;
        P.Z_old = [world_info.start ; 0 ; 0] ;
        
        % set up structure for info
        I = struct('time_to_feasible_sol',[],...
                   'N_nodes_for_feasible_sol',[],...
                   'N_nodes_successful',[],...
                   'N_nodes_attempted',[],...
                   'throttle_or_brake',[]) ;
        P.info = I ;
        
        % set up plotting info
        P.braking_flag=false;
        P.stop_flag=false;
        
        %get obstacle buffer
        rotated_vertices = [cos(P.max_heading) -sin(P.max_heading);sin(P.max_heading) cos(P.max_heading)]*agent_info.footprint_vertices;

        L = [min(rotated_vertices(1,:)),max(rotated_vertices(1,:))];
        W = [min(rotated_vertices(2,:)),max(rotated_vertices(2,:))];

        rotated_vertices = [cos(P.max_heading) -sin(P.max_heading);sin(P.max_heading) cos(P.max_heading)]'*agent_info.footprint_vertices;

        L = [min([L(1),rotated_vertices(1,:)]),max([L(2),rotated_vertices(1,:)])];
        W = [min([W(1),rotated_vertices(2,:)]),max([W(2),rotated_vertices(2,:)])];
        
        P.obstacle_buffer = P.buffer+[L;W];
        
        
        % reset trajectory and tree
        P.current.t = 0;
        P.current.mode = 'LTV';
        P.current.Z = [];
        P.current.U = [];
        P.current.T = [];
        P.current_nodes_thr = [] ;

        P.U_old = [];
        P.T_old = [];
        P.Z_old = [];
    end
    
    function update(P,A,W)
        % Function: update
        %
        % This needs to be run when the world's setup function is run, to
        % update the planner with any changes made to the world
        P.setup(A,W) ;
    end
    
    %% replan
    function [Tout,Uout,Zout,mode] = replan(P,agent_info,world_info)


        % set stop and braking flags to false 
        P.stop_flag = false;
        P.braking_flag = false;

        % get initial condition from agent
        P.agent_state = agent_info.state(:,end) ;

        % update current t. current stores the last reference trajectory this is
        % the time we are in terms of the input time vector for that
        % trajectory
        P.current.t = P.current.t+P.t_move;

        % store polygon representation of obstacles for waypoint selection
          O = world_info.obstacles;
        % put obstacles into problem (obstacles must be in ccw order
        % separated by NaNs)
        if ~isempty(O)
            
            NObs = size(O,2)/6;
            
            buffer_stencil = [P.obstacle_buffer(1,[1 2 2 1 1]);...
                P.obstacle_buffer(2,[1 1 2 2 1])];
            
            buffer_stencil = repmat([buffer_stencil,NaN(2,1)],[1 NObs]);
            
            O = O+buffer_stencil;
            P.current_obstacles = O;
            
        else
            
            NObs=0;
            P.current_obstacles = [] ;
            
        end

        % get waypoints at end of arc for current lane
              
        wp = P.HLP.get_waypoint(agent_info,world_info,P.lookahead_distance) ;      

        P.current_waypoint = wp ;
        t_start = tic ;
        

        
        P.vdisp('Initializing new tree.',4)
        
        % initialize vertices; each node is: [state ; time ; distance ; ctrlinput]
        Vthr = [[P.agent_state; 0; Inf; 0 ] , nan(length(P.agent_state(:))+3, P.iter_max - 1)] ;
        NVthr = 1 ; % node (aka vertex) count
        Uthr = zeros(2,P.iter_max) ; % keep track of control inputs (these are the edges of the tree)
        Zcelthr = cell([1,P.iter_max]);
        Zcelthr{1,1} = P.agent_state;
        
        if any(isnan(agent_info.input(:,end)))
            Uthr(:,1) = agent_info.input(:,end-1);
        else
            Uthr(:,1) = agent_info.input(:,end);
        end
        Adjthr = sparse(P.iter_max,P.iter_max) ; % graph adjacency matrix
        %         end
        
    %% setup for new tree
        % set new braking tree
        P.vdisp('Setting up braking tree',4)
        NVbrk = 1 ;
        Vbrk = [[P.agent_state; 0; Inf; 0] , nan(length(P.agent_state(:))+3, P.iter_max - 1)] ;
        Ubrk = zeros(2,P.iter_max) ;
        Zcelbrk = cell([1,P.iter_max]);
        Zcelbrk{1,1} = P.agent_state;
        Adjbrk = sparse(P.iter_max,P.iter_max) ; % graph adjacency matrix

        % get timing
        timeout = P.timeout ;

        % run RRT
        P.vdisp('Running RRT', 2)

        icur = 1 ;

        % set up flags to check for feasible trajectories
        t_to_feas_thr = tic ;
        feas_flag_thr = false ;
        N_to_feas_thr = nan ;
        
        t_to_feas_brk = t_to_feas_thr ;
        feas_flag_brk = false ;
        N_to_feas_brk = nan ;
        
        t_timeout = toc(t_start) ;
        
        % create new vertex
        Tin = unique([0:P.dt_edge:P.T_edge,P.T_edge]) ;
        
        reached_dist=false;
    
    %% rrt construction loop
        while t_timeout <= timeout && icur < P.iter_max
        %% randomly pick old vertex
            % choose random vertex for each tree
            if icur<50
                v_thr_idx=randi(NVthr);
            elseif reached_dist
                %v_thr_idx = round(randRange(1,NVthr,2*NVthr/4,NVthr/2)) ;
                v_thr_idx=randi(NVthr);
                reached_dist=false;
            else
                v_thr_idx = round(randRange(1,NVthr,3*NVthr/4,NVthr/2)) ;
            end
            v_brk_idx = round(randRange(1,NVbrk,3*NVbrk/4,NVbrk/2)) ;

            % get throttle input states
            x_near_thr  = Vthr(1,v_thr_idx) ;
            y_near_thr  = Vthr(2,v_thr_idx) ;
            h_near_thr  = Vthr(3,v_thr_idx) ;
            vx_near_thr = Vthr(4,v_thr_idx) ;
            z_near_thr  = Vthr(1:5,v_thr_idx) ;           
            delta_des_near_thr = Uthr(2,v_thr_idx);
            v_des_near_thr = Uthr(1,v_thr_idx);
            t_near_thr = Vthr(end-2,v_thr_idx) ;
            Ccum_near_thr = Vthr(end,v_thr_idx) ;
            
            % get brake input states
            x_near_brk = Vbrk(1,v_brk_idx) ;
            y_near_brk = Vbrk(2,v_brk_idx) ;
            h_near_brk = Vbrk(3,v_brk_idx) ;
            vx_near_brk = Vbrk(4,v_brk_idx) ;
            z_near_brk = Vbrk(1:5,v_brk_idx) ;           
            delta_des_near_brk = Ubrk(2,v_brk_idx);
            t_near_brk = Vbrk(end-2,v_brk_idx) ;
            Ccum_near_brk = Vbrk(end,v_brk_idx) ;

        %% generate random inputs
            % generate steering input to turn towards local waypoint for
            % throttle tree
            v_in = randRange(P.min_speed,P.max_speed);
            
            dupper_thr =  P.max_wheelangle*(P.upper_boundary(2,1)-y_near_thr)./P.upper_boundary(2,1);
            dlower_thr = -P.max_wheelangle*(P.lower_boundary(2,1)-y_near_thr)./P.lower_boundary(2,1);
            
            dupper_hdg_thr = atan(((P.max_heading-h_near_thr)/P.T_edge)*agent_info.wheelbase/v_in);
            dlower_hdg_thr = atan(((-P.max_heading-h_near_thr)/P.T_edge)*agent_info.wheelbase/v_in);
            
            wa_in_thr = randRange(max([-P.max_wheelangle,dlower_thr,dlower_hdg_thr]),min([P.max_wheelangle,dupper_thr,dupper_hdg_thr]));
            
            dupper_brk =  P.max_wheelangle*(P.upper_boundary(2,1)-y_near_brk)/P.upper_boundary(2,1);
            dlower_brk = -P.max_wheelangle*(P.lower_boundary(2,1)-y_near_brk)/P.lower_boundary(2,1);
            
            dupper_hdg_brk = atan(((P.max_heading-h_near_thr)/P.T_edge)*agent_info.wheelbase/vx_near_brk);
            dlower_hdg_brk = atan(((-P.max_heading-h_near_thr)/P.T_edge)*agent_info.wheelbase/vx_near_brk);
            
            wa_in_brk = randRange(max([-P.max_wheelangle,dlower_brk,dlower_hdg_brk]),min([P.max_wheelangle,dupper_brk,dupper_hdg_brk]));
            
            Uin_thr = [v_in; wa_in_thr];
            Uin_brk = [0; wa_in_brk];

        %% generate new node
            Z_new_thr = NaN(5,length(Tin));
            Z_new_thr(:,1) = z_near_thr;
            
            Z_new_brk = NaN(5,length(Tin));
            Z_new_brk(:,1) = z_near_brk;
            
            % Forward Euler integrate random inputs
            for i = 2:length(Tin)
                tidx = Tin(i) ;
                Z_new_thr(:,i) = P.dt_edge*P.rrt_dynamics(tidx,Z_new_thr(:,i-1),Uin_thr) + Z_new_thr(:,i-1);
                Z_new_brk(:,i) = P.dt_edge*P.rrt_dynamics(tidx,Z_new_brk(:,i-1),Uin_brk) + Z_new_brk(:,i-1);
            end


            z_new_thr = Z_new_thr(:,end) ;
            z_new_brk = Z_new_brk(:,end) ;

            x_new_thr = z_new_thr(1) ; y_new_thr = z_new_thr(2) ;
            x_new_brk = z_new_brk(1) ; y_new_brk = z_new_brk(2) ;

        %% check if new node is feasible
            traj_feasible_thr = true ;
            traj_feasible_brk = true ;
            
            % check if new vertex is inside world bounds
            bound_check_thr = all(y_new_thr >= P.lower_boundary(2,:)) && all(y_new_thr <= P.upper_boundary(2,:));
            bound_check_brk = all(y_new_brk >= P.lower_boundary(2,:)) && all(y_new_brk <= P.upper_boundary(2,:));
            
            traj_feasible_thr = traj_feasible_thr && bound_check_thr ;
            traj_feasible_brk = traj_feasible_brk && bound_check_brk ;
            
            if ~isempty(O)
                % check if new edge intersects obstacle
                if traj_feasible_thr
                    [in_thr,~] = inpolygon(Z_new_thr(1,:)',Z_new_thr(2,:)',O(1,:)',O(2,:)') ;
                    traj_feasible_thr = traj_feasible_thr && ~any(in_thr) ;
                end
                
                if traj_feasible_brk
                    [in_brk,~] = inpolygon(Z_new_brk(1,:)',Z_new_brk(2,:)',O(1,:)',O(2,:)') ;
                    traj_feasible_brk = traj_feasible_brk && ~any(in_brk) ;
                end
            end

            % check if new nodes are within max dist of current car
            % location
            node_dist_thr = dist_point_to_points(Z_new_thr(1:2,end),P.agent_state(agent_info.position_indices)) ;
            node_dist_brk = dist_point_to_points(Z_new_brk(1:2,end),P.agent_state(agent_info.position_indices)) ;
            
            if node_dist_thr>P.max_dist
                reached_dist=true;
            end
            
            traj_feasible_thr = traj_feasible_thr && (node_dist_thr <= P.max_dist) ;
            traj_feasible_brk = traj_feasible_brk && (node_dist_brk <= P.max_dist) ;
            
        %% update throttle and braking trees with new node
            % hardcoded weights on control inputs for now

            % update throttle tree
            if traj_feasible_thr
                % update index
                NVthr = NVthr + 1 ;

                % generate cost
                min_dist_to_obs = min(dist_point_to_points(z_new_thr(1:2),O)) ;
             
      
                dist_to_wp_x = (x_new_thr-wp(1))^2;
                dist_to_wp_y = (y_new_thr-wp(2))^2;
                
                Cnode_thr = 1/min_dist_to_obs ;
               
                
                % track control input
                
                Ccum_thr = P.T_edge*(P.control_weights'*((Uin_thr-[P.desired_speed;0]).^2) + P.y_weight*dist_to_wp_y + P.x_weight*dist_to_wp_x) + Ccum_near_thr  ;
                
                % add new node to tree
                t_new = t_near_thr + P.T_edge ;
                
                %if we havent reached the min time horizon jack up cost
                 if t_new < P.T_min
                    Cnode_thr = Cnode_thr +Inf;
                 end
                
                Vthr(:,NVthr) = [z_new_thr ; t_new ; Cnode_thr ; Ccum_thr ] ;
                Uthr(:,NVthr) = Uin_thr ;                    
                Zcelthr{NVthr} = Z_new_thr;
                
                % update adjacency matrix
                Adjthr(v_thr_idx,NVthr) = 1 ;
                
                % check if feasible trajectory is found
                if t_new > P.T_min && ~feas_flag_thr &&  NVthr >= P.node_min
                    t_to_feas_thr = toc(t_to_feas_thr) ;
                    N_to_feas_thr = NVthr ;
                    feas_flag_thr = true ;
                    
                    P.vdisp(['Time to feasible traj: ',num2str(t_to_feas_thr)],5) ;
                    P.vdisp(['Nodes on feasible traj: ',num2str(N_to_feas_thr)],5) ;
                end                
            end
            
            % update braking tree
            if traj_feasible_brk
                % update index
                NVbrk = NVbrk + 1 ;
                                    
                dist_to_wp = dist_point_to_points([x_near_brk;y_new_brk],wp) ;
                Cnode_brk= dist_to_wp ;
                
                % integrate control input squared
                Ccum_brk = 0;%ctrl_weights'*(Uin_brk.^2) + Cctrl_near_thr ;

                % add new node to tree
                Vbrk(:,NVbrk) = [z_new_brk ; t_near_brk + P.T_edge ; Cnode_brk ; Ccum_brk ] ;
                Ubrk(:,NVbrk) = Uin_brk ;                    
                Zcelbrk{NVbrk} = Z_new_brk;
                
                % update adjacency matrix
                Adjbrk(v_brk_idx,NVbrk) = 1 ;

                % check if braked to a stop
                 % add new node to tree
                t_new = t_near_brk + P.T_edge ;
                
                if  t_new > P.T_min && ~feas_flag_brk &&  NVbrk >= P.node_min
                    t_to_feas_brk = toc(t_to_feas_brk) ;
                    N_to_feas_brK = NVbrk ;
                    feas_flag_brk = true ;

                    P.vdisp(['Time to feasible braking traj: ',num2str(t_to_feas_brk)],5) ;
                    P.vdisp(['Nodes on feasible braking traj: ',num2str(N_to_feas_brK)],5) ;
                end
            end
            
            icur = icur + 1 ;
            t_timeout = toc(t_start) ;
        end

        P.vdisp(['Nodes on throttle tree: ',num2str(NVthr)],4) ;
        P.vdisp(['Nodes on braking tree: ',num2str(NVbrk)],4) ;
        P.current_nodes_thr = Vthr(:,1:NVthr) ;
        P.current_nodes_brk = Vbrk(:,1:NVbrk) ;
        
        %% use tree to construct output time/input/trajectory
        % decide whether to brake or not based on which trajectory has
        % nodes closest to the waypoint
        dists_to_wp_thr = dist_point_to_points(wp,Vthr(1:2,:)) ;
        dists_to_wp_brk = dist_point_to_points(wp,Vbrk(1:2,:)) ;
        min_dist_thr = min(dists_to_wp_thr) ;
        min_dist_brk = min(dists_to_wp_brk) ;
        
       

        % create output as throttle or braking traj
        I = P.info ;
        
        if feas_flag_thr 
            
            P.vdisp('RRT found feasible traj', 4) ;
            
            % extract lowest-cost path
            Cvals = sum(Vthr(end-1:end,:),1) ;
            [~,lowest_cost_idx] = min(Cvals) ;
            [~,tree_path,~] = graphshortestpath(Adjthr,1,lowest_cost_idx) ;

            % create output time
            T_old_tmp = Vthr(end-2,tree_path) ;
        
            % save new throttle tree as "old" for next iter
            P.T_old = T_old_tmp ;
            P.U_old = Uthr(:,tree_path) ;
            P.Z_old = Vthr(1:5,tree_path) ;
            
            % create output time, input, and traj
            Tout = zeros(1,(length(tree_path)-1)*(length(Tin)-1));
            Zout = NaN(5,length(Tout));
            Uout = NaN(2,length(Tout));
            start_i = 1;
            
            for i = 2:length(tree_path)
                end_i = start_i+length(Tin)-1;
                Tout(:,start_i:end_i) = Tin+P.T_edge*(i-2);
                Zout(:,start_i:end_i) = Zcelthr{tree_path(i)};
                Uout(:,start_i:end_i) = repmat(Uthr(:,tree_path(i)),[1,length(Tin)]);
                start_i = end_i;
            end
            
            P.current.t = 0;
            P.current.T = Tout;
            P.current.Z = Zout;
            P.current.U = Uout;
            
            % update planner info in case the "toc" never got called
            if isinteger(t_to_feas_thr)
                t_to_feas_thr = nan ;
            end
            
            I.throttle_or_brake = [I.throttle_or_brake, 't'] ;
            
        elseif feas_flag_brk
            P.vdisp('RRT found braking traj', 4)
            
            % extract lowest-cost path
            Cvals = sum(Vbrk(end-1:end,:),1) ;
            [~,lowest_cost_idx] = min(Cvals) ;
            [~,tree_path,~] = graphshortestpath(Adjbrk,1,lowest_cost_idx) ;
            
            % clear old tree for next iteration
            P.T_old = [] ;
            P.U_old = [] ;
            P.Z_old = [] ;
            
            % create output time, input, and traj
            Tout = zeros(1,(length(tree_path)-1)*(length(Tin)-1));
            Zout = NaN(5,length(Tout));
            Uout = NaN(2,length(Tout));
            start_i = 1;
            
            for i = 2:length(tree_path)
                end_i = start_i+length(Tin)-1;
                Tout(:,start_i:end_i) = Tin+P.T_edge*(i-2);
                Zout(:,start_i:end_i) = Zcelbrk{tree_path(i)};
                Uout(:,start_i:end_i) = repmat(Ubrk(:,tree_path(i)),[1,length(Tin)]);
                start_i = end_i;
            end
            
            P.current.t = 0;
            P.current.T = Tout;
            P.current.Z = Zout;
            P.current.U = Uout;
            
            if isinteger(t_to_feas_brk)
                t_to_feas_brk = nan ;
            end

            I.throttle_or_brake = [I.throttle_or_brake, 'b'] ;
        else
            P.vdisp('RRT unsuccessful and no braking traj found!', 4)
            P.braking_flag = true;

            Tout = [0,P.T_max];
            Zout = [[P.agent_state(1:3);0;0],[P.agent_state(1:3);0;0]];
            
            Uout = [0,0;agent_info.input(2,end-1),agent_info.input(2,end-1)];
            P.stop_flag = true;

            I.throttle_or_brake = [I.throttle_or_brake, 'B'] ;
            
        end     

        if ~isempty(Zout)
            P.current_plan = Zout(agent_info.position_indices,:) ;
        else
            P.current_plan = [] ;
        end

        % match replan function to default format if mode is not needed
        if nargout < 4
            mode = [] ;
        end
        
        I.time_to_feasible_sol = [I.time_to_feasible_sol, [t_to_feas_thr;t_to_feas_brk]] ;
        I.N_nodes_for_feasible_sol = [I.N_nodes_for_feasible_sol, N_to_feas_brk] ;
        I.N_nodes_successful = [I.N_nodes_successful, [NVthr; NVbrk]] ;
        I.N_nodes_attempted = [I.N_nodes_attempted, icur] ;
        P.info = I ;
    end
    
    %% plotting
    function plotInLoop(P,n,c)
        figure(n)
        
        xlim(P.z0(1)+[-P.plot_radius,P.plot_radius])
        ylim(P.z0(2)+[-P.plot_radius,P.plot_radius])
        
        if nargin < 3
            c = [0 0 1] ;
        end
        
        % current nodes
        P.vdisp('Plotting current nodes!',4)
        Vthr = P.current_nodes_thr ;
        
        if ~isempty(Vthr) && P.plot_current_nodes 
            Cthr = sum(Vthr(end-1:end,2:end),1) ;
            Cmax = max(Cthr) ;
            Cthr = Cthr./Cmax ;
            Cthr = [1, Cthr] ;
            for idx = 1:size(Vthr,2)
                cidx = Cthr(idx).*[1 1 1] ;
  
                plot(Vthr(1,idx),Vthr(2,idx),'.','Color',cidx) ;
            end
        end
        
        Vbrk = P.current_nodes_brk ;
        if ~isempty(Vbrk) && P.plot_current_nodes
           plot(Vbrk(1,:),Vbrk(2,:),'.','Color',[1 0.2 0.2])
        end
                
        plotInLoop@planner2D(P,n,c)
        
        % waypoint finder
        if ~isempty( P.current_waypoint)   
            plot(P.current_waypoint(1),P.current_waypoint(2),'ro')
        end
        
        % obstacles
        O = P.current_obstacles ;
        if ~isempty(O)
            plot(O(1,:),O(2,:),'-','Color',[1 0.5 0.5]) ;
        end
    end
    end
end


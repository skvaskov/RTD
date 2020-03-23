classdef rover_RTD_planner < generic_RTD_planner
    properties
        point_spacing
        arc_point_spacing
        
        w_polynomial_info
        
        current_obstacles_in_FRS_coords
        
        bounds_as_obstacle
    end
    methods
        %% constructor and setup
        function P = rover_RTD_planner(varargin)
            % set default parameters
            t_plan = 0.5 ;
            t_move = 0.5 ;
            lookahead_distance = 5 ;
            
            % parse arguments
            P@generic_RTD_planner('lookahead_distance',lookahead_distance,...
                't_plan',t_plan,'t_move',t_move,varargin{:})
        end
    
        
        function setup(P,agent_info,world_info)
            % call superclass setup
            setup@generic_RTD_planner(P,agent_info,world_info)
            
            % make sure buffer is small enough, i.e. b < \bar{b}
            bbar = agent_info.footprint(2)/2;
            b = P.buffer ;
            
            if b >= bbar
                P.buffer = bbar - 0.001 ;
                P.vdisp('Reducing buffer to be feasible!',2)
            end
            
            % set point spacings
            [r,a] = compute_point_spacings(agent_info.footprint,P.buffer) ;
            P.point_spacing = r ;
            P.arc_point_spacing = a ;
            
            % set FRS buffer
            P.FRS_buffer = P.buffer ;
            
            % set the world bounds with the new buffer
            P.bounds = world_info.bounds + P.FRS_buffer.*[1 -1 1 -1] ;
            
            % create world bounds as an obstacle; note this passes the
            % bounds in as a clockwise polyline, so everything outside of
            % the world bounds counts as inside the polyline if using
            % functions like inpolygon
            xlo = P.bounds(1) ; xhi = P.bounds(2) ;
            ylo = P.bounds(3) ; yhi = P.bounds(4) ;
            
            Bl = [xlo, xhi;
                ylo, ylo];
            
             Bu = [xlo, xhi;
                yhi, yhi];
            
            
            P.bounds_as_obstacle = [interpolate_polyline_with_spacing(Bu,P.point_spacing),NaN(2,1),...
                                    interpolate_polyline_with_spacing(Bl,P.point_spacing)];

        end
        
        %% online planning: process obstacles
        function determine_current_FRS(P,agent_info)
            
            P.current_FRS_index = NaN;
            v0 = agent_info.state(agent_info.velocity_index,end);
            h0 = agent_info.state(agent_info.heading_index,end);
            
            for i = 1:length(P.FRS)
                if P.FRS{i}.v0_min == 0
                    v0_min = -1;
                else
                    v0_min = P.FRS{i}.v0_min;
                end
                
                if P.FRS{i}.v0_max == 2
                    v0_max = 3;
                else
                    v0_max = P.FRS{i}.v0_max;
                end
                
                if P.FRS{i}.psi0_max == 0
                    psi0_max = 0;
                    psi0_min = -Inf;
                else
                    psi0_max = Inf;
                    psi0_min = 0;
                end
                
                if v0 >= v0_min && v0 <= v0_max && h0 >= psi0_min && h0 <= psi0_max
                    P.current_FRS_index = i;
                    break
                end
            end
          
        end
        
        function process_world_info(P,world_info,~)
            
            F = P.FRS{P.current_FRS_index};
            % process the w polynomial from the rover's FRS; this just
            % extracts the msspoly's powers and coefficients to speed up
            % evaluating the polynomial online
            w = F.w;
            k = F.k ;
            y = F.y;
            
            %shift to 0 [0,1];
            w_full = w - 1.0001 ; % this is so (x,k) \in FRS => w(x,k) > 0
            
            %set second parameter to negative heading (end trajectory plan
            %aligned with road
            psiend_k = (-P.agent_state(3)-F.psi_end_min)*2/(F.psi_end_max-F.psi_end_min)-1;
            psiend_k = bound_values(psiend_k,1);
            
     
            w = msubs(w_full,k(2),psiend_k);
            
            z = [F.x ;y];

            P.w_polynomial_info = decompose_w_polynomial(w,z,k([1,3])) ;
            
            % get obstacles; for this planner, we assume the obstacles are
            % handed in as a 2-by-N array of boxes separated by columns of
            % NaNs
            
            % extract obstacles
            O = world_info.obstacles ;
            
           
            % buffer and discretize the obstacles
            if ~isempty(O)
                O = buffer_box_obstacles(O,P.FRS_buffer,'a',P.arc_point_spacing) ;
                O = interpolate_polyline_with_spacing(O,P.point_spacing) ;
                
                %add boundary points
                O = [O,P.bounds_as_obstacle];
                
                % move obstacles into FRS coordinates for generating nonlinear
                % constraints with the w polynomial
                x0 = F.xoffset(1);
                y0 = F.xoffset(2);
                Dx = F.xscale(1);
                Dy = F.xscale(2);
               
                
                R = [cos(P.agent_state(3)), -sin(P.agent_state(3)) ;
                    sin(P.agent_state(3)), cos(P.agent_state(3))] ;
                
                O_center =  R'*(O-P.agent_state(1:2));
                
                %get rid of obstacles that are unreachable because they lie
                %outside of a polygon that contains the reachable set for
                %all trajecotry parameters
                reachable_set_poly = [-0.3, 0.2,   1.75,  3.5,  3.5 ,  1.75, 0.2,-0.3,-0.3;...
                      -0.2,-0.3,-1.5,-1.5,1.5  1.5,0.3,0.2,-0.2];
                  
                L = inpolygon(O_center(1,:)',O_center(2,:)',reachable_set_poly(1,:),reachable_set_poly(2,:)');
                O_center = O_center(:,L);
                
                O_FRS = (O_center+[x0;y0])./[Dx;Dy];


                % get rid of obstacle points that are definitely unreachable
                % because they lie outside of the unit circle in the FRS
                % coordinate frame (this is because of how we create the SOS
                % program to compute the FRS)
                O_FRS = crop_points_outside_region(0,0,O_FRS,0.9) ;
%                 O_mirrored_FRS = crop_points_outside_region(0,0,O_mirrored_FRS,0.9);
            else
                O_FRS = [] ;
            end
            
            % save buffered, discretized obstacles
            P.current_obstacles = O ;       
            
            % save the processed obstacles
            P.current_obstacles_in_FRS_coords = O_FRS ;
        end
        
        %% online planning: cost function
        function create_cost_function(P,agent_info,world_info,start_tic)
            
            P.create_waypoint(agent_info,world_info);
            
            %get frs
            F = P.FRS{P.current_FRS_index};
            
            % get agent state
            z = P.agent_state ;
            
            % rotate waypoint to body-fixed frame
            wp = P.current_waypoint ;
            wp_local = world_to_local(z(1:3),wp(1:2)) ;
            
            % create cost function
            w0_max = F.w0_des_max;
            w0_min = -w0_max;
            v_max = F.v_des_max;
            v_min = F.v_des_min;

            P.trajopt_problem.cost_function = @(k) cost_rover(P,k,-z(3),w0_max,w0_min,v_max,v_min,wp_local(1),wp_local(2),start_tic);
        end
        
        function [c, gc] = cost_rover(P,k,psi_end,w0_max,w0_min,v_max,v_min,x_des,y_des,start_tic)     
            
            c = rover_cost(k(1),psi_end,k(2),w0_max,w0_min,v_max,v_min,x_des,y_des,1,2);
            gc = rover_cost_grad(k(1),psi_end,k(2),w0_max,w0_min,v_max,v_min,x_des,y_des,1,2);
            
            if toc(start_tic) > P.t_plan
                error('Trajopt timed out!')
            end
        end
        
        %% online planning: constraint function
        function create_constraints(P,start_tic)
            % get the processed obstacles
    
            O = P.current_obstacles_in_FRS_coords ;
  
            
            F = P.FRS{P.current_FRS_index};
            
            if ~isempty(O)
                % remove NaNs
                O_log = isnan(O(1,:)) ;
                O = O(:,~O_log) ;

                % plug in to w polynomial to get list of constraints w(x,k) for
                % each x \in O
                w = P.w_polynomial_info ;

                wk = sub_z_into_w(w,O) ;
                wkcoef = wk.wkcoef ;
                wkpows = wk.wkpows ;

                % create Jacobian of the constraints
                [Jcoef, Jpows] = diff_wk_wrt_k(wk) ;
                N = wk.N ;

                % create constraint function
                P.trajopt_problem.nonlcon_function = @(k) P.nonlcon_rover(k,wkcoef,wkpows,Jcoef,Jpows,N,start_tic) ;
            else
                % if there are no obstacles then we don't need to consider
                % any constraints
                P.trajopt_problem.nonlcon_function = [] ;
            end
           
            
            w0_des_max  = F.w0_des_max;
            w0_des_min =  F.w0_des_min;
            
            v_des_max = F.v_des_max;
            v_des_min = F.v_des_min;
            diff_v = F.diff_v;
            v0 = P.agent_state(4);
            
            psi_end_max = 0.5;
            psi_end_min = -0.5;
            
            psi_end = -P.agent_state(3);
            
            lower_k3 = ((v0-diff_v)-v_des_min)*2/(v_des_max-v_des_min)-1;
            upper_k3 = ((v0+diff_v)-v_des_min)*2/(v_des_max-v_des_min)-1;
            
            upper_psi_end = -w0_des_max/psi_end_min*psi_end+w0_des_max;
            lower_psi_end = -w0_des_min/psi_end_max*psi_end+w0_des_min;
            
            upper_k1 = (upper_psi_end-w0_des_min)*2/(w0_des_max-w0_des_min)-1;
            lower_k1 = (lower_psi_end-w0_des_min)*2/(w0_des_max-w0_des_min)-1;
             
            % create the inequality constraints and problem bounds
            P.trajopt_problem.Aineq = [];
            
            P.trajopt_problem.bineq = [] ;

            P.trajopt_problem.k_bounds = [max([lower_k1,-1],[],'omitnan'),min([upper_k1,1],[],'omitnan');...
                    max([lower_k3,-1]),min([upper_k3,1]) ];
         
            
        end
        
      
        
        function [n, neq, gn, gneq] = nonlcon_rover(P,k,wkcoef,wkpows,Jcoef,Jpows,N,start_tic)
            % constraint is active when p(k) > 0
            n = eval_w(k,wkcoef,wkpows) ;
            neq = zeros(size(n)) ;
            gn = eval_J(k,Jcoef,Jpows,N)' ;
            gneq = zeros(size(gn)) ;
           
            if toc(start_tic) > P.t_plan
                error('Trajopt timed out!')
            end
        end
        %%
        function create_initial_guess(P,~)
            % P.create_initial_guess(start_tic)
            %
            % By default, set the initial guess for trajopt to the previous k_opt,
            % or else zeros. This is a useful one to overwrite in a subclass,
            % probably.
            
            k_prev = P.latest_plan.k_opt ;
            
            if isempty(k_prev) || any(isnan(k_prev))
                
                P.trajopt_problem.initial_guess = zeros(2,1) ;
                
            else
                P.trajopt_problem.initial_guess = k_prev ;
            end
        end
  
        %% online planning: create output given successful replan
        function [T_out,U_out,Z_out] = create_desired_trajectory(P,agent_info,k_opt)
            
            % get current FRS and k
            F = P.FRS{P.current_FRS_index} ;
            w0_des_max = F.w0_des_max;
            w0_des_min = F.w0_des_min;
            v_des_max = F.v_des_max;
            v_des_min = F.v_des_min;
            
            psi_end = bound_values(-P.agent_state(3),-0.5,0.5);
            
            pose0 = P.agent_state([agent_info.position_indices,agent_info.heading_index]);
      
            % get the desired speed and yaw rate
            w0_des = (w0_des_max-w0_des_min)/2*(k_opt(1)+1)+w0_des_min;
            v_des = (v_des_max-v_des_min)/2*(k_opt(2)+1)+v_des_min;

            % create the desired trajectory
            t_stop =  2;
            [T_out,U_out,Z_out] = make_rover_braking_trajectory(P.t_move,F.t_f,...
                                    t_stop,w0_des,psi_end,v_des) ;
            
            %rotate and place at current position/heading
            Z_out(1:2,:) = rotation_matrix_2D(pose0(3))*Z_out(1:2,:)+pose0(1:2);
            Z_out(3,:) =   wrapToPi(Z_out(3,:)+pose0(3));
            
            %update current plan 
            P.current_plan = Z_out(1:2,:);
           
        end

        function plot(P,color)
            
           if nargin<2
               color = [0 0 1];
           end
           
           plot@generic_RTD_planner(P,color)
           
           
            if P.plot_FRS_flag
                
                
                if ~isnan(P.latest_plan.k_opt)
                    
                    k_opt = P.latest_plan.k_opt;
                    
                    
                    F = P.FRS{P.latest_plan.current_FRS_index};
                    k = F.k;
                    y = F.y;
                    
                    pose0 = P.latest_plan.agent_state(1:3);
                    
                    psiend_k2 = (-pose0(3)-F.psi_end_min)*2/(F.psi_end_max-F.psi_end_min)-1;
                    psiend_k2 = bound_values(psiend_k2,1);

                    
                    wx = subs(F.w,k,[k_opt(1);psiend_k2;k_opt(2)]);
                    
                    h = get_2D_msspoly_contour(wx,[F.x;y],1,'Scale',F.xscale,'Offset',-F.xoffset,'pose',pose0);
                    
                    
                    if ~check_if_plot_is_available(P,'FRS_contour')
                        P.plot_data.FRS_contour = line(h(1,:),h(2,:),'Color',[0 0.75 0.25],'LineWidth',1.0);
                    else
                        P.plot_data.FRS_contour.XData = h(1,:);
                        P.plot_data.FRS_contour.YData = h(2,:);
                    end
                else
                    
                     if check_if_plot_is_available(P,'FRS_contour')
                  
                        P.plot_data.FRS_contour.XData =  P.plot_data.FRS_contour.XData;
                        P.plot_data.FRS_contour.YData = P.plot_data.FRS_contour.YData;
                    end
                    
                end
                
            end
          
        end
       
       
    end
end

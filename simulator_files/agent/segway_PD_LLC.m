classdef segway_PD_LLC < low_level_controller
    properties
        % control gains
        x_position_gain = 20 ;
        y_position_gain = 20 ;
        heading_gain = 10 ;
        yaw_rate_gain = 20 ;
        speed_gain = 20 ;
        
        % timing
        lookahead_time = 0.02 ; % s (0.1 works very well)
    end
    
    methods
        function LLC = segway_PD_LLC(varargin)
            LLC@low_level_controller(varargin{:}) ;
        end
        
        function u = get_control_inputs(LLC,agent,t,z,varargin)
            % u = get_control_inputs(LLC,agent,t,z,T,U,Z)
            %
            % Use PD feedback to compute a desired yaw rate and speed for
            % the segway RTD agent. The output of this function is a 2-by-1
            % vector: u = [w_des ; v_des] 
            
            % extract reference trajectory time, input, and state
            T = varargin{1} ;
            U = varargin{2} ;
            Z = varargin{3} ;
            
            % get the time at which to do feedback
            t_fdbk = min(t + LLC.lookahead_time, T(end)) ;
            
            % get desired state and feedforward input
            z_des = match_trajectories(t_fdbk,T,Z) ;
            u_ff = match_trajectories(t_fdbk,T,U,'previous') ;
            
            % get states
            h = z(3) ;
            w = z(4) ;
            v = z(5) ;
            
            % get desired states
            h_des = z_des(3) ;
            w_des = z_des(4) ;
            v_des = z_des(5) ;
            
            % rotate current and desired position into zero-heading
            R_h = rotation_matrix_2D(-h) ;
            p_err = R_h*(z_des(1:2) - z(1:2)) ;
            
            % get control gains
            k_x = LLC.x_position_gain ;
            k_y = LLC.y_position_gain ;
            k_h = LLC.heading_gain ;
            k_v = LLC.speed_gain ;
            k_w = LLC.yaw_rate_gain ;
            
            % compute input
            u = [k_h*(h_des - h) + k_w*(w_des - w) + k_y*p_err(2) ;
                 k_v*(v_des - v) + k_x*p_err(1)] ;
             
            % add feedforward input
            u = u + u_ff ;
        end
    end
end
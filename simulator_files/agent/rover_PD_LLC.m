classdef rover_PD_LLC < low_level_controller
    properties
        % control gains
        yaw_gain = 2 ;
        yaw_rate_gain = 1 ;
    end
    
    methods
        %% constructor
        function LLC = rover_PD_LLC(varargin)
            n_agent_states = 5 ;
            n_agent_inputs = 2 ;
            
            LLC = parse_args(LLC,'n_agent_states',n_agent_states,...
                'n_agent_inputs',n_agent_inputs,varargin{:}) ;
        end
        
        %% get control inputs
        function U = get_control_inputs(LLC,A,t_cur,z_cur,T_des,U_des,Z_des)
            % get current state
            h_cur = z_cur(A.heading_index) ;
            v_cur = z_cur(A.speed_index) ;
            delta_cur = z_cur(5);
            
            % get desired state and inputs (assumes zero-order hold)
            if isempty(Z_des)
                % if no desired trajectory is passed in, then we assume
                % that we are emergency braking
                u_des = match_trajectories(t_cur,T_des,U_des,'previous') ;
                v_des = 0 ;
                h_des = h_cur ;
            else
                % otherwise, we are doing feedback about a desired
                % trajectory
                [u_des,z_des] = match_trajectories(t_cur,T_des,U_des,T_des,Z_des,'previous') ;
                v_des = u_des(1) ;
                h_des = z_des(A.heading_index) ;
            end
            
            %convert yawrate to wheelangle 
            w_des = v_des*tan(u_des(2))/(A.wheelbase+4.4e-7*v_des^2);

            
            
            k_h = LLC.yaw_gain ;
            k_w = LLC.yaw_rate_gain ;
            
            w_cur = tan(delta_cur)*v_cur/A.wheelbase;
            
            w_cmd = w_des+k_h*wrapToPi(h_des-h_cur)+k_w*(w_des-w_cur);
            
            if v_cur ~=0
                delta_des = atan(w_cmd*A.wheelbase/v_cur);
            else
                delta_des = A.max_wheelangle*sign(w_cmd);
            end

            % create output
            U = [v_des+0.05 ; delta_des] ;
        end
    end
end
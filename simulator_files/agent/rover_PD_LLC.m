classdef rover_PD_LLC < low_level_controller
    properties
        % control gains
        yaw_gain = 1 ;
        yaw_rate_gain = 1 ;
        lateral_position_gain = 1;
        longitudinal_position_gain = 1;
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
                % create output
                if numel(T_des) == 1
                    U  = u_des;
                else
                    U  = match_trajectories(t_cur,T_des,U_des,'previous');
                end
                
            else
                % otherwise, we are doing feedback about a desired
                % trajectory
                [u_des,z_des] = match_trajectories(t_cur,T_des,U_des,T_des,Z_des,'previous') ;
                
                
                
                h_des = z_des(A.heading_index) ;
                
                local_position = rotation_matrix_2D(h_des)'*(z_cur(A.position_indices)-z_des(A.position_indices));
                
                v_des = u_des(1)-LLC.longitudinal_position_gain*local_position(1) ;
                
                
                k_h = LLC.yaw_gain ;
                k_w = LLC.yaw_rate_gain ;
                k_y = LLC.lateral_position_gain;
                
                %convert yawrate to wheelangle
                w_des = A.wheelangle_to_yawrate(u_des(1),u_des(2)); 
                w_cur = A.wheelangle_to_yawrate(v_cur,delta_cur);
                
                w_cmd = w_des-k_h*wrapToPi(h_cur-h_des)-k_y*local_position(2)-k_w*(w_cur-w_des);
                
                if v_cur ~=0
                    delta_des = atan(w_cmd*A.wheelbase/v_cur);
                else
                    delta_des = 0;
                end
               
                U = [v_des;delta_des];
            end
           
        end
    end
end
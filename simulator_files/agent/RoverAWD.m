classdef RoverAWD < RTD_agent_2D
    properties
        max_speed = 2;
        min_speed = 0;
        max_wheelangle = 0.5;
        speed_index = 4;
        wheelbase = 0.3265;
        rear_axel_to_center_of_mass = 0.0765;
        
    end
        
     methods
        %% constructor
        function A = RoverAWD(varargin)
            % set up default superclass values
            name = 'RoverAWD' ;
            default_footprint = [0.5 0.29] ;
            n_states = 5 ;
            n_inputs = 2 ; % two reference actually
            stopping_time = 3; % conservative estimate
            sensor_radius = 30;
            LLC = rover_PD_LLC; % doesnt need a controller

            % create agent
            A@RTD_agent_2D('name',name,...
                'footprint',default_footprint,...
                'n_states',n_states,'n_inputs',n_inputs,...
                'stopping_time',stopping_time,'sensor_radius',sensor_radius,varargin{:},'LLC',LLC,varargin{:}) ;
        end
        
        %% functions to convert yawrate and velocty to wheel angle and vice-versa
        
        function yawrate = wheelangle_to_yawrate(A,v,wheelangle)
            l = A.wheelbase;
            yawrate = tan(wheelangle).*v./(l+4.4e-7*v.^2);
        end
        
        function wheelangle = yawrate_to_wheelangle(A,v,yawrate)
            l = A.wheelbase;
            wheelangle = atan(yawrate.*(l+4.4e-7*v.^2)./v);
            
            wheelangle(v==0) = sign(yawrate)*A.max_wheelangle;  
        end
        
        function vy = wheelangle_to_lateral_veloctity(A,v,wheelangle)
              w = A.wheelangle_to_yawrate(v,wheelangle);
              vy = w.*(A.rear_axel_to_center_of_mass-0.0140*v.^2);
        end
         %% dynamics
        function zd = dynamics(A,t,z,T,U,Z)
            % handle no desired trajectory input
            if nargin < 6
                Z = [] ;
            end
    
            
            % extract the states
            h = z(A.heading_index) ;
            v = z(A.speed_index) ;
            wheelangle = z(5);
            
            % get nominal control inputs
            u = A.LLC.get_control_inputs(A,t,z,T,U,Z) ;
            u(isnan(u)) = 0 ; % safety check
            v_des = u(1);
            delta_des = u(2);
            
            % saturate the inputs
            v_des = bound_values(v_des,A.min_speed,A.max_speed) ;
            delta_des = bound_values(delta_des,A.max_wheelangle) ;
                   
            % calculate the derivatives
            w = A.wheelangle_to_yawrate(v,wheelangle);
            vy = A.wheelangle_to_lateral_veloctity(v,wheelangle);
            
            if v>0
                cr = -0.0811;
            elseif v< 0
                cr = 0.0811;
            else
                cr = 0;
            end
            
            xd = v*cos(h)-vy*sin(h);
            yd = v*sin(h)+vy*cos(h);
            
            hd = w ;
            if v_des == 0 && abs(v) <0.1
              vd = -30*(v-v_des);
            elseif v_des == 0 && abs(v) < 0.5
              vd = -4*(v-v_des);
            else
               vd = cr-1.4736*(v-v_des)+0.1257*(v-v_des)^2; 
            end
            deltad = -5*(wheelangle-delta_des);
            % return state derivative
            zd = [xd ; yd ; hd ; vd;deltad] ;
            
        end
        
                %% get agent into
        function agent_info = get_agent_info(A)
            % call superclass method
            agent_info = get_agent_info@agent(A) ;
            
            % additional fields
            agent_info.input = A.input;
            agent_info.input_time = A.input_time;
            agent_info.heading_index = A.heading_index ;
            agent_info.velocity_index = 4;
            agent_info.yaw_rate = A.wheelangle_to_yawrate(A.state(4,:),A.state(5,:));
            agent_info.desired_time = A.desired_time ;
            agent_info.desired_input = A.desired_input ;
            agent_info.desired_trajectory = A.desired_trajectory ;
            agent_info.heading_index = A.heading_index ;
            agent_info.footprint = A.footprint ;
            agent_info.footprint_vertices = A.footprint_vertices ;
            agent_info.wheelbase = A.wheelbase;
            agent_info.rear_axel_to_center_of_mass = A.rear_axel_to_center_of_mass;
        end
     end
    
end
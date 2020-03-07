classdef RoverAWD < RTD_agent_2D
    properties
        max_speed = 2;
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
            default_footprint = [0.5 0.2] ;
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
        
         %% dynamics
        function zd = dynamics(A,t,z,T,U,Z)
            % handle no desired trajectory input
            if nargin < 6
                Z = [] ;
            end
         
            l = A.wheelbase;
            lr = A.rear_axel_to_center_of_mass;
            
            % extract the states
            h = z(A.heading_index) ;
            v = z(A.speed_index) ;
            delta = z(5);
            
            % get nominal control inputs
            u = A.LLC.get_control_inputs(A,t,z,T,U,Z) ;
            u(isnan(u)) = 0 ; % safety check
            v_des = u(1);
            delta_des = u(2);
            
            % saturate the inputs
            v_des = bound_values(v_des,A.max_speed) ;
            delta_des = bound_values(delta_des,A.max_wheelangle) ;
            
            
            
            % calculate the derivatives
            w = tan(delta)*v/(l+4.4e-7*v^2);
            vy = w*(lr-0.0140*v^2);
            
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
            vd = cr-1.4736*(v-v_des)+0.1257*(v-v_des)^2;
            deltad = -5*(delta-delta_des);
            % return state derivative
            zd = [xd ; yd ; hd ; vd;deltad] ;
        end
        
     end
    
end
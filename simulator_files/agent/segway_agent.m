classdef segway_agent < RTD_agent_2D
    properties
        % dynamics bounds
        max_speed = 1.5 ; % m/s
        max_accel = 3.75 % m/s^2
        max_yaw_rate = 1.0 ; % rad/s
        max_yaw_accel = 5.9 ; % rad/s^2 ;
        
        % state indices
        yaw_rate_index = 4 ;
        speed_index = 5 ;
        
        % gains from sys id
        accel_motor_gain = 3.0 ;
        yaw_accel_motor_gain = 2.95 ;
    end
    
    methods
        %% constructor
        function A = segway_agent(varargin)
            % set up default superclass values
            default_footprint = 0.38 ;
            n_states = 5 ;
            n_inputs = 2 ;
            stopping_time = 1.5 ;
            sensor_radius = 4 ;
            LLC = segway_PD_LLC() ;
            
            % create agent
            A@RTD_agent_2D('name','segway','footprint',default_footprint,...
                'n_states',n_states,'n_inputs',n_inputs,...
                'stopping_time',stopping_time,'sensor_radius',sensor_radius,...
                'LLC',LLC,varargin{:}) ;
        end
        
        %% get agent info
        function I = get_agent_info(A)
            I = get_agent_info@RTD_agent_2D(A) ;
            I.yaw_rate_index = A.yaw_rate_index ;
            I.speed_index = A.speed_index ;
            I.max_speed = A.max_speed ;
            I.max_accel = A.max_accel ;
            I.max_yaw_rate = A.max_yaw_rate ;
        end
        
        %% dynamics
        function zd = dynamics(A,t,z,T,U,Z)
            if nargin < 6
                Z = [] ;
            end
            
            % extract the states
            h = z(A.heading_index) ;
            w = z(A.yaw_rate_index) ;
            v = z(A.speed_index) ;
            
            % get inputs from low-level controller
            u = A.LLC.get_control_inputs(A,t,z,T,U,Z) ;
            w_des = u(1) ;
            v_des = u(2) ;
            
            % saturate commanded yaw rate and speed
            % v_des = bound_values(v_des,A.max_speed) ;
            % w_des = bound_values(w_des,A.max_yaw_rate) ;

            % compute accelerations produced by motors
            k_g = A.yaw_accel_motor_gain ;
            k_a = A.accel_motor_gain ;
            g = k_g*(w_des - w) ;
            a = k_a*(v_des - v) ;
            
            % saturate accelerations
            g = bound_values(g,A.max_yaw_accel) ;
            a = bound_values(a,A.max_accel) ;
            
            % output the dynamics
            xd = v*cos(h) ;
            yd = v*sin(h) ;
            hd = w ;
            wd = g ;
            vd = a ;

            zd = [xd ; yd ; hd ; wd ; vd] ;
        end
    end
end
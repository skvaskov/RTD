function zd = segway_symbolic_model(z,u)
% zd = segway_symbolic_model(z,u)
%
% This is the dynamics function used to generate symbolic trajectories for
% the segway.
%
% Author: Shreyas Kousik
% Created: a long time ago in a galaxy very very near
% Updated: 24 Mar 2020

	% get states
    h = z(3);
    w = z(4);
    v = z(5);
    
    % compute yaw acceleration
    w_des = u(1) ;
    K_g = 2.95 ;
    g = K_g*(w_des - w) ;

    % compute longitudinal acceleration
    v_des = u(2) ;
    K_a = 3 ;
    a = K_a*(v_des - v) ;

    % compute dynamics
    x_dot     = v .* cos(h) ;
    y_dot     = v .* sin(h) ;
    theta_dot = w ;
    omega_dot = g ;
    v_dot = a ;

    zd = [x_dot ;
        y_dot ;
        theta_dot ;
        omega_dot ;
        v_dot ] ;
end
function [T,U,Z] = make_segway_spin_trajectory(t_move,w_cur,delta_w,w_max)
% [T,U,Z] = make_segway_spin_trajectory(t_move,w_cur,delta_w,w_max)
%
% Generate a spin-in-place reference trajectory with initial position and
% heading of (x,y,h) = (0,0,0)
%
% Author: Shreyas Kousik
% Created: 16 Mar 2020
% Updated: nah

    % make time vector
    T = 0:0.1:t_move ;
    if T(end) < t_move
        T = [T, t_move] ;
    end
    N_T = length(T) ;
    
    % make feedforward input
    w_range = [w_cur - delta_w, w_cur + delta_w] ;
    w_range = bound_values(w_range, -w_max, w_max) ;
    
    % pick either the max or min allowable yaw rate
    u = [w_range(round(rand_range(1,2))) ; 0] ;
    U = repmat(u,1,N_T) ;

    
    % make desired trajectory
    dh = u(1)*t_move ;
    Z = [zeros(2,N_T) ;
        linspace(0,dh,N_T) ;
        U] ;
end
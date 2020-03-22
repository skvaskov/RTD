function [T,U,Z] = make_segway_spin_trajectory(t_move,w_des)
% [T,U,Z] = make_segway_spin_trajectory(t_move,w_des)
%
% Generate a spin-in-place reference trajectory with initial position and
% heading of (x,y,h) = (0,0,0)
%
% Author: Shreyas Kousik
% Created: 16 Mar 2020
% Updated: 19 Mar 2020

    % make time vector
    T = 0:0.1:t_move ;
    if T(end) < t_move
        T = [T, t_move] ;
    end
    N_T = length(T) ;
    
    % create the feedforward control input
    u = [w_des ; 0] ;
    U = repmat(u,1,N_T) ;
    
    % make desired trajectory
    dh = u(1)*t_move ;
    Z = [zeros(2,N_T) ;
        linspace(0,dh,N_T) ;
        U] ;
end
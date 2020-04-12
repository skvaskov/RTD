function [tout,yout,Rout] = ode1_with_SO3(dyn,tspan,y0,R0,dt,O_idxs)
%  [tout,yout,Rout] = ode1_with_SO3(dyn,tspan,y0,R0,dt,O_idxs)
%
% This function runs Euler integration with a rotation matrix as an
% additional state. I'll write how to use it more clearly later!
%
% Author: Shreyas Kousik
% Created: shrug
% Updated: 10 Apr 2020

    %% parse inputs
    if nargin < 5
        % take ten time steps by default
        dt = (tspan(end) - tspan(1))/10 ;
    end
        
    if nargin < 6
        % assume last three indices of y are the angular velocity (3-by-1
        % vector that can be mapped to so(3))
        N_states = length(y0) ;
        O_idxs = (N_states-2):N_states ;
    end
    
    %% setup
    % setup timing
    if length(tspan) > 2
        tout = tspan ;
    else
        tout = tspan(1):dt:tspan(end) ;
        if tout(end) < tspan(end)
            tout = [tout, tspan(end)] ;
        end
    end
    dt_vec = diff(tout) ;
    N_steps = length(tout) ;
    
    % allocate output
    yout = [y0(:), nan(length(y0),N_steps-1)] ;
    Rout = cat(3,R0,nan(3,3,N_steps-1)) ;
    
    %% run integration loop
    for idx = 2:N_steps
        % get time and time step
        t_idx = tout(idx) ;
        dt_idx = dt_vec(idx-1) ;
        
        % get current state and orientation
        y_idx = yout(:,idx-1) ;
        R_idx = Rout(:,:,idx-1) ;
        
        % compute state dynamics
        y_dot = dyn(t_idx,y_idx,R_idx) ;
        
        % compute rotation matrix dynamics
        O_idx = y_idx(O_idxs) ;
        F_idx = expm_SO3(dt_idx.*skew(O_idx)) ;
        
        % compute new state
        yout(:,idx) = y_idx + dt_idx.*y_dot ; % Euler step in state
        Rout(:,:,idx) = F_idx*R_idx ; % Euler step in rotation
    end
    
    %% fix output
    tout = tout(:) ;
    yout = yout' ;
end

%% helper functions
function S = skew(O)
    % compute the skew symetric matrix S from angular velocity O
    S = [ 0   -O(3)  O(2) ;
         O(3)   0   -O(1) ;
        -O(2)  O(1)   0   ];
end

function R = expm_SO3(w)
% R = expm_SO3(w)
%
% Computes the vectorized exponential map (at the identity) as defined in:
% http://ethaneade.com/lie.pdf
%
% This version is from: https://github.com/RossHartley/lie, modified to
% take in a skew-symmetric matrix.

    theta = norm(w);
    if theta == 0
        R = eye(3);
    else
        R = eye(3) + (sin(theta)/theta)*w + ((1-cos(theta))/(theta^2))*w^2;
    end
end

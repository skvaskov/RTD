function [tout,yout,Rout] = ode2_with_SO3(dyn,tspan,y0,R0,dt,O_idxs)
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
        half_dt_idx = dt_idx/2 ;
        
        % get current state and orientation
        y1_idx = yout(:,idx-1) ;
        R1_idx = Rout(:,:,idx-1) ;
        
    % compute state and rotation matrix dynamics with RK2
        % get first time step
        y1_dot = dyn(t_idx,y1_idx,R1_idx) ;
        k1 = half_dt_idx*y1_dot ;
        O1_idx = y1_idx(O_idxs) ;
        F1_idx = expm_SO3(half_dt_idx.*skew(O1_idx)) ;
        R2_idx = F1_idx*R1_idx ;

        % get second time step
        y2_dot = dyn(t_idx + half_dt_idx, y1_idx + k1, R2_idx) ;
        if ~any(isnan(y2_dot))
            k2 = dt_idx*y2_dot ;
            y2_idx = y1_idx + k1 ;
            O2_idx = y2_idx(O_idxs) ;
            F2_idx = expm_SO3(half_dt_idx.*skew(O1_idx + O2_idx)) ;
        else
            warning('NaNs detected in dynamics! Attempting Euler step.')
            k2 = dt_idx*y1_dot ;
            F2_idx = expm_SO3(dt_idx.*skew(O1_idx)) ;
        end

        % compute new state
        yout(:,idx) = y1_idx + k2 ;
        Rout(:,:,idx) = F2_idx*R1_idx ;
    end
end

%% helper functions
function S = skew(O)
    % compute the skew symetric matrix S from angular velocity O
    S = [ 0   -O(3)  O(2) ;
         O(3)   0   -O(1) ;
        -O(2)  O(1)   0   ];
end


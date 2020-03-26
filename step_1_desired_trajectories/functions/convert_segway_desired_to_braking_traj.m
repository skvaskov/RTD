function [T,U,Z] = convert_segway_desired_to_braking_traj(t_brk,T,U,Z)
% [T,U,Z] = convert_segway_desired_to_braking_traj(t_brk,T,U,Z)
%
% Given a segway trajectory without braking, make it brake to a stop for
% the entire distance from t_brk onwards
%
% Author: Shreyas Kousik
% Created: 24 Mar 2020
% Updated: - 

    % get all the time greater than t_brk
    T_log = T >= t_brk ;
    N = sum(T_log) ;

    % if the entire trajectory is too short, make the whole thing into a
    % braking trajectory
    if N < 2
        T_log = true(size(T)) ;
        N = length(T) ;
    end

    % create a linear scale to decrease the speed and yawrate
    s = linspace(0,1,N) ;
    s = repmat(s(end:-1:1),2,1) ;

    % multiply the velocity and yaw rate by s
    Z(4:5,T_log) = s.*Z(4:5,T_log) ;
    U(:,T_log) = s.*U(:,T_log) ;

    % get the distance traveled while braking
%     int_v = cumsum(Z(5,T_log).*[diff(T(T_log)),0]) ;
%     X = Z(1:2,T_log) ;
%     dist_X = dist_polyline_cumulative(X) ;
%     
%     % if less distance is traveled by the reduced-speed trajectory, shrink
%     % the distance traveled to match
%     if int_v(end) < dist_X(end)
%         X = match_trajectories(linspace(0,int_v(end),N),linspace(0,dist_X(end),N),X) ;
%     end
%     
%     % get the heading numerically from the new X trajectory
%     dX = diff(X,[],2) ;
%     h = atan2(dX(2,:),dX(1,:)) ;
%     N_h = length(h) ;
%     
%     Z(1:2,T_log) = X ;
%     Z(3,(end-N_h+1):end) = h ;
end
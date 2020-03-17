function P_out = local_to_world(robot_pose, P_local)
% P_out = local_to_world(robot_pose, P_local)
%
% Given a robot's pose (x,y,heading) in the world frame, and points in the
% local frame, shift and rotate these points to the world frame. The local
% points should either be passed in as a 2-by-N array of (x,y) points, or
% a 3-by-N array of (x,y,heading) points.
%
% Author: Sean Vaskov and Shreyas Kousik
% Created: 2016
% Updated: 29 Oct 2019
%
% See also: world_to_local, world_to_FRS, FRS_to_world

    % extract position and heading from input
    x = robot_pose(1,1) ;
    y = robot_pose(2,1) ;
    h = robot_pose(3,1) ;
    
    % prep
    P_out = P_local ;
    [N_rows,N_cols] = size(P_local) ;
    
    % rotate points to world frame direction
    R = [cos(h), -sin(h) ;
         sin(h),  cos(h) ] ;
    P_out(1:2,:) = R*P_out(1:2,:) ;
    
    if N_rows > 2
        P_out(3,:) = P_out(3,:) + h ;
    end
    
    % shift points to world frame location
    P_out(1:2,:) = P_out(1:2,:) + repmat([x;y],1,N_cols) ;
end
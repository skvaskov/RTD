function P_out = world_to_local(robot_pose, P_world)
% P_out = world_to_local(robot_pose, P_world)
%
% Given a robot's pose (x,y,heading) in the world frame, and points in the
% world frame, shift and rotate these points to the local frame defined by
% the robot's pose. The world points should either be passed in as a 2-by-N
% array of (x,y) points, or a 3-by-N array of (x,y,heading) points.
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
    P_out = P_world ;
    [N_rows, N_cols] = size(P_world) ;
    
    % shift all the world points to the position of the robot
    P_out(1:2,:) = P_world(1:2,:) - repmat([x;y],1,N_cols) ;
    
    % rotate the shifted world points to the robot's heading
    R = [ cos(h), sin(h) ;
         -sin(h), cos(h) ] ;
    
    P_out(1:2,:) = R*P_out(1:2,:) ;
    
    % if the input had a heading, rotate it
    if N_cols > 2
        P_out(3,:) = P_out(3,:) - h ;
    end
 end

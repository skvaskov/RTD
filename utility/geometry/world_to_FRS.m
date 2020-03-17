function P_FRS = world_to_FRS(P_world, pose, x0_FRS, y0_FRS, x_scale, y_scale)
% P_FRS = world_to_FRS(P_world, pose, x0_FRS, y0_FRS, x_scale, y_scale)
%
% Given a position and heading of the robot in the world frame, positions
% in the world as xy points, and an offset of the robot's FRS computation
% frame (x0_FRS, y0_FRS), transform the points from the world frame to the
% shifted FRS frame and scale them down by the distance D
%
% INPUTS
%   P_world     (x,y) points in world frame as a 2-by-N
%   pose        robot pose (x,y,heading)
%   x0_FRS      x position of robot in the FRS frame
%   y0_FRS      y position of robot in the FRS frame
%   x_scale     scaling in the x dimension
%   y_scale     scaling in the y dimension (equal to x_scale by default)
%
% OUTPUTS:
%   P_FRS       points in robot's local frame

    if ~exist('y_scale','var')
        y_scale = x_scale ;
    end
    
    % extract position and heading from input
    x = pose(1,1) ;
    y = pose(2,1) ;
    h = pose(3,1) ;
    
    % get the number of world points
    [N_rows, N_cols] = size(P_world);
    
    % shift all the world points to the position of the robot
    I_mat = ones(N_rows,N_cols) ;
    
    world_offset = [x, 0 ;
                    0, y] ;
      
    P_FRS = (P_world - world_offset*I_mat) ;
    
    % rotate the world points about the robot
    R = [cos(h), sin(h) ;
         -sin(h), cos(h)] ;
     
    P_FRS = R*P_FRS ;
    
    % scale the world points to the FRS scale
    P_FRS = [(1/x_scale)*P_FRS(1,:) ;
             (1/y_scale)*P_FRS(2,:)] ;
    
    % create the final shifted and scaled version of the obstacle points
    FRS_offset = [x0_FRS, 0 ; 0, y0_FRS] ;
    
    P_FRS = FRS_offset*I_mat + P_FRS ;
 end

function P_world = FRS_to_world(P_FRS, pose, x0_FRS, y0_FRS, x_scale, y_scale)
% P_world = FRS_to_world(P_FRS, pose, x0_FRS, y0_FRS, x_scale, y_scale)
%
% Given a position and heading of the robot in the world frame, points
% in the FRS frame as xy points, and an offset of the robot's FRS frame
% (x0_FRS, y0_FRS), transform the points from the FRS frame to the world
% frame.
%
% INPUTS
%   P_FRS       (x,y) points in FRS frame as a 2-by-N
%   pose        robot pose (x,y,heading)
%   x0_FRS      x position of robot in the FRS frame
%   y0_FRS      y position of robot in the FRS frame
%   x_scale     scaling in the x dimension
%   y_scale     scaling in the y dimension (equal to x_scale by default)
%
% OUTPUTS:
%   P_world     points in robot's global frame

    if ~exist('y_scale','var')
        y_scale = x_scale ;
    end
    
    % extract position and heading from input
    x = pose(1,1) ;
    y = pose(2,1) ;
    h = pose(3,1) ;
    
    % get the number of FRS points
    [N_rows, N_cols] = size(P_FRS);
    
    % shift all the FRS frame points by the FRS frame offset
    I_mat = ones(N_rows,N_cols) ;
    
    FRS_offset = [x0_FRS, 0 ;
                  0, y0_FRS] ;
      
    P_world = (P_FRS - FRS_offset*I_mat) ;
    
    % unscale the FRS frame points
    P_world = [x_scale*P_world(1,:) ; 
               y_scale*P_world(2,:)] ;
    
    % rotate the FRS points to the robot's heading
    R = [cos(h), -sin(h) ;
         sin(h),  cos(h)] ;
     
    P_world = R*P_world ;
    
    % create the final shifted and scaled version of the obstacle points
    world_offset = [x, 0 ; 0, y] ;
    
    P_world = world_offset*I_mat + P_world ;
 end

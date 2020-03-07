function [F,V] = make_cylinder_for_patch(radius,height,N_sides,use_triangles_flag,cap_ends_flag)
% [F,V] = make_cylinder_for_patch(radius,height,N,...
%                                 use_triangles_flag,...
%                                 cap_ends_flag)
%
% Generate face and vertices array for a cylinder of the given radius and
% height; the circular cross-section of the cylinder is approximated as a
% polygon with N sides. When the use_triangles_flag is true, the output
% has faces composed of triangles (i.e., F has 3 columns); the default uses
% rectangles, so F has 4 columns. If the cap_ends_flag is set to true, then
% the faces are triangles by default, and additional faces are added to cap
% off the ends of the cylinder.

    if nargin < 3
        N_sides = 10 ;
    end
    
    if nargin < 4
        use_triangles_flag = false ;
    end
    
    if nargin < 5
        cap_ends_flag = false ;
    end
    
    % create points
    [x,y,z] = cylinder([radius, radius],N_sides) ;
    
    % make z the height and center the cylinder at the origin
    z(1,:) = -height/2 ;
    z(2,:) =  height/2 ;
    
    % make patch
    if use_triangles_flag
        [F,V] = surf2patch(x,y,z,'triangles') ;
    else
        [F,V] = surf2patch(x,y,z) ;
    end
    
    % cap ends if necessary
    if cap_ends_flag
        F = convhull(V) ;
    end
end
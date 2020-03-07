function [F,V] = make_cone_for_patch(height,radius,N_base)
% [F,V] = make_cone_for_patch(height,radius)
% [F,V] = make_cone_for_patch(height,radius,N_base)
%
% Make faces and vertices of a cone to be used with patch. The optional
% N_base argument specifies the number of points around the circular base
% used to approximate the cone (by default it is 10).

    if nargin < 3
        N_base = 10 ;
    end

    theta_vals = linspace(0,2*pi,N_base) ;
    x_vals = [radius*cos(theta_vals), 0, 0] ;
    y_vals = [radius*sin(theta_vals), 0, 0] ;
    z_vals = [zeros(1,length(theta_vals)), 0, height] ;
    
    % create vertices
    V = [x_vals ; y_vals ; z_vals]' ;
    
    % number of vertices
    NV = size(V,1) ;
    
    % create faces
    F =  [] ;
    for idx = 1:(NV-3)
        % for each vertex, create a triangle for the base and a triangle
        % for the side
        f_base = [idx, idx+1, NV-1, idx] ;
        f_side = [idx, idx+1, NV, idx] ;
        
        F = [F ; f_base ; f_side] ;
    end
end
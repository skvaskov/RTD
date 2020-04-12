function [F,V] = make_cuboid_for_patch(L,W,H,C)
% [F,V] = make_cuboid_for_patch([L,W,H])
% [F,V] = make_cuboid_for_patch(L,W,H)
% [F,V] = make_cuboid_for_patch(L,W,H,C)
%
% Generate 6 faces and 8 vertices of an axis-aligned cuboid (in 3-D) of
% length L in x, width W in y, and depth D in Z. An optional fourth
% argument defining the center of the cuboid can also be passed in.
%
% The output of this function can be passed directly into the patch
% function as faces and vertices, to plot a cuboid in 3-D.
%
% using https://www.mathworks.com/help/matlab/visualize/multifaceted-patches.html

    if nargin < 4
        C = zeros(3,1) ;
    end

    if nargin == 1
        dims = L ;
        L = dims(1) ;
        W = dims(2) ;
        H = dims(3) ;
    elseif nargin < 1
        L = 1 ;
        W = 1 ;
        H = 1 ;
    end

    % make vertices
    Vx = L.*[0 1 1 0 0 1 1 0]' - L/2 + C(1) ;
    Vy = W.*[0 0 1 1 0 0 1 1]' - W/2 + C(2) ;
    Vz = H.*[0 0 0 0 1 1 1 1]' - H/2 + C(3) ;
    V = [Vx Vy Vz] ;

    % make faces
    F = [1 2 6 5 ; 2 3 7 6 ; 3 4 8 7 ; 4 1 5 8 ; 1 2 3 4 ; 5 6 7 8 ] ;
end
function [out1,out2] = make_box(S,C)
% B = make_box(S)
% [F,B] = make_box(S)
% [F,B] = make_box(S,c)
%
% Return a 2-by-5 "box" of xy-coordinates that are CCW with side length
% given by S(1) (for a square) or S = [s1, s2] for a rectangle. The box can
% be centered at the (optional) second input, c.
%
% The second (optional) output is a vector F that contains the order of the
% vertices in B used to define a patch face: F = [1 2 3 4 1]. In this case,
% the box B is returned as a 2-by-4 list of vertices.
%
% Examples:
%   make_box() returns a square of side length 2
%
%   make_box(2) returns a box of side length 4
%
%   make_box([1 2]) returns a 1x2 m box
%
%   make_box(1,[0;1]) returns a 1x1 m box centered at (0,1)

    if nargin < 1
        L = 1 ; W = 1 ;
    elseif length(S) == 1
        L = S ; W = S ;
    else
        L = S(1) ; W = S(2) ;
    end
    
    if nargin ~= 2
        C = [0;0] ;
    end

    x = (L/2)*[-1 1 1 -1 -1] + C(1) ;
    y = (W/2)*[-1 -1 1 1 -1] + C(2) ;

    out1 = [x;y] ;

    if nargout > 1
        out2 = out1(:,1:4)' ;
        out1 = [1 2 3 4 1] ;
    end
end
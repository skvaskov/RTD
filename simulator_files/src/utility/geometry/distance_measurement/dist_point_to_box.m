function [d,Q] = dist_point_to_box(P,varargin)
% d = dist_point_to_box(p,B)
% d = dist_point_to_box(p,l,w,h,c)
% [d,q] = dist_point_to_box(...)
%
% Return the distance from a point p \in \R^3 to a box. The box can be
% specified as B = [xlo xhi ylo yhi zlo zhi], or as a length/width/height
% and center (l,w,h,c).
%
% The optional second output q is the closest point on the box to the point
% p in the Euclidean norm.
%
% The first input, p, can either be a 3-by-1 point, or a 3-by-N array of
% points. If an array is provided, then the outputs d and q have the same
% number of columns as the input p.

    if length(varargin) == 1
        B = varargin{1} ;
    else
        l = varargin{1} ;
        w = varargin{2} ;
        h = varargin{3} ;
        c = varargin{4} ;
        B = box_to_bounds(l,w,h,c) ;
    end

    % get bounds as lo and hi
    lo = B([1 3 5]) ;
    hi = B([2 4 6]) ;

    % set up q
    Q = P ;
    
    % do logical check of p inside box
    N = size(P,2) ;
    lo = repmat(lo(:),1,N) ;
    hi = repmat(hi(:),1,N) ;
    chk_lo = P >= lo ;
    chk_hi = P <= hi ;
    
    % for all p outside box, return the closest point on the box
    Q(~chk_lo) = lo(~chk_lo) ;
    Q(~chk_hi) = hi(~chk_hi) ;

    % get final distances
    d = vecnorm(Q - P) ;
end
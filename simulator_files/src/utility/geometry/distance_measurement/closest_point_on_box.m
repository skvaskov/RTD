function p_closest = closest_point_on_box(p,V)
% p_closest = closest_point_on_box(p,V)
%
% Given the vertices of an axis-aligned box V, and a point p, find the
% point on the box that is closest to p. The input V should be N-by-3, and
% p should be 3-by-1 or 1-by-3.

lb = min(V) ;
ub = max(V) ;

p = p(:)' ;

p_closest = p ;

p_closest(p < lb) = lb(p < lb) ;
p_closest(p > ub) = ub(p > ub) ;

p_closest = p_closest(:) ;
end
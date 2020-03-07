function out = check_point_in_bounds(p,B)
% out = check_point_in_bounds(p,B)
%
% Given bounds B = [xlo, xhi, ylo, yhi] (can also have zlo, zhi), check
% that the 2-D (or 3-D) point p lies within them.
%
% Author: Shreyas Kousik
% Created: 6 Nov 2019
    out = (p(1) >= B(1)) && (p(1) <= B(2)) && ...
        (p(2) >= B(3)) && (p(2) <= B(4)) ;

    if length(p) == 3
        out = bounds_check && (p(3) >= B(5)) && (p(3) <= B(6)) ;
    end
end
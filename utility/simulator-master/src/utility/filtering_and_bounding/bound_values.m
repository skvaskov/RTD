function vo = bound_values(vi,lo,hi)
% vo = bound_values(vi,lo,hi)
%
% Given a vector vi, a min, and a max, return the same vector with all
% values clipped to either the min or the max
%
% Examples:
%   bound_values(1,2) returns 1 \in [-2,2]
%
%   bound_values(-3,2) returns -2
%
%   bound_values(1,[],0) returns 0 \in (-Inf,0]
%
%   bound_values(Inf,0,[]) returns Inf
%
%   bound_values(1,[2,3]) returns 2

    if nargin == 2
        if length(lo) > 1
            hi = lo(2) ;
            lo = lo(1) ;
        else
            hi = abs(lo) ;
            lo = -abs(lo) ;
        end
    else
        if isempty(lo)
            lo = -Inf ;
        end

        if isempty(hi)
            hi = +Inf ;
        end
    end
    
    if hi < lo
        error('Upper bound must be less than lower bound.')
    end

    vo = vi ;
    vo(vi > hi) = hi ;
    vo(vi < lo) = lo ;
end
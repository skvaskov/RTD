function vo = saturate(vi,lo,hi)
% vo = saturate(vi,lo,hi)
%
% Given a vector vi, a min, and a max, return the same vector with all
% values clipped to either the min or the max
%
% Examples:
%   saturate(1,2) returns 1 \in [-2,2]
%
%   saturate(-3,2) returns -2
%
%   saturate(1,[],0) returns 0 \in (-Inf,0]
%
%   saturate(Inf,0,[]) returns Inf
%
%   saturate(1,[2,3]) returns 2

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
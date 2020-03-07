function A_out = bound_array_elementwise(A_in,lo,hi)
% B = bound_array_elementwise(A,A_lo,A_hi)
%
% Given an input array A, an array A_lo of lower bounds of each element of
% A, and an array A_hi of upper bounds of each element of A, return the
% array B that is the same size as A, but with the bounds applied.
%
% The inputs lo and hi can also be given as scalars, in which case they are
% applied to every element of A_in.
%
% Author: Shreyas Kousik
% Updated: 19 Nov 2019

    if nargin < 3
        hi = abs(lo) ;
        lo = -abs(lo) ;
    end

    [r,c] = size(A_in) ;
    [rlo,clo] = size(lo) ;
    [rhi,chi] = size(hi) ;
    
    if (rlo == 1) && (clo == 1)
        lo = lo.*ones(r,c) ;
    elseif (rlo ~= r) || (clo ~= c)
        error(['The lower bound input must be either a scalar, or ',...
            'the same size as the input array.'])
    end
    
    if (rhi == 1) && (chi == 1)
        hi = hi.*ones(r,c) ;
    elseif (rhi ~= r) || (chi ~= c)
        error(['The upper bound input must be either a scalar, or ',...
            'the same size as the input array.'])
    end
    
    A_out = min(max(A_in,lo),hi) ;
end
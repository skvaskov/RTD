function n = rand_range(lo,hi,m,s,r,c)
% n = rand_range(lo,hi)
% n = rand_range(lo,hi,m,s)
% n = rand_range(lo,hi,m,s,r,c)
% n = rand_range(lo,hi,[],[],r,c)
%
% Create random real numbers within a provided range
%
% INPUTS:
%   lo  lower bound of an interval
%   hi  upper bound of an interval
%   m   mean of a normal distribution (optional)
%   s   standard deviation of a normal distribution
%   r   number of rows to output
%   c   number of columns to output
%
% OUTPUTS:
%   n   a real number or matrix of real numbers between lo and hi
%
% If m and s are empty then n is uniformly distributed between lo and hi,
% whereas if they are not empty then n is normally distributed about m with
% standard deviation of s, and in the interval [lo, hi]
%
% USAGE:
%   n = rand_range(1,3) % a uniformly distributed number in [1,3]
%
%   n = rand_range(1,3,2,0.5) % a pseudonormally distributed number
%                             % in [1,3], with mean 2 and std 0.5
%
%   n = rand_range(1,3,[],[],5,5) % a 5x5 matrix of uniformly distributed
%                                 % numbers between 1 and 3
%
%   n = rand_range([1 2 3],[2 3 4]) % a 1x3 vector of unif. dist. numbers
%                                   % with the first between 1 and 2

    if nargin == 2
        n = (hi-lo).*rand(size(hi)) + lo ;
    elseif nargin == 4
        n = trandn((lo-m)/s,(hi-m)/s)*s + m ;
    elseif nargin == 6
        if isempty(m) && isempty(s)
            n = (hi-lo).*rand(r,c) + lo ;
        elseif ~isempty(m) && ~isempty(s)
            n = trandn(((lo-m)/s).*ones(r*c,1),((hi-m)/s).*ones(r*c,1))*s + m ;
            n = reshape(n,r,c) ;
        else
            error(['Either the mean and standard deviation must both be ',...
                   'empty, or must both be scalars. They are the third ',...
                   'and fourth arguments to this function.'])
        end
    else
        error(['Input must either be an interval (2 inputs); or ',...
               'an interval, a mean, and a standard deviation (4 inputs); ',...
               'or an interal, a mean/std, and a number of rows and ',...
               'columns (6 inputs).'])
    end
end
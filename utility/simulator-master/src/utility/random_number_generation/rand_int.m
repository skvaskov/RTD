function out = rand_int(varargin)
% random_integer = rand_int(lo,hi,mean,std,N_rows,N_cols)
%
% Return an N_rows-by-N_cols array of random integers between the values lo
% and hi. If the mean and std inputs are nonempty, then these integers will
% be (roughly) normally distributed. Alternatively, you can just pass in an
% n-by-m array for lo and for hi, in which case you'll get an n-by-m array
% of random integers for which the i-th entry is between lo(i) and hi(i),
% uniformly distributed.
%
% Note, this is basically just like rand_range, so see that function for
% more usage details.
%
% Author: Shreyas Kousik
% Created: 24 Mar 2020
% Updated: - 
%
% See also: rand_range
    out = round(rand_range(varargin{:})) ;
end
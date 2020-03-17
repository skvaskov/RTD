function q = dmsubs_sparse(p,x,v)
% function q = dmsubs_sparse(p,x,v)
%
% Data matrix substitution into a sparsely-represented polynomial object q,
% which is very similar to an msspoly but doesn't have all the extra
% methods
%
% INPUTS:
%   p  -  m-by-1 poly object
%   x  -  k-by-1 free msspoly 
%   v  -  k-by-n real double 
%
% OUTPUT:
%   q  -  m-by-n double
%
% DESCRIPTION: q(:,i) is the result of substituting v(:,i) for x in p
% p must be an msspoly in x alone.

%% NEED TO FINISH WRITING THIS FUNCTION OMG
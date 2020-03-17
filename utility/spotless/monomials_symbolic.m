function Z = monomials_symbolic(vars,degree)
% function Z = monomials_symbolic(p,degree)
%
% DESCRIPTION
%   Construct a list of monomials of MATLAB symbolic variables
%
% INPUTS
%   p: a vector of sym vars
%   degree: a vector of non-negative integers specifying the degrees of the
%      monomials to be included in Z
%
% OUTPUTS
%   Z: lz-by-1 list of monomials

nvar = length(vars) ;

% recursively construct the monomials vector (ugh)
if nvar == 0
    degmat = zeros(1,0) ;
elseif nvar == 1
    degmat = degree(:) ;
else
    % recursive call for nvar > 1
    degree = degree(:) ;
    maxd = max(degree) ;
    
    % compute all degree matrices for nvar-1 vars and degrees from 0 up to
    % maxd
    degmatcell = cell(maxd+1,1) ;
    for idx = 0:maxd
        degmatcell{idx+1} = monomials(nvar-1,idx) ;
    end
    
    % stack on degrees for last variable
    degmat = [] ;
    for idx = 1:length(degree)
        degidx = degree(idx) ;
        degmatidx = [] ;
        for idx2 = 0:degidx
            temp = degmatcell{degidx-idx2+1} ;
            temp = [temp, repmat(idx2, [size(temp,1),1])] ;
            degmatidx = [degmatidx; temp] ;
        end
        degmat = [degmat;degmatidx] ;
    end
end

% create output
if isempty(vars)
    Z = degmat ;
else
    Z = prod(vars'.^degmat,2) ;
end
end
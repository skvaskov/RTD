function q = msubs_sparse(p,xid,v)
% function q = msubs_sparse(p,x,v)
%
% For a polynomial p of variables in x (but not limited to x), sub in the
% points in v and return a sparse matrix representation q of the resulting
% polynomial
%
% INPUTS:
%   p   -  m-by-1 msspoly with non-negative powers
%   xid -  k-by-1 list of msspoly free variable IDs
%   v   -  k-by-n real double 
%
% OUTPUT:
%   q   -  m-by-n polynomial represented as sparse matrices
%
% DESCRIPTION: q(:,i) is the result of substituting v(:,i) for x in p

N = size(v,2); % number of points to evaluate

% find which vars in p to substitute in
match = msspoly.match_list(xid,p.var)';

% get the powers of p
pow = p.pow';

% set up the points to sub in
vo = [ones(1,N) ; v];

% extract the values to sub in
values = vo(match(:) + 1,1:N);

% raise the values to their corresponding powers in the polynomial
values = values.^repmat(abs(pow(:)),1,N);

% reshape the values 
values = reshape(values,size(p.var,2),[])';

% multiply the coefficients by the resulting values
coeff = repmat(p.coeff,N,1).*prod(values,2);

% construct the new power and variable matrices
pow = repmat(p.pow.*(match' == 0),N,1);
var = repmat(p.var.*(match' == 0),N,1);

% construct the new subscript matrices
psub = p.sub ;
i   = repmat(psub(:,1),N,1);
j   = repmat(1:N,size(p.coeff,1),1);
j   = j(:);

% construct the output structure, which contains all the information of an
% msspoly without calling the msspoly make_canonical function
pdim = p.dim ;
q.dim = [pdim(1), N] ;
q.sub = [i j] ;
q.var = var ;
q.pow = pow ;
q.coeff = coeff ;
end

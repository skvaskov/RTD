function J = diff_sparse(p,xid)
%  J = diff_sparse(p,xid)
%
%  p -- n-by-1 polynomial in a sparse representation
%  xid -  k-by-1 list of msspoly free variable IDs (must be all IDs in p)
%
%  J is the jacobian of p with respect to all of its variables, which
%  would be an n-by-m msspoly, but this function takes in a sparse
%  representation p and outputs a sparse representation of J

% first get a matrix to determine the subscripts corresponding to the
% variables of p (this is like msspoly.match_list)
xid_indices = 1:length(xid) ;

tic
pvar_size = size(p.var) ;
match = sparse(pvar_size(1),pvar_size(2)) ;
toc

tic
% using a for loop here since the memory is already allocated and this only
% needs to run around 2 times... probably this can be improved with a
% matrix operation though
for idx = xid_indices
    match = match + idx.*(p.var == xid(idx)).*idx ;
end
toc

tic
msk = match ~= 0 ;
toc

tic
ind = find(msk) ;
toc

[i,j] = ind2sub(size(p.var),find(msk));

% match(msk) -- which column to place result in
% i -- which entries to draw var/pow/coeff from
% j -- which power to decrement / multiply by coeff
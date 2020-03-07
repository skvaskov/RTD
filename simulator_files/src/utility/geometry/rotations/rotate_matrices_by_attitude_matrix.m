function M_out = rotate_matrices_by_attitude_matrix(M_in,R_in)
% M_out = rotate_matrices_by_attitude_matrix(M_in,R_in)
%
% Given a 3-by-P-by-N array of matrices M_in and an attitude matrix R_in,
% rotate all of the matrices M_in and return a 3-by-P-by-N matrix out; in
% particular, for each M in, we apply the similarity transform:
%
% M_out = R_in' * M_in * R_in ;
    
    [~,P,N] = size(M_in) ;
    M_cell = mat2cell(M_in,3,P,ones(1,N)) ;
    M_out = cell2mat(cellfun(@(M) R_in'*M*R_in, M_cell, 'UniformOutput',false)) ;
end
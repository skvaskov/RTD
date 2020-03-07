function R_out = transpose_attitudes(R_in)
% R_out = transpose_attitudes(R_in)
%
% Given a 3-by-3-by-N array of rotation (attitude) matrices, transpose each
% of them and return the corresponding orientation matrices.

    N = size(R_in,3) ;
    R_in = permute(mat2cell(R_in,3,3,ones(1,N)), [1 3 2]) ;
    R_out = cellfun(@(R) R', R_in,'UniformOutput',false) ;
    R_out = cell2mat(permute(R_out, [1 3 2])) ;
end
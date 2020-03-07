function v_out = rotate_points_by_attitude_matrices(R_in,v_in)
% v_out = rotate_points_by_attitude_matrices(R_in,v_in)
%
% Given a 3-by-3-by-N array of rotation matrices and a 3-by-N array of
% points, rotate each ith point by the ith matrix.

    N = size(R_in,3) ;
    R_in = permute(mat2cell(R_in,3,3,ones(1,N)), [1 3 2]) ;
    v_in = mat2cell(v_in,3,ones(1,N)) ;
    v_out = cell2mat(cellfun(@(F,v) F*v, R_in, v_in,'UniformOutput',false)) ;
end
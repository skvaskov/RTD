function R_out = transpose_attitude_matrices(R_in)
% R_out = transpose_attitude_matrices(R_in)
%
% Given a 3-by-3-by-N array of matrices R_in, return a 3-by-3-by-N array of
% the same matrices transposed.
%
% Author: Shreyas Kousik
% Created: 19 Nov 2019
% Updated: -
    R_cell = mat2cell(R_in,3,3,ones(1,size(R_in,3))) ;
    R_temp = cellfun(@(r) r',R_cell,'UniformOutput',false) ;
    R_out = cell2mat(R_temp) ;
end
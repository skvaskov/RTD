function R = axis_angle_to_rotation_matrix_3D(axang)
% R = axis_angle_to_rotation_matrix_3D(axang)
%
% Given a 1-by-4 vector (can be symbolic) where the first three elements
% are a rotation axis and the fourth element is a rotation angle, return
% the corresponding rotation matrix.
%
% See: https://en.wikipedia.org/wiki/Axis?angle_representation
    e = axang(1:3) ;
    e = e ./ norm(e) ;
    theta = axang(4) ;
    K = skew(e) ;
    R = eye(3) + sin(theta)*K + (1 - cos(theta))*(K^2) ;
end
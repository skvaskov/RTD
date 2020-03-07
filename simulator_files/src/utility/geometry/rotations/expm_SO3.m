function R = expm_SO3(w)
% R = expm_SO3(w)
%
% Computes the vectorized exponential map (at the identity) as defined in:
% http://ethaneade.com/lie.pdf
%
% This version is from: https://github.com/RossHartley/lie, modified to
% take in a skew-symmetric matrix.

theta = norm(w);
if theta == 0
    R = eye(3);
else
    R = eye(3) + (sin(theta)/theta)*w + ((1-cos(theta))/(theta^2))*w^2;
end

end
function v = unskew(S)
% v = unskew(S)
%
% Get the vector in R^3 corresponding to the skew symmetric matrix S \in
% so(3)
    v = [S(3,2); S(1,3); S(2,1)] ;
end
function S = skew(v)
% S = skew(v)
%
% Compute the skew symmetric matrix S from the vector v (this is
% commonly called the "hat map"
    S = [ 0   -v(3)  v(2) ;
         v(3)   0   -v(1) ;
        -v(2)  v(1)   0   ];
end
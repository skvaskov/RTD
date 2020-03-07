function d = dist_point_to_points(p,P)
% given a point p (m x 1) and points P (m x n),
% compute the distance from p to each of the points in P, and return it
% as a 1 x n distance vector (of Euclidean distances)
    if size(P,2) > 0
        d = sqrt(sum((abs(P - repmat(p,1,size(P,2)))).^2,1)) ;
    else
        d = Inf;
    end
end
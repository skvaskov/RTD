function [D,d] = dist_points_to_points(P1,P2)
    % [D,d] = dist_points_to_points(P1,P2)
    %
    % Given two sets of points P1 (p x m) and P2 (p x n), calculate the
    % distances from every point of P1 to every point of P2 and return it
    % as a matrix D (m x n) and as a vector (1 x mn)
    
    m = size(P1,2) ;
    n = size(P2,2) ;
    

    p = size(P1,1) ;
    
    P1 = repmat(P1,1,n) ;
    P2 = reshape(repmat(P2,m,1),p,[]) ;

    
    d = sqrt(sum((P1 - P2).^2,1)) ;
    D = reshape(d,m,n) ;
    if nargout < 1
        d = [] ;
    end
end
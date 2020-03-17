function [P,P_log] = crop_points_outside_region(x,y,P,L,method)
% [P,P_log] = crop_points_outside_region(x,y,P,L,method)
%
% Given a n-by-n set of points, a position (x,y), and a distance L, remove
% all points that are not within the [-L,L]^2 box, or not in the L-radius
% super-ellipse. The 5th input parameter can be either 'rect' or an even
% number specifying the degree of the super-ellipse

if isempty(P)
    P = [] ;
    P_log = [] ;
else
    P = P(1:2,:) ;
    
    if nargin < 5
        method = 'rect' ;
    end
    
    if strcmp(method,'rect')
        P_log = any(abs(P - repmat([x;y],1,size(P,2))) > L, 1) ;
    else % use a super-ellipse
        d = sqrt(sum((abs(P-repmat([x;y],1,size(P,2)))).^method,1)) ;
        P_log = d > L ;
    end
    
    P = P(:, ~P_log) ;
end

if nargout < 2
    P_log = [] ;
end
end

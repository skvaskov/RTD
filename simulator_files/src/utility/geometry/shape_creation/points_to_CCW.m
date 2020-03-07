function P_out = points_to_CCW(P_in,close_loop)
% Given a 2-by-n set of points where the first row is x coordinates and the
% second row is y coordinates, rearrange the points in counter-clockwise
% order. If the "close loop" flag is set to true

    if nargin < 2
        close_loop = false ;
    end
    
    % get x/y coords
    Px = P_in(1,:) ; Py = P_in(2,:) ;
    
    % get centroid
    cx = mean(Px) ; cy = mean(Py) ;
    
    % get angle and distance from centroid to each point
    ang = atan2(Py-cy,Px-cx) ;
    dst = dist_point_to_points([cx;cy],P_in) ;
    
    % sort by angle in [-pi,pi] then by distance
    [~,idx] = sortrows([ang(:) dst(:)]) ;
    
    % rearrange points and create output
    P_out = [Px(1,idx) ; Py(1,idx)] ;
    if close_loop
        P_out = [P_out, P_out(:,1)] ;
    end
end
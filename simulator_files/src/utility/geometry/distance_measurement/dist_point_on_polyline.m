function [d,pct,d_along] = dist_point_on_polyline(p,P)
% [d,pct,d_total] = dist_point_on_polyline(p,P)
%
% Given a point p (2-by-1) on a polyline P (2-by-n), return the distance of
% the point along the polyline, and (optionally) the percentage of the
% total distance and the distances along the polyline
%
% See also: dist_point_to_polyline, dist_point_to_points
%
% Author: Shreyas Kousik
% Created: shrug
% Updated: 23 Dec 2019

    [~,~,p,p_idx] = dist_point_to_polyline(p,P) ;
    pct = [] ;
    
    % get distances along the polyline
    P_a = P(:,1:end-1) ; Pb = P(:,2:end) ;
    dP = Pb - P_a ;
    dP_dists = sqrt(sum(dP.^2,1)) ;
    P_dists = [0, cumsum(dP_dists)] ;
    P_pcnts = P_dists./P_dists(end) ;

    % get distance up to closest index and add distance from the point at
    % the closest index to the point itself
    d0 = P_dists(p_idx) ;
    d1 = dist_points_to_points(p,P(:,p_idx)) ;
    d = d0 + d1 ;
    
    if nargout > 1
        p0 = P_pcnts(p_idx) ;    
        p1 = d1./P_dists(end) ;
        pct = p0 + p1 ;
        
        if nargout > 2
            d_along = P_dists ;
        end
    end
end
function [d_min,d_out,p_out,p_idx] = dist_point_to_polyline(p,P)
% [d_min,d_out,p_out,p_idx] = dist_point_to_polyline(p,P)
%
% Given a point p (2-by-1) and a polyline (2-by-m), return a 1-by-m vector
% of the closest distances from the point to the polyline and (optionally)
% the minimum distance, the closest point on the polyline to p, and the
% index of the line segment on which the closest point lies (if the index
% is m, then the closest point is the last point of the polyline P)
%
% The polyline is P consists of m-1 line segments, from the first to m-1
% points "Pa" to the second to mth points "Pb" so the output distances are
% to these m-1 line segments and to the last point of Pb.
%
% Note this also works if the inputs are 3D or higher dimensional -- the
% number of rows of P is the dimension of the polyline.
%
% Modified from: http://www.alecjacobson.com/weblog/?p=1486
%
% Author: Shreyas Kousik
% Created: who knows???
% Updated: 23 Dec 2019

    % get the dimension of N
    N = size(P,1) ;

    % get the start and endpoints of the polyline
    Pa = P(:,1:end-1) ;
    Pb = P(:,2:end) ;

    % get the squared euclidean distances along the polyline
    dP = Pb - Pa ;
    P2 = sum(dP.*dP,1) ;
    
    % get the vector from the point of interest to each starting point
    P2p = repmat(p,1,size(P,2)-1) - Pa ;

    % linear algebra magic
    t = sum(P2p.*dP,1)./P2 ;

    % figure out which line segment of the polyline is closest to p
    tlog = t > 0 & t < 1 ;

    % get the points closest to p along P
    if any(tlog)
        Pa(:,tlog) = Pa(:,tlog) + repmat(t(tlog),N,1).*dP(:,tlog) ;
        Pall = [Pa,P(:,end)] ;
    else
        Pall = P ;
    end
    
    % get the distance from p to the closest point on P
    d_out = dist_point_to_points(p,Pall) ;
    [d_min,p_idx] = min(d_out) ;
    
    % get the closest point on P
    p_out = Pall(:,p_idx) ;
end
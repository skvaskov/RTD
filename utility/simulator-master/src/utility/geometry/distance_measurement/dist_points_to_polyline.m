function d_out = dist_points_to_polyline(P_pts,P_polyline)
% d_out = dist_points_to_polyline(pts,P)
%
% Given points P_pts (2-by-N or 3-by-N) and a polyline as a 2-by-N or
% 3-by-N array, return a 1-by-N vector of the distance from each point to
% the polyline.

    [d,N] = size(P_pts) ;
    p_cell = mat2cell(P_pts,d,ones(1,N)) ;
    d_out = cellfun(@(p) dist_point_to_polyline(p,P_polyline),p_cell) ;
end
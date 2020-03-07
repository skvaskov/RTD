function [c,d] = dist_polyline_cumulative(p)
% [cumulative_dist,dist_diffs] = dist_polyline_cumulative(p)
%
% For an M-by-N polyline, where each column is a point along the polyline,
% return a 1-by-N vector of the cumulative distance along the polyline.

    d = diff(p,[],2) ;
    d = [0, sqrt(sum(d.*d,1))] ;
    c = cumsum(d) ;
    if nargout < 2
        d = [] ;
    end
end
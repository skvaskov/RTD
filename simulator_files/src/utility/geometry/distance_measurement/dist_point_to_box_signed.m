function [d,Q] = dist_point_to_box_signed(P,varargin)
% d = dist_point_to_box_signed(p,B)
% d = dist_point_to_box_signed(p,l,w,h,c)
% [d,q] = dist_point_to_box_signed(...)
%
% Return the signed distance from a point p \in \R^3 to a box. The box can
% be specified as B = [xlo xhi ylo yhi zlo zhi], or by length/width/height
% and center (l,w,h,c).
%
% If the point is inside the box, the signed distance is negative.
%
% The optional second output q is the closest point on the box to the point
% p in the Euclidean norm.
%
% The first input, p, can either be a 3-by-1 point, or a 3-by-N array of
% points. If an array is provided, then the outputs d and q have the same
% number of columns as the input p.
%
% See also: dist_point_to_box

    if length(varargin) == 1
        B = varargin{1} ;
    else
        l = varargin{1} ;
        w = varargin{2} ;
        h = varargin{3} ;
        c = varargin{4} ;
        B = box_to_bounds(l,w,h,c) ;
    end

    % get bounds as lo and hi
    lo = B([1 3 5]) ;
    hi = B([2 4 6]) ;

    % set up Q for points
    Q = P ;
    
    % do logical check of p inside box
    N = size(P,2) ;
    lo = repmat(lo(:),1,N) ;
    hi = repmat(hi(:),1,N) ;
    chk_lo_out = P < lo ;
    chk_hi_out = P > hi ;
    
    % for all P outside the box, get the closest point on the box boundary
    Q(chk_lo_out) = lo(chk_lo_out) ;
    Q(chk_hi_out) = hi(chk_hi_out) ;
    
    % for all P inside the box, get the closest point on the box boundary
    in_log = all(~chk_lo_out & ~chk_hi_out,1) ;
    
    % get the points inside the box
    Q_in = P(:,in_log) ;
    N_in = sum(in_log) ;
    
    % for each of those points, get the closest point on the box boundary
    lo_in = lo(:,in_log) ;
    hi_in = hi(:,in_log) ;
    
    % get the distance from the points inside the box to the box boundary
    DL = Q_in - lo_in ;
    DH = hi_in - Q_in ;
    DLH = [DL;DH] ;
    [~,DLH_idx] = min(DLH,[],1) ;
    
    % create a logical that tells whether which high or low boundary the
    % inside points are closest to
    DLH_log = repmat(DLH_idx,6,1) == repmat((1:6)',1,N_in) ;
    
    % create a logical array for which elements of the inside points to
    % swap out with the value of the nearest boundary
    Q_idx = DLH_idx ;
    Q_idx(Q_idx > 3) = Q_idx(Q_idx > 3) - 3 ;
    Q_log = repmat(Q_idx,3,1) == repmat((1:3)',1,N_in) ;
    
    % get the boundary in a format that can be indexed by DLH_log
    LH = [lo_in ; hi_in] ;
    
    % replace the inside points with the closest boundary points
    Q_in(Q_log) = LH(DLH_log) ;
    Q(:,in_log) = Q_in ;
    

    % get distances
    d = vecnorm(Q - P) ;
    
    % set distances of points inside to negative values
    d(in_log) = -d(in_log) ;
end
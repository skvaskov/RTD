function O = interpolate_polyline_with_spacing(P,d)
% O = interpolate_polyline_with_spacing(P,d)
%
% Given a polyline P and a desired minimum point spacing d, create a new
% polyline O that has additional points added to the segments that are
% longer than the distance d

    % check if polyline is closed
    if P(1,1) == P(1,end) && P(2,1) == P(2,end)
        polyline_closed = true ;
    else
        polyline_closed = false ;
    end

    % get the inter-vertex distances
    dP = diff(P,1,2) ;
    Pdists = sqrt(sum(dP.^2,1)) ;

    % element i of "dlog" indicates that the segment of P from vertex i to
    % vertex i+1 is too long
    dlog = Pdists > d ;

    % get the indices of the start of each too-short segment
    idxs = 1:size(dlog,2) ;
    idxs = idxs(dlog) ;

    O = [] ; % initialize the output
    for idx = 1:size(P,2)-1
        % if the index is one of the starting indices of a too-long segment,
        % then create new segments that are more densely spaced; otherwise,
        % append the current segment to O
        if any(idxs == idx)
            didx = Pdists(idx) ;
            Nidx = ceil(didx/d) + 1 ; % need the "+1" because we drop the
                                      % last point of Oidx, so this
                                      % guarantees that the minimum spacing
                                      % is d
            Oidx = [linspace(P(1,idx), P(1,idx+1), Nidx) ;
                    linspace(P(2,idx), P(2,idx+1), Nidx)] ;
        else
            Oidx = P(:,idx:idx+1) ;
        end
        O = [O, Oidx(:,1:end-1)] ;
    end
    
    % close O if needed
    if polyline_closed && ~(O(1,1) == O(1,end) && O(2,1) == O(2,end))
        O = [O, O(:,1)] ;
    end
end
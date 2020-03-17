function P_out = buffer_polygon_obstacles(P,b,miterlim)
% P_out = buffer_polygon_obstacles(P,b,miter_type)
%
% Given obstacles P as a 2-by-N array of (x,y) points, buffer them by a
% distance b and return a 2-by-(however many are needed) polygon array.
% Input either a miter limit or 'square' as the miter type, to be used by
% the default polybuffer function.
%
% Author: Shreyas Kousik
% Created: some time in 2019
% Updated: 30 Oct 2019

    % remove duplicate vertices
    P = unique(P','rows','stable') ;

    % create polyshape
    P = polyshape(P(:,1),P(:,2)) ;
    if nargin < 3
        P = polybuffer(P,b) ;
    elseif strcmp(miterlim,'square')
        P = polybuffer(P,b,'JointType','square');
    else
        P = polybuffer(P,b,'JointType','miter','MiterLimit',miterlim);
    end

    % get the vertices as the output
    [x,y] = boundary(P) ;
    P_out = [x y]' ;
end
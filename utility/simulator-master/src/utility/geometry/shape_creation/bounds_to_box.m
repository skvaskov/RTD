function varargout = bounds_to_box(B)
% Given bounds [xmin xmax ymin ymax zmin zmax], make a box given by
% length/width/height/center [l,w,h,c]
% 
% Usage:
%   [l,w,c] = bounds_to_box([xmin xmax ymin ymax])
%   [l,w,h,c] = bounds_to_box([xmin xmax ymin ymax zmin zmax])
%   B_polyline = bounds_to_box([xmin xmax ymin ymax])
%
% Author: Shreyas Kousik
% Created: Mar 2019
% Updated: Mar 24 2020
    switch length(B)
        case 4
            B = reshape(B(:),2,2)' ;
            c = mean(B,2) ;
            l = B(1,2) - B(1,1) ;
            w = B(2,2) - B(2,1) ;
            if nargout == 1
                varargout = {make_box([l w],c)} ;
            elseif nargout == 3
                varargout = {l,w,c} ;
            else
                error('Incorrect number of output arguments!')
            end
        case 6
            B = reshape(B(:),2,3)' ;
            c = mean(B,2) ;
            l = B(1,2) - B(1,1) ;
            w = B(2,2) - B(2,1) ;
            h = B(3,2) - B(3,1) ;
            varargout = {l,w,h,c} ;
        otherwise
            error(['The input should be either [xmin xmax ymin ymax] ',...
                'or [xmin xmax ymin ymax zmin zmax].'])
    end
end
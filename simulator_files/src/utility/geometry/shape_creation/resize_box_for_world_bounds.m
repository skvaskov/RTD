function [l,w,h,c] = resize_box_for_world_bounds(l,w,h,c,B)
    % given a box defined by length/width/height/center and world bounds B
    % given as [xmin xmax ymin ymax zmin zmax], trim the box and recenter
    % it to fit inside the world bounds
    b = box_to_bounds(l,w,h,c) ;
    lo = max([b([1 3 5]) ; B([1 3 5])],[],1) ;
    hi = min([b([2 4 6]) ; B([2 4 6])],[],1) ;
    [l,w,h,c] = bounds_to_box(reshape([lo;hi],1,6)) ;
end
function [l,w,h,c] = bounds_to_box(B)
    % given bounds [xmin xmax ymin ymax zmin zmax], make a box given by
    % length/width/height/center [l,w,h,c]
    B = reshape(B(:),2,3)' ;
    c = mean(B,2) ;
    l = B(1,2) - B(1,1) ;
    w = B(2,2) - B(2,1) ;
    h = B(3,2) - B(3,1) ;
end
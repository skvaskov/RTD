function B = box_to_bounds(l,w,h,c)
    % given a box length/width/height/center, make bounds [xmin xmax ymin
    % ymax zmin zmax]
    xmin = c(1) - l/2 ;
    xmax = c(1) + l/2 ;
    ymin = c(2) - w/2 ;
    ymax = c(2) + w/2 ;
    zmin = c(3) - h/2 ;
    zmax = c(3) + h/2 ;
    
   B = [xmin xmax ymin ymax zmin zmax] ;
end
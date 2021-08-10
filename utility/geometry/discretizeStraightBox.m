function P = discretizeStraightBox(corners,b,r)

minx = min(corners(1,:));
maxx = max(corners(1,:));
miny = min(corners(2,:));
maxy = max(corners(2,:));

length = (maxx-minx)+2*b;
width = (maxy-miny)+2*b;

n_length_pts = ceil(length/r)+1;

n_width_pts = ceil(width/r)+1;

length_pts = linspace(0,length,n_length_pts);

width_pts = linspace(0,width,n_width_pts);

P = [minx-b + length_pts,minx+b + length_pts,minx-b * ones(1,n_width_pts-2), maxx+b * ones(1,n_width_pts-2) ;...
     miny-b * ones(1,n_length_pts),maxy+b * ones(1,n_length_pts),miny-b+width_pts(2:end-1),miny-b+width_pts(2:end-1)];


end


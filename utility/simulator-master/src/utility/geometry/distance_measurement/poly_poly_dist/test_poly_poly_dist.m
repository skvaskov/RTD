function [hf, hp, d_min, xc_is, yc_is, idxc_is, is_vertex] = test_poly_poly_dist(xv1, yv1, xv2, yv2)
%test_poly_poly_dist Plot the results of a call to poly_poly_dist function
% Input arguments:
% xv1 - vector of X coordinates of the first polyline vertices (1 X nv1)
% yv1 - vector of Y coordinates of the first polyline vertices (1 X nv1)
% xv2 - vector of X coordinates of the second polyline vertices (1 X nv2)
% yv2 - vector of Y coordinates of the second polyline vertices (1 X nv2)
%
% Output arguments:
% hf - figure handle
% hp - plot handles
% d_min, xc_is, yc_is, idxc_is - poly_poly_dist output arguments
%
% Usage:
% [hf,hp,d_min,xc_is,yc_is,idxc_is,is_vertex]=test_poly_poly_dist(xv1,yv1,xv2,yv2);
% 
% Suggested test cases:
% 1. Intersecting polylines - find interesection points
% xv1 = [1 0 2 1.5];
% yv1 = [-3 -2 1 -1.5];
% xv2 = [-3 -1 0 1 2 2 0];
% yv2 = [-1 0 1 1 -1 -2 -3];
% 2. Non-interesecting polylines, 2 points with same minimum distance at
% vertices of each polyline
% xv1 = [1 0 0 2 1.5] + 3;
% yv1 = [-3 -2 -1 1 -1.5];
% xv2 = [-3 -1 0 1 2 2 0];
% yv2 = [-1 0 1 1 -1 -2 -3];
% 2. Non-interesecting polylines, 2 points with same minimum distance
% xv1 = [1 0 0 2 1.5] + 3;
% yv1 = [-3 -2 -1 1 -1.5] -0.5;
% xv2 = [-3 -1 0 1 2 2 0];
% yv2 = [-1 0 1 1 -1 -2 -3];
% 2. 2 single-segment polylines, poly1's vertex touches poly2 segment
% xv1 = [1 -2];
% yv1 = [-3 -0.5];
% xv2 = [-3 -1];
% yv2 = [-1 0];
% 5. 2 intersecting single-segment polylines
% xv1 = [1 -2];
% yv1 = [-3 0];
% xv2 = [-3 -1];
% yv2 = [-1 0];
% 6. Closed intersecting polylines
% npt = 20; theta = 0:2*pi/npt:2*pi; A = 1; B=0.5;
% xv1 = A*cos(theta); yv1 = B*sin(theta); xv2 = B*cos(theta); yv2 = A*sin(theta);
% 7. Closed non-intersecting polylines
% npt = 20; theta = 0:2*pi/npt:2*pi; A = 1; B=0.5;
% xv1 = A*cos(theta) + 2; yv1 = B*sin(theta) + 1.1; xv2 = B*cos(theta); yv2 = A*sin(theta);
%
% Revision history:
% Nov 3, 2015 - Created (Michael Yoshpe).
%**************************************************************************

nv2 = length(xv2);
nv1 = length(xv1);

hf = figure;
grid on;
axis equal;
hold on

% poly 1
hp(1) = plot(xv1, yv1, 'go-', 'Linewidth', 2);

% poly 2
hp(2) = plot(xv2, yv2, 'bo-', 'Linewidth', 2);

legend_str = {'poly1', 'poly2'};

% text offsets
off_x = 0.05;
off_y = 0.1;

for j=1:(nv2),
   ht=text(xv2(j)+off_x, yv2(j)+off_y, int2str(j));
end

for k=1:nv1,
   ht=text(xv1(k)+off_x, yv1(k)+off_y, int2str(k));
end

xlabel('X');
ylabel('Y');

[d_min, xc_is, yc_is, idxc_is, is_vertex] = poly_poly_dist(xv1, yv1, xv2, yv2);

% closest or intersection points
if(any(any(is_vertex)))
   nhp = length(hp);
   hp(nhp+1:nhp+2) = ...
      plot(xc_is(is_vertex), yc_is(is_vertex), 'ro', 'MarkerFaceColor', 'r');
   legend_str = [legend_str, 'c/is vertices'];
end

if(any(any(~is_vertex)))
   nhp = length(hp);   
   hp(nhp+1:nhp+2) = ...
      plot(xc_is(~is_vertex), yc_is(~is_vertex), 'mo', 'MarkerFaceColor', 'm');
   legend_str = [legend_str, 'c/is non-vertices'];
end

legend(unique(hp), legend_str, 'Location', 'EastOutside');

end


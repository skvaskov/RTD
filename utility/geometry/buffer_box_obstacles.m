function B_out = buffer_box_obstacles(B,b,varargin)
% B_out = buffer_box_obstacles(B,b,varargin)
%
% Given a 2-by-n matrix of 2-D box obstacles, buffer them by a distance b
% according to the type of buffer specified
%
% The user can specify the third argument as the number of points on
% the circle that is Minkowski-summed with the box obstacle.:
%
%   Bout = buffer_box_obstacles(B,b,N)
%
% Or, the user can specify that the third argument is a maximum distance
% between points on the circle, in which case N is found automatically:
%
%   Bout = buffer_box_obstacles(B,b,'a',a)
%
% where the input a is the distance.
    
    if nargin < 3
        N = 29 ;
    else
        if length(varargin) == 1
            N = varargin{1} ; % default fineness of angle discretization
        elseif length(varargin) == 2
            C = 2*pi*b ; % circumference of circle
            a = varargin{2} ;
            N = ceil(C / a) ;
        else
            error('incorrect number of input args!')
        end
    end
    
    % check beginning and end of B for nans
    if ~isnan(B(1,end))
        B = [B, nan(2,1)] ;
    end
    
    if isnan(B(1,1))
        B = B(:,2:end) ;
    end

    % get x and y coords
    Bx = B(1,:) ; By = B(2,:) ;

	% create circles to place at vertices
    Nx = 4 ; % number of circles to add to corners
    t = linspace(0,2*pi,N) ; % angle of circle
    xc = b*cos(t) ; yc = b*sin(t) ; % circle primitive
    XC = repmat(xc,Nx,1) ; % circle x coords to add to each corner
    YC = repmat(yc,Nx,1) ;

    % this is done with a for loop and no preallocation because the convex
    % hull will return something of uncertain length... luckily, this still
    % runs in just a few milliseconds for around 10 obstacles
    B_out = [] ;
    for idx = 1:6:size(B,2)-1
        X = repmat(Bx(idx:idx+3)',1,N) + XC ;
        Y = repmat(By(idx:idx+3)',1,N) + YC ;
        X = X(:) ; Y = Y(:) ;
        k = convhull(X,Y) ;
        Bnew = [X(k)' ; Y(k)'] ;
        
        % remove segments that are too small
        too_small = [false, sqrt(sum(diff(Bnew,1,2).^2,1)) < 1e-14] ;
        Bnew = Bnew(:,~too_small) ;
        
        % append to output
        B_out = [B_out, nan(2,1), Bnew] ;
    end
    
    % remove initial column of nans
    B_out = B_out(:,2:end) ;
end
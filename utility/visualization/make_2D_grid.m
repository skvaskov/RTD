function [X,Y] = make_2D_grid(N,B)
% X = make_2D_grid(N,B)
% [X,Y] = make_2D_grid(N,B)
%
% Given a discretization N and bounds B = [low,high], create a grid of x
% and y points; if one output is specified, then it is a 2-by-N^2 array
% where each column is one grid point. If two outputs are specified, then
% they are the same as the output of meshgrid.

    if nargin < 2
        B = [-1,1] ;
        if nargin < 1
            N = 100 ;
        end
    end
    
    v = linspace(B(1),B(2),N) ;
    
    [X,Y] = meshgrid(v,v) ;
    
    if nargout == 1
        X = [X(:), Y(:)]' ;
    end
end
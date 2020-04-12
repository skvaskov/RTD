function S = make_random_convex_polytope(N,c)
% Given a number of vertices N and a center c \in \R^3, return a structure
% S that has two fields (Faces and Vertices) that can be used as the input
% to the patch function.
%
% Author: Shreyas Kousik
% Created: 27 Dec 2019
% Updated: not yet
    if nargin < 1
        N = round(rand_range(4,12)) ;
    end
    
    if nargin < 2
        c = zeros(3,1) ;
    end
    
    c = c(:)' ;
    
    % make vertices
    V = (2.*rand(N,3)-1) + repmat(c,N,1) ;
    
    % get convex hull
    F = convhull(V) ;
    
    % discard vertices that are not in convex hull
    V = V(unique(F(:)),:) ;
    
    % get convex hull again
    F = convhull(V) ;
    
    % create structure
    S.faces = F ;
    S.vertices = V ;
end
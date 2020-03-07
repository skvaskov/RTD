function P = make_random_polygon(N_vertices,offset,scale)
% P = make_random_polygon(N_vertices,offset,scale)
%
% Creates a random 2-D polygon with a given number of vertices, offset from
% the point (0,0), and scaled up or down as desired. The polygon is a
% 2-by-N, counterclockwise polyline.
%
% If the inputs N_vertices, offset, and scale are vectors, then the output
% is a 2-by-N polyline with as many polygons are the length of N_vertices,
% with each polygon separated by a column of NaNs.

    if nargin < 1
            N_vertices = round(randRange(3,9)) ;
    end
    
    Npolygons = size(N_vertices,2) ;
    
    if nargin < 2
        offset = zeros(2,Npolygons) ;
    end
    
    if nargin < 3
        scale = ones(1,Npolygons) ;
    end

    if length(N_vertices) == 1
        vertices = rand(2,N_vertices) - 0.5 ;
        P = scale*points_to_CCW(vertices, true) + offset ;
    else
        P = [] ;
        for idx = 1:length(N_vertices)
            P = [P, nan(2,1), make_random_polygon(N_vertices(idx),offset(:,idx),scale(idx))] ;
        end
        P = P(:,2:end) ;
    end
end
function [F,V] = make_rectangle_for_patch(L,W)
    % make vertices
    V = 0.5.*[-L L L -L ; -W -W W W ; 0 0 0 0]' ;
    
    % make face
    F = [1 2 3 4] ;
end
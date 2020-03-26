function varargout = convert_box_obstacles_to_halfplanes(O,b)
% O_struct = convert_box_obstacles_to_halfplanes(O,b)
% [O_buf,A_O,b_O,N_obs,N_halfplanes] = segway_box_obstacles_to_halfplanes(O)
%
% Given box obstacles (all assumed to be the same size!) for the segway,
% return the halfplane representation of these obstacles; that is, a matrix
% A and vector b for which, if x \in \R^2,
%
%     x \in obs  <=>  Ax - b > 0
%
% Author: Shreyas Kousik
% Created: 23 Mar 2020
% Updated: 25 Mar 2020

    if nargin < 2
        b = 0 ;
    end

    O_buf = [] ;
    A_O = [] ;
    b_O = [] ;

    % make sure O does not have a column of nans to start
    if isnan(O(1,1))
        O = O(:,2:end) ;
    end

    % create buffered obstacles and halfplane representations
    N_O = size(O,2) ;
    N_obs = ceil(N_O/6) ;
    for idx = 1:6:N_O
        % get obstacle (we know it's a written as 5 points in CCW
        % order, so we can cheat a bit here)
        o = O(:,idx:idx+4) ;

        % create buffered obstacles
        if b > 0
            o_buf = buffer_box_obstacles(o,b,13) ;
        else
            o_buf = o ;
        end
        O_buf = [O_buf, nan(2,1), o_buf] ;

        % create halfplane representation for collision checking
        [A_idx,b_idx] = vert2lcon(o_buf') ;
        A_O = [A_O ; A_idx] ;
        b_O = [b_O ; b_idx] ;

        % get the number of halfplanes (thank goodness this ends up
        % being exactly the same for every obstacle, saving us a
        % lot of work)
        N_halfplanes = length(b_idx) ;
    end
    
    if nargout == 1
       O_str.O = O_buf ;
       O_str.A = A_O ;
       O_str.b = b_O ;
       O_str.N_obs = N_obs ;
       O_str.N_halfplanes = N_halfplanes ;
       varargout = {O_str} ;
    else
       varargout = {O_buf,A_O,b_O,N_obs,N_halfplanes} ;
    end
end
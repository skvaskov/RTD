function out = check_points_in_halfplane_obstacles(P,varargin)
% out = check_points_in_halfplane_obstacles(P,obstacle_struct)
% out = check_points_in_halfplane_obstacles(P,A_obs,b_obs,N_obs,N_hfp)
%
% Given a 2-by-N set of points P and obstacles represented by halfplaces,
% return out as a 1-by-N logical array with entries that are TRUE for
% points that are INSIDE obstacles
%
% The input can be the obstacle structure returned by
% segway_box_obstacles_to_halfplanes
%
% Author: Shreyas Kousik
% Created: 23 Mar 2020
% Updated: 25 Mar 2020

    if length(varargin) == 1
        A_obs = varargin{1}.A ;
        b_obs = varargin{1}.b ;
        N_obs = varargin{1}.N_obs ;
        N_hfp = varargin{1}.N_halfplanes ;
    else
        A_obs = varargin{1} ;
        b_obs = varargin{2} ;
        N_obs = varargin{3} ;
        N_hfp = varargin{4} ;
    end

    if isempty(A_obs)
        % no obstacles, no problem!
        out = false(1,length(P)) ;
    else
        % yes obstacles, maybe problem?
        P_chk = A_obs*P - b_obs ;
        P_chk = reshape(P_chk,N_hfp,[]) ;
        P_chk = reshape(max(P_chk,[],1),N_obs,[]) ;
        out_dbl = -min(P_chk,[],1) ;
        out = (out_dbl) >= 0 ;
    end
end
function t_f = get_t_f_from_v_0_for_segway(v_0,t_f_all,v_0_min,v_0_max)
% t_f = get_t_f_from_v_0_for_segway(v_0,t_f_all,v_0_min,v_0_max)
%
% Given an initial speed v_0, return the time horizon t_f required for the
% FRS computation corresponding to the highest speed range that v_0 is in.
%
% This is basically identical to get_t_f_from_v_0 for the turtlebot.
%
% Author: Shreyas Kousik
% Created: 10 Mar 2020
% Updated: -

    if nargin < 2
        t_f_all = [0.6 0.80 0.80] ;
    end
    
    if nargin < 3
        v_0_min = [0.0 0.5 1.0] ;
    end

    if nargin < 4
        v_0_max = [0.5 1.0 1.5] ;
    end
    
    % figure out which interval the given v_0 is in
    idxs = (v_0 >= v_0_min) & (v_0 <= v_0_max) ;

    % get t_f
    t_f = max(t_f_all(idxs)) ;
end
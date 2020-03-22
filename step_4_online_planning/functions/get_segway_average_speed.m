function v_avg = get_segway_average_speed(T,V,t_h)
% v_avg = get_segway_average_speed(T,V,time_horizon)
%
% Get the segway's average speed over a given time horizon. The inputs are:
%
%   T   -   the segway's time as an array (1-by-N)
%   V   -   the segway's speed as an array (1-by-N)
%   t_h -   the amount of time over which to average
%
% This averages the last t_h seconds of V.
%
% Author: Shreyas
    t_f = T(end) ;
    t_0 = max(T(1), t_f - t_h) ;
    
    if t_f > t_0
        T_avg = linspace(t_0,t_f) ;
    else
        T_avg = t_f ;
    end
    
    V_avg = match_trajectories(T_avg,T,V) ;
    v_avg = mean(V_avg) ;
end
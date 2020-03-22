function w_des = make_segway_random_yaw_rate(w_cur,delta_w,w_max)
% w_des = make_segway_random_yaw_rate(w_cur,delta_w,w_max)
%
% Generate a random yaw rate for the Segway, given the current yaw rate,
% the allowable change in yaw rate, and the yaw rate bounds.
%
% Author: Shreyas Kousik
% Created: 19 May 2020

    % make feedforward input
    w_range = [w_cur - delta_w, w_cur + delta_w] ;
    w_range = bound_values(w_range, -w_max, w_max) ;
    w_des = w_range(round(rand_range(1,2))) ;
end
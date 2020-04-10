function k = get_segway_k_from_w_and_v(FRS,w,v)
% k = get_segway_k_from_w_and_v(FRS,w,v)
%
% Given a yaw rate w and speed v, figure out the corresponding trajectory
% parameter k \in [-1,1]^2
%
% Author: Shreyas Kousik
% Created: 9 Apr 2020

    k = nan(2,1) ;

    % get the yaw rate parameter
    k(1) = w ./ FRS.w_max ;

    % get the speed parameter
    k(2) = (v - mean(FRS.v_range))*(2/diff(FRS.v_range)) ;
    
end
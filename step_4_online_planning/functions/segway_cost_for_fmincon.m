function [c, gc] = segway_cost_for_fmincon(k,FRS,wp_local,start_tic,timeout)
% [c, gc] = segway_cost_for_fmincon(k,FRS,wp_local)
% [c, gc] = segway_cost_for_fmincon(k,FRS,wp_local,start_tic,timeout)
%
% Evaluate the cost and cost gradient for use with fmincon when performing
% trajectory optimization for the segway.
%
% Author: Shreyas Kousik
% Created: 13 Mar 2020
% Updated: -

    % evaluate cost and gradient
    c = segway_cost(k(1),k(2),FRS.w_max,FRS.v_range(2),wp_local(1),wp_local(2)) ;
    gc = segway_cost_grad(k(1),k(2),FRS.w_max,FRS.v_range(2),wp_local(1),wp_local(2)) ;
    
    % perform timeout check
    if nargin > 3 && toc(start_tic) > timeout
        error('Timed out while evaluating cost function!')
    end
end
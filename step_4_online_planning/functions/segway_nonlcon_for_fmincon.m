function [n, neq, gn, gneq] = segway_nonlcon_for_fmincon(k,...
            cons_poly,cons_poly_grad,start_tic,timeout)
        
% [n, neq, gn, gneq] = segway_nonlcon_for_fmincon(k,...
%                          cons_poly,cons_poly_grad,start_tic,timeout)
%
% Evaluate the constraint Jacobian for use with fmincon when performing
% trajectory optimization for the segway.
%
% Author: Shreyas Kousik
% Created: 31 May 2019

    n = evaluate_constraint_polynomial(cons_poly,k) ;
    neq = [] ;
    gn = evaluate_constraint_polynomial_gradient(cons_poly_grad,k)' ;
    gneq = [] ;
    
    % perform timeout check
    if nargin > 3 && toc(start_tic) > timeout
        error('Timed out while evaluating cost function!')
    end
end
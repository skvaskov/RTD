function out = evaluate_constraint_polynomial(p_k,k)
% out = evaluate_constraint_polynomial(p_k,k)
%
% Evaluate a list of polynomial constraints represented by the struct p_k
% on the point k (2-by-1).
%
% Note that this is the same as the function eval_w.
%
% Author: Shreyas Kousik
% Created: 31 May 2019
%
% See also: evaluate_constraint_polynomial_gradient,
% evaluate_FRS_polynomial_on_obstacle_points, get_FRS_polynomial_structure
% get_constraint_polynomial_gradient, 
    out = p_k.coef*prod(repmat(k',size(p_k.pows,1),1).^p_k.pows,2) ;
end
function out = evaluate_constraint_polynomial_gradient(J,k)
% out = evaluate_constraint_polynomial_gradient(J,k
%
% Given a Jacobian matrix represented as a coefficients matrix and a powers
% matrix in the structure J, evaluate it at the point k (N_k-by-1, usually
% 2-by-1)
%
% Note that this is the same as the function eval_J
%
% Authors: Shreyas Kousik and Sean Vaskov
% Created: 31 May 2019
%
% See also: evaluate_constraint_polynomial, get_FRS_polynomial_structure,
% evaluate_FRS_polynomial_on_obstacle_points, 
% get_constraint_polynomial_gradient, 

    N_k = length(k);
    N = J.N ;

    out = zeros(N,N_k) ;
 
    for idx = 1:N_k
        out(:,idx) = J.coef(1+N*(idx-1):N*idx,:) * ...
            (prod(repmat(k',size(J.pows,1),1).^J.pows(:,(idx-1)*N_k+1:N_k*idx),2)) ;
    end
end
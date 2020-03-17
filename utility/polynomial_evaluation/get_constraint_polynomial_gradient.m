function J_k_struct = get_constraint_polynomial_gradient(p_k_struct)
% J_k_struct = get_constraint_polynomial_gradient(p_k_struct)
%
% Given an N-by-1 polynomial in the 2-by-1 variable k, create an N-by-2
% Jacobian represented by its coefficients and powers; this is the gradient
% of the constraint polynomial with respect to k.
%
% Note that this is the same as the function diff_wk_wrt_k
%
% INPUTS:
%  p_k      a structure describing an N-by-1 list of polynomials in the
%           variables [k1, k2], which contains a coefficient matrix that is
%           N-by-Nterms and a powers matrix that is Nterms-by-2
%
% OUTPUTS:
%  J_k     a structure with fields as follows:
%
%    coef  a matrix of coefficients of the Jacobian, which is 2*N-by-Nterms
%          in size; the first N rows correspond to the k1 partial
%          derivative, and the last N to the k2 partial derivative
%
%    pows  a matrix of powers of k1 and k2; the first column is the powers
%          of k1 in the 1st column of the Jacobian; the second column is
%          the powers of k2 in the 1st column of the Jacobian; the third
%          column is the powers of k1 in the second column of the Jacobian;
%          and the fourth column is the powers of k2 in the second column
%          of the Jacobian
%
% Authors: Shreyas Kousik and Sean Vaskov
% Created: 31 May 2019
%
% See also: evaluate_constraint_polynomial, get_FRS_polynomial_structure,
% evaluate_FRS_polynomial_on_obstacle_points, 
% evaluate_constraint_polynomial_gradient, 

% extract info
coef = p_k_struct.coef ;
pows = p_k_struct.pows ;
N = p_k_struct.N ;
Nk = length(p_k_struct.k_cols);


% multiply each row of coeffs by each column of powers independently and
% stack them on top of each other, so that wkcoef is 2*N x Nterms
J_coef = [];
for idx = 1:Nk
    J_coef = [J_coef;coef .* repmat(pows(:,idx)',N,1)];
end

% create the powers of dwdk as an Nterms x 4 matrix; the first two columns
% correspond to the powers of k1 and k2 for the first column of the
% Jacobian dpdk, and the second two columns correspond to the second column
% of the Jacobian
J_pows = [];

for idx = 1:Nk
    dpdk_pows(:,idx) = pows(:,idx) + (pows(:,idx) == 0) - 1 ;
end

for idx = 1:Nk
    J_pows = [J_pows,pows(:,1:idx-1), dpdk_pows(:,idx), pows(:,idx+1:Nk)];
end

J_coef = sparse(J_coef) ;
J_pows = sparse(J_pows) ;

% create output structure
J_k_struct.coef = J_coef ;
J_k_struct.pows = J_pows ;
J_k_struct.N = N ;
end
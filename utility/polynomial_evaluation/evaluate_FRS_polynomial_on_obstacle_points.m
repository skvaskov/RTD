function p_k_struct = evaluate_FRS_polynomial_on_obstacle_points(p_struct,O)
% p_k = evaluate_FRS_polynomial_on_obstacle_points(p,O)
%
% This function evaluates the FRS polynomial on a finite list of obstacle
% points. It returns a list of constraint polynomials to be used for online
% trajectory optimization.
%
% Note that this is the same as the function sub_z_into_w
%
% INPUTS:
%   p   a structure representing the polynomial w, which must contain the
%       outputs from the msspoly function decomp, as the fields:
%           wvars - vector of free msspoly vars called k1,k2,z1,z2
%           wpows - m-by-4 array of powers of each term of w
%           wcoef - 1-by-m array of the coefficients of w
%           zcols - 1-by-2 vector of the columns of wpows for z1 and z2
%                   (which must be in the order [z1col z2col])
%           kcols - 1-by-2 vector of the columns of wpows for k1 and k2 (in
%                   that order)
%
%   O   the 2-by-N points in coordinates (z1,z2) to plug in to w and
%       return an N-by-1 list of polynomials in k1 and k2
%
% OUTPUTS:
%   p_k   a structure representing an N-by-1 list of polynomials in k1 and
%         k2, with fields wkpows, wkcoef, N, and kcols
%
% Authors: Shreyas Kousik and Sean Vaskov
% Created: 31 May 2019
%
% See also: get_FRS_polynomial_structure, evaluate_constraint_polynomial,
% evaluate_constraint_polynomial_gradient,
% get_constraint_polynomial_gradient

% extract info from w
pows = p_struct.pows ;
coef = p_struct.coef ;

% get the number of points to be evaluated
N = size(O,2) ;

% get the powers of z1 and z2 that will be used to evaluate p
sub_pows = pows(:,p_struct.z_cols) ;

% every point in P(1,:) needs to be raised to every power in sub_pows(:,1),
% and the same for P(2,:) wrt sub_pows(:,2)
P = sub_pows' ;

% get points in the format to the size: 2*Nterms x Npoints, where each pair
% of rows is alternating [z1;z2;z1;z2;...] to match the number of terms in
% the polynomial
O_mat = repmat(O,size(P,2),1) ;

% get powers to the same size as the points, where each pair of rows
% corresponds to [z1;z2;...] and each column is  repeated to be applied to
% each point (which is a column of O)
P_mat = repmat(P(:),1,N) ;

% now raise the points to their corresponding powers
O_mat = O_mat.^P_mat ;

% collapse O_mat by multiplying odd rows against even rows, since each row
% corresponds to z1 or z2
O_mat = O_mat(1:2:end,:).*O_mat(2:2:end,:) ;

% create a new coefficients matrix by replicating the current one Nterms
% times and multiplying each copy with the corresponding column in O_mat
p_k_coef = repmat(coef,N,1).*O_mat' ;

% create the new powers matrix of the N polynomials in k1 and k2
p_k_pows = pows(:,p_struct.k_cols) ;

%% COLLAPSING ROWS
% get all rows of wpows that are matched and sum the corresponding
% coefficients to create a row of coefficients that correspond to the
% reduced rows of powers

% sort pows first
[p_k_pows_sorted, i_s] = sortrows(p_k_pows) ;
[p_k_pows_unique,~, i_c] = unique(p_k_pows_sorted,'rows','sorted') ;

% sort coef's columns in the same order as pows' rows
p_k_coef = p_k_coef(:,i_s) ;

% sum the columns of wkcoef in groups based on how wkpows was made unique
% row by row (so identical rows of wkpows correspond to disparate columns
% of wkcoef, and these columns must be summed because they correspond to
% the terms in the polynomial with the same variables and powers)
summed_columns = splitapply(@(c) {sum(c,2)}, p_k_coef, i_c') ;

% concatenate each summed group of columns left to right
p_k_coef_collapsed = cat(2,summed_columns{:}) ;

%% FINAL OUTPUT
% create the output structure
p_k_struct.coef = p_k_coef_collapsed ;
p_k_struct.pows = p_k_pows_unique ;
p_k_struct.N = N ;
p_k_struct.k_cols = 1:length(p_struct.k_cols); % this is a formality
end
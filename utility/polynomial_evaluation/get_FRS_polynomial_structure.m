function p_str = get_FRS_polynomial_structure(p,z,k,t)
% p_str = get_FRS_polynomial_structure(p,z,k)
% p_str = get_FRS_polynomial_structure(p,z,k,t)
%
% Given a polynomial p in the variables z and k, decompose it into
% coefficients and powers, and return an object with this information in a
% useful format. If p also has terms in t (an optional fourth argument),
% then the decomposition will include t as well.
%
% Note that this is the same as the function decompose_w_polynomial
%
% Authors: Shreyas Kousik and Sean Vaskov
% Created: 31 May 2019
%
% See also: evaluate_constraint_polynomial,
% evaluate_FRS_polynomial_on_obstacle_points, 
% evaluate_constraint_polynomial_gradient, 
% get_constraint_polynomial_gradient, 


    % decompose w into matrices and variable IDs
    [vars,pows,coef] = decomp(p) ;
    [~,var_id] = isfree(vars) ;
    
    % iterate through z and get the corresponding columns of the powers
    % matrix, then do the same for k
    Nz = length(z) ;
    z_cols = nan(1,Nz) ;
    for idx = 1:Nz
        [~,z_id] = isfree(z(idx)) ;
        zc = find(var_id == z_id) ;
        z_cols(idx) = zc ;        
    end
    
    Nk = length(k) ;
    k_cols = nan(1,Nk) ;
    for idx = 1:Nk
        [~,k_id] = isfree(k(idx)) ;
        kc = find(var_id == k_id) ;
        k_cols(idx) = kc ;        
    end
    
    % create output structure
    p_str.pows = pows ;
    p_str.coef = coef ;
    p_str.z_cols = z_cols ;
    p_str.k_cols = k_cols ;
    p_str.t_cols = [] ;
    
    if nargin > 3
        % get the last column as the time index
        [~,t_id] = isfree(t) ;
        t_cols = find(var_id == t_id) ;
        p_str.t_cols = t_cols ;
    end
end
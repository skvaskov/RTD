function pout = cat_polynomial_structures(p_structs)
%pout = cat_polynomial_structures(p_structs)
%given a cell array of polynomial_structures of k (generated with
%get_FRS_polynomial_structure and
%evaluate_FRS_polynomial_on_obstacle_points.m and
%get_onstraint_polynomial_gradient.m)

%pout joins them into 1 structure. This function currently throws an error
%flag if the powers matrices are different, but functionality to adjust the
%matrices in the structure can be added

%Author: Sean Vaskov
%Created 19 Aug 2021

% See also: evaluate_constraint_polynomial_gradient,
% evaluate_FRS_polynomial_on_obstacle_points, get_FRS_polynomial_structure
% get_constraint_polynomial_gradient, 

pout = p_structs{1};

for i = 2:length(p_structs)
    ptmp = p_structs{i};
    
    if size(pout.pows,1) ~= size(ptmp.pows,1) && all(pout.k_cols == ptmp.k_cols)
        
        [pout,ptmp] = matchsize_of_polynomial_structure(pout,ptmp);
        
    end
    
    if size(pout.pows,1) == size(ptmp.pows,1) && all(all(pout.pows == ptmp.pows)) && all(pout.k_cols == ptmp.k_cols)
        pout.N = pout.N + ptmp.N;
        pout.coef = [pout.coef;ptmp.coef];
    else
       error('cat_polynomial_structures: difference in powers matrix or k_cols order') 
    end
end

end


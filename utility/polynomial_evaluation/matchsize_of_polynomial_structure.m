

function [Aout,Bout] = matchsize_of_polynomial_structure(Ain,Bin)
%pout = matchsize_degree_of_polynomial_structure(pin)

%given 2 structure for polynomial funcitons of k, output 2 structures where
%the monomials for their coeffiecient and power matrices are the same

%Author: Sean Vaskov
%Created 19 Aug 2021

% See also: evaluate_constraint_polynomial_gradient,
% evaluate_FRS_polynomial_on_obstacle_points, get_FRS_polynomial_structure
% get_constraint_polynomial_gradient, 
powA = Ain.pows;
powB = Bin.pows;

powout = unique([powA;powB],'rows');

if size(powA,1) == size(powout,1) && nnz(powout-powA) == 0
    Aout = Ain;
else
    idxj = [];
    idxi = [];
    
    for i = 1:size(powout,1)
        [~,loca] = ismember(powout(i,:),powA,'rows');
        if loca >0
            idxi = [idxi;i];
            idxj = [idxj;loca];
        end
    end
    Atransform = sparse(idxi,idxj,ones(length(idxi),1),size(powout,1),size(powA,1));
    
    Aout.pows = powout;
    Aout.coef = Ain.coef*(Atransform');
    Aout.N = Ain.N;
    Aout.k_cols = Ain.k_cols;

end
    
if size(powB,1) == size(powout,1) && nnz(powout-powB) == 0
    Bout = Bin;
else
    idxj = [];
    idxi = [];
    
    for i = 1:size(powout,1)
        [~,locb] = ismember(powout(i,:),powB,'rows');
        if locb >0
            idxi = [idxi;i];
            idxj = [idxj;locb];
        end
    end
    Btransform = sparse(idxi,idxj,ones(length(idxi),1),size(powout,1),size(powB,1));
    
    Bout.pows = powout;
    Bout.coef = Bin.coef*(Btransform');
    Bout.N = Bin.N;
    Bout.k_cols = Bin.k_cols;
end
    

end


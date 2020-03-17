function J_out = eval_J(k,J_coef,J_pows,N)
% J_out = eval_J(k,J_coef,J_pows,N)
%
% Given a Jacobian matrix represented as a coefficients matrix and a powers
% matrix (see diff_wk_wrt_k), evaluate the jacobian at k (Nk-by-1)

    Nk = length(k);
    
    J_out = zeros(N,Nk) ;
 
    for idx = 1:Nk
        J_out(:,idx) = J_coef(1+N*(idx-1):N*idx,:) * ...
            (prod(repmat(k',size(J_pows,1),1).^J_pows(:,(idx-1)*Nk+1:Nk*idx),2)) ;
    end
end
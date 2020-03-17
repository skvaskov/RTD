function [Jcoef, Jpows] = diff_wk_wrt_k(wk)
% [Jcoef, Jpows] = diff_wk_wrt_k(wk)
%
% Given an N-by-1 polynomial in the 2-by-1 variable k, create an N-by-2
% Jacobian represented by its coefficients and powers
%
% INPUTS:
%   wk   -  a structure describing an N-by-1 list of polynomials in the
%           variables [k1, k2], which contains a coefficient matrix that is
%           N-by-Nterms and a powers matrix that is Nterms-by-2
%
% OUTPUTS:
%  Jcoef - a matrix of coefficients of the Jacobian, which is 2*N-by-Nterms
%          in size; the first N rows correspond to the k1 partial
%          derivative, and the last N to the k2 partial derivative
%
%  Jpows - a matrix of powers of k1 and k2; the first column is the powers
%          of k1 in the 1st column of the Jacobian; the second column is
%          the powers of k2 in the 1st column of the Jacobian; the third
%          column is the powers of k1 in the second column of the Jacobian;
%          and the fourth column is the powers of k2 in the second column
%          of the Jacobian
%
% To recompose the Jacobian into an msspoly, use the following (but be
% aware it runs very slowly):
%   Jk1 = Jcoef(1:N,:)*((k(1).^Jpows(:,1)).*(k(2).^Jpows(:,2))) ;
%   Jk2 = Jcoef(N+1:end,:)*((k(1).^Jpows(:,3)).*(k(2).^Jpows(:,4))) ;

    % extract info
    wkcoef = wk.wkcoef ;
    wkpows = wk.wkpows ;
    N = wk.N ;
    Nk = length(wk.kcols);


    % multiply each row of coeffs by each column of powers independently and
    % stack them on top of each other, so that wkcoef is 2*N x Nterms
    Jcoef = [];
    for idx = 1:Nk
        Jcoef = [Jcoef;wkcoef .* repmat(wkpows(:,idx)',N,1)];
    end
    % create the powers of dwdk as an Nterms x 4 matrix; the first two columns
    % correspond to the powers of k1 and k2 for the first column of the
    % Jacobian dwdk, and the second two columns correspond to the second column
    % of the Jacobian
    Jpows = [];

    for idx = 1:Nk
        dwdkpows(:,idx) = wkpows(:,idx) + (wkpows(:,idx) == 0) - 1 ;
    end

    for idx = 1:Nk
        Jpows = [Jpows,wkpows(:,1:idx-1), dwdkpows(:,idx), wkpows(:,idx+1:Nk)];
    end

    Jcoef = sparse(Jcoef) ;
    Jpows = sparse(Jpows) ;
end

% function [Jcoef, Jpows] = diff_wk_wrt_k(wk)
% % [Jcoef, Jpows] = diff_wk_wrt_k(wk)
% %
% % Given an N-by-1 polynomial in the 2-by-1 variable k, create an N-by-2
% % Jacobian represented by its coefficients and powers
% %
% % INPUTS:
% %   wk   -  a structure describing an N-by-1 list of polynomials in the
% %           variables [k1, k2], which contains a coefficient matrix that is
% %           N-by-Nterms and a powers matrix that is Nterms-by-2
% %
% % OUTPUTS:
% %  Jcoef - a matrix of coefficients of the Jacobian, which is 2*N-by-Nterms
% %          in size; the first N rows correspond to the k1 partial
% %          derivative, and the last N to the k2 partial derivative
% %
% %  Jpows - a matrix of powers of k1 and k2; the first column is the powers
% %          of k1 in the 1st column of the Jacobian; the second column is
% %          the powers of k2 in the 1st column of the Jacobian; the third
% %          column is the powers of k1 in the second column of the Jacobian;
% %          and the fourth column is the powers of k2 in the second column
% %          of the Jacobian
% %
% % To recompose the Jacobian into an msspoly, use the following (but be
% % aware it runs very slowly):
% %   Jk1 = Jcoef(1:N,:)*((k(1).^Jpows(:,1)).*(k(2).^Jpows(:,2))) ;
% %   Jk2 = Jcoef(N+1:end,:)*((k(1).^Jpows(:,3)).*(k(2).^Jpows(:,4))) ;
%
% % extract info
% wkcoef = wk.wkcoef ;
% wkpows = wk.wkpows ;
% N = wk.N ;
%
% % get k1 and k2's powers separately
% wk1pows = wkpows(:,1) ;
% wk2pows = wkpows(:,2) ;
%
% % multiply each row of coeffs by each column of powers independently and
% % stack them on top of each other, so that wkcoef is 2*N x Nterms
% Jcoef = [wkcoef .* repmat(wk1pows',N,1) ;
%          wkcoef .* repmat(wk2pows',N,1) ] ;
%
% % create the powers of dwdk as an Nterms x 4 matrix; the first two columns
% % correspond to the powers of k1 and k2 for the first column of the
% % Jacobian dwdk, and the second two columns correspond to the second column
% % of the Jacobian
% dwdk1pows = wk1pows + (wk1pows == 0) - 1 ;
% dwdk2pows = wk2pows + (wk2pows == 0) - 1 ;
%
% Jpows = [dwdk1pows, wk2pows, wk1pows, dwdk2pows];
%
% Jcoef = sparse(Jcoef) ;
% Jpows = sparse(Jpows) ;
% end
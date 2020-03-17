function mon = generate_rip_monomials(var, degree, k_rip, normflag)
% mon = generate_rip_monomials(var, degree, k_rip, normflag)
%
% RIP: Running Intersection Property
%
% Given a d-by-1 free variable "var" and a max degree "degree" compute the
% monomial basis that has no more than "k_rip" cross terms per monomial. If
% the normflag is set to 'Inf' then bound the max degree of each monomial
% to degree (default); if 'one' then bound the total degree.
    
    x = var(:) ;
    d = length(var) ; % dimension of the input variables
    nk = degree ; % max degree of RIP poly
    k = k_rip ; % max number of cross terms

    if nargin < 4
        normflag = 'Inf' ;
    end

    % generate powers of RIP poly
    rip_poly_powers = unique(nchoosek(repmat(0:nk,1,d),d),'rows') ;
    pow_is_0 = rip_poly_powers == 0 ;
    k_limited_rows = sum(pow_is_0,2) >= (d - k) ;
    k_all_powers = unique(rip_poly_powers(k_limited_rows,:),'rows') ;

    % generate the monomials
    switch normflag
        case 'Inf'
            L = size(k_all_powers,1) ;
            X = repmat(x,1,L) ;
            mon_temp = X.^(k_all_powers') ;
        case 'one'
            sum_k_pows = sum(k_all_powers,2) ;
            Lk_1_log = sum_k_pows <= nk ;
            k_1_pows = unique(k_all_powers(Lk_1_log ,:)','rows')' ;
            L = size(k_1_pows,1) ;
            X = repmat(x,1,L) ;
            mon_temp = X.^(k_1_pows') ;
        otherwise
            error(['The norm to use to limit the monomial degree must be ',...
                   'Inf or one'])
    end
    
    mon = mon_temp(1,:) ;
    for idx = 2:size(mon_temp,1)
        mon = mon.*mon_temp(idx,:) ;
    end
    
    mon = mon(:) ;
end
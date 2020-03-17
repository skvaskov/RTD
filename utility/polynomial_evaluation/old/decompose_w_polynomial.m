function wstr = decompose_w_polynomial(w,z,k,t)
% Given a polynomial w in the variables z and k, decompose it into
% coefficients and powers, and return an object with this information in a
% useful format. If w also has terms in t (an optional fourth argument),
% then the decomposition will include t as well.

    % decompose w into matrices and variable IDs
    [wvars,wpows,wcoef] = decomp(w) ;
    [~,wid] = isfree(wvars) ;
    
    % iterate through z and get the corresponding columns of the powers
    % matrix, then do the same for k
    Nz = length(z) ;
    zcols = nan(1,Nz) ;
    for idx = 1:Nz
        [~,zid] = isfree(z(idx)) ;
        zc = find(wid == zid) ;
        zcols(idx) = zc ;        
    end
    
    Nk = length(k) ;
    kcols = nan(1,Nk) ;
    for idx = 1:Nk
        [~,kid] = isfree(k(idx)) ;
        kc = find(wid == kid) ;
        kcols(idx) = kc ;        
    end
    
    % create output w structure
    wstr.wpows = wpows ;
    wstr.wcoef = wcoef ;
    wstr.zcols = zcols ;
    wstr.kcols = kcols ;
    wstr.tcols = [] ;
    
    if nargin > 3
        % get the last column as the time index
        [~,tid] = isfree(t) ;
        tcols = find(wid == tid) ;
        wstr.tcols = tcols ;
    end   
end
function m = sphericalMoments(x,c,R)
    n = length(x);
    function l = moments(p)
    % Step one, normalize problem.
    %
        [var,pow,Coeff] = decomp(subs(p,x,x+c));
        mat = match(var,x);

        powx = zeros(size(pow,1),n);
        powx(:,mat ~= 0) = pow(:,mat(mat~=0));
        
        beta = 0.5*(powx+1);
        
        lbnd = 2*prod(gamma(beta),2)./gamma(sum(beta,2));
        
        lbnd(any(mod(powx,2)==1,2)) = 0;

        exp = sum(powx,2)+n;
        Coeff = Coeff*diag(lbnd.*(R.^exp)./exp);
        
        I = (1:length(var))';
        I(mat(mat~=0)) = [];
        
        if isempty(I), l = sum(Coeff,2);
        else,
            l = recomp(var(I),pow(:,I),Coeff);
        end
    end
    
    m = @moments;
end
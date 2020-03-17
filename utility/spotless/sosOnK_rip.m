% [ prgout, tk ] = sosOnK_rip( prg, p, x, h, d, m );
%
% prg -- spotsosprg.
% p   -- 1-by-1 msspoly in x.
% x   -- n-by-1 free msspoly
% h   -- m-by-1 msspoly in x. (semialgebraic constraints defining K)
% d   -- scalar integer, d > deg(g).
% rip_cap -- scalar integer, max number of cross terms per monomial
%
% prgout --  new program with constraints
%       s(i) SOS, p - s'*h SOS
%       s(i) of maximal degree s.t. deg(s'*h) <= d.
% tk -- token associated with p - s'*h SOS.


function [ prg, tk ] = sosOnK_rip( prg, p, x, h, d, rip_cap )
if isa(x,'msspoly')
    deg_p= max( mod(deg(p,x),2) + deg(p,x), d);
else
    deg_p=0;
end
    Nh = size( h, 1 ); % total number of h's
    S = msspoly( zeros( Nh, 1 ) ); % create this many multiplier variables
    for i = 1:Nh
        deg_h=deg(h(i),x);
        if nargin < 6
            bases = monomials( x, 0:max(0, mod(deg_p - deg_h,2) + (deg_p - deg_h)) );
        else
            degree = max(0, mod(deg_p - deg_h,2) + (deg_p - deg_h)) ;
            bases = generate_rip_monomials(x,degree,rip_cap,'one') ;
        end
        [ prg, S( i ) ] = prg.newFreePoly( bases ); % make a polynomial whose coefficients are free variables
        prg = prg.withSOS( S( i ) ); % ensure that the polynomial is SOS
    end
%     keyboard
if ~isempty(S)
    [ prg, tk ] = prg.withSOS( p - S'*h ); % ensure that p is SOS on K 
else
    [ prg, tk ] = prg.withSOS( p ); % ensure that p is SOS on K 
end
%     prg.sosTokens=[prg.sosTokens;tk]; % storing the tokens corresponding to the desired SOS constraint
end
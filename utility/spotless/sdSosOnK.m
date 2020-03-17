% [ prgout, tk ] = sosOnK( prg, p, x, h, d, b );
%
% prg -- spotsosprg.
% p   -- 1-by-1 msspoly in x.
% x   -- n-by-1 free msspoly
% h   -- m-by-1 msspoly in x. (semialgebraic constraints defining K)
% d   -- scalar integer, d > deg(g).
% b   -- cell array of subsets of x
%
% prgout --  new program with constraints
%       s(i) SOS, p - s'*h SOS
%       s(i) of maximal degree s.t. deg(s'*h) <= d.
% tk -- token associated with p - s'*h SOS.
%
% function [ prg, tk ] = sosOnK( prg, p, x, h, d )
%     m = size( h, 1 ); % total number of h's
%     S = msspoly( zeros( m, 1 ) ); % create this many multiplier variables
%     bases = monomials( x, 0:d );
%     for i = 1:m
%         [ prg, S( i ) ] = prg.newFreePoly( bases ); % make a polynomial whose coefficients are free variables
%         prg = prg.withSOS( S( i ) ); % ensure that the polynomial is SOS
%     end
%     [ prg, tk ] = prg.withSOS( p - S'*h ); % ensure that p is SOS on K 
% end

function [ prg, tk ] = sdSosOnK( prg, p, x, h, d, b )
if isa(x,'msspoly')
    deg_p= max( mod(deg(p,x),2) + deg(p,x), d);
else
    deg_p=0;
end
    m = size( h, 1 ); % total number of h's
    S = msspoly( zeros( m, 1 ) ); % create this many multiplier variables
    for i = 1:m
        deg_h=deg(h(i),x);
        if nargin < 6
            bases = monomials( x, 0:max(0, mod(deg_p - deg_h,2) + (deg_p - deg_h)) );
        else
            bases = [] ;
            for bidx = 1:length(b)
                bases = [bases ;
                         monomials( b{bidx}, 0:max(0, mod(deg_p - deg_h,2) + (deg_p - deg_h)) ) ] ;
            end
        end
        [ prg, S( i ) ] = prg.newFreePoly( bases ); % make a polynomial whose coefficients are free variables
        prg = prg.withSDSOS( S( i ) ); % ensure that the polynomial is SOS
    end
%     keyboard
if ~isempty(S)
    [ prg, tk ] = prg.withSDSOS( p - S'*h ); % ensure that p is SOS on K 
else
    [ prg, tk ] = prg.withSDSOS( p ); % ensure that p is SOS on K 
end
%     prg.sosTokens=[prg.sosTokens;tk]; % storing the tokens corresponding to the desired SOS constraint
end

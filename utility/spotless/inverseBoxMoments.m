function m = inverseBoxMoments(x,a,b,a_dom,b_dom)
%
% m = boxMoments(x,am,bm,ap,bp)
%
% Produces m(p), p being an msspoly, which integrates out the variables
%   x over the complement of the box a(i) <= x(i) <= b(i) in the domain
%   a_dom(i) <= x(i) <= b_dom(i).

    function l = moments(p)
        l_plus = p;
        for i = 1:length(x)
            l_plus = subs(integral(l_plus,x(i)),x(i),b_dom(i)) ...
                   - subs(integral(l_plus,x(i)),x(i),a_dom(i));
        end
        
        l_minus = p;
        for i = 1:length(x)
            l_minus = subs(integral(l_minus,x(i)),x(i),b(i)) ...
                    - subs(integral(l_minus,x(i)),x(i),a(i));
        end

        l = l_plus - l_minus;        
    end
    m = @moments;
end
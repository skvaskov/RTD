function m = polyBoxMoments(x,t,p,eps,T)
%
% m = boxMoments(p,x,eps)
%
% Produces m(q), q being an msspoly, which integrates out the variables
%   x and t over the box p(t)(i) - eps(i)/2 <= x(i) <= p(t)(i) + eps(i)/2, 0 <= t <= T.
    function l = moments(q)
        l = q;
        for i = 1:length(x)
            l = subs(integral(l,x(i)),x(i),p(i) + eps(i)/2) ...
                - subs(integral(l,x(i)),x(i),p(i) - eps(i)/2);
        end
        
        l = subs(integral(l,t),t,T) - subs(integral(l,t),t,0);
    end
    m = @moments;
end
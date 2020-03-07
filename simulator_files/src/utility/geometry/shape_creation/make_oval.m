function [out1,out2] = make_oval(S,N)
% O = make_oval(S)
% [F,O] = make_oval(S)
% [F,O] = make_oval(S,N)
%
% Return a 2-by-M oval of xy-coordinates that are CCW that fits in a box of
% side length s (if s is a scalar) or s(1)-by-s(2) (if s is a vector). The
% optional second input N determines the number of sides to use to
% approximate the circular ends of the oval.
%
% If two outputs are specified, then this function outputs faces F and
% vertices V that can be used with patch.

    if nargin < 2
        N = 20 ;
    end

    if length(S) == 1
        % if s is a scalar, return a circle
        out1 = make_circle(S,N) ;
    else
        % if s is a vector, first sort s
        if S(1) > S(2)
            S = S([2 1]) ;
        end
        
        r = S(1)/2 ; % radius of caps
        l = S(2) - 2*r ; % length between cap centers
        
        % create the left cap
        t_left = linspace(pi/2,3*pi/2,ceil(N/2)) ;
        x_left = r*cos(t_left) - l/2 ;
        y_left = r*sin(t_left) ;
        
        % create the right cap
        t_right = linspace(-pi/2,pi/2,ceil(N/2)) ;
        x_right = r*cos(t_right) + l/2 ;
        y_right = r*sin(t_right) ;
        
        % create the oval
        out1 = [x_left, x_right ; y_left, y_right] ;
    end
    
    if nargout > 1
        out2 = out1' ;
        out1 = [1:size(out1,2), 1] ;
    else
        out1 = [out1(:,1:end), out1(:,1)] ;
    end
end
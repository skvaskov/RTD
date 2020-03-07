function C = make_circle(R,N)
% C = make_circle(R,N)
%
% Make a circle of radius R, approximated as an N-sided polygon. By
% default, R = 1 and N = 100. The output C is a 2-by-N array.
    if nargin < 2
        N = 100 ;
    end
    
    if nargin < 1
        R = 1 ;
    end
    
    % make angle vector
    a_vec = linspace(0,2*pi,N) ;
    
    % make C
    C = [R*cos(a_vec) ;
         R*sin(a_vec) ] ;
end
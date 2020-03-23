function [r,a] = compute_point_spacings(FP,b)
% [R,A] = COMPUTE_POINT_SPACINGS(FOOTPRINT,BUFFER_SIZE)
%
% Given a footprint and a buffer size, compute the point spacings r and a.
% The input "footprint" should be a 2-by-1 vector for a rectangular robot,
% or a scalar for a circular robot.
%
% Paper: https://arxiv.org/abs/1809.06746
%
% This implements the examples in Section 6.
%
% Author: Shreyas Kousik
% Date:   12 Apr 2019

    if length(FP) == 2
        W = min(FP) ; % robot width
        
        % make sure the buffer is small enough
        if b > W/2
            disp('Resizing the buffer to be smaller!')
            b = W/2 ;
        end
        
        % compute point spacings
        r = 2*b ;
        a = r*sin(pi/4) ;
    elseif length(FP) == 1
        R = FP ; % robot radius
        
        if b < R
            disp('Resizing the buffer to be smaller!')
            b = R ;
        end

        % compute point spacings
        t1 = acos((R-b)/R) ;
        t2 = acos(b/(2*R)) ;
        r = 2*R*sin(t1) ;
        a = 2*b*sin(t2) ;
    else
        error('Invalid footprint input!')
    end
end
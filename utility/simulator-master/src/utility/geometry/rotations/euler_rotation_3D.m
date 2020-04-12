function R = euler_rotation_3D(a,c)
    La = length(a) ;
    if nargin < 2
        if La == 1
            c = 'x' ;
        elseif La == 3
            c = 'zyx' ;
        else
            error('The Euler angle input must be a scalar or a 3-element vector')            
        end
    end
    
    c = lower(c) ;
    
    if La == 1
        switch c
            case 'x'
                a = [a 0 0] ;
            case 'y'
                a = [0 a 0] ;
            case 'z'
                a = [0 0 a] ;
            otherwise
                error(['If the first input is a scalar, the second input ',...
                    'must be X, Y, or Z as a string.'])
        end
    end
    
    R = eye(3) ;
    for idx = 1:length(c)
        switch c(idx)
            case 'x'
                R = eul_X(a(idx))*R ;
            case 'y'
                R = eul_Y(a(idx))*R ;
            case 'z'
                R = eul_Z(a(idx))*R ;
        end
    end
end

%% helper functions
function R = eul_X(a)
    R = [1  0      0 ;
         0  cos(a) sin(a) ;
         0 -sin(a) cos(a)] ;
end

function R = eul_Y(a)
    R = [cos(a) 0 -sin(a) ;
         0      1  0 ;
         sin(a) 0  cos(a)] ;
end

function R = eul_Z(a)
    R = [ cos(a) sin(a) 0 ;
         -sin(a) cos(a) 0 ;
          0      0      1] ;
end
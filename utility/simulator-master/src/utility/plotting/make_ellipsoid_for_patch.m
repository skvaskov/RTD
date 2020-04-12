function [F,V] = make_ellipsoid_for_patch(Rx,Ry,Rz,C,N)
% [F,V] = make_ellipsoid_for_patch([Rx,Ry,Rz])
% [F,V] = make_ellipsoid_for_patch(Rx,Ry,Rz)
% [F,V] = make_ellipsoid_for_patch(Rx,Ry,Rz,C,N)
%
% Make an ellipsoid of radii Rx, Ry, and Rz, centered at C, and represent
% it as faces F and vertices V to be used with the patch function. The
% input "N" specifies the number of points per dimension to be generated
% for the ellipsoid.

    if nargin < 5
        N = 20 ;
    end

    if nargin < 4
        C = zeros(3,1) ;
    end

    if nargin == 1
        dims = Rx ;
        Rx = dims(1) ;
        Ry = dims(2) ;
        Rz = dims(3) ;
    elseif nargin < 1
        Rx = 1 ;
        Ry = 1 ;
        Rz = 1 ;
    end

    % make points of ellipsoid
    [Vx,Vy,Vz] = ellipsoid(C(1),C(2),C(3),Rx,Ry,Rz,N) ;
    
    % transform into patch features
    [F,V] = surf2patch(Vx,Vy,Vz) ;
end
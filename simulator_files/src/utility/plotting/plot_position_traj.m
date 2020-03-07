function plot_position_traj(Z,D,varargin)
    if nargin < 2
        D = 3 ;
    end
    
    switch D
        case 2
            plot(Z(1,:),Z(2,:),varargin{:})
        case 3
            plot3(Z(1,:),Z(2,:),Z(3,:),varargin{:})
    end
end
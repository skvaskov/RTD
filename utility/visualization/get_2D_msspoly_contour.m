function h = get_2D_msspoly_contour(p,x,l,varargin)
% plot_2D_msspoly_contour(p,x,l,'property1',value1,'property2',value2,...)
%
% Given an msspoly p in the 2D variable x, plot its l-level set. If l is
% not provided, then l = 0 is used as the default. After the 3 default
% inputs, this function takes in the regular property/value pairs of the
% MATLAB default contour function. In addition, you can provide the
% following properties:
%   Offset     a 2-by-1 vector (x,y) to shift the origin to
%   Scale      a scalar value to scale the size of the plot by
%   FillColor  a color in usual matlab plotting syntax to fill the contour
%   pose       a 3-by-1 vecto(x,y,heading) to rotate and translate the
%                 shifted and scaled origin to
    
%% parse inputs
    if nargin < 3
        l = 0 ;
        varargin = {} ;
    end
    
    % create default inputs
    Offset = [0;0] ;
    Scale = [1;1] ;
    pose0 = [0;0;0];
    N = 100;
    % iterate through varargin to find Offset and Scale
    varargin_new = {} ;
    idx_new = 1 ;
    for idx = 1:2:length(varargin)
        switch varargin{idx}
            case 'Offset'
                Offset = varargin{idx+1} ;
            case 'Scale'
                Scale = varargin{idx+1} ;
            case 'pose'
                pose0 = varargin{idx+1};
            case 'N'
                N = varargin{idx+1};
            otherwise
                varargin_new{idx_new} = varargin{idx} ;
                varargin_new{idx_new+1} = varargin{idx+1} ;
                idx_new = idx_new + 2 ;
        end
    end

%% set up for plotting
    % set up grid for plotting
    x_vec = linspace(-1,1,N) ;
    [X1,X2] = meshgrid(x_vec,x_vec) ;
    X = [X1(:), X2(:)]' ;
    
    % create msspoly surface
     P = reshape(full(msubs(p,x,X)),N,N) ;
    
     h = contourc(x_vec,x_vec,P,[l l]) ;
     
    h(:,h(1,:) == l) = NaN;


    % scale and shift for plotting
    h = repmat([Scale(1);Scale(2)],[1 size(h,2)]).*h + [Offset(1);Offset(2)] ;
   
%% plot
 
    
    
    if any(pose0~=0)
    h = rotation_matrix_2D(pose0(3))*h+pose0(1:2);
    end
    
    if nargout < 1
        clear h
    end
end
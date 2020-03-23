function h = plot_2D_msspoly_contour(p,x,l,varargin)
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
    
%% parse inputs
    if nargin < 3
        l = 0 ;
        varargin = {} ;
    end
    
    % create default inputs
    Offset = [0;0] ;
    Scale = 1 ;
    FillColor = [] ;
    
    % iterate through varargin to find Offset and Scale
    varargin_new = {} ;
    idx_new = 1 ;
    for idx = 1:2:length(varargin)
        switch varargin{idx}
            case 'Offset'
                Offset = varargin{idx+1} ;
            case 'Scale'
                Scale = varargin{idx+1} ;
            case 'FillColor'
                FillColor = varargin{idx+1} ;
            otherwise
                varargin_new{idx_new} = varargin{idx} ;
                varargin_new{idx_new+1} = varargin{idx+1} ;
                idx_new = idx_new + 2 ;
        end
    end

%% set up for plotting
    % set up grid for plotting
    x_vec = linspace(-1,1,100) ;
    [X1,X2] = meshgrid(x_vec,x_vec) ;
    X = [X1(:), X2(:)]' ;
    
    % create msspoly surface
    P = reshape(full(msubs(p,x,X)),100,100) ;
    
%     % scale and shift for plotting
%     X1 = Scale*(X1 + Offset(1)) ;
%     X2 = Scale*(X2 + Offset(2)) ;
    if numel(Scale) ==1
        Scale = [Scale,Scale];
    end
    % scale and shift for plotting
    X1 = Scale(1)*(X1) + Offset(1) ;
    X2 = Scale(2)*(X2) + Offset(2) ;
    
%% plot
    if ~isempty(FillColor)
        [~,h] = contourf(X1,X2,P,[l l],'Fill','on',varargin_new{:}) ;
        colormap(FillColor)
    else
       [~, h] = contour(X1,X2,P,[l l],varargin_new{:}) ;
    end
    
    if nargout < 1
        clear h
    end
end
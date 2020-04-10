function projlims=  get_1D_msspoly_projection(p,x,l,dproj,varargin)
% get_1D_msspoly_projection(p,x,l,dproj,'property1',value1,'property2',value2,...)
%
% Given an msspoly p in the 2D variable x, plot its l-level set. If l is
% not provided, then l = 0 is used as the default.

%the function will find the lower and upper limits of {x_dproj | p(x) > 1}
%by evaluating p on a uniform mesh grid.
%it is currently set up to handle dimension(x) = 2

%After the 4 default
% inputs, this function takes in the regular property/value pairs of the
% MATLAB default contour function. In addition, you can provide the
% following properties:
%   Offset     a 2-by-1 vector (x,y) to shift the origin to (default 0)
%   Scale      a scalar value to scale the size of the plot by (default 1)
%   N          number of points in each dimension to sample p at (default 100)

% Author: Sean Vaskov
% Created: 10 March 2020
    
%% parse inputs
projlims = NaN;

if nargin<4
    varargin = {};
    dproj = 1;
    if nargin < 3
        l = 1 ;
    end
end


d = length(x);
    % create default inputs
    Offset = zeros(d,1) ;
    Scale = ones(d,1) ;
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
            case 'N'
                N = varargin{idx+1} ;
            otherwise
                varargin_new{idx_new} = varargin{idx} ;
                varargin_new{idx_new+1} = varargin{idx+1} ;
                idx_new = idx_new + 2 ;
        end
    end

%% set up for plotting
    % set up grid for plotting
    
    if d == 2
        x_vec = linspace(-1,1,N) ;
        
        [X1,X2] = meshgrid(x_vec,x_vec) ;
        X1 = X1(:)';
        X2 = X2(:)';
        
        X = [X1; X2] ;
        
        % create msspoly surface and find indexs above level sets
        P = full(msubs(p,x,X));
        Idx_in = find(P >= l);
        
        if ~isempty(Idx_in)
            % scale and shift for plotting
           
            if dproj == 1
                X1 = Scale(1)*(X1) + Offset(1) ;
                
                
                [~,I1] = min(X1(Idx_in));
                [~,I2] = max(X1(Idx_in));
                
                
                [X1,~,IC] = unique(X1);
                I1 = IC(Idx_in(I1));
                I2 = IC(Idx_in(I2));
                
            
                    projlims(1) = X1(I1);    
            
                
        
                    projlims(2) = X1(I2);
          
                
            elseif dproj == 2
                X2 = Scale(2)*(X2) + Offset(2) ;
                
                [~,I1] = min(X2(Idx_in));
                [~,I2] = max(X2(Idx_in));
                
                
                   [X2,~,IC] = unique(X2);
                

                I1 = IC(Idx_in(I1));
                I2 = IC(Idx_in(I2));
                
           
                projlims(1) = X2(I1);    
               
                
             
                projlims(2) = X2(I2);
         
                
                
            end
            
        end
    else
        warning('not configured for dim other than 2 right now')
    end
end
    

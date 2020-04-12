function plot_data = plot_arrow(p,varargin)
    if mod(length(varargin),2) == 1
        p_start = p ;
        p_end = varargin{1} ;
        
        if length(varargin) > 1
            varargin = varargin(2:end) ;
        else
            varargin = {} ;
        end
    else
        p_start = zeros(3,1) ;
        p_end = p ;
    end
    
    % get the arrow-related keywords from varargin
    patch_args_in = {} ;
    plot_args_in = {} ;
    for idx = 1:2:length(varargin)
        switch lower(varargin{idx})
            case {'shaft_width','head_length','head_width'}
                patch_args_in = [patch_args_in, varargin(idx:idx+1)] ;
            otherwise
                plot_args_in = [plot_args_in, varargin(idx:idx+1)] ;
        end
    end
    
    % get arrow plot info
    [F,V] = make_arrow_for_patch(p_start,p_end,patch_args_in{:}) ;
    
    % make patch
    if nargout > 0
        plot_data = patch('Faces',F,'Vertices',V,plot_args_in{:}) ;
    else
        patch('Faces',F,'Vertices',V,plot_args_in{:}) ;
    end
end
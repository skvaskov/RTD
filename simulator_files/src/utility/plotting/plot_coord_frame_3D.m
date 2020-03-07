function plot_data = plot_coord_frame_3D(R,varargin)
% plot_data = plot_coord_frame_3D(orientation,varargin)
%
% Given a pose in SE(3) as a rotation matrix (3x3) and a position (3x1),
% plot a right-handed coordinate frame representing the pose. The default
% is to plot the local x-axis in red, y in green, and z in blue, each with
% length 1. This function also returns the figure handle and handles to the
% plot data if more output arguments are specified.
%
% USAGE:
% Default usage assumes p = (0,0,0):
%    plot_coord_frame_3D(R)
%
% Plot the coordinate frame centered at p:
%    plot_coord_frame_3D(R,p)
%
% Use keyword options:
%    plot_coord_frame_3D(R,p,'option1',option1,'option2',option2,...)
%
% Use keyword with p = (0,0,0):
%    plot_coord_frame_3D(R,'option1',option1,'option2',option2,...)
%
% KEYWORD OPTIONS:
% The plotting options are the same as the options for plot3, plus the
% following that are specific to this function:
%
% 'Colors' - takes a 3-by-3 matrix where each row specifies the color of a
%            coordinate vector in the order x-y-z
%
% 'Scale'  - specifies the length of the plotted unit vectors
%
% 'Data'   - takes in the plot_data object output by this function; this
%            is used to update the plots instead of completely refreshing
%            the plot (which causes a jittery appearance)

    args_list = {} ;

    switch length(varargin)
        case 0
            p = zeros(3,1) ;
        otherwise
            if ~ischar(varargin{1})
                p = varargin{1} ;
                arg_in_idxs = 2:2:length(varargin) ;
            else
                p = zeros(3,1) ;
                arg_in_idxs = 1:2:length(varargin) ;
            end

            for idx = arg_in_idxs
                switch varargin{idx}
                    case 'Colors'
                        c = varargin{idx+1} ;
                    case 'Scale'
                        s = varargin{idx+1} ;
                    case 'Data'
                        plot_data = varargin{idx+1} ;
                    otherwise
                        args_list = [args_list, varargin(idx:idx+1)] ;
                end
            end
    end

    % set up default variables
    if ~exist('c','var')
        c = eye(3) ;
    end

    if ~exist('s','var')
        s = 1 ;
    end

    % create vectors to plot
    e1 = [p, s.*R(:,1) + p] ;
    e2 = [p, s.*R(:,2) + p] ;
    e3 = [p, s.*R(:,3) + p] ;

    % start new plot if none is available
    if ~exist('plot_data','var')

        hold_check = ishold ;

        if ~hold_check
            hold on
        end

        % plot 'em!
        e1_data = plot3(e1(1,:),e1(2,:),e1(3,:),'Color',c(1,:),args_list{:}) ;
        e2_data = plot3(e2(1,:),e2(2,:),e2(3,:),'Color',c(2,:),args_list{:}) ;
        e3_data = plot3(e3(1,:),e3(2,:),e3(3,:),'Color',c(3,:),args_list{:}) ;

        if ~hold_check
            hold off
        end

        % create output
        if nargout > 0
            plot_data.e1 = e1_data ;
            plot_data.e2 = e2_data ;
            plot_data.e3 = e3_data ;
        end
    else
        % get data
        e1_data = plot_data.e1 ;
        e2_data = plot_data.e2 ;
        e3_data = plot_data.e3 ;

        % update axes
        e1_data.XData = e1(1,:) ;
        e1_data.YData = e1(2,:) ;
        e1_data.ZData = e1(3,:) ;

        e2_data.XData = e2(1,:) ;
        e2_data.YData = e2(2,:) ;
        e2_data.ZData = e2(3,:) ;

        e3_data.XData = e3(1,:) ;
        e3_data.YData = e3(2,:) ;
        e3_data.ZData = e3(3,:) ;

        % update plot_data
        plot_data.e1 = e1_data ;
        plot_data.e2 = e2_data ;
        plot_data.e3 = e3_data ;
    end
end
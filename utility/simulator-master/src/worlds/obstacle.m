classdef obstacle < handle
% Class: obstacle
%
% The obstacle superclass, which should be inherited by specific obstacle
% types. This obstacle can be static or dynamic, and 2-D or 3-D.
    
properties
    dimension = 2;% 2-D or 3-D
    
    position      % array of positions (as column vectors) over time; if
                  % this has only one column, then the obstacle is static
    
    time = [] ;   % times corresponding to the obstacle's position; if this
                  % is empty, then the obstacle is static
    
    plot_data     % a structure that stores the output of the plot or plot3
                  % functions in its "body" field, so that plots can update
                  % smoothly

    % plotting parameters
    plot_face_color = [1 0 0] ;
    plot_edge_color = [0 0 0] ;
    plot_face_opacity = 0.20 ;
    plot_edge_opacity = 0.75 ;
end

methods
%% constructor
function O = obstacle(varargin)
    O = parse_args(O,varargin{:}) ;
    if isempty(O.plot_data)
        O.plot_data.body = [] ;
    end
end

%% reset
function reset(O)
    O.position = [] ;
    O.time = [] ;
    O.plot_data.body = [] ;
end
    
%% utility
function vdisp(O,s,l)
% Display a string s if the message's verbose level l is greater
% than or equal to the planner's verbose level.
    if nargin < 3
        l = 1 ;
    end
    if O.verbose >= l
        if ischar(s)
            disp(['        O: ',s])
        else
            disp('        O: String not provided!')
        end
    end
end

%% plotting
function plot(O)
    % plot the obstacle's positions by default; this should probably be
    % overwritten in any subclass
    if ~isempty(O.position)
        XData = O.position(1,:) ;
        YData = O.position(2,:) ;
        if O.dimension == 3
            ZData = O.position(3,:) ;
        end
        
        if check_if_plot_is_available(O,'body')
            O.plot_data.body.XData = XData ;
            O.plot_data.body.YData = YData ;
            if O.dimension == 3
                O.plot_data.body.ZData = ZData ;
            end
        else
            switch O.dimension
                case 2
                    data = plot(O.position(1,:),O.position(2,:),'r') ;
                case 3
                    data = plot3(O.position(1,:),O.position(2,:),O.position(3,:),'r') ;
            end
            O.plot_data.body = data ;
        end
    end
end
end
end
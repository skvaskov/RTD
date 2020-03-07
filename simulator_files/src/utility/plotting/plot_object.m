function plot_object(obj,data,plot_data_fieldname,varargin)
% plot_object(obj,data,plot_data_fieldname,'property',value,...)
%
% Given an object for the simulator framework, plot the given data and
% update the object's corresponding plot_data field.
%
% The user can pass in the usual keyword/value arguments expected by the
% default plot or patch functions as additional arguments in. If the input
% data is a structure, it is assumed to have a 'faces' field and a
% 'vertices' field, and is passed to the patch function.
%
% Example: Suppose A is an agent instance and Z is A's trajectory as a
% 2-by-N array. Then we can plot the trajectory with
%   plot_object(A,Z,'trajectory','b--')
%
% Author: Shreyas Kousik
% Created: 3 Nov 2019
% Updated: 4 Nov 2019

    % check if plot is already up
    plot_available = check_if_plot_is_available(obj,plot_data_fieldname) ;

    % depending on the input data type, plot or patch
    if isa(data,'struct')
        % get the faces and vertices
        F = getfieldi(data,'faces') ;
        V = getfieldi(data,'vertices') ;

        if plot_available
            obj.plot_data.(plot_data_fieldname).Faces = F ;
            obj.plot_data.(plot_data_fieldname).Vertices = V ;
        else
            obj.plot_data.(plot_data_fieldname) = patch('Faces',F,...
                'Vertices',V,...
                varargin{:}) ;
        end
    else
        % get the size of the input data
        [r,~] = size(data) ;

        if plot_available
            obj.plot_data.(plot_data_fieldname).XData = data(1,:) ;
            obj.plot_data.(plot_data_fieldname).YData = data(2,:) ;

            if r == 3
                obj.plot_data.(plot_data_fieldname).ZData = data(3,:) ;
            end
        else
            obj.plot_data.(plot_data_fieldname) = plot_path(data,varargin{:}) ;
        end
    end
end

function value = getfieldi(S,field)
% from: https://stackoverflow.com/questions/28179401/matlab-get-value-from-a-struct-with-mixed-capitalization-of-field-names
    names   = fieldnames(S);
    isField = strcmpi(field,names);

    if any(isField)
        value = S.(names{isField});
    else
        value = [];
    end
end
function simulator_object = clear_plot_data(simulator_object,fieldname)
    % simulator_object = clear_plot_data(simulator_object)
    % simulator_object = clear_plot_data(simulator_object,fieldname)
    %
    % Clear the plot_data structure of the provided simulator object, which
    % can be an agent, world, or planner. This iterates through the
    % fieldnames of plot_data and sets each one to an empty vector, so
    % that, the next time the plot() method is called on that object, it
    % plots everything fresh (as opposed to attempting to update the plot,
    % which is much faster but requires a figure to already be open).
    %
    % The optional second argument clears just the plot data for the given
    % fieldname.
    
    % check if the input is a world
    world_check = isa(simulator_object,'world') ;

    % get fields to clear
    if nargin < 2
        F = fieldnames(simulator_object.plot_data) ;
    else
        F = {fieldname} ;
        world_check = world_check && strcmpi(fieldname,'obstacles') ;
    end

    % clear the plot data
    for idx = 1:length(F)
        simulator_object.plot_data.(F{idx}) = [] ;
    end

    % if the object is a world, also try to clear the obstacles' plot data
    if world_check
        O = simulator_object.obstacles ;
        
        if iscell(O)
            for idx = 1:length(O)
                o = O{idx} ;
                try
                    o = clear_plot_data(o) ;
                    O{idx} = o ;
                catch
                    warning('Unable to clear obstacle plot data!')
                end
            end

            W.obstacles = O ;
        end
    end
end
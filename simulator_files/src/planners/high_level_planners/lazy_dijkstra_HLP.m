classdef lazy_dijkstra_HLP < high_level_planner
    properties
        % graph growth
        N_new_nodes_per_iteration = 200 ;
        N_nodes_total = 100000 ;
        d_connect_threshold = 0.1 ; % m
        
        % graph data
        nodes
        adjacency_matrix
        best_path_nodes
        best_path_idxs
        best_path_distances
        
        % agent info
        agent_sensor_radius = 1 ;
        agent_position_indices ;
    end
    
    methods
        function HLP = lazy_dijkstra_HLP(varargin)
            HLP@high_level_planner(varargin{:}) ;
        end
        
        function setup(HLP,agent_info,world_info)
            % create the adjacency matrix
            N = HLP.N_nodes_total ;
            HLP.adjacency_matrix = sparse(N,N) ;
            
            % set up HLP info
            HLP.agent_sensor_radius = agent_info.sensor_radius ;
            HLP.agent_position_indices = agent_info.position_indices ;
            HLP.dimension = world_info.dimension ;
            
            % set up the nodes
            HLP.nodes = nan(HLP.dimension,N) ;
            HLP.nodes(:,1) = agent_info.state(HLP.agent_position_indices) ;
        end
        
        function get_waypoint(HLP,agent_info,world_info,lookahead_distance)
            
            % get the agent's current location
            z = agent_info.state(HLP.agent_position_indices) ;
            
            % TO DO: check that we haven't maxed out the number of nodes!
            
            % create new nodes within the sensor radius
            d = HLP.dimension ;
            N = HLP.N_new_nodes_per_iteration ;
            R = HLP.agent_sensor_radius ;
            new_node_directions = make_unit_length(rand_range(-1,1,[],[],d,N)) ;
            new_node_distances = repmat(rand_range(0,R,[],[],1,N),2,1) ;
            new_nodes = new_node_distances .* new_node_directions + repmat(z,1,N) ;
            
            % get distances between the nodes
            error('left off here')
        end
    end
end
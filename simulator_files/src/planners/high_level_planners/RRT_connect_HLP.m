classdef RRT_connect_HLP < RRT_HLP
% Class: RRT_connect_HLP < RRT_HLP
%
% This implements RRT-Connect as a high-level planner for the simulator
% framework. For now, it only works for 2D worlds/obstacles.
%
% Authors: Shreyas Kousik
% Created: 6 Nov 2019
% Updated: -
    
    methods
        %% constructor
        function HLP = RRT_connect_HLP(varargin)
            new_node_growth_distance = 0.25 ;
            HLP@RRT_HLP('new_node_growth_distance',new_node_growth_distance,...
                varargin{:}) ;
            HLP.plot_data.latest_edge = [] ;
        end
        
        %% extend
        function extend_success_flag = extend(HLP,agent_info,world_info,rand_node)
            % extend as far as possible in direction of rand node
            [nearest_node,nearest_node_distance,nearest_node_index] = HLP.find_nearest_node(rand_node) ;
            dir_new = (rand_node - nearest_node)./nearest_node_distance ;
            
            % we check for collision from the first node each time, since
            % we could overshoot obstacles otherwise
            first_node = nearest_node ;
            
            % pessimism
            extend_success_flag = false ;
            
            while true
                NNGD = HLP.new_node_growth_distance ;
                new_node = nearest_node + NNGD.*dir_new ;
                
                % make sure the new node is in bounds and within the sensor
                % radius of the agent
                if ~check_point_in_bounds(new_node,HLP.bounds) || ...
                        (vecnorm(new_node - agent_info.position(:,end)) > agent_info.sensor_radius) ;
                    break
                end
                
                extend_success_flag = HLP.edge_feasibility_check(first_node,new_node,world_info) ;
                
                if extend_success_flag
                    HLP.add_node_to_tree(new_node,nearest_node_index) ;
                else
                    break
                end
                
                if HLP.plot_while_growing_tree_flag
                    plot_object(HLP,[first_node, new_node],'latest_edge','b')
                    drawnow()
                end
                
                nearest_node = new_node ;
                nearest_node_index = HLP.N_nodes ;
            end
        end
    end
end
classdef lane_HLP < high_level_planner
methods
    function HLP = lane_HLP(varargin)
        HLP@high_level_planner(varargin{:}) ;
    end

    function waypoint = get_waypoint(HLP,agent_info,world_info,lookahead_distance)
        if nargin < 4
            lookahead_distance = HLP.default_lookahead_distance ;
        end

        lane_centerlines = world_info.lane_center_lines;
        
        
        z = agent_info.position(:,end) ;
        zpos = z(agent_info.position_indices);
        Nlanes = length(lane_centerlines);
        
         d = NaN(Nlanes,1);
        for i = 1:Nlanes
            d(i) =  dist_point_to_polyline(zpos, world_info.lane_center_lines{i});
        end
        
        [~,lane_idx] = min(d);
        
        cur_dist = dist_along_polyline(zpos,world_info.lane_center_lines{lane_idx});
        
        
        cum_dists = dist_polyline_cumulative(world_info.lane_center_lines{lane_idx}(1:2,:));
        
        waypoint_dist = cur_dist+lookahead_distance;
        
        waypoint = interp1(cum_dists',world_info.lane_center_lines{lane_idx}',waypoint_dist,'linear','extrap')';
        
        if ~HLP.waypoints_include_heading
            waypoint = waypoint(1:2);
        end

        % update current waypoints
        HLP.current_waypoint = waypoint ;
        HLP.waypoints = [HLP.waypoints, waypoint] ;
        HLP.current_waypoint_index = HLP.current_waypoint_index + 1 ;
    end
end
end

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
         waypoint_candidate = NaN(size(world_info.lane_center_lines{1},1),Nlanes);
         obstacle_present = false(Nlanes,1);
         
         %get distance too and check if there is obstacle in lanes
        for i = 1:Nlanes
            
            %find distance to lane 
            d(i) =  dist_point_to_polyline(zpos, world_info.lane_center_lines{i});
            
            %get distance along lane and waypoint
            cur_dist = dist_along_polyline(zpos,world_info.lane_center_lines{i});
            cum_dists = dist_polyline_cumulative(world_info.lane_center_lines{i}(1:2,:));
            waypoint_dist = cur_dist+lookahead_distance;
            
            waypoint_polyline = interp1(cum_dists',world_info.lane_center_lines{i}',[cur_dist;waypoint_dist],'linear','extrap')';
            
            waypoint_candidate(:,i) = waypoint_polyline(:,end);
            
            %coarsely check if polyline intersects obstale
            if ~isempty(world_info.obstacles)
                if ~isempty(polyxpoly(world_info.obstacles(1,:),world_info.obstacles(2,:),waypoint_polyline(1,:),waypoint_polyline(2,:)))
                    obstacle_present(i) = true;
                end
            end
            
        end
        
        % find lane you are closest to
        [~,lane_idx] = min(d);
        
        if ~all(obstacle_present) && obstacle_present(lane_idx)
            Lfree = ~obstacle_present;
            idxfree = find(Lfree);
            dfree = d(Lfree);
            [~,lane_idx_free] = min(dfree);
            lane_idx = idxfree(lane_idx_free);  
        end
      
        
        if ~HLP.waypoints_include_heading
            waypoint = waypoint_candidate(1:2,lane_idx);
        else
            waypoint = waypoint_candidate(:,lane_idx);
        end

        % update current waypoints
        HLP.current_waypoint = waypoint ;
        HLP.waypoints = [HLP.waypoints, waypoint] ;
        HLP.current_waypoint_index = HLP.current_waypoint_index + 1 ;
    end
end
end

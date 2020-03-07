classdef straight_line_HLP < high_level_planner
methods
    function HLP = straight_line_HLP(varargin)
        HLP@high_level_planner(varargin{:}) ;
    end

    function waypoint = get_waypoint(HLP,agent_info,world_info,lookahead_distance)
        if nargin < 4
            lookahead_distance = HLP.default_lookahead_distance ;
        end

        g = world_info.goal ;
        z = agent_info.position(:,end) ;
        dir_des = g - z ;
        dir_des = dir_des./norm(dir_des) ;
        
        % adjust lookahead distance to not overshoot goal
        dist_to_goal = norm(g - z) ;
        if lookahead_distance > dist_to_goal
            lookahead_distance = dist_to_goal ;
        end
        
        waypoint = lookahead_distance.*dir_des + z ;
        
        if HLP.waypoints_include_heading
            waypoint(3) = atan2(dir_des(2),dir_des(1)) ;
        end

        % update current waypoints
        HLP.current_waypoint = waypoint ;
        HLP.waypoints = [HLP.waypoints, waypoint] ;
        HLP.current_waypoint_index = HLP.current_waypoint_index + 1 ;
    end
end
end

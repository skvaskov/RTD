classdef two_lane_road_static < static_box_world
    properties
       lane_center_lines
       road_upper_boundary
       road_lower_boundary
       obstacle_lat_spacing = [-1;1] %[min; max] distance from lane center
       obstacle_long_spacing  = [40;80] %[min; max] obstacle distance from eachother
       lane_width = 4;
       road_length = 250;
       merge_zone = [NaN,NaN]
       bound_space = 8;
    end

    methods
function W = two_lane_road_static(varargin)
    W@static_box_world('start',[0;0;0],'N_obstacles',0)
    
    default_size_bounds = [3.5,5.5;1.5,2.5];
    

    W = parse_args(W,'obstacle_size_bounds',default_size_bounds,varargin{:});

    W.bounds = (W.lane_width+W.bound_space)*[-1 1 -1 1]+[0 W.road_length 0 0];
    
    merge_length = W.merge_zone(2)-W.merge_zone(1);
    
    if W.merge_zone(2) < W.road_length && merge_length > 0
        ypts = linspace(-W.lane_width,0,ceil(merge_length));
        acoeff = (-W.lane_width^2+(merge_length))/W.lane_width;
        xpts =  merge_zone(2)- ypts.*(ypts-acoeff);
     
        W.road_upper_boundary = [0 W.road_length;W.lane_width,W.lane_width];
         W.road_lower_boundary = [0 W.merge_zone(1) xpts W.road_length; -W.lane_width,-W.lane_width,ypts,0];
         
         W.lane_center_lines={[0,W.merge_zone(1);-W.lane_width/2,-W.lane_width/2],...
        [0,W.road_length;W.lane_width/2,W.lane_width/2]};
    
    else
        W.road_upper_boundary = [0 W.road_length;W.lane_width,W.lane_width];
        W.road_lower_boundary = [0 W.road_length;-W.lane_width,-W.lane_width];
        
            W.lane_center_lines={[0,W.road_length;-W.lane_width/2,-W.lane_width/2],...
        [0,W.road_length;W.lane_width/2,W.lane_width/2]};
    
    end
    
   
    W.goal = [W.road_length;0];
    W.reset();
    % call setup to generate obstacles if Nobstacles is > 0 but no
    % obstacles were setup
    if W.N_obstacles > 0 && isempty(W.obstacles_unseen) && isempty(W.obstacles_seen)
        W.setup() ;
    end
    
    % get room bounds
    B = W.bounds ;
    xlo = B(1) ; xhi = B(2) ; ylo = B(3) ; yhi = B(4) ;
    W.bounds_as_polyline = [[xlo,xhi;ylo,ylo],NaN(2,1),[xlo,xhi;yhi,yhi]];
    
    % check for mapping toolbox
    try
        polyxpoly([],[],[],[]) ;
    catch
        error(['Please make sure the Mapping Toolbox is installed',...
            ' so you can use polyxpoly.'])
    end
    
end
%%
% obs paths are an Nobs x 1 cell array containing pose positions for
% the obstacles to move along
function setup(W)

    
    % generate obstacles around room
    N_obs = W.N_obstacles ;
    
    mean_long_spacing=mean(W.obstacle_long_spacing);
    diff_long_spacing=abs((W.obstacle_long_spacing(2)-W.obstacle_long_spacing(1)));

    
    lastlong= 0 + mean_long_spacing;
    
    O = nan(2, 6*N_obs) ; % preallocate obstacle matrix

    
    
    if W.start(2)>0
        j=1;
    else
        j=0;
    end
    
    for idx = 1:6:(6*N_obs-1)
        
        %obstacle center in X
        spacing=min(W.obstacle_long_spacing)+diff_long_spacing*rand;

        cx=lastlong+spacing;
        
        
        % obstacle rotation
        r= abs(wrapToPi(diff(W.obstacle_rotation_bounds)))*rand;
        R = [cos(r) sin(r) ; -sin(r) cos(r)] ;
        
        obstacle_size = (W.obstacle_size_bounds(:,2)-W.obstacle_size_bounds(:,1)).*rand(2,1)+W.obstacle_size_bounds(:,1);
        % obstacle base
        o = [-obstacle_size(1)/2  obstacle_size(1)/2 obstacle_size(1)/2 -obstacle_size(1)/2 -obstacle_size(1)/2 ;
            -obstacle_size(2)/2 -obstacle_size(2)/2 obstacle_size(2)/2  obstacle_size(2)/2 -obstacle_size(2)/2 ] ;
        
        if mod(j,2)
            c=[cx;abs(diff(W.obstacle_lat_spacing))*rand+min(W.obstacle_lat_spacing)-W.lane_width/2];
        else
            c=[cx;abs(diff(W.obstacle_lat_spacing))*rand+min(W.obstacle_lat_spacing)+W.lane_width/2];
        end
        
        lastlong=c(1);
        O(:,idx:idx+4) = R*o + repmat(c,1,5) ;
        
        j = j+1;
    end
    if N_obs >0
        if isnan(O(1,end))
            O = O(:,1:end-1) ;
        end
    end
    
    W.obstacles = O ;
    W.obstacles_seen = [] ;
    W.N_obstacles = N_obs ;
    
    
    W.obstacles_unseen = W.obstacles ;
    
    % set up plot data
    W.plot_data.obstacles_seen = [] ;
    W.plot_data.obstacles_unseen = [] ;
    W.plot_data.start = [] ;
    W.plot_data.goal = [] ;
    W.plot_data.goal_zone = [] ;
    W.plot_data.bounds = [] ;
end

 
    %% goal check
    function out = goal_check(W,agent_info)
        % Method: out = goal_check(agent_info)
        %
        % Checks if the agent's center of mass is within W.goal_radius of
        % W.goal in the 2-norm.
        z = agent_info.position(:,end);
        
        out = z(1) > (W.road_length - W.goal_radius) && abs(z(2)) < W.lane_width;
    end
        
%%
function plot_at_time(W,t)
    hold on

    LineWidth=2.0;

    
    %plot boundary
    plot(W.road_lower_boundary(1,:),W.road_lower_boundary(2,:),'k','LineWidth',LineWidth)
    plot(W.road_upper_boundary(1,:),W.road_upper_boundary(2,:),'k','LineWidth',LineWidth)
    
    plot([0 ,W.road_length],[0 0],'--','Color',[0.9290, 0.6940, 0.1250],'LineWidth',LineWidth)
    
    plot_at_time@static_box_world(W,t)
   
end

function reset(W)

    W.current_time = 0 ;
    
    mult = randi(2)-3;
    
    W.start = [0;1^mult*W.lane_width/2;0];
    
end
        
%%
function plot(W)
    hold on

    LineWidth=2.0;

    
    %plot boundary
    plot(W.road_lower_boundary(1,:),W.road_lower_boundary(2,:),'k','LineWidth',LineWidth)
    plot(W.road_upper_boundary(1,:),W.road_upper_boundary(2,:),'k','LineWidth',LineWidth)
    
    plot([0 ,W.road_length],[0 0],'--','Color',[0.9290, 0.6940, 0.1250],'LineWidth',LineWidth)
    
    plot@static_box_world(W)
   
end

%% get workd info
function W_info = get_world_info(W,agent_info,P)
    W_info = get_world_info@static_box_world(W,agent_info,P);

    W_info.lane_width = W.lane_width;
    W_info.lane_center_lines = W.lane_center_lines;
    W_info.road_upper_boundary = W.road_upper_boundary;
    W_info.road_lower_boundary = W.road_lower_boundary;
end
    end

end

%% description
% This script generates Fig. 5 in the paper, which demonstrated the lane change parameterization 

% Author: Sean Vaskov
% Created: 10 Apr 2020

%% user parameters
save_pdf_flag = true ;
sub_figure = 'b';

%%
if strcmpi(sub_figure,'a')
agent_pose = [1.5;-0.3;0.0;0;0];
w0_des_light = linspace(-0.1,0.6,7); 
waypoint = [6.0;0.3]; 
else
    agent_pose =  [2.5;-0.15;0.25;0;0] ;
w0_des_light =  linspace(-0.7,0.0,7);
waypoint = [6.5;0.3];
end

close all
fh = figure;

plot_footprint_opacity = 1;

waypoint_marker_size = 10;


ftprint_color = [0.8 0.8 1];
yplotlimits = [-1 1];
xplotlimits = [1 7];


light_traj_color = [0.25 0.75 1];
dark_traj_color = [0 0 1];

v_des = 2;

t_f = 2;
%% create world
N_obstacles = 0 ;
road_length = 20;
lane_width =  0.6;
bound_space = 0.4;

W = two_lane_road_static('lane_width',lane_width,'bound_space',bound_space,'N_obstacles',N_obstacles,'road_length',road_length);
W.bounds_as_polyline = NaN(2,5);
W.plot()

plot(waypoint(1,:),waypoint(2,:),'k*','MarkerSize',waypoint_marker_size,'LineWidth',1)

%% plot light trajectories
for i = 1:length(w0_des_light)
    
[~,~,Z] = make_rover_desired_trajectory(t_f,w0_des_light(i),-agent_pose(3),v_des);
Z = rotation_matrix_2D(agent_pose(3))*Z(1:2,:)+agent_pose(1:2);

plot(Z(1,:),Z(2,:),'Color',light_traj_color,'LineWidth',1.0)

end


%find and plot dark trajectory
w0_max = max(w0_des_light);
w0_min = min(w0_des_light);
y_dist = Inf;

for w0_test = linspace(w0_min,w0_max,100)
    [~,~,Ztemp] = make_rover_desired_trajectory(t_f,w0_test,-agent_pose(3),v_des);
    Ztemp = rotation_matrix_2D(agent_pose(3))*Ztemp(1:2,:)+agent_pose(1:2);
    
    if abs(Ztemp(2,end) - waypoint(2)) < y_dist
        y_dist = abs(Ztemp(2,end) - waypoint(2));
        Z = Ztemp;
    end
end
plot(Z(1,:),Z(2,:),'Color',dark_traj_color,'LineWidth',1.0)


%% create dummy agent and plot
A = RoverAWD('plot_footprint_opacity',plot_footprint_opacity,'plot_footprint_color',ftprint_color);
A.reset(agent_pose);
A.plot();

axis equal
  set(gca,'Layer','Top',...
      'Box',    'on',...
      'TickDir', 'in',...
      'Xminortick', 'off',...
      'Yminortick', 'off',...
      'YGrid',  'off',...
      'XColor', [0 0 0],...
      'Ycolor', [0 0 0],...
      'Xtick',  linspace(xplotlimits(1),xplotlimits(2),5),...
       'Ytick',  linspace(yplotlimits(1),yplotlimits(2),5),...
      'Linewidth', 1.0 );
  set(gca,'Fontsize',15);
  set(gca,'fontname','Times New Roman')
  xlabel('x [m]')
 xlim(xplotlimits)
 ylim(yplotlimits)
  ylabel('y [m]')
  
  set_plot_linewidths(2)
  
  ax = gca;
  ax.YAxis.TickLabelFormat = '%.1f';
  ax.XAxis.TickLabelFormat = '%.0f';
  
  if save_pdf_flag
    save_figure_to_pdf(fh,['figure5_lanechange_',sub_figure,'.pdf'])
end

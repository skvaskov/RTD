%% description
% This script generates Fig. 16 in the paper, for the rrt and NMPC planners
%
%
% Author: Sean Vaskov
% Created: 10 Apr 2020

%% user parameters
save_pdf_flag = true ;
planner_name = 'RRT'; %set to 'RRT' or 'NMPC'
trial = 187; %% 22 for planners stop (d,f), 187 for trial where RRT crashes (c,d)
xplotlimits = [-1 19]; %[-1 15] for trial 22, [-1 19] for trial 187
time_step = 1.5; %vector of timesteps 
max_time = 10; %dont plot afeter this time

%%
close all
fh = figure;
%trial 187 for success, xplotlimits = [3 23];yplotlimits = [-1 1]; textpos
%[21.5,0.6]
%trial 22 for failuer, xplotlimits = [-1 15];textpos = [13.5,0.6];
load('rover_simulation_worlds.mat')
load(['experiment_2/rover_data/rover_experiment_2_summary_world_',num2str(trial,'%04d'),'.mat'])

frs_color = [0,0.75,0.25];
ftprint_color = [0.8 0.8 1];
yplotlimits = [-1 1];
obs_size = 1;
obstacle_light_line_color = [1, 0.74,0.53];

if strcmpi(planner_name,'rrt')
summary = summary(2);
else
summary = summary(3); 
end



W_all{trial}.bounds_as_polyline = NaN(2,5);
W_all{trial}.plot;



A = RoverAWD();
max_heading = 0.6;
buffer = 0;

rotated_vertices = [cos(max_heading) -sin(max_heading);sin(max_heading) cos(max_heading)]*A.footprint_vertices;

L = [min(rotated_vertices(1,:)),max(rotated_vertices(1,:))];
W = [min(rotated_vertices(2,:)),max(rotated_vertices(2,:))];

rotated_vertices = [cos(max_heading) -sin(max_heading);sin(max_heading) cos(max_heading)]'*A.footprint_vertices;

L = [min([L(1),rotated_vertices(1,:)]),max([L(2),rotated_vertices(1,:)])];
W = [min([W(1),rotated_vertices(2,:)]),max([W(2),rotated_vertices(2,:)])];

obstacle_buffer = buffer*[-1,1;-1,1]+[L;W];

buffer_stencil = [obstacle_buffer(1,[1 2 2 1 1]);...
    obstacle_buffer(2,[1 1 2 2 1])];

buffer_stencil = repmat([buffer_stencil,NaN(2,1)],[1 3]);

O = [W_all{trial}.obstacles,NaN(2,1)]+buffer_stencil;

plot(summary.trajectory(1,:),summary.trajectory(2,:),'b','LineWidth',1.0)
plot(W_all{trial}.obstacles(1,:),W_all{trial}.obstacles(2,:),'r','LineWidth',obs_size)
plot(O(1,:),O(2,:),'Color',obstacle_light_line_color,'LineWidth',obs_size)

max_time =min(max_time,max(summary.total_simulated_time));
time_vector = unique([0:time_step:max_time,max_time]);

for t = time_vector
    
    z_act = interp1(summary.total_simulated_time',summary.trajectory',t)';
    %plot footprint
     V_fp = z_act(1:2)+rotation_matrix_2D(z_act(3))*A.footprint_vertices;
            V_arrow = z_act(1:2)+rotation_matrix_2D(z_act(3))*A.arrow_vertices;
            
      
            hold on
            fill(V_fp(1,:),V_fp(2,:),ftprint_color,...
                'EdgeColor',A.plot_footprint_edge_color,...
                'FaceAlpha',1,...
                'EdgeAlpha',A.plot_footprint_edge_opacity) ;
            
            % plot arrow on footprint
            fill(V_arrow(1,:),V_arrow(2,:),A.plot_arrow_color,...
                'EdgeColor',A.plot_arrow_color,...
                'FaceAlpha',1,...
                'EdgeAlpha',A.plot_arrow_opacity) ;
   
      
end

textpos = [xplotlimits(2)-0.2,1];
text(textpos(1),textpos(2),planner_name, 'EdgeColor','none','Margin',0.1,...
  'HorizontalAlignment', 'right','VerticalAlignment','top','FontSize',14,'BackgroundColor','w')

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
       'Ytick',  linspace(yplotlimits(1),yplotlimits(2),3),...
      'Linewidth', 1.0 );
  set(gca,'Fontsize',15);
  set(gca,'fontname','Times New Roman')
  xlabel('x [m]')
 xlim(xplotlimits)
 ylim(yplotlimits)
  ylabel('y [m]')
  
  ax = gca;
  ax.YAxis.TickLabelFormat = '%.0f';
  ax.XAxis.TickLabelFormat = '%.0f';

 set_plot_linewidths(1.25)
 
if save_pdf_flag
    save_figure_to_pdf(fh,[planner_name,'_rover_trial_',num2str(trial),'.pdf'])
end

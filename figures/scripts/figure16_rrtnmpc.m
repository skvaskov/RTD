close all
clear

%trial 187 for success, xplotlimits = [3 23];yplotlimits = [-1 1]; textpos
%[21.5,0.6]
%trial 22 for failuer, xplotlimits = [-1 15];textpos = [13.5,0.6];
trial = 187;
load('rover_simulation_worlds.mat')
load(['experiment_2/rover_data/rover_experiment_2_summary_world_',num2str(trial,'%04d'),'.mat'])

frs_color = [0,0.75,0.25];
ftprint_color = [0.8 0.8 1];
xplotlimits = [-1 19];
yplotlimits = [-1 1];
obs_size = 1;

summary = summary(3);
planner_name = 'NMPC';
textpos = [17,0.6];
time_vector = linspace(0,10,10);
W_all{trial}.plot;

A = RoverAWD();


plot(summary.trajectory(1,:),summary.trajectory(2,:),'b','LineWidth',1.0)
plot(W_all{trial}.obstacles(1,:),W_all{trial}.obstacles(2,:),'r','LineWidth',obs_size)
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

text(textpos(1),textpos(2),planner_name,'FontSize',12,'BackgroundColor','w')

axis equal
  set(gca,'Layer','Top',...
      'Box',    'on',...
      'TickDir', 'in',...
      'Ticklength', [0.005 0.005],...
      'Xminortick', 'off',...
      'Yminortick', 'off',...
      'YGrid',  'off',...
      'XColor', [0 0 0],...
      'Ycolor', [0 0 0],...
      'Xtick',  linspace(xplotlimits(1),xplotlimits(2),5),...
       'Ytick',  linspace(yplotlimits(1),yplotlimits(2),5),...
      'Linewidth', 0.5 );
  set(gca,'Fontsize',10);
  set(gca,'fontname','Times New Roman')
  xlabel('x (m)')
 xlim(xplotlimits)
 ylim(yplotlimits)
  ylabel('y (m)')
  
  ax = gca;
  ax.YAxis.TickLabelFormat = '%.1f';
  ax.XAxis.TickLabelFormat = '%.0f';

   


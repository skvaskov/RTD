clear
close

RoverLLC = rover_PD_LLC('yaw_gain',5,'yaw_rate_gain',0.5);
A = RoverAWD('LLC',RoverLLC,'max_speed',3) ;

v0 = linspace(0,2,10);
distance = NaN(size(v0));

T = 4;
for i = 1:length(v0)
    A.reset([0;0;0;v0(i);0]);
    A.move(T,[0,T],[0,0;0,0])
    
    distance(i) = sqrt(A.state(1,end)^2+A.state(2,end)^2);
  
    
end

plot(v0,distance,'*','LineWidth',1.0,'MarkerSize',5)
xlabel('v_0 m/s')
ylabel('braking dist.')

 set(gca,'Layer','Top',...
      'Box',    'on',...
      'TickDir', 'in',...
      'Ticklength', [0.005 0.005],...
      'Xminortick', 'off',...
      'Yminortick', 'off',...
      'YGrid',  'off',...
      'XColor', [0 0 0],...
      'Ycolor', [0 0 0],...
      'Xtick',  linspace(min(v0),max(v0),5),...
       'Ytick',  linspace(min(distance),max(distance),5),...
      'Linewidth', 0.5 );
  set(gca,'Fontsize',12);
  set(gca,'fontname','Times New Roman')
  xlabel('v_0 (m/s)')

  ylabel('braking dist. (m)')
  
  ax = gca;
  ax.YAxis.TickLabelFormat = '%.2f';
  ax.XAxis.TickLabelFormat = '%.2f';
  

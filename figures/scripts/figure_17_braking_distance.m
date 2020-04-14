%% description
% This script generates Fig. 17 in the paper, which plots the braking
% distance of the Segway and Rover robots
%
% Author: Sean Vaskov
% Created: 10 Apr 2020

%% user parameters
save_pdf_flag = false ;
robot = 'rover'; %set to 'rover' or 'segway'
v0_range = [0.5 2]; 
N_sample = 10;
ymax = 1.5; %upper limit on y axis (leave empty and it will set to max(braking distance)
find_c_v = true; %find and plot linear bound on braking distance
T = 4; %time horizon (make large to ensure braking to stop)
%%
close all
fh = figure;
if strcmpi(robot,'rover')
    RoverLLC = rover_PD_LLC('yaw_gain',5,'yaw_rate_gain',0.5);
    A = RoverAWD('LLC',RoverLLC,'max_speed',3) ;
    load('rover_timing.mat')
else
    A = segway_agent();
    load('segway_timing.mat')
end

v0 = linspace(min(v0_range),max(v0_range),N_sample);

distance = NaN(size(v0));


for i = 1:length(v0)
    z0 = zeros(A.n_states,1);
    z0(A.speed_index) = v0(i);
    
    A.reset(z0);
    
    if strcmpi(robot,'rover')
        
        psi_end = 0;
        w0_des = 0 ;
        v_des = v0(i) ;
        [T_brk,U_brk,Z_brk] = make_rover_braking_trajectory(t_plan,t_f,t_stop,w0_des,psi_end,v_des) ;
      
    else
        
        w_des = 0 ;
        v_des = v0(i) ;
        t_stop = A.max_speed ./ A.max_accel ;
        [T_brk,U_brk,Z_brk] = make_segway_braking_trajectory(t_plan,t_stop,w_des,v_des) ;

    end
    
    if T_brk(end) < T
        T_brk = [T_brk,T];
        U_brk = [U_brk,zeros(A.n_inputs,1)];
        Z_brk = [Z_brk,Z_brk(:,end)];
    end
    
    A.move(T_brk(end),T_brk,U_brk,Z_brk)
    
    distance_trial = [0,cumsum(sqrt(diff(A.state(1,:)).^2+diff(A.state(2,:)).^2))];
    
    distance_t_tplan = match_trajectories(t_plan,A.time,distance_trial);
    idx_stopped = find(A.state(A.speed_index,:) < 0.01,1);
    
    distance(i) = distance_trial(idx_stopped) - distance_t_tplan;
    
   
    
end

if min(v0_range)~=0
    v0 = [0,v0];
    distance = [0,distance];
end

 

plot(v0,distance,'b*','LineWidth',2.0,'MarkerSize',5)
xlabel('v_0 m/s')
ylabel('braking dist.')

if find_c_v
    hold on
c_v = linprog(1, -v0(:),-distance);
c_v = ceil(c_v*10)/10;
plot(v0,c_v*v0,'r--','LineWidth',2.0)

end

if isempty(ymax)
    ymax = max(distance);
end

 set(gca,'Layer','Top',...
      'Box',    'on',...
      'TickDir', 'in',...
      'Xminortick', 'off',...
      'Yminortick', 'off',...
      'YGrid',  'off',...
      'XColor', [0 0 0],...
      'Ycolor', [0 0 0],...
      'Xtick',  linspace(min(v0),max(v0),5),...
       'Ytick',  linspace(min(distance),ymax,5),...
      'Linewidth', 1.0 );
  set(gca,'Fontsize',15);
  set(gca,'fontname','Times New Roman')
  xlabel('v_0 [m/s]')

  ylabel('braking dist. [m]')
  ylim([0 ymax])
  
  ax = gca;
  ax.YAxis.TickLabelFormat = '%.2f';
  ax.XAxis.TickLabelFormat = '%.2f';
  
if save_pdf_flag
    save_figure_to_pdf(fh,['figure17_',robot,'_braking_distance.pdf'])
end
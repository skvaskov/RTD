clear
close all
load('rover_reconstructed_deg10_frsdeg8_T1.5_v0_0.8_to_1.5_delta0_-0.05_to_0.05.mat')
full_file = load('rover_xy_FRS_full_T1.5_deg6_v0_0.8_to_1.5_delta0_-0.05_to_0.05.mat');

k_test = [0.5;-1;0.75]; %a [0.5;-1;0.75]

frs_color = [0,0.75,0.25];
ftprint_color = [0.8 0.8 1];
full_color = [0.25,0.5,0.25];

N_traj_samples = 10;

xplotlimits = [-0.75,3.75];
yplotlimits = [-0.75,1.25];


v_des = (v_des_max-v_des_min)/2*(k_test(3)+1)+v_des_min;
w0_des = (w0_des_max-w0_des_min)/2*(k_test(1)+1)+w0_des_min;
psi_end = (psi_end_max-psi_end_min)/2*(k_test(2)+1) + psi_end_min;

t_f = 2;

[Tref,Uref,Zref] = make_rover_desired_trajectory(t_f,w0_des,psi_end,v_des);

tvec = [0 0.75 1.5];


RoverLLC = rover_PD_LLC('yaw_gain',5,'yaw_rate_gain',0.5);
A = RoverAWD('LLC',RoverLLC,'max_speed',3) ;
hold on
for i = 1:N_traj_samples
    z0 = randRange([0;0;0;v0_min;delta0_min],[0;0;0;v0_max;delta0_max]);
    A.reset(z0);
    A.move(T,Tref,Uref,Zref);
    
    z_act = A.state;
    
    R_1 = rotation_matrix_2D(z_act(3,:));
    
    V1 = repmat(A.footprint_vertices(:,1),[1 length(A.time)]);
    V2 = repmat(A.footprint_vertices(:,2),[1 length(A.time)]);
    V3 = repmat(A.footprint_vertices(:,3),[1 length(A.time)]);
    V4 = repmat(A.footprint_vertices(:,4),[1 length(A.time)]);
    
    V1_act = reshape(R_1*(V1(:)),[2 length(A.time)])+z_act(A.position_indices,:);
    V2_act = reshape(R_1*(V2(:)),[2 length(A.time)])+z_act(A.position_indices,:);
    V3_act = reshape(R_1*(V3(:)),[2 length(A.time)])+z_act(A.position_indices,:);
    V4_act = reshape(R_1*(V4(:)),[2 length(A.time)])+z_act(A.position_indices,:);
    
    
   plot(V1_act(1,:),V1_act(2,:),'b')
   plot(V2_act(1,:),V2_act(2,:),'b')
   plot(V3_act(1,:),V3_act(2,:),'b')
   plot(V4_act(1,:),V4_act(2,:),'b')
    

    
        for idxplot = [1,length(A.time)]
            
            V_fp = z_act(1:2,idxplot)+rotation_matrix_2D(z_act(3,idxplot))*A.footprint_vertices;
            V_arrow = z_act(1:2,idxplot)+rotation_matrix_2D(z_act(3,idxplot))*A.arrow_vertices;
            
      
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
  
end

plot_2D_msspoly_contour(subs(w,k,k_test),z(1:2),1,'Scale',zscale(1:2),'Offset',-zoffset(1:2),'Color',frs_color,'LineWidth',1.5)
plot_2D_msspoly_contour(subs(full_file.FRS_polynomial,k,k_test),z(1:2),1,'Scale',zscale(1:2),'Offset',-zoffset(1:2),'Color',full_color,'LineWidth',1.5,'LineStyle','--')
    figure(1)
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
  ax.YAxis.TickLabelFormat = '%.2f';
  ax.XAxis.TickLabelFormat = '%.2f';
  

  
 box on


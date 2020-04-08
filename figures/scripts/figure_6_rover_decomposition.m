clear
close all
load('rover_reconstructed_deg10_frsdeg8_T1.5_v0_0.8_to_1.5_delta0_-0.05_to_0.05.mat')

k_test = [0.5; -1; 0.5];

frs_color = [0,   0.75,0.25];
box_color = [0.25, 0.5, 0.25];
x_color =   [0.1,  0.8,  0.5];
y_color =   [0.1,  0.8, 0.7];
ftprint_color = [0.8 0.8 1];

xplotlimits = [-0.5,3.25];
yplotlimits = [-0.5,1.0];
textpos = [-0.3,0.9];

tvec = [0 0.75 1.5];

N = 250;

%%

v_des = (v_des_max-v_des_min)/2*(k_test(3)+1)+v_des_min;
w0_des = (w0_des_max-w0_des_min)/2*(k_test(1)+1)+w0_des_min;
psi_end = (psi_end_max-psi_end_min)/2*(k_test(2)+1) + psi_end_min;

t_f = 2;

[Tref,Uref,Zref] = make_rover_desired_trajectory(t_f,w0_des,psi_end,v_des);




A= RoverAWD();
for i = 1:3
    tplot = tvec(i);
    hold on
    z_t = interp1(Tref',Zref',tplot)';
    
    p_t = z_t(A.position_indices) ;
    h_t = z_t(A.heading_index) ;
    R_t = rotation_matrix_2D(h_t) ;
    fp_t = A.footprint_vertices(:,1:end-1) ;
    N_fp = size(fp_t,2) ;
    V_fp = R_t*fp_t + repmat(p_t,1,N_fp) ;
    
    % make arrow for plot
    V_arrow = R_t*A.arrow_vertices + repmat(p_t,1,3) ;
    
    xlims = get_1D_msspoly_projection(msubs(-FRS_lyapunov_function_x,[t;k],[tplot/T;k_test]),z([1,3]),0,1,'Scale',zscale([1,3]),'Offset',-zoffset([1,3]),'N',N);
    ylims = get_1D_msspoly_projection(msubs(-FRS_lyapunov_function_y,[t;k],[tplot/T;k_test]),z([2,3]),0,1,'Scale',zscale([2,3]),'Offset',-zoffset([2,3]),'N',N);

    if tplot == 0 
       xlims(1) = min([xlims(1),A.footprint_vertices(1,:)]);
       xlims(2) = max([xlims(2),A.footprint_vertices(1,:)]);
       
       ylims(1) = min([ylims(1),A.footprint_vertices(2,:)]);
       ylims(2) = max([ylims(2),A.footprint_vertices(2,:)]);
    end
        xbox = make_box([diff(xlims),diff(ylims)])+[mean(xlims);mean(ylims)];
    
    figure(i)
    xbar = [xlims(1) xlims(2) xlims(2) xlims(1) xlims(1);....
            yplotlimits(1) yplotlimits(1) yplotlimits(2) yplotlimits(2) yplotlimits(1)];
        
     ybar = [xplotlimits(1) xplotlimits(1) xplotlimits(2) xplotlimits(2) xplotlimits(1);...
         ylims(1) ylims(2) ylims(2) ylims(1) ylims(1)];   
        
     fill(xbar(1,:),xbar(2,:),x_color,'FaceAlpha',0.5,'EdgeColor','None')
     hold on
     fill(ybar(1,:),ybar(2,:),y_color,'FaceAlpha',0.5,'EdgeColor','None')
        for fplot = [i,4]
            
            figure(fplot)
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
            plot(xbox(1,:),xbox(2,:),'--','Color',box_color,'LineWidth',1.5)
            
        end
        figure(i)
            text(textpos(1),textpos(2),['time = ',num2str(tvec(i),'%0.2f'),' s'],'FontSize',12)
            figure(4)
            text(textpos(1),textpos(2),'composite','FontSize',12)


end

figure(4)
plot_2D_msspoly_contour(subs(w,k,k_test),z(1:2),1,'Scale',zscale(1:2),'Offset',-zoffset(1:2),'Color',frs_color,'LineWidth',1.5)

for i = 1:4
    figure(i)
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
  set(gca,'Fontsize',12);
  set(gca,'fontname','Times New Roman')
  xlabel('x (m)')
 xlim(xplotlimits)
 ylim(yplotlimits)
  ylabel('y (m)')
  
  ax = gca;
  ax.YAxis.TickLabelFormat = '%.2f';
  ax.XAxis.TickLabelFormat = '%.2f';
  

  
 box on
end

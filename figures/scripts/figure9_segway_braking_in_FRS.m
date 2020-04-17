%% description
% This script generates Fig. 8 in the paper, which shows the Segway robot
% and the corresponding reachable set for two different trajectory plans.
%
% Author: Shreyas Kousik
% Created: 9 Apr 2020
%
%% user parameters
save_pdf_flag = true ;

%% automated from here
% initial condition
w_0 = 0.0 ;
v_0 = 1.5 ;

% commanded trajectory
w_des = 1.0 ;
v_des = 1.5 ;

% plotting and file i/o
frs_color = [0, 0.75, 0.25] ;

% create segway
A = segway_agent() ;
A_go = segway_agent() ;
A_brk = segway_agent('plot_footprint_color',[1 0 0],...
    'plot_footprint_edge_color',[1 0 0]) ;

% set initial state
z_0 = [0;0;0;w_0;v_0] ;
A_go.reset(z_0) ;
A_brk.reset(z_0) ;

% load timing info
load('segway_timing.mat')
t_f = get_t_f_from_v_0_for_segway(v_0) ;

% load FRS
FRS = get_segway_FRS_from_v_0(v_0) ;

% make trajectory
t_stop = v_0 / A_brk.max_accel ;
[T_go,U_go,Z_go] = make_segway_desired_trajectory(t_f,w_des,v_des) ;
[T_brk,U_brk,Z_brk] = make_segway_braking_trajectory(t_plan,t_stop,w_des,v_des) ;

% track the desired trajectories
A_go.move(t_f,T_go,U_go,Z_go) ;
A_brk.move(t_plan+t_stop,T_brk,U_brk,Z_brk) ;

% get the FRS plotting info
k_des = get_segway_k_from_w_and_v(FRS,w_des,v_des) ;
FRS_poly = msubs(FRS.FRS_polynomial,FRS.k,k_des) ;
P = get_2D_contour_points(FRS_poly,FRS.z,1,'Bounds',0.9) ;
P = FRS_to_world(P,A_go.state(:,1),FRS.initial_x,FRS.initial_y,FRS.distance_scale) ;

% make circle to plot at trajectory ends
N_C = 100 ;
th = linspace(0,2*pi,N_C) ;
C = A.footprint.*[cos(th) ; sin(th)] ;

%% plotting
fh = figure(1) ; clf ; hold on ; axis equal

plot(A)
plot_path(P,'Color',frs_color)
plot_path(Z_go,'b--')
plot_path(A_go.state,'b')
plot_path(A_brk.state,'r')

plot_path(C + repmat(A_go.state(1:2,end),1,N_C),'b')
plot_path(C + repmat(A_brk.state(1:2,end),1,N_C),'r')
plot_path(C + repmat(Z_go(1:2,end),1,N_C),'b--')

xlabel('x [m]')
ylabel('y [m]')

axis([-0.5 2 -0.5 1])
xticks(-1:0.5:2)
yticks(-0.5:0.5:1.5)
box on

set_plot_linewidths(2)
set(gca,'FontSize',15)

%% save output
if save_pdf_flag
    save_figure_to_pdf(fh,'segway_braking.pdf')
end
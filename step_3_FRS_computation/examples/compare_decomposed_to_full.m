clear; close




%% decomposed systems
max_psi = 0.6;
%% automated from here

% create agent to use for footprint
A = RoverAWD ;

rotated_vertices = [cos(max_psi) -sin(max_psi);sin(max_psi) cos(max_psi)]*A.footprint_vertices;

L = [min(rotated_vertices(1,:)),max(rotated_vertices(1,:))];
W = [min(rotated_vertices(2,:)),max(rotated_vertices(2,:))];

rotated_vertices = [cos(max_psi) -sin(max_psi);sin(max_psi) cos(max_psi)]'*A.footprint_vertices;

L = [min([L(1),rotated_vertices(1,:)]),max([L(2),rotated_vertices(1,:)])];
W = [min([W(1),rotated_vertices(2,:)]),max([W(2),rotated_vertices(2,:)])];


load('rover_FRS_xy_scaling_T1.5_w0des_0.0_to_1.0_v0_1.0_to_2.0_19-Mar-2020.mat') 
load ('rover_FRS_xy_T1.5_deg8_v0_1.0_to_2.0_19-Mar-2020.mat')

tvec = linspace(0,1,8);
plotted = false;
for tp = tvec
    hold on


    figure(1)
      xlims = get_1D_msspoly_projection(msubs(-FRS_lyapunov_function_x,[t;k],[tp;k_test]),z([1,3]),0,1,'Scale',zscale(1:3),'Offset',-zoffset(1:3))+L;
      ylims =  get_1D_msspoly_projection(msubs(-FRS_lyapunov_function_y,[t;k],[tp;k_test]),z([2,3]),0,1,'Scale',zscale(2:3),'Offset',-zoffset(2:3))+W;
      if all(~isnan(xlims) & ~isnan(ylims))
          xbox = make_box([diff(xlims),diff(ylims)])+[mean(xlims);mean(ylims)];
      else

          xbox = make_box([diff(L),diff(W)])+[mean(L);mean(W)];
          
      end
          if ~plotted
              h{1} = fill(xbox(1,:),xbox(2,:),[0 0.5 0.25],'FaceAlpha',0.25);
          else
              fill(xbox(1,:),xbox(2,:),[0 0.5 0.25]);
          end
   
      
end


%% FRS

FRS{1} = load('rover_FRS_full_T1.5_deg6_v0_1.0_to_2.0_20-Mar-2020.mat');
 FRS{2} = load('rover_FRS_full_T1.5_deg6_v0_1.0_to_2.0_22-Mar-2020.mat');
FRS{3} = load('rover_FRS_full_T1.5_deg6_psi0_-0.5_to_0.0_v0_1.0_to_2.0_18-Mar-2020.mat');
FRS{2}.x = FRS{2}.z(1);
FRS{2}.y = FRS{2}.z(2);
FRS{2}.xscale = FRS{2}.zscale(1:2);
FRS{2}.xoffset = FRS{2}.zoffset(1:2);


cols = [1 0 0;0 0.8 0.2;0 0.5 0.5];

k_test{1} = [1;0;0];
k_test{2} = [1;0;0];
k_test{3} = [1;0;0];

for i = 1:length(FRS)
   
   wx = subs(FRS{i}.w,FRS{i}.k,k_test{i}); 
   
   h{i+1} = plot_2D_msspoly_contour(wx,[FRS{i}.x;FRS{i}.y],1,'Scale',FRS{i}.xscale,'Offset',-FRS{i}.xoffset,'Color',cols(i,:),'LineWidth',1);
   hold on
   
end
legend([h{1},h{2} ],{'subsystems degree 8','Full System degree 6'});

% legend([h{1},h{2} h{3} h{4}],{'decomp','Full 6','All 6','All 8'});

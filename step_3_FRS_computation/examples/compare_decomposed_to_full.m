clear; close


k_test = [0;0;-1];

%% decomposed systems

load('rover_FRS_xy_scaling_T1.5_v0_1.0_to_2.0_24-Mar-2020.mat') 
load ('rover_FRS_xy_T1.5_deg8_v0_1.0_to_2.0_25-Mar-2020.mat')

tvec = linspace(0,1,8);
plotted = false;
for tp = tvec
    hold on


    figure(1)
      xlims = get_1D_msspoly_projection(msubs(-FRS_lyapunov_function_x,[t;k],[tp;k_test]),z([1,3]),0,1,'Scale',zscale(1:3),'Offset',-zoffset(1:3));
      ylims =  get_1D_msspoly_projection(msubs(-FRS_lyapunov_function_y,[t;k],[tp;k_test]),z([2,3]),0,1,'Scale',zscale(2:3),'Offset',-zoffset(2:3));
      
      if all(~isnan(xlims) & ~isnan(ylims))
          xbox = make_box([diff(xlims),diff(ylims)])+[mean(xlims);mean(ylims)];

      end
          if ~plotted
              h{1} = fill(xbox(1,:),xbox(2,:),[0 0.5 0.25],'FaceAlpha',0.25);
          else
              fill(xbox(1,:),xbox(2,:),[0 0.5 0.25]);
          end
   
      
end


%% FRS

FRS{1} = load('rover_FRS_all_T1.5_deg8_v0_1.0_to_2.0_25-Mar-2020.mat');
 FRS{2} = load('rover_xy_FRS_full_T1.5_deg6_v0_1.0_to_2.0_25-Mar-2020.mat');


cols = [1 0 0;0 0.8 0.2;0 0.5 0.5];

for i = 1:length(FRS)
   
   wx = subs(FRS{i}.w,FRS{i}.k,k_test); 
   
   h{i+1} = plot_2D_msspoly_contour(wx,FRS{i}.z(1:2),1,'Scale',FRS{i}.zscale(1:2),'Offset',-FRS{i}.zoffset(1:2),'Color',cols(i,:),'LineWidth',1);
   hold on
   
end
legend([h{1},h{2},h{3} ],{'decomp','Full 6','All 8'});

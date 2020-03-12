%% description
% This script executed the reconstruction progmram (XX) Ffor the Rover. The
% user specifies the range of initial speeds; all other info is loaded from
% the relevant .mat files.
%
% Author: Sean Vaskov
% Created: 10 March 2020
%
clear ; clc ; close all ;
%% user parameters
% degree of SOS polynomial solution
degree = 6 ; % this is the degree of the x and y reachsets
degree_reconstruction = 6; %this is the degree of the final (w) reachset

% speed range (uncomment one of the following)
v_0_range = [1.0, 2.0] ;

% whether or not to save output
save_result = true;

%enter maximum heading rotation (footprint will be a bounding box covering
%this)

max_psi = 0.6;
%% automated from here
% load timing
load('rover_timing.mat')

% load the error functions and distance scales
switch v_0_range(1)
    case 1.0
        load(['rover_pos_error_functions_v0_1.0_to_2.0.mat'])
        load(['rover_FRS_deg_',num2str(degree),'_v0_1.0_to_2.0.mat'])
        load('rover_FRS_scaling_v0_1.0_to_2.0.mat')
    otherwise
        error('Hey! You picked an invalid speed range for the RTD tutorial!')
end

%% create agent to use for footprint
A = RoverAWD ;

rotated_vertices = [cos(max_psi) -sin(max_psi);sin(max_psi) cos(max_psi)]*A.footprint_vertices;

L = [min(rotated_vertices(1,:)),max(rotated_vertices(1,:))];
W = [min(rotated_vertices(2,:)),max(rotated_vertices(2,:))];

rotated_vertices = [cos(max_psi) -sin(max_psi);sin(max_psi) cos(max_psi)]'*A.footprint_vertices;

L = [min([L(1),rotated_vertices(1,:)]),max([L(2),rotated_vertices(1,:)])];
W = [min([W(1),rotated_vertices(2,:)]),max([W(2),rotated_vertices(2,:)])];

%%
x = msspoly('x',1);
y = msspoly('y',1);

xc_unscaled = zscale(1)*z(1)-zoffset(1);
yc_unscaled = zscale(2)*z(2)-zoffset(2);

X_range = [-zscale(1:2)-zoffset(1:2),zscale(1:2)-zoffset(1:2)];
X_range = X_range+2*[L;W];

xscale = (X_range(:,2)-X_range(:,1))/2;
xoffset = -(X_range(:,2)+X_range(:,1))/2;


x_unscaled = xscale(1)*x-xoffset(1);
y_unscaled = xscale(2)*y-xoffset(2);

hX =  sqrt(2)-x^2-y^2;
hB = [(x_unscaled-xc_unscaled-L(1))*(L(2)-x_unscaled+xc_unscaled);...
      (y_unscaled-yc_unscaled-W(1))*(W(2)-y_unscaled+yc_unscaled)];
hT = t*(1-t);
hK = [(k+1).*(1-k);solver_input_problem(1).hK(end-1:end)];
hZ = (z+1).*(1-z);
  
int_XK = boxMoments([x;y;k],-ones(5,1),ones(5,1));

%%
prog = spotsosprog();

prog = prog.withIndeterminate([t;x;y;z;k]);

wmon = monomials([x;y;k],0:degree);

[prog,w,wcoeff] = prog.newFreePoly(wmon);

prog = sosOnK(prog,w-1,[t;x;z;k],[t*(1-t);hX;hB;hZ;hK;-FRS_lyapunov_function_x;-FRS_lyapunov_function_y],degree_reconstruction);

prog = sosOnK(prog,w,[x;y;k],[hX;hK(1:end-2)],degree_reconstruction);

obj = int_XK(wmon)'*wcoeff;

%% 
disp('Solving for the FRS')
options = spot_sdp_default_options() ;
options.verbose = 1 ;
options.domain_size = 1;
options.solveroptions = [];


start_tic = tic ;
sol = prog.minimize(obj, @spot_mosek, options);
end_time = toc(start_tic) ;

w = sol.eval(w);


%% save result
if save_result
    % create the filename for saving
    filename = ['rover_FRS_deg',num2str(degree),'_reconstructed_deg',num2str(degree_reconstruction),'_v0_',...
                num2str(v0_min,'%0.1f'),'_to_',...
                num2str(v0_max,'%0.1f'),'.mat'] ;

    % save output
    disp(['Saving FRS output to file: ',filename])
    save(filename,'FRS_polynomial*','FRS_lyapunov_function*','w','x','y','t','z','k',...
        'f','g','t_plan','degree','degree_reconstruction','solver_input_problem','*_max','*_min','*scale')
end

%%
k_test = [1;0;1];

close all

 hold on
 
xlim(zscale(1)*[-1 1]-zoffset(1))
ylim(zscale(2)*[-1 1]-zoffset(2))

for tp = linspace(0,1,5)
    hold on
      xlims = get_1D_msspoly_projection(msubs(-FRS_lyapunov_function_x,[t;k],[tp;k_test]),z([1,3]),0,1,'Scale',zscale([1,3]),'Offset',-zoffset([1,3]));
      ylims =  get_1D_msspoly_projection(msubs(-FRS_lyapunov_function_y,[t;k],[tp;k_test]),z([2,3]),0,1,'Scale',zscale([2,3]),'Offset',-zoffset([2,3]));
      if all(~isnan(xlims) & ~isnan(ylims))
      xbox = make_box([diff(xlims),diff(ylims)])+[mean(xlims);mean(ylims)];
      fill(xbox(1,:),xbox(2,:),[0 0.5 0.25])
      end
end


plot_2D_msspoly_contour(subs(w,k,k_test),[x;y],1,'Scale',xscale,'Offset',-xoffset,'LineWidth',1.5,'Color',[0 0.75 0.25])






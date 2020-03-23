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
degree_reconstruction = 4; %this is the degree of the final (w) reachset

% load the error functions and distance scales
load('rover_FRS_xy_scaling_T1.5_v0_1.0_to_2.0_20-Mar-2020.mat')
load('rover_FRS_xy_T1.5_deg6_v0_1.0_to_2.0_20-Mar-2020.mat')
  
% whether or not to save output
save_result = true;

%enter maximum heading rotation (footprint will be a bounding box covering
%this)

max_psi = 0.6;
%% automated from here
if ~exist('lower_lim_k1','var')
    lower_lim_k1 = -1;
end
if ~exist('upper_lim_k1','var')
    upper_lim_k1 = 1;
end

% create agent to use for footprint
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
X_range = X_range+sqrt(2)*[L;W];

xscale = (X_range(:,2)-X_range(:,1))/2;
xoffset = -(X_range(:,2)+X_range(:,1))/2;


x_unscaled = xscale(1)*x-xoffset(1);
y_unscaled = xscale(2)*y-xoffset(2);

hX =  [1-x^2;1-y^2];
hB = [(x_unscaled-xc_unscaled-L(1))*(L(2)-x_unscaled+xc_unscaled);...
      (y_unscaled-yc_unscaled-W(1))*(W(2)-y_unscaled+yc_unscaled)];
hT = t*(1-t);
hK = solver_input_problem(1).hK;
hZ = (z+1).*(1-z);
  
int_XK = boxMoments([x;y;k(2);k(1);k(3)],[-1,-1,lower_lim_k2,lower_lim_k1,-1],[1,1,upper_lim_k2,upper_lim_k1,1]);



%% solve reconstruction
prog = spotsosprog();

prog = prog.withIndeterminate([t;x;y;z;k]);

wmon = monomials([x;y;k],0:degree_reconstruction);

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
    filename = ['rover_reconstructed_deg',num2str(degree_reconstruction),'_frsdeg',num2str(degree),'_T',num2str(T),'_v0_',...
                num2str(v0_min,'%0.1f'),'_to_',...
                num2str(v0_max,'%0.1f'),'.mat'] ;

    % save output
    disp(['Saving FRS output to file: ',filename])
    save(filename,'FRS_polynomial*','FRS_lyapunov_function*','w','x','y','t','z','k',...
        'f','g','t_f','degree','lower_lim*','upper_lim*','degree_reconstruction','solver_input_problem','*_max','*_min','*scale','*offset')
end




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
degree_reconstruction = 8; %this is the degree of the final (w) reachset

% load the error functions and distance scales
load('rover_FRS_xy_scaling_T1.2_v0_0.0_to_0.8_delta0_-0.05_to_0.05.mat')
load('rover_FRS_xy_T1.25_deg8_v0_0.0_to_0.8_delta0_-0.05_to_0.05_27-Mar-2020.mat')
  
% whether or not to save output
save_result = true;

%% automated
hT = t*(1-t);

upper_lim_k1 =  1 ;

lower_lim_k1 =  -0.5+0.5*k(2);

hK = [(k(1)-lower_lim_k1)*(upper_lim_k1-k(1));...
      (k(2)+1)*(1-k(2));...
      (k(3)+1)*(1-k(3))]; 
  
hZ = (z+1).*(1-z);

int_ZK = boxMoments([z(1:2);k], [-1;-1;lower_lim_k1;-1;-1],[1;1;upper_lim_k1;1;1]);


%% solve reconstruction
prog = spotsosprog();

prog = prog.withIndeterminate([t;z;k]);

wmon = monomials([z(1:2);k],0:degree_reconstruction);

[prog,w,wcoeff] = prog.newFreePoly(wmon);

prog = sosOnK(prog,w-1,[t;z;k],[t*(1-t);hZ;hK;-FRS_lyapunov_function_x;-FRS_lyapunov_function_y],degree_reconstruction);

prog = sosOnK(prog,w,[z(1:2);k],[hZ(1:2);hK],degree_reconstruction);

obj = int_ZK(wmon)'*wcoeff;

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
    filename = ['rover_reconstructed_deg',num2str(degree_reconstruction),'_frsdeg',num2str(degree),'_T',num2str(T),...
        '_v0_', num2str(v0_min,'%0.1f'),'_to_',num2str(v0_max,'%0.1f'),...
        '_delta0_',num2str(delta0_min,'%0.2f'),'_to_',num2str(delta0_max,'%0.2f'),'.mat'] ;

    % save output
    disp(['Saving FRS output to file: ',filename])
    save(filename,'FRS_polynomial*','FRS_lyapunov_function*','w','t','z','k',...
        'f','g','t_f','T','degree','degree_reconstruction','solver_input_problem','*_max','*_min','*scale','*offset')
end




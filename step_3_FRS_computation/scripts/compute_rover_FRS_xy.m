%% description
% This script computes a Forward-Reachable Sets (FRS) for x and y subsystems of the Rover.
% By solving (D_i^l) for i = {1,2} 
% the user provides .mat files to load relevant info

% Author: Sean Vaskov
% Created: 17 March 2020
%
clear ; clc ; close all ;
%% user parameters
% degree of SOS polynomial solution
degree = 4 ; % this should be 4 or 6 unless you have like 100+ GB of RAM


%error function file
load('rover_xy_error_functions_T1.2_v0_0.0_to_0.8_delta0_-0.50_to_-0.30_degx3_degy3.mat')
%scaling function file
load('rover_FRS_xy_scaling_T1.2_v0_0.0_to_0.8_delta0_-0.50_to_-0.30.mat')

save_result = true;


%% set up the FRS computation variables and dynamics
% set up the indeterminates
t = msspoly('t', 1) ; % time t \in T
z = msspoly('z', 3) ; % state z = (x,y) \in Z
k = msspoly('k', 3) ; % parameters k \in K

%unscaled states
x = zscale(1)*z(1)-zoffset(1);
y = zscale(2)*z(2)-zoffset(2);
psi = zscale(3)*z(3)-zoffset(3);

%unscaled parameters
w0_des =   (w0_des_max-w0_des_min)/2*(k(1)+1)+w0_des_min;
psi_end =  (psi_end_max-psi_end_min)/2*(k(2)+1)+psi_end_min;
v_des =    (v_des_max-v_des_min)/2*(k(3)+1)+v_des_min;


% create polynomials that are positive on Z, and K, thereby
% defining them as semi-algebraic sets; h_T is automatically generated
upper_lim_k1 =  1 ;
lower_lim_k1 =  -0.5+0.5*k(2);

hK = [(k(1)-lower_lim_k1)*(upper_lim_k1-k(1));...
      (k(2)+1)*(1-k(2));...
      (k(3)+1)*(1-k(3))]; 
  
hZ = (z+1).*(1-z);

%define initial footprint

L = [min(A.footprint_vertices(1,:)), max(A.footprint_vertices(1,:))];
W = [min(A.footprint_vertices(2,:)), max(A.footprint_vertices(2,:))];

hZ0 = [(x-L(1))*(L(2)-x);(y-W(1))*(W(2)-y);-psi^2];

%% specify dynamics and error function
cos_psi = 1-psi^2/2;
sin_psi = psi-psi^3/6;

% create dynamics
scale = (T./zscale) ;

w_slope =  -2*(t_f*w0_des-psi_end)/t_f^2;

w_des = w_slope*T*t+w0_des;

f = scale.*[v_des*cos_psi-A.rear_axel_to_center_of_mass*w_des*sin_psi;...
            v_des*sin_psi+A.rear_axel_to_center_of_mass*w_des*cos_psi;...
            w_des] ;

% create tracking error dynamics; first, make msspoly functions for the
% velocity errors
g_x = subs(g_x,[t;z;k],[T*t;x;y;psi;w0_des;psi_end;v_des]);
g_y = subs(g_y,[t;z;k],[T*t;x;y;psi;w0_des;psi_end;v_des]);

g = scale.*[g_x;...
            g_y;...
            0] ;

%% create cost function
% this time around, we care about the indicator function being on Z x K
int_TZK{1} = boxMoments([t;z(1);k], [0;-1;lower_lim_k1;-1;-1],[1;1;upper_lim_k1;1;1]);
int_TZK{2} = boxMoments([t;z(2);k], [0;-1;lower_lim_k1;-1;-1],[1;1;upper_lim_k1;1;1]);


%% setup the problem structure for x and y
for i = 1:2
    
solver_input_problem(i).t = t ;
solver_input_problem(i).z = z([i, 3]) ;
solver_input_problem(i).k = k ;

solver_input_problem(i).f = f([i, 3]) ;

solver_input_problem(i).hZ = hZ([i,3]);
solver_input_problem(i).hZ0 = hZ0([i,3]);
solver_input_problem(i).hK = hK ;

solver_input_problem(i).cost = int_TZK{i} ;
solver_input_problem(i).degree = degree ;
solver_input_problem(i).FRS_states = [t;z(i);k];
solver_input_problem(i).hFRS_states = [t*(1-t);1-z(i)^2;hK];

solver_input_problem(i).g = g([i,3],1) ;

end

%% compute FRS for x and y positions
for i = 1:2
solver_output(i) = compute_FRS(solver_input_problem(i)) ;
end

%% extract FRS polynomial result
FRS_polynomial_x = solver_output(1).indicator_function ;
FRS_lyapunov_function_x = solver_output(1).lyapunov_function ;

FRS_polynomial_y = solver_output(2).indicator_function ;
FRS_lyapunov_function_y = solver_output(2).lyapunov_function ;


%% save result
if save_result
    % create the filename for saving
    filename = ['rover_FRS_xy_T',num2str(T),'_deg',num2str(degree),...
               '_v0_',num2str(v0_min,'%0.1f'),'_to_',num2str(v0_max,'%0.1f'),...
               '_delta0_',num2str(delta0_min,'%0.2f'),'_to_',num2str(delta0_max,'%0.2f'),...
               '_',date,'.mat'] ;

    % save output
    disp(['Saving FRS output to file: ',filename])
    save(filename,'FRS_polynomial*','FRS_lyapunov_function*','t_f','t','z','k',...
       'T', 'f','g','degree','solver_input_problem')
end
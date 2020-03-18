%% description
% This script computes a Forward-Reachable Set (FRS) for the Rover. The
% user specifies the range of initial speeds; all other info is loaded from
% the relevant .mat files.
%
% Author: Sean Vaskov
% Created: 08 March 2020
%
clear ; clc ; close all ;
%% user parameters
% degree of SOS polynomial solution
degree = 4 ; % this should be 4 or 6 unless you have like 100+ GB of RAM


%error_functions
load('rover_pos_error_functions_T1.5_v0_1.0_to_2.0_degx3_degy3.mat')
%scaling function file
load('rover_FRS_scaling_T1.5_psi0_-0.5_to_0.0_v0_1.0_to_2.0_17-Mar-2020.mat')

if ~exist('A','var')
    A = RoverAWD ;
end

max_psi = 0.6; % maximum heading 
% whether or not to save output
save_result = true;

%% set up the FRS computation variables and dynamics
% set up the indeterminates
t = msspoly('t', 1) ; % time t \in T
z = msspoly('z', 3) ; % state z = (x,y) \in Z
k = msspoly('k', 3) ; % parameters k \in K

%unscaled states
xc_unscaled = zscale(1)*z(1)-zoffset(1);
yc_unscaled = zscale(2)*z(2)-zoffset(2);
psi_unscaled = zscale(3)*z(3)-zoffset(3);

%unscaled parameters
w0_des =   (w0_des_max-w0_des_min)/2*(k(1)+1)+w0_des_min;
psi_end =  (psi_end_max-psi_end_min)/2*(k(2)+1)+psi_end_min;
v_des =    (v_des_max-v_des_min)/2*(k(3)+1)+v_des_min;


% create polynomials that are positive on Z, and K, therebyﬂ
% defining them as semi-algebraic sets; h_T is automatically generated
lower_lim_k1 = -(w0_des_max/2 - w0_des_min/2 + (w0_des_min*(psi_end_min - (k(2) + 1)*(psi_end_min/2 - psi_end_max/2)))/psi_end_max)/(w0_des_max/2 - w0_des_min/2);
upper_lim_k1 = 1;

hK = [(k(1)-lower_lim_k1)*(upper_lim_k1-k(1));(k(2:3)+1).*(1-k(2:3))];

hZ = (z+1).*(1-z);

hZ0 = msspoly(zeros(2,1));
hZ0(1) = -xc_unscaled^2-psi_unscaled^2;
hZ0(2) = -yc_unscaled^2-psi_unscaled^2;

%% specify dynamics and error function
cos_psi = 1-psi_unscaled^2/2;
sin_psi = psi_unscaled-psi_unscaled^3/6;

% create dynamics
scale = (T./zscale) ;

w_slope =  -2*(t_f*w0_des-psi_end)/t_f^2;

w_des = w_slope*T*t+w0_des;

f = scale.*[v_des*cos_psi-A.rear_axel_to_center_of_mass*w_des*sin_psi;...
            v_des*sin_psi+A.rear_axel_to_center_of_mass*w_des*cos_psi;...
            w_des] ;

% create tracking error dynamics; first, make msspoly functions for the
% velocity errors
g_v_cos = subs(g_v_cos,[t;z;k],[T*t;xc_unscaled;yc_unscaled;psi_unscaled;w0_des;psi_end;v_des]);
g_v_sin = subs(g_v_sin,[t;z;k],[T*t;xc_unscaled;yc_unscaled;psi_unscaled;w0_des;psi_end;v_des]);

g_vy_cos = subs(g_vy_cos,[t;z;k],[T*t;xc_unscaled;yc_unscaled;psi_unscaled;w0_des;psi_end;v_des]);
g_vy_sin = subs(g_vy_sin,[t;z;k],[T*t;xc_unscaled;yc_unscaled;psi_unscaled;w0_des;psi_end;v_des]);

g = [scale,scale].*[g_v_cos, -g_vy_sin;...
                    g_v_sin, g_vy_cos;...
                     0, 0] ;


%% create agent to use for footprint

rotated_vertices = [cos(max_psi) -sin(max_psi);sin(max_psi) cos(max_psi)]*A.footprint_vertices;

L = [min(rotated_vertices(1,:)),max(rotated_vertices(1,:))];
W = [min(rotated_vertices(2,:)),max(rotated_vertices(2,:))];

rotated_vertices = [cos(max_psi) -sin(max_psi);sin(max_psi) cos(max_psi)]'*A.footprint_vertices;

L = [min([L(1),rotated_vertices(1,:)]),max([L(2),rotated_vertices(1,:)])];
W = [min([W(1),rotated_vertices(2,:)]),max([W(2),rotated_vertices(2,:)])];

%expand scaling factors by footpring
X_range = [-zscale(1:2)-zoffset(1:2),zscale(1:2)-zoffset(1:2)];
X_range = X_range+sqrt(2)*[L;W];

xscale = (X_range(:,2)-X_range(:,1))/2;
xoffset = -(X_range(:,2)+X_range(:,1))/2;



%% create cost function
x = msspoly('x',1);
y = msspoly('y',1);

x_unscaled = xscale(1)*x-xoffset(1);
y_unscaled = xscale(2)*y-xoffset(2);

% this time around, we care about the indicator function being on Z x K
int_TZK = boxMoments([x;y;k], [-1;-1;lower_lim_k1;-1;-1], [1;1;upper_lim_k1;1;1]);

%% setup the problem structure for x and y

solver_input_problem.x = [x;y];
solver_input_problem.FRS_states = [x;y;k];
solver_input_problem.hFRS_states = [1-x^2;1-y^2;hK];
solver_input_problem.hBody =  [(x_unscaled-xc_unscaled-L(1))*(L(2)-x_unscaled+xc_unscaled);...
                                (y_unscaled-yc_unscaled-W(1))*(W(2)-y_unscaled+yc_unscaled)];
   
solver_input_problem.t = t ;
solver_input_problem.z = z ;
solver_input_problem.k = k ;
solver_input_problem.f = f ;
solver_input_problem.hZ = hZ;
solver_input_problem.hZ0 = hZ0;
solver_input_problem.hK = hK ;
solver_input_problem.cost = int_TZK ;
solver_input_problem.degree = degree ;

solver_input_problem.g = g ;


%% compute FRS 
solver_output = compute_FRS(solver_input_problem) ;


%% extract FRS polynomial result
FRS_polynomial = solver_output.indicator_function ;
FRS_lyapunov_function = solver_output.lyapunov_function ;

w = FRS_polynomial;
%% save result
if save_result
    % create the filename for saving
    filename = ['rover_FRS_full_T',num2str(T,'%0.1f'),'_deg',num2str(degree),...
                '_psi0_',num2str(psi0_min,'%0.1f'),'_to_',num2str(psi0_max,'%0.1f'),...
                '_v0_',num2str(v0_min,'%0.1f'),'_to_',num2str(v0_max,'%0.1f'),'_',date,'.mat'] ;

    % save output
    disp(['Saving FRS output to file: ',filename])
    save(filename,'FRS_polynomial*','FRS_lyapunov_function*','T','t_f','t','x','y','z','k',...
        'diff_v','f','g','w','degree','*max','*min','solver_input_problem','*scale','*offset')
end
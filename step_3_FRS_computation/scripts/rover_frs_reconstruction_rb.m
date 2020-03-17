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
degree_reconstruction = 6; %this is the degree of the final (w) reachset

% load timing and relavent files
load('rover_timing.mat')

% load the error functions and distance scales
load('rover_FRS_scaling_T1.5_v0_1.0_to_2.0.mat')
load('rover_FRS_rb_T1.5_deg6_v0_1.0_to_2.0.mat')
  


plotting = true; %plot sample trajectory

% whether or not to save output
save_result = true;

A = RoverAWD();
max_psi = 0.6;

%% create agent to use for footprint

rotated_vertices = [cos(max_psi) -sin(max_psi);sin(max_psi) cos(max_psi)]*A.footprint_vertices;

L = [min(rotated_vertices(1,:)),max(rotated_vertices(1,:))];
W = [min(rotated_vertices(2,:)),max(rotated_vertices(2,:))];

rotated_vertices = [cos(max_psi) -sin(max_psi);sin(max_psi) cos(max_psi)]'*A.footprint_vertices;

L = [min([L(1),rotated_vertices(1,:)]),max([L(2),rotated_vertices(1,:)])];
W = [min([W(1),rotated_vertices(2,:)]),max([W(2),rotated_vertices(2,:)])];

ftprint = [L(1),L(2),L(2),L(1),L(1);W(1),W(1),W(2),W(2),W(1)];
%% automated from here


hX =  [1-x^2;1-y^2];

hT = t*(1-t);
hK = [(k+1).*(1-k);solver_input_problem(1).hK(end-1:end)];
  
int_XK = boxMoments([x;y;k],-ones(5,1),ones(5,1));

%% if plotting sample f and g
if plotting
Nsamp = 50;
k_test = [1;0;1];

Xvec = linspace(-1,1,100);
hold on 
 for tp = linspace(0,1,5)

%       xsample = msubs(FRS_polynomial_x,[t;x;k],[repmat(tp,[1 100]);Xvec;repmat(k_test,[1 100])]);
%       ysample = msubs(FRS_polynomial_y,[t;y;k],[repmat(tp,[1 100]);Xvec;repmat(k_test,[1 100])]);
%       
%       figure(1)
%       subplot(2,1,1)
%       hold on
%       plot(Xvec,xsample)
%       subplot(2,1,2)
%       hold on
%       plot(Xvec,ysample)
%       
%       Lx = abs(xsample) < sqrt(2)/2 & xsample >= 0;
%       Ly = abs(ysample) < sqrt(2)/2 & ysample >= 0;
%       
%       xlims = [min(Xvec(Lx)), max(Xvec(Lx))];
%       ylims = [min(Xvec(Ly)), max(Xvec(Ly))];
%       
%       xlims = xscale(1)*xlims-xoffset(1);
%       ylims = xscale(2)*ylims-xoffset(2);
% 
%       figure(2)
%       xbox = make_box([diff(xlims),diff(ylims)])+[mean(xlims);mean(ylims)];
%       fill(xbox(1,:),xbox(2,:),[0 0.5 0.25])
%  
        plot_2D_msspoly_contour(msubs(FRS_polynomial_x+FRS_polynomial_y-2,[t;k],[tp;k_test]),...
            [x;y],0,'Scale',xscale,'Offset',-xoffset,'LineWidth',1.5,'Color',[0 0.75 0.25],'Scale',xscale,'Offset',-xoffset)


 end


ztemp= cell(1,Nsamp);
f_fun = msspoly_to_fun(f,{t,z,k});
g_fun = msspoly_to_fun(g,{t,z,k});
for i = 1:Nsamp
    d = 2*rand(2,1)-1;
    z0 = [0;0;0];
    
    [~,ztemp{i}] = ode45(@(t,z) f_fun(t,z,k_test)+g_fun(t,z,k_test)*d,[0 1],(z0+zoffset)./zscale);
    
end

ztemp = cat(1,ztemp{:})';

ztemp = repmat(zscale,[1 size(ztemp,2)]).*ztemp-repmat(zoffset,[1 size(ztemp,2)]);
for i = 1:size(ztemp,2)
    ftcur = ftprint+ztemp(1:2,i);
plot(ftcur(1,:),ftcur(2,:),'b')
end
end

%% solve reconstruction
prog = spotsosprog();

prog = prog.withIndeterminate([t;x;y;z;k]);

wmon = monomials([x;y;k],0:degree_reconstruction);

[prog,w,wcoeff] = prog.newFreePoly(wmon);

prog = sosOnK(prog,w-1-(FRS_polynomial_x-1)-(FRS_polynomial_y-1),[t;x;y;k],[t*(1-t);hX;hK],degree_reconstruction);

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
    filename = ['rover_rb_reconstructed_deg',num2str(degree_reconstruction),'_T',num2str(T,'%01f'),'_v0_',...
                num2str(v0_min,'%0.1f'),'_to_',...
                num2str(v0_max,'%0.1f'),'.mat'] ;

    % save output
    disp(['Saving FRS output to file: ',filename])
    save(filename,'FRS_polynomial*','w','x','y','t','z','k',...
        'f','g','t_plan','t_f','degree','degree_reconstruction','solver_input_problem','*_max','*_min','*scale','*offset')
end

%%




 hold on

xlim(xscale(1)*[-1 1]-xoffset(1))
ylim(xscale(2)*[-1 1]-xoffset(2))


plot_2D_msspoly_contour(subs(w,k,k_test),[x;y],1,'Scale',xscale,'Offset',-xoffset,'LineWidth',1.5,'Color',[0 0.75 0.25])






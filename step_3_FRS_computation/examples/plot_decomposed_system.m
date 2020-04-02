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

% load the error functions and distance scales
load('rover_xy_error_functions_T1.5_v0_1.0_to_2.0_degx3_degy3.mat')
load('rover_FRS_xy_scaling_T1.5_v0_1.0_to_2.0_24-Mar-2020.mat')
load('rover_FRS_xy_T1.5_deg8_v0_1.0_to_2.0_25-Mar-2020.mat')
  
%enter maximum heading rotation (footprint will be a bounding box covering
%this)

%
% create tracking error dynamics; first, make msspoly functions for the
% velocity errors

g = [g(1,1),0;0,g(2,1);0,0];

%% if plotting sample f and g

Nsamp = 50;
k_test = [0;-1;1];

Xvec = linspace(-1,1,100);
tvec = linspace(0,1,5);
for tp = tvec
    hold on


    figure(1)
      xlims = get_1D_msspoly_projection(msubs(-FRS_lyapunov_function_x,[t;k],[tp;k_test]),z([1,3]),0,1,'Scale',zscale([1,3]),'Offset',-zoffset([1,3]));
      ylims =  get_1D_msspoly_projection(msubs(-FRS_lyapunov_function_y,[t;k],[tp;k_test]),z([2,3]),0,1,'Scale',zscale([2,3]),'Offset',-zoffset([2,3]));
      if all(~isnan(xlims) & ~isnan(ylims))
      xbox = make_box([diff(xlims),diff(ylims)])+[mean(xlims);mean(ylims)];
      fill(xbox(1,:),xbox(2,:),[0 0.5 0.25])
      end
      
      figure(2)
      subplot(2,1,1)
      plot(Xvec,msubs(FRS_polynomial_x,[t;z(1);k],[tp*ones(1,100);Xvec;repmat(k_test,[1 100])]))
      hold on
        subplot(2,1,2)
      plot(Xvec,msubs(FRS_polynomial_y,[t;z(2);k],[tp*ones(1,100);Xvec;repmat(k_test,[1 100])]))
      hold on
      
      figure(3)
      subplot(2,1,1)
      hold on
      plot_2D_msspoly_contour(msubs(-FRS_lyapunov_function_x,[t;k],[tp;k_test]),z([1,3]),0)
      
      subplot(2,1,2)
      hold on
      plot_2D_msspoly_contour(msubs(-FRS_lyapunov_function_y,[t;k],[tp;k_test]),z([2,3]),0)
      
end

figure(1)
ztemp= cell(1,Nsamp);
ztvec = cell(1,Nsamp);
f_fun = msspoly_to_fun(f,{t,z,k});

if exist('g','var')
g_fun = msspoly_to_fun(g ,{t,z,k});
else
    g_fun = @(t,z,k) zeros(3,2);
end

for i = 1:Nsamp
    d = 2*rand(2,1)-1;
    z0 = randRange([min(A.footprint_vertices,[],2);0],[max(A.footprint_vertices,[],2);0]);
    
    [ttemp,ztemp{i}] = ode45(@(t,z) f_fun(t,z,k_test)+g_fun(t,z,k_test)*d,[0 1],(z0+zoffset)./zscale);
    ztvec{i} = interp1(ttemp,ztemp{i},tvec);
end

ztemp = cat(1,ztemp{:})';
ztvec = cat(1,ztvec{:});
ttvec = repmat(tvec',[Nsamp,1]);

ztemp = repmat(zscale,[1 size(ztemp,2)]).*ztemp-repmat(zoffset,[1 size(ztemp,2)]);

plot(ztemp(1,:),ztemp(2,:),'b.')
xlabel('x')
ylabel('y')

figure(2)
subplot(2,1,1)
plot(ztvec(:,1),ones(size(ztvec,1),1),'b.')
ylabel('x')

subplot(2,1,2)
plot(ztvec(:,2),ones(size(ztvec,1),1),'b.')
xlabel('y')

figure(3)
subplot(2,1,1)
plot(ztvec(:,1),ztvec(:,3),'b.')
xlabel('x')
ylabel('\psi')

subplot(2,1,2)
plot(ztvec(:,2),ztvec(:,3),'b.')
xlabel('y')
ylabel('\psi')





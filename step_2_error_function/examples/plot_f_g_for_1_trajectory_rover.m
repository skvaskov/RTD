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

% load timing and relavent files
load('rover_timing.mat')

% load the error functions and distance scales
load('rover_sincos_error_functions_withfootrptint_T1.5_v0_1.0_to_2.0_w0_-0.2_to_0.2_degx3_degy3.mat')

%test parameter
k_test = [0;0;1];

%T
T = 1.5;

%% forward integrate samples
L =[min(A.footprint_vertices(1,:)),max(A.footprint_vertices(1,:))];
W =[min(A.footprint_vertices(2,:)),max(A.footprint_vertices(2,:))];



Nsamp = 1000;

ztemp= cell(1,Nsamp);

g = [g_v_cos,-g_vy_sin;g_v_sin,g_vy_cos;0,0];

g_fun = msspoly_to_fun(g,{t,z,k});

for i = 1:Nsamp
    
    d = 2*rand(2,1)-1;
    z0 = [randRange([L(1);W(1)],[L(2);W(2)]);0];
    
    [~,ztemp{i}] = ode45(@(t,z) rover_trajectory_producing_model(t,z,k_test,t_f)+g_fun(t,z,k_test)*d,[0 T],z0);
    
end

ztemp = cat(1,ztemp{:})';

plot(ztemp(1,:),ztemp(2,:),'b.')

%% forward integrate traj model 

[~,znom] = ode45(@(t,z) rover_trajectory_producing_model(t,z,k_test,t_f),[0 T],[0;0;0]);
hold on
plot(znom(:,1),znom(:,2),'r','LineWidth',1.0)
xlabel('x')
ylabel('y')

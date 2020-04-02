clear; close




%% decomposed systems
max_psi = 0.6;
%% automated from here

% create agent to use for footprint
A = RoverAWD('footprint',[0.5 0.2]) ;

rotated_vertices = [cos(max_psi) -sin(max_psi);sin(max_psi) cos(max_psi)]*A.footprint_vertices;

L = [min(rotated_vertices(1,:)),max(rotated_vertices(1,:))];
W = [min(rotated_vertices(2,:)),max(rotated_vertices(2,:))];

rotated_vertices = [cos(max_psi) -sin(max_psi);sin(max_psi) cos(max_psi)]'*A.footprint_vertices;

L = [min([L(1),rotated_vertices(1,:)]),max([L(2),rotated_vertices(1,:)])];
W = [min([W(1),rotated_vertices(2,:)]),max([W(2),rotated_vertices(2,:)])];




%% FRS
load('rover_FRS_full_T1.5_deg6_v0_1.0_to_2.0_24-Mar-2020.mat');
%  FRS{2} = load('rover_FRS_full_T1.5_deg6_v0_1.0_to_2.0_22-Mar-2020.mat');
% FRS{2} = load('rover_FRS_full_T1.5_deg6_psi0_-0.5_to_0.0_v0_1.0_to_2.0_18-Mar-2020.mat');
% FRS{2}.x = FRS{2}.z(1);
% FRS{2}.y = FRS{2}.z(2);
% FRS{2}.xscale = FRS{2}.zscale(1:2);
% FRS{2}.xoffset = FRS{2}.zoffset(1:2);


k_test = [-1;0;1];
% k_test{2} = [1;0;0];
% % k_test{3} = [1;0;0];

   
   wx = subs(w,k,k_test); 
   
plot_2D_msspoly_contour(wx,[x;y],1,'Scale',xscale,'Offset',-xoffset,'Color',[0 0.8 0.2],'LineWidth',1);
   hold on
   

 %% samples
Nsamp = 100;

ztemp= cell(1,Nsamp);

% g = [g_v_cos,-g_vy_sin;g_v_sin,g_vy_cos;0,0];
f_fun = msspoly_to_fun(f,{t,z,k});
g_offset_fun = msspoly_to_fun(g_offset,{t,z,k});
g_fun = msspoly_to_fun(g,{t,z,k});
    z0 = ([0;0;0]+zoffset)./zscale;
for i = 1:Nsamp
    
    d = 2*rand(2,1)-1;

    
    [~,ztemp{i}] = ode45(@(t,z) f_fun(t,z,k_test)+g_offset_fun(t,z,k_test)+g_fun(t,z,k_test)*d,[0 1],z0);
    
end

ztemp = cat(1,ztemp{:})';

plot(zscale(1)*ztemp(1,:)-zoffset(1),zscale(2)*ztemp(2,:)-zoffset(2),'b.')
hold on
plot(zscale(1)*ztemp(1,:)-zoffset(1)+L(1),zscale(2)*ztemp(2,:)-zoffset(2)+W(1),'b.')
plot(zscale(1)*ztemp(1,:)-zoffset(1)+L(1),zscale(2)*ztemp(2,:)-zoffset(2)+W(2),'b.')
plot(zscale(1)*ztemp(1,:)-zoffset(1)+L(2),zscale(2)*ztemp(2,:)-zoffset(2)+W(1),'b.')
plot(zscale(1)*ztemp(1,:)-zoffset(1)+L(2),zscale(2)*ztemp(2,:)-zoffset(2)+W(2),'b.')



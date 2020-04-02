clear; close

load('rover_FRS_all_T1.5_deg8_v0_1.0_to_2.0_25-Mar-2020.mat')

pose0 = [0;0;0];

A = RoverAWD();
A.reset(pose0);
A.plot_at_time(0)
hold on


cmd_test_p = [-0.5;0.25;2]

k_test_p = [ (cmd_test_p(1)-w0_des_min)*2/(w0_des_max-w0_des_min)-1;...
             (cmd_test_p(2)-psi_end_min)*2/(psi_end_max-psi_end_min)-1;...
             (cmd_test_p(3)-v_des_min)*2/(v_des_max-v_des_min)-1]


% pose0 = [0;0;0];
%sub k1 so it goes from [0 to 1]

wx_p = msubs(w,k,k_test_p);

h = get_2D_msspoly_contour(wx_p,z(1:2),1,'Scale',zscale(1:2),'Offset',-zoffset(1:2),'pose',pose0);

plot(h(1,:),h(2,:))

%% mirrored
cmd_test_m = [0.5;-0.25;2]

w0_des_min_m = -w0_des_max;
w0_des_max_m = -w0_des_min;
psi_end_min_m = -psi_end_max;
psi_end_max_m = -psi_end_min;

k_test_m = [ (cmd_test_m(1)-w0_des_min_m)*2/(w0_des_max_m-w0_des_min_m)-1;...
             (cmd_test_m(2)-psi_end_min_m)*2/(psi_end_max_m-psi_end_min_m)-1;...
             (cmd_test_m(3)-v_des_min)*2/(v_des_max-v_des_min)-1]

wm = subs(w,[z(2);k(1);k(2)],[-z(2);-k(1);-k(2)]);
wx_m = msubs(wm,k,k_test_m);

hold on
hm = get_2D_msspoly_contour(wx_m,z(1:2),1,'Scale',zscale(1:2),'Offset',[-zoffset(1);zoffset(2)],'pose',[pose0(1);pose0(2);pose0(3)]);
plot(hm(1,:),hm(2,:))
plot(hm(1,:),-hm(2,:),'--')

%% check point
pt_check = [3;1]+pose0(1:2);

plot(pt_check(1),pt_check(2),'o')

pt_check_p = (rotation_matrix_2D(pose0(3))'*(pt_check-pose0(1:2))+zoffset(1:2))./zscale(1:2);

disp('plus')
msubs(wx_p,z(1:2),pt_check_p)

pt_check_m = (rotation_matrix_2D(pose0(3))'*(pt_check-pose0(1:2))+[zoffset(1);-zoffset(2)])./zscale(1:2);

disp('minus')
msubs(wx_m,z(1:2),pt_check_m)
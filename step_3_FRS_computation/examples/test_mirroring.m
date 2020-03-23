clear; close

load('rover_FRS_full_T1.5_deg6_v0_1.0_to_2.0_20-Mar-2020.mat')

pose0 = [0.75;-1.5;0.5];

A = RoverAWD();
A.reset(pose0);
A.plot_at_time(0)
hold on
k_test = [0.5;1;1];


% pose0 = [0;0;0];
%sub k1 so it goes from [0 to 1]

wx_p = msubs(w,k,k_test);

h = get_2D_msspoly_contour(wx_p,[x;y],1,'Scale',xscale,'Offset',-xoffset,'pose',pose0);
plot(h(1,:),h(2,:))
%sub k1 so it goes from [0 to 1]
wm = subs(w,[y;k(1);k(2)],[-y;-k(1);-k(2)]);
wx_m = msubs(wm,k,[-k_test(1);k_test(2);k_test(3)]);

hold on
hm = get_2D_msspoly_contour(wx_m,[x;y],1,'Scale',xscale,'Offset',[-xoffset(1);xoffset(2)],'pose',[pose0(1);pose0(2);pose0(3)]);
plot(hm(1,:),hm(2,:))

pt_check = [3;-1.5]+pose0(1:2);

plot(pt_check(1),pt_check(2),'o')

pt_check_p = (rotation_matrix_2D(pose0(3))'*(pt_check-pose0(1:2))+xoffset)./xscale;

disp('plus')
msubs(wx_p,[x;y],pt_check_p)

pt_check_m = (rotation_matrix_2D(pose0(3))'*(pt_check-pose0(1:2))+[xoffset(1);-xoffset(2)])./xscale;

disp('minus')
msubs(wx_m,[x;y],pt_check_m)
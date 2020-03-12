clear
%% description
% In this script, we create a cost function and its analytic derivative for
% the Rover. The cost function is simply to minimize distance to a
% waypoint while tracking a trajectory. We'll save the cost and gradient as
% MATLAB functions.
%
% Author: Sean Vaskov
% Created: 11 March 2019
%
%% user parameters
% numerical integration parameters
dt_int = 0.05 ; % s
discount_factor = [0.9 ; 1.0] ; % compensates for integration error in x/y

% desired parameters for validation
w0_val = 0.8; % rad/s
psi_end_val = 0; % rad
v_val = 1.5 ; % m/s

% max speed and yaw rate for validation
w0_max_val = 1.0 ;
w0_min_val = -1.0 ;

v_max_val = 2.0 ;
v_min_val = 1.0 ;

% save cost and gradient flag
save_flag = true ;


%% automated from here
% load timing
load('rover_timing.mat')

% create symbolic variables to use in the cost
syms w0_max psi_end v_max w0_min v_min k_1 k_3 t real
syms x y psi real

% get the desired speed and yaw rate
w0_des = (w0_max - w0_min)/2 * ( k_1 + 1 ) + w0_min;
v_des =  (v_max - v_min)/2 * ( k_3 + 1 )+ v_min;

dzdt = rover_trajectory_producing_model(t,[x;y;psi],[w0_des;psi_end;v_des],t_f);

psi_t = int(dzdt(3),t,0,t);

y_T = int(subs(dzdt(2),psi,psi_t),t,0,t_f);

x_T = int(subs(dzdt(1),psi,psi_t),t,0,t_f);

psi_T = subs(psi_t,t,t_f);



% make a function out of the endpoint to use for validation
z_fn = matlabFunction([x_T;y_T;psi_T],'Vars',[k_1,psi_end,k_3,w0_max,w0_min,v_max,v_min]) ;


%% make the cost and analytic gradient
% create waypoints
syms x_des y_des cx cy real

% make the cost
cost =  cx*(x_T-x_des)^2+cy*(y_T-y_des)^2;

% get the analytic gradient
cost_grad = [diff(cost,k_1), diff(cost,k_3)] ;

%% make a function out of the cost and gradient
if save_flag
    disp('Saving cost and cost gradient')
    matlabFunction(cost,'Vars',[k_1,psi_end,k_3,w0_max,w0_min,v_max,v_min,x_des,y_des,cx,cy],'File','rover_cost')
    matlabFunction(cost_grad,'Vars',[k_1,psi_end,k_3,w0_max,w0_min,v_max,v_min,x_des,y_des,cx,cy],'File','rover_cost_grad')
else
    disp('Not saving cost and cost gradient')
end


%% time the cost and gradient
try
    test_input = num2cell(2*rand(1,11)) ;
    disp('Timing cost function')
    cost_time = timeit(@() rover_cost(test_input{:})) ;
    disp(['    Average exec time: ', num2str(cost_time),' s'])
    
    disp('Timing gradient function')
    cost_grad_time = timeit(@() rover_cost_grad(test_input{:})) ;
    disp(['    Average exec time: ', num2str(cost_grad_time),' s'])
catch
    disp('Error testing timing!')
end

%% validation
% get the actual endpoint at t_plan
[~,~,Z] = make_rover_desired_trajectory(t_f,w0_val, psi_end_val, v_val) ;

z_true = Z(1:3,end) ;

% sub in k_1, and k_2 to z
k_1_val = (w0_val - w0_min_val)*2/(w0_max_val - w0_min_val) - 1;
k_3_val = (v_val - v_min_val)*2/(v_max_val - v_min_val) - 1;

% z_val = double(subs([x_T;y_T;psi_T],[k_1,psi_end,k_3,w0_max,w0_min,v_max,v_min], [k_1_val,psi_end_val,k_3_val,w0_max_val,w0_min_val,v_max_val,v_min_val])) ;
z_val = z_fn(k_1_val,psi_end_val,k_3_val,w0_max_val,w0_min_val,v_max_val,v_min_val);


%% plotting
figure(1) ; clf ; hold on ; axis equal ; grid on

% plot desired trajectory
plot(Z(1,:),Z(2,:),'b')

% plot approximate endpoint
plot(z_val(1),z_val(2),'bo') ;

% cleanup
legend('desired traj.','numerical endpoint')

set(gca,'FontSize',15)
xlabel('x')
ylabel('y')
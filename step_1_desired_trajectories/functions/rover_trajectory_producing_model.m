function zd = rover_trajectory_producing_model(t,z,k,t_f)
% zd = rover_trajectory_producing_model(t,z,k)
%
% Output the dynamics of the lane change model that we use as a
% trajectory-producing model for the rover. This can get plugged in to
% ode45, for example. See make_rover_desired_trajectory for how it is
% used.
%
% Author: Sean Vaskov
% Created: 06 March 2020

%extract parameters
w_0 = k(1);
psi_end = k(2);
v_des = k(3);

% distance from the ear wheels to center of mass
lr = 0.0765;

w_slope = -2*(t_f*w_0-psi_end)/t_f^2;

w_des = w_0+w_slope*t;

% extract states
psi = z(3);

%taylor approximation for sin and cos
cos_psi = 1-psi^2/2;
sin_psi = psi-psi^3/6;


% compute dynamics (wheel slip term will be incorporated into g function)
zd = [v_des*cos_psi-lr*w_des*sin_psi ;
    v_des*sin_psi+lr*w_des*cos_psi ;
    w_des] ;

end
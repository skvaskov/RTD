%% description
% This script generates Figure 3 in the paper. It is the same as the script
% that generates tracking error data for the segway with modifcations to
% plotting
%
% Author: Shreyas Kousik
% Created: 10 Mar 2020
% Updated: 15 April 2020

%% user parameters
% initial condition bounds
w_0_min = -1.0 ; % rad/s
w_0_max = +1.0 ; % rad/s
v_0_min = +1.0 ; % m/s
v_0_max = +1.5 ; % m/s

% yaw rate command bounds
w_min = -1.0 ; % rad/s
w_max = +1.0 ; % rad/s
delta_w = +1.0 ; % rad/s

% speed command bounds
v_min = +0.0 ; % m/s
v_max = +1.5 ; % ms
delta_v = +0.5 ; % m/s

% number of samples in v_0, w, and v
N_samples = 5 ;

% polyfit info
g_x_fit_degree = 4 ; % default is 4
g_y_fit_degree = 3 ;

% file i/o
save_pdf_flag = true ;

% line color
g_color = [0,0.75,0.25];

%% automated from here
% create turtlebot
A = segway_agent() ;

% create initial condition vectors
w_0_vec = linspace(w_0_min,w_0_max,N_samples) ;
v_0_vec = linspace(v_0_min,v_0_max,N_samples) ;

% get time horizon of desired trajectory
t_f = get_t_f_from_v_0_for_segway(v_0_min) ;

% initialize arrays for saving tracking error data
x_err = [] ;
y_err = [] ;

% set up for display
N_samples_total = N_samples.^2 ;
idx_sample = 1 ;

%% tracking eror computation loop
disp('Computing tracking error')

start_tic = tic ;
for w_0 = w_0_vec
    %disp(['w_0 = ',num2str(w_0,'%0.2f')])
    
    % create yaw commands
    w_des_min = max(w_min, w_0 - delta_w) ;
    w_des_max = min(w_max, w_0 + delta_w) ;
    w_vec = linspace(w_des_min, w_des_max, N_samples) ;
    
    for v_0 = v_0_vec
        %disp(['   v_0 = ',num2str(v_0,'%0.2f')])
        
        % create the initial condition
        z_0 = [0;0;0;w_0;v_0] ; % (x,y,h,v)
        
        % create the feasible speed commands from the initial condition
        v_des_min = max(0, v_0 - delta_v) ;
        v_des_max = min(v_max, v_0 + delta_v) ;
        v_vec = linspace(v_des_min, v_des_max, N_samples) ;
        
        for w_des = w_vec
            % disp(['       w_des = ',num2str(w_des,'%0.2f')])
            
            for v_des = v_vec
                % disp(['v_des = ',num2str(v_des,'%0.2f')])
                % make the desired trajectory
                [T_des,U_des,Z_des] = make_segway_desired_trajectory(t_f,w_des,v_des) ;
                
                % reset the robot
                A.reset(z_0)
                
                % track the desired trajectory
                A.move(T_des(end),T_des,U_des,Z_des) ;
                
                % get the realized position trajectory
                T = A.time ;
                X = A.state(A.position_indices,:) ;
                
                % interpolate the desired and realized trajectory to match
                X_des = Z_des(1:2,:) ;
                X = match_trajectories(T_des,T,X) ;
                
                % compute the tracking error
                pos_err = X - X_des ;
                
                % collect the data
                x_err = [x_err ; pos_err(1,:)] ;
                y_err = [y_err ; pos_err(2,:)] ;
                
                % % FOR DEBUGGING:
                % figure(1) ; clf ; hold on ; axis equal; grid on ;
                % plot_path(X,'b--','LineWidth',1.5) ;
                % plot(A)
                % figure(2) ; clf ; plot(pos_err')
            end
        end
        
        % display progress
        current_duration = toc(start_tic) ;
        disp([num2str(100*idx_sample./N_samples_total,'%0.1f'),'% / ',num2str(current_duration,'%0.2f'),' s'])
        idx_sample = idx_sample + 1 ;
    end
end
total_time_spent = toc(start_tic) ;


%% fit tracking error function
% get max of absolute tracking error
x_err = abs(x_err) ;
y_err = abs(y_err) ;
x_max = max(x_err,[],1) ;
y_max = max(y_err,[],1) ;

% fit polynomial to the data
int_g_x_coeffs = polyfit(T_des,x_max,g_x_fit_degree) ;
int_g_y_coeffs = polyfit(T_des,y_max,g_y_fit_degree) ;

% take the time derivative of these to get the g functions in x and y
g_x_coeffs = polyder(int_g_x_coeffs) ;
g_y_coeffs = polyder(int_g_y_coeffs) ;

%% correct the fit to make it greater than the data
% evaluate g
int_g_x_coeffs = polyint(g_x_coeffs) ;
int_g_y_coeffs = polyint(g_y_coeffs) ;
int_g_x_vals = polyval(int_g_x_coeffs,T_des) ;
int_g_y_vals = polyval(int_g_y_coeffs,T_des) ;

% figure out the maximum ratio of the error data to the int g values
r_x_err = x_max ./ int_g_x_vals ;
r_x_max = max([1,r_x_err]) ;
r_y_err = y_max ./ int_g_y_vals ;
r_y_max = max([1,r_y_err]) ;

% multiply the g_x and g_y coefficients by the error data ratio
g_x_coeffs = r_x_max .* g_x_coeffs ;
g_y_coeffs = r_y_max .* g_y_coeffs ;

% re-integrate g with the new coefficients
int_g_x_coeffs = polyint(g_x_coeffs) ;
int_g_y_coeffs = polyint(g_y_coeffs) ;
int_g_x_vals = polyval(int_g_x_coeffs,T_des) ;
int_g_y_vals = polyval(int_g_y_coeffs,T_des) ;


%% plotting
fha = figure(1) ; clf ;

% plot x error
hold on ;
plot(T_des,x_err','b.')
g_x_handle =  plot(T_des,int_g_x_vals,'-','Color',g_color,'LineWidth',2.0) ;
xlabel('time [s]')
ylabel('x error [m]')
legend(g_x_handle,'\int g_x(t) dt','Location','NorthWest')
set(gca,'FontSize',15)
box on
pbaspect([2 1 1])

fhb = figure(2) ; clf ;
% plot y error
hold on ;
plot(T_des,y_err','b.')
g_y_handle = plot(T_des,int_g_y_vals,'-','Color',g_color,'LineWidth',2.0) ;
xlabel('time [s]')
ylabel('y error [m]')
legend(g_y_handle,'\int g_y(t) dt','Location','NorthWest')
box on
set(gca,'FontSize',15)
pbaspect([2 1 1])
%% save_figures
if save_pdf_flag
    save_figure_to_pdf(fha,'segway_gx.pdf')
    save_figure_to_pdf(fhb,'segway_gy.pdf')
end
    
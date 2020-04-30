function [path_figure, result_figure,Outline,ax1,ax2,line1,line2] = visualization(...
    AlgParams, Reference, VehicleParams, Vehicle_State, Control_State,...
    simulation_time)

screen_size = get(groot, 'Screensize'); 
screen_width = screen_size(3);
screen_height = screen_size(4);

path_figure = figure('name', 'Path Tracking', 'position',...
    [0, screen_height*1/7, screen_width/2, screen_height*3/4]);
hold on;
grid minor;
axis equal;
path_figure_title_name = set_title_name(AlgParams.type);
title(path_figure_title_name, 'fontsize', 15);
xlabel('X(m)', 'fontsize', 15);
ylabel('Y(m)', 'fontsize', 15);

plot(Reference.cx, Reference.cy, 'r.', 'markersize',3,'DisplayName','Reference');
Outline = plot_car(VehicleParams, Vehicle_State, Control_State);
% legend({'trajref', 'vehicle pose'}, 'fontsize', 12);

result_figure = figure('name', 'Path Tracking', 'position',...
    [screen_width/2, screen_height*1/7, screen_width/2, screen_height*3/4]);
ax1=subplot(2,1,1);
hold on;
grid minor;
steer_plot_title_name = set_title_name(AlgParams.type);
title(steer_plot_title_name, 'fontsize', 15);
xlabel('time(s)','fontsize', 15);
ylabel('steer command(deg)', 'fontsize', 15);
ylim([VehicleParams.min_steer_angle,VehicleParams.max_steer_angle]);
line1=plot(simulation_time, Control_State(1), 'b.', 'markersize', 15,'DisplayName','steer command');
% legend({'steer command'}, 'fontsize', 12);
ax2=subplot(2,1,2);
hold on;
grid minor;
error_plot_title_name = set_title_name(AlgParams.type);
title(error_plot_title_name, 'fontsize', 15);
xlabel('time(s)','fontsize', 15);
ylabel('error(m)', 'fontsize', 15);
[error, ~] = calc_nearest_point(Reference, Vehicle_State);
line2=plot(simulation_time, error, 'b.', 'markersize',15,'DisplayName','error');

    function title_name = set_title_name(path_tracking_alg)
    switch (path_tracking_alg)
        case 'Pure Pursuit'
            title_name = 'Path Tracking - Pure Pursuit';
        case 'Stanley'
            title_name = 'Path Tracking - Stanley';
        case 'Kinematics MPC'
            title_name = 'Path Tracking - Kinematics MPC';
        case 'Dynamics MPC'
            title_name = 'Path Tracking - Dynamics MPC';
        otherwise
            title_name = 'Error - No Algorithm';
            disp('There is no this path tracking algorithm!');
    end
    
function [path_figure,preview_point_global,result_figure,delta_line,error_line,solve_time_line]=...
    Visualization_Init(AlgParams,Reference,VehicleParams,Vehicle_State,Control_State,simulation_time)

screen_size = get(groot, 'Screensize'); 
screen_width = screen_size(3);
screen_height = screen_size(4);

path_figure = figure('name', 'Path Tracking', 'position',...
    [0, screen_height*1/7, screen_width/2, screen_height*3/4]);
hold on;
grid minor;
xlabel('X(m)', 'fontsize', 15);
ylabel('Y(m)', 'fontsize', 15);
title(['Path Tracking - ',AlgParams.type,'- ',Reference.type]);
cla;
switch (Reference.type)
    case {"eight" "road" }
        axis equal;
        plot(Reference.cx, Reference.cy, 'k.', 'markersize',2,'DisplayName','Reference');
        preview_point_global=plot(Vehicle_State(1),Vehicle_State(2));
        plot_car(VehicleParams, Vehicle_State,0);
    case "double"
        scale = 1.1;
        axis(scale*[min(Reference.cx),max(Reference.cx),min(Reference.cy),max(Reference.cy)]);
        plot(Reference.cx, Reference.cy, 'r.', 'markersize',3,'DisplayName','Reference');
        preview_point_global=plot(Vehicle_State(1),Vehicle_State(2),'b.','markersize',15);
end

result_figure = figure('name', 'Path Tracking', 'position',...
    [screen_width/2, screen_height*1/7, screen_width/2, screen_height*3/4]);
subplot(3,1,1);
hold on;
grid minor;
axis auto;
xlabel('time(s)','fontsize', 15);
ylabel('steer command(deg)', 'fontsize', 15);
delta_line=plot(simulation_time, Control_State(1), 'b.', 'markersize', 15,'DisplayName','delta command');

subplot(3,1,2);
hold on;
grid minor;
xlabel('time(s)','fontsize', 15);
ylabel('error(m)', 'fontsize', 15);
[error, ~] = calc_nearest_point(Reference, Vehicle_State);
error_line=plot(simulation_time, error, 'b.', 'markersize',15,'DisplayName','error');

subplot(3,1,3);
hold on;
grid minor;
title(error_plot_title_name, 'fontsize', 15);
xlabel('time(s)','fontsize', 15);
ylabel('error(m)','fontsize', 15);
[error, ~] = calc_nearest_point(Reference, Vehicle_State);
solve_time_line=plot(simulation_time, error, 'b.', 'markersize',15,'DisplayName','error');

    function title_name = set_Algorithm_name(path_tracking_alg)
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
    
function [path_figure,result_figure,delta_line,error_line,solve_time_line]=...
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
legend('show');
cla;
switch (Reference.type)
    case {'eight' 'road' }
        axis equal;
        plot(Reference.cx, Reference.cy, '-k.', 'markersize',5,'DisplayName','Reference');
        plot(Vehicle_State(1),Vehicle_State(2),'Marker','p','MarkerFaceColor','red','MarkerSize',12.0,'DisplayName','CoG');    
        plot_car(VehicleParams, Vehicle_State,0);
    case 'double'
        axis([min(Reference.cx)-1,max(Reference.cx)+1,min(Reference.cy)-1,max(Reference.cy)+1]);
        plot(Reference.cx, Reference.cy, 'r.','DisplayName','Reference');
        plot(Vehicle_State(1),Vehicle_State(2),'Marker','p','MarkerFaceColor','red','MarkerSize',12.0,'DisplayName','CoG');    
end

result_figure = figure('name', 'Path Tracking', 'position',...
    [screen_width/2, screen_height*1/7, screen_width/2, screen_height*3/4]);
subplot(3,1,1);
hold on;
grid minor;
axis auto;
xlabel('time(s)','fontsize', 10);
ylabel('steer command(deg)', 'fontsize', 10);
delta_line=plot(simulation_time, Control_State(1),'-k.','LineWidth',1,'markersize',1,'DisplayName','delta command');
legend('show');

subplot(3,1,2);
hold on;
grid minor;
xlabel('time(s)','fontsize', 10);
ylabel('error(m)', 'fontsize', 10);
[error, ~] = calc_nearest_point(Reference, Vehicle_State);
error_line=plot(simulation_time, error, '-k.','LineWidth', 1,'markersize',1,'DisplayName','error');
legend('show');

subplot(3,1,3);
hold on;
grid minor;
xlabel('time(s)','fontsize', 10);
ylabel('solve time(s)','fontsize', 10);
solve_time_line=plot(simulation_time, 0, '-k.','LineWidth',1, 'markersize',1,'DisplayName','solve time');
legend('show');
    
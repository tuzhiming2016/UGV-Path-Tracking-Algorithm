function [path_figure, result_figure,Outline] = visualization(...
    AlgParams, Reference, VehicleParams, Vehicle_State, Control_State,...
    simulation_time)
% 路径跟踪效果可视化：车辆跟踪效果、期望前轮偏角

% 输出:
% path_figure           : 路径跟踪效果的figure
% steer_figure          : 控制量 前轮偏角的figure

% 输入:
% AlgParams             : 路径跟踪算法
% Reference             : 期望路径
% VehicleParams         : 车辆参数
% Vehicle_State         : 车辆状态
% steer_state           : 前轮偏角
% simulation_time       : 当前仿真时间

% 1. 画出期望路径及车辆初始位姿状态
screen_size = get(groot, 'Screensize'); %获取电脑显示屏的宽度和高度
screen_width = screen_size(3);          %屏幕宽度
screen_height = screen_size(4);         %屏幕高度

path_figure = figure('name', 'Path Tracking', 'position',...
    [0, screen_height*1/7, screen_width/2, screen_height*3/4]);
hold on;
grid minor;
axis equal;
path_figure_title_name = set_title_name(AlgParams.type);
title(path_figure_title_name, 'fontsize', 15);  %设置title名称
xlabel('X(m)', 'fontsize', 15);         %x轴名称
ylabel('Y(m)', 'fontsize', 15);         %y轴名称

plot(Reference.cx, Reference.cy, 'r.', 'markersize',3,'DisplayName','Reference'); %期望轨迹可视化
Outline = plot_car(VehicleParams, Vehicle_State, Control_State);    %车辆初始位姿可视化
% legend({'trajref', 'vehicle pose'}, 'fontsize', 12);        %图例


% 2. 画出前轮偏角与误差
result_figure = figure('name', 'Path Tracking', 'position',...
    [screen_width/2, screen_height*1/7, screen_width/2, screen_height*3/4]);
subplot(2,1,1);
hold on;
grid minor;
steer_figure_title_name = set_title_name(AlgParams.type);
title(steer_figure_title_name, 'fontsize', 15);
% steer_figure_xlimit = simulation_stop_time;
% steer_figure_ylimit = veh_params.max_steer_angle / pi * 180;
% axis([0, steer_figure_xlimit, -steer_figure_ylimit, steer_figure_ylimit]);
xlabel('time(s)','fontsize', 15);
ylabel('steer command(deg)', 'fontsize', 15);
plot(simulation_time, Control_State, 'b.', 'markersize', 15);
legend({'steer command'}, 'fontsize', 12);
subplot(2,1,2);
hold on;
grid minor;
error_figure_title_name = set_title_name(AlgParams.type);
title(error_figure_title_name, 'fontsize', 15);
xlabel('time(s)','fontsize', 15);
ylabel('error(m)', 'fontsize', 15);
[error, ~] = calc_nearest_point(Reference, Vehicle_State);
plot(simulation_time, error, 'b.', 'markersize', 15);
legend({'error'}, 'fontsize', 12);


    function title_name = set_title_name(path_tracking_alg)
    % 设置使用的路径跟踪方法名称

    % 输出:
    % title_name        : 标题名称

    % 输入
    % path_tracking_alg : 路径跟踪算法
    % Pure Pursuit,Stanley,Kinematics MPC,Dynamics MPC
    switch (path_tracking_alg)
        % 根据所选路径跟踪算法指定figure的title
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
    
clear;
clc;
close all;
addpath('Params','TargetCourse');

Vehicle = "C-Class-Hatchback"; % B-Class-Hatchback C-Class-Hatchback
path_tracking_alg = "Pure Pursuit"; % Pure Pursuit,Stanley,Kinematics MPC,Dynamics MPC
roadmap_name = "road";  % eight road double

Reference = getTargetCourseParams(roadmap_name);
Reference = splinfy(Reference);
VehicleParams = getVehicleParams(Vehicle);
AlgParams = getAlgParams(path_tracking_alg,VehicleParams);
Reference.type = roadmap_name;
VehicleParams.type = Vehicle;
AlgParams.type = path_tracking_alg;
time_step = AlgParams.ts;

x0 = Reference.cx(10000);y0 = Reference.cy(10000);yaw0 = Reference.cyaw(10000);s0 = Reference.s(10000);
delta0 = 0;v0 = 20;w0 = 0;
desired_velocity = 20;
desired_angular_v = 0;

i = 0;simulation_time = 0;
Vehicle_State = [x0,y0,yaw0,s0,v0,w0];
Control_State = delta0;
[path_figure, result_figure,Outline] = visualization(AlgParams, Reference,... 
    VehicleParams, Vehicle_State, Control_State,simulation_time);
set(groot, 'CurrentFigure', path_figure);
preview_point_global = plot(x0,y0,'b*');
isGoal = norm(Vehicle_State(1:2)-[Reference.cx(end),Reference.cy(end)])<1^2 && (Reference.s(end)-Vehicle_State(4))<1;
% log = log_init(time_step, simulation_stop_time);
disp([path_tracking_alg,' simulation start!']);
while ~isGoal
    tic;
    i = i + 1;
    simulation_time = simulation_time + time_step;
    switch AlgParams.type
        case "Pure Pursuit"
            [steer_cmd,error,preview_point] = UGV_PP(Reference,VehicleParams,AlgParams,Vehicle_State,Control_State);
            set(groot, 'CurrentFigure', path_figure);
            set(preview_point_global,'XData',preview_point(1),'YData',preview_point(2));
        case "Stanley"
            [steer_cmd,error,front_point] = UGV_Stanley(Reference,VehicleParams,AlgParams,Vehicle_State,Control_State);
            set(groot, 'CurrentFigure', path_figure);
            set(preview_point_global,'XData',front_point(1),'YData',front_point(2));
        case "Kinematics MPC"
            Control_ref=[desired_velocity,desired_angular_v];
            [control_cmd,error,MPCprediction,qptime] = UGV_Kinematics_MPC(Reference,VehicleParams,AlgParams,Vehicle_State,Control_ref);
        case "Dynamics MPC"
            Control_State=[delta0,desired_velocity];
            [steer_cmd,error,MPCprediction,qptime] = UGV_Dynamics_MPC(Reference,VehicleParams,AlgParams,Vehicle_State,Control_State);
    end
    if AlgParams.type == "Pure Pursuit" || AlgParams.type == "Stanley" || AlgParams.type == "Dynamics MPC"
        wheel_base = VehicleParams.wheel_base;t=time_step;delta=steer_cmd;
        x0=Vehicle_State(1);y0=Vehicle_State(2);yaw0=Vehicle_State(3);s0=Vehicle_State(4);v0=Vehicle_State(5);
        x1=x0+v0*cos(yaw0)*t;y1=y0+v0*sin(yaw0)*t;yaw1=yaw0+v0/wheel_base*tan(delta)*t;s1=s0+v0*t;v1=v0;w1=(yaw1-yaw0)/t;
        Vehicle_State=[x1,y1,yaw1,s1,v1,w1];
%         syms x(t) y(t) yaw(t) s(t);
%         eqn1 = diff(x,t) == v0*cos(yaw); eqn2 = diff(y,t) == v0*sin(yaw);
%         eqn3 = diff(yaw,t) == v0*tan(steer_cmd)/wheel_base; eqn4 = diff(s,t) == v0;
%         cond1 = x(0) == x0;cond2 = y(0) == y0;cond3 = yaw(0) == yaw0;cond4 = s(0) == s0;
%         Up_State = dsolve(eqn1,eqn2,eqn3,eqn4,cond1,cond2,cond3,cond4);
%         t=time_step;
%         Vehicle_State = [eval([Up_State.x,Up_State.y,Up_State.yaw,eval(Up_State.s)]),v0,(eval(Up_State.yaw)-yaw0)/t];
        Vehicle_State(3)=wrapTo2Pi(Vehicle_State(3));
        set(groot,'CurrentFigure',path_figure);
%         delete(Outline);
        Outline = plot_car(VehicleParams, Vehicle_State, Control_State);
        pause(0.00001);
        toc;
    elseif AlgParams.type == "Kinematics MPC"
        wheel_base = VehicleParams.wheel_base;t=time_step;
        x0=Vehicle_State(1);y0=Vehicle_State(2);yaw0=Vehicle_State(3);s0=Vehicle_State(4);
        v0=control_cmd(1);w0=control_cmd(2);
        x1=x0+v0*cos(yaw0)*t;y1=y0+v0*sin(yaw0)*t;yaw1=yaw0+w0*t;s1=s0+v0*t;v1=v0;w1=(yaw1-yaw0)/t;
        Vehicle_State=[x1,y1,yaw1,s1,v1,w1];
        
%         wheel_base = VehicleParams.wheel_base;
%         x0=Vehicle_State(1);y0=Vehicle_State(2);yaw0=Vehicle_State(3);s0=Vehicle_State(4);
%         v0=control_cmd(1);w0=control_cmd(2);
%         syms x(t) y(t) yaw(t) s(t);
%         eqn1 = diff(x,t) == v0*cos(yaw); eqn2 = diff(y,t) == v0*sin(yaw);
%         eqn3 = diff(yaw,t) == w0; eqn4 = diff(s,t) == v0;
%         cond1 = x(0) == x0;cond2 = y(0) == y0;cond3 = yaw(0) == yaw0;cond4 = s(0) == s0;
%         Up_State = dsolve(eqn1,eqn2,eqn3,eqn4,cond1,cond2,cond3,cond4);
%         t=time_step;
%         Vehicle_State = [eval([Up_State.x,Up_State.y,Up_State.yaw,eval(Up_State.s)]),v0,(eval(Up_State.yaw)-yaw0)/t];
        Vehicle_State(3)=wrapTo2Pi(Vehicle_State(3));
        Control_State=[desired_velocity,desired_angular_v];
        set(groot,'CurrentFigure',path_figure);
        delete(Outline);
        Outline = plot_car(VehicleParams, Vehicle_State, Control_State);
        pause(0.00001);
        toc;
    end
    isGoal = norm(Vehicle_State(1:2)-[Reference.cx(end),Reference.cy(end)])<1^2 && (Reference.s(end)-Vehicle_State(4))<1;
end
disp([path_tracking_alg,'Get Goal ! simulation stop!']);
clear;
clc;
close all;
addpath('Params','TargetCourse');

%% Choose Vehicle Algrithm Course 
Vehicle = 'C-Class-Hatchback'; 
% B-Class-Hatchback C-Class-Hatchback
path_tracking_alg = 'Kinematics MPC V W'; 
% Pure Pursuit,Stanley,Kinematics MPC V Delta,Dynamics MPC,Kinematics MPC V W
roadmap_name = 'eight';
% eight road double

%% Get Params
Reference = getTargetCourseParams(roadmap_name);
Reference = splinfy(Reference);
VehicleParams = getVehicleParams(Vehicle);
AlgParams = getAlgParams(path_tracking_alg,VehicleParams);
Reference.type = roadmap_name;
VehicleParams.type = Vehicle;
AlgParams.type = path_tracking_alg;
time_step = AlgParams.ts;

%% Initialize State
x0 = Reference.cx(1000);y0 = Reference.cy(1000);yaw0 = Reference.cyaw(1000);s0 = Reference.s(1000);
delta0 = 0;v0 = 20;w0 = 0;vy0=0;
desired_velocity = 20;
desired_angular_v = 0;
desired_delta = 0;

i = 0;simulation_time = 0;
Vehicle_State = [x0,y0,yaw0,s0,v0,w0,vy0];
Control_State = delta0;

%% Log
log.i=i;log.time=simulation_time;
log.X=x0;log.Y=y0;log.Yaw=yaw0;log.Odometry=s0;
log.Vx=v0;log.Angular_V=w0;
log.delta=delta0;
log.error=0;log.solvertime=0;

[path_figure,result_figure,delta_line,error_line,solve_time_line]= Visualization_Init(AlgParams, Reference,... 
    VehicleParams, Vehicle_State, Control_State,simulation_time);

isGoal = norm(Vehicle_State(1:2)-[Reference.cx(end),Reference.cy(end)])<1 && (Reference.s(end)-Vehicle_State(4))<1;
disp([path_tracking_alg,' ',roadmap_name,' simulation start!']);

%% path tracking algrithm
while ~isGoal
    tic;
    i = i + 1;
    simulation_time = simulation_time + time_step;
    tic;
    switch AlgParams.type
        case "Pure Pursuit"
            [steer_cmd,error,preview_point] = UGV_PP(Reference,VehicleParams,AlgParams,Vehicle_State,Control_State);
        case "Stanley"
            [steer_cmd,error,preview_point] = UGV_Stanley(Reference,VehicleParams,AlgParams,Vehicle_State,Control_State);
        case "Kinematics MPC V W"
            Control_ref=[desired_velocity,desired_angular_v];
            [control_cmd,error,MPCprediction] = UGV_Kinematics_MPC_V_W(Reference,VehicleParams,AlgParams,Vehicle_State,Control_ref);
        case "Kinematics MPC V Delta"
            Control_ref=[desired_velocity,desired_delta];
            [control_cmd,error,MPCprediction] = UGV_Kinematics_MPC_V_Delta(Reference,VehicleParams,AlgParams,Vehicle_State,Control_ref);
        case "Dynamics MPC"
            Control_State=[delta0,desired_velocity];
            [steer_cmd,error,MPCprediction,update_state] = UGV_Dynamics_MPC(Reference,VehicleParams,AlgParams,Vehicle_State,Control_State);
    end
    toc;
    
%% update vehicle state
    if AlgParams.type == "Pure Pursuit" || AlgParams.type == "Stanley" || AlgParams.type == "Dynamics MPC" || AlgParams.type == "Kinematics MPC V Delta"
        wheel_base = VehicleParams.wheel_base;t=time_step;
        if AlgParams.type ~= "Kinematics MPC V Delta"
            delta=steer_cmd;v1=v0;
        else
            delta=control_cmd(2);v1=control_cmd(1);
        end
        x0=Vehicle_State(1);y0=Vehicle_State(2);yaw0=Vehicle_State(3);s0=Vehicle_State(4);v0=Vehicle_State(5);
        x1=x0+v0*cos(yaw0)*t;y1=y0+v0*sin(yaw0)*t;yaw1=yaw0+v0/wheel_base*tan(delta)*t;s1=s0+v0*t;w1=(yaw1-yaw0)/t;
        Vehicle_State=[x1,y1,yaw1,s1,v1,w1];
        Vehicle_State(3)=wrapTo2Pi(Vehicle_State(3));
        if AlgParams.type == "Dynamics MPC"
            Vehicle_State(7)=update_state(2);
        end
       
    elseif AlgParams.type == "Kinematics MPC V W"
        wheel_base = VehicleParams.wheel_base;t=time_step;
        x0=Vehicle_State(1);y0=Vehicle_State(2);yaw0=Vehicle_State(3);s0=Vehicle_State(4);
        v1=control_cmd(1);w1=control_cmd(2);
        x1=x0+v1*cos(yaw0)*t;y1=y0+v1*sin(yaw0)*t;yaw1=yaw0+w1*t;s1=s0+v1*t;
        Vehicle_State=[x1,y1,yaw1,s1,v1,w1];
        Vehicle_State(3)=wrapTo2Pi(Vehicle_State(3));
        delta = atan(w1*wheel_base/v1);
    end
    
    log.i(end+1)=i;log.time(end+1)=simulation_time;
    log.X(end+1)=x1;log.Y(end+1)=y1;log.Yaw(end+1)=yaw1;log.Odometry(end+1)=s1;
    log.Vx(end+1)=v1;log.Angular_V(end+1)=w1;log.delta(end+1)=delta;
    log.error(end+1)=error;log.solvertime(end+1)=toc;
    
%% show animation
    set(groot, 'CurrentFigure', path_figure);cla;
    switch (Reference.type)
        case {'eight' 'road'}
            axis([x1-40,x1+40,y1-40,y1+40]);
            plot_car(VehicleParams, Vehicle_State, delta);
        case {'double','Emergency'}
            
    end
    h1=plot(Reference.cx, Reference.cy, '-k.','LineWidth',3, 'markersize',3,'DisplayName','Target Trajectory');
    h2=plot(log.X, log.Y, '-b.','LineWidth', 3,'markersize',3,'DisplayName','Real Trajectory');
    h3=plot(Vehicle_State(1),Vehicle_State(2),'Marker','p','MarkerFaceColor','red','MarkerSize',12.0,'DisplayName','CoG');
    switch (AlgParams.type)
        case {"Pure Pursuit","Stanley"}
            h4=plot(preview_point(1),preview_point(2),'d','MarkerFaceColor','yellow','MarkerSize',12,'DisplayName','Preview Point');
            legend([h1 h2 h3 h4],{'Target Trajectory','Real Trajectory','CoG','Preview Point'});
        case {"Kinematics MPC V W","Kinematics MPC V Delta","Dynamics MPC"}
            h4=plot(MPCprediction(1,:),MPCprediction(2,:), '-y.','LineWidth', 3,'markersize',3,'DisplayName','Prediction Trajectory');
            legend([h1 h2 h3 h4],{'Target Trajectory','Real Trajectory','CoG','MPC Prediction Trajectory'});
    end
    title(['Time[s]:',num2str(round(simulation_time,3),3),'s',' Velocity[m/s]:',num2str(round(v1,2))]);
    
    set(groot, 'CurrentFigure', result_figure);
    set(delta_line,'Xdata',log.time,'Ydata',log.delta/pi*180);
    set(error_line,'Xdata',log.time,'Ydata',log.error);
    set(solve_time_line,'Xdata',log.time,'Ydata',log.solvertime);
    pause(0.0001);
    isGoal = norm(Vehicle_State(1:2)-[Reference.cx(end),Reference.cy(end)])<1^2 && (Reference.s(end)-Vehicle_State(4))<1;
end
disp([path_tracking_alg,' Get Goal ! simulation stop!']);






%         syms x(t) y(t) yaw(t) s(t);
%         eqn1 = diff(x,t) == v0*cos(yaw); eqn2 = diff(y,t) == v0*sin(yaw);
%         eqn3 = diff(yaw,t) == v0*tan(steer_cmd)/wheel_base; eqn4 = diff(s,t) == v0;
%         cond1 = x(0) == x0;cond2 = y(0) == y0;cond3 = yaw(0) == yaw0;cond4 = s(0) == s0;
%         Up_State = dsolve(eqn1,eqn2,eqn3,eqn4,cond1,cond2,cond3,cond4);
%         t=time_step;
%         Vehicle_State = [eval([Up_State.x,Up_State.y,Up_State.yaw,eval(Up_State.s)]),v0,(eval(Up_State.yaw)-yaw0)/t];

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
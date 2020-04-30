function [steer_cmd,error,MPCprediction,qptime] = UGV_Dynamics_MPC(Reference,VehicleParams,AlgParams,Vehicle_State,Control_State)
    [error, target_index] = calc_nearest_point(Reference, Vehicle_State);
    cx=Reference.cx;cy=Reference.cy;cyaw=Reference.cyaw;ck=Reference.ck;
    vel_pose=Vehicle_State(1:3);
    desired_velocity=Control_State(2);
    Longitudinal_V=Vehicle_State(5);Angular_V=Vehicle_State(6);
    p1=[cx(target_index),cy(target_index),cyaw(target_index)];
    p2=[cx(target_index+5),cy(target_index+5),cyaw(target_index+5)];
    proj_pose = calc_proj_pose(vel_pose, p1, p2);
    theta_des = cyaw(target_index);
    delta_pose = vel_pose-proj_pose;
    dx = delta_pose(1);dy = delta_pose(2);dyaw = wrapToPi(delta_pose(3));
    kesi0=zeros(4,1);
    kesi0(1)=dot([cos(theta_des+pi/2),sin(theta_des+pi/2)],[dx,dy]);
    kesi0(3)=dyaw;
    kesi0(2)=Longitudinal_V*sin(dyaw);
    kesi0(4)=Angular_V-desired_velocity*ck(target_index);
    [Ad,Bd,Cd]=getMatrices(AlgParams,Longitudinal_V,kesi0(4));
    [qptime,solved_steer_angle]=solve_MPC(Ad,Bd,Cd,VehicleParams,AlgParams,kesi0,Control_State);
    solved_steer_angle=wrapToPi(solved_steer_angle);
    steer_cmd = solved_steer_angle(1);
    MPCprediction = 1;
function proj_pose = calc_proj_pose(p0, p1, p2)
    % 求点p0在点p1和点p2组成直线上的投影点坐标及航向

    % 输出:
    % proj_point : 投影点位姿[x, y, theta]

    % 输入:
    % p0    : point0点位姿[x0, y0, theta0]
    % p1    : point0点位姿[x1, y1, theta1]
    % p2    : point0点位姿[x2, y2, theta2]

    tol = 0.0001;
    proj_pose = [0, 0, 0];

    if abs(p2(1) - p1(1)) < tol
        % p1和p2直线的斜率无穷大
        x = p1(1);     %投影点x坐标为p1的x坐标
        y = p0(2);     %投影点y坐标为p0的y坐标

    elseif abs(p2(2) - p1(2)) < tol
        % p1和p2直线的斜率无穷大
        x = p0(1);     %投影点x坐标为p1的x坐标
        y = p1(2);     %投影点y坐标为p0的y坐标

    else
        k1 = (p2(2) - p1(2)) / (p2(1) - p1(1)); %p1和p2的直线斜率
        k2 = -1 / k1;   %p0和投影点的直线斜率? two perpendicular line mutiply is -1

        x = (p0(2) - p1(2) + k1 * p1(1) - k2 * p0(1)) / (k1 - k2);
        y = p0(2) + k2 * (x - p0(1));
    end

    proj_pose(1) = x;
    proj_pose(2) = y;

    dist = norm(p2(1:2) - p1(1:2));         %点p1到点p2的距离
    dist2 = norm(p2(1:2) - proj_pose(1:2)); %投影点到点p2的距离

    ratio = dist2 / dist;
    theta = ratio * p1(3) + (1 - ratio) * p2(3);

    proj_pose(3) = theta;
    
function [Ad,Bd,Cd]=getMatrices(AlgParams,Vx,heading_error_rate)
    Ac=AlgParams.Ac;Bc=AlgParams.Bc;Cc=AlgParams.Cc;I=AlgParams.I;T=AlgParams.ts;
    Ac(2,2)=Ac(2,2)/Vx;Ac(2,4)=Ac(2,4)/Vx;Ac(4,2)=Ac(4,2)/Vx;Ac(4,4)=Ac(4,4)/Vx;
    Cc(2)=Cc(2)/Vx-Vx;Cc(4)=Cc(4)/Vx;
    Ad=(I+T*Ac/2)/(I-T*Ac/2);
    Bd=T*Bc;
    Cd=T*Cc*heading_error_rate;
    
function [qptime,solved_steer_angle]=solve_MPC(Ad,Bd,Cd,VehicleParams,AlgParams,kesi0,Control_State)
    switch (AlgParams.solver)
        case "quadprog"
            N=AlgParams.N;Nx=AlgParams.Nx;Nu=AlgParams.Nu;
            Q=AlgParams.Q;R=AlgParams.R;
            A_tilde_cell=cell(N,1);
            K_tilde_cell=cell(N,N);
            C_tilde_cell=cell(N,1);
            Q_tilde_cell=cell(N,N);
            R_tilde_cell=cell(N,N);
            for i=1:N
                A_tilde_cell{i,1}=Ad^i;
                C_tilde_cell{i,1}=zeros(size(Cd));
                for j=1:N
                    if i>=j
                        K_tilde_cell{i,j}=Ad^(i-j)*Bd;
                    else
                        K_tilde_cell{i,j}=zeros(Nx,Nu);
                    end
                    if i==j
                        Q_tilde_cell{i,j}=Q;
                        R_tilde_cell{i,j}=R;
                    else
                        Q_tilde_cell{i,j}=zeros(Nx,Nx);
                        R_tilde_cell{i,j}=zeros(Nu,Nu);
                    end
                end
                for j=1:i
                    C_tilde_cell{i,1}= C_tilde_cell{i,1}+Ad^(j-1)*Cd;
                end
            end
            A_tilde=cell2mat(A_tilde_cell);
            K_tilde=cell2mat(K_tilde_cell);
            C_tilde=cell2mat(C_tilde_cell);
            Q_tilde=cell2mat(Q_tilde_cell);
            R_tilde=cell2mat(R_tilde_cell);
            ERROR = A_tilde*kesi0 + C_tilde;
            
            H=2*K_tilde'*Q_tilde*K_tilde+2*R_tilde;
            H=(H+H')/2;
            f=(2*ERROR'*Q_tilde*K_tilde)';
            ub=kron(ones(N,1),VehicleParams.max_steer_angle/180*pi);
            lb=kron(ones(N,1),VehicleParams.min_steer_angle/180*pi);
            last_steer=Control_State(1);
            x0=kron(ones(N,1),last_steer);
            options=optimset('Algorithm','interior-point-convex','Display', 'iter');
            tic
            [x,~,exitflag,~]=quadprog(H,f,[],[],[],[],lb,ub,x0,options);
            if exitflag==1
                qptime=toc;
                solved_steer_angle=x;
            else
                solved_steer_angle=last_steer*ones(N,1);
            end
    end
            
            
            
            
        
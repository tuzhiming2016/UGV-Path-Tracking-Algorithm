function [steer_cmd,error,MPCprediction,update_state] = UGV_Dynamics_MPC(Reference,VehicleParams,AlgParams,Vehicle_State,Control_State)
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
    solved_steer_angle=solve_MPC(Ad,Bd,Cd,VehicleParams,AlgParams,kesi0,Control_State);
    solved_steer_angle=wrapToPi(solved_steer_angle);
    steer_cmd = solved_steer_angle(1);
    
    x=Vehicle_State(1);y=Vehicle_State(2);yaw=Vehicle_State(3);
    Vy=Vehicle_State(7);Wz=Angular_V;
    x0=[0;Vy;0;Wz;];
    
    MPCout = predict_motion(AlgParams,Longitudinal_V,x0,solved_steer_angle);
    update_state = MPCout(:,1);
    Tsim = AlgParams.N;t = AlgParams.ts;
    local_x = Longitudinal_V*t*(1:Tsim);
    local_y = MPCout(1,:);
    local_yaw = MPCout(3,:);
    rotA = [cos(-yaw) sin(-yaw);-sin(-yaw) cos(-yaw);];
    MPCprediction(1:2,:)=rotA*[local_x;local_y;]+repmat([x;y],1,Tsim);
    MPCprediction(3,:)=wrapTo2Pi(local_yaw+yaw);

    
function proj_pose = calc_proj_pose(p0, p1, p2)
    tol = 0.0001;
    proj_pose = [0, 0, 0];
    if abs(p2(1) - p1(1)) < tol
        x = p1(1);
        y = p0(2);
    elseif abs(p2(2) - p1(2)) < tol
        x = p0(1);
        y = p1(2);
    else
        k1 = (p2(2) - p1(2)) / (p2(1) - p1(1));
        k2 = -1 / k1;
        x = (p0(2) - p1(2) + k1 * p1(1) - k2 * p0(1)) / (k1 - k2);
        y = p0(2) + k2 * (x - p0(1));
    end
    proj_pose(1) = x;
    proj_pose(2) = y;
    dist = norm(p2(1:2) - p1(1:2)); 
    dist2 = norm(p2(1:2) - proj_pose(1:2)); 
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
    
function solved_steer_angle=solve_MPC(Ad,Bd,Cd,VehicleParams,AlgParams,kesi0,Control_State)
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
                solved_steer_angle=x;
            else
                solved_steer_angle=last_steer*ones(N,1);
            end
    end
    
function MPCprediction = predict_motion(AlgParams,Vx,x0,U)
    A=AlgParams.matrix_A;
    B=AlgParams.matrix_B;
    A(2,2)=A(2,2)/Vx;A(2,4)=A(2,4)/Vx-Vx;
    A(4,2)=A(4,2)/Vx;A(4,4)=A(4,4)/Vx;
    Tsim=AlgParams.N;
    Nx=AlgParams.Nx;Nu=AlgParams.Nu;T=AlgParams.ts;
    A=T*A+eye(Nx);B=T*B;
    Abar_cell=cell(Tsim,1);
    Bbar_cell=cell(Tsim,Tsim);
    for i=1:Tsim
        Abar_cell{i,1}=A^i;
        for j=1:Tsim
            if i>=j
                Bbar_cell{i,j}=A^(i-j)*B;
            else
                Bbar_cell{i,j}=zeros(Nx,Nu);
            end
        end
    end
    Abar=cell2mat(Abar_cell);
    Bbar=cell2mat(Bbar_cell);
    MPCprediction = Abar*x0+Bbar*U;
    MPCprediction = reshape(MPCprediction,Nx,Tsim);
    MPCprediction(3,:) = wrapTo2Pi(MPCprediction(3,:));
            
            
            
        
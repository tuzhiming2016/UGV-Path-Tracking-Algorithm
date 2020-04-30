function [control_cmd,error,MPCprediction,qptime] = UGV_Kinematics_MPC(Reference,VehicleParams,AlgParams,Vehicle_State,Control_ref)
    [error, target_index] = calc_nearest_point(Reference, Vehicle_State);
    cx=Reference.cx;cy=Reference.cy;cyaw=Reference.cyaw;
    xref=cx(target_index);yref=cy(target_index);yawref=cyaw(target_index);
    v_ref=Control_ref(1);w_ref=Control_ref(2);
    state_ref=[xref;yref;yawref;];
    state_actual=Vehicle_State(1:3)';
    delta_state=state_actual-state_ref;
    delta_state(3)=wrapToPi(delta_state(3));
    Nx=AlgParams.Nx;Nu=AlgParams.Nu;Tsim=AlgParams.N;T=AlgParams.ts;
    Ad=[0 0 -v_ref*sin(yawref);
      0 0 v_ref*cos(yawref);
      0 0 0;]*T+eye(Nx);
    Bd=[cos(yawref)   0;
      sin(yawref)   0;
      0  1;]*T;
    Abar_cell=cell(Tsim,1);
    Bbar_cell=cell(Tsim,Tsim);
    for i=1:Tsim
        Abar_cell{i,1}=Ad^i;
        for j=1:Tsim
            if i>=j
                Bbar_cell{i,j}=Ad^(i-j)*Bd;
            else
                Bbar_cell{i,j}=zeros(Nx,Nu);
            end
        end
    end
    Abar=cell2mat(Abar_cell);
    Bbar=cell2mat(Bbar_cell);
    Qbar=AlgParams.Qbar;
    Rbar=AlgParams.Rbar;
    H=2*(Bbar'*Qbar*Bbar+Rbar);
    H=(H+H')/2;
    f=2*(Bbar'*Qbar*Abar*delta_state);
    ub=kron(ones(Tsim,1),[VehicleParams.Vmax-v_ref;VehicleParams.Wmax-w_ref;]);
    lb=kron(ones(Tsim,1),[VehicleParams.Vmin-v_ref;VehicleParams.Wmin-w_ref;]);
    tic;
    options=optimset('Algorithm','interior-point-convex','Display', 'iter');
    [Ubar,~,exitflag,~]=quadprog(H,f,[],[],[],[],lb,ub,zeros(size(ub)),options);  
    if exitflag==1
        MPCprediction = Abar*delta_state+Bbar*Ubar;
        qptime=toc;
    end
    Ubar(1:2)=Ubar(1:2)+[0.0001;0.0001];
    control_cmd = Ubar(1:2)+[v_ref;w_ref];
end

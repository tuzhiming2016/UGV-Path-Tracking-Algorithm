function AlgParams = getAlgParams(path_tracking_alg, VehicleParams)
    if path_tracking_alg == "Pure Pursuit"
        AlgParams.k = 0.1;                    %预瞄距离速度系数
        AlgParams.min_preview_dist = 3;      %最小预瞄距离, m 3
        AlgParams.max_preview_dist = 25;     %最大预瞄距离, m 25
    end
    if path_tracking_alg == "Stanley"
        AlgParams.k = 1;                    %速度增益系数
        AlgParams.min_preview_dist = 3;      %最小预瞄距离, m 3
        AlgParams.max_preview_dist = 25;     %最大预瞄距离, m 25
    end
    
    if path_tracking_alg == "Kinematics MPC"
        solver = "quadprog";
        N = 10;
        Nx = 3;
        Nu = 2;
        q1=1;q2=1;q3=0.5;
        r1=0.1;r2=0.1;rd=0;
        AlgParams.N=10;
        AlgParams.Nx=Nx;
        AlgParams.Nu=Nu;
        AlgParams.N=N;
        AlgParams.Q=diag([q1 q2 q3]);
        AlgParams.R=diag([r1 r2]);
        AlgParams.Rd=diag(rd);
        AlgParams.Qbar=diag(diag(repmat(AlgParams.Q,N,N)));
        AlgParams.Rbar=diag(diag(repmat(AlgParams.R,N,N)));
        AlgParams.Rdbar=diag(diag(repmat(AlgParams.Rd,N,N)));
        AlgParams.solver = solver;
    end
    
    if path_tracking_alg == "Dynamics MPC"
        solver = "quadprog";
        cf = VehicleParams.Cf; 
        cr = VehicleParams.Cr;
        mass = VehicleParams.mass;
        lf = VehicleParams.wheel_base_front;
        lr = VehicleParams.wheel_base_rear;
        iz = VehicleParams.Izz; 

        N = 10;
        Nx = 4;
        Nu = 1;
        q1=0.05;q2=0;q3=1;q4=0;
        r=1;rd=0;
        
        I=eye(Nx,Nx);
        matrix_a=zeros(Nx,Nx);
        matrix_a(1,2) = 1.0;
        matrix_a(2,2) = -2*(cf + cr) / mass;
        matrix_a(2,3) = 2*(cf + cr ) / mass;
        matrix_a(2,4) = 2*(lr * cr - lf * cf) / mass;
        matrix_a(3,4) = 1.0;
        matrix_a(4,2) = 2*(lr * cr - lf * cf) / iz;
        matrix_a(4,3) = 2*(lf * cf - lr * cr) / iz;
        matrix_a(4,4) = -2 * (lf * lf * cf + lr * lr * cr)/iz;
        matrix_b=zeros(Nx, Nu);
        matrix_b(2,1) = 2*cf/mass;
        matrix_b(4,1) = 2*lf * cf /iz;
        matrix_c=zeros(Nu,1);
        matrix_c(2,1)=2*(cr*lr-cf*lf)/mass;
        matrix_c(4,1)=-2*(cf*lf*lf+cr*lr*lr)/iz;

        AlgParams.Nx=Nx;
        AlgParams.Nu=Nu;
        AlgParams.Q=diag([q1,q2,q3,q4]);
        AlgParams.R=diag(r);
        AlgParams.Rd=diag(rd);
        AlgParams.N=N;
        AlgParams.Ac=matrix_a;
        AlgParams.Bc=matrix_b;
        AlgParams.Cc=matrix_c;
        AlgParams.I=I;
        AlgParams.lr=lr;
        AlgParams.lf=lf;
        AlgParams.cf=cf;
        AlgParams.cr=cr;
        AlgParams.iz=iz;
        AlgParams.mass=mass;
        AlgParams.solver=solver;
    end
        AlgParams.ts = 0.05; % Control time period
end
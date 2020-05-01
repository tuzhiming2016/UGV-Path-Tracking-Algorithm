function VehicleParams = getVehicleParams(Vehicle)
    if Vehicle == "C-Class-Hatchback"
        VehicleParams = load('VehicleParams_C_Class.mat');
        Width=VehicleParams.Width;
        wheel_base=VehicleParams.wheel_base;
        wheel_base_front=VehicleParams.wheel_base_front;
        wheel_base_rear=VehicleParams.wheel_base_rear;
        Lr=VehicleParams.Lr;
        Lf=0.6;
        TireWidth=VehicleParams.tire.tirewidth;
        TireR=VehicleParams.tire.tireR;
        
        Tread=0.85*Width;
        Length=wheel_base+Lr+Lf;
        Length_front=wheel_base_front+Lf;
        Length_rear=wheel_base_rear+Lr;
        front_axle_center=[wheel_base_front;0];
        rear_axle_center=[-wheel_base_rear;0];
        longitudinal_axle=[front_axle_center,rear_axle_center];
        
        Outline=[Length_front,-Length_rear,-Length_rear,Length_front,Length_front;
            Width/2,Width/2,-Width/2,-Width/2,Width/2;];
        tire_fl=[TireR,-TireR,-TireR,TireR,TireR;
            (Tread+TireWidth)/2,(Tread+TireWidth)/2,(Tread-TireWidth)/2,(Tread-TireWidth)/2,(Tread+TireWidth)/2;];
        tire_rl=tire_fl;tire_fr=tire_fl;tire_rr=tire_fl;
        tire_fr(2,:)=tire_fr(2,:)*(-1);tire_rr(2,:)=tire_rr(2,:)*(-1);
        tire_fl_center=[0;Tread/2;];tire_fr_center=[0;-Tread/2;];
        tire_rl_center=[0;Tread/2;];tire_rr_center=[0;-Tread/2;];
        
        front_axle=[tire_fl_center,tire_fr_center;];
        rear_axle=[tire_rl_center,tire_rr_center;];
        
        Wmax=0.85*1.0*9.806/10;Wmin=-0.85*1.0*9.806/10;
        max_steer_angle=33;min_steer_angle=-33;
        max_steer_angular_vel=9.45;min_steer_angular_vel=-9.45;
        Vmax=150;Vmin=-60;

        VehicleParams.Lf=Lf;
        VehicleParams.Length=Length;
        VehicleParams.Tread=Tread;
        VehicleParams.front_axle_center=front_axle_center;
        VehicleParams.rear_axle_center=rear_axle_center;
        VehicleParams.longitudinal_axle=longitudinal_axle;
        VehicleParams.front_axle=front_axle;
        VehicleParams.rear_axle=rear_axle;
        VehicleParams.Outline=Outline;
        VehicleParams.Tire_fl=tire_fl;VehicleParams.Tire_fr=tire_fr;
        VehicleParams.Tire_rl=tire_rl;VehicleParams.Tire_rr=tire_rr;
        VehicleParams.Tire_fl_center=tire_fl_center; VehicleParams.Tire_fr_center=tire_fr_center;
        VehicleParams.Tire_rl_center=tire_rl_center; VehicleParams.Tire_rr_center=tire_rr_center;
        VehicleParams.Wmax=Wmax;
        VehicleParams.Wmin=Wmin;
        VehicleParams.max_steer_angle=max_steer_angle;
        VehicleParams.min_steer_angle=min_steer_angle;
        VehicleParams.max_steer_angular_vel=max_steer_angular_vel;
        VehicleParams.min_steer_angular_vel=min_steer_angular_vel;
        VehicleParams.Vmax=Vmax;
        VehicleParams.Vmin=Vmin;
    end
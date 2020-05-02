function plot_car(VehicleParams, Vehicle_State, steer_state)
        CoG = [Vehicle_State(1);Vehicle_State(2)];
        yaw = Vehicle_State(3);
        delta = steer_state;

        wheel_base_front = VehicleParams.wheel_base_front;
        wheel_base_rear = VehicleParams.wheel_base_rear;

        front_axle = VehicleParams.front_axle;
        rear_axle = VehicleParams.rear_axle;
        longitudinal_axle = VehicleParams.longitudinal_axle;

        outline = VehicleParams.Outline;
        tire_fl = VehicleParams.Tire_fl;
        tire_fr = VehicleParams.Tire_fr;
        tire_rl = VehicleParams.Tire_rl;
        tire_rr = VehicleParams.Tire_rr;

        rotA = [cos(yaw) -sin(yaw);sin(yaw) cos(yaw);];
        rotB = [cos(delta) -sin(delta);sin(delta) cos(delta);];

        tire_fl = rotB*tire_fl; tire_fl(1,:)=tire_fl(1,:)+wheel_base_front;
        tire_fr = rotB*tire_fr; tire_fr(1,:)=tire_fr(1,:)+wheel_base_front;
        front_axle = rotB*front_axle; front_axle(1,:)=front_axle(1,:)+wheel_base_front;
        tire_rl(1,:) = tire_rl(1,:)-wheel_base_rear;
        tire_rr(1,:) = tire_rr(1,:)-wheel_base_rear;
        rear_axle(1,:)=rear_axle(1,:)-wheel_base_rear;

        outline = rotA*outline;
        outline = outline + repmat(CoG,1,5);
        tire_fl = rotA*tire_fl + repmat(CoG,1,5);
        tire_fr = rotA*tire_fr + repmat(CoG,1,5);
        tire_rl = rotA*tire_rl + repmat(CoG,1,5);
        tire_rr = rotA*tire_rr + repmat(CoG,1,5);
        front_axle = rotA*front_axle + CoG;
        rear_axle = rotA*rear_axle + CoG;
        longitudinal_axle = rotA*longitudinal_axle + CoG;

        plot(outline(1,:),outline(2,:),'Color','black','LineWidth',1.5);
        plot(tire_fl(1,:),tire_fl(2,:),'Color','blue','LineWidth',1.2);
        plot(tire_fr(1,:),tire_fr(2,:),'Color','blue','LineWidth',1.2);
        plot(tire_rl(1,:),tire_rl(2,:),'Color','blue','LineWidth',1.2);
        plot(tire_rr(1,:),tire_rr(2,:),'Color','blue','LineWidth',1.2);
        plot(front_axle(1,:),front_axle(2,:),'Color','black','LineWidth',1.5);
        plot(rear_axle(1,:),rear_axle(2,:),'Color','black','LineWidth',1.5);
        plot(longitudinal_axle(1,:),longitudinal_axle(2,:),'Color','black','LineWidth',1.5);
end
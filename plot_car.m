function Outline = plot_car(VehicleParams, Vehicle_State, steer_state)
    CoG = [Vehicle_State(1),Vehicle_State(2)];
    yaw = Vehicle_State(3);
    delta = steer_state;
    
    wheel_base_front = VehicleParams.wheel_base_front;
    wheel_base_rear = VehicleParams.wheel_base_rear;
    Lf = VehicleParams.Lf;
    Lr = VehicleParams.Lr;
    Width = VehicleParams.Width;
    tmp_Length_front = wheel_base_front + Lf;
    tmp_Length_rear = wheel_base_rear + Lr;
    
    Vehicle_rear_left = CoG - tmp_Length_rear.*[cos(yaw) sin(yaw)] + Width.*[cos(yaw+pi/2) sin(yaw+pi/2)];
    Vehicle_rear_right = CoG - tmp_Length_rear.*[cos(yaw) sin(yaw)] + Width.*[cos(yaw-pi/2) sin(yaw-pi/2)];
    Vehicle_front_left = CoG + tmp_Length_front.*[cos(yaw) sin(yaw)] + Width.*[cos(yaw+pi/2) sin(yaw+pi/2)];
    Vehicle_front_right = CoG + tmp_Length_front.*[cos(yaw) sin(yaw)] + Width.*[cos(yaw-pi/2) sin(yaw-pi/2)];
    Vehicle_Outline=[Vehicle_rear_left;Vehicle_rear_right;Vehicle_front_right;Vehicle_front_left;Vehicle_rear_left;];
    Outline=line(Vehicle_Outline(:,1),Vehicle_Outline(:,2),'Color','black','LineWidth',1.5);
    legend(Outline,{'Vehicle'});
%     Vehicle_rear_left = [tmp_Length_rear; Width];
%     Vehicle_rear_right = [tmp_Length_rear; -Width];
%     Vehicle_front_left = [tmp_Length_rear; Width];
%     Vehicle_front_right = [tmp_Length_rear;-Width];
%     rotA = [cos(yaw) sin(yaw);-sin(yaw) cos(yaw);];
%     Vehicle_rear_left = CoG + rotA * (Vehicle_rear_left);
%     Vehicle_rear_right = CoG + rotA * (Vehicle_rear_right);
%     Vehicle_front_left = CoG + rotA * (Vehicle_front_left);
%     Vehicle_front_right = CoG + rotA * (Vehicle_front_right);
%     Vehicle_Outline=[Vehicle_rear_left;Vehicle_rear_right;Vehicle_front_right;Vehicle_front_left;Vehicle_rear_left;];
%     Outline=line(Vehicle_Outline(:,1),Vehicle_Outline(:,2),'Color','black','LineWidth',1.5);

%     tireWidth = VehicleParams.tire.tirewidth;
%     tireR = VehicleParams.tire.tireR;
%     tire_rear_left_CoG = CoG - wheel_base_rear.*[cos(yaw) sin(yaw)] + (Width-tireWidth).*[cos(yaw+pi/2) sin(yaw+pi/2)];
%     tire_rear_right_CoG = CoG - wheel_base_rear.*[cos(yaw) sin(yaw)] + (Width-tireWidth).*[cos(yaw-pi/2) sin(yaw-pi/2)];
%     tire_front_left_CoG = CoG + wheel_base_front.*[cos(yaw) sin(yaw)] + (Width-tireWidth).*[cos(yaw+pi/2) sin(yaw+pi/2)];
%     tire_front_right_CoG =CoG + wheel_base_front.*[cos(yaw) sin(yaw)] + (Width-tireWidth).*[cos(yaw-pi/2) sin(yaw-pi/2)];
    
end
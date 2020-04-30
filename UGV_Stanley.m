function [steer_cmd,error,front_point] = UGV_Stanley(Reference,VehicleParams,AlgParams,Vehicle_State,Control_State)
    yaw = Vehicle_State(3);v = Vehicle_State(5);
    Vehicle_State_Front = Vehicle_State;
    Vehicle_State_Front(1:2) = Vehicle_State(1:2) + VehicleParams.wheel_base_front*[cos(yaw),sin(yaw)];
    [front_axle_error, target_index] = calc_nearest_point(Reference, Vehicle_State_Front);
    
    cyaw=Reference.cyaw;
    k = AlgParams.k;
    theta_p = cyaw(target_index);
    theta_e = wrapToPi(theta_p - yaw);
    theta_d = atan2(k * front_axle_error, v);
    steer_cmd = theta_e + theta_d;
    error = front_axle_error;
    front_point = [Reference.cx(target_index),Reference.cy(target_index)];

%     min_preview_dist = AlgParams.min_preview_dist;
%     max_preview_dist = AlgParams.max_preview_dist;
%     Ts = AlgParams.ts;
    
%     front_axle_vector = [cos(theta_p+pi/2) sin(theta_p+pi/2);];
%     front_point = [Reference.cx(target_index),Reference.cy(target_index)];
%     front_axle_error = dot(front_point-[x,y],front_axle_vector);

%     preview_dist = k * v;
%     preview_dist = max(min_preview_dist, preview_dist);
%     preview_dist = min(max_preview_dist, preview_dist);
    
%     dist = sqrt((cx(target_index:target_index+2*max_preview_dist/ds)-x).^2+(cy(target_index:target_index+2*max_preview_dist/ds)-y).^2);
%     tmp_index = find((dist- preview_dist)>=0);
%     preview_index = target_index + tmp_index(1) - 1;
%     preview_point_global = [cx(preview_index);cy(preview_index)];
%     rotA =[cos(yaw) sin(yaw);-sin(yaw) cos(yaw);];
%     preview_point_local = rotA * (preview_point_global - [x;y]);
%     preview_dist = norm(preview_point_local);
%     e = preview_point_local(2);
%     alpha = asin(e/preview_dist); 

    delta0 = Control_State(1);
%     steer_cmd = min(delta0+VehicleParams.max_steer_angular_vel*pi/180*Ts,steer_cmd);
%     steer_cmd = max(delta0+VehicleParams.min_steer_angular_vel*pi/180*Ts,steer_cmd);
    steer_cmd = min(VehicleParams.max_steer_angle*pi/180,steer_cmd);
    steer_cmd = max(VehicleParams.min_steer_angle*pi/180,steer_cmd);
end

function [steer_cmd,error,preview_point_global] = UGV_PP(Reference,VehicleParams,AlgParams,Vehicle_State,Control_State)
    [error, target_index] = calc_nearest_point(Reference, Vehicle_State);
    
    k = AlgParams.k;
    min_preview_dist = AlgParams.min_preview_dist;
    max_preview_dist = AlgParams.max_preview_dist;
    Ts = AlgParams.ts;
    cx=Reference.cx;cy=Reference.cy;ds=Reference.ds;
    
    x = Vehicle_State(1);y = Vehicle_State(2);
    yaw = Vehicle_State(3);v = Vehicle_State(5);delta0 = Control_State(1);
    preview_dist = k * v;
    preview_dist = max(min_preview_dist, preview_dist);
    preview_dist = min(max_preview_dist, preview_dist);
    
    last_search_index = min(target_index+2*max_preview_dist/ds, length(cx));
    dist = sqrt((cx(target_index:last_search_index)-x).^2+(cy(target_index:last_search_index)-y).^2);
    tmp_index = find((dist- preview_dist)>=0);
    if isempty(tmp_index)
        preview_index = length(cx);
    else
        preview_index = target_index + tmp_index(1) - 1;
    end
    preview_point_global = [cx(preview_index);cy(preview_index)];
    rotA =[cos(yaw) sin(yaw);-sin(yaw) cos(yaw);];
    preview_point_local = rotA * (preview_point_global - [x;y]);
    preview_dist = norm(preview_point_local);
    e = preview_point_local(2);
    alpha = asin(e/preview_dist);
    
    steer_cmd = atan(2*VehicleParams.wheel_base*sin(alpha)/preview_dist);
%     steer_cmd = min(delta0+VehicleParams.max_steer_angular_vel*pi/180*Ts,steer_cmd);
%     steer_cmd = max(delta0+VehicleParams.min_steer_angular_vel*pi/180*Ts,steer_cmd);
    steer_cmd = min(VehicleParams.max_steer_angle*pi/180,steer_cmd);
    steer_cmd = max(VehicleParams.min_steer_angle*pi/180,steer_cmd);
    
end
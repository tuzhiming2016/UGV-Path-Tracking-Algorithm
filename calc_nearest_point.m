function [error, target_index] = calc_nearest_point(Reference, Vehicle_State)
    cx = Reference.cx;cy = Reference.cy;cyaw = Reference.cyaw;s = Reference.s;
    x = Vehicle_State(1);y = Vehicle_State(2);Odometer = Vehicle_State(4);
    sequence =  find(abs(s-Odometer)<5);
    distance = sqrt((cx(sequence)-x).^2 + (cy(sequence)-y).^2);
    [~,minindex] = min(distance);
    target_index = minindex + sequence(1)-1;
    theta_path = cyaw(target_index);
    path_vector = [cos(theta_path+pi/2) sin(theta_path+pi/2)];
    ref_point = [cx(target_index) cy(target_index)];
    error = dot(ref_point-[x,y],path_vector);
%     p0 = Vehicle_State(1:3);
%     p1 = [cx(target_index),cy(target_index),cyaw(target_index)];
%     p2 = [cx(target_index+1),cy(target_index+1),cyaw(target_index+1)];
%     ref_point = calc_proj_pose(p0,p1,p2);


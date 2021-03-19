function [q1, q2, q3, q4] = inverse_kinematics(x_des,y_des,z_des,phi_des)
%INVERSE_KINEMATICS Given end-effector position and orientation ([x,y,z] and phi) find joint angles ([q1, q2, q3, q4]).
%   Detailed explanation goes here

l1 = 0.3;
l2 = 2.5;
l3 = 1.5;
l4 = 0;

% Calculate the radial desired coordinate
r = sqrt(x_des^2 + y_des^2);

% Calculate yaw angle
q1 = atan2(y_des, x_des);

% Calculate support variables
r_ = r - l4*cos(phi_des);
z_ = z_des - l4*sin(phi_des)-l1;

gamma = atan2(-z_/sqrt(r_^2 + z_^2),-r_/sqrt(r_^2 + z_^2));

sigma = -1;

% Calculate pitch angles
q2 = gamma + sigma*acos(-(r_^2 + z_^2 + l2^2 - l3^2)/(2*l2*sqrt(r_^2 + z_^2)));
q3 = atan2((z_-l2*sin(q2))/l3,(r_-l2*cos(q2))/l3) - q2;
q4 = phi_des - (q2 + q3); 

end


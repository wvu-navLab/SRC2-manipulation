clear all, close all, clc;

mdl_src2m;
src2m.base.t = [0;0;0];    
src2m.plot(qz);

Q = [   0,      0,      0,      0;
        0,      pi/3,   -pi/3,  0;
        pi/2,   pi/3,   -pi/3,  0;
        pi/2,   pi/3,   -pi/3,  pi;
        pi/2,   -pi/6,  -pi/3,  pi;
        pi/2,   -pi/6,  -pi/3,  pi/2;
        pi/2,   pi/3,   -pi/3,  0;
        -pi/4,  pi/3,   -pi/3,  0;
        -pi/4,  pi/3,   -pi/3,  pi;
        0,      pi/3,   -pi/3,  0];

%% Link Lengths

l1 = 0.3;
l2 = 2.5;
l3 = 1.5;
l4 = 0;

%% Forward Kinematics
%Given joint angles ([q1, q2, q3, q4]) find end-effector position and
%orientation ([x,y,z] and phi).

q_input = [pi/3, pi/3, -pi/3, 0];
phi_EE = q_input(2)+q_input(3)+q_input(4);

r       = l2*cos(q_input(2)) + l3*cos(phi_EE-q_input(4)) + l4*cos(phi_EE);
x_EE    = r * cos(q_input(1));
y_EE    = r * sin(q_input(1));
z_EE    = l1 + l2*sin(q_input(2)) + l3*sin(phi_EE-q_input(4)) + l4*sin(phi_EE);

pos_EE = [x_EE, y_EE, z_EE];

t=zeros(size(Q,1),3);
for i=1:size(Q,1)
    T = src2m.fkine(Q(i,:));
    t(i,:) = T.t';
    src2m.animate(Q(i,:))
end


T = src2m.fkine(q_input);    
home_position = T.t;

%% Inverse Kinematics

pos_des = [1.375, 2.382, 2.465];
phi_des = deg2rad(90);

x_des = pos_des(1);
y_des = pos_des(2);

z_des = pos_des(3);

% Call the inverse kinematics function
[q1, q2, q3, q4] = inverse_kinematics(x_des,y_des,z_des,phi_des);

q_ik = [q1, q2, q3, q4];

src2m.animate(q_ik)
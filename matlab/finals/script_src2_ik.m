clear all, close all, clc;

mdl_src2m;
src2m.base.t = [0;0;3]    
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

    
Q_smooth = Q(1,:);
for i=1:9
    Q_ = jtraj(Q(i,:),Q(i+1,:),10);
    Q_smooth = vertcat(Q_smooth,Q_(2:end,:));
end

filename = 'matlab/trajectoryAnimation.gif';
for i=1:size(Q_smooth,1)
    src2m.animate(Q_smooth(i,:));
    zlim([0 7])
    pause(.1);

    % Capture the plot as an image 
    frame = getframe(gcf); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if i == 1 
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    else 
        imwrite(imind,cm,filename,'gif','WriteMode','append'); 
    end
end


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
%Given end-effector position and orientation ([x,y,z] and phi) find joint
%angles ([q1, q2, q3, q4]).


pos_des = [1.375, 2.382, 2.465];
phi = deg2rad(90);

x = pos_des(1);
y = pos_des(2);
z = pos_des(3);
r = sqrt(x^2 + y^2);

q1 = atan2(y,x);

r_ = r - l4*cos(phi);
z_ = z - l4*sin(phi)-l1;

gamma = atan2(-z_/sqrt(r_^2 + z_^2),-r_/sqrt(r_^2 + z_^2));

sigma = -1;
q2 = gamma + sigma*acos(-(r_^2 + z_^2 + l2^2 - l3^2)/(2*l2*sqrt(r_^2 + z_^2)));

q3 = atan2((z_-l2*sin(q2))/l3,(r_-l2*cos(q2))/l3) - q2;

q4 = phi - (q2 + q3); 

q_ik = [q1, q2, q3, q4];

src2m.animate(q_ik)


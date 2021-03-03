clear all, close all, clc;

mdl_src2m2;
src2m.plot(qz);

%% Jacobian and Trajectory Control

% Give a list of waypoints
Q = [   0,      pi/3,   -pi/3,  0;
        pi/2,   pi/3,   -pi/3,  0;
        pi/2,   pi/3,   -pi/3,  pi;
        pi/2,   -pi/6,  -pi/6,  pi;
        pi/2,   -pi/6,  -pi/3,  pi/2;
        pi/2,   pi/3,   -pi/3,  0;
        -pi/4,  pi/3,   -pi/3,  0;
        -pi/4,  pi/3,   -pi/3,  pi;
        0,      pi/3,   -pi/3,  0];

    
max_pos_error = 0.01;
max_q4_error = 0.01;
dt = 0.1;
K_pos = 0.2;
K_q4 = 0.2;
q = Q(1,:);
p = src2m.fkine(q).transl;

% for i = 2:size(Q,1)

P = [];
e = [];

filename = 'animation/trajectoryAnimation.gif';

src2m.plot(q), hold on;
% Capture the plot as an image 
frame = getframe(gcf); 
im = frame2im(frame); 
[imind,cm] = rgb2ind(im,256); 
% Write to the GIF File 
imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 

for i=1:(size(Q,1)-1)
    pos_goal = src2m.fkine(Q(i+1,:)).transl;
    q_goal = Q(i+1,:);
    
    disp(['GOING TO WAYPOINT ', num2str(i+1)]); 
    disp(' ');   
    disp('Arm Control.');
    disp(['Current position: (', num2str(round(p,3)), ') m']);
    disp(['Position goal: (', num2str(round(pos_goal,3)), ') m']);
    while (norm(p-pos_goal) > max_pos_error)
        P = [P; p];
        e = (pos_goal-p);
        e = e/norm(e);
        v = K_pos*e;
        J = src2m.jacob0(q,'trans');
        qdot = pinv(J) * v';
        q = q + dt * qdot';
        p = src2m.fkine(q).transl;
    end
    src2m.plot(q), hold on;
    % Capture the plot as an image 
    frame = getframe(gcf); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    imwrite(imind,cm,filename,'gif','WriteMode','append'); 
    
    disp(['Reached: (', num2str(round(p,3)), ') m']);  
    disp('');
    disp('Bucket Control.');
    disp(['Current joint angles: (', num2str(round(q,3)), ') rad']);
    disp(['Joint angles goal: (', num2str(round(q_goal,3)), ') rad']);
    while (norm(q_goal(4)-q(4)) > max_q4_error)
        e = q_goal(4) - q(4);
        e = e/norm(e);
        q4_dot = K_q4*e;
        q(4) = q(4) + dt * q4_dot;
        p = src2m.fkine(q).transl;
    end
    src2m.plot(q), hold on;
    % Capture the plot as an image 
    frame = getframe(gcf); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    imwrite(imind,cm,filename,'gif','WriteMode','append'); 
    disp(['Reached: (', num2str(round(q,3)), ') rad']);
    disp('-------------------------------------------');
end

plot3(P(:,1),P(:,2),P(:,3))
frame = getframe(gcf); 
im = frame2im(frame); 
[imind,cm] = rgb2ind(im,256); 
% Write to the GIF File 
imwrite(imind,cm,filename,'gif','WriteMode','append'); 
% end


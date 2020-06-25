dtheta = 5/180*pi;
dR = 0.3;
theta = 0;
R = 1.5;
Z = 0;
dZ = 0.3;
P = [R*cos(theta), R*sin(theta),Z]';
N = 4;
for i = 1:4*N
    k = mod(i-1,4);
    if (k == 0)
        R = R + dR;
        Z = Z + dZ;
    elseif (k==2)
        R = R - dR;
        Z = Z + dZ;
    else
        theta = theta + dtheta;
        Z = Z - dZ;
    end
    newP = [R*cos(theta), R*sin(theta),Z]';
    P = [P, newP];
end
newP = P - [0.4,0.0,0]';
newP2 = P - [0.8,0.0,0]';
P = [P, newP, newP2];

plot3(P(1,:),P(2,:),P(3,:))
axis equal
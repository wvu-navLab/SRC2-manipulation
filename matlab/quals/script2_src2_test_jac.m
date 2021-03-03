close all; clear; clc;

% Define robot model
mdl_src2m2;

% Choose if symbolic or numeric
method = 'numeric';
switch method
    case 'numeric'
        q1 = 0;
        q2 = 0;
        q3 = 0;
        q4 = 0;
    case 'symbolic'
        syms q1 q2 q3 q4
end

% Robot Parameters
h0 = 0.3556;

l1 = 0.1128;
h1 = 0.0100;
d1 = sqrt(l1*l1 + h1*h1);

l2 = 1.5644;
h2 = 0.9644;
a2 = sqrt(l2*l2 + h2*h2);

l3 = 0.7394;
h3 = 0.5356;
a3 = sqrt(l3*l3 + h3*h3);

l4 = 0.2500;
h4 = 0.2855;
a4 = sqrt(l4*l4 + h4*h4);

th2 = atan2(h2,l2);
th3 = atan2(l3,h3);
th4 = atan2(h4,l4);

th2_star = -th2;
th3_star = th2-th3+pi/2;
th4_star = th3-th4-pi/2;

q1 = 1.53;
q2 = 0.275;
q3 = 0.264;
q4 = 0.089;

% Denavit-Hartenberg Table
a_DH        = [0, 0, -a2, -a3, -a4];
alpha_DH    = [0, pi/2, 0, 0, 0]; 
d_DH        = [h0, d1, 0, 0, 0];
theta_DH    = [0, q1, q2+th2_star, q3+th3_star, q4+th4_star];

% My functions
T = fwd_kine(a_DH, alpha_DH, d_DH, theta_DH,method);
J = get_jacobian(a_DH, alpha_DH, d_DH, theta_DH,method);

% RTB functions
q  = [q1, q2, q3, q4];
T0 = src2m.fkine(q);
J0 = src2m.jacob0(q);


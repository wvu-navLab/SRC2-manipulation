function J = get_jacobian(a_DH, alpha_DH, d_DH, theta_DH, method)

N = length(theta_DH);

if strcmp(method,'symbolic')
    Ai = sym('Ai',[4 4 N]);
    T0Ti = sym('T0Ti',[4 4 N]);
    o = sym('A',[3 1 N]);
    z = sym('A',[3 1 N]);
    J = sym('J',[6 N-1]);
else
    Ai = zeros([4 4 N]);
    T0Ti = zeros([4 4 N]);
    o = zeros([3 1 N]);
    z = zeros([3 1 N]);
    J = zeros([6 N-1]);
end

for i = 1:N
    Ai(:,:,i) = trotz(theta_DH(i)) * transl(0, 0, d_DH(i)) ...
        * transl(a_DH(i), 0, 0) * trotx(alpha_DH(i));
    if i == 1
        T0Ti(:,:,i) = Ai(:,:,i);
    else
        T0Ti(:,:,i) = T0Ti(:,:,i-1)*Ai(:,:,i);
    end
    o(:,:,i) = T0Ti(1:3,4,i);
    z(:,:,i) = T0Ti(1:3,3,i);
end

for i = 2:N
    J(1:3,i-1) = cross(z(:,:,i-1), o(:,:,N) - o(:,:,i-1));
    J(4:6,i-1) = z(:,:,i-1);
end

J;
end
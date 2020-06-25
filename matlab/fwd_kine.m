function T = fwd_kine(a_DH, alpha_DH, d_DH, theta_DH, method)

N = length(theta_DH);

if strcmp(method,'symbolic')
    Ai = sym('Ai',[4 4 N]);
    T0Ti = sym('T0Ti',[4 4 N]);
else
    Ai = zeros([4 4 N]);
    T0Ti = zeros([4 4 N]);
end

for i = 1:N
    Ai(:,:,i) = trotz(theta_DH(i)) * transl(0, 0, d_DH(i)) ...
        * transl(a_DH(i), 0, 0) * trotx(alpha_DH(i));
    if i == 1
        T0Ti(:,:,i) = Ai(:,:,i);
    else
        T0Ti(:,:,i) = T0Ti(:,:,i-1)*Ai(:,:,i);
    end
end

T = T0Ti(:,:,N);
end
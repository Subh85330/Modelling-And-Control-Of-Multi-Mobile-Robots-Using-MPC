function A0 = get_a0(A, K)
Ak = eye(4);
A0.pos = [];
A0.vel = [];
for  k = 1:K
   Ak = A*Ak;
   A0.pos = [A0.pos; Ak(1:2,:)];
   A0.vel = [A0.vel; Ak(3:4,:)];
end
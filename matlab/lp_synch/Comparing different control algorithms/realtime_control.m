function u = realtime_control(n0,l0)
% The control input obtained real-time is considered here
smallNetwork;

K_dem = 40;
K_cool = 80;
K = K_dem + K_cool;
n = sdpvar(2,K+1,'full');
f = sdpvar(2,K);
l = sdpvar(2,K+1,'full');
r = sdpvar(2,K);

cost = sum(sum(n)) + sum(sum(l)) - etha*(sum(sum(f)) - etha* sum(sum(r)));
cons = [];
% General Constraints
cons = [cons, n(:,1) == n0];
cons = [cons, f <= f1_bar];
% cons = [cons, 0<= r <=r1_bar];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cons = [cons, l(:,1)==l0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:K
    if i>K_dem
        d1 = 0; d2 = 0; d3 = 0;
    end;
%     Conservations
    cons = [cons, n(1,i+1) == n(1,i) - (beta1_bar^-1)*f(1,i) + d1 + r(1,i)];
    cons = [cons, n(2,i+1) == n(2,i) + f(1,i) + r(2,i) - (beta2_bar^-1)*f(2,i)];
    cons = [cons, l(1,i+1) == l(1,i) - r(1,i)+ d2];
    cons = [cons, l(2,i+1) == l(2,i) - r(2,i)+ d3];
    % MainLine FreeFlow
    cons = [cons, f(1,i) <= beta1_bar*v1*n(1,i) + beta1_bar*v1*Gamma*d1 + beta1_bar*v1*Gamma*r(1,i)];%?
    cons = [cons, f(2,i) <= beta2_bar*v2*n(2,i) +  beta2_bar*v2*Gamma*r(2,i)];
%     MainLine Congestion
    cons = [cons, f(1,i) <= w2*n2_jam-w2*n(2,i)-w2*Gamma*r(2,i)];
%     OR Flow
    cons = [cons, r(1,i) <= d2 + l(1,i)];
    cons = [cons, r(2,i) <= d3 + l(2,i)];
    % OR Bounds
    cons = [cons, 0 <= r(1,i) <= r1_bar];
    cons = [cons, 0 <= r(2,i) <= r2_bar];
%     Ramp Flow Constraints
    N_c = 1;
%     cons = [cons, r(1,i) <= N_c*(n1_jam-n(1,i))];
%     cons = [cons, r(2,i) <= N_c*(n2_jam-n(2,i))];
end

solvesdp(cons,cost);
r = double(r);
u = r(:,1);




function [n,l,f,r,cost] = CTM_no_control_sim(n0,l0, K_sim)
NetworkData;
n = zeros(2,K_sim + 1);
l = zeros(2,K_sim + 1);
f = zeros(2,K_sim);
r = zeros(2,K_sim);
s = zeros(2,K_sim);
n(:,1) = n0;
l(:,1) = l0;
for k = 1:K_sim
%     r(1,k) = min([l(1,k)+d2; N_c*(n1_jam-n(1,k))]);
    r(1,k) = min([l(1,k)+d2; N_c*(n1_jam-n(1,k)); r1_bar]);
%     r(2,k) = min([l(2,k)+d3; N_c*(n2_jam-n(2,k))]);
    r(2,k) = min([l(2,k)+d3; N_c*(n2_jam-n(2,k)); r2_bar]);
    f(1,k) = min([beta1_bar*v1*(n(1,k)+Gamma*r(1,k)); w2*(n2_jam-n(2,k)-Gamma*r(2,k));f1_bar]);
    f(2,k) = min([beta2_bar*v2*(n(2,k)+Gamma*r(2,k));f2_bar]);
    s(1,k) = (beta1/beta1_bar)*f(1,k);
    s(2,k) = (beta2/beta2_bar)*f(2,k);
    n(1,k+1) = n(1,k)+d1+r(1,k)-f(1,k)/beta1_bar;
    n(2,k+1) = n(2,k)+f(1,k)+r(2,k)-f(2,k)/beta2_bar;
    l(1,k+1) = l(1,k)+d2-r(1,k);
    l(2,k+1) = l(2,k)+d3-r(2,k);
end

cost = sum(sum(n)) + sum(sum(l)) - etha * sum(sum(r)) - etha*sum(sum(f));


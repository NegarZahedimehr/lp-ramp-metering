function [n_exp,l_exp,f_exp,r_exp,cost_exp] = CTM_explicit_control(n0,l0, K_sim)
NetworkData;
n_exp = zeros(2,K_sim+1);
l_exp = zeros(2,K_sim+1);
f_exp = zeros(2,K_sim);
r_exp = zeros(2,K_sim);
s = zeros(2,K_sim);
n_exp(:,1) = n0;
l_exp(:,1) = l0;
for k = 1:K_sim
    %     r(1,k) = min([l(1,k)+d2; N_c*(n1_jam-n(1,k))]);
    if n_exp(1,k) <= (n1_jam - f1_bar)
        r_exp(1,k) = f1_bar;
    else
        r_exp(1,k) = -n_exp(1,k) + n1_jam;
    end
    
    %     r_exp(1,k) = min([l_exp(1,k)+d2; N_c*(n1_jam-n_exp(1,k)); r1_bar]);
    %     r(2,k) = min([l(2,k)+d3; N_c*(n2_jam-n(2,k))]);
    if n_exp(2,k) <= n2_crt - f2_bar
        r_exp(2,k) = f2_bar;
    elseif n_exp(2,k) <= n2_crt
        r_exp(2,k) = -n_exp(2,k) + n2_jam;
    else
        r_exp(2,k) = 0;
    end
    %     r_exp(2,k) = min([l_exp(2,k)+d3; N_c*(n2_jam-n_exp(2,k)); r2_bar]);
    f_exp(1,k) = min([beta1_bar*v1*(n_exp(1,k)+Gamma*r_exp(1,k)); w2*(n2_jam-n_exp(2,k)-Gamma*r_exp(2,k));f1_bar]);
    f_exp(2,k) = min([beta2_bar*v2*(n_exp(2,k)+Gamma*r_exp(2,k));f2_bar]);
    s(1,k) = (beta1/beta1_bar)*f_exp(1,k);
    s(2,k) = (beta2/beta2_bar)*f_exp(2,k);
    n_exp(1,k+1) = n_exp(1,k)+d1+r_exp(1,k)-f_exp(1,k)/beta1_bar;
    n_exp(2,k+1) = n_exp(2,k)+f_exp(1,k)+r_exp(2,k)-f_exp(2,k)/beta2_bar;
    l_exp(1,k+1) = l_exp(1,k)+d2-r_exp(1,k);
    l_exp(2,k+1) = l_exp(2,k)+d3-r_exp(2,k);
end
cost_exp = sum(sum(n_exp)) + sum(sum(l_exp)) - etha * sum(sum(r_exp)) - etha*sum(sum(f_exp));


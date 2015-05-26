function [n_next,l_next,f_next,r_next] = CTM_one_step(n,l,r)
smallNetwork;

f_next(1) = min([beta1_bar*v1*(n(1)+Gamma*r(1)); w2*(n2_jam-n(2)-Gamma*r(2));f1_bar]);
f_next(2) = min([beta2_bar*v2*(n(2)+Gamma*r(2));f2_bar]);

% s_next(1) = (beta1/beta1_bar)*f(1);
% s_next(2) = (beta2/beta2_bar)*f(2);

n_next(1) = n(1)+d1+r(1)-f_next(1)/beta1_bar;
n_next(2) = n(2)+f_next(1)+r(2)-f_next(2)/beta2_bar;

l_next(1) = l(1)+d2-r(1);
l_next(2) = l(2)+d3-r(2);

r_next = r;



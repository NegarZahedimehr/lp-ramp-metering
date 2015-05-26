function [n_MPC,l_MPC,f_MPC,r_MPC, cost_MPC] = CTM_MPC_control(n0, l0, K_sim)

NetworkData;
n_MPC = zeros(2,K_sim+1);
l_MPC = zeros(2,K_sim+1);
f_MPC = zeros(2,K_sim);
r_MPC = zeros(2,K_sim);
s = zeros(2,K_sim);
n_MPC(:,1) = n0;
l_MPC(:,1) = l0;
for i = 1:K_sim
        n_opt = sdpvar(2,K+1);
        f_opt = sdpvar(2,K);
        l_opt = sdpvar(2,K+1);
        r_opt = sdpvar(2,K);
        
        cost = sum(sum(n_opt + l_opt)) - etha*(sum(sum(f_opt + r_opt)));
        cons = [];
        % General Constraints
        
        cons = [cons, n_opt(:,1) == n_MPC(:,i)];
        cons = [cons, f_opt <= f1_bar];
        cons = [cons, 0 <= r_opt <= r1_bar];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        cons = [cons, l_opt(:,1)== l_MPC(:,i)];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for k = 1:K
            if i> K_dem
                d1 = 0; d2 = 0; d3 = 0;
            end;
            % Conservations
            cons = [cons, n_opt(1,k+1) == n_opt(1,k) - (beta1_bar^-1)*f_opt(1,k) + d1 + r_opt(1,k)];
            cons = [cons, n_opt(2,k+1) == n_opt(2,k) + f_opt(1,k) + r_opt(2,k) - (beta2_bar^-1)*f_opt(2,k)];
            cons = [cons, l_opt(1,k+1) == l_opt(1,k) - r_opt(1,k)+ d2];
            cons = [cons, l_opt(2,k+1) == l_opt(2,k) - r_opt(2,k)+ d3];
            % MainLine FreeFlow
            cons = [cons, f_opt(1,k) <= beta1_bar*v1*n_opt(1,k) + beta1_bar*v1*Gamma*d1 + beta1_bar*v1*Gamma*r_opt(1,k)];%?
            cons = [cons, f_opt(2,k) <= beta2_bar*v2*n_opt(2,k) +  beta2_bar*v2*Gamma*r_opt(2,k)];
            % MainLine Congestion
            cons = [cons, f_opt(1,k) <= w2*n2_jam-w2*n_opt(2,k)-w2*Gamma*r_opt(2,k)];
            % OR Flow
            cons = [cons, r_opt(1,k) <= d2 + l_opt(1,k)];
            cons = [cons, r_opt(2,k) <= d3 + l_opt(2,k)];
            % Ramp Flow Constraints
            N_c = 1;
            cons = [cons, r_opt(1,k) <= N_c*(n1_jam-n_opt(1,k))];
            cons = [cons, r_opt(2,k) <= N_c*(n2_jam-n_opt(2,k))];
        end
        solvesdp(cons,cost);
        %%
        n_opt = double(n_opt);
        l_opt = double(l_opt);
        f_opt = double(f_opt);
        r_opt = double(r_opt);
        %%
        r_MPC(:,i) = r_opt(:,1);
        %     r_exp(2,k) = min([l_exp(2,k)+d3; N_c*(n2_jam-n_exp(2,k)); r2_bar]);
        f_MPC(1,i) = min([beta1_bar*v1*(n_MPC(1,i)+Gamma*r_MPC(1,i)); w2*(n2_jam-n_MPC(2,i)-Gamma*r_MPC(2,i));f1_bar]);
        f_MPC(2,i) = min([beta2_bar*v2*(n_MPC(2,i)+Gamma*r_MPC(2,i));f2_bar]);
        s(1,i) = (beta1/beta1_bar)*f_MPC(1,i);
        s(2,i) = (beta2/beta2_bar)*f_MPC(2,i);
        n_MPC(1,i+1) = n_MPC(1,i)+d1+r_MPC(1,i)-f_MPC(1,i)/beta1_bar;
        n_MPC(2,i+1) = n_MPC(2,i)+f_MPC(1,i)+r_MPC(2,i)-f_MPC(2,i)/beta2_bar;
        l_MPC(1,i+1) = l_MPC(1,i)+d2-r_MPC(1,i);
        l_MPC(2,i+1) = l_MPC(2,i)+d3-r_MPC(2,i);
        
        clear cost cons n_opt l_opt f_opt r_opt
end
cost_MPC = sum(sum(n_MPC + l_MPC)) - etha * sum(sum(f_MPC + r_MPC));

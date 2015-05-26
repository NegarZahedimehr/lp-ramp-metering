%% Explicit MPC of LP
% Network Data
clear all;
close all;
clc;
smallNetwork;
MP = 1;
n0 = [0;0];
l0 = [10;10];
%% Cost construction
n_opt = sdpvar(2,K+1);
f_opt = sdpvar(2,K);
l_opt = sdpvar(2,K+1);
r_opt = sdpvar(2,K);

cost = sum(sum(n_opt + l_opt)) - etha * (sum(sum(r_opt + f_opt))); 
cons = [];
% General Constraints
% eps = 0.1
% n0 = 0.5*n1_jam * ones(2,1);
if MP == 0
    cons = [cons, n_opt(:,1) == n0];
elseif MP == 1
    cons = [cons, 0 <= n_opt(:,1) <= n1_jam];
end;
cons = [cons, f_opt <= f1_bar];
cons = [cons, 0 <= r_opt <= r1_bar];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cons = [cons, l_opt(:,1) == l0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k = 1:K
    if k>K_dem
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
    cons = [cons, r_opt(1,k) <= v_ramp*(d2 + l_opt(1,k))];
    cons = [cons, r_opt(2,k) <= v_ramp*(d3 + l_opt(2,k))];
%     % Maximum Ramp Flowscl
%     cons = [cons, r(1,i) <= r1_bar];
%     cons = [cons, r(2,i) <= r2_bar];
    % Ramp Flow Constraints
    N_c = 1-w1;
    cons = [cons, r_opt(1,k) <= N_c*(n1_jam-n_opt(1,k))];
    cons = [cons, r_opt(2,k) <= N_c*(n2_jam-n_opt(2,k))];
end


if MP == 1
    MPLP = Opt(cons,cost,n_opt(:,1),r_opt(:,1));
    solution = MPLP.solve();
    % solution.xopt.merge('obj');
    % solution.xopt.merge('primal');
    % figure; hold on; for i=1:N solution.xopt.Set(i).plot; end;
    %% Controller
    N = solution.xopt.Num
    for k=1:N
        F = solution.xopt.Set(k).Functions('primal').F;
        g = solution.xopt.Set(k).Functions('primal').g;
        F11(k) = F(1,1);    F12(k) = F(1,2);   % F13(i) = F(1,3);
        F21(k) = F(2,1);    F22(k) = F(2,2);   % F23(i) = F(2,3);
        g11(k) = g(1,1);    g21(k) = g(2,1);
    end;
    figure;
    subplot(2,3,1); plot(F11); title('F_{11}'); grid; xlim([1 N]); h = findobj(gcf,'type','line'); set(h,'linewidth',2);
    subplot(2,3,2); plot(F12); title('F_{12}'); grid; xlim([1 N]); h = findobj(gcf,'type','line'); set(h,'linewidth',2);
    %subplot(2,3,3); plot(F13); title('F_{13}');grid; xlim([1 N]); h = findobj(gcf,'type','line'); set(h,'linewidth',2);
    subplot(2,3,4); plot(F21); title('F_{21}');grid; xlim([1 N]); h = findobj(gcf,'type','line'); set(h,'linewidth',2);
    subplot(2,3,5); plot(F22); title('F_{22}');grid; xlim([1 N]); h = findobj(gcf,'type','line'); set(h,'linewidth',2);
    %subplot(2,3,6); plot(F23); title('F_{23}');grid; xlim([1 N]); h = findobj(gcf,'type','line'); set(h,'linewidth',2);
    figure;
    subplot(2,1,1); plot(g11); title('g_{1}');grid; xlim([1 N]); h = findobj(gcf,'type','line'); set(h,'linewidth',2);
    subplot(2,1,2); plot(g21); title('g_{2}');grid; xlim([1 N]); h = findobj(gcf,'type','line'); set(h,'linewidth',2);
    
    for k=1:N
        R_equal{k} = find((abs(F11-F11(k))<1e-3)+(abs(F12-F12(k))<1e-3)+(abs(F21-F21(k))<1e-3)+(abs(F22-F22(k))<1e-3)+(abs(g11-g11(k))<1e-3)+(abs(g21-g21(k))<1e-3) == 6);
        %         R_equal{i} = find((F11==F11(i))+(F12==F12(i))+(F13==F13(i))+(F21==F21(i))+(F22==F22(i))+(F23==F23(i))+(g11==g11(i))+(g21==g21(i))==8);
    end;
    R_equal_indx = [1:N; ones(1,N)];
    
    j = 1; counter = 0;
    while j<=N
        counter = counter+1;
        comb_reg.set(counter).r = R_equal{j};
        comb_reg.set(counter).region = solution.xopt.Set(R_equal{j});
        comb_reg.set(counter).F = solution.xopt.Set(R_equal{j}(1)).Functions('primal').F;
        comb_reg.set(counter).g = solution.xopt.Set(R_equal{j}(1)).Functions('primal').g;
        for k=1:size(R_equal{j},2)
            R_equal_indx(2,R_equal{j}(k)) = 0;
        end;
        %         j = R_equal{j}(end)+1;
        j = min(find(R_equal_indx(2,:)>0));
                figure; comb_reg.set(counter).region.plot;
    end;
    comb_reg.N = counter
    %%%%%
    %%
    figure; 
    for k = 1:counter
        if mod(counter,3) == 0 N_plt = (counter-mod(counter,3))/3;
        else
            N_plt = (counter-mod(counter,3))/3+1;
        end;
        subplot(3,N_plt,k); comb_reg.set(k).region.plot;
    end;
    %%
for k = 1:counter    
    clear temp_Xmin temp_Xmax temp_Ymin temp_Ymax
    N_r = size(comb_reg.set(k).region,1);
    for j = 1:N_r
        V = comb_reg.set(k).region(j).V;
        temp_Xmin(j) = min(V(:,1)); temp_Xmax(j) = max(V(:,1));
        temp_Ymin(j) = min(V(:,2)); temp_Ymax(j) = max(V(:,2));
    end;
    Xmin(k) = min(temp_Xmin); Xmax(k) = max(temp_Xmax);
    Ymin(k) = min(temp_Ymin); Ymax(k) = max(temp_Ymax);   
end;
Regions = 1:counter;
region_sel = find(abs(Xmin-min(Xmin))<1e-3 & abs(Ymin-min(Ymin))<1e-3);
Regions(region_sel) = 0;
% Orders = [5; 3; 1; 7; 4; 2; 8; 6];
% Orders = Orders(end:-1:1)
% %% figures (sorted)
%     figure; 
%     for i = 1:counter
%         k = Orders(i);
%         subplot(3,mod(counter,3)+1,i); comb_reg.set(k).region.plot;
%     end;
    
    %%
    % finding V represntation of the whole region
    clear R F_cnt g_cnt R_cnt
    for k = 1:counter
        nV0 = 0;
        clear V
        for j = 1:size(comb_reg.set(k).region,1)
            nV1 = nV0+size(comb_reg.set(k).region(j).V,1);
            V(nV0+1:nV1,:) = comb_reg.set(k).region(j).V;
            nV0 = nV1;
        end;
        R(k) = Polyhedron('V',V);
        F_cnt(:,:,k) = comb_reg.set(k).F;
        g_cnt(:,:,k) = comb_reg.set(k).g;
        R_cnt(k) = F_cnt(:,:,k)*R(k)+g_cnt(:,:,k);
    end;
    %%%%%%%%
    ncr = f1_bar/v1;
    %%%%%%%%
    
    figure; hold on; R.plot; line([ncr;ncr],[0;ncr]); line([ncr,n1_jam],[ncr,ncr]); line([0,ncr],[n2_jam, ncr]); h = findobj(gcf,'type','line'); set(h,'linewidth',2); set(h,'color','k'); set(h,'LineStyle','-.');
    figure; R_cnt.plot;
        figure;
    for k = 1:size(R,2)
        R_size = size(R,2);
        if mod(R_size,3) == 0 N_plt = (R_size-mod(R_size,3))/3;
        else
            N_plt = (R_size-mod(R_size,3))/3+1
        end;
        subplot(3,N_plt,k); R(k).plot; xlim([0 n1_jam]); ylim([0 n1_jam]);
    end;
    
        figure; 
    for k = 1:size(R,2)
        R_cnt_size = size(R_cnt,2);
        if mod(R_cnt_size,3) == 0 N_plt = (R_cnt_size-mod(R_cnt_size,3))/3;
        else
            N_plt = (R_cnt_size-mod(R_cnt_size,3))/3+1
        end;
        subplot(3,N_plt,k); R_cnt(k).plot; xlim([-0.1 1.2*r1_bar]); ylim([-0.1 1.2*r1_bar]);
    end;
    
    %%
    
    %%%%%
    figure; hold on; for k=1:comb_reg.N comb_reg.set(k).region.plot; end;
    % Cost
        F_obj = solution.xopt.Set(k).Functions('obj').F;
        g_obj = solution.xopt.Set(k).Functions('obj').g;
        F11_obj(k) = F_obj(1,1);    F12_obj(k) = F_obj(1,2);    %F13_obj(i) = F_obj(1,3);
        g11_obj(k) = g_obj(1,1);
%     end;
    figure;
        subplot(1,2,1); plot(F11_obj); grid; xlim([1 N]); h = findobj(gcf,'type','line'); set(h,'linewidth',2);
        subplot(1,2,2); plot(F12_obj); grid; xlim([1 N]); h = findobj(gcf,'type','line'); set(h,'linewidth',2);
    %subplot(1,3,3); plot(F13_obj); grid; xlim([1 N]); h = findobj(gcf,'type','line'); set(h,'linewidth',2);
    figure;
        plot(g11_obj); grid; xlim([1 N]); h = findobj(gcf,'type','line'); set(h,'linewidth',2);
end;

if MP == 0
    DIAGNOSTIC = solvesdp(cons,cost);
%     DIAGNOSTIC = optimize(cons,cost);
    if DIAGNOSTIC.problem ~= 0
        error('The problem is infeasible')
    end
    n_opt = double(n_opt);
    l_opt = double(l_opt);
    f_opt = double(f_opt);
    r_opt = double(r_opt);
    
    [CTM_flow_1, CTM_flow_2, CTM_dens_1, CTM_dens_2] = CTM_check(n_opt,l_opt,f_opt,r_opt);
    if CTM_dens_1 == false || CTM_dens_2 == false || CTM_flow_1 == false || CTM_flow_2 == false
        error('The solution is not CTM-like')
    end

%%  
%     figure('name','f','position',[0 +100 500 400]);
%     subplot(2,1,1); hold on; plot(f_opt(1,:)); plot(K_dem,f_opt(1,K_dem),'r*'); grid;
%     subplot(2,1,2); hold on; plot(f_opt(2,:)); plot(K_dem,f_opt(2,K_dem),'r*'); grid;
%     %%
%     figure('name','l','position',[600 +100 500 400]);
%     subplot(2,1,1); hold on; plot(l_opt(1,:)); plot(K_dem+1,l_opt(1,K_dem+1),'r*'); grid;
%     subplot(2,1,2); hold on; plot(l_opt(2,:)); plot(K_dem+1,l_opt(2,K_dem+1),'r*'); grid;
%     %%
%     figure('name','r','position',[1200 +100 500 400]);
%     subplot(2,1,1); hold on; plot(r_opt(1,:)); plot(K_dem,r_opt(1,K_dem),'r*'); grid;
%     subplot(2,1,2); hold on; plot(r_opt(2,:)); plot(K_dem,r_opt(2,K_dem),'r*'); grid;
%     %% figures
%     figure('name','n','position',[600 600 500 400]);
%     subplot(2,1,1); hold on; plot(n_opt(1,:)); plot(K_dem+1,n_opt(1,K_dem+1),'r*'); grid;
%     subplot(2,1,2); hold on; plot(n_opt(2,:)); plot(K_dem+1,n_opt(2,K_dem+1),'r*'); grid;    
end;
%%
[n_no,l_no,f_no,r_no,cost_no] = CTM_no_control(n0,l0);
% figure('name','f No Control');
% subplot(2,1,1); hold on; plot(f_no(1,:)); plot(K_dem,f_no(1,K_dem),'r*'); grid;
% subplot(2,1,2); hold on; plot(f_no(2,:)); plot(K_dem,f_no(2,K_dem),'r*'); grid;
% 
% figure('name','n No Control');
% subplot(2,1,1); hold on; plot(n_no(1,:)); plot(K_dem+1,n_no(1,K_dem+1),'r*'); grid;
% subplot(2,1,2); hold on; plot(n_no(2,:)); plot(K_dem+1,n_no(2,K_dem+1),'r*'); grid;
% 
% figure('name','l No Control');
% subplot(2,1,1); hold on; plot(l_no(1,:)); plot(K_dem+1,l_no(1,K_dem+1),'r*'); grid;
% subplot(2,1,2); hold on; plot(l_no(2,:)); plot(K_dem+1,l_no(2,K_dem+1),'r*'); grid;
% 
% figure('name','r No Control');
% subplot(2,1,1); hold on; plot(r_no(1,:)); plot(K_dem,r_no(1,K_dem),'r*'); grid;
% subplot(2,1,2); hold on; plot(r_no(2,:)); plot(K_dem,r_no(2,K_dem),'r*'); grid;

figure('name','f No Control');
subplot(2,1,1); hold on; plot(f_no(1,:)); plot(K_dem,f_no(1,K_dem),'r*'); plot(f_opt(1,:),'g'); grid;
subplot(2,1,2); hold on; plot(f_no(2,:)); plot(K_dem,f_no(2,K_dem),'r*'); plot(f_opt(2,:),'g'); grid;

figure('name','n No Control');
subplot(2,1,1); hold on; plot(n_no(1,:)); plot(K_dem+1,n_no(1,K_dem+1),'r*'); plot(n_opt(1,:),'g'); grid;
subplot(2,1,2); hold on; plot(n_no(2,:)); plot(K_dem+1,n_no(2,K_dem+1),'r*'); plot(n_opt(2,:),'g'); grid;

figure('name','l No Control');
subplot(2,1,1); hold on; plot(l_no(1,:)); plot(K_dem+1,l_no(1,K_dem+1),'r*'); plot(l_opt(1,:),'g'); grid;
subplot(2,1,2); hold on; plot(l_no(2,:)); plot(K_dem+1,l_no(2,K_dem+1),'r*'); plot(l_opt(2,:),'g'); grid;

figure('name','r No Control');
subplot(2,1,1); hold on; plot(r_no(1,:)); plot(K_dem,r_no(1,K_dem),'r*'); plot(r_opt(1,:),'g'); grid;
subplot(2,1,2); hold on; plot(r_no(2,:)); plot(K_dem,r_no(2,K_dem),'r*'); plot(r_opt(2,:),'g'); grid;

cost_optimal = sum(sum(n_opt + l_opt)) - etha * (sum(sum(r_opt + f_opt))); 
display(cost_no);
display(cost_optimal);
% abs(cost_optimal-cost_no)<= 0.001
% display('Main-line Flows are equal?')
% all(all(abs(f_opt-f_no)<= 0.0001))
% display('On-ramp Flows are equal?')
% all(all(abs(r_opt-r_no)<= 0.0001))
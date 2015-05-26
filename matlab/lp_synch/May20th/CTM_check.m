function [CTM_flow_1, CTM_flow_2, CTM_dens_1, CTM_dens_2] = CTM_check(n_opt,l_opt,f_opt,r_opt)

smallNetwork;
CTM_flow_1 = true;
CTM_flow_2 = true;
CTM_dens_1 = true;
CTM_dens_2 = true;

for j = 1:K
    if j>K_dem
        d1 = 0; d2 = 0; d3 = 0;
    end;
    % CTM main-line flow
    CTM_ml1_flow = min( [beta1_bar*v1*n_opt(1,j)+beta1_bar*v1*Gamma*d1+beta1_bar*v1*Gamma*r_opt(1,j), ...
        w2*n2_jam-w2*n_opt( 2,j)-w2*Gamma*r_opt(2,j), f1_bar]);
    CTM_ml2_flow = min( [beta2_bar*v2*n_opt(2,j)+beta2_bar*v2*Gamma*r_opt(2,j), f2_bar]);
    % CTM on-ramp flow
    CTM_or1_flow = min([d2 + l_opt(1,j), r1_bar]);
    CTM_or2_flow = min([d3 + l_opt(2,j), r2_bar]);
    
    % LP solution vs. CTM
    is_CTM_1 = abs(CTM_ml1_flow-f_opt(1,j))<=0.001;
    is_CTM_2 = abs(CTM_ml2_flow-f_opt(2,j))<=0.001;
    
    if is_CTM_1 == false
        CTM_flow_1 = false;
        break;
    end
    
    if is_CTM_2 == false
        CTM_flow_2 = false;
        break;
    end
    
    if n_opt(1,j) >= n1_jam
        CTM_dens_1 = false;
    end
    
    if n_opt(2,j) >= n2_jam
        CTM_dens_2 = false;
    end
   
end

    
    
    

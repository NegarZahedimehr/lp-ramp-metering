function [r_ALINEA] = ALINEA(n,r)
% Computing Ramp Flows using ALINEa
smallNetwork;
K = (70/3600)*sim_dt;
r_ALINEA = K * [n1_crit - n(1); n2_crit - n(2)] + r;
if r_ALINEA <= [0;0]
    r_ALINEA = [0;0];
end


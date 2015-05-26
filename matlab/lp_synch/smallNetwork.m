%% Small Network
sim_dt = 3;
Gamma = 1;

N_c = 1;
MP = 0;

K = 50;
K_dem = 15;
Gamma = 1;
v_ramp = 1;

beta1 = 0.2;
% beta2 = 0.1;
beta2 = 0;

d1 = 0*sim_dt;%0*0.35* sim_dt;
d2 = 2.0002;%0.1*0.5 * sim_dt;
% d2 = 0.1 * sim_dt;%0.1*0.5 * sim_dt;
% d3 = 0.1 * sim_dt;%0.1*0.5 * sim_dt;
d3 = 2.0002;

beta1_bar = 1-beta1;
beta2_bar = 1-beta2;

v1 = 0.24135 * sim_dt;
v2 = 0.24135 * sim_dt;

w1 = 0.06034 * sim_dt;
w2 = 0.06034 * sim_dt;

n1_jam = 11.51003;
n2_jam = 11.51003;

l1_jam = inf;
l2_jam = inf;

f1_bar = 1.2*0.5556 * sim_dt;
f2_bar = 1.2*0.5556 * sim_dt;

% r1_bar = 1.2*0.5556 * sim_dt;
% r2_bar = 1.2*0.5556 * sim_dt;

r1_bar = 0.4*0.5556 * sim_dt;
r2_bar = 0.4*0.5556 * sim_dt;

etha = 0.1;

n1_crit = f1_bar/v1;
n2_crit = f2_bar/v2;
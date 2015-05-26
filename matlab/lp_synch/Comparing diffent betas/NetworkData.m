%% Network Data
sim_dt = 3;
% On-ramp Flow Blending Coefficient
Gamma = 1;

v_ramp = 1;

% Split Ratios
beta1 = 0.2;
% beta1 = 0.1;
beta2 = 0;

% Multi Parametric Solution
MP = 1;

% Demands
d1 = 0*sim_dt;%0*0.35* sim_dt;
%d2 = 2.0002;0.1*0.5 * sim_dt;
d2 = 0.3 * sim_dt;%0.1*0.5 * sim_dt;
d3 = 0.3 * sim_dt;%0.1*0.5 * sim_dt;
%d3 = 2.0002;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Period of time we consider non-zero demands
K_dem = 20;
% Cool down period
K_cool = 20;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
K = K_dem + K_cool;

% beta_bar = 1 - beta
beta1_bar = 1-beta1;
beta2_bar = 1-beta2;
% beta3_bar = 1-beta3;

% Free Flow Speeds
% v1 = 0.1 * sim_dt;
% v2 = 0.1 * sim_dt;

v1 = 0.024135 * sim_dt;
v2 = 0.024135 * sim_dt;

% Congestion Wave speed
% w1 = 0.03 * sim_dt;
% w2 = 0.03 * sim_dt;
w1 = 0.006034 * sim_dt;
w2 = 0.006034 * sim_dt;

% Jam Densities
% n1_jam = 15;
n1_jam = 11.51003;
% n1_jam = 138.114;
n2_jam = 11.51003;
% n2_jam = 15;
% n2_jam = 138.114;

% Jam densities for on-ramps
l1_jam = inf;
l2_jam = inf;

% The ratio of available speed in the main-line that can be allocated to
% on-ramps
N_c = 1-w1;

% Maximum Flow(Capacity)
f1_bar = 0.6667 * sim_dt;
f2_bar = 0.6667 * sim_dt;
% f1_bar = 1.2*0.5556 * sim_dt;
% f1_bar = 4;
% f2_bar = 4;
% f2_bar = 1.2*0.5556 * sim_dt;
% r1_bar = 1.2*0.5556 * sim_dt;
% r1_bar = 1;
 r1_bar = 2;
% r2_bar = 1.2*0.5556 * sim_dt;
 r2_bar = 2;
% r2_bar = 1;
% The weight of flows in the cost 
etha = 0.1;

n1_crt = f1_bar/v1;
n2_crt = f2_bar/v2;

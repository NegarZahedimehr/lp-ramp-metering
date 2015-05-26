%% Comparison of Explicit and on-line ramp metering:
% Network Data
clear all;
close all;
clc;
%% Variables
% Initial Condition
smallNetwork;
n0 = [5;5];
l0 = [5;5];
K_sim = 10;
% Explicit MPC variables
n_exp = zeros(2,K_sim+1);
l_exp = zeros(2,K_sim+1);
f_exp = zeros(2,K_sim);
r_exp = zeros(2,K_sim);
s_exp = zeros(2,K_sim);
n_exp(:,1) = n0;
l_exp(:,1) = l0;
% Real-time optimization variables
n_real = zeros(2,K_sim+1);
l_real = zeros(2,K_sim+1);
f_real = zeros(2,K_sim);
r_real = zeros(2,K_sim);
s_real = zeros(2,K_sim);
n_real(:,1) = n0;
l_real(:,1) = l0;
% ALINEA
n_ALINEA = zeros(2,K_sim+1);
l_ALINEA = zeros(2,K_sim+1);
f_ALINEA = zeros(2,K_sim);
r_ALINEA = zeros(2,K_sim);
s_ALINEA = zeros(2,K_sim);
n_ALINEA(:,1) = n0;
l_ALINEA(:,1) = l0;


%% Implementation

% No Control Case
[n_no_ctrl,l_no_ctrl,f_no_ctrl,r_no_ctrl,cost_no_ctrl] = CTM_no_control(n0,l0,K_sim);

% Real Time vs. Explicit
for k = 1:K_sim
    u_explicit = explicit_control(n_exp(:,k));
    u_real_time = realtime_control(n_real(:,k),l_real(:,k));
    if k == 1
        u_ALINEA = ALINEA(n_ALINEA(:,k),[0;0]);
    else
        u_ALINEA = ALINEA(n_ALINEA(:,k),r_ALINEA(:,k-1));
    end
    % Advancing the model one step
    [n_exp(:,k+1),l_exp(:,k+1),f_exp(:,k),r_exp(:,k)] = CTM_one_step(n_exp(:,k),l_exp(:,k),u_explicit);
    [n_real(:,k+1),l_real(:,k+1),f_real(:,k),r_real(:,k)] = CTM_one_step(n_real(:,k),l_real(:,k),u_real_time);
    [n_ALINEA(:,k+1),l_ALINEA(:,k+1),f_ALINEA(:,k),r_ALINEA(:,k)] = CTM_one_step(n_ALINEA(:,k),l_ALINEA(:,k),u_ALINEA);
end
% Cost Calculation
cost_explicit = sum(sum(n_exp + l_exp)) - etha * sum(sum(f_exp + r_exp));
cost_real = sum(sum(n_real + l_real)) - etha * sum(sum(f_real + r_real));
cost_ALINEA = sum(sum(n_ALINEA + l_ALINEA)) - etha * sum(sum(f_ALINEA + r_ALINEA));

%% Plots
% Density Plot
figure('name','n');
subplot(2,1,1)
plot(1:K_sim+1,n_real(1,:),'b');
hold on;
plot(1:K_sim+1,n_exp(1,:),'r');
hold on;
plot(1:K_sim+1,n_no_ctrl(1,:),'k');
hold on;
plot(1:K_sim+1,n_ALINEA(1,:),'y');
% legend('Real Time MPC','No Control');
legend('Real Time MPC','Explicit MPC','No Control','ALINEA');

subplot(2,1,2)
plot(1:K_sim+1,n_real(2,:),'b');
hold on;
plot(1:K_sim+1,n_exp(2,:),'r');
hold on;
plot(1:K_sim+1,n_no_ctrl(2,:),'k');
hold on;
plot(1:K_sim+1,n_ALINEA(2,:),'y');
% legend('Real Time MPC','No Control');
legend('Real Time MPC','Explicit MPC','No Control','ALINEA');


% On-ramp Queue Plot
figure('name','l');
subplot(2,1,1)
plot(1:K_sim+1,l_real(1,:),'b');
hold on;
plot(1:K_sim+1,l_exp(1,:),'r');
hold on;
plot(1:K_sim+1,l_no_ctrl(1,:),'k');
hold on;
plot(1:K_sim+1,l_ALINEA(1,:),'y');
legend('Real Time MPC','Explicit MPC','No Control','ALINEA');

subplot(2,1,2)
plot(1:K_sim+1,l_real(2,:),'b');
hold on;
plot(1:K_sim+1,l_exp(2,:),'r');
hold on;
plot(1:K_sim+1,l_no_ctrl(2,:),'k');
hold on;
plot(1:K_sim+1,l_ALINEA(2,:),'y');
legend('Real Time MPC','Explicit MPC','No Control','ALINEA');

% Mainline Flow Plot
figure('name','f');
subplot(2,1,1)
plot(1:K_sim,f_real(1,:),'b');
hold on;
plot(1:K_sim,f_exp(1,:),'r');
hold on;
plot(1:K_sim,f_no_ctrl(1,:),'k');
hold on;
plot(1:K_sim,f_ALINEA(1,:),'y');
legend('Real Time MPC','Explicit MPC','No Control','ALINEA');


subplot(2,1,2)
plot(1:K_sim,f_real(2,:),'b');
hold on;
plot(1:K_sim,f_exp(2,:),'r');
hold on;
plot(1:K_sim,f_no_ctrl(2,:),'k');
hold on;
plot(1:K_sim,f_ALINEA(2,:),'y');
legend('Real Time MPC','Explicit MPC','No Control','ALINEA');

% On-ramp Flow Plot
figure('name','r');
subplot(2,1,1)
plot(1:K_sim,r_real(1,:),'b');
hold on;
plot(1:K_sim,r_exp(1,:),'r');
hold on;
plot(1:K_sim,r_no_ctrl(1,:),'k');
hold on;
plot(1:K_sim,r_ALINEA(1,:),'y');
legend('Real Time MPC','Explicit MPC','No Control','ALINEA');

subplot(2,1,2)
plot(1:K_sim,r_real(2,:),'b');
hold on;
plot(1:K_sim,r_exp(2,:),'r');
hold on;
plot(1:K_sim,r_no_ctrl(2,:),'k');
hold on;
plot(1:K_sim,f_ALINEA(2,:),'y');
legend('Real Time MPC','Explicit MPC','No Control','ALINEA');

%% Cost Comparison
figure('name','cost');
plot(cost_no_ctrl,'*','MarkerSize',10);
hold on;
plot(cost_explicit,'o','MarkerSize',10);
hold on;
plot(cost_real,'.','MarkerSize',10);
hold on;
plot(cost_ALINEA,'*','MarkerSize',10);
legend('No Control Cost','Explicit MPC cost','Real Time Cost','ALINEA');




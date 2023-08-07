function [EMS_Results,EMS_xu_res] = ConvexEMS(Eco_xu_res,varargin)
% Convex optimization of lower-level energy management for an FCV.

p = inputParser;
addParameter(p,'constant_v_lim',0);
addParameter(p,'seed',1);
addParameter(p,'idx_list',1:3);
addParameter(p,'ds',10);
addParameter(p,'save_desired_res',0);
addParameter(p,'plt_save_fig',0);
parse(p,varargin{:});

idx_max = max(p.Results.idx_list);
% main results
EMS_Results = zeros(idx_max,5);
EMS_xu = cell(idx_max,1);
EMS_xu_res = cell(idx_max,1);

for ii = p.Results.idx_list

tic

Scenario = get_scenarios_by_index(ii);
s_max = Scenario.s_max;

ds = p.Results.ds; % delta_s
N = s_max/ds;
% input from upper level
F_v = Eco_xu_res{ii}.F_v;
v = Eco_xu_res{ii}.v;
F_mot_e = Eco_xu_res{ii}.F_mot_e;

%% Vehicle model
% FCS model
load ..\Functions\f_f_ax2+bx.mat
alpha_f = [f_f.fitresult.p1, f_f.fitresult.p2, f_f.fitresult.p3];
FCV.fc_pwr_map = [0, 2, 5, 7.5, 10, 20, 30, 40, 50]; % % kW (net) including parasitic losses
FCV.fc_eff_map = [10, 33, 49.2, 53.3, 55.9, 59.6, 59.1, 56.2, 50.8] / 100; % % efficiency indexed by fc_pwr

% battery model
FCV.Num_cell = 25;
FCV.ess_Q = 26 * 3600; % coulombs, battery package capacity
FCV.ess_r_dis_map = [40.7, 37.0, 33.8, 26.9, 19.3, 15.1, 13.1, 12.3, 11.7, 11.8, 12.2] / 1000 * FCV.Num_cell; % (ohm)
FCV.ess_r_chg_map = [31.6, 29.8, 29.5, 28.7, 28.0, 26.9, 23.1, 25.0, 26.1, 28.8, 47.2] / 1000 * FCV.Num_cell; % (ohm)
FCV.ess_voc_map = [11.70, 11.85, 11.96, 12.11, 12.26, 12.37, 12.48, 12.59, 12.67, 12.78, 12.89] * FCV.Num_cell; % (V)
% alpha_b = (FCV.ess_r_dis_map(7)+FCV.ess_r_chg_map(7))./2./FCV.ess_voc_map(7).^2.*1000;
alpha_b = FCV.ess_r_dis_map(7)./FCV.ess_voc_map(7).^2.*1000;
alpha_OC = FCV.ess_Q*FCV.ess_voc_map(7)./1000;

%% Preparation
% initial and final states
SoC_base = 0.6;
SoC_0 = 0.6-SoC_base;
SoC_f = SoC_0;
E_OC_0 = alpha_OC*SoC_0;
E_OC_f = alpha_OC*SoC_f;
x0 = E_OC_0;

% limits on states and controls
P_bat_min = -1./alpha_b/4;
P_bat_max = 1./alpha_b/4;
P_OC_min = -1./alpha_b/2;
P_OC_max = 1./alpha_b/2;
P_fcs_min = FCV.fc_pwr_map(1);
P_fcs_max = FCV.fc_pwr_map(end);
P_H2_min = P_fcs_min/FCV.fc_eff_map(1);
P_H2_max = P_fcs_max/FCV.fc_eff_map(end);
% P_H2_min = f_f.fitresult(P_fcs_min);
% P_H2_max = f_f.fitresult(P_fcs_max);
SoC_min = 0.3-SoC_base;
SoC_max = 0.9-SoC_base;
E_OC_min = alpha_OC*SoC_min;
E_OC_max = alpha_OC*SoC_max;

%% Problem pre-formulation
nx_EMS = 1; % x: E_OC
nu_EMS = 4; % u: F_OC,F_bat,F_fcs,F_H2

% - objective
% c = sparse([zeros(nx_EMS*(N+1),1);repmat(reshape([0,0,0,1],[],1),N,1)]);
c = sparse([zeros(nx_EMS*(N+1),1);repmat(reshape([0,0,0,1],[],1),N,1)]);
C = sparse(diag(c));

% - linear dynamics
Ad = [1];
Bd = [-ds,0,0,0];
Ax = kron(speye(N+1), -speye(nx_EMS)) + kron(sparse(diag(ones(N, 1), -1)), Ad);
Bu = kron([sparse(1, N); speye(N)], Bd);
Aeq = [Ax, Bu];
leq = [-x0; sparse(N*nx_EMS, 1)];

Ax2 = sparse(N,(N+1)*nx_EMS);
Bu2 = kron(speye(N),[0,1,1,0]);
Aeq2 = [Ax2, Bu2];
leq2 = [F_mot_e];

% - linear constraints on states and controls
E_OC_min_list = repmat(E_OC_min,N+1,1);
E_OC_max_list = repmat(E_OC_max,N+1,1);
x_min_list = [E_OC_min_list]';
x_min_list = x_min_list(:);
x_max_list = [E_OC_max_list]';
x_max_list = x_max_list(:);

u_min_B = speye(nu_EMS);
u_min = [P_OC_min*F_v,P_bat_min*F_v,P_fcs_min*F_v,P_H2_min*F_v];
u_max_B = speye(nu_EMS);
u_max = [P_OC_max*F_v,P_bat_max*F_v,P_fcs_max*F_v,P_H2_max*F_v];

x_min_A = speye(nx_EMS);
x_max_A = speye(nx_EMS);

u_min_list = repmat(reshape(u_min',[],1), 1, 1);
u_max_list = repmat(reshape(u_max',[],1), 1, 1);

xu_min_blk = blkdiag(kron(speye(N+1),x_min_A), kron(speye(N),u_min_B));
xu_max_blk = blkdiag(kron(speye(N+1),x_max_A), kron(speye(N),u_max_B));
xu_min_list = [x_min_list; u_min_list];
xu_max_list = [x_max_list; u_max_list];

% - SOCP constraints
x_SOC_A = {};
x_SOC_c = {};
u_SOC_A = {};
u_SOC_b = {};
u_SOC_c = {};
u_SOC_d = {};
x_SOC_A{end+1} = [0;0];
x_SOC_c{end+1} = [0];
u_SOC_A{end+1} = [[2*sqrt(alpha_b),0,0,0];[-1,1,0,0]];
u_SOC_b{end+1} = [zeros(N,1),F_v];
u_SOC_c{end+1} = [1,-1,0,0];
u_SOC_d{end+1} = [F_v];

x_SOC_A{end+1} = [0;0];
x_SOC_c{end+1} = [0];
u_SOC_A{end+1} = [[0,0,2*sqrt(alpha_f(1)),0];[0,0,alpha_f(2),-1]];
u_SOC_b{end+1} = [zeros(N,1),(1+alpha_f(3))*F_v];
u_SOC_c{end+1} = [0,0,-alpha_f(2),1];
u_SOC_d{end+1} = [(1-alpha_f(3))*F_v];

xu_SOC_A_blk = cell(length(x_SOC_A),2);
xu_SOC_b_list = cell(length(x_SOC_A),2);
xu_SOC_c_blk = cell(length(x_SOC_A),1);
xu_SOC_d_list = cell(length(x_SOC_A),1);

for i=1:length(x_SOC_A)
x_SOC_c_blk = kron(speye(N),x_SOC_c{i});
u_SOC_c_blk = kron(speye(N),u_SOC_c{i});
xu_SOC_c_blk{i,1} = [sparse(N,nx_EMS),x_SOC_c_blk,u_SOC_c_blk];
xu_SOC_d_list{i,1} = repmat(u_SOC_d{i},1,1);
for j=1:2
x_SOC_A_blk = kron(speye(N),x_SOC_A{i}(j,:));
u_SOC_A_blk = kron(speye(N),u_SOC_A{i}(j,:));
xu_SOC_A_blk{i,j} = [sparse(N,nx_EMS),x_SOC_A_blk,u_SOC_A_blk];
xu_SOC_b_list{i,j} = repmat(u_SOC_b{i}(:,j),1,1);
end
end

%% solve the SOCP problem
% cvx_solver —— Gurobi, Mosek, SDPT3, SeDuMi
try
cvx_solver Gurobi
catch
    cvx_solver
end
cvx_begin
    cvx_precision medium
    variable xu((N+1)*nx_EMS + N*nu_EMS) % xu = (x(0),x(1),...,x(N),u(0),...,u(N-1))
    minimize( c'*xu*ds )
%     minimize( abs(c'*xu*ds) )
%     minimize( (xu*ds)'*C*(xu*ds) )
    subject to
    % - linear equality constraints
    Aeq*xu == leq
    Aeq2*xu == leq2
    % - linear inequality constraints
    xu_min_blk*xu >= xu_min_list
    xu_max_blk*xu <= xu_max_list
    % - socp constraints
    for i=1:length(x_SOC_A)
        for j=1:length(xu_SOC_d_list{i})
    norm([xu_SOC_A_blk{i,1}(j,:)*xu + xu_SOC_b_list{i,1}(j,:);xu_SOC_A_blk{i,2}(j,:)*xu + xu_SOC_b_list{i,2}(j,:)])...
    <= xu_SOC_c_blk{i,1}(j,:)*xu + xu_SOC_d_list{i}(j,:);
        end
    end
    % - terminal state
    xu(1*(N+1)) == E_OC_f
cvx_end

%% save results
EMS_xu{ii,1} = xu;

E_OC = xu(1:nx_EMS:nx_EMS*(N+1));
F_OC = xu(nx_EMS*(N+1)+1:nu_EMS:end);
F_bat = xu(nx_EMS*(N+1)+2:nu_EMS:end);
F_fcs = xu(nx_EMS*(N+1)+3:nu_EMS:end);
F_H2 = xu(nx_EMS*(N+1)+4:nu_EMS:end);

SoC = E_OC/alpha_OC+SoC_base;
P_OC = F_OC.*v;
P_bat = F_bat.*v;
P_fcs = F_fcs.*v;
P_H2 = F_H2.*v;

e_f_f = P_H2-f_f.fitresult(P_fcs); % kW
e_f_b = P_bat-(-alpha_b.*P_OC.^2+P_OC); % kW

EMS_xu_res{ii,1}.E_OC = E_OC;
EMS_xu_res{ii,1}.F_OC = F_OC;
EMS_xu_res{ii,1}.F_bat = F_bat;
EMS_xu_res{ii,1}.F_fcs = F_fcs;
EMS_xu_res{ii,1}.F_H2 = F_H2;

EMS_xu_res{ii,1}.SoC = SoC;
EMS_xu_res{ii,1}.P_OC = P_OC;
EMS_xu_res{ii,1}.P_bat = P_bat;
EMS_xu_res{ii,1}.P_fcs = P_fcs;
EMS_xu_res{ii,1}.P_H2 = P_H2;

EMS_xu_res{ii,1}.e_f_f = e_f_f;
EMS_xu_res{ii,1}.e_f_b = e_f_b;

H2 = sum(F_H2)*ds/(120); % g
SoC_f_real = SoC(end);

EMS_Results(ii,[1,2,5]) = [toc,H2,SoC_f_real];

if p.Results.save_desired_res
save('EMS_Results','EMS_Results');
save('EMS_xu_res','EMS_xu_res');
end
%% plot
if p.Results.plt_save_fig

ss = 0:ds:Scenario.s_max;

figure(2)
subplot(311)
plot(ss,SoC); hold on; grid on
xlabel('s [m]'); ylabel('SoC [-]')

subplot(312)
plot(ss(2:end),P_fcs); hold on; grid on
xlabel('s [m]'); ylabel('P_{fcs} [kW]')

subplot(313)
plot(ss(2:end),F_mot_e.*v); hold on; grid on
xlabel('s [m]'); ylabel('P_{mot,e} [kW]')

saveas(gcf,'EMS_CVX.png')
% saveas(gcf,['EMS_Scenario' num2str(p.Results.seed) '_CVX.png'])
end

end

end
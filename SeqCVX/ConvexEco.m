function [Eco_Results,Eco_xu_res] = ConvexEco(t_p_ref,varargin)
% Convex optimization of upper-level speed planning for an FCV through signalized intersections.

p = inputParser;
addParameter(p,'destination_as_light',1);
addParameter(p,'constant_v_lim',0);
addParameter(p,'constant_slope',0);
addParameter(p,'seed',1);
addParameter(p,'v_0',2);
addParameter(p,'a_min',-2);
addParameter(p,'a_max',2);
addParameter(p,'t_f_unfixed',1);
addParameter(p,'dt_f',50);
addParameter(p,'idx_list',1:3);
addParameter(p,'ds',10);
addParameter(p,'save_desired_res',0);
addParameter(p,'plt_save_fig',0);
addParameter(p,'t_p_is_fixed',0);
parse(p,varargin{:});

idx_max = max(p.Results.idx_list);
% main results
Eco_Results = zeros(idx_max,5);
Eco_xu = cell(idx_max,1);
Eco_xu_res = cell(idx_max,1);

for ii = p.Results.idx_list

tic

%% Environment model
Scenario = get_scenarios_by_index(ii,destination_as_light=p.Results.destination_as_light,...
    constant_v_lim=p.Results.constant_v_lim,constant_slope=p.Results.constant_slope,...
    seed=p.Results.seed);
Scenario1 = get_scenarios_by_index(ii,destination_as_light=0,...
    constant_v_lim=p.Results.constant_v_lim,constant_slope=p.Results.constant_slope,...
    seed=p.Results.seed);
s_max = Scenario.s_max;
s_brk = Scenario.s_brk;
v_lim_brk = Scenario.v_lim_brk;

ds = p.Results.ds; % step
N = s_max/ds; % stage
a_min = p.Results.a_min;
a_max = p.Results.a_max;
v_0 = p.Results.v_0;
v_f = v_0;

N_tl = Scenario1.S_list / ds;

% speed limits
v_lim = Scenario.v_lim;
% road slope, rad
slope = Scenario.slope';
slope_list = slope(ds:ds:end);
%% Vehicle model
% motor model
load ..\Functions\f_m.mat
alpha_m = [f_m.fitresult.p1, f_m.fitresult.p2, f_m.fitresult.p3];

% vehicle parameters
par.veh_mass = 1380./1000;
par.veh_air_density = 1.2;
par.veh_CD = 0.335;
par.veh_FA = 2.0;
par.veh_gravity = 9.81;
par.veh_rrc = 0.009;
c_a = par.veh_air_density.*par.veh_FA.*par.veh_CD./(par.veh_mass.*1000);
F_f_list = par.veh_mass.*par.veh_gravity.*(par.veh_rrc.*cos(slope_list)+sin(slope_list));

%% Preparation
% initial and final states
t_0 = 0;
t_f_des = t_p_ref.t_p_e(end);
E_kin_0 = 0.5*par.veh_mass*v_0.^2;
E_kin_f = 0.5*par.veh_mass*v_f.^2;
x0 = [t_0;E_kin_0];

% limits on states and controls
v_min_list = v_0.*ones(N,1);
v_max_list = v_lim(ds:ds:end);
t_min_list = t_0*ones(N+1,1);
t_max_list = t_f_des*ones(N+1,1);
E_kin_min_list = 0.5*par.veh_mass*[v_min_list(1);v_min_list].^2;
E_kin_max_list = 0.5*par.veh_mass*[v_max_list(2);v_max_list].^2;
P_mot_m1_min_list = par.veh_mass.*(a_min+0.5*c_a*v_max_list.^2) .* v_max_list; % F_mot_m1 = F_mot_m-F_f
P_mot_m1_max_list = par.veh_mass.*(a_max+0.5*c_a*v_max_list.^2) .* v_max_list;
P_mot_m_min_list = P_mot_m1_min_list + F_f_list.*v_max_list;
P_mot_m_max_list = P_mot_m1_max_list + F_f_list.*v_max_list;
P_mot_e_min_list = f_m.fitresult(P_mot_m_min_list,v_min_list);
P_mot_e_max_list = f_m.fitresult(P_mot_m_max_list,v_max_list);
% F_mot_m1_min = par.veh_mass.*(a_min+0.5*c_a*v_min.^2);
% F_mot_m1_max = par.veh_mass.*(a_max+0.5*c_a*v_max.^2);

%% Problem pre-formulation
nx_Eco = 2; % x: t,E_kin
nu_Eco = 4; % u: F_v,F_mot_m1,v,F_mot_e

% - objective
c = sparse([zeros(nx_Eco*(N+1),1);repmat(reshape([0,0,0,1],[],1),N,1)]);
C = sparse(diag(c));

% - linear dynamics
Ad = [[1,0];[0,1-c_a*ds]];
Bd = [[ds,0,0,0];[0,ds,0,0]];

Ax = kron(speye(N+1), -speye(nx_Eco)) + kron(sparse(diag(ones(N, 1), -1)), Ad);
Bu = kron([sparse(1, N); speye(N)], Bd);
Aeq = [Ax, Bu];
leq = [-x0; sparse(N*nx_Eco, 1)];

% - linear constraints on states and controls
x_min_A = speye(nx_Eco);
x_max_A = speye(nx_Eco);
x_min_A_blk = kron(speye(N+1),x_min_A);
x_max_A_blk = kron(speye(N+1),x_max_A);
x_min_list = [t_min_list,E_kin_min_list]';
x_min_list = x_min_list(:);
x_max_list = [t_max_list,E_kin_max_list]';
x_max_list = x_max_list(:);

u_min_B = eye(nu_Eco);
u_min_B_blk_3D = u_min_B(:,:,ones(1,N));
u_min_B_blk_3D(2,1,:) = -P_mot_m1_min_list;
u_min_B_blk_3D(4,1,:) = -P_mot_e_min_list;
u_max_B_blk_3D = u_min_B(:,:,ones(1,N));
u_max_B_blk_3D(2,1,:) = -P_mot_m1_max_list;
u_max_B_blk_3D(4,1,:) = -P_mot_e_max_list;

clear u_min_B_blk u_max_B_blk
for n=1:N
u_min_B_blk(n) = {u_min_B_blk_3D(:,:,n)};
u_max_B_blk(n) = {u_max_B_blk_3D(:,:,n)};
end
u_min_B_blk = blkdiag(u_min_B_blk{:});
u_max_B_blk = blkdiag(u_max_B_blk{:});

u_min_blk = [1./v_max_list,sparse(N,1),v_min_list,sparse(N,1)]';
u_max_blk = [1./v_min_list,sparse(N,1),v_max_list,sparse(N,1)]';
u_min_list = u_min_blk(:);
u_max_list = u_max_blk(:);

xu_min_blk = blkdiag(x_min_A_blk, u_min_B_blk);
xu_max_blk = blkdiag(x_max_A_blk, u_max_B_blk);
xu_min_list = [x_min_list; u_min_list];
xu_max_list = [x_max_list; u_max_list];

% - SOCP constraints
x_SOC_A = {};
x_SOC_c = {};
u_SOC_A = {};
u_SOC_b = {};
u_SOC_c = {};
u_SOC_d = {};
x_SOC_A{end+1} = [[0,0];[0,0]];
x_SOC_c{end+1} = [0,0];
u_SOC_A{end+1} = [[0,0,0,0];[1,0,-1,0]];
u_SOC_b{end+1} = [2,0];
u_SOC_c{end+1} = [1,0,1,0];
u_SOC_d{end+1} = [0];

x_SOC_A{end+1} = [[0,0];[0,0]];
x_SOC_c{end+1} = [0,0];
u_SOC_A{end+1} = [[0,2*sqrt(alpha_m(1)),0,0];[1,alpha_m(2),0,-1]];
u_SOC_b{end+1} = [2*sqrt(alpha_m(1))*F_f_list,alpha_m(2)*F_f_list+alpha_m(3)];
u_SOC_c{end+1} = [1,-alpha_m(2),0,1];
u_SOC_d{end+1} = [-alpha_m(2)*F_f_list-alpha_m(3)];

x_SOC_A{end+1} = [[0,0];[0,1]];
x_SOC_c{end+1} = [0,1];
u_SOC_A{end+1} = [[0,0,sqrt(2*par.veh_mass),0];[0,0,0,0]];
u_SOC_b{end+1} = [0,-1];
u_SOC_c{end+1} = [0,0,0,0];
u_SOC_d{end+1} = [1];

xu_SOC_A_blk = cell(length(x_SOC_A),2);
xu_SOC_b_list = cell(length(x_SOC_A),2);
xu_SOC_c_blk = cell(length(x_SOC_A),1);
xu_SOC_d_list = cell(length(x_SOC_A),1);

for i=1:length(x_SOC_A)
x_SOC_c_blk = kron(speye(N),x_SOC_c{i});
u_SOC_c_blk = kron(speye(N),u_SOC_c{i});
xu_SOC_c_blk{i,1} = [sparse(N,nx_Eco),x_SOC_c_blk,u_SOC_c_blk];
if i~=2
xu_SOC_d_list{i,1} = repmat(u_SOC_d{i},N,1);
else
xu_SOC_d_list{i,1} = repmat(u_SOC_d{i},1,1);
end
for j=1:2
x_SOC_A_blk = kron(speye(N),x_SOC_A{i}(j,:));
u_SOC_A_blk = kron(speye(N),u_SOC_A{i}(j,:));
xu_SOC_A_blk{i,j} = [sparse(N,nx_Eco),x_SOC_A_blk,u_SOC_A_blk];
if i~=2
xu_SOC_b_list{i,j} = repmat(u_SOC_b{i}(:,j),N,1);
else
xu_SOC_b_list{i,j} = repmat(u_SOC_b{i}(:,j),1,1);
end
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
    variable xu((N+1)*nx_Eco + N*nu_Eco) % xu = (x(0),x(1),...,x(N),u(0),...,u(N-1))
    minimize( c'*xu*ds )
%     minimize( abs(c'*xu*ds) )
%     minimize( (xu*ds)'*C*(xu*ds) )
    subject to
    % - linear equality constraints
    Aeq*xu == leq
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
    if p.Results.t_f_unfixed
    xu(2*(N+1)) == E_kin_f
    else
    xu(2*N+1:2*N+2) == [t_f_des;E_kin_f]
    end

    for i = 1:length(N_tl)
        if p.Results.t_p_is_fixed
    xu(nx_Eco*N_tl(i)+1) >= t_p_ref.t_p(i) - 0.5
    xu(nx_Eco*N_tl(i)+1) <= t_p_ref.t_p(i) + 0.5
        else
    xu(nx_Eco*N_tl(i)+1) >= t_p_ref.t_p_min(i)
    xu(nx_Eco*N_tl(i)+1) <= t_p_ref.t_p_max(i)
        end
    end
cvx_end

%% save results
Eco_xu{ii,1} = xu;

t = xu(1:nx_Eco:nx_Eco*(N+1));
E_kin = xu(2:nx_Eco:nx_Eco*(N+1));
F_v = xu(nx_Eco*(N+1)+1:nu_Eco:end);
F_mot_m1 = xu(nx_Eco*(N+1)+2:nu_Eco:end);
v = xu(nx_Eco*(N+1)+3:nu_Eco:end);
F_mot_e = xu(nx_Eco*(N+1)+4:nu_Eco:end);

F_mot_m = F_mot_m1+F_f_list;
a = diff([v_0.^2;v.^2])./(2*ds);
a2 = diff([v_0;v])./(F_v*ds);
P_mot_m = F_mot_m.*v;
P_mot_e = F_mot_e.*v;

e_F_v = F_v.*v-1;
e_E_kin = (E_kin(2:end)-0.5*par.veh_mass*v.^2); % kJ
e_f_m = P_mot_e-f_m.fitresult(P_mot_m,v); % kW

Eco_xu_res{ii,1}.t = t;
Eco_xu_res{ii,1}.E_kin = E_kin;
Eco_xu_res{ii,1}.F_v = F_v;
Eco_xu_res{ii,1}.F_mot_m1 = F_mot_m1;
Eco_xu_res{ii,1}.v = v;
Eco_xu_res{ii,1}.F_mot_e = F_mot_e;

Eco_xu_res{ii,1}.F_mot_m = F_mot_m;
Eco_xu_res{ii,1}.a = a;
Eco_xu_res{ii,1}.a2 = a2;
Eco_xu_res{ii,1}.P_mot_m = P_mot_m;
Eco_xu_res{ii,1}.P_mot_e = P_mot_e;

Eco_xu_res{ii,1}.t_min_list = t_min_list;
Eco_xu_res{ii,1}.t_max_list = t_max_list;

Eco_xu_res{ii,1}.e_F_v = e_F_v;
Eco_xu_res{ii,1}.e_E_kin = e_E_kin;
Eco_xu_res{ii,1}.e_f_m = e_f_m;

t_f_real = t(end);
v_f_real = v(end);

Eco_Results(ii,[1,3,4]) = [toc,t_f_real,v_f_real];

if p.Results.save_desired_res
save('Eco_Results','Eco_Results');
save('Eco_xu_res','Eco_xu_res');
end
%% plot
if p.Results.plt_save_fig
    
ss = 0:ds:Scenario.s_max;

figure(1)
subplot(311)
% plot(ss,t_min_list); hold on; grid on
% plot(ss,t_max_list);
plot(ss,t); hold on; grid on
plot_scenario(Scenario1,t_max_list(end))
xlabel('s [m]'); ylabel('t [s]')

subplot(312)
plot(ss(2:end),v); hold on; grid on
plot(ss(2:end),v_max_list);
xlabel('s [m]'); ylabel('v [m/s]')

subplot(313)
% yyaxis left
plot(ss(2:end),a); hold on; grid on
xlabel('s [m]'); ylabel('a [m/s^2]')
% yyaxis right
% plot(ss(2:end),slope_list); hold on; grid on
% xlabel('s [m]'); ylabel('\theta [rad]')

saveas(gcf,'Eco_CVX.png')
% saveas(gcf,['Eco_Scenario' num2str(p.Results.seed) '_CVX.png'])
end

end

end
% DP-based Eco-driving of an FCV through signalized intersections.
close all; clear all; clc

N_list = [20,30];

EcoEMS_Results_DP = zeros(3,5);
EcoEMS_xu_DP = cell(3,1);

for Grid = 1:length(N_list)

N_all = N_list(Grid)*ones(1,5)+1;
ref_use_EcoEMS = 1;
seed = 100;
seed_idx = 7;
dt_f_idx = 1;

par.use_gpu = 1; % use GPU if 1

model = 'FCV_EcoEMS_DP_advisor_model_S';

tic

%% Environment model
Scenario = get_scenarios_by_index(5,seed=seed,destination_as_light=0,...
    constant_v_lim=0,constant_slope=0);
s_max = Scenario.s_max;
s_brk = Scenario.s_brk;
v_lim_brk = Scenario.v_lim_brk;
S_list = Scenario.S_list;
S_N = Scenario.N;

ds = 10; % step
par.ds = ds;
N = s_max/ds; % stage
S_list = S_list/ds;
a_min = -2;
a_max = 2;
v_0 = 2;
v_f = v_0;
par.a_last=0;

% time references
Scenario_for_IDM = get_scenarios_by_index(5,seed=seed,destination_as_light=1,...
    constant_v_lim=0,constant_slope=0);
% speed limits
v_lim = Scenario_for_IDM.v_lim;
% road slope, rad
slope = Scenario.slope';
slope_list = slope(ds:ds:end);

%% Preparation
% initial and final states
t_0 = 0;
SoC_0 = 0.6;

% limits on states and controls

CVX_res = load('..\DataForFig\CVX_res.mat');
CVX_res = CVX_res.CVX_res;

if ref_use_EcoEMS
t_ref = CVX_res{dt_f_idx,seed_idx}.t.EcoEMS;
v_ref = CVX_res{dt_f_idx,seed_idx}.v.EcoEMS;
soc_ref = CVX_res{dt_f_idx,seed_idx}.soc.EcoEMS;
else
t_ref = CVX_res{dt_f_idx,seed_idx}.t.Eco_EMS;
v_ref = CVX_res{dt_f_idx,seed_idx}.v.Eco_EMS;
soc_ref = CVX_res{dt_f_idx,seed_idx}.soc.Eco_EMS;
end

half_delta_t = 20;
half_delta_v = 10;
half_delta_soc = 0.002;

half_delta_t_f = half_delta_t/N_all(1);
half_delta_v_f = half_delta_v/N_all(2);
half_delta_soc_f = half_delta_soc/N_all(3);

t_f_min = t_ref(end)-half_delta_t_f;
t_f_max = t_ref(end)+half_delta_t_f;
v_f_min = v_ref(end)-half_delta_v_f;
v_f_max = v_ref(end)+half_delta_v_f;
SoC_f_min = soc_ref(end)-half_delta_soc_f;
SoC_f_max = soc_ref(end)+half_delta_soc_f;

t_min_list = max(t_ref-half_delta_t,0);
t_max_list = t_ref+half_delta_t;
v_min_list = v_0.*ones(N+1,1);
v_max_list = [v_lim(1);v_lim(ds:ds:end)];
% v_min_list = max(v_ref-5,v_min_list);
% v_max_list = min(v_ref+5,v_max_list);
% SoC_min = 0.595;
% SoC_max = 0.605;
SoC_min = soc_ref-half_delta_soc;
SoC_max = soc_ref+half_delta_soc;
P_fcs_min = 0;
P_fcs_max = 50*1000;

%% Problem
prb.Ts = ds; % step
prb.N  = N; 
prb.N0 = 1;

prb.W{1} = zeros(1,N);    
prb.W{2} = zeros(1,N);  
prb.W{3} = zeros(1,N);     
prb.W{4} = zeros(1,N);
prb.W{5} = slope_list'; 

% prb.W1: 0-speed limit, 1-traffic light, 2-stop sign, 3-ped crossing
% prb.W2: 0-limit value, 1-cycle (red to green)
% prb.W3:                1-red light duration
% prb.W4:                1-offset
% prb.W5:                1-slope

prb.W{1}(1,S_list) = ones(1,S_N);
prb.W{2}(1,S_list) = Scenario.T;
prb.W{3}(1,S_list) = Scenario.T_r;
prb.W{4}(1,S_list) = Scenario.T_0;

% State variables
grd.Nx{1} = N_all(1) + 1;   % time, s
grd.X0{1} = t_0;
grd.Xn{1}.hi = t_max_list;
grd.Xn{1}.lo = t_min_list;
grd.XN{1}.hi = t_f_max;
grd.XN{1}.lo = t_f_min;

grd.Nx{2} = N_all(2) + 1;   % velocity, m/s
grd.X0{2} = v_0;
grd.Xn{2}.hi = v_max_list;
grd.Xn{2}.lo = v_min_list;
grd.XN{2}.hi = v_f_max;
grd.XN{2}.lo = v_f_min;

grd.Nx{3}    = N_all(3) + 1;    % SoC
grd.X0{3}    = SoC_0;
grd.Xn{3}.hi = SoC_max;
grd.Xn{3}.lo = SoC_min;
grd.XN{3}.hi = SoC_f_max;
grd.XN{3}.lo = SoC_f_min;

% Control variables
grd.Nu{1} = N_all(4) + 1;   % acc, m/s^2
grd.Un{1}.hi = a_max;
grd.Un{1}.lo = a_min;

grd.Nu{2} = N_all(5) + 1;   % P_fcs, W
grd.Un{2}.hi = P_fcs_max;
grd.Un{2}.lo = P_fcs_min; 

options = dpm();
options.MyInf = 1e10;
options.BoundaryMethod = 'none'; % also possible: 'none' or 'LevelSet';
if strcmp(options.BoundaryMethod,'Line') 
    %these options are only needed if 'Line' is used
    options.Iter = 10;
    options.Tol = 1e-8;
    options.FixedGrid = 0;
end
if par.use_gpu
[res,dyn] = dpm_gpu(model,par,grd,prb,options);

keys = fieldnames(res);
for i = 1:length(keys)
    key = keys(i);
    key = key{1};
    res.(key) = gather(res.(key));
end

for i = 1:length(res.X)
    res.X{i} = gather(res.X{i});
end

else
[res,dyn] = dpm(model,par,grd,prb,options);
end

res.grd = grd;

t = res.X{1,1};
v = res.X{1,2};
SoC = res.X{1,3};
a = res.veh_acc;
P_fcs = res.FCS_pwr;

H2 = sum(res.H2_cost);
t_f_real = t(end);
v_f_real = v(end);
SoC_f_real = SoC(end);


EcoEMS_Results_DP(Grid,1:5) = [toc,H2,t_f_real,v_f_real,SoC_f_real];
EcoEMS_xu_DP{Grid,1} = res;

disp([sum(res.H2_cost),sum(res.acc_cost)]);
fprintf('SOC_0: %.4f , SOC_T: %.4f , H2 Consumption: %.4f g.\n', SoC_0, SoC_f_real, H2);

save('EcoEMS_Results_DP','EcoEMS_Results_DP');
save('EcoEMS_xu_DP','EcoEMS_xu_DP');

% save(['EcoEMS_Results_DP_S' num2str(seed)],'EcoEMS_Results_DP');
% save(['EcoEMS_xu_DP_S' num2str(seed)],'EcoEMS_xu_DP');
%% plot Eco
LineWidth = 1;

figure
subplot(221)
tt = 1:ceil(res.X{1}(end));
ss = interp1(res.X{1},1:length(res.X{1}),tt) * par.ds;
plot(tt,ss,'b-','linewidth',LineWidth); hold on
plot_scenario(Scenario_for_IDM,tt(end),mode_TS=0);
axis([0 tt(end)+3 0 N * ds]); grid on; hold off
ylabel('Distance (m)'); xlabel('Time (s)')

subplot(222)
plot(res.X{1}(2:end),res.FC_fuel / 1000,'b-','linewidth',LineWidth); grid on
axis([0 tt(end)+3 0 max(res.FC_fuel)+2])
axis 'auto y'
xlabel('Time (s)'); ylabel('e (g)')

subplot(223)
plot(res.X{1},res.X{2},'b-','linewidth',LineWidth)
axis([0 tt(end)+3 0 max(res.X{2})+2]); grid on
xlabel('Time (s)'); ylabel('Speed (m/s)')

subplot(224)
plot(res.X{1}(2:end),res.veh_acc,'b-','linewidth',LineWidth)
axis([0 tt(end)+3 min(res.veh_acc)-1 max(res.veh_acc)+1]); grid on
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)')

saveas(gcf,'Eco_DP.png')
%% plot EMS
m = 0000;
n = tt(end)+3;

figure
subplot(211)
yyaxis left
plot(res.X{1}(1:end),res.X{2},'b-','linewidth',LineWidth)
ylabel('Speed (m/s)')
yyaxis right
plot(res.X{1}(1:end),SoC,'r-','linewidth',LineWidth)
ylabel('SOC (-)')
axis([m n 0 10])
axis 'auto y'

subplot(212)
plot(res.X{1}(2:end),res.FCS_pwr/1000,'r-','linewidth',LineWidth)
hold on
plot(res.X{1}(2:end),res.Bat_pwr/1000,'b-','linewidth',LineWidth)
hold on
plot(res.X{1}(2:end),res.P_dem_m/1000, 'k-');
ylabel('power <kW>')
legend('FCS_{pwr}','Bat_{pwr}','P_{dem}','location','southwest');
axis([m n 0 10])
axis 'auto y'
grid on

saveas(gcf,'EMS_DP.png')

end
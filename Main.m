%% Main file for long-term eco-driving of FCHEVs.
close all;clear;clc

% green window routers
GWR_list = {'MILP','IDM'};
% arrival time decidedd by delta_t_f
d_t_f_list = 0:20:120;
% scenario indexed by seed
seed_list = [1,5,10,25,50,75,100];

count = 0;
h = waitbar(0,'please wait');

% Main results
CVX_res = cell(length(GWR_list),length(d_t_f_list),length(seed_list));
tc_res = cell(length(GWR_list),length(d_t_f_list),length(seed_list));
for n = 1%1:length(GWR_list)
% Green Window Router
run_get_t_p_ref = GWR_list{n};%'MILP'; % 'MILP', 'IDM'
% Speed Planner and SoC Planner
run_ConCVX = 1;
run_SeqCVX = 1;
run_PMP = 1;
% Control to plant
apply_2_real = 1;

for m = 1%1:length(d_t_f_list)
for k = 1%1:length(seed_list)
%% parameters
% scenario
scenarios_idx = 5;
constant_v_lim = 0;
constant_slope = 0;
seed = seed_list(k);
ds = 10;
dt_f = d_t_f_list(m);

Scenario = get_scenarios_by_index(scenarios_idx,constant_v_lim=constant_v_lim,...
    constant_slope=constant_slope,seed=seed);
Scenario1 = get_scenarios_by_index(scenarios_idx,destination_as_light=0,constant_v_lim=constant_v_lim,...
    constant_slope=constant_slope,seed=seed);
slope = Scenario.slope';
slope = slope(ds:ds:Scenario.s_max);
height = Scenario.height';
height = height(ds:ds:Scenario.s_max);

% GWR-MILP
num = 5; % node number of each green window

% CVX
t_f_unfixed = 0;
save_desired_res = 1;
plt_save_fig = 1;
t_p_is_fixed = 0;

% PMP
eqn_order = 2;

try
%% get t_p_ref
if strcmp(run_get_t_p_ref,'MILP')
    MILP_error = 0;
    cd GWR_MILP\
    try
        t_p_ref = get_t_p_ref_by_MILP(Scenario,dt_f,num);
        cd ..
    catch
        cd ..
        MILP_error = 1;
    end
    while MILP_error == 1
        num = num + 5;
        cd GWR_MILP\
    try
        t_p_ref = get_t_p_ref_by_MILP(Scenario,dt_f,num);
        cd ..
        MILP_error = 0;
    catch
        cd ..
        MILP_error = 1;
    end
    if num > 5*4
        break;
    end
    end
tc_res{n,m,k}.MILP = t_p_ref.tc;

elseif strcmp(run_get_t_p_ref,'IDM')
    cd GWR_IDM\
    t_p_ref = get_t_p_ref_by_IDM(Scenario,dt_f=dt_f);
    cd ..
    tc_res{n,m,k}.IDM = t_p_ref.tc;
end
%% ConCVX
% close all
if run_ConCVX
cd ConCVX\
try
[EcoEMS_Results,EcoEMS_xu_res] = ConvexEcoEMS(t_p_ref,constant_v_lim=constant_v_lim,constant_slope=constant_slope,...
    seed=seed,t_f_unfixed=t_f_unfixed,dt_f=dt_f,idx_list=scenarios_idx,save_desired_res=save_desired_res,...
    plt_save_fig=plt_save_fig,t_p_is_fixed=t_p_is_fixed);
cd ..
tc_res{n,m,k}.ConCVX = EcoEMS_Results(scenarios_idx,1);
catch
    cd ..
end
end
%% SeqCVX
if run_SeqCVX
cd SeqCVX\
try
[Eco_Results,Eco_xu_res] = ConvexEco(t_p_ref,constant_v_lim=constant_v_lim,constant_slope=constant_slope,seed=seed,...
    t_f_unfixed=t_f_unfixed,dt_f=dt_f,idx_list=scenarios_idx,save_desired_res=save_desired_res,...
    plt_save_fig=plt_save_fig,t_p_is_fixed=t_p_is_fixed);
[EMS_Results,EMS_xu_res] = ConvexEMS(Eco_xu_res,constant_v_lim=constant_v_lim,seed=seed,...
    idx_list=scenarios_idx,save_desired_res=save_desired_res,plt_save_fig=plt_save_fig);
[Eco_EMS_Results,Eco_EMS_xu_res] = MergeResults(Eco_Results,Eco_xu_res,EMS_Results,EMS_xu_res,...
    constant_v_lim=constant_v_lim,seed=seed,idx_list=scenarios_idx,save_desired_res=save_desired_res);
cd ..
tc_res{n,m,k}.SeqCVX_Eco = Eco_Results(scenarios_idx,1);
tc_res{n,m,k}.SeqCVX_EMS = EMS_Results(scenarios_idx,1);
tc_res{n,m,k}.SeqCVX = Eco_EMS_Results(scenarios_idx,1);
catch
    cd ..
end
end

%% PMP
if run_PMP
    cd PMP\
    try
        PMP_res_ux = AnalyticalSolution(Scenario,Scenario1,eqn_order,t_p_ref,ds);
        cd ..
        tc_res{n,m,k}.PMP = PMP_res_ux.tc;
    catch
        cd ..
    end
end

%% generated control to the plant, spatial domain
if apply_2_real

m_H_is_0_0 = 0;

[t,v,soc] = ini_state();

res = {};
out = FCV_EcoEMS_Plant_S(t,v,soc,0,0,ds,slope(1),m_H_is_0_0);
keys = fieldnames(out);
for idx_key = 1:length(keys)
   res = setfield(res,char(keys{idx_key,1}),'EcoEMS',[]);
   res = setfield(res,char(keys{idx_key,1}),'Eco_EMS',[]);
   res = setfield(res,char(keys{idx_key,1}),'PMP',[]);
end
res.v.EcoEMS = v;
res.v.Eco_EMS = v;
res.v.PMP = v;
res.t.EcoEMS = t;
res.t.Eco_EMS = t;
res.t.PMP = t;
res.soc.EcoEMS = soc;
res.soc.Eco_EMS = soc;
res.soc.PMP = soc;

if run_ConCVX
    try
    [t,v,soc] = ini_state();
    for idx_s = (ds:ds:Scenario.s_max)/ds
out = FCV_EcoEMS_Plant_S(t,v,soc,EcoEMS_xu_res{scenarios_idx,1}.a(idx_s),...
    EcoEMS_xu_res{scenarios_idx,1}.P_fcs(idx_s)*1000,ds,slope(idx_s),m_H_is_0_0);
t = out.t;
v = out.v;
soc = out.soc;
for idx_key = 1:length(keys)
res = setfield(res,char(keys{idx_key,1}),'EcoEMS',[getfield(res,char(keys{idx_key,1}),'EcoEMS'),getfield(out,char(keys{idx_key,1}))]);
end
    end
    catch
    end
end

if run_SeqCVX
    try
    [t,v,soc] = ini_state();
    for idx_s = (ds:ds:Scenario.s_max)/ds
out = FCV_EcoEMS_Plant_S(t,v,soc,Eco_EMS_xu_res{scenarios_idx,1}.a(idx_s),...
    Eco_EMS_xu_res{scenarios_idx,1}.P_fcs(idx_s)*1000,ds,slope(idx_s),m_H_is_0_0);   
t = out.t;
v = out.v;
soc = out.soc;
for idx_key = 1:length(keys)
res = setfield(res,char(keys{idx_key,1}),'Eco_EMS',[getfield(res,char(keys{idx_key,1}),'Eco_EMS'),getfield(out,char(keys{idx_key,1}))]);
end
    end
    catch
    end
end

if run_PMP
    try
    [t,v,soc] = ini_state();
    for idx_s = (ds:ds:Scenario.s_max)/ds
out = FCV_EcoEMS_Plant_S(t,v,soc,PMP_res_ux.a_S_from_v_S(idx_s),...
    0*1000,ds,slope(idx_s),m_H_is_0_0);
t = out.t;
v = out.v;
soc = out.soc;
for idx_key = 1:length(keys)
res = setfield(res,char(keys{idx_key,1}),'PMP',[getfield(res,char(keys{idx_key,1}),'PMP'),getfield(out,char(keys{idx_key,1}))]);
end
    end
    catch
    end
end

res.final_t = [res.t.EcoEMS(end),res.t.Eco_EMS(end),res.t.PMP(end)];
res.final_v = [res.v.EcoEMS(end),res.v.Eco_EMS(end),res.v.PMP(end)];
res.final_soc = [res.soc.EcoEMS(end),res.soc.Eco_EMS(end),res.soc.PMP(end)];
res.FC_fuel_sum = [sum(res.FC_fuel.EcoEMS),sum(res.FC_fuel.Eco_EMS),sum(res.FC_fuel.PMP)];
res.E_dem_m_sum = [sum(res.E_dem_m.EcoEMS),sum(res.E_dem_m.Eco_EMS),sum(res.E_dem_m.PMP)];
res.E_dem_e_sum = [sum(res.E_dem_e.EcoEMS),sum(res.E_dem_e.Eco_EMS),sum(res.E_dem_e.PMP)];
res.E_dem_m_pos_sum = [sum(res.E_dem_m_pos.EcoEMS),sum(res.E_dem_m_pos.Eco_EMS),sum(res.E_dem_m_pos.PMP)];
res.E_dem_e_pos_sum = [sum(res.E_dem_e_pos.EcoEMS),sum(res.E_dem_e_pos.Eco_EMS),sum(res.E_dem_e_pos.PMP)];
res.E_dem_m_pos_90neg_sum = [sum(res.E_dem_m_pos_90neg.EcoEMS),sum(res.E_dem_m_pos_90neg.Eco_EMS),sum(res.E_dem_m_pos_90neg.PMP)];
res.E_dem_e_pos_90neg_sum = [sum(res.E_dem_e_pos_90neg.EcoEMS),sum(res.E_dem_e_pos_90neg.Eco_EMS),sum(res.E_dem_e_pos_90neg.PMP)];
res.v_mean = [mean(res.v.EcoEMS),mean(res.v.Eco_EMS),mean(res.v.PMP)];
res.v_std = [std(res.v.EcoEMS),std(res.v.Eco_EMS),std(res.v.PMP)];
res.v_cv = res.v_std ./ max(abs(res.v_mean),1e-20);
res.a_mean = [mean(res.veh_acc.EcoEMS),mean(res.veh_acc.Eco_EMS),mean(res.veh_acc.PMP)];
res.a_std = [std(res.veh_acc.EcoEMS),std(res.veh_acc.Eco_EMS),std(res.veh_acc.PMP)];
res.a_cv = res.a_std ./ max(abs(res.a_mean),1e-20);
res.Mot_eta_mean = [mean(res.Mot_eta.EcoEMS),mean(res.Mot_eta.Eco_EMS),mean(res.Mot_eta.PMP)];
res.m_H_cum = [sum(res.H2_cost.EcoEMS),sum(res.H2_cost.Eco_EMS),sum(res.H2_cost.PMP)];
res.m_H_cum_add = (0.6-res.final_soc)*26*3600*12.48*25/0.5/(120*10^3);
res.m_H_cum_eqv = res.m_H_cum + res.m_H_cum_add;
end

CVX_res{n,m,k} = res;
% save('DataForFig\CVX_res.mat','CVX_res');

catch
    CVX_res{n,m,k} = res;
    disp(count);
end

count = count + 1;
waitbar(count/((length(GWR_list)*length(d_t_f_list)*length(seed_list))),h);
end
end
end

delete(h);

CVX_res = reshape(CVX_res,[],length(seed_list));
tc_res = reshape(tc_res,[],length(seed_list));

% save('DataForFig\CVX_res.mat','CVX_res'); % [(MILP+IDM)*dt_f, seed]
% save('DataForFig\tc_res.mat','tc_res'); % [(MILP+IDM)*dt_f, seed]


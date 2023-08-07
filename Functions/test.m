% close all;clear;clc

idx = 5;
seed = 100;
for seed=100
Scenario = get_scenarios_by_index(idx,seed=seed,constant_v_lim=0,constant_slope=0);
Scenario_plt = get_scenarios_by_index(idx,seed=seed,constant_v_lim=0,constant_slope=0,destination_as_light=0);
[s,v,a,sv_brk] = IDM(Scenario,v_0=2);
v_lim_IDM = interp1(sv_brk.s_brk,sv_brk.v_lim_brk,1:Scenario.s_max);
v_IDM = interp1(cumsum(v),v,1:Scenario.s_max,'linear','extrap');
t_IDM = interp1(cumsum(v),1:length(v),1:Scenario.s_max,'linear','extrap');
[s,v,a,sv_brk] = IDM(Scenario,v_0=2);
v_lim_IDM_2 = interp1(sv_brk.s_brk,sv_brk.v_lim_brk,1:Scenario.s_max);
% v_IDM_2 = interp1(cumsum(v),v,1:Scenario.s_max,'linear','extrap');
% t_IDM_2 = interp1(cumsum(v),1:length(v),1:Scenario.s_max,'linear','extrap');
s_ref = get_s_ref(Scenario,v_0=2,t_f_unfixed=0,dt_f=50);

figure
subplot(311)
plot(Scenario.slope);
ylabel('\theta [rad]')
subplot(312)
plot(Scenario.v_lim*3.6);
ylabel('v_{lim} [km/h]')
hold on
plot(v_IDM*3.6);
% plot(v_IDM_2*3.6);
subplot(313)
plot(s_ref.t_lower_ref_2);
% plot(t_IDM_2);
hold on
plot(s_ref.t_upper_ref);
plot_scenario(Scenario_plt,s_ref.t_upper_ref(end),mode_TS=1);
xlabel('s [m]')
ylabel('t [s]')
sgtitle(['Scenario' num2str(seed)])

% save('Scenario_plt','Scenario_plt')
% save(['Scenario_' num2str(seed)],'Scenario_plt')
end
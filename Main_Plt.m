%% Plot figures

%% GWR
% scenario
scenarios_idx = 5;
constant_v_lim = 0;
constant_slope = 0;
seed = 1;
ds = 10;

Scenario = get_scenarios_by_index(scenarios_idx,constant_v_lim=constant_v_lim,...
    constant_slope=constant_slope,seed=seed);
Scenario1 = get_scenarios_by_index(scenarios_idx,destination_as_light=0,constant_v_lim=constant_v_lim,...
    constant_slope=constant_slope,seed=seed);


% data
load 'DataForFig\CVX_res_08032209.mat'

% dt_f = 80
idx_1 = 9;
idx_2 = 1;
t_MILP_80 = CVX_res{idx_1,idx_2}.t.Eco_EMS; % 1x301
t_IDM_80 = CVX_res{idx_1+1,idx_2}.t.Eco_EMS;
E_MILP_80 = cumsum(CVX_res{idx_1,idx_2}.E_dem_e_pos_90neg.Eco_EMS)./1000; % 1x300
E_IDM_80 = cumsum(CVX_res{idx_1+1,idx_2}.E_dem_e_pos_90neg.Eco_EMS)./1000;

ss = 0:ds:Scenario.s_max; % 301x1
t_lim = max(t_MILP_80(end),t_IDM_80(end));
E_lim = max(E_MILP_80(end),E_IDM_80(end));

try
close figure 1
catch
end
figure(1)
num_subfig = 2;
e_left = 0.12;
e_right = 1- 0.16;
e_bottom = 0.1;
e_top = 1- 0.02;
e_space = 0.05;
e_height = ((e_top-e_bottom)-(num_subfig-1)*e_space)/num_subfig;
set(gcf,'position',[100 100 400 400])

subplot(num_subfig,1,1)
plot(ss,t_MILP_80); hold on; grid on; 
plot(ss,t_IDM_80);
plot_scenario(Scenario1,t_lim);
% xlabel('Distance \its \rm[m]');
ylabel('Time \itt \rm[s]');
xlim([0,Scenario.s_max]);
ylim([0,t_lim]);
set(gca,'position',[e_left e_top-e_height e_right e_height]);
set(gca,'FontName','Times New Roman');
legend('GWR-MILP','GWR-IDM',Location='best');

subplot(num_subfig,1,2)
plot(ss(2:end),E_MILP_80); hold on; grid on;
plot(ss(2:end),E_IDM_80);
xlabel('Distance \its \rm[m]'); 
ylabel('Cumulative Energy Cost \itE_{mot,eq}^{cum} \rm[kJ]');
xlim([0,Scenario.s_max]);
ylim([0,E_lim]);
set(gca,'position',[e_left e_bottom e_right e_height]);
set(gca,'FontName','Times New Roman');
legend('GWR-MILP','GWR-IDM',Location='best');

print(gcf,'Fig/GWR','-dpng','-r600');
print(gcf,'Fig/GWR','-depsc');
% saveas(gcf, 'Fig/GWR.pdf')
%% Eco
% scenario
scenarios_idx = 5;
constant_v_lim = 0;
constant_slope = 0;
seed = 25;
ds = 10;

Scenario = get_scenarios_by_index(scenarios_idx,constant_v_lim=constant_v_lim,...
    constant_slope=constant_slope,seed=seed);
Scenario1 = get_scenarios_by_index(scenarios_idx,destination_as_light=0,constant_v_lim=constant_v_lim,...
    constant_slope=constant_slope,seed=seed);

% data
load 'DataForFig\CVX_res_08032209.mat'

% dt_f=20
idx_1 = 3;
idx_2 = 4;
v_MILP_60_ConCVX = CVX_res{idx_1,idx_2}.v.EcoEMS; % 1x301
v_MILP_60_SeqCVX = CVX_res{idx_1,idx_2}.v.Eco_EMS; % 1x301
v_MILP_60_PMP = CVX_res{idx_1,idx_2}.v.PMP;
a_MILP_60_ConCVX = CVX_res{idx_1,idx_2}.veh_acc.EcoEMS; % 1x300
a_MILP_60_SeqCVX = CVX_res{idx_1,idx_2}.veh_acc.Eco_EMS; % 1x300
a_MILP_60_PMP = CVX_res{idx_1,idx_2}.veh_acc.PMP;

ss = 0:ds:Scenario.s_max; % 301x1
v_lim_list = Scenario.v_lim([1,ss(2:end)]);

try
close figure 2
catch
end
figure(2)
num_subfig = 2;
e_left = 0.12;
e_right = 1- 0.16;
e_bottom = 0.1;
e_top = 1- 0.02;
e_space = 0.05;
e_height = ((e_top-e_bottom)-(num_subfig-1)*e_space)/num_subfig;
set(gcf,'position',[100 100 400 400])

subplot(num_subfig,1,1)
plot(ss,v_MILP_60_ConCVX); hold on; grid on; 
plot(ss,v_MILP_60_SeqCVX);
plot(ss,v_MILP_60_PMP);
plot(ss,v_lim_list,Color='k',LineStyle='--');
% xlabel('Distance \its \rm[m]');
ylabel('Speed \itv \rm[m/s]');
xlim([0,Scenario.s_max]);
ylim([0,max(v_lim_list)+1]);
set(gca,'position',[e_left e_top-e_height e_right e_height]);
set(gca,'FontName','Times New Roman');
legend('ConCVX','SeqCVX','PMP','\itv_{lim}','NumColumns',2,Location='best');

subplot(num_subfig,1,2)
plot(ss(2:end),a_MILP_60_ConCVX); hold on; grid on;
plot(ss(2:end),a_MILP_60_SeqCVX);
plot(ss(2:end),a_MILP_60_PMP);
xlabel('Distance \its \rm[m]'); 
ylabel('Acceleration \ita \rm[m/s^2]');
xlim([0,Scenario.s_max]);
ylim([-2,2]);
set(gca,'position',[e_left e_bottom e_right e_height]);
set(gca,'FontName','Times New Roman');
legend('ConCVX','SeqCVX','PMP','NumColumns',2,Location='best');

print(gcf,'Fig/Eco','-dpng','-r600');
print(gcf,'Fig/Eco','-depsc');

%% EMS
% scenario
scenarios_idx = 5;
constant_v_lim = 0;
constant_slope = 0;
seed = 50;
ds = 10;

Scenario = get_scenarios_by_index(scenarios_idx,constant_v_lim=constant_v_lim,...
    constant_slope=constant_slope,seed=seed);
Scenario1 = get_scenarios_by_index(scenarios_idx,destination_as_light=0,constant_v_lim=constant_v_lim,...
    constant_slope=constant_slope,seed=seed);

% data
load 'DataForFig\CVX_res_08032209.mat'

% dt_f=20
idx_1 = 3;
idx_2 = 5;
soc_MILP_60_ConCVX = CVX_res{idx_1,idx_2}.soc.EcoEMS; % 1x301
soc_MILP_60_SeqCVX = CVX_res{idx_1,idx_2}.soc.Eco_EMS; % 1x301
P_fcs_MILP_60_ConCVX = CVX_res{idx_1,idx_2}.FCS_pwr.EcoEMS./1000; % 1x300
P_fcs_MILP_60_SeqCVX = CVX_res{idx_1,idx_2}.FCS_pwr.Eco_EMS./1000; % 1x300
m_MILP_60_ConCVX = cumsum(CVX_res{idx_1,idx_2}.H2_cost.EcoEMS) + (0.6-soc_MILP_60_ConCVX(2:end))*26*3600*12.48*25/0.5/(120*10^3); % 1x300
m_MILP_60_SeqCVX = cumsum(CVX_res{idx_1,idx_2}.H2_cost.Eco_EMS)  + (0.6-soc_MILP_60_SeqCVX(2:end))*26*3600*12.48*25/0.5/(120*10^3); % 1x300

ss = 0:ds:Scenario.s_max; % 301x1

try
close figure 5
catch
end
figure(5)
num_subfig = 3;
e_left = 0.12;
e_right = 1- 0.16;
e_bottom = 0.1;
e_top = 1- 0.02;
e_space = 0.05;
e_height = ((e_top-e_bottom)-(num_subfig-1)*e_space)/num_subfig;
set(gcf,'position',[100 100 400 400])

subplot(num_subfig,1,1)
plot(ss,soc_MILP_60_ConCVX*100); hold on; grid on; 
plot(ss,soc_MILP_60_SeqCVX*100);
% xlabel('Distance \its \rm[m]');
ylabel('Battery \itSoC \rm[%]');
xlim([0,Scenario.s_max]);
% ylim([0,max(v_lim_list)+1]);
set(gca,'position',[e_left e_top-e_height e_right e_height]);
set(gca,'FontName','Times New Roman');
legend('ConCVX','SeqCVX','NumColumns',2,Location='best');

subplot(num_subfig,1,2)
plot(ss(2:end),P_fcs_MILP_60_ConCVX); hold on; grid on;
plot(ss(2:end),P_fcs_MILP_60_SeqCVX);
% xlabel('Distance \its \rm[m]'); 
ylabel('FCS Net Power \itP_{fcs} \rm[kW]');
% xlim([0,Scenario.s_max]);
% ylim([-2,2]);
set(gca,'position',[e_left e_bottom+e_height+e_space e_right e_height]);
set(gca,'FontName','Times New Roman');
legend('ConCVX','SeqCVX','NumColumns',2,Location='best');

subplot(num_subfig,1,3)
plot(ss(2:end),m_MILP_60_ConCVX); hold on; grid on;
plot(ss(2:end),m_MILP_60_SeqCVX);
xlabel('Distance \its \rm[m]'); 
ylabel('Energy cost \itm_{H,eq}^{cum} \rm[g]');
% xlim([0,Scenario.s_max]);
% ylim([-2,2]);
set(gca,'position',[e_left e_bottom e_right e_height]);
set(gca,'FontName','Times New Roman');
legend('ConCVX','SeqCVX','NumColumns',2,Location='best');

print(gcf,'Fig/EMS','-dpng','-r600');
print(gcf,'Fig/EMS','-depsc');

%% EcoEMS
% scenario
scenarios_idx = 5;
constant_v_lim = 0;
constant_slope = 0;
seed = 100;
ds = 10;

Scenario = get_scenarios_by_index(scenarios_idx,constant_v_lim=constant_v_lim,...
    constant_slope=constant_slope,seed=seed);
Scenario1 = get_scenarios_by_index(scenarios_idx,destination_as_light=0,constant_v_lim=constant_v_lim,...
    constant_slope=constant_slope,seed=seed);

% data
load 'DataForFig\CVX_res_08032209.mat'
load 'ConDP\Res_40\EcoEMS_xu_DP_S100.mat'

% dt_f=20
idx_1 = 1;
idx_2 = 7;
t_MILP_60_ConCVX = CVX_res{idx_1,idx_2}.t.EcoEMS; % 1x301
v_MILP_60_ConCVX = CVX_res{idx_1,idx_2}.v.EcoEMS; % 1x301
a_MILP_60_ConCVX = CVX_res{idx_1,idx_2}.veh_acc.EcoEMS; % 1x300
soc_MILP_60_ConCVX = CVX_res{idx_1,idx_2}.soc.EcoEMS*100; % 1x301
P_fcs_MILP_60_ConCVX = CVX_res{idx_1,idx_2}.FCS_pwr.EcoEMS./1000; % 1x300
t_DP = EcoEMS_xu_DP{1,1}.X{1};
v_DP = EcoEMS_xu_DP{1,1}.X{2};
a_DP = EcoEMS_xu_DP{1,1}.veh_acc;
soc_DP = EcoEMS_xu_DP{1,1}.X{3}*100;
P_fcs_DP = EcoEMS_xu_DP{1,1}.FCS_pwr./1000;

ss = 0:ds:Scenario.s_max; % 301x1
v_lim_list = Scenario.v_lim([1,ss(2:end)]);
t_lim = max([t_MILP_60_ConCVX(end),t_DP(end)]);

try
close figure 6
catch
end
figure(6)
num_subfig = 5;
e_left = 0.12;
e_right = 1- 0.16;
e_bottom = 0.05;
e_top = 1- 0.02;
e_space = 0.05;
e_height = ((e_top-e_bottom)-(num_subfig-1)*e_space)/num_subfig;
set(gcf,'position',[100 100 400 600])

subplot(num_subfig,1,1)
plot(ss,t_MILP_60_ConCVX); hold on; grid on; 
plot(ss,t_DP);
plot_scenario(Scenario1,t_lim);
% xlabel('Distance \its \rm[m]');
ylabel('Time \itt \rm[s]');
xlim([0,Scenario.s_max]);
ylim([0,t_lim]);
set(gca,'position',[e_left e_top-e_height e_right e_height]);
set(gca,'FontName','Times New Roman');
legend('GWR-MILP+ConCVX','ConDP@Grid40','NumColumns',2,Location='best');

subplot(num_subfig,1,2)
plot(ss,v_MILP_60_ConCVX); hold on; grid on; 
plot(ss,v_DP);
plot(ss,v_lim_list,Color='k',LineStyle='--');
% xlabel('Distance \its \rm[m]');
ylabel('Speed \itv \rm[m/s]');
xlim([0,Scenario.s_max]);
ylim([0,max(v_lim_list)+1]);
set(gca,'position',[e_left e_top-e_height*2-e_space e_right e_height]);
set(gca,'FontName','Times New Roman');
legend('GWR-MILP+ConCVX','ConDP@Grid40','NumColumns',2,Location='best');

subplot(num_subfig,1,3)
plot(ss(2:end),a_MILP_60_ConCVX); hold on; grid on; 
plot(ss(2:end),a_DP);
% xlabel('Distance \its \rm[m]');
ylabel('Acceleration \ita \rm[m/s^2]');
xlim([0,Scenario.s_max]);
ylim([-2,2]);
set(gca,'position',[e_left e_top-e_height*3-e_space*2 e_right e_height]);
set(gca,'FontName','Times New Roman');
legend('GWR-MILP+ConCVX','ConDP@Grid40','NumColumns',2,Location='best');

subplot(num_subfig,1,4)
plot(ss,soc_MILP_60_ConCVX); hold on; grid on; 
plot(ss,soc_DP);
% xlabel('Distance \its \rm[m]');
ylabel('Battery \itSoC \rm[%]');
xlim([0,Scenario.s_max]);
% ylim([0,max(v_lim_list)+1]);
set(gca,'position',[e_left e_top-e_height*4-e_space*3 e_right e_height]);
set(gca,'FontName','Times New Roman');
legend('GWR-MILP+ConCVX','ConDP@Grid40','NumColumns',2,Location='best');

subplot(num_subfig,1,5)
plot(ss(2:end),P_fcs_MILP_60_ConCVX); hold on; grid on;
plot(ss(2:end),P_fcs_DP);
xlabel('Distance \its \rm[m]'); 
ylabel('FCS Net Power \itP_{fcs} \rm[kW]');
xlim([0,Scenario.s_max]);
% ylim([-2,2]);
set(gca,'position',[e_left e_bottom e_right e_height]);
set(gca,'FontName','Times New Roman');
legend('GWR-MILP+ConCVX','ConDP@Grid40','NumColumns',2,Location='best');

print(gcf,'Fig/EcoEMS','-dpng','-r600');
print(gcf,'Fig/EcoEMS','-depsc');

%%
try
close figure 20
catch
end
figure(20)
plot(ss,t_MILP_60_ConCVX); hold on; grid on; 
plot(ss,t_DP);
plot_scenario(Scenario1,t_lim);
% xlabel('Distance \its \rm[m]');
ylabel('Time \itt \rm[s]');
xlim([0,Scenario.s_max]);
ylim([0,t_lim]);
% set(gca,'position',[e_left e_bottom e_right e_height]);
set(gca,'FontName','Times New Roman');
legend('GWR-MILP+ConCVX','ConDP','NumColumns',2,Location='best');

[max(abs(t_MILP_60_ConCVX-t_DP)),max(abs(v_MILP_60_ConCVX-v_DP)),max(abs(soc_MILP_60_ConCVX-soc_DP))]
%% CVX
% data
load 'DataForFig\CVX_res_08032209.mat'
load 'DataForFig\tc_res_08032209.mat'

E_MILP_Eco = [];
E_IDM_Eco = [];
m_MILP_Eco = [];
m_IDM_Eco = [];
m_f_all = [];
MILP_v = [];
IDM_v = [];
t_f_all = [];
v_f_all = [];
soc_f_all = [];
tc_all = [];

for m = 1:7
for k = 1:7
E_MILP_Eco = [E_MILP_Eco; CVX_res{2*m-1,k}.E_dem_e_pos_90neg_sum];
E_IDM_Eco = [E_IDM_Eco; CVX_res{2*m,k}.E_dem_e_pos_90neg_sum];
m_MILP_Eco = [m_MILP_Eco; CVX_res{2*m-1,k}.m_H_cum_eqv];
m_IDM_Eco = [m_IDM_Eco; CVX_res{2*m,k}.m_H_cum_eqv];
m_f_all = [m_f_all; CVX_res{2*m-1,k}.m_H_cum; CVX_res{2*m,k}.m_H_cum];
MILP_v = [MILP_v; CVX_res{2*m-1,k}.v_std];
IDM_v = [IDM_v; CVX_res{2*m,k}.v_std];
t_f_all = [t_f_all; CVX_res{2*m-1,k}.final_t; CVX_res{2*m,k}.final_t];
v_f_all = [v_f_all; CVX_res{2*m-1,k}.final_v; CVX_res{2*m,k}.final_v];
soc_f_all = [soc_f_all; CVX_res{2*m-1,k}.final_soc; CVX_res{2*m,k}.final_soc];
try
tc_all = [tc_all; [tc_res{2*m-1,k}.MILP.tc_CPU,tc_res{2*m,k}.IDM.tc_CPU,...
    nanmean([tc_res{2*m-1,k}.ConCVX,tc_res{2*m,k}.ConCVX]),...
    nanmean([tc_res{2*m-1,k}.SeqCVX,tc_res{2*m,k}.SeqCVX]),...
    nanmean([tc_res{2*m-1,k}.PMP.tc_CPU,tc_res{2*m,k}.PMP.tc_CPU]),...
    nanmean([tc_res{2*m-1,k}.SeqCVX_Eco,tc_res{2*m,k}.SeqCVX_Eco]),...
    nanmean([tc_res{2*m-1,k}.SeqCVX_EMS,tc_res{2*m,k}.SeqCVX_EMS])]];
catch
end

end
end

% E_mot
% MILP vs IDM
E_MILP_vs_IDM = E_MILP_Eco ./ E_IDM_Eco;
E_MILP_vs_IDM_at_dtf = get_vs_at_dtf(E_MILP_vs_IDM);
m_MILP_vs_IDM = m_MILP_Eco ./ m_IDM_Eco;
m_MILP_vs_IDM_at_dtf = get_vs_at_dtf(m_MILP_vs_IDM);
E_MILP_vs_IDM_mean = nanmean(1-E_MILP_vs_IDM)*100; % %

E_all_mean = nanmean([E_MILP_Eco;E_IDM_Eco])./1000;
m_all_mean = nanmean([m_MILP_Eco;m_IDM_Eco]);
m_f_all_mean = nanmean(m_f_all);

% E_vs_t
E_MILP_vs_t = get_vs_at_dtf(E_MILP_Eco);
E_IDM_vs_t = get_vs_at_dtf(E_IDM_Eco);
m_MILP_vs_t = get_vs_at_dtf(m_MILP_Eco);
m_IDM_vs_t = get_vs_at_dtf(m_IDM_Eco);

% Con vs Seq, Seq vs PMP, Con vs PMP
E_MILP_Eco_vs = get_vs(E_MILP_Eco);
E_MILP_Eco_vs_at_dtf = get_vs_at_dtf(E_MILP_Eco_vs);
E_IDM_Eco_vs = get_vs(E_IDM_Eco);
E_IDM_Eco_vs_at_dtf = get_vs_at_dtf(E_IDM_Eco_vs);

m_MILP_Eco_vs = get_vs(m_MILP_Eco);
m_MILP_Eco_vs_at_dtf = get_vs_at_dtf(m_MILP_Eco_vs);
m_IDM_Eco_vs = get_vs(m_IDM_Eco);
m_IDM_Eco_vs_at_dtf = get_vs_at_dtf(m_IDM_Eco_vs);
% 车速分析
MILP_v_vs = get_vs(MILP_v);
MILP_v_vs_at_dtf = get_vs_at_dtf(MILP_v_vs);

IDM_v_vs = get_vs(IDM_v);
IDM_v_vs_at_dtf = get_vs_at_dtf(IDM_v_vs);

% final state t异常值30000，v 0.1
t_f_all(t_f_all==30000) = nan;
v_f_all(v_f_all==0.1) = nan;
% soc_f_all()
t_v_soc_f_all_mean = [nanmean(t_f_all);nanmean(v_f_all);nanmean(soc_f_all)]';

% t_c
tc_all_mean = mean(tc_all);
%% GWR_vs
try
close figure 3
catch
end
figure(3)
num_subfig = 1;
e_left = 0.12;
e_right = 1- 0.16;
e_bottom = 0.1;
e_top = 1- 0.02;
e_space = 0.05;
e_height = ((e_top-e_bottom)-(num_subfig-1)*e_space)/num_subfig;
set(gcf,'position',[100 100 400 250])

subplot(num_subfig,1,1)
boxplot((1-E_MILP_vs_IDM)*100,'Notch','on'); hold on; grid on; 
ylabel('Energy Saving \rm[%]');
% xlim([0,Scenario.s_max]);
% ylim([0,max(v_lim_list)+1]);
set(gca,'position',[e_left e_top-e_height e_right e_height]);
set(gca,'FontName','Times New Roman');
set(gca,'xticklabel',{'ConCVX','SeqCVX','PMP'});

print(gcf,'Fig/GWR_vs','-dpng','-r600');
print(gcf,'Fig/GWR_vs','-depsc');

%% E_vs_t
try
close figure 4
catch
end
figure(4)
num_subfig = 1;
e_left = 0.12;
e_right = 1- 0.16;
e_bottom = 0.18;
e_top = 1- 0.02;
e_space = 0.05;
e_height = ((e_top-e_bottom)-(num_subfig-1)*e_space)/num_subfig;
set(gcf,'position',[100 100 400 250])

subplot(num_subfig,1,1)
plot(E_MILP_vs_t./1000); hold on; grid on; 
plot(E_IDM_vs_t./1000,LineStyle="--");
xlabel('Arrival Time Increment \it\Delta {{t}_{f}} \rm[s]');
ylabel('Cumulative Energy Cost \itE_{mot,eq}^{cum} \rm[kJ]');
% xlim([0,Scenario.s_max]);
ylim([640,800]);
set(gca,'position',[e_left e_top-e_height e_right e_height]);
set(gca,'FontName','Times New Roman');
set(gca,'xticklabel',{'0','20','40','60','80','100','120'});
legend('GWR-MILP+ConCVX','GWR-MILP+SeqCVX','GWR-MILP+PMP',...
    'GWR-IDM+ConCVX','GWR-IDM+SeqCVX','GWR-IDM+PMP','NumColumns',2,Location='best');

print(gcf,'Fig/E_vs_t','-dpng','-r600');
print(gcf,'Fig/E_vs_t','-depsc');

%% Functions
function GWR_variable_vs = get_vs(GWR_variable)
GWR_variable_vs = [GWR_variable(:,1) ./ GWR_variable(:,2), GWR_variable(:,2) ./ GWR_variable(:,3), GWR_variable(:,1) ./ GWR_variable(:,3)];
end

function GWR_variable_vs_at_dtf = get_vs_at_dtf(GWR_variable_vs)
GWR_variable_vs_at_dtf = [];
for i = 1:7
GWR_variable_vs_at_dtf = [GWR_variable_vs_at_dtf;nanmean(GWR_variable_vs(7*(i-1)+1:7*i,:))];
end
end



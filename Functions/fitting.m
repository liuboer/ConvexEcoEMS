% Fitting functions f_m, f_f, and f_SoC
clear;close;clc

%% fit motor efficiency
mc_map_spd = (0:1000:10000) * (2 * pi) / 60; % motor speed list (rad/s)
mc_map_trq = (-200:20:200) * 4.448 / 3.281; % motor torque list (Nm)
mc_eff_map=[...
0.7 	0.7     0.7     0.7     0.7 	0.7     0.7 	0.7     0.7     0.7     0.7	0.7     0.7     0.7 	0.7 	0.7     0.7 	0.7 	0.7     0.7 	0.7
0.78	0.78	0.79	0.8 	0.81	0.82	0.82	0.82	0.81	0.77	0.7	0.77	0.81	0.82	0.82	0.82	0.81	0.8 	0.79	0.78	0.78
0.85	0.86	0.86	0.86	0.87	0.88	0.87	0.86	0.85	0.82	0.7	0.82	0.85	0.86	0.87	0.88	0.87	0.86	0.86	0.86	0.85
0.86	0.87	0.88	0.89	0.9     0.9 	0.9     0.9     0.89	0.87	0.7	0.87	0.89	0.9 	0.9     0.9     0.9     0.89	0.88	0.87	0.86
0.81	0.82	0.85	0.87	0.88	0.9 	0.91	0.91	0.91	0.88	0.7	0.88	0.91	0.91	0.91	0.9     0.88	0.87	0.85	0.82	0.81
0.82	0.82	0.82	0.82	0.85	0.87	0.9     0.91	0.91	0.89	0.7	0.89	0.91	0.91	0.9     0.87	0.85	0.82	0.82	0.82	0.82
0.79	0.79	0.79	0.78	0.79	0.82	0.86	0.9     0.91	0.9     0.7	0.9     0.91	0.9     0.86	0.82	0.79	0.78	0.79	0.79	0.79
0.78	0.78	0.78	0.78	0.78	0.78	0.8     0.88	0.91	0.91	0.7	0.91	0.91	0.88	0.8     0.78	0.78	0.78	0.78	0.78	0.78
0.78	0.78	0.78	0.78	0.78	0.78	0.78	0.8     0.9     0.92	0.7	0.92	0.9     0.8     0.78	0.78	0.78	0.78	0.78	0.78	0.78
0.78	0.78	0.78	0.78	0.78	0.78	0.78	0.78	0.88	0.92	0.7	0.92	0.88	0.78	0.78	0.78	0.78	0.78	0.78	0.78	0.78
0.78	0.78	0.78	0.78	0.78	0.78	0.78	0.78	0.8 	0.92	0.7	0.92	0.8     0.78	0.78	0.78	0.78	0.78	0.78	0.78	0.78];
% max torque curve of the motor indexed by mc_map_spd
mc_max_trq = [200, 200, 200, 175.2, 131.4, 105.1, 87.6, 75.1, 65.7, 58.4, 52.5] * 4.448/3.281; % (N*m)
mc_max_gen_trq = -1 * [200, 200, 200, 175.2, 131.4, 105.1, 87.6, 75.1, 65.7, 58.4, 52.5] * 4.448/3.281; % (N*m), estimate

% remove infeasible work points
[T_map, w_map] = meshgrid(mc_map_trq, mc_map_spd);

for ii = 1:length(mc_map_spd)
    spd = mc_map_spd(ii);
    trq_pos = mc_max_trq(ii);
    trq_neg = mc_max_gen_trq(ii);
    none_idx = [find(T_map(ii,:)>trq_pos),find(T_map(ii,:)<trq_neg)];
    mc_eff_map(ii,none_idx) = nan;
    T_map(ii,none_idx) = nan;
    w_map(ii,none_idx) = nan;
end

P_mot_m_map = T_map .* w_map /1000; % kW
P_mot_e_map = P_mot_m_map .* (mc_eff_map.^(-sign((P_mot_m_map))));
v_map = w_map/6.67 * 0.282; % m/s

[xData, yData, zData] = prepareSurfaceData( P_mot_m_map, v_map, P_mot_e_map );

% Set up fittype and options.
ft = fittype( 'p1*x^2 + p2*x + p3*y', 'independent', {'x', 'y'}, 'dependent', 'z' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.Robust = 'LAR';
opts.StartPoint = [2e-3 1 3e-2];
% opts.Lower = [0 0.9 1e-3];
% opts.Upper = [1e-2 1.1 1e-1];

% Fit model to data.
[f_m.fitresult, f_m.gof] = fit( [xData, yData], zData, ft, opts );

% save('f_m.mat','f_m')

try
close figure 1
catch
end
figure(1)
num_subfig = 1;
e_left = 0.12;
e_right = 1- 0.16;
e_bottom = 0.14;
e_top = 1- 0.02;
e_space = 0.05;
e_height = ((e_top-e_bottom)-(num_subfig-1)*e_space)/num_subfig;
set(gcf,'position',[100 100 400 250])

subplot(num_subfig,1,1)
plot(f_m.fitresult,[xData, yData],zData); hold on; grid on; 
xlabel('${{P}_{mot,m}}$ \rm[kW]','interpreter','latex');
ylabel('\itv \rm[m/s]');
zlabel('${{P}_{mot,e}}$ \rm[kW]','interpreter','latex');
% xlim([0,Scenario.s_max]);
% ylim([0,max(v_lim_list)+1]);
set(gca,'position',[e_left e_top-e_height e_right e_height]);
set(gca,'FontName','Times New Roman');
% legend('Original $f_m$','Fitted $f_m$','interpreter','latex');

print(gcf,'../Fig/Fitted_f_m','-dpng','-r600');
print(gcf,'../Fig/Fitted_f_m','-depsc');
%% fit FCS efficiency
fc_pwr_map = [0, 2, 5, 7.5, 10, 20, 30, 40, 50]; % % kW (net) including parasitic losses
fc_eff_map = [10, 33, 49.2, 53.3, 55.9, 59.6, 59.1, 56.2, 50.8] / 100; % % efficiency indexed by fc_pwr
h2_pwr_map = fc_pwr_map./fc_eff_map;

fc_fuel_map = [0.012, 0.05, 0.085, 0.117, 0.149, 0.280, 0.423, 0.594, 0.821]; % fuel use map (g/s)
fc_fuel_lhv = 120.0*1000; % (J/g), lower heating value of the fuel
h2_pwr_map = fc_fuel_map.*fc_fuel_lhv/1000;

[xData, yData] = prepareCurveData( fc_pwr_map, h2_pwr_map );

ft = fittype( 'poly2' );
opts = fitoptions( 'Method', 'LinearLeastSquares' );

% % acutally used in paper
% opts.Lower = [0 -Inf -Inf];
% opts.Upper = [0 Inf Inf];
% [f_f.fitresult, f_f.gof] = fit( xData, yData, ft, opts );
% % save('f_f_ax2+bx.mat','f_f');

% 
[f_f.fitresult, f_f.gof] = fit( xData, yData, ft, opts );
save('f_f.mat','f_f');

try
close figure 2
catch
end
figure(2)
num_subfig = 1;
e_left = 0.12;
e_right = 1- 0.16;
e_bottom = 0.18;
e_top = 1- 0.02;
e_space = 0.05;
e_height = ((e_top-e_bottom)-(num_subfig-1)*e_space)/num_subfig;
set(gcf,'position',[100 100 400 250])

subplot(num_subfig,1,1)
plot(f_f.fitresult,fc_pwr_map',h2_pwr_map); hold on; grid on; 
xlabel('Net Power \it{P_{fcs}} \rm[kW]');
ylabel('Hydrogen Power ${{P}_{H}}$ \rm[kW]','interpreter','latex');
% xlim([0,Scenario.s_max]);
% ylim([0,max(v_lim_list)+1]);
set(gca,'position',[e_left e_top-e_height e_right e_height]);
set(gca,'FontName','Times New Roman');
legend('Original', 'Fitted',Location='best');

print(gcf,'../Fig/Fitted_f_f','-dpng','-r600');
print(gcf,'../Fig/Fitted_f_f','-depsc');

pwr = 1:50;
eff = pwr'./f_f.fitresult(pwr);

try
close figure 3
catch
end
figure(3)
num_subfig = 1;
e_left = 0.12;
e_right = 1- 0.16;
e_bottom = 0.18;
e_top = 1- 0.02;
e_space = 0.05;
e_height = ((e_top-e_bottom)-(num_subfig-1)*e_space)/num_subfig;
set(gcf,'position',[100 100 400 250])

subplot(num_subfig,1,1)
plot(fc_pwr_map,fc_eff_map); hold on; grid on; 
plot(eff);
xlabel('Net Power \it{P_{fcs}} \rm[kW]');
ylabel('Efficiency ${{\eta}_{fcs}}$ \rm[1]','interpreter','latex');
% xlim([0,Scenario.s_max]);
% ylim([0,max(v_lim_list)+1]);
set(gca,'position',[e_left e_top-e_height e_right e_height]);
set(gca,'FontName','Times New Roman');
legend('Original', 'Fitted',Location='best');

print(gcf,'../Fig/eta_fcs','-dpng','-r600');
print(gcf,'../Fig/eta_fcs','-depsc');

%% Battery
% Battery parameters
Num_cell = 25;
ess_Q = 26 * 3600; % coulombs, battery package capacity
% resistance and OCV list
ess_soc_map = 0:0.1:1;
% module's resistance to being discharged, indexed by ess_soc
ess_r_dis_map = [40.7, 37.0, 33.8, 26.9, 19.3, 15.1, 13.1, 12.3, 11.7, 11.8, 12.2] / 1000 * Num_cell; % (ohm)
% module's resistance to being charged, indexed by ess_soc
ess_r_chg_map = [31.6, 29.8, 29.5, 28.7, 28.0, 26.9, 23.1, 25.0, 26.1, 28.8, 47.2] / 1000 * Num_cell; % (ohm)
% module's open-circuit (a.k.a. no-load) voltage, indexed by ess_soc
ess_voc_map = [11.70, 11.85, 11.96, 12.11, 12.26, 12.37, 12.48, 12.59, 12.67, 12.78, 12.89] * Num_cell; % (V)
% Battery limitations
ess_min_volts = 9.5 * Num_cell; % 237.5 V
ess_max_volts = 16.5 * Num_cell; % 412.5 V


P_bat = (-50:2:50)*1000;
soc = 0.5:0.1:0.7;
[ess_soc,ess_pwr] = meshgrid(soc,P_bat);

ess_eff = (ess_pwr > 0) + (ess_pwr <= 0) .* 0.9;
ess_voc = interp1(ess_soc_map, ess_voc_map, ess_soc, 'linear', 'extrap');
ess_r_int = (ess_pwr > 0) .* interp1(ess_soc_map, ess_r_dis_map, ess_soc, 'linear', 'extrap')...
    + (ess_pwr <= 0) .* interp1(ess_soc_map, ess_r_chg_map, ess_soc, 'linear', 'extrap');
ess_cur = ess_eff .* (ess_voc - sqrt(ess_voc.^2 - 4 .* ess_r_int .* ess_pwr)) ./ (2*ess_r_int);
ess_volt = ess_voc - ess_cur .* ess_r_int;
ess_soc_new = ess_soc - ess_cur ./ ess_Q;
ess_soc_new = (conj(ess_soc_new) + ess_soc_new) ./ 2;
soc_d = ess_soc_new - ess_soc;

P_bat = P_bat./1000;
[xData, yData, zData] = prepareSurfaceData( P_bat, soc, soc_d );

% Set up fittype and options.
ft = fittype( 'poly11' );

% Fit model to data.
[fit_bat.fitresult, fit_bat.gof] = fit( [xData, yData], zData, ft );
par_bat = [fit_bat.fitresult.p00,fit_bat.fitresult.p10,fit_bat.fitresult.p01];

% 
try
close figure 4
catch
end
figure(4)
num_subfig = 1;
e_left = 0.12;
e_right = 1- 0.16;
e_bottom = 0.1;
e_top = 1- 0.02;
e_space = 0.05;
e_height = ((e_top-e_bottom)-(num_subfig-1)*e_space)/num_subfig;
set(gcf,'position',[100 100 400 250])

subplot(num_subfig,1,1)
plot(fit_bat.fitresult,[xData, yData],zData); hold on; grid on; 
xlabel('\it{P_{bat}} \rm[kW]');
ylabel('\itSoC \rm[1]');
zlabel('$S\dot{o}C$ \rm[1/s]','interpreter','latex');
% xlim([0,Scenario.s_max]);
% ylim([0,max(v_lim_list)+1]);
set(gca,'position',[e_left e_top-e_height e_right e_height]);
set(gca,'FontName','Times New Roman');
view([45,45])

print(gcf,'../Fig/dSoC_linear','-dpng','-r600');
print(gcf,'../Fig/dSoC_linear','-depsc');

function out = FCV_EcoEMS_Plant_T(s,v,soc,a,p_fcs,delta_t,slope_t)
%% FCHEV plant in time domain.

% dynamics 
v_new = v + a * delta_t;
v_mean = (v+v_new)/2;
v_mean = v_new;
s_new = s + v_mean * delta_t;

%% parametas
% Vehicle parametas
veh_whl_radius = 0.282;
veh_mass = 1380;
veh_rrc  = 0.009;
veh_air_density = 1.2;
veh_FA = 2.0;
veh_CD = 0.335;
veh_gravity = 9.81;
veh_fd_ratio = 6.67;


% Fuel cell parameters
fc_pwr_min = 0;
fc_pwr_max = 50*1000;
% fc_pwr_dif_max = 20*1000;
fc_pwr_map = [0, 2, 5, 7.5, 10, 20, 30, 40, 50] * 1000; % % kW (net) including parasitic losses
% fc_fuel_map = [0.012, 0.05, 0.085, 0.117, 0.149, 0.280, 0.423, 0.594, 0.821]; % fuel use map (g/s)

% another war to calculate fc_fuel_map
fc_eff_map = [10, 33, 49.2, 53.3, 55.9, 59.6, 59.1, 56.2, 50.8] / 100; % % efficiency indexed by fc_pwr
fc_fuel_lhv = 120.0*1000; % (J/g), lower heating value of the fuel
fc_fuel_map2 = fc_pwr_map .* (1./fc_eff_map) / fc_fuel_lhv; % fuel consumption map (g/s)


% Motor parameters
% efficiency map indexed vertically by mc_map_spd and horizontally by mc_map_trq
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

% % another way to calculate mc_inpwr_map
% % find indices of well-defined efficiencies (where speed and torque > 0)
% pos_trqs = mc_map_trq > 0;
% pos_spds = mc_map_spd > 0;
% % compute losses in well-defined efficiency area
% [T1, w1] = meshgrid(mc_map_trq(pos_trqs), mc_map_spd(pos_spds));
% mc_outpwr1_map = T1 .* w1;
% mc_losspwr_map = (1./mc_eff_map(pos_spds,pos_trqs)-1) .* mc_outpwr1_map;
% % compute losses in entire operating range
% mc_losspwr_map = [fliplr(mc_losspwr_map), mc_losspwr_map(:,1), mc_losspwr_map];
% mc_losspwr_map = [mc_losspwr_map(1,:); mc_losspwr_map];
% % compute input power (power req'd at electrical side of motor/inverter set)
% [T,w] = meshgrid(mc_map_trq, mc_map_spd);
% mc_outpwr_map = T .* w;
% mc_inpwr_map = mc_outpwr_map + mc_losspwr_map;


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

%% Update
alpha_a = 0.5 .* veh_air_density .* veh_FA .* veh_CD;
F_roll = veh_mass .* veh_gravity .* (veh_rrc.*cos(slope_t)+sin(slope_t));
F_drag = alpha_a .* (v_mean.^2);
F_acc = veh_mass .* a;
P_dem_m = v_mean .* (F_acc + F_roll + F_drag);
w_whl = v_mean ./ veh_whl_radius;
T_whl = veh_whl_radius .* (F_acc + F_roll + F_drag);

% Fuel cell
% fc_pwr = (P_dem_m > 0) .* fc_pwr;
fc_fuel = interp1(fc_pwr_map, fc_fuel_map2, p_fcs, 'linear', 'extrap');
% fc_fuel = interp1(fc_pwr_map, fc_fuel_map, fc_pwr, 'linear', 'extrap');
inf_fc = (p_fcs < fc_pwr_min) + (p_fcs > fc_pwr_max);% + (abs(fc_pwr-fc_pwr2) > fc_pwr_dif_max);
% X{2} = fc_pwr;

% Motor
mc_spd = w_whl .* veh_fd_ratio;
mc_trq = T_whl ./ veh_fd_ratio;
mc_eff = (mc_spd == 0) + (mc_spd ~= 0) .* interp2(mc_map_trq, mc_map_spd, mc_eff_map, mc_trq, mc_spd);
inf_mc = (isnan(mc_eff)) + (mc_trq < 0) .* (mc_trq < interp1(mc_map_spd, mc_max_gen_trq, mc_spd, 'linear', 'extrap')) + ...
    (mc_trq >= 0) .* (mc_trq > interp1(mc_map_spd, mc_max_trq, mc_spd, 'linear', 'extrap'));
mc_eff(isnan(mc_eff)) = 1;
mc_outpwr = mc_trq .* mc_spd;
mc_inpwr =  mc_outpwr .* (mc_eff.^(-sign((mc_outpwr))));
% mc_inpwr = (mc_spd ~= 0) .* interp2(mc_map_trq, mc_map_spd, mc_inpwr_map, mc_trq, mc_spd); % another way to calculate mc_inpwr

% Battery
ess_pwr = mc_inpwr - p_fcs;
ess_eff = (ess_pwr > 0) + (ess_pwr <= 0) .* 0.9;
ess_voc = interp1(ess_soc_map, ess_voc_map, soc, 'linear', 'extrap');
ess_r_int = (ess_pwr > 0) .* interp1(ess_soc_map, ess_r_dis_map, soc, 'linear', 'extrap')...
    + (ess_pwr <= 0) .* interp1(ess_soc_map, ess_r_chg_map, soc, 'linear', 'extrap');
ess_cur = ess_eff .* (ess_voc - sqrt(ess_voc.^2 - 4 .* ess_r_int .* ess_pwr)) ./ (2*ess_r_int);
ess_volt = ess_voc - ess_cur .* ess_r_int;
inf_ess = (ess_voc.^2 < 4 .* ess_r_int .* ess_pwr) + (ess_volt < ess_min_volts) + (ess_volt > ess_max_volts);
soc_new = soc - ess_cur ./ ess_Q;
soc_new = (conj(soc_new) + soc_new) ./ 2;

% infeasiable
I = (inf_fc + inf_mc + inf_ess ~= 0);

H2_cost = fc_fuel .* delta_t;

% Output
out.veh_spd = v_new; %
out.veh_dis = s_new; %
out.P_dem_m = P_dem_m;
out.P_dem_e = mc_inpwr;
out.Mot_spd = mc_spd;
out.Mot_trq = mc_trq;
out.Mot_pwr = mc_outpwr;
out.Bat_soc = soc_new; %
out.Bat_vol = ess_volt;
out.Bat_pwr = ess_pwr;
out.Mot_eta = mc_eff;
out.FCS_pwr = p_fcs;
% out.FCS_eta = fc_eff;
out.FC_fuel = fc_fuel;
out.Inf_tot = I;
out.H2_cost = H2_cost;
end

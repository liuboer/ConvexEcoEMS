function [X, C, I, out] = FCV_EcoEMS_DP_advisor_model_S(inp,par)
%% FCHEV model used for DP.

%% define varibles
ds = par.ds;
use_gpu = par.use_gpu;

if use_gpu
veh_spd = gpuArray(inp.X{2}); % speed v, m/s
ess_soc = gpuArray(inp.X{3}); % SoC
veh_acc = gpuArray(inp.U{1}); % acceleration a, m/s^2
fc_pwr = gpuArray(inp.U{2}); % P_fcs, W
inp.X{1} = gpuArray(inp.X{1});
inp.W{1} = gpuArray(inp.W{1});
inp.W{2} = gpuArray(inp.W{2});
inp.W{3} = gpuArray(inp.W{3});
inp.W{4} = gpuArray(inp.W{4});
else
veh_spd = inp.X{2}; % speed v, m/s
ess_soc = inp.X{3}; % SoC
veh_acc = inp.U{1}; % acceleration a, m/s^2
fc_pwr = inp.U{2}; % P_fcs, W
end

% update states
X{2} = sqrt(max(veh_spd.^2 + 2.*veh_acc.*ds,2));    % update v
v_mean= (veh_spd+X{2})./2;
% if veh_acc~=0
% v_mean = ((veh_spd.^2+2.*veh_acc.*ds).^(3/2)-veh_spd.^3) ./ (3.*veh_acc.*ds);
% end
v_mean = X{2};
dt = ds./v_mean;

% dt = ds./(veh_spd);
% X{2} = veh_spd + veh_acc.*dt;    % update v

X{1} = inp.X{1} + dt;               % update t

% environment constraints: traffic lights
inf_traff = zeros(size(X{2}));
if inp.W{1} == 1 % running a red light
    traf_t = (X{1} + inp.W{4} - floor((X{1}+inp.W{4})/inp.W{2})*inp.W{2}).*inp.W{1};
    inf_traff = (traf_t <= inp.W{3});
end

%% Vehicle model
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
fc_fuel_map = [0.012, 0.05, 0.085, 0.117, 0.149, 0.280, 0.423, 0.594, 0.821]; % fuel use map (g/s)

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
% Power demand
alpha_a = 0.5 .* veh_air_density .* veh_FA .* veh_CD;
F_roll = veh_mass .* veh_gravity .* (veh_rrc.*cos(inp.W{5})+sin(inp.W{5}));
F_drag = alpha_a .* (v_mean.^2);
F_acc = veh_mass .* veh_acc;
P_dem_m = v_mean .* (F_acc + F_roll + F_drag);
% if veh_acc~=0
% P_dem_m = (F_acc+F_roll).*v_mean + alpha_a.*((veh_spd.^2+2.*veh_acc.*ds).^(5/2)-veh_spd.^5) ./ (5.*veh_acc.*ds);
% end
w_whl = v_mean ./ veh_whl_radius;
T_whl = veh_whl_radius .* (F_acc + F_roll + F_drag);

% Fuel cell
% fc_pwr = (P_dem_m > 0) .* fc_pwr;
% fc_fuel = interp1(fc_pwr_map, fc_fuel_map2, fc_pwr, 'linear', 'extrap');
fc_fuel = interp1(fc_pwr_map, fc_fuel_map, fc_pwr, 'linear', 'extrap');
inf_fc = (fc_pwr < fc_pwr_min) + (fc_pwr > fc_pwr_max);% + (abs(fc_pwr-fc_pwr2) > fc_pwr_dif_max);
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
ess_pwr = mc_inpwr - fc_pwr;
ess_eff = (ess_pwr > 0) + (ess_pwr <= 0) .* 0.9;
ess_voc = interp1(ess_soc_map, ess_voc_map, ess_soc, 'linear', 'extrap');
ess_r_int = (ess_pwr > 0) .* interp1(ess_soc_map, ess_r_dis_map, ess_soc, 'linear', 'extrap')...
    + (ess_pwr <= 0) .* interp1(ess_soc_map, ess_r_chg_map, ess_soc, 'linear', 'extrap');
ess_cur = ess_eff .* (ess_voc - sqrt(ess_voc.^2 - 4 .* ess_r_int .* ess_pwr)) ./ (2*ess_r_int); % note here
ess_volt = ess_voc - ess_cur .* ess_r_int;
inf_ess = (ess_voc.^2 < 4 .* ess_r_int .* ess_pwr) + (ess_volt < ess_min_volts) + (ess_volt > ess_max_volts);
ess_soc_new = ess_soc - ess_cur ./ ess_Q .* dt;
ess_soc_new = (conj(ess_soc_new) + ess_soc_new) ./ 2;
X{3} = ess_soc_new;

% infeasiable
if use_gpu
I = uint32(inf_fc + inf_mc + inf_ess + inf_traff ~= 0);
else
I = inf_fc + inf_mc + inf_ess + inf_traff ~= 0;
end

% COST
H2_cost = fc_fuel .* dt;
acc_cost = abs(veh_acc).^2 .* 0.1 .* dt;
C{1} = H2_cost + acc_cost;

par.a_last = veh_acc;
% Output
out.veh_acc = veh_acc;
out.FCS_pwr = fc_pwr;
out.P_dem_m = P_dem_m;
out.P_dem_e = mc_inpwr;
out.Bat_pwr = ess_pwr;
out.FC_fuel = fc_fuel;
out.H2_cost = H2_cost;
out.acc_cost = acc_cost;
out.Inf_tot = I;

out.Mot_spd = mc_spd;
out.Mot_trq = mc_trq;
out.Bat_vol = ess_volt;
out.Mot_eta = mc_eff;
end

function PMP_res_ux = AnalyticalSolution(Scenario,Scenario1,eqn_order,t_p_ref,ds)
v_0 = 2;
a_0 = 0;
a_T = 0;

tc_start_1 = tic;
tc_start_2 = cputime;

v_mean_BVP =  diff(Scenario.S_list_e) ./ diff(t_p_ref.t_p_e);

PMP_res_ux.a = [];
PMP_res_ux.v = [];

for i = 1:Scenario.N

T = t_p_ref.t_p_e(i+1) - t_p_ref.t_p_e(i);
v_max = Scenario.v_lim_brk(i);

s_0 = Scenario.S_list_e(i);
s_T = Scenario.S_list_e(i+1);

if i == Scenario.N
v_T = 2;
else
v_T = min([mean(v_mean_BVP(i:i+1)),v_max,Scenario.v_lim_brk(i+1)]);
end

[a_PMP_t,v_PMP_t,s_PMP_t,x,error_flag] = PMP(v_0,v_T,s_0,s_T,v_max,T,eqn_order,a_0,a_T,1);

if error_flag
disp(error_flag);
% [a_PMP_t,v_PMP_t,s_PMP_t,x,error_flag] = PMP(v_0,v_T,s_0,s_T,v_max,T,eqn_order,a_0,a_T,1);
end

if i == 1
PMP_res_ux.a = [PMP_res_ux.a,a_PMP_t];
PMP_res_ux.v = [PMP_res_ux.v,v_PMP_t];
else
PMP_res_ux.a = [PMP_res_ux.a,a_PMP_t(2:end)];
PMP_res_ux.v = [PMP_res_ux.v,v_PMP_t(2:end)];
end

v_0 = v_T;
a_0 = PMP_res_ux.a(end);

end
PMP_res_ux.s = cumsum(PMP_res_ux.v);

PMP_res_ux.a_S = interp1(PMP_res_ux.s,PMP_res_ux.a,ds:ds:Scenario.s_max,"nearest","extrap");
PMP_res_ux.v_S = interp1(PMP_res_ux.s,PMP_res_ux.v,0:ds:Scenario.s_max,"nearest","extrap");
PMP_res_ux.a_S_from_v_S = diff(PMP_res_ux.v_S.^2)./(2*ds);

tc_end_1 = toc;
tc_end_2 = cputime;
tc = tc_end_1 - tc_start_1;
tc_CPU = tc_end_2 - tc_start_2;
PMP_res_ux.tc.tc = tc;
PMP_res_ux.tc.tc_CPU = tc_CPU;

% plot
s_PMP = PMP_res_ux.s;
t_PMP = 1:t_p_ref.t_p_e(end);
a_PMP = PMP_res_ux.a;
v_PMP = PMP_res_ux.v;

v_max_list = interp1(1:Scenario.s_max,Scenario.v_lim,s_PMP,"nearest","extrap");
t_T = min(length(s_PMP),length(t_PMP));

figure(1)
num_subfig = 3;
e_left = 0.1;
e_right = 1- 0.15;
e_below = 0.06;
e_top = 1- 0.02;
e_space = 0.05;
e_height = ((e_top-e_below)-(num_subfig-1)*e_space)/num_subfig;
set(gcf,'position',[100 100 400 600])

subplot(311)
plot(s_PMP(1:t_T),t_PMP(1:t_T)); hold on; grid on
plot_scenario(Scenario1,t_PMP(end))
% xlabel('Distance \its \rm[m]');
ylabel('Time \itt \rm[s]');
xlim([0,Scenario.s_max]);
ylim([0,t_PMP(end)]);
set(gca,'position',[e_left e_top-e_height e_right e_height]);
set(gca,'FontName','Times New Roman');

subplot(312)
plot(s_PMP,v_PMP); hold on; grid on
% plot(s_PMP,v_max_list);
% xlabel('Distance \its \rm[m]'); 
ylabel('Speed \itv \rm[m/s]');
xlim([0,Scenario.s_max]);
ylim([0,max(v_max_list)+2]);
set(gca,'position',[e_left e_below+e_height+e_space e_right e_height]);
set(gca,'FontName','Times New Roman');

subplot(313)
plot(s_PMP,a_PMP); hold on; grid on
xlabel('Distance \its \rm[m]');
ylabel('Acceleration \ita \rm[m/s^2]');
xlim([0,Scenario.s_max]);
ylim([-2,2]);
set(gca,'position',[e_left e_below e_right e_height]);
set(gca,'FontName','Times New Roman');

% saveas(gcf,['Fig/GWR' num2str(seed) '.png'])
% saveas(gcf,['Fig/GWR' num2str(seed) '.eps'])
end
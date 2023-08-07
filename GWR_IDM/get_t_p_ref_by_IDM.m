function par = get_t_p_ref_by_IDM(Scenario,varargin)
% get references for vehicle positon and time.
p = inputParser;
addParameter(p,'v_0',2);
addParameter(p,'a_min',-2);
addParameter(p,'a_max',2);
addParameter(p,'t_f_unfixed',1);
addParameter(p,'dt_f',5);
addParameter(p,'dt_f_max',50);
parse(p,varargin{:});

% Output:
% par.N: driving time
% par.s_upper_ref: indexd by time, length=par.N
% par.s_lower_ref: indexd by time, length=par.N
% par.t_lower_ref: indexd by position, length=par.s_max
% par.t_upper_ref: indexd by position, length=par.s_max

tc_start_1 = tic;
tc_start_2 = cputime;

 % Use IDM to get s_ref_upper
[s_upper_ref,~,~,~] = IDM(Scenario,v_0=p.Results.v_0,a_min=p.Results.a_min,a_max=p.Results.a_max); % indexd by time

% Remove the same point
same_s_idx = find(diff(s_upper_ref)==0);
s_upper_ref_idx = 1:length(s_upper_ref);
if ~isnan(same_s_idx)
s_upper_ref(same_s_idx) = [];
s_upper_ref_idx(same_s_idx) = [];
end

t_small_IDM = interp1(s_upper_ref,s_upper_ref_idx,1:Scenario.s_max, 'linear', 'extrap'); % indexd by position
t_pass_light_small = interp1(1:Scenario.s_max,t_small_IDM,Scenario.S_list, 'linear', 'extrap'); % passing time
t_pass_light_big = zeros(size(t_pass_light_small)); % end time of the current green phase
for i = 1:length(t_pass_light_small)
    t_pass_light_big(i) = ceil((Scenario.T_0(i)+t_pass_light_small(i))/Scenario.T(i)).*Scenario.T(i)-Scenario.T_0(i);  
    t_pass_light_min(i) = t_pass_light_big(i)- Scenario.T_g(i);
end

tc_end_1 = toc;
tc_end_2 = cputime;

t_f_fixed = ceil(t_pass_light_small(end))+p.Results.dt_f;
if p.Results.t_f_unfixed
t_pass_light_small(end) = ceil(t_pass_light_small(end));
t_pass_light_big(end) = round(t_pass_light_small(end))+p.Results.dt_f_max;
else
t_pass_light_small(end) = t_f_fixed-p.Results.dt_f_max/2;
t_pass_light_big(end) = t_f_fixed+p.Results.dt_f_max/2;
end
par.N = round(t_pass_light_big(end)); % maximum final time

for idx_i = length(t_pass_light_big):-1:2
    if (t_pass_light_big(idx_i) - t_pass_light_big(idx_i-1)) <= 0
        t_pass_light_big(idx_i-1) = t_pass_light_big(idx_i) - 1;
    end
end

% get s_ref_lower
s_lower_ref = interp1([0,1,5,10,t_pass_light_big],[0,0.5,1,10,Scenario.S_list],1:par.N, 'linear', 'extrap');
t_big = interp1(s_lower_ref,1:length(s_lower_ref),1:Scenario.s_max, 'linear', 'extrap');
t_small_2 = interp1([0,20,Scenario.S_list],[0,0.1,t_pass_light_small],1:Scenario.s_max, 'linear', 'extrap');
s_lower_ref_2 = interp1([0,0.1,t_pass_light_small],[0,20,Scenario.S_list],1:t_pass_light_small(end), 'linear', 'extrap');

% final references
par.s_upper_ref = reshape([s_upper_ref(2:end),s_upper_ref(end)*ones(1,par.N-length(s_upper_ref(2:end)))],[],1);
par.s_upper_ref_2 = reshape(s_lower_ref_2,[],1);
par.s_lower_ref = reshape(s_lower_ref,[],1);
par.t_lower_ref = reshape(t_small_IDM,[],1);
par.t_lower_ref_2 = reshape(t_small_2,[],1);
par.t_upper_ref = reshape(t_big,[],1);
par.t_f_fixed = t_f_fixed;

par.t_p = t_pass_light_small(1:end-1);
par.t_p_min = t_pass_light_min(1:end-1);
par.t_p_max = t_pass_light_big(1:end-1);
par.t_p_ = [par.t_p;par.t_p_min;par.t_p_max];

t_f_des = ceil(length(s_upper_ref)/5)*5+p.Results.dt_f;
par.t_p_e = [0,par.t_p,t_f_des];
par.t_p_min_e = [0,par.t_p_min,t_f_des];
par.t_p_max_e = [0,par.t_p_max,t_f_des];
par.t_p_e_ = [par.t_p_e;par.t_p_min_e;par.t_p_max_e];

tc = tc_end_1 - tc_start_1;
tc_CPU = tc_end_2 - tc_start_2;
par.tc.tc = tc;
par.tc.tc_CPU = tc_CPU;
end
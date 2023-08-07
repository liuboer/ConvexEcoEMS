function [s,v,a,sv_brk] = IDM(Scenario,varargin)
p = inputParser;
addParameter(p,'v_0',2);
addParameter(p,'a_min',-2);
addParameter(p,'a_max',2);
parse(p,varargin{:});

s_max = Scenario.s_max;
S_list = Scenario.S_list;
s_brk = Scenario.s_brk;
v_lim_brk = Scenario.v_lim_brk;
T = Scenario.T;
T_r = Scenario.T_r+3;
T_0 = Scenario.T_0+3;

ss = 1:s_max;
v_lim = Scenario.v_lim;

if s_brk(1)>100
    sv_brk.s_brk = [0,sort([s_brk(1:end-1)-100,s_brk])];
    sv_brk.v_lim_brk = reshape(repmat(v_lim_brk,2,1),1,[]);
else
    sv_brk.s_brk = max(0,sort([s_brk(1:end-1)-100,s_brk]));
    sv_brk.v_lim_brk = reshape(repmat(v_lim_brk,2,1),1,[]);
    sv_brk.v_lim_brk = sv_brk.v_lim_brk(2:end);
end

v_max = 60/3.6; % m/s
a_max = p.Results.a_max; % m/s^2
b_com = abs(p.Results.a_min); % m/s^2

s_0 = 0; % spacing, m
Ts = 0; % time spacing, s
v(1) = p.Results.v_0;
s(1) = 0;
ds_v2v_des(1) = s_0; 
ds_v2v(1) = s_0; % 
v_ref = v_max;
b_max = b_com;
t = 2;

mm = 0;
statu_T = [];
v_max_list = [];

while t < s_max % /m
    v_max = interp1(ss,v_lim,s(t-1)); %
    v_max_ = v_max; %
    if interp1(sv_brk.s_brk,sv_brk.v_lim_brk,s(t-1)+100) < v_max
    v_max = min(interp1(sv_brk.s_brk,sv_brk.v_lim_brk,s(t-1)),interp1(sv_brk.s_brk,sv_brk.v_lim_brk,s(t-1)+100));
    else
    v_max = min(interp1(ss,v_lim,s(t-1)),interp1(ss,v_lim,s(t-1)+100));
    end
    v_max_list(t-1) = v_max;

    id1 = sum(S_list - s(t-1) <= 0) + 1;
    [~,id2] = find(0 < (S_list - s(t-1)) & (S_list - s(t-1)) < 100);
    [~,id3] = find(0 < (S_list - s(t-1)) & (S_list - s(t-1)) <= 30);

    ds_v2v(t-1) = S_list(id1) - s(t-1);
    
    if t>2
        [~,id4] = find(30 <= (S_list - s(t-2)) & (S_list - s(t-2)) < 100);
        if ~isempty(id3) && ~isempty(id4)
            mm = mm + 1;
            statu_T(mm) = t-1 + T_0(id1) - floor( (t-1+T_0(id1)) / T(id1) ) * T(id1) > T_r(id1); % green 0-30m
        end
    end

    statu = (t-1 + T_0(id1) - floor( (t-1+T_0(id1)) / T(id1) ) * T(id1) <= T_r(id1)); % red or yellow
    
    if ~isempty(id2)
        v_ref = 0;
        b_max = 2 * b_com;
        if statu
            a(t-1) = max(-(v(t-1)^2)/(2*ds_v2v(t-1)));
        else
            a(t-1) = a_max * (1- (v(t-1)/v_max)^4);
        end
        
        if isempty(id3) && statu
            a(t-1) = a_max * (1- (v(t-1)/v_max)^4 - (ds_v2v_des(t-1) / ds_v2v(t-1))^2);
        end

        if ~isempty(id3) && statu_T(mm)
            a(t-1) = a_max * (1- (v(t-1)/v_max)^4);
        end

        if id1>=Scenario.N
            a(t-1) = max(-(v(t-1)^2-v(1)^2)/(2*ds_v2v(t-1)));
        end
        
    else
        v_ref = v_max;
        b_max = b_com; 
        a(t-1) = a_max * (1- (v(t-1)/v_max)^4 - (ds_v2v_des(t-1) / ds_v2v(t-1))^2);
    end

    
    v(t) = min(max(v(t-1)+a(t-1) , 0),v_max_); % >=0
    s(t) = s(t-1) + v(t);

    deta_v(t) = v_ref - v(t); % 
    ds_v2v_des(t) = max(v(t)*Ts - v(t)*deta_v(t) / (2*(a_max*b_max)^0.5) , 0) + s_0;
    t = t+1;
    
    if (-1e-1 < (s(t-1) - s_max) && (s(t-1) - s_max) <= 0) || (s(t-1) - s_max) > 0
        t = 1e10;
    end
end

end
function Scenario = get_scenarios_by_index(S_idx,varargin)
p = inputParser;
addParameter(p,'destination_as_light',1);
addParameter(p,'constant_v_lim',0);
addParameter(p,'constant_slope',0);
% only for scenario 0,11-13
addParameter(p,'s_loss',200);
addParameter(p,'num_light',6);
% only for random scenarios
addParameter(p,'seed',1);
parse(p,varargin{:});

switch S_idx

    case 1
        Scenario.S_list = [300, 900, 1200, 1700]; % traffic light position, m
        Scenario.s_max = Scenario.S_list(end)+300;
        Scenario.T_0 = [10, 40, 50, 30];
        Scenario.T = [70, 60, 50, 60];
        Scenario.T_g = [30, 40, 30, 30];
        Scenario.s_brk = [400,800,1200,1600,Scenario.s_max];
        Scenario.v_lim_brk = [60,40,50,60,40]/3.6;
        s_grid = 1:Scenario.s_max;
        Scenario.slope = 0.020*sin(2*pi/(Scenario.s_max/1)*(s_grid-Scenario.S_list(2)))...
            + 0.010*sin(2*pi/(Scenario.s_max/2)*(s_grid-Scenario.S_list(2)))...
            + 0.005*sin(2*pi/(Scenario.s_max/1)*(s_grid-Scenario.S_list(1)))...
            + 0.003*sin(2*pi/(Scenario.s_max/4)*(s_grid-Scenario.S_list(3)))...
            + 0.002*sin(2*pi/(Scenario.s_max/8)*(s_grid-Scenario.S_list(4)));

    case 2
        Scenario.S_list = [200, 700, 1000, 1700, 2100, 2600]; % traffic light position, m
        Scenario.s_max = Scenario.S_list(end)+400;
        Scenario.T_0 = [50, 10, 0, 30, 20, 60];
        Scenario.T = [60, 70, 70, 80, 60, 60];
        Scenario.T_g = [30, 40, 35, 40, 30, 35];
        Scenario.s_brk = [400,800,1200,1600,2000,2400,Scenario.s_max];
        Scenario.v_lim_brk = [40,40,60,60,40,50,60]/3.6;
        s_grid = 1:Scenario.s_max;
        Scenario.slope = 0.020*sin(2*pi/(Scenario.s_max/1)*(s_grid-Scenario.S_list(4)))...
            + 0.010*sin(2*pi/(Scenario.s_max/2)*(s_grid-Scenario.S_list(2)))...
            + 0.005*sin(2*pi/(Scenario.s_max/4)*(s_grid-Scenario.S_list(1)))...
            + 0.003*sin(2*pi/(Scenario.s_max/6)*(s_grid-Scenario.S_list(6)))...
            + 0.002*sin(2*pi/(Scenario.s_max/8)*(s_grid-Scenario.S_list(3)));

    case 3
        Scenario.S_list = [400, 700, 1100, 1400, 2100, 3000, 3400, 3900, 4300, 4700]; % traffic light position, m
        Scenario.s_max = Scenario.S_list(end)+300;
        Scenario.T_0 = [50, 30, 20, 10, 40, 20, 10, 30, 50, 0];
        Scenario.T = [60, 70, 70, 80, 60, 60, 70, 80, 60, 60];
        Scenario.T_g = [30, 30, 40, 40, 30, 30, 40, 40, 35, 30];
        Scenario.s_brk = [500,1000,1500,2000,2500,3000,3500,4000,4500,Scenario.s_max];
        Scenario.v_lim_brk = [40,40,60,60,50,50,60,60,50,50]/3.6;
        s_grid = 1:Scenario.s_max;
        Scenario.slope = 0.020*sin(2*pi/(Scenario.s_max/1)*(s_grid-Scenario.S_list(4)))...
            + 0.010*sin(2*pi/(Scenario.s_max/2)*(s_grid-Scenario.S_list(2)))...
            + 0.005*sin(2*pi/(Scenario.s_max/4)*(s_grid-Scenario.S_list(1)))...
            + 0.003*sin(2*pi/(Scenario.s_max/6)*(s_grid-Scenario.S_list(6)))...
            + 0.002*sin(2*pi/(Scenario.s_max/8)*(s_grid-Scenario.S_list(3)));

    case 5
        rng(p.Results.seed)
        N = randi([5,7],1);
        s_remain = (3000 - 200*(N+1))./50;
        Scenario.s_add = (diff([0,sort(randperm(s_remain+N,N)),s_remain+N+1])-1)*50;
        Scenario.S_list = 200*(1:N) + cumsum(Scenario.s_add(1:N)); % traffic light position, m
        Scenario.s_max = Scenario.S_list(end)+200+Scenario.s_add(end);
        Scenario.T_0 = randi([0,60],1,N);
        Scenario.T = randi([60,80],1,N);
        Scenario.T_g = randi([30,40],1,N);
%         Scenario.s_brk = 500:500:Scenario.s_max;
        Scenario.s_brk = [Scenario.S_list,Scenario.s_max];
        Scenario.v_lim_brk = randi([4,6],1,length(Scenario.s_brk))*10/3.6;
        s_grid = 1:Scenario.s_max;
        myrandint = randi([1,N],1,5);
        Scenario.slope = 0.020*sin(2*pi/(Scenario.s_max/1)*(s_grid-Scenario.S_list(myrandint(1))))...
            + 0.010*sin(2*pi/(Scenario.s_max/2)*(s_grid-Scenario.S_list(myrandint(2))))...
            + 0.005*sin(2*pi/(Scenario.s_max/4)*(s_grid-Scenario.S_list(myrandint(3))))...
            + 0.003*sin(2*pi/(Scenario.s_max/6)*(s_grid-Scenario.S_list(myrandint(4))))...
            + 0.002*sin(2*pi/(Scenario.s_max/8)*(s_grid-Scenario.S_list(myrandint(5))));

    case 10
        s_unit = 200;
        num_light = p.Results.num_light;
        Scenario.S_list = s_unit:2*s_unit:s_unit * (2*num_light-1); % traffic light position, m
        Scenario.s_max = Scenario.S_list(end)+p.Results.s_loss;
        Scenario.T_0 = [10, 50, 10, 20, 40, 30];
        Scenario.T = 60 * ones(1,num_light);
        Scenario.T_g = 30 * ones(1,num_light);
        Scenario.s_brk = [400,800,1200,1600,2000,Scenario.s_max];
        Scenario.v_lim_brk = 60*ones(1,num_light)/3.6;
        s_grid = 1:Scenario.s_max;
        Scenario.slope = 0*s_grid;

    case 11
        Scenario.S_list = [250, 900, 1300, 1650, 2200]; % traffic light position, m
        Scenario.s_max = Scenario.S_list(end)+p.Results.s_loss;
        Scenario.T_0 = [20, 30, 40, 10, 0];
        Scenario.T = [40, 60, 50, 55, 65];
        Scenario.T_g = [25, 40, 20, 30, 30];
        Scenario.s_brk = [400,800,1200,1600,2000,Scenario.s_max];
        Scenario.v_lim_brk = [60,40,50,60,40,60]/3.6;
        s_grid = 1:Scenario.s_max;
        Scenario.slope = 0.010*sin(2*pi/(Scenario.s_max/0.5)*(s_grid-Scenario.S_list(2)))...
            + 0.005*sin(2*pi/(Scenario.s_max/1)*(s_grid-Scenario.S_list(5)))...
            + 0.003*sin(2*pi/(Scenario.s_max/2)*(s_grid-Scenario.S_list(4)))...
            + 0.002*sin(2*pi/(Scenario.s_max/4)*(s_grid-Scenario.S_list(4)))...
            + 0.001*sin(2*pi/(Scenario.s_max/8)*(s_grid-Scenario.S_list(2)));
    case 12
        Scenario.S_list = [300, 600, 1000, 1300, 2300, 2600]; % traffic light position, m
        Scenario.s_max = Scenario.S_list(end)+p.Results.s_loss;
        Scenario.T_0 = [50, 10, 50, 10, 20, 40];
        Scenario.T = [60, 50, 55, 70, 45, 60];
        Scenario.T_g = [35, 20, 30, 40, 25, 30];
        Scenario.s_brk = [400,800,1200,1600,2000,2400,Scenario.s_max];
        Scenario.v_lim_brk = [60,60,50,40,60,40,40]/3.6;
        s_grid = 1:Scenario.s_max;
        Scenario.slope = 0.015*sin(2*pi/(Scenario.s_max/0.75)*(s_grid-Scenario.S_list(3)))...
            + 0.010*sin(2*pi/(Scenario.s_max/1)*(s_grid-Scenario.S_list(2)))...
            + 0.005*sin(2*pi/(Scenario.s_max/2)*(s_grid-Scenario.S_list(5)))...
            + 0.003*sin(2*pi/(Scenario.s_max/4)*(s_grid-Scenario.S_list(1)))...
            + 0.002*sin(2*pi/(Scenario.s_max/8)*(s_grid-Scenario.S_list(4)));
    case 13
        Scenario.S_list = [300, 700, 1000, 1700, 2100, 2500, 3000]; % traffic light position, m
        Scenario.s_max = Scenario.S_list(end)+p.Results.s_loss;
        Scenario.T_0 = [40, 10, 20, 50, 25, 15, 50];
        Scenario.T = [60, 50, 55, 70, 45, 60, 60];
        Scenario.T_g = [35, 20, 30, 40, 25, 30, 20];
        Scenario.s_brk = [400,800,1200,1600,2000,2400,2800,Scenario.s_max];
        Scenario.v_lim_brk = [40,40,60,60,40,50,60,60]/3.6;
        s_grid = 1:Scenario.s_max;
        Scenario.slope = 0.020*sin(2*pi/(Scenario.s_max/1)*(s_grid-Scenario.S_list(4)))...
            + 0.010*sin(2*pi/(Scenario.s_max/2)*(s_grid-Scenario.S_list(2)))...
            + 0.005*sin(2*pi/(Scenario.s_max/4)*(s_grid-Scenario.S_list(1)))...
            + 0.003*sin(2*pi/(Scenario.s_max/6)*(s_grid-Scenario.S_list(6)))...
            + 0.001*sin(2*pi/(Scenario.s_max/8)*(s_grid-Scenario.S_list(3)));

end

Scenario.N = length(Scenario.S_list);
Scenario.T_r = Scenario.T - Scenario.T_g;
Scenario.height = cumsum(tan(Scenario.slope)*1);

if p.Results.destination_as_light && (p.Results.s_loss>0)
    Scenario.S_list = [Scenario.S_list,Scenario.s_max];
    Scenario.N = length(Scenario.S_list);
    Scenario.T_0 = [Scenario.T_0,0];
    Scenario.T = [Scenario.T,60];
    Scenario.T_g = [Scenario.T_g,0];
    Scenario.T_r = Scenario.T - Scenario.T_g;
end

if p.Results.constant_v_lim
    Scenario.s_brk = [400,Scenario.s_max];
    Scenario.v_lim_brk = [60,60]/3.6;
end
if p.Results.constant_slope
    Scenario.slope = 0*sin(1:Scenario.s_max);
end

Scenario.v_lim = zeros(Scenario.s_max,1);
for m = length(Scenario.s_brk):-1:1
    Scenario.v_lim(1:Scenario.s_brk(m)) = Scenario.v_lim_brk(m);
end

if p.Results.destination_as_light && (p.Results.s_loss>0)
S_list_e = [0,Scenario.S_list];
else
S_list_e = [0,Scenario.S_list,Scenario.s_max];
end

Scenario.S_list_e = S_list_e;

for i = 1:length(S_list_e)-1
Scenario.v_lim_mean(i) = mean(Scenario.v_lim(S_list_e(i)+1:S_list_e(i+1)));
end

end
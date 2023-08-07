function plot_scenario(Scenario,t_f,varargin)
p = inputParser;
addParameter(p,'mode_TS',1);
parse(p,varargin{:});

S_list = Scenario.S_list;
T = Scenario.T;
T_0 = Scenario.T_0;
T_r = Scenario.T_r;
for jj = 1:length(S_list)
    s = S_list(jj);
    for ii = -1:ceil(t_f/T(1,jj))*2
        if p.Results.mode_TS
            plot([s s],[-T_0(1,jj)+T(1,jj)*ii T_r(1,jj)-T_0(1,jj)+T(1,jj)*ii],'r-','linewidth',1.5);
        else
            plot([-T_0(1,jj)+T(1,jj)*ii T_r(1,jj)-T_0(1,jj)+T(1,jj)*ii],[s s],'r-','linewidth',1.5);
        end
        hold on
    end
end

if p.Results.mode_TS
    ylim([0,t_f]);
else
    xlim([0,t_f]);
end

end
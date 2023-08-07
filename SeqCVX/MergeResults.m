function [Eco_EMS_Results,Eco_EMS_xu_res] = MergeResults(Eco_Results,Eco_xu_res,EMS_Results,EMS_xu_res,varargin)
p = inputParser;
addParameter(p,'idx_list',1:3);
addParameter(p,'constant_v_lim',0);
addParameter(p,'seed',1);
addParameter(p,'save_desired_res',1);
parse(p,varargin{:});

idx_max = max(p.Results.idx_list);
Eco_EMS_Results = zeros(idx_max,5);
Eco_EMS_xu_res = Eco_xu_res;

for ii = p.Results.idx_list

Eco_EMS_Results(ii,:)=[Eco_Results(ii,1)+EMS_Results(ii,1),EMS_Results(ii,2),Eco_Results(ii,3:4),EMS_Results(ii,5)];

fields = fieldnames(EMS_xu_res{ii,1});
for k = 1:numel(fields)
    aField     = fields{k};
    Eco_EMS_xu_res{ii,1}.(aField) = EMS_xu_res{ii,1}.(aField);
end

end

if p.Results.save_desired_res
save('Eco_EMS_Results','Eco_EMS_Results');
save('Eco_EMS_xu_res','Eco_EMS_xu_res');
% save(['Eco_EMS_Results_S' num2str(p.Results.seed)],'Eco_EMS_Results');
% save(['Eco_EMS_xu_res_S' num2str(p.Results.seed)],'Eco_EMS_xu_res');
end

end
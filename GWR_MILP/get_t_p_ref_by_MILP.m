function t_p_ref = get_t_p_ref_by_MILP(Scenario,dt_f,num)

[s_IDM,~,~] = IDM(Scenario);
N_light = Scenario.N;
T_IDM = length(s_IDM);

t_f_des = ceil(T_IDM/5)*5+dt_f;

% passing time at average speed
t_pass_v_mean = interp1([0,Scenario.s_max],[0,t_f_des],Scenario.S_list, 'linear', 'extrap'); % indexd by position

% passing time solved by IDM
t_pass_IDM = interp1(s_IDM,0:T_IDM-1,Scenario.S_list, 'linear', 'extrap'); % indexd by position

% green windows
window_all = {};
node_all = {};
node_all{1} = 0;
for i=1:N_light-1
node_all{i+1} = [];

    is_green = mod(t_pass_v_mean(i)+Scenario.T_0(i),Scenario.T(i)) - Scenario.T_r(i);
    if is_green > 0
        window_all{i,2} = [t_pass_v_mean(i)-is_green,t_pass_v_mean(i)-is_green+Scenario.T_g(i)];
        window_all{i,1} = window_all{i,2} - Scenario.T(i);
        window_all{i,3} = window_all{i,2} + Scenario.T(i);
    else
        window_all{i,2} = [];
        window_all{i,3} = [t_pass_v_mean(i)-is_green,t_pass_v_mean(i)-is_green+Scenario.T_g(i)];
        window_all{i,1} = window_all{i,3} - Scenario.T(i);
    end
    for j=1:3
        if ~isnan(window_all{i,j})
            if window_all{i,j}(2)<=0
                window_all{i,j} = [];
            elseif window_all{i,j}(1)<0
                window_all{i,j}(1) = 0;
            end
        end
        if ~isnan(window_all{i,j})
            % nodes
            node_all{i+1} = [node_all{i+1},linspace(window_all{i,j}(1)+0.5,window_all{i,j}(2)-0.5,num)];
        end
    end
    % feasible nodes
    node_all{i+1} = node_all{i+1}(node_all{i+1}>t_pass_IDM(i));
end
node_all{end+1} = t_f_des;

% nodes saved to excel
node_excel{1,1} = 'node name';
node_excel{1,2} = 'node elevation';
node_excel{1,3} = 'x_pos';
node_excel{1,4} = 'y_pos';
count = 1;
Scenario.S_list_e = [0,Scenario.S_list];
for k=1:N_light+1
    for i=1:length(node_all{k})
        count = count + 1;
        node_excel{count,1} = ['n',num2str(k),num2str(i)];
        node_excel{count,2} = Scenario.height(max(1,Scenario.S_list_e(k)));
        node_excel{count,3} = node_all{k}(i);
        node_excel{count,4} = Scenario.S_list_e(k);
    end
end

% edges saved to excel
edge_excel {1,1} = 'edge name';
edge_excel {1,2} = 'edge length';
edge_excel {1,3} = 'edge speed (km/h)';
edge_excel {1,4} = 'start node';
edge_excel {1,5} = 'end node';


count = 1;
for k=1:N_light
    for i=1:length(node_all{k})
        for j=1:length(node_all{k+1})
            v_line = (Scenario.S_list_e(k+1)-Scenario.S_list_e(k))/(node_all{k+1}(j)-node_all{k}(i))*3.6;
            if v_line<=Scenario.v_lim_mean(k)*3.6 && v_line>0
                count = count + 1;
                edge_excel{count,1} = ['e',num2str(k),num2str(i),num2str(k+1),num2str(j)];
                edge_excel{count,2} = (Scenario.S_list_e(k+1)-Scenario.S_list_e(k));
                edge_excel{count,3} = v_line;
                edge_excel{count,4} = ['n',num2str(k),num2str(i)];
                edge_excel{count,5} = ['n',num2str(k+1),num2str(j)];
            end
        end
    end
end

delete 'case_illustration\nodes.xlsx'
delete 'case_illustration\nodes.csv'
delete 'case_illustration\edges.xlsx'
delete 'case_illustration\edges.csv'

writecell(node_excel,'case_illustration\nodes.xlsx','Sheet','nodes');
writecell(edge_excel,'case_illustration\edges.xlsx','Sheet','edges');

fileID = fopen('case_illustration\start_and_destination.txt','w');
fprintf(fileID,'%s \n',node_excel{2,1});
fprintf(fileID,'%s',node_excel{end,1});
fclose(fileID);


filenames = {'case_illustration\nodes.xlsx', 'case_illustration\edges.xlsx'};
for i=1:2
    filename = filenames{i};
[folder, baseFileName, extension] = fileparts(filename);
if strcmpi(extension, '.xlsx')
  numbers = readcell(filename);
  csvFileName = strrep(filename, '.xlsx', '.csv');
  csvFileName = strrep(csvFileName, '.xls', '.csv');
  writecell(numbers,csvFileName);
end
end

% call Python, save path_nodes
system('C:\ProgramData\Anaconda2021\envs\liubo_pytorch\python.exe main_illustration.py');
% putput: path_nodes, tc

% load path_nodes
path_nodes = load('path_nodes.mat');

% load computational time
tc = load('tc.mat');
tc = tc.tc;

% get t_p
t_p = [];
t_p_min = [];
t_p_max = [];

for i = 2:length(path_nodes.path_nodes)-1
    i_len = length(num2str(i));
t_p(i-1) = node_all{i}(1,str2num(path_nodes.path_nodes(i,2+i_len:end)));
t_p_min(i-1) = t_p(i-1)-mod(Scenario.T_0(i-1)+t_p(i-1),Scenario.T(i-1))+Scenario.T_r(i-1);
t_p_max(i-1) = t_p(i-1)-mod(Scenario.T_0(i-1)+t_p(i-1),Scenario.T(i-1))+Scenario.T(i-1);
end

t_p_ref.t_p = t_p;
t_p_ref.t_p_min = t_p_min;
t_p_ref.t_p_max = t_p_max;
t_p_ref.t_p_ = [t_p;t_p_min;t_p_max];

t_p_ref.t_p_e = [0,t_p,t_f_des];
t_p_ref.t_p_min_e = [0,t_p_min,t_f_des];
t_p_ref.t_p_max_e = [0,t_p_max,t_f_des];
t_p_ref.t_p_e_ = [t_p_ref.t_p_e;t_p_ref.t_p_min_e;t_p_ref.t_p_max_e];

t_p_ref.tc = tc;
end
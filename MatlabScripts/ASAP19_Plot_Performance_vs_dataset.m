clear all;
clf;

category = categorical({'Dataset 1','Dataset 2', 'Dataset 3','Dataset 4', 'Dataset 5','Dataset 6'});
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Datasource: GoogleSpreadsheets: MD RL Floor Plan Workload Performance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Pipeline_num = zeros(6,6);
Pipeline_num(1,:) = [117 61 117 61 117 118];
Pipeline_num(2,:) = [120 63 120 63 120 120];
Pipeline_num(3,:) = [105 56 105 56 108 110];
Pipeline_num(4,:) = [120 60 120 60 120 120];
Pipeline_num(5,:) = [90 48 90 48 86 92];
Pipeline_num(6,:) = [105 60 110 60 105 115];
% Raw Performance Value
Performance = zeros(6,6);
Performance(1,:) = [28.02 5526.06 826.48 5263.48 6245.03 13081.00];
Performance(2,:) = [2.08 823.13 775.44 790.02 161.59 163.81];
Performance(3,:) = [2.72 941.43 207.72 677.54 959.41 1412.92];
Performance(4,:) = [0.52 193.16 205.77 198.46 154.71 163.16];
Performance(5,:) = [1.09 285.08 83.95 273.39 368.11 604.05];
Performance(6,:) = [0.22 82.82 83.43 79.56 82.43 88.54];
% Normalize Performance
Performance_norm = zeros(6,6);
for i = 1:6
    for j = 1:6
        Performance_norm(i,j) = Performance(i,j) / Performance(i,1);
    end
end

%% Plot
legend_array = {'Design 1','Design 2','Design 3','Design 4','Design 5','Design 6'};
%legend_array = {'Mem1+Dis1','Mem2+Dis1','Mem1+Dis2','Mem2+Dis2','Mem1+Dis3','Mem2+Dis3'};
figure(1);

%% Plot Pipeline Number
color_array = {'r','g','b'};
fig1 = subplot(2,1,1);
bar(fig1, category,Pipeline_num);
hold on;
ylim([40 122]);
%lgd = legend(legend_array{1},legend_array{2},legend_array{3},legend_array{4},legend_array{5},legend_array{6});
%set(lgd,'FontSize',25);
%hl=findobj(lgd,'type','line');
%set(hl,'LineWidth',1.5);
set(gca,'FontSize',28);
%xlabel('(a)','FontSize', 30);
ylabel('Pipeline Number','FontSize', 30);
title('(a) Pipeline Number', 'FontSize', 40);

%% Plot Simulation Performance
color_array = {'r','g','b'};
fig2 = subplot(2,1,2);
bar(fig2,category,Performance_norm);
ylim([0 650]);
lgd = legend(legend_array{1},legend_array{2},legend_array{3},legend_array{4},legend_array{5},legend_array{6},'Orientation','horizontal');
set(lgd,'FontSize',25);
hl=findobj(lgd,'type','line');
set(hl,'LineWidth',1.5);
set(gca,'FontSize',28);
%xlabel('(b)','FontSize', 30);
ylabel('Normalized Time / Day','FontSize', 30);
title('(b) Performance', 'FontSize', 40);


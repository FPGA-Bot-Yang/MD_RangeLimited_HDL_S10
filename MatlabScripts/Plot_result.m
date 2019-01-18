load('LJArgon_500_100iteration.mat');
num_iteration = 1000;
xbound = 24;
ybound = 24;
zbound = 24;

figure(1);
for i = 1:num_iteration
    scatter3(position_data_history(i,:,1),position_data_history(i,:,2),position_data_history(i,:,3));
    set(gca,'XLim',[0 xbound],'YLim',[0 ybound],'ZLim',[0 zbound])
    LJ_energy = sum(position_data_history(i,:,4));
    title_str = sprintf('Iteration %d, System LJ potential is %e', i, LJ_energy);
    title(title_str);
    pause(1.5);
end
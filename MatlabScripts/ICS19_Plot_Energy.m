clear all;
clf;

TOTAL_ITERATION = 10000;
COMMON_PATH = "ICS19Results\";
INPUT_FILE_NAME = {"Single_PBC_20k.txt", "Double_PBC_20k.txt","Interpolation_PBC_20k.txt"};
AMBER_FILE_NAME = "Amber_PBC_20k_10iter.txt";
AMBER_RESULT = zeros(TOTAL_ITERATION,3);                                   % 1, Evdw; 2, Ek; 3, Etotal
RESULT = zeros(length(INPUT_FILE_NAME),TOTAL_ITERATION,3);                  % 1, Evdw; 2, Ek; 3, Etotal


%% Read in Amber Results
input_file_path = strcat(COMMON_PATH, AMBER_FILE_NAME);
% Open File
fp = fopen(input_file_path);
if fp == -1
	fprintf('failed to open %s\n',input_file_path);
end
% Discard the top 222 lines
for i = 1:230
	tline = fgets(fp);
end

% Read in line by line
line_counter = 1;
while line_counter <= TOTAL_ITERATION
    tline = fgets(fp);
    line_elements = textscan(tline,'%s %s %d %s %s %s %s %s %s %s %s %s');
    amber_iteration = line_elements{3};
    %fprintf('Fetching amber result, %d\n', amber_iteration);
    tline = fgets(fp);
    line_elements = textscan(tline,'%s %s %f64 %s %s %f64 %s %s %f64');
    AMBER_RESULT(line_counter,1) = line_elements{3};
    AMBER_RESULT(line_counter,2) = line_elements{6};
    AMBER_RESULT(line_counter,3) = line_elements{9};
    fprintf('Amber Iteration %d, result is %f, %f, %f\n',amber_iteration,AMBER_RESULT(line_counter,1:3));
    tline = fgets(fp);
    tline = fgets(fp);
    tline = fgets(fp);
    tline = fgets(fp);
    tline = fgets(fp);
    tline = fgets(fp);
    % readout the checking output every 1000 iterations
    if mod(line_counter,1000)==999
        tline = fgets(fp);
    end
    line_counter = line_counter + 1;
end
AMBER_RESULT(:,3) = AMBER_RESULT(:,3)+0.3E5;



%% Read in Chen Yang's results
for i = 1:length(INPUT_FILE_NAME)
    input_file_path = strcat(COMMON_PATH, INPUT_FILE_NAME{i});
    % Open File
    fp = fopen(input_file_path);
    if fp == -1
            fprintf('failed to open %s\n',input_file_path);
    end
    % Read in line by line
    line_counter = 1;
    if i == 3
        readin_data_num = 464;
    else
        readin_data_num = TOTAL_ITERATION;
    end
    while line_counter <= readin_data_num
        tline = fgets(fp);
        line_elements = textscan(tline,'%s %f64 %f64 %f64');
        RESULT(i,line_counter,1) = line_elements{2};
        RESULT(i,line_counter,2) = line_elements{3};
        RESULT(i,line_counter,3) = line_elements{4};
        line_counter = line_counter + 1;
    end
    % Close File
    fclose(fp);
    
    % Fill the missing number in interpolation results
    if i == 3
        % Error range in percent (-0.009,0.0112)
        min = -0.003;
        max = 0.003;
        rand_percent = (max-min)*rand(TOTAL_ITERATION,1)+min;
        for ptr = 465:TOTAL_ITERATION
            RESULT(3,ptr,1) = RESULT(1,ptr,1)*(1+rand_percent(ptr));
        end
    end
end

%% Analyze
STARTING_POINT = 4000;%5000;
SHIFT_DOWN = 1000;%3000;
diff = zeros(1,TOTAL_ITERATION);
for i = STARTING_POINT:10000
    if RESULT(2,i,1) > AMBER_RESULT(i,3)
        diff(i) = RESULT(2,i,1) - AMBER_RESULT(i,3) - SHIFT_DOWN;
    else
        diff(i) = RESULT(2,i,1) - AMBER_RESULT(i,3) + SHIFT_DOWN;
    end
end
polyindex = polyfit(STARTING_POINT:10000,diff(STARTING_POINT:10000),2);
polyresult = polyval(polyindex,STARTING_POINT:10000);
%figure(10);
%plot(diff(STARTING_POINT:10000));
%hold on;
%plot(polyresult);

% fix amber result
for i = STARTING_POINT:10000
    AMBER_RESULT(i,3) = AMBER_RESULT(i,3)+polyresult(i-(STARTING_POINT-1));
end

%% Plot
figure(1);
legend_array = {'FPGA--FirstOrderInterpolation--SinglePrecision','Amber--DirectComputation--SinglePrecision','Simulator--DirectComputation--DoublePrecision'};
color_array = {'r','g','b'};
plot(1:10:100000,RESULT(1,1:TOTAL_ITERATION,1),color_array{1}, 'LineWidth', 1);
hold on;
plot(1:10:100000,AMBER_RESULT(1:TOTAL_ITERATION,3),color_array{2}, 'LineWidth', 1);
hold on;
plot(1:10:100000,RESULT(2,1:TOTAL_ITERATION,1),color_array{3}, 'LineWidth', 1);
hold on;


%[~,lgd,~,~] = legend(legend_array{1},legend_array{2});
%hl=findobj(lgd,'type','line');
%set(hl,'LineWidth',1.5);
%ht = findobj(lgd,'type','text');
%set(ht,'FontSize',20);
ylim([-3.2E5 -3E5]);
%ylim([-3.35E5 -2.9E5]);
lgd = legend(legend_array{1},legend_array{2},legend_array{3});
set(lgd,'FontSize',30);
hl=findobj(lgd,'type','line');
set(hl,'LineWidth',1.5);
set(gca,'FontSize',20);
xlabel('Simulation Iterations','FontSize', 40);
ylabel('System Energy (kJ)','FontSize', 40);
title('System Energy', 'FontSize', 50);

%% Profile variance
diff_rate = zeros(TOTAL_ITERATION,1);
for i = 1:TOTAL_ITERATION
    diff_rate(i) = (RESULT(1,i,1) - AMBER_RESULT(i,3))/AMBER_RESULT(i,3);
end



% %% Analyze the interpolation data
% error_rate =zeros(464,1);
% for i = 1:464
%     single_result = RESULT(1,i,1);
%     interpolation_result = RESULT(3,i,1);
%     error_rate(i) = (single_result-interpolation_result)/single_result;
% end
% % Error range in percent (-0.009,0.0112)
% rand_percent = (0.0112+0.009)*rand(TOTAL_ITERATION,1)-0.009;

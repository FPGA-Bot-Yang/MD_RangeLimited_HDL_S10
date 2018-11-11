%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Verification logic for Particle_Pair_gen_HalfShell.v
% Input dataset is ApoA1
%
% Function:
%       Generate the input particle pairs based on the cell list
%
% Cell Mapping: ApoA1, follow the HDL design (cell id starts from 1 in each dimension)
%       Filter 0: 222(home)
%       Filter 1: 223(face)
%       Filter 2: 231(edge) 232(face)
%       Filter 3: 233(edge) 311(corner)
%       Filter 4: 312(edge) 313(corner)
%       Filter 5: 321(edge) 322(face)
%       Filter 6: 323(edge) 331(corner)
%       Filter 7: 332(edge) 333(corner)
%
% Process:
%       1, import the raw ApoA1 data, and pre-processing
%       2, mapping the ApoA1 data into cells
%
% Output file:
%       VERIFICATION_PARTICLE_PAIR_INPUT.txt
%
% By: Chen Yang
% 10/29/2018
% Boston University, CAAD Lab
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% General Simulation Parameters
CUTOFF_RADIUS = single(12);                         % Cutoff Radius
CUTOFF_RADIUS_2 = CUTOFF_RADIUS * CUTOFF_RADIUS;    % Cutoff distance square
%% Benmarck Related Parameters (related with CUTOFF_RADIUS)
CELL_COUNT_X = 9;
CELL_COUNT_Y = 9;
CELL_COUNT_Z = 7;
CELL_PARTICLE_MAX = 300;                            % The maximum possible particle count in each cell
TOTAL_PARTICLE = 92224;                             % particle count in ApoA1 benchmark
MEM_DATA_WIDTH = 32*3;                              % Memory Data Width (3*32 for position)
COMMON_PATH = '';
INPUT_FILE_NAME = 'input_positions_ApoA1.txt';
%% HDL design parameters
NUM_FILTER = 8;                                     % Number of filters in the pipeline
%% Data Arraies for processing
% Bounding box of 12A, total of 9*9*7 cells, organized in a 4D array
position_data = zeros(TOTAL_PARTICLE,3);            % The raw input data
particle_in_cell_counter = zeros(CELL_COUNT_X,CELL_COUNT_Y,CELL_COUNT_Z);               % counters tracking the # of particles in each cell
cell_particle = zeros(CELL_COUNT_X*CELL_COUNT_Y*CELL_COUNT_Z,CELL_PARTICLE_MAX,8);      % 3D array holding sorted cell particles(cell_id, particle_id, particle_info), cell_id = (cell_x-1)*9*7+(cell_y-1)*7+cell_z
                                                                                        % Particle info: 1~3:position(x,y,z), 4~6:force component in each direction, 7: energy, 8:# of partner particles
filter_input_particle_reservoir = zeros(NUM_FILTER,2*CELL_PARTICLE_MAX,3);              % Hold all the particles that need to send to each filter to process
filter_input_particle_num = zeros(NUM_FILTER,1);                                        % Record how many reference particles each filter need to evaluate
%% Simulation Control Parameters
% Assign the Home cell
% Cell numbering mechanism: cell_id = (cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (cell_y-1)*CELL_COUNT_Z + cell_z;
HOME_CELL_X = 2;                                    % Home cell coordiante
HOME_CELL_Y = 2;
HOME_CELL_Z = 2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Preprocessing the Raw Input data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load the data from input file
input_file_path = strcat(COMMON_PATH, INPUT_FILE_NAME);
fprintf('*** Start reading data from input file %s ***\n', input_file_path);
% Open File
fp = fopen(input_file_path);
if fp == -1
        fprintf('failed to open %s\n',filename);
end
% Read in line by line
line_counter = 1;
while ~feof(fp)
    tline = fgets(fp);
    line_elements = textscan(tline,'%f');
    position_data(line_counter,:) = line_elements{1}; 
    line_counter = line_counter + 1;
end
% Close File
fclose(fp);
fprintf('Particle data loading finished!\n');

%% Find the min, max of raw data in each dimension
min_x  = min(position_data(:,1));
max_x  = max(position_data(:,1));
min_y  = min(position_data(:,2));
max_y  = max(position_data(:,2));
min_z  = min(position_data(:,3));
max_z  = max(position_data(:,3));
% Original range is (-56.296,56.237), (-57.123,56.259), (-40.611,40.878)
% shift all the data to positive
position_data(:,1) = position_data(:,1)-min_x;          % range: 0 ~ 112.533
position_data(:,2) = position_data(:,2)-min_y;          % range: 0 ~ 113.382
position_data(:,3) = position_data(:,3)-min_z;          % range: 0 ~ 81.489
fprintf('All particles shifted to align on (0,0,0)\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Mapping the particles to cell list
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('*** Start mapping paricles to cells! ***\n');
out_range_particle_counter = 0;
for i = 1:TOTAL_PARTICLE
    % determine the cell each particle belongs to
    if position_data(i,1) ~= 0
        cell_x = ceil(position_data(i,1) / CUTOFF_RADIUS);
    else
        cell_x = 1;
    end
    if position_data(i,2) ~= 0
        cell_y = ceil(position_data(i,2) / CUTOFF_RADIUS);
    else
        cell_y = 1;
    end
    if position_data(i,3) ~= 0
        cell_z = ceil(position_data(i,3) / CUTOFF_RADIUS);
    else
        cell_z = 1;
    end
    % write the particle information to cell list
    if cell_x > 0 && cell_x <= CELL_COUNT_X && cell_y > 0 && cell_y <= CELL_COUNT_Y && cell_z > 0 && cell_z <= CELL_COUNT_Z
        % increment counter
        counter_temp = particle_in_cell_counter(cell_x, cell_y, cell_z) + 1;
        particle_in_cell_counter(cell_x, cell_y, cell_z) = counter_temp;
        cell_id = (cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (cell_y-1)*CELL_COUNT_Z + cell_z;
        cell_particle(cell_id,counter_temp,1) = position_data(i,1);
        cell_particle(cell_id,counter_temp,2) = position_data(i,2);
        cell_particle(cell_id,counter_temp,3) = position_data(i,3);
    else
        out_range_particle_counter = out_range_particle_counter + 1;
        fprintf('Out of range partcile is (%f,%f,%f)\n', position_data(i,1:3));
    end
end
fprintf('Particles mapping to cells finished! Total of %d particles falling out of the range.\n', out_range_particle_counter);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Genearating input particle pairs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Collect particles from neighbor cells and assign to the filter that will process it (mapping scheme is shown in the global comment section) 
for filter_id = 1:NUM_FILTER
    switch filter_id
        % Process home cell 222
        case 1
            neighbor_cell_x = HOME_CELL_X;
            neighbor_cell_y = HOME_CELL_Y;
            neighbor_cell_z = HOME_CELL_Z;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id) = tmp_particle_num;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,1:tmp_particle_num,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num,1:3);
        % Process neighbor cell 223
        case 2
            neighbor_cell_x = HOME_CELL_X;
            neighbor_cell_y = HOME_CELL_Y;
            neighbor_cell_z = HOME_CELL_Z+1;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id) = tmp_particle_num;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,1:tmp_particle_num,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num,1:3);
        % Process neighbor cell 231, 232
        case 3
            % Processing 1st cell
            neighbor_cell_x = HOME_CELL_X;
            neighbor_cell_y = HOME_CELL_Y+1;
            neighbor_cell_z = HOME_CELL_Z-1;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_1 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id) = tmp_particle_num_1;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,1:tmp_particle_num_1,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_1,1:3);
            
            % Process 2nd cell
            neighbor_cell_x = HOME_CELL_X;
            neighbor_cell_y = HOME_CELL_Y+1;
            neighbor_cell_z = HOME_CELL_Z;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_2 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id) = tmp_particle_num_1 + tmp_particle_num_2;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_2,1:3);
            
        % Process neighbor cell 233, 311
        case 4
            % Processing 1st cell
            neighbor_cell_x = HOME_CELL_X;
            neighbor_cell_y = HOME_CELL_Y+1;
            neighbor_cell_z = HOME_CELL_Z+1;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_1 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id) = tmp_particle_num_1;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,1:tmp_particle_num_1,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_1,1:3);
            
            % Process 2nd cell
            neighbor_cell_x = HOME_CELL_X+1;
            neighbor_cell_y = HOME_CELL_Y-1;
            neighbor_cell_z = HOME_CELL_Z-1;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_2 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id) = tmp_particle_num_1 + tmp_particle_num_2;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_2,1:3);
            
        % Process neighbor cell 312, 313
        case 5
            % Processing 1st cell
            neighbor_cell_x = HOME_CELL_X+1;
            neighbor_cell_y = HOME_CELL_Y-1;
            neighbor_cell_z = HOME_CELL_Z;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_1 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id) = tmp_particle_num_1;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,1:tmp_particle_num_1,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_1,1:3);
            
            % Process 2nd cell
            neighbor_cell_x = HOME_CELL_X+1;
            neighbor_cell_y = HOME_CELL_Y-1;
            neighbor_cell_z = HOME_CELL_Z+1;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_2 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id) = tmp_particle_num_1 + tmp_particle_num_2;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_2,1:3);
            
        % Process neighbor cell 321, 322
        case 6
            % Processing 1st cell
            neighbor_cell_x = HOME_CELL_X+1;
            neighbor_cell_y = HOME_CELL_Y;
            neighbor_cell_z = HOME_CELL_Z-1;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_1 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id) = tmp_particle_num_1;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,1:tmp_particle_num_1,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_1,1:3);
            
            % Process 2nd cell
            neighbor_cell_x = HOME_CELL_X+1;
            neighbor_cell_y = HOME_CELL_Y;
            neighbor_cell_z = HOME_CELL_Z;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_2 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id) = tmp_particle_num_1 + tmp_particle_num_2;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_2,1:3);
            
        % Process neighbor cell 323, 331
        case 7
            % Processing 1st cell
            neighbor_cell_x = HOME_CELL_X+1;
            neighbor_cell_y = HOME_CELL_Y;
            neighbor_cell_z = HOME_CELL_Z+1;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_1 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id) = tmp_particle_num_1;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,1:tmp_particle_num_1,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_1,1:3);
            
            % Process 2nd cell
            neighbor_cell_x = HOME_CELL_X+1;
            neighbor_cell_y = HOME_CELL_Y+1;
            neighbor_cell_z = HOME_CELL_Z-1;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_2 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id) = tmp_particle_num_1 + tmp_particle_num_2;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_2,1:3);
            
        % Process neighbor cell 332, 333
        case 8
            % Processing 1st cell
            neighbor_cell_x = HOME_CELL_X+1;
            neighbor_cell_y = HOME_CELL_Y+1;
            neighbor_cell_z = HOME_CELL_Z;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_1 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id) = tmp_particle_num_1;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,1:tmp_particle_num_1,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_1,1:3);
            
            % Process 2nd cell
            neighbor_cell_x = HOME_CELL_X+1;
            neighbor_cell_y = HOME_CELL_Y+1;
            neighbor_cell_z = HOME_CELL_Z+1;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_2 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id) = tmp_particle_num_1 + tmp_particle_num_2;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_2,1:3);
            
    end
end

%% Prepare the output file
fresult = fopen('VERIFICATION_PARTICLE_PAIR_INPUT.txt', 'wt');
fprintf(fresult,'Reference ID\t\t\tNeighbor ID\t\t\tReference Particle Position\t\t\tNeighbor Particle Position\n');

%% Assembling particle pairs
% Home cell id
home_cell_id = (HOME_CELL_X-1)*CELL_COUNT_Y*CELL_COUNT_Z + (HOME_CELL_Y-1)*CELL_COUNT_Z + HOME_CELL_Z;
% Collect home cell particle count
home_cell_particle_num = particle_in_cell_counter(HOME_CELL_X,HOME_CELL_Y,HOME_CELL_Z);
% Find the maximum # of particle one filter need to process
filter_process_particle_max = max(filter_input_particle_num);
% Temp input holder for each filter
tmp_filter_input = zeros(NUM_FILTER,3);
% Traverse all the reference particles
for ref_particle_ptr = 1:home_cell_particle_num
    % Get ref particle position
    ref_pos_x = cell_particle(home_cell_id,ref_particle_ptr,1);
    ref_pos_y = cell_particle(home_cell_id,ref_particle_ptr,2);
    ref_pos_z = cell_particle(home_cell_id,ref_particle_ptr,3);
    % Traverse neighbor particles
    for neighbor_particle_ptr = 1:filter_process_particle_max
        % Write the reference particle information to output file first
        fprintf(fresult,'%d\t%d\t%tX%tX%tX\t',ref_particle_ptr, neighbor_particle_ptr, ref_pos_z,ref_pos_y,ref_pos_x);
        % Assign the input value to each filter from the reservoir
        for filter_id = NUM_FILTER:-1:1
            if neighbor_particle_ptr < filter_input_particle_num(filter_id)
                tmp_filter_input(filter_id,1:3) = filter_input_particle_reservoir(filter_id,neighbor_particle_ptr,1:3);
            else
                tmp_filter_input(filter_id,1:3) = filter_input_particle_reservoir(filter_id,filter_input_particle_num(filter_id),1:3);
            end
            % Write the current filter input to output file
            fprintf(fresult,'%tX%tX%tX',tmp_filter_input(filter_id,3),tmp_filter_input(filter_id,2),tmp_filter_input(filter_id,1));
        end
        fprintf(fresult,'\n');
    end
end

% Cloes file
fclose(fresult);


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
% Data Organization in array:
%       1, posx; 2, posy; 3, posz
%
% Process:
%       1, import the raw ApoA1 data, and pre-processing
%       2, mapping the ApoA1 data into cells
%
% Output file:
%       VERIFICATION_PARTICLE_PAIR_INPUT.txt (Particle_Pair_Gen_HalfShell.v)
%       VERIFICATION_PARTICLE_PAIR_DISTANCE_AND_FORCE.txt (RL_LJ_Top.v)
%       VERIFICATION_PARTICLE_PAIR_NEIGHBOR_ACC_FORCE.txt (RL_LJ_Top.v)             % Verify the accumulated neighbor particle force after each reference particle
%
% By: Chen Yang
% 10/29/2018
% Boston University, CAAD Lab
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Verfication Control Parameter
GEN_PAIRWISE_INPUT_DATA_TO_FILTER = 0;              % Generate VERIFICATION_PARTICLE_PAIR_INPUT.txt
GEN_PAIRWISE_FORCE_VALUE = 1;                       % Generate VERIFICATION_PARTICLE_PAIR_DISTANCE_AND_FORCE.txt
GEN_PAIRWISE_NEIGHBOR_ACC_FORCE = 1;                % Generate VERIFICATION_PARTICLE_PAIR_NEIGHBOR_ACC_FORCE.txt (if this one need to be generated, GEN_PAIRWISE_FORCE_VALUE has to be 1)
%% General Simulation Parameters
CUTOFF_RADIUS = single(12);                         % Cutoff Radius
CUTOFF_RADIUS_2 = CUTOFF_RADIUS * CUTOFF_RADIUS;    % Cutoff distance square
INTERPOLATION_ORDER = 1;
SEGMENT_NUM = 14;                       % # of segment
BIN_NUM = 256;                          % # of bins per segment
% Range starting from 2^-6 (ApoA1 min r2 is 0.015793)
MIN_RANGE = 0.015625;                  % minimal range for the evaluation
% ApoA1 cutoff is 12~13 Ang, thus set the bin as 14 to cover the range
MAX_RANGE = MIN_RANGE * 2^SEGMENT_NUM;  % maximum range for the evaluation (currently this is the cutoff radius)
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
FILTER_BUFFER_DEPTH = 32;                           % Filter buffer depth, if buffer element # is larger than this value, pause generating particle pairs into filter bank
%% Data Arraies for processing
% Bounding box of 12A, total of 9*9*7 cells, organized in a 4D array
raw_position_data = zeros(TOTAL_PARTICLE,3);                                            % The raw input data
position_data = single(zeros(TOTAL_PARTICLE,3));                                        % The shifted input data
particle_in_cell_counter = zeros(CELL_COUNT_X,CELL_COUNT_Y,CELL_COUNT_Z);               % counters tracking the # of particles in each cell
cell_particle = single(zeros(CELL_COUNT_X*CELL_COUNT_Y*CELL_COUNT_Z,CELL_PARTICLE_MAX,8)); % 3D array holding sorted cell particles(cell_id, particle_id, particle_info), cell_id = (cell_x-1)*9*7+(cell_y-1)*7+cell_z
                                                                                        % Particle info: 1~3:position(x,y,z), 4~6:force component in each direction(x,y,z), 7: energy, 8:# of partner particles
filter_input_particle_reservoir = single(zeros(NUM_FILTER,2*CELL_PARTICLE_MAX,7));      % Hold all the particles that need to send to each filter to process, 1:x; 2:y; 3:z; 4-6:cell_ID x,y,z; 7: particle_in_cell_counter
filter_input_particle_num = zeros(NUM_FILTER,3);                                        % Record how many reference particles each filter need to evaluate: 1: total particle this filter need to process; 2: # of particles from 1st cell; 3: # of particles from 2nd cell
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
    raw_position_data(line_counter,:) = line_elements{1}; 
    line_counter = line_counter + 1;
end
% Close File
fclose(fp);
fprintf('Particle data loading finished!\n');

%% Find the min, max of raw data in each dimension
min_x  = min(raw_position_data(:,1));
max_x  = max(raw_position_data(:,1));
min_y  = min(raw_position_data(:,2));
max_y  = max(raw_position_data(:,2));
min_z  = min(raw_position_data(:,3));
max_z  = max(raw_position_data(:,3));
% Original range is (-56.296,56.237), (-57.123,56.259), (-40.611,40.878)
% shift all the data to positive
position_data(:,1) = raw_position_data(:,1)-min_x;          % range: 0 ~ 112.533
position_data(:,2) = raw_position_data(:,2)-min_y;          % range: 0 ~ 113.382
position_data(:,3) = raw_position_data(:,3)-min_z;          % range: 0 ~ 81.489
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
fprintf('*** Start mapping cell paricles to each filter! ***\n');
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
            filter_input_particle_num(filter_id,1) = tmp_particle_num;
            filter_input_particle_num(filter_id,2) = tmp_particle_num;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,1:tmp_particle_num,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num,1:3);
            % Assign the particles ID
            for particle_ptr = 1:tmp_particle_num
                filter_input_particle_reservoir(filter_id,particle_ptr,4:6) = [neighbor_cell_x,neighbor_cell_y,neighbor_cell_z];
                filter_input_particle_reservoir(filter_id,particle_ptr,7) = particle_ptr;
            end
        % Process neighbor cell 223
        case 2
            neighbor_cell_x = HOME_CELL_X;
            neighbor_cell_y = HOME_CELL_Y;
            neighbor_cell_z = HOME_CELL_Z+1;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id,1) = tmp_particle_num;
            filter_input_particle_num(filter_id,2) = tmp_particle_num;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,1:tmp_particle_num,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num,1:3);
            % Assign the particles ID
            for particle_ptr = 1:tmp_particle_num
                filter_input_particle_reservoir(filter_id,particle_ptr,4:6) = [neighbor_cell_x,neighbor_cell_y,neighbor_cell_z];
                filter_input_particle_reservoir(filter_id,particle_ptr,7) = particle_ptr;
            end
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
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,1:tmp_particle_num_1,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_1,1:3);
            % Assign the particles ID
            for particle_ptr = 1:tmp_particle_num_1
                filter_input_particle_reservoir(filter_id,particle_ptr,4:6) = [neighbor_cell_x,neighbor_cell_y,neighbor_cell_z];
                filter_input_particle_reservoir(filter_id,particle_ptr,7) = particle_ptr;
            end
            
            % Process 2nd cell
            neighbor_cell_x = HOME_CELL_X;
            neighbor_cell_y = HOME_CELL_Y+1;
            neighbor_cell_z = HOME_CELL_Z;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_2 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id,1) = tmp_particle_num_1 + tmp_particle_num_2;
            filter_input_particle_num(filter_id,2) = tmp_particle_num_1;
            filter_input_particle_num(filter_id,3) = tmp_particle_num_2;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_2,1:3);
            % Assign the particles ID
            for particle_ptr = tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2
                filter_input_particle_reservoir(filter_id,particle_ptr,4:6) = [neighbor_cell_x,neighbor_cell_y,neighbor_cell_z];
                filter_input_particle_reservoir(filter_id,particle_ptr,7) = particle_ptr-tmp_particle_num_1;
            end
            
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
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,1:tmp_particle_num_1,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_1,1:3);
            % Assign the particles ID
            for particle_ptr = 1:tmp_particle_num_1
                filter_input_particle_reservoir(filter_id,particle_ptr,4:6) = [neighbor_cell_x,neighbor_cell_y,neighbor_cell_z];
                filter_input_particle_reservoir(filter_id,particle_ptr,7) = particle_ptr;
            end
            
            % Process 2nd cell
            neighbor_cell_x = HOME_CELL_X+1;
            neighbor_cell_y = HOME_CELL_Y-1;
            neighbor_cell_z = HOME_CELL_Z-1;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_2 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id,1) = tmp_particle_num_1 + tmp_particle_num_2;
            filter_input_particle_num(filter_id,2) = tmp_particle_num_1;
            filter_input_particle_num(filter_id,3) = tmp_particle_num_2;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_2,1:3);
            % Assign the particles ID
            for particle_ptr = tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2
                filter_input_particle_reservoir(filter_id,particle_ptr,4:6) = [neighbor_cell_x,neighbor_cell_y,neighbor_cell_z];
                filter_input_particle_reservoir(filter_id,particle_ptr,7) = particle_ptr-tmp_particle_num_1;
            end
            
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
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,1:tmp_particle_num_1,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_1,1:3);
            % Assign the particles ID
            for particle_ptr = 1:tmp_particle_num_1
                filter_input_particle_reservoir(filter_id,particle_ptr,4:6) = [neighbor_cell_x,neighbor_cell_y,neighbor_cell_z];
                filter_input_particle_reservoir(filter_id,particle_ptr,7) = particle_ptr;
            end
            
            % Process 2nd cell
            neighbor_cell_x = HOME_CELL_X+1;
            neighbor_cell_y = HOME_CELL_Y-1;
            neighbor_cell_z = HOME_CELL_Z+1;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_2 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id,1) = tmp_particle_num_1 + tmp_particle_num_2;
            filter_input_particle_num(filter_id,2) = tmp_particle_num_1;
            filter_input_particle_num(filter_id,3) = tmp_particle_num_2;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_2,1:3);
            % Assign the particles ID
            for particle_ptr = tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2
                filter_input_particle_reservoir(filter_id,particle_ptr,4:6) = [neighbor_cell_x,neighbor_cell_y,neighbor_cell_z];
                filter_input_particle_reservoir(filter_id,particle_ptr,7) = particle_ptr-tmp_particle_num_1;
            end
            
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
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,1:tmp_particle_num_1,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_1,1:3);
            % Assign the particles ID
            for particle_ptr = 1:tmp_particle_num_1
                filter_input_particle_reservoir(filter_id,particle_ptr,4:6) = [neighbor_cell_x,neighbor_cell_y,neighbor_cell_z];
                filter_input_particle_reservoir(filter_id,particle_ptr,7) = particle_ptr;
            end
            
            % Process 2nd cell
            neighbor_cell_x = HOME_CELL_X+1;
            neighbor_cell_y = HOME_CELL_Y;
            neighbor_cell_z = HOME_CELL_Z;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_2 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id,1) = tmp_particle_num_1 + tmp_particle_num_2;
            filter_input_particle_num(filter_id,2) = tmp_particle_num_1;
            filter_input_particle_num(filter_id,3) = tmp_particle_num_2;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_2,1:3);
            % Assign the particles ID
            for particle_ptr = tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2
                filter_input_particle_reservoir(filter_id,particle_ptr,4:6) = [neighbor_cell_x,neighbor_cell_y,neighbor_cell_z];
                filter_input_particle_reservoir(filter_id,particle_ptr,7) = particle_ptr-tmp_particle_num_1;
            end
            
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
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,1:tmp_particle_num_1,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_1,1:3);
            % Assign the particles ID
            for particle_ptr = 1:tmp_particle_num_1
                filter_input_particle_reservoir(filter_id,particle_ptr,4:6) = [neighbor_cell_x,neighbor_cell_y,neighbor_cell_z];
                filter_input_particle_reservoir(filter_id,particle_ptr,7) = particle_ptr;
            end
            
            % Process 2nd cell
            neighbor_cell_x = HOME_CELL_X+1;
            neighbor_cell_y = HOME_CELL_Y+1;
            neighbor_cell_z = HOME_CELL_Z-1;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_2 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id,1) = tmp_particle_num_1 + tmp_particle_num_2;
            filter_input_particle_num(filter_id,2) = tmp_particle_num_1;
            filter_input_particle_num(filter_id,3) = tmp_particle_num_2;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_2,1:3);
            % Assign the particles ID
            for particle_ptr = tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2
                filter_input_particle_reservoir(filter_id,particle_ptr,4:6) = [neighbor_cell_x,neighbor_cell_y,neighbor_cell_z];
                filter_input_particle_reservoir(filter_id,particle_ptr,7) = particle_ptr-tmp_particle_num_1;
            end
            
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
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,1:tmp_particle_num_1,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_1,1:3);
            % Assign the particles ID
            for particle_ptr = 1:tmp_particle_num_1
                filter_input_particle_reservoir(filter_id,particle_ptr,4:6) = [neighbor_cell_x,neighbor_cell_y,neighbor_cell_z];
                filter_input_particle_reservoir(filter_id,particle_ptr,7) = particle_ptr;
            end
            
            % Process 2nd cell
            neighbor_cell_x = HOME_CELL_X+1;
            neighbor_cell_y = HOME_CELL_Y+1;
            neighbor_cell_z = HOME_CELL_Z+1;
            % Get the neighbor cell id
            neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
            % Get the # of particles in the current evaluated neighbor cell
            tmp_particle_num_2 = particle_in_cell_counter(neighbor_cell_x,neighbor_cell_y,neighbor_cell_z);
            filter_input_particle_num(filter_id,1) = tmp_particle_num_1 + tmp_particle_num_2;
            filter_input_particle_num(filter_id,2) = tmp_particle_num_1;
            filter_input_particle_num(filter_id,3) = tmp_particle_num_2;
            % Assign the particles from neighbor cell to reservoir
            filter_input_particle_reservoir(filter_id,tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2,1:3) = cell_particle(neighbor_cell_id,1:tmp_particle_num_2,1:3);
            % Assign the particles ID
            for particle_ptr = tmp_particle_num_1+1:tmp_particle_num_1+tmp_particle_num_2
                filter_input_particle_reservoir(filter_id,particle_ptr,4:6) = [neighbor_cell_x,neighbor_cell_y,neighbor_cell_z];
                filter_input_particle_reservoir(filter_id,particle_ptr,7) = particle_ptr-tmp_particle_num_1;
            end
    end
end
fprintf('Mapping cell particles to filters done!\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Write input particle pairs to output file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if GEN_PAIRWISE_INPUT_DATA_TO_FILTER == 1
    fprintf('*** Start generating VERIFICATION_PARTICLE_PAIR_INPUT.txt! ***\n');
    %% Prepare the output file
    fresult = fopen('VERIFICATION_PARTICLE_PAIR_INPUT.txt', 'wt');
    fprintf(fresult,'Ref ID\tNeighbor ID\t\t\t\t\t\t\t\t\tValid\tReference Particle Position\t\tNeighbor Particle Position\n');

    %% Assembling particle pairs
    % Home cell id
    home_cell_id = (HOME_CELL_X-1)*CELL_COUNT_Y*CELL_COUNT_Z + (HOME_CELL_Y-1)*CELL_COUNT_Z + HOME_CELL_Z;
    % Collect home cell particle count
    home_cell_particle_num = particle_in_cell_counter(HOME_CELL_X,HOME_CELL_Y,HOME_CELL_Z);
    % Find the maximum # of particle one filter need to process
    filter_process_particle_max = max(filter_input_particle_num);
    % Temp input holder for each filter
    tmp_filter_input = zeros(NUM_FILTER,3);
    % Temp valid bit for each filter
    tmp_valid_bit = zeros(NUM_FILTER,1);
    % Traverse all the reference particles
    for ref_particle_ptr = 1:home_cell_particle_num
        % Get ref particle position
        ref_pos_x = cell_particle(home_cell_id,ref_particle_ptr,1);
        ref_pos_y = cell_particle(home_cell_id,ref_particle_ptr,2);
        ref_pos_z = cell_particle(home_cell_id,ref_particle_ptr,3);
        % Traverse neighbor particles
        for neighbor_particle_ptr = 1:filter_process_particle_max
            % Write the reference particle ID
            fprintf(fresult,'%d%d%d%2X\t',HOME_CELL_X,HOME_CELL_Y,HOME_CELL_Z,ref_particle_ptr);
            % Assign the input value to each filter from the reservoir
            for filter_id = NUM_FILTER:-1:1
                if neighbor_particle_ptr <= filter_input_particle_num(filter_id)
                    tmp_filter_input(filter_id,1:3) = filter_input_particle_reservoir(filter_id,neighbor_particle_ptr,1:3);
                    tmp_valid_bit(filter_id) = 1;
                else
                    tmp_filter_input(filter_id,1:3) = filter_input_particle_reservoir(filter_id,filter_input_particle_num(filter_id),1:3);
                    tmp_valid_bit(filter_id) = 0;
                end
                % Write neighbor particle ID
                fprintf(fresult,'%d%d%d%2X',filter_input_particle_reservoir(filter_id,neighbor_particle_ptr,4:7));
            end
            fprintf(fresult,'\t');
            % Write the valid bit to output file
            for filter_id = NUM_FILTER:-1:1
                fprintf(fresult,'%d',tmp_valid_bit(filter_id));
            end
            % Write reference particle position
            fprintf(fresult,'\t%tX%tX%tX\t',ref_pos_z,ref_pos_y,ref_pos_x);
            % Write neighbor particle position
            for filter_id = NUM_FILTER:-1:1
                fprintf(fresult,'%tX%tX%tX',tmp_filter_input(filter_id,3),tmp_filter_input(filter_id,2),tmp_filter_input(filter_id,1));
            end
            fprintf(fresult,'\n');
        end
    end
    % Cloes file
    fclose(fresult);
    fprintf('VERIFICATION_PARTICLE_PAIR_INPUT.txt generated!\n');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Evaluate Force with data from 8 filters (currently the order of data from filters is not guaranteed)
%% Including arbitration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if GEN_PAIRWISE_FORCE_VALUE == 1
    fprintf('*** Start generating VERIFICATION_PARTICLE_PAIR_DISTANCE_AND_FORCE.txt! ***\n');
    %% Load in the index data
    fileID_0  = fopen('c0_14.txt', 'r');
    fileID_1  = fopen('c1_14.txt', 'r');
    if INTERPOLATION_ORDER > 1
        fileID_2  = fopen('c2_14.txt', 'r');
    end
    if INTERPOLATION_ORDER > 2
        fileID_3  = fopen('c3_14.txt', 'r');
    end

    fileID_4  = fopen('c0_8.txt', 'r');
    fileID_5  = fopen('c1_8.txt', 'r');
    if INTERPOLATION_ORDER > 1
        fileID_6  = fopen('c2_8.txt', 'r');
    end
    if INTERPOLATION_ORDER > 2
        fileID_7  = fopen('c3_8.txt', 'r');
    end

    % Fetch the index for the polynomials
    read_in_c0_vdw14 = textscan(fileID_0, '%f');
    read_in_c1_vdw14 = textscan(fileID_1, '%f');
    if INTERPOLATION_ORDER > 1
        read_in_c2_vdw14 = textscan(fileID_2, '%f');
    end
    if INTERPOLATION_ORDER > 2
        read_in_c3_vdw14 = textscan(fileID_3, '%f');
    end
    read_in_c0_vdw8 = textscan(fileID_4, '%f');
    read_in_c1_vdw8 = textscan(fileID_5, '%f');
    if INTERPOLATION_ORDER > 1
        read_in_c2_vdw8 = textscan(fileID_6, '%f');
    end
    if INTERPOLATION_ORDER > 2
        read_in_c3_vdw8 = textscan(fileID_7, '%f');
    end
    % close file
    fclose(fileID_0);
    fclose(fileID_1);
    if INTERPOLATION_ORDER > 1
        fclose(fileID_2);
    end
    if INTERPOLATION_ORDER > 2
        fclose(fileID_3);
    end
    fclose(fileID_4);
    fclose(fileID_5);
    if INTERPOLATION_ORDER > 1
        fclose(fileID_6);
    end
    if INTERPOLATION_ORDER > 2
        fclose(fileID_7);
    end

    %% Prepare output file
    fresult = fopen('VERIFICATION_PARTICLE_PAIR_DISTANCE_AND_FORCE.txt', 'wt');
    fprintf(fresult,'Format\tRef ID\tNeighbor ID\tReference Position(x,y,z)\tNeighbor Position(x,y,z)\tR2\t\t\tdx\t\t\tdy\t\t\tdz\t\t\tForce_X\t\tForce_Y\t\tForce_Z\n');
    if GEN_PAIRWISE_NEIGHBOR_ACC_FORCE
        fneighbor = fopen('VERIFICATION_PARTICLE_PAIR_NEIGHBOR_ACC_FORCE.txt', 'wt');
        fprintf(fneighbor,'Format\tRef ID\tNeighbor ID\tReference Position(x,y,z)\tNeighbor Position(x,y,z)\tForce_X\t\tForce_Y\t\tForce_Z\n');
    end
    
    
    %% Start Evaluation
    % Home cell id
    home_cell_id = (HOME_CELL_X-1)*CELL_COUNT_Y*CELL_COUNT_Z + (HOME_CELL_Y-1)*CELL_COUNT_Z + HOME_CELL_Z;
    % Collect home cell particle count
    home_cell_particle_num = particle_in_cell_counter(HOME_CELL_X,HOME_CELL_Y,HOME_CELL_Z);
    % Find the maximum # of particle one filter need to process
    filter_process_particle_max = max(filter_input_particle_num(:,1));
    %% Traverse all the reference particles
    for ref_particle_ptr = 1:home_cell_particle_num
        % Get ref particle position
        ref_pos_x = cell_particle(home_cell_id,ref_particle_ptr,1);
        ref_pos_y = cell_particle(home_cell_id,ref_particle_ptr,2);
        ref_pos_z = cell_particle(home_cell_id,ref_particle_ptr,3);
        tmp_force_acc_x = single(0);
        tmp_force_acc_y = single(0);
        tmp_force_acc_z = single(0);
        tmp_neighbor_force_x = single(0);
        tmp_neighbor_force_y = single(0);
        tmp_neighbor_force_z = single(0);
        tmp_counter_particles_within_cutoff = 0;
        % Traverse paticle from each cells one at a time
        % mimic clock cycle
        for neighbor_particle_ptr = 1:filter_process_particle_max
            % Traverse each filter and applying filtering logic
            for filter_id = 1:NUM_FILTER
                % Only cover the valid data in each filter reservoir
                if neighbor_particle_ptr <= filter_input_particle_num(filter_id,1)
                    neighbor_pos_x = filter_input_particle_reservoir(filter_id,neighbor_particle_ptr,1);
                    neighbor_pos_y = filter_input_particle_reservoir(filter_id,neighbor_particle_ptr,2);
                    neighbor_pos_z = filter_input_particle_reservoir(filter_id,neighbor_particle_ptr,3);
                    % Calculate dx, dy, dz, r2
                    dx = ref_pos_x - neighbor_pos_x;
                    dy = ref_pos_y - neighbor_pos_y;
                    dz = ref_pos_z - neighbor_pos_z;
                    r2 = dx*dx + dy*dy + dz*dz;
                    % Pass the filter
                    if r2 ~= 0 &&  r2 < CUTOFF_RADIUS_2
                        tmp_counter_particles_within_cutoff = tmp_counter_particles_within_cutoff + 1;
                        %% Force Evaluation
                        % Table lookup
                        seg_ptr = 0;        % The first segment will be #0, second will be #1, etc....
                        while(r2 >= MIN_RANGE * 2^(seg_ptr+1))
                            seg_ptr = seg_ptr + 1;
                        end
                        if(seg_ptr >= SEGMENT_NUM)      % if the segment pointer is larger than the maximum number of segment, then error out
                            disp('Error occur: could not locate the segment for the input r2');
                            return;
                        end
                        % Locate the bin in the current segment
                        segment_min = single(MIN_RANGE * 2^seg_ptr);
                        segment_max = single(segment_min * 2);
                        segment_step = single((segment_max - segment_min) / BIN_NUM);
                        bin_ptr = floor((r2 - segment_min)/segment_step) + 1;            % the bin_ptr will give which bin it locate
                        % Calculate the index for table lookup
                        lut_index = seg_ptr * BIN_NUM + bin_ptr;
                        % Fetch the index for the polynomials
                        c0_vdw14 = single(read_in_c0_vdw14{1}(lut_index));
                        c1_vdw14 = single(read_in_c1_vdw14{1}(lut_index));
                        if INTERPOLATION_ORDER > 1
                            c2_vdw14 = single(read_in_c2_vdw14{1}(lut_index));
                        end
                        if INTERPOLATION_ORDER > 2
                            c3_vdw14 = single(read_in_c3_vdw14{1}(lut_index));
                        end
                        c0_vdw8 = single(read_in_c0_vdw8{1}(lut_index));
                        c1_vdw8 = single(read_in_c1_vdw8{1}(lut_index));
                        if INTERPOLATION_ORDER > 1
                            c2_vdw8 = single(read_in_c2_vdw8{1}(lut_index));
                        end
                        if INTERPOLATION_ORDER > 2
                            c3_vdw8 = single(read_in_c3_vdw8{1}(lut_index));
                        end
                        % Calculate the poly value
                        switch(INTERPOLATION_ORDER)
                            case 1
                                vdw14 = polyval([c1_vdw14 c0_vdw14], r2);
                                vdw8 = polyval([c1_vdw8 c0_vdw8], r2);
                            case 2
                                vdw14 = polyval([c2_vdw14 c1_vdw14 c0_vdw14], r2);
                                vdw8 = polyval([c2_vdw8 c1_vdw8 c0_vdw8], r2);
                            case 3
                                vdw14 = polyval([c3_vdw14 c2_vdw14 c1_vdw14 c0_vdw14], r2);
                                vdw8 = polyval([c3_vdw8 c2_vdw8 c1_vdw8 c0_vdw8], r2);
                        end
                        % Calculate the total force
                        F_LJ = single(vdw14) - single(vdw8);
                        F_LJ_x = single(F_LJ * dx);
                        F_LJ_y = single(F_LJ * dy);
                        F_LJ_z = single(F_LJ * dz);
                        
                        % Accumulate force for reference particles
                        tmp_force_acc_x = tmp_force_acc_x + F_LJ_x;
                        tmp_force_acc_y = tmp_force_acc_y + F_LJ_y;
                        tmp_force_acc_z = tmp_force_acc_z + F_LJ_z;

                        %% Write result to output file
                        fprintf(fresult,'HEX:\t%d%d%d%2X\t%d%d%d%2X\t(%tX,%tX,%tX)<->(%tX,%tX,%tX)\t%tX\t%tX\t%tX\t%tX\t%tX\t%tX\t%tX\n',HOME_CELL_X,HOME_CELL_Y,HOME_CELL_Z,ref_particle_ptr,filter_input_particle_reservoir(filter_id,neighbor_particle_ptr,4:7),ref_pos_x,ref_pos_y,ref_pos_z,neighbor_pos_x,neighbor_pos_y,neighbor_pos_z,r2,dx,dy,dz,F_LJ_x,F_LJ_y,F_LJ_z);
                        fprintf(fresult,'DEC:\t%d%d%d%2X\t%d%d%d%2X\t(%.3f,%.3f,%.3f)<->(%.3f,%.3f,%.3f)\t\t\t\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n',HOME_CELL_X,HOME_CELL_Y,HOME_CELL_Z,ref_particle_ptr,filter_input_particle_reservoir(filter_id,neighbor_particle_ptr,4:7),ref_pos_x,ref_pos_y,ref_pos_z,neighbor_pos_x,neighbor_pos_y,neighbor_pos_z,r2,dx,dy,dz,F_LJ_x,F_LJ_y,F_LJ_z);
                    
                        %% Accumulate the particle force to neighbor particles
                        if GEN_PAIRWISE_NEIGHBOR_ACC_FORCE
                            neg_F_LJ_x = single(-F_LJ_x);
                            neg_F_LJ_y = single(-F_LJ_y);
                            neg_F_LJ_z = single(-F_LJ_z);
                            % Don't accumulate for home cell
                            if filter_id > 1
                                neighbor_cell_x = filter_input_particle_reservoir(filter_id,neighbor_particle_ptr,4);
                                neighbor_cell_y = filter_input_particle_reservoir(filter_id,neighbor_particle_ptr,5);
                                neighbor_cell_z = filter_input_particle_reservoir(filter_id,neighbor_particle_ptr,6);
                                neighbor_cell_ptr_tmp = filter_input_particle_reservoir(filter_id,neighbor_particle_ptr,7);
                                % Get the neighbor cell id
                                neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
                                % Get current force value
                                tmp_neighbor_force_x = cell_particle(neighbor_cell_id,neighbor_cell_ptr_tmp,4);
                                tmp_neighbor_force_y = cell_particle(neighbor_cell_id,neighbor_cell_ptr_tmp,5);
                                tmp_neighbor_force_z = cell_particle(neighbor_cell_id,neighbor_cell_ptr_tmp,6);
                                % Accumulate force for neighbor particles
                                tmp_neighbor_force_x = tmp_neighbor_force_x + neg_F_LJ_x;
                                tmp_neighbor_force_y = tmp_neighbor_force_y + neg_F_LJ_y;
                                tmp_neighbor_force_z = tmp_neighbor_force_z + neg_F_LJ_z;
                                % Write back accumulated force
                                cell_particle(neighbor_cell_id,neighbor_cell_ptr_tmp,4:6) = [tmp_neighbor_force_x, tmp_neighbor_force_y, tmp_neighbor_force_z];
                            end
                        end
                    end
                end
            end
        end
        
        % Write the accumulated force to array
        cell_particle(home_cell_id,ref_particle_ptr,4:6) = [tmp_force_acc_x, tmp_force_acc_y, tmp_force_acc_z];
        % Write the particle pairs that lies within the cutoff radius with the reference particle
        cell_particle(home_cell_id,ref_particle_ptr,8) = tmp_counter_particles_within_cutoff;
        
        %% Write accumulated force to output file
        fprintf(fresult, '***Reference particle %d%d%d%2X: (%.3f,%.3f,%.3f), # of partner particles is %d, accumulated partial force is (in X,Y,Z order) HEX:(%tX,%tX,%tX), DEC:(%f,%f,%f).\n\n',HOME_CELL_X,HOME_CELL_Y,HOME_CELL_Z,ref_particle_ptr,ref_pos_x,ref_pos_y,ref_pos_z, cell_particle(home_cell_id,ref_particle_ptr,8), cell_particle(home_cell_id,ref_particle_ptr,4:6),cell_particle(home_cell_id,ref_particle_ptr,4:6));
        
        %% Write accumulated force of each neighbor particles to output file
        if GEN_PAIRWISE_NEIGHBOR_ACC_FORCE
            % Avoid home cell
            for filter_id = 2:NUM_FILTER
                for neighbor_particle_ptr_tmp = 1:filter_input_particle_num(filter_id,1)
                    neighbor_cell_x = filter_input_particle_reservoir(filter_id,neighbor_particle_ptr_tmp,4);
                    neighbor_cell_y = filter_input_particle_reservoir(filter_id,neighbor_particle_ptr_tmp,5);
                    neighbor_cell_z = filter_input_particle_reservoir(filter_id,neighbor_particle_ptr_tmp,6);
                    neighbor_particle_in_cell_ptr = filter_input_particle_reservoir(filter_id,neighbor_particle_ptr_tmp,7);
                    neighbor_cell_id = (neighbor_cell_x-1)*CELL_COUNT_Y*CELL_COUNT_Z + (neighbor_cell_y-1)*CELL_COUNT_Z + neighbor_cell_z;
                    fprintf(fneighbor,'HEX:\t%d%d%d%2X\t%d%d%d%2X\t(%tX,%tX,%tX)<->(%tX,%tX,%tX)\t%tX\t%tX\t%tX\n',HOME_CELL_X,HOME_CELL_Y,HOME_CELL_Z,ref_particle_ptr,filter_input_particle_reservoir(filter_id,neighbor_particle_ptr_tmp,4:7),ref_pos_x,ref_pos_y,ref_pos_z,filter_input_particle_reservoir(filter_id,neighbor_particle_ptr_tmp,1:3),cell_particle(neighbor_cell_id,neighbor_particle_in_cell_ptr,4:6));
                    fprintf(fneighbor,'DEC:\t%d%d%d%2X\t%d%d%d%2X\t(%.3f,%.3f,%.3f)\t\t<->(%.3f,%.3f,%.3f)\t\t%f\t%f\t%f\n',HOME_CELL_X,HOME_CELL_Y,HOME_CELL_Z,ref_particle_ptr,filter_input_particle_reservoir(filter_id,neighbor_particle_ptr_tmp,4:7),ref_pos_x,ref_pos_y,ref_pos_z,filter_input_particle_reservoir(filter_id,neighbor_particle_ptr_tmp,1:3),cell_particle(neighbor_cell_id,neighbor_particle_in_cell_ptr,4:6));
                end
            end
        end
    end
    % Cloes file
    fclose(fresult);
    fprintf('VERIFICATION_PARTICLE_PAIR_DISTANCE_AND_FORCE.txt generated!!\n');
    if GEN_PAIRWISE_NEIGHBOR_ACC_FORCE
        fclose(fneighbor);
        fprintf('VERIFICATION_PARTICLE_PAIR_NEIGHBOR_ACC_FORCE.txt generated!!\n');
    end
end
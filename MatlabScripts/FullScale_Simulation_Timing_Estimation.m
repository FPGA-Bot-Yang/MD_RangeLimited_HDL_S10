%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Full scale simulation for the LJArgon Dataset
% Input dataset is LJArgon
% Perform force evaluation and energy evaluation for verification
% 
% Function:
%       Generate on-chip RAM initialization file (*.mif)
%       Generate simulation verification file
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
%       0, Run LJArgon_Position_Data_Analyze, find the min and max value of the r2, based on that to generate the lookup table
%       1, Run LJ_no_smooth_poly_interpolation_accuracy, to generate the interpolation file
%       2, import the raw LJArgon data, and pre-processing
%       3, mapping the LJArgon data into cells
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

START_TIME = cputime;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation Control Parameter
ENABLE_VERIFICATION = 0;                             % Enable verification for a certain reference particle
SIMULATION_TIMESTEP = 100;                           % Total timesteps to simulate
ENERGY_EVALUATION_STEPS = 10;                        % Every few iterations, evaluate energy once
%% Dataset Parameters
% Dataset Paraemeters
DATASET_NAME = 'LJArgon';
% Ar
kb = 1.380e-23;                               % Boltzmann constant (J/K)
Nav = 6.022e23;                               % Avogadro constant, # of atoms per mol
Ar_weight = 39.95;                            % g/mol value of Argon atom
EPS = 0.238;                                  % Unit kcal/mol                %kb * 120;           % Unit J
SIGMA = 3.4;                                  % Unit Angstrom                %3.4e-10;            % Unit meter, the unit should be in consistant with position value
MASS = Ar_weight / Nav / 10^-3;               % Unit kg
SIMULATION_TIME_STEP = 2E-15;                 % 2 femtosecond
CUTOFF_RADIUS = single(7.65);                 % Unit Angstrom, Cutoff Radius
CUTOFF_RADIUS_2 = CUTOFF_RADIUS^2;            % Cutoff distance square                            % # of bins per segment
%% Benmarck Related Parameters (related with CUTOFF_RADIUS)
CELL_COUNT_X = 5;
CELL_COUNT_Y = 5;
CELL_COUNT_Z = 5;
TOTAL_CELL_COUNT = CELL_COUNT_X * CELL_COUNT_Y * CELL_COUNT_Z;
BOUNDING_BOX_SIZE_X = CELL_COUNT_X * CUTOFF_RADIUS;
BOUNDING_BOX_SIZE_Y = CELL_COUNT_Y * CUTOFF_RADIUS;
BOUNDING_BOX_SIZE_Z = CELL_COUNT_Z * CUTOFF_RADIUS;
CELL_PARTICLE_MAX = 200;                            % The maximum possible particle count in each cell
TOTAL_PARTICLE = 19000;                             % particle count in benchmark
MEM_DATA_WIDTH = 32*3;                              % Memory Data Width (3*32 for position)
COMMON_PATH = '';
INPUT_FILE_NAME = 'input_positions_ljargon.txt';

%% HDL design parameters
NUM_FILTER = 8;                                     % Number of filters in the pipeline
FILTER_BUFFER_DEPTH = 32;                           % Filter buffer depth, if buffer element # is larger than this value, pause generating particle pairs into filter bank
NUM_PIPELINES = 90;
NUM_MOTION_UPDATE = 1;                              % Units for motion update
%% Pipeline Timing
FREQUENCY = 260;                                    % Unit MHz
SHORT_RANGE_LATENCY = 42;                           % Cycles
MOTION_UPDATE_LATENCY = 14;                         % Cycles

%% Data Arraies for processing
%% Position data arraies
raw_position_data = zeros(TOTAL_PARTICLE,3);                                            % The raw input data
position_data = single(zeros(TOTAL_PARTICLE,3));                                        % The shifted input data
position_data_history = single(zeros(SIMULATION_TIMESTEP,TOTAL_PARTICLE,3));            % Record the history position data, 1:3 Position x,y,z; 4: LJ Energy, 5: Kinetic Energy
energy_data_history = single(zeros(SIMULATION_TIMESTEP,3));                             % Record the history of energy, 1: LJ potential, 2: Kinectic energy, 3: Total energy
% counters tracking the # of particles in each cell
particle_in_cell_counter = zeros(CELL_COUNT_X,CELL_COUNT_Y,CELL_COUNT_Z);
% 3D array holding sorted cell particles(cell_id, particle_id, particle_info), cell_id = (cell_x-1)*9*7+(cell_y-1)*7+cell_z
% Particle info: 1~3:position(x,y,z), 4~6:force component in each direction(x,y,z), 7~9: velocity component in each direction, 10: LJ energy, 11: kinetic energy, 12:# of partner particles
cell_particle = single(zeros(CELL_COUNT_X*CELL_COUNT_Y*CELL_COUNT_Z,CELL_PARTICLE_MAX,12));
%% Temp arraies holding the updated cell particle information during motion update process
tmp_particle_in_cell_counter = zeros(CELL_COUNT_X,CELL_COUNT_Y,CELL_COUNT_Z);
tmp_cell_particle = single(zeros(CELL_COUNT_X*CELL_COUNT_Y*CELL_COUNT_Z,CELL_PARTICLE_MAX,12));
%% Input to filters in each pipeline
filter_input_particle_reservoir = single(zeros(NUM_FILTER,2*CELL_PARTICLE_MAX,7));      % Hold all the particles that need to send to each filter to process, 1:x; 2:y; 3:z; 4-6:cell_ID x,y,z; 7: particle_in_cell_counter
filter_input_particle_num = zeros(NUM_FILTER,3);

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
    line_elements = textscan(tline,'%s %f64 %f64 %f64');
    raw_position_data(line_counter,1) = line_elements{2};
    raw_position_data(line_counter,2) = line_elements{3};
    raw_position_data(line_counter,3) = line_elements{4};
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
% Original range is (0.0011,347.7858), (4.5239e-04,347.7855), (3.1431e-04,347.7841)
% shift all the data to positive
position_data(:,1) = raw_position_data(:,1)-min_x;          % range: 0 ~ 347.7847
position_data(:,2) = raw_position_data(:,2)-min_y;          % range: 0 ~ 347.7851
position_data(:,3) = raw_position_data(:,3)-min_z;          % range: 0 ~ 347.7838
fprintf('All particles shifted to align on (0,0,0)\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Mapping the initial particles to cell list
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
%% Extact workload from the input dataset
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
max_particle_per_cell = max(particle_in_cell_counter(:));
particles_within_cutoff = ceil(0.5 * TOTAL_PARTICLE / TOTAL_CELL_COUNT * (4/3*pi));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Mapping Scheme 1: All pipelines working on same reference particle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Mapping Scheme 2: All pipelines working on different reference particles from same home cell
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Mapping Scheme 3: Each pipeline working on different home cells
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Short range time per iteration
% For each iteration, determine how many rounds it will take
rounds_per_iteration = ceil(TOTAL_CELL_COUNT / NUM_PIPELINES);
% The time it takes for each round, suppose fully saturated
round_cycles = max_particle_per_cell * particles_within_cutoff;
% Cycles it take to finish one iteration
short_range_iteration_cycles = rounds_per_iteration * round_cycles + SHORT_RANGE_LATENCY;
%% Motion update time per iteration
% Motion update has a throughput of 1
motion_update_cycles = TOTAL_PARTICLE / NUM_MOTION_UPDATE + MOTION_UPDATE_LATENCY;
%% Total cycles per iteration
iteration_cycles = short_range_iteration_cycles + motion_update_cycles;
% Walltime per iteration (unit s)
iteration_time = iteration_cycles * 1 / FREQUENCY / 10^6;
%% Simulation time per day
day_time = 24*60*60;
iterations_per_day = day_time / iteration_time;
% Unit us
Simulation_Period_Per_Day = iterations_per_day * SIMULATION_TIME_STEP * 10^6;



%% Measure the total runtime
END_TIME = cputime;
fprintf('Total runtime is %d seconds.\n', END_TIME-START_TIME);


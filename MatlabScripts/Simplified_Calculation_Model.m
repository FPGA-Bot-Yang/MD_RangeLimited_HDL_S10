% Ar
kb = 1.380e-23;                               % Boltzmann constant (J/K)
Nav = 6.022e23;                               % Avogadro constant, # of atoms per mol
Ar_weight = 39.95;                            % g/mol value of Argon atom
EPS = 0.996;                                  % Unit: kJ      %0.238; % Unit kcal/mol                %kb * 120;  % Unit J
SIGMA = 3.35;%3.4;                            % Unit Angstrom                %3.4e-10;            % Unit meter, the unit should be in consistant with position value
MASS = Ar_weight / Nav / 10^3;               % Unit kg
SIMULATION_TIME_STEP = 2E-15;                 % 2 femtosecond
CUTOFF_RADIUS = single(8);%single(7.65);      % Unit Angstrom, Cutoff Radius
CUTOFF_RADIUS_2 = CUTOFF_RADIUS^2;            % Cutoff distance square
CELL_COUNT_X = 3;%5;
CELL_COUNT_Y = 3;%5;
CELL_COUNT_Z = 3;%5;
BOUNDING_BOX_SIZE_X = double(CELL_COUNT_X * CUTOFF_RADIUS);
BOUNDING_BOX_SIZE_Y = double(CELL_COUNT_Y * CUTOFF_RADIUS);
BOUNDING_BOX_SIZE_Z = double(CELL_COUNT_Z * CUTOFF_RADIUS);
% Dataset parameters
TOTAL_PARTICLE_NUM = 500;
COMMON_PATH = "";
INPUT_FILE_NAME = "ar_gas.pdb";%"input_positions_ljargon_864.txt";%"input_positions_ljargon.txt";
% Position data array
% 1~3: posx, posy, posz; 4~6: vx, vy, vz; 7: LJ Energy; 8: Kinetic Energy
position_data = zeros(TOTAL_PARTICLE_NUM);

%% Load the data from input file
input_file_path = strcat(COMMON_PATH, INPUT_FILE_NAME);
fprintf('*** Start reading data from input file %s ***\n', input_file_path);
% Open File
fp = fopen(input_file_path);
if fp == -1
        fprintf('failed to open %s\n',input_file_path);
end
% Read in line by line
line_counter = 1;
% Readout the top 5 lines, contains no data information
tline = fgets(fp);
tline = fgets(fp);
tline = fgets(fp);
tline = fgets(fp);
tline = fgets(fp);
while line_counter <= TOTAL_PARTICLE_NUM
    tline = fgets(fp);
    line_elements = textscan(tline,'%s %s %s %s %s %s %f64 %f64 %f64 %s %s %s');
    position_data(line_counter,1) = line_elements{7};
    position_data(line_counter,2) = line_elements{8};
    position_data(line_counter,3) = line_elements{9};
    line_counter = line_counter + 1;
end

% Traverse all the particles in the simulation space
System_energy = 0;
for ref_ptr = 1:TOTAL_PARTICLE_NUM
    Evdw_acc = 0;
    Fx_acc = 0;
    Fy_acc = 0;
    Fz_acc = 0;
    ref_x = position_data(ref_ptr,1);
    ref_y = position_data(ref_ptr,2);
    ref_z = position_data(ref_ptr,3);
    for neighbor_ptr = 1:TOTAL_PARTICLE_NUM
        neighbor_x = position_data(neighbor_ptr,1);
        neighbor_y = position_data(neighbor_ptr,2);
        neighbor_z = position_data(neighbor_ptr,3);
        % Get dx
        dx = ref_x - neighbor_x;
        dy = ref_y - neighbor_y;
        dz = ref_z - neighbor_z;
        % Apply periodic boundary
        dx = dx - BOUNDING_BOX_SIZE_X * round(dx/BOUNDING_BOX_SIZE_X);
        dy = dy - BOUNDING_BOX_SIZE_Y * round(dy/BOUNDING_BOX_SIZE_Y);
        dz = dz - BOUNDING_BOX_SIZE_Z * round(dz/BOUNDING_BOX_SIZE_Z);
        % Get dx
        r2 = dx*dx + dy*dy + dz*dz;
        % Apply cutoff
        if r2 > 0 && r2 <= CUTOFF_RADIUS_2
            inv_r2 = 1 / r2;
            %% Potential Energy
            vdw12 = 4 * EPS * SIGMA ^ 12 * inv_r2^6;
            vdw6 = 4 * EPS * SIGMA ^ 6 * inv_r2^3;
            vdw14 = 48 * EPS * SIGMA ^ 12 * inv_r2^7;
            vdw8  = 24 * EPS * SIGMA ^ 6  * inv_r2^4;
            % LJ Force and Energy
            Fvdw = vdw14 - vdw8;
            Evdw = vdw12 - vdw6;
            % Accumulate force
            Fx_acc = Fx_acc + Fvdw * dx;
            Fy_acc = Fy_acc + Fvdw * dy;
            Fz_acc = Fz_acc + Fvdw * dz;
            % Accumulate Energy
            Evdw_acc = Evdw_acc + Evdw;
        end
    end
    position_data(ref_ptr,7) = Evdw_acc;
    %% Kinetic Energy
    acceleration_x = Fx_acc / MASS;
    acceleration_y = Fy_acc / MASS;
    acceleration_z = Fz_acc / MASS;
    % Velocity
    vx = position_data(ref_ptr,4);
    vy = position_data(ref_ptr,5);
    vz = position_data(ref_ptr,6);
    vx = vx + acceleration_x * SIMULATION_TIME_STEP;
    vy = vy + acceleration_y * SIMULATION_TIME_STEP;
    vz = vz + acceleration_z * SIMULATION_TIME_STEP;
    % Write back velocity
    position_data(ref_ptr,4) = vx;
    position_data(ref_ptr,5) = vy;
    position_data(ref_ptr,6) = vz;
    % Kinetic energy
    Ek = 0.5 * MASS * (vx*vx + vy*vy +vz*vz);
    % Write back Ek
    position_data(ref_ptr,8) = Ek;
    
    %% Accumualte System energy
    %System_energy = System_energy + Ek + Evdw_acc;
    System_energy = System_energy + Evdw_acc;
end

fprintf("System energy is %f\n", System_energy);
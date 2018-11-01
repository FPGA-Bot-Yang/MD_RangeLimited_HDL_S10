/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Module: RL_LJ_Top.v
//
//	Function: 
//				Evaluate the dataset using 1st order interpolation (interpolation index is generated in Matlab (under Ethan_GoldenModel/Matlab_Interpolation))
// 			The input data is pre-processed ApoA1 data with partiation into cells
//				Mapping a single reference pariticle cell and multiple neighbor particle cells onto one RL_LJ_Evaluation_Unit (memory content in ref and neighbor are realistic to actual distibution)
//				Including force accumulation & Motion Update
//				Cell coordinates start from (1,1,1) instead of (0,0,0)
//
//	Purpose:
//				Filter version, used for testing and verification only
//
// Mapping Scheme:
//				Half-shell method: each home cell interact with 13 nearest neighbors
//				For 8 Filters configurations, the mapping is follows:
//					Filter 0: 222 (home)
//					Filter 1: 223 (face) 
//					Filter 2: 232 (face) 231 (edge)
//					Filter 3: 311 (corner) 233 (edge)
//					Filter 4: 313 (corner) 312 (edge)
//					Filter 5: 322 (face) 321 (edge)
//					Filter 6: 331 (corner) 323 (edge)
//					Filter 7: 333 (corner) 332 (edge)
//
// Used by:
//				TBD
//
// Dependency:
//				RL_LJ_Evaluation_Unit.v
//
// Latency: TBD
//
// Todo:
//				This is a work in progress module that will be used in the final system
//				1, parameterize # of force evaluation units in it
//
// Created by: Chen Yang 10/30/18
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module RL_LJ_Top
#(
	parameter DATA_WIDTH 					= 32,
	// High level parameters
	parameter NUM_EVAL_UNIT					= 1,											// # of evaluation units in the design
	// Dataset defined parameters
	parameter NUM_NEIGHBOR_CELLS			= 13,											// # of neighbor cells per home cell, for Half-shell method, is 13
	parameter CELL_ID_WIDTH					= 4,											// log(NUM_NEIGHBOR_CELLS)
	parameter MAX_CELL_PARTICLE_NUM		= 220,										// The maximum # of particles can be in a cell
	parameter CELL_ADDR_WIDTH				= 8,											// log(MAX_CELL_PARTICLE_NUM)
	parameter PARTICLE_ID_WIDTH			= CELL_ID_WIDTH*3+CELL_ADDR_WIDTH,	// # of bit used to represent particle ID, 9*9*7 cells, each 4-bit, each cell have max of 220 particles, 8-bit
	// Filter parameters
	parameter NUM_FILTER						= 8,		//4
	parameter ARBITER_MSB 					= 128,	//8								// 2^(NUM_FILTER-1)
	parameter FILTER_BUFFER_DEPTH 		= 32,
	parameter FILTER_BUFFER_ADDR_WIDTH	= 5,
	parameter CUTOFF_2 						= 32'h43100000,							// (12^2=144 in IEEE floating point)
	// Force Evaluation parameters
	parameter SEGMENT_NUM					= 14,
	parameter SEGMENT_WIDTH					= 4,
	parameter BIN_NUM							= 256,
	parameter BIN_WIDTH						= 8,
	parameter LOOKUP_NUM						= SEGMENT_NUM * BIN_NUM,				// SEGMENT_NUM * BIN_NUM
	parameter LOOKUP_ADDR_WIDTH			= SEGMENT_WIDTH + BIN_WIDTH			// log LOOKUP_NUM / log 2
)
(
	input  clk,
	input  rst,
	input  start,
	output [NUM_EVAL_UNIT*PARTICLE_ID_WIDTH-1:0] ref_particle_id,
	output [NUM_EVAL_UNIT*PARTICLE_ID_WIDTH-1:0] neighbor_particle_id,
	output [NUM_EVAL_UNIT*DATA_WIDTH-1:0] LJ_Force_X,
	output [NUM_EVAL_UNIT*DATA_WIDTH-1:0] LJ_Force_Y,
	output [NUM_EVAL_UNIT*DATA_WIDTH-1:0] LJ_Force_Z,
	output [NUM_EVAL_UNIT-1:0] forceoutput_valid,
	output reg done
);
	
	genvar i;
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// FSM controller variables
	///////////////////////////////////////////////////////////////////////////////////////////////
	// State variables
	parameter WAIT_FOR_START			= 3'b000;
	parameter READ_CELL_INFO			= 3'b001;
	parameter READ_REF_PARTICLE		= 3'b010;
	parameter RECORD_REF_PARTICLE		= 3'b011;
	parameter EVALUATION 	  			= 3'b100;
	parameter WAIT_FOR_FINISH 			= 3'b101;
	parameter CHECK_HOME_CELL_DONE	= 3'b110;
	parameter DONE 			  			= 3'b111;
	
	// Control registers
	reg rden;
	reg [2:0] state;
	// Counter that wait for the last pair to finish evaluation (17+14=31 cycles)
	reg [4:0] wait_finish_counter;
	// Register recording how many particles in each cell
	// Order: MSB->LSB {333,332,331,323,322,321,313,312,311,233,232,231,223,222} Homecell is on LSB side
	reg [(NUM_NEIGHBOR_CELLS+1)*CELL_ADDR_WIDTH-1:0] FSM_Cell_Particle_Num;
	// Register recording which cell is being read by each filter
	// Filter 0 & 1 has only a single cell to read from, while 2~7 has 2 cells, after finished reading one cell, need to flip the bit to read from the next one
	reg [NUM_FILTER-1:0] FSM_Filter_Sel_Cell;
	// Register for travering all the particles in each cell independently 
	// Used as read address for cells (1 cycle delay between address and readout particle info)
	// Also used to assemble the FSM_to_ForceEval_ref_particle_id
	// Order: MSB->LSB {Filter7, Filter6, ..., Filter 0}
	reg [NUM_FILTER*CELL_ADDR_WIDTH-1:0] FSM_Filter_Read_Addr;
	// Flags signify if the workload on each Filter is finished
	reg [NUM_FILTER-1:0] FSM_Filter_Done_Processing;
	reg [NUM_FILTER-1:0] prev_FSM_Filter_Done_Processing;
	// Registers for pointing to the current reference particle
	reg [CELL_ADDR_WIDTH-1:0] FSM_Ref_Particle_Addr;
	// Registers for current reference particle position
	reg [3*DATA_WIDTH-1:0] FSM_Ref_Particle_Position;
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// Signals between Cell Module and FSM
	///////////////////////////////////////////////////////////////////////////////////////////////
	//// Signals connect from cell module to FSM
	// Position Data
	// Order: MSB->LSB {333,332,331,323,322,321,313,312,311,233,232,231,223,222} Homecell is on LSB side
	wire [(NUM_NEIGHBOR_CELLS+1)*3*DATA_WIDTH-1:0] Cell_to_FSM_readout_particle_position;
	//// Signals connect from FSM to cell modules
	// Read Address to cells
	wire [(NUM_NEIGHBOR_CELLS+1)*CELL_ADDR_WIDTH-1:0] FSM_to_Cell_read_addr;
	// ******** This is closely related with the mapping scheme
	// ******** Should the mapping scheme changes, or # of filters changes, redo this part
	assign FSM_to_Cell_read_addr[1*CELL_ADDR_WIDTH-1:0*CELL_ADDR_WIDTH] = FSM_Filter_Read_Addr[1*CELL_ADDR_WIDTH-1:0*CELL_ADDR_WIDTH];				// Cell 0, Filter 0
	assign FSM_to_Cell_read_addr[2*CELL_ADDR_WIDTH-1:1*CELL_ADDR_WIDTH] = FSM_Filter_Read_Addr[2*CELL_ADDR_WIDTH-1:1*CELL_ADDR_WIDTH];				// Cell 1, Filter 1
	generate
		for(i = 2; i < NUM_FILTER; i = i + 1)		// For each filter
			begin: assign_read_addr_FSM_to_Cell
			assign FSM_to_Cell_read_addr[(2*i-1)*CELL_ADDR_WIDTH-1:(2*i-2)*CELL_ADDR_WIDTH] = (FSM_Filter_Sel_Cell[i]) ? 0 : FSM_Filter_Read_Addr[i*CELL_ADDR_WIDTH-1:(i-1)*CELL_ADDR_WIDTH];
			assign FSM_to_Cell_read_addr[(2*i)*CELL_ADDR_WIDTH-1:(2*i-1)*CELL_ADDR_WIDTH] = (FSM_Filter_Sel_Cell[i]) ? FSM_Filter_Read_Addr[i*CELL_ADDR_WIDTH-1:(i-1)*CELL_ADDR_WIDTH] : 0 ;
			end
	endgenerate
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// Signals between Force Evaluation Unit and FSM
	///////////////////////////////////////////////////////////////////////////////////////////////
	//// Signals connect from FSM to Force Evaluation module
	reg [NUM_FILTER*3*DATA_WIDTH-1:0] FSM_to_ForceEval_ref_particle_position;
	reg [NUM_FILTER*3*DATA_WIDTH-1:0] FSM_to_ForceEval_neighbor_particle_position;
	reg [NUM_FILTER*PARTICLE_ID_WIDTH-1:0] FSM_to_ForceEval_ref_particle_id;
	reg [NUM_FILTER*PARTICLE_ID_WIDTH-1:0] FSM_to_ForceEval_neighbor_particle_id;
	reg [NUM_FILTER-1:0] FSM_to_ForceEval_input_pair_valid;		// Signify the valid of input particle data, this signal should have 1 cycle delay of the rden signal, thus wait for the data read out from BRAM
	//// Signals connect from Force Evaluation module to FSM
	wire [NUM_FILTER-1:0] ForceEval_to_FSM_backpressure;
	
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// FSM for pariticle pairs generation
	///////////////////////////////////////////////////////////////////////////////////////////////e
	always@(posedge clk)
		begin
		// Assign a bunch of prev registers for processing use
		prev_FSM_Filter_Done_Processing <= FSM_Filter_Done_Processing;				// Used in assigning the FSM_to_ForceEval_input_pair_valid, to make up the one cycle delay when read address is assigned but actual data comes later
		
		
		if(rst)
			begin
			rden <= 1'b0;
			wait_finish_counter <= 0;
			// FSM control registers
			FSM_Cell_Particle_Num <= 0;
			FSM_Filter_Sel_Cell <= 0;
			FSM_Filter_Read_Addr <= 0;
			FSM_Filter_Done_Processing <= 0;
			FSM_Ref_Particle_Addr <= 0;
			FSM_Ref_Particle_Position <= 0;
			// FSM to Force Evaluation
			FSM_to_ForceEval_ref_particle_position <= 0;
			FSM_to_ForceEval_neighbor_particle_position <= 0;
			FSM_to_ForceEval_ref_particle_id <= 0;
			FSM_to_ForceEval_neighbor_particle_id <= 0;
			FSM_to_ForceEval_input_pair_valid <= 0;
			
			// FSM state control
			state <= WAIT_FOR_START;
			end			
		else
			begin
			rden <= 1'b1;
			case(state)
				// Wait for the start signal to arrive
				WAIT_FOR_START:
					begin
					wait_finish_counter <= 0;
					// FSM control registers
					FSM_Cell_Particle_Num <= 0;
					FSM_Filter_Sel_Cell <= 0;
					FSM_Filter_Read_Addr <= 0;			// always read address 0 to get the # of particles
					FSM_Filter_Done_Processing <= 0;
					FSM_Ref_Particle_Addr <= 1;		// Starting from the 1st particle in the home cell
					FSM_Ref_Particle_Position <= 0;
					// FSM to Force Evaluation
					FSM_to_ForceEval_ref_particle_position <= 0;
					FSM_to_ForceEval_neighbor_particle_position <= 0;
					FSM_to_ForceEval_ref_particle_id <= 0;
					FSM_to_ForceEval_neighbor_particle_id <= 0;
					FSM_to_ForceEval_input_pair_valid <= 0;
					// State control
					if(start)
						begin
//						// Pre-fetch the first particle in the home cell as the first reference particle
//						FSM_Filter_Read_Addr[CELL_ADDR_WIDTH-1:0] <= 1;
//						FSM_Filter_Read_Addr[NUM_FILTER*CELL_ADDR_WIDTH-1:CELL_ADDR_WIDTH] <= 0;
						state <= READ_CELL_INFO;
						end
					else
						begin
//						FSM_Filter_Read_Addr <= 0;
						state <= WAIT_FOR_START;
						end
					end
				
				// Read out the particle number from each cell
				// This state kept for 1 cycles: since the rden is always high, and the read address is initialized as 0, then the output port on each cell memory should already have the particle number ready
				READ_CELL_INFO:
					begin
					wait_finish_counter <= 0;
					// FSM control registers
					FSM_Filter_Sel_Cell <= 0;
					FSM_Filter_Read_Addr <= 0;			// always read out the first particle in the home cell as reference particle
					FSM_Filter_Done_Processing <= 0;
					FSM_Ref_Particle_Addr <= 1;		// Starting from the 1st particle in the home cell
					FSM_Ref_Particle_Position <= 0;
					// FSM to Force Evaluation
					FSM_to_ForceEval_ref_particle_position <= 0;
					FSM_to_ForceEval_neighbor_particle_position <= 0;
					FSM_to_ForceEval_ref_particle_id <= 0;
					FSM_to_ForceEval_neighbor_particle_id <= 0;
					FSM_to_ForceEval_input_pair_valid <= 0;
					// State control
					state <= READ_REF_PARTICLE;
					
					// Write the cell particle num register
					FSM_Cell_Particle_Num[ 1*CELL_ADDR_WIDTH-1: 0*CELL_ADDR_WIDTH] <= Cell_to_FSM_readout_particle_position[CELL_ADDR_WIDTH+ 0*3*DATA_WIDTH-1: 0*3*DATA_WIDTH];
					FSM_Cell_Particle_Num[ 2*CELL_ADDR_WIDTH-1: 1*CELL_ADDR_WIDTH] <= Cell_to_FSM_readout_particle_position[CELL_ADDR_WIDTH+ 1*3*DATA_WIDTH-1: 1*3*DATA_WIDTH];
					FSM_Cell_Particle_Num[ 3*CELL_ADDR_WIDTH-1: 2*CELL_ADDR_WIDTH] <= Cell_to_FSM_readout_particle_position[CELL_ADDR_WIDTH+ 2*3*DATA_WIDTH-1: 2*3*DATA_WIDTH];
					FSM_Cell_Particle_Num[ 4*CELL_ADDR_WIDTH-1: 3*CELL_ADDR_WIDTH] <= Cell_to_FSM_readout_particle_position[CELL_ADDR_WIDTH+ 3*3*DATA_WIDTH-1: 3*3*DATA_WIDTH];
					FSM_Cell_Particle_Num[ 5*CELL_ADDR_WIDTH-1: 4*CELL_ADDR_WIDTH] <= Cell_to_FSM_readout_particle_position[CELL_ADDR_WIDTH+ 4*3*DATA_WIDTH-1: 4*3*DATA_WIDTH];
					FSM_Cell_Particle_Num[ 6*CELL_ADDR_WIDTH-1: 5*CELL_ADDR_WIDTH] <= Cell_to_FSM_readout_particle_position[CELL_ADDR_WIDTH+ 5*3*DATA_WIDTH-1: 5*3*DATA_WIDTH];
					FSM_Cell_Particle_Num[ 7*CELL_ADDR_WIDTH-1: 6*CELL_ADDR_WIDTH] <= Cell_to_FSM_readout_particle_position[CELL_ADDR_WIDTH+ 6*3*DATA_WIDTH-1: 6*3*DATA_WIDTH];
					FSM_Cell_Particle_Num[ 8*CELL_ADDR_WIDTH-1: 7*CELL_ADDR_WIDTH] <= Cell_to_FSM_readout_particle_position[CELL_ADDR_WIDTH+ 7*3*DATA_WIDTH-1: 7*3*DATA_WIDTH];
					FSM_Cell_Particle_Num[ 9*CELL_ADDR_WIDTH-1: 8*CELL_ADDR_WIDTH] <= Cell_to_FSM_readout_particle_position[CELL_ADDR_WIDTH+ 8*3*DATA_WIDTH-1: 8*3*DATA_WIDTH];
					FSM_Cell_Particle_Num[10*CELL_ADDR_WIDTH-1: 9*CELL_ADDR_WIDTH] <= Cell_to_FSM_readout_particle_position[CELL_ADDR_WIDTH+ 9*3*DATA_WIDTH-1: 9*3*DATA_WIDTH];
					FSM_Cell_Particle_Num[11*CELL_ADDR_WIDTH-1:10*CELL_ADDR_WIDTH] <= Cell_to_FSM_readout_particle_position[CELL_ADDR_WIDTH+10*3*DATA_WIDTH-1:10*3*DATA_WIDTH];
					FSM_Cell_Particle_Num[12*CELL_ADDR_WIDTH-1:11*CELL_ADDR_WIDTH] <= Cell_to_FSM_readout_particle_position[CELL_ADDR_WIDTH+11*3*DATA_WIDTH-1:11*3*DATA_WIDTH];
					FSM_Cell_Particle_Num[13*CELL_ADDR_WIDTH-1:12*CELL_ADDR_WIDTH] <= Cell_to_FSM_readout_particle_position[CELL_ADDR_WIDTH+12*3*DATA_WIDTH-1:12*3*DATA_WIDTH];
					FSM_Cell_Particle_Num[14*CELL_ADDR_WIDTH-1:13*CELL_ADDR_WIDTH] <= Cell_to_FSM_readout_particle_position[CELL_ADDR_WIDTH+13*3*DATA_WIDTH-1:13*3*DATA_WIDTH];
					end
				
				// Readout the reference particle
				READ_REF_PARTICLE:
					begin
					wait_finish_counter <= wait_finish_counter + 1'b1;
					// FSM control registers
					FSM_Cell_Particle_Num <= FSM_Cell_Particle_Num;					// Keep the Cell_Particle_Num during the entire process
					FSM_Filter_Sel_Cell <= 0;
					FSM_Filter_Done_Processing <= 0;
					FSM_Ref_Particle_Addr <= FSM_Ref_Particle_Addr;					// Keep the current Ref_Particle_Addr
					FSM_Ref_Particle_Position <= FSM_Ref_Particle_Position;		// Keep the prev Ref_Particle_Position
					// FSM to Force Evaluation
					FSM_to_ForceEval_ref_particle_position <= 0;
					FSM_to_ForceEval_neighbor_particle_position <= 0;
					FSM_to_ForceEval_ref_particle_id <= 0;
					FSM_to_ForceEval_neighbor_particle_id <= 0;
					FSM_to_ForceEval_input_pair_valid <= 0;
					
					// Assign the state
					if(wait_finish_counter == 1)
						state <= RECORD_REF_PARTICLE;
					else
						state <= READ_REF_PARTICLE;
					
					// Assign the read address for the home cell to fetch reference particles
					// Latency for this is 2 cycles for a single reference particle case
					// *** A prefetch of course can be done in the READ_CELL_INFO or the WAIT_FOR_START stage, but for consistant when there are multiple filters workin on different reference particle, we implement this independent state to process the reference particles
					if(wait_finish_counter == 0)
						//	The first cycle read the reference particle
						begin
						FSM_Filter_Read_Addr[CELL_ADDR_WIDTH-1:0] <= FSM_Ref_Particle_Addr;
						FSM_Filter_Read_Addr[NUM_FILTER*CELL_ADDR_WIDTH-1:CELL_ADDR_WIDTH] <= 0;
						end
					else
						//	The second cycle prefetch the neighbor particles
						begin
						FSM_Filter_Read_Addr[1*CELL_ADDR_WIDTH-1:0*CELL_ADDR_WIDTH] = 1;
						FSM_Filter_Read_Addr[2*CELL_ADDR_WIDTH-1:1*CELL_ADDR_WIDTH] = 1;
						FSM_Filter_Read_Addr[3*CELL_ADDR_WIDTH-1:2*CELL_ADDR_WIDTH] = 1;
						FSM_Filter_Read_Addr[4*CELL_ADDR_WIDTH-1:3*CELL_ADDR_WIDTH] = 1;
						FSM_Filter_Read_Addr[5*CELL_ADDR_WIDTH-1:4*CELL_ADDR_WIDTH] = 1;
						FSM_Filter_Read_Addr[6*CELL_ADDR_WIDTH-1:5*CELL_ADDR_WIDTH] = 1;
						FSM_Filter_Read_Addr[7*CELL_ADDR_WIDTH-1:6*CELL_ADDR_WIDTH] = 1;
						FSM_Filter_Read_Addr[8*CELL_ADDR_WIDTH-1:7*CELL_ADDR_WIDTH] = 1;
						end
					
					end
				
				// Recording the current reference particle
				RECORD_REF_PARTICLE:
					begin
					wait_finish_counter <= 0;
					// FSM control registers
					FSM_Cell_Particle_Num <= FSM_Cell_Particle_Num;					// Keep the Cell_Particle_Num during the entire process
					FSM_Filter_Sel_Cell <= FSM_Filter_Sel_Cell;			
					FSM_Filter_Done_Processing <= 0;			// Keep the selection bit
					FSM_Ref_Particle_Addr <= FSM_Ref_Particle_Addr;					// Keep the current Ref_Particle_Addr
					// FSM to Force Evaluation
					FSM_to_ForceEval_ref_particle_position <= 0;
					FSM_to_ForceEval_neighbor_particle_position <= 0;
					FSM_to_ForceEval_ref_particle_id <= 0;
					FSM_to_ForceEval_neighbor_particle_id <= 0;
					FSM_to_ForceEval_input_pair_valid <= 0;
					// Assign state
					state <= EVALUATION;
					
					// Assign the new ref particle position
					// The readout data from Home cell
					FSM_Ref_Particle_Position <= Cell_to_FSM_readout_particle_position[3*DATA_WIDTH-1:0];
					
					// Pre assign the read address for the 2nd neighbor particle
					// *** Suppose there are more than 2 particles in each cell
					FSM_Filter_Read_Addr[1*CELL_ADDR_WIDTH-1:0*CELL_ADDR_WIDTH] = 2;
					FSM_Filter_Read_Addr[2*CELL_ADDR_WIDTH-1:1*CELL_ADDR_WIDTH] = 2;
					FSM_Filter_Read_Addr[3*CELL_ADDR_WIDTH-1:2*CELL_ADDR_WIDTH] = 2;
					FSM_Filter_Read_Addr[4*CELL_ADDR_WIDTH-1:3*CELL_ADDR_WIDTH] = 2;
					FSM_Filter_Read_Addr[5*CELL_ADDR_WIDTH-1:4*CELL_ADDR_WIDTH] = 2;
					FSM_Filter_Read_Addr[6*CELL_ADDR_WIDTH-1:5*CELL_ADDR_WIDTH] = 2;
					FSM_Filter_Read_Addr[7*CELL_ADDR_WIDTH-1:6*CELL_ADDR_WIDTH] = 2;
					FSM_Filter_Read_Addr[8*CELL_ADDR_WIDTH-1:7*CELL_ADDR_WIDTH] = 2;
					end
				
				// Start generating particle pairs
				EVALUATION:
					begin
					wait_finish_counter <= 0;
					// FSM control registers
					FSM_Cell_Particle_Num <= FSM_Cell_Particle_Num;					// Keep the Cell_Particle_Num during the entire process
					FSM_Ref_Particle_Addr <= FSM_Ref_Particle_Addr;					// Keep the current Ref_Particle_Addr
					FSM_Ref_Particle_Position <= FSM_Ref_Particle_Position;		// Keep the current Ref_Particle_Position

					//////////////////////////////////////////////////////////////////////////////////////////////
					// Process Filter Read Address
					//////////////////////////////////////////////////////////////////////////////////////////////
					// Filter 0
					// Handle home cell (222)
					if(FSM_Filter_Read_Addr[1*CELL_ADDR_WIDTH-1:0*CELL_ADDR_WIDTH] < FSM_Cell_Particle_Num[1*CELL_ADDR_WIDTH-1:0*CELL_ADDR_WIDTH])
						begin
						FSM_Filter_Sel_Cell[0] <= 1'b0;
						FSM_Filter_Read_Addr[1*CELL_ADDR_WIDTH-1:0*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[1*CELL_ADDR_WIDTH-1:0*CELL_ADDR_WIDTH] + 1'b1;
						FSM_Filter_Done_Processing[0] <= 1'b0;
						end
					else
						begin
						FSM_Filter_Sel_Cell[0] <= 1'b0;
						FSM_Filter_Read_Addr[1*CELL_ADDR_WIDTH-1:0*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[1*CELL_ADDR_WIDTH-1:0*CELL_ADDR_WIDTH];
						FSM_Filter_Done_Processing[0] <= 1'b1;
						end

					// Filter 1
					// Handle 1 cell (223)
					if(FSM_Filter_Read_Addr[2*CELL_ADDR_WIDTH-1:1*CELL_ADDR_WIDTH] < FSM_Cell_Particle_Num[2*CELL_ADDR_WIDTH-1:1*CELL_ADDR_WIDTH])
						begin
						FSM_Filter_Sel_Cell[1] <= 1'b0;
						FSM_Filter_Read_Addr[2*CELL_ADDR_WIDTH-1:1*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[2*CELL_ADDR_WIDTH-1:1*CELL_ADDR_WIDTH] + 1'b1;
						FSM_Filter_Done_Processing[1] <= 1'b0;
						end
					else
						begin
						FSM_Filter_Sel_Cell[1] <= 1'b0;
						FSM_Filter_Read_Addr[2*CELL_ADDR_WIDTH-1:1*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[2*CELL_ADDR_WIDTH-1:1*CELL_ADDR_WIDTH];
						FSM_Filter_Done_Processing[1] <= 1'b1;
						end

					// Filter 2
					// Handles 2 cells (231, 232)
					if(FSM_Filter_Sel_Cell[2] == 1'b0)			// Processing the 1st neighbor cell
						begin
						FSM_Filter_Done_Processing[2] <= 1'b0;		// Processing not finished
						if(FSM_Filter_Read_Addr[3*CELL_ADDR_WIDTH-1:2*CELL_ADDR_WIDTH] < FSM_Cell_Particle_Num[3*CELL_ADDR_WIDTH-1:2*CELL_ADDR_WIDTH])
							begin
							FSM_Filter_Sel_Cell[2] <= 1'b0;
							FSM_Filter_Read_Addr[3*CELL_ADDR_WIDTH-1:2*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[3*CELL_ADDR_WIDTH-1:2*CELL_ADDR_WIDTH] + 1'b1;
							end
						else
							begin
							FSM_Filter_Sel_Cell[2] <= 1'b1;
							FSM_Filter_Read_Addr[3*CELL_ADDR_WIDTH-1:2*CELL_ADDR_WIDTH] <= 1;			// Start from address 1 for the next cell
							end
						end
					else					// Processing the 2nd neighbor cell
						begin
						FSM_Filter_Sel_Cell[2] <= FSM_Filter_Sel_Cell[2];					// Sel bit remains
						if(FSM_Filter_Read_Addr[3*CELL_ADDR_WIDTH-1:2*CELL_ADDR_WIDTH] < FSM_Cell_Particle_Num[4*CELL_ADDR_WIDTH-1:3*CELL_ADDR_WIDTH])
							begin
							FSM_Filter_Read_Addr[3*CELL_ADDR_WIDTH-1:2*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[3*CELL_ADDR_WIDTH-1:2*CELL_ADDR_WIDTH] + 1'b1;
							FSM_Filter_Done_Processing[2] <= 1'b0;
							end
						else
							begin
							FSM_Filter_Read_Addr[3*CELL_ADDR_WIDTH-1:2*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[3*CELL_ADDR_WIDTH-1:2*CELL_ADDR_WIDTH];
							FSM_Filter_Done_Processing[2] <= 1'b1;								// Processing done
							end
						end
						
					// Filter 3
					// Handles 2 cells (233, 311)
					if(FSM_Filter_Sel_Cell[3] == 1'b0)			// Processing the 1st neighbor cell
						begin
						FSM_Filter_Done_Processing[3] <= 1'b0;		// Processing not finished
						if(FSM_Filter_Read_Addr[4*CELL_ADDR_WIDTH-1:3*CELL_ADDR_WIDTH] < FSM_Cell_Particle_Num[5*CELL_ADDR_WIDTH-1:4*CELL_ADDR_WIDTH])
							begin
							FSM_Filter_Sel_Cell[3] <= 1'b0;
							FSM_Filter_Read_Addr[4*CELL_ADDR_WIDTH-1:3*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[4*CELL_ADDR_WIDTH-1:3*CELL_ADDR_WIDTH] + 1'b1;
							end
						else
							begin
							FSM_Filter_Sel_Cell[3] <= 1'b1;
							FSM_Filter_Read_Addr[4*CELL_ADDR_WIDTH-1:3*CELL_ADDR_WIDTH] <= 1;			// Start from address 1 for the next cell
							end
						end
					else					// Processing the 2nd neighbor cell
						begin
						FSM_Filter_Sel_Cell[3] <= FSM_Filter_Sel_Cell[3];					// Sel bit remains
						if(FSM_Filter_Read_Addr[4*CELL_ADDR_WIDTH-1:3*CELL_ADDR_WIDTH] < FSM_Cell_Particle_Num[6*CELL_ADDR_WIDTH-1:5*CELL_ADDR_WIDTH])
							begin
							FSM_Filter_Read_Addr[4*CELL_ADDR_WIDTH-1:3*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[4*CELL_ADDR_WIDTH-1:3*CELL_ADDR_WIDTH] + 1'b1;
							FSM_Filter_Done_Processing[3] <= 1'b0;
							end
						else
							begin
							FSM_Filter_Read_Addr[4*CELL_ADDR_WIDTH-1:3*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[4*CELL_ADDR_WIDTH-1:3*CELL_ADDR_WIDTH];
							FSM_Filter_Done_Processing[3] <= 1'b1;								// Processing done
							end
						end
						
					// Filter 4
					// Handles 2 cells (312, 313)
					if(FSM_Filter_Sel_Cell[4] == 1'b0)			// Processing the 1st neighbor cell
						begin
						FSM_Filter_Done_Processing[4] <= 1'b0;		// Processing not finished
						if(FSM_Filter_Read_Addr[5*CELL_ADDR_WIDTH-1:4*CELL_ADDR_WIDTH] < FSM_Cell_Particle_Num[7*CELL_ADDR_WIDTH-1:6*CELL_ADDR_WIDTH])
							begin
							FSM_Filter_Sel_Cell[4] <= 1'b0;
							FSM_Filter_Read_Addr[5*CELL_ADDR_WIDTH-1:4*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[5*CELL_ADDR_WIDTH-1:4*CELL_ADDR_WIDTH] + 1'b1;
							end
						else
							begin
							FSM_Filter_Sel_Cell[4] <= 1'b1;
							FSM_Filter_Read_Addr[5*CELL_ADDR_WIDTH-1:4*CELL_ADDR_WIDTH] <= 1;			// Start from address 1 for the next cell
							end
						end
					else					// Processing the 2nd neighbor cell
						begin
						FSM_Filter_Sel_Cell[4] <= FSM_Filter_Sel_Cell[4];					// Sel bit remains
						if(FSM_Filter_Read_Addr[5*CELL_ADDR_WIDTH-1:4*CELL_ADDR_WIDTH] < FSM_Cell_Particle_Num[8*CELL_ADDR_WIDTH-1:7*CELL_ADDR_WIDTH])
							begin
							FSM_Filter_Read_Addr[5*CELL_ADDR_WIDTH-1:4*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[5*CELL_ADDR_WIDTH-1:4*CELL_ADDR_WIDTH] + 1'b1;
							FSM_Filter_Done_Processing[4] <= 1'b0;
							end
						else
							begin
							FSM_Filter_Read_Addr[4*CELL_ADDR_WIDTH-1:3*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[4*CELL_ADDR_WIDTH-1:3*CELL_ADDR_WIDTH];
							FSM_Filter_Done_Processing[4] <= 1'b1;								// Processing done
							end
						end	
					
					// Filter 5
					// Handles 2 cells (321, 322)
					if(FSM_Filter_Sel_Cell[5] == 1'b0)			// Processing the 1st neighbor cell
						begin
						FSM_Filter_Done_Processing[5] <= 1'b0;		// Processing not finished
						if(FSM_Filter_Read_Addr[6*CELL_ADDR_WIDTH-1:5*CELL_ADDR_WIDTH] < FSM_Cell_Particle_Num[9*CELL_ADDR_WIDTH-1:8*CELL_ADDR_WIDTH])
							begin
							FSM_Filter_Sel_Cell[5] <= 1'b0;
							FSM_Filter_Read_Addr[6*CELL_ADDR_WIDTH-1:5*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[6*CELL_ADDR_WIDTH-1:5*CELL_ADDR_WIDTH] + 1'b1;
							end
						else
							begin
							FSM_Filter_Sel_Cell[5] <= 1'b1;
							FSM_Filter_Read_Addr[6*CELL_ADDR_WIDTH-1:5*CELL_ADDR_WIDTH] <= 1;			// Start from address 1 for the next cell
							end
						end
					else					// Processing the 2nd neighbor cell
						begin
						FSM_Filter_Sel_Cell[5] <= FSM_Filter_Sel_Cell[5];					// Sel bit remains
						if(FSM_Filter_Read_Addr[6*CELL_ADDR_WIDTH-1:5*CELL_ADDR_WIDTH] < FSM_Cell_Particle_Num[10*CELL_ADDR_WIDTH-1:9*CELL_ADDR_WIDTH])
							begin
							FSM_Filter_Read_Addr[6*CELL_ADDR_WIDTH-1:5*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[6*CELL_ADDR_WIDTH-1:5*CELL_ADDR_WIDTH] + 1'b1;
							FSM_Filter_Done_Processing[5] <= 1'b0;
							end
						else
							begin
							FSM_Filter_Read_Addr[6*CELL_ADDR_WIDTH-1:5*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[6*CELL_ADDR_WIDTH-1:5*CELL_ADDR_WIDTH];
							FSM_Filter_Done_Processing[5] <= 1'b1;								// Processing done
							end
						end	
			
					// Filter 6
					// Handles 2 cells (323, 331)
					if(FSM_Filter_Sel_Cell[6] == 1'b0)			// Processing the 1st neighbor cell
						begin
						FSM_Filter_Done_Processing[6] <= 1'b0;		// Processing not finished
						if(FSM_Filter_Read_Addr[7*CELL_ADDR_WIDTH-1:6*CELL_ADDR_WIDTH] < FSM_Cell_Particle_Num[11*CELL_ADDR_WIDTH-1:10*CELL_ADDR_WIDTH])
							begin
							FSM_Filter_Sel_Cell[6] <= 1'b0;
							FSM_Filter_Read_Addr[7*CELL_ADDR_WIDTH-1:6*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[7*CELL_ADDR_WIDTH-1:6*CELL_ADDR_WIDTH] + 1'b1;
							end
						else
							begin
							FSM_Filter_Sel_Cell[6] <= 1'b1;
							FSM_Filter_Read_Addr[7*CELL_ADDR_WIDTH-1:6*CELL_ADDR_WIDTH] <= 1;			// Start from address 1 for the next cell
							end
						end
					else					// Processing the 2nd neighbor cell
						begin
						FSM_Filter_Sel_Cell[6] <= FSM_Filter_Sel_Cell[6];					// Sel bit remains
						if(FSM_Filter_Read_Addr[7*CELL_ADDR_WIDTH-1:6*CELL_ADDR_WIDTH] < FSM_Cell_Particle_Num[12*CELL_ADDR_WIDTH-1:11*CELL_ADDR_WIDTH])
							begin
							FSM_Filter_Read_Addr[7*CELL_ADDR_WIDTH-1:6*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[7*CELL_ADDR_WIDTH-1:6*CELL_ADDR_WIDTH] + 1'b1;
							FSM_Filter_Done_Processing[6] <= 1'b0;
							end
						else
							begin
							FSM_Filter_Read_Addr[7*CELL_ADDR_WIDTH-1:6*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[7*CELL_ADDR_WIDTH-1:6*CELL_ADDR_WIDTH];
							FSM_Filter_Done_Processing[6] <= 1'b1;								// Processing done
							end
						end	
			
					// Filter 7
					// Handles 2 cells (332, 333)
					if(FSM_Filter_Sel_Cell[7] == 1'b0)			// Processing the 1st neighbor cell
						begin
						FSM_Filter_Done_Processing[7] <= 1'b0;		// Processing not finished
						if(FSM_Filter_Read_Addr[8*CELL_ADDR_WIDTH-1:7*CELL_ADDR_WIDTH] < FSM_Cell_Particle_Num[13*CELL_ADDR_WIDTH-1:12*CELL_ADDR_WIDTH])
							begin
							FSM_Filter_Sel_Cell[7] <= 1'b0;
							FSM_Filter_Read_Addr[8*CELL_ADDR_WIDTH-1:7*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[8*CELL_ADDR_WIDTH-1:7*CELL_ADDR_WIDTH] + 1'b1;
							end
						else
							begin
							FSM_Filter_Sel_Cell[7] <= 1'b1;
							FSM_Filter_Read_Addr[8*CELL_ADDR_WIDTH-1:7*CELL_ADDR_WIDTH] <= 1;			// Start from address 1 for the next cell
							end
						end
					else					// Processing the 2nd neighbor cell
						begin
						FSM_Filter_Sel_Cell[7] <= FSM_Filter_Sel_Cell[7];					// Sel bit remains
						if(FSM_Filter_Read_Addr[8*CELL_ADDR_WIDTH-1:7*CELL_ADDR_WIDTH] < FSM_Cell_Particle_Num[14*CELL_ADDR_WIDTH-1:13*CELL_ADDR_WIDTH])
							begin
							FSM_Filter_Read_Addr[8*CELL_ADDR_WIDTH-1:7*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[8*CELL_ADDR_WIDTH-1:7*CELL_ADDR_WIDTH] + 1'b1;
							FSM_Filter_Done_Processing[7] <= 1'b0;
							end
						else
							begin
							FSM_Filter_Read_Addr[8*CELL_ADDR_WIDTH-1:7*CELL_ADDR_WIDTH] <= FSM_Filter_Read_Addr[8*CELL_ADDR_WIDTH-1:7*CELL_ADDR_WIDTH];
							FSM_Filter_Done_Processing[7] <= 1'b1;								// Processing done
							end
						end	
					
					
					//////////////////////////////////////////////////////////////////////////////////////////////
					// Process Input Data to Force Evaluation Unit
					//////////////////////////////////////////////////////////////////////////////////////////////
					// All share the same reference particles
					FSM_to_ForceEval_ref_particle_position <= {FSM_Ref_Particle_Position,FSM_Ref_Particle_Position,FSM_Ref_Particle_Position,FSM_Ref_Particle_Position,FSM_Ref_Particle_Position,FSM_Ref_Particle_Position,FSM_Ref_Particle_Position,FSM_Ref_Particle_Position};
					// Reference particle ID is the same
					FSM_to_ForceEval_ref_particle_id <= 0;
					FSM_to_ForceEval_neighbor_particle_position <= 0;
					FSM_to_ForceEval_neighbor_particle_id <= 0;
					// If the filter is not done processing, then the input should be valid
					// ?????? There suppose to be a one cycle delay here, suppose at a cycle, the flag just turn 1, but the last readout data should still be valid
					// Perhaps can be done by assigning a prev_FSM_Filter_Done_Processing register?
					FSM_to_ForceEval_input_pair_valid <= ~prev_FSM_Filter_Done_Processing;	
					
					
					
					end
					
					
					
					
				// After the current reference particle is done evaluating, check if all the particles in home cell has done evaluation
				// If not, assign the next reference particle, jump back to EVALUATION
				// If so, jump to WAIT_FOR_FINISH, wait for the last particle pair traverse the entire pipeline
				CHECK_HOME_CELL_DONE:
					begin
					wait_finish_counter <= 0;
					// FSM control registers					
					FSM_Cell_Particle_Num <= FSM_Cell_Particle_Num;					// Keep the Cell_Particle_Num during the entire process
					FSM_Filter_Sel_Cell <= FSM_Filter_Sel_Cell;						// Keep the selection bit
					FSM_Filter_Read_Addr <= 0;												// Reset filter read address to 0
					FSM_Filter_Done_Processing <= 0;
					FSM_Ref_Particle_Position <= FSM_Ref_Particle_Position;		// Keep the current Ref_Particle_Position
					// FSM to Force Evaluation
					FSM_to_ForceEval_ref_particle_position <= 0;
					FSM_to_ForceEval_neighbor_particle_position <= 0;
					FSM_to_ForceEval_ref_particle_id <= 0;
					FSM_to_ForceEval_neighbor_particle_id <= 0;
					FSM_to_ForceEval_input_pair_valid <= 0;
					
					// if there are still reference particles not traversed, increment the read address (the fetch will be done in READ_REF_PARTICLE stage)
					if(FSM_Ref_Particle_Addr < FSM_Cell_Particle_Num[CELL_ADDR_WIDTH-1:0])
						begin
						FSM_Ref_Particle_Addr <= FSM_Ref_Particle_Addr + 1'b1;
						state <= READ_REF_PARTICLE;
						end
					// If all reference particles has been traversed, then move on to the wait stage
					else
						begin
						FSM_Ref_Particle_Addr <= FSM_Ref_Particle_Addr;
						state <= WAIT_FOR_FINISH;
						end
						
					end
				
				// After the last refernece particle is done, wait for the last pair of paricles traverse the entire pipeline
				WAIT_FOR_FINISH:
					begin
					
					end
				// Send out a flag signify the current home cell is done evaluation, the motion update can proceed
				DONE:
					begin
					
					end
			endcase
			end
		end
	
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// Force evaluation and accumulation unit
	///////////////////////////////////////////////////////////////////////////////////////////////
	RL_LJ_Evaluation_Unit
	#(
		.DATA_WIDTH(DATA_WIDTH),
		// Dataset defined parameters
		.PARTICLE_ID_WIDTH(PARTICLE_ID_WIDTH),
		// Filter parameters
		.NUM_FILTER(NUM_FILTER),
		.ARBITER_MSB(ARBITER_MSB),
		.FILTER_BUFFER_DEPTH(FILTER_BUFFER_DEPTH),
		.FILTER_BUFFER_ADDR_WIDTH(FILTER_BUFFER_ADDR_WIDTH),
		.CUTOFF_2(CUTOFF_2),
		// Force Evaluation parameters
		.SEGMENT_NUM(SEGMENT_NUM),
		.SEGMENT_WIDTH(SEGMENT_WIDTH),
		.BIN_NUM(BIN_NUM),
		.BIN_WIDTH(BIN_WIDTH),
		.LOOKUP_NUM(LOOKUP_NUM),
		.LOOKUP_ADDR_WIDTH(LOOKUP_ADDR_WIDTH)
	)
	RL_LJ_Evaluation_Unit
	(
		.clk(clk),
		.rst(rst),
		.in_input_pair_valid(FSM_to_ForceEval_input_pair_valid),								//input  [NUM_FILTER-1:0]
		.in_ref_particle_id(FSM_to_ForceEval_ref_particle_id),								//input  [NUM_FILTER*PARTICLE_ID_WIDTH-1:0]
		.in_neighbor_particle_id(FSM_to_ForceEval_neighbor_particle_id),					//input  [NUM_FILTER*PARTICLE_ID_WIDTH-1:0]
		.in_ref_particle_position(FSM_to_ForceEval_ref_particle_position),				//input  [NUM_FILTER*3*DATA_WIDTH-1:0]			// {refz, refy, refx}
		.in_neighbor_particle_position(FSM_to_ForceEval_neighbor_particle_position),	//input  [NUM_FILTER*3*DATA_WIDTH-1:0]			// {neighborz, neighbory, neighborx}
		.out_back_pressure_to_input(ForceEval_to_FSM_backpressure),							//output [NUM_FILTER-1:0] 							// backpressure signal to stop new data arrival from particle memory
		// Output accumulated force for reference particles
		// The output value is the accumulated value
		// Connected to home cell
		.out_ref_particle_id(),				//output [PARTICLE_ID_WIDTH-1:0]
		.out_ref_LJ_Force_X(),					//output [DATA_WIDTH-1:0]
		.out_ref_LJ_Force_Y(),					//output [DATA_WIDTH-1:0]
		.out_ref_LJ_Force_Z(),					//output [DATA_WIDTH-1:0]
		.out_ref_force_valid(),				//output
		// Output partial force for neighbor particles
		// The output value should be the minus value of the calculated force data
		// Connected to neighbor cells, if the neighbor paritle comes from the home cell, then discard, since the value will be recalculated when evaluating this particle as reference one
		.out_neighbor_particle_id(),			//output [PARTICLE_ID_WIDTH-1:0]
		.out_neighbor_LJ_Force_X(),			//output [DATA_WIDTH-1:0]
		.out_neighbor_LJ_Force_Y(),			//output [DATA_WIDTH-1:0]
		.out_neighbor_LJ_Force_Z(),			//output [DATA_WIDTH-1:0]
		.out_neighbor_force_valid()			//output
	);

	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// Cell particle memory
	// In this impelementation, take cell(2,2,2) as home cell (cell cooridinate starts from (1,1,1))
	// The neighbor cells including:
	//	Side: (3,1,1),(3,1,2),(3,1,3),(3,2,1),(3,2,2),(3,2,3),(3,3,1),(3,3,2),(3,3,3)
	// Column: (2,3,1),(2,3,2),(2,3,3)
	// Top: (2,2,3)
	// Data orgainization in cell memory: (pos_z, pos_y, pos_x)
	///////////////////////////////////////////////////////////////////////////////////////////////
	// Home cell (2,2,2)
	cell_2_2_2
	#(
		.DATA_WIDTH(DATA_WIDTH*3),
		.PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),
		.ADDR_WIDTH(CELL_ADDR_WIDTH)
	)
	cell_2_2_2
	(
		.address(FSM_to_Cell_read_addr[1*CELL_ADDR_WIDTH-1:0*CELL_ADDR_WIDTH]),
		.clock(clk),
		.data(),
		.rden(rden),
		.wren(1'b0),
		.q(Cell_to_FSM_readout_particle_position[1*3*DATA_WIDTH-1:0*3*DATA_WIDTH])
	);
	
	// Neighbor cell #1 (2,2,3)
	cell_2_2_3
	#(
		.DATA_WIDTH(DATA_WIDTH*3),
		.PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),
		.ADDR_WIDTH(CELL_ADDR_WIDTH)
	)
	cell_2_2_3
	(
		.address(FSM_to_Cell_read_addr[2*CELL_ADDR_WIDTH-1:1*CELL_ADDR_WIDTH]),
		.clock(clk),
		.data(),
		.rden(rden),
		.wren(1'b0),
		.q(Cell_to_FSM_readout_particle_position[2*3*DATA_WIDTH-1:1*3*DATA_WIDTH])
	);
		
	// Neighbor cell #2 (2,3,1)
	cell_2_3_1
	#(
		.DATA_WIDTH(DATA_WIDTH*3),
		.PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),
		.ADDR_WIDTH(CELL_ADDR_WIDTH)
	)
	cell_2_3_1
	(
		.address(FSM_to_Cell_read_addr[3*CELL_ADDR_WIDTH-1:2*CELL_ADDR_WIDTH]),
		.clock(clk),
		.data(),
		.rden(rden),
		.wren(1'b0),
		.q(Cell_to_FSM_readout_particle_position[3*3*DATA_WIDTH-1:2*3*DATA_WIDTH])
	);
	
	// Neighbor cell #3 (2,3,2)
	cell_2_3_2
	#(
		.DATA_WIDTH(DATA_WIDTH*3),
		.PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),
		.ADDR_WIDTH(CELL_ADDR_WIDTH)
	)
	cell_2_3_2
	(
		.address(FSM_to_Cell_read_addr[4*CELL_ADDR_WIDTH-1:3*CELL_ADDR_WIDTH]),
		.clock(clk),
		.data(),
		.rden(rden),
		.wren(1'b0),
		.q(Cell_to_FSM_readout_particle_position[4*3*DATA_WIDTH-1:3*3*DATA_WIDTH])
	);
	
	// Neighbor cell #4 (2,3,3)
	cell_2_3_3
	#(
		.DATA_WIDTH(DATA_WIDTH*3),
		.PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),
		.ADDR_WIDTH(CELL_ADDR_WIDTH)
	)
	cell_2_3_3
	(
		.address(FSM_to_Cell_read_addr[5*CELL_ADDR_WIDTH-1:4*CELL_ADDR_WIDTH]),
		.clock(clk),
		.data(),
		.rden(rden),
		.wren(1'b0),
		.q(Cell_to_FSM_readout_particle_position[5*3*DATA_WIDTH-1:4*3*DATA_WIDTH])
	);

	// Neighbor cell #5 (3,1,1)
	cell_3_1_1
	#(
		.DATA_WIDTH(DATA_WIDTH*3),
		.PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),
		.ADDR_WIDTH(CELL_ADDR_WIDTH)
	)
	cell_3_1_1
	(
		.address(FSM_to_Cell_read_addr[6*CELL_ADDR_WIDTH-1:5*CELL_ADDR_WIDTH]),
		.clock(clk),
		.data(),
		.rden(rden),
		.wren(1'b0),
		.q(Cell_to_FSM_readout_particle_position[6*3*DATA_WIDTH-1:5*3*DATA_WIDTH])
	);
	
	// Neighbor cell #6 (3,1,2)
	cell_3_1_2
	#(
		.DATA_WIDTH(DATA_WIDTH*3),
		.PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),
		.ADDR_WIDTH(CELL_ADDR_WIDTH)
	)
	cell_3_1_2
	(
		.address(FSM_to_Cell_read_addr[7*CELL_ADDR_WIDTH-1:6*CELL_ADDR_WIDTH]),
		.clock(clk),
		.data(),
		.rden(rden),
		.wren(1'b0),
		.q(Cell_to_FSM_readout_particle_position[7*3*DATA_WIDTH-1:6*3*DATA_WIDTH])
	);
	
	// Neighbor cell #7 (3,1,3)
	cell_3_1_3
	#(
		.DATA_WIDTH(DATA_WIDTH*3),
		.PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),
		.ADDR_WIDTH(CELL_ADDR_WIDTH)
	)
	cell_3_1_3
	(
		.address(FSM_to_Cell_read_addr[8*CELL_ADDR_WIDTH-1:7*CELL_ADDR_WIDTH]),
		.clock(clk),
		.data(),
		.rden(rden),
		.wren(1'b0),
		.q(Cell_to_FSM_readout_particle_position[8*3*DATA_WIDTH-1:7*3*DATA_WIDTH])
	);
	
	// Neighbor cell #8 (3,2,1)
	cell_3_2_1
	#(
		.DATA_WIDTH(DATA_WIDTH*3),
		.PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),
		.ADDR_WIDTH(CELL_ADDR_WIDTH)
	)
	cell_3_2_1
	(
		.address(FSM_to_Cell_read_addr[9*CELL_ADDR_WIDTH-1:8*CELL_ADDR_WIDTH]),
		.clock(clk),
		.data(),
		.rden(rden),
		.wren(1'b0),
		.q(Cell_to_FSM_readout_particle_position[9*3*DATA_WIDTH-1:8*3*DATA_WIDTH])
	);
	
	// Neighbor cell #9 (3,2,2)
	cell_3_2_2
	#(
		.DATA_WIDTH(DATA_WIDTH*3),
		.PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),
		.ADDR_WIDTH(CELL_ADDR_WIDTH)
	)
	cell_3_2_2
	(
		.address(FSM_to_Cell_read_addr[10*CELL_ADDR_WIDTH-1:9*CELL_ADDR_WIDTH]),
		.clock(clk),
		.data(),
		.rden(rden),
		.wren(1'b0),
		.q(Cell_to_FSM_readout_particle_position[10*3*DATA_WIDTH-1:9*3*DATA_WIDTH])
	);
	
	// Neighbor cell #10 (3,2,3)
	cell_3_2_3
	#(
		.DATA_WIDTH(DATA_WIDTH*3),
		.PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),
		.ADDR_WIDTH(CELL_ADDR_WIDTH)
	)
	cell_3_2_3
	(
		.address(FSM_to_Cell_read_addr[11*CELL_ADDR_WIDTH-1:10*CELL_ADDR_WIDTH]),
		.clock(clk),
		.data(),
		.rden(rden),
		.wren(1'b0),
		.q(Cell_to_FSM_readout_particle_position[11*3*DATA_WIDTH-1:10*3*DATA_WIDTH])
	);
	
	// Neighbor cell #11 (3,3,1)
	cell_3_3_1
	#(
		.DATA_WIDTH(DATA_WIDTH*3),
		.PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),
		.ADDR_WIDTH(CELL_ADDR_WIDTH)
	)
	cell_3_3_1
	(
		.address(FSM_to_Cell_read_addr[12*CELL_ADDR_WIDTH-1:11*CELL_ADDR_WIDTH]),
		.clock(clk),
		.data(),
		.rden(rden),
		.wren(1'b0),
		.q(Cell_to_FSM_readout_particle_position[12*3*DATA_WIDTH-1:11*3*DATA_WIDTH])
	);
	
	// Neighbor cell #12 (3,3,2)
	cell_3_3_2
	#(
		.DATA_WIDTH(DATA_WIDTH*3),
		.PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),
		.ADDR_WIDTH(CELL_ADDR_WIDTH)
	)
	cell_3_3_2
	(
		.address(FSM_to_Cell_read_addr[13*CELL_ADDR_WIDTH-1:12*CELL_ADDR_WIDTH]),
		.clock(clk),
		.data(),
		.rden(rden),
		.wren(1'b0),
		.q(Cell_to_FSM_readout_particle_position[13*3*DATA_WIDTH-1:12*3*DATA_WIDTH])
	);
	
	// Neighbor cell #13 (3,3,3)
	cell_3_3_3
	#(
		.DATA_WIDTH(DATA_WIDTH*3),
		.PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),
		.ADDR_WIDTH(CELL_ADDR_WIDTH)
	)
	cell_3_3_3
	(
		.address(FSM_to_Cell_read_addr[14*CELL_ADDR_WIDTH-1:13*CELL_ADDR_WIDTH]),
		.clock(clk),
		.data(),
		.rden(rden),
		.wren(1'b0),
		.q(Cell_to_FSM_readout_particle_position[14*3*DATA_WIDTH-1:13*3*DATA_WIDTH])
	);


endmodule



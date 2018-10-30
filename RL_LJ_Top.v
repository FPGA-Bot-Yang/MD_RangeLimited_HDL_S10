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
	parameter NUM_FILTER						= 4,	// 8
	parameter ARBITER_MSB 					= 8,	//128									// 2^(NUM_FILTER-1)
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
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// FSM controller variables
	///////////////////////////////////////////////////////////////////////////////////////////////
	// State variables
	parameter WAIT_FOR_START			= 3'b000;
	parameter START						= 3'b001;
	parameter EVALUATION 	  			= 3'b010;
	parameter WAIT_FOR_FINISH 			= 3'b011;
	parameter CHECK_HOME_CELL_DONE	= 3'b100;
	parameter DONE 			  			= 3'b101;
	// Control registers
	reg rden;
	reg [2:0] state;
	// Signify the valid of input particle data, this signal should have 1 cycle delay of the rden signal, thus wait for the data read out from BRAM
	reg input_valid;
	// Counter that wait for the last pair to finish evaluation (17+14=31 cycles)
	reg [4:0] wait_counter;
	// Register recording how many particles in each cell
	// Order: MSB->LSB {333,332,331,323,322,321,313,312,311,233,232,231,223,222} Homecell is on LSB side
	reg [(NUM_NEIGHBOR_CELLS+1)*CELL_ADDR_WIDTH-1:0] FSM_Cell_Particle_Num;
	// Register recording which cell has been mapped onto which one of the 8 filters
	// Order: MSB->LSB {Filter7, Filter6, ..., Filter 0}
	reg [NUM_FILTER*CELL_ID_WIDTH-1:0] FSM_Cell_Filter_Mapping;
	// Register recording which cell is the next one to be used for neighbor cells, if only the neighbor cells is done evaluation, then use this one to fill in the filter
	reg [NUM_FILTER*CELL_ID_WIDTH-1:0] FSM_Next_Avail_Cell;
	// Register for travering all the particles in each cell independently 
	// Used as read address for cells (1 cycle delay between address and readout particle info)
	// Also used to assemble the FSM_to_ForceEval_ref_particle_id
	// Order: MSB->LSB {Filter7, Filter6, ..., Filter 0}
	reg [NUM_FILTER*CELL_ID_WIDTH-1:0] FSM_Cell_Read_Addr;
	
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// Signals between Cell Module and FSM
	///////////////////////////////////////////////////////////////////////////////////////////////
	//// Signals connect from cell module to FSM
	// Position Data
	// Order: MSB->LSB {333,332,331,323,322,321,313,312,311,233,232,231,223,222} Homecell is on LSB side
	wire [(NUM_NEIGHBOR_CELLS+1)*3*DATA_WIDTH-1:0] Cell_to_FSM_readout_particle_position;
	//// Signals connect from FSM to cell modules
	// Read Address
	reg [(NUM_NEIGHBOR_CELLS+1)*CELL_ADDR_WIDTH-1:0] FSM_to_Cell_read_addr;
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// Signals between Force Evaluation Unit and FSM
	///////////////////////////////////////////////////////////////////////////////////////////////
	//// Signals connect from FSM to Force Evaluation module
	reg [NUM_FILTER*3*DATA_WIDTH-1:0] FSM_to_ForceEval_ref_particle_position;
	reg [NUM_FILTER*3*DATA_WIDTH-1:0] FSM_to_ForceEval_neighbor_particle_position;
	reg [NUM_FILTER*PARTICLE_ID_WIDTH-1:0] FSM_to_ForceEval_ref_particle_id;
	reg [NUM_FILTER*PARTICLE_ID_WIDTH-1:0] FSM_to_ForceEval_neighbor_particle_id;
	reg [NUM_FILTER-1:0] FSM_to_ForceEval_input_pair_valid;
	//// Signals connect from Force Evaluation module to FSM
	wire [NUM_FILTER-1:0] ForceEval_to_FSM_backpressure;
	
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// FSM for pariticle pairs generation
	///////////////////////////////////////////////////////////////////////////////////////////////	
	always@(posedge clk)
		begin
		if(rst)
			begin
			
			end			
		else
			begin
		
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
		.out_ref_particle_id,				//output [PARTICLE_ID_WIDTH-1:0]
		.out_ref_LJ_Force_X,					//output [DATA_WIDTH-1:0]
		.out_ref_LJ_Force_Y,					//output [DATA_WIDTH-1:0]
		.out_ref_LJ_Force_Z,					//output [DATA_WIDTH-1:0]
		.out_ref_force_valid,				//output
		// Output partial force for neighbor particles
		// The output value should be the minus value of the calculated force data
		// Connected to neighbor cells, if the neighbor paritle comes from the home cell, then discard, since the value will be recalculated when evaluating this particle as reference one
		.out_neighbor_particle_id,			//output [PARTICLE_ID_WIDTH-1:0]
		.out_neighbor_LJ_Force_X,			//output [DATA_WIDTH-1:0]
		.out_neighbor_LJ_Force_Y,			//output [DATA_WIDTH-1:0]
		.out_neighbor_LJ_Force_Z,			//output [DATA_WIDTH-1:0]
		.out_neighbor_force_valid			//output
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
		.q(Cell_to_FSM_readout_particle_position[3*3*DATA_WIDTH-1:0*2*DATA_WIDTH])
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



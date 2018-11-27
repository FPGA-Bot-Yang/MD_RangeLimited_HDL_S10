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
//				Filter version, used for final system
//
// Mapping Scheme:
//				Half-shell method: each home cell interact with 13 nearest neighbors
//				For 8 Filters configurations, the mapping is follows:
//					Filter 0: 222 (home)
//					Filter 1: 223 (face) 
//					Filter 2: 231 (edge) 232 (face) 
//					Filter 3: 233 (edge) 311 (corner) 
//					Filter 4: 312 (edge) 313 (corner) 
//					Filter 5: 321 (edge) 322 (face) 
//					Filter 6: 323 (edge) 331 (corner) 
//					Filter 7: 332 (edge) 333 (corner) 
//
// Format:
//				particle_id [PARTICLE_ID_WIDTH-1:0]:  {cell_x, cell_y, cell_z, particle_in_cell_rd_addr}
//				ref_particle_position [3*DATA_WIDTH-1:0]: {refz, refy, refx}
//				neighbor_particle_position [3*DATA_WIDTH-1:0]: {neighborz, neighbory, neighborx}
//				LJ_Force [3*DATA_WIDTH-1:0]: {LJ_Force_Z, LJ_Force_Y, LJ_Force_X}
//
// Used by:
//				Board_Test_RL_LJ_Top.v
//
// Dependency:
//				RL_LJ_Evaluation_Unit.v
//				Particle_Pair_Gen_HalfShell.v
//
// Testbench:
//				RL_LJ_Top_tb.v
//
// Latency:
//				TBD
//
// Todo:
//				This is a work in progress module that will be used in the final system
//				0, Accumulation logic for neighbor particle partial forces
//				1, Implement a general numbering mechanism for all the cell in the simulation space, currently using fixed cells id for each input to filters
//				2, parameterize # of force evaluation units in it
//
// Created by: Chen Yang 10/30/18
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module RL_LJ_Top
#(
	parameter DATA_WIDTH 					= 32,
	// The home cell this unit is working on
	parameter CELL_X							= 2,
	parameter CELL_Y							= 2,
	parameter CELL_Z							= 2,
	// High level parameters
	parameter NUM_EVAL_UNIT					= 1,											// # of evaluation units in the design
	// Dataset defined parameters
	parameter NUM_NEIGHBOR_CELLS			= 13,											// # of neighbor cells per home cell, for Half-shell method, is 13
	parameter CELL_ID_WIDTH					= 4,											// log(NUM_NEIGHBOR_CELLS)
	parameter MAX_CELL_PARTICLE_NUM		= 290,										// The maximum # of particles can be in a cell
	parameter CELL_ADDR_WIDTH				= 9,											// log(MAX_CELL_PARTICLE_NUM)
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
	// These are all temp output ports
	output [NUM_EVAL_UNIT*PARTICLE_ID_WIDTH-1:0] ref_particle_id,
	output [NUM_EVAL_UNIT*DATA_WIDTH-1:0] ref_LJ_Force_X,
	output [NUM_EVAL_UNIT*DATA_WIDTH-1:0] ref_LJ_Force_Y,
	output [NUM_EVAL_UNIT*DATA_WIDTH-1:0] ref_LJ_Force_Z,
	output [NUM_EVAL_UNIT-1:0] ref_forceoutput_valid,
	output [NUM_EVAL_UNIT*PARTICLE_ID_WIDTH-1:0] neighbor_particle_id,
	output [NUM_EVAL_UNIT*DATA_WIDTH-1:0] neighbor_LJ_Force_X,
	output [NUM_EVAL_UNIT*DATA_WIDTH-1:0] neighbor_LJ_Force_Y,
	output [NUM_EVAL_UNIT*DATA_WIDTH-1:0] neighbor_LJ_Force_Z,
	output [NUM_EVAL_UNIT-1:0] neighbor_forceoutput_valid,
	// Done signal, when entire home cell is done processing, this will keep high until the next time 'start' signal turn high
	output done
);
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// Signals between Cell Module and FSM
	///////////////////////////////////////////////////////////////////////////////////////////////
	//// Signals connect from cell module to FSM
	// Position Data
	// Order: MSB->LSB {333,332,331,323,322,321,313,312,311,233,232,231,223,222} Homecell is on LSB side
	wire [(NUM_NEIGHBOR_CELLS+1)*3*DATA_WIDTH-1:0] Cell_to_FSM_readout_particle_position;
	//// Signals connect from FSM to cell modules
	wire rden;
	// Read Address to cells
	wire [(NUM_NEIGHBOR_CELLS+1)*CELL_ADDR_WIDTH-1:0] FSM_to_Cell_read_addr;

	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// Signals between Force Evaluation Unit and FSM
	///////////////////////////////////////////////////////////////////////////////////////////////
	//// Signals connect from FSM to Force Evaluation module
	wire [NUM_FILTER*3*DATA_WIDTH-1:0] FSM_to_ForceEval_ref_particle_position;
	wire [NUM_FILTER*3*DATA_WIDTH-1:0] FSM_to_ForceEval_neighbor_particle_position;
	wire [NUM_FILTER*PARTICLE_ID_WIDTH-1:0] FSM_to_ForceEval_ref_particle_id;				// {cell_x, cell_y, cell_z, ref_particle_rd_addr}
	wire [NUM_FILTER*PARTICLE_ID_WIDTH-1:0] FSM_to_ForceEval_neighbor_particle_id;		// {cell_x, cell_y, cell_z, neighbor_particle_rd_addr}
	wire [NUM_FILTER-1:0] FSM_to_ForceEval_input_pair_valid;			// Signify the valid of input particle data, this signal should have 1 cycle delay of the rden signal, thus wait for the data read out from BRAM
	//// Signals connect from Force Evaluation module to FSM
	wire [NUM_FILTER-1:0] ForceEval_to_FSM_backpressure;
	wire ForceEval_to_FSM_all_buffer_empty;

	
	// FSM for generating particle pairs
	Particle_Pair_Gen_HalfShell
	#(
		.DATA_WIDTH(DATA_WIDTH),
		// The home cell this unit is working on
		.CELL_X(2),
		.CELL_Y(2),
		.CELL_Z(2),
		// High level parameters
		.NUM_EVAL_UNIT(NUM_EVAL_UNIT),							// # of evaluation units in the design
		// Dataset defined parameters
		.NUM_NEIGHBOR_CELLS(NUM_NEIGHBOR_CELLS),				// # of neighbor cells per home cell, for Half-shell method, is 13
		.CELL_ID_WIDTH(CELL_ID_WIDTH),							// log(NUM_NEIGHBOR_CELLS)
		.MAX_CELL_PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),
		.CELL_ADDR_WIDTH(CELL_ADDR_WIDTH),						// log(MAX_CELL_PARTICLE_NUM)
		.PARTICLE_ID_WIDTH(PARTICLE_ID_WIDTH),					// # of bit used to represent particle ID, 9*9*7 cells, each 4-bit, each cell have max of 220 particles, 8-bit
		// Filter parameters
		.NUM_FILTER(NUM_FILTER),
		.ARBITER_MSB(ARBITER_MSB),									// 2^(NUM_FILTER-1)
		.FILTER_BUFFER_DEPTH(FILTER_BUFFER_DEPTH),
		.FILTER_BUFFER_ADDR_WIDTH(FILTER_BUFFER_ADDR_WIDTH)
	)
	FSM
	(
		.clk(clk),
		.rst(rst),
		.start(start),
		// Ports connect to Cell Memory Module
		// Order: MSB->LSB {333,332,331,323,322,321,313,312,311,233,232,231,223,222} Homecell is on LSB side
		.Cell_to_FSM_readout_particle_position(Cell_to_FSM_readout_particle_position),					// input  [(NUM_NEIGHBOR_CELLS+1)*3*DATA_WIDTH-1:0] 
		.FSM_to_Cell_read_addr(FSM_to_Cell_read_addr),																// output [(NUM_NEIGHBOR_CELLS+1)*CELL_ADDR_WIDTH-1:0] 
		.FSM_to_Cell_rden(rden),
		// Ports connect to Force Evaluation Unit
		.ForceEval_to_FSM_backpressure(ForceEval_to_FSM_backpressure),											// input  [NUM_FILTER-1:0]  								// Backpressure signal from Force Evaluation Unit
		.ForceEval_to_FSM_all_buffer_empty(ForceEval_to_FSM_all_buffer_empty),								// input															// Only when all the filter buffers are empty, then the FSM will move on to the next reference particle
		.FSM_to_ForceEval_ref_particle_position(FSM_to_ForceEval_ref_particle_position),  				// output [NUM_FILTER*3*DATA_WIDTH-1:0] 
		.FSM_to_ForceEval_neighbor_particle_position(FSM_to_ForceEval_neighbor_particle_position),	// output [NUM_FILTER*3*DATA_WIDTH-1:0] 
		.FSM_to_ForceEval_ref_particle_id(FSM_to_ForceEval_ref_particle_id),									// output [NUM_FILTER*PARTICLE_ID_WIDTH-1:0] 		// {cell_z, cell_y, cell_x, ref_particle_rd_addr}
		.FSM_to_ForceEval_neighbor_particle_id(FSM_to_ForceEval_neighbor_particle_id),					// output [NUM_FILTER*PARTICLE_ID_WIDTH-1:0] 		// {cell_z, cell_y, cell_x, neighbor_particle_rd_addr}
		.FSM_to_ForceEval_input_pair_valid(FSM_to_ForceEval_input_pair_valid),								// output reg [NUM_FILTER-1:0]   						// Signify the valid of input particle data, this signal should have 1 cycle delay of the rden signal, thus wait for the data read out from BRAM
		// Ports to top level modules
		.done(done)
	);
	
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// Force evaluation and accumulation unit
	///////////////////////////////////////////////////////////////////////////////////////////////
	RL_LJ_Evaluation_Unit
	#(
		.DATA_WIDTH(DATA_WIDTH),
		// The home cell this unit is working on
		.CELL_X(CELL_X),
		.CELL_Y(CELL_Y),
		.CELL_Z(CELL_Z),
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
		.out_all_buffer_empty_to_input(ForceEval_to_FSM_all_buffer_empty),				//output													// Only when all the filter buffers are empty, then the FSM will move on to the next reference particle
		// Output accumulated force for reference particles
		// The output value is the accumulated value
		// Connected to home cell
		.out_ref_particle_id(ref_particle_id),						//output [PARTICLE_ID_WIDTH-1:0]
		.out_ref_LJ_Force_X(ref_LJ_Force_X),						//output [DATA_WIDTH-1:0]
		.out_ref_LJ_Force_Y(ref_LJ_Force_Y),						//output [DATA_WIDTH-1:0]
		.out_ref_LJ_Force_Z(ref_LJ_Force_Z),						//output [DATA_WIDTH-1:0]
		.out_ref_force_valid(ref_forceoutput_valid),				//output
		// Output partial force for neighbor particles
		// The output value should be the minus value of the calculated force data
		// Connected to neighbor cells, if the neighbor paritle comes from the home cell, then discard, since the value will be recalculated when evaluating this particle as reference one
		.out_neighbor_particle_id(neighbor_particle_id),		//output [PARTICLE_ID_WIDTH-1:0]
		.out_neighbor_LJ_Force_X(neighbor_LJ_Force_X),			//output [DATA_WIDTH-1:0]
		.out_neighbor_LJ_Force_Y(neighbor_LJ_Force_Y),			//output [DATA_WIDTH-1:0]
		.out_neighbor_LJ_Force_Z(neighbor_LJ_Force_Z),			//output [DATA_WIDTH-1:0]
		.out_neighbor_force_valid(neighbor_forceoutput_valid)	//output
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

	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// Force cache
	// Each cell has an independent cache, MSB -> LSB:{Force_Z, Force_Y, Force_X}
	// The force Serve as the buffer to hold evaluated force values during evaluation
	// The initial force value is 0
	//	When new force value arrives, it will accumulate to the current stored value
	///////////////////////////////////////////////////////////////////////////////////////////////
	// Home cell 222
	Force_Write_Back_Controller
	#(
		.DATA_WIDTH(DATA_WIDTH),
		// Dell id this unit related to
		.CELL_X(CELL_X),
		.CELL_Y(CELL_Y),
		.CELL_Z(CELL_Z),
		// Dataset defined parameters
		.CELL_ID_WIDTH(CELL_ID_WIDTH),
		.MAX_CELL_PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),		// The maximum # of particles can be in a cell
		.CELL_ADDR_WIDTH(CELL_ADDR_WIDTH),						// log(MAX_CELL_PARTICLE_NUM)
		.PARTICLE_ID_WIDTH(PARTICLE_ID_WIDTH)					// # of bit used to represent particle ID, 9*9*7 cells, each 4-bit, each cell have max of 220 particles, 8-bit
	)
	Cell_222_Force_Cache
	(
		.clk(clk),
		.rst(rst),
		// Cache input force
		.in_partial_force_valid(ref_forceoutput_valid),
		.in_particle_id(ref_particle_id),
		.in_partial_force({ref_LJ_Force_Z, ref_LJ_Force_Y, ref_LJ_Force_X}),
		// Cache output force
		.in_read_data_request(),									// Enables read data from the force cache, if this signal is high, then no write operation is permitted
		.in_cache_read_address(),
		.out_partial_force(),
		.out_cache_readout_valid()
);

endmodule



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Module: RL_LJ_Evaluation_Unit.v
//
//	Function: Evaluate the accumulated LJ force of given datasets using 1st order interpolation (interpolation index is generated in Matlab (under Ethan_GoldenModel/Matlab_Interpolation))
// 			Force_Evaluation_Unit with Accumulation_Unit and send out neighbor particle force (with negation)
//				Single set of force evaluation unit, including:
//							* Single force evaluation pipeline
//							* Multiple filters
//							* Accumulation unit for reference particles
//				Output:
//							* Each iteration, output neighbor particle's partial force
//							* when the reference particle is done, output the accumulated force on this reference particle
//
// Mapping Model:
//				Half-shell mapping
//				Each force pipeline working on a single reference particle until all the neighboring particles are evaluated, then move to the next reference particle
//				Depending the # of cells, each unit will be responsible for part of a home cell, or a single home cell, or multiple home cells
//
//	Purpose:
//				Filter version, used for final system (half-shell mapping scheme)
//
// Used by:
//				RL_LJ_Top.v
//
// Dependency:
//				RL_LJ_Force_Evaluation_Unit.v
//
// Latency: TBD
//
// Created by: Chen Yang 10/23/18
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module RL_LJ_Evaluation_Unit
#(
	parameter DATA_WIDTH 					= 32,
	// Dataset defined parameters
	parameter PARTICLE_ID_WIDTH			= 20,										// # of bit used to represent particle ID, 9*9*7 cells, each 4-bit, each cell have max of 200 particles, 8-bit
	// Filter parameters
	parameter NUM_FILTER						= 4,	// 8
	parameter ARBITER_MSB 					= 8,	//128								// 2^(NUM_FILTER-1)
	parameter FILTER_BUFFER_DEPTH 		= 32,
	parameter FILTER_BUFFER_ADDR_WIDTH	= 5,
	parameter CUTOFF_2 						= 32'h43100000,						// (12^2=144 in IEEE floating point)
	// Force Evaluation parameters
	parameter SEGMENT_NUM					= 14,
	parameter SEGMENT_WIDTH					= 4,
	parameter BIN_NUM							= 256,
	parameter BIN_WIDTH						= 8,
	parameter LOOKUP_NUM						= SEGMENT_NUM * BIN_NUM,			// SEGMENT_NUM * BIN_NUM
	parameter LOOKUP_ADDR_WIDTH			= SEGMENT_WIDTH + BIN_WIDTH		// log LOOKUP_NUM / log 2
)
(
	input  clk,
	input  rst,
	input  start,
	input  [NUM_FILTER-1:0] in_input_pair_valid,
	input  [NUM_FILTER*PARTICLE_ID_WIDTH-1:0] in_ref_particle_id,
	input  [NUM_FILTER*PARTICLE_ID_WIDTH-1:0] in_neighbor_particle_id,
	input  [NUM_FILTER*3*DATA_WIDTH-1:0] in_ref_particle_position,			// {refz, refy, refx}
	input  [NUM_FILTER*3*DATA_WIDTH-1:0] in_neighbor_particle_position,	// {neighborz, neighbory, neighborx}
	output [NUM_FILTER-1:0] out_back_pressure_to_input,						// backpressure signal to stop new data arrival from particle memory
	// Output partial force for neighbor particles
	// The output value should be the minus value of the calculated force data 
	output [PARTICLE_ID_WIDTH-1:0] out_ref_particle_id,
	output [PARTICLE_ID_WIDTH-1:0] out_neighbor_particle_id,
	output [DATA_WIDTH-1:0] out_LJ_Force_X,
	output [DATA_WIDTH-1:0] out_LJ_Force_Y,
	output [DATA_WIDTH-1:0] out_LJ_Force_Z,
	output out_forceoutput_valid
);

	// Wires for assigning the output reference particle value
	wire [DATA_WIDTH-1:0] LJ_Force_X_wire;
	wire [DATA_WIDTH-1:0] LJ_Force_Y_wire;
	wire [DATA_WIDTH-1:0] LJ_Force_Z_wire;
	generate
		begin: neighbor_particle_partial_force_assignment
		assign out_LJ_Force_X[DATA_WIDTH-2:0] = LJ_Force_X_wire[DATA_WIDTH-2:0];	
		assign out_LJ_Force_X[DATA_WIDTH-1] = ~LJ_Force_X_wire[DATA_WIDTH-1];		// Negate the sign bit
		assign out_LJ_Force_Y[DATA_WIDTH-2:0] = LJ_Force_Y_wire[DATA_WIDTH-2:0];
		assign out_LJ_Force_Y[DATA_WIDTH-1] = ~LJ_Force_Y_wire[DATA_WIDTH-1];		// Negate the sign bit
		assign out_LJ_Force_Z[DATA_WIDTH-2:0] = LJ_Force_Z_wire[DATA_WIDTH-2:0];
		assign out_LJ_Force_Z[DATA_WIDTH-1] = ~LJ_Force_Z_wire[DATA_WIDTH-1];		// Negate the sign bit
		end
	endgenerate
	
	// Wires for assigning input particle data to Force Evaluation Unit
	wire [NUM_FILTER*DATA_WIDTH-1:0] refx_in_wire, refy_in_wire, refz_in_wire;
	wire [NUM_FILTER*DATA_WIDTH-1:0] neighborx_in_wire, neighbory_in_wire, neighborz_in_wire;
	genvar i;
	generate 
		for(i = 0; i < NUM_FILTER; i = i + 1)
			begin: input_wire_assignment
			assign refx_in_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH] = in_ref_particle_position[i*3*DATA_WIDTH+DATA_WIDTH-1:i*3*DATA_WIDTH];
			assign refy_in_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH] = in_ref_particle_position[i*3*DATA_WIDTH+2*DATA_WIDTH-1:i*3*DATA_WIDTH+DATA_WIDTH];
			assign refz_in_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH] = in_ref_particle_position[i*3*DATA_WIDTH+3*DATA_WIDTH-1:i*3*DATA_WIDTH+2*DATA_WIDTH];
			assign neighborx_in_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH] = in_neighbor_particle_position[i*3*DATA_WIDTH+DATA_WIDTH-1:i*3*DATA_WIDTH];
			assign neighbory_in_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH] = in_neighbor_particle_position[i*3*DATA_WIDTH+2*DATA_WIDTH-1:i*3*DATA_WIDTH+DATA_WIDTH];
			assign neighborz_in_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH] = in_neighbor_particle_position[i*3*DATA_WIDTH+3*DATA_WIDTH-1:i*3*DATA_WIDTH+2*DATA_WIDTH];
			end
	endgenerate
	
	// Force evaluation unit
	// Including filters and force evaluation pipeline
	RL_LJ_Force_Evaluation_Unit
	#(
		.DATA_WIDTH(DATA_WIDTH),
		// Dataset defined parameters
		.PARTICLE_ID_WIDTH(PARTICLE_ID_WIDTH),							// # of bit used to represent particle ID, 9*9*7 cells, each 4-bit, each cell have max of 200 particles, 8-bit
		// Filter parameters
		.NUM_FILTER(NUM_FILTER),
		.ARBITER_MSB(ARBITER_MSB),											// 2^(NUM_FILTER-1)
		.FILTER_BUFFER_DEPTH(FILTER_BUFFER_DEPTH),
		.FILTER_BUFFER_ADDR_WIDTH(FILTER_BUFFER_ADDR_WIDTH),
		.CUTOFF_2(CUTOFF_2),													// in IEEE floating point format
		// Force Evaluation parameters
		.SEGMENT_NUM(SEGMENT_NUM),
		.SEGMENT_WIDTH(SEGMENT_WIDTH),
		.BIN_NUM(BIN_NUM),
		.BIN_WIDTH(BIN_WIDTH),
		.LOOKUP_NUM(LOOKUP_NUM),											// SEGMENT_NUM * BIN_NUM
		.LOOKUP_ADDR_WIDTH(LOOKUP_ADDR_WIDTH)							// log(LOOKUP_NUM) / log 2
	)
	RL_LJ_Force_Evaluation_Unit
	(
		.clk(clk),
		.rst(rst),
		.input_valid(in_input_pair_valid),								// INPUT [NUM_FILTER-1:0]
		.ref_particle_id(in_ref_particle_id),							// INPUT [NUM_FILTER*PARTICLE_ID_WIDTH-1:0]
		.neighbor_particle_id(in_neighbor_particle_id),				// INPUT [NUM_FILTER*PARTICLE_ID_WIDTH-1:0]
		.refx(refx_in_wire),													// INPUT [NUM_FILTER*DATA_WIDTH-1:0]
		.refy(refy_in_wire),													// INPUT [NUM_FILTER*DATA_WIDTH-1:0]
		.refz(refz_in_wire),													// INPUT [NUM_FILTER*DATA_WIDTH-1:0]
		.neighborx(neighborx_in_wire),									// INPUT [NUM_FILTER*DATA_WIDTH-1:0]
		.neighbory(neighbory_in_wire),									// INPUT [NUM_FILTER*DATA_WIDTH-1:0]
		.neighborz(neighborz_in_wire),									// INPUT [NUM_FILTER*DATA_WIDTH-1:0]
		.ref_particle_id_out(out_ref_particle_id),					// OUTPUT [PARTICLE_ID_WIDTH-1:0]
		.neighbor_particle_id_out(out_neighbor_particle_id),		// OUTPUT [PARTICLE_ID_WIDTH-1:0]
		.LJ_Force_X(LJ_Force_X_wire),										// OUTPUT [DATA_WIDTH-1:0]
		.LJ_Force_Y(LJ_Force_Y_wire),										// OUTPUT [DATA_WIDTH-1:0]
		.LJ_Force_Z(LJ_Force_Z_wire),										// OUTPUT [DATA_WIDTH-1:0]
		.forceoutput_valid(out_forceoutput_valid),					// OUTPUT
		.back_pressure_to_input(out_back_pressure_to_input)		// OUTPUT [NUM_FILTER-1:0]
	);

endmodule



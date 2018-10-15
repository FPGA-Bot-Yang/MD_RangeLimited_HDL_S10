/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Module: Filter_Bank.v
//
//	Function: Holding multiple filter, perform arbitration to read from multiple available filter buffers
//					Sending selected data to force evaluation pipeline
//
// Dependency:
// 			Filter_Logic.v
//				Filter_Arbiter.v
//
//
// Created by: Chen Yang 10/10/18
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module Filter_Bank
#(
	parameter DATA_WIDTH 					= 32,
	parameter NUM_FILTER						= 4,	// 8
	parameter ARBITER_MSB 					= 8,	//128				// 2^(NUM_FILTER-1)
	parameter FILTER_BUFFER_DEPTH 		= 32,
	parameter FILTER_BUFFER_ADDR_WIDTH	= 5,
	parameter CUTOFF_2 						= 32'h43100000						// (12^2=144 in IEEE floating point)
)
(
	input clk,
	input rst,
	input [NUM_FILTER-1:0] input_valid,
	input [NUM_FILTER*DATA_WIDTH-1:0] refx,
	input [NUM_FILTER*DATA_WIDTH-1:0] refy,
	input [NUM_FILTER*DATA_WIDTH-1:0] refz,
	input [NUM_FILTER*DATA_WIDTH-1:0] neighborx,
	input [NUM_FILTER*DATA_WIDTH-1:0] neighbory,
	input [NUM_FILTER*DATA_WIDTH-1:0] neighborz,
	
	output [DATA_WIDTH-1:0] r2,
	output [DATA_WIDTH-1:0] dx,
	output [DATA_WIDTH-1:0] dy,
	output [DATA_WIDTH-1:0] dz,
	output out_valid,
	
	output [NUM_FILTER-1:0] back_pressure_to_input							// If one of the FIFO is full, then set the back_pressure flag to stop more incoming particle pairs
);

	// Wires between Filter_Logic and Arbiter
	wire [NUM_FILTER*DATA_WIDTH-1:0] r2_wire, dx_wire, dy_wire, dz_wire;
	wire [NUM_FILTER-1:0] arbitration_result;								// Arbitor -> Filter
	wire [NUM_FILTER-1:0] filter_data_available;							// Filter -> Arbitor

	// Instantiate the Filter_Logic modules
	genvar i;
	generate 
		for(i = 0; i < NUM_FILTER; i = i + 1) begin: Filter_Unit
		Filter_Logic
		#(
			.DATA_WIDTH(DATA_WIDTH),
			.FILTER_BUFFER_DEPTH(FILTER_BUFFER_DEPTH),
			.FILTER_BUFFER_ADDR_WIDTH(FILTER_BUFFER_ADDR_WIDTH),
			.CUTOFF_2(CUTOFF_2)													// (12^2=144 in IEEE floating point)
		)
		Filter_Logic
		(
			.clk(clk),
			.rst(rst),
			.input_valid(input_valid[i]),
			.refx(refx[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH]),
			.refy(refy[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH]),
			.refz(refz[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH]),
			.neighborx(neighborx[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH]),
			.neighbory(neighbory[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH]),
			.neighborz(neighborz[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH]),
			.r2(r2_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH]),
			.dx(dx_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH]),
			.dy(dy_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH]),
			.dz(dz_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH]),
			// Connect to filter arbiter
			.sel(arbitration_result[i]),									// Input
			.particle_pair_available(filter_data_available[i]),	// Output
			// Connect to input generator
			.filter_back_pressure()											// Output: Buffer should have enough space to store 17 pairs after the input stop coming
	);
	end
	endgenerate
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Arbitration logic
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	Filter_Arbiter
	#(
		.NUM_FILTER(NUM_FILTER),
		.ARBITER_MSB(ARBITER_MSB)
	)
	Filter_Arbiter
	(
		.clk(clk),
		.rst(rst),
		.Filter_Available_Flag(filter_data_available),
		.Arbitration_Result(arbitration_result)
	);


endmodule
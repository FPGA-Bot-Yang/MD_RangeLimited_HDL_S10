/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Module: Filter_Bank.v
//
//	Function: Holding multiple filter, perform arbitration to read from multiple available filter buffers
//					Sending selected data to force evaluation pipeline
//
// Used by:
//				RL_LJ_Force_Evaluation_Unit.v
//
// Dependency:
// 			Filter_Logic.v
//				Filter_Arbiter.v
//
// Created by: Chen Yang 10/10/18
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module Filter_Bank
#(
	parameter DATA_WIDTH 					= 32,
	parameter NUM_FILTER						= 4,	// 8
	parameter ARBITER_MSB 					= 8,	//128							// 2^(NUM_FILTER-1)
	parameter PARTICLE_ID_WIDTH			= 20,									// # of bit used to represent particle ID, 9*9*7 cells, each 4-bit, each cell have max of 200 particles, 8-bit
	parameter FILTER_BUFFER_DEPTH 		= 32,
	parameter FILTER_BUFFER_ADDR_WIDTH	= 5,
	parameter CUTOFF_2 						= 32'h43100000						// (12^2=144 in IEEE floating point)
)
(
	input clk,
	input rst,
	input [NUM_FILTER-1:0] input_valid,
	input [NUM_FILTER*PARTICLE_ID_WIDTH-1:0] ref_particle_id,
	input [NUM_FILTER*PARTICLE_ID_WIDTH-1:0] neighbor_particle_id,
	input [NUM_FILTER*DATA_WIDTH-1:0] refx,
	input [NUM_FILTER*DATA_WIDTH-1:0] refy,
	input [NUM_FILTER*DATA_WIDTH-1:0] refz,
	input [NUM_FILTER*DATA_WIDTH-1:0] neighborx,
	input [NUM_FILTER*DATA_WIDTH-1:0] neighbory,
	input [NUM_FILTER*DATA_WIDTH-1:0] neighborz,
	output reg [NUM_FILTER*PARTICLE_ID_WIDTH-1:0] ref_particle_id_out,
	output reg [NUM_FILTER*PARTICLE_ID_WIDTH-1:0] neighbor_particle_id_out,
	output reg [DATA_WIDTH-1:0] r2,
	output reg [DATA_WIDTH-1:0] dx,
	output reg [DATA_WIDTH-1:0] dy,
	output reg [DATA_WIDTH-1:0] dz,
	output reg out_valid,
	
	output [NUM_FILTER-1:0] back_pressure_to_input							// If one of the FIFO is full, then set the back_pressure flag to stop more incoming particle pairs
);

	// Wires between Filter_Logic and Arbiter
	wire [NUM_FILTER*DATA_WIDTH-1:0] r2_wire, dx_wire, dy_wire, dz_wire;
	wire [NUM_FILTER*PARTICLE_ID_WIDTH-1:0] ref_particle_id_out_wire, neighbor_particle_id_out_wire;
	wire [NUM_FILTER-1:0] arbitration_result;								// Arbitor -> Filter
	wire [NUM_FILTER-1:0] filter_data_available;							// Filter -> Arbitor
	
	// Assign out_valid
	always@(posedge clk)
		begin
		if(arbitration_result == 0)
			out_valid <= 1'b0;
		else
			out_valid <= 1'b1;
		end
	always@(posedge clk)
		begin
		case(arbitration_result)
			8'b00000001:
				begin
				ref_particle_id_out <= ref_particle_id_out_wire[PARTICLE_ID_WIDTH-1:0];
				neighbor_particle_id_out <= neighbor_particle_id_out_wire[PARTICLE_ID_WIDTH-1:0];
				r2 <= r2_wire[DATA_WIDTH-1:0];
				dx <= dx_wire[DATA_WIDTH-1:0];
				dy <= dy_wire[DATA_WIDTH-1:0];
				dz <= dz_wire[DATA_WIDTH-1:0];
				end
			8'b00000010:
				begin
				ref_particle_id_out <= ref_particle_id_out_wire[2*PARTICLE_ID_WIDTH-1:1*PARTICLE_ID_WIDTH];
				neighbor_particle_id_out <= neighbor_particle_id_out_wire[2*PARTICLE_ID_WIDTH-1:1*PARTICLE_ID_WIDTH];
				r2 <= r2_wire[2*DATA_WIDTH-1:1*DATA_WIDTH];
				dx <= dx_wire[2*DATA_WIDTH-1:1*DATA_WIDTH];
				dy <= dy_wire[2*DATA_WIDTH-1:1*DATA_WIDTH];
				dz <= dz_wire[2*DATA_WIDTH-1:1*DATA_WIDTH];
				end
			8'b00000100:
				begin
				ref_particle_id_out <= ref_particle_id_out_wire[3*PARTICLE_ID_WIDTH-1:2*PARTICLE_ID_WIDTH];
				neighbor_particle_id_out <= neighbor_particle_id_out_wire[3*PARTICLE_ID_WIDTH-1:2*PARTICLE_ID_WIDTH];
				r2 <= r2_wire[3*DATA_WIDTH-1:2*DATA_WIDTH];
				dx <= dx_wire[3*DATA_WIDTH-1:2*DATA_WIDTH];
				dy <= dy_wire[3*DATA_WIDTH-1:2*DATA_WIDTH];
				dz <= dz_wire[3*DATA_WIDTH-1:2*DATA_WIDTH];
				end
			8'b00001000:
				begin
				ref_particle_id_out <= ref_particle_id_out_wire[4*PARTICLE_ID_WIDTH-1:3*PARTICLE_ID_WIDTH];
				neighbor_particle_id_out <= neighbor_particle_id_out_wire[4*PARTICLE_ID_WIDTH-1:3*PARTICLE_ID_WIDTH];
				r2 <= r2_wire[4*DATA_WIDTH-1:3*DATA_WIDTH];
				dx <= dx_wire[4*DATA_WIDTH-1:3*DATA_WIDTH];
				dy <= dy_wire[4*DATA_WIDTH-1:3*DATA_WIDTH];
				dz <= dz_wire[4*DATA_WIDTH-1:3*DATA_WIDTH];
				end
		endcase
		end
	
	// Assign output data
	genvar i;
	always@(posedge clk)
		begin
		for(i = 0; i < NUM_FILTER; i = i + 1) begin: Output_selection
			if(arbitration_result == (1 << i))
				begin
				ref_particle_id_out <= ref_particle_id_out_wire[(i+1)*PARTICLE_ID_WIDTH-1:i*PARTICLE_ID_WIDTH];
				neighbor_particle_id_out <= neighbor_particle_id_out_wire[(i+1)*PARTICLE_ID_WIDTH-1:i*PARTICLE_ID_WIDTH];
				r2 <= r2_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH];
				dx <= dx_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH];
				dy <= dy_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH];
				dz <= dz_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH];
				end
			end
		end
	/*
	always@(posedge clk)
		begin
		tmp_1 <= 1;
		for(i = 0; i < NUM_FILTER; i = i + 1)
			begin
			tmp <= tmp_1 << i;
			// Assign output data
			if(arbitration_result == tmp)
				begin
				ref_particle_id_out <= ref_particle_id_out_wire[(i+1)*PARTICLE_ID_WIDTH-1:i*PARTICLE_ID_WIDTH];
				neighbor_particle_id_out <= neighbor_particle_id_out_wire[(i+1)*PARTICLE_ID_WIDTH-1:i*PARTICLE_ID_WIDTH];
				r2 <= r2_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH];
				dx <= dx_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH];
				dy <= dy_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH];
				dz <= dz_wire[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH];
				end
			// Assign out_valid
			if(arbitration_result == 0)
				out_valid <= 1'b0;
			else
				out_valid <= 1'b1;
			end

		end
*/
	// Instantiate the Filter_Logic modules
	generate 
		for(i = 0; i < NUM_FILTER; i = i + 1) begin: Filter_Unit
		Filter_Logic
		#(
			.DATA_WIDTH(DATA_WIDTH),
			.PARTICLE_ID_WIDTH(PARTICLE_ID_WIDTH),
			.FILTER_BUFFER_DEPTH(FILTER_BUFFER_DEPTH),
			.FILTER_BUFFER_ADDR_WIDTH(FILTER_BUFFER_ADDR_WIDTH),
			.CUTOFF_2(CUTOFF_2)													// (12^2=144 in IEEE floating point)
		)
		Filter_Logic
		(
			.clk(clk),
			.rst(rst),
			.input_valid(input_valid[i]),
			.ref_particle_id(ref_particle_id[(i+1)*PARTICLE_ID_WIDTH-1:i*PARTICLE_ID_WIDTH]),
			.neighbor_particle_id(neighbor_particle_id[(i+1)*PARTICLE_ID_WIDTH-1:i*PARTICLE_ID_WIDTH]),
			.refx(refx[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH]),
			.refy(refy[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH]),
			.refz(refz[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH]),
			.neighborx(neighborx[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH]),
			.neighbory(neighbory[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH]),
			.neighborz(neighborz[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH]),
			.ref_particle_id_out(ref_particle_id_out_wire[(i+1)*PARTICLE_ID_WIDTH-1:i*PARTICLE_ID_WIDTH]),
			.neighbor_particle_id_out(ref_particle_id_out_wire[(i+1)*PARTICLE_ID_WIDTH-1:i*PARTICLE_ID_WIDTH]),
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
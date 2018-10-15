/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Module: Filter_Logic.v
//
//	Function: Filter logic, send only particle pairs that within cutoff radius to force pipeline
//					Multiple filters are corresponding to a single force pipleine
//					Buffer to store the filtered particle pairs -> Backpressure needed when buffer is full
//					An arbitration will be needed when implement multiple filters (Filter_Bank) to select from one of the available ones
//					The data valid signal should be assigned in the Filter_Bank module
//
// Data Organization:
//				Data organization in buffer: MSB-LSB: {r2, dz, dy, dx}
//
// Dependency:
// 			r2_compute.v
//				Filter_Buffer.v
//
// Latency: total: xx cycles
//				r2_compute													17 cycles
//
// Created by: Chen Yang 10/10/18
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module Filter_Logic
#(
	parameter DATA_WIDTH 					= 32,
	parameter FILTER_BUFFER_DEPTH 		= 32,
	parameter FILTER_BUFFER_ADDR_WIDTH	= 5,
	parameter CUTOFF_2 						= 32'h43100000						// (12^2=144 in IEEE floating point)
)
(
	input clk,
	input rst,
	input input_valid,
	input [DATA_WIDTH-1:0] refx,
	input [DATA_WIDTH-1:0] refy,
	input [DATA_WIDTH-1:0] refz,
	input [DATA_WIDTH-1:0] neighborx,
	input [DATA_WIDTH-1:0] neighbory,
	input [DATA_WIDTH-1:0] neighborz,
	output [DATA_WIDTH-1:0] r2,
	output [DATA_WIDTH-1:0] dx,
	output [DATA_WIDTH-1:0] dy,
	output [DATA_WIDTH-1:0] dz,
	// Connect to filter arbiter
	input sel,
	output particle_pair_available,
	// Connect to input generator
	output filter_back_pressure								// Buffer should have enough space to store 17 pairs after the input stop coming
);


	// Wires connect r2_compute and Filter_Buffer
	wire [DATA_WIDTH-1:0] r2_wire, dx_wire, dy_wire, dz_wire;
	wire r2_valid;
	
	// Assign Output: backpressure
	// 17 is the latency in r2_compute
	wire [FILTER_BUFFER_ADDR_WIDTH-1:0] buffer_usedw;
	assign filter_back_pressure = (FILTER_BUFFER_DEPTH - buffer_usedw <= 17) ? 1'b1 : 1'b0;
	
	// Assign Output: particle_pair_available
	wire buffer_empty;
	assign particle_pair_available = ~buffer_empty;
	
	/////////////////////////////////////////////////////////////////////////////
	// Filter Logic
	/////////////////////////////////////////////////////////////////////////////
	reg buffer_wr;
	reg [DATA_WIDTH*4-1:0] buffer_wr_data;
	always@(posedge clk)
		begin
		if(rst)
			begin
			buffer_wr_data <= 0;
			buffer_wr <= 1'b0;
			end
		else if(r2_valid && r2_wire < CUTOFF_2)
			begin
			buffer_wr_data <= {r2_wire, dz_wire, dy_wire, dx_wire};
			buffer_wr <= 1'b1;
			end
		else
			begin
			buffer_wr_data <= 0;
			buffer_wr <= 1'b0;
			end
		end
	
	// Evaluate r2 between particle pairs
	r2_compute #(
		.DATA_WIDTH(DATA_WIDTH)
	)
	r2_evaluate(
		.clk(clk),
		.rst(rst),
		.enable(input_valid),
		.refx(refx),
		.refy(refy),
		.refz(refz),
		.neighborx(neighborx),
		.neighbory(neighbory),
		.neighborz(neighborz),
		.r2(r2_wire),
		.dx_out(dx_wire),
		.dy_out(dy_wire),
		.dz_out(dz_wire),
		.r2_valid(r2_valid)
	);
	
	// Buffer for pairs passed the filter logic
	// Data organization in buffer: MSB-LSB: {r2, dz, dy, dx}
	Filter_Buffer
	#(
		.DATA_WIDTH(DATA_WIDTH*4),														// hold r2, dx, dy, dz
		.FILTER_BUFFER_DEPTH(FILTER_BUFFER_DEPTH),
		.FILTER_BUFFER_ADDR_WIDTH(FILTER_BUFFER_ADDR_WIDTH)					// log(FILTER_BUFFER_DEPTH) / log 2
	)
	Filter_Buffer
	(
		 .clock(clk),
		 .data(buffer_wr_data),
		 .rdreq(sel),
		 .wrreq(buffer_wr),
		 .empty(buffer_empty),
		 .full(),
		 .q({r2, dz, dy, dx}),
		 .usedw(buffer_usedw)
	);


endmodule
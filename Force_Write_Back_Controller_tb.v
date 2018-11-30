/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Module: Force_Write_Back_Controller_tb.v
//
//	Function: 
//				
//
//	Purpose:
//				
//
// Mapping Scheme:
//				
//
// Format:
//				
//
// Used by:
//				.v
//
// Dependency:
//				.v
//
// Testbench:
//				_tb.v
//
// Timing:
//				TBD
//
// Todo:
//				
//
// Created by: 
//				Chen Yang 11/29/18
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ns
module Force_Write_Back_Controller_tb;

	parameter DATA_WIDTH 					= 32;											// Data width of a single force value, 32-bit
	// Cell id this unit related to
	parameter CELL_X							= 2;
	parameter CELL_Y							= 2;
	parameter CELL_Z							= 2;
	// Force cache input buffer
	parameter FORCE_CACHE_BUFFER_DEPTH	= 16;
	parameter FORCE_CACHE_BUFFER_ADDR_WIDTH = 4;										// log(FORCE_CACHE_BUFFER_DEPTH) / log 2
	// Dataset defined parameters
	parameter CELL_ID_WIDTH					= 4;
	parameter MAX_CELL_PARTICLE_NUM		= 290;										// The maximum # of particles can be in a cell
	parameter CELL_ADDR_WIDTH				= 9;											// log(MAX_CELL_PARTICLE_NUM)
	parameter PARTICLE_ID_WIDTH			= CELL_ID_WIDTH*3+CELL_ADDR_WIDTH;	// # of bit used to represent particle ID, 9*9*7 cells, each 4-bit, each cell have max of 220 particles, 8-bit

	reg clk, rst;
	// Cache input force
	reg  in_partial_force_valid;
	reg  [PARTICLE_ID_WIDTH-1:0] in_particle_id;
	reg  [3*DATA_WIDTH-1:0] in_partial_force;
	// Cache output force
	reg  in_read_data_request;																	// Enables read data from the force cache, if this signal is high, then no write operation is permitted
	reg  [CELL_ADDR_WIDTH-1:0] in_cache_read_address;
	wire [3*DATA_WIDTH-1:0] out_partial_force;
	wire out_cache_readout_valid;
	
	always #1 clk <= ~clk;
	
	
	initial begin
		clk <= 1'b1;
		rst <= 1'b1;
		in_partial_force_valid <= 1'b0;
		in_particle_id <= {4'd2,4'd2,4'd2,9'd1};
		in_partial_force <= {32'h3F800000, 32'h3F800000, 32'h3F800000};				// input value: {1.0,1.0,1.0}
		in_read_data_request <= 1'b0;
		in_cache_read_address <= 9'd1;
		
		// Clear reset signal
		#10
		rst <= 1'b0;
		
		// Input valid for 5 cycles
		#10
		in_partial_force_valid <= 1'b1;
		
		// Input invalid
		#10
		in_partial_force_valid <= 1'b0;
		
		// Readout the final result
		#50
		in_read_data_request <= 1'b1;
		
		
	end
	
	
	// UUT
	Force_Write_Back_Controller
	#(
		.DATA_WIDTH(DATA_WIDTH),									// Data width of a single force value, 32-bit
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
	Force_Write_Back_Controller
	(
		.clk(clk),
		.rst(rst),
		// Cache input force
		.in_partial_force_valid(in_partial_force_valid),
		.in_particle_id(in_particle_id),
		.in_partial_force(in_partial_force),
		// Cache output force
		.in_read_data_request(in_read_data_request),									// Enables read data from the force cache, if this signal is high, then no write operation is permitted
		.in_cache_read_address(in_cache_read_address),
		.out_partial_force(out_partial_force),
		.out_cache_readout_valid(out_cache_readout_valid)
	);

endmodule
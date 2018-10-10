/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Module: Filter_Bank.v
//
//	Function: Holding multiple filter, perform arbitration to read from multiple available filter buffers
//					Sending selected data to force evaluation pipeline
//
// Dependency:
// 			Filter_Logic.v
//
//
// Created by: Chen Yang 10/10/18
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module Filter_Bank
#(
	parameter DATA_WIDTH 					= 32,
	parameter FILTER_BUFFER_DEPTH 		= 32,
	parameter FILTER_BUFFER_ADDR_WIDTH	= 5,
	parameter CUTOFF_2 						= 32'h43100000						// (12^2=144 in IEEE floating point)
)
(
	input clk,
	input rst,
	
	
	output [DATA_WIDTH-1:0] r2,
	output [DATA_WIDTH-1:0] dx,
	output [DATA_WIDTH-1:0] dy,
	output [DATA_WIDTH-1:0] dz
);


endmodule
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Module: .v
//
//	Function: 
//				
//
//	Purpose:
//				
//
// Data Organization:
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
//				Chen Yang 11/01/18
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module template
#(
	parameter A = 2
)
(
	input clk,
	input rst
);

	wire [A-1:0] B;
	assign B = {(A){1'b0}};


endmodule
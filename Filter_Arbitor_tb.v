/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Module: Filter_Arbitor_tb.v
//
//	Function: Testbench for Filter_Arbitor.v
//
// Dependency:
// 			Filter_Arbitor.v
//
// Created by: Chen Yang 10/12/18
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns

module Filter_Arbitor_tb;
	
	parameter NUM_FILTER = 4;
	parameter MASK_MAX = 15;					// 2^NUM_FILTER - 1
	parameter MAX_RESULT = 8;					// 2^(NUM_FILTER-1)

	reg clk, rst;
	reg [NUM_FILTER-1:0] Filter_Available_Flag;
	wire [NUM_FILTER-1:0] Arbitration_Result;
	
	reg [8:0] counter;
	
	always #1 clk <= ~clk;
	
	always@(posedge clk)
		if(rst)
			begin
			Filter_Available_Flag <= 4'b0000;
			counter <= 0;
			end
		else if(counter < 10)
			begin
			Filter_Available_Flag <= 4'b1111;
			counter <= counter + 1'b1;
			end
		else if(counter < 30)
			begin
			Filter_Available_Flag <= Filter_Available_Flag + 1'b1; 
			counter <= counter + 1'b1;
			end
		else
			begin
			Filter_Available_Flag <= 4'b1000;
			counter <= counter + 1'b1;
			end
	
	initial begin
		clk <= 1'b1;
		rst <= 1'b1;
		
		#10
		rst <= 1'b0;
	end
	
	// UUT
	Filter_Arbitor
	#(
		.NUM_FILTER(NUM_FILTER),
		.MASK_MAX(MASK_MAX),
		.MAX_RESULT(MAX_RESULT)
	)
	Filter_Arbitor
	(
		.clk(clk),
		.rst(rst),
		.Filter_Available_Flag(Filter_Available_Flag),
		.Arbitration_Result(Arbitration_Result)
	);

endmodule
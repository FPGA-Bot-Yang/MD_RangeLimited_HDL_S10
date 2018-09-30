/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Module: Board_Test_Top_RL_Pipeline_1st_Order.v
//
//	Function: Serve as the top module for on-board test
//				The rst and start signal is given by memory modules controlled by in memory content editor
//
// Dependency:
// 			RL_Pipeline_1st_Order.v
//
// Created by: Chen Yang 09/28/18
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module Board_Test_Top_RL_Pipeline_1st_Order
#(
	parameter DATA_WIDTH 				= 32,
	parameter INTERPOLATION_ORDER		= 1,
	parameter SEGMENT_NUM				= 12,
	parameter SEGMENT_WIDTH				= 4,
	parameter BIN_WIDTH					= 8,
	parameter BIN_NUM						= 256,
	parameter LOOKUP_NUM					= SEGMENT_NUM * BIN_NUM,			// SEGMENT_NUM * BIN_NUM
	parameter LOOKUP_ADDR_WIDTH		= SEGMENT_WIDTH + BIN_WIDTH		// log LOOKUP_NUM / log 2
)
(
	input  clk,
	output [DATA_WIDTH-1:0] forceoutput,
	output forceoutput_valid,
	output done
);

	wire rst;
	wire start;
	
	On_Board_Test_Control_RAM_rst CTRL_rst (
		.data    (),    //   input,  width = 1,  ram_input.datain
		.address (1'b0), //   input,  width = 1,           .address
		.wren    (1'b0),    //   input,  width = 1,           .wren
		.clock   (clk),   //   input,  width = 1,           .clk
		.q       (rst)        //  output,  width = 1, ram_output.dataout
	);
	
	On_Board_Test_Control_RAM_start CTRL_start (
		.data    (),    //   input,  width = 1,  ram_input.datain
		.address (1'b0), //   input,  width = 1,           .address
		.wren    (1'b0),    //   input,  width = 1,           .wren
		.clock   (clk),   //   input,  width = 1,           .clk
		.q       (start)        //  output,  width = 1, ram_output.dataout
	);
	


	RL_Pipeline_1st_Order
	#(
		.DATA_WIDTH(DATA_WIDTH),
		.INTERPOLATION_ORDER(INTERPOLATION_ORDER),
		.SEGMENT_NUM(SEGMENT_NUM),
		.SEGMENT_WIDTH(SEGMENT_WIDTH),
		.BIN_WIDTH(BIN_WIDTH),
		.BIN_NUM(BIN_NUM),
		.LOOKUP_NUM(LOOKUP_NUM),
		.LOOKUP_ADDR_WIDTH(LOOKUP_ADDR_WIDTH)
	)
	RL_Pipeline_1st_Order
	(
		.clk(clk),
		.rst(rst),
		.start(start),
		.forceoutput(forceoutput),
		.forceoutput_valid(forceoutput_valid),
		.done(done)
	);
	


endmodule
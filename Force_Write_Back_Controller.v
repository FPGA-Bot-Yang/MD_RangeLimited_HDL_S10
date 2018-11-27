/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Module: Force_Write_Back_Controller.v
//
//	Function: 
//				Receive input from Force_Evaluation_Unit (both reference output and neighbor output), each cell has a independent controller module
//				Perform FORCE ACCUMULATION and WRITE BACK
//				Works on top of force_cache.v
//				If this is the home cell, then receive input from reference particle partial force output (when the neighbor particle is from homecell, then it's discarded since it will be evaluated one more time)
//				If this is a neighbor cell, then receive input from neighbor particle partial force output. (All cells taking input from the same evaluation unit output port, if this is the designated cell, then process; otherwise, discard)
//
// Data Organization:
//				MSB -> LSB: {Force_Z, Force_Y, Force_X}
//
// Format:
//				particle_id [PARTICLE_ID_WIDTH-1:0]:  {cell_x, cell_y, cell_z, particle_in_cell_rd_addr}
//				ref_particle_position [3*DATA_WIDTH-1:0]: {refz, refy, refx}
//				neighbor_particle_position [3*DATA_WIDTH-1:0]: {neighborz, neighbory, neighborx}
//				LJ_Force [3*DATA_WIDTH-1:0]: {LJ_Force_Z, LJ_Force_Y, LJ_Force_X}
//
// Used by:
//				RL_LJ_Top.v
//
// Dependency:
//				force_cache.v
//				FP_ADD.v (latency 3)
//
// Testbench:
//				RL_LJ_Top_tb.v
//
// To do:
//				0, Implement a buffer when there are more than 1 force evaluation units working at the same time. Cause each module may receive partial force from different evaluation units at the same time
//
// Created by: Chen Yang 11/20/2018
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module Force_Write_Back_Controller
#(
	parameter DATA_WIDTH 					= 32,
	// Dell id this unit related to
	parameter CELL_X							= 2,
	parameter CELL_Y							= 2,
	parameter CELL_Z							= 2,
	// Dataset defined parameters
	parameter CELL_ID_WIDTH					= 4,
	parameter MAX_CELL_PARTICLE_NUM		= 290,										// The maximum # of particles can be in a cell
	parameter CELL_ADDR_WIDTH				= 9,											// log(MAX_CELL_PARTICLE_NUM)
	parameter PARTICLE_ID_WIDTH			= CELL_ID_WIDTH*3+CELL_ADDR_WIDTH	// # of bit used to represent particle ID, 9*9*7 cells, each 4-bit, each cell have max of 220 particles, 8-bit
)
(
	input  clk,
	input  rst,
	// Cache input force
	input  in_partial_force_valid,
	input  [PARTICLE_ID_WIDTH-1:0] in_particle_id,
	input  [3*DATA_WIDTH-1:0] in_partial_force,
	// Cache output force
	input  in_read_data_request,																	// Enables read data from the force cache, if this signal is high, then no write operation is permitted
	input  [CELL_ADDR_WIDTH-1:0] in_cache_read_address,
	output reg [3*DATA_WIDTH-1:0] out_partial_force,
	output reg out_cache_readout_valid
);

	//// Signals derived from input
	// Extract the cell id from the incoming particle id
	wire [CELL_ID_WIDTH-1:0] particle_cell_x, particle_cell_y, particle_cell_z;
	assign {particle_cell_x, particle_cell_y, particle_cell_z} = in_particle_id[PARTICLE_ID_WIDTH-1:PARTICLE_ID_WIDTH-3*CELL_ID_WIDTH];

	//// Signals connected to force_cache
	reg  [CELL_ADDR_WIDTH-1:0] cache_rd_address, cache_wr_address;
	wire  [3*DATA_WIDTH-1:0] cache_write_data;
	reg  cache_write_enable;
	wire [3*DATA_WIDTH-1:0] cache_readout_data;
	
	//// Delay registers
	// Conpensate for the 1 cycle delay of reading data out from cache
	reg delay_in_read_data_request;
	// Delay the input force value by 2 cycles: 1 cycle for generating cache read address, 1 cycle for fetching data from cache
	reg [3*DATA_WIDTH-1:0] in_partial_force_reg1;
	reg [3*DATA_WIDTH-1:0] delay_in_partial_force;				// This one connected to accumualtor input
	// Delay the write enable signal by 3 cycles: 1 cycle for fetching data from cache, 2 cycles for waiting addition finish
	reg cache_write_enable_reg1;
	reg cache_write_enable_reg2;
	reg delay_cache_write_enable;
	// Delay the cache write address by 3 cycles: 1 cycle for fetching data from cache, 2 cycles for waiting addition finish
	reg [CELL_ADDR_WIDTH-1:0] cache_wr_address_reg1;
	reg [CELL_ADDR_WIDTH-1:0] cache_wr_address_reg2;
	reg [CELL_ADDR_WIDTH-1:0] delay_cache_wr_address;
	
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Force Cache Controller
	// Since there is a 3 cycle latency for the adder, when there is a particle force being accumulated, while new forces for the same particle arrive, need to wait
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	always@(posedge clk)
		if(rst)
			begin
			// Delay registers
			// For assigning the read out valid
			delay_in_read_data_request <= 1'b0;
			// For conpensating the 2 cycles delay from input force to accumulator
			in_partial_force_reg1 <= 0;
			delay_in_partial_force <= 0;
			// For conpensating the 3 cycles delay from write enable is assigned and accumulated value is calculated
			cache_write_enable_reg1 <= 1'b0;
			cache_write_enable_reg2 <= 1'b0;
			delay_cache_write_enable <= 1'b0;
			// For conpensating the 3 cycles delay from write address is generated and accumulated value is calculated
			cache_wr_address_reg1 <= 0;
			cache_wr_address_reg2 <= 0;
			delay_cache_wr_address <= 0;
			
			// Read output ports
			out_partial_force <= {(3*DATA_WIDTH){1'b0}};
			out_cache_readout_valid <= 1'b0;
			// Cache control signals
			cache_rd_address <= {(CELL_ADDR_WIDTH){1'b0}};
			cache_wr_address <= {(CELL_ADDR_WIDTH){1'b0}};
//			cache_write_data <= {(3*DATA_WIDTH){1'b0}};
			cache_write_enable <= 1'b0;
			end
		else
			begin
			//// Delay registers
			// For assigning the read out valid
			delay_in_read_data_request <= in_read_data_request;
			// For generating the 2 cycles delay from input force to accumulator
			in_partial_force_reg1 <= in_partial_force;
			delay_in_partial_force <= in_partial_force_reg1;
			// For conpensating the 3 cycles delay from write enable is assigned and accumulated value is calculated
			cache_write_enable_reg1 <= cache_write_enable;
			cache_write_enable_reg2 <= cache_write_enable_reg1;
			delay_cache_write_enable <= cache_write_enable_reg2;
			// For conpensating the 3 cycles delay from write address is generated and accumulated value is calculated
			cache_wr_address_reg1 <= cache_wr_address;
			cache_wr_address_reg2 <= cache_wr_address_reg1;
			delay_cache_wr_address <= cache_wr_address_reg2;
			
			//// Priority grant to read request (usually read enable need to keep low during force evaluation process)
			// if outside read request set, then no write activity is permitted
			if(in_read_data_request)
				begin
				// Read output ports
				out_partial_force <= cache_readout_data;
				out_cache_readout_valid <= delay_in_read_data_request;
				// Cache control signals
				cache_rd_address <= in_cache_read_address;
				cache_wr_address <= {(CELL_ADDR_WIDTH){1'b0}};
//				cache_write_data <= {(3*DATA_WIDTH){1'b0}};
				cache_write_enable <= 1'b0;
				end
			//// Accumulation and write into force memory
			else
				begin
				// During force accumulation period, output the data that is being written into the memory
				out_partial_force <= cache_write_data;
				out_cache_readout_valid <= 1'b0;
				// Check if the incoming data is valid and belongs to the current cell
				if(in_partial_force_valid && particle_cell_x == CELL_X && particle_cell_y == CELL_Y && particle_cell_z == CELL_Z)
					begin
					cache_rd_address <= 1;
					cache_wr_address <= in_particle_id[CELL_ADDR_WIDTH-1:0];
					cache_write_enable <= 1'b1;
					end
				else
					begin
					cache_rd_address <= 1;
					cache_wr_address <= 0;
					cache_write_enable <= 1'b0;
					end
				end
			end
	
	////////////////////////////////////////////////////////////////////////////////////////////////
	// Force Accumulator
	////////////////////////////////////////////////////////////////////////////////////////////////
	// Force_X Accumulator
	FP_ADD Force_X_Acc(
		.clk(clk),
		.ena(1'b1),
		.clr(1'b0),
		.ax(cache_readout_data[1*DATA_WIDTH-1:0*DATA_WIDTH]),
		.ay(delay_in_partial_force[1*DATA_WIDTH-1:0*DATA_WIDTH]),
		.result(cache_write_data[1*DATA_WIDTH-1:0*DATA_WIDTH])
	);
	
	// Force_Y Accumulator
	FP_ADD Force_Y_Acc(
		.clk(clk),
		.ena(1'b1),
		.clr(1'b0),
		.ax(cache_readout_data[2*DATA_WIDTH-1:1*DATA_WIDTH]),
		.ay(delay_in_partial_force[2*DATA_WIDTH-1:1*DATA_WIDTH]),
		.result(cache_write_data[2*DATA_WIDTH-1:1*DATA_WIDTH])
	);
	
	// Force_Z Accumulator
	FP_ADD Force_Z_Acc(
		.clk(clk),
		.ena(1'b1),
		.clr(1'b0),
		.ax(cache_readout_data[3*DATA_WIDTH-1:2*DATA_WIDTH]),
		.ay(delay_in_partial_force[3*DATA_WIDTH-1:2*DATA_WIDTH]),
		.result(cache_write_data[3*DATA_WIDTH-1:2*DATA_WIDTH])
	);

	////////////////////////////////////////////////////////////////////////////////////////////////
	// Force Cache
	////////////////////////////////////////////////////////////////////////////////////////////////
	// Dual port ram
	force_cache
	#(
		.DATA_WIDTH(DATA_WIDTH*3),
		.PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),
		.ADDR_WIDTH(CELL_ADDR_WIDTH)
	)
	force_cache
	(
		.clock(clk),
		.data(cache_write_data),
		.rdaddress(cache_rd_address),
		.wraddress(delay_cache_wr_address),
		.wren(delay_cache_write_enable),
		.q(cache_readout_data)
	);

endmodule
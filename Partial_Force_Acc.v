/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Module: Partial_Force_Acc.v
//
//	Function: Accumulate the particle force for a single reference particle
//				Take the partial force from force evaluation module every cycle and perform accumulation
//				When the input particle id changed, which means the accumulation for the current particle has done, then output the accumulated force, and set as valid
//				When particle id change, reset the accumulated value and restart accumulation
//
// Used by:
//				RL_LJ_Evaluation_Unit.v
//
// Dependency:
//				FP_ACC.v
//
// Latency: TBD
//
// Created by: Chen Yang 10/23/18
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module Partial_Force_Acc
#(
	parameter DATA_WIDTH 					= 32,
	parameter PARTICLE_ID_WIDTH			= 20								// # of bit used to represent particle ID, 9*9*7 cells, each 4-bit, each cell have max of 200 particles, 8-bit
)
(
	input  clk,
	input  rst,
	input  in_input_valid,
	input  [PARTICLE_ID_WIDTH-1:0] in_particle_id,
	input  [DATA_WIDTH-1:0] in_partial_force_x,							// in IEEE single precision floating point format
	input  [DATA_WIDTH-1:0] in_partial_force_y,							// in IEEE single precision floating point format
	input  [DATA_WIDTH-1:0] in_partial_force_z,							// in IEEE single precision floating point format
	output reg [PARTICLE_ID_WIDTH-1:0] out_particle_id,
	output reg [DATA_WIDTH-1:0] out_particle_acc_force_x,
	output reg [DATA_WIDTH-1:0] out_particle_acc_force_y,
	output reg [DATA_WIDTH-1:0] out_particle_acc_force_z,
	output reg out_acc_force_valid											// only set as valid when the particle_id changes, which means the accumulation for the current particle is done
);

	reg [PARTICLE_ID_WIDTH-1:0] cur_particle_id;							// Record the particle id for the current accumulated particle
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	// Signals connected to accumulators
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	reg acc_enable;																// Enable the accumulation operation
	// Register the incoming force before sending for accumulation (for alignment with the acc_force and acc_enable)
	reg [DATA_WIDTH-1:0] partial_force_x_reg;
	reg [DATA_WIDTH-1:0] partial_force_y_reg;
	reg [DATA_WIDTH-1:0] partial_force_z_reg;
	// Register the accumulated force (for alignment with the in_partial_force and acc_enable)
	reg [DATA_WIDTH-1:0] acc_force_x_reg;
	reg [DATA_WIDTH-1:0] acc_force_y_reg;
	reg [DATA_WIDTH-1:0] acc_force_z_reg;
	// Accumulated output value
	wire [DATA_WIDTH-1:0] acc_value_out_x;									
	wire [DATA_WIDTH-1:0] acc_value_out_y;
	wire [DATA_WIDTH-1:0] acc_value_out_z;

	// Controller for accumulation operation
	always@(posedge clk)
		begin
		if(rst)
			begin
			cur_particle_id <= 0;
			// Accumulator signals
			acc_enable <= 1'b0;
			partial_force_x_reg <= 0;
			partial_force_y_reg <= 0;
			partial_force_z_reg <= 0;
			acc_force_x_reg <= 0;
			acc_force_y_reg <= 0;
			acc_force_z_reg <= 0;
			// Output signal
			out_particle_id <= 0;
			out_particle_acc_force_x <= 0;
			out_particle_acc_force_y <= 0;
			out_particle_acc_force_z <= 0;
			out_acc_force_valid <= 1'b0;
			end
		else
			begin
			// Always enable the accumulator
			// If the input value is invalid, then set the incoming value as 0
			acc_enable <= 1'b1;
				
			////////////////////////////////////////////////////
			// Assign the particle force to be accumulated
			////////////////////////////////////////////////////
			if(in_input_valid)
				begin
				partial_force_x_reg <= in_partial_force_x;
				partial_force_y_reg <= in_partial_force_y;
				partial_force_z_reg <= in_partial_force_z;
				end
			else
				begin
				partial_force_x_reg <= 0;
				partial_force_y_reg <= 0;
				partial_force_z_reg <= 0;
				end
			
			////////////////////////////////////////////////////
			// Register the accumulated force
			////////////////////////////////////////////////////
			if(cur_particle_id == in_particle_id)
				begin
				cur_particle_id <= cur_particle_id;
				acc_force_x_reg <= acc_value_out_x;
				acc_force_y_reg <= acc_value_out_y;
				acc_force_z_reg <= acc_value_out_z;
				end
			else
				begin
				cur_particle_id <= in_particle_id;			// update the particle id
				acc_force_x_reg <= 0;
				acc_force_y_reg <= 0;
				acc_force_z_reg <= 0;
				end
			
			////////////////////////////////////////////////////
			// Assign the accumulated output forces
			////////////////////////////////////////////////////
			// When the particle id changes, assign the valid output register
			out_particle_id <= cur_particle_id;
			out_particle_acc_force_x <= acc_value_out_x;
			out_particle_acc_force_y <= acc_value_out_y;
			out_particle_acc_force_z <= acc_value_out_z;
			if(cur_particle_id != in_particle_id)
				begin
				out_acc_force_valid <= 1'b1;
				end
			else
				begin
				out_acc_force_valid <= 1'b0;
				end
				
			end
		end
		
	// Acc_Value_X
	FP_ACC
	#(
		.DATA_WIDTH(DATA_WIDTH)
	)
	FP_ACC_X (
		.clk(clk),
		.clr(rst),
		.ena(acc_enable),
		.ax(partial_force_x_reg),
		.ay(acc_force_x_reg),
		.result(acc_value_out_x)
	);
	
	// Acc_Value_Y
	FP_ACC
	#(
		.DATA_WIDTH(DATA_WIDTH)
	)
	FP_ACC_Y (
		.clk(clk),
		.clr(rst),
		.ena(acc_enable),
		.ax(partial_force_y_reg),
		.ay(acc_force_y_reg),
		.result(acc_value_out_y)
	);
	
	// Acc_Value_Z
	FP_ACC
	#(
		.DATA_WIDTH(DATA_WIDTH)
	)
	FP_ACC_Z (
		.clk(clk),
		.clr(rst),
		.ena(acc_enable),
		.ax(partial_force_z_reg),
		.ay(acc_force_z_reg),
		.result(acc_value_out_z)
	);
		
endmodule
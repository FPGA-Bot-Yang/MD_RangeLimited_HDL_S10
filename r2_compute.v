/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Module: r2_compute.v
//
//	Function: Evaluate the r2 value between the reference particle and neighbor particle based on the input coordinate
//
// Dependency:
// 			FP_MUL.v
//				FP_SUB.v
//
// Latency: total: 17 cycles
//				Level 1: calculate dx, dy, dz (SUB)								3 cycles
//				Level 2: calculate x2 = dx * dx (MUL)							4 cycles
//				Level 3: calculate (x2 + y2) = (x2) + dy * dy (MUL_ADD)	5 cycles
//				Level 4: calculate (x2 + y2) + dz * dz (MUL_ADD)			5 cycles
//
//
// Created by: Tony Geng 03/01/18
// Update by: Chen Yang 05/21/18
//					Change the date type from interger to singlde precision floating poin
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module r2_compute
#(parameter DATA_WIDTH = 32)
(
	input clk,
	input rst,
	input enable,
	input [DATA_WIDTH-1:0] refx,
	input [DATA_WIDTH-1:0] refy,
	input [DATA_WIDTH-1:0] refz,
	input [DATA_WIDTH-1:0] posx,
	input [DATA_WIDTH-1:0] posy,
	input [DATA_WIDTH-1:0] posz,
	output [DATA_WIDTH-1:0] r2,
	output reg r2_valid
);
	
	wire [DATA_WIDTH-1:0] dx;
	wire [DATA_WIDTH-1:0] dy;
	wire [DATA_WIDTH-1:0] dz;
	wire [DATA_WIDTH-1:0] dx2;
	wire [DATA_WIDTH-1:0] r2_partial;
	
	// Delay the dy input before the x2 is calculated (4 cycles)
	reg [DATA_WIDTH-1:0] dy_reg1;
	reg [DATA_WIDTH-1:0] dy_reg2;
	reg [DATA_WIDTH-1:0] dy_reg3;
	reg [DATA_WIDTH-1:0] dy_delay;
	
	// Delay the dz input before the x2+y2 is calculated (4+5 cycles)
	reg [DATA_WIDTH-1:0] dz_reg1;
	reg [DATA_WIDTH-1:0] dz_reg2;
	reg [DATA_WIDTH-1:0] dz_reg3;
	reg [DATA_WIDTH-1:0] dz_reg4;
	reg [DATA_WIDTH-1:0] dz_reg5;
	reg [DATA_WIDTH-1:0] dz_reg6;
	reg [DATA_WIDTH-1:0] dz_reg7;
	reg [DATA_WIDTH-1:0] dz_reg8;
	reg [DATA_WIDTH-1:0] dz_delay;
	
	
	wire level1_en;			// Calculate dx, dy, dz
	reg level2_en;				// Calculate dx2
	reg level3_en;				// Calculate dx2 + dy2
	reg level4_en;				// Calculate r2 = dx2 + dy2 + dz2
	reg level1_en_reg1;
	reg level1_en_reg2;
	reg level2_en_reg1;
	reg level2_en_reg2;
	reg level2_en_reg3;
	reg level3_en_reg1;
	reg level3_en_reg2;
	reg level3_en_reg3;
	reg level3_en_reg4;
	reg level4_en_reg1;
	reg level4_en_reg2;
	reg level4_en_reg3;
	reg level4_en_reg4;
	
	assign level1_en = enable;
	
	always@(posedge clk)
		begin
		if(rst)
			begin
			level2_en <= 1'b0;
			level3_en <= 1'b0;
			level4_en <= 1'b0;
			r2_valid <= 1'b0;
			// delay registers to propagate the enable signal of FP IP units
			level1_en_reg1 <= 1'b0;
			level1_en_reg2 <= 1'b0;
			level2_en_reg1 <= 1'b0;
			level2_en_reg2 <= 1'b0;
			level2_en_reg3 <= 1'b0;
			level3_en_reg1 <= 1'b0;
			level3_en_reg2 <= 1'b0;
			level3_en_reg3 <= 1'b0;
			level3_en_reg4 <= 1'b0;
			level4_en_reg1 <= 1'b0;
			level4_en_reg2 <= 1'b0;
			level4_en_reg3 <= 1'b0;
			level4_en_reg4 <= 1'b0;
			// delay registers to propage the dy value before x2 is finished calculation
			dy_reg1 <= 0;
			dy_reg2 <= 0;
			dy_reg3 <= 0;
			dy_delay <= 0;
			// delay registers to propage the dz value before x2+y2 is finished calculation
			dz_reg1 <= 0;
			dz_reg2 <= 0;
			dz_reg3 <= 0;
			dz_reg4 <= 0;
			dz_reg5 <= 0;
			dz_reg6 <= 0;
			dz_reg7 <= 0;
			dz_reg8 <= 0;
			dz_delay <= 0;
			end
		else
			begin	
			level1_en_reg1 <= level1_en;
			level1_en_reg2 <= level1_en_reg1;
			
			level2_en <= level1_en_reg2;
			level2_en_reg1 <= level2_en;
			level2_en_reg2 <= level2_en_reg1;
			level2_en_reg3 <= level2_en_reg2;
			
			level3_en <= level2_en_reg3;
			level3_en_reg1 <= level3_en;
			level3_en_reg2 <= level3_en_reg1;
			level3_en_reg3 <= level3_en_reg2;
			level3_en_reg4 <= level3_en_reg3;
			
			level4_en <= level3_en_reg4;
			level4_en_reg1 <= level4_en;
			level4_en_reg2 <= level4_en_reg1;
			level4_en_reg3 <= level4_en_reg2;
			level4_en_reg4 <= level4_en_reg3;
			
			r2_valid <= level4_en_reg4;
			
			// delay registers to propage the dy value before x2 is finished calculation
			dy_reg1 <= dy;
			dy_reg2 <= dy_reg1;
			dy_reg3 <= dy_reg2;
			dy_delay <= dy_reg3;
			// delay registers to propage the dz value before x2+y2 is finished calculation
			dz_reg1 <= dz;
			dz_reg2 <= dz_reg1;
			dz_reg3 <= dz_reg2;
			dz_reg4 <= dz_reg3;
			dz_reg5 <= dz_reg4;
			dz_reg6 <= dz_reg5;
			dz_reg7 <= dz_reg6;
			dz_reg8 <= dz_reg7;
			dz_delay <= dz_reg8;
			end
		end

	// dx = refx - posx
	// 3 cycle delay
	FP_SUB FP_SUB_diff_x (
		.ax     (refx),     		//   input,  width = 32,     ax.ax
		.ay     (posx),     		//   input,  width = 32,     ay.ay
		.clk    (clk),    		//   input,   width = 1,    clk.clk
		.clr    (rst),    		//   input,   width = 2,    clr.clr
		.ena    (level1_en),    //   input,   width = 1,    ena.ena
		.result (dx)  				//  output,  width = 32, result.result
	);
	
	// dy = refy - posy
	// 3 cycle delay
	FP_SUB FP_SUB_diff_y (
		.ax     (refy),     		//   input,  width = 32,     ax.ax
		.ay     (posy),     		//   input,  width = 32,     ay.ay
		.clk    (clk),    		//   input,   width = 1,    clk.clk
		.clr    (rst),    		//   input,   width = 2,    clr.clr
		.ena    (level1_en),    //   input,   width = 1,    ena.ena
		.result (dy)  				//  output,  width = 32, result.result
	);
	
	// dz = refz - posz
	// 3 cycle delay
	FP_SUB FP_SUB_diff_z (
		.ax     (refz),     		//   input,  width = 32,     ax.ax
		.ay     (posz),     		//   input,  width = 32,     ay.ay
		.clk    (clk),    		//   input,   width = 1,    clk.clk
		.clr    (rst),    		//   input,   width = 2,    clr.clr
		.ena    (level1_en),    //   input,   width = 1,    ena.ena
		.result (dz)  				//  output,  width = 32, result.result
	);
	
	// dx2 = dx * dx
	// 4 cycle delay
	FP_MUL FP_MUL_Coulomb_Force (
		.ay(dx),     				//     ay.ay
		.az(dx),     				//     az.az
		.clk(clk),    				//    clk.clk
		.clr(rst),    				//    clr.clr
		.ena(level2_en),    		//    ena.ena
		.result(dx2)  				// result.result
	);
	
	// partial = dx2 + dy2
	// 5 cycle delay
	FP_MUL_ADD FP_MUL_ADD_Partial_r2 (
		.ax     (dx2), 			//   input,  width = 32,     ax.ax
		.ay     (dy_delay),     	//   input,  width = 32,     ay.ay
		.az     (dy_delay),     	//   input,  width = 32,     az.az
		.clk    (clk),          //   input,   width = 1,    clk.clk
		.clr    (rst),          //   input,   width = 2,    clr.clr
		.ena    (level3_en),    //   input,   width = 1,    ena.ena
		.result (r2_partial)  	//   output,  width = 32, result.result
	);
	
	// r2 = dx2 + dy2 + dz2
	// 5 cycle delay
	FP_MUL_ADD FP_MUL_ADD_final_r2 (
		.ax     (r2_partial), 	//   input,  width = 32,     ax.ax
		.ay     (dz_delay),     	//   input,  width = 32,     ay.ay
		.az     (dz_delay),     	//   input,  width = 32,     az.az
		.clk    (clk),          //   input,   width = 1,    clk.clk
		.clr    (rst),          //   input,   width = 2,    clr.clr
		.ena    (level4_en),    //   input,   width = 1,    ena.ena
		.result (r2)  				//   output,  width = 32, result.result
	);
 

endmodule


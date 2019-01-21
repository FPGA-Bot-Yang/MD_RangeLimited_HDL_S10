#include <fstream>
#include <math.h>
#include <ctime>
#include <string>
#include <stdlib.h>

#include "Gen_Sim_top.h"

using namespace std;

int Gen_Sim_Top(int num_cell_x, int num_cell_y, int num_cell_z, std::string* common_path){

	// Setup Generating file
	char filename[100];
	sprintf(filename,"RL_LJ_Intergrated_Top.v");

	std::string path = *common_path + "/" + std::string(filename);
	std::ofstream fout;
	fout.open(path.c_str());
	if (fout.fail()){
		printf("open %s failed, exiting\n", path.c_str());
		exit(-1);
	}
	
	fout << "/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////\n";
	fout << "// Module: RL_LJ_Intergrated_Top.v\n";
	fout << "//\n";
	fout << "//	Function:\n";
	fout << "//\t\t\t\tIntegrated top module holding all the cells in the simulation space\n";
	fout << "//\n";
	fout << "//	Purpose:\n";
	fout << "//\t\t\t\tFor real simulation with all the dataset\n";
	fout << "//\t\t\t\tTiming testing with the integrated system\n";
	fout << "//\n";
	fout << "// Data Organization:\n";
	fout << "//\t\t\t\tAddress 0 for each cell module: # of particles in the cell.\n";
	fout << "//\t\t\t\tMSB-LSB: {posz, posy, posx}\n";
	fout << "//\n";
	fout << "// Used by:\n";
	fout << "//\t\t\t\tN\A\n";
	fout << "//\n";
	fout << "// Dependency:\n";
	fout << "//\t\t\t\tRL_LJ_Evaluation_Unit.v\n";
	fout << "//\t\t\t\tParticle_Pair_Gen_HalfShell.v\n";
	fout << "//\t\t\t\tMotion_Update.v\n";
	fout << "//\t\t\t\tcell_x_y_z.v\n";
	fout << "//\t\t\t\tForce_Cache_x_y_z.v\n";
	fout << "//\t\t\t\tVelocity_Cache_x_y_z.v\n";
	fout << "//\n";
	fout << "// Testbench:\n";
	fout << "//\t\t\t\tTBD\n";
	fout << "//\n";
	fout << "// Timing:\n";
	fout << "//\t\t\t\tTBD\n";
	fout << "//\n";
	fout << "// Created by:\n";
	fout << "//\t\t\t\tChen Yang's Script (Gen_Sim_Top.cpp)\n";
	fout << "/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////\n\n";

	fout << "module RL_LJ_Intergrated_Top\n";
	fout << "#(\n";
	fout << "\tparameter DATA_WIDTH 					= 32,\n";
	fout << "\t// Simulation parameters\n";
	fout << "\tparameter TIME_STEP 						= 32'h27101D7D,							// 2fs time step\n";
	fout << "\t// The home cell this unit is working on\n";
	fout << "\tparameter CELL_X							= 4'd2,\n";
	fout << "\tparameter CELL_Y							= 4'd2,\n";
	fout << "\tparameter CELL_Z							= 4'd2,\n";
	fout << "\t// High level parameters\n";
	fout << "\tparameter NUM_EVAL_UNIT					= 1,									// # of evaluation units in the design\n";
	fout << "\t// Dataset defined parameters";
	fout << "\tparameter MAX_CELL_COUNT_PER_DIM 		= 5,									// Maximum cell count among the 3 dimensions\n";
	fout << "\tparameter NUM_NEIGHBOR_CELLS				= 13,									// # of neighbor cells per home cell, for Half-shell method, is 13\n";
	fout << "\tparameter CELL_ID_WIDTH					= 3,									// log(NUM_NEIGHBOR_CELLS)\n";
	fout << "\tparameter MAX_CELL_PARTICLE_NUM			= 290,									// The maximum # of particles can be in a cell\n";
	fout << "\tparameter CELL_ADDR_WIDTH				= 9,									// log(MAX_CELL_PARTICLE_NUM)\n";
	fout << "\tparameter PARTICLE_ID_WIDTH				= CELL_ID_WIDTH*3+CELL_ADDR_WIDTH,		// # of bit used to represent particle ID, 9*9*7 cells, each 4-bit, each cell have max of 220 particles, 8-bit\n";
	fout << "\t// Filter parameters";
	fout << "\tparameter NUM_FILTER						= 8,		//4\n";
	fout << "\tparameter ARBITER_MSB 					= 128,	//8								// 2^(NUM_FILTER-1)\n";
	fout << "\tparameter FILTER_BUFFER_DEPTH 			= 32,\n";
	fout << "\tparameter FILTER_BUFFER_ADDR_WIDTH		= 5,\n";
	fout << "\tparameter CUTOFF_2 						= 32'h43100000,							// (12^2=144 in IEEE floating point)\n";
	fout << "\t// Force Evaluation parameters";
	fout << "\tparameter SEGMENT_NUM					= 14,\n";
	fout << "\tparameter SEGMENT_WIDTH					= 4,\n";
	fout << "\tparameter BIN_NUM						= 256,\n";
	fout << "\tparameter BIN_WIDTH						= 8,\n";
	fout << "\tparameter LOOKUP_NUM						= SEGMENT_NUM * BIN_NUM,				// SEGMENT_NUM * BIN_NUM\n";
	fout << "\tparameter LOOKUP_ADDR_WIDTH				= SEGMENT_WIDTH + BIN_WIDTH,			// log LOOKUP_NUM / log 2\n";
	fout << "\t// Force (accmulation) cache parameters";
	fout << "\tparameter FORCE_CACHE_BUFFER_DEPTH		= 16,									// Force cache input buffer depth, for partial force accumulation\n";
	fout << "\tparameter FORCE_CACHE_BUFFER_ADDR_WIDTH	= 4										// log(FORCE_CACHE_BUFFER_DEPTH) / log 2\n";
	fout << ")\n";
	fout << "(\n";
	fout << "\tinput clk,\n";
	fout << "\tinput rst,\n";
	fout << "\tinput start,\n";
	fout << "\t// These are all temp output ports";
	fout << "\toutput [NUM_EVAL_UNIT*PARTICLE_ID_WIDTH-1:0] ref_particle_id,\n";
	fout << "\toutput [NUM_EVAL_UNIT*DATA_WIDTH-1:0] ref_LJ_Force_X,\n";
	fout << "\toutput [NUM_EVAL_UNIT*DATA_WIDTH-1:0] ref_LJ_Force_Y,\n";
	fout << "\toutput [NUM_EVAL_UNIT*DATA_WIDTH-1:0] ref_LJ_Force_Z,\n";
	fout << "\toutput [NUM_EVAL_UNIT-1:0] ref_forceoutput_valid,\n";
	fout << "\toutput [NUM_EVAL_UNIT*PARTICLE_ID_WIDTH-1:0] neighbor_particle_id,\n";
	fout << "\toutput [NUM_EVAL_UNIT*DATA_WIDTH-1:0] neighbor_LJ_Force_X,\n";
	fout << "\toutput [NUM_EVAL_UNIT*DATA_WIDTH-1:0] neighbor_LJ_Force_Y,\n";
	fout << "\toutput [NUM_EVAL_UNIT*DATA_WIDTH-1:0] neighbor_LJ_Force_Z,\n";
	fout << "\toutput [NUM_EVAL_UNIT-1:0] neighbor_forceoutput_valid,\n";
	fout << "\t// Done signals\n";
	fout << "\t// When entire home cell is done processing, this will keep high until the next time 'start' signal turn high\n";
	fout << "\toutput out_home_cell_evaluation_done,\n";
	fout << "\t// When motion update is done processing, remain high until the next motion update starts\n";
	fout << "\toutput out_motion_update_done,\n";
	fout << "\t// Dummy signal selecting which cell is the home cell\n";
	fout << "\t// Need a logic to replace this, generate by out_Motion_Update_cur_cell\n";
	fout << "\tinput [3:0] in_sel,\n";
	fout << "\t// Dummy output for motion update\n";
	fout << "\toutput [3*CELL_ID_WIDTH-1:0] out_Motion_Update_cur_cell\n";
	fout << ");\n\n";

	// Instantiate the position cache
	for(int cell_x = 1; cell_x <= num_cell_x; cell_x++){
		for(int cell_y = 1; cell_y <= num_cell_y; cell_y++){
			for(int cell_z = 1; cell_z <= num_cell_z; cell_z++){
				fout << "\tPos_Cache_"<< cell_x << "_" << cell_y << "_" << cell_z << "\n";
				fout << "\t#(\n";
				fout << "\t\t.DATA_WIDTH(DATA_WIDTH),\n";
				fout << "\t\t.PARTICLE_NUM(MAX_CELL_PARTICLE_NUM),\n";
				fout << "\t\t.ADDR_WIDTH(CELL_ADDR_WIDTH),\n";
				fout << "\t\t.CELL_ID_WIDTH(CELL_ID_WIDTH),\n";
				fout << "\t\t.CELL_X("<<cell_x<<"),\n";
				fout << "\t\t.CELL_X("<<cell_y<<"),\n";
				fout << "\t\t.CELL_X("<<cell_z<<"),\n";
				fout << "\t)\n";
				fout << "\tcell_"<< cell_x << "_" << cell_y << "_" << cell_z << "\n";
				fout << "\t(\n";
				fout << "\t\t.clk(clk),\n";
				fout << "\t\t.rst(rst),\n";
				fout << "\t\t.motion_update_enable(Motion_Update_enable),				// Keep this signal as high during the motion update process\n";
				fout << "\t\t.in_read_address(Motion_Update_enable ? Motion_Update_position_read_addr : FSM_to_Cell_read_addr[1*CELL_ADDR_WIDTH-1:0*CELL_ADDR_WIDTH]),\n";
				fout << "\t\t.in_data(Motion_Update_out_position_data),\n";
				fout << "\t\t.in_data_dst_cell(Motion_Update_dst_cell),					// The destination cell for the incoming data\n";
				fout << "\t\t.in_data_valid(Motion_Update_out_position_data_valid),						// Signify if the new incoming data is valid\n";
				fout << "\t\t.in_rden(Motion_Update_enable ? Motion_Update_position_read_en : FSM_to_Cell_rden),\n";
				fout << "\t\t.out_particle_info(Position_Cache_readout_position[1*3*DATA_WIDTH-1:0*3*DATA_WIDTH])\n";
				fout << "\t)\n\n";
			}
		}
	}

	fout << "endmodule\n";

	fout.close();

	return 1;
}

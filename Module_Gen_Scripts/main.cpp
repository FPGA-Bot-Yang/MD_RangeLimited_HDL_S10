#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string>
#include <fstream>
#include <ctime>

#include "homecell.h"
#include "Gen_Cell_Mem.h"
#include "Gen_Lookup_Mem.h"
#include "Gen_Sim_Script.h"

// The full path of the Quartus project root folder
std::string common_path = "F:/Dropbox/CAAD_Server/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0";
// The relative path where the simulation script will be generated
std::string sim_script_out_path = "/mentor";
// The full path where the source code is organized
std::string common_src_path = common_path + "/SourceCode";
int BIN_NUM = 256;
int SEG_NUM = 14;
int HOME_CELL_X = 2;
int HOME_CELL_Y = 2;
int HOME_CELL_Z = 2;
int INTERPOLATION_ORDER = 1;


int main(){
	/////////////////////////////////////////////////////////////////////////
	// Generate Cell Memory module
	/////////////////////////////////////////////////////////////////////////
	// Gen Home cell
	Gen_Cell_Mem(HOME_CELL_X, HOME_CELL_Y, HOME_CELL_Z, &common_src_path);
	// Gen 13 neighbor cells
	Gen_Cell_Mem(HOME_CELL_X, HOME_CELL_Y, HOME_CELL_Z+1, &common_src_path);
	Gen_Cell_Mem(HOME_CELL_X, HOME_CELL_Y+1, HOME_CELL_Z-1, &common_src_path);
	Gen_Cell_Mem(HOME_CELL_X, HOME_CELL_Y+1, HOME_CELL_Z, &common_src_path);
	Gen_Cell_Mem(HOME_CELL_X, HOME_CELL_Y+1, HOME_CELL_Z+1, &common_src_path);
	Gen_Cell_Mem(HOME_CELL_X+1, HOME_CELL_Y-1, HOME_CELL_Z-1, &common_src_path);
	Gen_Cell_Mem(HOME_CELL_X+1, HOME_CELL_Y-1, HOME_CELL_Z, &common_src_path);
	Gen_Cell_Mem(HOME_CELL_X+1, HOME_CELL_Y-1, HOME_CELL_Z+1, &common_src_path);
	Gen_Cell_Mem(HOME_CELL_X+1, HOME_CELL_Y, HOME_CELL_Z-1, &common_src_path);
	Gen_Cell_Mem(HOME_CELL_X+1, HOME_CELL_Y, HOME_CELL_Z, &common_src_path);
	Gen_Cell_Mem(HOME_CELL_X+1, HOME_CELL_Y, HOME_CELL_Z+1, &common_src_path);
	Gen_Cell_Mem(HOME_CELL_X+1, HOME_CELL_Y+1, HOME_CELL_Z-1, &common_src_path);
	Gen_Cell_Mem(HOME_CELL_X+1, HOME_CELL_Y+1, HOME_CELL_Z, &common_src_path);
	Gen_Cell_Mem(HOME_CELL_X+1, HOME_CELL_Y+1, HOME_CELL_Z+1, &common_src_path);
	
	/////////////////////////////////////////////////////////////////////////
	// Generate Interpolation Memory module
	/////////////////////////////////////////////////////////////////////////
	if(INTERPOLATION_ORDER >= 0){
		Gen_Lookup_Mem(0, 3, BIN_NUM, SEG_NUM, &common_src_path);
		Gen_Lookup_Mem(0, 8, BIN_NUM, SEG_NUM, &common_src_path);
		Gen_Lookup_Mem(0, 14, BIN_NUM, SEG_NUM, &common_src_path);
	}
	if(INTERPOLATION_ORDER >= 1){
		Gen_Lookup_Mem(1, 3, BIN_NUM, SEG_NUM, &common_src_path);
		Gen_Lookup_Mem(1, 8, BIN_NUM, SEG_NUM, &common_src_path);
		Gen_Lookup_Mem(1, 14, BIN_NUM, SEG_NUM, &common_src_path);
	}
	if(INTERPOLATION_ORDER >= 2){
		Gen_Lookup_Mem(2, 3, BIN_NUM, SEG_NUM, &common_src_path);
		Gen_Lookup_Mem(2, 8, BIN_NUM, SEG_NUM, &common_src_path);
		Gen_Lookup_Mem(2, 14, BIN_NUM, SEG_NUM, &common_src_path);
	}

	/////////////////////////////////////////////////////////////////////////
	// Generate Simulation Script (.do file)
	/////////////////////////////////////////////////////////////////////////
	// Make a copy to simulation path
	std::string tmp_script_path = "/SourceCode";
	Gen_Sim_Script(&common_path, &tmp_script_path);
	// Make a copy to SourceCode path
	Gen_Sim_Script(&common_path, &sim_script_out_path);

	return 0;
}
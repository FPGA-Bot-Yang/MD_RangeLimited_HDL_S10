# # QSYS_SIMDIR is used in the Quartus-generated IP simulation script to
# # construct paths to the files required to simulate the IP in your Quartus
# # project. By default, the IP script assumes that you are launching the
# # simulator from the IP script location. If launching from another
# # location, set QSYS_SIMDIR to the output directory you specified when you
# # generated the IP script, relative to the directory from which you launch
# # the simulator.
# #
 set QSYS_SIMDIR /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0

# #
# # Source the generated IP simulation script.
 source $QSYS_SIMDIR/mentor/msim_setup.tcl
# #
# # Set any compilation options you require (this is unusual).
# set USER_DEFINED_COMPILE_OPTIONS <compilation options>
# set USER_DEFINED_VHDL_COMPILE_OPTIONS <compilation options for VHDL>
# set USER_DEFINED_VERILOG_COMPILE_OPTIONS <compilation options for Verilog>
# #
# # Call command to compile the Quartus EDA simulation library.
 dev_com
# #
# # Call command to compile the Quartus-generated IP simulation files.
 com
# #
# # Add commands to compile all design files and testbench files, including
# # the top level. (These are all the files required for simulation other
# # than the files compiled by the Quartus-generated IP simulation script)
# #
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/define.v

 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/fp_accumulation_test.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/fp_accumulation_test_tb.v
 
 #vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/RL_Pipeline_1st_Order_.v
 #vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/RL_Pipeline_1st_Order_tb.v
 #vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/RL_Evaluate_Pairs_1st_Order.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/RL_LJ_Top_Raw_Data_Testing.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/RL_LJ_Top_Raw_Data_Testing_tb.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/RL_LJ_Top.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/RL_LJ_Top_tb.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/RL_LJ_Evaluation_Unit.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/RL_LJ_Force_Evaluation_Unit.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/Partial_Force_Acc.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/Partial_Force_Acc_tb.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/Filter_Arbiter.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/Filter_Arbiter_tb.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/Filter_Bank.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/Filter_Logic.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/Filter_Buffer.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/RL_LJ_Pipeline_1st_Order_no_filter.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/RL_LJ_Pipeline_1st_Order_no_filter_tb.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/RL_LJ_Evaluate_Pairs_1st_Order.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/RL_LJ_Evaluate_Pairs_1st_Order_tb.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/r2_compute.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/r2_compute_tb.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/FP_ADD.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/FP_ACC.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/FP_MUL.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/FP_MUL_ADD.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/FP_SUB.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/ram_ref_x.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/ram_ref_y.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/ram_ref_z.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/ram_neighbor_x.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/ram_neighbor_y.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/ram_neighbor_z.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/lut0_14.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/lut1_14.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/lut0_8.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/lut1_8.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/lut0_3.v
 vlog -vlog01compat -work work /home/jiayi/EthanWorkingDir/MD_RL_Pipeline/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/lut1_3.v


# #
# # Set the top-level simulation or testbench module/entity name, which is
# # used by the elab command to elaborate the top level.
# #
 set TOP_LEVEL_NAME RL_LJ_Top_Raw_Data_Testing_tb
# #
# # Set any elaboration options you require.
# set USER_DEFINED_ELAB_OPTIONS <elaboration options>
# #
# # Call command to elaborate your design and testbench.
 elab
# #
# # Run the simulation.
 add wave *
 view structure
 view signals

# add wave -position end  sim:/RL_Pipeline_1st_Order_tb/UUT/r2_evaluate/FP_SUB_diff_x/ax
# add wave -position end  sim:/RL_Pipeline_1st_Order_tb/UUT/r2_evaluate/FP_SUB_diff_x/ay
# add wave -position end  sim:/RL_Pipeline_1st_Order_tb/UUT/r2_evaluate/FP_SUB_diff_x/clk
# add wave -position end  sim:/RL_Pipeline_1st_Order_tb/UUT/r2_evaluate/FP_SUB_diff_x/clr
# add wave -position end  sim:/RL_Pipeline_1st_Order_tb/UUT/r2_evaluate/FP_SUB_diff_x/ena
# add wave -position end  sim:/RL_Pipeline_1st_Order_tb/UUT/r2_evaluate/FP_SUB_diff_x/result

 radix hex
 run 300ns
# #
# # Report success to the shell.
# exit -code 0
# #

These sets of scripts will do:
1, Generate input particle initilization files for on-chip particle memory modules (based on ApoA1 dataset)
2, Generate interpolation tables for 1st, 2nd, 3rd order interpolation
3, Generate verification file for simulation in HEX format (REFERENCE_OUTPUT.txt)

To use this:
1, Run LJ_no_smooth_poly_interpolation_accuracy.m: to generate the table lookup form (c0_8.txt, etc. and c0_8.mif, etc)
2, Convert the .mif file to .hex using Quartus (open the .mif file in quartus and save as .hex)
3, Before run simulation, initialize the lut modules with the c0_8.hex and etc. files
4, Run GenInputPositionFile_ApoA1.m: genearte the position data (particle_neighbor_x.mif, etc.)
5, Convert the position .mif file to .hex
6, Initilize the position module with .hex file
7, Run Simulation_Verification_LJ_no_smooth.m: This will generate REFERENCE_OUTPUT.txt, which in HEX format, use this directly to compare with the simulation waveform
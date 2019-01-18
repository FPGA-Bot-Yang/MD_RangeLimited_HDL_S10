#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string>
#include <fstream>
#include <ctime>

int MD_Evaluation_Model(){

	// Argon
	float kb = 1.380e-23;									// Boltzmann constant (J/K)
	float Nav = 6.022e23;									// Avogadro constant, # of atoms per mol
	float Ar_weight = 39.95;								// g/mol value of Argon atom
	float EPS = 0.996;										// Unit: kJ
	float SIGMA = 3.35;//3.4;								// Unit Angstrom
	float SIGMA_12 = pow(SIGMA,12);
	float SIGMA_6 = pow(SIGMA,6);
	float MASS = Ar_weight / Nav / 1000;					// Unit kg
	float SIMULATION_TIME_STEP = 2E-15;						// 2 femtosecond
	float CUTOFF_RADIUS = 8;								// Unit Angstrom, Cutoff Radius
	float CUTOFF_RADIUS_2 = CUTOFF_RADIUS*CUTOFF_RADIUS;    // Cutoff distance square
	// Dataset parameters
	float bounding_box_x = 24;
	float bounding_box_y = 24;
	float bounding_box_z = 24;
	int TOTAL_PARTICLE_NUM = 500;
	std::string input_file_path = "C:/Users/Ethan/Desktop/WorkingFolder/Ethan_RL_Pipeline_1st_Order_SingleFloat_18.0/SourceCode/MatlabScripts/";
	std::string input_file_name = "ar_gas.pdb";
	std::string input_file_read_path = input_file_path + input_file_name;

	// Particle Array
	float particle[500][8];									// 0~2: posx, posy, posz; 3~5: vx, vy, vz; 6: LJ Potential Energy; 7: Kinetic Energy
	// System energy
	float System_Energy = 0;

	// Middle Variables
	float dx, dy, dz;
	float ref_x, ref_y, ref_z;
	float r2, inv_r2, inv_r8, inv_r14, inv_r6, inv_r12;
	float neighbor_x, neighbor_y, neighbor_z;
	float vdw12, vdw6, vdw14, vdw8;
	float Fvdw, Evdw;
	float Eketic;
	float Evdw_acc, Fx_acc, Fy_acc, Fz_acc;
	float acceleration_x, acceleration_y, acceleration_z;
	float vx, vy, vz;
	
	
	// Readin particle information
	std::ifstream file(input_file_read_path);
	std::string str;
	float tmp_x, tmp_y, tmp_z;
	// Discard the unwanted lines
	for(int i = 0; i < 5; i++){
		std::getline(file, str);
	}
	for(int i = 0; i < TOTAL_PARTICLE_NUM; i++){
		std::getline(file, str);
		sscanf(str.c_str(), "%*s %*s %*s %*s %*s %*s %f %f %f", &tmp_x, &tmp_y, &tmp_z);
		particle[i][0] = tmp_x;
		particle[i][1] = tmp_y;
		particle[i][2] = tmp_z;
	};

	// Traverse all the particles in the simualtion space
	for(int ref_ptr = 0; ref_ptr < TOTAL_PARTICLE_NUM; ref_ptr++){
		Evdw_acc = 0;
		Fx_acc = 0;
		Fy_acc = 0;
		Fz_acc = 0;
		ref_x = particle[ref_ptr][0];
		ref_y = particle[ref_ptr][1];
		ref_z = particle[ref_ptr][2];
		int neighbor_particle_num = 0;
		for(int neighbor_ptr = 0; neighbor_ptr < TOTAL_PARTICLE_NUM; neighbor_ptr++){
			// Get r2
			neighbor_x = particle[neighbor_ptr][0];
			neighbor_y = particle[neighbor_ptr][1];
			neighbor_z = particle[neighbor_ptr][2];
			dx = ref_x - neighbor_x;
			dy = ref_y - neighbor_y;
			dz = ref_z - neighbor_z;
			// Apply periodic boundary
			if(dx >= 0){
				dx -= bounding_box_x * (int)(dx/bounding_box_x+0.5);
			}
			else{
				dx -= bounding_box_x * (int)(dx/bounding_box_x-0.5);
			}
			if(dy >= 0){
				dy -= bounding_box_y * (int)(dy/bounding_box_y+0.5);
			}
			else{
				dy -= bounding_box_y * (int)(dy/bounding_box_y-0.5);
			}
			if(dz >= 0){
				dz -= bounding_box_z * (int)(dz/bounding_box_z+0.5);
			}
			else{
				dz -= bounding_box_z * (int)(dz/bounding_box_z-0.5);
			}
			r2 = dx*dx + dy*dy + dz*dz;
			// Apply cutoff
			if(r2 > 0 && r2 <= CUTOFF_RADIUS_2){
				neighbor_particle_num++;
				//// Potential Energy
				inv_r2 = 1 / r2;
				inv_r8 = pow(inv_r2, 4);
				inv_r14 = pow(inv_r2, 7);
				inv_r6 = pow(inv_r2, 3);
				inv_r12= pow(inv_r2, 6);
				vdw12 = 4 * EPS * SIGMA_12 * inv_r12;
				vdw6 = 4 * EPS * SIGMA_6 * inv_r6;
				vdw14 = 48 * EPS * SIGMA_12 * inv_r14;
				vdw8  = 24 * EPS * SIGMA_6  * inv_r8;
				// LJ Force and Energy
				Fvdw = vdw14 - vdw8;
				Evdw = vdw12 - vdw6;
				// Accumualte Force
				Fx_acc += Fvdw * dx;
				Fy_acc += Fvdw * dy;
				Fz_acc += Fvdw * dz;
				// Accumulate Energy
				Evdw_acc += Evdw;

				/*****************************************************
				// OpenMM code
				******************************************************
				float r = sqrtf(r2);
				float inverseR = 1/r;
				float sig       = SIGMA + SIGMA;
				float sig2      = inverseR*sig;
				sig2     *= sig2;
				float sig6      = sig2*sig2*sig2;

				float eps       = EPS * EPS;
				float dEdR      = switchValue*eps*(12.0f*sig6 - 6.0f)*sig6;
				float chargeProd = ONE_4PI_EPS0*posq[4*ii+3]*posq[4*jj+3];
				if (cutoff)
					dEdR += (float) (chargeProd*(inverseR-2.0f*krf*r2));
				else
					dEdR += (float) (chargeProd*inverseR);
				dEdR *= inverseR*inverseR;
				float energy = eps*(sig6-1.0f)*sig6;
				if (useSwitch) {
					dEdR -= energy*switchDeriv*inverseR;
					energy *= switchValue;
				}

				// accumulate energies

				if (totalEnergy) {
					if (cutoff)
						energy += (float) (chargeProd*(inverseR+krf*r2-crf));
					else
						energy += (float) (chargeProd*inverseR);
					*totalEnergy += energy;
				}
				*/

			}
		}
		// Record the vdw energy
		particle[ref_ptr][6] = Evdw_acc;

		//// Evaluate Kinetic Energy
		acceleration_x = Fx_acc / MASS;
		acceleration_y = Fy_acc / MASS;
		acceleration_z = Fz_acc / MASS;
		// Velocity
		vx = particle[ref_ptr][3];
		vy = particle[ref_ptr][4];
		vz = particle[ref_ptr][5];
		vx += acceleration_x * SIMULATION_TIME_STEP;
		vy += acceleration_y * SIMULATION_TIME_STEP;
		vz += acceleration_z * SIMULATION_TIME_STEP;
		// Kinetic energy
		Eketic = 0.5 * MASS * (vx*vx + vy*vy +vz*vz);
		// Write back new velocity
		particle[ref_ptr][3] = vx;
		particle[ref_ptr][4] = vy;
		particle[ref_ptr][5] = vz;
		// Write back Kinetic energy
		particle[ref_ptr][7] = Eketic;

		// Accumualte to System total energy
		//System_Energy += Evdw_acc + Eketic;
		System_Energy += Evdw_acc;
	}

	printf("System energy is %f\n", System_Energy);

	/*****************************************
	// Motion update here
	*****************************************/

	return 1;
}
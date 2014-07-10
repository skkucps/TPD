/**
 *  File: main.h
	Description: This simulation model is for the Internet Connectivity on Vehicular Ad Hoc Networks (IVANET).
	Update Date: 09/14/2006
	Maker: Jaehoon Jeong, jjeong@cs.umn.edu
*/

#ifndef __MAIN_H__
#define __MAIN_H__

#include "common.h"
//#include "matlab-operation.h"

int run(unsigned int seed, struct parameter *param, char *graph_file, char *schedule_file, char *vanet_file, char *pathlist_file, char *output_file_1, char *output_file_2, char *trace_file_of_vehicle_convoy_length, int *trajectory, int trajectory_size, char *vanet_packet_carrier_trace_file);
//run simulation according to a given scenario

#endif

/**
 *  File: mobility.h
    Description: initialize the mobility of destination vehicle(s) with mobility_file_name
    Date: 07/10/2009
    Update Date: 07/10/2009
    Maker: Jaehoon Jeong
*/

#ifndef __MOBILITY_H__
#define __MOBILITY_H__

/** function declarations */
void init_mobility(char* mobility_file_name, boolean vehicle_id_override_flag, int new_simulation_node_id, destination_vehicle_queue_t* DVQ);
//initialize the mobility of destination vehicles with destination vehicle queue DVQ and update the vehicle id with the id starting from new_simulation_node_id according to vehicle_id_override_flag

//void init_mobility(char* mobility_file_name, destination_vehicle_queue_t* DVQ);
//initialize the mobility of destination vehicles with destination vehicle queue DVQ

#endif

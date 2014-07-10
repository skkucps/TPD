/** File: packet-model.h
	Description: specify the macro constants, structures, and enum types for packet model.
	Date: 07/24/2008
	Maker: Jaehoon Jeong, jjeong@cs.umn.edu
*/

#ifndef __PACKET_MODEL_H__
#define __PACKET_MODEL_H__

#include "common.h"
#include "queue.h"

/* packet structure */

typedef struct struct_packet
{
	int id;    /* packet id */
	int state; /* packet state */
        int source_vid;  /* source vehicle id */
	int seq;   /* sequence number */
        int carry_vid; /* carry vehicle id */     
	double direction; /* movement direction: {DIRECTION_FORWARD, DIRECTION_BACKWARD} */
        int size; /* data size in bytes */
	double generation_time;    /* time when this packet was generated */
	double arrival_time; /* time when this packet has arrived at access point */
        double expected_delivery_time; /* expected delivery time corresponding to the vehicle's EDD */
        double actual_delivery_time; /* actual delivery time from the packet source to the destination AP */
	struct_path_node *path_list; //list of vertices (i.e., intersections) on the path from source vehicle to destination access point
	struct struct_packet* next; /* next packet */
	struct struct_packet* prev; /* previous packet */
} struct_packet_t;

#endif

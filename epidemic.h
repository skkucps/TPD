/**
 * File: epidemic.h
 * Description: declare the data structures and operations for Epidemic Routing.
 * Origin Date: 01/29/2014
 * Update Date: 01/29/2014
 * Maker: Jaehoon Paul Jeong, pauljeong@skku.edu
 */

#ifndef __EPIDEMIC_H__
#define __EPIDEMIC_H__

#include "queue.h"

int EPIDEMIC_Perform_Packet_Dissemination_At_Intersection_For_AP(parameter_t *param,
		double current_time,
		struct_access_point_t *AP,
		struct_graph_node *G,
		int G_size,
		packet_delivery_statistics_t *packet_delivery_stat);
//Under V2V mode with Epidemic Routing, perform packet dissemination at an intersection with an AP by letting the AP give its packets to each cluster head for each outgoing edge at the intersection
	
int EPIDEMIC_Perform_Packet_Dissemination_At_Intersection(parameter_t *param,
		double current_time,
		struct_vehicle_t *vehicle,
		struct_graph_node *G,
		int G_size,
		packet_delivery_statistics_t *packet_delivery_stat);
//Under V2V mode, perform packet dissemination at an intersection by letting vehicle give its packets to each cluster head for each outgoing edge at the intersection

int EPIDEMIC_Forward_Packet_Copy_From_AP_To_Next_Carrier(parameter_t *param, double current_time, struct_access_point_t *AP, struct_vehicle_t *next_carrier, packet_delivery_statistics_t *packet_delivery_stat, int *discard_count);
//AP forwards its packet copies to the next carrier pointed by next_carrier for Epidemic Routing

int EPIDEMIC_Forward_Packet_Copy_To_Next_Carrier(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_vehicle_t *next_carrier, packet_delivery_statistics_t *packet_delivery_stat, int *discard_count);
//vehicle forwards its packet copies to the next carrier pointed by next_carrier for Epidemic Routing

int EPIDEMIC_Perform_Packet_Dissemination_On_Road_Segment(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, packet_delivery_statistics_t *packet_delivery_stat);
//perform packet dissemination on either one-way road segment or two-way road segment by Epidemic Routingand return the number of packet copies forwarded to a neighboring vehicle in a different cluster 

boolean EPIDEMIC_Is_There_Next_Carrier_On_Road_Segment(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier_1, struct_vehicle_t **next_carrier_2);
//determine whether to forward its packets to next carriers moving on the current road segment in Epidemic Routing and return the pointers to the next carriers through *next_carrier; Note that the convoy leaders become the next carriers

boolean EPIDEMIC_Is_There_Next_Carrier_On_One_Way_Road_Segment(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier);
//For one-way road segment, determine whether to forward its packets to next carrier moving on the current road segment in Epidemic Routing and return the pointer to the next carrier through *next_carrier; Note that the convoy leader becomes the next carrier 

boolean EPIDEMIC_Is_There_Next_Carrier_On_Two_Way_Road_Segment(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier_1, struct_vehicle_t **next_carrier_2);
//For two-way road segment, determine whether to forward its packets to next carriers moving on the current road segment in Epidemic Routing and return the pointers to the next carriers through *next_carrier_1 and *next_carrier_2; Note that the convoy leaders become the next carriers 

int EPIDEMIC_Discard_Original_Packet_In_AP(double current_time, 
		struct_access_point_t *AP);
//discard original packets in AP's packet queue by releasing the memory of the packets in the packet queue

#endif /* __EPIDEMIC_H__ */



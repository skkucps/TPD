/**
 * File: tpd.h
 * Description: specify the data structures and operations for TPD data forwarding.
 * Origin Date: 07/15/2013
 * Update Date: 07/17/2013
 * Maker: Jaehoon Paul Jeong, pauljeong@skku.edu
 */

#ifndef __TPD_H__
#define __TPD_H__

#include "queue.h"

/** The Operations of Predicted Encounter Graph */
int TPD_Allocate_Predicted_Encounter_Graph(parameter_t *param, struct_vehicle_t *vehicle);
//allocate the memory of a predicted encounter graph for vehicle

int TPD_Allocate_Predicted_Encounter_Graph_For_Packet(packet_queue_node_t* packet);
//allocate the memory of a predicted encounter graph for packet 

int TPD_Free_Predicted_Encounter_Graph(struct_vehicle_t *vehicle);
//free the memory of a predicted encounter graph for vehicle

int TPD_Free_Predicted_Encounter_Graph_For_Packet(packet_queue_node_t* packet);
//free the memory of a predicted encounter graph for packet 

int TPD_Reset_Queues_In_Predicted_Encounter_Graph(struct_vehicle_t *vehicle);
//reset two queues Q and G in the predicted encounter graph for vehicle

int TPD_Reset_Queues_In_Predicted_Encounter_Graph_Packet(packet_queue_node_t* packet);
//reset two queues Q and G in the predicted encounter graph for packet 

double TPD_Compute_EDR_and_EDD(double current_time,
		parameter_t *param,
		struct_vehicle_t *src_vehicle,
		struct_vehicle_t *dst_vehicle,
		boolean encounter_graph_display_flag);
//compute the EDR (Expected Delivery Ratio) and EDD (Expected Delivery Delay) of src_vehicle for dst_vehicle for either Greedy Routing or Source Routing

double TPD_Compute_EDR_and_EDD_For_Greedy_Routing(double current_time,
		parameter_t *param,
		struct_vehicle_t *src_vehicle,
		struct_vehicle_t *dst_vehicle,
		boolean encounter_graph_display_flag);
//compute the EDR (Expected Delivery Ratio) and EDD (Expected Delivery Delay) of src_vehicle for dst_vehicle for Greedy Routing

double TPD_Compute_EDR_and_EDD_For_Source_Routing(double current_time,
		parameter_t *param,
		packet_queue_node_t* packet,
		struct_vehicle_t *src_vehicle,
		struct_vehicle_t *dst_vehicle,
		boolean encounter_graph_display_flag);
//compute the EDR (Expected Delivery Ratio) and EDD (Expected Delivery Delay) of src_vehicle for dst_vehicle with constructing an encounter graph to deliver packet from src_vehicle to dst_vehicle by Source Routing

double TPD_Compute_EDR_and_EDD_For_Packet(double current_time,
		parameter_t *param,
		packet_queue_node_t *packet,
		struct_vehicle_t *src_vehicle,
		struct_vehicle_t *dst_vehicle);
//compute the EDR (Expected Delivery Ratio) and EDD (Expected Delivery Delay) of src_vehicle for dst_vehicle to deliver packet from src_vehicle to dst_vehicle

double TPD_Construct_Predicted_Encounter_Graph(double current_time,
		parameter_t *param,
		struct_vehicle_t *src_vehicle,
		struct_vehicle_t *dst_vehicle,
		boolean encounter_graph_display_flag);
//construct a predicted encounter graph from src_vehicle to dst_vehicle and return Expected Delivery Ratio (EDR).

double TPD_Construct_Predicted_Encounter_Graph_For_Packet(double current_time,
		parameter_t *param, 
		packet_queue_node_t *packet,
		struct_vehicle_t *src_vehicle, 
		struct_vehicle_t *dst_vehicle,
		boolean encounter_graph_display_flag);
//construct a predicted encounter graph from src_vehicle to dst_vehicle for packet and return Expected Delivery Ratio (EDR).

boolean TPD_Do_Vehicles_Encounter(double current_time,
		parameter_t *param,
		struct_vehicle_t *vehicle1, 
		struct_vehicle_t *vehicle2, 
		int *tail_vertex, 
		int *head_vertex,
		double *edge_length,
		double *P_encounter, 
		double *T_encounter,
		double *D_encounter,
		double *O_encounter);
//check whether vehicle1 and vehicle2 encounter at the offset O_encounter in an edge (tail_vertex, head_vertex) with P_encounter of at least encounter_probability_threshold at time T_encounter after the travel time (i.e., delay) of D_encounter.

int TPD_Prune_Encounter_Graph(adjacency_list_queue_t *G);
//prune the encounter graph G by removing graph nodes that are not used as intermediate nodes for the data forwarding from src_vehicle to dst_vehicle

double TPD_Compute_EDR_For_Encounter_Graph(adjacency_list_queue_t *G);
/* compute the EDR (Expected Delivery Ratio) for the given encounter graph G 
 by Breadth First Search (BFS) from destination vehicle to source vehicle in G */ 

double TPD_Compute_EDR_For_Encounter_Graph_By_DP(adjacency_list_queue_t *G);
/* compute the EDR (Expected Delivery Ratio) for the given encounter graph G 
 by Breadth First Search (BFS) from destination vehicle to source vehicle in G 
 and Dynamic Programming (DP) for an optimal forwarding subsequence */ 

double TPD_Compute_EDD_For_Encounter_Graph(adjacency_list_queue_t *G);
/* compute the EDD (Expected Delivery Delay) for the given encounter graph G
 by Breadth First Search (BFS) from destination vehicle to source vehicle in G */

boolean TPD_Compute_Encounter_Probability(parameter_t *param,
		struct_vehicle_t *vehicle1, 
		struct_vehicle_t *vehicle2,
		double T_threshold,
		int *tail_vertex, 
		int *head_vertex,
		double *edge_length,
		double *P_encounter, 
		double *T_encounter,
		double *D_encounter,
		double *O_encounter);
/* compute the encounter probability (called P_encounter) of vehicle1 and 
   vehicle2 that are expected to encounter in the edge (tail_vertex, head_vertex)
   of length edge_length and return the boolean flag to indicate whether these 
   two vehicles will encounter with some probability.
   Note: T_threshold is the encounter time of vehicle and its parent vehicle 
   that is used to determine the valid encounter of two vehicles during the 
   selection of child vehicles in the predicted encounter graph. */

double TPD_Compute_Forwarding_Probability(neighbor_list_queue_t *Q, 
		int i);
/* compute the forwarding probability that a packet carrier vehicle can
	forward its packets to the vehicle corresponding to index i (i.e.,
	the i-th vehicle) in Q when the packet carrier vehicle fails to encounter
	the vehicles before the i-th vehicle. */

double TPD_Compute_Forwarding_Probability_For_Optimal_Forwarding_Subsequence(neighbor_list_queue_node_t *pFirstPosition, 
		int first_position, 
		int candidate_position,	
		int last_position);
/* compute the forwarding probability that a packet carrier vehicle 
	candidate_position can forward its packets to the vehicle corresponding to 
	candidate_position in the neighbor-list when the packet carrier vehicle fails 
	to encounter the vehicles from first_position to candidate_position - 1. */

boolean TPD_Is_There_Next_Carrier_At_Intersection_For_AP(parameter_t *param, 
		double current_time, 
		struct_access_point_t *AP, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier);
//Under V2V mode, determine whether to forward AP's packets to next carrier moving on the other road segment with the highest EDR at intersection and return the pointer to the next carrier through *next_carrier

boolean TPD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_For_AP(
		parameter_t *param, 
		double current_time, 
		struct_access_point_t *AP, 
		char *tail_node_for_next_forwarding_edge, 
		char *head_node_for_next_forwarding_edge, 
		directional_edge_type_t edge_type, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier);
//Under V2V mode, determine whether to forward AP's packets to next carrier moving on the road segment incident to an intersection corresponding to tail_node_for_next_edge and return the pointer to the next carrier through *next_carrier

boolean TPD_Is_There_Next_Carrier_At_Intersection(parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier);
//determine whether to be able to forward its packets to next carrier at the intersection according to the forwarding criteria, depending on param's tpd_encounter_graph_source_routing_flag and return the pointer to the next carrier through *next_carrier

boolean TPD_Is_There_Next_Carrier_At_Intersection_For_Greedy_Routing(parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier);
//determine whether to be able to forward its packets to next carrier with the highest EDR, moving on the other road segment of the intersection and return the pointer to the next carrier through *next_carrier

boolean TPD_Is_There_Next_Carrier_At_Intersection_For_Source_Routing(parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier);
//determine whether to be able to forward its packets to next carrier that is one of the child vehicles of the current packet carrier vehicle in vehicle's predicted encounter graph at the intersection and return the pointer to the next carrier through *next_carrier.

boolean TPD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_For_Greedy_Routing(
		parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		char *tail_node_for_next_forwarding_edge, 
		char *head_node_for_next_forwarding_edge, 
		directional_edge_type_t edge_type, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier);
//determine whether to be able to forward its packets to next carrier with the highest EDR, moving on the road segment incident to an intersection corresponding to tail_node_for_next_edge and return the pointer to the next carrier through *next_carrier

boolean TPD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_For_Source_Routing(
		parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		char *tail_node_for_next_forwarding_edge, 
		char *head_node_for_next_forwarding_edge, 
		directional_edge_type_t edge_type, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier);
//determine whether to be able to forward its packets to next carrier (as a child vehicle of vehicle in vehicle's predicted encounter graph), moving on the road segment incident to an intersection corresponding to tail_node_for_next_edge and return the pointer to the next carrier through *next_carrier

int TPD_Perform_BFS_For_Encounter_Graph(adjacency_list_queue_t *G, boolean display_flag);
/* perform the Breadth-First-Search (BFS) for the given encounter graph G 
	from source vehicle to destination vehicle in G; trajectory_flag is used to determine
	whether to show the trajectory of each vehicle in G */

int TPD_Print_Vehicle_Trajectory(struct_vehicle_t *vehicle);
//print vehicle's trajectoy (i.e., path_list from the current tail node to the last node) along with arrival time

boolean TPD_Is_There_Next_Carrier_On_Road_Segment(parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier);
//determine whether to forward its packets to next carrier with a better EDR, moving on the current road segment and return the pointer to the next carrier through *next_carrier. 

boolean TPD_Is_There_Next_Carrier_On_One_Way_Road_Segment_For_Greedy_Routing(parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier);
//For one-way road segment, determine whether to forward its packets to next carrier with a better EDR, moving on the current road segment and return the pointer to the next carrier through *next_carrier.

boolean TPD_Is_There_Next_Carrier_On_One_Way_Road_Segment_For_Source_Routing(parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier);
//For one-way road segment, determine whether to forward its packets to next carrier in the predicted encounter graph for source routing and return the pointer to the next carrier through *next_carrier.

boolean TPD_Is_There_Next_Carrier_On_Two_Way_Road_Segment_For_Greedy_Routing(parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier);
//For two-way road segment, determine whether to forward its packets to next carrier with a better EDR, moving on the current road segment and return the pointer to the next carrier through *next_carrier.

boolean TPD_Is_There_Next_Carrier_On_Two_Way_Road_Segment_For_Source_Routing(parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier);
//For two-way road segment, determine whether to forward its packets to next carrier in the predicted encounter graph for source routing and return the pointer to the next carrier through *next_carrier.

boolean TPD_Check_Child_Vehicle_In_Encounter_Graph(double current_time, struct_vehicle_t *vehicle, struct_vehicle_t *candidate);
//check whether candidate is one of vehicle's child vehicles in vehicle's encounter graph or not

adjacency_list_queue_node_t* TPD_Find_GraphNode_In_EncounterGraph(adjacency_list_queue_t *G,
	struct_vehicle_t *vehicle);
//find the pointer to a graph node in the encounter graph G corresponding to vehicle

#endif

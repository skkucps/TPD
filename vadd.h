/**
 *  File: vadd.h
    Description: implementation of Vehicle-Assisted Data Delivery (VADD) scheme
    Date: 08/08/2008
    Update Date: 08/08/2008
    Maker: Jaehoon Jeong
*/

#ifndef __VADD_H__
#define __VADD_H__

/** include header files */
#include <math.h> //for exp()
#include "graph-data-struct.h" //struct_coordinate1_t
#include "param.h" //parameter_t
#include "queue.h" //angle_queue_t, angle_queue_node_t
#include "vehicle-model.h" //struct_vehicle_t
#include "access-point-model.h" //struct_access_point_t

/** macro functions */
#define VADD_Contacting_Time(r, v) (2*r/v)
/* contacting time where r is communication range and v is vehicle speed:
   time which can be spent for packet forwarding by packet carriers 
   which enter intersection I_i, and move towards neighboring intersection I_j.   
*/
 
#define VADD_Vehicle_Density(lambda, v) (lambda/v)
/* vehicle density where lambda is vehicle arrival rate and v is vehicle speed:
   density of vehicle in a unit road length, such as 1 meter.
*/

#define VADD_Vehicle_Lambda_1(rho, v) (rho*v)
/* vehicle lambda where rho is vehicle density and v is vehicle speed:
   lambda value which represents the vehicle arrival rate during the unit time, 
   such as 1 second.
*/

#define VADD_Vehicle_Lambda_2(I) ((I == INF)?(0):(1/I))
//#define VADD_Vehicle_Lambda_2(I) (1/I)
/* vehicle lambda where t is mean interarrival time:
   lambda value which represents the vehicle arrival rate during the unit time, 
   such as 1 second.
*/

#define VADD_Vehicle_Mean_Interarrival_Time(lambda) ((lambda == 0)?(INF):(1/lambda))
//#define VADD_Vehicle_Mean_Interarrival_Time(lambda) (1/lambda)
/* vehicle mean interarrival time where lambda is vehicle arrival rate:
   mean interarrival time which represents the average of vehicle interarrival times.
*/

#define VADD_CP(lambda, T) (1 - exp(-1*lambda*T))
/* contacting probability where lambda is vehicle arrival rate and T is contacting time:
   probability (C_ij) for a packet carrier to meet at least one contact towards road r_ij
   when the carrier moves within the intersection area.
*/

/** definition of structures */

/** declaration of functions */
double VADD_Compute_Angle(struct_coordinate1_t *coord_1, struct_coordinate1_t *coord_2, struct_coordinate1_t *coord_3);
/* where coord_1 is the coordinate of the current intersection I_i, coord_2 is the coordinate of the neighboring
     intersection I_j, and coord_3 is the coordination of the destination:
     compute the angle theta_ij between the direction of road r_ij and the vector from 
     the current intersection to the destination; theta_ij is an approximation of D_ij
     that is the expected delivery delay from intersection i to intersection j. */

void VADD_Compute_Forwarding_Probability(parameter_t *param, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, int ap_table_index);
//compute the forwarding probability at each intersection using road graph G along with access point table ap_table

void VADD_Compute_Forwarding_Probability_For_Target_Intersection(parameter_t *param, struct_graph_node *G, int G_size, int target_intersection_index);
//compute the forwarding probability at each intersection using road graph G for a target intersection whose intersection id is (target_intersection_index+1)

void VADD_Recompute_Forwarding_Probability(parameter_t *param, struct_graph_node *G, int G_size);
//recompute the forwarding probability at each intersection using road graph G with the computed EDD for each edge

double VADD_Compute_Conditional_Probability_For_Forwarding_Probability(angle_queue_t *M, int k, int h, angle_queue_node_t *pAngleNode);
//compute conditional probability that a vehicle moving in road r_ih will forward (or continue to carry in the case of k == h) a packet to another vehicle moving towards intersection I_k at intersection I_i along with angle queue M where pAngleNode is the pointer to the angle node corresponding to k

double VADD_Compute_Conditional_Probability_For_Pure_Forwarding_Probability(angle_queue_t *M, int k, int h, angle_queue_node_t *pAngleNode);
//compute conditional probability that a vehicle moving in road r_ih will forward a packet to another vehicle moving towards intersection I_k at intersection I_i along with angle queue M where pAngleNode is the pointer to the angle node corresponding to k; Note that this pure forwarding probability excludes the case where the carrier towards I_h keeps carrying the packet without forwarding.

void VADD_Compute_EDD_Based_On_Stochastic_Model(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, int ap_table_index);
//compute the Expected Delivery Delay (EDD) based on the stochastic model with graph G and directional edge queue EQ.

void VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model_For_Target_Intersection(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, int target_intersection_index);
//compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) based on the stochastic model with graph G and directional edge queue EQ for a target intersection whose intersection id is (target_intersection_index+1).

void VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model_For_Multiple_APs(parameter_t *param, struct_graph_node *Gr, int Gr_size, directional_edge_queue_t *DEr, struct_traffic_table *ap_table_for_Gr);
//compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) based on the stochastic model with graph Gr and directional edge queue EQ in the road network with multiple Internet Access Points (APs).

void VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model_For_V2V_Data_Delivery(
			parameter_t *param,
			struct_graph_node *Gr, 
			int Gr_size, 
			struct_graph_node **Gr_set, 
			int *Gr_set_size, 
			directional_edge_queue_t *DEr_set);
//compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) based on the stochastic model with road graph network set Gr_set and directional edge queue set DEr_set in the road network for the V2V data delivery.

void VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, int ap_table_index);
//compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) based on the stochastic model with graph G and directional edge queue EQ.

void VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model_For_Multiple_APs(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, boolean forwarding_table_update_flag);
//For the Multiple-AP road network, compute the Expected Delivery Delay (EDD) with graph G and directional edge queue EQ, based on the shortest path model along with the vanet metric type, such as EDD and EDD_VAR. Note that if fowarding_table_update_flag is TRUE, this functions updates the matrices for the forwarding table entry corresponding to ap_table_index. Otherwise, it updates the matrices of the global matrices, such as param->vanet_table.Dr_edd.

void VADD_Compute_EDC_And_EDC_SD_Based_On_Shortest_Path_Model_For_Multiple_APs(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, boolean forwarding_table_update_flag);
//For the Multiple-AP road network, compute the Expected Delivery Cost (EDC) with graph G and directional edge queue EQ, based on the shortest path model along with the vanet metric type, such as EDC. Note that if fowarding_table_update_flag is TRUE, this functions updates the matrices for the forwarding table entry corresponding to ap_table_index. Otherwise, it updates the matrices of the global matrices, such as param->vanet_table.Dr_edc.

void VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, int ap_table_index, boolean forwarding_table_update_flag);
//compute the Expected Delivery Delay (EDD) with graph G and directional edge queue EQ, based on the shortest path model along with the vanet metric type, such as EDD and EDD_VAR. Note that if fowarding_table_update_flag is TRUE, this functions updates the matrices for the forwarding table entry corresponding to ap_table_index. Otherwise, it updates the matrices of the global matrices, such as param->vanet_table.Dr_edd.

void VADD_Compute_EDC_And_EDC_SD_Based_On_Shortest_Path_Model(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, int ap_table_index, boolean forwarding_table_update_flag);
//compute the Expected Delivery Cost (EDC) with graph G and directional edge queue EQ, based on the shortest path model along with the vanet metric type, such as EDC. Note that if fowarding_table_update_flag is TRUE, this functions updates the matrices for the forwarding table entry corresponding to ap_table_index. Otherwise, it updates the matrices of the global matrices, such as param->vanet_table.Dr_edc.

void VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model_For_Shortest_EDD(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, int ap_table_index, boolean forwarding_table_update_flag);
//For the shortest EDD (i.e., expected delivery delay), compute the Expected Delivery Delay (EDD) and the Delivery Delay Standard Deviation based on the shortest path model with graph G and directional edge queue EQ. Note that if fowarding_table_update_flag is TRUE, this functions updates the matrices for the forwarding table entry corresponding to ap_table_index. Otherwise, it updates the matrices of the global matrices, such as param->vanet_table.Dr_edd.

void VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model_For_Shortest_EDD_VAR(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, int ap_table_index, boolean forwarding_table_update_flag);
//For the shortest EDD_VAR (i.e., delivery delay variance), compute the Expected Delivery Delay (EDD) and the Delivery Delay Standard Deviation based on the shortest path model with graph G and directional edge queue EQ. Note that if fowarding_table_update_flag is TRUE, this functions updates the matrices for the forwarding table entry corresponding to ap_table_index. Otherwise, it updates the matrices of the global matrices, such as param->vanet_table.Dr_edd.

void VADD_Compute_EDC_And_EDC_SD_Based_On_Shortest_Path_Model_For_Shortest_EDC(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, int ap_table_index, boolean forwarding_table_update_flag);
//For the shortest EDC (i.e., expected delivery cost), compute the Expected Delivery Cost (EDC) and the Delivery Cost Standard Deviation based on the shortest path model with graph G and directional edge queue EQ. Note that if fowarding_table_update_flag is TRUE, this functions updates the matrices for the forwarding table entry corresponding to ap_table_index. Otherwise, it updates the matrices of the global matrices, such as param->vanet_table.Dr_edc.

void VADD_Compute_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(parameter_t *param, int target_point_id, char *AP_vertex, forwarding_table_queue_t *FTQ, double *AP_EDD, double *AP_EDD_SD);
//compute the EDD and EDD_SD for a target point towards a destination vehicle, based on EDD Computation Model (i.e., Stochastic Model or Shortest Path Model) from an intersection having an access point, that is, from AP's gnode.

void VADD_Compute_EDD_And_EDD_SD_For_TargetPoint_At_Intersection_Based_On_Stochastic_Model(parameter_t *param, int target_point_id, char *AP_vertex, forwarding_table_queue_t *FTQ, double *AP_EDD, double *AP_EDD_SD);
//compute the EDD and EDD_SD for a target point towards a destination vehicle, based on VADD Stochastic Model (i.e., Per-intersection Model) from an intersection having an access point, that is, from AP's gnode.

void VADD_Compute_EDD_And_EDD_SD_For_TargetPoint_At_Intersection_For_V2V_Data_Delivery(parameter_t *param, int target_point_id, int source_intersection_id, struct_graph_node *Gr, int Gr_size, double *E2E_EDD, double *E2E_EDD_SD);
//compute the E2E EDD and EDD_SD fromfor a target point towards a destination vehicle, based on VADD Stochastic Model (i.e., Per-intersection Model) from an intersection having an access point, that is, from AP's gnode.

void VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(parameter_t *param, int target_point_id, char *intersection_vertex, forwarding_table_queue_t *FTQ, double *intersection_EDD, double *intersection_EDD_SD);
//get the EDD and EDD_SD for a target point towards a destination vehicle, based on VADD Model (i.e., Per-intersection Model) from an intersection having an access point or stationary node.

void VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection_For_V2V_Data_Delivery(parameter_t *param, int target_point_id, char *intersection_vertex, forwarding_table_queue_t *FTQ, struct_graph_node *Gr, int Gr_size, double *intersection_EDD, double *intersection_EDD_SD);
//get the EDD and EDD_SD from AP (denoted by intersection_vertex) to a target point toward a destination vehicle, based on VADD Model (i.e., Per-intersection Model).
 
void VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Carrier(parameter_t *param, int target_point_id, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, double *EDD, double *EDD_SD);
//get the EDD and EDD_SD for a target point towards a destination vehicle, based on VADD Model (i.e., Per-intersection Model) from the position of carrier_vehicle, that is, from carrier's edge offset on the road network graph.

double VADD_Get_EDD_And_EDD_SD_In_Shortest_Path_Model(parameter_t *param, int src_id, int dst_id, double *EDD, double *EDD_SD);
//get the EDD and EDD_SD from src_id to dst_id in the road network in the shortest path model

double VADD_Compute_Edge_Delay(parameter_t *param, struct_graph_node *pGraphNode);
//compute the edge delay for the road segment r_ij with head node pGraphNode->vertex

double VADD_Compute_Subedge_Delay(parameter_t *param, struct_graph_node *pGraphNode, double subedge_length);
//compute the edge delay for the subedge length in the road segment r_ij where the vehicle has moved

double VADD_Compute_Average_Convoy_Length(parameter_t *param, struct_graph_node *pGraphNode);
//compute the Average Convoy Length (ACL) for the road segment r_ij with head node pGraphNode->vertex

double VADD_Compute_Edge_Delay_Standard_Deviation(parameter_t *param, struct_graph_node *pGraphNode);
//compute the edge delay's standard deviation for the road segment r_ij with head node pGraphNode->vertex

double VADD_Compute_Subedge_Delay_Standard_Deviation(parameter_t *param, struct_graph_node *pGraphNode, double subedge_length);
//compute the edge delay standard deviation for the subedge length in the road segment r_ij where the vehicle has moved

double VADD_Compute_Average_Convoy_Length_Standard_Deviation(parameter_t *param, struct_graph_node *pGraphNode);
//compute the Average Convoy Length (ACL) Standard Deviation for the road segment r_ij with head node pGraphNode->vertex

/** Functions for Data Forwarding **/

boolean VADD_Is_Within_AP_Communication_Range(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, struct_graph_node **ap_graph_node);
//check whether this vehicle is within the communication range of an Internet access point and return the pointer to the graph node corresponding to the access point through *ap_graph_node

boolean VADD_Is_Within_Destination_Vehicle_Communication_Range(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, destination_vehicle_queue_t *Q, struct_vehicle_t **destination_vehicle);
//check whether this vehicle is within the communication range of one of destination vehicles in destination vehicle queue Q and return the pointer to the destination vehicle

boolean VADD_Is_There_Next_Carrier_On_Road_Segment(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier);
//determine whether to forward its packets to next carrier moving on the current road segment and return the pointer to the next carrier through *next_carrier

boolean VADD_Is_There_Next_Carrier_On_One_Way_Road_Segment(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier);
//For one-way road segment, determine whether to forward its packets to next carrier moving on the current road segment and return the pointer to the next carrier through *next_carrier; Note that the convoy leader becomes the next carrier 

boolean VADD_Is_There_Next_Carrier_On_Two_Way_Road_Segment(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier);
//For two-way road segment, determine whether to forward its packets to next carrier moving on the current road segment and return the pointer to the next carrier through *next_carrier; Note that the convoy leader becomes the next carrier 

boolean VADD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection(parameter_t *param, double current_time, struct_vehicle_t *vehicle, char *tail_node_for_next_forwarding_edge, char *head_node_for_next_forwarding_edge, directional_edge_type_t edge_type, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier);
//determine whether to forward its packets to next carrier moving on the road segment incident to an intersection corresponding to tail_node_for_next_edge and return the pointer to the next carrier through *next_carrier

boolean VADD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_For_AP(parameter_t *param, double current_time, struct_access_point_t *AP, char *tail_node_for_next_forwarding_edge, char *head_node_for_next_forwarding_edge, directional_edge_type_t edge_type, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier);
//Under download mode, determine whether to forward AP's packets to next carrier moving on the road segment incident to an intersection corresponding to tail_node_for_next_edge and return the pointer to the next carrier through *next_carrier

int VADD_Iterative_Forward_Packet_To_Next_Carrier_On_Road_Segment(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, forwarding_table_queue_t *FTQ, packet_delivery_statistics_t *packet_delivery_statistics);
//The packet holder vehicle (i.e., vehicle) tries to send its packets including the new packet to a better carrier. Also, iteratively, these packets are transmitted to the next carrier until the packets cannot be forwarded to the next carrier.

boolean VADD_Is_There_Next_Carrier_At_Intersection(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, forwarding_table_queue_t *FTQ, struct_vehicle_t **next_carrier);
//determine whether to forward its packets to next carrier moving on the other road segment with the smallest EDD at intersection and return the pointer to the next carrier through *next_carrier

boolean VADD_Is_There_Next_Carrier_At_Intersection_For_AP(parameter_t *param, double current_time, struct_access_point_t *AP, struct_graph_node *G, int G_size, forwarding_table_queue_t *FTQ, struct_vehicle_t **next_carrier);
//Under download mode or V2V mode, determine whether to forward AP's packets to next carrier moving on the other road segment with the smallest EDD for access point AP at intersection and return the pointer to the next carrier through *next_carrier

int VADD_Forward_Packet_To_Next_Carrier(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_vehicle_t *next_carrier, packet_delivery_statistics_t *packet_delivery_stat, int *discard_count);
//vehicle forwards its packet(s) to the next carrier pointed by next_carrier

void VADD_Forward_Packet_To_The_Following_Vehicle_In_Convoy(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, packet_delivery_statistics_t *packet_delivery_stat);
/* vehicle forwards its packet(s) to the following vehicle in the same convoy if this vehicle is not the minimum EDD vehicle and has the greater EDD in the next road segment on its path than the convoy leader's EDD.
     @NOTE: we need to consider the communication delay between vehicle and the following vehicle later.
*/

void VADD_Forward_Packet_To_AP(parameter_t *param, double current_time, struct_vehicle_t *vehicle, access_point_queue_node_t *AP, packet_delivery_statistics_t *packet_delivery_stat);
//vehicle forwards its packet(s) to the access point pointed by ap_graph_node and the log for the packet(s) is written into the packet logging file.

void VADD_Forward_Packet_To_Destination_Vehicle(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, struct_vehicle_t *destination_vehicle, packet_delivery_statistics_t *packet_delivery_stat);
//vehicle forwards its packet(s) to the destination vehicle pointed by destination_vehicle and the log for the packet(s) is written into the packet logging file.

void VADD_Forward_Packet_From_Stationary_Node_To_Destination_Vehicle(parameter_t *param, double current_time, int intersection_id, struct_vehicle_t *destination_vehicle, packet_delivery_statistics_t *packet_delivery_stat);
//The stationary node corresponding to intersection_id forwards its packet(s) to the destination vehicle pointed by destination_vehicle and the log for the packet(s) is written into the packet logging file.

int VADD_Forward_Packet_From_AP_To_Next_Carrier(parameter_t *param, double current_time, struct_access_point_t *AP, struct_vehicle_t *next_carrier, packet_delivery_statistics_t *packet_delivery_stat, int *discard_count);
//AP forwards its packet(s) to the next carrier pointed by next_carrier and return the forward count

void VADD_Forward_Packet_From_AP_To_Stationary_Node(parameter_t *param, double current_time, struct_access_point_t *AP, int intersection_id, packet_delivery_statistics_t *packet_delivery_stat);
//AP forwards its packet(s) to the stationary node at the same intersection

void VADD_Forward_Packet_From_Stationary_Node_To_Next_Carrier(parameter_t *param, double current_time, int intersection_id, struct_vehicle_t *next_carrier, packet_delivery_statistics_t *packet_delivery_stat);
//forward from the stationary node at the intersection corresponding to intersection_id to next_vehicle the packets that are needed to go to the edge where vehicle is moving

void VADD_Forward_Packet_To_Stationary_Node(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *Gr, int Gr_size, intersection_area_type_t intersection_area_type, int head_intersection, packet_delivery_statistics_t *packet_delivery_stat);
//vehicle forwards its packet(s) to the stationary node at the heading intersection

void VADD_Discard_Expired_Packet(parameter_t *param, double current_time, packet_queue_node_t *pPacketNode, vanet_node_type_t node_type, void *vanet_node, packet_delivery_statistics_t *packet_delivery_stat,int lineNumber); //discard the expired packet by destroying the memory of the packet while the correponding global packet queue node is dequeued and destroyed 

double VADD_Update_Vehicle_EDD(double update_time, parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table);
//compute vehicle's EDD using the vehicle's offset in directional edge along with real graph G and AP table 

void VADD_Update_Vehicle_EDD_And_EDD_SD(double update_time, parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table);
//compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge along with real graph G and AP table 

void VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(double update_time, parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, int source_intersection_id);
//compute vehicle's EDD and EDD_SD from source intersection to the target point for the road network graph G for V2V data delivery, using the vehicle's offset in directional edge in the graph G 

void VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download(double update_time, parameter_t *param, struct_vehicle_t *vehicle, forwarding_table_queue_t *FTQ);
//For download mode, compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge with forwarding table queue FTQ and update the vehicle's target point under dynamic target point selection

void VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download_With_Given_TargetPoint_And_SequenceNumber(double update_time, parameter_t *param, struct_vehicle_t *vehicle, forwarding_table_queue_t *FTQ, int target_point_id, unsigned int seq);
//Given a target point and the packet sequence number (to check the freshness of the destination vehicle information), compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge with forwarding table queue FTQ

void VADD_Update_VehicleTargetPoint_Along_With_EDD_And_EDD_SD_For_Download(double update_time, parameter_t *param, struct_vehicle_t *vehicle, forwarding_table_queue_t *FTQ, intersection_area_type_t intersection_area_type);
//For download mode, compute vehicle's target point for the destination vehicle and then recompute the EDD_for_download/EDD_SD_download with forwarding table queue FTQ; for a convoy-based forwarding, update the target point and the EDD_for_download/EDD_SD_download of vehicle's leader 

double VADD_Compute_TBD_Based_EDD(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table);
//compute the TBD-Model-Based EDD (i.e., Per-vehicle EDD) using vehicle's trajectory in the vehicle's current position on the current directional edge

double VADD_Compute_VADD_Based_EDD(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table);
//compute the VADD-Model-Based EDD (i.e., Per-intersection EDD) in vehicle's current position on the current directional edge

//double VADD_Compute_TBD_Based_EDD_On_Next_Road_Segment(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table);
//compute the TBD-Model-Based EDD using vehicle's trajectory on the vehicle's next road segment

void VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD);
//compute the EDD and EDD_SD based on TBD Model (i.e., Per-vehicle Model) in vehicle's current position on the current directional edge

void VADD_Compute_EDD_And_EDD_SD_Based_On_TBD_VERSION_1(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD);
//compute the EDD and EDD_SD based on TBD Model (i.e., Per-vehicle Model) in vehicle's current position on the current directional edge

void VADD_Compute_EDD_And_EDD_SD_Based_On_TBD_VERSION_2(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD);
//compute the EDD and EDD_SD based on TBD Model (i.e., Per-vehicle Model) in vehicle's current position on the current directional edge

void VADD_Compute_EDD_And_EDD_SD_Based_On_TBD_VERSION_3(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD);
//[10/06/09] compute the EDD and EDD_SD based on TBD Model (i.e., Per-vehicle Model) in vehicle's current position on the current directional edge

void VADD_Compute_EDD_And_EDD_SD_Based_On_TBD_VERSION_4(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD);
//compute the EDD and EDD_SD based on TBD Model (i.e., Per-vehicle Model) in vehicle's current position on the current directional edge; note that this version is for TBD journal submission

void VADD_Compute_EDD_And_EDD_SD_Based_On_TBD_For_V2V_Data_Delivery(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, int source_intersection_id, double *vehicle_EDD, double *vehicle_EDD_SD);
//compute the EDD and EDD_SD from source intersection to the target point for the road network graph G for V2V data delivery, based on TBD Model (i.e., Per-vehicle Model) in vehicle's current position on the current directional edge; note that this version is for TPD journal submission
	
void VADD_Compute_EDD_And_EDD_SD_Based_On_VADD(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD);
//compute the EDD and EDD_SD based on VADD Model (i.e., Per-intersection Model) in vehicle's current position on the current directional edge

void VADD_Compute_EDD_And_EDD_SD_Based_On_VADD_VERSION_1(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD);
//[OLD VERSION] compute the EDD and EDD_SD based on VADD Model (i.e., Per-intersection Model) in vehicle's current position on the current directional edge

void VADD_Compute_EDD_And_EDD_SD_Based_On_VADD_VERSION_2(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD);
//compute the EDD and EDD_SD based on VADD Model (i.e., Per-intersection Model) in vehicle's current position on the current directional edge; note that this version is for TBD journal submission

void VADD_Compute_EDD_And_EDD_SD_Based_On_VADD_VERSION_3(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD);
//compute the EDD and EDD_SD based on VADD Model (i.e., Per-intersection Model) in vehicle's current position on the current directional edge; [10/17/09] note that this version uses the link delays and link delay deviations from the current road segment and the next intersection

void VADD_Compute_EDD_And_EDD_SD_Based_On_VADD_For_V2V_Data_Delivery(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, int source_intersection_id, double *vehicle_EDD, double *vehicle_EDD_SD);
//compute the EDD and EDD_SD from source intersection to the target point for the road network graph G for V2V data delivery, based on VADD Model (i.e., Per-intersection Model) in vehicle's current position on the current directional edge; note that this version is for TPD journal submission and this function is the same as VADD_Compute_EDD_And_EDD_SD_Based_On_VADD_VERSION_2() except the parameter list
	

boolean VADD_Is_Within_Intersection_Area(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, intersection_area_type_t *intersection_area_type, int *tail_intersection_id, int *head_intersection_id);
//check whether this vehicle is within the Intersection areas of either the tail node or the head node of the current directed edge for the data forwarding to other vehicles moving on other road segments

boolean VADD_Is_There_Next_Carrier_At_Both_Intersection_Areas(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, forwarding_table_queue_t *FTQ, intersection_area_type_t intersection_area_type, struct_vehicle_t **next_carrier);
//check whether this vehicle can forward packets to the other vehicle as next carrier around both the tail intersection area and the head intersection area

boolean VADD_Is_There_Next_Carrier_At_Intersection_Area(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, forwarding_table_queue_t *FTQ, intersection_area_type_t intersection_area_type, struct_vehicle_t **next_carrier);
//check whether this vehicle can forward packets to the other vehicle as next carrier around either the tail intersection area or the head intersection area

boolean VADD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_Area(parameter_t *param, struct_vehicle_t *vehicle, char *tail_node_for_next_forwarding_edge, char *head_node_for_next_forwarding_edge, directional_edge_type_t edge_type, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier);
//determine whether to forward its packets to next carrier moving on the road segment incident to an intersection area corresponding to either the tail intersection or the head intersection and return the pointer to the next carrier through *next_carrier

boolean VADD_Is_Vehicle_Moving_On_Packet_Trajectory(struct_vehicle_t *vehicle, packet_trajectory_queue_t *packet_trajectory);
//check whether vehicle is moving the packet trajectory or not in order to determine the next packet carrier 

double VADD_Get_Initial_Minimum_Neighbor_EDD(parameter_t *param, struct_vehicle_t *vehicle);
//get the initial minimum neighboring EDD for vehicle according to param's data_forwarding_mode and vehicle_vanet_forwarding_type

/* taehwan 20140712 checking range */
boolean isLogOn();
void setLogOnOff(boolean onoff);

#endif

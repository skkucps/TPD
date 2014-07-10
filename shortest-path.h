/**
 *  File: shortest-path.h
	Description: operations for the single-source all-destination shortest path algorithm 
	 based on Dijkstra's shortest path algorithm
	Date: 09/22/2006	
	Maker: Jaehoon Jeong
*/

#ifndef __SHORTEST_PATH_H__
#define __SHORTEST_PATH_H__

#include "graph-data-struct.h"
#include "param.h" //boolean
#include "schedule.h"
#include "access-point-model.h" //struct_access_point_t

/** Amplification of edge length for large scale roadnetwork */
//#define EDGE_SCALE_FACTOR 3
//#define EDGE_SCALE_FACTOR 2
#define EDGE_SCALE_FACTOR 1
//#define EDGE_SCALE_FACTOR 10

/** default sensor density on each edge in the road network */
#define DEFAULT_SENSOR_DENSITY 1


struct_graph_node* Initialize_Graph(parameter_t *param, char *graph_file_name, int *G_size, struct_traffic_table *src_table, struct_traffic_table *dst_table, struct_traffic_table *ap_table, struct_traffic_table *sn_table, double *max_weight);
//initialize a adjacent list representing a graph

void AssignTypeAndRoleToGraphNode(struct_graph_node *G, int G_size, struct_traffic_table *src_table, struct_traffic_table *dst_table);
//assign each graph node's role using src and dst traffic tables

void AssignRoleToGraphNode(struct_graph_node *G, int G_size, struct_traffic_table *table, GRAPH_NODE_ROLE role);
//assign each graph node's role using table

void AssociateGraphEdgeWithEdgeEntry(struct_graph_node *G, int G_size, edge_queue_t *E);
//associate each pair of two adjacent graph nodes (i.e., physical edge) in graph G with the corresponding edge entry in E

void AssociateGraphEdgeWithDirectionalEdgeEntry(struct_graph_node *G, int G_size, directional_edge_queue_t *E);
//associate each pair of two adjacent graph nodes (i.e., physical edge) in graph G with the corresponding directional edge entry in E

struct_set_node* Dijkstra(struct_graph_node *G, int G_size, char *s);
//run Dijkstra's algorithm to compute the single-source shortest path

struct_shortest_path_node* Initialize_MIN_Priority_Queue(struct_graph_node *G, int G_size, char *s);
//initialize a minimum priority queue based on heap for Dijkstra's algorithm

void Relax(struct_shortest_path_node *u, struct_shortest_path_node *v, double w, struct_shortest_path_node *Q, int Q_size, int Q_index, struct_set_node *parent_node);
//void Relax(struct_shortest_path_node *u, struct_shortest_path_node *v, int w, struct_shortest_path_node *Q, int Q_size, int Q_index);
//update the current node u's cost and parent node towards the source

struct_graph_node* Make_Graph_Node(char *vertex, char *weight, char *density, GRAPH_NODE_TYPE type, GRAPH_NODE_ROLE role, double scale_factor);
//allocate the memory for a graph node

struct_graph_node* Make_Graph_Node2(char *vertex, double weight, double density, GRAPH_NODE_TYPE type, GRAPH_NODE_ROLE role);
//allocate the memory for a graph node for node copy operation

struct_set_node* Set_Init();
//initialize the set for shortest path

void Set_Free(struct_set_node *set);
//release the memory occupied by the set for shortest path

struct_set_node* Set_Insert(struct_set_node *S, struct_shortest_path_node *u);
//insert node u into set S

struct_shortest_path_node* Get_Heap_Node(char *vertex, struct_shortest_path_node *Q, int Q_size, int *Q_index);
//search the heap node corresponding to vertex name

void Free_Graph(struct_graph_node *G, int G_size);
//release the memory allocated to G

void Free_Traffic_Table(struct_traffic_table *table);
//release the memory occupied by the traffic table

void Initialize_Path_Table(struct_graph_node *G, int G_size, struct_traffic_table *src_table, struct_path_table *path_table);
//intialize path table including the shortest path information for all the sources in src_table

void Free_Path_Table(struct_path_table *table);
//release the memory occupied by the path table

struct_path_node* Path_List_Init();
//initialize the list for the path

struct_path_node* Path_List_Reverse_Insert(struct_path_node *path_list, struct_set_node *u);
//insert the information of node u into path_list using head node such that node u is the closest to head's next pointer in the head node

struct_path_node* Path_List_Add(struct_path_node *path_list, struct_path_node *u);
//add the information of node u to path_list using head node such that node u is the farest to head's next pointer in the head node

double Weight(struct_graph_node *gnode, char *neighbor);
//return the edge weight between nodes u and v

struct_path_node* Make_Path_List(struct_path_table *path_table, char *src, char *dst, int *path_hop_count);
//make a path list from src to dst using path_table

struct_path_node* Make_Path_List_Before_The_Closest_Protection_Point(struct_path_table *path_table, char *src, char *dst, struct_traffic_table *protection_set, int *path_hop_count);
//make a path list from src to dst using path_table before the closest protection point on the path

struct_path_node* Make_Path_List_For_Given_Trajectory(int *trajectory, int trajectory_size, struct_graph_node *G, int *path_hop_count);
//make a path list from src to dst using a given trajectory

struct_path_node* Make_Forward_Path_List_With_Mobility_List(mobility_queue_t *Q, struct_graph_node *G, int G_size, int *path_hop_count);
//make a forward path list with a given mobility list as a new vehicle trajectory

struct_path_node* Make_Backward_Path_List_With_Mobility_List(mobility_queue_t *Q, struct_graph_node *G, int G_size, int *path_hop_count);
//make a backward path list with a given mobility list as a new vehicle trajectory

void Free_Path_List(struct_path_node *path_list);
//release the memory occupied by the list for the shortest path from source to destination

void Initialize_Schedule_Table(struct_traffic_table *src_table, struct_schedule_table *sched_table); 
//initialize the schedule table to generate the traffic according to the source node and the corresponding distribution

char* Find_Traffic_Source(struct_schedule_table *sched_table, double current_time);
//find the traffic source corresponding to the current time having a new vehicle arrival

int Update_Schedule_Table(struct_schedule_table *sched_table, char *src, double current_time, double delay);
//update the time fields corresponding to the traffic source in schedule table

void Free_Schedule_Table(struct_schedule_table *table);
//release the memory occupied by the schedule table

boolean IsEndOfTravel(struct_path_node *path_list, struct_path_node *path_ptr);
//check whether the vehicle has arrived at its destination or whether path_ptr->vertex is equal to the destination or not

boolean IsProtectionPoint(char *path_vertex, struct_traffic_table *protection_set);
//check whether the vehicle has arrived at one of protection points

boolean IsNeighbor(int X, int Y, struct_graph_node *G, int G_size);
//check whether X is the neighbor of Y or not

int Lookup_Schedule_Table_Index(struct_schedule_table *sched_table, char *src);
//return the index corresponding to src in sched_table

void Store_Graph_Into_File_As_AdjacencyMatrix(struct_graph_node *G, int G_size, char* filename);
//store the adjacency list of graph G into a file in the form of adjacency matrix

void Store_Graph_Into_File_As_AdjacencyList(struct_graph_node *G, int G_size, char* filename, int indicator);
//store the adjacency list of graph G into a file in the form of adjacency list with indicator that determines the title text

void Store_Graph_Into_File_As_AdjacencyList_With_VANET_Statistics(parameter_t *param, struct_graph_node *G, int G_size, char* filename, int indicator);
//store the adjacency list of graph G into a file for VANET statistics in the form of adjacency list with indicator that determines the title text

double** Construct_Adjacency_Matrix(struct_graph_node *G, int G_size);
//construct an adjacency matrix from the adjacent list G representing the graph

struct_graph_node* LookupGraph(struct_graph_node *G, int G_size, char *u); 
//return the pointer to graph node corresponding to vertex u's name

struct_graph_node* LookupGraph_By_NodeID(struct_graph_node *G, int G_size, int node_id);
//return the pointer to graph node corresponding to node id

struct_graph_node* GetNeighborGraphNode(struct_graph_node *G, int G_size, char *u, char *v); 
//return the pointer to graph node corresponding to vertex v that is vertex u's neighbor

double FindShortestPath(struct_traffic_table *src_table, struct_traffic_table *dst_table, int G_size, double **D, char *shortest_path_src, char *shortest_path_dst);
//compute the length of the shortest path from Enter nodes to Exit nodes along with source and destination

int GetSensorNumberForShortestPath(char *src, char *dst, struct_graph_node *G, int n, int **M);
//get the number of sensors (that are live or dead) on the shortest path from src to dst

boolean Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(struct_graph_node **Gv, int *Gv_size, schedule_table_t *T, schedule_table_node_t *pTableNode, double left_hole_offset, double right_hole_offset, edge_queue_t *Er, struct_traffic_table *added_hole_set, struct_traffic_table *deleted_hole_set);
//update virtual graph Gv, schedule table T, and edge queue Er using pTableNode, left_hole_offset and right_hole_offset

void AddNeighborRelationship(struct_graph_node *G, char *node1, GRAPH_NODE_TYPE type1, GRAPH_NODE_ROLE role1, char *node2, GRAPH_NODE_TYPE type2, GRAPH_NODE_ROLE role2, double weight, double density);
//add neighbor relationship between node1 and node2 to graph G

void AddNeighborNode(struct_graph_node *G, char *node, char *neighbor_node, double weight, double density, GRAPH_NODE_TYPE type,  GRAPH_NODE_ROLE role);
//add neighbor node of a given node to adjacency list G

void DeleteNeighborRelationship(struct_graph_node *G, char *node1, char *node2);
//delete neighbor relationship between node1 and node2 from graph G

void DeleteNeighborNode(struct_graph_node *G, char *node, char *neighbor_node);
//delete neighbor node of a given node from adjacency list G

void AddIntersection_EDD_Queue_Relationship(struct_graph_node *G, int G_size, char *tail_node, char *head_node);
//add intersection_edd_queue relationship between virtual_node and tail_node

void  AddIntersection_EDD_Queue_Node(struct_graph_node *G, int G_size, char *tail_node, char *head_node);
//add head_node in tail_node's intersection_edd_queue_node

void DeleteIntersection_EDD_Queue_Relationship(struct_graph_node *G, int G_size, char *tail_node, char *head_node);
//delete intersection relationship between tail_node and head_node from graph G in the intersection_edd_queues of both tail_node and head_node

void  DeleteIntersection_EDD_Queue_Node(struct_graph_node *G, int G_size, char *tail_node, char *head_node);
//delete head_node in tail_node's intersection_edd_queue_node

void InitTrafficTable(struct_traffic_table* table);
//initialize traffic table

void MakeTrafficTable(struct_traffic_table* table, char *node1, char *node2);
//make traffic table with node1 and node2

void CopyTrafficTable(struct_traffic_table *dst, struct_traffic_table *src);
//copy traffic table from src to dst

void MergeTrafficTable(struct_traffic_table *dst, struct_traffic_table *src);
//merge the traffic table of src to the traffic table of dst

boolean AddTrafficTableEntry(struct_traffic_table *table, char *node);
//add traffic table entry corresponding to node to table

boolean IsVertexInTrafficTable(struct_traffic_table *table, char *node);
//check whether node already belongs to table

void SubtractTrafficTable(struct_traffic_table *table, struct_traffic_table *set);
//delete nodes in set from table

boolean DeleteTrafficTableEntry(struct_traffic_table *table, char *node);
//delete traffic table entry corresponding to node from table

int GetTrafficTableEntry_ID_With_Index(struct_traffic_table *table, int index);
//get the intersection id for a traffic table entry with index

void Adjust_Maximum_AccessPoint_Number(parameter_t *param, struct_traffic_table *ap_table);
//adjust the number of multiple APs according to the number of maximum APs

void Store_Sensing_Hole_Endpoints_And_Labeling_Into_File(edge_queue_t *Er, struct_traffic_table *src_table_for_Gr, struct_traffic_table *dst_table_for_Gr, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, char* filename, int indicator);
//store sensing hole endpoints for each edge in the real graph Gr with labeling (i.e., CLUSTER_ENTRANCE or CLUSTER_PROTECTION) into file

void Store_Traffic_Table_Into_File(FILE *fp, struct_traffic_table *traffic_table);
//store traffic table into file pointed by file pointer fp

void Store_Sensing_Hole_Endpoints_Into_File(FILE *fp, edge_queue_t *Er);
//store initial sensing hole endpoints for each edge in the real graph Gr into file pointed by file pointer fp

void SortTrafficTable(struct_traffic_table *T);
//sort traffic table entries in ascending order according to vertex name

void SortSensingHoleEndpointQueue(hole_endpoint_queue_t *Q);
//sort the sensing hole endpoints of the hole endpoint queue Q according to the offsets in the real graph Gr in the ascending order

void RearrangeSensingHoleEndpointQueue(hole_endpoint_queue_t* Q, hole_endpoint_queue_node_t **A);
//rearrange the pointers of sensing hole endpoint queue Q using A in the ascending order

struct_graph_node* GetUnusedGraphNode(struct_graph_node **Gv, int *Gv_size);
//get an unused graph node from the array including Gv; if needed, update the information for virtual graph in T

void UpdateEdgeWeight(struct_graph_node *G, char *u, char *v, double new_weight);
//update the weights of the edge (u,v) and edge (v,u)

void DeleteGraphNode(struct_graph_node *G, int G_size, char *u);
//delete graph node u from graph G for future reuse for another vertex

void SetPhysicalPosition(char *node, double virtual_offset, struct_graph_node *G, int G_size, schedule_table_node_t *pTableNode);
//set node's physical position (i.e., pos_in_Gr) to the actual physical position corresponding to virtual offset right_hole_offset in Gv.

double GetPhysicalOffset(char *node, struct_graph_node *G, int G_size);
//get node's physical offset in the physical edge in the real graph G.

double SetPhysicalOffset(char *node, double new_offset, struct_graph_node *G, int G_size);
//set node's physical offset in the physical edge in the real graph G with new_offset.

struct_graph_node* Make_Forwarding_Graph(struct_graph_node* G, int G_size, int* G_new_size);
//make a new forwarding graph for data forwarding used by Access Point (AP) based on the road network graph G

void CopyGraphNodeAndNeighborList(struct_graph_node* src, struct_graph_node* dst, struct_graph_node* G, int G_size);
//copy src's node information and neighbor list into dst

void SetGraphNodeStatus(struct_graph_node *G, int G_size, USAGE_STATUS status);
//set graph node's status to status, such as USAGE_USED, in order to let LookupGraph() return the pointer to the graph node correctly

void Make_Intersection_EDD_Queue_And_NeighborList_For_VirtualNode(int virtual_node_id, struct_graph_node* dst, struct_graph_node* Gr, int Gr_size, struct_graph_node* Ga, int Ga_size, edge_queue_t *E, directional_edge_queue_t *DE, char *tail_node, char *head_node, double left_edge_length, double right_edge_length);
//make the intersection EDD queue and the neighbor list for a virtual node that is the target point for the packet delivery towards a vehicle

boolean PickTargetPoint(int *trajectory, int trajectory_size, double target_point_interdistance, int target_point_index, struct_graph_node *G, struct_coordinate3_t *target_point);
//pick a target point on the target vehicle's trajectory

void AugmentGraph_With_TargetPoint(struct_access_point_t *AP, struct_coordinate3_t *target_point, struct_graph_node *Gr, int Gr_size);
//make an augmented graph Ga for data forwarding with the road network graph Gr and the target point

void Set_IntermediateNode_GeographicCoordinate(struct_graph_node *virtual_node_gnode, struct_graph_node *tail_node_gnode, struct_graph_node *head_node_gnode, double left_edge_length, double right_edge_length);
//determine the geographic coordination of the intermedidate node with the tail_node's and head_node's ones

void Expand_Graph_Memory(struct_graph_node **G, int *G_size);
//expand graph G by one to accommodate another graph node

void Update_NeighborNode_Gnode(struct_graph_node *G, int G_size);
//update the values of the gnode pointers of the neighbor nodes in the neighbor lists

void Update_Intersection_EDD_QueueNode_Gnode(struct_graph_node *G, int G_size);
//update the values of the gnode pointers of tail_gnode and head_gnode in the intersection_edd_queues for each node in the node array for G

void CopyVehicularTrafficStatistics(struct_graph_node *Gr, int Gr_size, struct_graph_node *Ga, int Ga_size);
//copy the vehicular traffic statistics of the edges in Gr into that in Ga

void CopyVehicularTrafficStatistics_For_NeighborList(struct_graph_node *src, struct_graph_node *dst);
//copy the vehicular traffic information in src's neighbor list into that in dst

void CopyVehicularTrafficStatistics_From_DirectionalEdges_To_DirectionalSubedges(struct_graph_node *Gr, int Gr_size, struct_graph_node *Ga, int Ga_size, char *tail_node, char *head_node, char *virtual_node);
//copy the vehicular traffic information in the directional edges into its subdivided subedXSges in the directional edges

void Set_Forwarding_Information_For_Multiple_APs(parameter_t *param, struct_graph_node *Gr, int Gr_size, struct_graph_node **G_set, int *G_set_size, int ap_number);
//set the real graph Gr with the graph set G_set after processing the EDD for each access point

struct_graph_node* Get_Minimum_EDD_GraphNode(char *tail_vertex, char *head_vertex, struct_graph_node **G_set, int *G_set_size, int ap_number);
//get the pointer to the head node with a minimum EDD among the graphs in the graph set G_set given an edge (tail->vertex, head->vertex)

void SetTargetPoint_In_TafficTable(struct_traffic_table *table, char *target_point);
//set a target point in traffice table

int GetTargetPoint_For_Multiple_APs(parameter_t *param, double current_time, access_point_queue_t *APQ, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, double *EDD_p, double *EAD_p, access_point_queue_node_t **transmitter_AP_qnode);
//In the case where multiple APs exist, this function returns the pointer to the AP queue node corresponding to the packet-transmitter AP in order to let the AP with the shortest delivery delay transmit this packet towards the destination vehicle

int GetTargetPoint_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, double *EDD_p, double *EAD_p);
//get the intersection id of a target point for AP where the target point is on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle

int GetTargetPoint_For_AP_For_V2V_Data_Delivery(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, forwarding_table_queue_t *FTQ, int Gr_set_number, struct_graph_node **Gr_set, int *Gr_set_size, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
//get the id of a target point for AP such as an intersection on the destination vehicle's trajectory used to deliver a packet towards the destination vehicle

int GetMultipleTargetPoints_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, target_point_queue_t *global_TPQ);
// get the list of intersection ids of target points for AP where each target point is on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle

int GetTargetPoint_For_Carrier(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p, double *EAD_p);
//get the intersection id of a target point for carrier vehicle where the target point is on the destination vehicle trajectory in the carrier's packet

int GetTargetPoint_For_Carrier_At_TrajectoryIntersection_Before_DestinationVehicle(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, vehicle_trajectory_queue_t *pTrajectory_Queue, vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode, int destination_vehicle_hop, double destination_vehicle_offset, int vertex_hop, double *EDD_p, double *EAD_p);
//get the intersection id of a target point for carrier vehicle that is within the communication range of an intersection on the destination vehicle trajectory where the intersection is before the destination vehicle on the trajectory

int GetTargetPoint_For_StationaryNode(parameter_t *param, double current_time, stationary_node_queue_node_t *stationary_node, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
//get the intersection id of a target point for the stationary node where the target point is on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle

int GetTargetPoint_For_Packet(parameter_t *param, double current_time, packet_queue_node_t *packet, int current_intersection_id, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
//get the intersection id of a target point for the packet where the target point is on the packet's vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle

int GetTargetPoint_By_HeadingIntersection_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
// get the id of a target point for AP, such as intersection id on the vehicle trajectory, used to deliver a packet towards vehicle that is a destination vehicle

int GetTargetPoint_By_EndIntersection_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
// get the id of a target point for AP for the end intersection of destination vehicle's trajectory

int GetTargetPoint_By_RandomIntersection_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
// get the id of a target point for AP, such as a random intersection id on the vehicle trajectory used to deliver a packet towards the vehicle that is a destination vehicle

int GetMultipleTargetPoints_By_RandomIntersection_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, target_point_queue_t *TPQ);
// get the list of the ids of target points for AP, such as random intersection ids on the vehicle trajectory used to deliver the copies of a packet towards the vehicle that is a destination vehicle

int GetTargetPoint_By_OptimalIntersection_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
// get the id of an optimal target point (e.g., intersection point or arbitrary point) on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle

int GetTargetPoint_By_PacketEarliestArrivingIntersection_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
// get the id of a target point p for AP such as intersection point on the destination vehicle's trajectory where p is geographically closest to the destination vehicle such that EDD_p <= EAD_p

int GetTargetPoint_By_PacketTrajectory_For_Multiple_APs(parameter_t *param, double current_time, access_point_queue_t *APQ, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point, access_point_queue_node_t **transmitter_AP_qnode);
//In the Multiple-AP road network, with the packet trajectory from the AP to the destination, return the id of an optimal target point for AP such as intersection point or arbitrary point on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle and also return the pointer to the AP queue node for the actual transmitter AP 

int GetTargetPoint_By_EndIntersection_For_Multiple_APs(parameter_t *param, double current_time, access_point_queue_t *APQ, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point, access_point_queue_node_t **transmitter_AP_qnode);
//In the Multiple-AP road network, we select a target point with the last intersection on the vehicle trajectory and select the actual transmitter AP with the one having the shortest delivery delay to the last intersection and also return the pointer to the AP queue node for the actual transmitter AP 

int GetTargetPoint_By_RandomIntersection_For_Multiple_APs(parameter_t *param, double current_time, access_point_queue_t *APQ, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point, access_point_queue_node_t **transmitter_AP_qnode);
//In the Multiple-AP road network, we select a random target point among the intersections on the vehicle trajectory and select the actual transmitter AP with the one having the shortest delivery delay to the target point and also return the pointer to the AP queue node for the actual transmitter AP 

int GetTargetPoint_By_PacketTrajectory_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
//With the packet trajectory from the AP to the destination, get the id of an optimal target point for AP such as intersection point on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle

int GetMultipleTargetPoints_By_PacketTrajectory_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, target_point_queue_t *TPQ);
//With the packet trajectory from the AP to the destination, get the list of the ids of optimal target points for AP such as intersection points on the vehicle trajectory used to deliver the copies of a packet towards the vehicle that is a destination vehicle

int GetTargetPoint_By_HeadingIntersection_For_Carrier(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
//get the id of a target point for carrier vehicle, such as the destination vehicle's heading intersection on its trajectory, used to deliver a packet towards vehicle that is a destination vehicle

int GetTargetPoint_By_RandomIntersection_For_Carrier(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
//get the id of a target point for carrier vehicle, such as a random intersection on the destination vehicle trajectory, used to deliver a packet towards vehicle that is a destination vehicle

int GetTargetPoint_By_OptimalIntersection_For_Carrier(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
// get the id of an optimal target point for carrier vehicle such as intersection point or arbitrary point on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle

int GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
// get the id of a target point p for carrier vehicle such as intersection point on the destination vehicle's trajectory where p is geographically closest to the destination vehicle such that EDD_p <= EAD_p 

int GetTargetPoint_By_ClosestIntersection_For_Carrier(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
// get the id of a target point p for carrier vehicle such that the target point p is the geographically closest point on the destination vehicle's trajectory from the carrier's current position 

int GetTargetPoint_By_PacketTrajectory_For_Carrier(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
//With the packet trajectory from the carrier to the destination, get the id of an optimal target point for carrier vehicle such as intersection point or arbitrary point on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle

int GetTargetPoint_By_HeadingIntersection_For_Carrier_At_TrajectoryIntersection_Before_DestinationVehicle(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, vehicle_trajectory_queue_t *pTrajectory_Queue, vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode, int destination_vehicle_hop, double destination_vehicle_offset, int vertex_hop, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
//get the id of a target point for carrier vehicle that is the destination vehicle's heading intersection between destination_vehicle_hop and vertex_hop (carrier's position) on the destination vehicle trajectory

int GetTargetPoint_By_RandomIntersection_For_Carrier_At_TrajectoryIntersection_Before_DestinationVehicle(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, vehicle_trajectory_queue_t *pTrajectory_Queue, vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode, int destination_vehicle_hop, double destination_vehicle_offset, int vertex_hop, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
//get the id of a target point for carrier vehicle that is a random intersection point between destination_vehicle_hop and vertex_hop (carrier's position) on the destination vehicle trajectory

int GetTargetPoint_By_OptimalIntersection_For_Carrier_At_TrajectoryIntersection_Before_DestinationVehicle(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, vehicle_trajectory_queue_t *pTrajectory_Queue, vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode, int destination_vehicle_hop, double destination_vehicle_offset, int vertex_hop, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point);
// get the id of an optimal target point for carrier vehicle that is an intersection point between destination_vehicle_hop and vertex_hop (carrier's position) on the destination vehicle trajectory

double Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList(double current_time, struct_vehicle_t *vehicle, char *target_point);
//compute the path distance for the trajectory from the current position to target_point with vehicle's path list

double Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList_Along_With_PathTravelTime_And_Deviation(parameter_t *param, double current_time, struct_vehicle_t *vehicle, char *target_point, double *travel_time, double *travel_time_deviation);
//compute the path distance, the path travel time and the deviation for the trajectory from the current position to target_point with vehicle's path list

double Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory(double current_time, vehicle_trajectory_queue_t *pTrajectory_Queue, vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode, double offset_in_current_edge, char *target_point);
//compute the path distance for the trajectory from the current position to target_point with destination vehicle trajectory where destination_vehicle_travel_distance is the distance from the trajectory's start position to the destination vehicle's current position

double Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory_Along_With_PathTravelTime_And_Deviation(parameter_t *param, double current_time, vehicle_trajectory_queue_t *pTrajectory_Queue, vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode, double offset_in_current_edge, char *target_point, double *travel_time, double *travel_time_deviation);
//compute the path distance, the path travel time and the deviation for the trajectory from the current position to target_point with destination vehicle trajectory where destination_vehicle_travel_distance is the distance from the trajectory's start position to the destination vehicle's target point

double Compute_PathDistance_From_CurrentPosition_To_EndPosition_With_VehicleTrajectory_Along_With_PathTravelTime_And_Deviation(parameter_t *param, double current_time, packet_queue_node_t *packet, double *travel_time, double *travel_time_deviation);
//compute the vehicle's path distance, the path travel time and the deviation for the trajectory from the current position to the end position on the vehicle trajectory where destination_vehicle_travel_distance is the distance from the trajectory's start position to the destination vehicle's end position on its vehicle trajectory

double Compute_PathDistance_For_VehicleTrajectory(vehicle_trajectory_queue_t *pTrajectory_Queue);
//compute the path distance for the vehicle trajectory from the start position to the end position

double Compute_TravelTime_For_VehicleTrajectory(parameter_t *param, double current_time, vehicle_trajectory_queue_t *pTrajectory_Queue);
//compute the travel time for the path distance for the trajectory from the start position to the end position in the vehicle trajectory. 

int Find_NewTargetPoint_Where_Vehicle_Is_Close_To_DestinationVehicleTrajectory(parameter_t *param, double current_time, struct_vehicle_t *vehicle, forwarding_table_queue_t *FTQ, intersection_area_type_t input_intersection_area_type);
//[NEW] find a new target point to which vehicle is closest from the intersection area containing vehicle and which is on the destination vehicle trajectory
//[OLD] find a new target point where the intersection area(s) having vehicle has an intersection on the destination vehicle

vehicle_trajectory_queue_node_t* Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, int *current_hop, double *current_offset);
//find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory for carrier_vehicle 

vehicle_trajectory_queue_node_t* Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_StationaryNode(parameter_t *param, double current_time, stationary_node_queue_node_t *stationary_node, int *current_hop, double *current_offset);
//find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory for stationary_node 

vehicle_trajectory_queue_node_t* Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_Packet(parameter_t *param, double current_time, packet_queue_node_t *pPacket, int *current_hop, double *current_offset);
//find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the vehicle trajectory contained in packet 

int Find_VertexHop_On_VehicleTrajectory(vehicle_trajectory_queue_t *pTrajectory_Queue, char *vertex);
//find the hop of vertex on the destination vehicle trajectory

vehicle_trajectory_queue_node_t* Find_VehicleTrajectoryQueueNode_For_VertexHop_On_VehicleTrajectory(vehicle_trajectory_queue_t *pTrajectory_Queue, char *vertex);
//find the vehicle trajectory queue node corresponding to the hop of vertex on the destination vehicle trajectory

void Update_LinkDelay_Information(parameter_t *param, struct_graph_node *G, int G_size);
//update the link delay, link delay variance, link delay standard deviation for each edge in the graph G with the traffic statistics of vehicular traffic density and vehicle speed

double Compute_TargetPoint_OptimizationValue(parameter_t *param, int target_point_id, double EDD_p, double EDD_SD_p, double EAD_p, double EAD_SD_p, double packet_ttl, double *max_constraint_value, int *max_constraint_value_target_point_id);
//compute the optimization value for an intersection p that is an optimal target point candidate given the the packet delivery delay distribution (EDD_p, EDD_SD_p) and the destination vehicle arrival delay distribution (EAD_p, EAD_SD_p). Note that max_constraint_value and max_constraint_value_target_point_id are used only in the case where there is no target point to satisfy the required constraint, such as delivery probability threshold

boolean Is_Vertex_On_VehicleTrajectory_With_New_Destination(parameter_t *param, double current_time, packet_queue_node_t *packet, int vertex, int *new_dst, boolean *packet_earlier_arrival_flag);
//check whether vertex is on the destination vehicle's trajectory; if so, this function returns an adjacent vertex closer to the destination vehicle's current position through new_dst

boolean Is_Vertex_On_VehicleTrajectory_With_New_Destination_VERSION_1(parameter_t *param, double current_time, packet_queue_node_t *packet, int vertex, int *new_dst, boolean *packet_earlier_arrival_flag);
//Using the estimation of the destination vehicle's position, check whether vertex is on the destination vehicle's trajectory; if so, this function returns an adjacent vertex closer to the destination vehicle's current position through new_dst

boolean Is_Vertex_On_VehicleTrajectory_With_New_Destination_VERSION_2(parameter_t *param, double current_time, packet_queue_node_t *packet, int vertex, int *new_dst, boolean *packet_earlier_arrival_flag);
//Using the destination vehicle passing time for the target point, check whether vertex is on the destination vehicle's trajectory; if so, this function returns an adjacent vertex closer to the destination vehicle's current position through new_dst

boolean Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination(parameter_t *param, double current_time, packet_queue_node_t *packet, int *new_dst);
//check whether the packet has already arrived earlier at the target point than the destination vehicle or not; regardless of the result, it returns a new destination node

boolean Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination_VERSION_1(parameter_t *param, double current_time, packet_queue_node_t *packet, int *new_dst);
//Using the estimation of the destination vehicle's position, check whether the packet has already arrived earlier at the target point than the destination vehicle or not; regardless of the result, it returns a new destination node

boolean Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination_VERSION_2(parameter_t *param, double current_time, packet_queue_node_t *packet, int *new_dst);
//Using the destination vehicle passing time for the target point, check whether the packet has already arrived earlier at the target point than the destination vehicle or not; regardless of the result, it returns a new destination node

boolean Is_There_NextHop_In_PacketReverseTraversal(packet_queue_node_t *packet, int current_intersection_id, int *new_dst);
//check whether there exists a next hop for the packet's reverse traversal given the vehicle trajectory and the current intersection id; if so, return the next hop through next_dst

boolean Update_CurrentHop_VehicleTrajectoryQueueNode_In_PacketReverseTraversal(packet_queue_node_t *packet);
//update the current-hop vehicle trajectory queue node with the next hop in the packet reverse traversal

void Update_LinkCost_Information(parameter_t *param, struct_graph_node *G, int G_size);
//update the link cost (e.g., link utilization), link cost variance, link cost standard deviation for each edge in the graph G with the traffic statistics of vehicular traffic density, vehicle speed, and communication range

void Init_RoadNetworkGraphNode_StationaryNodeFlag_With_StationaryNodeList(parameter_t *param, struct_graph_node *Gr, int Gr_size, struct_traffic_table *sn_table_for_Gr);
/* specify which intersections have their stationary node as temporary packet holder in the road network graph Gr */

void Set_RoadNetworkGraphNode_StationaryNodeFlag(struct_graph_node *Gr, int Gr_size);
/* set the stationary node flags of all the graph nodes in Gr */

#endif

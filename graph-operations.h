/**
 *  File: graph-operations.h
	Description: Graph operations, such as Breadth First Search.
	Date: 11/8/2010
	Maker: Jaehoon Jeong
*/

#ifndef __GRAPH_OPERATIONS_H__ /* [ */
#define __GRAPH_OPERATIONS_H__

int GO_Delete_Edges_From_Graph_With_PacketTrajectoryQueue(double **M, int matrix_size, packet_trajectory_queue_t *PTQ);
/* delete the edges of the path corresponding to the packet trajectory PTQ from adjacency matrix A representing a graph */

int GO_Add_Edges_To_Graph_With_PacketTrajectoryQueue(double **M, int matrix_size, packet_trajectory_queue_t *PTQ, int start_vertex, int end_vertex);
/* add the edges of the path from start_vertex to end_vertex corresponding to the packet trajectory PTQ to adjacency matrix M representing a graph where start_vertex and end_vertex are virtual vertices, so we do not count the edges related to these virtual edges */
/* Note that for start_vertex = end_vertex = 0, we add all the edges in PTQ to M */

int GO_Find_ComponentVertices(double **M, int matrix_size, int u, component_vertex_queue_t *CVQ);
/* find the vertices connected to a vertex u in the graph denoted by adjacency matrix M by Breadth First Search (BFS) */

boolean GO_Find_Best_Alternative_Path_For_Tree_Connectivity_With_DeliveryProbabilityConstraint(parameter_t *param, double current_time, int src, target_point_queue_t *TPQ, destination_vehicle_queue_t *DVQ, double **A, int A_size, const double **T, int T_size, packet_trajectory_queue_t *alternative_path);
  /* make alernative path q that is a delay bounded shortest path between trees 
   * T1 and T2 in the forest T by computing k-shortest paths from v1 to v2 in 
   * tree T_new described by adjacency matrix A such that v1 is connected to 
   * all vertices in T1 as tail node and v2 is connected by all vertices in T2 
   * as head node.
   * Note that the delivery probability to each target point in target point 
   * queue TPQ in the tree consisting of T and alternative path should be 
   * satisfied to connect these two tree by the best alternative path. */

boolean GO_Find_Best_Alternative_Path_For_Tree_Connectivity_With_DeliveryProbabilityConstraint_And_DeliveryDelayConstraint(parameter_t *param, double current_time, int src, target_point_queue_t *TPQ, destination_vehicle_queue_t *DVQ, double **A, int A_size, const double **T, int T_size, double beta, packet_trajectory_queue_t *alternative_path);
  /* make alernative path q that is a delay bounded shortest path between trees 
   * T1 and T2 in the forest T by computing k-shortest paths from v1 to v2 in 
   * tree T_new described by adjacency matrix A such that v1 is connected to 
   * all vertices in T1 as tail node and v2 is connected by all vertices in T2 
   * as head node.
   * Note that the delivery probability (for delivery ratio alpha) and 
   * the delivery delay (for delay threshold beta) to each target point in 
   * target point queue TPQ in the tree consisting of T and alternative path 
   * should be satisfied to connect these two tree by the best alternative path. */

boolean GO_Find_Best_Alternative_Path_For_Tree_Connectivity_Without_Constraint(parameter_t *param, double current_time, int src, target_point_queue_t *TPQ, destination_vehicle_queue_t *DVQ, double **A, int A_size, const double **T, int T_size, packet_trajectory_queue_t *alternative_path);
  /* make alernative path q that is a delay bounded shortest path between trees 
   * T1 and T2 in the forest T by computing k-shortest paths from v1 to v2 in 
   * tree T_new described by adjacency matrix A such that v1 is connected to 
   * all vertices in T1 as tail node and v2 is connected by all vertices in T2 
   * as head node.
   * Note that the delivery probability to each target point in target point 
   * queue TPQ in the tree consisting of T and alternative path is not guaranted 
   * to connect these two tree by the best alternative path. */

boolean GO_Does_ForwardingTree_Satisfy_PacketEarlierArrival_Than_DestinationVehicle_In_Mean_Arrival_Time(parameter_t *param, double current_time, double **T, double **D, double **S, int T_size, int src, destination_vehicle_queue_node_t *destination_vehicle);
/* check whether or not the augmented tree satisfies the packet's ealier arrival than the destination vehicle in mean arrival time where T is the delay cost matrix for the adjacency matrix of the forwarding tree T, D is the delay matrix, and S is the delay variance matrix */

boolean GO_Does_ForwardingTree_Satisfy_FinitePacketDeliveryDelay_For_DestinationVehicle(parameter_t *param, double current_time, double **T, double **D, double **S, int T_size, int src, destination_vehicle_queue_node_t *destination_vehicle);
/* check whether or not the augmented tree satisfies the finite packet delivery delay to the target points in the destination vehicle's FTPQ where T is the delay cost matrix for the adjacency matrix of the forwarding tree T, D is the delay matrix, and S is the delay variance matrix */

boolean GO_Does_ForwardingTree_Satisfy_DeliveryConstraint_For_DestinationVehicle(parameter_t *param, double current_time, double **T, double **D, double **S, int T_size, int src, destination_vehicle_queue_node_t *destination_vehicle);
/* check whether or not the augmented tree satisfies the delivery ratio for all the target points for a destination vehicle where T is the delay cost matrix for the adjacency matrix of the forwarding tree T, D is the delay matrix, and S is the delay variance matrix */

boolean GO_Does_ForwardingTree_Satisfy_AllDeliveryConstraints_For_DestinationVehicle(parameter_t *param, double current_time, double **T, double **D, double **S, int T_size, int src, destination_vehicle_queue_node_t *destination_vehicle, double beta);
/* check whether or not the augmented tree satisfies the delivery ratio and the delivery delay bound beta for all the target points for a destination vehicle where T is the delay cost matrix for the adjacency matrix of the forwarding tree T, D is the delay matrix, and S is the delay variance matrix */

int GO_Construct_EdgeQueue_With_AdjacencyMatrix_And_FeasibleTargetPointQueue(double **T, int T_size, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ);
/* construct edge set queue ESQ with adjacency matrix T and feasible target point queue FTPQ */

void GO_Construct_AdjacencyMatrix_With_EdgeQueue(edge_set_queue_t *ESQ, double **T, int T_size);
/* construct adjacency matrix T with edge set queue ESQ */

int GO_Find_RedundantTargetPoint_From_EdgeSetQueue_AlongWith_Queue_Adjustment(int Gr_size, destination_vehicle_queue_t *DVQ, edge_set_queue_t *ESQ);
/* find a redundant target point that can be removed from the multicast tree represented as edge set queue ESQ and then adjust DVQ and ESG for the deletion of a target point */

void GO_FilterOut_RedundantTargetPoints_From_Anycast_Sets(target_point_queue_t *FTPQ, destination_vehicle_queue_t *DVQ);
/* filter out target points per destination vehicle's anycast set with FTPQ */

void GO_Mark_Duplicate_Covered_TargetPoint_In_TargetPointSet(double current_time, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ);
/* mark target points in FTPQ that are covered by more than one anycast set
 * by setting cover_flag to TRUE for those target points that are redundant
 * target points as relay nodes in the feasible target point queue FTPQ */

void GO_Rebuild_AdjacencyMatrix_With_SpanningTree(double **T, int T_size, boolean *spanning_tree_node_flag_vector, int spanning_tree_node_flag_vector_size);
/* rebuild the adjacency matrix T with spanning_tree_node_flag_vector for the spanning tree where the vertex with flag TRUE is one of the spanning tree vertices; note that the index is the vertex id, so index 0 has no vertex */

void GO_Construct_EdgeQueue(parameter_t *param, double current_time, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ);
/* construct edge set queue ESQ with Minimum Steiner Tree (MST) based on FTPQ */

#endif /* ] */



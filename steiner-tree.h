/** File: steiner-tree.h
	Description: specify the macro constants, structures, and enum types for Steiner Tree.
	Date: 10/24/2010
	Maker: Jaehoon Jeong, jjeong@cs.umn.edu
*/

#ifndef __STEINER_TREE_H__
#define __STEINER_TREE_H__


/** function declarations */
int ST_Get_GreedyMulticastTree(parameter_t *param, double current_time, int src, target_point_queue_t *FTPQ, destination_vehicle_queue_t *DVQ);
//get a greedy multicast tree (i.e., delay-constrained mininum Steiner tree) where FTPQ is the set of destination intersections (i.e., feasible target points).

int ST_Get_GreedyMulticastTree_With_DelayConstraint(parameter_t *param, double current_time, int src, target_point_queue_t *FTPQ, destination_vehicle_queue_t *DVQ, double beta);
//get a greedy multicast tree (i.e., delay-constrained mininum Steiner tree) with delay constraint beta where FTPQ is the set of destination intersections (i.e., target points); note that any forwarding path from AP to a target point must have a delivery delay shorter than or equal to beta.

int ST_Get_ShortestPathTree(parameter_t *param, double current_time, int src, target_point_queue_t *FTPQ, destination_vehicle_queue_t *DVQ);
//get the shortest path tree (i.e., the shortest delay path tree) where FTPQ is the set of destination intersections (i.e., target points).

int ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree(parameter_t *param, double current_time, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ);
/* filter out redundant target points from Minimum Steiner Tree T consisting of anycast sets of destination vehicles and make a pruned multicast tree represented as edge set queue ESQ */

int ST_FilterOut_RedundantTargetPoints_From_ShortestPathTree(parameter_t *param, double current_time, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ);
/* filter out redundant target points from the Shortest Path Tree T consisting of anycast sets of destination vehicles and make a pruned multicast tree represented as edge set queue ESQ */

int ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree_With_MinimumSpanningTreeAlgorithm(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ);
/* filter out redundant target points from Minimum Steiner Tree T consisting of anycast sets of destination vehicles with Minimum Spanning Tree Algorithm and make a pruned multicast tree represented as edge set queue ESQ */

int ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree_With_MinimumSpanningTreeAlgorithm_And_DeliveryDelayBound(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, double beta, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ);
/* filter out redundant target points from Minimum Steiner Tree T consisting of anycast sets of destination vehicles with Minimum Spanning Tree Algorithm along with delivery delay bound beta and make a pruned multicast tree represented as edge set queue ESQ */

int ST_FilterOut_RedundantTargetPoints_From_AllAnycastSets_With_MinimumSpanningTreeAlgorithm_And_DeliveryDelayBound(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, double beta, target_point_queue_t *FTPQ);
/* filter out redundant target points from All Anycast Sets of destination vehicles with Minimum Spanning Tree Algorithm along with delivery delay bound beta and make a feasible target point set FTPQ */ 

void ST_FilterOut_InfeasibleTargetPoints_From_AllAnycastSets(destination_vehicle_queue_t *DVQ, double beta);
/* filter out target points not satisfying the delay bound beta from the feasible target point queues in DVQ */

void ST_FilterOut_InfeasibleTargetPoints_From_AnycastSet(target_point_queue_t *FTPQ, double beta);
/* filter out target points not satisfying the delay bound beta from the feasible target point queue FTPQ */

/** Multicast Tree Construction Operations */
void ST_Construct_Multicast_Tree(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet);
/* construct a multicast tree according to the construction algorithm */

void ST_Construct_Multicast_Forest(parameter_t *param, double current_time, access_point_queue_t *APQ, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet);
/* construct a multicast forest consisting of multiple multicast trees whose root is an AP according to the construction algorithm */

/** Construction of Multicast Tree in Single AP Scenarios */
void ST_Construct_Multicast_Tree_Based_On_TMA_Feasible_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet);
/* construct a multicast tree according to the TMA-feasible-target-point-selection algorithm */

void ST_Construct_Multicast_Tree_Based_On_Random_Feasible_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet);
/* construct a multicast tree according to the Random-feasible-target-point-selection algorithm */

void ST_Construct_Multicast_Tree_Based_On_MST_Feasible_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet);
/* construct a multicast tree according to the MST-feasible-target-point-selection algorithm */

void ST_Construct_Multicast_Tree_Based_On_Minimum_Delay_Feasible_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet);
/* construct a multicast tree according to the Minimum-delay-feasible-target-point-selection algorithm */

void ST_Construct_Multicast_Tree_Based_On_Minimum_Cost_Feasible_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet);
/* construct a multicast tree according to the Minimum-cost-feasible-target-point-selection algorithm */

void ST_Construct_Multicast_Tree_Based_On_Simple_AST_Feasible_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet);
/* construct a multicast tree according to the Simple Anycast-set Spanning Tree (SAST) Feasible-target-point-selection algorithm */

void ST_Construct_Multicast_Tree_Based_On_AST_Feasible_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet);
/* construct a multicast tree according to the Anycast-set Spanning Tree (AST) Feasible-target-point-selection algorithm */

void ST_Construct_Multicast_Tree_Based_On_Simple_Random_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet);
/* construct a multicast tree according to the Simple-Random-target-point-selection algorithm */

void ST_Construct_Multicast_Tree_Based_On_Flooding_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet);
/* construct a multicast tree according to the Flooding-target-point-selection algorithm */

void ST_Construct_Multicast_Tree_Based_On_Last_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet);
/* construct a multicast tree according to the Last-target-point-selection algorithm */

void ST_Construct_Multicast_Tree_Based_On_Simple_Minimum_Delay_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet);
/* construct a multicast tree according to the Simple-Minimum-delay-target-point-selection algorithm */

void ST_Construct_Multicast_Tree_Based_On_Simple_Minimum_Cost_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet);
/* construct a multicast tree according to the Simple-Minimum-cost-target-point-selection algorithm */

void ST_Construct_Multicast_Tree_Based_On_SPT_Feasible_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet);
/* construct a multicast tree according to the Shortest Path Tree (SPT) Feasible-target-point-selection algorithm such that the shortest path is the shortest-delay path */

void ST_Construct_Multicast_Tree_Based_On_Simple_SPT_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet);
/* construct a multicast tree according to the Simple Shortest Path Tree (SSPT) target-point-selection algorithm such that the shortest path is the shortest-delay path */

/** Construction of Multicast Forest in Multiple AP Scenarios */
void ST_Construct_Multicast_Forest_Based_On_Minimum_Cost_Feasible_Target_Point_Selection(parameter_t *param, double current_time, access_point_queue_t *APQ, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet);
/* construct a multicast forest according to the Minimum-cost-feasible-target-point-selection algorithm */

#endif

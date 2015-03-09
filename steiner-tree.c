/** File: steiner-tree.c
	Description: specify the macro constants, structures, and enum types for Steiner Tree.
	Date: 10/24/2010
	Maker: Jaehoon Jeong, jjeong@cs.umn.edu
*/

#include "stdafx.h"
#include "graph-data-struct.h"
#include "queue.h"
#include "shortest-path.h"
#include "all-pairs-shortest-paths.h"
#include "graph-operations.h"
#include "util.h"
#include "steiner-tree.h"

/** Multicast Tree Construction Operations */
void ST_Construct_Multicast_Tree(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet)
{ /* construct a multicast tree according to the construction algorithm */
	/** construct a multicast forwarding tree based on the multicast-tree-construction alogirthm */
	switch(param->multicast_tree_construction_algorithm)
	{
	case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_TMA_FEASIBLE_TARGET_POINT_SELECTION:
		ST_Construct_Multicast_Tree_Based_On_TMA_Feasible_Target_Point_Selection(param, current_time, AP_vertex, DVQ, FTPQ, ESQ, global_packet);
		break;

	case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_RANDOM_FEASIBLE_TARGET_POINT_SELECTION:
		ST_Construct_Multicast_Tree_Based_On_Random_Feasible_Target_Point_Selection(param, current_time, AP_vertex, DVQ, FTPQ, ESQ, global_packet);
		break;

	case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_MST_FEASIBLE_TARGET_POINT_SELECTION:
		ST_Construct_Multicast_Tree_Based_On_MST_Feasible_Target_Point_Selection(param, current_time, AP_vertex, DVQ, FTPQ, ESQ, global_packet);
		break;

	case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_AST_FEASIBLE_TARGET_POINT_SELECTION:
		ST_Construct_Multicast_Tree_Based_On_AST_Feasible_Target_Point_Selection(param, current_time, AP_vertex, DVQ, FTPQ, ESQ, global_packet);
		break;

	case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_SIMPLE_AST_FEASIBLE_TARGET_POINT_SELECTION:
		ST_Construct_Multicast_Tree_Based_On_Simple_AST_Feasible_Target_Point_Selection(param, current_time, AP_vertex, DVQ, FTPQ, ESQ, global_packet);
		break;

	case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_MINIMUM_DELAY_FEASIBLE_TARGET_POINT_SELECTION:
		ST_Construct_Multicast_Tree_Based_On_Minimum_Delay_Feasible_Target_Point_Selection(param, current_time, AP_vertex, DVQ, FTPQ, ESQ, global_packet);
		break;

	case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_MINIMUM_COST_FEASIBLE_TARGET_POINT_SELECTION:
		ST_Construct_Multicast_Tree_Based_On_Minimum_Cost_Feasible_Target_Point_Selection(param, current_time, AP_vertex, DVQ, FTPQ, ESQ, global_packet);
		break;

	case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_SIMPLE_RANDOM_TARGET_POINT_SELECTION:
		ST_Construct_Multicast_Tree_Based_On_Simple_Random_Target_Point_Selection(param, current_time, AP_vertex, DVQ, FTPQ, ESQ, global_packet);
		break;

	case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_FLOODING_TARGET_POINT_SELECTION:
		ST_Construct_Multicast_Tree_Based_On_Flooding_Target_Point_Selection(param, current_time, AP_vertex, DVQ, FTPQ, ESQ, global_packet);
		break;

	case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_LAST_TARGET_POINT_SELECTION:
		ST_Construct_Multicast_Tree_Based_On_Last_Target_Point_Selection(param, current_time, AP_vertex, DVQ, FTPQ, ESQ, global_packet);
		break;

	case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_SIMPLE_MINIMUM_DELAY_TARGET_POINT_SELECTION:
		ST_Construct_Multicast_Tree_Based_On_Simple_Minimum_Delay_Target_Point_Selection(param, current_time, AP_vertex, DVQ, FTPQ, ESQ, global_packet);
		break;

	case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_SIMPLE_MINIMUM_COST_TARGET_POINT_SELECTION:
		ST_Construct_Multicast_Tree_Based_On_Simple_Minimum_Cost_Target_Point_Selection(param, current_time, AP_vertex, DVQ, FTPQ, ESQ, global_packet);
		break;

	case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_SIMPLE_SPT_TARGET_POINT_SELECTION:
		ST_Construct_Multicast_Tree_Based_On_Simple_SPT_Target_Point_Selection(param, current_time, AP_vertex, DVQ, FTPQ, ESQ, global_packet);
		break;

	case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_SPT_FEASIBLE_TARGET_POINT_SELECTION:
		ST_Construct_Multicast_Tree_Based_On_SPT_Feasible_Target_Point_Selection(param, current_time, AP_vertex, DVQ, FTPQ, ESQ, global_packet);
		break;

	default:
		printf("ST_Construct_Multicast_Tree(): Error: param->multicast_tree_construction_algorithm(%d) is not supported\n", param->multicast_tree_construction_algorithm);
		exit(1);
	}
}

void ST_Construct_Multicast_Forest(parameter_t *param, double current_time, access_point_queue_t *APQ, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet)
{ /* construct a multicast forest consisting of multiple multicast trees whose root is an AP according to the construction algorithm */
	/** construct a multicast forwarding forest with multiple APs based on the multicast-tree-construction alogirthm */
	switch(param->multicast_tree_construction_algorithm)
	{
	case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_MINIMUM_COST_FEASIBLE_TARGET_POINT_SELECTION:
		ST_Construct_Multicast_Forest_Based_On_Minimum_Cost_Feasible_Target_Point_Selection(param, current_time, APQ, DVQ, FTPQ, ESQ, global_packet);
		break;

#if 0 /* [ */
	case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_SIMPLE_RANDOM_TARGET_POINT_SELECTION:
		ST_Construct_Multicast_Forest_Based_On_Simple_Random_Target_Point_Selection(param, current_time, APQ, DVQ, FTPQ, ESQ, global_packet);
		break;

	case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_FLOODING_TARGET_POINT_SELECTION:
		ST_Construct_Multicast_Forest_Based_On_Flooding_Target_Point_Selection(param, current_time, APQ, DVQ, FTPQ, ESQ, global_packet);
		break;
#endif /* ] */

	default:
		printf("ST_Construct_Multicast_Forest(): Error: param->multicast_tree_construction_algorithm(%d) is not supported\n", param->multicast_tree_construction_algorithm);
		exit(1);
	}
}

int ST_Get_GreedyMulticastTree(parameter_t *param, double current_time, int src, target_point_queue_t *FTPQ, destination_vehicle_queue_t *DVQ)
{ //get a greedy multicast tree (i.e., delay-constrained mininum Steiner tree) where FTPQ is the set of destination intersections (i.e., target points).
	int result = 0; //return-result
	struct_graph_node *G = param->vanet_table.Gr; //road network graph G
	int G_size = param->vanet_table.Gr_size; //size of graph G in terms of the number of vertices in G
	int matrix_size = G_size; //matrix size
	double g_max = 0; //maximum gain
	double g = 0; //gain

	packet_trajectory_queue_t alternative_path; //alternative path to replace the superedge
	packet_trajectory_queue_t p_max; //superedge to replaced with q_max for the maximum gain
	packet_trajectory_queue_t q_max; //alternative path to replace p_max for the maximum gain

	int i = 0; //for-loop index
	double cost_p; //cost of path p
	double cost_q; //cost of path q

	packet_forwarding_tree_path_queue_t PFTPQ; //packet forwarding tree path queue PFTPQ
	packet_forwarding_tree_path_queue_node_t *pSuperedge = NULL; //pointer to a superedge implemented as a packet forwarding tree path queue node

	double **T = param->vanet_table.Tr_mcast; //multicast tree that initially points to the shortest path matrix D
	double **D = param->vanet_table.Dr_mcast; //the shortest path matrix
	int **M = param->vanet_table.Mr_mcast; //the predecessor matrix
	double **S = param->vanet_table.Sr_mcast; //the supplementary metric matrix

#if MINIMUM_STEINER_TREE_OPTIMIZATION_CHECK_FLAG
	double **T_org = NULL; //original multicast tree that initially points to the shortest path matrix D
	double cost_of_T_org = 0; //cost of the shortest path tree T_org
	double cost_of_T = 0; //cost of the minimum steiner tree T
#endif

	int v = 0; //vertex v that is a target point
	int u = 0; //vertex u that is the source node of the superedge containing the target point v
	component_vertex_queue_t CVQ1; //component vertex queue for tree T1 containing source node u
	component_vertex_queue_t CVQ2; //component vertex queue for tree T2 containing target point v
	double **A = NULL; //virtual graph's adjacency matrix containing two virtual nodes v1 and v2
	int A_size = 0; //size of matrix A
	int v1 = G_size + 1; //virtual node v1 for tree T1
	int v2 = G_size + 2; //virtual node v2 for tree T2
	boolean flag = FALSE; //flag to find the boolean function result

	/* check whether FTPQ has at least one target point */
	if(FTPQ->size == 0)
	{
		printf("ST_Get_GreedyMulticastTree(): packet forwarding tree path queue has no target point\n");
		return 1;
	}
	else if(G_size == 0)
	{
		printf("ST_Get_GreedyMulticastTree(): G_size is 0\n");
		return 1;
	}

	/** construct packet forwarding tree path queue for target points in FTPQ */
	/* initialize packet forwarding tree path queue PFTPQ */
	InitQueue((queue_t*)&PFTPQ, QTYPE_PACKET_FORWARDING_TREE_PATH);
	
	/* construct packet forwarding tree path queue PFTPQ with feasible target points in FTPQ */
	Construct_PacketForwardingTreePathQueue_With_FeasibleTargetPoints(param, src, FTPQ, &PFTPQ);

	/** construct a minimum-delay tree T spanning both src and target points */
	/* construct the shortest packet delivery path tree matrices D, S, and M for
	 * the given packet forwarding tree path queue PFTPQ where W is the adjacency 
	 * matrix, D is the shortest path matrix, M is the predecessor matrix, and S 
	 * is the supplementary metric matrix */
	Construct_ShortestPacketDeliveryPathTree_Matrices_With_PacketForwardingTreePathQueue(&PFTPQ, matrix_size, T, D, M, S);

#if MINIMUM_STEINER_TREE_OPTIMIZATION_CHECK_FLAG
	/** copy T into T_org for the verification of optimization */
	/* alloate the adjacency matrix W for the shortest packet delivery path tree */
	T_org = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(matrix_size);

	/* copy the matrix T into adjacency matrix T_org for the link cost matrix T corresponding to the shortest path tree */
	Floyd_Warshall_Copy_Matrix_Of_Type_Double(T_org, T, matrix_size);
#endif

	/** initialize component vertex queues */
	InitQueue((queue_t*)&CVQ1, QTYPE_COMPONENT_VERTEX);
	InitQueue((queue_t*)&CVQ2, QTYPE_COMPONENT_VERTEX);

	/** initialize p_max, q_max, and alternative path of type packet trajectory queue */
	InitQueue((queue_t*)&p_max, QTYPE_PACKET_TRAJECTORY);
	InitQueue((queue_t*)&q_max, QTYPE_PACKET_TRAJECTORY);
	InitQueue((queue_t*)&alternative_path, QTYPE_PACKET_TRAJECTORY);

	/** make virtual graph containing two virtual nodes v1 and v2 */
	/* allocate the memory of virtual graph's adjacency matrix of size matrix_size*/
	A_size = matrix_size + 2;
	A = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(A_size);
	assert_memory(A);

	/* make virtual graph's adjacency matrix A with road network graph G */
	Floyd_Warshall_Make_VirtualGraph_With_Graph_And_ComponentVertexQueues(G, G_size, &CVQ1, &CVQ2, &A, A_size);

	/** find a delay-bounded shortest path p_s */
	/* set g_max to 0 */
	g_max = 0;
	do 
	{
		/** update the delay cost matrix D, the delay cost variance matrix S, and the predecessor matrix M with the adjacency matrix for tree T */
		/* construct delay matrices D and S from the adjacency cost matrix T_prime for the augmented tree */
		Floyd_Warshall_Make_Weight_Matrix_For_EDC_With_AdjacencyMatrix(D, S, matrix_size, T, param);

		/* compute the matrices for all-pairs shortest-paths using the Floyd-Warshall algorithm */
		Floyd_Warshall_Construct_Shortest_PathInfo_For_EDC(D, M, S, matrix_size);

		/* construct packet forwarding tree path queue PFTPQ containing superedges with feasible target points in FTPQ and the packet forwarding tree T */
		Construct_PacketForwardingTreePathQueue_With_FeasibleTargetPoints_And_PacketForwardingTree(param, matrix_size, T, D, M, src, FTPQ, &PFTPQ);

		/* filter out non-superedges from PFTPQ such that one relay node in the path is one of target points */
		FilterOut_NonSuperedges_From_PacketForwardingTreePathQueue(param, FTPQ, &PFTPQ);

		/* find the best alternative shortest path to reduce the cost of the minimum Steiner tree */
		pSuperedge = &(PFTPQ.head);
		for(i = 0; i < PFTPQ.size; i++) /* [ for-1 */
		{
			pSuperedge = pSuperedge->next;
			/* check whether this superedge is valid or not */
			if(pSuperedge->non_superedge_flag)
			{
				continue;
			}

			/** remove a superedge p from tree T to get T1 and T2 */
			/* check the validity of superedge's packet trajectory queue */
			if(pSuperedge->PTQ.size <= 1)
			{
				printf("ST_Get_GreedyMulticastTree(): Error: pSuperedge->PTQ.size(%d) is not greater than 1\n", pSuperedge->PTQ.size);
				return 1;
			}

			/* locate the target point v at the superedge */
			v = pSuperedge->PTQ.head.prev->intersection_id;

			/* locate the source node u of the deleted path including the target point v at the tree */
			u = pSuperedge->PTQ.head.next->intersection_id;

			/* delete the edges from source node u to target point v */
			GO_Delete_Edges_From_Graph_With_PacketTrajectoryQueue(T, matrix_size, &(pSuperedge->PTQ));

			/* find the vertices of tree T1 containing source node u by Breadth First Search */
			GO_Find_ComponentVertices(T, matrix_size, u, &CVQ1);

			/* find the vertices of tree T2 containing target point v by Breadth First Search */
			GO_Find_ComponentVertices(T, matrix_size, v, &CVQ2);

			/* make a new adjacency matrix A for the tree T_new having two virtual vertices v1 and v2 such that v1 is a virtual node for T1 and v2 is a virtual node for T2 */
			Floyd_Warshall_Make_VirtualGraph_With_Graph_And_ComponentVertexQueues(G, G_size, &CVQ1, &CVQ2, &A, A_size);

			/** make alternative path q (denoted by alternative_path) that is a delay bounded shortest path between T1 and T2 by computing k-shortest paths from v1 to v2 in tree T_new */
			switch(param->multicast_tree_construction_algorithm)
			{
				case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_SIMPLE_RANDOM_TARGET_POINT_SELECTION:
				case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_SIMPLE_MINIMUM_DELAY_TARGET_POINT_SELECTION:
				case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_SIMPLE_MINIMUM_COST_TARGET_POINT_SELECTION:
					flag = GO_Find_Best_Alternative_Path_For_Tree_Connectivity_Without_Constraint(param, current_time, src, FTPQ, DVQ, A, A_size, (const double**)T, matrix_size, &alternative_path);
					break;

				case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_TMA_FEASIBLE_TARGET_POINT_SELECTION:
				case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_RANDOM_FEASIBLE_TARGET_POINT_SELECTION:
				case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_MST_FEASIBLE_TARGET_POINT_SELECTION:
				case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_MINIMUM_DELAY_FEASIBLE_TARGET_POINT_SELECTION:
				case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_MINIMUM_COST_FEASIBLE_TARGET_POINT_SELECTION:
					flag = GO_Find_Best_Alternative_Path_For_Tree_Connectivity_With_DeliveryProbabilityConstraint(param, current_time, src, FTPQ, DVQ, A, A_size, (const double**)T, matrix_size, &alternative_path);
					break;

				default:
				  printf("ST_Get_GreedyMulticastTree(): param->multicast_tree_construction_algorithm (%d) is not supported in this function\n", param->multicast_tree_construction_algorithm);
					exit(1);
			}

			if(flag)
			{
				/** set cost_p to the cost of path p */
				cost_p = pSuperedge->PTQ.path_cost;

				/** set cost_q to the cost of path q */
				cost_q = alternative_path.path_cost;

				/** compute gain g from cost_p and cost_q */
				g = cost_p - cost_q;

				/** update the maximum gain along with p_max and q_max */
				if(g_max < g)
				{
					g_max = g;
				
					/* copy path p (i.e., pSuperedge->PTQ) into p_max */
					CopyQueue((queue_t*)&p_max, (queue_t*)&(pSuperedge->PTQ));

					/* copy path q (i.e., alternative_path) into q_max */
					CopyQueue((queue_t*)&q_max, (queue_t*)&alternative_path);
				}
			}

			/** connect trees T1 and T2 with path p, going to the original tree T */
			/* add the edges from source node u to target point v to T */
			GO_Add_Edges_To_Graph_With_PacketTrajectoryQueue(T, matrix_size, &(pSuperedge->PTQ), 0, 0);
		} /* ] for-1 */

		/** update the multicast tree T if we have additional gain, that is, g_max is greater than 0 */
		if(g_max > 0)
		{
			/* remove a superedge p_max from tree T to get T1 and T2 */
			GO_Delete_Edges_From_Graph_With_PacketTrajectoryQueue(T, matrix_size, &p_max);

			/* connect trees T1 and T2 with path q_max, updating the original tree T for a better multicast tree */
			GO_Add_Edges_To_Graph_With_PacketTrajectoryQueue(T, matrix_size, &q_max, 0, 0);
		}
	} while(g_max);

#if MINIMUM_STEINER_TREE_OPTIMIZATION_CHECK_FLAG
	/* check whether the optimized minimum steiner tree T has less cost than the original shortest path tree T_org or not */
	cost_of_T_org = Floyd_Warshall_Sum_Matrix_Weights(T_org, matrix_size);
	cost_of_T = Floyd_Warshall_Sum_Matrix_Weights(T, matrix_size);

	if(cost_of_T < cost_of_T_org)
	{
		printf("ST_Get_GreedyMulticastTree(): The Mimimum Steiner Tree T's cost(%.2f) is less than the Shortest Path Tree T_org's cost(%.2f)\n", (float)cost_of_T, (float)cost_of_T_org);
	}
	else
	{
		printf("ST_Get_GreedyMulticastTree(): The Mimimum Steiner Tree T's cost(%.2f) is NOT less than the Shortest Path Tree T_org's cost(%.2f)\n", (float)cost_of_T, (float)cost_of_T_org);
	}
#endif

	/** destroy packet forwarding tree path queue PFTPQ for target points in FTPQ */
	DestroyQueue((queue_t*)&PFTPQ);

	/** destroy component vertex queues */
	DestroyQueue((queue_t*)&CVQ1);
	DestroyQueue((queue_t*)&CVQ2);

	/** destroy p_max, q_max, and alternative path of type packet trajectory queue */
	DestroyQueue((queue_t*)&p_max);
	DestroyQueue((queue_t*)&q_max);
	DestroyQueue((queue_t*)&alternative_path);

#if MINIMUM_STEINER_TREE_OPTIMIZATION_CHECK_FLAG
	/** free the memory of matrix T_org */
	Floyd_Warshall_Free_Matrix_Of_Type_Double(T_org, matrix_size);
#endif

	/** free the memory of virtual graph's adjacency matrix A */
	Floyd_Warshall_Free_Matrix_Of_Type_Double(A, matrix_size+2);

	return result;	
}

int ST_Get_GreedyMulticastTree_With_DelayConstraint(parameter_t *param, double current_time, int src, target_point_queue_t *FTPQ, destination_vehicle_queue_t *DVQ, double beta)
{ //get a greedy multicast tree (i.e., delay-constrained mininum Steiner tree) with delay constraint beta where FTPQ is the set of destination intersections (i.e., target points); note that any forwarding path from AP to a target point must have a delivery delay shorter than or equal to beta.
	int result = 0; //return-result
	struct_graph_node *G = param->vanet_table.Gr; //road network graph G
	int G_size = param->vanet_table.Gr_size; //size of graph G in terms of the number of vertices in G
	int matrix_size = G_size; //matrix size
	double g_max = 0; //maximum gain
	double g = 0; //gain

	packet_trajectory_queue_t alternative_path; //alternative path to replace the superedge
	packet_trajectory_queue_t p_max; //superedge to replaced with q_max for the maximum gain
	packet_trajectory_queue_t q_max; //alternative path to replace p_max for the maximum gain

	int i = 0; //for-loop index
	double cost_p; //cost of path p
	double cost_q; //cost of path q

	packet_forwarding_tree_path_queue_t PFTPQ; //packet forwarding tree path queue PFTPQ
	packet_forwarding_tree_path_queue_node_t *pSuperedge = NULL; //pointer to a superedge implemented as a packet forwarding tree path queue node

	double **T = param->vanet_table.Tr_mcast; //multicast tree that initially points to the shortest path matrix D
	double **D = param->vanet_table.Dr_mcast; //the shortest path matrix
	int **M = param->vanet_table.Mr_mcast; //the predecessor matrix
	double **S = param->vanet_table.Sr_mcast; //the supplementary metric matrix

#if MINIMUM_STEINER_TREE_OPTIMIZATION_CHECK_FLAG
	double **T_org = NULL; //original multicast tree that initially points to the shortest path matrix D
	double cost_of_T_org = 0; //cost of the shortest path tree T_org
	double cost_of_T = 0; //cost of the minimum steiner tree T
#endif

	int v = 0; //vertex v that is a target point
	int u = 0; //vertex u that is the source node of the superedge containing the target point v
	component_vertex_queue_t CVQ1; //component vertex queue for tree T1 containing source node u
	component_vertex_queue_t CVQ2; //component vertex queue for tree T2 containing target point v
	double **A = NULL; //virtual graph's adjacency matrix containing two virtual nodes v1 and v2
	int A_size = 0; //size of matrix A
	int v1 = G_size + 1; //virtual node v1 for tree T1
	int v2 = G_size + 2; //virtual node v2 for tree T2
	boolean flag = FALSE; //flag to find the boolean function result

	/* check whether FTPQ has at least one target point */
	if(FTPQ->size == 0)
	{
		printf("ST_Get_GreedyMulticastTree(): packet forwarding tree path queue has no target point\n");
		return 1;
	}
	else if(G_size == 0)
	{
		printf("ST_Get_GreedyMulticastTree(): G_size is 0\n");
		return 1;
	}

	/** construct packet forwarding tree path queue for target points in FTPQ */
	/* initialize packet forwarding tree path queue PFTPQ */
	InitQueue((queue_t*)&PFTPQ, QTYPE_PACKET_FORWARDING_TREE_PATH);
	
	/* construct packet forwarding tree path queue PFTPQ with feasible target points in FTPQ */
	Construct_PacketForwardingTreePathQueue_With_FeasibleTargetPoints(param, src, FTPQ, &PFTPQ);

	/** construct a minimum-delay tree T spanning both src and target points */
	/* construct the shortest packet delivery path tree matrices D, S, and M for
	 * the given packet forwarding tree path queue PFTPQ where W is the adjacency 
	 * matrix, D is the shortest path matrix, M is the predecessor matrix, and S 
	 * is the supplementary metric matrix */
	Construct_ShortestPacketDeliveryPathTree_Matrices_With_PacketForwardingTreePathQueue(&PFTPQ, matrix_size, T, D, M, S);

#if MINIMUM_STEINER_TREE_OPTIMIZATION_CHECK_FLAG
	/** copy T into T_org for the verification of optimization */
	/* alloate the adjacency matrix W for the shortest packet delivery path tree */
	T_org = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(matrix_size);

	/* copy the matrix T into adjacency matrix T_org for the link cost matrix T corresponding to the shortest path tree */
	Floyd_Warshall_Copy_Matrix_Of_Type_Double(T_org, T, matrix_size);
#endif

	/** initialize component vertex queues */
	InitQueue((queue_t*)&CVQ1, QTYPE_COMPONENT_VERTEX);
	InitQueue((queue_t*)&CVQ2, QTYPE_COMPONENT_VERTEX);

	/** initialize p_max, q_max, and alternative path of type packet trajectory queue */
	InitQueue((queue_t*)&p_max, QTYPE_PACKET_TRAJECTORY);
	InitQueue((queue_t*)&q_max, QTYPE_PACKET_TRAJECTORY);
	InitQueue((queue_t*)&alternative_path, QTYPE_PACKET_TRAJECTORY);

	/** make virtual graph containing two virtual nodes v1 and v2 */
	/* allocate the memory of virtual graph's adjacency matrix of size matrix_size*/
	A_size = matrix_size + 2;
	A = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(A_size);
	assert_memory(A);

	/* make virtual graph's adjacency matrix A with road network graph G */
	Floyd_Warshall_Make_VirtualGraph_With_Graph_And_ComponentVertexQueues(G, G_size, &CVQ1, &CVQ2, &A, A_size);

	/** find a delay-bounded shortest path p_s */
	/* set g_max to 0 */
	g_max = 0;
	do 
	{
		/** update the delay cost matrix D, the delay cost variance matrix S, and the predecessor matrix M with the adjacency matrix for tree T */
		/* construct delay matrices D and S from the adjacency cost matrix T_prime for the augmented tree */
		Floyd_Warshall_Make_Weight_Matrix_For_EDC_With_AdjacencyMatrix(D, S, matrix_size, T, param);

		/* compute the matrices for all-pairs shortest-paths using the Floyd-Warshall algorithm */
		Floyd_Warshall_Construct_Shortest_PathInfo_For_EDC(D, M, S, matrix_size);

		/* construct packet forwarding tree path queue PFTPQ containing superedges with feasible target points in FTPQ and the packet forwarding tree T */
		Construct_PacketForwardingTreePathQueue_With_FeasibleTargetPoints_And_PacketForwardingTree(param, matrix_size, T, D, M, src, FTPQ, &PFTPQ);

		/* filter out non-superedges from PFTPQ such that one relay node in the path is one of target points */
		FilterOut_NonSuperedges_From_PacketForwardingTreePathQueue(param, FTPQ, &PFTPQ);

		/* find the best alternative shortest path to reduce the cost of the minimum Steiner tree */
		pSuperedge = &(PFTPQ.head);
		for(i = 0; i < PFTPQ.size; i++) /* [ for-1 */
		{
			pSuperedge = pSuperedge->next;
			/* check whether this superedge is valid or not */
			if(pSuperedge->non_superedge_flag)
			{
				continue;
			}

			/** remove a superedge p from tree T to get T1 and T2 */
			/* check the validity of superedge's packet trajectory queue */
			if(pSuperedge->PTQ.size <= 1)
			{
				printf("ST_Get_GreedyMulticastTree(): Error: pSuperedge->PTQ.size(%d) is not greater than 1\n", pSuperedge->PTQ.size);
				return 1;
			}

			/* locate the target point v at the superedge */
			v = pSuperedge->PTQ.head.prev->intersection_id;

			/* locate the source node u of the deleted path including the target point v at the tree */
			u = pSuperedge->PTQ.head.next->intersection_id;

			/* delete the edges from source node u to target point v */
			GO_Delete_Edges_From_Graph_With_PacketTrajectoryQueue(T, matrix_size, &(pSuperedge->PTQ));

			/* find the vertices of tree T1 containing source node u by Breadth First Search */
			GO_Find_ComponentVertices(T, matrix_size, u, &CVQ1);

			/* find the vertices of tree T2 containing target point v by Breadth First Search */
			GO_Find_ComponentVertices(T, matrix_size, v, &CVQ2);

			/* make a new adjacency matrix A for the tree T_new having two virtual vertices v1 and v2 such that v1 is a virtual node for T1 and v2 is a virtual node for T2 */
			Floyd_Warshall_Make_VirtualGraph_With_Graph_And_ComponentVertexQueues(G, G_size, &CVQ1, &CVQ2, &A, A_size);

			/** make alternative path q (denoted by alternative_path) that is a delay bounded shortest path between T1 and T2 by computing k-shortest paths from v1 to v2 in tree T_new */
			switch(param->multicast_tree_construction_algorithm)
			{
				case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_SIMPLE_AST_FEASIBLE_TARGET_POINT_SELECTION:
				case MULTICAST_TREE_CONSTRUCTION_ALGORITHM_AST_FEASIBLE_TARGET_POINT_SELECTION:
					flag = GO_Find_Best_Alternative_Path_For_Tree_Connectivity_With_DeliveryProbabilityConstraint_And_DeliveryDelayConstraint(param, current_time, src, FTPQ, DVQ, A, A_size, (const double**)T, matrix_size, beta, &alternative_path);
					break;

				default:
					printf("ST_Get_GreedyMulticastTree_With_DelayConstraint(): param->multicast_tree_construction_algorithm (%d) is not supported in this function\n", param->multicast_tree_construction_algorithm);
					exit(1);
			}
			if(flag)
			{
				/** set cost_p to the cost of path p */
				cost_p = pSuperedge->PTQ.path_cost;

				/** set cost_q to the cost of path q */
				cost_q = alternative_path.path_cost;

				/** compute gain g from cost_p and cost_q */
				g = cost_p - cost_q;

				/** update the maximum gain along with p_max and q_max */
				if(g_max < g)
				{
					g_max = g;
				
					/* copy path p (i.e., pSuperedge->PTQ) into p_max */
					CopyQueue((queue_t*)&p_max, (queue_t*)&(pSuperedge->PTQ));

					/* copy path q (i.e., alternative_path) into q_max */
					CopyQueue((queue_t*)&q_max, (queue_t*)&alternative_path);
				}
			}

			/** connect trees T1 and T2 with path p, going to the original tree T */
			/* add the edges from source node u to target point v to T */
			GO_Add_Edges_To_Graph_With_PacketTrajectoryQueue(T, matrix_size, &(pSuperedge->PTQ), 0, 0);
		} /* ] for-1 */

		/** update the multicast tree T if we have additional gain, that is, g_max is greater than 0 */
		if(g_max > 0)
		{
			/* remove a superedge p_max from tree T to get T1 and T2 */
			GO_Delete_Edges_From_Graph_With_PacketTrajectoryQueue(T, matrix_size, &p_max);

			/* connect trees T1 and T2 with path q_max, updating the original tree T for a better multicast tree */
			GO_Add_Edges_To_Graph_With_PacketTrajectoryQueue(T, matrix_size, &q_max, 0, 0);
		}
	} while(g_max);

#if MINIMUM_STEINER_TREE_OPTIMIZATION_CHECK_FLAG
	/* check whether the optimized minimum steiner tree T has less cost than the original shortest path tree T_org or not */
	cost_of_T_org = Floyd_Warshall_Sum_Matrix_Weights(T_org, matrix_size);
	cost_of_T = Floyd_Warshall_Sum_Matrix_Weights(T, matrix_size);

	if(cost_of_T < cost_of_T_org)
	{
		printf("ST_Get_GreedyMulticastTree(): The Mimimum Steiner Tree T's cost(%.2f) is less than the Shortest Path Tree T_org's cost(%.2f)\n", (float)cost_of_T, (float)cost_of_T_org);
	}
	else
	{
		printf("ST_Get_GreedyMulticastTree(): The Mimimum Steiner Tree T's cost(%.2f) is NOT less than the Shortest Path Tree T_org's cost(%.2f)\n", (float)cost_of_T, (float)cost_of_T_org);
	}
#endif

	/** destroy packet forwarding tree path queue PFTPQ for target points in FTPQ */
	DestroyQueue((queue_t*)&PFTPQ);

	/** destroy component vertex queues */
	DestroyQueue((queue_t*)&CVQ1);
	DestroyQueue((queue_t*)&CVQ2);

	/** destroy p_max, q_max, and alternative path of type packet trajectory queue */
	DestroyQueue((queue_t*)&p_max);
	DestroyQueue((queue_t*)&q_max);
	DestroyQueue((queue_t*)&alternative_path);

#if MINIMUM_STEINER_TREE_OPTIMIZATION_CHECK_FLAG
	/** free the memory of matrix T_org */
	Floyd_Warshall_Free_Matrix_Of_Type_Double(T_org, matrix_size);
#endif

	/** free the memory of virtual graph's adjacency matrix A */
	Floyd_Warshall_Free_Matrix_Of_Type_Double(A, matrix_size+2);

	return result;	
}

int ST_Get_ShortestPathTree(parameter_t *param, double current_time, int src, target_point_queue_t *FTPQ, destination_vehicle_queue_t *DVQ)
{ //get the shortest path tree (i.e., the shortest delay path tree) where FTPQ is the set of destination intersections (i.e., target points).
	int result = 0; //return-result
	struct_graph_node *G = param->vanet_table.Gr; //road network graph G
	int G_size = param->vanet_table.Gr_size; //size of graph G in terms of the number of vertices in G
	int matrix_size = G_size; //matrix size
	packet_forwarding_tree_path_queue_t PFTPQ; //packet forwarding tree path queue PFTPQ

	double **T = param->vanet_table.Tr_mcast; //multicast tree that initially points to the shortest path matrix D
	double **D = param->vanet_table.Dr_mcast; //the shortest path matrix
	int **M = param->vanet_table.Mr_mcast; //the predecessor matrix
	double **S = param->vanet_table.Sr_mcast; //the supplementary metric matrix

	/* check whether FTPQ has at least one target point */
	if(FTPQ->size == 0)
	{
		printf("ST_Get_GreedyMulticastTree(): packet forwarding tree path queue has no target point\n");
		return 1;
	}
	else if(G_size == 0)
	{
		printf("ST_Get_GreedyMulticastTree(): G_size is 0\n");
		return 1;
	}

	/** construct packet forwarding tree path queue for target points in FTPQ */
	/* initialize packet forwarding tree path queue PFTPQ */
	InitQueue((queue_t*)&PFTPQ, QTYPE_PACKET_FORWARDING_TREE_PATH);
	
	/* construct packet forwarding tree path queue PFTPQ with feasible target points in FTPQ */
	Construct_PacketForwardingTreePathQueue_With_FeasibleTargetPoints(param, src, FTPQ, &PFTPQ);

	/** construct a minimum-delay tree T spanning both src and target points */
	/* construct the shortest packet delivery path tree matrices D, S, and M for
	 * the given packet forwarding tree path queue PFTPQ where W is the adjacency 
	 * matrix, D is the shortest path matrix, M is the predecessor matrix, and S 
	 * is the supplementary metric matrix */
	Construct_ShortestPacketDeliveryPathTree_Matrices_With_PacketForwardingTreePathQueue(&PFTPQ, matrix_size, T, D, M, S);

	/** destroy packet forwarding tree path queue PFTPQ for target points in FTPQ */
	DestroyQueue((queue_t*)&PFTPQ);

	return result;
}

int ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree(parameter_t *param, double current_time, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ)
{ /* filter out redundant target points from Minimum Steiner Tree T consisting of anycast sets of destination vehicles and make a pruned multicast tree represented as edge set queue ESQ */
	int result = 0;
	double **T = param->vanet_table.Tr_mcast;
	int T_size = param->vanet_table.matrix_size_for_mcast_in_Gr;
	int redundant_target_point = 0; //redundant target point id

	/*@ Note that each destination vehicle has feasible target point queue FTPQ */

	/** make leaf node queue containing leaf nodes whose indegree is one and whose outdegree is zero */

	/* Destroy edge set queue ESQ */
	if(ESQ->size > 0)
	{
		DestroyQueue((queue_t*)ESQ);
	}

	/* construct tree edge queue TEQ with multicast tree T and feasible target point queue FTPQ */
	GO_Construct_EdgeQueue_With_AdjacencyMatrix_And_FeasibleTargetPointQueue(T, T_size, FTPQ, ESQ);

	/* sort edge set queue in the descending order of edge weight */
	SortEdgeSetQueue_For_MultiAnycast(ESQ);

	/** Phase 1: loop until each anycast has one target point */
	/* Step 1: search a redundant target point to be deleted from the multicast tree, and mark the corresponding target point in the corresponding destination vehicles' feasible target point queue FTPQ */

	/* Step 2: check whether this leaf node can be deleted from the tree or not:
	 * 1. This deletion does not make any ancast set empty */
		
	/* Step 3: if this leaf node can be deleted, 
	 * 1. delete it from the corresponding anycast set and the leaf queue
	 * 2. if the deleted node's parent node becomes a leaf node, 
	 *		if it is a target point, add it to the leaf queue;
	 *		otherwise, go to Step 3.2. */

	/* Step 4: if this leaf node cannot be deleted,
	 * 1. mark this node as a checked target point that will not be considered as deleted node */

	do
  	{
		/* find a redundant target point to be deleted from the multicast tree consisting of FTPQ */
		redundant_target_point = GO_Find_RedundantTargetPoint_From_EdgeSetQueue_AlongWith_Queue_Adjustment(param->vanet_table.Gr_size, DVQ, ESQ);
		if(redundant_target_point == -1)
		{ /* there is no more redundant target point to be deleted from the multicast tree */
			/* check the number of target points in the final feasible target point set to see
			 * the number of target points is not more than the number of destination vehicles */
#if TARGET_POINT_FILTERING_TRACE_FLAG
			if(FTPQ->size > DVQ->size)
			{
				printf("[%.2f] ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree(): FTPQ->size(%d) must be less than or equal to DVQ->size(%d)\n", (float)current_time, FTPQ->size, DVQ->size);
			}
#endif

			break;
		}

#if REDUNDANT_TARGET_POINT_PRINTOUT_FLAG
		printf("ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree(): redundant target point is %d\n", redundant_target_point);
#endif
		/* delete redundant_target_point from feasible target point queue FTPQ */
		Delete_TargetPoint_With_TargetPointID(FTPQ, redundant_target_point);
	} while(1);

	/** Phase 2: designate each one target point per anycast set when 
	 * the anycast set has more than one target point; this designation is 
	 * performed when a target point plays a role of relay node and is 
	 * covered by another anycast set of another veicle */

	/* filter out target points per destination vehicle's anycast set with FTPQ */
	//GO_FilterOut_RedundantTargetPoints_From_Anycast_Sets(FTPQ, DVQ);	

	/* mark target points in FTPQ that are covered by more than one anycast set
	 * by setting cover_flag to TRUE for those target points that are redundant 
	 * target points as relay nodes in the feasible target point queue FTPQ */
	GO_Mark_Duplicate_Covered_TargetPoint_In_TargetPointSet(current_time, DVQ, FTPQ);

	/* Note: target points with filter_flag set to FALSE will have a copy of the 
	 * multicasted packet */

	return result;
}

int ST_FilterOut_RedundantTargetPoints_From_ShortestPathTree(parameter_t *param, double current_time, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ)
{ /* filter out redundant target points from the Shortest Path Tree T consisting of anycast sets of destination vehicles and make a pruned multicast tree represented as edge set queue ESQ */
	int result = 0;
	
	result = ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree(param, current_time, DVQ, FTPQ, ESQ);

	return result;
}

int ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree_With_MinimumSpanningTreeAlgorithm(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ)
{ /* filter out redundant target points from Minimum Steiner Tree T consisting of anycast sets of destination vehicles with Minimum Spanning Tree Algorithm and make a pruned multicast tree represented as edge set queue ESQ */
	int result = 0;
	double **T = param->vanet_table.Tr_mcast;
	double **D = param->vanet_table.Dr_mcast;
	int **M = param->vanet_table.Mr_mcast;
	double **S = param->vanet_table.Sr_mcast;
	int T_size = param->vanet_table.matrix_size_for_mcast_in_Gr;
	int redundant_target_point = 0; //redundant target point id
	int AP_vertex_id = atoi(AP_vertex); //vertex id of AP

	double min_distance = INF; //minimum interdistance between the spanning tree and an uncovered anycast set
	int min_target_point_id = 0; //vertex id of a target point with a minimum interdistance
	int min_anycast_set_id = 0; //anycast set id with a minimum interdistance
	struct_vehicle_t *min_anycast_set_vnode = NULL; //pointer to the vehicle with the minimum distance
	destination_vehicle_queue_node_t *min_anycast_set_DV_qnode = NULL; //pointer to the destination vehicle queue node with a minimum interdistance

	double distance = 0; //shortest path distance from src to dst
	int i = 0, j = 0, k = 0; //loop-indices
	destination_vehicle_queue_node_t *pDV_QueueNode = NULL; //pointer to a destination vehicle queue node
	target_point_queue_node_t *pTP_QueueNode = NULL; //pointer to a target point queue node
	target_point_queue_node_t tp_qnode; //target point queue node

	boolean *target_point_flag_vector = NULL; //flag vector to indicate whether the target point whose id is the same as the index in the vector is included in the spanning tree or not
	int target_point_flag_vector_size = 0; //size of target point flag vector
	int target_point_number = 0; //number of target points 

	multicast_group_vehicle_queue_t mcast_group; //multicast group vehicle queue to represent which destination vehicles can be covered by the spanning tree
	multicast_group_vehicle_queue_node_t mcast_qnode; //multicast group vehicle queue node

	boolean *spanning_tree_node_flag_vector = NULL; //flag vector to indicate whether the node corresponding to the index belongs to the spanning tree node set or not
	int spanning_tree_node_flag_vector_size = 0; //size of spanning_tree_node_flag_vector_size

	packet_trajectory_queue_t path; //path from src to dst
	packet_trajectory_queue_node_t *pTR_QueueNode = NULL; //pointer to a trajectory queue node
	
	vertex_queue_t vertex_set; //vertex set for the spanning tree
	vertex_queue_node_t *pVertex_QueueNode = NULL; //pointer to a vertex queue node
	vertex_queue_node_t vertex_qnode; //vertex queue node

	int id = 0; //vertex id
	boolean flag = FALSE;
	boolean flag_for_mcast_group = FALSE; //flag used in the multicast group member addition

	int covering_target_point = 0; //covering target point for an anycast set

#if DEBUG_12_19_2010
	if(current_time > 7723)
		printf("ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree_With_MinimumSpanningTreeAlgorithm(): [%.2f] debugging\n", (float)current_time);	
#endif

	/* check the validity of AP_vertex_id */
	if((AP_vertex_id < 1) || (AP_vertex_id > T_size))
	{
		printf("ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree_With_MinimumSpanningTreeAlgorithm(): AP_vertex_id(%d) must be bounded between 1 and T_size(%d)\n", AP_vertex_id, T_size);
		exit(1);
	}

	/* check the validity of DVQ */
	if(DVQ->size <= 0)
	{
		printf("ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree_With_MinimumSpanningTreeAlgorithm(): DVQ->size(%d) is invalid\n", DVQ->size);
		exit(1);
	}

	/* initialize packet trajectory queue path */
	InitQueue((queue_t*)&path, QTYPE_PACKET_TRAJECTORY);

	/* initialize vertex queue vertex_set */
	InitQueue((queue_t*)&vertex_set, QTYPE_VERTEX);

	/* initialize multicast group vehicle queue mcast_group */
	InitQueue((queue_t*)&mcast_group, QTYPE_MULTICAST_GROUP_VEHICLE);

	/* allocate the memory for target_point_flag_vector */
	target_point_flag_vector_size = T_size + 1;
	target_point_flag_vector = (boolean*)calloc(target_point_flag_vector_size, sizeof(boolean));
	assert_memory(target_point_flag_vector);

	/* allocate the memory for spanning_tree_node_flag_vector */
	spanning_tree_node_flag_vector_size = T_size + 1;
	spanning_tree_node_flag_vector = (boolean*)calloc(spanning_tree_node_flag_vector_size, sizeof(boolean));
	assert_memory(spanning_tree_node_flag_vector);

	/* find a target point in an anycast set that is closest to the packet source, that is, AP */
	pDV_QueueNode = &(DVQ->head);
	for(i = 0; i < DVQ->size; i++)
	{
		pDV_QueueNode = pDV_QueueNode->next;
		pTP_QueueNode = &(pDV_QueueNode->FTPQ.head);
		for(j = 0; j < pDV_QueueNode->FTPQ.size; j++)
		{
			pTP_QueueNode = pTP_QueueNode->next;

			/* get the shortest path distance from AP to this target point */
			distance = D[AP_vertex_id-1][pTP_QueueNode->target_point_id-1];

			/* update the min_distance */
			if(distance < min_distance)
			{
				min_distance = distance;
				min_target_point_id = pTP_QueueNode->target_point_id;
				min_anycast_set_id = pDV_QueueNode->vid;
				min_anycast_set_vnode = pDV_QueueNode->vnode;
				min_anycast_set_DV_qnode = pDV_QueueNode;
			}
		}
	}

	/** include min_anycast_set_id to anycast_set_id_vector and anycast_set_flag_vector */
	/* check the validity of min_anycast_set_id */
	if(min_distance == INF && min_anycast_set_id == 0 && min_target_point_id == 0)
	{
		printf("ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree_With_MinimumSpanningTreeAlgorithm(): there is no target point to be added into the spanning tree\n");
		exit(1);
	}
	else
	{
		/* update target_point_flag_vector */
		target_point_flag_vector[min_target_point_id] = TRUE;
		target_point_number++;		

		/* add min_anycast_set_id to mcast_group */
		memset(&mcast_qnode, 0, sizeof(mcast_qnode));
		mcast_qnode.vid = min_anycast_set_id;
		mcast_qnode.vnode = min_anycast_set_vnode;
		Enqueue((queue_t*)&mcast_group, (queue_node_t*)&mcast_qnode);

		/* turn on filter_flags of target points in the destination vehicle's FTPQ except min_target_point_id */
		Set_FilterFlags_In_DestinationVehicle_TargetPointSet_For_TargetPointFiltering(&(min_anycast_set_DV_qnode->FTPQ), min_target_point_id);
	}

	/** make a spanning tree T' with the path from AP to min_target_point_id */
	/* make the shortest path from AP to min_target_point_id */
	Make_PacketTrajectory_For_DeliveryCost_In_PacketForwardingTree(param, M, AP_vertex_id, min_target_point_id, &path);

	/* set spanning_tree_node_flag_vector with the vertices in the path */
	/* Also,make a vertex queue for the spanning tree with spanning_tree_node_flag_vector */
	pTR_QueueNode = &(path.head);
	for(i = 0; i < path.size; i++)
	{	
		pTR_QueueNode = pTR_QueueNode->next;
		id = pTR_QueueNode->intersection_id;

		/* enable the flag corresponding to id */
		spanning_tree_node_flag_vector[id] = TRUE;

		/* add vertex queue node for the id to vertex_set */
		memset(&vertex_qnode, 0, sizeof(vertex_qnode));
		vertex_qnode.vertex = id;
		Enqueue((queue_t*)&vertex_set, (queue_node_t*)&vertex_qnode);
	}

	/* check whether the current spanning tree can cover each destination vehicle's anycast set or not */
	pDV_QueueNode = &(DVQ->head);
	for(i = 0; i < DVQ->size; i++)
	{
		pDV_QueueNode = pDV_QueueNode->next;
		
		/* check whether this vehicle has already been in mcast_group */
		flag = Is_Vehicle_In_MulticastGroupVehicleQueue(&mcast_group, pDV_QueueNode->vid);
		if(flag)
		  continue;

		/* check whether this vehicle's anycast set can be covered by the spanning tree */
		flag = Is_AnycastSet_Covered_By_SpanningTree_TargetPointSet(&(pDV_QueueNode->FTPQ), target_point_flag_vector, target_point_flag_vector_size, &covering_target_point);
		if(flag)
		{ /* this vehicle's anycast set is covered, so add this vehicle to mcast_group */
			memset(&mcast_qnode, 0, sizeof(mcast_qnode));
			mcast_qnode.vid = pDV_QueueNode->vid;
			mcast_qnode.vnode = pDV_QueueNode->vnode;
			Enqueue((queue_t*)&mcast_group, (queue_node_t*)&mcast_qnode);

			/* set cover_flag for covering_target_point and turn on filter_flags of all target points in the destination vehicle's FTPQ */
			Set_CoverFlags_And_FilterFlags_In_DestinationVehicle_TargetPointSet_For_TargetPointCovering(&(pDV_QueueNode->FTPQ), covering_target_point);
		}
	}
				
	/** find an anycast set closest to the spanning tree and add it to the spanning tree set
	 * untill all of anycast sets are added to the spanning tree set */
	while(mcast_group.size < DVQ->size)
  	{
		/** for each node in T', find the target point u closest to T' where u's anycast set A(u) does not belong to S(T') where S(T') is the set of anycast sets covered by T' */
		/* reset min_distance variables */
		min_distance = INF;
		min_target_point_id = 0;
		min_anycast_set_id = 0;
		flag_for_mcast_group = FALSE;

		pDV_QueueNode = &(DVQ->head);
		for(i = 0; i < DVQ->size; i++) //@for-1:start
		{
			pDV_QueueNode = pDV_QueueNode->next;

			/* check whether this vehicle has already been in mcast_group */
			flag = Is_Vehicle_In_MulticastGroupVehicleQueue(&mcast_group, pDV_QueueNode->vid);
			if(flag)
		  		continue;

			pTP_QueueNode = &(pDV_QueueNode->FTPQ.head);
			for(j = 0; j < pDV_QueueNode->FTPQ.size; j++) //@for-1.1:start
			{
				pTP_QueueNode = pTP_QueueNode->next;

				/* check whether this vertex has already been in vertex_set or not */
				//flag = Is_Vertex_In_VertexQueue(&vertex_set, pTP_QueueNode->target_point_id);
				flag = spanning_tree_node_flag_vector[pTP_QueueNode->target_point_id];
#if 0 /* [ */
				if(flag)
					continue;
#endif /* ] */

#if 1 /* [[ */
				if(flag)
				{
					min_distance = 0;
					min_target_point_id = pTP_QueueNode->target_point_id;
					min_anycast_set_id = pDV_QueueNode->vid;
					min_anycast_set_DV_qnode = pDV_QueueNode;
					flag_for_mcast_group = TRUE;
					break;
				}
#endif /* ]] */

				pVertex_QueueNode = &(vertex_set.head);
				for(k = 0; k < vertex_set.size; k++) //@for-1.1.1:start
				{
					pVertex_QueueNode = pVertex_QueueNode->next;							

					/* get the shortest path distance from AP to this target point */
					distance = D[pVertex_QueueNode->vertex-1][pTP_QueueNode->target_point_id-1];

					/* update the min_distance */
					if(distance < min_distance)
					{
						min_distance = distance;
						min_target_point_id = pTP_QueueNode->target_point_id;
						min_anycast_set_id = pDV_QueueNode->vid;
						min_anycast_set_DV_qnode = pDV_QueueNode;
					}
				} //@for-1.1.1:end
			} //@for-1.1:end
		} //@for-1:end


		/** include min_anycast_set_id to anycast_set_id_vector and anycast_set_flag_vector */
		/* check the validity of min_anycast_set_id */
		if(min_distance == INF && min_anycast_set_id == 0 && min_target_point_id == 0)
		{
			printf("ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree_With_MinimumSpanningTreeAlgorithm(): there is no target point to be added into the spanning tree\n");
			exit(1);
		}
		else
		{
			/* update target_point_flag_vector */
			target_point_flag_vector[min_target_point_id] = TRUE;
			target_point_number++;		

			/* add min_anycast_set_id to mcast_group */
			memset(&mcast_qnode, 0, sizeof(mcast_qnode));
			mcast_qnode.vid = min_anycast_set_id;
			mcast_qnode.vnode = min_anycast_set_vnode;
			Enqueue((queue_t*)&mcast_group, (queue_node_t*)&mcast_qnode);

			/* turn on filter_flags of target points in the destination vehicle's FTPQ except min_target_point_id */
			Set_FilterFlags_In_DestinationVehicle_TargetPointSet_For_TargetPointFiltering(&(min_anycast_set_DV_qnode->FTPQ), min_target_point_id);
		}

		/** make a spanning tree T' with the path from AP to min_target_point_id */
		/* make the shortest path from AP to min_target_point_id */
		
		Make_PacketTrajectory_For_DeliveryCost_In_PacketForwardingTree(param, M, AP_vertex_id, min_target_point_id, &path);

		/* set spanning_tree_node_flag_vector with the vertices in the path */
		/* Also,make a vertex queue for the spanning tree with spanning_tree_node_flag_vector */
		pTR_QueueNode = &(path.head);
		for(i = 0; i < path.size; i++)
		{	
			pTR_QueueNode = pTR_QueueNode->next;
			id = pTR_QueueNode->intersection_id;

			/* check whether vertex corresponding to id has already been added to vertex_set or not */
			//flag = Is_Vertex_In_VertexQueue(&vertex_set, id);
			flag = spanning_tree_node_flag_vector[id];
			if(flag == FALSE)
			{ /* add vertex queue node for the id to vertex_set */
				/* enable the flag corresponding to id */
				spanning_tree_node_flag_vector[id] = TRUE;

				memset(&vertex_qnode, 0, sizeof(vertex_qnode));
				vertex_qnode.vertex = id;
				Enqueue((queue_t*)&vertex_set, (queue_node_t*)&vertex_qnode);
			}
		}

		/* check whether the current spanning tree can cover each destination vehicle's anycast set or not */
		pDV_QueueNode = &(DVQ->head);
		for(i = 0; i < DVQ->size; i++)
		{
			pDV_QueueNode = pDV_QueueNode->next;
		
			/* check whether this vehicle has already been in mcast_group */
			flag = Is_Vehicle_In_MulticastGroupVehicleQueue(&mcast_group, pDV_QueueNode->vid);
			if(flag)
		  		continue;

			/* check whether this vehicle's anycast set can be covered by the spanning tree */
			flag = Is_AnycastSet_Covered_By_SpanningTree_TargetPointSet(&(pDV_QueueNode->FTPQ), target_point_flag_vector, target_point_flag_vector_size, &covering_target_point);
			if(flag)
			{ /* this vehicle's anycast set is covered, so add this vehicle to mcast_group */
				memset(&mcast_qnode, 0, sizeof(mcast_qnode));
				mcast_qnode.vid = pDV_QueueNode->vid;
				mcast_qnode.vnode = pDV_QueueNode->vnode;
				Enqueue((queue_t*)&mcast_group, (queue_node_t*)&mcast_qnode);

				/* set cover_flag for covering_target_point and turn on filter_flags of all target points in the destination vehicle's FTPQ */
				Set_CoverFlags_And_FilterFlags_In_DestinationVehicle_TargetPointSet_For_TargetPointCovering(&(pDV_QueueNode->FTPQ), covering_target_point);
			}
		}
	}

	/** rebuild the adjacency matrix T with the spanning tree's vertex_set */
	GO_Rebuild_AdjacencyMatrix_With_SpanningTree(T, T_size, spanning_tree_node_flag_vector, spanning_tree_node_flag_vector_size);

	/** recompute the shortest path matrices D, M, and S for the steiner tree T */
	/* construct delay matrices D and S from the adjacency cost matrix T_prime for the augmented tree */
	Floyd_Warshall_Make_Weight_Matrix_For_EDC_With_AdjacencyMatrix(D, S, T_size, T, param);

	/* compute the matrices for all-pairs shortest-paths using the Floyd-Warshall algorithm */
	Floyd_Warshall_Construct_Shortest_PathInfo_For_EDC(D, M, S, T_size);

	/** reconstruct the feasible target point queue FTPQ with vertex_set */
	/* Destroy feasible target point queue FTPQ */
	if(FTPQ->size > 0)
	{
		DestroyQueue((queue_t*)FTPQ);
	}

	/* build FTPQ with target_point_flag_vector and target_point_flag_vector_size; note that the index of 1 is the first intersection id and the index of (target_point_flag_vector_size-1) is the last one */
	for(i = 1; i < target_point_flag_vector_size; i++)
	{
		if(target_point_flag_vector[i] == FALSE)
			continue;

		/* enqueue the vertex into FTPQ */
		memset(&tp_qnode, 0, sizeof(tp_qnode));
		tp_qnode.target_point_id = i;
		Enqueue((queue_t*)FTPQ, (queue_node_t*)&tp_qnode);
	}

	/** construct edge set queue ESQ with the spanning tree */
	/* Destroy edge set queue ESQ */
	if(ESQ->size > 0)
	{
		DestroyQueue((queue_t*)ESQ);
	}

	/* construct tree edge queue TEQ with multicast tree T and feasible target point queue FTPQ */
	GO_Construct_EdgeQueue_With_AdjacencyMatrix_And_FeasibleTargetPointQueue(T, T_size, FTPQ, ESQ);

	/* sort edge set queue in the descending order of edge weight */
	SortEdgeSetQueue_For_MultiAnycast(ESQ);

	/* mark target points in FTPQ that are covered by more than one anycast set
	 * by setting cover_flag to TRUE for those target points that are redundant 
	 * target points as relay nodes in the feasible target point queue FTPQ */
	GO_Mark_Duplicate_Covered_TargetPoint_In_TargetPointSet(current_time, DVQ, FTPQ);

	/* Note: target points with filter_flag set to FALSE will have a copy of the 
	 * multicasted packet */

	/** release the resources used in the computation of the optimal steiner tree */
	/* deallocate the memory for target_point_flag_vector */
	free(target_point_flag_vector);

	/* deallocate the memory for spanning_tree_node_flag_vector */
	free(spanning_tree_node_flag_vector);

	/* destroy packet trajectory queue path */
	DestroyQueue((queue_t*)&path);

	/* destroy vertex queue vertex_set */
	DestroyQueue((queue_t*)&vertex_set);

	/* destroy multicast group vehicle queue mcast_group */
	DestroyQueue((queue_t*)&mcast_group);

	return result;
}

int ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree_With_MinimumSpanningTreeAlgorithm_And_DeliveryDelayBound(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, double beta, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ)
{ /* filter out redundant target points from Minimum Steiner Tree T consisting of anycast sets of destination vehicles with Minimum Spanning Tree Algorithm and make a pruned multicast tree represented as edge set queue ESQ */
	int result = 0;
	double **T = param->vanet_table.Tr_mcast;
	double **D = param->vanet_table.Dr_mcast;
	int **M = param->vanet_table.Mr_mcast;
	double **S = param->vanet_table.Sr_mcast;
	int T_size = param->vanet_table.matrix_size_for_mcast_in_Gr;
	int redundant_target_point = 0; //redundant target point id
	int AP_vertex_id = atoi(AP_vertex); //vertex id of AP

	double min_distance = INF; //minimum interdistance between the spanning tree and an uncovered anycast set
	int min_target_point_id = 0; //vertex id of a target point with a minimum interdistance
	int min_anycast_set_id = 0; //anycast set id with a minimum interdistance
	struct_vehicle_t *min_anycast_set_vnode = NULL; //pointer to the vehicle with the minimum distance
	destination_vehicle_queue_node_t *min_anycast_set_DV_qnode = NULL; //pointer to the destination vehicle queue node with a minimum interdistance

	double distance = 0; //shortest path distance from a target point to another target point
	double distance_from_src_to_dst = 0; //get the shortest path distance from AP to a target point

	int i = 0, j = 0, k = 0; //loop-indices
	destination_vehicle_queue_node_t *pDV_QueueNode = NULL; //pointer to a destination vehicle queue node
	target_point_queue_node_t *pTP_QueueNode = NULL; //pointer to a target point queue node
	target_point_queue_node_t tp_qnode; //target point queue node

	boolean *target_point_flag_vector = NULL; //flag vector to indicate whether the target point whose id is the same as the index in the vector is included in the spanning tree or not
	int target_point_flag_vector_size = 0; //size of target point flag vector
	int target_point_number = 0; //number of target points 

	multicast_group_vehicle_queue_t mcast_group; //multicast group vehicle queue to represent which destination vehicles can be covered by the spanning tree
	multicast_group_vehicle_queue_node_t mcast_qnode; //multicast group vehicle queue node

	boolean *spanning_tree_node_flag_vector = NULL; //flag vector to indicate whether the node corresponding to the index belongs to the spanning tree node set or not
	int spanning_tree_node_flag_vector_size = 0; //size of spanning_tree_node_flag_vector_size

	packet_trajectory_queue_t path; //path from src to dst
	packet_trajectory_queue_node_t *pTR_QueueNode = NULL; //pointer to a trajectory queue node
	
	vertex_queue_t vertex_set; //vertex set for the spanning tree
	vertex_queue_node_t *pVertex_QueueNode = NULL; //pointer to a vertex queue node
	vertex_queue_node_t vertex_qnode; //vertex queue node

	int id = 0; //vertex id
	boolean flag = FALSE;
	boolean flag_for_mcast_group = FALSE; //flag used in the multicast group member addition

	int covering_target_point = 0; //covering target point for an anycast set

#if DEBUG_12_19_2010
	if(current_time > 7723)
		printf("ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree_With_MinimumSpanningTreeAlgorithm(): [%.2f] debugging\n", (float)current_time);	
#endif

	/* check the validity of AP_vertex_id */
	if((AP_vertex_id < 1) || (AP_vertex_id > T_size))
	{
		printf("ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree_With_MinimumSpanningTreeAlgorithm(): AP_vertex_id(%d) must be bounded between 1 and T_size(%d)\n", AP_vertex_id, T_size);
		exit(1);
	}

	/* check the validity of DVQ */
	if(DVQ->size <= 0)
	{
		printf("ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree_With_MinimumSpanningTreeAlgorithm(): DVQ->size(%d) is invalid\n", DVQ->size);
		exit(1);
	}

	/* initialize packet trajectory queue path */
	InitQueue((queue_t*)&path, QTYPE_PACKET_TRAJECTORY);

	/* initialize vertex queue vertex_set */
	InitQueue((queue_t*)&vertex_set, QTYPE_VERTEX);

	/* initialize multicast group vehicle queue mcast_group */
	InitQueue((queue_t*)&mcast_group, QTYPE_MULTICAST_GROUP_VEHICLE);

	/* allocate the memory for target_point_flag_vector */
	target_point_flag_vector_size = T_size + 1;
	target_point_flag_vector = (boolean*)calloc(target_point_flag_vector_size, sizeof(boolean));
	assert_memory(target_point_flag_vector);

	/* allocate the memory for spanning_tree_node_flag_vector */
	spanning_tree_node_flag_vector_size = T_size + 1;
	spanning_tree_node_flag_vector = (boolean*)calloc(spanning_tree_node_flag_vector_size, sizeof(boolean));
	assert_memory(spanning_tree_node_flag_vector);

	/* find a target point in an anycast set that is closest to the packet source, that is, AP */
	pDV_QueueNode = &(DVQ->head);
	for(i = 0; i < DVQ->size; i++)
	{
		pDV_QueueNode = pDV_QueueNode->next;
		pTP_QueueNode = &(pDV_QueueNode->FTPQ.head);
		for(j = 0; j < pDV_QueueNode->FTPQ.size; j++)
		{
			pTP_QueueNode = pTP_QueueNode->next;

			/* get the shortest path distance from AP to this target point */
			distance = D[AP_vertex_id-1][pTP_QueueNode->target_point_id-1];

			/* update the min_distance satisfying the delivery bound beta */
			if((distance <= beta) && (distance < min_distance))
			{
				min_distance = distance;
				min_target_point_id = pTP_QueueNode->target_point_id;
				min_anycast_set_id = pDV_QueueNode->vid;
				min_anycast_set_vnode = pDV_QueueNode->vnode;
				min_anycast_set_DV_qnode = pDV_QueueNode;
			}
		}
	}

	/** include min_anycast_set_id to anycast_set_id_vector and anycast_set_flag_vector */
	/* check the validity of min_anycast_set_id */
	if(min_distance == INF && min_anycast_set_id == 0 && min_target_point_id == 0)
	{
		printf("ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree_With_MinimumSpanningTreeAlgorithm(): there is no target point to be added into the spanning tree\n");
		exit(1);
	}
	else
	{
		/* update target_point_flag_vector */
		target_point_flag_vector[min_target_point_id] = TRUE;
		target_point_number++;		

		/* add min_anycast_set_id to mcast_group */
		memset(&mcast_qnode, 0, sizeof(mcast_qnode));
		mcast_qnode.vid = min_anycast_set_id;
		mcast_qnode.vnode = min_anycast_set_vnode;
		Enqueue((queue_t*)&mcast_group, (queue_node_t*)&mcast_qnode);

		/* turn on filter_flags of target points in the destination vehicle's FTPQ except min_target_point_id */
		Set_FilterFlags_In_DestinationVehicle_TargetPointSet_For_TargetPointFiltering(&(min_anycast_set_DV_qnode->FTPQ), min_target_point_id);
	}

	/** make a spanning tree T' with the path from AP to min_target_point_id */
	/* make the shortest path from AP to min_target_point_id */
	Make_PacketTrajectory_For_DeliveryCost_In_PacketForwardingTree(param, M, AP_vertex_id, min_target_point_id, &path);

	/* set spanning_tree_node_flag_vector with the vertices in the path */
	/* Also,make a vertex queue for the spanning tree with spanning_tree_node_flag_vector */
	pTR_QueueNode = &(path.head);
	for(i = 0; i < path.size; i++)
	{	
		pTR_QueueNode = pTR_QueueNode->next;
		id = pTR_QueueNode->intersection_id;

		/* enable the flag corresponding to id */
		spanning_tree_node_flag_vector[id] = TRUE;

		/* add vertex queue node for the id to vertex_set */
		memset(&vertex_qnode, 0, sizeof(vertex_qnode));
		vertex_qnode.vertex = id;
		Enqueue((queue_t*)&vertex_set, (queue_node_t*)&vertex_qnode);
	}

	/* check whether the current spanning tree can cover each destination vehicle's anycast set or not */
	pDV_QueueNode = &(DVQ->head);
	for(i = 0; i < DVQ->size; i++)
	{
		pDV_QueueNode = pDV_QueueNode->next;
		
		/* check whether this vehicle has already been in mcast_group */
		flag = Is_Vehicle_In_MulticastGroupVehicleQueue(&mcast_group, pDV_QueueNode->vid);
		if(flag)
		  continue;

		/* check whether this vehicle's anycast set can be covered by the spanning tree */
		flag = Is_AnycastSet_Covered_By_SpanningTree_TargetPointSet(&(pDV_QueueNode->FTPQ), target_point_flag_vector, target_point_flag_vector_size, &covering_target_point);
		if(flag)
		{ /* this vehicle's anycast set is covered, so add this vehicle to mcast_group */
			memset(&mcast_qnode, 0, sizeof(mcast_qnode));
			mcast_qnode.vid = pDV_QueueNode->vid;
			mcast_qnode.vnode = pDV_QueueNode->vnode;
			Enqueue((queue_t*)&mcast_group, (queue_node_t*)&mcast_qnode);

			/* set cover_flag for covering_target_point and turn on filter_flags of all target points in the destination vehicle's FTPQ */
			Set_CoverFlags_And_FilterFlags_In_DestinationVehicle_TargetPointSet_For_TargetPointCovering(&(pDV_QueueNode->FTPQ), covering_target_point);
		}
	}
				
	/** find an anycast set closest to the spanning tree and add it to the spanning tree set
	 * untill all of anycast sets are added to the spanning tree set */
	while(mcast_group.size < DVQ->size)
  	{
		/** for each node in T', find the target point u closest to T' where u's anycast set A(u) does not belong to S(T') where S(T') is the set of anycast sets covered by T' */
		/* reset min_distance variables */
		min_distance = INF;
		min_target_point_id = 0;
		min_anycast_set_id = 0;
		flag_for_mcast_group = FALSE;

		pDV_QueueNode = &(DVQ->head);
		for(i = 0; i < DVQ->size; i++) //@for-1:start
		{
			pDV_QueueNode = pDV_QueueNode->next;

			/* check whether this vehicle has already been in mcast_group */
			flag = Is_Vehicle_In_MulticastGroupVehicleQueue(&mcast_group, pDV_QueueNode->vid);
			if(flag)
		  		continue;

			pTP_QueueNode = &(pDV_QueueNode->FTPQ.head);
			for(j = 0; j < pDV_QueueNode->FTPQ.size; j++) //@for-1.1:start
			{
				pTP_QueueNode = pTP_QueueNode->next;

				/* check whether this vertex has already been in vertex_set or not */
				//flag = Is_Vertex_In_VertexQueue(&vertex_set, pTP_QueueNode->target_point_id);
				flag = spanning_tree_node_flag_vector[pTP_QueueNode->target_point_id];
#if 0 /* [ */
				if(flag)
					continue;
#endif /* ] */

#if 1 /* [[ */
				if(flag)
				{
					min_distance = 0;
					min_target_point_id = pTP_QueueNode->target_point_id;
					min_anycast_set_id = pDV_QueueNode->vid;
					min_anycast_set_DV_qnode = pDV_QueueNode;
					flag_for_mcast_group = TRUE;
					break;
				}
#endif /* ]] */

				pVertex_QueueNode = &(vertex_set.head);
				for(k = 0; k < vertex_set.size; k++) //@for-1.1.1:start
				{
					pVertex_QueueNode = pVertex_QueueNode->next;							

					/* get the shortest path distance from AP to this target point */
					distance = D[pVertex_QueueNode->vertex-1][pTP_QueueNode->target_point_id-1];

					/* get the shortest path distance from AP to this target point */
					distance_from_src_to_dst = D[AP_vertex_id-1][pTP_QueueNode->target_point_id-1];

					/* update the min_distance satisfying the delivery delay bound beta */
					if((distance_from_src_to_dst <= beta) && (distance < min_distance))
					{
						min_distance = distance;
						min_target_point_id = pTP_QueueNode->target_point_id;
						min_anycast_set_id = pDV_QueueNode->vid;
						min_anycast_set_DV_qnode = pDV_QueueNode;
					}
				} //@for-1.1.1:end
			} //@for-1.1:end
		} //@for-1:end


		/** include min_anycast_set_id to anycast_set_id_vector and anycast_set_flag_vector */
		/* check the validity of min_anycast_set_id */
		if(min_distance == INF && min_anycast_set_id == 0 && min_target_point_id == 0)
		{
			printf("ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree_With_MinimumSpanningTreeAlgorithm(): there is no target point to be added into the spanning tree\n");
			exit(1);
		}
		else
		{
			/* update target_point_flag_vector */
			target_point_flag_vector[min_target_point_id] = TRUE;
			target_point_number++;		

			/* add min_anycast_set_id to mcast_group */
			memset(&mcast_qnode, 0, sizeof(mcast_qnode));
			mcast_qnode.vid = min_anycast_set_id;
			mcast_qnode.vnode = min_anycast_set_vnode;
			Enqueue((queue_t*)&mcast_group, (queue_node_t*)&mcast_qnode);

			/* turn on filter_flags of target points in the destination vehicle's FTPQ except min_target_point_id */
			Set_FilterFlags_In_DestinationVehicle_TargetPointSet_For_TargetPointFiltering(&(min_anycast_set_DV_qnode->FTPQ), min_target_point_id);
		}

		/** make a spanning tree T' with the path from AP to min_target_point_id */
		/* make the shortest path from AP to min_target_point_id */
		
		Make_PacketTrajectory_For_DeliveryCost_In_PacketForwardingTree(param, M, AP_vertex_id, min_target_point_id, &path);

		/* set spanning_tree_node_flag_vector with the vertices in the path */
		/* Also,make a vertex queue for the spanning tree with spanning_tree_node_flag_vector */
		pTR_QueueNode = &(path.head);
		for(i = 0; i < path.size; i++)
		{	
			pTR_QueueNode = pTR_QueueNode->next;
			id = pTR_QueueNode->intersection_id;

			/* check whether vertex corresponding to id has already been added to vertex_set or not */
			//flag = Is_Vertex_In_VertexQueue(&vertex_set, id);
			flag = spanning_tree_node_flag_vector[id];
			if(flag == FALSE)
			{ /* add vertex queue node for the id to vertex_set */
				/* enable the flag corresponding to id */
				spanning_tree_node_flag_vector[id] = TRUE;

				memset(&vertex_qnode, 0, sizeof(vertex_qnode));
				vertex_qnode.vertex = id;
				Enqueue((queue_t*)&vertex_set, (queue_node_t*)&vertex_qnode);
			}
		}

		/* check whether the current spanning tree can cover each destination vehicle's anycast set or not */
		pDV_QueueNode = &(DVQ->head);
		for(i = 0; i < DVQ->size; i++)
		{
			pDV_QueueNode = pDV_QueueNode->next;
		
			/* check whether this vehicle has already been in mcast_group */
			flag = Is_Vehicle_In_MulticastGroupVehicleQueue(&mcast_group, pDV_QueueNode->vid);
			if(flag)
		  		continue;

			/* check whether this vehicle's anycast set can be covered by the spanning tree */
			flag = Is_AnycastSet_Covered_By_SpanningTree_TargetPointSet(&(pDV_QueueNode->FTPQ), target_point_flag_vector, target_point_flag_vector_size, &covering_target_point);
			if(flag)
			{ /* this vehicle's anycast set is covered, so add this vehicle to mcast_group */
				memset(&mcast_qnode, 0, sizeof(mcast_qnode));
				mcast_qnode.vid = pDV_QueueNode->vid;
				mcast_qnode.vnode = pDV_QueueNode->vnode;
				Enqueue((queue_t*)&mcast_group, (queue_node_t*)&mcast_qnode);

				/* set cover_flag for covering_target_point and turn on filter_flags of all target points in the destination vehicle's FTPQ */
				Set_CoverFlags_And_FilterFlags_In_DestinationVehicle_TargetPointSet_For_TargetPointCovering(&(pDV_QueueNode->FTPQ), covering_target_point);
			}
		}
	}

	/** rebuild the adjacency matrix T with the spanning tree's vertex_set */
	GO_Rebuild_AdjacencyMatrix_With_SpanningTree(T, T_size, spanning_tree_node_flag_vector, spanning_tree_node_flag_vector_size);

	/** recompute the shortest path matrices D, M, and S for the steiner tree T */
	/* construct delay matrices D and S from the adjacency cost matrix T_prime for the augmented tree */
	Floyd_Warshall_Make_Weight_Matrix_For_EDC_With_AdjacencyMatrix(D, S, T_size, T, param);

	/* compute the matrices for all-pairs shortest-paths using the Floyd-Warshall algorithm */
	Floyd_Warshall_Construct_Shortest_PathInfo_For_EDC(D, M, S, T_size);

	/** reconstruct the feasible target point queue FTPQ with vertex_set */
	/* Destroy feasible target point queue FTPQ */
	if(FTPQ->size > 0)
	{
		DestroyQueue((queue_t*)FTPQ);
	}

	/* build FTPQ with target_point_flag_vector and target_point_flag_vector_size; note that the index of 1 is the first intersection id and the index of (target_point_flag_vector_size-1) is the last one */
	for(i = 1; i < target_point_flag_vector_size; i++)
	{
		if(target_point_flag_vector[i] == FALSE)
			continue;

		/* enqueue the vertex into FTPQ */
		memset(&tp_qnode, 0, sizeof(tp_qnode));
		tp_qnode.target_point_id = i;
		Enqueue((queue_t*)FTPQ, (queue_node_t*)&tp_qnode);
	}

	/** construct edge set queue ESQ with the spanning tree */
	/* Destroy edge set queue ESQ */
	if(ESQ->size > 0)
	{
		DestroyQueue((queue_t*)ESQ);
	}

	/* construct tree edge queue TEQ with multicast tree T and feasible target point queue FTPQ */
	GO_Construct_EdgeQueue_With_AdjacencyMatrix_And_FeasibleTargetPointQueue(T, T_size, FTPQ, ESQ);

	/* sort edge set queue in the descending order of edge weight */
	SortEdgeSetQueue_For_MultiAnycast(ESQ);

	/* mark target points in FTPQ that are covered by more than one anycast set
	 * by setting cover_flag to TRUE for those target points that are redundant 
	 * target points as relay nodes in the feasible target point queue FTPQ */
	GO_Mark_Duplicate_Covered_TargetPoint_In_TargetPointSet(current_time, DVQ, FTPQ);

	/* Note: target points with filter_flag set to FALSE will have a copy of the 
	 * multicasted packet */

	/** release the resources used in the computation of the optimal steiner tree */
	/* deallocate the memory for target_point_flag_vector */
	free(target_point_flag_vector);

	/* deallocate the memory for spanning_tree_node_flag_vector */
	free(spanning_tree_node_flag_vector);

	/* destroy packet trajectory queue path */
	DestroyQueue((queue_t*)&path);

	/* destroy vertex queue vertex_set */
	DestroyQueue((queue_t*)&vertex_set);

	/* destroy multicast group vehicle queue mcast_group */
	DestroyQueue((queue_t*)&mcast_group);

	return result;
}

int ST_FilterOut_RedundantTargetPoints_From_AllAnycastSets_With_MinimumSpanningTreeAlgorithm_And_DeliveryDelayBound(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, double beta, target_point_queue_t *FTPQ)
{ /* filter out redundant target points from All Anycast Sets of destination vehicles with Minimum Spanning Tree Algorithm along with delivery delay bound beta and make a feasible target point set FTPQ */ 
	int result = 0;
	double **T = param->vanet_table.Wr_edc;
	double **D = param->vanet_table.Dr_edc;
	int **M = param->vanet_table.Mr_edc;
	double **S = param->vanet_table.Sr_edc;
	int T_size = param->vanet_table.matrix_size_for_edc_in_Gr;
	int redundant_target_point = 0; //redundant target point id
	int AP_vertex_id = atoi(AP_vertex); //vertex id of AP

	double min_distance = INF; //minimum interdistance between the spanning tree and an uncovered anycast set
	int min_target_point_id = 0; //vertex id of a target point with a minimum interdistance
	int min_anycast_set_id = 0; //anycast set id with a minimum interdistance
	struct_vehicle_t *min_anycast_set_vnode = NULL; //pointer to the vehicle with the minimum distance
	destination_vehicle_queue_node_t *min_anycast_set_DV_qnode = NULL; //pointer to the destination vehicle queue node with a minimum interdistance

	double distance = 0; //shortest path distance from a target point to another target point
	double distance_from_src_to_dst = 0; //get the shortest path distance from AP to a target point

	int i = 0, j = 0, k = 0; //loop-indices
	destination_vehicle_queue_node_t *pDV_QueueNode = NULL; //pointer to a destination vehicle queue node
	target_point_queue_node_t *pTP_QueueNode = NULL; //pointer to a target point queue node
	target_point_queue_node_t tp_qnode; //target point queue node

	boolean *target_point_flag_vector = NULL; //flag vector to indicate whether the target point whose id is the same as the index in the vector is included in the spanning tree or not
	int target_point_flag_vector_size = 0; //size of target point flag vector
	int target_point_number = 0; //number of target points 

	multicast_group_vehicle_queue_t mcast_group; //multicast group vehicle queue to represent which destination vehicles can be covered by the spanning tree
	multicast_group_vehicle_queue_node_t mcast_qnode; //multicast group vehicle queue node

	boolean *spanning_tree_node_flag_vector = NULL; //flag vector to indicate whether the node corresponding to the index belongs to the spanning tree node set or not
	int spanning_tree_node_flag_vector_size = 0; //size of spanning_tree_node_flag_vector_size

	packet_trajectory_queue_t path; //path from src to dst
	packet_trajectory_queue_node_t *pTR_QueueNode = NULL; //pointer to a trajectory queue node
	
	vertex_queue_t vertex_set; //vertex set for the spanning tree
	vertex_queue_node_t *pVertex_QueueNode = NULL; //pointer to a vertex queue node
	vertex_queue_node_t vertex_qnode; //vertex queue node

	int id = 0; //vertex id
	boolean flag = FALSE;
	boolean flag_for_mcast_group = FALSE; //flag used in the multicast group member addition

	int covering_target_point = 0; //covering target point for an anycast set

#if DEBUG_02_09_2011
	if(current_time > 7682)
		printf("ST_FilterOut_RedundantTargetPoints_From_AllAnycastSets_With_MinimumSpanningTreeAlgorithm(): [%.2f] debugging\n", (float)current_time);	
#endif

	/* check the validity of AP_vertex_id */
	if((AP_vertex_id < 1) || (AP_vertex_id > T_size))
	{
		printf("ST_FilterOut_RedundantTargetPoints_From_AllAnycastSets_With_MinimumSpanningTreeAlgorithm(): AP_vertex_id(%d) must be bounded between 1 and T_size(%d)\n", AP_vertex_id, T_size);
		exit(1);
	}

	/* check the validity of DVQ */
	if(DVQ->size <= 0)
	{
		printf("ST_FilterOut_RedundantTargetPoints_From_AllAnycastSets_With_MinimumSpanningTreeAlgorithm(): DVQ->size(%d) is invalid\n", DVQ->size);
		exit(1);
	}

	/* initialize packet trajectory queue path */
	InitQueue((queue_t*)&path, QTYPE_PACKET_TRAJECTORY);

	/* initialize vertex queue vertex_set */
	InitQueue((queue_t*)&vertex_set, QTYPE_VERTEX);

	/* initialize multicast group vehicle queue mcast_group */
	InitQueue((queue_t*)&mcast_group, QTYPE_MULTICAST_GROUP_VEHICLE);

	/* allocate the memory for target_point_flag_vector */
	target_point_flag_vector_size = T_size + 1;
	target_point_flag_vector = (boolean*)calloc(target_point_flag_vector_size, sizeof(boolean));
	assert_memory(target_point_flag_vector);

	/* allocate the memory for spanning_tree_node_flag_vector */
	spanning_tree_node_flag_vector_size = T_size + 1;
	spanning_tree_node_flag_vector = (boolean*)calloc(spanning_tree_node_flag_vector_size, sizeof(boolean));
	assert_memory(spanning_tree_node_flag_vector);

	/* find a target point in an anycast set that is closest to the packet source, that is, AP, while satifying the delay bound beta */
	pDV_QueueNode = &(DVQ->head);
	for(i = 0; i < DVQ->size; i++)
	{
		pDV_QueueNode = pDV_QueueNode->next;
		pTP_QueueNode = &(pDV_QueueNode->FTPQ.head);
		for(j = 0; j < pDV_QueueNode->FTPQ.size; j++)
		{
			pTP_QueueNode = pTP_QueueNode->next;

			/* get the shortest path distance from AP to this target point */
			distance = D[AP_vertex_id-1][pTP_QueueNode->target_point_id-1];

			/* update the min_distance satisfying the delivery bound beta */
			if((distance <= beta) && (distance < min_distance))
			{
				min_distance = distance;
				min_target_point_id = pTP_QueueNode->target_point_id;
				min_anycast_set_id = pDV_QueueNode->vid;
				min_anycast_set_vnode = pDV_QueueNode->vnode;
				min_anycast_set_DV_qnode = pDV_QueueNode;
			}
		}
	}

	/** include min_anycast_set_id to anycast_set_id_vector and anycast_set_flag_vector */
	/* check the validity of min_anycast_set_id */
	if(min_distance == INF && min_anycast_set_id == 0 && min_target_point_id == 0)
	{
	  printf("ST_FilterOut_RedundantTargetPoints_From_AllAnycastSets_With_MinimumSpanningTreeAlgorithm(): line %d: there is no target point to be added into the spanning tree\n", __LINE__);
		exit(1);
	}
	else
	{
		/* update target_point_flag_vector */
		target_point_flag_vector[min_target_point_id] = TRUE;
		target_point_number++;		

		/* add min_anycast_set_id to mcast_group */
		memset(&mcast_qnode, 0, sizeof(mcast_qnode));
		mcast_qnode.vid = min_anycast_set_id;
		mcast_qnode.vnode = min_anycast_set_vnode;
		Enqueue((queue_t*)&mcast_group, (queue_node_t*)&mcast_qnode);

		/* turn on filter_flags of target points in the destination vehicle's FTPQ except min_target_point_id */
		Set_FilterFlags_In_DestinationVehicle_TargetPointSet_For_TargetPointFiltering(&(min_anycast_set_DV_qnode->FTPQ), min_target_point_id);
	}

	/** make a spanning tree T' with the path from AP to min_target_point_id */
	/* make the shortest path from AP to min_target_point_id */
	Make_PacketTrajectory_For_DeliveryCost_In_PacketForwardingTree(param, M, AP_vertex_id, min_target_point_id, &path);

	/* set spanning_tree_node_flag_vector with the vertices in the path */
	/* Also,make a vertex queue for the spanning tree with spanning_tree_node_flag_vector */
	pTR_QueueNode = &(path.head);
	for(i = 0; i < path.size; i++)
	{	
		pTR_QueueNode = pTR_QueueNode->next;
		id = pTR_QueueNode->intersection_id;

		/* enable the flag corresponding to id */
		spanning_tree_node_flag_vector[id] = TRUE;

		/* add vertex queue node for the id to vertex_set */
		memset(&vertex_qnode, 0, sizeof(vertex_qnode));
		vertex_qnode.vertex = id;
		Enqueue((queue_t*)&vertex_set, (queue_node_t*)&vertex_qnode);
	}

	/* check whether the current spanning tree can cover each destination vehicle's anycast set or not */
	pDV_QueueNode = &(DVQ->head);
	for(i = 0; i < DVQ->size; i++)
	{
		pDV_QueueNode = pDV_QueueNode->next;
		
		/* check whether this vehicle has already been in mcast_group */
		flag = Is_Vehicle_In_MulticastGroupVehicleQueue(&mcast_group, pDV_QueueNode->vid);
		if(flag)
		  continue;

		/* check whether this vehicle's anycast set can be covered by the spanning tree */
		flag = Is_AnycastSet_Covered_By_SpanningTree_TargetPointSet(&(pDV_QueueNode->FTPQ), target_point_flag_vector, target_point_flag_vector_size, &covering_target_point);
		if(flag)
		{ /* this vehicle's anycast set is covered, so add this vehicle to mcast_group */
			memset(&mcast_qnode, 0, sizeof(mcast_qnode));
			mcast_qnode.vid = pDV_QueueNode->vid;
			mcast_qnode.vnode = pDV_QueueNode->vnode;
			Enqueue((queue_t*)&mcast_group, (queue_node_t*)&mcast_qnode);

			/* set cover_flag for covering_target_point and turn on filter_flags of all target points in the destination vehicle's FTPQ */
			Set_CoverFlags_And_FilterFlags_In_DestinationVehicle_TargetPointSet_For_TargetPointCovering(&(pDV_QueueNode->FTPQ), covering_target_point);
		}
	}
				
	/** find an anycast set closest to the spanning tree and add it to the spanning tree set
	 * untill all of anycast sets are added to the spanning tree set */
	while (mcast_group.size < DVQ->size)	
  	{
		/** for each node in T', find the target point u closest to T' where u's anycast set A(u) does not belong to S(T') where S(T') is the set of anycast sets covered by T' */
		/* reset min_distance variables */
		min_distance = INF;
		min_target_point_id = 0;
		min_anycast_set_id = 0;
		flag_for_mcast_group = FALSE;

		pDV_QueueNode = &(DVQ->head);
		for(i = 0; (i < DVQ->size) && (flag_for_mcast_group == FALSE); i++) //@for-1:start
		{
			pDV_QueueNode = pDV_QueueNode->next;

			/* check whether this vehicle has already been in mcast_group */
			flag = Is_Vehicle_In_MulticastGroupVehicleQueue(&mcast_group, pDV_QueueNode->vid);
			if(flag)
		  		continue;

			pTP_QueueNode = &(pDV_QueueNode->FTPQ.head);
			for(j = 0; j < pDV_QueueNode->FTPQ.size; j++) //@for-1.1:start
			{
				pTP_QueueNode = pTP_QueueNode->next;

				/* check whether this vertex has already been in vertex_set or not */
				//flag = Is_Vertex_In_VertexQueue(&vertex_set, pTP_QueueNode->target_point_id);

				flag = spanning_tree_node_flag_vector[pTP_QueueNode->target_point_id];
#if 0 /* [ */
				if(flag)
					continue;
#endif /* ] */

#if 1 /* [[ */
				if(flag)
				{
					min_distance = 0;
					min_target_point_id = pTP_QueueNode->target_point_id;
					min_anycast_set_id = pDV_QueueNode->vid;
					min_anycast_set_DV_qnode = pDV_QueueNode;
					flag_for_mcast_group = TRUE;
					break;
				}
#endif /* ]] */

				pVertex_QueueNode = &(vertex_set.head);
				for(k = 0; k < vertex_set.size; k++) //@for-1.1.1:start
				{
					pVertex_QueueNode = pVertex_QueueNode->next;							

					/* get the shortest path distance from vertex to this target point */
					distance = D[pVertex_QueueNode->vertex-1][pTP_QueueNode->target_point_id-1];
					
					/* get the shortest path distance from AP to this target point */
					distance_from_src_to_dst = D[AP_vertex_id-1][pTP_QueueNode->target_point_id-1];

					/* update the min_distance satisfying the delivery delay bound beta */
					if((distance_from_src_to_dst <= beta) && (distance < min_distance))
					{
						min_distance = distance;
						min_target_point_id = pTP_QueueNode->target_point_id;
						min_anycast_set_id = pDV_QueueNode->vid;
						min_anycast_set_DV_qnode = pDV_QueueNode;
					}
				} //@for-1.1.1:end
			} //@for-1.1:end
		} //@for-1:end


		/** include min_anycast_set_id to anycast_set_id_vector and anycast_set_flag_vector */
		/* check the validity of min_anycast_set_id */
		if(min_distance == INF && min_anycast_set_id == 0 && min_target_point_id == 0)
		{
		  printf("ST_FilterOut_RedundantTargetPoints_From_AllAnycastSets_With_MinimumSpanningTreeAlgorithm(): line %d: there is no target point to be added into the spanning tree\n", __LINE__);
			exit(1);
		}
		else
		{
			/* update target_point_flag_vector */
			target_point_flag_vector[min_target_point_id] = TRUE;
			target_point_number++;		

			/* add min_anycast_set_id to mcast_group */
			memset(&mcast_qnode, 0, sizeof(mcast_qnode));
			mcast_qnode.vid = min_anycast_set_id;
			mcast_qnode.vnode = min_anycast_set_vnode;
			Enqueue((queue_t*)&mcast_group, (queue_node_t*)&mcast_qnode);

			/* turn on filter_flags of target points in the destination vehicle's FTPQ except min_target_point_id */
			Set_FilterFlags_In_DestinationVehicle_TargetPointSet_For_TargetPointFiltering(&(min_anycast_set_DV_qnode->FTPQ), min_target_point_id);
		}

		/** make a spanning tree T' with the path from AP to min_target_point_id */
		/* make the shortest path from AP to min_target_point_id */
		
		Make_PacketTrajectory_For_DeliveryCost_In_PacketForwardingTree(param, M, AP_vertex_id, min_target_point_id, &path);

		/* set spanning_tree_node_flag_vector with the vertices in the path */
		/* Also,make a vertex queue for the spanning tree with spanning_tree_node_flag_vector */
		pTR_QueueNode = &(path.head);
		for(i = 0; i < path.size; i++)
		{	
			pTR_QueueNode = pTR_QueueNode->next;
			id = pTR_QueueNode->intersection_id;

			/* check whether vertex corresponding to id has already been added to vertex_set or not */
			//flag = Is_Vertex_In_VertexQueue(&vertex_set, id);
			flag = spanning_tree_node_flag_vector[id];
			if(flag == FALSE)
			{ /* add vertex queue node for the id to vertex_set */
				/* enable the flag corresponding to id */
				spanning_tree_node_flag_vector[id] = TRUE;

				memset(&vertex_qnode, 0, sizeof(vertex_qnode));
				vertex_qnode.vertex = id;
				Enqueue((queue_t*)&vertex_set, (queue_node_t*)&vertex_qnode);
			}
		}

		/* check whether the current spanning tree can cover each destination vehicle's anycast set or not */
		pDV_QueueNode = &(DVQ->head);
		for(i = 0; i < DVQ->size; i++)
		{
			pDV_QueueNode = pDV_QueueNode->next;
		
			/* check whether this vehicle has already been in mcast_group */
			flag = Is_Vehicle_In_MulticastGroupVehicleQueue(&mcast_group, pDV_QueueNode->vid);
			if(flag)
		  		continue;

			/* check whether this vehicle's anycast set can be covered by the spanning tree */
			flag = Is_AnycastSet_Covered_By_SpanningTree_TargetPointSet(&(pDV_QueueNode->FTPQ), target_point_flag_vector, target_point_flag_vector_size, &covering_target_point);
			if(flag)
			{ /* this vehicle's anycast set is covered, so add this vehicle to mcast_group */
				memset(&mcast_qnode, 0, sizeof(mcast_qnode));
				mcast_qnode.vid = pDV_QueueNode->vid;
				mcast_qnode.vnode = pDV_QueueNode->vnode;
				Enqueue((queue_t*)&mcast_group, (queue_node_t*)&mcast_qnode);

				/* set cover_flag for covering_target_point and turn on filter_flags of all target points in the destination vehicle's FTPQ */
				Set_CoverFlags_And_FilterFlags_In_DestinationVehicle_TargetPointSet_For_TargetPointCovering(&(pDV_QueueNode->FTPQ), covering_target_point);
			}
		}
	}

#if 0 /* [ */
	/** build the adjacency matrix T with the spanning tree's vertex_set */
	GO_Rebuild_AdjacencyMatrix_With_SpanningTree(T, T_size, spanning_tree_node_flag_vector, spanning_tree_node_flag_vector_size);

	/** recompute the shortest path matrices D, M, and S for the adjacency matrix T */
	/* construct delay matrices D and S from the adjacency cost matrix T_prime for the augmented tree */
	Floyd_Warshall_Make_Weight_Matrix_For_EDC_With_AdjacencyMatrix(D, S, T_size, T, param);

	/* compute the matrices for all-pairs shortest-paths using the Floyd-Warshall algorithm */
	Floyd_Warshall_Construct_Shortest_PathInfo_For_EDC(D, M, S, T_size);
#endif /* ] */


	/** reconstruct the feasible target point queue FTPQ with vertex_set */
	/* Destroy feasible target point queue FTPQ */
	if(FTPQ->size > 0)
	{
		DestroyQueue((queue_t*)FTPQ);
	}

	/* build FTPQ with target_point_flag_vector and target_point_flag_vector_size; note that the index of 1 is the first intersection id and the index of (target_point_flag_vector_size-1) is the last one */
	for(i = 1; i < target_point_flag_vector_size; i++)
	{
		if(target_point_flag_vector[i] == FALSE)
			continue;

		/* enqueue the vertex into FTPQ */
		memset(&tp_qnode, 0, sizeof(tp_qnode));
		tp_qnode.target_point_id = i;
		Enqueue((queue_t*)FTPQ, (queue_node_t*)&tp_qnode);
	}


#if 0 /* [ */
	/** construct edge set queue ESQ with the spanning tree */
	/* Destroy edge set queue ESQ */
	if(ESQ->size > 0)
	{
		DestroyQueue((queue_t*)ESQ);
	}

	/* construct tree edge queue TEQ with multicast tree T and feasible target point queue FTPQ */
	GO_Construct_EdgeQueue_With_AdjacencyMatrix_And_FeasibleTargetPointQueue(T, T_size, FTPQ, ESQ);

	/* sort edge set queue in the descending order of edge weight */
	SortEdgeSetQueue_For_MultiAnycast(ESQ);

	/* mark target points in FTPQ that are covered by more than one anycast set
	 * by setting cover_flag to TRUE for those target points that are redundant 
	 * target points as relay nodes in the feasible target point queue FTPQ */
	GO_Mark_Duplicate_Covered_TargetPoint_In_TargetPointSet(current_time, DVQ, FTPQ);

	/* Note: target points with filter_flag set to FALSE will have a copy of the 
	 * multicasted packet */
#endif /* ] */

	/** release the resources used in the computation of the optimal steiner tree */
	/* deallocate the memory for target_point_flag_vector */
	free(target_point_flag_vector);

	/* deallocate the memory for spanning_tree_node_flag_vector */
	free(spanning_tree_node_flag_vector);

	/* destroy packet trajectory queue path */
	DestroyQueue((queue_t*)&path);

	/* destroy vertex queue vertex_set */
	DestroyQueue((queue_t*)&vertex_set);

	/* destroy multicast group vehicle queue mcast_group */
	DestroyQueue((queue_t*)&mcast_group);

	return result;
}

void ST_FilterOut_InfeasibleTargetPoints_From_AllAnycastSets(destination_vehicle_queue_t *DVQ, double beta)
{ /* filter out target points not satisfying the delay bound beta from the feasible target point queues in DVQ */
	destination_vehicle_queue_node_t *pDV_QueueNode = NULL; //pointer to a destination queue node
	target_point_queue_t *FTPQ = NULL; //pointer to a feasible target point queue 
	int i = 0; //loop index

	pDV_QueueNode = &(DVQ->head);
	for(i = 0; i < DVQ->size; i++)
	{
		pDV_QueueNode = pDV_QueueNode->next;

		/* filter out target points not satisfying the delay bound beta from the feasible target point queue FTPQ */
		ST_FilterOut_InfeasibleTargetPoints_From_AnycastSet(&(pDV_QueueNode->FTPQ), beta);
	}
}

void ST_FilterOut_InfeasibleTargetPoints_From_AnycastSet(target_point_queue_t *FTPQ, double beta)
{ /* filter out target points not satisfying the delay bound beta from the feasible target point queue FTPQ */
	target_point_queue_node_t *pTP_QueueNode = NULL; //pointer to a target point queue node
	int i = 0; //loop index

	pTP_QueueNode = &(FTPQ->head);
	for(i = 0; i < FTPQ->size; i++)
	{
	  pTP_QueueNode = (target_point_queue_node_t*)GetQueueNode((queue_t*)FTPQ, i);

		/* filter out target points not satisfying the delay bound beta from the feasible target point queue FTPQ */
		if((pTP_QueueNode != NULL) && (pTP_QueueNode->delivery_delay > beta))
		{
			Dequeue_With_QueueNodePointer((queue_t*)FTPQ, (queue_node_t*)pTP_QueueNode);
			delete(pTP_QueueNode);
			i--;
			continue; //decrease i by 1 to let all queue nodes searched
		}
	}
}		

/** Construction of Multicast Tree in Single AP Scenarios */
void ST_Construct_Multicast_Tree_Based_On_TMA_Feasible_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet)
{ /* construct a multicast tree according to the TMA-feasible-target-point-selection algorithm */

#if 0
	if(current_time > 7463 && global_packet->id == 194)
		printf("[%.2f] ST_Construct_Multicast_Tree_Based_On_TMA_Feasible_Target_Point_Selection():\n packet_id=%d is traced\n", (float)current_time, global_packet->id);
#endif

	/* compute feasible target point sets for all the destination vehicles in destination vehicle queue DVQ */
	Compute_Feasible_TargetPoint_Sets_For_All_DestinationVehicles(param, current_time, AP_vertex, DVQ);
			
	/* [////////////////////////////////////////////////////////////////////////////////////////////////
	/* merge the feasible target point sets in DVQ into one target point set called FTPQ */
	Merge_DestinationVehicle_FeasibleTargetPointSets_By_TMA_TargetPoint_Selection(param, DVQ, FTPQ);
	////////////////////////////////////////////////////////////////////////////////////////////////] */

#if TARGET_POINT_ANYCAST_SET_TRACE_FLAG
	/* print out target points in the multicast tree */
	printf("pid=%d<<Before Filtering:\n", global_packet->id);
	PrintOut_TargetPoints_In_MulticastTree(current_time, FTPQ);
#endif	

	/* construct a Steiner Tree with the target point set FTPS */
	ST_Get_GreedyMulticastTree(param, current_time, atoi(AP_vertex), FTPQ, DVQ);

	/* filter out redundant target points from Minimum Steiner Tree T consisting of anycast sets of destination vehicles */
	ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree(param, current_time, DVQ, FTPQ, ESQ);

	/* compute Packet TTL with the longest vehicle travel time from DVQ according to param->communication_packet_ttl_override_flag */
	global_packet->ttl = Compute_Packet_TTL_With_VehicleTravelTime(param, DVQ);

	/* construct multicast tree information with target points to keep the copy of this packet */
	Create_Multicast_Forwarding_Table_For_GlobalPacket_AlongWith_MulticastTreeCost(param, ESQ, FTPQ, global_packet);

#if FEASIBLE_TARGET_POINT_QUEUE_DISPLAY_FLAG
	/* display the feasible target point queue for the multicast tree */
	Show_FeasibleTargetPoints_For_TargetPointQueue(current_time, FTPQ);
#endif

#if EDGE_SET_QUEUE_DISPLAY_FLAG
	/* display the edge set queue for the multicast tree */
	Show_EdgeSetQueue_For_MulticastTree(current_time, ESQ);
#endif

#if DESTINATION_VEHICLES_TARGET_POINT_QUEUES_DISPLAY_FLAG
	/* display the target point queues for the destination vehicles */
	Show_TargetPointInformation_For_All_DestinationVehicles(current_time, DVQ);
#endif
}

void ST_Construct_Multicast_Tree_Based_On_Random_Feasible_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet)
{ /* construct a multicast tree according to the Random-feasible-target-point-selection algorithm */

#if 0
	if(current_time > 7359 && global_packet->id == 38)
		printf("[%.2f] ST_Construct_Multicast_Tree_Based_On_Random_Feasible_Target_Point_Selection():\n packet_id=%d is traced\n", (float)current_time, global_packet->id);
#endif


	/* compute feasible target point sets for all the destination vehicles in destination vehicle queue DVQ */
	Compute_Feasible_TargetPoint_Sets_For_All_DestinationVehicles(param, current_time, AP_vertex, DVQ);
			
	/* [////////////////////////////////////////////////////////////////////////////////////////////////
	/* merge the feasible target point sets in DVQ into one target point set called FTPQ */
	Merge_DestinationVehicle_TargetPointSets_By_Random_TargetPoint_Selection(param, DVQ, FTPQ);
	////////////////////////////////////////////////////////////////////////////////////////////////] */

#if TARGET_POINT_ANYCAST_SET_TRACE_FLAG
	/* print out target points in the multicast tree */
	printf("pid=%d<<Before Filtering:\n", global_packet->id);
	PrintOut_TargetPoints_In_MulticastTree(current_time, FTPQ);
#endif	

	/* construct a Steiner Tree with the target point set FTPS */
	ST_Get_GreedyMulticastTree(param, current_time, atoi(AP_vertex), FTPQ, DVQ);

	/* filter out redundant target points from Minimum Steiner Tree T consisting of anycast sets of destination vehicles */
	ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree(param, current_time, DVQ, FTPQ, ESQ);

	/* compute Packet TTL with the longest vehicle travel time from DVQ according to param->communication_packet_ttl_override_flag */
	global_packet->ttl = Compute_Packet_TTL_With_VehicleTravelTime(param, DVQ);

	/* construct multicast tree information with target points to keep the copy of this packet */
	Create_Multicast_Forwarding_Table_For_GlobalPacket_AlongWith_MulticastTreeCost(param, ESQ, FTPQ, global_packet);

#if FEASIBLE_TARGET_POINT_QUEUE_DISPLAY_FLAG
	/* display the feasible target point queue for the multicast tree */
	Show_FeasibleTargetPoints_For_TargetPointQueue(current_time, FTPQ);
#endif

#if EDGE_SET_QUEUE_DISPLAY_FLAG
	/* display the edge set queue for the multicast tree */
	Show_EdgeSetQueue_For_MulticastTree(current_time, ESQ);
#endif

#if DESTINATION_VEHICLES_TARGET_POINT_QUEUES_DISPLAY_FLAG
	/* display the target point queues for the destination vehicles */
	Show_TargetPointInformation_For_All_DestinationVehicles(current_time, DVQ);
#endif
}

void ST_Construct_Multicast_Tree_Based_On_MST_Feasible_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet)
{ /* construct a multicast tree according to the MST-feasible-target-point-selection algorithm */

#if 0
	if(current_time > 7463 && global_packet->id == 194)
		printf("[%.2f] ST_Construct_Multicast_Tree_Based_On_MST_Feasible_Target_Point_Selection():\n packet_id=%d is traced\n", (float)current_time, global_packet->id);
#endif

	/* compute feasible target point sets for all the destination vehicles in destination vehicle queue DVQ */
	Compute_Feasible_TargetPoint_Sets_For_All_DestinationVehicles(param, current_time, AP_vertex, DVQ);
			
	/* [////////////////////////////////////////////////////////////////////////////////////////////////
	/* merge the feasible target point sets in DVQ into one target point set called FTPQ */
	Merge_DestinationVehicle_FeasibleTargetPointSets_By_TMA_TargetPoint_Selection(param, DVQ, FTPQ);
	////////////////////////////////////////////////////////////////////////////////////////////////] */

#if TARGET_POINT_ANYCAST_SET_TRACE_FLAG
	/* print out target points in the multicast tree */
	printf("pid=%d<<Before Filtering:\n", global_packet->id);
	PrintOut_TargetPoints_In_MulticastTree(current_time, FTPQ);
#endif	

	/* construct a Steiner Tree with the target point set FTPS */
	ST_Get_GreedyMulticastTree(param, current_time, atoi(AP_vertex), FTPQ, DVQ);

	/* filter out redundant target points from Minimum Steiner Tree T consisting of anycast sets of destination vehicles with Minimum Spanning Tree Algorithm */
	ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree_With_MinimumSpanningTreeAlgorithm(param, current_time, AP_vertex, DVQ, FTPQ, ESQ);

	/* compute Packet TTL with the longest vehicle travel time from DVQ according to param->communication_packet_ttl_override_flag */
	global_packet->ttl = Compute_Packet_TTL_With_VehicleTravelTime(param, DVQ);

	/* construct multicast tree information with target points to keep the copy of this packet */
	Create_Multicast_Forwarding_Table_For_GlobalPacket_AlongWith_MulticastTreeCost(param, ESQ, FTPQ, global_packet);

#if FEASIBLE_TARGET_POINT_QUEUE_DISPLAY_FLAG
	/* display the feasible target point queue for the multicast tree */
	Show_FeasibleTargetPoints_For_TargetPointQueue(current_time, FTPQ);
#endif

#if EDGE_SET_QUEUE_DISPLAY_FLAG
	/* display the edge set queue for the multicast tree */
	Show_EdgeSetQueue_For_MulticastTree(current_time, ESQ);
#endif

#if DESTINATION_VEHICLES_TARGET_POINT_QUEUES_DISPLAY_FLAG
	/* display the target point queues for the destination vehicles */
	Show_TargetPointInformation_For_All_DestinationVehicles(current_time, DVQ);
#endif
}

void ST_Construct_Multicast_Tree_Based_On_Minimum_Delay_Feasible_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet)
{ /* construct a multicast tree according to the Minimum-delay-feasible-target-point-selection algorithm */

	/* compute feasible target point sets for all the destination vehicles in destination vehicle queue DVQ */
	Compute_Feasible_TargetPoint_Sets_For_All_DestinationVehicles(param, current_time, AP_vertex, DVQ);
			
	/* [////////////////////////////////////////////////////////////////////////////////////////////////
	/* merge the feasible target point sets in DVQ into one target point set called FTPQ */
	Merge_DestinationVehicle_FeasibleTargetPointSets_By_Minimum_Delay_TargetPoint_Selection(param, DVQ, FTPQ);
	////////////////////////////////////////////////////////////////////////////////////////////////] */

#if TARGET_POINT_ANYCAST_SET_TRACE_FLAG
	/* print out target points in the multicast tree */
	printf("pid=%d<<Before Filtering:\n", global_packet->id);
	PrintOut_TargetPoints_In_MulticastTree(current_time, FTPQ);
#endif	

	/* construct a Steiner Tree with the target point set FTPQ */
	ST_Get_GreedyMulticastTree(param, current_time, atoi(AP_vertex), FTPQ, DVQ);

	/* filter out redundant target points from Minimum Steiner Tree T consisting of anycast sets of destination vehicles */
	ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree(param, current_time, DVQ, FTPQ, ESQ);

	/* compute Packet TTL with the longest vehicle travel time from DVQ according to param->communication_packet_ttl_override_flag */
	global_packet->ttl = Compute_Packet_TTL_With_VehicleTravelTime(param, DVQ);

	/* construct multicast tree information with target points to keep the copy of this packet */
	Create_Multicast_Forwarding_Table_For_GlobalPacket_AlongWith_MulticastTreeCost(param, ESQ, FTPQ, global_packet);

#if FEASIBLE_TARGET_POINT_QUEUE_DISPLAY_FLAG
	/* display the feasible target point queue for the multicast tree */
	Show_FeasibleTargetPoints_For_TargetPointQueue(current_time, FTPQ);
#endif

#if EDGE_SET_QUEUE_DISPLAY_FLAG
	/* display the edge set queue for the multicast tree */
	Show_EdgeSetQueue_For_MulticastTree(current_time, ESQ);
#endif

#if DESTINATION_VEHICLES_TARGET_POINT_QUEUES_DISPLAY_FLAG
	/* display the target point queues for the destination vehicles */
	Show_TargetPointInformation_For_All_DestinationVehicles(current_time, DVQ);
#endif
}

void ST_Construct_Multicast_Tree_Based_On_Minimum_Cost_Feasible_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet)
{ /* construct a multicast tree according to the Minimum-cost-feasible-target-point-selection algorithm */

#if 0
	if(current_time > 7249 && global_packet->id == 32)
		printf("[%.2f] ST_Construct_Multicast_Tree_Based_On_Minimum_Cost_Feasible_Target_Point_Selection():\n packet_id=%d is traced\n", (float)current_time, global_packet->id);
#endif

	/* compute feasible target point sets for all the destination vehicles in destination vehicle queue DVQ */
	Compute_Feasible_TargetPoint_Sets_For_All_DestinationVehicles(param, current_time, AP_vertex, DVQ);
			
	/* [////////////////////////////////////////////////////////////////////////////////////////////////
	/* merge the feasible target point sets in DVQ into one target point set called FTPQ */
	Merge_DestinationVehicle_FeasibleTargetPointSets_By_Minimum_Cost_TargetPoint_Selection(param, DVQ, FTPQ);
	////////////////////////////////////////////////////////////////////////////////////////////////] */

#if TARGET_POINT_ANYCAST_SET_TRACE_FLAG
	/* print out target points in the multicast tree */
	printf("pid=%d<<Before Filtering:\n", global_packet->id);
	PrintOut_TargetPoints_In_MulticastTree(current_time, FTPQ);
#endif	

	/* construct a Steiner Tree with the target point set FTPS */
	ST_Get_GreedyMulticastTree(param, current_time, atoi(AP_vertex), FTPQ, DVQ);

	/* filter out redundant target points from Minimum Steiner Tree T consisting of anycast sets of destination vehicles */
	ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree(param, current_time, DVQ, FTPQ, ESQ);

	/* compute Packet TTL with the longest vehicle travel time from DVQ according to param->communication_packet_ttl_override_flag */
	global_packet->ttl = Compute_Packet_TTL_With_VehicleTravelTime(param, DVQ);

	/* construct multicast tree information with target points to keep the copy of this packet */
	Create_Multicast_Forwarding_Table_For_GlobalPacket_AlongWith_MulticastTreeCost(param, ESQ, FTPQ, global_packet);

#if FEASIBLE_TARGET_POINT_QUEUE_DISPLAY_FLAG
	/* display the feasible target point queue for the multicast tree */
	Show_FeasibleTargetPoints_For_TargetPointQueue(current_time, FTPQ);
#endif

#if EDGE_SET_QUEUE_DISPLAY_FLAG
	/* display the edge set queue for the multicast tree */
	Show_EdgeSetQueue_For_MulticastTree(current_time, ESQ);
#endif

#if DESTINATION_VEHICLES_TARGET_POINT_QUEUES_DISPLAY_FLAG
	/* display the target point queues for the destination vehicles */
	Show_TargetPointInformation_For_All_DestinationVehicles(current_time, DVQ);
#endif
}

void ST_Construct_Multicast_Tree_Based_On_Simple_AST_Feasible_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet)
{ /* construct a multicast tree according to the Simple Anycast-set Spanning Tree (SAST) Feasible-target-point-selection algorithm */
	double beta = 0; //delay bound that is the max min delay among the delivery delays for all the anycast sets
	int max_min_delay_anycast_set = 0; //anycast set id for max min delay
	int max_min_delay_target_point = 0; //target point id for max min delay

#if 0
	if(current_time > 7463 && global_packet->id == 194)
		printf("[%.2f] ST_Construct_Multicast_Tree_Based_On_MST_Feasible_Target_Point_Selection():\n packet_id=%d is traced\n", (float)current_time, global_packet->id);
#endif

	/* compute feasible target point sets for all the destination vehicles in destination vehicle queue DVQ */
	Compute_Feasible_TargetPoint_Sets_For_All_DestinationVehicles(param, current_time, AP_vertex, DVQ);

	/* find the maximum minimum delay from AP to Anycast Sets as the delay bound Beta */
	beta = Find_MaxMin_DeliveryDelay_For_AnycastSets(DVQ, &max_min_delay_anycast_set, &max_min_delay_target_point);
	if(beta == -1)
	{
	  printf("ST_Construct_Multicast_Tree_Based_On_AST_Feasible_Target_Point_Selection(): Error: Find_MaxMin_DeliveryDelay_For_AnycastSets returns a negative delay (i.e., beta=%2f)\n", beta);
	  exit(1);
	}

	/* filter out target points not satisfying the delay bound beta */
	ST_FilterOut_InfeasibleTargetPoints_From_AllAnycastSets(DVQ, beta);

	/* filter out redundant target points from All Anycast Sets for destination vehicles with Minimum Spanning Tree Algorithm */
	ST_FilterOut_RedundantTargetPoints_From_AllAnycastSets_With_MinimumSpanningTreeAlgorithm_And_DeliveryDelayBound(param, current_time, AP_vertex, DVQ, beta, FTPQ);

	/* merge the feasible target point sets in DVQ into one target point set called FTPQ */
	//Merge_DestinationVehicle_FeasibleTargetPointSets_By_TMA_TargetPoint_Selection(param, DVQ, FTPQ);

	/* construct a spanning tree T covering all the anycast sets by selecting one target point for each 
	 * anycast set */
	//ST_Construct_DirectedSpanningTree(param, current_time, AP_vertex, DVQ, FTPQ, ESQ);

	/* search a better Delay-Constrained Minimum Steiner Tree T' with the initial tree T */

#if TARGET_POINT_ANYCAST_SET_TRACE_FLAG
	/* print out target points in the multicast tree */
	printf("pid=%d<<Before Filtering:\n", global_packet->id);
	PrintOut_TargetPoints_In_MulticastTree(current_time, FTPQ);
#endif	

	/* construct a Steiner Tree with the target point set FTPS */
	ST_Get_GreedyMulticastTree_With_DelayConstraint(param, current_time, atoi(AP_vertex), FTPQ, DVQ, beta);

	/* construct edge set queue ESQ with Minimum Steiner Tree (MST) based on FTPQ */
	GO_Construct_EdgeQueue(param, current_time, FTPQ, ESQ);

	/* compute Packet TTL with the longest vehicle travel time from DVQ according to param->communication_packet_ttl_override_flag */
	global_packet->ttl = Compute_Packet_TTL_With_VehicleTravelTime(param, DVQ);

	/* construct multicast tree information with target points to keep the copy of this packet */
	Create_Multicast_Forwarding_Table_For_GlobalPacket_AlongWith_MulticastTreeCost(param, ESQ, FTPQ, global_packet);

#if FEASIBLE_TARGET_POINT_QUEUE_DISPLAY_FLAG
	/* display the feasible target point queue for the multicast tree */
	Show_FeasibleTargetPoints_For_TargetPointQueue(current_time, FTPQ);
#endif

#if EDGE_SET_QUEUE_DISPLAY_FLAG
	/* display the edge set queue for the multicast tree */
	Show_EdgeSetQueue_For_MulticastTree(current_time, ESQ);
#endif

#if DESTINATION_VEHICLES_TARGET_POINT_QUEUES_DISPLAY_FLAG
	/* display the target point queues for the destination vehicles */
	Show_TargetPointInformation_For_All_DestinationVehicles(current_time, DVQ);
#endif
}

void ST_Construct_Multicast_Tree_Based_On_AST_Feasible_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet)
{ /* construct a multicast tree according to the Anycast-set Spanning Tree (AST) Feasible-target-point-selection algorithm */
	double beta = 0; //delay bound that is the max min delay among the delivery delays for all the anycast sets
	int max_min_delay_anycast_set = 0; //anycast set id for max min delay
	int max_min_delay_target_point = 0; //target point id for max min delay

#if 0
	if(current_time > 7463 && global_packet->id == 194)
		printf("[%.2f] ST_Construct_Multicast_Tree_Based_On_MST_Feasible_Target_Point_Selection():\n packet_id=%d is traced\n", (float)current_time, global_packet->id);
#endif

	/* compute feasible target point sets for all the destination vehicles in destination vehicle queue DVQ */
	Compute_Feasible_TargetPoint_Sets_For_All_DestinationVehicles(param, current_time, AP_vertex, DVQ);

	/* find the maximum minimum delay from AP to Anycast Sets as the delay bound Beta */
	beta = Find_MaxMin_DeliveryDelay_For_AnycastSets(DVQ, &max_min_delay_anycast_set, &max_min_delay_target_point);
	if(beta == -1)
	{
	  printf("ST_Construct_Multicast_Tree_Based_On_AST_Feasible_Target_Point_Selection(): Error: Find_MaxMin_DeliveryDelay_For_AnycastSets returns a negative delay (i.e., beta=%2f)\n", beta);
	  exit(1);
	}

	/* filter out target points not satisfying the delay bound beta */
	ST_FilterOut_InfeasibleTargetPoints_From_AllAnycastSets(DVQ, beta);

	/* merge the feasible target point sets in DVQ into one target point set called FTPQ */
	Merge_DestinationVehicle_FeasibleTargetPointSets_By_TMA_TargetPoint_Selection(param, DVQ, FTPQ);

	/* construct a spanning tree T covering all the anycast sets by selecting one target point for each 
	 * anycast set */
	//ST_Construct_DirectedSpanningTree(param, current_time, AP_vertex, DVQ, FTPQ, ESQ);

	/* search a better Delay-Constrained Minimum Steiner Tree T' with the initial tree T */

#if TARGET_POINT_ANYCAST_SET_TRACE_FLAG
	/* print out target points in the multicast tree */
	printf("pid=%d<<Before Filtering:\n", global_packet->id);
	PrintOut_TargetPoints_In_MulticastTree(current_time, FTPQ);
#endif	

	/* construct a Steiner Tree with the target point set FTPS */
	ST_Get_GreedyMulticastTree_With_DelayConstraint(param, current_time, atoi(AP_vertex), FTPQ, DVQ, beta);

	/* filter out redundant target points from Minimum Steiner Tree T consisting of anycast sets of destination vehicles with Minimum Spanning Tree Algorithm */
	ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree_With_MinimumSpanningTreeAlgorithm_And_DeliveryDelayBound(param, current_time, AP_vertex, DVQ, beta, FTPQ, ESQ);
	//ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree_With_MinimumSpanningTreeAlgorithm(param, current_time, AP_vertex, DVQ, FTPQ, ESQ);

	/* construct edge set queue ESQ with Minimum Steiner Tree (MST) based on FTPQ */

	/* compute Packet TTL with the longest vehicle travel time from DVQ according to param->communication_packet_ttl_override_flag */
	global_packet->ttl = Compute_Packet_TTL_With_VehicleTravelTime(param, DVQ);

	/* construct multicast tree information with target points to keep the copy of this packet */
	Create_Multicast_Forwarding_Table_For_GlobalPacket_AlongWith_MulticastTreeCost(param, ESQ, FTPQ, global_packet);

#if FEASIBLE_TARGET_POINT_QUEUE_DISPLAY_FLAG
	/* display the feasible target point queue for the multicast tree */
	Show_FeasibleTargetPoints_For_TargetPointQueue(current_time, FTPQ);
#endif

#if EDGE_SET_QUEUE_DISPLAY_FLAG
	/* display the edge set queue for the multicast tree */
	Show_EdgeSetQueue_For_MulticastTree(current_time, ESQ);
#endif

#if DESTINATION_VEHICLES_TARGET_POINT_QUEUES_DISPLAY_FLAG
	/* display the target point queues for the destination vehicles */
	Show_TargetPointInformation_For_All_DestinationVehicles(current_time, DVQ);
#endif
}

void ST_Construct_Multicast_Tree_Based_On_Simple_Random_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet)
{ /* construct a multicast tree according to the Simple-Random-target-point-selection algorithm */

#if 0 /* [ */
	if(current_time > 10000 && global_packet->id == 620)
		printf("[%.2f] ST_Construct_Multicast_Tree_Based_On_Simple_Random_Target_Point_Selection():\n packet_id=%d is traced\n", (float)current_time, global_packet->id);
#endif /* ] */

	/* compute target point sets for all the destination vehicles in destination vehicle queue DVQ */
	Compute_TargetPoint_Sets_For_All_DestinationVehicles(param, current_time, AP_vertex, DVQ);
			
	/* [////////////////////////////////////////////////////////////////////////////////////////////////
	/* merge the target point sets in DVQ into one target point set called FTPQ */
	Merge_DestinationVehicle_TargetPointSets_By_Random_TargetPoint_Selection(param, DVQ, FTPQ);
	////////////////////////////////////////////////////////////////////////////////////////////////] */

#if TARGET_POINT_ANYCAST_SET_TRACE_FLAG
	/* print out target points in the multicast tree */
	printf("pid=%d<<Before Filtering:\n", global_packet->id);
	PrintOut_TargetPoints_In_MulticastTree(current_time, FTPQ);
#endif	
#if 0 /* [ */
	/* construct a Steiner Tree with the target point set FTPS */
	ST_Get_GreedyMulticastTree(param, current_time, atoi(AP_vertex), FTPQ, DVQ);

	/* filter out redundant target points from Minimum Steiner Tree T consisting of anycast sets of destination vehicles */
	ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree(param, current_time, DVQ, FTPQ, ESQ);
#endif /* ] */

#if 1 /* [[ */
	/* construct the Shortest Path Tree with the target point set FTPS */
	ST_Get_ShortestPathTree(param, current_time, atoi(AP_vertex), FTPQ, DVQ);

	/* filter out redundant target points from the Shortest Path Tree T consisting of anycast sets of destination vehicles */
	ST_FilterOut_RedundantTargetPoints_From_ShortestPathTree(param, current_time, DVQ, FTPQ, ESQ);
#endif /* ]] */

	/* compute Packet TTL with the longest vehicle travel time from DVQ according to param->communication_packet_ttl_override_flag */
	global_packet->ttl = Compute_Packet_TTL_With_VehicleTravelTime(param, DVQ);

	/* construct multicast tree information with target points to keep the copy of this packet */
	Create_Multicast_Forwarding_Table_For_GlobalPacket_AlongWith_MulticastTreeCost(param, ESQ, FTPQ, global_packet);

#if FEASIBLE_TARGET_POINT_QUEUE_DISPLAY_FLAG
	/* display the feasible target point queue for the multicast tree */
	Show_FeasibleTargetPoints_For_TargetPointQueue(current_time, FTPQ);
#endif

#if EDGE_SET_QUEUE_DISPLAY_FLAG
	/* display the edge set queue for the multicast tree */
	Show_EdgeSetQueue_For_MulticastTree(current_time, ESQ);
#endif

#if DESTINATION_VEHICLES_TARGET_POINT_QUEUES_DISPLAY_FLAG
	/* display the target point queues for the destination vehicles */
	Show_TargetPointInformation_For_All_DestinationVehicles(current_time, DVQ);
#endif
}

void ST_Construct_Multicast_Tree_Based_On_Flooding_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet)
{ /* construct a multicast tree according to the Flooding-target-point-selection algorithm */

	/* compute target point sets for all the destination vehicles in destination vehicle queue DVQ */
#if 1 /* [ */
	Compute_TargetPoint_Sets_For_All_DestinationVehicles(param, current_time, AP_vertex, DVQ);
#endif /* ] */
			
	/* [////////////////////////////////////////////////////////////////////////////////////////////////
	/* make one target point set called FTPQ with all intersections in the road network graph */
	Make_TargetPointSet_By_Flooding_TargetPoint_Selection(param, DVQ, FTPQ);
	////////////////////////////////////////////////////////////////////////////////////////////////] */

	/* compute Packet TTL with the longest vehicle travel time from DVQ according to param->communication_packet_ttl_override_flag */
	global_packet->ttl = Compute_Packet_TTL_With_VehicleTravelTime(param, DVQ);

	/* construct broadcast graph information with road intersections to deliver one copy of this packet to each intersection in the road network */
	Create_Multicast_Forwarding_Table_For_GlobalPacket_Flooding(param, FTPQ, global_packet);

#if FEASIBLE_TARGET_POINT_QUEUE_DISPLAY_FLAG
	/* display the feasible target point queue for the multicast tree */
	Show_FeasibleTargetPoints_For_TargetPointQueue(current_time, FTPQ);
#endif

#if EDGE_SET_QUEUE_DISPLAY_FLAG
	/* display the edge set queue for the multicast tree */
	Show_EdgeSetQueue_For_MulticastTree(current_time, ESQ);
#endif

#if DESTINATION_VEHICLES_TARGET_POINT_QUEUES_DISPLAY_FLAG
	/* display the target point queues for the destination vehicles */
	Show_TargetPointInformation_For_All_DestinationVehicles(current_time, DVQ);
#endif
}

void ST_Construct_Multicast_Tree_Based_On_Last_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet)
{ /* construct a multicast tree according to the Last-target-point-selection algorithm */

	/* compute target point sets for all the destination vehicles in destination vehicle queue DVQ */
	Compute_TargetPoint_Sets_For_All_DestinationVehicles(param, current_time, AP_vertex, DVQ);
			
	/* [////////////////////////////////////////////////////////////////////////////////////////////////
	/* merge the target point sets in DVQ into one target point set called FTPQ */
	Merge_DestinationVehicle_TargetPointSets_By_Last_TargetPoint_Selection(param, DVQ, FTPQ);
	////////////////////////////////////////////////////////////////////////////////////////////////] */

#if TARGET_POINT_ANYCAST_SET_TRACE_FLAG
	/* print out target points in the multicast tree */
	printf("pid=%d<<Before Filtering:\n", global_packet->id);
	PrintOut_TargetPoints_In_MulticastTree(current_time, FTPQ);
#endif	
#if 0 /* [ */
	/* construct a Steiner Tree with the target point set FTPS */
	ST_Get_GreedyMulticastTree(param, current_time, atoi(AP_vertex), FTPQ, DVQ);

	/* filter out redundant target points from Minimum Steiner Tree T consisting of anycast sets of destination vehicles */
	ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree(param, current_time, DVQ, FTPQ, ESQ);
#endif /* ] */

#if 1 /* [[ */
	/* construct the Shortest Path Tree with the target point set FTPS */
	ST_Get_ShortestPathTree(param, current_time, atoi(AP_vertex), FTPQ, DVQ);

	/* filter out redundant target points from the Shortest Path Tree T consisting of anycast sets of destination vehicles */
	ST_FilterOut_RedundantTargetPoints_From_ShortestPathTree(param, current_time, DVQ, FTPQ, ESQ);
#endif /* ]] */

	/* compute Packet TTL with the longest vehicle travel time from DVQ according to param->communication_packet_ttl_override_flag */
	global_packet->ttl = Compute_Packet_TTL_With_VehicleTravelTime(param, DVQ);

	/* construct multicast tree information with target points to keep the copy of this packet */
	Create_Multicast_Forwarding_Table_For_GlobalPacket_AlongWith_MulticastTreeCost(param, ESQ, FTPQ, global_packet);

#if FEASIBLE_TARGET_POINT_QUEUE_DISPLAY_FLAG
	/* display the feasible target point queue for the multicast tree */
	Show_FeasibleTargetPoints_For_TargetPointQueue(current_time, FTPQ);
#endif

#if EDGE_SET_QUEUE_DISPLAY_FLAG
	/* display the edge set queue for the multicast tree */
	Show_EdgeSetQueue_For_MulticastTree(current_time, ESQ);
#endif

#if DESTINATION_VEHICLES_TARGET_POINT_QUEUES_DISPLAY_FLAG
	/* display the target point queues for the destination vehicles */
	Show_TargetPointInformation_For_All_DestinationVehicles(current_time, DVQ);
#endif
}

void ST_Construct_Multicast_Tree_Based_On_Simple_Minimum_Delay_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet)
{ /* construct a multicast tree according to the Simple-Minimum-delay-target-point-selection algorithm */

	/* compute target point sets for all the destination vehicles in destination vehicle queue DVQ */
	Compute_TargetPoint_Sets_For_All_DestinationVehicles(param, current_time, AP_vertex, DVQ);
			
	/* [////////////////////////////////////////////////////////////////////////////////////////////////
	/* merge the feasible target point sets in DVQ into one target point set called FTPQ */
	Merge_DestinationVehicle_TargetPointSets_By_Minimum_Delay_TargetPoint_Selection(param, DVQ, FTPQ);
	////////////////////////////////////////////////////////////////////////////////////////////////] */

#if TARGET_POINT_ANYCAST_SET_TRACE_FLAG
	/* print out target points in the multicast tree */
	printf("pid=%d<<Before Filtering:\n", global_packet->id);
	PrintOut_TargetPoints_In_MulticastTree(current_time, FTPQ);
#endif	

	/* construct a Steiner Tree with the target point set FTPS */
	ST_Get_GreedyMulticastTree(param, current_time, atoi(AP_vertex), FTPQ, DVQ);

	/* filter out redundant target points from Minimum Steiner Tree T consisting of anycast sets of destination vehicles */
	ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree(param, current_time, DVQ, FTPQ, ESQ);

	/* compute Packet TTL with the longest vehicle travel time from DVQ according to param->communication_packet_ttl_override_flag */
	global_packet->ttl = Compute_Packet_TTL_With_VehicleTravelTime(param, DVQ);

	/* construct multicast tree information with target points to keep the copy of this packet */
	Create_Multicast_Forwarding_Table_For_GlobalPacket_AlongWith_MulticastTreeCost(param, ESQ, FTPQ, global_packet);

#if FEASIBLE_TARGET_POINT_QUEUE_DISPLAY_FLAG
	/* display the feasible target point queue for the multicast tree */
	Show_FeasibleTargetPoints_For_TargetPointQueue(current_time, FTPQ);
#endif

#if EDGE_SET_QUEUE_DISPLAY_FLAG
	/* display the edge set queue for the multicast tree */
	Show_EdgeSetQueue_For_MulticastTree(current_time, ESQ);
#endif

#if DESTINATION_VEHICLES_TARGET_POINT_QUEUES_DISPLAY_FLAG
	/* display the target point queues for the destination vehicles */
	Show_TargetPointInformation_For_All_DestinationVehicles(current_time, DVQ);
#endif
}

void ST_Construct_Multicast_Tree_Based_On_Simple_Minimum_Cost_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet)
{ /* construct a multicast tree according to the Simple-Minimum-cost-target-point-selection algorithm */

#if 0
	if(current_time > 7249 && global_packet->id == 32)
		printf("[%.2f] ST_Construct_Multicast_Tree_Based_On_Minimum_Cost_Feasible_Target_Point_Selection():\n packet_id=%d is traced\n", (float)current_time, global_packet->id);
#endif

	/* compute feasible target point sets for all the destination vehicles in destination vehicle queue DVQ */
	Compute_Feasible_TargetPoint_Sets_For_All_DestinationVehicles(param, current_time, AP_vertex, DVQ);
			
	/* [////////////////////////////////////////////////////////////////////////////////////////////////
	/* merge the feasible target point sets in DVQ into one target point set called FTPQ */
	Merge_DestinationVehicle_FeasibleTargetPointSets_By_Minimum_Cost_TargetPoint_Selection(param, DVQ, FTPQ);
	////////////////////////////////////////////////////////////////////////////////////////////////] */

#if TARGET_POINT_ANYCAST_SET_TRACE_FLAG
	/* print out target points in the multicast tree */
	printf("pid=%d<<Before Filtering:\n", global_packet->id);
	PrintOut_TargetPoints_In_MulticastTree(current_time, FTPQ);
#endif	

	/* construct a Steiner Tree with the target point set FTPS */
	ST_Get_GreedyMulticastTree(param, current_time, atoi(AP_vertex), FTPQ, DVQ);

	/* filter out redundant target points from Minimum Steiner Tree T consisting of anycast sets of destination vehicles */
	ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree(param, current_time, DVQ, FTPQ, ESQ);

	/* compute Packet TTL with the longest vehicle travel time from DVQ according to param->communication_packet_ttl_override_flag */
	global_packet->ttl = Compute_Packet_TTL_With_VehicleTravelTime(param, DVQ);

	/* construct multicast tree information with target points to keep the copy of this packet */
	Create_Multicast_Forwarding_Table_For_GlobalPacket_AlongWith_MulticastTreeCost(param, ESQ, FTPQ, global_packet);

#if FEASIBLE_TARGET_POINT_QUEUE_DISPLAY_FLAG
	/* display the feasible target point queue for the multicast tree */
	Show_FeasibleTargetPoints_For_TargetPointQueue(current_time, FTPQ);
#endif

#if EDGE_SET_QUEUE_DISPLAY_FLAG
	/* display the edge set queue for the multicast tree */
	Show_EdgeSetQueue_For_MulticastTree(current_time, ESQ);
#endif

#if DESTINATION_VEHICLES_TARGET_POINT_QUEUES_DISPLAY_FLAG
	/* display the target point queues for the destination vehicles */
	Show_TargetPointInformation_For_All_DestinationVehicles(current_time, DVQ);
#endif
}

void ST_Construct_Multicast_Tree_Based_On_SPT_Feasible_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet)
{ /* construct a multicast tree according to the Shortest Path Tree (SPT) Feasible-target-point-selection algorithm such that the shortest path is the shortest-delay path */
	/* compute feasible target point sets for all the destination vehicles in destination vehicle queue DVQ */
	Compute_Feasible_TargetPoint_Sets_For_All_DestinationVehicles(param, current_time, AP_vertex, DVQ);

	/* [////////////////////////////////////////////////////////////////////////////////////////////////
	/* merge the feasible target point sets in DVQ into one target point set called FTPQ */
	Merge_DestinationVehicle_FeasibleTargetPointSets_By_Minimum_Delay_TargetPoint_Selection(param, DVQ, FTPQ);
	////////////////////////////////////////////////////////////////////////////////////////////////] */

#if TARGET_POINT_ANYCAST_SET_TRACE_FLAG
	/* print out target points in the multicast tree */
	printf("pid=%d<<Before Filtering:\n", global_packet->id);
	PrintOut_TargetPoints_In_MulticastTree(current_time, FTPQ);
#endif	

	/* construct the Shortest Path Tree with the target point set FTPS */
	ST_Get_ShortestPathTree(param, current_time, atoi(AP_vertex), FTPQ, DVQ);

	/* filter out redundant target points from the Shortest Path Tree T consisting of anycast sets of destination vehicles */
	ST_FilterOut_RedundantTargetPoints_From_ShortestPathTree(param, current_time, DVQ, FTPQ, ESQ);

	/* compute Packet TTL with the longest vehicle travel time from DVQ according to param->communication_packet_ttl_override_flag */
	global_packet->ttl = Compute_Packet_TTL_With_VehicleTravelTime(param, DVQ);

	/* construct multicast tree information with target points to keep the copy of this packet */
	Create_Multicast_Forwarding_Table_For_GlobalPacket_AlongWith_MulticastTreeCost(param, ESQ, FTPQ, global_packet);

#if FEASIBLE_TARGET_POINT_QUEUE_DISPLAY_FLAG
	/* display the feasible target point queue for the multicast tree */
	Show_FeasibleTargetPoints_For_TargetPointQueue(current_time, FTPQ);
#endif

#if EDGE_SET_QUEUE_DISPLAY_FLAG
	/* display the edge set queue for the multicast tree */
	Show_EdgeSetQueue_For_MulticastTree(current_time, ESQ);
#endif

#if DESTINATION_VEHICLES_TARGET_POINT_QUEUES_DISPLAY_FLAG
	/* display the target point queues for the destination vehicles */
	Show_TargetPointInformation_For_All_DestinationVehicles(current_time, DVQ);
#endif
}

void ST_Construct_Multicast_Tree_Based_On_Simple_SPT_Target_Point_Selection(parameter_t *param, double current_time, char *AP_vertex, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet)
{ /* construct a multicast tree according to the Simple Shortest Path Tree (SSPT) target-point-selection algorithm such that the shortest path is the shortest-delay path */
	/* compute target point sets for all the destination vehicles in destination vehicle queue DVQ */
	Compute_TargetPoint_Sets_For_All_DestinationVehicles(param, current_time, AP_vertex, DVQ);

	/* [////////////////////////////////////////////////////////////////////////////////////////////////
	/* merge the target point sets in DVQ into one target point set called FTPQ */
	Merge_DestinationVehicle_TargetPointSets_By_Minimum_Delay_TargetPoint_Selection(param, DVQ, FTPQ);
	////////////////////////////////////////////////////////////////////////////////////////////////] */

#if TARGET_POINT_ANYCAST_SET_TRACE_FLAG
	/* print out target points in the multicast tree */
	printf("pid=%d<<Before Filtering:\n", global_packet->id);
	PrintOut_TargetPoints_In_MulticastTree(current_time, FTPQ);
#endif	

	/* construct the Shortest Path Tree with the target point set FTPS */
	ST_Get_ShortestPathTree(param, current_time, atoi(AP_vertex), FTPQ, DVQ);

	/* filter out redundant target points from the Shortest Path Tree T consisting of anycast sets of destination vehicles */
	ST_FilterOut_RedundantTargetPoints_From_ShortestPathTree(param, current_time, DVQ, FTPQ, ESQ);

	/* compute Packet TTL with the longest vehicle travel time from DVQ according to param->communication_packet_ttl_override_flag */
	global_packet->ttl = Compute_Packet_TTL_With_VehicleTravelTime(param, DVQ);

	/* construct multicast tree information with target points to keep the copy of this packet */
	Create_Multicast_Forwarding_Table_For_GlobalPacket_AlongWith_MulticastTreeCost(param, ESQ, FTPQ, global_packet);

#if FEASIBLE_TARGET_POINT_QUEUE_DISPLAY_FLAG
	/* display the feasible target point queue for the multicast tree */
	Show_FeasibleTargetPoints_For_TargetPointQueue(current_time, FTPQ);
#endif

#if EDGE_SET_QUEUE_DISPLAY_FLAG
	/* display the edge set queue for the multicast tree */
	Show_EdgeSetQueue_For_MulticastTree(current_time, ESQ);
#endif

#if DESTINATION_VEHICLES_TARGET_POINT_QUEUES_DISPLAY_FLAG
	/* display the target point queues for the destination vehicles */
	Show_TargetPointInformation_For_All_DestinationVehicles(current_time, DVQ);
#endif
}

/** Construction of Multicast Forest in Multiple AP Scenarios */
void ST_Construct_Multicast_Forest_Based_On_Minimum_Cost_Feasible_Target_Point_Selection(parameter_t *param, double current_time, access_point_queue_t *APQ, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ, global_packet_queue_node_t *global_packet)
{ /* construct a multicast forest according to the Minimum-cost-feasible-target-point-selection algorithm */
	access_point_queue_node_t *pAP = NULL; //pointer to an access point queue node
	int i = 0; //loop-index

#if 0
	if(current_time > 7200 && global_packet->id == 0)
		printf("[%.2f] ST_Construct_Multicast_Forest_Based_On_Minimum_Cost_Feasible_Target_Point_Selection():\n packet_id=%d is traced\n", (float)current_time, global_packet->id);
#endif

	/* compute feasible target point sets for all the destination vehicles in destination vehicle queue DVQ per AP */
	pAP = &(APQ->head);
	for(i = 0; i < APQ->size; i++)
	{
		pAP = pAP->next;
		
		/* compute the feasible target point sets for all destination vehicles for pAP->vertex */
		Compute_Feasible_TargetPoint_Sets_For_All_DestinationVehicles(param, current_time, pAP->vertex, &(pAP->DVQ));
	}
			

/************/
#if 0 /* < */

	/* [////////////////////////////////////////////////////////////////////////////////////////////////
	/* merge the feasible target point sets in DVQ into one target point set called FTPQ */
	Merge_DestinationVehicle_FeasibleTargetPointSets_By_Minimum_Cost_TargetPoint_Selection(param, DVQ, FTPQ);
	////////////////////////////////////////////////////////////////////////////////////////////////] */

#if TARGET_POINT_ANYCAST_SET_TRACE_FLAG
	/* print out target points in the multicast tree */
	printf("pid=%d<<Before Filtering:\n", global_packet->id);
	PrintOut_TargetPoints_In_MulticastTree(current_time, FTPQ);
#endif	

	/* construct a Steiner Tree with the target point set FTPS */
	ST_Get_GreedyMulticastTree(param, current_time, atoi(AP_vertex), FTPQ, DVQ);

	/* filter out redundant target points from Minimum Steiner Tree T consisting of anycast sets of destination vehicles */
	ST_FilterOut_RedundantTargetPoints_From_MinimumSteinerTree(param, current_time, DVQ, FTPQ, ESQ);

	/* compute Packet TTL with the longest vehicle travel time from DVQ according to param->communication_packet_ttl_override_flag */
	global_packet->ttl = Compute_Packet_TTL_With_VehicleTravelTime(param, DVQ);

	/* construct multicast tree information with target points to keep the copy of this packet */
	Create_Multicast_Forwarding_Table_For_GlobalPacket_AlongWith_MulticastTreeCost(param, ESQ, FTPQ, global_packet);

#endif /* > */
/************/

#if FEASIBLE_TARGET_POINT_QUEUE_DISPLAY_FLAG
	/* display the feasible target point queue for the multicast tree */
	Show_FeasibleTargetPoints_For_TargetPointQueue(current_time, FTPQ);
#endif

#if EDGE_SET_QUEUE_DISPLAY_FLAG
	/* display the edge set queue for the multicast tree */
	Show_EdgeSetQueue_For_MulticastTree(current_time, ESQ);
#endif

#if DESTINATION_VEHICLES_TARGET_POINT_QUEUES_DISPLAY_FLAG
	/* display the target point queues for the destination vehicles */
	Show_TargetPointInformation_For_All_DestinationVehicles(current_time, DVQ);
#endif
}

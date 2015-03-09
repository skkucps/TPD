/**
 *  File: graph-operations.c
	Description: Graph operations, such as Breadth First Search.
	Date: 11/8/2010
	Maker: Jaehoon Jeong
*/

#include "stdafx.h"
#include "queue.h"
#include "util.h"
#include "graph-operations.h"
#include "all-pairs-shortest-paths.h"
#include "shortest-path.h"
#include "gsl-util.h" //gsl utility funtions

/* header files and definition for KShortestPaths */
///////////////////
#include <limits>
#include <set>
#include <map>
#include <queue>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <algorithm>
#include "GraphElements.h"
#include "Graph.h"
#include "DijkstraShortestPathAlg.h"
#include "YenTopKShortestPathsAlg.h"

using namespace std;
///////////////////

/** Function Declarations */
static boolean Is_TargetPoint_Removable(int target_point, destination_vehicle_queue_t *DVQ);
/* check whethter target_point can be removable from destination vehicles' feasible target point queue FTPQs and then filter out the target point from the destination vehicles' anycast sets */

static boolean Is_TargetPoint_Removable_From_DestinationVehicle(int target_point, destination_vehicle_queue_node_t *destination_vehicle);
/* check whethter target_point is removable from destination vehicle's feasible target point queue FTPQ and even if target_point does not exist in destination_vehicle's FTPQ, we regard target_point as removable */

static int FilterOut_TargetPoint_From_FeasibleTargetPointQueues(int target_point, destination_vehicle_queue_t *DVQ);
/* filter out target_point from the corresponding destination vehicles by setting filter_flag in the corresponding target point queue node */

static int FilterOut_TargetPoint_From_One_FeasibleTargetPointQueue(int target_point, destination_vehicle_queue_node_t *destination_vehicle);
/* filter out target_point from the corresponding destination vehicle by setting filter_flag of the target point queue node in destination_vehicle's FTPQ */

static void FilterOut_RedundantTargetPoints_From_DestinationVehicle_TargetPointQueue(target_point_queue_t *FTPQ, target_point_queue_t *Q);
/* filter out redundant target points from a destination vehicle's target point queue Q */

static void Adjust_NodeDegreeInformation_In_EdgeSetQueue(edge_set_queue_t *ESQ, double **T, int T_size);
/* adjust the degree information of nodes in edge set queue ESQ when tail_node is deleted from ESQ and tail_node is a head node for another edge */

static void Fast_Adjust_NodeDegreeInformation_In_EdgeSetQueue(edge_set_queue_t *ESQ, int tail_node_id);
/* adjust fast the degree information of head_nodes in edge set queue ESQ that have tail_node_id */

static boolean Is_TargetPoint_Covered_By_Another_AnycastSet(int vehicle_id, int target_point_id, destination_vehicle_queue_t *DVQ, int *cover_vehicle_id);
/* check whether the target point in vehicle_is's anycast set is covered by another vehicle's anycast set and return the vehicle id (denoted as cover_vehicle_id) of the covering ancast set */

static boolean Is_There_TargetPoint_Covered_By_Another_AnycastSet(target_point_queue_t *Q);
/* check whether there is a target point covered by another anycast set; 
 * in this case, we do not consider this vehicle's anycast set because 
 * a representative target point can be covered by another anycast set */

static target_point_queue_node_t* Find_Representative_TargetPoint_From_AnycastSet(target_point_queue_t *Q);
/* find a representative target point from the anycast set; if there is 
 * a target point with positive reference count, we select this target point
 * as a representative for this anycast set */

static void Add_FeasibleTargetPoints_From_AnycastSet_To_GlobalTargetPointSet(target_point_queue_t *dst_vehicle_FTPQ, target_point_queue_t *FTPQ);
/* add target points from the anycast set for a destination vehicle (denoted as dst_vehicle_FTPQ) to the global target point set FTPQ. */

/**************************/

int GO_Delete_Edges_From_Graph_With_PacketTrajectoryQueue(double **M, int matrix_size, packet_trajectory_queue_t *PTQ)
{ /* delete the edges of the path corresponding to the packet trajectory PTQ from adjacency matrix M representing a graph */

	int number = 0; //the number of deleted edges
	packet_trajectory_queue_node_t *pQueueNode = NULL; //pointer to a packet trajectory queue node
	int u = 0, v = 0; //vertices
	int i = 0; //for-loop index

	pQueueNode = &(PTQ->head);
	for(i = 0; i < PTQ->size; i++)
	{
		pQueueNode = pQueueNode->next;
		if(i == 0)
		{
			u = pQueueNode->intersection_id;

			/* check the validity of u */
			if(u > matrix_size)
			{
				printf("GO_Delete_Edges_From_Graph_With_PacketTrajectoryQueue(): u(%d) is invalid: u(%d) > matrix_size(%d)\n", u, u, matrix_size);
				exit(1);
			}
		}
		else
		{
			v = pQueueNode->intersection_id;

			/* check the validity of v */
			if(v > matrix_size)
			{
				printf("GO_Delete_Edges_From_Graph_With_PacketTrajectoryQueue(): v(%d) is invalid: v(%d) > matrix_size(%d)\n", v, v, matrix_size);
				exit(1);
			}

			/* delete the edge of (u,v) by setting the edge weight to INF */
			M[u-1][v-1] = INF;

			/* update u with v */
			u = v;

			number++;
		}
	}

	return number;
}

int GO_Add_Edges_To_Graph_With_PacketTrajectoryQueue(double **M, int matrix_size, packet_trajectory_queue_t *PTQ, int start_vertex, int end_vertex)
{ /* add the edges of the path from start_vertex to end_vertex corresponding to the packet trajectory PTQ to adjacency matrix M representing a graph where start_vertex and end_vertex are virtual vertices, so we do not count the edges related to these virtual edges */
  /* Note that for start_vertex = end_vertex = 0, we add all the edges in PTQ to M */

	int number = 0; //the number of deleted edges
	packet_trajectory_queue_node_t *pQueueNode = NULL; //pointer to a packet trajectory queue node
	int u = 0, v = 0; //vertices
	int i = 0; //for-loop index
	double weight = 0; //edge weight, that is, edge cost

	pQueueNode = &(PTQ->head);
	for(i = 0; i < PTQ->size; i++)
	{
		pQueueNode = pQueueNode->next;
		if(i == 0)
		{
			u = pQueueNode->intersection_id;

			/* check the validity of u */
			if(u > matrix_size)
			{
				printf("GO_Add_Edges_To_Graph_With_PacketTrajectoryQueue(): u(%d) is invalid: u(%d) > matrix_size(%d)\n", u, u, matrix_size);
				exit(1);
			}
		}
		else
		{
			v = pQueueNode->intersection_id;

			/* check the validity of v */
			if(v > matrix_size)
			{
				printf("GO_Add_Edges_To_Graph_With_PacketTrajectoryQueue(): v(%d) is invalid: v(%d) > matrix_size(%d)\n", v, v, matrix_size);
				exit(1);
			}

			/* check whether u is start_vertex or v is end_vertex in the path */
			if((u != start_vertex) && (v != end_vertex))
			{
				/* get the edge cost */
				weight = pQueueNode->edge_cost;

				/* add the edge of (u,v) to M by setting the edge weight to weight */
				M[u-1][v-1] = weight;			
			}

			/* update u with v */
			u = v;

			number++;
		}
	}

	return number;
}

int GO_Find_ComponentVertices(double **M, int matrix_size, int u, component_vertex_queue_t *CVQ)
{ /* find the vertices connected to a vertex u in the graph denoted by adjacency matrix M by Breadth First Search (BFS) */

	int *Bitmap = NULL; //Bitmap to mark that a vertex is expanded into its neighboring vertices
	component_vertex_queue_t Q; //queue used to perform BFS
	component_vertex_queue_node_t qnode; //component vertex queue node
	component_vertex_queue_node_t *pQueueNode = NULL; //pointer to a component vertex queue node
	int vertex_id = 0; //vertex id
	int neighbor_id = 0; //neighbor id
	int i = 0; //loop index

	/** destroy component queue CVQ */
	if(CVQ->size > 0)
	{
		DestroyQueue((queue_t*)CVQ);	
	}

	/** allocate the memory for Bitmap */
	Bitmap = (int*)calloc(matrix_size+1, sizeof(int));
	assert_memory(Bitmap);

	/** initialize component vertex queues */
    InitQueue((queue_t*)&Q, QTYPE_COMPONENT_VERTEX);
	
	/** search the graph by using a queue */
	/* enqueue vertex u into Q */
	memset(&qnode, 0, sizeof(qnode));
	qnode.vertex = u;
	Enqueue((queue_t*)&Q, (queue_node_t*)&qnode);

	/* search the connected vertices */
	while(Q.size > 0)
	{
		/* dequeue a queue from the head of Q */
		pQueueNode = (component_vertex_queue_node_t*)Dequeue((queue_t*)&Q);

		/* id is set to vertex id */	
		vertex_id = pQueueNode->vertex;

		/* check the validity of vertex id */
		if(vertex_id < 1 || vertex_id > matrix_size)
		{
			printf("GO_Find_ComponentVertices(): vertex_id (%d) must be at least 1 and at most matrix_size (%d)\n", vertex_id, matrix_size);	
			exit(1);
		}

		/* mark that the vertex in this queue node is expanded */
		Bitmap[vertex_id] = 1;	

		/* enqueue this vertex into component vertex queue CVQ */
		Enqueue_With_QueueNodePointer((queue_t*)CVQ, (queue_node_t*)pQueueNode);

		/* enqueue the unexpanded neighbor vertices of vertex id into queue Q */
		for(i = 0; i < matrix_size; i++)
		{
			neighbor_id = i+1; //neighbor id
			if((M[vertex_id-1][neighbor_id-1] < INF) && (Bitmap[neighbor_id] == 0))
			{
				memset(&qnode, 0, sizeof(qnode));
				qnode.vertex = neighbor_id;
				Enqueue((queue_t*)&Q, (queue_node_t*)&qnode);	
			}
		}
	}

	/** free the memory of Bitmap */
	free(Bitmap);

	/** destroy component vertex queue Q */
	DestroyQueue((queue_t*)&Q);

	return CVQ->size;
}

boolean GO_Find_Best_Alternative_Path_For_Tree_Connectivity_With_DeliveryProbabilityConstraint(parameter_t *param, double current_time, int src, target_point_queue_t *TPQ, destination_vehicle_queue_t *DVQ, double **A, int A_size, const double **T, int T_size, packet_trajectory_queue_t *alternative_path)
{ /* make alernative path q that is a delay bounded shortest path between trees 
   * T1 and T2 in the forest T by computing k-shortest paths from v1 to v2 in 
   * tree T_new described by adjacency matrix A such that v1 is connected to 
   * all vertices in T1 as tail node and v2 is connected by all vertices in T2 
   * as head node.
   * Note that the delivery probability to each target point in target point 
   * queue TPQ in the tree consisting of T and alternative path should be 
   * satisfied to connect these two tree by the best alternative path. */

	boolean result = FALSE; //function result
	boolean flag = FALSE; //flag to indicate that the augmented tree satisfies the delivery ratio for all the target points for a destination vehicle
	int start_vertex = param->vanet_table.Gr_size + 1; //start vertex in the path as virtual node
	int end_vertex = param->vanet_table.Gr_size + 2; //end vertex in the path as virtual node
	Graph *digraph = NULL; //digraph used to search k-shortest paths
	YenTopKShortestPathsAlg *yenAlg = NULL; //Yen Algorithm object to perform k-shortest path search
	BasePath* path = NULL; //path from start_vertex to end_vertex
	double **T_prime = NULL; //augmented tree T_prime consisting of tree T and alternative path
	double **D = NULL; //D is the shortest delivery delay matrix
	double **S = NULL; //S is the shortest delivery delay variance matrix
	int **M = NULL; //M is the predecessor matrix

	int i = 0, j = 0; //loop indices
	destination_vehicle_queue_node_t *pQueueNode = NULL; //pointer to a destination vehicle queue node

	/* allocate the memory of T_prime, D_prime, S_prime, and M_prime */
	T_prime = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(T_size);
	assert_memory(T_prime);

	D = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(T_size);
	assert_memory(D);

	S = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(T_size);
	assert_memory(S);

	M = Floyd_Warshall_Allocate_Matrix_Of_Type_Int(T_size);
	assert_memory(M);

	/* create digraph object */
	digraph = new Graph;
	
	/* initialize digraph with adjacency matrix A */
	digraph->initialize((const double**)A, (const int)A_size);

	/* create yenAlg object */
	yenAlg = new YenTopKShortestPathsAlg;

	/* initialize yenAlg with (i) digraph, and (ii) start_vertex and end_vertex in the path */
	yenAlg->initialize(digraph, digraph->get_vertex(start_vertex), digraph->get_vertex(end_vertex));
	//yenAlg->initialize(*digraph, digraph->get_vertex(start_vertex), digraph->get_vertex(end_vertex));

	/** find the best shortest path among the k-shortest paths in terms of delivery cost */
	/* find the best shortest path satisfying the delivery ratio for each 
	 * target point, especially target points in T2 because target points 
	 * in T1 are already satisfied with the delivery ratio 
	 * 	Note that we search k shortest paths by at most param->k_shortest_paths_number. */
	while((i++ < param->k_shortest_paths_number) && (yenAlg->has_next()))
	{
		/* get the next path from start_vertex to end_vertex */
		path = yenAlg->next();

		/* store path into alternative_path of packet trajectory queue 
		 * @Note: we consider the subpath from start_vertex to end_vertex since start_vertex and end_vertex are virtual nodes */
		//path->Store_Path_Into_PacketTrajectoryQueue(param, alternative_path, start_vertex, end_vertex);
		path->Store_SubPath_From_StartVetex_To_EndVertex_Into_PacketTrajectoryQueue(param, alternative_path, start_vertex, end_vertex);

#if K_SHORTEST_PATH_PRINTOUT_FLAG
		/* print out the k-shortest path */
		path->PrintOut(cout);
#endif

	  	/* copy T into T_prime for augmented tree construction */
	  	Floyd_Warshall_Copy_Matrix_Of_Type_Double(T_prime, (double**)T, T_size);

		/** augment tree T_prime with alternative path, that is, 
		 * connecting T1 and T2 with alternative path */
		GO_Add_Edges_To_Graph_With_PacketTrajectoryQueue(T_prime, T_size, alternative_path, start_vertex, end_vertex);

		/** update the delivery cost matrix D_prime and the predecessor matrix M_prime along with the delivery cost variance matrix S_prime */
		/* construct delay matrices D and S from the adjacency cost matrix T_prime for the augmented tree */
		Floyd_Warshall_Make_Weight_Matrix_For_EDD_With_AdjacencyMatrix(D, S, T_size, T_prime, param);

		/* compute the matrices for all-pairs shortest-paths using the Floyd-Warshall algorithm */
		Floyd_Warshall_Construct_Shortest_PathInfo_For_EDD(D, M, S, T_size);

		/* check whether packets can be delivered to all target points with
		 * the required delivery ratio */
		pQueueNode = &(DVQ->head);
		for(j = 0; j < DVQ->size; j++)
		{
			pQueueNode = pQueueNode->next;

			/* check whether the augmented tree satisfies the delivery ratio for all the target points for a destination vehicle or not */
			flag = GO_Does_ForwardingTree_Satisfy_DeliveryConstraint_For_DestinationVehicle(param, current_time, T_prime, D, S, T_size, src, pQueueNode);
			if(flag == FALSE)
			{ /* this alternative path cannot satisfy the delivery constraint of the destination vehicle pointed by pQueueNode */
				break;
			}
		}

		/* if this augmented tree T_prime satisfies the delivery constraint,
		 * returns this alternative path; otherwise, keep searching another path
		 * until k_shortest_paths_number paths are investigated */
		if(flag == TRUE)
		{ /* we have found a good alternative path */
			result = TRUE;
			break;
		}
	}

	/* free the memory of matrices T_prime, D, S, and M */
	Floyd_Warshall_Free_Matrix_Of_Type_Double(T_prime, T_size);
	Floyd_Warshall_Free_Matrix_Of_Type_Double(D, T_size);
	Floyd_Warshall_Free_Matrix_Of_Type_Double(S, T_size);
	Floyd_Warshall_Free_Matrix_Of_Type_Int(M, T_size);

	/* delete digraph object and yenAlg object */
	delete digraph;
	delete yenAlg;

	return result;
}

boolean GO_Find_Best_Alternative_Path_For_Tree_Connectivity_With_DeliveryProbabilityConstraint_And_DeliveryDelayConstraint(parameter_t *param, double current_time, int src, target_point_queue_t *TPQ, destination_vehicle_queue_t *DVQ, double **A, int A_size, const double **T, int T_size, double beta, packet_trajectory_queue_t *alternative_path)
{ /* make alernative path q that is a delay bounded shortest path between trees 
   * T1 and T2 in the forest T by computing k-shortest paths from v1 to v2 in 
   * tree T_new described by adjacency matrix A such that v1 is connected to 
   * all vertices in T1 as tail node and v2 is connected by all vertices in T2 
   * as head node.
   * Note that the delivery probability (for delivery ratio alpha) and 
   * the delivery delay (for delay threshold beta) to each target point in 
   * target point queue TPQ in the tree consisting of T and alternative path 
   * should be satisfied to connect these two tree by the best alternative path. */

	boolean result = FALSE; //function result
	boolean flag = FALSE; //flag to indicate that the augmented tree satisfies the delivery ratio for all the target points for a destination vehicle
	int start_vertex = param->vanet_table.Gr_size + 1; //start vertex in the path as virtual node
	int end_vertex = param->vanet_table.Gr_size + 2; //end vertex in the path as virtual node
	Graph *digraph = NULL; //digraph used to search k-shortest paths
	YenTopKShortestPathsAlg *yenAlg = NULL; //Yen Algorithm object to perform k-shortest path search
	BasePath* path = NULL; //path from start_vertex to end_vertex
	double **T_prime = NULL; //augmented tree T_prime consisting of tree T and alternative path
	double **D = NULL; //D is the shortest delivery delay matrix
	double **S = NULL; //S is the shortest delivery delay variance matrix
	int **M = NULL; //M is the predecessor matrix

	int i = 0, j = 0; //loop indices
	destination_vehicle_queue_node_t *pQueueNode = NULL; //pointer to a destination vehicle queue node

	/* allocate the memory of T_prime, D_prime, S_prime, and M_prime */
	T_prime = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(T_size);
	assert_memory(T_prime);

	D = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(T_size);
	assert_memory(D);

	S = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(T_size);
	assert_memory(S);

	M = Floyd_Warshall_Allocate_Matrix_Of_Type_Int(T_size);
	assert_memory(M);

	/* create digraph object */
	digraph = new Graph;
	
	/* initialize digraph with adjacency matrix A */
	digraph->initialize((const double**)A, (const int)A_size);

	/* create yenAlg object */
	yenAlg = new YenTopKShortestPathsAlg;

	/* initialize yenAlg with (i) digraph, and (ii) start_vertex and end_vertex in the path */
	yenAlg->initialize(digraph, digraph->get_vertex(start_vertex), digraph->get_vertex(end_vertex));
	//yenAlg->initialize(*digraph, digraph->get_vertex(start_vertex), digraph->get_vertex(end_vertex));

	/** find the best shortest path among the k-shortest paths in terms of delivery cost */
	/* find the best shortest path satisfying the delivery ratio for each 
	 * target point, especially target points in T2 because target points 
	 * in T1 are already satisfied with the delivery ratio 
	 * 	Note that we search k shortest paths by at most param->k_shortest_paths_number. */
	while((i++ < param->k_shortest_paths_number) && (yenAlg->has_next()))
	{
		/* get the next path from start_vertex to end_vertex */
		path = yenAlg->next();

		/* store path into alternative_path of packet trajectory queue 
		 * @Note: we consider the subpath from start_vertex to end_vertex since start_vertex and end_vertex are virtual nodes */
		//path->Store_Path_Into_PacketTrajectoryQueue(param, alternative_path, start_vertex, end_vertex);
		path->Store_SubPath_From_StartVetex_To_EndVertex_Into_PacketTrajectoryQueue(param, alternative_path, start_vertex, end_vertex);

#if K_SHORTEST_PATH_PRINTOUT_FLAG
		/* print out the k-shortest path */
		path->PrintOut(cout);
#endif

	  	/* copy T into T_prime for augmented tree construction */
	  	Floyd_Warshall_Copy_Matrix_Of_Type_Double(T_prime, (double**)T, T_size);

		/** augment tree T_prime with alternative path, that is, 
		 * connecting T1 and T2 with alternative path */
		GO_Add_Edges_To_Graph_With_PacketTrajectoryQueue(T_prime, T_size, alternative_path, start_vertex, end_vertex);

		/** update the delivery cost matrix D_prime and the predecessor matrix M_prime along with the delivery cost variance matrix S_prime */
		/* construct delay matrices D and S from the adjacency cost matrix T_prime for the augmented tree */
		Floyd_Warshall_Make_Weight_Matrix_For_EDD_With_AdjacencyMatrix(D, S, T_size, T_prime, param);

		/* compute the matrices for all-pairs shortest-paths using the Floyd-Warshall algorithm */
		Floyd_Warshall_Construct_Shortest_PathInfo_For_EDD(D, M, S, T_size);

		/* check whether packets can be delivered to all target points with
		 * the required delivery ratio */
		pQueueNode = &(DVQ->head);
		for(j = 0; j < DVQ->size; j++)
		{
			pQueueNode = pQueueNode->next;

			/* check whether the augmented tree satisfies the delivery ratio for all the target points for a destination vehicle or not */
			flag = GO_Does_ForwardingTree_Satisfy_AllDeliveryConstraints_For_DestinationVehicle(param, current_time, T_prime, D, S, T_size, src, pQueueNode, beta);
			if(flag == FALSE)
			{ /* this alternative path cannot satisfy the delivery constraint of the destination vehicle pointed by pQueueNode */
				break;
			}
		}

		/* if this augmented tree T_prime satisfies the delivery constraint,
		 * returns this alternative path; otherwise, keep searching another path
		 * until k_shortest_paths_number paths are investigated */
		if(flag == TRUE)
		{ /* we have found a good alternative path */
			result = TRUE;
			break;
		}
	}

	/* free the memory of matrices T_prime, D, S, and M */
	Floyd_Warshall_Free_Matrix_Of_Type_Double(T_prime, T_size);
	Floyd_Warshall_Free_Matrix_Of_Type_Double(D, T_size);
	Floyd_Warshall_Free_Matrix_Of_Type_Double(S, T_size);
	Floyd_Warshall_Free_Matrix_Of_Type_Int(M, T_size);

	/* delete digraph object and yenAlg object */
	delete digraph;
	delete yenAlg;

	return result;
}

boolean GO_Find_Best_Alternative_Path_For_Tree_Connectivity_Without_Constraint(parameter_t *param, double current_time, int src, target_point_queue_t *TPQ, destination_vehicle_queue_t *DVQ, double **A, int A_size, const double **T, int T_size, packet_trajectory_queue_t *alternative_path)
{ /* make alernative path q that is a delay bounded shortest path between trees 
   * T1 and T2 in the forest T by computing k-shortest paths from v1 to v2 in 
   * tree T_new described by adjacency matrix A such that v1 is connected to 
   * all vertices in T1 as tail node and v2 is connected by all vertices in T2 
   * as head node.
   * Note that the delivery probability to each target point in target point 
   * queue TPQ in the tree consisting of T and alternative path is not guaranted 
   * to connect these two tree by the best alternative path. */

	boolean result = FALSE; //function result
	boolean flag = FALSE; //flag to indicate that the augmented tree satisfies the packet delivery for all the target points for destination vehicles
	int start_vertex = param->vanet_table.Gr_size + 1; //start vertex in the path as virtual node
	int end_vertex = param->vanet_table.Gr_size + 2; //end vertex in the path as virtual node
	Graph *digraph = NULL; //digraph used to search k-shortest paths
	YenTopKShortestPathsAlg *yenAlg = NULL; //Yen Algorithm object to perform k-shortest path search
	BasePath* path = NULL; //path from start_vertex to end_vertex
	double **T_prime = NULL; //augmented tree T_prime consisting of tree T and alternative path
	double **D = NULL; //D is the shortest delivery delay matrix
	double **S = NULL; //S is the shortest delivery delay variance matrix
	int **M = NULL; //M is the predecessor matrix

	int i = 0, j = 0; //loop indices
	destination_vehicle_queue_node_t *pQueueNode = NULL; //pointer to a destination vehicle queue node

	/* allocate the memory of T_prime, D_prime, S_prime, and M_prime */
	T_prime = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(T_size);
	assert_memory(T_prime);

	D = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(T_size);
	assert_memory(D);

	S = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(T_size);
	assert_memory(S);

	M = Floyd_Warshall_Allocate_Matrix_Of_Type_Int(T_size);
	assert_memory(M);

	/* create digraph object */
	digraph = new Graph;
	
	/* initialize digraph with adjacency matrix A */
	digraph->initialize((const double**)A, (const int)A_size);

	/* create yenAlg object */
	yenAlg = new YenTopKShortestPathsAlg;

	/* initialize yenAlg with (i) digraph, and (ii) start_vertex and end_vertex in the path */
	yenAlg->initialize(digraph, digraph->get_vertex(start_vertex), digraph->get_vertex(end_vertex));
	//yenAlg->initialize(*digraph, digraph->get_vertex(start_vertex), digraph->get_vertex(end_vertex));

	/** find the best shortest path among the k-shortest paths in terms of delivery cost */
	/* find the best shortest path satisfying the delivery ratio for each 
	 * target point, especially target points in T2 because target points 
	 * in T1 are already satisfied with the delivery ratio 
	 * 	Note that we search k shortest paths by at most param->k_shortest_paths_number. */
	while((i++ < param->k_shortest_paths_number) && (yenAlg->has_next()))
	{
		/* get the next path from start_vertex to end_vertex */
		path = yenAlg->next();

		/* store path into alternative_path of packet trajectory queue 
		 * @Note: we consider the subpath from start_vertex to end_vertex since start_vertex and end_vertex are virtual nodes */
		//path->Store_Path_Into_PacketTrajectoryQueue(param, alternative_path, start_vertex, end_vertex);
		path->Store_SubPath_From_StartVetex_To_EndVertex_Into_PacketTrajectoryQueue(param, alternative_path, start_vertex, end_vertex);

#if K_SHORTEST_PATH_PRINTOUT_FLAG
		/* print out the k-shortest path */
		path->PrintOut(cout);
#endif

	  	/* copy T into T_prime for augmented tree construction */
	  	Floyd_Warshall_Copy_Matrix_Of_Type_Double(T_prime, (double**)T, T_size);

		/** augment tree T_prime with alternative path, that is, 
		 * connecting T1 and T2 with alternative path */
		GO_Add_Edges_To_Graph_With_PacketTrajectoryQueue(T_prime, T_size, alternative_path, start_vertex, end_vertex);

		/** update the delivery cost matrix D_prime and the predecessor matrix M_prime along with the delivery cost variance matrix S_prime */
		/* construct delay matrices D and S from the adjacency cost matrix T_prime for the augmented tree */
		Floyd_Warshall_Make_Weight_Matrix_For_EDD_With_AdjacencyMatrix(D, S, T_size, T_prime, param);

		/* compute the matrices for all-pairs shortest-paths using the Floyd-Warshall algorithm */
		Floyd_Warshall_Construct_Shortest_PathInfo_For_EDD(D, M, S, T_size);

#if 0 /* [ */
		/* we have found a good alternative path */
		flag = TRUE;
#endif /* ] */

#if 1 /* [ */
		/* check whether packets can be delivered to all target points with
		 * the required delivery ratio */
		pQueueNode = &(DVQ->head);
		for(j = 0; j < DVQ->size; j++)
		{
			pQueueNode = pQueueNode->next;

#if 0 /* [[ */
			/* check whether the augmented tree satisfies the packet delivery for all the target points for a destination vehicle or not */
			flag = GO_Does_ForwardingTree_Satisfy_PacketEarlierArrival_Than_DestinationVehicle_In_Mean_Arrival_Time(param, current_time, T_prime, D, S, T_size, src, pQueueNode);
#endif /* ]] */

#if 1 /* [[ */
			/* check whether the augmented tree satisfies the finite packet delivery delay for all the target points for a destination vehicle or not */
			flag = GO_Does_ForwardingTree_Satisfy_FinitePacketDeliveryDelay_For_DestinationVehicle(param, current_time, T_prime, D, S, T_size, src, pQueueNode);
#endif /* ]] */
			if(flag == FALSE)
			{ /* this alternative path cannot satisfy the delivery constraint of the destination vehicle pointed by pQueueNode */
				break;
			}
		}
#endif /* ] */

		/* if this augmented tree T_prime satisfies the delivery constraint,
		 * returns this alternative path; otherwise, keep searching another path
		 * until k_shortest_paths_number paths are investigated */
		if(flag == TRUE)
		{ /* we have found a good alternative path */
			result = TRUE;
			break;
		}
	}

	/* free the memory of matrices T_prime, D, S, and M */
	Floyd_Warshall_Free_Matrix_Of_Type_Double(T_prime, T_size);
	Floyd_Warshall_Free_Matrix_Of_Type_Double(D, T_size);
	Floyd_Warshall_Free_Matrix_Of_Type_Double(S, T_size);
	Floyd_Warshall_Free_Matrix_Of_Type_Int(M, T_size);

	/* delete digraph object and yenAlg object */
	delete digraph;
	delete yenAlg;

	return result;
}

boolean GO_Does_ForwardingTree_Satisfy_PacketEarlierArrival_Than_DestinationVehicle_In_Mean_Arrival_Time(parameter_t *param, double current_time, double **T, double **D, double **S, int T_size, int src, destination_vehicle_queue_node_t *destination_vehicle)
{ /* check whether or not the augmented tree satisfies the packet's ealier arrival than the destination vehicle in mean arrival time where T is the delay cost matrix for the adjacency matrix of the forwarding tree T, D is the delay matrix, and S is the delay variance matrix */

	boolean flag = FALSE;
	target_point_queue_t *Q = &(destination_vehicle->FTPQ); //destination vehicle's feasible target point queue
	target_point_queue_node_t *pQueueNode = NULL; //pointer to a target point queue node
	int i = 0; //loop index
	int target_point_id = 0; //target point id
	char target_point[NAME_SIZE]; //target point as string representation

	double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
	double path_travel_time = 0; //path travel time
	double path_travel_time_deviation = 0; //path travel time deviation

	double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p

	double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p

	/* check whether or not the delivery constraint is satified for each target point in destination vehicle's feasible target point set */
	pQueueNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pQueueNode = pQueueNode->next;
		target_point_id = pQueueNode->target_point_id;
		itoa(target_point_id, target_point);

		/* compute the path distance, the path travel time, and the path travel time deviation from the destination vehicle's current position to the target point */
		path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList_Along_With_PathTravelTime_And_Deviation(param, current_time, destination_vehicle->vnode, target_point, &path_travel_time, &path_travel_time_deviation);

		/* compute the mean and standard deviation of the travel time duration 
			@Note: make sure that EAD_p is non-zero for the Gamma distribution */
		if(path_travel_time < 0)
		{ /* In this case, the destination vehicle has already passed this target point */
			return FALSE;
		}
		else
		{
			EAD_p = path_travel_time; //param->vehicle_speed should be used since it is average vehicle speed
		}   

		/** compute EDD_p for a target point p */
		/* compute the EDD for the target point at the intersection having this src AP */
		EDD_p = D[src - 1][target_point_id - 1]; //delivery delay from source to target point

		/** determine whether packet arrives at the target point earlier than the destination vehicle in mean arrival time */
		if(EDD_p > EAD_p)
		{
			return FALSE;
		}
	}
	
	return TRUE;
}

boolean GO_Does_ForwardingTree_Satisfy_FinitePacketDeliveryDelay_For_DestinationVehicle(parameter_t *param, double current_time, double **T, double **D, double **S, int T_size, int src, destination_vehicle_queue_node_t *destination_vehicle)
{ /* check whether or not the augmented tree satisfies the finite packet delivery delay to the target points in the destination vehicle's FTPQ where T is the delay cost matrix for the adjacency matrix of the forwarding tree T, D is the delay matrix, and S is the delay variance matrix */

	boolean flag = FALSE;
	target_point_queue_t *Q = &(destination_vehicle->FTPQ); //destination vehicle's feasible target point queue
	target_point_queue_node_t *pQueueNode = NULL; //pointer to a target point queue node
	int i = 0; //loop index
	int target_point_id = 0; //target point id
	char target_point[NAME_SIZE]; //target point as string representation
	double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p

	/* check whether or not the delivery constraint is satified for each target point in destination vehicle's feasible target point set */
	pQueueNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pQueueNode = pQueueNode->next;
		target_point_id = pQueueNode->target_point_id;
		itoa(target_point_id, target_point);

		/** compute EDD_p for a target point p */
		/* compute the EDD and EDD_SD for the target point at the intersection having this src AP */
		EDD_p = D[src - 1][target_point_id - 1]; //delivery delay from source to target point

		/** determine whether the packet delivery delay is finite or not */
		if(EDD_p >= INF)
		{
			return FALSE;
		}
	}
	
	return TRUE;
}

boolean GO_Does_ForwardingTree_Satisfy_DeliveryConstraint_For_DestinationVehicle(parameter_t *param, double current_time, double **T, double **D, double **S, int T_size, int src, destination_vehicle_queue_node_t *destination_vehicle)
{ /* check whether or not the augmented tree satisfies the delivery ratio for all the target points for a destination vehicle where T is the delay cost matrix for the adjacency matrix of the forwarding tree T, D is the delay matrix, and S is the delay variance matrix */

	boolean flag = FALSE;
	target_point_queue_t *Q = &(destination_vehicle->FTPQ); //destination vehicle's feasible target point queue
	target_point_queue_node_t *pQueueNode = NULL; //pointer to a target point queue node
	int i = 0; //loop index
	int target_point_id = 0; //target point id
	char target_point[NAME_SIZE]; //target point as string representation

	double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
	double path_travel_time = 0; //path travel time
	double path_travel_time_deviation = 0; //path travel time deviation

	double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
	double EDD_VAR_p = 0; //EDD_VAR_p is the packet's expected delivery delay variance
	double EDD_SD_p = 0; //EDD_SD_p is the packet's expected delivery delay standard deviation to target point p

	double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
	double EAD_VAR_p = 0; //EAD_VAR_p is the variance of the destination vehicle's arrival delay to target point p
	double EAD_SD_p = 0; //EAD_SD_p is the standard deviation of the destination vehicle's arrival delay to target point p
	double delivery_probability = 0; //delivery probability that the packet will arrive at the target point earlier that the destination vehicle 
	double packet_ttl = param->communication_packet_ttl; //packet's TTL

	double time_difference = 0; //time difference between EDD_p and EAD_p
	double movement_time = (param->communication_range)/(param->vehicle_speed); //movement time for the communication range by vehicle

	/* check whether or not the delivery constraint is satified for each target point in destination vehicle's feasible target point set */
	pQueueNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pQueueNode = pQueueNode->next;
		target_point_id = pQueueNode->target_point_id;
		itoa(target_point_id, target_point);

#if 0 /* [ */
		/* compute the path distance, the path travel time, and the path travel time deviation from the destination vehicle's current position to the target point */
		path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList_Along_With_PathTravelTime_And_Deviation(param, current_time, destination_vehicle->vnode, target_point, &path_travel_time, &path_travel_time_deviation);
#endif /* ] */
#if 1 /* [[ */
		/* compute the path distance, the path travel time, and the path travel time deviation from the destination vehicle's current position to the target point */
		path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList_Along_With_PathTravelTime_And_Deviation_AccordingTo_PathNodeIndex(param, current_time, destination_vehicle->vnode, target_point, i, &path_travel_time, &path_travel_time_deviation);
#endif /* ]] */

		/* compute the mean and standard deviation of the travel time duration 
			@Note: make sure that EAD_p is non-zero for the Gamma distribution */
		if(path_travel_time < 0)
		{ /* In this case, the destination vehicle has already passed this target point */
			return FALSE;
		}
		else
		{
			EAD_p = path_travel_time; //param->vehicle_speed should be used since it is average vehicle speed
			EAD_SD_p = path_travel_time_deviation;
		}   

		/** compute EDD_p for a target point p */
		/* compute the EDD and EDD_SD for the target point at the intersection having this src AP */
		EDD_p = D[src - 1][target_point_id - 1]; //delivery delay from source to target point
		EDD_VAR_p = S[src - 1][target_point_id - 1]; //delivery delay variance from source to target point
		EDD_SD_p = sqrt(EDD_VAR_p); //delivery delay standard deviation

		/* check the validity of EAD_p and EDD_p */
		if(EAD_p < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
		{
			if(EDD_p > EAD_p)
			{
#if 0 /* [ */
				return FALSE;
#endif /* ] */
				time_difference = EDD_p - EAD_p;
				if(time_difference <= movement_time)
				{
					delivery_probability = 1;
				}
				else
				{
					return FALSE;
				}
			}
			else
			{
				delivery_probability = 1;
			}
		}
		else
		{
			/** Compute the probability that the packet will arrive at the target point earlier than the destination vehicle */
			/* Note that the probability is P[T_i^p <= T_i^v] = \int_{0}^{\infy} \int_{0}^{y} {f(x)g(y)}\,dx\,dy */
			delivery_probability = GSL_Vanet_Delivery_Probability_For_Gamma_Distribution(EDD_p, EDD_SD_p, EAD_p, EAD_SD_p, 0, packet_ttl); //compute the delivery probability for two Gamma random variables: (i) Packet delay of N(mu_p, sigma_p) and (ii) Vehicle delay of N(mu_v, sigma_v)
		}

		/** check whether the packet arrives at the target intersection p earlier than the destination vehicle */
		if(delivery_probability < param->communication_packet_delivery_probability_threshold)
		{ /* In this case, the forwarding tree cannot deliver the packet to the destination vehicle with the required delivery ratio */
			return FALSE;
		}
	}
	
	return TRUE;
}

boolean GO_Does_ForwardingTree_Satisfy_AllDeliveryConstraints_For_DestinationVehicle(parameter_t *param, double current_time, double **T, double **D, double **S, int T_size, int src, destination_vehicle_queue_node_t *destination_vehicle, double beta)
{ /* check whether or not the augmented tree satisfies the delivery ratio and the delivery delay bound beta for all the target points for a destination vehicle where T is the delay cost matrix for the adjacency matrix of the forwarding tree T, D is the delay matrix, and S is the delay variance matrix */

	boolean flag = FALSE;
	target_point_queue_t *Q = &(destination_vehicle->FTPQ); //destination vehicle's feasible target point queue
	target_point_queue_node_t *pQueueNode = NULL; //pointer to a target point queue node
	int i = 0; //loop index
	int target_point_id = 0; //target point id
	char target_point[NAME_SIZE]; //target point as string representation

	double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
	double path_travel_time = 0; //path travel time
	double path_travel_time_deviation = 0; //path travel time deviation

	double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
	double EDD_VAR_p = 0; //EDD_VAR_p is the packet's expected delivery delay variance
	double EDD_SD_p = 0; //EDD_SD_p is the packet's expected delivery delay standard deviation to target point p

	double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
	double EAD_VAR_p = 0; //EAD_VAR_p is the variance of the destination vehicle's arrival delay to target point p
	double EAD_SD_p = 0; //EAD_SD_p is the standard deviation of the destination vehicle's arrival delay to target point p
	double delivery_probability = 0; //delivery probability that the packet will arrive at the target point earlier that the destination vehicle 
	double packet_ttl = param->communication_packet_ttl; //packet's TTL

	double time_difference = 0; //time difference between EDD_p and EAD_p
	double movement_time = (param->communication_range)/(param->vehicle_speed); //movement time for the communication range by vehicle

	/* check whether or not the delivery constraint is satified for each target point in destination vehicle's feasible target point set */
	pQueueNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pQueueNode = pQueueNode->next;
		target_point_id = pQueueNode->target_point_id;
		itoa(target_point_id, target_point);

#if 0 /* [ */
		/* compute the path distance, the path travel time, and the path travel time deviation from the destination vehicle's current position to the target point */
		path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList_Along_With_PathTravelTime_And_Deviation(param, current_time, destination_vehicle->vnode, target_point, &path_travel_time, &path_travel_time_deviation);
#endif /* ] */
#if 1 /* [[ */
		/* compute the path distance, the path travel time, and the path travel time deviation from the destination vehicle's current position to the target point */
		path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList_Along_With_PathTravelTime_And_Deviation_AccordingTo_PathNodeIndex(param, current_time, destination_vehicle->vnode, target_point, i, &path_travel_time, &path_travel_time_deviation);
#endif /* ] */

		/* compute the mean and standard deviation of the travel time duration 
			@Note: make sure that EAD_p is non-zero for the Gamma distribution */
		if(path_travel_time < 0)
		{ /* In this case, the destination vehicle has already passed this target point */
			return FALSE;
		}
		else
		{
			EAD_p = path_travel_time; //param->vehicle_speed should be used since it is average vehicle speed
			EAD_SD_p = path_travel_time_deviation;
		}   

		/** compute EDD_p for a target point p */
		/* compute the EDD and EDD_SD for the target point at the intersection having this src AP */
		EDD_p = D[src - 1][target_point_id - 1]; //delivery delay from source to target point
		EDD_VAR_p = S[src - 1][target_point_id - 1]; //delivery delay variance from source to target point
		EDD_SD_p = sqrt(EDD_VAR_p); //delivery delay standard deviation

		/* check the validity of EAD_p */
		if(EAD_p < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
		{
			if(EDD_p > EAD_p)
			{
#if 0 /* [ */
				return FALSE;
#endif /* ] */
				time_difference = EDD_p - EAD_p;
				if(time_difference <= movement_time)
				{
					delivery_probability = 1;
				}
				else
				{
					return FALSE;
				}
			}
			else
			{
				delivery_probability = 1;
			}
		}
		else
		{
			/** Compute the probability that the packet will arrive at the target point earlier than the destination vehicle */
			/* Note that the probability is P[T_i^p <= T_i^v] = \int_{0}^{\infy} \int_{0}^{y} {f(x)g(y)}\,dx\,dy */
			delivery_probability = GSL_Vanet_Delivery_Probability_For_Gamma_Distribution(EDD_p, EDD_SD_p, EAD_p, EAD_SD_p, 0, packet_ttl); //compute the delivery probability for two Gamma random variables: (i) Packet delay of N(mu_p, sigma_p) and (ii) Vehicle delay of N(mu_v, sigma_v)
		}

		/** check whether the packet arrives at the target intersection p earlier than the destination vehicle with delivery delay bound beta */
		if((delivery_probability < param->communication_packet_delivery_probability_threshold) || (EDD_p > beta))
		{ /* In this case, the forwarding tree cannot deliver the packet to the destination vehicle with the required delivery ratio */
			return FALSE;
		}
	}
	
	return TRUE;
}

int GO_Construct_EdgeQueue_With_AdjacencyMatrix_And_FeasibleTargetPointQueue(double **T, int T_size, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ)
{ /* construct edge set queue ESQ with adjacency matrix T and feasible target point queue FTPQ */
	int edge_number = 0;
	int i = 0, j = 0; //loop-indices
	int index = 0; //index for an arrary or vector such that (index+1) is node_id
	edge_set_queue_node_t qnode; //edge set queue node
	int *indegrees = NULL; //indegree vector where the index corresponds to node_id - 1
	int *outdegrees = NULL; //outdegree vector where the index corresponds to node_id - 1
	boolean *target_point_flags = NULL; 
	/* vector to indicate whether the node corresponding to index is a target point or not */
	target_point_queue_node_t *pTP_QueueNode = NULL; //pointer to a target point queue node

	/** allocate the memory for vectors indegree and outdegree and 
	 * set each vector's elements to 0 */
	indegrees = (int*)calloc(T_size, sizeof(int));
	assert_memory(indegrees);

	outdegrees = (int*)calloc(T_size, sizeof(int));
	assert_memory(outdegrees);

	/** allocate the memory for target point flag vector */
	target_point_flags = (boolean*)calloc(T_size, sizeof(boolean));
	assert_memory(target_point_flags);

	/** compute outdegree and indegree per node */
	for(i = 0; i < T_size; i++)
	{
		for(j = 0; j < T_size; j++)
		{
		  	/* ignore the case where tail_node and head_node are the same */
			if(i == j)
				continue;
			
			/* count the outdegree for node_id (i+1) */
			if(T[i][j] < INF)
			{
				outdegrees[i]++; //for the outdegree of index i
				indegrees[j]++; //for the indegree of index j
			}
			
		}
	}

	/** set up vector target_point_flags with feasible target point queue FTPQ */
	pTP_QueueNode = &(FTPQ->head);
	for(i = 0; i < FTPQ->size; i++)
	{
		pTP_QueueNode = pTP_QueueNode->next;
		index = pTP_QueueNode->target_point_id - 1;
		if(index < 0)
		{
		  printf("Construct_EdgeQueue_With_AdjacencyMatrix_And_FeasibleTargetPointQueue(): index (%d) must be non-negative integer\n", index);
		  exit(1);
		}
		target_point_flags[index] = TRUE;
	}

	/** set ESQ's source_node_id to FTPQ's source_node_id as packe source node id */
	ESQ->source_node_id = FTPQ->source_node_id;

	/** enqueue directed edges into edge set queue ESQ */
	for(i = 0; i < T_size; i++)
	{
		for(j = 0; j < T_size; j++)
		{
			if((i == j) || (T[i][j] == INF))
				continue;

			/* initialize queue node */
			memset(&qnode, 0, sizeof(qnode));
			qnode.tail_node_id = i+1;
			qnode.head_node_id = j+1;
			itoa(i+1, qnode.tail_node);
			itoa(j+1, qnode.head_node);
			qnode.weight = T[i][j];

			/* set up the indegree and outdegree of tail_node and head_node */
			qnode.tail_node_indegree = indegrees[i];
			qnode.tail_node_outdegree = outdegrees[i];
			qnode.head_node_indegree = indegrees[j];
			qnode.head_node_outdegree = outdegrees[j];

			/* set up tail_node's target_point_flag and head_node's target_point_flag with target_point_flags */
			qnode.tail_node_target_point_flag = target_point_flags[i];
			qnode.head_node_target_point_flag = target_point_flags[j];

			/* enqueue qnode into ESQ */
			Enqueue((queue_t*)ESQ, (queue_node_t*)&qnode);
			edge_number++;
		}
	}

	/** free the memory of vectors indegrees and outdegrees */
	free(indegrees);
	free(outdegrees);

	/** free the memory for target point flag vector */
	free(target_point_flags);

	return edge_number;
}

void GO_Construct_AdjacencyMatrix_With_EdgeQueue(edge_set_queue_t *ESQ, double **T, int T_size)
{ /* construct adjacency matrix T with edge set queue ESQ */
	edge_set_queue_node_t *pQueueNode = NULL; //pointer to an edge set queue node
	int i = 0, j = 0; //loop-indices
	int tail_node = 0, head_node = 0;

	if(T == NULL)
	{
		printf("GO_Construct_AdjacencyMatrix_With_EdgeQueue(): Error: T is NULL\n");
		exit(1);
	}
	else if(T_size <= 0)
	{
		printf("GO_Construct_AdjacencyMatrix_With_EdgeQueue(): Error: T_size (%d) is not positve number\n", T_size);
		exit(1);
	}
	else if(ESQ == NULL)
	{
		printf("GO_Construct_AdjacencyMatrix_With_EdgeQueue(): Error: ESQ is NULL\n");
		exit(1);
	}

	/* initialize adjacency matrix T */
	for(i = 0; i < T_size; i++)
	{
	  	T[i][i] = 0;
		for(j = i+1; j < T_size; j++)
		{
			T[i][j] = INF;
			T[j][i] = INF;
		}
	}

	pQueueNode = &(ESQ->head);
	for(i = 0; i < ESQ->size; i++)
	{
		pQueueNode = pQueueNode->next;
		tail_node = pQueueNode->tail_node_id;
		head_node = pQueueNode->head_node_id;
		T[tail_node-1][head_node-1] = pQueueNode->weight;
	}
}


int GO_Find_RedundantTargetPoint_From_EdgeSetQueue_AlongWith_Queue_Adjustment(int Gr_size, destination_vehicle_queue_t *DVQ, edge_set_queue_t *ESQ)
{ /* find a redundant target point that can be removed from the multicast tree represented as edge set queue ESQ and then adjust DVQ and ESG for the deletion of a target point */
	int i = 0; //loop-index
	edge_set_queue_node_t *pQueueNode = NULL; //pointer to an edge set queue node
	edge_set_queue_node_t *pParentQueueNode = NULL; //pointer to the redundant edge's tail_node's incoming edge
	boolean flag = FALSE; //flag to indicate whether an eligible edge is found or not
	int target_point = 0; //redundant target point to be removed from queue ESQ
	boolean	flag_for_parent_edge = FALSE; //flag to indicate whether there is another edge to delete from the forwarding tree
	double **T = NULL; //adjacency matrix for forwarding tree T
	int T_size = Gr_size; //size of adjacency matrix

	/** allocate the memory for adjacency matrix T */
	T = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(T_size);
	assert_memory(T);

	/** find an edge whose head_node is leaf node as a redundant target point in the Anycast set for a destination vehicle */
	pQueueNode = &(ESQ->head);
	for(i = 0; i < ESQ->size; i++)
	{
		pQueueNode = pQueueNode->next;

		/* check whether this edge is eligible to be removed as an edge with 
		 * a redundant target point that is the head node of the edge */
		if((pQueueNode->head_node_target_point_flag == TRUE) && (pQueueNode->head_node_outdegree == 0))
		{
			/* check whether the target point in this edge can be deleted from 
			 * Anycast sets of destination vehicles.
			 * Note that each Anycast set's must have at least one target point. */
			if(Is_TargetPoint_Removable(pQueueNode->head_node_id, DVQ))
			{
				flag = TRUE;
				break;
			}
		}
	}

	/* if an eligible edge is found, dequeue this queue node from queue ESQ */
	if(flag)
	{	
		/* set target_point to the redundant target point pointed by pQueueNode->head_node_id */
		target_point = pQueueNode->head_node_id;

		do
		{
			/* dequeue the queue corresponding to pQueueNode */
			Dequeue_With_QueueNodePointer((queue_t*)ESQ, (queue_node_t*)pQueueNode);

#if REDUNDANT_TARGET_POINT_PRINTOUT_FLAG
			printf("GO_Find_RedundantTargetPoint_From_EdgeSetQueue_AlongWith_Queue_Adjustment(): the deleted redundant edge is <%d, %d>\n", pQueueNode->tail_node_id, pQueueNode->head_node_id);
#endif

			/* if the deleted edge's tail_node is non-target-point and its outdegree is one,
			 * we can remove this tail_node and the incident edge from the tree root.
			 * This is because the tail_node is not needed any more for multicast forwarding.
			 * We can repeat this removal of non-target-point relay nodes until the tail_node is
			 * a packet source or another target point */
			if((pQueueNode->tail_node_outdegree == 1) && (pQueueNode->tail_node_target_point_flag == FALSE) && (pQueueNode->tail_node_id != ESQ->source_node_id))
			{ /* delete the edge where tail_node is a head node */
			  	/* make sure tail_node_indegree is one as tree edge */
				if(pQueueNode->tail_node_indegree != 1)
				{
					printf("GO_Find_Edge_With_RedundantTargetPoint_From_EdgeSetQueue_AlongWith_Queue_Adjustment(): tail_node_indegree (%d) must be one in a tree\n", pQueueNode->tail_node_indegree);
					exit(1);
				}

				/** find the edge where tail_node is a head node */

				/* set flag_for_parent_edge to FALSE as initial value */
				flag_for_parent_edge = FALSE;

				pParentQueueNode = &(ESQ->head);
				for(i = 0; i < ESQ->size; i++)
				{
					pParentQueueNode = pParentQueueNode->next;

					/* check whether this edge is eligible to be removed as an edge with 
					 * a redundant target point that is the head node of the edge */
					if((pQueueNode->tail_node_id == pParentQueueNode->head_node_id) && (pParentQueueNode->head_node_outdegree == 1))
					{
						/* set flag_for_parent_edge to TRUE to indicate that there is 
						 * another edge to delete in the forwarding tree */
					  	flag_for_parent_edge = TRUE;
						break;
					}
				}			
			}
			else
			{
				/* set flag_for_parent_edge to FALSE to indicate that there is 
				 * no other edge to delete in the forwarding tree */
				flag_for_parent_edge = FALSE;
				pParentQueueNode = NULL;
			}
	
			/* adjust the degree information of nodes in edge set queue ESQ when tail_node is deleted from ESQ */
			Fast_Adjust_NodeDegreeInformation_In_EdgeSetQueue(ESQ, pQueueNode->tail_node_id);
			//Adjust_NodeDegreeInformation_In_EdgeSetQueue(ESQ, T, T_size);

			/* free the memory of edge set queue node */
			free(pQueueNode);

			/* set pQueueNode to pParentQueueNode */
			if(flag_for_parent_edge)
			{
				pQueueNode = pParentQueueNode;
			}
			else
		  	{   /* there is no more edge to delete from the forwarding tree */
			    break;
			}
		} while(1);
	}
	else
	  {	/* set target_point to -1, indicating that there is no redundant target point */
		target_point = -1;
	}

	/** free the memory for adjacency matrix T */
	Floyd_Warshall_Free_Matrix_Of_Type_Double(T, T_size);

	return target_point;
}

static boolean Is_TargetPoint_Removable(int target_point, destination_vehicle_queue_t *DVQ)
{ /* check whethter target_point can be removable from destination vehicles' feasible target point queue FTPQs and then filter out the target point from the destination vehicles' anycast sets */
	boolean flag = TRUE; //flag to indicate whether target_point can be removed from FTPQs
	destination_vehicle_queue_node_t *pQueueNode = NULL; //pointer to a destination vehicle queue node
	int i = 0; //loop-index

	/** This function checks that the removal of the passed target_point does not
 empty any Anycast set of destination vehicle. */
	pQueueNode = &(DVQ->head);
	for(i = 0; i < DVQ->size; i++)
	{
		pQueueNode = pQueueNode->next;
	
		/* check whether this target point can be removed from the Anycast set of the destination vehicle for pQueueNode */
		if(!Is_TargetPoint_Removable_From_DestinationVehicle(target_point, pQueueNode))
		{
			flag = FALSE;
			break;
		}
	}

	/* filter out target_point from the corresponding destination vehicles by setting filter_flag in the corresponding target point queue node */
	if((DVQ->size > 0) && (flag == TRUE))
	{
		FilterOut_TargetPoint_From_FeasibleTargetPointQueues(target_point, DVQ);
	}

	return flag;
}

static boolean Is_TargetPoint_Removable_From_DestinationVehicle(int target_point, destination_vehicle_queue_node_t *destination_vehicle)
{ /* check whethter target_point is removable from destination vehicle's feasible target point queue FTPQ and even if target_point does not exist in destination_vehicle's FTPQ, we regard target_point as removable */
	boolean flag = TRUE;
	target_point_queue_t *Q = &(destination_vehicle->FTPQ); //destination vehicle's feasible target point queue
	target_point_queue_node_t *pQueueNode = NULL; //pointer to a target point queue
	int i = 0; //loop-index

	/* check whether this target point can be removable from Q */
	pQueueNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pQueueNode = pQueueNode->next;
		if((pQueueNode->target_point_id == target_point) && (pQueueNode->filter_flag == TRUE))
		{
#if TARGET_POINT_FILTERING_TRACE_FLAG
			printf("Is_TargetPoint_Removable_From_DestinationVehicle(): target_point (%d) for destination vehicle (vid=%d) is already filtered out\n", target_point, destination_vehicle->vnode->id);
#endif
			flag = FALSE;
			break;
		}
		else if((pQueueNode->target_point_id == target_point) && (pQueueNode->filter_flag == FALSE))
		{
			/* check whether feasible target point number is greater than one */
		  	if(Q->feasible_target_point_number == 1)
			{
#if TARGET_POINT_FILTERING_TRACE_FLAG
				printf("Is_TargetPoint_Removable_From_DestinationVehicle(): target_point (%d) for destination vehicle (vid=%d) is not removable because the anycast set (size=%d) has only this target point as feasible target point\n", target_point, destination_vehicle->vnode->id, Q->feasible_target_point_number);
#endif
				flag = FALSE;
				break;
			}
			else
			{
#if TARGET_POINT_FILTERING_TRACE_FLAG
				printf("Is_TargetPoint_Removable_From_DestinationVehicle(): target_point (%d) for destination vehicle (vid=%d) is removable because the anycast set (size=%d) has more than one target point as feasible target points\n", target_point, destination_vehicle->vnode->id, Q->feasible_target_point_number);
#endif
				flag = TRUE;
				break;
			}
		}		
	}

	return flag;
}
	
static int FilterOut_TargetPoint_From_FeasibleTargetPointQueues(int target_point, destination_vehicle_queue_t *DVQ)
{ /* filter out target_point from the corresponding destination vehicles by setting filter_flag in the corresponding target point queue node */
  int result = 0; //function result
	int number = 0; //the number of anycast sets to filter out target_point
	destination_vehicle_queue_node_t *pQueueNode = NULL; //pointer to a destination vehicle queue node
	int i = 0; //loop-index

	/** This function filter out target_point in the anycast set of destination vehicle. */
	pQueueNode = &(DVQ->head);
	for(i = 0; i < DVQ->size; i++)
	{
		pQueueNode = pQueueNode->next;

		/* filter out target_point from pQueueNode's FTPQ */
		result = FilterOut_TargetPoint_From_One_FeasibleTargetPointQueue(target_point, pQueueNode);
		if(result == 1)
		{
			number++;
#if TARGET_POINT_FILTER_TEST_FLAG
			printf("FilterOut_TargetPoint_From_FeasibleTargetPointQueues(): target_point (%d) is filtered out from the FTPQ of destination vehicle (vid=%d)\n", target_point, pQueueNode->vnode->id);
#endif
		}
	}

	return number;
}

static int FilterOut_TargetPoint_From_One_FeasibleTargetPointQueue(int target_point, destination_vehicle_queue_node_t *destination_vehicle)
{ /* filter out target_point from the corresponding destination vehicle by setting filter_flag of the target point queue node in destination_vehicle's FTPQ */
	target_point_queue_t *Q = &(destination_vehicle->FTPQ);
	target_point_queue_node_t *pQueueNode = NULL;
	int i = 0; //loop-index
	int number = 0; //the number of target point filtered out

	pQueueNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pQueueNode = pQueueNode->next;

		if((pQueueNode->target_point_id == target_point) && (pQueueNode->filter_flag == FALSE))
		{
			pQueueNode->filter_flag = TRUE;
			Q->feasible_target_point_number--; //decrease the number of actual feasible target points
			number++;
			break;
		}
	}
	
	return number;
}

static void Adjust_NodeDegreeInformation_In_EdgeSetQueue(edge_set_queue_t *ESQ, double **T, int T_size)
{ /* adjust the degree information of nodes in edge set queue ESQ when tail_node is deleted from ESQ and tail_node is a head node for another edge */
	int *indegrees = NULL; //indegree vector where the index corresponds to node_id - 1
	int *outdegrees = NULL; //outdegree vector where the index corresponds to node_id - 1
	int tail_node = 0, head_node = 0; //tail_node's id and head_node's id
	int i = 0, j = 0; //loop-indices
	edge_set_queue_node_t *pQueueNode = NULL; //pointer to an edge set queue node

	/** construct adjacency matrix T with edge set queue ESQ */
	GO_Construct_AdjacencyMatrix_With_EdgeQueue(ESQ, T, T_size);

	/** allocate the memory for vectors indegree and outdegree and 
	 * set each vector's elements to 0 */
	indegrees = (int*)calloc(T_size, sizeof(int));
	assert_memory(indegrees);

	outdegrees = (int*)calloc(T_size, sizeof(int));
	assert_memory(outdegrees);

	/** compute outdegree and indegree per node */
	for(i = 0; i < T_size; i++)
	{
		for(j = 0; j < T_size; j++)
		{
		  	/* ignore the case where tail_node and head_node are the same */
			if(i == j)
				continue;
			
			/* count the outdegree for node_id (i+1) */
			if(T[i][j] < INF)
			{
				outdegrees[i]++; //for the outdegree of index i
				indegrees[j]++; //for the indegree of index j
			}			
		}
	}

	/* update the node degree information of nodes in the queue ESQ */
	pQueueNode = &(ESQ->head);
	for(i = 0; i < ESQ->size; i++)
	{
		pQueueNode = pQueueNode->next;
		tail_node = pQueueNode->tail_node_id;
		head_node = pQueueNode->head_node_id;

		/* update tail_node's degree information */
		pQueueNode->tail_node_indegree = indegrees[tail_node-1];
		pQueueNode->tail_node_outdegree = outdegrees[tail_node-1];

		/* update head_node's degree information */
		pQueueNode->head_node_indegree = indegrees[head_node-1];
		pQueueNode->head_node_outdegree = outdegrees[head_node-1];
	}

	/** free the memory of vectors indegrees and outdegrees */
	free(indegrees);
	free(outdegrees);
}

static void Fast_Adjust_NodeDegreeInformation_In_EdgeSetQueue(edge_set_queue_t *ESQ, int tail_node_id)
{ /* adjust fast the degree information of head_nodes in edge set queue ESQ that have tail_node_id */
	int i = 0; //loop-index
	edge_set_queue_node_t *pQueueNode = NULL; //pointer to an edge set queue node

	pQueueNode = &(ESQ->head);
	for(i = 0; i < ESQ->size; i++)
	{
		pQueueNode = pQueueNode->next;
		
		/* update the outdegree of nodes related to tail_node_id */
		if(pQueueNode->head_node_id == tail_node_id)
		{ /* update the outdegree of head_nodes that are tail_node_id */
			/* decrease head_node_outdegree by 1 */
			pQueueNode->head_node_outdegree--;
		}
		else if(pQueueNode->tail_node_id == tail_node_id)
		{ /* update the outdegree of tail_nodes that are tail_node_id */
			/* decrease tail_node_outdegree by 1 */
			pQueueNode->tail_node_outdegree--;
		}
	}	
}

void GO_FilterOut_RedundantTargetPoints_From_Anycast_Sets(target_point_queue_t *FTPQ, destination_vehicle_queue_t *DVQ)
{ /* filter out target points per destination vehicle's anycast set with FTPQ */
	destination_vehicle_queue_node_t *pQueueNode = NULL; //pointer to a destination vehicle queue node
	int i = 0; //loop index
	boolean flag = FALSE; //flag to determine whether a target point should be filtered out or not

	pQueueNode = &(DVQ->head);
	for(i = 0; i < DVQ->size; i++)
	{
		pQueueNode = pQueueNode->next;

		/* set filter_flag to TRUE for target points that do not belong to FTPQ */
		FilterOut_RedundantTargetPoints_From_DestinationVehicle_TargetPointQueue(FTPQ, &(pQueueNode->FTPQ));
	}
}

static void FilterOut_RedundantTargetPoints_From_DestinationVehicle_TargetPointQueue(target_point_queue_t *FTPQ, target_point_queue_t *Q)
{ /* filter out redundant target points from a destination vehicle's target point queue Q */
	target_point_queue_node_t *pQueueNode = NULL; //pointer to a target point queue node
	int i = 0;
	boolean flag = FALSE; //flag to check whether a target point belongs to feasible target point queue FTPQ

	pQueueNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pQueueNode = pQueueNode->next;

		/* check whether target_point pointed by pQueueNode belongs to FTPQ */
		flag = Is_TargetPoint_In_TargetQueue(pQueueNode->target_point_id, FTPQ);
		if(flag == FALSE)
		{ /* in the case where pQueueNode's target point does not belong to FTPQ */
			pQueueNode->filter_flag = TRUE;
		}
	}
}

void GO_Mark_Duplicate_Covered_TargetPoint_In_TargetPointSet(double current_time, destination_vehicle_queue_t *DVQ, target_point_queue_t *FTPQ)
{ /* mark target points in FTPQ that are covered by more than one anycast set 
   * by setting cover_flag to TRUE for those target points that are redundant 
   * target points as relay nodes in the feasible target point queue FTPQ */
	destination_vehicle_queue_node_t *pDV_QueueNode = NULL; //pointer to a destination vehicle queue node
	target_point_queue_node_t *pTP_QueueNode = NULL; //pointer to a target point queue node
	//target_point_queue_node_t qnode; //target point queue node
	int i = 0, j = 0; //loop index
	boolean flag = FALSE; //flag to check whether each anycast set has only one target point
	int cover_vehicle_id = 0; //vehicle id of an anycast set covering a target point in another anycast set

	/** mark duplicately covered target points for each anycast set */
	pDV_QueueNode = &(DVQ->head);
	for(i = 0; i < DVQ->size; i++)
	{
		pDV_QueueNode = pDV_QueueNode->next;

		/* check whether each target point in an anycast set is covered 
		 * by another anycast set */
		pTP_QueueNode = &(pDV_QueueNode->FTPQ.head);
		for(j = 0; j < pDV_QueueNode->FTPQ.size; j++)
		{
			pTP_QueueNode = pTP_QueueNode->next;

			/* check whether the target point is already filtered out or not; 
			 * if so, we exclude this target point for duplicate cover */
			if(pTP_QueueNode->filter_flag == TRUE)
			{
				continue;
			}

			/* check whether the target point pointed by pTP_QueueNode 
			 * is covered by another vehicle's anycast set */
			flag = Is_TargetPoint_Covered_By_Another_AnycastSet(pDV_QueueNode->vid, pTP_QueueNode->target_point_id, DVQ, &cover_vehicle_id);
			if(flag == TRUE)
			{
				/* mark this target point such that it is covered by another anycast set */
				pTP_QueueNode->cover_flag = TRUE;

				/* set this target point's filter flag to TRUE because it is covered by the anycast set of cover_vehicle_id */
				pTP_QueueNode->filter_flag = TRUE;
			
				/* @Note: set which vehicle anycast set cover this target point; this cover_vid will be used to check whether this vehicle's same target point is covered by another vehicle; That is, in this check, we make sure that this vehicle's target point is not covered by another one */
				pTP_QueueNode->cover_vehicle_id = cover_vehicle_id;
			}
		}
	}

	/** mark duplicately-covered target points in FTPQ with DVQ's anycasts */
	/* destroy FTPQ */
	if(FTPQ->size)
	{
		DestroyQueue((queue_t*)FTPQ);
	}

	/* construct FTPQ with destination vehicles' FTPQ */
	pDV_QueueNode = &(DVQ->head);
	for(i = 0; i < DVQ->size; i++)
	{
		pDV_QueueNode = pDV_QueueNode->next;

		/* add feasible target points from the anycast set to the global target point set FTPQ. */
		Add_FeasibleTargetPoints_From_AnycastSet_To_GlobalTargetPointSet(&(pDV_QueueNode->FTPQ), FTPQ);
	}	
}

static boolean Is_TargetPoint_Covered_By_Another_AnycastSet(int vehicle_id, int target_point_id, destination_vehicle_queue_t *DVQ, int *cover_vehicle_id)
{ /* check whether the target point in vehicle_is's anycast set is covered by another vehicle's anycast set and return the vehicle id (denoted as cover_vehicle_id) of the covering ancast set */
	boolean flag = FALSE; //flag to indicate whether target_point_id is covered by another vehicle's anycast set
	destination_vehicle_queue_node_t *pQueueNode = NULL; //pointer to a destination vehicle queue node
	int i = 0; //loop index

	pQueueNode = &(DVQ->head);
	for(i = 0; i < DVQ->size; i++)
	{
		pQueueNode = pQueueNode->next;

		if(vehicle_id == pQueueNode->vid)
		{ /* skip the same vehicle's anycast set */
			continue;
		}

		/* check whether target_point_id is in pQueueNode->FTPQ, 
		 * considering the anycast set cover; if a target point in pQueueNode->FTPQ 
		 * has its cover_flag set to TRUE, it cannot cover another target point 
		 * denoted as target_point_id;
		 * this function updates the reference count of a target point in pQueueNode->FTPQ 
		 * if this target point can cover target_point_id */
		flag = Is_TargetPoint_In_TargetQueue_With_Anycast_Set_Cover_Along_With_ReferenceCount_Update(vehicle_id, target_point_id, &(pQueueNode->FTPQ));
		//flag = Is_TargetPoint_In_TargetQueue(target_point_id, &(pQueueNode->FTPQ));
		if(flag)
		{
		  	/* set cover_vehicle_id to the covering anycast set's vid */
			*cover_vehicle_id = pQueueNode->FTPQ.anycast_set_vehicle_id;
			break;
		}
	}
	
	return flag;
}

static boolean Is_There_TargetPoint_Covered_By_Another_AnycastSet(target_point_queue_t *Q)
{ /* check whether there is a target point covered by another anycast set; 
   * in this case, we do not consider this vehicle's anycast set because 
   * a representative target point can be covered by another anycast set */
	boolean flag = FALSE;
	target_point_queue_node_t *pQueueNode = NULL;
	int i = 0;

	pQueueNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pQueueNode = pQueueNode->next;
		if(pQueueNode->cover_flag)
		{
			flag = TRUE;
			break;
		}
	}
	
	return flag;
}

static target_point_queue_node_t* Find_Representative_TargetPoint_From_AnycastSet(target_point_queue_t *Q)
{ /* find a representative target point from the anycast set; if there is 
   * a target point with positive reference count, we select this target point
   * as a representative for this anycast set */
	target_point_queue_node_t* referenced_target_point = NULL; //representative target point refereced by another target point in another anycast set
	target_point_queue_node_t* first_nonfiltered_target_point = NULL; //pointer to the first non-filtered target point
	target_point_queue_node_t* pQueueNode = NULL; //pointer to a target point queue node
	int i = 0; //loop index

	/* check whether each target point in an anycast set can be added to FTPQ */ 
	pQueueNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pQueueNode = pQueueNode->next;

		/* search first_nonfiltered_target_point and referenced_target_point in FTPQ */
		if(pQueueNode->filter_flag == FALSE)
		{ //this target point is eligible for representative candidate
			/* check whether this is the first non-filtered target point */
			if(first_nonfiltered_target_point == NULL)
			{
				first_nonfiltered_target_point = pQueueNode;
			}
		}
		else
		{
			continue;
		}

		/* check whether this target point is referenced by another target point in another anycast set */
		if(pQueueNode->reference_count > 0)
		{
			referenced_target_point = pQueueNode;
			break;
		}

	}

	if(referenced_target_point != NULL)
	{
		return referenced_target_point;
	}	
	else
	{
		return first_nonfiltered_target_point;
	}
}

static void Add_FeasibleTargetPoints_From_AnycastSet_To_GlobalTargetPointSet(target_point_queue_t *dst_vehicle_FTPQ, target_point_queue_t *FTPQ)
{ /* add target points from the anycast set for a destination vehicle (denoted as dst_vehicle_FTPQ) to the global target point set FTPQ. */
	target_point_queue_node_t* pQueueNode = NULL; //pointer to a target point queue node
	int i = 0; //loop index
	boolean flag = FALSE; //flag to check the duplication of target point addition

	/* check whether each target point in an anycast set can be added to FTPQ */ 
	pQueueNode = &(dst_vehicle_FTPQ->head);
	for(i = 0; i < dst_vehicle_FTPQ->size; i++)
	{
		pQueueNode = pQueueNode->next;

		/* search first_nonfiltered_target_point and referenced_target_point in FTPQ */
		if(pQueueNode->filter_flag == FALSE)
		{ //this target point is eligible for the addition to the global target point set
			/* check whether this target point is already in FTPQ to prevent the duplicate addition */
			flag = Is_TargetPoint_In_TargetQueue(pQueueNode->target_point_id, FTPQ);
			if(flag)
			{
				printf("Add_FeasibleTargetPoints_From_AnycastSet_To_GlobalTargetPointSet(): Duplicate Addtition Try: target_point(%d) is already in FTPQ\n", pQueueNode->target_point_id);
				exit(1);
			}
			
			/* enqueue the contents of pQueueNode into FTPQ */
			Enqueue((queue_t*)FTPQ, (queue_node_t*)pQueueNode);
		}
	}
}

void GO_Rebuild_AdjacencyMatrix_With_SpanningTree(double **T, int T_size, boolean *spanning_tree_node_flag_vector, int spanning_tree_node_flag_vector_size)
{ /* rebuild the adjacency matrix T with spanning_tree_node_flag_vector for the spanning tree where the vertex with flag TRUE is one of the spanning tree vertices; note that the index is the vertex id, so index 0 has no vertex */
	int i = 0, j = 0; //loop-indices
	boolean flag = FALSE;

	/* check the validity of the parameters */
	if((T_size + 1) !=  spanning_tree_node_flag_vector_size)
	{
		printf("GO_Rebuild_AdjacencyMatrix_With_SpanningTree(): Error: (T_size + 1) !=  spanning_tree_node_flag_vector_size where T_size=%d and spanning_tree_node_flag_vector_size=%d\n", T_size, spanning_tree_node_flag_vector_size);
		exit(1);
	}

	/* prune the edges not belonging to spanning tree from the steiner tree T */
	for(i = 0; i < T_size; i++)
	{
		/* check whether vertex id of i+1 belongs to the spanning tree or not */
		flag = spanning_tree_node_flag_vector[i+1];

		if(flag == FALSE)
		{ /* invalidate the directed edges related to vertex (i+1) */
			for(j = 0; j < T_size; j++)
			{
				if(i == j)
					continue;

				T[i][j] = INF;
				T[j][i] = INF;
			}
		}
	}
}

void GO_Construct_EdgeQueue(parameter_t *param, double current_time, target_point_queue_t *FTPQ, edge_set_queue_t *ESQ)
{ /* construct edge set queue ESQ with Minimum Steiner Tree (MST) based on FTPQ */
	double **T = param->vanet_table.Tr_mcast;
	int T_size = param->vanet_table.matrix_size_for_mcast_in_Gr;

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
}

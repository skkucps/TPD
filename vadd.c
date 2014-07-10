/**
 *  File: vadd.c
    Description: implementation of Vehicle-Assisted Data Delivery (VADD) scheme
    Date: 08/08/2008
    Update Date: 08/08/2008
    Maker: Jaehoon Jeong
*/

#include "stdafx.h"
#include "vadd.h" //parameter structure and related constants
#include "util.h" //utility funtions
#include "linear-algebra.h" //functions for linear algebraic systems 
#include "shortest-path.h"
#include "all-pairs-shortest-paths.h"
#include <math.h> //pow()
#include "tpd.h"

double VADD_Compute_Angle(struct_coordinate1_t *coord_1, struct_coordinate1_t *coord_2, struct_coordinate1_t *coord_3)
{ /* where coord_1 is the coordinate of the current intersection I_i, coord_2 is the coordinate of the neighboring intersection I_j, and coord_3 is the coordination of the destination; compute the angle theta_ij between the direction of road r_ij and the vector from the current intersection to the destination; theta_ij is an approximation of D_ij that is the expected delivery delay from intersection i to intersection j. */
  double theta_in_radian = 0; //angle in radians
  double theta_in_degree = 0; //angle in degrees
  double x1 = coord_1->x, x2 = coord_2->x, x3 = coord_3->x; //x-coordinates of coord_1, coord_2 and coord_3, respectively
  double y1 = coord_1->y, y2 = coord_2->y, y3 = coord_3->y; //y-coordinates of coord_1, coord_2 and coord_3, respectively 
  double denominator = 0; //denominator for cos(theta)
  double numerator = 0; //numerator for cos(theta)
  double cos_theta = 0; //the value of cos(theta)
  
  denominator = sqrt(pow(x2-x1, 2) + pow(y2-y1, 2)) * sqrt(pow(x3-x1, 2) + pow(y3-y1, 2)); 
  //sqrt((x2-x1)^2 + (y2-y1)^2) * sqrt((x3-x1)^2 + (y3-y1)^2)

  numerator = (x2-x1)*(x3-x1) + (y2-y1)*(y3-y1);

  if(denominator == 0)
  {
    if(x1 == x3 && y1 == y3) //the current intersection is equal to one of the access points
    { //When one point corresponds to one point of a vector's endpoints, we regard the angle between one point and a vector as zero. Thus, cos(theta) = 1 where theta = 0. 
      cos_theta = 1;
    }
    else
    {
      printf("VADD_Computer_Angle(): Error: denominator is zero!\n");
      exit(1);
    }
  }
  else
  {
    cos_theta = numerator/denominator;

    if(cos_theta < -1 || cos_theta > 1)
    {
      printf("VADD_Computer_Angle(): Error: cos_theta(%f) must fall within the range -1 to 1\n", (float)cos_theta);
      exit(1);
    }
  }

  /* obtain the angle theta in radians using inverse function acos */
  theta_in_radian = acos(cos_theta);

  /* convert the theta in radians into the value in degrees */
  theta_in_degree = 180/PI*theta_in_radian;

  return theta_in_degree;
}

void VADD_Compute_Forwarding_Probability(parameter_t *param, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, int ap_table_index)
{ //compute the forwarding probability at each intersection using road graph G along with access point table ap_table
	angle_queue_t *M = NULL; //pointer to the angle queue related to a tail_node towards its neighboring nodes head_node
	angle_queue_node_t angle_node; //angle queue node
	angle_queue_node_t *pAngleNode = NULL; //pointer to an angle node
	angle_queue_node_t *pAngleNode2 = NULL; //pointer to an angle node  
	struct_graph_node *pGraphNode = NULL; //pointer to a graph node
	struct_graph_node *pNeighborNode = NULL; //pointer to the neighbor graph node of a graph node  
	struct_graph_node *pAP_Node = NULL; //pointer to the graph node corresponding to the access point
	int ap_node_id = 0; //access point node id, assuming that there is only one access point in this version; note that we need to consider multiple access points for the Internet connectivity later.
	int neighbor_node_id = 0; //neighbor node id in road graph

	int i = 0, k = 0, s = 0, h = 0; //indices for for-loop
	int m = 0; //number of neighboring nodes in graph G
	double T = 0; //contacting time
	double lambda = 0; //vehicle arrival rate
	double I = 0; //vehicle interarrival time
	struct_coordinate1_t *coord_1, *coord_2, *coord_3; //coord_1 is the coordinate of the current intersection I_i, coord_2 is the coordinate of the neighboring intersection I_j, and coord_3 is the coordination of the destination
	double CP = 0; //contacting probability
	double CP_step = 0; //contacting probability of the current step
	double P = 0; //forwarding probability including the probability that the original packet carrier continues to carry packet without actual forwarding
	double P_pure = 0; //pure forwarding probability excluding the probability that the original packet carrier continues to carry packet without actual forwarding
	double P_cond = 0; //conditional probability

	double **D_edd = NULL; //weight matrix for all-pairs shortest paths in terms of Expected Delivery Delay (EDD) in real graph Gr
	int **M_edd = NULL; //predecessor matrix for all-pairs shortest paths in terms of EDD in real graph G
	double **S_edd = NULL; //supplementary metric matrix for all-pairs shortest paths in terms of EDD in real graph G
	int matrix_size_for_edd_in_G = 0; //matrix size of matrices D_move and M_move for EDD in G

	double **D_move = NULL; //weight matrix for all-pairs shortest paths in terms of geographically travel distance in real graph G
	int **M_move = NULL; //predecessor matrix for all-pairs shortest paths in terms of distance in real graph G
	int matrix_size_for_move_in_G = 0; //matrix size of maTrices D_move and M_move for distance in G

	int u = 0, v = 0, w = 0; //indicies for the matrix for the all-pairs-shortest-path for graph G

	conditional_forwarding_probability_queue_t *Q = NULL; //pointer to a conditional forwarding probability queue
	conditional_forwarding_probability_queue_node_t *pQueueNode = NULL; //pointer to a conditional forwarding probability queue node
  
	/* obtain ap node id from ap_table */
	if(ap_table->number <= 0)
	{
		printf("VADD_Compute_Forwarding_Probability(): Error: ap_table->number(%d) must be greater than zero!\n", ap_table->number);
		exit(1);
	}

	/* get ap_node_id and pAP_Node for the ap_table entry corresponding to ap_table_index */
	ap_node_id = atoi(ap_table->list[ap_table_index].vertex);
	pAP_Node = &(G[ap_node_id-1]);
	/**************************************************************************************/

	/** construct data structures according to data forwarding link selection */
	switch(param->data_forwarding_link_selection)
	{        
		case FORWARDING_LINK_SELECTION_ANGLE: //link selection based on the angle between the vehicle movement vector and the destination vector
			break;

		case FORWARDING_LINK_SELECTION_DISTANCE: //link selection based on the geographic distance between the vehicle position and the target point, such as access point and target vehicle
			/** Compute the shortest path length in terms of the geographically travel distance in G */
			/* allocate the matrices for the edge length in G */
			matrix_size_for_move_in_G = G_size;
			Floyd_Warshall_Allocate_Matrices_For_Movement(&D_move, &M_move, matrix_size_for_move_in_G);

			/** construct the shortest path weight matrix D_move and predecessor matrix M_move in terms of distance */
			Floyd_Warshall_Construct_Matrices_For_Movement(G, G_size, &D_move, &M_move, &matrix_size_for_move_in_G);
			break;

		case FORWARDING_LINK_SELECTION_DELAY: //link selection based on the aggregated link delay between the vehicle position and the target point, such as access point and target vehicle
			/** Compute the shortest path length in terms of delivery delay in G */
			/* allocate the matrices for Expected Delivery Delay (EDD) in G */
			matrix_size_for_edd_in_G = G_size;
			Floyd_Warshall_Allocate_Matrices_For_EDD(&D_edd, &M_edd, &S_edd, matrix_size_for_edd_in_G);

			/** construct the shortest path weight matrix D_edd and predecessor matrix M_edd in terms of delivery delay */
			Floyd_Warshall_Construct_Matrices_For_EDD(G, G_size, &D_edd, &M_edd, &S_edd, &matrix_size_for_edd_in_G, param);
			break;

		default:
			printf("VADD_Compute_Forwarding_Probability(): param->data_forwarding_link_selection)(%d) is not supported!\n", param->data_forwarding_link_selection);
			exit(1);
	}
  
	/** compute the forwarding probability for the neighbor intersections per intersection */
	for(i = 0; i < G_size; i++) //for-1
	{
		m = (int)G[i].weight;
		pGraphNode = &(G[i]);
		pNeighborNode = pGraphNode;

		/* get the pointer to the angle queue for pGraphNode */
		M = pGraphNode->angle_queue;

		if(M->size > 0)
			DestroyQueue((queue_t*)M); //destory angle queue M

		/** compute thr average forwarding probability along with the conditional forwarding probability for the edges where pGraphNode is tail_node */
		for(k = 0; k < m; k++) //for-2
		{     
			/* get the pointer to the k-th neighboring node */
			pNeighborNode = pNeighborNode->next;

			/* compute the vehicle arrival rate lambda */
			if(pNeighborNode->number_of_interarrivals == 0)
			{
#ifdef __DEBUG_LEVEL_VADD_COMPUTE_FORWARDING_PROBABILITY__
				printf("VADD_Compute_Forwarding_Probability(): pNeighborNode->number_of_interarrivals is zero!\n");
#endif

				I = INF;
				pNeighborNode->mean_interarrival_time = I; //set up mean interarrival time for this road segment of <pGraphNode->vertex, pNeighborNode->vertex>	
			}
			else
			{
				I = pNeighborNode->sum_of_interarrival_time / pNeighborNode->number_of_interarrivals;
				pNeighborNode->mean_interarrival_time = I; //set up mean interarrival time for this road segment of <pGraphNode->vertex, pNeighborNode->vertex>
			}

			if(I < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
			{
				printf("VADD_Compute_Forwarding_Probability(): Error: vehicle average interarrival time I is zero!\n");
				exit(1);
			}

			/* set lambda with mean interarrival time */
			lambda = VADD_Vehicle_Lambda_2(I); //lambda = 1/I where I is mean interarrival time
			pNeighborNode->lambda = lambda;

			/* compute the contacting time */
			T = VADD_Contacting_Time(param->communication_range, param->vehicle_speed);

			/* compute the contacting probability */
			pNeighborNode->CP = VADD_CP(lambda, T);

			/** compute the value to sort the elements in queue M, such as angle, geographic shortest path, and aggregated-link-delay shortest path */
			switch(param->data_forwarding_link_selection)
			{        
				case FORWARDING_LINK_SELECTION_ANGLE: //link selection based on the angle between the vehicle movement vector and the destination vector
					coord_1 = &(pGraphNode->coordinate);
					coord_2 = &(pNeighborNode->gnode->coordinate);
					coord_3 = &(pAP_Node->coordinate);
					pNeighborNode->theta = VADD_Compute_Angle(coord_1, coord_2, coord_3); //compute the angle between the vehicle movement vector and the destination vector
					break;

				case FORWARDING_LINK_SELECTION_DISTANCE: //link selection based on the geographic distance between the vehicle position and the target point, such as access point and target vehicle
					u = atoi(G[i].vertex) - 1;
					v = atoi(pNeighborNode->vertex) - 1;	
					w = ap_node_id - 1; //index for the target point, such as AP

					pNeighborNode->theta = D_move[u][v] + D_move[v][w]; //the shortest path distance from u to w via v
					break;

				case FORWARDING_LINK_SELECTION_DELAY: //link selection based on the aggregated link delay between the vehicle position and the target point, such as access point and target vehicle
					u = atoi(G[i].vertex) - 1;
					v = atoi(pNeighborNode->vertex) - 1;	
					w = ap_node_id - 1; //index for the target point, such as AP

					pNeighborNode->theta = D_edd[u][v] + D_edd[v][w]; //the shortest aggregated link delay path from u to w via v
					break;

				default:
					printf("VADD_Compute_Forwarding_Probability(): param->data_forwarding_link_selection)(%d) is not supported!\n", param->data_forwarding_link_selection);
					exit(1);
			}

			/* initialize angle node */
			strcpy(angle_node.tail_node, pGraphNode->vertex);
			strcpy(angle_node.head_node, pNeighborNode->vertex);
			angle_node.theta = pNeighborNode->theta; //primary key for ascending sorting
			angle_node.CP = pNeighborNode->CP; //secondary key for ascending sorting
			angle_node.tail_gnode = pGraphNode; //pointer to the graph node corresponding to tail_node in edge <tail_node, head_node>
			angle_node.head_gnode = pNeighborNode; //pointer to the neighbor graph node corresponding to head_node in edge <tail_node, head_node>
      
			/* insert the angle node into angle queue M */
			Enqueue((queue_t*)M, (queue_node_t*)&angle_node);
		} //end of for-2

		/* sort the angle queue nodes in queue M in non-decreasing order according to the angles; 
			We need to sort the nodes with the same angle by the secondary key CPs; 
			note that this secondary-key sorting is not implemented yext. */
		SortAngleQueue(M);

		/* compute the unconditional forwarding probability P_prime */
		pAngleNode = &(M->head);
		for(k = 0; k < m; k++) //for-3
		{
			pAngleNode = pAngleNode->next;
			pAngleNode2 = &(M->head);
			CP = pAngleNode->head_gnode->CP;
			for(s = 0; s < k; s++) //for-4
			{
				pAngleNode2 = pAngleNode2->next;
				CP_step = pAngleNode2->head_gnode->CP;
				CP *= (1 - CP_step);
			} //end of for-4
			pAngleNode->head_gnode->P_prime = CP; //set the unconditional forwarding probability to P_prime
		} //end of for-3

		/* compute the branch probability */
		pAngleNode = &(M->head);
		for(k = 0; k < m; k++) //for-5
		{
			pAngleNode = pAngleNode->next;

			if(pGraphNode->number_of_arrivals == 0)
			{
				pAngleNode->head_gnode->Q = 0;
				printf("VADD_Compute_Forwarding_Probability(): Branch probability of edge <%s,%s> is zero!\n", pAngleNode->tail_node, pAngleNode->head_node);
			}
			else
				pAngleNode->head_gnode->Q = (double)pAngleNode->head_gnode->number_of_branching/pGraphNode->number_of_arrivals;
		} //end of for-5

		/* compute the forwarding probability P at intersection I_i */
		pAngleNode = &(M->head);
		for(k = 0; k < m; k++) //for-6
		{     
			/* get the pointer to the k-th neighboring node in angle queue M */
			pAngleNode = pAngleNode->next;
			P = 0;
			pAngleNode2 = &(M->head);
			for(h = 0; h < m; h++)
			{
				pAngleNode2 = pAngleNode2->next;
				P_cond = VADD_Compute_Conditional_Probability_For_Forwarding_Probability(M, k, h, pAngleNode); //compute conditional probability that a vehicle moving in road r_ih will forward a packet to another vehicle moving towards intersection I_k at intersection I_i

				/* set the conditional forwarding probability P_cond to the conditional forwarding probability of the pair of the next carrier edge k given the current carrier edge h */
				Q = pAngleNode2->head_gnode->conditional_forwarding_probability_queue;
				pQueueNode = GetConditionalForwardingProbabilityQueueNode(Q, pAngleNode->tail_node, pAngleNode->head_node); //get the pointer to a conditional forwarding probability queue node corresponding to the edge (pAngleNode->tail_node, pAngleNode->head_node) given the current carrier edge pointed by pAngleNode2 with the current carrier edge's conditional forwarding probability queue Q 

				pQueueNode->conditional_forwarding_probability = P_cond;

				/* accumulate the conditional forwarding probability into the forwarding probability */
				P += pAngleNode2->head_gnode->Q * P_cond;
			}
			pAngleNode->head_gnode->P = P; //set the forwarding probability to neighbor node's P 
		} //end of for-6

		/* compute the pure forwarding probability P_pure at intersection I_i:
Note that this pure forwarding probability excludes the case where the carrier towards I_h keeps carrying the packet without forwarding. */
		pAngleNode = &(M->head);
		for(k = 0; k < m; k++) //for-7
		{     
			/* get the pointer to the k-th neighboring node in angle queue M */
			pAngleNode = pAngleNode->next;
			P_pure = 0;
			pAngleNode2 = &(M->head);
			for(h = 0; h < m; h++)
			{
				pAngleNode2 = pAngleNode2->next;
				P_cond = VADD_Compute_Conditional_Probability_For_Pure_Forwarding_Probability(M, k, h, pAngleNode); //compute conditional probability that a vehicle moving in road r_ih will forward a packet to another vehicle moving towards intersection I_k at intersection I_i
				P_pure += pAngleNode2->head_gnode->Q * P_cond;
			}
			pAngleNode->head_gnode->P_pure = P_pure; //set the pure forwarding probability to neighbor node's P_pure 
		} //end of for-7

		/* compute the pure forwarding probability P_prime_pure for TBD at intersection I_i */
		pAngleNode = &(M->head);
		for(k = 0; k < m; k++) //for-7
		{     
			/* get the pointer to the k-th neighboring node in angle queue M */
			pAngleNode = pAngleNode->next;
			pAngleNode->head_gnode->P_prime_pure = VADD_Compute_Conditional_Probability_For_Pure_Forwarding_Probability(M, k, h, pAngleNode); //compute conditional probability in TBD that a vehicle moving in road r_ih according to its trajectory will forward a packet to another vehicle moving towards intersection I_k at intersection I_i;
		} //end of for-7
	} //end of for-1

	/** delete data structures according to data forwarding link selection */
	switch(param->data_forwarding_link_selection)
	{        
		case FORWARDING_LINK_SELECTION_ANGLE: //link selection based on the angle between the vehicle movement vector and the destination vector
			break;

		case FORWARDING_LINK_SELECTION_DISTANCE: //link selection based on the geographic distance between the vehicle position and the target point, such as access point and target vehicle
			/** free matrices for EDD in Gr */
			Floyd_Warshall_Free_Matrices_For_Movement(&D_move, &M_move, &matrix_size_for_move_in_G);
			break;

		case FORWARDING_LINK_SELECTION_DELAY: //link selection based on the aggregated link delay between the vehicle position and the target point, such as access point and target vehicle
			/** free matrices for EDD in Gr */
			Floyd_Warshall_Free_Matrices_For_EDD(&D_edd, &M_edd, &S_edd, &matrix_size_for_edd_in_G);
			break;

		default:
			printf("VADD_Compute_Forwarding_Probability(): param->data_forwarding_link_selection)(%d) is not supported!\n", param->data_forwarding_link_selection);
			exit(1);
	}
}

void VADD_Compute_Forwarding_Probability_For_Target_Intersection(parameter_t *param, struct_graph_node *G, int G_size, int target_intersection_index)
{ //compute the forwarding probability at each intersection using road graph G for a target intersection whose intersection id is (target_intersection_index+1)
	angle_queue_t *M = NULL; //pointer to the angle queue related to a tail_node towards its neighboring nodes head_node
	angle_queue_node_t angle_node; //angle queue node
	angle_queue_node_t *pAngleNode = NULL; //pointer to an angle node
	angle_queue_node_t *pAngleNode2 = NULL; //pointer to an angle node  
	struct_graph_node *pGraphNode = NULL; //pointer to a graph node
	struct_graph_node *pNeighborNode = NULL; //pointer to the neighbor graph node of a graph node  
	struct_graph_node *pTargetNode = NULL; //pointer to the graph node corresponding to the target intersection
	int target_node_id = 0; //target intersection's node id
	int neighbor_node_id = 0; //neighbor node id in road graph

	int i = 0, k = 0, s = 0, h = 0; //indices for for-loop
	int m = 0; //number of neighboring nodes in graph G
	double T = 0; //contacting time
	double lambda = 0; //vehicle arrival rate
	double I = 0; //vehicle interarrival time
	struct_coordinate1_t *coord_1, *coord_2, *coord_3; //coord_1 is the coordinate of the current intersection I_i, coord_2 is the coordinate of the neighboring intersection I_j, and coord_3 is the coordination of the destination
	double CP = 0; //contacting probability
	double CP_step = 0; //contacting probability of the current step
	double P = 0; //forwarding probability including the probability that the original packet carrier continues to carry packet without actual forwarding
	double P_pure = 0; //pure forwarding probability excluding the probability that the original packet carrier continues to carry packet without actual forwarding
	double P_cond = 0; //conditional probability

	double **D_edd = NULL; //weight matrix for all-pairs shortest paths in terms of Expected Delivery Delay (EDD) in real graph Gr
	int **M_edd = NULL; //predecessor matrix for all-pairs shortest paths in terms of EDD in real graph G
	double **S_edd = NULL; //supplementary metric matrix for all-pairs shortest paths in terms of EDD in real graph G
	int matrix_size_for_edd_in_G = 0; //matrix size of matrices D_move and M_move for EDD in G

	double **D_move = NULL; //weight matrix for all-pairs shortest paths in terms of geographically travel distance in real graph G
	int **M_move = NULL; //predecessor matrix for all-pairs shortest paths in terms of distance in real graph G
	int matrix_size_for_move_in_G = 0; //matrix size of maTrices D_move and M_move for distance in G

	int u = 0, v = 0, w = 0; //indicies for the matrix for the all-pairs-shortest-path for graph G

	conditional_forwarding_probability_queue_t *Q = NULL; //pointer to a conditional forwarding probability queue
	conditional_forwarding_probability_queue_node_t *pQueueNode = NULL; //pointer to a conditional forwarding probability queue node
  
	/* get target_node_id and pTargetNode for target_intersection_index */
	target_node_id = target_intersection_index + 1;
	pTargetNode = &(G[target_node_id-1]);
	/**************************************************************************************/

	/** construct data structures according to data forwarding link selection */
	switch(param->data_forwarding_link_selection)
	{        
		case FORWARDING_LINK_SELECTION_ANGLE: //link selection based on the angle between the vehicle movement vector and the destination vector
			break;

		case FORWARDING_LINK_SELECTION_DISTANCE: //link selection based on the geographic distance between the vehicle position and the target point, such as access point and target vehicle
			/** Compute the shortest path length in terms of the geographically travel distance in G */
			/* allocate the matrices for the edge length in G */
			matrix_size_for_move_in_G = G_size;
			Floyd_Warshall_Allocate_Matrices_For_Movement(&D_move, &M_move, matrix_size_for_move_in_G);

			/** construct the shortest path weight matrix D_move and predecessor matrix M_move in terms of distance */
			Floyd_Warshall_Construct_Matrices_For_Movement(G, G_size, &D_move, &M_move, &matrix_size_for_move_in_G);
			break;

		case FORWARDING_LINK_SELECTION_DELAY: //link selection based on the aggregated link delay between the vehicle position and the target point, such as access point and target vehicle
			/** Compute the shortest path length in terms of delivery delay in G */
			/* allocate the matrices for Expected Delivery Delay (EDD) in G */
			matrix_size_for_edd_in_G = G_size;
			Floyd_Warshall_Allocate_Matrices_For_EDD(&D_edd, &M_edd, &S_edd, matrix_size_for_edd_in_G);

			/** construct the shortest path weight matrix D_edd and predecessor matrix M_edd in terms of delivery delay */
			Floyd_Warshall_Construct_Matrices_For_EDD(G, G_size, &D_edd, &M_edd, &S_edd, &matrix_size_for_edd_in_G, param);
			break;

		default:
			printf("%s:%d: param->data_forwarding_link_selection)(%d) is not supported!\n", 
					__FUNCTION__, __LINE__,
					param->data_forwarding_link_selection);
			exit(1);
	}
  
	/** compute the forwarding probability for the neighbor intersections per intersection */
	for(i = 0; i < G_size; i++) //for-1
	{
		m = (int)G[i].weight;
		pGraphNode = &(G[i]);
		pNeighborNode = pGraphNode;

		/* get the pointer to the angle queue for pGraphNode */
		M = pGraphNode->angle_queue;

		if(M->size > 0)
			DestroyQueue((queue_t*)M); //destory angle queue M

		/** compute thr average forwarding probability along with the conditional forwarding probability for the edges where pGraphNode is tail_node */
		for(k = 0; k < m; k++) //for-2
		{     
			/* get the pointer to the k-th neighboring node */
			pNeighborNode = pNeighborNode->next;

			/* compute the vehicle arrival rate lambda */
			if(pNeighborNode->number_of_interarrivals == 0)
			{
#ifdef __DEBUG_LEVEL_VADD_COMPUTE_FORWARDING_PROBABILITY__
				printf("%s:%d: pNeighborNode->number_of_interarrivals is zero!\n",
						__FUNCTION__, __LINE__);
#endif

				I = INF;
				pNeighborNode->mean_interarrival_time = I; //set up mean interarrival time for this road segment of <pGraphNode->vertex, pNeighborNode->vertex>	
			}
			else
			{
				I = pNeighborNode->sum_of_interarrival_time / pNeighborNode->number_of_interarrivals;
				pNeighborNode->mean_interarrival_time = I; //set up mean interarrival time for this road segment of <pGraphNode->vertex, pNeighborNode->vertex>
			}

			if(I < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
			{
				printf("%s:%d: Error: vehicle average interarrival time I is zero!\n",
						__FUNCTION__, __LINE__);
				exit(1);
			}

			/* set lambda with mean interarrival time */
			lambda = VADD_Vehicle_Lambda_2(I); //lambda = 1/I where I is mean interarrival time
			pNeighborNode->lambda = lambda;

			/* compute the contacting time */
			T = VADD_Contacting_Time(param->communication_range, param->vehicle_speed);

			/* compute the contacting probability */
			pNeighborNode->CP = VADD_CP(lambda, T);

			/** compute the value to sort the elements in queue M, such as angle, geographic shortest path, and aggregated-link-delay shortest path */
			switch(param->data_forwarding_link_selection)
			{        
				case FORWARDING_LINK_SELECTION_ANGLE: //link selection based on the angle between the vehicle movement vector and the destination vector
					coord_1 = &(pGraphNode->coordinate);
					coord_2 = &(pNeighborNode->gnode->coordinate);
					coord_3 = &(pTargetNode->coordinate);
					pNeighborNode->theta = VADD_Compute_Angle(coord_1, coord_2, coord_3); //compute the angle between the vehicle movement vector and the destination vector
					break;

				case FORWARDING_LINK_SELECTION_DISTANCE: //link selection based on the geographic distance between the vehicle position and the target point, such as access point and target vehicle
					u = atoi(G[i].vertex) - 1;
					v = atoi(pNeighborNode->vertex) - 1;	
					w = target_node_id - 1; //index for the target point, such as target intersection

					pNeighborNode->theta = D_move[u][v] + D_move[v][w]; //the shortest path distance from u to w via v
					break;

				case FORWARDING_LINK_SELECTION_DELAY: //link selection based on the aggregated link delay between the vehicle position and the target point, such as access point and target vehicle
					u = atoi(G[i].vertex) - 1;
					v = atoi(pNeighborNode->vertex) - 1;	
					w = target_node_id - 1; //index for the target point, such as target intersection

					pNeighborNode->theta = D_edd[u][v] + D_edd[v][w]; //the shortest aggregated link delay path from u to w via v
					break;

				default:
					printf("%s:%d: param->data_forwarding_link_selection)(%d) is not supported!\n", 
							__FUNCTION__, __LINE__,
							param->data_forwarding_link_selection);
					exit(1);
			}

			/* initialize angle node */
			strcpy(angle_node.tail_node, pGraphNode->vertex);
			strcpy(angle_node.head_node, pNeighborNode->vertex);
			angle_node.theta = pNeighborNode->theta; //primary key for ascending sorting
			angle_node.CP = pNeighborNode->CP; //secondary key for ascending sorting
			angle_node.tail_gnode = pGraphNode; //pointer to the graph node corresponding to tail_node in edge <tail_node, head_node>
			angle_node.head_gnode = pNeighborNode; //pointer to the neighbor graph node corresponding to head_node in edge <tail_node, head_node>
      
			/* insert the angle node into angle queue M */
			Enqueue((queue_t*)M, (queue_node_t*)&angle_node);
		} //end of for-2

		/* sort the angle queue nodes in queue M in non-decreasing order according to the angles; 
			We need to sort the nodes with the same angle by the secondary key CPs; 
			note that this secondary-key sorting is not implemented yext. */
		SortAngleQueue(M);

		/* compute the unconditional forwarding probability P_prime */
		pAngleNode = &(M->head);
		for(k = 0; k < m; k++) //for-3
		{
			pAngleNode = pAngleNode->next;
			pAngleNode2 = &(M->head);
			CP = pAngleNode->head_gnode->CP;
			for(s = 0; s < k; s++) //for-4
			{
				pAngleNode2 = pAngleNode2->next;
				CP_step = pAngleNode2->head_gnode->CP;
				CP *= (1 - CP_step);
			} //end of for-4
			pAngleNode->head_gnode->P_prime = CP; //set the unconditional forwarding probability to P_prime
		} //end of for-3

		/* compute the branch probability */
		pAngleNode = &(M->head);
		for(k = 0; k < m; k++) //for-5
		{
			pAngleNode = pAngleNode->next;

			if(pGraphNode->number_of_arrivals == 0)
			{
				pAngleNode->head_gnode->Q = 0;
				printf("%s:%d: Branch probability of edge <%s,%s> is zero!\n", 
						__FUNCTION__, __LINE__,
						pAngleNode->tail_node, pAngleNode->head_node);
			}
			else
				pAngleNode->head_gnode->Q = (double)pAngleNode->head_gnode->number_of_branching/pGraphNode->number_of_arrivals;
		} //end of for-5

		/* compute the forwarding probability P at intersection I_i */
		pAngleNode = &(M->head);
		for(k = 0; k < m; k++) //for-6
		{     
			/* get the pointer to the k-th neighboring node in angle queue M */
			pAngleNode = pAngleNode->next;
			P = 0;
			pAngleNode2 = &(M->head);
			for(h = 0; h < m; h++)
			{
				pAngleNode2 = pAngleNode2->next;
				P_cond = VADD_Compute_Conditional_Probability_For_Forwarding_Probability(M, k, h, pAngleNode); //compute conditional probability that a vehicle moving in road r_ih will forward a packet to another vehicle moving towards intersection I_k at intersection I_i

				/* set the conditional forwarding probability P_cond to the conditional forwarding probability of the pair of the next carrier edge k given the current carrier edge h */
				Q = pAngleNode2->head_gnode->conditional_forwarding_probability_queue;
				pQueueNode = GetConditionalForwardingProbabilityQueueNode(Q, pAngleNode->tail_node, pAngleNode->head_node); //get the pointer to a conditional forwarding probability queue node corresponding to the edge (pAngleNode->tail_node, pAngleNode->head_node) given the current carrier edge pointed by pAngleNode2 with the current carrier edge's conditional forwarding probability queue Q 

				pQueueNode->conditional_forwarding_probability = P_cond;

				/* accumulate the conditional forwarding probability into the forwarding probability */
				P += pAngleNode2->head_gnode->Q * P_cond;
			}
			pAngleNode->head_gnode->P = P; //set the forwarding probability to neighbor node's P 
		} //end of for-6

		/* compute the pure forwarding probability P_pure at intersection I_i:
Note that this pure forwarding probability excludes the case where the carrier towards I_h keeps carrying the packet without forwarding. */
		pAngleNode = &(M->head);
		for(k = 0; k < m; k++) //for-7
		{     
			/* get the pointer to the k-th neighboring node in angle queue M */
			pAngleNode = pAngleNode->next;
			P_pure = 0;
			pAngleNode2 = &(M->head);
			for(h = 0; h < m; h++)
			{
				pAngleNode2 = pAngleNode2->next;
				P_cond = VADD_Compute_Conditional_Probability_For_Pure_Forwarding_Probability(M, k, h, pAngleNode); //compute conditional probability that a vehicle moving in road r_ih will forward a packet to another vehicle moving towards intersection I_k at intersection I_i
				P_pure += pAngleNode2->head_gnode->Q * P_cond;
			}
			pAngleNode->head_gnode->P_pure = P_pure; //set the pure forwarding probability to neighbor node's P_pure 
		} //end of for-7

		/* compute the pure forwarding probability P_prime_pure for TBD at intersection I_i */
		pAngleNode = &(M->head);
		for(k = 0; k < m; k++) //for-7
		{     
			/* get the pointer to the k-th neighboring node in angle queue M */
			pAngleNode = pAngleNode->next;
			pAngleNode->head_gnode->P_prime_pure = VADD_Compute_Conditional_Probability_For_Pure_Forwarding_Probability(M, k, h, pAngleNode); //compute conditional probability in TBD that a vehicle moving in road r_ih according to its trajectory will forward a packet to another vehicle moving towards intersection I_k at intersection I_i;
		} //end of for-7
	} //end of for-1

	/** delete data structures according to data forwarding link selection */
	switch(param->data_forwarding_link_selection)
	{        
		case FORWARDING_LINK_SELECTION_ANGLE: //link selection based on the angle between the vehicle movement vector and the destination vector
			break;

		case FORWARDING_LINK_SELECTION_DISTANCE: //link selection based on the geographic distance between the vehicle position and the target point, such as access point and target vehicle
			/** free matrices for EDD in Gr */
			Floyd_Warshall_Free_Matrices_For_Movement(&D_move, &M_move, &matrix_size_for_move_in_G);
			break;

		case FORWARDING_LINK_SELECTION_DELAY: //link selection based on the aggregated link delay between the vehicle position and the target point, such as access point and target vehicle
			/** free matrices for EDD in Gr */
			Floyd_Warshall_Free_Matrices_For_EDD(&D_edd, &M_edd, &S_edd, &matrix_size_for_edd_in_G);
			break;

		default:
			printf("%s:%d: param->data_forwarding_link_selection)(%d) is not supported!\n", 
					__FUNCTION__, __LINE__,
					param->data_forwarding_link_selection);
			exit(1);
	}
}

void VADD_Recompute_Forwarding_Probability(parameter_t *param, struct_graph_node *G, int G_size)
{ //recompute the forwarding probability at each intersection using road graph G with the computed EDD for each edge
  //angle_queue_t M; //angle queue
  angle_queue_t *M = NULL; //pointer to the angle queue related to a tail_node towards its neighboring nodes head_node
  angle_queue_node_t angle_node; //angle queue node
  angle_queue_node_t *pAngleNode = NULL; //pointer to an angle node
  angle_queue_node_t *pAngleNode2 = NULL; //pointer to an angle node  
  struct_graph_node *pGraphNode = NULL; //pointer to a graph node
  struct_graph_node *pNeighborNode = NULL; //pointer to the neighbor graph node of a graph node  
  struct_graph_node *pAP_Node = NULL; //pointer to the graph node corresponding to the access point
  int ap_node_id = 0; //access point node id, assuming that there is only one access point in this version; note that we need to consider multiple access points for the Internet connectivity later.
  int neighbor_node_id = 0; //neighbor node id in road graph

  int i = 0, k = 0, s = 0, h = 0; //indices for for-loop
  int m = 0; //number of neighboring nodes in graph G
  double T = 0; //contacting time
  double lambda = 0; //vehicle arrival rate
  double I = 0; //vehicle interarrival time
  struct_coordinate1_t *coord_1, *coord_2, *coord_3; //coord_1 is the coordinate of the current intersection I_i, coord_2 is the coordinate of the neighboring intersection I_j, and coord_3 is the coordination of the destination
  double CP = 0; //contacting probability
  double CP_step = 0; //contacting probability of the current step
  double P = 0; //forwarding probability including the probability that the original packet carrier continues to carry packet without actual forwarding
  double P_pure = 0; //pure forwarding probability excluding the probability that the original packet carrier continues to carry packet without actual forwarding
  double P_cond = 0; //conditional probability

  ///* initialize angle queue M */
  //InitQueue((queue_t*)&M, QTYPE_ANGLE); 

  /** compute the forwarding probability for the neighbor intersections per intersection */
  for(i = 0; i < G_size; i++) //for-1
  {
    m = (int)G[i].weight;
    pGraphNode = &(G[i]);
    pNeighborNode = pGraphNode;

    /* get the pointer to the angle queue for pGraphNode */
    M = pGraphNode->angle_queue;

    if(M->size > 0)
      DestroyQueue((queue_t*)M); //destory angle queue M

    /** compute thr average forwarding probability along with the conditional forwarding probability for the edges where pGraphNode is tail_node */
    for(k = 0; k < m; k++) //for-2
    {     
      /* get the pointer to the k-th neighboring node */
      pNeighborNode = pNeighborNode->next;

      /* compute the vehicle arrival rate lambda */
      if(pNeighborNode->number_of_interarrivals == 0)
      {
#ifdef __DEBUG_LEVEL_VADD_RECOMPUTE_FORWARDING_PROBABILITY__
	printf("VADD_Recompute_Forwarding_Probability(): pNeighborNode->number_of_interarrivals is zero!\n");
#endif

	I = INF;
	pNeighborNode->mean_interarrival_time = I; //set up mean interarrival time for this road segment of <pGraphNode->vertex, pNeighborNode->vertex>	
      }
      else
      {
	I = pNeighborNode->sum_of_interarrival_time / pNeighborNode->number_of_interarrivals;
	pNeighborNode->mean_interarrival_time = I; //set up mean interarrival time for this road segment of <pGraphNode->vertex, pNeighborNode->vertex>
      }

      if(I < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
      //if(I == 0)
      {
	printf("VADD_Recompute_Forwarding_Probability(): Error: vehicle average interarrival time I is zero!\n");
	exit(1);
      }

      /* set lambda with mean interarrival time */
      lambda = VADD_Vehicle_Lambda_2(I); //lambda = 1/I where I is mean interarrival time
      pNeighborNode->lambda = lambda;

      /* compute the contacting time */
      T = VADD_Contacting_Time(param->communication_range, param->vehicle_speed);

      /* compute the contacting probability */
      pNeighborNode->CP = VADD_CP(lambda, T);

      /* set the EDD value to sort the elements in queue M */
      pNeighborNode->theta = pNeighborNode->EDD;

      /* initialize angle node */
      strcpy(angle_node.tail_node, pGraphNode->vertex);
      strcpy(angle_node.head_node, pNeighborNode->vertex);
      angle_node.theta = pNeighborNode->theta; //primary key for ascending sorting
      angle_node.CP = pNeighborNode->CP; //secondary key for ascending sorting
      angle_node.tail_gnode = pGraphNode; //pointer to the graph node corresponding to tail_node in edge <tail_node, head_node>
      angle_node.head_gnode = pNeighborNode; //pointer to the neighbor graph node corresponding to head_node in edge <tail_node, head_node>
      
      /* insert the angle node into angle queue M */
      Enqueue((queue_t*)M, (queue_node_t*)&angle_node);
    } //end of for-2

    /* sort the angle queue nodes in queue M in non-decreasing order according to the angles; 
       We need to sort the nodes with the same angle by the secondary key CPs; 
       note that this secondary-key sorting is not implemented yext. */
    SortAngleQueue(M);

    /* compute the unconditional forwarding probability P_prime */
    pAngleNode = &(M->head);
    for(k = 0; k < m; k++) //for-3
    {
      pAngleNode = pAngleNode->next;
      pAngleNode2 = &(M->head);
      CP = pAngleNode->head_gnode->CP;
      for(s = 0; s < k; s++) //for-4
      {
	pAngleNode2 = pAngleNode2->next;
	CP_step = pAngleNode2->head_gnode->CP;
	CP *= (1 - CP_step);
      } //end of for-4
      pAngleNode->head_gnode->P_prime = CP; //set the unconditional forwarding probability to P_prime
    } //end of for-3

    /* compute the branch probability */
    pAngleNode = &(M->head);
    for(k = 0; k < m; k++) //for-5
    {
      pAngleNode = pAngleNode->next;

      if(pGraphNode->number_of_arrivals == 0)
      {
	pAngleNode->head_gnode->Q = 0;
	printf("VADD_Recompute_Forwarding_Probability(): Branch probability of edge <%s,%s> is zero!\n", pAngleNode->tail_node, pAngleNode->head_node);
      }
      else
	pAngleNode->head_gnode->Q = (double)pAngleNode->head_gnode->number_of_branching/pGraphNode->number_of_arrivals;
    } //end of for-5

    /* compute the forwarding probability P at intersection I_i */
    pAngleNode = &(M->head);
    for(k = 0; k < m; k++) //for-6
    {     
      /* get the pointer to the k-th neighboring node in angle queue M */
      pAngleNode = pAngleNode->next;
      P = 0;
      pAngleNode2 = &(M->head);
      for(h = 0; h < m; h++)
      {
	pAngleNode2 = pAngleNode2->next;
	P_cond = VADD_Compute_Conditional_Probability_For_Forwarding_Probability(M, k, h, pAngleNode); //compute conditional probability that a vehicle moving in road r_ih will forward a packet to another vehicle moving towards intersection I_k at intersection I_i
	P += pAngleNode2->head_gnode->Q * P_cond;
      }
      pAngleNode->head_gnode->P = P; //set the forwarding probability to neighbor node's P 
    } //end of for-6

    /* compute the pure forwarding probability P_pure at intersection I_i:
       Note that this pure forwarding probability excludes the case where the carrier towards I_h keeps carrying the packet without forwarding. */
    pAngleNode = &(M->head);
    for(k = 0; k < m; k++) //for-7
    {     
      /* get the pointer to the k-th neighboring node in angle queue M */
      pAngleNode = pAngleNode->next;
      P_pure = 0;
      pAngleNode2 = &(M->head);
      for(h = 0; h < m; h++)
      {
	pAngleNode2 = pAngleNode2->next;
	P_cond = VADD_Compute_Conditional_Probability_For_Pure_Forwarding_Probability(M, k, h, pAngleNode); //compute conditional probability that a vehicle moving in road r_ih will forward a packet to another vehicle moving towards intersection I_k at intersection I_i
	P_pure += pAngleNode2->head_gnode->Q * P_cond;
      }
      pAngleNode->head_gnode->P_pure = P_pure; //set the pure forwarding probability to neighbor node's P_pure 
    } //end of for-7

    /* compute the pure forwarding probability P_prime_pure for TBD at intersection I_i */
    pAngleNode = &(M->head);
    for(k = 0; k < m; k++) //for-7
    {     
      /* get the pointer to the k-th neighboring node in angle queue M */
      pAngleNode = pAngleNode->next;
      pAngleNode->head_gnode->P_prime_pure = VADD_Compute_Conditional_Probability_For_Pure_Forwarding_Probability(M, k, h, pAngleNode); //compute conditional probability in TBD that a vehicle moving in road r_ih according to its trajectory will forward a packet to another vehicle moving towards intersection I_k at intersection I_i;
    } //end of for-7
    
    //DestroyQueue((queue_t*)&M); //destory angle queue M

  } //end of for-1
}

double VADD_Compute_Conditional_Probability_For_Forwarding_Probability(angle_queue_t *M, int k, int h, angle_queue_node_t *pAngleNode)
{ //compute conditional probability that a vehicle moving in road r_ih will forward (or continue to carry in the case of k == h) a packet to another vehicle moving towards intersection I_k at intersection I_i along with angle queue M where pAngleNode is the pointer to the angle node corresponding to k
  angle_queue_node_t *pAngleNode2 = NULL; //pointer to an angle node
  double P_cond = 0; //final conditional probability
  double sum = 0; //sum of conditional probability
  int m = M->size; //size of angle queue M
  int s = 0; //index of for-loop
  
  if(k < h)
  { //the probability below is the probability that the carrier will forward its packets to another vehicle moving on the better edge in terms of the EDD.
    P_cond = pAngleNode->head_gnode->P_prime;
  }
  else if(k == h)
  { //the probability below is the probability that the carrier will forward its packets to another vehicle moving to the same edge e_{ih} or continue to carry its packets with itself.
    pAngleNode2 = &(M->head);
    for(s = 0; s < k; s++)
    {
      pAngleNode2 = pAngleNode2->next;
      sum += pAngleNode2->head_gnode->P_prime;
    }
    P_cond = 1 - sum;
  }
  else
  {
    P_cond = 0;
  }

  return P_cond;
}

double VADD_Compute_Conditional_Probability_For_Pure_Forwarding_Probability(angle_queue_t *M, int k, int h, angle_queue_node_t *pAngleNode)
{ //compute conditional probability that a vehicle moving in road r_ih will only forward a packet to another vehicle moving towards intersection I_k at intersection I_i along with angle queue M where pAngleNode is the pointer to the angle node corresponding to k; Note that this pure forwarding probability excludes the case where the carrier towards I_h keeps carrying the packet without forwarding.
  angle_queue_node_t *pAngleNode2 = NULL; //pointer to an angle node
  double P_cond = 0; //final conditional probability
  //double sum = 0; //sum of conditional probability
  //int m = M->size; //size of angle queue M
  //int i = 0; //index of for-loop
  
  if(k <= h)
  {
    P_cond = pAngleNode->head_gnode->P_prime;
  }
  else
  {
    P_cond = 0;
  }

  return P_cond;
}

void VADD_Compute_EDD_Based_On_Stochastic_Model(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, int ap_table_index)
{ //compute the Expected Delivery Delay (EDD) based on the stochastic model with graph G and directional edge queue EQ.
  directional_edge_queue_node_t *pEdgeNode = NULL, *pEdgeNode2 = NULL;  //pointers to directional edge queue nodes
  delay_queue_t DQ; //delay queue DQ containing EDD for each road segment in road network
  delay_queue_node_t delay_node; //contains a queue of delay components of type
  delay_queue_node_t *pDelayNode = NULL; //pointer to delay queue node with delay_component_list containing forwarding probability P and EDD D per edge.
  delay_component_queue_node_t delay_component_node; //delay component node
  delay_component_queue_node_t *pDelayComponentNode = NULL; //pointer to delay component node
  int n = EQ->size; //number of road segments
  int i = 0, j = 0; //indices of for-loops
  struct_graph_node *pTailNode = NULL, *pHeadNode = NULL, *pNeighborNode = NULL; //pointers to graph nodes
  int head_id = 0; //id for a head node in the edge <tail_node, head_node>
  double **A = NULL; //n x (n+1) EDD matrix containing n x n forwarding probability matrix and edge delay vector
  int row_size = 0, column_size = 0; //row size and column size of a matrix
  int row_id = 0, column_id = 0; //row index i and column index j of the P_ij in the forwarding probability matrix P
  double P = 0; //forwarding probability for an edge
  double *x = NULL; //solution vector of size row_size
  int *p = NULL; //permutation vector of size row_size
  int label = 0; //label to determine P_ij and edge_delay

  /** Compute Forwarding Probability for each directional edge in G */
  VADD_Compute_Forwarding_Probability(param, G, G_size, ap_table, ap_table_index); //compute forwarding probability P per road segment

  /** Initialize Delay Queue DQ */
  InitQueue((queue_t*)&DQ, QTYPE_DELAY);

  /** sort the directional edge queue nodes in the increasing order of eid */
  SortDirectionalEdgeQueue(EQ);

  /** make a list for the linear combination of delay component multiplied by forwarding probability */
  pEdgeNode = &(EQ->head);
  for(i = 0; i < n; i++) //for-3
  {
    pEdgeNode = pEdgeNode->next;

    /* set up delay queue node:
       Note that eid cannot be used for variable indexing since eid keeps increasing during the graph modification for the virtual nodes */
    delay_node.order = pEdgeNode->order; //Note: delay variable id to determine the index of variable in matrix P
    delay_node.eid = pEdgeNode->eid;
    delay_node.edge_delay = VADD_Compute_Edge_Delay(param, pEdgeNode->head_gnode); 
    //compute the edge delay for the road segment r_ij
    delay_node.EDD = 0;
    delay_node.enode = pEdgeNode; //pointer to an edge node in directional edge queue EQ corresponding to eid

    /* set edge_delay to pEdgeNode->head_gnode->edge_delay */
    pEdgeNode->head_gnode->edge_delay = delay_node.edge_delay;

    pDelayNode = (delay_queue_node_t*)Enqueue((queue_t*)&DQ, (queue_node_t*)&delay_node); //enqueue a new delay node
    InitQueue((queue_t*)&(pDelayNode->delay_component_list), QTYPE_DELAY_COMPONENT);
    //initialize a delay component queue for delay components

    head_id = atoi(pEdgeNode->head_node);
    pHeadNode = &(G[head_id-1]);
    pNeighborNode = pHeadNode;
    for(j = 0; j < pHeadNode->weight; j++) //for-4
    {
      pNeighborNode = pNeighborNode->next;
      pEdgeNode2 = FastLookupDirectionalEdgeQueue(G, pHeadNode->vertex, pNeighborNode->vertex);
      //The graph node corresponding to the head node points to the directional edge node corresponding to edge <pHeadNode->vertex, pNeighborNode->vertex>.
      if(pEdgeNode2 == NULL)
      {
	printf("VADD_Compute_EDD_Based_On_Stochastic_Model(): pEdgeNode2 for <%s,%s> is NULL\n", pHeadNode->vertex, pNeighborNode->vertex);
	exit(1);
      }

      /* set up delay component queue node */
      delay_component_node.order = pEdgeNode2->order; //order is used to index an element in matrix P      
      delay_component_node.eid = pEdgeNode2->eid;
      delay_component_node.P = pEdgeNode2->head_gnode->P; //forwarding probability for the edge <pHeadNode->vertex, pNeighborNode->vertex>
      delay_component_node.enode = pEdgeNode2; //pointer to the directional edge node corresponding to eid

      Enqueue((queue_t*)&(pDelayNode->delay_component_list), (queue_node_t*)&delay_component_node);
    } //end of for-4
  } //end of for-3
  
  /** construct n x (n+1) matrix containing the forwarding probability matrix and edge delay vector */
  /* allocate the memory of the row_size x column_size matrix A of type double */
  row_size = n;
  column_size = n + 1;
  A = LA_Allocate_Matrix_Of_Type_Double(row_size, column_size);

  /* allocate the memory for solution vector x */
  x = (double*)calloc(row_size, sizeof(double));
  assert_memory(x);

  /* allocate the memory for permutation vector p */
  p = (int*)calloc(row_size, sizeof(int));
  assert_memory(p);

  /* initialize the matrix A with delay queue DQ */  
  pDelayNode = &(DQ.head);
  for(i = 0; i < n; i++) //for-5
  {
    pDelayNode = pDelayNode->next;

    label = 0; //label to determine P_ij and edge_delay
    if(IsVertexInTrafficTable(ap_table, pDelayNode->enode->tail_node) == TRUE) 
    /* check whether pDelayNode's tail_node is equal to one of access points in ap_table */
    {
      label = 1;
      
      /* set the edge_delay of the edge corresponding to pDelayNode with eid */
      //pDelayNode->enode->head_gnode->edge_delay = 0;	
    }
    else if(IsVertexInTrafficTable(ap_table, pDelayNode->enode->head_node) == TRUE)
    /* check whether pDelayNode's head_node is equal to one of access points in ap_table */
      label = 2;

    /* Note: pDelayNode->order is the order to determine the row index of variable in matrix P */       
    row_id = pDelayNode->order; //row id i of the P_ij in the forwarding probability matrix P
    //row_id = pDelayNode->eid; //row id i of the P_ij in the forwarding probability matrix P

    pDelayComponentNode = &(pDelayNode->delay_component_list.head);
    for(j = 0; j < pDelayNode->delay_component_list.size; j++) //for-6
    {
      pDelayComponentNode = pDelayComponentNode->next;

      /* Note: pDelayComponentNode->order is the order to determine the column index of matrix P */
      column_id = pDelayComponentNode->order; //column id j of the P_ij in the forwarding probability matrix P
      //column_id = pDelayComponentNode->eid; //column id j of the P_ij in the forwarding probability matrix P

      if(label == 2)
      //if(label == 1 || label == 2)
      { 
	P = 0; //Since a destination is placed at the end of this road segment, the forwarding probability must be zero
      }
      else 
	P = pDelayComponentNode->P; //forwarding probability P for the edge corresponding to eid

      A[row_id][column_id] = P; //set forwarding probability P to A_ij
      //A[row_id-1][column_id-1] = P; //set forwarding probability P to A_ij
    } //end of for-6
    
    /* set the diagonal entry to -1:
       Since matrix (P-E)x = -d is used to compute forwarding probability, the diagonal entry must have additional -1.
    */
    A[row_id][row_id] = -1;
    //A[row_id-1][row_id-1] = -1;

    /* set the (n+1)-column entry to -d where d is edge delay */
    //if(label == 1)
    //  A[row_id][n] = 0; //Since pDelayNode's tail_node is equal to one of access points, the edge delay must be zero.
    //else

    A[row_id][n] = -pDelayNode->edge_delay;

  } //end of for-5
  

  /** solve the linear system using Gaussian Elimination */

  /* perform Gaussian elimination for nonsingular case of matrix A where A's column_size is equal to row_size+1, since matrix A is the augmented matrix A = (P | -d) where P is n x n matrix and d is a column vector */
  LA_Perform_Gaussian_Elimination_For_Nonsingular_Case(A, row_size, column_size, p);
  
  /* perform Backward substitution to obtain the solution x for the augmented matrix A where x is a column vector of dimension row_size */
  LA_Perform_Backward_Substitution(A, row_size, p, x);

  /** set the solutions in x to the EDDs in delay queue DQ */
  pDelayNode = &(DQ.head);
  for(i = 0; i < n; i++) //for-7
  {
    pDelayNode = pDelayNode->next;
    pDelayNode->EDD = x[i];
    pDelayNode->enode->head_gnode->EDD = pDelayNode->EDD; //let the edge's head_node have its EDD
  } //end of for-7

  /** sort the intersection edd queue for each intersection in graph G */
  SortIntersection_EDD_Queues_In_Graph(param, G, G_size);

  /** free the memory for the matrix A */
  LA_Free_Matrix_Of_Type_Double(A, row_size);

  /** free the memory for solution vector x */
  free(x);

  /** free the memory for permutation vector p */
  free(p);

  /** destory delay queue DQ */
  DestroyQueue((queue_t*) &DQ);

} //end of function 

void VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model_For_Multiple_APs(parameter_t *param, struct_graph_node *Gr, int Gr_size, directional_edge_queue_t *DEr, struct_traffic_table *ap_table_for_Gr)
{ //compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) based on the stochastic model with graph Gr and directional edge queue EQ in the road network with multiple Internet Access Points (APs).
  struct_graph_node **G_set = NULL; //set of road network graphs for multiple APs
  int *G_set_size = NULL; //size of the road network graph in G_set
  directional_edge_queue_t *DE_set = NULL; //set of directional edge queues
  int number = ap_table_for_Gr->number; //number of access points
  int i = 0; //index for for-loop

  G_set = (struct_graph_node**) calloc(number, sizeof(struct_graph_node*)); //allocate the memory for the set of road network graphs
  assert_memory(G_set);

  G_set_size = (int*) calloc(number, sizeof(int)); //allocate the memory for the size array for the road network graphs
  assert_memory(G_set_size);

  DE_set = (directional_edge_queue_t*) calloc(number, sizeof(directional_edge_queue_t)); //allocate the memory for the set of directional edge queues
  assert_memory(DE_set);

  /** copy the graph G into G_set[i] for i = 0..number-1 */
  for(i = 0; i < number; i++)
  {
    G_set[i] = Make_Forwarding_Graph(Gr, Gr_size, &(G_set_size[i]));
    //make a new forwarding graph G_set[i] for data forwarding for the i-th access point

    CopyVehicularTrafficStatistics(Gr, Gr_size, G_set[i], G_set_size[i]);
    //copy the vehicular traffic statistics of the edges in Gr into that in G_set[i]
    
    ConstructDirectionalEdgeQueue(param, &(DE_set[i]), G_set[i], G_set_size[i]);
    //construct a directional edge queue DE_set[i] containing the information of directional edges in graph G_set[i]
    
    AssociateGraphEdgeWithDirectionalEdgeEntry(G_set[i], G_set_size[i], &(DE_set[i]));
    //associate each pair of two adjacent graph nodes (i.e., physical edge) with the corresponding directional edge entry in DE_set[i]

    VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model(param, G_set[i], G_set_size[i], &(DE_set[i]), ap_table_for_Gr, i); //compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) based on the stochastic model with graph Gr and directional edge queue DEr.  
  }

  /** set the real graph Gr with the graph set G_set after processing the EDD for each access point */
  Set_Forwarding_Information_For_Multiple_APs(param, Gr, Gr_size, G_set, G_set_size, number);

  /** destroy road network graphs */
  for(i = 0; i < number; i++)
  {
    Free_Graph(G_set[i], G_set_size[i]); //release the memory allocated to graph G_set[i]
    DestroyQueue((queue_t*) &(DE_set[i])); //destory directional edge queue DE_set[i]
  }

  free(G_set); //free the allocated memory for G_set
  free(G_set_size); //free the allocated memory for G_set_size
  free(DE_set); //free the allocated memory for DE_set
}

void VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model_For_V2V_Data_Delivery(
			parameter_t *param,
			struct_graph_node *Gr, 
			int Gr_size, 
			struct_graph_node **Gr_set, 
			int *Gr_set_size, 
			directional_edge_queue_t *DEr_set)
{ //compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) based on the stochastic model with road graph network set Gr_set and directional edge queue set DEr_set in the road network for the V2V data delivery.
	int number = Gr_size; //the number of intersections in the road network graph Gr
	int i = 0, j = 0; //loop-indices
	double E2E_EDD = 0; //End-to-End (E2E) EDD at intersection
	double E2E_EDD_SD = 0; //E2E EDD_SD at intersection

	/* check the validity of param, Gr, and Gr_size */
	if(param == NULL)
	{
		printf("%s:%d param is NULL!\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}
	else if(Gr == NULL)
	{
		printf("%s:%d Gr is NULL!\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}
	else if(Gr_size <= 0)
	{
		printf("%s:%d Gr_size(%d) is not positive!\n",
				__FUNCTION__, __LINE__,
				Gr_size);
		exit(1);
	}
	else if(Gr_set == NULL)
	{
		printf("%s:%d Gr_set is NULL!\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}
	else if(Gr_set_size == NULL)
	{
		printf("%s:%d Gr_set_size is NULL!\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}
	else if(DEr_set == NULL)
	{
		printf("%s:%d DEr_set is NULL!\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}

	/** copy the graph G into G_set[i] for i = 0..number-1 */
	for(i = 0; i < number; i++)
	{
		CopyVehicularTrafficStatistics(Gr, Gr_size, Gr_set[i], Gr_set_size[i]);
		//copy the vehicular traffic statistics of the edges in Gr into that in Gr_set[i]

		VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model_For_Target_Intersection(param, Gr_set[i], Gr_set_size[i], &(DEr_set[i]), i); //compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) based on the stochastic model with graph Gr and directional edge queue DEr for the target intersection (i+1). 

		/* compute the EDD and EDD_SD from each source intersection to the target intersection  (i+1) in the road network graph Gr_set[i] */
		for(j = 0; j < number; j++)
		{
			VADD_Compute_EDD_And_EDD_SD_For_TargetPoint_At_Intersection_For_V2V_Data_Delivery(param, i+1, j+1, Gr_set[i], Gr_set_size[i], &E2E_EDD, &E2E_EDD_SD); //compute the End-to-End(E2E) EDD and EDD_SD from each source intersection (j+1) to the target intersection (i+1)
		}
	}
}

void VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, int ap_table_index)
{ //compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) based on the stochastic model with graph G and directional edge queue EQ.
	directional_edge_queue_node_t *pEdgeNode = NULL, *pEdgeNode2 = NULL;  //pointers to directional edge queue nodes
	delay_queue_t DQ; //delay queue DQ containing EDD for each road segment in road network
	delay_queue_node_t delay_node; //contains a queue of delay components of type
	delay_queue_node_t *pDelayNode = NULL; //pointer to delay queue node with delay_component_list containing forwarding probability P and EDD D per edge.
	delay_component_queue_node_t delay_component_node; //delay component node
	delay_component_queue_node_t *pDelayComponentNode = NULL, *pDelayComponentNode2 = NULL; //pointers to delay component node
	int n = EQ->size; //number of road segments
	int i = 0, j = 0, k = 0; //indices of for-loops
	struct_graph_node *pTailNode = NULL, *pHeadNode = NULL, *pNeighborNode = NULL; //pointers to graph nodes
	int head_id = 0; //id for a head node in the edge <tail_node, head_node>
	double **A = NULL; //n x (n+1) EDD matrix containing n x n forwarding probability matrix and edge delay vector
	int row_size = 0, column_size = 0; //row size and column size of a matrix
	int row_id = 0, column_id = 0; //row index i and column index j of the P_ij in the forwarding probability matrix P
	double P = 0; //forwarding probability for an edge
	double *x = NULL; //solution vector of size row_size
	int *p = NULL; //permutation vector of size row_size
	int label = 0; //label to determine P_ij and edge_delay

	/** Compute Forwarding Probability for each directional edge in G */
	VADD_Compute_Forwarding_Probability(param, G, G_size, ap_table, ap_table_index); //compute forwarding probability P per road segment

	/** Initialize Delay Queue DQ */
	InitQueue((queue_t*)&DQ, QTYPE_DELAY);

	/** sort the directional edge queue nodes in the increasing order of eid */
	SortDirectionalEdgeQueue(EQ);

	/** make a list for the linear combination of delay component multiplied by forwarding probability */
	pEdgeNode = &(EQ->head);
	for(i = 0; i < n; i++) //for-3
	{
		pEdgeNode = pEdgeNode->next;

		/* set up delay queue node:
			Note that eid cannot be used for variable indexing since eid keeps increasing during the graph modification for the virtual nodes */
		delay_node.order = pEdgeNode->order; //Note: delay variable id to determine the index of variable in matrix P
		delay_node.eid = pEdgeNode->eid; 
		delay_node.edge_delay = VADD_Compute_Edge_Delay(param, pEdgeNode->head_gnode); 
		//compute the edge delay for the road segment r_ij
		delay_node.edge_delay_standard_deviation = VADD_Compute_Edge_Delay_Standard_Deviation(param, pEdgeNode->head_gnode); 
		delay_node.edge_delay_variance = pow(delay_node.edge_delay_standard_deviation, 2);
		//compute the edge delay variance for the road segment r_ij
    
		delay_node.EDD = 0;
		delay_node.enode = pEdgeNode; //pointer to an edge node in directional edge queue EQ corresponding to eid

		/* set edge_delay to pEdgeNode->head_gnode->edge_delay */
		pEdgeNode->head_gnode->edge_delay = delay_node.edge_delay;

		/* set edge_delay_variance to pEdgeNode->head_gnode->edge_delay_variance */
		pEdgeNode->head_gnode->edge_delay_variance = delay_node.edge_delay_variance;

		/* set the square root of edge_delay_variance to pEdgeNode->head_gnode->edge_delay_standard_deviation */
		pEdgeNode->head_gnode->edge_delay_standard_deviation = sqrt(delay_node.edge_delay_variance);

		pDelayNode = (delay_queue_node_t*)Enqueue((queue_t*)&DQ, (queue_node_t*)&delay_node); //enqueue a new delay node
		InitQueue((queue_t*)&(pDelayNode->delay_component_list), QTYPE_DELAY_COMPONENT);
		//initialize a delay component queue for delay components

		head_id = atoi(pEdgeNode->head_node);
		pHeadNode = &(G[head_id-1]);
		pNeighborNode = pHeadNode;
		for(j = 0; j < pHeadNode->weight; j++) //for-4
		{
			pNeighborNode = pNeighborNode->next;
			pEdgeNode2 = FastLookupDirectionalEdgeQueue(G, pHeadNode->vertex, pNeighborNode->vertex); //The graph node corresponding to the head node points to the directional edge node corresponding to edge <pHeadNode->vertex, pNeighborNode->vertex>.
			if(pEdgeNode2 == NULL)
			{
				printf("VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model(): pEdgeNode2 for <%s,%s> is NULL\n", pHeadNode->vertex, pNeighborNode->vertex);
				exit(1);
			}

			/* set up delay component queue node */
			delay_component_node.order = pEdgeNode2->order; //order is used to index an element in matrix P
			delay_component_node.eid = pEdgeNode2->eid;
			delay_component_node.P = pEdgeNode2->head_gnode->P; //forwarding probability for the edge <pHeadNode->vertex, pNeighborNode->vertex>
			delay_component_node.enode = pEdgeNode2; //pointer to the directional edge node corresponding to eid

			Enqueue((queue_t*)&(pDelayNode->delay_component_list), (queue_node_t*)&delay_component_node);
		} //end of for-4
	} //end of for-3
  
	/** construct n x (n+1) matrix containing the forwarding probability matrix and edge delay vector */
	/* allocate the memory of the row_size x column_size matrix A of type double */
	row_size = n;
	column_size = n + 1;
	A = LA_Allocate_Matrix_Of_Type_Double(row_size, column_size);

	/* allocate the memory for solution vector x */
	x = (double*)calloc(row_size, sizeof(double));
	assert_memory(x);

	/* allocate the memory for permutation vector p */
	p = (int*)calloc(row_size, sizeof(int));
	assert_memory(p);

	/* initialize the matrix A with delay queue DQ */  
	pDelayNode = &(DQ.head);
	for(i = 0; i < n; i++) //for-5
	{
		pDelayNode = pDelayNode->next;

		label = 0; //label to determine P_ij and edge_delay
		if(IsVertexInTrafficTable(ap_table, pDelayNode->enode->tail_node) == TRUE) 
		/* check whether pDelayNode's tail_node is equal to one of access points in ap_table */
		{
			label = 1;
		}
		else if(IsVertexInTrafficTable(ap_table, pDelayNode->enode->head_node) == TRUE)
		/* check whether pDelayNode's head_node is equal to one of access points in ap_table */
		{
			label = 2;
		}

		/* Note: pDelayNode->order is the order to determine the row index of variable in matrix P */       
		row_id = pDelayNode->order; //row id i of the P_ij in the forwarding probability matrix P

		pDelayComponentNode = &(pDelayNode->delay_component_list.head);
		for(j = 0; j < pDelayNode->delay_component_list.size; j++) //for-6
		{
			pDelayComponentNode = pDelayComponentNode->next;

			/* Note: pDelayComponentNode->order is the order to determine the column index of matrix P */
			column_id = pDelayComponentNode->order; //column id j of the P_ij in the forwarding probability matrix P

			if(label == 2)
			{ 
				P = 0; //Since a destination is placed at the end of this road segment, the forwarding probability must be zero
			}
			else
			{
				P = pDelayComponentNode->P; //forwarding probability P for the edge corresponding to eid
			}

			A[row_id][column_id] = P; //set forwarding probability P to A_ij
		} //end of for-6
    
		/* set the diagonal entry to -1:
			Since matrix (P-E)x = -d is used to compute forwarding probability, the diagonal entry must have additional -1.
		*/
		A[row_id][row_id] = -1;

		/* set the (n+1)-column entry to -d where d is edge delay */
		A[row_id][n] = -pDelayNode->edge_delay;    
	} //end of for-5

	/** solve the linear system using Gaussian Elimination */

	/* perform Gaussian elimination for nonsingular case of matrix A where A's column_size is equal to row_size+1, since matrix A is the augmented matrix A = (P | -d) where P is n x n matrix and d is a column vector */
	LA_Perform_Gaussian_Elimination_For_Nonsingular_Case(A, row_size, column_size, p);
  
	/* perform Backward substitution to obtain the solution x for the augmented matrix A where x is a column vector of dimension row_size */
	LA_Perform_Backward_Substitution(A, row_size, p, x);

	/** set the solutions in x to the EDDs in delay queue DQ */
	pDelayNode = &(DQ.head);
	for(i = 0; i < n; i++) //for-7
	{
		pDelayNode = pDelayNode->next;
		pDelayNode->EDD = x[i];

		pDelayNode->enode->head_gnode->EDD = MIN(pDelayNode->EDD, INF); //[10/26/09] let the edge's head_node have its EDD while pDelayNode->EDD is at most INF under the extremely light-traffic condition
	} //end of for-7

#ifdef __LOG_LEVEL_GAUSSIAN_ELIMINATION__
	/**@ log the linear system and the solution */
	LA_Log_LinearSystem_And_Solution(NULL, A, row_size, x);
#endif

	/********************************************************/

	/*** compute the variance of the E2E delivery delay ***/

	/* reset matrix A and vector x */
	LA_Reset_Matrix_Of_Type_Double(A, row_size, column_size);
	//reset the memory of row_size x column_size matrix A of type double

	memset(x, 0, row_size*sizeof(double));
	//reset the memory of row_size vector x

	/* initialize the matrix A with delay queue DQ */  
	pDelayNode = &(DQ.head);
	for(i = 0; i < n; i++) //for-5
	{
		pDelayNode = pDelayNode->next;

		label = 0; //label to determine P_ij and edge_delay_variance
		if(IsVertexInTrafficTable(ap_table, pDelayNode->enode->tail_node) == TRUE) 
		/* check whether pDelayNode's tail_node is equal to one of access points in ap_table */
		{
			label = 1;
		}
		else if(IsVertexInTrafficTable(ap_table, pDelayNode->enode->head_node) == TRUE)
		/* check whether pDelayNode's head_node is equal to one of access points in ap_table */
		{
			label = 2;
		}

		row_id = pDelayNode->order; //row id i of the P_ij in the forwarding probability matrix P

		pDelayComponentNode = &(pDelayNode->delay_component_list.head);
		for(j = 0; j < pDelayNode->delay_component_list.size; j++) //for-6
		{
			pDelayComponentNode = pDelayComponentNode->next;
			column_id = pDelayComponentNode->order; //column id j of the P_ij in the forwarding probability matrix P

			if(label == 2)
			{ 
				P = 0; //Since a destination is placed at the end of this road segment, the forwarding probability must be zero
			}
			else
			{
				P = pDelayComponentNode->P; //forwarding probability P for the edge corresponding to eid
			}

			A[row_id][column_id] = P*P; //set the square of forwarding probability P to A_ij
		} //end of for-6
    
		/* set the diagonal entry to -1:
			Since matrix (P-E)x = -d is used to compute forwarding probability, the diagonal entry must have additional -1.
		*/
		A[row_id][row_id] = -1;

		/* set the (n+1)-column entry to 0 */
		A[row_id][n] = -pDelayNode->edge_delay_variance; //set zero for the other cases except for label == 1 or label == 2 
	} //end of for-5

	/** solve the linear system using Gaussian Elimination */

	/* perform Gaussian elimination for nonsingular case of matrix A where A's column_size is equal to row_size+1, since matrix A is the augmented matrix A = (P | -d) where P is n x n matrix and d is a column vector */
	LA_Perform_Gaussian_Elimination_For_Nonsingular_Case(A, row_size, column_size, p);
  
	/* perform Backward substitution to obtain the solution x for the augmented matrix A where x is a column vector of dimension row_size */
	LA_Perform_Backward_Substitution(A, row_size, p, x);

	/** set the solutions in x to the delivery delay variances in delay queue DQ */
	pDelayNode = &(DQ.head);
	for(i = 0; i < n; i++) //for-7
	{
		pDelayNode = pDelayNode->next;
		pDelayNode->delivery_delay_variance = x[i];

		pDelayNode->enode->head_gnode->EDD_VAR = pDelayNode->delivery_delay_variance; //let the edge's head_node have its delivery delay variance

		/* compute the standard deviation of delivery delay */
		pDelayNode->enode->head_gnode->EDD_SD = MIN(sqrt(pDelayNode->enode->head_gnode->EDD_VAR), INF); //[10/26/09] let the edge's head_node have its EDD_SD while delivery_delay_deviation is at most INF under the extremely light-traffic condition

#ifdef __DEBUG_DELIVERY_DELAY_STANDARD_DEVIATION__
		if(pDelayNode->enode->head_gnode->EDD_SD > 1)
			printf("expected delivery delay standard deviation (EDD_SD) = %f\n", pDelayNode->enode->head_gnode->EDD_SD);
#endif
	} //end of for-7

	/********************************************************/

	/** sort the intersection edd queue for each intersection in graph G */
	SortIntersection_EDD_Queues_In_Graph(param, G, G_size);

	/** free the memory for the matrix A */
	LA_Free_Matrix_Of_Type_Double(A, row_size);

	/** free the memory for solution vector x */
	free(x);

	/** free the memory for permutation vector p */
	free(p);

	/** destory delay queue DQ */
	DestroyQueue((queue_t*) &DQ);
} //end of function 

void VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model_For_Target_Intersection(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, int target_intersection_index)
{ //compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) based on the stochastic model with graph G and directional edge queue EQ for a target intersection whose intersection id is (target_intersection_index+1).
	directional_edge_queue_node_t *pEdgeNode = NULL, *pEdgeNode2 = NULL;  //pointers to directional edge queue nodes
	delay_queue_t DQ; //delay queue DQ containing EDD for each road segment in road network
	delay_queue_node_t delay_node; //contains a queue of delay components of type
	delay_queue_node_t *pDelayNode = NULL; //pointer to delay queue node with delay_component_list containing forwarding probability P and EDD D per edge.
	delay_component_queue_node_t delay_component_node; //delay component node
	delay_component_queue_node_t *pDelayComponentNode = NULL, *pDelayComponentNode2 = NULL; //pointers to delay component node
	int n = EQ->size; //number of road segments
	int i = 0, j = 0, k = 0; //indices of for-loops
	struct_graph_node *pTailNode = NULL, *pHeadNode = NULL, *pNeighborNode = NULL; //pointers to graph nodes
	int head_id = 0; //id for a head node in the edge <tail_node, head_node>
	double **A = NULL; //n x (n+1) EDD matrix containing n x n forwarding probability matrix and edge delay vector
	int row_size = 0, column_size = 0; //row size and column size of a matrix
	int row_id = 0, column_id = 0; //row index i and column index j of the P_ij in the forwarding probability matrix P
	double P = 0; //forwarding probability for an edge
	double *x = NULL; //solution vector of size row_size
	int *p = NULL; //permutation vector of size row_size
	int label = 0; //label to determine P_ij and edge_delay
	int target_node_id = target_intersection_index + 1; //target intersection's node id

	/** Compute Forwarding Probability for each directional edge in G */
	VADD_Compute_Forwarding_Probability_For_Target_Intersection(param, G, G_size, target_intersection_index); //compute forwarding probability P per road segment for a target intersection

	/** Initialize Delay Queue DQ */
	InitQueue((queue_t*)&DQ, QTYPE_DELAY);

	/** sort the directional edge queue nodes in the increasing order of eid */
	SortDirectionalEdgeQueue(EQ);

	/** make a list for the linear combination of delay component multiplied by forwarding probability */
	pEdgeNode = &(EQ->head);
	for(i = 0; i < n; i++) //for-3
	{
		pEdgeNode = pEdgeNode->next;

		/* set up delay queue node:
			Note that eid cannot be used for variable indexing since eid keeps increasing during the graph modification for the virtual nodes */
		delay_node.order = pEdgeNode->order; //Note: delay variable id to determine the index of variable in matrix P
		delay_node.eid = pEdgeNode->eid; 
		delay_node.edge_delay = VADD_Compute_Edge_Delay(param, pEdgeNode->head_gnode); 
		//compute the edge delay for the road segment r_ij
		delay_node.edge_delay_standard_deviation = VADD_Compute_Edge_Delay_Standard_Deviation(param, pEdgeNode->head_gnode); 
		delay_node.edge_delay_variance = pow(delay_node.edge_delay_standard_deviation, 2);
		//compute the edge delay variance for the road segment r_ij
    
		delay_node.EDD = 0;
		delay_node.enode = pEdgeNode; //pointer to an edge node in directional edge queue EQ corresponding to eid

		/* set edge_delay to pEdgeNode->head_gnode->edge_delay */
		pEdgeNode->head_gnode->edge_delay = delay_node.edge_delay;

		/* set edge_delay_variance to pEdgeNode->head_gnode->edge_delay_variance */
		pEdgeNode->head_gnode->edge_delay_variance = delay_node.edge_delay_variance;

		/* set the square root of edge_delay_variance to pEdgeNode->head_gnode->edge_delay_standard_deviation */
		pEdgeNode->head_gnode->edge_delay_standard_deviation = sqrt(delay_node.edge_delay_variance);

		pDelayNode = (delay_queue_node_t*)Enqueue((queue_t*)&DQ, (queue_node_t*)&delay_node); //enqueue a new delay node
		InitQueue((queue_t*)&(pDelayNode->delay_component_list), QTYPE_DELAY_COMPONENT);
		//initialize a delay component queue for delay components

		head_id = atoi(pEdgeNode->head_node);
		pHeadNode = &(G[head_id-1]);
		pNeighborNode = pHeadNode;
		for(j = 0; j < pHeadNode->weight; j++) //for-4
		{
			pNeighborNode = pNeighborNode->next;
			pEdgeNode2 = FastLookupDirectionalEdgeQueue(G, pHeadNode->vertex, pNeighborNode->vertex); //The graph node corresponding to the head node points to the directional edge node corresponding to edge <pHeadNode->vertex, pNeighborNode->vertex>.
			if(pEdgeNode2 == NULL)
			{
				printf("%s:%d: pEdgeNode2 for <%s,%s> is NULL\n", 
						__FUNCTION__, __LINE__,
						pHeadNode->vertex, pNeighborNode->vertex);
				exit(1);
			}

			/* set up delay component queue node */
			delay_component_node.order = pEdgeNode2->order; //order is used to index an element in matrix P
			delay_component_node.eid = pEdgeNode2->eid;
			delay_component_node.P = pEdgeNode2->head_gnode->P; //forwarding probability for the edge <pHeadNode->vertex, pNeighborNode->vertex>
			delay_component_node.enode = pEdgeNode2; //pointer to the directional edge node corresponding to eid

			Enqueue((queue_t*)&(pDelayNode->delay_component_list), (queue_node_t*)&delay_component_node);
		} //end of for-4
	} //end of for-3
  
	/** construct n x (n+1) matrix containing the forwarding probability matrix and edge delay vector */
	/* allocate the memory of the row_size x column_size matrix A of type double */
	row_size = n;
	column_size = n + 1;
	A = LA_Allocate_Matrix_Of_Type_Double(row_size, column_size);

	/* allocate the memory for solution vector x */
	x = (double*)calloc(row_size, sizeof(double));
	assert_memory(x);

	/* allocate the memory for permutation vector p */
	p = (int*)calloc(row_size, sizeof(int));
	assert_memory(p);

	/* initialize the matrix A with delay queue DQ */  
	pDelayNode = &(DQ.head);
	for(i = 0; i < n; i++) //for-5
	{
		pDelayNode = pDelayNode->next;

		label = 0; //label to determine P_ij and edge_delay
		if(target_node_id == atoi(pDelayNode->enode->tail_node))
		//if(IsVertexInTrafficTable(ap_table, pDelayNode->enode->tail_node) == TRUE) 
		/* check whether pDelayNode's tail_node is equal to one of access points in ap_table */
		{
			label = 1;
		}
		else if(target_node_id == atoi(pDelayNode->enode->head_node))
		//else if(IsVertexInTrafficTable(ap_table, pDelayNode->enode->head_node) == TRUE)
		/* check whether pDelayNode's head_node is equal to one of access points in ap_table */
		{
			label = 2;
		}

		/* Note: pDelayNode->order is the order to determine the row index of variable in matrix P */       
		row_id = pDelayNode->order; //row id i of the P_ij in the forwarding probability matrix P

		pDelayComponentNode = &(pDelayNode->delay_component_list.head);
		for(j = 0; j < pDelayNode->delay_component_list.size; j++) //for-6
		{
			pDelayComponentNode = pDelayComponentNode->next;

			/* Note: pDelayComponentNode->order is the order to determine the column index of matrix P */
			column_id = pDelayComponentNode->order; //column id j of the P_ij in the forwarding probability matrix P

			if(label == 2)
			{ 
				P = 0; //Since a destination is placed at the end of this road segment, the forwarding probability must be zero
			}
			else
			{
				P = pDelayComponentNode->P; //forwarding probability P for the edge corresponding to eid
			}

			A[row_id][column_id] = P; //set forwarding probability P to A_ij
		} //end of for-6
    
		/* set the diagonal entry to -1:
			Since matrix (P-E)x = -d is used to compute forwarding probability, the diagonal entry must have additional -1.
		*/
		A[row_id][row_id] = -1;

		/* set the (n+1)-column entry to -d where d is edge delay */
		A[row_id][n] = -pDelayNode->edge_delay;    
	} //end of for-5

	/** solve the linear system using Gaussian Elimination */

	/* perform Gaussian elimination for nonsingular case of matrix A where A's column_size is equal to row_size+1, since matrix A is the augmented matrix A = (P | -d) where P is n x n matrix and d is a column vector */
	LA_Perform_Gaussian_Elimination_For_Nonsingular_Case(A, row_size, column_size, p);
  
	/* perform Backward substitution to obtain the solution x for the augmented matrix A where x is a column vector of dimension row_size */
	LA_Perform_Backward_Substitution(A, row_size, p, x);

	/** set the solutions in x to the EDDs in delay queue DQ */
	pDelayNode = &(DQ.head);
	for(i = 0; i < n; i++) //for-7
	{
		pDelayNode = pDelayNode->next;
		pDelayNode->EDD = x[i];

		pDelayNode->enode->head_gnode->EDD = MIN(pDelayNode->EDD, INF); //[10/26/09] let the edge's head_node have its EDD while pDelayNode->EDD is at most INF under the extremely light-traffic condition
	} //end of for-7

#ifdef __LOG_LEVEL_GAUSSIAN_ELIMINATION__
	/**@ log the linear system and the solution */
	LA_Log_LinearSystem_And_Solution(NULL, A, row_size, x);
#endif

	/********************************************************/

	/*** compute the variance of the E2E delivery delay ***/

	/* reset matrix A and vector x */
	LA_Reset_Matrix_Of_Type_Double(A, row_size, column_size);
	//reset the memory of row_size x column_size matrix A of type double

	memset(x, 0, row_size*sizeof(double));
	//reset the memory of row_size vector x

	/* initialize the matrix A with delay queue DQ */  
	pDelayNode = &(DQ.head);
	for(i = 0; i < n; i++) //for-5
	{
		pDelayNode = pDelayNode->next;

		label = 0; //label to determine P_ij and edge_delay_variance
		if(target_node_id == atoi(pDelayNode->enode->tail_node))
		//if(IsVertexInTrafficTable(ap_table, pDelayNode->enode->tail_node) == TRUE) 
		/* check whether pDelayNode's tail_node is equal to one of access points in ap_table */
		{
			label = 1;
		}
		else if(target_node_id == atoi(pDelayNode->enode->head_node))
		//else if(IsVertexInTrafficTable(ap_table, pDelayNode->enode->head_node) == TRUE)
		/* check whether pDelayNode's head_node is equal to one of access points in ap_table */
		{
			label = 2;
		}

		row_id = pDelayNode->order; //row id i of the P_ij in the forwarding probability matrix P

		pDelayComponentNode = &(pDelayNode->delay_component_list.head);
		for(j = 0; j < pDelayNode->delay_component_list.size; j++) //for-6
		{
			pDelayComponentNode = pDelayComponentNode->next;
			column_id = pDelayComponentNode->order; //column id j of the P_ij in the forwarding probability matrix P

			if(label == 2)
			{ 
				P = 0; //Since a destination is placed at the end of this road segment, the forwarding probability must be zero
			}
			else
			{
				P = pDelayComponentNode->P; //forwarding probability P for the edge corresponding to eid
			}

			A[row_id][column_id] = P*P; //set the square of forwarding probability P to A_ij
		} //end of for-6
    
		/* set the diagonal entry to -1:
			Since matrix (P-E)x = -d is used to compute forwarding probability, the diagonal entry must have additional -1.
		*/
		A[row_id][row_id] = -1;

		/* set the (n+1)-column entry to 0 */
		A[row_id][n] = -pDelayNode->edge_delay_variance; //set zero for the other cases except for label == 1 or label == 2 
	} //end of for-5

	/** solve the linear system using Gaussian Elimination */

	/* perform Gaussian elimination for nonsingular case of matrix A where A's column_size is equal to row_size+1, since matrix A is the augmented matrix A = (P | -d) where P is n x n matrix and d is a column vector */
	LA_Perform_Gaussian_Elimination_For_Nonsingular_Case(A, row_size, column_size, p);
  
	/* perform Backward substitution to obtain the solution x for the augmented matrix A where x is a column vector of dimension row_size */
	LA_Perform_Backward_Substitution(A, row_size, p, x);

	/** set the solutions in x to the delivery delay variances in delay queue DQ */
	pDelayNode = &(DQ.head);
	for(i = 0; i < n; i++) //for-7
	{
		pDelayNode = pDelayNode->next;
		pDelayNode->delivery_delay_variance = x[i];

		pDelayNode->enode->head_gnode->EDD_VAR = pDelayNode->delivery_delay_variance; //let the edge's head_node have its delivery delay variance

		/* compute the standard deviation of delivery delay */
		pDelayNode->enode->head_gnode->EDD_SD = MIN(sqrt(pDelayNode->enode->head_gnode->EDD_VAR), INF); //[10/26/09] let the edge's head_node have its EDD_SD while delivery_delay_deviation is at most INF under the extremely light-traffic condition

#ifdef __DEBUG_DELIVERY_DELAY_STANDARD_DEVIATION__
		if(pDelayNode->enode->head_gnode->EDD_SD > 1)
			printf("%s:%d: expected delivery delay standard deviation (EDD_SD) = %f\n", 
					__FUNCTION__, __LINE__,
					pDelayNode->enode->head_gnode->EDD_SD);
#endif
	} //end of for-7

	/********************************************************/

	/** sort the intersection edd queue for each intersection in graph G */
	SortIntersection_EDD_Queues_In_Graph(param, G, G_size);

	/** free the memory for the matrix A */
	LA_Free_Matrix_Of_Type_Double(A, row_size);

	/** free the memory for solution vector x */
	free(x);

	/** free the memory for permutation vector p */
	free(p);

	/** destory delay queue DQ */
	DestroyQueue((queue_t*) &DQ);
} //end of function 

void VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model_For_Multiple_APs(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, boolean forwarding_table_update_flag)
{ //For the Multiple-AP road network, compute the Expected Delivery Delay (EDD) with graph G and directional edge queue EQ, based on the shortest path model along with the vanet metric type, such as EDD and EDD_VAR. Note that if fowarding_table_update_flag is TRUE, this functions updates the matrices for the forwarding table entry corresponding to ap_table_index. Otherwise, it updates the matrices of the global matrices, such as param->vanet_table.Dr_edd.
    int ap_table_index = 0; //AP table index
    int size = ap_table->number; //size of AP table
    int i = 0; //for-loop index
    
    for(i = 0; i < size; i++)
    {
        ap_table_index = i;

        /* compute EDD and EDD_SD for the AP corresponding ot ap_table_index */
        VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model(param, G, G_size, EQ, ap_table, ap_table_index, forwarding_table_update_flag);
    }
}

void VADD_Compute_EDC_And_EDC_SD_Based_On_Shortest_Path_Model_For_Multiple_APs(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, boolean forwarding_table_update_flag)
{ //For the Multiple-AP road network, compute the Expected Delivery Cost (EDC) with graph G and directional edge queue EQ, based on the shortest path model along with the vanet metric type, such as EDC and EDC_VAR. Note that if fowarding_table_update_flag is TRUE, this functions updates the matrices for the forwarding table entry corresponding to ap_table_index. Otherwise, it updates the matrices of the global matrices, such as param->vanet_table.Dr_edc.
    int ap_table_index = 0; //AP table index
    int size = ap_table->number; //size of AP table
    int i = 0; //for-loop index
    
    for(i = 0; i < size; i++)
    {
        ap_table_index = i;

        /* compute EDC and EDC_SD for the AP corresponding ot ap_table_index */
        VADD_Compute_EDC_And_EDC_SD_Based_On_Shortest_Path_Model(param, G, G_size, EQ, ap_table, ap_table_index, forwarding_table_update_flag);
    }
}

void VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, int ap_table_index, boolean forwarding_table_update_flag)
{ //compute the Expected Delivery Delay (EDD) with graph G and directional edge queue EQ, based on the shortest path model along with the vanet metric type, such as EDD and EDD_VAR. Note that if fowarding_table_update_flag is TRUE, this functions updates the matrices for the forwarding table entry corresponding to ap_table_index. Otherwise, it updates the matrices of the global matrices, such as param->vanet_table.Dr_edd.
    if(param->vehicle_vanet_metric_type == VANET_METRIC_EDD)
	{
        VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model_For_Shortest_EDD(param, G, G_size, EQ, ap_table, ap_table_index, forwarding_table_update_flag); //for vanet_metric_type == VANET_METRIC_EDD
	}
    else if(param->vehicle_vanet_metric_type == VANET_METRIC_EDD_VAR)
	{
        VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model_For_Shortest_EDD_VAR(param, G, G_size, EQ, ap_table, ap_table_index, forwarding_table_update_flag); //for vanet_metric_type == VANET_METRIC_EDD_VAR
	}
	else
	{
		printf("%s:%d: Error: param->vehicle_vanet_metric_type(%d) is not supported yet!\n",
				__FUNCTION__, __LINE__,
				param->vehicle_vanet_metric_type);
		exit(1);
	}
} //end of function 

void VADD_Compute_EDC_And_EDC_SD_Based_On_Shortest_Path_Model(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, int ap_table_index, boolean forwarding_table_update_flag)
{ //compute the Expected Delivery Cost (EDC) with graph G and directional edge queue EQ, based on the shortest path model along with the vanet metric type, such as EDC and EDC_VAR. Note that if fowarding_table_update_flag is TRUE, this functions updates the matrices for the forwarding table entry corresponding to ap_table_index. Otherwise, it updates the matrices of the global matrices, such as param->vanet_table.Dr_edc.
	VADD_Compute_EDC_And_EDC_SD_Based_On_Shortest_Path_Model_For_Shortest_EDC(param, G, G_size, EQ, ap_table, ap_table_index, forwarding_table_update_flag);
} //end of function 


void VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model_For_Shortest_EDD(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, int ap_table_index, boolean forwarding_table_update_flag)
{ //For the shortest EDD (i.e., expected delivery delay), compute the Expected Delivery Delay (EDD) and the Delivery Delay Standard Deviation based on the shortest path model with graph G and directional edge queue EQ. Note that if fowarding_table_update_flag is TRUE, this functions updates the matrices for the forwarding table entry corresponding to ap_table_index. Otherwise, it updates the matrices of the global matrices, such as param->vanet_table.Dr_edd.
  int n = G_size; //number of intersections in G

  double ***D = NULL; //weight matrix for the EDD shortest path for a forwarding table entry
  int ***M = NULL; //predecessor matrix for the EDD shortest path for a forwarding table entry
  double ***S = NULL; //supplementary metric matrix for the EDD shortest path for a forwarding table entry
  int *matrix_size = &(param->vanet_table.matrix_size_for_edd_in_Gr); //matrix size for the EDD shortest path matrix for a forwarding table entry 

  double **D_edd = NULL; //weight matrix for all-pairs shortest paths in terms of Expected Delivery Delay (EDD) in real graph Gr
  int **M_edd = NULL; //predecessor matrix for all-pairs shortest paths in terms of EDD in real graph Gr
  double **S_edd = NULL; //supplementary metric matrix for all-pairs shortest paths in terms of Expected Delivery Delay (EDD) in real graph Gr
  int matrix_size_for_edd_in_G = 0; //matrix size of matrices D_edd and M_edd for EDD in Gr
  struct_graph_node *ptr = NULL; //pointer to graph node
  int dst_intersection_id = 0; //destination intersection id 
  int i = 0, j = 0; //indices of for-loops
  int k = 0; //index for destination intersection

  forwarding_table_queue_t *FTQ = param->vanet_table.FTQ; //pointer to forwarding table queue    
  forwarding_table_queue_node_t *FTQ_Entry = NULL; //pointer to a forwarding table queue node for a target point

  /** get the index in graph G of the vertex corresponding to ap_table[ap_table_index] */
  dst_intersection_id = GetTrafficTableEntry_ID_With_Index(ap_table, ap_table_index); //get the intersection id for a traffic table entry with index
  k = dst_intersection_id - 1; //index for destination intersection

  /** set matrices D, M, and S according to forwarding_table_update_flag **/
  if(forwarding_table_update_flag)
  { /** the case where the matrices for the forwarding table entry is used */
  	/* get the forwarding table entry corresponding to dst_intersection_id */
	FTQ_Entry = FTQ->index_table[dst_intersection_id-1];

	/* set the pointers of matrices D, M, S with a forwarding table entry for a target point */
	D = &(FTQ_Entry->Dr_edd);
	M = &(FTQ_Entry->Mr_edd);
    S = &(FTQ_Entry->Sr_edd);
  }
  else
  { /** the case where the global matrices are used */
  	D = &(param->vanet_table.Dr_edd); //weight matrix for the EDD shortest path
  	M = &(param->vanet_table.Mr_edd); //predecessor matrix for the EDD shortest path
  	S = &(param->vanet_table.Sr_edd); //supplementary metric matrix for the EDD shortest path
  }

  /** Compute Forwarding Probability for each directional edge in G; the forwarding probability can be used for the computation of the EDD and EDD_SD at each intersection */
  VADD_Compute_Forwarding_Probability(param, G, G_size, ap_table, ap_table_index); //compute forwarding probability P per road segment; even though P is not used in this function, but it will be used to EDD in TBD.

  /** Compute the shortest path length in terms of delivery delay in G */
  ///* allocate the matrices for Expected Delivery Delay (EDD) in G */
  //matrix_size_for_edd_in_G = G_size;
  //Floyd_Warshall_Allocate_Matrices_For_EDD(&D_edd, &M_edd, &S_edd, matrix_size_for_edd_in_G);

  /* construct the shortest path delay matrix D_edd, the predecessor matrix M_edd, and the supplementary metric matrix S in terms of delivery delay */
  Floyd_Warshall_Construct_Matrices_For_EDD(G, G_size, D, M, S, matrix_size, param);

  /* set up D_edd, M_edd, S_edd and matrix_size_for_edd_in_G */
  D_edd = *D;
  M_edd = *M;
  S_edd = *S;
  matrix_size_for_edd_in_G = *matrix_size;

  /** set the EDD for the shortest path from source intersection I_i to destination intersection I_j to the road graph G */
  for(i = 0; i < G_size; i++)
  {
    ptr = G[i].next;
    if(ptr == NULL)
      continue;

    /* set up the EDD, EDD_VAR, and EDD_SD of intersection G[i] */
    G[i].EDD = D_edd[i][k];
    G[i].EDD_VAR = S_edd[i][k];
    G[i].EDD_SD = sqrt(G[i].EDD_VAR);

    while(ptr != NULL)
    {
      j = atoi(ptr->vertex) - 1;
      //node id starts from 1, but it should decrease by 1 for weight matrix where the id starts from 0.

      ptr->EDD = ptr->edge_delay + D_edd[j][k]; //E2E delay for the shortest path via road segment r_ij in term of delay
      //ptr->EDD = D_edd[i][j]; //EDD via road segment r_ij

      ptr->EDD_VAR = ptr->edge_delay_variance + S_edd[j][k]; //E2E delay variance for the shortest path
      ptr->EDD_SD = sqrt(ptr->EDD_VAR); //E2E delay standard deviation for the shortest path

      ptr = ptr->next;
    }
  }

  /** sort the intersection edd queue for each intersection in graph G */
  SortIntersection_EDD_Queues_In_Graph(param, G, G_size);

  /** update the matrices for the corresponding forwarding table entry */

  ///** free matrices for EDD in Gr */
  //Floyd_Warshall_Free_Matrices_For_EDD(&D_edd, &M_edd, &S_edd, &matrix_size_for_edd_in_G);
} //end of function 

void VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model_For_Shortest_EDD_VAR(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, int ap_table_index, boolean forwarding_table_update_flag)
{ //For the shortest EDD_VAR (i.e., delivery delay variance), compute the Expected Delivery Delay (EDD) and the Delivery Delay Standard Deviation based on the shortest path model with graph G and directional edge queue EQ. Note that if fowarding_table_update_flag is TRUE, this functions updates the matrices for the forwarding table entry corresponding to ap_table_index. Otherwise, it updates the matrices of the global matrices, such as param->vanet_table.Dr_edd.
  int n = G_size; //number of intersections in G

  double ***D = NULL; //weight matrix for the EDD shortest path
  int ***M = NULL; //predecessor matrix for the EDD shortest path
  double ***S = NULL; //supplementary metric matrix for the EDD shortest path
  int *matrix_size = &(param->vanet_table.matrix_size_for_edd_in_Gr); //matrix size for the EDD shortest path matrix 

  double **D_edd_var = NULL; //weight matrix for all-pairs shortest paths in terms of Delivery Delay Variance (EDD_VAR) in real graph Gr
  int **M_edd_var = NULL; //predecessor matrix for all-pairs shortest paths in terms of EDD_VAR in real graph Gr
  double **S_edd_var = NULL; //supplementary metric matrix for all-pairs shortest paths in terms of Expected Delivery Delay (EDD) in real graph Gr
  int matrix_size_for_edd_var_in_G = 0; //matrix size of matrices D_edd_var and M_edd_var for EDD_VAR in Gr
  struct_graph_node *ptr = NULL; //pointer to graph node
  int dst_intersection_id = 0; //destination intersection id 
  int i = 0, j = 0; //indices of for-loops
  int k = 0; //index for destination intersection

  forwarding_table_queue_t *FTQ = param->vanet_table.FTQ; //pointer to forwarding table queue    
  forwarding_table_queue_node_t *FTQ_Entry = NULL; //pointer to a forwarding table queue node for a target point

  /** get the index in graph G of the vertex corresponding to ap_table[ap_table_index] */
  dst_intersection_id = GetTrafficTableEntry_ID_With_Index(ap_table, ap_table_index); //get the intersection id for a traffic table entry with index
  k = dst_intersection_id - 1; //index for destination intersection

  /** set matrices D, M, and S according to forwarding_table_update_flag **/
  if(forwarding_table_update_flag)
  { /** the case where the matrices for the forwarding table entry is used */
  	/* get the forwarding table entry corresponding to dst_intersection_id */
	FTQ_Entry = FTQ->index_table[dst_intersection_id-1];

	/* set the pointers of matrices D, M, S with a forwarding table entry for a target point */
	D = &(FTQ_Entry->Dr_edd);
	M = &(FTQ_Entry->Mr_edd);
    S = &(FTQ_Entry->Sr_edd);
  }
  else
  { /** the case where the global matrices are used */
  	D = &(param->vanet_table.Dr_edd); //weight matrix for the EDD shortest path
  	M = &(param->vanet_table.Mr_edd); //predecessor matrix for the EDD shortest path
  	S = &(param->vanet_table.Sr_edd); //supplementary metric matrix for the EDD shortest path
  }

  /** Compute Forwarding Probability for each directional edge in G; the forwarding probability can be used for the computation of the EDD and EDD_SD at each intersection */
  VADD_Compute_Forwarding_Probability(param, G, G_size, ap_table, ap_table_index); //compute forwarding probability P per road segment; even though P is not used in this function, but it will be used to EDD in TBD.

  /** Compute the shortest path length in terms of delivery delay variance in G */
  ///* allocate the matrices for Delivery Delay Variance (EDD_VAR) in G */
  //matrix_size_for_edd_var_in_G = G_size;
  //Floyd_Warshall_Allocate_Matrices_For_EDD_VAR(&D_edd_var, &M_edd_var, &S_edd_var, matrix_size_for_edd_var_in_G);

  /** construct the shortest path delay variance matrix D_edd_var, the predecessor matrix M_edd_var, and the supplementary metric matrix S in terms of delivery delay */
  Floyd_Warshall_Construct_Matrices_For_EDD(G, G_size, D, M, S, matrix_size, param);

  /* set up D_edd, M_edd, S_edd and matrix_size_for_edd_in_G */
  D_edd_var = *D;
  M_edd_var = *M;
  S_edd_var = *S;
  matrix_size_for_edd_var_in_G = *matrix_size;

  /** set the EDD_VAR for the shortest path from source intersection I_i to destination intersection I_j to the road graph G */
  for(i = 0; i < G_size; i++)
  {
    ptr = G[i].next;
    if(ptr == NULL)
      continue;

    /* set up the EDD, EDD_VAR, and EDD_SD of intersection G[i] */
    G[i].EDD = S_edd_var[i][k];
    G[i].EDD_VAR = D_edd_var[i][k];
    G[i].EDD_SD = sqrt(G[i].EDD_VAR);

    while(ptr != NULL)
    {
      j = atoi(ptr->vertex) - 1;
      //node id starts from 1, but it should decrease by 1 for weight matrix where the id starts from 0.

      ptr->EDD_VAR = ptr->edge_delay_variance + D_edd_var[j][k]; //E2E delay variance for the shortest path via road segment r_ij in terms of delay variance
      ptr->EDD_SD = sqrt(ptr->EDD_VAR); //E2E delay standard deviation for the shortest path

      ptr->EDD = ptr->edge_delay + S_edd_var[j][k]; //E2E delay for the shortest path

      ptr = ptr->next;
    }
  }

  /** sort the intersection edd_var queue for each intersection in graph G */
  SortIntersection_EDD_Queues_In_Graph(param, G, G_size);

  ///** free matrices for EDD_VAR in Gr */
  //Floyd_Warshall_Free_Matrices_For_EDD_VAR(&D_edd_var, &M_edd_var, &S_edd_var, &matrix_size_for_edd_var_in_G);
} //end of function 

void VADD_Compute_EDC_And_EDC_SD_Based_On_Shortest_Path_Model_For_Shortest_EDC(parameter_t *param, struct_graph_node *G, int G_size, directional_edge_queue_t *EQ, struct_traffic_table *ap_table, int ap_table_index, boolean forwarding_table_update_flag)
{ //For the shortest EDC (i.e., expected delivery cost), compute the Expected Delivery Cost (EDC) and the Delivery Cost Standard Deviation based on the shortest path model with graph G and directional edge queue EQ. Note that if fowarding_table_update_flag is TRUE, this functions updates the matrices for the forwarding table entry corresponding to ap_table_index. Otherwise, it updates the matrices of the global matrices, such as param->vanet_table.Dr_edd.
  int n = G_size; //number of intersections in G

  double ***W = NULL; //weight matrix for the EDC shortest path
  double ***D = NULL; //shortest path matrix for the EDC shortest path
  int ***M = NULL; //predecessor matrix for the EDC shortest path
  double ***S = NULL; //supplementary metric matrix for the EDC shortest path
  int *matrix_size = &(param->vanet_table.matrix_size_for_edc_in_Gr); //matrix size for the EDC shortest path matrix 

  double **W_edc = NULL; //weight matrix for all-pairs shortest paths in terms of Expected Delivery Cost (EDC) in real graph Gr
  double **D_edc = NULL; //shortest path weight matrix for all-pairs shortest paths in terms of Expected Delivery Cost (EDC) in real graph Gr
  int **M_edc = NULL; //predecessor matrix for all-pairs shortest paths in terms of EDC in real graph Gr
  double **S_edc = NULL; //supplementary metric matrix for all-pairs shortest paths in terms of Expected Delivery Cost (EDC) in real graph Gr
  int matrix_size_for_edc_in_G = 0; //matrix size of matrices D_edc and M_edc for EDC in Gr
  struct_graph_node *ptr = NULL; //pointer to graph node
  int dst_intersection_id = 0; //destination intersection id 
  int i = 0, j = 0; //indices of for-loops
  int k = 0; //index for destination intersection

  forwarding_table_queue_t *FTQ = param->vanet_table.FTQ; //pointer to forwarding table queue    
  forwarding_table_queue_node_t *FTQ_Entry = NULL; //pointer to a forwarding table queue node for a target point

  /** get the index in graph G of the vertex corresponding to ap_table[ap_table_index] */
  dst_intersection_id = GetTrafficTableEntry_ID_With_Index(ap_table, ap_table_index); //get the intersection id for a traffic table entry with index
  k = dst_intersection_id - 1; //index for destination intersection

  /** set matrices D, M, and S according to forwarding_table_update_flag **/
  if(forwarding_table_update_flag)
  { /** the case where the matrices for the forwarding table entry is used */
  	/* get the forwarding table entry corresponding to dst_intersection_id */
	FTQ_Entry = FTQ->index_table[dst_intersection_id-1];

	/* set the pointers of matrices W, D, M, S with a forwarding table entry for a target point */
	W = &(FTQ_Entry->Wr_edc);
	D = &(FTQ_Entry->Dr_edc);
	M = &(FTQ_Entry->Mr_edc);
    S = &(FTQ_Entry->Sr_edc);
  }
  else
  { /** the case where the global matrices are used */
  	W = &(param->vanet_table.Wr_edc); //weight matrix for the EDD shortest path
  	D = &(param->vanet_table.Dr_edc); //shortest path weight matrix for the EDD shortest path
  	M = &(param->vanet_table.Mr_edc); //predecessor matrix for the EDD shortest path
  	S = &(param->vanet_table.Sr_edc); //supplementary metric matrix for the EDD shortest path
  }

  /** Compute Forwarding Probability for each directional edge in G; the forwarding probability can be used for the computation of the EDC and EDC_SD at each intersection */
  VADD_Compute_Forwarding_Probability(param, G, G_size, ap_table, ap_table_index); //compute forwarding probability P per road segment; even though P is not used in this function, but it will be used to EDD in TBD.

  /** Compute the shortest path length in terms of delivery cost in G */
  ///* allocate the matrices for Expected Delivery Cost (EDC) in G */
  //matrix_size_for_edc_in_G = G_size;
  //Floyd_Warshall_Allocate_Matrices_For_EDC(&D_edc, &M_edc, &S_edc, matrix_size_for_edc_in_G);

  /* construct the shortest path cost matrix D_edc, the predecessor matrix M_edc, and the supplementary metric matrix S_edc in terms of delivery delay */
  Floyd_Warshall_Construct_Matrices_For_EDC(G, G_size, W, D, M, S, matrix_size, param);

  /* set up W_edc, D_edc, M_edc, S_edc and matrix_size_for_edc_in_G */
  W_edc = *W;
  D_edc = *D;
  M_edc = *M;
  S_edc = *S;
  matrix_size_for_edc_in_G = *matrix_size;

  /** set the EDC for the shortest path from source intersection I_i to destination intersection I_j to the road graph G */
  for(i = 0; i < G_size; i++)
  {
    ptr = G[i].next;
    if(ptr == NULL)
      continue;

    /* set up the EDC, EDC_VAR, and EDC_SD of intersection G[i] */
    G[i].EDC = D_edc[i][k];
    G[i].EDC_VAR = S_edc[i][k];
    G[i].EDC_SD = sqrt(G[i].EDC_VAR);

    while(ptr != NULL)
    {
      j = atoi(ptr->vertex) - 1;
      //node id starts from 1, but it should decrease by 1 for weight matrix where the id starts from 0.

      ptr->EDC = ptr->edge_cost + D_edc[j][k]; //E2E cost for the shortest path via road segment r_ij in term of delay
      //ptr->EDC = D_edc[i][j]; //EDC via road segment r_ij

      ptr->EDC_VAR = ptr->edge_cost_variance + S_edc[j][k]; //E2E cost variance for the shortest path
      ptr->EDC_SD = sqrt(ptr->EDC_VAR); //E2E cost standard deviation for the shortest path

      ptr = ptr->next;
    }
  }

  /** sort the intersection edd queue for each intersection in graph G */
  //SortIntersection_EDD_Queues_In_Graph(param, G, G_size);

  ///** free matrices for EDD in Gr */
  //Floyd_Warshall_Free_Matrices_For_EDD(&D_edd, &M_edd, &S_edd, &matrix_size_for_edd_in_G);
} //end of function 

double VADD_Compute_Edge_Delay(parameter_t *param, struct_graph_node *pGraphNode)
{ //compute the edge delay for the road segment r_ij with head node pGraphNode->vertex
  double delay = 0; //edge delay
  double lambda = 0; //vehicle arrival rate per second
  double rho = 0; //vehicle density on road segment r_ij
  double v = param->vehicle_speed; //average vehicle speed on road segment r_ij
  double l = pGraphNode->weight; //road segment length
  double R = param->communication_range; //radio communication range
  double c = param->communication_one_hop_delay; //average one-hop packet transmission delay for the radio communication range; unit is second.
  double ACL = 0; //Average Convoy Length (ACL) used as forwarding distance
  double a = R/v; //movement time within the communication range R with the speed v
  double alpha = 0; //alpha for ACL computation
  double beta = 0; //beta for ACL computation
  double P = 0; //P is the probability that there is another vehicle to which the packet carrier can forward its packet when it enters the road segment; P is beta.
  double N = 0; //index N for ACL computation
  double carry_distance = 0; //carry distance
  double B = 0; //coefficient

  /* check the vehicle arrival for the directional edge:
     Note that for the directional edge with no vehicle arrival, the edge's mean link delay is 
     the edge's length divided by mean vehicle speed and the link delay standard deviation is zero.*/

  if(pGraphNode->number_of_interarrivals == 0) 
  //if(pGraphNode->mean_interarrival_time == INF)
  { //since there is no vehicle arrival, this link's weight should be infinite
    delay = INF; //[9/20/2009] let this link delay be infinite since there is no vehicle moving over this edge, assuming that the road segment length l is greater than the communication range R
    //delay = l/v; //[9/20/2009] commented

    return delay;
  }
  else
  {
    /* compute the mean interarrival time for the road segment whose head node is pGraphNode */
    pGraphNode->mean_interarrival_time = pGraphNode->sum_of_interarrival_time / pGraphNode->number_of_interarrivals; 
  }

  /** check the validity of parameters */
  /* make sure that communication range R is positive */
  if(R == 0)
  {
	printf("VADD_Compute_Edge_Delay(): Error: R(%.2f) has a wrong value\n", (float)R);
	exit(1);
  }

  switch(param->vehicle_vanet_edge_delay_model)
  {
  case VANET_EDGE_DELAY_TBD_MODEL_FOR_FINITE_ROAD:
    /** compute Average Convoy Length (ACL) for the directional edge where the vehicle is moving */

    lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
    alpha = v*exp(-1*lambda*a)*(1/lambda - (a + 1/lambda)*exp(-1*lambda*a));

	/* check the validity of alpha 
	 * @Note: if alpha is close to 0, we regard the delay as l/v */
	if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
	{
	  	/* compute edge delay as carry delay */
		delay = l/v;
		break;
	}

    beta = 1 - exp(-1*lambda*a);
    N = ceil(beta*(1-beta)/alpha*l); //N is the index where the expected convoy length is greater than l

    B = (N-1)*pow(beta,N) - N*pow(beta,N-1) + 1;
    if(B < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
        B = 0; //Note that B may be a extremely small number, such as 1*10^(-3). In this case, we regard B as zero.

    ACL = alpha/pow(1-beta,2)*B + l*pow(beta,N); //when B is close to zero, we ignore the terms related to B.
    //ACL = alpha*((N-1)*pow(beta,N) - N*pow(beta,N-1) + 1)/pow(1-beta,2) + l*pow(beta,N);
    ACL = MIN_In_Double(ACL, l); // 0 <= ACL <= l
    if(ACL < -ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        printf("VADD_Compute_Edge_Delay(): Error: ACL(%f) is negative\n", (float)ACL);
        exit(1);
    }
    
    /* compute edge delay based on TBD edge delay model */
    carry_distance = l - ACL;
    delay = ACL/R*c + carry_distance/v;

    //P = 1 - exp(-1*lambda*a); //P is the probability that there is another vehicle to which the packet carrier can forward its packet when it enters the road segment.
    //delay = (ACL/R*c + carry_distance/v)*P + (l/v)*(1 - P); //(i) the case where there is another vehicle as next carrier and (ii) the case where there is no other vehicle as next carrier, so the original carrier must carry its packet to the road segment

    break;

  case VANET_EDGE_DELAY_TBD_MODEL_FOR_INFINITE_ROAD:
    /** compute Average Convoy Length (ACL) for the directional edge with the infinite length where the vehicle is moving */
      
    lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
    alpha = v*exp(-1*lambda*a)*(1/lambda - (a + 1/lambda)*exp(-1*lambda*a));

	/* check the validity of alpha 
	 * @Note: if alpha is close to 0, we regard the delay as l/v */
	if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
	{
	  	/* compute edge delay as carry delay */
		delay = l/v;
		break;
	}

    beta = 1 - exp(-1*lambda*a);
    ACL = alpha/pow(1-beta,2);
    ACL = MIN_In_Double(ACL, l); // 0 <= ACL <= l
    if(ACL < -ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        printf("VADD_Compute_Edge_Delay(): Error: ACL(%f) is negative\n", (float)ACL);
        exit(1);
    }
    
    /* compute edge delay based on TBD edge delay model */
    carry_distance = l - ACL;
    delay = ACL/R*c + carry_distance/v;

    //P = 1 - exp(-1*lambda*a); //P is the probability that there is another vehicle to which the packet carrier can forward its packet when it enters the road segment.
    //delay = (ACL/R*c + carry_distance/v)*P + (l/v)*(1 - P); //(i) the case where there is another vehicle as next carrier and (ii) the case where there is no other vehicle as next carrier, so the original carrier must carry its packet to the road segment

    break;

  case VANET_EDGE_DELAY_VADD_MODEL:
    /** compute vehicle density from mean interarrival time */
    lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
    rho = VADD_Vehicle_Density(lambda, v);

    /* compute edge delay based on VADD edge delay model */
    delay = (1 - exp(-R*rho))*l/R*c + exp(-R*rho)*l/v;

    break;

  case VANET_EDGE_DELAY_TSF_MODEL:
    /** compute the link delay for the directional edge with TSF link delay model */

    /* check whether l > R or not */
    if(l > R)
    {
        lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
        alpha = v*exp(-1*lambda*a)*(1/lambda - (a + 1/lambda)*exp(-1*lambda*a));

		/* check the validity of alpha 
		 * @Note: if alpha is close to 0, we regard the delay as l/v */
		if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
		{
		  	/* compute edge delay as carry delay */
			delay = l/v;
			break;
		}

        beta = 1 - exp(-1*lambda*a);

		/* check the validity of beta
		 * @Note: if beta is close to 0, we regard the delay as l/v */
		if(beta <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
		{
		  	/* compute edge delay as carry delay */
			delay = l/v;
			break;
		}

        N = ceil(beta*(1-beta)/alpha*(l-R)); //N is the index where the expected convoy length is greater than l-R
        //N = ceil(beta*(1-beta)/alpha*l); //N is the index where the expected convoy length is greater than l

        B = (N-1)*pow(beta,N) - N*pow(beta,N-1) + 1;
        if(B < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
            B = 0; //Note that B may be a extremely small number, such as 1*10^(-3). In this case, we regard B as zero.

        ACL = alpha/pow(1-beta,2)*B + (l-R)*pow(beta,N); //when B is close to zero, we ignore the terms related to B.
        //ACL = alpha/pow(1-beta,2)*B + l*pow(beta,N); //when B is close to zero, we ignore the terms related to B.
        ACL = MIN_In_Double(ACL, l-R); // 0 <= ACL <= l-R
        if(ACL < -ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
        {
            printf("VADD_Compute_Edge_Delay(): Error: ACL(%f) is negative\n", (float)ACL);
            exit(1);
        }
    }
    else
    {
        ACL = l;
    }

	/* check the validity of beta
	 * @Note: if beta is close to 0, we regard the delay as l/v */
	if(beta <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
	{
	  	/* compute edge delay as carry delay */
		delay = l/v;
	}
	else
	{    
    	/* compute edge delay based on TSF edge delay model */
	  delay = (l - R - ACL)/v*beta + (1/lambda + (l - R)/v)*(1 - beta);
	}

    break;

  default:
    printf("VADD_Compute_Edge_Delay(): vehicle_vanet_edge_delay_model(%d) is not supported yet!\n", param->vehicle_vanet_edge_delay_model);  
    exit(1);
  }
 
  return delay;
}

double VADD_Compute_Subedge_Delay(parameter_t *param, struct_graph_node *pGraphNode, double subedge_length)
{ //compute the edge delay for the subedge length in the road segment r_ij where the vehicle has moved
  double delay = 0; //edge delay
  double lambda = 0; //vehicle arrival rate per second
  double rho = 0; //vehicle density on road segment r_ij
  double v = param->vehicle_speed; //average vehicle speed on road segment r_ij
  double l = subedge_length; //road length where the vehicle has moved
  double R = param->communication_range; //radio communication range
  double c = param->communication_one_hop_delay; //average one-hop packet transmission delay for the radio communication range; unit is second.
  double ACL = 0; //Average Convoy Length (ACL) used as forwarding distance
  double a = R/v; //movement time within the communication range R with the speed v
  double alpha = 0; //alpha for ACL computation
  double beta = 0; //beta for ACL computation
  double P = 0; //P is the probability that there is another vehicle to which the packet carrier can forward its packet when it enters the road segment; P is beta.
  double N = 0; //index N for ACL computation
  double carry_distance = 0; //carry distance
  double B = 0; //coefficient

  /* check the vehicle arrival for the directional edge:
     Note that for the directional edge with no vehicle arrival, the edge's mean link delay is 
     the edge's length divided by mean vehicle speed and the link delay standard deviation is zero.*/
  if(pGraphNode->number_of_interarrivals == 0) 
  //if(pGraphNode->mean_interarrival_time == INF)
  { //since there is no vehicle arrival, this link's weight should be infinite
    delay = INF; //[9/20/2009] let this link delay be infinite since there is no vehicle moving over this edge, assuming that the road segment length l is greater than the communication range R
    //delay = l/v; //[9/20/2009] commented

    return delay;
  }
  else
  {
    /* compute the mean interarrival time for the road segment whose head node is pGraphNode */
    pGraphNode->mean_interarrival_time = pGraphNode->sum_of_interarrival_time / pGraphNode->number_of_interarrivals; 
  }

  /** check the validity of parameters */
  /* make sure that communication range R is positive */
  if(R == 0)
  {
	printf("VADD_Compute_Subedge_Delay(): Error: R(%.2f) has a wrong value\n", (float)R);
	exit(1);
  }

  switch(param->vehicle_vanet_edge_delay_model)
  {
  case VANET_EDGE_DELAY_TBD_MODEL_FOR_FINITE_ROAD:
    /** compute Average Convoy Length (ACL) for the directional edge where the vehicle is moving */

    lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
    alpha = v*exp(-1*lambda*a)*(1/lambda - (a + 1/lambda)*exp(-1*lambda*a));

	/* check the validity of alpha 
	 * @Note: if alpha is close to 0, we regard the delay as l/v */
	if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
	{
	  	/* compute edge delay as carry delay */
		delay = l/v;
		break;
	}

    beta = 1 - exp(-1*lambda*a);
    N = ceil(beta*(1-beta)/alpha*l); //N is the index where the expected convoy length is greater than l

    B = (N-1)*pow(beta,N) - N*pow(beta,N-1) + 1;
    if(B < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
        B = 0; //Note that B may be a extremely small number, such as 1*10^(-3). In this case, we regard B as zero.

    ACL = alpha/pow(1-beta,2)*B + l*pow(beta,N); //when B is close to zero, we ignore the terms related to B.
    //ACL = alpha*((N-1)*pow(beta,N) - N*pow(beta,N-1) + 1)/pow(1-beta,2) + l*pow(beta,N);
    ACL = MIN_In_Double(ACL, l); // 0 <= ACL <= l
    if(ACL < -ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
	/* allow ACL to have -0.000000 */
    {
        printf("VADD_Compute_Subedge_Delay(): Error: ACL(%f) is negative\n", (float)ACL);
        exit(1);
    }

    /* compute edge delay based on TBD edge delay model */
    carry_distance = l - ACL;
    delay = ACL/R*c + carry_distance/v;

    //P = 1 - exp(-1*lambda*a); //P is the probability that there is another vehicle to which the packet carrier can forward its packet when it enters the road segment.
    //delay = (ACL/R*c + carry_distance/v)*P + (l/v)*(1 - P); //(i) the case where there is another vehicle as next carrier and (ii) the case where there is no other vehicle as next carrier, so the original carrier must carry its packet to the road segment

    break;

  case VANET_EDGE_DELAY_TBD_MODEL_FOR_INFINITE_ROAD:
    /** compute Average Convoy Length (ACL) for the directional edge with the infinite length where the vehicle is moving */

    lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
    alpha = v*exp(-1*lambda*a)*(1/lambda - (a + 1/lambda)*exp(-1*lambda*a));

	/* check the validity of alpha 
	 * @Note: if alpha is close to 0, we regard the delay as l/v */
	if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
	{
	  	/* compute edge delay as carry delay */
		delay = l/v;
		break;
	}

    beta = 1 - exp(-1*lambda*a);
    ACL = alpha/pow(1-beta,2);
    ACL = MIN_In_Double(ACL, l); // 0 <= ACL <= l
    if(ACL < -ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        printf("VADD_Compute_Subedge_Delay(): Error: ACL(%f) is negative\n", (float)ACL);
        exit(1);
    }
    
    /* compute edge delay based on TBD edge delay model */
    carry_distance = l - ACL;
    delay = ACL/R*c + carry_distance/v;

    //P = 1 - exp(-1*lambda*a); //P is the probability that there is another vehicle to which the packet carrier can forward its packet when it enters the road segment.
    //delay = (ACL/R*c + carry_distance/v)*P + (l/v)*(1 - P); //(i) the case where there is another vehicle as next carrier and (ii) the case where there is no other vehicle as next carrier, so the original carrier must carry its packet to the road segment

    break;

  case VANET_EDGE_DELAY_VADD_MODEL:
    /** compute vehicle density from mean interarrival time */
    lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
    rho = VADD_Vehicle_Density(lambda, v);

    /* compute edge delay based on VADD edge delay model */
    delay = (1 - exp(-R*rho))*l/R*c + exp(-R*rho)*l/v;

    break;


  case VANET_EDGE_DELAY_TSF_MODEL:
    /** compute the link delay for the directional edge with TSF link delay model */

    /* check whether l > R or not */
    if(l > R)
    {
        lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
        alpha = v*exp(-1*lambda*a)*(1/lambda - (a + 1/lambda)*exp(-1*lambda*a));

		/* check the validity of alpha 
		 * @Note: if alpha is close to 0, we regard the delay as l/v */
		if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
		{
		  	/* compute edge delay as carry delay */
			delay = l/v;
			break;
		}

        beta = 1 - exp(-1*lambda*a);

		/* check the validity of beta
		 * @Note: if beta is close to 0, we regard the delay as l/v */
		if(beta <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
		{
	  		/* compute edge delay as carry delay */
			delay = l/v;
			break;
		}

        N = ceil(beta*(1-beta)/alpha*(l-R)); //N is the index where the expected convoy length is greater than l-R
        //N = ceil(beta*(1-beta)/alpha*l); //N is the index where the expected convoy length is greater than l

        B = (N-1)*pow(beta,N) - N*pow(beta,N-1) + 1;
        if(B < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
            B = 0; //Note that B may be a extremely small number, such as 1*10^(-3). In this case, we regard B as zero.

        ACL = alpha/pow(1-beta,2)*B + (l-R)*pow(beta,N); //when B is close to zero, we ignore the terms related to B.
        //ACL = alpha/pow(1-beta,2)*B + l*pow(beta,N); //when B is close to zero, we ignore the terms related to B.        
        ACL = MIN_In_Double(ACL, l-R); // 0 <= ACL <= l-R
        if(ACL < -ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
        {
            printf("VADD_Compute_Subedge_Delay(): Error: ACL(%f) is negative\n", (float)ACL);
            exit(1);
        }
    }
    else
    {
        ACL = l;
    }

	/* check the validity of beta
	 * @Note: if beta is close to 0, we regard the delay as l/v */
	if(beta <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
	{
	  	/* compute edge delay as carry delay */
		delay = l/v;
	}
	else
	{
        /* compute edge delay based on TSF edge delay model */
    	delay = (l - R - ACL)/v*beta + (1/lambda + (l - R)/v)*(1 - beta);
	}

    break;

  default:
    printf("VADD_Compute_Remaining_Edge_Delay(): vehicle_vanet_edge_delay_model(%d) is not supported yet!\n", param->vehicle_vanet_edge_delay_model);  
    exit(1);
  }
 
  return delay;
}

double VADD_Compute_Average_Convoy_Length(parameter_t *param, struct_graph_node *pGraphNode)
{ //compute the Average Convoy Length (ACL) for the road segment r_ij with head node pGraphNode->vertex
  double lambda = 0; //vehicle arrival rate per second
  double rho = 0; //vehicle density on road segment r_ij
  double v = param->vehicle_speed; //average vehicle speed on road segment r_ij
  double l = pGraphNode->weight; //road segment length
  double R = param->communication_range; //radio communication range
  double ACL = 0; //Average Convoy Length (ACL) used as forwarding distance
  double a = R/v; //movement time within the communication range R with the speed v
  double alpha = 0; //alpha for ACL computation
  double beta = 0; //beta for ACL computation
  double N = 0; //index N for ACL computation
  double B = 0; //coefficient

  /* check the vehicle arrival for the directional edge:
     Note that for the directional edge with no vehicle arrival, the Average Component Length (ACL) is the edge's lengthand the component length deviation is zero.*/
  if(pGraphNode->number_of_interarrivals == 0) 
  //if(pGraphNode->mean_interarrival_time == INF)
  { //since there is no vehicle arrival, this link's ACL should be zero
    ACL = 0;
    return ACL;
  }
  else
  {
    /* compute the mean interarrival time for the road segment whose head node is pGraphNode */
    pGraphNode->mean_interarrival_time = pGraphNode->sum_of_interarrival_time / pGraphNode->number_of_interarrivals; 
  }

  /** check the validity of parameters */
  /* make sure that communication range R is positive */
  if(R == 0)
  {
	printf("VADD_Compute_Average_Convoy_Length(): Error: R(%.2f) has a wrong value\n", (float)R);
	exit(1);
  }
  
  switch(param->vehicle_vanet_edge_delay_model)
  {
  case VANET_EDGE_DELAY_TBD_MODEL_FOR_FINITE_ROAD:
    /** compute Average Convoy Length (ACL) for the directional edge where the vehicle is moving */

    lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
    alpha = v*exp(-1*lambda*a)*(1/lambda - (a + 1/lambda)*exp(-1*lambda*a));

	/* check the validity of alpha 
	 * @Note: if alpha is close to 0, we regard the delay as l/v */
	if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
	{
	  	/* set ACL to 0 */
		ACL = 0;
		break;
	}

    beta = 1 - exp(-1*lambda*a);
    N = ceil(beta*(1-beta)/alpha*l); //N is the index where the expected convoy length is greater than l

    B = (N-1)*pow(beta,N) - N*pow(beta,N-1) + 1;
    if(B < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
        B = 0; //Note that B may be a extremely small number, such as 1*10^(-3). In this case, we regard B as zero.

    ACL = alpha/pow(1-beta,2)*B + l*pow(beta,N); //when B is close to zero, we ignore the terms related to B.
    //ACL = alpha*((N-1)*pow(beta,N) - N*pow(beta,N-1) + 1)/pow(1-beta,2) + l*pow(beta,N);
    ACL = MIN_In_Double(ACL, l); // 0 <= ACL <= l
    if(ACL < -ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        printf("VADD_Compute_Average_Convoy_Length(): Error: ACL(%f) is negative\n", (float)ACL);
        exit(1);
    }

    break;

  case VANET_EDGE_DELAY_TBD_MODEL_FOR_INFINITE_ROAD:
    /** compute Average Convoy Length (ACL) for the directional edge with the infinite length where the vehicle is moving */

    lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
    alpha = v*exp(-1*lambda*a)*(1/lambda - (a + 1/lambda)*exp(-1*lambda*a));
	/* check the validity of alpha 
	 * @Note: if alpha is close to 0, we regard the delay as l/v */
	if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
	{
	  	/* set ACL to 0 */
		ACL = 0;
		break;
	}

    beta = 1 - exp(-1*lambda*a);
    ACL = alpha/pow(1-beta,2);
    ACL = MIN_In_Double(ACL, l); // 0 <= ACL <= l
    if(ACL < -ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
        printf("VADD_Compute_Average_Convoy_Length(): Error: ACL(%f) is negative\n", (float)ACL);
        exit(1);
    }

    break;

  case VANET_EDGE_DELAY_VADD_MODEL:
    /** compute vehicle density from mean interarrival time */
    lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
    rho = VADD_Vehicle_Density(lambda, v);

    /* compute the ACL based on VADD edge delay model */
    ACL = (1 - exp(-R*rho))*l;

    break;

  case VANET_EDGE_DELAY_TSF_MODEL:
    /** compute Average Convoy Length (ACL) for the directional edge where the vehicle is moving */

    /* check whether l > R or not */
    if(l > R)
    {
        lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
        alpha = v*exp(-1*lambda*a)*(1/lambda - (a + 1/lambda)*exp(-1*lambda*a));

		/* check the validity of alpha 
		 * @Note: if alpha is close to 0, we regard the delay as l/v */
		if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
		{
			/* set ACL to 0 */
			ACL = 0;
			break;
		}

        beta = 1 - exp(-1*lambda*a);

		/* check the validity of beta
		 * @Note: if beta is close to 0, we regard the delay as l/v */
		if(beta <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
		{
			/* set ACL to 0 */
			ACL = 0;
			break;
		}


        N = ceil(beta*(1-beta)/alpha*(l-R)); //N is the index where the expected convoy length is greater than l-R
        //N = ceil(beta*(1-beta)/alpha*l); //N is the index where the expected convoy length is greater than l

        B = (N-1)*pow(beta,N) - N*pow(beta,N-1) + 1;
        if(B < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
            B = 0; //Note that B may be a extremely small number, such as 1*10^(-3). In this case, we regard B as zero.

        ACL = alpha/pow(1-beta,2)*B + (l-R)*pow(beta,N); //when B is close to zero, we ignore the terms related to B.
        //ACL = alpha/pow(1-beta,2)*B + l*pow(beta,N); //when B is close to zero, we ignore the terms related to B.
        //ACL = alpha*((N-1)*pow(beta,N) - N*pow(beta,N-1) + 1)/pow(1-beta,2) + l*pow(beta,N);
        ACL = MIN_In_Double(ACL, l-R); // 0 <= ACL <= l-R
        if(ACL < -ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
        {
            printf("VADD_Compute_Average_Convoy_Length(): Error: ACL(%f) is negative\n", (float)ACL);
            exit(1);
        }
    }
    else
    {
        ACL = l;
    }

    break;

  default:
    printf("VADD_Compute_Average_Convoy_Length(): vehicle_vanet_edge_delay_model(%d) is not supported yet!\n", param->vehicle_vanet_edge_delay_model);  
    exit(1);
  }
 
  return ACL;
}

double VADD_Compute_Edge_Delay_Standard_Deviation(parameter_t *param, struct_graph_node *pGraphNode)
{ //compute the edge delay's standard deviation for the road segment r_ij with head node pGraphNode->vertex
  double delay_standard_deviation = 0; //edge delay standard deviation
  //double delay = edge_delay; //edge delay
  double lambda = 0; //vehicle arrival rate per second
  //double rho = 0; //vehicle density on road segment r_ij
  double v = param->vehicle_speed; //average vehicle speed on road segment r_ij
  double l = pGraphNode->weight; //road segment length
  double R = param->communication_range; //radio communication range
  double c = param->communication_one_hop_delay; //average one-hop packet transmission delay for the radio communication range; unit is second.
  double ACL = 0; //average component length (ACL), also called average convoy length
  double CL_VAR = 0; //Convoy Length Variance (CL_VAR) used as forwarding distance
  double CL_SD = 0; //Convoy Length Standard Deviation (CL_SD) used as forwarding distance
  double a = R/v; //movement time within the communication range R with the speed v
  double b = (1/lambda - (a + 1/lambda)*exp(-1*lambda*a)) / (1 - exp(-1*lambda*a)); //the expected interarrival time of two vehicles in the case where two vehicles are connected by the communication range
  double gamma = 0; //gamma for CL_VAR computation
  double rho = 0; //rho for ACL and CL_VAR computation
  double alpha = 0; //alpha for ACL computation
  double delta = 0; //delta for CL_VAR computation
  double beta = 0; //beta for CL_VAR computation
  double A = 0, B = 0; //coefficients
  double N = 0; //index N for CL_VAR computation
  double margin = 0; //length by which the sum of ACL and CL_SD is longer than the road length l;
  double E_L_2 = 0; //second moment of the component length
  double E_L_1 = 0; //first moment of the component length, that is, ACL
  double carry_distance_sd = 0; //carry distance standard deviation
  double delay_sd = 0; //edge (or link) delay standard deviation

  double E_D_2 = 0; //second moment of the link delay
  double E_D_1 = 0; //first moment of the link delay, i.e., average link delay
  double LD_VAR = 0; //Link Delay Variance (LD_VAR)
  double LD_SD = 0; //Link Delay Standard Deviation (LD_SD)

  /* check the vehicle arrival for the directional edge:
     Note that for the directional edge with no vehicle arrival, the edge's mean link delay is 
     the edge's length divided by mean vehicle speed and the link delay standard deviation is zero.*/
  if(pGraphNode->number_of_interarrivals == 0) 
  //if(pGraphNode->mean_interarrival_time == INF)
  { //since there is no vehicle arrival, this link delay's standard deviation should be zero
    delay_standard_deviation = 0;
    return delay_standard_deviation;
  }
  else
  {
    /* compute the mean interarrival time for the road segment whose head node is pGraphNode */
    pGraphNode->mean_interarrival_time = pGraphNode->sum_of_interarrival_time / pGraphNode->number_of_interarrivals; 
  }

  /** check the validity of parameters */
  /* make sure that communication range R is positive */
  if(R == 0)
  {
	printf("VADD_Compute_Edge_Delay_Standard_Deviation(): Error: R(%.2f) has a wrong value\n", (float)R);
	exit(1);
  }

  switch(param->vehicle_vanet_edge_delay_model)
  {
  case VANET_EDGE_DELAY_TBD_MODEL_FOR_FINITE_ROAD:
    /** compute Component Length Variance (CL_VAR) for the directional edge where the vehicle is moving */

    lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
    beta = 1 - exp(-1*lambda*a);
    rho = 1/lambda - (a + 1/lambda)*exp(-1*lambda*a);
    alpha = v*(1 - beta)*rho;
    //alpha = v*(1 - beta);

	/* check the validity of alpha 
	 * @Note: if alpha is close to 0, we regard the delay as l/v */
	if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
	{
	  	/* compute edge delay standard deviation as carry delay */
		delay_standard_deviation = l/v;
		break;
	}

    delta = -1*pow(a, 2)*(1 - beta) + 2*rho/lambda;

    N = ceil(beta*(1-beta)/alpha*l); //N is the index where the expected component length is greater than l
    
    B = (N-1)*pow(beta,N) - N*pow(beta,N-1) + 1;
    if(B < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
        B = 0; //Note that B may be a extremely small number, such as 1*10^(-3). In this case, we regard B as zero.

    A = (N*(N-1)*pow(beta,N-1) - N*(N-1)*pow(beta,N-2))*pow(1-beta,2) + 2*B*(1-beta);
    if(A < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC) //Note that A may be a extremely small number, such as 1*10^(-3). In this case, we regard A as zero.
        A = 0;

    ACL = E_L_1 = alpha/pow(1-beta,2)*B + l*pow(beta,N); //When B is close to zero, we ignore the term related to B
    //E_L_1 = alpha*B/pow(1-beta,2) + l*pow(beta,N);

    E_L_2 = pow(v,2)*delta*(1-beta)/pow(1-beta,2)*B + pow(v*rho,2)*(1-beta)/pow(1-beta,4)*A + pow(l,2)*pow(beta,N); //When A or B is close to zero, we ignore the terms related to A or B
    //E_L_2 = pow(v,2)*delta*(1-beta)/pow(1-beta,2)*B + pow(v*rho,2)/pow(1-beta,4)*A + pow(l,2)*pow(beta,N); //When A or B is close to zero, we ignore the terms related to A or B
    //////////////////////////////

    CL_VAR = E_L_2 - pow(E_L_1, 2);

    CL_SD = sqrt(CL_VAR);
    CL_SD = MIN_In_Double(ACL, CL_SD); //let CL_SD less than ACL
    margin = (ACL + CL_SD) - l; //let margin have the length by which the sum of ACL and CL_SD is longer than the road length l such that ACL + CL_SD <= l;
    if(margin > 0)
      CL_SD -= margin; 

    carry_distance_sd = CL_SD;
    if(carry_distance_sd < 0)
    {
      printf("VADD_Compute_Edge_Delay_Standard_Deviation(): Error: carry_distance_sd(%f) is negative\n", (float)carry_distance_sd);
      exit(1);
    }
    
    /* compute edge delay standard deviation based on TBD edge delay model */
    delay_standard_deviation = carry_distance_sd/v; //since the carry delay is order-of-magnitude longer than the forwarding delay, we consider just carry_distance_sd for the calculation of delay_standard_deviation for the edge

    break;

  case VANET_EDGE_DELAY_TBD_MODEL_FOR_INFINITE_ROAD:
    /** compute Component Length Variance (CL_VAR) for the directional edge with the infinite length where the vehicle is moving */

    lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
    beta = 1 - exp(-1*lambda*a);
    rho = 1/lambda - (a + 1/lambda)*exp(-1*lambda*a);
    alpha = v*(1 - beta)*rho;

	/* check the validity of alpha 
	 * @Note: if alpha is close to 0, we regard the delay as l/v */
	if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
	{
	  	/* compute edge delay standard deviation as carry delay */
		delay_standard_deviation = l/v;
		break;
	}

    delta = -1*pow(a, 2)*(1 - beta) + 2*rho/lambda;
    gamma = pow(v, 2)*(1 - beta)*delta;

    ACL = E_L_1 = alpha/pow(1-beta, 2);
    E_L_2 = gamma/pow(1-beta, 2) + 2*pow(alpha/pow(1-beta, 2), 2);
    ////////////////////////////////

    CL_VAR = E_L_2 + pow(E_L_1, 2);

    CL_SD = sqrt(CL_VAR);

    ACL = MIN_In_Double(ACL, l); //we let ACL be at most l

    CL_SD = MIN_In_Double(ACL, CL_SD); //let CL_SD less than ACL
    margin = (ACL + CL_SD) - l; //let margin have the length by which the sum of ACL and CL_SD is longer than the road length l such that ACL + CL_SD <= l;
    if(margin > 0)
      CL_SD -= margin; 

    carry_distance_sd = CL_SD;
    if(carry_distance_sd < 0)
    {
      printf("VADD_Compute_Edge_Delay_Standard_Deviation(): Error: carry_distance_sd(%f) is negative\n", (float)carry_distance_sd);
      exit(1);
    }

    /* compute edge delay standard deviation based on TBD edge delay model */
    delay_standard_deviation = carry_distance_sd/v; //since the carry delay is order-of-magnitude longer than the forwarding delay, we consider just carry_distance_sd for the calculation of delay_standard_deviation for the edge

    break;

  case VANET_EDGE_DELAY_VADD_MODEL:
    /** NOT IMPLEMENTED! */
    //printf("VADD_Compute_Edge_Delay_Standard_Deviation(): VANET_EDGE_DELAY_VADD_MODEL is not implemented yet!\n");
    //exit(0);

    delay_standard_deviation = l/v;

    break;

  case VANET_EDGE_DELAY_TSF_MODEL:
    /** compute Component Length Variance (CL_VAR) for the directional edge where the vehicle is moving */

    /* check whether l > R or not */
    if(l > R)
    {
        lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
        beta = 1 - exp(-1*lambda*a);

		/* check the validity of beta
		 * @Note: if beta is close to 0, we regard the delay as l/v */
		if(beta <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
		{
		  	/* compute edge delay standard deviation as carry delay */
			delay_standard_deviation = l/v;
			break;
		}

        rho = 1/lambda - (a + 1/lambda)*exp(-1*lambda*a);
        alpha = v*(1 - beta)*rho;
        //alpha = v*(1 - beta);

		/* check the validity of alpha 
		 * @Note: if alpha is close to 0, we regard the delay as l/v */
		if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
		{
	  		/* compute edge delay standard deviation as carry delay */
			delay_standard_deviation = l/v;
			break;
		}

        delta = -1*pow(a, 2)*(1 - beta) + 2*rho/lambda;

        N = ceil(beta*(1-beta)/alpha*(l-R)); //N is the index where the expected component length is greater than l-R
        //N = ceil(beta*(1-beta)/alpha*l); //N is the index where the expected component length is greater than l
    
        B = (N-1)*pow(beta,N) - N*pow(beta,N-1) + 1;
        if(B < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
            B = 0; //Note that B may be a extremely small number, such as 1*10^(-3). In this case, we regard B as zero.

        A = (N*(N-1)*pow(beta,N-1) - N*(N-1)*pow(beta,N-2))*pow(1-beta,2) + 2*B*(1-beta);
        if(A < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC) //Note that A may be a extremely small number, such as 1*10^(-3). In this case, we regard A as zero.
            A = 0;

        ACL = E_L_1 = alpha/pow(1-beta,2)*B + (l-R)*pow(beta,N); //When B is close to zero, we ignore the term related to B
        //ACL = E_L_1 = alpha/pow(1-beta,2)*B + l*pow(beta,N); //When B is close to zero, we ignore the term related to B
        //E_L_1 = alpha*B/pow(1-beta,2) + l*pow(beta,N);

        E_L_2 = pow(v,2)*delta*(1-beta)/pow(1-beta,2)*B + pow(v*rho,2)*(1-beta)/pow(1-beta,4)*A + pow(l-R,2)*pow(beta,N); //When A or B is close to zero, we ignore the terms related to A or B
        //E_L_2 = pow(v,2)*delta*(1-beta)/pow(1-beta,2)*B + pow(v*rho,2)*(1-beta)/pow(1-beta,4)*A + pow(l,2)*pow(beta,N); //When A or B is close to zero, we ignore the terms related to A or B
        //E_L_2 = pow(v,2)*delta*(1-beta)/pow(1-beta,2)*B + pow(v*rho,2)/pow(1-beta,4)*A + pow(l,2)*pow(beta,N); //When A or B is close to zero, we ignore the terms related to A or B
    }
    else
    {
		//ACL = E_L_1 = l;
		//E_L_2 = pow(l, 2);

		/* compute edge delay standard deviation as carry delay */
		delay_standard_deviation = l/v;
		break;
    }

    /** compute the variance of link delay */
    E_D_2 = (pow(l-R,2) - 2*(l-R)*E_L_1 + E_L_2)*beta/pow(v,2) + pow(1/lambda+(l-R)/v,2)*(1-beta); 
    //E_D_2 = (pow(l-R,2) - 2*(l-R)*E_L_1 + pow(E_L_2,2))*beta/pow(v,2) + pow(1/lambda+(l-R)/v,2)*(1-beta); //@Note that this has an error in  pow(E_L_2,2) and that the correct one is E_L_2 without squaring; that is why the previous actual variance is much greater than the expected variance

    E_D_1 = (l-R-E_L_1)*beta/v + (1/lambda+(1-R)/v)*(1-beta);

    LD_VAR = E_D_2 - pow(E_D_1,2);

    LD_SD = sqrt(LD_VAR);
    LD_SD = MIN_In_Double((l-ACL)/v, LD_SD); //let LD_SD less than (l-ACL)/v => Check whether this LD_SD is correct or not

    /* compute edge delay standard deviation based on TSF edge delay model */
    delay_standard_deviation = LD_SD; //since the carry delay is order-of-magnitude longer than the forwarding delay, we consider just carry_distance_sd for the calculation of delay_standard_deviation for the edge

    if(delay_standard_deviation < 0)
    {
        printf("VADD_Compute_Edge_Delay_Standard_Deviation(): Error: link_delay_sd(%f) is negative\n", (float)delay_standard_deviation);
      exit(1);
    }
    break;

  default:
    printf("VADD_Compute_Edge_Delay_Standard_Deviation(): vehicle_vanet_edge_delay_model(%d) is not supported yet!\n", param->vehicle_vanet_edge_delay_model);  
    exit(1);
  }
 
  return delay_standard_deviation;
}


double VADD_Compute_Subedge_Delay_Standard_Deviation(parameter_t *param, struct_graph_node *pGraphNode, double subedge_length)
{ //compute the edge delay standard deviation for the subedge length in the road segment r_ij where the vehicle has moved

  double delay_standard_deviation = 0; //edge delay standard deviation
  //double delay = edge_delay; //edge delay
  double lambda = 0; //vehicle arrival rate per second
  //double rho = 0; //vehicle density on road segment r_ij
  double v = param->vehicle_speed; //average vehicle speed on road segment r_ij
  //double l = pGraphNode->weight; //road segment length
  double l = subedge_length; //road length where the vehicle has moved
  double R = param->communication_range; //radio communication range
  double c = param->communication_one_hop_delay; //average one-hop packet transmission delay for the radio communication range; unit is second.
  double ACL = 0; //average component length (ACL), also called average convoy length
  double CL_VAR = 0; //Convoy Length Variance (CL_VAR) used as forwarding distance
  double CL_SD = 0; //Convoy Length Standard Deviation (CL_SD) used as forwarding distance
  double a = R/v; //movement time within the communication range R with the speed v
  double b = (1/lambda - (a + 1/lambda)*exp(-1*lambda*a)) / (1 - exp(-1*lambda*a)); //the expected interarrival time of two vehicles in the case where two vehicles are connected by the communication range
  double gamma = 0; //gamma for CL_VAR computation
  double rho = 0; //rho for ACL and CL_VAR computation
  double alpha = 0; //alpha for ACL computation
  double delta = 0; //delta for CL_VAR computation
  double beta = 0; //beta for CL_VAR computation
  double A = 0, B = 0; //coefficients
  double N = 0; //index N for CL_VAR computation
  double margin = 0; //length by which the sum of ACL and CL_SD is longer than the road length l;
  double E_L_2 = 0; //second moment of the component length
  double E_L_1 = 0; //first moment of the component length, that is, ACL
  double carry_distance_sd = 0; //carry distance standard deviation

  double E_D_2 = 0; //second moment of the link delay
  double E_D_1 = 0; //first moment of the link delay, i.e., average link delay
  double LD_VAR = 0; //Link Delay Variance (LD_VAR)
  double LD_SD = 0; //Link Delay Standard Deviation (LD_SD)
  
  /* check the vehicle arrival for the directional edge:
     Note that for the directional edge with no vehicle arrival, the edge's mean link delay is 
     the edge's length divided by mean vehicle speed and the link delay standard deviation is zero.*/
  if(pGraphNode->number_of_interarrivals == 0) 
  //if(pGraphNode->mean_interarrival_time == INF)
  { //since there is no vehicle arrival, this link delay's standard deviation should be zero
    delay_standard_deviation = 0;
    return delay_standard_deviation;
  }
  else
  {
    /* compute the mean interarrival time for the road segment whose head node is pGraphNode */
    pGraphNode->mean_interarrival_time = pGraphNode->sum_of_interarrival_time / pGraphNode->number_of_interarrivals; 
  }

  /** check the validity of parameters */
  /* make sure that communication range R is positive */
  if(R == 0)
  {
	printf("VADD_Compute_Subedge_Delay_Standard_Deviation(): Error: R(%.2f) has a wrong value\n", (float)R);
	exit(1);
  }

  switch(param->vehicle_vanet_edge_delay_model)
  {
  case VANET_EDGE_DELAY_TBD_MODEL_FOR_FINITE_ROAD:
    /** compute Component Length Variance (CL_VAR) for the directional edge where the vehicle is moving */

    lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
    beta = 1 - exp(-1*lambda*a);
    rho = 1/lambda - (a + 1/lambda)*exp(-1*lambda*a);
    alpha = v*(1 - beta)*rho;

	/* check the validity of alpha 
	 * @Note: if alpha is close to 0, we regard the delay as l/v */
	if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
	{
	  	/* compute edge delay standard deviation as carry delay */
		delay_standard_deviation = l/v;
		break;
	}

    delta = -1*pow(a, 2)*(1 - beta) + 2*rho/lambda;

    N = ceil(beta*(1-beta)/alpha*l); //N is the index where the expected component length is greater than l

    B = (N-1)*pow(beta,N) - N*pow(beta,N-1) + 1;
    if(B < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
        B = 0; //Note that B may be a extremely small number, such as 1*10^(-3). In this case, we regard B as zero.

    A = (N*(N-1)*pow(beta,N-1) - N*(N-1)*pow(beta,N-2))*pow(1-beta,2) + 2*B*(1-beta);
    if(A < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC) //Note that A may be a extremely small number, such as 1*10^(-3). In this case, we regard A as zero.
        A = 0;

    ACL = E_L_1 = alpha/pow(1-beta,2)*B + l*pow(beta,N); //When B is close to zero, we ignore the term related to B
    //E_L_1 = alpha*B/pow(1-beta,2) + l*pow(beta,N);

    E_L_2 = pow(v,2)*delta*(1-beta)/pow(1-beta,2)*B + pow(v*rho,2)*(1-beta)/pow(1-beta,4)*A + pow(l,2)*pow(beta,N); //When A or B is close to zero, we ignore the terms related to A or B
    //E_L_2 = pow(v,2)*delta*(1-beta)/pow(1-beta,2)*B + pow(v*rho,2)/pow(1-beta,4)*A + pow(l,2)*pow(beta,N); //When A or B is close to zero, we ignore the terms related to A or B
    //E_L_2 = pow(v,2)*delta*(1-beta)*B/pow(1-beta,2) + pow(v*rho,2)*A/pow(1-beta,4) + pow(l,2)*pow(beta,N); 
    ////////////////////////

    CL_VAR = E_L_2 - pow(E_L_1, 2);

    CL_SD = sqrt(CL_VAR);
    CL_SD = MIN_In_Double(ACL, CL_SD); //let CL_SD less than ACL
    margin = (ACL + CL_SD) - l; //let margin have the length by which the sum of ACL and CL_SD is longer than the road length l such that ACL + CL_SD <= l;
    if(margin > 0)
      CL_SD -= margin; 

    carry_distance_sd = CL_SD;
    if(carry_distance_sd < -ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
      printf("VADD_Compute_Subedge_Delay_Standard_Deviation(): Error: carry_distance_sd(%f) is negative\n", (float)carry_distance_sd);
      exit(1);
    }
    
    /* compute edge delay standard deviation based on TBD edge delay model */
    delay_standard_deviation = carry_distance_sd/v; //since the carry delay is order-of-magnitude longer than the forwarding delay, we consider just carry_distance_sd for the calculation of delay_standard_deviation for the edge

    break;

  case VANET_EDGE_DELAY_TBD_MODEL_FOR_INFINITE_ROAD:
    /** compute Component Length Variance (CL_VAR) for the directional edge with the infinite length where the vehicle is moving */

    lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
    beta = 1 - exp(-1*lambda*a);
    rho = 1/lambda - (a + 1/lambda)*exp(-1*lambda*a);
    alpha = v*(1 - beta)*rho;

	/* check the validity of alpha 
	 * @Note: if alpha is close to 0, we regard the delay as l/v */
	if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
	{
	  	/* compute edge delay standard deviation as carry delay */
		delay_standard_deviation = l/v;
		break;
	}

    delta = -1*pow(a, 2)*(1 - beta) + 2*rho/lambda;
    gamma = pow(v, 2)*(1 - beta)*delta;

    ACL = E_L_1 = alpha/pow(1-beta, 2);
    E_L_2 = gamma/pow(1-beta, 2) + 2*pow(alpha/pow(1-beta, 2), 2);
    //////////////////////////////

    CL_VAR = E_L_2 + pow(E_L_1, 2);

    CL_SD = sqrt(CL_VAR);

    ACL = MIN_In_Double(ACL, l); //we let ACL be at most l

    CL_SD = MIN_In_Double(ACL, CL_SD); //let CL_SD less than ACL
    margin = (ACL + CL_SD) - l; //let margin have the length by which the sum of ACL and CL_SD is longer than the road length l such that ACL + CL_SD <= l;
    if(margin > 0)
      CL_SD -= margin; 

    carry_distance_sd = CL_SD;
    if(carry_distance_sd < 0)
    {
      printf("VADD_Compute_Subedge_Delay_Standard_Deviation(): Error: carry_distance_sd(%f) is negative\n", (float)carry_distance_sd);
      exit(1);
    }

    /* compute edge delay standard deviation based on TBD edge delay model */
    delay_standard_deviation = carry_distance_sd/v; //since the carry delay is order-of-magnitude longer than the forwarding delay, we consider just carry_distance_sd for the calculation of delay_standard_deviation for the edge

    break;

  case VANET_EDGE_DELAY_VADD_MODEL:
    /** NOT IMPLEMENTED! */
    //printf("VADD_Compute_Subedge_Delay_Standard_Deviation(): VANET_EDGE_DELAY_VADD_MODEL is not implemented yet!\n");
    //exit(0);

    delay_standard_deviation = l/v;

    break;

  case VANET_EDGE_DELAY_TSF_MODEL:
    /** compute Component Length Variance (CL_VAR) for the directional edge where the vehicle is moving */

    /* check whether l > R or not */
    if(l > R)
    {
        lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
        beta = 1 - exp(-1*lambda*a);

		/* check the validity of beta
		 * @Note: if beta is close to 0, we regard the delay as l/v */
		if(beta <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
		{
		  	/* compute edge delay standard deviation as carry delay */
			delay_standard_deviation = l/v;
			break;
		}

        rho = 1/lambda - (a + 1/lambda)*exp(-1*lambda*a);
        alpha = v*(1 - beta)*rho;
        //alpha = v*(1 - beta);

		/* check the validity of alpha 
		 * @Note: if alpha is close to 0, we regard the delay as l/v */
		if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
		{
	  		/* compute edge delay standard deviation as carry delay */
			delay_standard_deviation = l/v;
			break;
		}

        delta = -1*pow(a, 2)*(1 - beta) + 2*rho/lambda;

        N = ceil(beta*(1-beta)/alpha*(l-R)); //N is the index where the expected component length is greater than l-R
        //N = ceil(beta*(1-beta)/alpha*l); //N is the index where the expected component length is greater than l
    
        B = (N-1)*pow(beta,N) - N*pow(beta,N-1) + 1;
        if(B < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
            B = 0; //Note that B may be a extremely small number, such as 1*10^(-3). In this case, we regard B as zero.

        A = (N*(N-1)*pow(beta,N-1) - N*(N-1)*pow(beta,N-2))*pow(1-beta,2) + 2*B*(1-beta);
        if(A < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC) //Note that A may be a extremely small number, such as 1*10^(-3). In this case, we regard A as zero.
            A = 0;

        ACL = E_L_1 = alpha/pow(1-beta,2)*B + (l-R)*pow(beta,N); //When B is close to zero, we ignore the term related to B
        //ACL = E_L_1 = alpha/pow(1-beta,2)*B + l*pow(beta,N); //When B is close to zero, we ignore the term related to B
        //E_L_1 = alpha*B/pow(1-beta,2) + l*pow(beta,N);

        E_L_2 = pow(v,2)*delta*(1-beta)/pow(1-beta,2)*B + pow(v*rho,2)*(1-beta)/pow(1-beta,4)*A + pow(l-R,2)*pow(beta,N); //When A or B is close to zero, we ignore the terms related to A or B
        //E_L_2 = pow(v,2)*delta*(1-beta)/pow(1-beta,2)*B + pow(v*rho,2)*(1-beta)/pow(1-beta,4)*A + pow(l,2)*pow(beta,N); //When A or B is close to zero, we ignore the terms related to A or B
        //E_L_2 = pow(v,2)*delta*(1-beta)/pow(1-beta,2)*B + pow(v*rho,2)/pow(1-beta,4)*A + pow(l,2)*pow(beta,N); //When A or B is close to zero, we ignore the terms related to A or B
    }
    else
    {
		//ACL = E_L_1 = l;
		//E_L_2 = pow(l, 2);

	  	/* compute edge delay standard deviation as carry delay */
		delay_standard_deviation = l/v;
		break;
    }

    /** compute the variance of link delay */
    E_D_2 = (pow(l-R,2) - 2*(l-R)*E_L_1 + E_L_2)*beta/pow(v,2) + pow(1/lambda+(l-R)/v,2)*(1-beta); 
    //E_D_2 = (pow(l-R,2) - 2*(l-R)*E_L_1 + pow(E_L_2,2))*beta/pow(v,2) + pow(1/lambda+(l-R)/v,2)*(1-beta); //@Note that this has an error in  pow(E_L_2,2) and that the correct one is E_L_2 without squaring; that is why the previous actual variance is much greater than the expected variance

    E_D_1 = (l-R-E_L_1)*beta/v + (1/lambda+(1-R)/v)*(1-beta);

    LD_VAR = E_D_2 - pow(E_D_1,2);

    LD_SD = sqrt(LD_VAR);
    LD_SD = MIN_In_Double((l-ACL)/v, LD_SD); //let LD_SD less than (l-ACL)/v => Check whether this LD_SD is correct or not

    /* compute edge delay standard deviation based on TSF edge delay model */
    delay_standard_deviation = LD_SD; //since the carry delay is order-of-magnitude longer than the forwarding delay, we consider just carry_distance_sd for the calculation of delay_standard_deviation for the edge

    if(delay_standard_deviation < 0)
    {
        printf("VADD_Compute_Subedge_Delay_Standard_Deviation(): Error: link_delay_sd(%f) is negative\n", (float)delay_standard_deviation);
      exit(1);
    }
    break;

  default:
    printf("VADD_Compute_Subedge_Delay_Standard_Deviation(): vehicle_vanet_edge_delay_model(%d) is not supported yet!\n", param->vehicle_vanet_edge_delay_model);  
    exit(1);
  }
 
  return delay_standard_deviation;
}
double VADD_Compute_Average_Convoy_Length_Standard_Deviation(parameter_t *param, struct_graph_node *pGraphNode)
{ //compute the Average Convoy Length (ACL) Standard Deviation for the road segment r_ij with head node pGraphNode->vertex
  double delay_standard_deviation = 0; //edge delay standard deviation
  //double delay = edge_delay; //edge delay
  double lambda = 0; //vehicle arrival rate per second
  //double rho = 0; //vehicle density on road segment r_ij
  double v = param->vehicle_speed; //average vehicle speed on road segment r_ij
  double l = pGraphNode->weight; //road segment length
  double R = param->communication_range; //radio communication range
  double c = param->communication_one_hop_delay; //average one-hop packet transmission delay for the radio communication range; unit is second.
  double ACL = 0; //average component length (ACL), also called average convoy length
  double CL_VAR = 0; //Convoy Length Variance (CL_VAR) used as forwarding distance
  double CL_SD = 0; //Convoy Length Standard Deviation (CL_SD) used as forwarding distance
  double a = R/v; //movement time within the communication range R with the speed v
  double b = (1/lambda - (a + 1/lambda)*exp(-1*lambda*a)) / (1 - exp(-1*lambda*a)); //the expected interarrival time of two vehicles in the case where two vehicles are connected by the communication range
  double gamma = 0; //gamma for CL_VAR computation
  double rho = 0; //rho for ACL and CL_VAR computation
  double alpha = 0; //alpha for ACL computation
  double delta = 0; //delta for CL_VAR computation
  double beta = 0; //beta for CL_VAR computation
  double A = 0, B = 0; //coefficients
  double N = 0; //index N for CL_VAR computation
  double margin = 0; //length by which the sum of ACL and CL_SD is longer than the road length l;
  double E_L_2 = 0; //second moment of the component length
  double E_L_1 = 0; //first moment of the component length, that is, ACL
  double forwarding_distance_sd = 0; //forwarding distance standard deviation

  /* check the vehicle arrival for the directional edge:
     Note that for the directional edge with no vehicle arrival, the edge's mean link delay is 
     the edge's length divided by mean vehicle speed and the link delay standard deviation is zero.*/
  if(pGraphNode->number_of_interarrivals == 0) 
  //if(pGraphNode->mean_interarrival_time == INF)
  { //since there is no vehicle arrival, this link delay's standard deviation should be zero
    delay_standard_deviation = 0;
    return delay_standard_deviation;
  }
  else
  {
    /* compute the mean interarrival time for the road segment whose head node is pGraphNode */
    pGraphNode->mean_interarrival_time = pGraphNode->sum_of_interarrival_time / pGraphNode->number_of_interarrivals; 
  }

  /** check the validity of parameters */
  /* make sure that communication range R is positive */
  if(R == 0)
  {
	printf("VADD_Compute_Average_Convoy_Length_Standard_Deviation(): Error: R(%.2f) has a wrong value\n", (float)R);
	exit(1);
  }

  switch(param->vehicle_vanet_edge_delay_model)
  {
  case VANET_EDGE_DELAY_TBD_MODEL_FOR_FINITE_ROAD:
    /** compute Component Length Variance (CL_VAR) for the directional edge where the vehicle is moving */

    lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
    beta = 1 - exp(-1*lambda*a);
    rho = 1/lambda - (a + 1/lambda)*exp(-1*lambda*a);
    alpha = v*(1 - beta)*rho;
    //alpha = v*(1 - beta);

	/* check the validity of alpha 
	 * @Note: if alpha is close to 0, we regard the delay as l/v */
	if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
	{
	  	/* compute ACL standard deviation as 0 */
		delay_standard_deviation = 0;
		break;
	}

    delta = -1*pow(a, 2)*(1 - beta) + 2*rho/lambda;

    N = ceil(beta*(1-beta)/alpha*l); //N is the index where the expected component length is greater than l
    
    B = (N-1)*pow(beta,N) - N*pow(beta,N-1) + 1;
    if(B < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
        B = 0; //Note that B may be a extremely small number, such as 1*10^(-3). In this case, we regard B as zero.

    A = (N*(N-1)*pow(beta,N-1) - N*(N-1)*pow(beta,N-2))*pow(1-beta,2) + 2*B*(1-beta);
    if(A < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC) //Note that A may be a extremely small number, such as 1*10^(-3). In this case, we regard A as zero.
        A = 0;

    ACL = E_L_1 = alpha/pow(1-beta,2)*B + l*pow(beta,N); //When B is close to zero, we ignore the term related to B
    //E_L_1 = alpha*B/pow(1-beta,2) + l*pow(beta,N);

    E_L_2 = pow(v,2)*delta*(1-beta)/pow(1-beta,2)*B + pow(v*rho,2)*(1-beta)/pow(1-beta,4)*A + pow(l,2)*pow(beta,N); //When A or B is close to zero, we ignore the terms related to A or B
    //E_L_2 = pow(v,2)*delta*(1-beta)/pow(1-beta,2)*B + pow(v*rho,2)/pow(1-beta,4)*A + pow(l,2)*pow(beta,N); //When A or B is close to zero, we ignore the terms related to A or B
    //////////////////////////////

    CL_VAR = E_L_2 - pow(E_L_1, 2);

    CL_SD = sqrt(CL_VAR);
    CL_SD = MIN_In_Double(ACL, CL_SD); //let CL_SD less than ACL

	forwarding_distance_sd = CL_SD;
    break;

  case VANET_EDGE_DELAY_TBD_MODEL_FOR_INFINITE_ROAD:
    /** compute Component Length Variance (CL_VAR) for the directional edge with the infinite length where the vehicle is moving */

    lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
    beta = 1 - exp(-1*lambda*a);
    rho = 1/lambda - (a + 1/lambda)*exp(-1*lambda*a);
    alpha = v*(1 - beta)*rho;

	/* check the validity of alpha 
	 * @Note: if alpha is close to 0, we regard the delay as l/v */
	if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
	{
	  	/* compute ACL standard deviation as 0 */
		delay_standard_deviation = 0;
		break;
	}

    delta = -1*pow(a, 2)*(1 - beta) + 2*rho/lambda;
    gamma = pow(v, 2)*(1 - beta)*delta;

    ACL = E_L_1 = alpha/pow(1-beta, 2);
    E_L_2 = gamma/pow(1-beta, 2) + 2*pow(alpha/pow(1-beta, 2), 2);
    ////////////////////////////////

    CL_VAR = E_L_2 + pow(E_L_1, 2);

    CL_SD = sqrt(CL_VAR);

    ACL = MIN_In_Double(ACL, l); //we let ACL be at most l

    CL_SD = MIN_In_Double(ACL, CL_SD); //let CL_SD less than ACL

	forwarding_distance_sd = CL_SD;
    break;

  case VANET_EDGE_DELAY_VADD_MODEL:
    /** NOT IMPLEMENTED! */
    //printf("VADD_Compute_Edge_Delay_Standard_Deviation(): VANET_EDGE_DELAY_VADD_MODEL is not implemented yet!\n");
    //exit(0);

	forwarding_distance_sd = 0;
    break;

  case VANET_EDGE_DELAY_TSF_MODEL:
    /** compute Component Length Variance (CL_VAR) for the directional edge where the vehicle is moving */

    /* check whether l > R or not */
    if(l > R)
    {
        lambda = VADD_Vehicle_Lambda_2(pGraphNode->mean_interarrival_time); //lambda = 1/mean_interarrival_time
        beta = 1 - exp(-1*lambda*a);

		/* check the validity of beta
		 * @Note: if beta is close to 0, we regard the ACL as 0 */
		if(beta <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
		{
		  	/* compute ACL standard deviation as 0 */
			delay_standard_deviation = 0;
			break;
		}

        rho = 1/lambda - (a + 1/lambda)*exp(-1*lambda*a);
        alpha = v*(1 - beta)*rho;
        //alpha = v*(1 - beta);

		/* check the validity of alpha 
		 * @Note: if alpha is close to 0, we regard the delay as l/v */
		if(alpha <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
		{
	  		/* compute ACL standard deviation as 0 */
			delay_standard_deviation = 0;
			break;
		}

        delta = -1*pow(a, 2)*(1 - beta) + 2*rho/lambda;

        N = ceil(beta*(1-beta)/alpha*(l-R)); //N is the index where the expected component length is greater than l-R
        //N = ceil(beta*(1-beta)/alpha*l); //N is the index where the expected component length is greater than l
    
        B = (N-1)*pow(beta,N) - N*pow(beta,N-1) + 1;
        if(B < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
            B = 0; //Note that B may be a extremely small number, such as 1*10^(-3). In this case, we regard B as zero.

        A = (N*(N-1)*pow(beta,N-1) - N*(N-1)*pow(beta,N-2))*pow(1-beta,2) + 2*B*(1-beta);
        if(A < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC) //Note that A may be a extremely small number, such as 1*10^(-3). In this case, we regard A as zero.
            A = 0;

        ACL = E_L_1 = alpha/pow(1-beta,2)*B + (l-R)*pow(beta,N); //When B is close to zero, we ignore the term related to B
        //ACL = E_L_1 = alpha/pow(1-beta,2)*B + l*pow(beta,N); //When B is close to zero, we ignore the term related to B
        //E_L_1 = alpha*B/pow(1-beta,2) + l*pow(beta,N);

        E_L_2 = pow(v,2)*delta*(1-beta)/pow(1-beta,2)*B + pow(v*rho,2)*(1-beta)/pow(1-beta,4)*A + pow(l-R,2)*pow(beta,N); //When A or B is close to zero, we ignore the terms related to A or B
        //E_L_2 = pow(v,2)*delta*(1-beta)/pow(1-beta,2)*B + pow(v*rho,2)*(1-beta)/pow(1-beta,4)*A + pow(l,2)*pow(beta,N); //When A or B is close to zero, we ignore the terms related to A or B
        //E_L_2 = pow(v,2)*delta*(1-beta)/pow(1-beta,2)*B + pow(v*rho,2)/pow(1-beta,4)*A + pow(l,2)*pow(beta,N); //When A or B is close to zero, we ignore the terms related to A or B

    	CL_VAR = E_L_2 - pow(E_L_1, 2);

	    CL_SD = sqrt(CL_VAR);
    	CL_SD = MIN_In_Double(ACL, CL_SD); //let CL_SD less than ACL

		forwarding_distance_sd = CL_SD;
    }
    else
    {
		//ACL = E_L_1 = l;
		//E_L_2 = pow(l, 2);

		/* compute edge delay standard deviation as carry delay */
		forwarding_distance_sd = 0;
    }
    break;

  default:
    printf("VADD_Compute_Average_Convoy_Length_Standard_Deviation(): vehicle_vanet_edge_delay_model(%d) is not supported yet!\n", param->vehicle_vanet_edge_delay_model);  
    exit(1);
  }
 
  return forwarding_distance_sd;
}

/** Functions for Data Forwarding **/

boolean VADD_Is_Within_AP_Communication_Range(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, struct_graph_node **ap_graph_node)
{ //check whether this vehicle is within the communication range of an Internet access point and return the pointer to the graph node corresponding to the access point through *ap_graph_node
  /*@Note: in order to match the actual delivery delay with the expected delivery delay, we let the vehicle be able to transmit its packets to AP when the distance between the vehicle and the AP is close to zero; actually, we can allow the vehicle to be able to transmit when it approaches the AP by the communication range. */

  boolean result = FALSE;
  boolean flag = FALSE;
  char *tail_node = vehicle->path_ptr->vertex; //tail node of the directional edge where vehicle is moving
  char *head_node = vehicle->path_ptr->next->vertex; //head node of the directional edge where vehicle is moving
  double distance = 0; //distance between vehicle and access point
  int node_id = 0; //graph node id
  MOVE_TYPE move_type = vehicle->move_type; //movement type to indicate which direction is taken by the vehicle on the undirectional edge 

  if(move_type == MOVE_FORWARD)
  {
    /* check whether or not the vehicle's head_node of the directional edge is one of access points */
    flag = IsVertexInTrafficTable(ap_table, head_node);
    if(flag)
    {    
      distance = fabs(vehicle->edge_length - vehicle->current_pos_in_Gr.offset);
    
      /* check whether or not the distance is greater than the communication range of access point */    
      if(distance <= param->communication_range)
        result = TRUE;

      /*
      if(distance <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
        result = TRUE;
      else
        return result;
      */
         
      /* obtain the pointer to the graph node corresponding to head_node */
      node_id = atoi(head_node);
      *ap_graph_node = &(G[node_id-1]);
    }

    /* check whether or not the vehicle's tail_node of the directional edge is one of access points */
    flag = IsVertexInTrafficTable(ap_table, tail_node);
    if(flag)
    {    
      distance = vehicle->current_pos_in_Gr.offset;
    
      /* check whether or not the distance is greater than the communication range of access point */    
      if(distance <= param->communication_range)
        result = TRUE;

      /*
      if(distance <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
        result = TRUE;
      */
         
      /* obtain the pointer to the graph node corresponding to tail_node */
      node_id = atoi(tail_node);
      *ap_graph_node = &(G[node_id-1]);
    }
  }
  else if(move_type == MOVE_BACKWARD)
  {
    /* check whether or not the vehicle's head_node of the directional edge is one of access points */
    flag = IsVertexInTrafficTable(ap_table, head_node);
    if(flag)
    {
      distance = vehicle->current_pos_in_Gr.offset;

      /* check whether or not the distance is greater than the communication range of access point */
      if(distance <= param->communication_range)
        result = TRUE;

      /*
      if(distance <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
        result = TRUE;
      */
         
      /* obtain the pointer to the graph node corresponding to head_node */
      node_id = atoi(head_node);
      *ap_graph_node = &(G[node_id-1]);
    }

    /* check whether or not the vehicle's tail_node of the directional edge is one of access points */
    flag = IsVertexInTrafficTable(ap_table, tail_node);
    if(flag)
    {
      distance = fabs(vehicle->edge_length - vehicle->current_pos_in_Gr.offset);

      /* check whether or not the distance is greater than the communication range of access point */
      if(distance <= param->communication_range)
        result = TRUE;

      /*
      if(distance <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
        result = TRUE;
      */
         
      /* obtain the pointer to the graph node corresponding to tail_node */
      node_id = atoi(tail_node);
      *ap_graph_node = &(G[node_id-1]);
    }
  }      
  else
  {
    printf("VADD_Is_Within_AP_Communication_Range(): move_type(%d) is invalid!\n", vehicle->move_type);
    exit(1);
  }

  return result;
}

boolean VADD_Is_Within_Destination_Vehicle_Communication_Range(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, destination_vehicle_queue_t *Q, struct_vehicle_t **destination_vehicle)
{ //check whether this vehicle is within the communication range of one of destination vehicles in destination vehicle queue Q and return the pointer to the destination vehicle

  boolean result = FALSE;
  char *tail_node = vehicle->path_ptr->vertex; //tail node of the directional edge where vehicle is moving
  char *head_node = vehicle->path_ptr->next->vertex; //head node of the directional edge where vehicle is moving
  double distance = 0; //distance between vehicle and access point
  int node_id = 0; //graph node id
  MOVE_TYPE move_type = vehicle->move_type; //movement type to indicate which direction is taken by the vehicle on the undirectional edge 
  destination_vehicle_queue_node_t *pQueueNode = NULL; //pointer to a destination vehicle queue node
  int i = 0; //index i

  /* initialize destination vehicle with NULL */
  *destination_vehicle = NULL;

  /* search a destination vehicle within the communication range of vehicle */
  pQueueNode = &(Q->head);
  for(i = 0; i < Q->size; i++)
  {
    pQueueNode = pQueueNode->next;

    if(pQueueNode->vnode->id == vehicle->id)
      continue;

    if(pQueueNode->vnode->current_pos_in_Gr.eid == vehicle->current_pos_in_Gr.eid)
    { //two vehicles are moving on the same edge on the road network graph
      distance = fabs(pQueueNode->vnode->current_pos_in_Gr.offset - vehicle->current_pos_in_Gr.offset);
      if(distance <= param->communication_range)
      {
	*destination_vehicle = pQueueNode->vnode;
	result = TRUE;
	break;
      }
    }
  }

  return result;
}

int VADD_Iterative_Forward_Packet_To_Next_Carrier_On_Road_Segment(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, forwarding_table_queue_t *FTQ, packet_delivery_statistics_t *packet_delivery_statistics)
{ //The packet holder vehicle (i.e., vehicle) tries to send its packets including the new packet to a better carrier. Also, iteratively, these packets are transmitted to the next carrier until the packets cannot be forwarded to the next carrier.

  int transmission_component_number = 0; //number of the connected components to forward the packets
  boolean flag = TRUE; //flag to determine whether to continue the iterative forwarding
  boolean flag2 = FALSE; //flag to determine whether the vehicle is within an intersection area
  intersection_area_type_t intersection_area_type; //intersection area type to tell which part the vehicle is located between the tail intersection and the head intersection
  struct_vehicle_t *next_carrier = NULL; //pointer to the next carrier vehicle
  int tail_intersection_id = 0; //tail intersection id of an edge
  int head_intersection_id = 0; //head intersection id of an edge
  int discard_count = 0; //count for discarded packets

  /**@for debugging */
  /*
  boolean debug_flag = FALSE;
  if(current_time >= 22781.13 && vehicle->id == 6)
  {
    printf("VADD_Iterative_Forward_Pa.cket_To_Next_Carrier_On_Road_Segment(): current_time = %.2f, vid = %d\n", (float)current_time, vehicle->id);
    debug_flag = TRUE;
  }
  */
  /******************/

  while(flag)
  {
    flag = VADD_Is_There_Next_Carrier_On_Road_Segment(param, current_time, vehicle, G, G_size, &next_carrier);

    /**@for debugging */
    /*
    if(flag && debug_flag)
    {
      printf("src vid=%d, dst vid=%d, src position=(%d,%.2f), dst position=(%d,%.2f)\n", vehicle->id, next_carrier->id, vehicle->current_pos_in_digraph.eid, vehicle->current_pos_in_digraph.offset, next_carrier->current_pos_in_digraph.eid, next_carrier->current_pos_in_digraph.offset);
    }
    */
    /******************/

    if(flag == TRUE) //if-1
    {
      VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier, packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
      vehicle = next_carrier; //change vehicle with next_carrier as a new source
      transmission_component_number++;
    } //end of if-1
    else if(VADD_Is_Within_Intersection_Area(param, vehicle, G, G_size, &intersection_area_type, &tail_intersection_id, &head_intersection_id) == TRUE)
    { //In the case where vehicle is the convoy leader, check whether the vehicle is within an intersection area corresponding to the upcoming intersection. If the vehicle is within the intersection area, it lets the vehicle's leader send its packets to the convoy leader of another convoy in another road segment 
      flag2 = VADD_Is_There_Next_Carrier_At_Both_Intersection_Areas(param, current_time, vehicle, G, G_size, FTQ, intersection_area_type, &next_carrier); //check whether this vehicle can forward with the communication to the other vehicle as next carrier around the upcoming intersection
      if(flag2) //if-4.1.1
      {
		VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier, packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier

        vehicle = next_carrier; //change vehicle with next_carrier as a new source
        transmission_component_number++;
      }		
    }
  }

  return transmission_component_number;
}

boolean VADD_Is_There_Next_Carrier_On_Road_Segment(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier)
{ //determine whether to forward its packets to next carrier moving on the current road segment and return the pointer to the next carrier through *next_carrier; Note that the convoy leader becomes the next carrier 
	boolean result = FALSE;

	/** select a data forwarding method according to two-way forwarding flag */
	if(param->data_forwarding_two_way_forwarding_flag)
	{ /* Data forwarding for two-way road segment */
		result = VADD_Is_There_Next_Carrier_On_Two_Way_Road_Segment(param, current_time, vehicle, G, G_size, next_carrier);
	}
	else
	{ /* Data forwarding for one-way road segment */
		result = VADD_Is_There_Next_Carrier_On_One_Way_Road_Segment(param, current_time, vehicle, G, G_size, next_carrier);
	}

	return result;
}

boolean VADD_Is_There_Next_Carrier_On_One_Way_Road_Segment(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier)
{ //For one-way road segment, determine whether to forward its packets to next carrier moving on the current road segment and return the pointer to the next carrier through *next_carrier; Note that the convoy leader becomes the next carrier 
	boolean result = FALSE;
	boolean flag = FALSE; //flag to indicate whether a next carrier is determined or not
	double distance = 0; //distance between vehicle and another vehicle moving on the same directional edge
	double min_neighbor_EDD = INF; //neighbor vehicle or neighbor convoy with a minimum EDD
	double max_neighbor_offset = -1; //neighbor's offset such that the distance between vehicle and preceding neighbor vehicle is maximum
	char *tail_node = vehicle->path_ptr->vertex; //tail node of the directional edge where vehicle is moving
	char *head_node = vehicle->path_ptr->next->vertex; //head node of the directional edge where vehicle is moving
	directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
	vehicle_movement_queue_node_t *pMoveNode = NULL; //pointer to the vehicle movement queue
	int i = 0; //index for for-loop
	int size = 0; //size of vehicle movement queue
	double offset_in_undirectional_edge = 0; //vehicle's offset on the undirectional edge
	double offset_in_directional_edge = 0; //vehicle's offset on the directional edge

	int source_intersection_id = atoi(tail_node); //the id of the tail intersection of the directed edge where the packet carrier is moving

	struct_graph_node *Gr_global = param->vanet_table.Gr; //pointer to the global road network graph having the global directional queue DEr to manage the clusters of vehicles 
	int Gr_global_size = param->vanet_table.Gr_size; //size of Gr_global

	/** check the validity of the global road network graph Gr_global and its size Gr_global_size */
	if(Gr_global == NULL)
	{
		printf("%s:%d: Gr_global is NULL!\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}
	else if(Gr_global_size <= 0)
	{
		printf("%s:%d: Gr_global_size(%d) is not positive!\n",
				__FUNCTION__, __LINE__,
				Gr_global_size);
		exit(1);
	}

	/**@for debugging */
	//printf("VADD_Is_There_Next_Carrier_On_Road_Segment(): at time=%f, vid=%d\n", (float)vehicle->state_time, vehicle->id);
	//if(vehicle->id == 1 && vehicle->state_time >= 3602)
		//  printf("VADD_Is_There_Next_Carrier_On_Road_Segment(): trace vid=1\n");
	/******************/

	/** reset *next_carrier to NULL */
	*next_carrier = NULL;

	/**@ Search the next carrier on the directional edge where vehicle is moving **/

	/** obtain the pointer to the directional edge of <tail_node,head_node> */

#if 1 /* [ */
	pEdgeNode = FastLookupDirectionalEdgeQueue(Gr_global, tail_node, head_node);
#else	
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
#endif /* ] */
	if(pEdgeNode == NULL)
	{
		printf("%s:%d: pEdgeNode for <%s,%s> is NULL\n", 
				__FUNCTION__, __LINE__,
				tail_node, head_node);
		exit(1);
	}

#if 1 /* [ */
	/* update the EDDs and EDD_SDs of vehicle and its convoy head for V2V data delivery */
	if((param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V) && (param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD || param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC))
	{
		VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, vehicle, G, G_size, source_intersection_id);
		VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, vehicle->ptr_convoy_queue_node->leader_vehicle, G, G_size, source_intersection_id);
	}

	/* set min_neighbor_EDD according to param's data_forwarding_mode and vehicle_vanet_forwarding_type */
	min_neighbor_EDD = VADD_Get_Initial_Minimum_Neighbor_EDD(param, vehicle);
#endif /* ] */

	/** compute the offset in directional edge */
	offset_in_undirectional_edge = vehicle->current_pos_in_Gr.offset;
	if(vehicle->move_type == MOVE_FORWARD)
	{
		offset_in_directional_edge = offset_in_undirectional_edge;
	}    
	else if(vehicle->move_type == MOVE_BACKWARD)
	{
		offset_in_directional_edge = vehicle->edge_length - offset_in_undirectional_edge;
	}
	else
	{	
		printf("%s:%d: vehicle->move_type(%d) is invalid\n", 
				__FUNCTION__, __LINE__,
				vehicle->move_type);
		exit(1);
	}
  
	size = pEdgeNode->vehicle_movement_list.size;
	pMoveNode = &(pEdgeNode->vehicle_movement_list.head);
	for(i = 0; i < size && flag == FALSE; i++) //for-1
	{
		pMoveNode = pMoveNode->next;
		if(vehicle->id == pMoveNode->vid)
			continue;

		distance = fabs(pMoveNode->offset - offset_in_directional_edge);

		/* choose a next carrier with smaller EDD according to forwording type */
		switch(param->vehicle_vanet_forwarding_type) //switch-1
		{
			case VANET_FORWARDING_BASED_ON_VEHICLE:
				if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
				{
					if((distance <= param->communication_range) && (pMoveNode->vnode->EDD < min_neighbor_EDD))
					//if((distance <= param->communication_range) && (pMoveNode->vnode->EDD < vehicle->EDD) && (pMoveNode->vnode->EDD < min_neighbor_EDD))
					{ //we also check vehicles' EDDs
						min_neighbor_EDD = pMoveNode->vnode->EDD;
						*next_carrier = pMoveNode->vnode; //update the pointer of neighbor vehicle node with smaller EDD; Note in VADD, a vehicle with farther offset is chosen as next carrier.
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
				{ 
					if((distance <= param->communication_range) && (pMoveNode->vnode->EDD_for_download < min_neighbor_EDD))
					{ //we also check vehicles' EDDs
						min_neighbor_EDD = pMoveNode->vnode->EDD_for_download;
						*next_carrier = pMoveNode->vnode; //update the pointer of neighbor vehicle node with smaller EDD; Note in VADD, a vehicle with farther offset is chosen as next carrier.
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
				{
					if(param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD || param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC)
					{
						if(distance <= param->communication_range)
						{ //we also check vehicles' EDDs
							/* update the vehicle's EDD and EDD_SD_for_V2V */
							VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, pMoveNode->vnode, G, G_size, source_intersection_id);

							if(pMoveNode->vnode->EDD_for_V2V < min_neighbor_EDD)
							{
								min_neighbor_EDD = pMoveNode->vnode->EDD_for_V2V;
								*next_carrier = pMoveNode->vnode; //update the pointer of neighbor vehicle node with smaller EDD; Note in VADD, a vehicle with farther offset is chosen as next carrier.
							}
						}
					}
					else
					{
						printf("%s:%d: param->vanet_forwarding_scheme(%d) is not supported!\n ",
								__FUNCTION__, __LINE__,
								param->vanet_forwarding_scheme);
						exit(1);
					}
				}
				break;

			case VANET_FORWARDING_BASED_ON_CONVOY:
				/** check whether vehicle_vanet_stationary_vehicle_flag is turned on */
				if(param->vehicle_vanet_stationary_vehicle_flag && vehicle->id == 1 && distance <= param->communication_range)
				{
					//*next_carrier = pMoveNode->vnode;
					//*next_carrier = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
					/*@ Note: we need to consider the communication delay between the sensor and the leader */
					*next_carrier = pMoveNode->vnode->ptr_convoy_queue_node->tail_vehicle;
					flag = TRUE; //[03/18/09] set flag to let the for-loop terminate

					//@ Note: if the next carrier is convoy leader and the leader is the vehicle of vid=1 (i.e., the sender), then the packet cannot be transmitted to another vehicle, so the next carrier must be another vehicle.
					//result = TRUE;
					//return result;
					break;
				}

				/** update the pointer of neighbor vehicle node:
					Note that we check whether next_carrier candidate's EDD is less than
					vehicle's EDD
				*/
				if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
				{
					if((distance <= param->communication_range) && (pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid) && (pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle->EDD < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD))
					//if(distance <= param->communication_range)
					//if((distance <= param->communication_range) && (pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle->EDD < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD) && (pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle->EDD < min_neighbor_EDD))
					{ 
						/* update min_neighbor's EDD with pMoveNode's convoy leader's EDD */
						// min_neighbor_EDD = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle->EDD;

						/* set next_carrier to pMoveNode->vnode */
						*next_carrier = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
						flag = TRUE; //[03/18/09] set flag to let the for-loop terminate

						//@ Note: we need to consider the communication delay between the packet source vehicle and the next_carrier
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
				{
					if((distance <= param->communication_range) && (pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid))
					{
						*next_carrier = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
						flag = TRUE; //[03/18/09] set flag to let the for-loop terminate
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
				{
					if(param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD)
					{
						if(distance <= param->communication_range) 
						{
							/* update the vehicle's EDD and EDD_SD_for_V2V */
							VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle, G, G_size, source_intersection_id);

							if((pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid) && (pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle->EDD_for_V2V < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD_for_V2V))
							{
								*next_carrier = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
								flag = TRUE;
							}
						}
					}
					else if(param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC)
					{
						if((distance <= param->communication_range) && (pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid))
						{
							*next_carrier = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
							flag = TRUE;
						}
					}
					else
					{
						printf("%s:%d: param->vanet_forwarding_scheme(%d) is not supported!\n ",
								__FUNCTION__, __LINE__,
								param->vanet_forwarding_scheme);
						exit(1);
					}
				}
				break;

			default:
				printf("%s:%d: vanet forwarding type(%d) is not known!\n", 
						__FUNCTION__, __LINE__,
						param->vehicle_vanet_forwarding_type);
				exit(1);
		} //end of switch-1
	} //end of for-1
   
	/** [03/18/09] set the next_carrier to the vehicle's convoy leader when there exists no next carrier candidate in a different convoy with less EDD */
	if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY)
	{
		if((flag == FALSE) && (vehicle->id != vehicle->ptr_convoy_queue_node->leader_vehicle->id))
		{
			*next_carrier = vehicle->ptr_convoy_queue_node->leader_vehicle;      
		}
	}
	/******************************************************************************/

	/** check whether a next carrier vehicle exists */
	if(*next_carrier != NULL)
	{
		if(vehicle == *next_carrier)
		{ //if the destination vehicle is vehicle, we don't let vehicle send its packets to itself 
			printf("%s:%d: Error: vehicle(id=%d) is the same as next_carrier\n", 
					__FUNCTION__, __LINE__,
					vehicle->id);
			exit(1);
		}
		else
			result = TRUE;
	}

	return result;
}

boolean VADD_Is_There_Next_Carrier_On_Two_Way_Road_Segment(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier)
{ //For two-way road segment, determine whether to forward its packets to next carrier moving on the current road segment and return the pointer to the next carrier through *next_carrier; Note that the convoy leader becomes the next carrier 
	boolean result = FALSE;
	boolean flag = FALSE; //flag to indicate whether a next carrier is determined or not
	double distance = 0; //distance between vehicle and another vehicle moving on the same directional edge
	double min_neighbor_EDD = INF; //neighbor vehicle or neighbor convoy with a minimum EDD
	double max_neighbor_offset = -1; //neighbor's offset such that the distance between vehicle and preceding neighbor vehicle is maximum
	char *tail_node = vehicle->path_ptr->vertex; //tail node of the directional edge where vehicle is moving
	char *head_node = vehicle->path_ptr->next->vertex; //head node of the directional edge where vehicle is moving
	directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
	vehicle_movement_queue_node_t *pMoveNode = NULL; //pointer to the vehicle movement queue
	int i = 0; //index for for-loop
	int size = 0; //size of vehicle movement queue
	double offset_in_undirectional_edge = 0; //vehicle's offset on the undirectional edge
	double offset_in_directional_edge = 0; //vehicle's offset on the directional edge
	struct_vehicle_t *next_carrier_candidate_1 = NULL; //next carrier candidate on the same directional edge for vehicle
	struct_vehicle_t *next_carrier_candidate_2 = NULL; //next carrier candidate on the opposite directional edge for vehicle

	int source_intersection_id = atoi(tail_node); //the id of the tail intersection of the directed edge where the packet carrier is moving

	struct_graph_node *Gr_global = param->vanet_table.Gr; //pointer to the global road network graph having the global directional queue DEr to manage the clusters of vehicles 
	int Gr_global_size = param->vanet_table.Gr_size; //size of Gr_global

	/**@for debugging */
	//printf("VADD_Is_There_Next_Carrier_On_Road_Segment(): at time=%f, vid=%d\n", (float)vehicle->state_time, vehicle->id);
	//if(vehicle->id == 1 && vehicle->state_time >= 3602)
	//  printf("VADD_Is_There_Next_Carrier_On_Road_Segment(): trace vid=1\n");
	/******************/

	/** reset *next_carrier to NULL */
	*next_carrier = NULL;

	/**@ Search the next carrier on the directional edge where vehicle is moving **/

	/** obtain the pointer to the directional edge of <tail_node,head_node> */
#if 1 /* [ */
	pEdgeNode = FastLookupDirectionalEdgeQueue(Gr_global, tail_node, head_node);
#else	
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
#endif /* ] */
	if(pEdgeNode == NULL)
	{
		printf("%s:%d: pEdgeNode for <%s,%s> is NULL\n", 
				__FUNCTION__, __LINE__,
				tail_node, head_node);
		exit(1);
	}

#if 1 /* [ */
	/* update the EDDs and EDD_SDs of vehicle and its convoy head for V2V data delivery */
	if((param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V) && (param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD || param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC))
	{
		VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, vehicle, G, G_size, source_intersection_id);
		VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, vehicle->ptr_convoy_queue_node->leader_vehicle, G, G_size, source_intersection_id);
	}

	/* set min_neighbor_EDD according to param's data_forwarding_mode and vehicle_vanet_forwarding_type */
	min_neighbor_EDD = VADD_Get_Initial_Minimum_Neighbor_EDD(param, vehicle);
#endif /* ] */

	/** compute the offset in directional edge */
	offset_in_undirectional_edge = vehicle->current_pos_in_Gr.offset;
	if(vehicle->move_type == MOVE_FORWARD)
	{
		offset_in_directional_edge = offset_in_undirectional_edge;
	}    
	else if(vehicle->move_type == MOVE_BACKWARD)
	{
		offset_in_directional_edge = vehicle->edge_length - offset_in_undirectional_edge;
	}
	else
	{
		printf("%s:%d: vehicle->move_type(%d) is invalid\n", 
				__FUNCTION__, __LINE__,
				vehicle->move_type);
		exit(1);
	}
  
	size = pEdgeNode->vehicle_movement_list.size;
	pMoveNode = &(pEdgeNode->vehicle_movement_list.head);
	for(i = 0; i < size && flag == FALSE; i++) //for-1
	//for(i = 0; i < size; i++) //for-1
	{
		pMoveNode = pMoveNode->next;
		if(vehicle->id == pMoveNode->vid)
			continue;

		distance = fabs(pMoveNode->offset - offset_in_directional_edge);

		/* choose a next carrier with smaller EDD according to forwording type */
		switch(param->vehicle_vanet_forwarding_type) //switch-1
		{
			case VANET_FORWARDING_BASED_ON_VEHICLE:
				if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
				{
					if((distance <= param->communication_range) && (pMoveNode->vnode->EDD < min_neighbor_EDD))
					{ //we also check vehicles' EDDs
						min_neighbor_EDD = pMoveNode->vnode->EDD;
						next_carrier_candidate_1 = pMoveNode->vnode; //update the pointer of neighbor vehicle node with smaller EDD; Note in VADD, a vehicle with farther offset is chosen as next carrier.
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
				{
					if((distance <= param->communication_range) && (pMoveNode->vnode->EDD_for_download < min_neighbor_EDD))
					{ //we also check vehicles' EDDs
						min_neighbor_EDD = pMoveNode->vnode->EDD_for_download;
						next_carrier_candidate_1 = pMoveNode->vnode; //update the pointer of neighbor vehicle node with smaller EDD; Note in VADD, a vehicle with farther offset is chosen as next carrier.
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
				{
					if(param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD || param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC)
					{
						if(distance <= param->communication_range)
						{ //we also check vehicles' EDDs
							/* update the vehicle's EDD and EDD_SD_for_V2V */
							VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, pMoveNode->vnode, G, G_size, source_intersection_id);

							if(pMoveNode->vnode->EDD_for_V2V < min_neighbor_EDD)
							{
								min_neighbor_EDD = pMoveNode->vnode->EDD_for_V2V;
								next_carrier_candidate_1 = pMoveNode->vnode; //update the pointer of neighbor vehicle node with smaller EDD; Note in VADD, a vehicle with farther offset is chosen as next carrier.
							}
						}
					}
					else
					{
						printf("%s:%d: param->vanet_forwarding_scheme(%d) is not supported!\n ",
								__FUNCTION__, __LINE__,
								param->vanet_forwarding_scheme);
						exit(1);
					}
				}
	
				break;

			case VANET_FORWARDING_BASED_ON_CONVOY:
				/** check whether vehicle_vanet_stationary_vehicle_flag is turned on */
				if(param->vehicle_vanet_stationary_vehicle_flag && vehicle->id == 1 && distance <= param->communication_range)
				{
					//*next_carrier = pMoveNode->vnode;
					//*next_carrier = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
					/*@ Note: we need to consider the communication delay between the sensor and the leader */
					next_carrier_candidate_1 = pMoveNode->vnode->ptr_convoy_queue_node->tail_vehicle;
					flag = TRUE; //[03/18/09] set flag to let the for-loop terminate
					break;
				}

				/** update the pointer of neighbor vehicle node:
					Note that we check whether next_carrier candidate's EDD is less than
					vehicle's EDD
				*/
				if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
				{
					if((distance <= param->communication_range) && (pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid) && (pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle->EDD < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD))
					//if(distance <= param->communication_range)
					//if((distance <= param->communication_range) && (pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle->EDD < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD) && (pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle->EDD < min_neighbor_EDD))
					{ 
						/* update min_neighbor's EDD with pMoveNode's convoy leader's EDD */
						// min_neighbor_EDD = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle->EDD;

						/* set next_carrier to pMoveNode->vnode */
						next_carrier_candidate_1 = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
						flag = TRUE; //[03/18/09] set flag to let the for-loop terminate

						//@ Note: we need to consider the communication delay between the packet source vehicle and the next_carrier
						//return TRUE;
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
				{
					if((distance <= param->communication_range) && (pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid))
					{
						next_carrier_candidate_1 = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
						flag = TRUE; //[03/18/09] set flag to let the for-loop terminate
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
				{
					if(param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD)
					{
						if(distance <= param->communication_range) 
						{
							/* update the vehicle's EDD and EDD_SD_for_V2V */
							VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle, G, G_size, source_intersection_id);

							if((pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid) && (pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle->EDD_for_V2V < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD_for_V2V))
							{
								next_carrier_candidate_1 = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
								flag = TRUE;
							}
						}
					}
					else if(param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC)
					{
						if((distance <= param->communication_range) && (pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid))
						{
							next_carrier_candidate_1 = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
							flag = TRUE;
						}
					}
					else
					{
						printf("%s:%d: param->vanet_forwarding_scheme(%d) is not supported!\n ",
								__FUNCTION__, __LINE__,
								param->vanet_forwarding_scheme);
						exit(1);
					}
				}
				break;

			default:
				printf("%s:%d: vanet forwarding type(%d) is not known!\n", 
						__FUNCTION__, __LINE__,
						param->vehicle_vanet_forwarding_type);
				exit(1);
		} //end of switch-1
	} //end of for-1
 
	/******************************************************************************/

	/**@ Search the next carrier on the opposite directional edge of the directional edge where vehicle is moving **/
	/* initialize the variables for the searching of next carrier */
#if 0 /* [ */
	min_neighbor_EDD = INF; //neighbor vehicle or neighbor convoy with a minimum EDD
#endif /* ] */

	flag = FALSE; //flag is set to FALSE to indicate that a next carrier is determined

	/** obtain the pointer to the directional edge of <head_node,tail_node> */
#if 1 /* [ */
	pEdgeNode = FastLookupDirectionalEdgeQueue(Gr_global, head_node, tail_node);
#else	
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, head_node, tail_node);
#endif /* ] */
	if(pEdgeNode == NULL)
	{
		printf("%s:%d: pEdgeNode for <%s,%s> is NULL\n", 
				__FUNCTION__, __LINE__,
				head_node, tail_node);
		exit(1);
	}

#if 0 /* [ */
	/* update the EDDs and EDD_SDs of vehicle and its convoy head for V2V data delivery */
	if((param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V) && (param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD || param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC))
	{
		VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, vehicle, G, G_size, source_intersection_id);
		VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, vehicle->ptr_convoy_queue_node->leader_vehicle, G, G_size, source_intersection_id);
	}
#endif /* ] */

	/* set min_neighbor_EDD according to param's data_forwarding_mode and vehicle_vanet_forwarding_type */
	min_neighbor_EDD = VADD_Get_Initial_Minimum_Neighbor_EDD(param, vehicle);
//#endif /* ] */

	/** compute the offset in directional edge:
		Note: imagine that vehicle is moving on the opposite directional edge */
	offset_in_undirectional_edge = vehicle->current_pos_in_Gr.offset;
	if(vehicle->move_type == MOVE_FORWARD)
	{
		offset_in_directional_edge = vehicle->edge_length - offset_in_undirectional_edge;
	}    
	else if(vehicle->move_type == MOVE_BACKWARD)
	{
		offset_in_directional_edge = offset_in_undirectional_edge;
	}
	else
	{
		printf("%s:%d: vehicle->move_type(%d) is invalid\n", 
				__FUNCTION__, __LINE__,
				vehicle->move_type);
		exit(1);
	}
  
	size = pEdgeNode->vehicle_movement_list.size;
	pMoveNode = &(pEdgeNode->vehicle_movement_list.head);
	for(i = 0; i < size && flag == FALSE; i++) //for-1
	{
		pMoveNode = pMoveNode->next;
		if(vehicle->id == pMoveNode->vid)
			continue;

		distance = fabs(pMoveNode->offset - offset_in_directional_edge);

		/* choose a next carrier with smaller EDD according to forwording type */
		switch(param->vehicle_vanet_forwarding_type) //switch-1
		{
			case VANET_FORWARDING_BASED_ON_VEHICLE:
				if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
				{ 
					if((distance <= param->communication_range) && (pMoveNode->vnode->EDD < min_neighbor_EDD))
					{ //we also check vehicles' EDDs
						min_neighbor_EDD = pMoveNode->vnode->EDD;
						next_carrier_candidate_2 = pMoveNode->vnode; //update the pointer of neighbor vehicle node with smaller EDD; Note in VADD, a vehicle with farther offset is chosen as next carrier.
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
				{
					if((distance <= param->communication_range) && (pMoveNode->vnode->EDD_for_download < min_neighbor_EDD))
					{ //we also check vehicles' EDDs
						min_neighbor_EDD = pMoveNode->vnode->EDD_for_download;
						next_carrier_candidate_2 = pMoveNode->vnode; //update the pointer of neighbor vehicle node with smaller EDD; Note in VADD, a vehicle with farther offset is chosen as next carrier.
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
				{
					if(param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD || param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC)
					{
						if(distance <= param->communication_range)
						{ //we also check vehicles' EDDs
							/* update the vehicle's EDD and EDD_SD_for_V2V */
							VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, pMoveNode->vnode, G, G_size, source_intersection_id);

							if(pMoveNode->vnode->EDD_for_V2V < min_neighbor_EDD)
							{
								min_neighbor_EDD = pMoveNode->vnode->EDD_for_V2V;
								next_carrier_candidate_2 = pMoveNode->vnode; //update the pointer of neighbor vehicle node with smaller EDD; Note in VADD, a vehicle with farther offset is chosen as next carrier.
							}
						}
					}
					else
					{
						printf("%s:%d: param->vanet_forwarding_scheme(%d) is not supported!\n ",
								__FUNCTION__, __LINE__,
								param->vanet_forwarding_scheme);
						exit(1);
					}
				}
	
				break;

			case VANET_FORWARDING_BASED_ON_CONVOY:
				/** check whether vehicle_vanet_stationary_vehicle_flag is turned on */
				if(param->vehicle_vanet_stationary_vehicle_flag && vehicle->id == 1 && distance <= param->communication_range)
				{
					/*@ Note: we need to consider the communication delay between the sender and the leader */
					next_carrier_candidate_2 = pMoveNode->vnode->ptr_convoy_queue_node->tail_vehicle;
					flag = TRUE; //[03/18/09] set flag to let the for-loop terminate
					break;
				}

				/** update the pointer of neighbor vehicle node:
					Note that we check whether next_carrier candidate's EDD is less than
					vehicle's EDD
				*/
				if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
				{
					if((distance <= param->communication_range) && (pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid) && (pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle->EDD < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD))
					{ 
						/* set next_carrier to pMoveNode->vnode */
						next_carrier_candidate_2 = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
						flag = TRUE; //[03/18/09] set flag to let the for-loop terminate
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
				{
					if((distance <= param->communication_range) && (pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid))
					{
						next_carrier_candidate_2 = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
						flag = TRUE; //[03/18/09] set flag to let the for-loop terminate
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
				{
					if(param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD)
					{
						if(distance <= param->communication_range) 
						{
							/* update the vehicle's EDD and EDD_SD_for_V2V */
							VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle, G, G_size, source_intersection_id);

							if((pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid) && (pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle->EDD_for_V2V < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD_for_V2V))
							{
								next_carrier_candidate_2 = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
								flag = TRUE;
							}
						}
					}
					else if(param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC)
					{
						if((distance <= param->communication_range) && (pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid))
						{
							next_carrier_candidate_2 = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
							flag = TRUE;
						}
					}
					else
					{
						printf("%s:%d: param->vanet_forwarding_scheme(%d) is not supported!\n ",
								__FUNCTION__, __LINE__,
								param->vanet_forwarding_scheme);
						exit(1);
					}
				}
	
				break;

			default:
				printf("%s:%d: vanet forwarding type(%d) is not known!\n", 
						__FUNCTION__, __LINE__,
						param->vehicle_vanet_forwarding_type);
				exit(1);
		} //end of switch-1
	} //end of for-1
   
	/******************************************************************************/

	/** select the best carrier between next_carrier_candidate_1 and next_carrier_candidate_2 */
	if((next_carrier_candidate_1 != NULL) && (next_carrier_candidate_2 != NULL))
	{
		if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
		{
			if(next_carrier_candidate_1->EDD <= next_carrier_candidate_2->EDD)
			{
				*next_carrier = next_carrier_candidate_1;
			}
			else
			{
				*next_carrier = next_carrier_candidate_2;
			}
		}
		else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
		{
			if(next_carrier_candidate_1->EDD_for_download <= next_carrier_candidate_2->EDD_for_download)
			{
				*next_carrier = next_carrier_candidate_1;
			}
			else
			{
				*next_carrier = next_carrier_candidate_2;
			}
		}
		else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
		{
			if(next_carrier_candidate_1->EDD_for_V2V <= next_carrier_candidate_2->EDD_for_V2V)
			{
				*next_carrier = next_carrier_candidate_1;
			}
			else
			{
				*next_carrier = next_carrier_candidate_2;
			}
		}

	}
	else if(next_carrier_candidate_1 != NULL)
	{
		*next_carrier = next_carrier_candidate_1;
	}
	else if(next_carrier_candidate_2 != NULL)
	{
		*next_carrier = next_carrier_candidate_2;
	}

	/** set the next_carrier to the vehicle's convoy leader when there exists no next carrier candidate in a different convoy with less EDD */
	if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY)
	{
		if((next_carrier_candidate_1 == NULL) && (next_carrier_candidate_2 == NULL) && (vehicle->id != vehicle->ptr_convoy_queue_node->leader_vehicle->id))
		{
			*next_carrier = vehicle->ptr_convoy_queue_node->leader_vehicle;      
		}
	}

	/** check whether next_carrier is equal to vehicle */
	if(*next_carrier != NULL)
	{
		if(vehicle == *next_carrier)
		{ //if next_carrier is equal to vehicle, we don't let vehicle send its packets to itself 
			printf("%s:%d: Error: vehicle(id=%d) is the same as next_carrier\n", 
					__FUNCTION__, __LINE__,
					vehicle->id);
			exit(1);
		}
		else
			result = TRUE;
	}

	return result;
}

boolean VADD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection(parameter_t *param, double current_time, struct_vehicle_t *vehicle, char *tail_node_for_next_forwarding_edge, char *head_node_for_next_forwarding_edge, directional_edge_type_t edge_type, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier)
{ //determine whether to forward its packets to next carrier moving on the road segment incident to an intersection corresponding to tail_node_for_next_edge and return the pointer to the next carrier through *next_carrier
	boolean result = FALSE;
	boolean flag = FALSE; //flag to indicate whether a next carrier is determined or not
	double distance = 0; //distance between vehicle and another vehicle moving on the same directional edge
	double max_neighbor_offset = -1; //neighbor's offset such the distance between vehicle and preceding neighbor vehicle is maximum
	double min_neighbor_EDD = INF; //neighbor vehicle or neighbor convoy with a minimum EDD
	char *tail_node = tail_node_for_next_forwarding_edge; //tail node of the next directional edge for forwarding
	char *head_node = head_node_for_next_forwarding_edge; //head node of the next directional edge for forwarding
	directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
	vehicle_movement_queue_node_t *pMoveNode = NULL; //pointer to the vehicle movement queue
	int i = 0; //index for for-loop
	int size = 0; //size of vehicle movement queue

	int source_intersection_id = atoi(tail_node); //the id of the tail intersection of the directed edge where the packet carrier is moving

	struct_graph_node *Gr_global = param->vanet_table.Gr; //pointer to the global road network graph having the global directional queue DEr to manage the clusters of vehicles 
	int Gr_global_size = param->vanet_table.Gr_size; //size of Gr_global

	/** check the validity of the global road network graph Gr_global and its size Gr_global_size */
	if(Gr_global == NULL)
	{
		printf("%s:%d: Gr_global is NULL!\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}
	else if(Gr_global_size <= 0)
	{
		printf("%s:%d: Gr_global_size(%d) is not positive!\n",
				__FUNCTION__, __LINE__,
				Gr_global_size);
		exit(1);
	}

	/** reset *next_carrier to NULL */
	*next_carrier = NULL;

	/** obtain the pointer to the direaction edge of <tail_node,head_node> */
#if 1 /* [ */
	pEdgeNode = FastLookupDirectionalEdgeQueue(Gr_global, tail_node, head_node);
#else
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
#endif /* ] */

	if(pEdgeNode == NULL)
	{
		if(edge_type == OUTGOING_EDGE) //outgoing edge for tail_node
			printf("%s:%d: pEdgeNode for <%s,%s> is NULL\n", 
					__FUNCTION__, __LINE__,
					tail_node, head_node);
		else
			printf("%s:%d: pEdgeNode for <%s,%s> is NULL\n", 
					__FUNCTION__, __LINE__,
					head_node, tail_node);

		exit(1);
	}

	/* update the EDDs and EDD_SDs of vehicle and its convoy head for V2V data delivery */
	if((param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V) && (param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD || param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC))
	{
		VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, vehicle, G, G_size, source_intersection_id);
		VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, vehicle->ptr_convoy_queue_node->leader_vehicle, G, G_size, source_intersection_id);
	}

	/* set min_neighbor_EDD according to param's data_forwarding_mode and vehicle_vanet_forwarding_type */
	min_neighbor_EDD = VADD_Get_Initial_Minimum_Neighbor_EDD(param, vehicle);

	size = pEdgeNode->vehicle_movement_list.size;
	pMoveNode = &(pEdgeNode->vehicle_movement_list.head);
	for(i = 0; i < size && flag == FALSE; i++) //[03/18/09] for-1
	{
		pMoveNode = pMoveNode->next;
		if(vehicle->id == pMoveNode->vid)
			continue;

		if(edge_type == OUTGOING_EDGE) //for the outgoing edge of the intersection (i.e., tail_node)
			distance = pMoveNode->offset;
		else //for the incoming edge of the intersection (i.e., head_node)
			distance = fabs(pEdgeNode->weight - pMoveNode->offset);
    
		/* select a next vehicle with the shortest EDD according to data_forwarding_mode, vanet_forwarding_scheme and vehicle_vanet_forwarding_type */
		switch(param->vehicle_vanet_forwarding_type) //switch-1
		{
			case VANET_FORWARDING_BASED_ON_VEHICLE:
				if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
				{
					if((distance <= param->communication_range) && (pMoveNode->vnode->EDD < min_neighbor_EDD))
					{ //we also check vehicles' EDDs
						min_neighbor_EDD = pMoveNode->vnode->EDD;
						*next_carrier = pMoveNode->vnode; //update the pointer of neighbor vehicle node with smaller EDD; Note in VADD, a vehicle with farther offset is chosen as next carrier.
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
				{ /* Note: This data_forwarding_mode should be used for VANET_FORWARDING_BASED_ON_VEHICLE */
					if((distance <= param->communication_range) && (pMoveNode->vnode->EDD_for_download < min_neighbor_EDD))
					{ //we also check vehicles' EDDs
						min_neighbor_EDD = pMoveNode->vnode->EDD_for_download;
						*next_carrier = pMoveNode->vnode; //update the pointer of neighbor vehicle node with smaller EDD; Note in VADD, a vehicle with farther offset is chosen as next carrier.
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
				{
					if(param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD || param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC)
					{
						if(distance <= param->communication_range)
						{ //we also check vehicles' EDDs
							/* update the vehicle's EDD and EDD_SD_for_V2V */
							VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, pMoveNode->vnode, G, G_size, source_intersection_id);

							if(pMoveNode->vnode->EDD_for_V2V < min_neighbor_EDD)
							{
								min_neighbor_EDD = pMoveNode->vnode->EDD_for_V2V;
								*next_carrier = pMoveNode->vnode; //update the pointer of neighbor vehicle node with smaller EDD; Note in VADD, a vehicle with farther offset is chosen as next carrier.
							}
						}
					}
					else
					{
						printf("%s:%d: param->vanet_forwarding_scheme(%d) is not supported!\n ",
								__FUNCTION__, __LINE__,
								param->vanet_forwarding_scheme);
						exit(1);
					}
				}
				break;

			case VANET_FORWARDING_BASED_ON_CONVOY:
				if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
				{
					if((distance <= param->communication_range) && (pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid) && (pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle->EDD < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD))
					{ 
						*next_carrier = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
						flag = TRUE; //[03/18/09] set flag to let the for-loop terminate
						//@ Note: we need to consider the communication range between the packet source vehicle and the next_carrier since the next_carrier is connected to the leader vehicle with the minimum EDD withing the connected component including the next_carrier
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
				{
					if((distance <= param->communication_range) && (pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid))
					{
						*next_carrier = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
						flag = TRUE; //[03/18/09] set flag to let the for-loop terminate
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
				{
					if(param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD)
					{
						if(distance <= param->communication_range) 
						{
							/* update the vehicle's EDD and EDD_SD_for_V2V */
							VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle, G, G_size, source_intersection_id);

							if((pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid) && (pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle->EDD_for_V2V < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD_for_V2V))
							{
								*next_carrier = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
								flag = TRUE;
							}
						}
					}
					else if(param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC)
					{
						if((distance <= param->communication_range) && (pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid))
						{
							*next_carrier = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
							flag = TRUE;
						}
					}
					else
					{
						printf("%s:%d: param->vanet_forwarding_scheme(%d) is not supported!\n ",
								__FUNCTION__, __LINE__,
								param->vanet_forwarding_scheme);
						exit(1);
					}
				}
				break;

			default:
				printf("%s:%d: vanet forwarding type (%d) is not known!\n", 
						__FUNCTION__, __LINE__,
						param->vehicle_vanet_forwarding_type);
				exit(1);
		} //end of switch-1
	} //end of for-1

	/** check whether a next carrier vehicle exists */
	if(*next_carrier != NULL)
		result = TRUE;

	return result;
}

boolean VADD_Is_There_Next_Carrier_At_Intersection(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, forwarding_table_queue_t *FTQ, struct_vehicle_t **next_carrier)
{ //determine whether to forward its packets to next carrier moving on the other road segment with the smallest EDD at intersection and return the pointer to the next carrier through *next_carrier
	boolean result = FALSE; //return value
	boolean flag = FALSE; //flag to indicate there exists a next carrier candidate

	struct_graph_node *pGraphNode = NULL; //pointer to a graph node
	char *tail_node = vehicle->path_ptr->vertex; //tail node of the directional edge where vehicle is moving
	char *head_node = vehicle->path_ptr->next->vertex; //head node of the directional edge where vehicle is moving
	char *tail_node_for_next_forwarding_edge = NULL; //tail node of the next directional edge for forwarding
	char *head_node_for_next_forwarding_edge = NULL; //head node of the next directional edge for forwarding
	directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
	int size = 0; //size of intersection EDD queue
	int i = 0; //index for for-loop
	double min_next_carrier_EDD = INF; //minimum value of next carrier's EDD
	double max_next_carrier_offset = -1; //maximum value of next carrier's offset
	struct_vehicle_t *next_carrier_candidate = NULL; //pointer to the next carrier candidate
	directional_edge_type_t edge_type = OUTGOING_EDGE; //directional edge type for tail_node
	struct_graph_node *intersection_gnode = NULL; //pointer to the graph node of the intersection where the vehicle is arriving
	struct_graph_node *neighboring_intersection_gnode = NULL; //pointer to the graph node of the neighboring intersection for the intersection where the vehicle is arriving

	/** reset *next_carrier to NULL */
	*next_carrier = NULL;

	/** check whether vehicle has packets to forward to a next carrier */
	if(vehicle->packet_queue->size == 0)
	{
		return FALSE;
	}

	/** check whether vehicle has vaild target point and sequence number under download mode */
	/*@[11/11/09] Note: I made a mistake that I omitted the following condition "if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)", so I prevented the forwarding at intersections */
	if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD || param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
	{
		/** check whether vehicle has vaild target point and sequence number under download mode */
		if(vehicle->target_point_id == 0 || vehicle->latest_packet_seq == 0)
		{
			printf("%s:%d: vehicle->target_point_id(%d) == 0 or vehicle->latest_packet_seq(%d) == 0\n",
					__FUNCTION__, __LINE__,
					vehicle->target_point_id, vehicle->latest_packet_seq);

			return FALSE;
		}
	}

	/** search for a best next carrier in the order of the smallest EDDs assigned to directional edges incident to the current intersect where the vehicle has reached */

	/* obtain the pointer to the direaction edge of <tail_node,head_node> */
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
	if(pEdgeNode == NULL)
	{
		printf("VADD_Is_There_Next_Carrier_At_Intersection(): pEdgeNode for <%s,%s> is NULL\n", tail_node, head_node);
		exit(1);
	}

	/** Next carrier selection rule: We select a vehicle with the shortest vehicle EDD regardless of branch EDDs: Note that in Per-Intersection model we select a farther vehicle on the best branch with the shortest branch EDD */  

	intersection_gnode = pEdgeNode->head_gnode->gnode;
	neighboring_intersection_gnode = intersection_gnode;
	size = (int)intersection_gnode->weight;
	for(i = 0; i < size; i++) //for-1
	{
		neighboring_intersection_gnode = neighboring_intersection_gnode->next;
    
		/** search for a next carrier candidate in the outgoing edge of <intersection,neighboring_intersection> */
		edge_type = OUTGOING_EDGE;
		tail_node_for_next_forwarding_edge = intersection_gnode->vertex;
		head_node_for_next_forwarding_edge = neighboring_intersection_gnode->vertex;
    
		flag = VADD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection(param, current_time, vehicle, tail_node_for_next_forwarding_edge, head_node_for_next_forwarding_edge, edge_type, G, G_size, &next_carrier_candidate);
		//determine whether to forward its packets to next carrier moving on the road segment incident to an intersection corresponding to tail_node and return the pointer to the next carrier through *next_carrier

		if(flag) //if-1
		{
			/* select a vehicle with the minimum next carrier EDD as a next carrier */
			switch(param->vehicle_vanet_forwarding_type) //switch-1
			{
				case VANET_FORWARDING_BASED_ON_VEHICLE:
					if(next_carrier_candidate->EDD < min_next_carrier_EDD)
					{
						*next_carrier = next_carrier_candidate;
						min_next_carrier_EDD = next_carrier_candidate->EDD;
						result = flag; //set result to TRUE since there exists next carrier candidate
					}
					break;

				case VANET_FORWARDING_BASED_ON_CONVOY:
					if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
					{
						if((next_carrier_candidate->EDD < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD) && (next_carrier_candidate->EDD < min_next_carrier_EDD))
						{
							*next_carrier = next_carrier_candidate;
							min_next_carrier_EDD = next_carrier_candidate->EDD;
							result = flag; //set result to TRUE since there exists next carrier candidate
						}
					}
					else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
					{
						/** let next_carrier_candidate compute its new EDD_for_download with AP's target point */   
						VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download_With_Given_TargetPoint_And_SequenceNumber(current_time, param, next_carrier_candidate, FTQ, vehicle->target_point_id, vehicle->latest_packet_seq);

						if((next_carrier_candidate->EDD_for_download < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD_for_download) && (next_carrier_candidate->EDD_for_download < min_next_carrier_EDD))
						{
							*next_carrier = next_carrier_candidate;
							min_next_carrier_EDD = next_carrier_candidate->EDD_for_download;
							result = flag; //set result to TRUE since there exists next carrier candidate
						}
					}
	
					break;

				default:
					printf("VADD_Is_There_Next_Carrier_At_Intersection(): param->vehicle_vanet_forwarding_type(%d) is not supported!\n", param->vehicle_vanet_forwarding_type);
					exit(1);
			} //end of switch-1
		} //end of if-1

		/** search a next carrier candidate in the outgoing edge of <intersection, neighboring_intersection> */
		edge_type = INCOMING_EDGE;
		tail_node_for_next_forwarding_edge = neighboring_intersection_gnode->vertex;
		head_node_for_next_forwarding_edge = intersection_gnode->vertex;
    
		flag = VADD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection(param, current_time, vehicle, tail_node_for_next_forwarding_edge, head_node_for_next_forwarding_edge, edge_type, G, G_size, &next_carrier_candidate);
		//determine whether to forward its packets to next carrier moving on the road segment incident to an intersection corresponding to tail_node and return the pointer to the next carrier through *next_carrier

		if(flag) //if-2
		{    
			/* select a vehicle with the minimum next carrier EDD as a next carrier */
			switch(param->vehicle_vanet_forwarding_type) //switch-2
			{
				case VANET_FORWARDING_BASED_ON_VEHICLE:
					if(next_carrier_candidate->EDD < min_next_carrier_EDD)
					{
						*next_carrier = next_carrier_candidate;
						min_next_carrier_EDD = next_carrier_candidate->EDD;
						result = flag; //set result to TRUE since there exists next carrier candidate
					}
					break;

				case VANET_FORWARDING_BASED_ON_CONVOY:
					if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
					{
						if((next_carrier_candidate->EDD < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD) && (next_carrier_candidate->EDD < min_next_carrier_EDD))
						{
							*next_carrier = next_carrier_candidate;
							min_next_carrier_EDD = next_carrier_candidate->EDD;
							result = flag; //set result to TRUE since there exists next carrier candidate
						}
					}
					else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
					{
						/** let next_carrier_candidate compute its new EDD_for_download with AP's target point */   
						VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download_With_Given_TargetPoint_And_SequenceNumber(current_time, param, next_carrier_candidate, FTQ, vehicle->target_point_id, vehicle->latest_packet_seq);

						if((next_carrier_candidate->EDD_for_download < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD_for_download) && (next_carrier_candidate->EDD_for_download < min_next_carrier_EDD))
						{
							*next_carrier = next_carrier_candidate;
							min_next_carrier_EDD = next_carrier_candidate->EDD_for_download;
							result = flag; //set result to TRUE since there exists next carrier candidate
						}
					}

					break;

				default:
					printf("VADD_Is_There_Next_Carrier_At_Intersection(): param->vehicle_vanet_forwarding_type(%d) is not supported!\n", param->vehicle_vanet_forwarding_type);
					exit(1);
			} //end of switch-2
		} //end of if-2
	} //end of for-1

	/** [03/18/09] set the next_carrier to the vehicle's convoy leader when there exists no next carrier candidate in a different convoy with less EDD */
	if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY)
	{
		if((result == FALSE) && (vehicle->id != vehicle->ptr_convoy_queue_node->leader_vehicle->id))
		{
			*next_carrier = vehicle->ptr_convoy_queue_node->leader_vehicle;
			result = TRUE;
		}
	}

	return result;
}

boolean VADD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_For_AP(parameter_t *param, double current_time, struct_access_point_t *AP, char *tail_node_for_next_forwarding_edge, char *head_node_for_next_forwarding_edge, directional_edge_type_t edge_type, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier)
{ //Under download mode or V2V mode, determine whether to forward AP's packets to next carrier moving on the road segment incident to an intersection corresponding to tail_node_for_next_edge and return the pointer to the next carrier through *next_carrier
	boolean result = FALSE;
	boolean flag = FALSE; //flag to indicate whether a next carrier is determined or not
	double distance = 0; //distance between vehicle and another vehicle moving on the same directional edge
	double max_neighbor_offset = -1; //neighbor's offset such the distance between vehicle and preceding neighbor vehicle is maximum
	double min_neighbor_EDD = INF; //neighbor vehicle or neighbor convoy with a minimum EDD
	char *tail_node = tail_node_for_next_forwarding_edge; //tail node of the next directional edge for forwarding
	char *head_node = head_node_for_next_forwarding_edge; //head node of the next directional edge for forwarding
	directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
	vehicle_movement_queue_node_t *pMoveNode = NULL; //pointer to the vehicle movement queue
	int i = 0; //index for for-loop
	int size = 0; //size of vehicle movement queue
	int source_intersection_id = atoi(AP->vertex); //the id of the packet source intersection having the AP
	struct_graph_node *Gr_global = param->vanet_table.Gr; //pointer to the global road network graph having the global directional queue DEr to manage the clusters of vehicles 
	int Gr_global_size = param->vanet_table.Gr_size; //size of Gr_global
	/** NOTE: 
	 * 1. G and G_size are used to get the EDD and EDD_SD from a source intersection
	 *	to the target intersection where the road graph network is generated for the 
	 *	data delivery to the target intersection by VADD or TBD.
	 *
	 * 2. Gr_global and Gr_global_size are used to get the vehicle list for the 
	 *	directed edge uner check to select the next-carrier vehicle. 
	 * */

	/** check the validity of the global road network graph Gr_global and its size Gr_global_size */
	if(Gr_global == NULL)
	{
		printf("%s:%d: Gr_global is NULL!\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}
	else if(Gr_global_size <= 0)
	{
		printf("%s:%d: Gr_global_size(%d) is not positive!\n",
				__FUNCTION__, __LINE__,
				Gr_global_size);
		exit(1);
	}

	/** reset *next_carrier to NULL */
	*next_carrier = NULL;

	/** obtain the pointer to the direaction edge of <tail_node,head_node> */
#if 1 /* [ */
	pEdgeNode = FastLookupDirectionalEdgeQueue(Gr_global, tail_node, head_node);
#else
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
#endif /* ] */

	if(pEdgeNode == NULL)
	{
		if(edge_type == OUTGOING_EDGE) //outgoing edge for tail_node
			printf("%s:%d: pEdgeNode for <%s,%s> is NULL\n", 
					__FUNCTION__, __LINE__,
					tail_node, head_node);
		else
			printf("%s:%d: pEdgeNode for <%s,%s> is NULL\n", 
					__FUNCTION__, __LINE__,
					head_node, tail_node);

		exit(1);
	}

	size = pEdgeNode->vehicle_movement_list.size;
	pMoveNode = &(pEdgeNode->vehicle_movement_list.head);
	for(i = 0; i < size && flag == FALSE; i++) //[03/18/09] for-1
	{
		pMoveNode = pMoveNode->next;

		if(edge_type == OUTGOING_EDGE) //for the outgoing edge of the intersection (i.e., tail_node)
			distance = pMoveNode->offset;
		else //for the incoming edge of the intersection (i.e., head_node)
			distance = fabs(pEdgeNode->weight - pMoveNode->offset);
    
		/** Next carrier selection rule: We select a vehicle with the shortest vehicle EDD */  
		/* select a next vehicle according to Forwarding type */
		switch(param->vehicle_vanet_forwarding_type) //switch-1
		{
			case VANET_FORWARDING_BASED_ON_VEHICLE:
				if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
				{
					if((distance <= param->communication_range) && (pMoveNode->vnode->EDD < min_neighbor_EDD))
					{ //we also check vehicles' EDDs
						min_neighbor_EDD = pMoveNode->vnode->EDD;
						*next_carrier = pMoveNode->vnode; //update the pointer of neighbor vehicle node with smaller EDD; Note in VADD, a vehicle with farther offset is chosen as next carrier.
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
				{
					if(distance <= param->communication_range)
					{
						/* update the vehicle's EDD and EDD_SD_for_V2V */
						VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, pMoveNode->vnode, G, G_size, source_intersection_id);

						/* update min_neighbor_EDD */
						if(pMoveNode->vnode->EDD_for_V2V < min_neighbor_EDD)
						{
							min_neighbor_EDD = pMoveNode->vnode->EDD_for_V2V;
							*next_carrier = pMoveNode->vnode; //update the pointer of neighbor vehicle node with smaller EDD; Note in VADD, a vehicle with farther offset is chosen as next carrier.
						}
					}
				}
				else
				{
					printf("%s:%d: param->data_forwarding_mode(%d) is not supported here!\n",
							__FUNCTION__, __LINE__,
							param->data_forwarding_mode);
					exit(1);
				}
				break;

			case VANET_FORWARDING_BASED_ON_CONVOY:
				if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
				{
					if(distance <= param->communication_range)
					{ 
						/* set next_carrier to pMoveNode->vnode's leader vehicle */
						*next_carrier = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
						flag = TRUE; //[03/18/09] set flag to let the for-loop terminate
						//@ Note: we need to consider the communication range between the packet source vehicle and the next_carrier since the next_carrier is connected to the leader vehicle with the minimum EDD withing the connected component including the next_carrier
					}
				}
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
				{
					if(distance <= param->communication_range)
					{ 
						/* set next_carrier to pMoveNode->vnode's leader vehicle */
						*next_carrier = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
						flag = TRUE; //[03/18/09] set flag to let the for-loop terminate

						/* update the leader_vehicle's EDD_for_V2V and EDD_SD_for_V2V */
						VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(current_time, param, *next_carrier, G, G_size, source_intersection_id);
					}
				}
				else
				{
					printf("%s:%d: param->data_forwarding_mode(%d) is not supported here!\n",
							__FUNCTION__, __LINE__,
							param->data_forwarding_mode);
					exit(1);
				}
				break;

			default:
				printf("%s:%d: vanet forwarding type (%d) is not known!\n", 
						__FUNCTION__, __LINE__,
						param->vehicle_vanet_forwarding_type);
				exit(1);
		} //end of switch-1
	} //end of for-1

	/** check whether a next carrier vehicle exists */
	if(*next_carrier != NULL)
		result = TRUE;

	return result;
}

boolean VADD_Is_There_Next_Carrier_At_Intersection_For_AP(parameter_t *param, double current_time, struct_access_point_t *AP, struct_graph_node *G, int G_size, forwarding_table_queue_t *FTQ, struct_vehicle_t **next_carrier)
{ //Under download mode or V2V mode, determine whether to forward AP's packets to next carrier moving on the other road segment with the smallest EDD at intersection and return the pointer to the next carrier through *next_carrier
	boolean result = FALSE; //return value
	boolean flag = FALSE; //flag to indicate there exists a next carrier candidate
	struct_graph_node *pGraphNode = NULL; //pointer to a graph node
	char *tail_node_for_next_forwarding_edge = NULL; //tail node of the next directional edge for forwarding
	char *head_node_for_next_forwarding_edge = NULL; //head node of the next directional edge for forwarding
	int size = 0; //size of intersection EDD queue
	int i = 0; //index for for-loop
	double min_next_carrier_EDD = INF; //minimum value of next carrier's EDD
	double max_next_carrier_offset = -1; //maximum value of next carrier's offset
	struct_vehicle_t *next_carrier_candidate = NULL; //pointer to the next carrier candidate
	directional_edge_type_t edge_type = OUTGOING_EDGE; //directional edge type for tail_node
	struct_graph_node *intersection_gnode = NULL; //pointer to the graph node of the intersection where the vehicle is arriving
	struct_graph_node *neighboring_intersection_gnode = NULL; //pointer to the graph node of the neighboring intersection for the intersection where the vehicle is arriving

	/** reset *next_carrier to NULL */
	*next_carrier = NULL;

	/** check whether AP has packets to forward to a next carrier; if there is no packet, return FALSE without further checking for a next carrier */
	if(AP->packet_queue->size == 0)
	{
		return FALSE;
	}

	/** search for a best next carrier in the order of the smallest EDDs assigned to directional edges incident to the current intersect where the vehicle has reached. */
	intersection_gnode = AP->gnode; //let AP's gnode become intersection gnode
	neighboring_intersection_gnode = intersection_gnode;
	size = (int)intersection_gnode->weight;
	for(i = 0; i < size; i++) //for-1
	{
		neighboring_intersection_gnode = neighboring_intersection_gnode->next;
    
		/** search a next carrier candidate in the outgoing edge of <intersection, neighboring_intersection> */
		edge_type = OUTGOING_EDGE;
		tail_node_for_next_forwarding_edge = intersection_gnode->vertex;
		head_node_for_next_forwarding_edge = neighboring_intersection_gnode->vertex;
    
		/* check whether the tail node is one of APs; ap_flag is used to determine next carrier */
		flag = VADD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_For_AP(param, current_time, AP, tail_node_for_next_forwarding_edge, head_node_for_next_forwarding_edge, edge_type, G, G_size, &next_carrier_candidate);
		//determine whether to forward its packets to next carrier moving on the road segment incident to an intersection corresponding to tail_node for access point AP and return the pointer to the next carrier through *next_carrier

		if(flag) //if-1
		{ 
			if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD) //if-1.1
			{
				/** let next_carrier_candidate compute its new EDD_for_download with AP's target point */   
				VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download_With_Given_TargetPoint_And_SequenceNumber(current_time, param, next_carrier_candidate, FTQ, AP->target_point_id, AP->seq);

				/* select a vehicle with the minimum next carrier EDD as a next carrier */
				switch(param->vehicle_vanet_forwarding_type) //switch-1
				{
					case VANET_FORWARDING_BASED_ON_VEHICLE:
						if(next_carrier_candidate->EDD_for_download < min_next_carrier_EDD)
						{
							*next_carrier = next_carrier_candidate;
							min_next_carrier_EDD = next_carrier_candidate->EDD_for_download;
							result = flag; //set result to TRUE since there exists next carrier candidate
						}
						break;

					case VANET_FORWARDING_BASED_ON_CONVOY:
						if(next_carrier_candidate->EDD_for_download < min_next_carrier_EDD)
						{
							*next_carrier = next_carrier_candidate;
							min_next_carrier_EDD = next_carrier_candidate->EDD_for_download;
							result = flag; //set result to TRUE since there exists next carrier candidate
						}	
						break;

					default:
						printf("%s:%d: param->vehicle_vanet_forwarding_type(%d) is not supported!\n", 
								__FUNCTION__, __LINE__,
								param->vehicle_vanet_forwarding_type);
						exit(1);
				} //end of switch-1
			} //end of if-1.1
			else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V) //else if-1.2
			{
				if(next_carrier_candidate->EDD_for_V2V < min_next_carrier_EDD)
				{
					*next_carrier = next_carrier_candidate;
					min_next_carrier_EDD = next_carrier_candidate->EDD_for_V2V;
					result = flag; //set result to TRUE since there exists next carrier candidate
				}
			} //end of else if-1.2
		} //end of if-1
	} //end of for-1

	return result;
}

int VADD_Forward_Packet_To_Next_Carrier(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_vehicle_t *next_carrier, packet_delivery_statistics_t *packet_delivery_stat, int *discard_count)
{ //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
	packet_queue_node_t *pPacketNode = NULL; //pointer to a packet node
	int size = 0; //size of packet queue
	int i = 0; //index for for-loop
	double lifetime = 0; //packet lifetime
	double EDD = 0; //Expected Delivery Delay
	double EDD_SD = 0; // Delivery Delay Standard Deviation

	double distance = 0; //distance between vehicle and next_carrier
	double R = param->communication_range; //communication range
	int hop_number = 0 ; //hop number from vehicle to next_carrier in terms of communication range
	int forward_count = 0; //count for forwarded packets
	adjacency_list_queue_node_t *gnode = NULL; //pointer to a graph node in the encounter graph in the packet

	*discard_count = 0; //count for discarded packets

	/** check whether vehicle's id is equal to next_carrier's id or not */
	if(vehicle->id == next_carrier->id)
	{
		printf("%s:%d: vehicle(%d) is the same as next_carrier(%d)\n",
				__FUNCTION__, __LINE__,
				vehicle->id, next_carrier->id);

		return 0;
	}

	/** forward vehicle's packets to next_carrier */
	size = vehicle->packet_queue->size;
	if(size == 0)
		return 0;

	for(i = 0; i < size; i++) //for-1
	{
		pPacketNode = (packet_queue_node_t*) Dequeue((queue_t*)vehicle->packet_queue); //dequeue a packet placed at vehicle's packet queue front

		/* compute the packet's lifetime to check whether the packet expires */
		lifetime = current_time - pPacketNode->generation_time; //compute the packet's lifetime

		/** check whether packet's lifetime is greater than the predefined packet TTL */
		if(lifetime > pPacketNode->ttl) //if-1.1
		{
			VADD_Discard_Expired_Packet(param, current_time, pPacketNode, VANET_NODE_VEHICLE, (void*)next_carrier, packet_delivery_stat,pPacketNode->ttl); //discard the expired packet by destroying the memory of the packet while the correponding global packet queue node is dequeued and destroyed 
			(*discard_count)++;
			
			continue;
		} //end of if-1.1

		/* set up packet's fields */
		pPacketNode->last_receive_time = current_time; //last packet receive time
		pPacketNode->carry_src_id = vehicle->id; //current packet carrier's id
		pPacketNode->carry_dst_id = next_carrier->id; //next packet carrier's id

		/* set up pointers to the carrier vehicle in both the packet queue node and the global packet queue node corresponding to pPacketNode */
		pPacketNode->carrier_vnode = next_carrier; //pointer to the next packet carrier vehicle
		pPacketNode->global_packet->carrier_vnode = next_carrier; //pointer to the next packet carrier vehicle

		/* update packet_transmission_count */
		distance = euclidean_distance2(&(vehicle->current_pos), &(next_carrier->current_pos));
		if(R <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
		{
			printf("VADD_Forward_Packet_To_Next_Carrier(): Error: communication_range(%.2f) is zero!\n", (float)R);
			exit(1);
		}

		hop_number = (int)ceil(distance/R);
		pPacketNode->packet_transmission_count += hop_number;

		/** update target point information */
		if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD) //if-1
		{
			if((next_carrier->latest_packet_ptr == NULL) || (next_carrier->latest_packet_seq <= pPacketNode->seq)) 
			/* (i) "vehicle->latest_packet_ptr == NULL" means that vehicle has no packet, so updates its latest packet information
			   (ii) "== case" is that next_carrier has previously carried the packet and passed it to another vehicle. Now next_carrier receives the packet again
*/
			{
				next_carrier->target_point_id = pPacketNode->target_point_id; //update a target point with the new one
				next_carrier->latest_packet_seq = pPacketNode->seq;
				next_carrier->latest_packet_receive_time = current_time;
				next_carrier->latest_packet_ptr = pPacketNode;

				/* update the packet's EDD and EDD_SD */
				VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Carrier(param, next_carrier->target_point_id, next_carrier, param->vanet_table.FTQ, &EDD, &EDD_SD); //get the EDD and EDD_SD for a target point towards a destination vehicle, based on VADD Model (i.e., Per-intersection Model) from the position of carrier_vehicle, that is, from carrier's edge offset on the road network graph.
          
				next_carrier->EDD_for_download = EDD;
				next_carrier->EDD_SD_for_download = EDD_SD;
			}        
		} //end of if-1

		Enqueue_With_QueueNodePointer((queue_t*)next_carrier->packet_queue, (queue_node_t*)pPacketNode); //enqueue the packet into next_carrier's packet queue rear

		forward_count++; //increase packet count

		if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
		{
			/* update the latest packet information */
			if((next_carrier->latest_packet_ptr == NULL) || (next_carrier->latest_packet_seq <= pPacketNode->seq)) 
			/* (i) "vehicle->latest_packet_ptr == NULL" means that vehicle has no packet, so updates its latest packet information
			   (ii) "== case" is that next_carrier has previously carried the packet and passed it to another vehicle. Now next_carrier receives the packet again
*/
			{
				next_carrier->target_point_id = pPacketNode->target_point_id; //update a target point with the new one
				next_carrier->latest_packet_seq = pPacketNode->seq;
				next_carrier->latest_packet_receive_time = current_time;
				next_carrier->latest_packet_ptr = pPacketNode;
			}

			/* perform the forwarding task for TPD forwardig scheme */
			if(param->vanet_forwarding_scheme == VANET_FORWARDING_TPD && param->tpd_encounter_graph_source_routing_flag == 1)
			{
				/* search the graph node corresponding to next_carrier in the 
				 * encounter graph in the packet in the source routing mode in 
				 * TPD forwarding scheme */
				if(gnode == NULL)
				{
					gnode = TPD_Find_GraphNode_In_EncounterGraph(&(pPacketNode->predicted_encounter_graph->G), next_carrier);

					if(gnode == NULL)
					{
						printf("%s:%d gnode for next_carrier(%d) is NULL: G.bitmap[next_carrier->id-1]=%d, G.bitmap_gnodes[next_carrier->id-1]=%p\n",
								__FUNCTION__, __LINE__,
								next_carrier->id,
								pPacketNode->predicted_encounter_graph->G.bitmap[next_carrier->id-1],
								pPacketNode->predicted_encounter_graph->G.bitmap_gnodes[next_carrier->id-1]);
						exit(1);
					}
				}

				/* update the current carrier vehicle in the encounter graph */	
				pPacketNode->predicted_encounter_graph->G.carrier_vehicle_gnode = gnode;
			}
		}

#ifdef  __LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__
		/** enqueue the packet carrier trace entry into packet's carrier trace queue */
		Enqueue_CarrierTraceEntry(param, current_time, pPacketNode, VANET_NODE_VEHICLE, (void*)next_carrier);
#endif
   
	} //end of for-1

	/** reset the latest packet information if vehicle has no packet to carry with it */
	if(vehicle->packet_queue->size == 0)
	{
		vehicle->latest_packet_seq = 0; //reset the latest packet seq number
		vehicle->latest_packet_receive_time = 0;
		vehicle->latest_packet_ptr = NULL;
		vehicle->target_point_id = 0; //reset the target point id
	}

	/* change the types of vehicle and nect_carrier */
	if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
	{
		if(forward_count > 0)
		{
			if(vehicle->type == VEHICLE_CURRENT_PACKET_CARRIER)
			{
				vehicle->type = VEHICLE_CARRIER;
			}

			if(next_carrier->type == VEHICLE_CARRIER)
			{
				next_carrier->type = VEHICLE_CURRENT_PACKET_CARRIER;
				//next_carrier becomes a new current packet carrier
			}
		}
		else
		{
			if(vehicle->type == VEHICLE_CURRENT_PACKET_CARRIER)
			{
				vehicle->type = VEHICLE_CARRIER;
			}
		}
	}

	return forward_count;
}

void VADD_Forward_Packet_To_The_Following_Vehicle_In_Convoy(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, packet_delivery_statistics_t *packet_delivery_stat)
{ /* vehicle forwards its packet(s) to the following vehicle in the same convoy if this vehicle is not the minimum EDD vehicle and has the greater EDD in the next road segment on its path than the convoy leader's EDD.
     @NOTE: we need to consider the communication delay between vehicle and the following vehicle later.
  */
  struct_vehicle_t *following_vehicle = NULL; //next head vehicle in the convoy
  packet_queue_node_t *pPacketNode = NULL; //pointer to a packet node
  int size = 0; //size of packet queue
  int i = 0; //index for for-loop
  double lifetime = 0; //packet lifetime
  double new_vehicle_EDD = 0; //vehicle EDD on the vehicle's next road segment on the vehicle's path

  /**@for debugging */
  //if(current_time >= 3600 && vehicle->id == 3)
  //  printf("VADD_Forward_Packet_To_The_Following_Vehicle_In_Convoy(): vehicle (id=%d) is tracked\n", vehicle->id);
  /******************/

  /** check whether the intersection forwarding type is for deferred type or not */
  if(param->vehicle_vanet_intersection_forwarding_type != VANET_INTERSECTION_FORWARDING_DEFERRED_FORWARDING)
    return;

  /** check whether the vehicle is registered with a convoy or not */
  if(vehicle->flag_convoy_registration == FALSE)
    return;

  /** check whether the vehicle has packets or not */
  if(vehicle->packet_queue->size == 0)
    return; //there is no packet to pass

  /** check whether there exists the following vehicle in this convoy or not */
  //if(vehicle->ptr_convoy_queue_node->tail_vehicle == vehicle) //there is no other vehicle except for this vehicle in the convoy
  //  return;

  if(vehicle->ptr_convoy_queue_node->leader_vehicle == vehicle) //if this vehicle is leader, it carries packets with itself without passing packets to the following vehicle
    return;
  else if(vehicle->ptr_convoy_queue_node->vehicle_list.size == 1)
    return; //Note that in this case the vehicle must be convoy leader; but, in version 1.3.3, the vehicle is not leader
  else if(vehicle->ptr_convoy_queue_node->vehicle_list.size >= 2)
  { //@ Note that the vehicles in the vehicle list is sorted in the ascending order of offsets
    //following_vehicle = vehicle->ptr_convoy_queue_node->vehicle_list.head.next->next->ptr_vehicle;

    if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
    {
      new_vehicle_EDD = VADD_Update_Vehicle_EDD(current_time, param, vehicle, G, G_size, ap_table);
      //new_vehicle_EDD = VADD_Compute_TBD_Based_EDD(param, vehicle, G, G_size, ap_table);

      if(new_vehicle_EDD <= vehicle->ptr_convoy_queue_node->leader_vehicle->EDD)
        return; //this vehicle carries the convoy's packets; NOTE that we need to check whether this can happen during the simulation; however, since VADD_Compute_TBD_Based_EDD() is decreasing function before the vehicle goes out the communication range from the intersection, it seems that the condition never holds.
    }
    else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
    {
      printf("VADD_Forward_Packet_To_The_Following_Vehicle_In_Convoy(): The download mode is not implemented yet!\n");
      exit(1);
      
      new_vehicle_EDD = VADD_Update_Vehicle_EDD(current_time, param, vehicle, G, G_size, ap_table);

      if(new_vehicle_EDD <= vehicle->ptr_convoy_queue_node->leader_vehicle->EDD_for_download)
        return; //this vehicle carries the convoy's packets; NOTE that we need to check whether this can happen during the simulation; however, since VADD_Compute_TBD_Based_EDD() is decreasing function before the vehicle goes out the communication range from the intersection, it seems that the condition never holds.
    }

    following_vehicle = Find_Vehicle_Following_Convoy_Head(vehicle->ptr_convoy_queue_node);
    if(following_vehicle == NULL)
      return; //there is no vehicle following the convoy head
  }
  else
  {
    printf("VADD_Forward_Packet_To_The_Following_Vehicle_In_Convoy(): the convoy's vehicle_list (size=%d) has some problem where vehicle id = %d\n", vehicle->ptr_convoy_queue_node->vehicle_list.size, vehicle->id);
    exit(1);
  }

  /** check whether following_vehicle's id is equal to vehicle's id */
  if(following_vehicle->id == vehicle->id)
  {
    printf("VADD_Forward_Packet_To_The_Following_Vehicle_In_Convoy(): following_vehicle is the same as vehicle(vid=%d)\n", vehicle->id);
    exit(1);
  }

  /** forward vehicle's packets to next_carrier */
  size = vehicle->packet_queue->size;
  for(i = 0; i < size; i++)
  {
    pPacketNode = (packet_queue_node_t*) Dequeue((queue_t*)vehicle->packet_queue); //dequeue a packet placed at vehicle's packet queue front

    /* compute the packet's lifetime to check whether the packet expires */
    lifetime = current_time - pPacketNode->generation_time; //compute the packet's lifetime

    /** check whether packet's lifetime is greater than the predefined packet TTL */
    if(lifetime > pPacketNode->ttl) //if-1.1
    //if(lifetime > param->communication_packet_ttl) //if-1.1
    {
      VADD_Discard_Expired_Packet(param, current_time, pPacketNode, VANET_NODE_VEHICLE, (void*)following_vehicle, packet_delivery_stat,pPacketNode->ttl); //discard the expired packet by destroying the memory of the packet while the correponding global packet queue node is dequeued and destroyed 
     

      continue;
    } //end of if-1.1

    /* set up packet's fields */
    pPacketNode->last_receive_time = current_time; //last packet receive time
    pPacketNode->carry_src_id = vehicle->id; //current packet carrier's id
    pPacketNode->carry_dst_id = following_vehicle->id; //next packet carrier's id
	pPacketNode->packet_transmission_count++;

    /* set up pointers to the carrier vehicle in both the packet queue node and the global packet queue node corresponding to pPacketNode */
    pPacketNode->carrier_vnode = following_vehicle; //pointer to the next packet carrier vehicle
    pPacketNode->global_packet->carrier_vnode = following_vehicle; //pointer to the next packet carrier vehicle

    Enqueue_With_QueueNodePointer((queue_t*)following_vehicle->packet_queue, (queue_node_t*)pPacketNode); //enqueue the packet into the following vehicle's packet queue rear

#ifdef  __LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__
      /** enqueue the packet carrier trace entry into packet's carrier trace queue */
    Enqueue_CarrierTraceEntry(param, current_time, pPacketNode, VANET_NODE_VEHICLE, (void*)following_vehicle);
#endif
  }
}

void VADD_Forward_Packet_To_AP(parameter_t *param, double current_time, struct_vehicle_t *vehicle, access_point_queue_node_t *AP, packet_delivery_statistics_t *packet_delivery_stat)
{ //vehicle forwards its packet(s) to the access point pointed by ap_graph_node and the log for the packet(s) is written into the packet logging file.
  int i = 0; //index for for-loop
  packet_queue_t *Q = vehicle->packet_queue; //pointer to packet queue
  packet_queue_node_t *pPacketNode = NULL; //pointer to packet queue node  
  int size = vehicle->packet_queue->size; //size of packet queue
  double lifetime = 0; //packet lifetime

  for(i = 0; i < size; i++)
  {
    pPacketNode = (packet_queue_node_t*) Dequeue((queue_t*)Q);

    /* compute the packet's lifetime to check whether the packet expires */
    lifetime = current_time - pPacketNode->generation_time; //compute the packet's lifetime

    /** check whether packet's lifetime is greater than the predefined packet TTL */
    if(lifetime > pPacketNode->ttl) //if-1.1
    //if(lifetime > param->communication_packet_ttl) //if-1.1
    {
      VADD_Discard_Expired_Packet(param, current_time, pPacketNode, VANET_NODE_AP, (void*)AP, packet_delivery_stat,pPacketNode->ttl); //discard the expired packet by destroying the memory of the packet while the correponding global packet queue node is dequeued and destroyed 
      
      continue;
    } //end of if-1.1

    /* set up packet's fields */
    pPacketNode->carry_src_id = vehicle->id; //update the packet's carry_src_id with vehicle's id
    pPacketNode->carry_dst_id = AP->id; //update the packet's carry_dst_id with AP's id
	pPacketNode->packet_transmission_count++;

    /* set up pointers to the carrier vehicle in both the packet queue node and the global packet queue node corresponding to pPacketNode */
    pPacketNode->carrier_vnode = vehicle; //pointer to the final vehicle
    pPacketNode->global_packet->carrier_vnode = vehicle; //pointer to the final vehicle

    pPacketNode->actual_dst_id = AP->id; //AP's ID
    pPacketNode->last_receive_time = current_time; //last packet receive time
    pPacketNode->destination_arrival_time = current_time; //packet's AP arrival time

//#ifdef __LOG_LEVEL_VANET_PACKET_AP_ARRIVAL__
    log_vanet(VANET_LOG_PACKET_AP_ARRIVAL, current_time, pPacketNode, packet_delivery_stat);
    //log the data forwarding to Internet access point on VANET into VANET logging file
//#endif

#ifdef  __LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__
    /** enqueue the packet carrier trace entry into packet's carrier trace queue */
    Enqueue_CarrierTraceEntry(param, current_time, pPacketNode, VANET_NODE_AP, (void*)AP);

    /** log packet carrier trace for this dropped packet into carrier_trace_file */
    log_vanet_packet_carrier_trace(VANET_LOG_PACKET_AP_ARRIVAL, current_time, pPacketNode);
#endif

    /* dequeue global packet queue node, destroy global packet queue node and then free the memory of global packet queue node */
    DestroyGlobalPacketQueueNode(pPacketNode->global_packet);    

    /* destroy packet queue node along with vehicle trajectory queue and carrier trace queue, and then free the memory of packet queue node */
    DestroyPacketQueueNode(pPacketNode);
  }
}

void VADD_Forward_Packet_To_Destination_Vehicle(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, struct_vehicle_t *destination_vehicle, packet_delivery_statistics_t *packet_delivery_stat)
{ //vehicle forwards its packet(s) to the destination vehicle pointed by destination_vehicle and the log for the packet(s) is written into the packet logging file.
  int i = 0; //index for for-loop
  packet_queue_t *Q = carrier_vehicle->packet_queue; //pointer to packet queue
  packet_queue_node_t *pPacketNode = NULL; //pointer to packet queue node  
  int size = carrier_vehicle->packet_queue->size; //size of packet queue
  double lifetime = 0; //packet lifetime
  int delivered_packet_number = 0; //the number of packets delivered to destination_vehicle with valid TTL 

  if(size == 0)
    return;

  /*@for debugging */
  //if(current_time > 3600)
  //  printf("VADD_Forward_Packet_To_Destination_Vehicle(): time=%.2f, trace!\n", current_time);
  /*****************/

#if TPD_SOURCE_ROUTING_DESTINATION_VEHICLE__FORWARDING_TRACE_FLAG /* [*/
	printf("<D> %s:%d [%0.f] current_carrier(%d) in (%s->%s: %0.f) encounters destination_vehicle(%s->%s: %0.f).\n",
			__FUNCTION__, __LINE__,
			(float)current_time,
			carrier_vehicle->id,
			carrier_vehicle->current_pos_in_digraph.tail_node,
			carrier_vehicle->current_pos_in_digraph.head_node,
			carrier_vehicle->current_pos_in_digraph.offset,
			param->vanet_table.dst_vnode->current_pos_in_digraph.tail_node,			
   			param->vanet_table.dst_vnode->current_pos_in_digraph.head_node,
   			param->vanet_table.dst_vnode->current_pos_in_digraph.offset);
#endif /* ] */

  for(i = 0; i < Q->size; i++)
  //for(i = 0; i < size; i++)
  {
    pPacketNode = (packet_queue_node_t*) GetQueueNode((queue_t*)Q, i);
    //pPacketNode = (packet_queue_node_t*) Dequeue((queue_t*)Q);

    /* check whether this packet's destination is destination_vehicle or not */
    if(pPacketNode->dst_id != destination_vehicle->id)
    {
      /* enqueue the packet pointed by pPacketNode into queue Q again */
      //Enqueue((queue_t*)Q, (queue_node_t*)pPacketNode);

      /* free the memory allocated to packet node pointed by pPacketNode */
      //free(pPacketNode); 

      continue;
    }
    else
    {
      pPacketNode = (packet_queue_node_t*) Dequeue_With_QueueNodePointer((queue_t*)Q, (queue_node_t*)pPacketNode);
      i--;  //this decrement allows the next queue node be checked after this queue node
    }

    /* compute the packet's lifetime to check whether the packet expires */
    lifetime = current_time - pPacketNode->generation_time; //compute the packet's lifetime

    /** check whether packet's lifetime is greater than the predefined packet TTL */
    if(lifetime > pPacketNode->ttl) //if-1.1
    //if(lifetime > param->communication_packet_ttl) //if-1.1
    {
#if TPD_SOURCE_ROUTING_DESTINATION_VEHICLE_FORWARDING_TRACE_FLAG /* [*/
		printf("<D> %s:%d [%0.f] current_carrier(%d) in (%s->%s: %0.f) cannot deliver packet(id=%d, lifetime=%0.f, TTL=%0.f) to destination_vehicle(%s->%s: %0.f).\n",
				__FUNCTION__, __LINE__,
				(float)current_time,
				carrier_vehicle->id,
				carrier_vehicle->current_pos_in_digraph.tail_node,
				carrier_vehicle->current_pos_in_digraph.head_node,
				carrier_vehicle->current_pos_in_digraph.offset,
				pPacketNode->id,
				(float)lifetime,
				(float)param->communication_packet_ttl,
				param->vanet_table.dst_vnode->current_pos_in_digraph.tail_node,			
				param->vanet_table.dst_vnode->current_pos_in_digraph.head_node,
	   			param->vanet_table.dst_vnode->current_pos_in_digraph.offset);
#endif /* ] */
      VADD_Discard_Expired_Packet(param, current_time, pPacketNode, VANET_NODE_VEHICLE, (void*)destination_vehicle, packet_delivery_stat,pPacketNode->ttl); //discard the expired packet by destroying the memory of the packet while the correponding global packet queue node is dequeued and destroyed 
      

      continue;
    } //end of if-1.1

#if TPD_SOURCE_ROUTING_DESTINATION_VEHICLE_FORWARDING_TRACE_FLAG /* [*/
		printf("<D> %s:%d [%0.f] current_carrier(%d) in (%s->%s: %0.f) can deliver packet(id=%d, lifetime=%0.f, TTL=%0.f) to destination_vehicle(%s->%s: %0.f).\n",
				__FUNCTION__, __LINE__,
				(float)current_time,
				carrier_vehicle->id,
				carrier_vehicle->current_pos_in_digraph.tail_node,
				carrier_vehicle->current_pos_in_digraph.head_node,
				carrier_vehicle->current_pos_in_digraph.offset,
				pPacketNode->id,
				(float)lifetime,
				(float)param->communication_packet_ttl,
				param->vanet_table.dst_vnode->current_pos_in_digraph.tail_node,			
				param->vanet_table.dst_vnode->current_pos_in_digraph.head_node,
	   			param->vanet_table.dst_vnode->current_pos_in_digraph.offset);
#endif /* ] */

    /* set up packet's fields */
    pPacketNode->carry_src_id = carrier_vehicle->id; //update the packet's carry_src_vid with carrier_vehicle's id
    pPacketNode->carry_dst_id = destination_vehicle->id; //update the packet's carry_dst_vid with destination_vehicle's id

#if 1 /* [ */
	/* Note: if the destination vehicle received the packet from itself, let the packet_transmission_count not increase. */
	if(carrier_vehicle->id != destination_vehicle->id)
	{
		pPacketNode->packet_transmission_count++;
	}
	else
	{
		printf("%s:%d: carrier_vehicle(%d) is the same as destination_vehicle(%d)\n",
				__FUNCTION__, __LINE__,
				carrier_vehicle->id, destination_vehicle->id);
	}
#endif /* ] */

    /* set up pointers to the carrier vehicle in both the packet queue node and the global packet queue node corresponding to pPacketNode */
    pPacketNode->carrier_vnode = destination_vehicle; //pointer to the destination vehicle
    pPacketNode->global_packet->carrier_vnode = destination_vehicle; //pointer to the destination vehicle

    pPacketNode->last_receive_time = current_time; //last packet receive time
    pPacketNode->destination_arrival_time = current_time; //packet's AP arrival time means the arrival time to the destination vehicle

	/* increase the number of delivered packets */
	delivered_packet_number++; 

//#ifdef __LOG_LEVEL_VANET_PACKET_DESTINATION_VEHICLE_ARRIVAL__
    log_vanet(VANET_LOG_PACKET_DESTINATION_VEHICLE_ARRIVAL, current_time, pPacketNode, packet_delivery_stat);
    //log the data forwarding to the destination vehicle on VANET into VANET logging file
//#endif

#if defined(__LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__) || defined(__LOG_LEVEL_VANET_PACKET_CARRIER_TRACE_FOR_STATIONARY_NODE__)
    /** enqueue the packet carrier trace entry into packet's carrier trace queue */
    Enqueue_CarrierTraceEntry(param, current_time, pPacketNode, VANET_NODE_VEHICLE, (void*)destination_vehicle);

    /** log packet carrier trace for this packet into carrier_trace_file */
    log_vanet_packet_carrier_trace(VANET_LOG_PACKET_DESTINATION_VEHICLE_ARRIVAL, current_time, pPacketNode);
#endif
    
    /* dequeue global packet queue node, destroy global packet queue node and then free the memory of global packet queue node */
    DestroyGlobalPacketQueueNode(pPacketNode->global_packet);   

    /* destroy packet queue node along with vehicle trajectory queue and carrier trace queue, and then free the memory of packet queue node */
    DestroyPacketQueueNode(pPacketNode);
  }

  /* check whether carrier's packet queue is empty or not; if so, reset the latest packet information */
  if(carrier_vehicle->packet_queue->size == 0)
  {
    carrier_vehicle->latest_packet_seq = 0; //reset the latest packet seq number
    carrier_vehicle->latest_packet_receive_time = 0;
    carrier_vehicle->latest_packet_ptr = NULL;
    carrier_vehicle->target_point_id = 0; //reset the target point id
  }
  else
  {
    //update the carrier's latest packet with the greatest seq in packet queue
    printf("VADD_Forward_Packet_To_Destination_Vehicle(): carrier_vehicle->packet_queue->size(%d) is greater than zero\n", carrier_vehicle->packet_queue->size);
    exit(1);
  }

#if TPD_SOURCE_ROUTING_ROAD_SEGMENT_FORWARDING_TRACE_FLAG || TPD_SOURCE_ROUTING_DESTINATION_VEHICLE_FORWARDING_TRACE_FLAG /* [*/
	printf("<D> %s:%d [%0.f] current_carrier(%d) in (%s->%s: %0.f) delivers %d packets among %d packets in Q to destination_vehicle(%s->%s: %0.f).\n",
			__FUNCTION__, __LINE__,
			(float)current_time,
			carrier_vehicle->id,
			carrier_vehicle->current_pos_in_digraph.tail_node,
			carrier_vehicle->current_pos_in_digraph.head_node,
			carrier_vehicle->current_pos_in_digraph.offset,
			delivered_packet_number,
			size,
			param->vanet_table.dst_vnode->current_pos_in_digraph.tail_node,			
   			param->vanet_table.dst_vnode->current_pos_in_digraph.head_node,
   			param->vanet_table.dst_vnode->current_pos_in_digraph.offset);
#endif /* ] */
}

int VADD_Forward_Packet_From_AP_To_Next_Carrier(parameter_t *param, double current_time, struct_access_point_t *AP, struct_vehicle_t *next_carrier, packet_delivery_statistics_t *packet_delivery_stat, int *discard_count)
{ //AP forwards its packet(s) to the next carrier pointed by next_carrier and return forward count
	packet_queue_node_t *pPacketNode = NULL; //pointer to a packet node
	int size = 0; //size of packet queue
	int i = 0; //index for for-loop
	double lifetime = 0; //packet lifetime
	int forward_count = 0; //count for forwarded packets
	double EDR = 0; //Expected Delivery Ratio (EDR)

	*discard_count = 0; //count for discarded packets

	/** forward AP's packets to next_carrier */
	size = AP->packet_queue->size;
	if(size == 0)
		return 0;

	for(i = 0; i < size; i++)
	{
		pPacketNode = (packet_queue_node_t*) Dequeue((queue_t*)AP->packet_queue); //dequeue a packet placed at AP's packet queue front

#if 1 /* [ */
		/* reset packet generation time to the delivery start time from AP to the next carrier */
		pPacketNode->generation_time = current_time;	
#endif /* ] */

		/* compute the packet's lifetime to check whether the packet expires */
		lifetime = current_time - pPacketNode->generation_time; //compute the packet's lifetime

		/** check whether packet's lifetime is greater than the predefined packet TTL */
		if(lifetime > pPacketNode->ttl) //if-1.1
		{
			VADD_Discard_Expired_Packet(param, current_time, pPacketNode, VANET_NODE_VEHICLE, (void*)next_carrier, packet_delivery_stat,pPacketNode->ttl); //discard the expired packet by destroying the memory of the packet while the correponding global packet queue node is dequeued and destroyed 
			(*discard_count)++;

			continue;
		} //end of if-1.1

		/* set up packet's fields */
		pPacketNode->last_receive_time = current_time; //last packet receive time
		pPacketNode->carry_src_id = AP->id; //AP's id
		pPacketNode->carry_dst_id = next_carrier->id; //next packet carrier's id
		pPacketNode->packet_transmission_count++;

		/* set up pointers to the carrier vehicle in both the packet queue node and the global packet queue node corresponding to pPacketNode */
		pPacketNode->carrier_vnode = next_carrier; //pointer to the next packet carrier vehicle
		pPacketNode->global_packet->carrier_vnode = next_carrier; //pointer to the next packet carrier vehicle

#if 0 /* [ */
		/* contruct a predicted encounter graph for the packet with next_carrier */
#if TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FOR_PACKET_FLAG /* [ */
		EDR = TPD_Construct_Predicted_Encounter_Graph_For_Packet(current_time, param, pPacketNode, next_carrier, param->vanet_table.dst_vnode, TRUE);
#else
		EDR = TPD_Construct_Predicted_Encounter_Graph_For_Packet(current_time, param, pPacketNode, next_carrier, param->vanet_table.dst_vnode, FALSE);
#endif /* ] */
#endif /* ] */
    
	    Enqueue_With_QueueNodePointer((queue_t*)next_carrier->packet_queue, (queue_node_t*)pPacketNode); //enqueue the packet into next_carrier's packet queue rear

		forward_count++; //increase packet count

#ifdef  __LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__
	    /** enqueue the packet carrier trace entry into packet's carrier trace queue */
	    Enqueue_CarrierTraceEntry(param, current_time, pPacketNode, VANET_NODE_VEHICLE, (void*)next_carrier);
#endif

		/** update target point information */
		if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD) //if-1
		{
			if((next_carrier->latest_packet_ptr == NULL) || (next_carrier->latest_packet_seq <= pPacketNode->seq)) 
			/* (i) "vehicle->latest_packet_ptr == NULL" means that vehicle has no packet, so updates its latest packet information
			   (ii) "== case" is that next_carrier has previously carried the packet and passed it to another vehicle. Now next_carrier receives the packet again
*/
			{
				next_carrier->target_point_id = pPacketNode->target_point_id; //update a target point with the new one
				next_carrier->latest_packet_seq = pPacketNode->seq;
				next_carrier->latest_packet_receive_time = current_time;
				next_carrier->latest_packet_ptr = pPacketNode;
			}
		} //end of if-1
		else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
		{
			if(param->vanet_forwarding_scheme == VANET_FORWARDING_TPD)
			{
				/* construct a predicted encounter graph for the packet with next_carrier that will be used by TPD forwarding scheme */
#if TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FOR_PACKET_FLAG /* [ */
				EDR = TPD_Construct_Predicted_Encounter_Graph_For_Packet(current_time, param, pPacketNode, next_carrier, param->vanet_table.dst_vnode, TRUE);
#else
				EDR = TPD_Construct_Predicted_Encounter_Graph_For_Packet(current_time, param, pPacketNode, next_carrier, param->vanet_table.dst_vnode, FALSE);
#endif /* ] */
			}

			if(forward_count > 0)
			{
				if(next_carrier->type == VEHICLE_CARRIER)
				{
					next_carrier->type = VEHICLE_CURRENT_PACKET_CARRIER; //let this vehicle be a packet carrier
					if((next_carrier->latest_packet_ptr == NULL) || (next_carrier->latest_packet_seq <= pPacketNode->seq))
					{
						next_carrier->target_point_id = pPacketNode->target_point_id; //update a target point with the new one
						next_carrier->latest_packet_seq = pPacketNode->seq;
						next_carrier->latest_packet_receive_time = current_time;
						next_carrier->latest_packet_ptr = pPacketNode;
					}
				}
				else if(next_carrier->type == VEHICLE_DESTINATION)
				{
					printf("%s:%d next_carrier(%d) is destination_vehicle!\n",
							__FUNCTION__, __LINE__,
							next_carrier->id);	

					VADD_Forward_Packet_To_Destination_Vehicle(param, current_time, next_carrier, next_carrier, packet_delivery_stat); //next_carrier delivers its packet to itself.
				}
			}
		}
	}

	return forward_count;
}

void VADD_Forward_Packet_From_AP_To_Stationary_Node(parameter_t *param, double current_time, struct_access_point_t *AP, int intersection_id, packet_delivery_statistics_t *packet_delivery_stat)
{ //AP forwards its packet(s) to the stationary node at the same intersection
  packet_queue_node_t *pPacketNode = NULL; //pointer to a packet node
  int size = 0; //size of packet queue
  int i = 0; //index for for-loop
  double lifetime = 0; //packet lifetime
  stationary_node_queue_node_t *stationary_node = NULL; //pointer to the stationary node corresponding to intersection_id
  int src_id = 0; //source intersection id
  int dst_id = 0; //destination intersection id
  double EDD = 0; //Expected Delivery Delay
  double EDD_SD = 0; // Delivery Delay Standard Deviation  

  /** get the stationary node for intersection_id */
  if(intersection_id > param->vanet_table.Gr_size)
  {
    printf("VADD_Forward_Packet_From_AP_To_Stationary_Node: Error: intersection_id(%d) > param->vanet_table.Gr_size(%d)\n", intersection_id, param->vanet_table.Gr_size);
    exit(1);
  }
  stationary_node = param->vanet_table.Gr[intersection_id-1].ptr_stationary_node;

  /** forward AP's packets to next_carrier */
  size = AP->packet_queue->size;
  if(size == 0)
    return;

  /**@ for debugging */
  //if(current_time > 3600 && next_carrier->id == 19)
  //{
  //  printf("VADD_Forward_Packet_From_AP_To_Next_Carrier(): time=%.2f, debugging\n", current_time);    
  //}
  /*******************/

#ifdef __DEBUG_LEVEL_CARRIER_SEARCH_BY_AP__
  printf("VADD_Forward_Packet_From_AP_To_Stationary_Node():[Stationary Node] stationary_node=%d\n", intersection_id);
  fgetc(stdin);
#endif

  for(i = 0; i < size; i++)
  {
    pPacketNode = (packet_queue_node_t*) Dequeue((queue_t*)AP->packet_queue); //dequeue a packet placed at AP's packet queue front

    /* compute the packet's lifetime to check whether the packet expires */
    lifetime = current_time - pPacketNode->generation_time; //compute the packet's lifetime

    /** check whether packet's lifetime is greater than the predefined packet TTL */
    if(lifetime > pPacketNode->ttl) //if-1.1
    //if(lifetime > param->communication_packet_ttl) //if-1.1
    {
      VADD_Discard_Expired_Packet(param, current_time, pPacketNode, VANET_NODE_SNODE, (void*)stationary_node, packet_delivery_stat,pPacketNode->ttl); //discard the expired packet by destroying the memory of the packet while the correponding global packet queue node is dequeued and destroyed 

      continue;
    } //end of if-1.1

    /* set up packet's fields */
    pPacketNode->last_receive_time = current_time; //last packet receive time
    pPacketNode->carry_src_id = AP->id; //AP's id
    pPacketNode->carry_dst_id = stationary_node->intersection_id; //next packet carrier's id

    /* set up pointers to the carrier vehicle in both the packet queue node and the global packet queue node corresponding to pPacketNode */
    pPacketNode->carrier_vnode = NULL; //pointer to the next packet carrier vehicle
    pPacketNode->global_packet->carrier_vnode = NULL; //pointer to the next packet carrier vehicle
    
    Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue the packet into stationary node's packet queue rear

#if defined(__LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__) || defined(__LOG_LEVEL_VANET_PACKET_CARRIER_TRACE_FOR_STATIONARY_NODE__)
    /** enqueue the packet carrier trace entry into packet's carrier trace queue */
    Enqueue_CarrierTraceEntry(param, current_time, pPacketNode, VANET_NODE_SNODE, (void*)stationary_node);
#endif

    /** update target point information */
    if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD) //if-1
    {
      if((stationary_node->latest_packet_ptr == NULL) || (stationary_node->latest_packet_seq <= pPacketNode->seq)) 
	/* (i) "stationary_node->latest_packet_ptr == NULL" means that stationary node has no packet, so updates its latest packet information
	   (ii) "== case" is that stationary node has previously carried the packet and passed it to another vehicle. Now stationary node receives the packet again
	 */
      {
        stationary_node->target_point_id = pPacketNode->target_point_id; //update a target point with the new one
	stationary_node->latest_packet_seq = pPacketNode->seq;
	stationary_node->latest_packet_receive_time = current_time;
	stationary_node->latest_packet_ptr = pPacketNode;
      }
    } //end of if-1
  }
}

void VADD_Forward_Packet_From_Stationary_Node_To_Next_Carrier(parameter_t *param, double current_time, int intersection_id, struct_vehicle_t *next_carrier, packet_delivery_statistics_t *packet_delivery_stat)
{ //forward from the stationary node at the intersection corresponding to intersection_id to next_vehicle the packets that are needed to go to the edge where vehicle is moving
  packet_queue_node_t *pPacketNode = NULL; //pointer to a packet node
  int size = 0; //size of packet queue
  int i = 0; //index for for-loop
  double lifetime = 0; //packet lifetime
  stationary_node_queue_node_t *stationary_node = NULL; //pointer to the stationary node corresponding to intersection_id
  boolean flag = FALSE; //boolean flag
  boolean packet_earlier_arrival_flag = FALSE; //flag to indicate whether the packet has arrived earlier than the destination vehicle at a target point on the destination vehicle's trajectory
  int new_dst = 0; //new packet destination, i.e., a tentative target point
  int new_target_point = 0; //new target point considering the destination vehicle's current position
  char tail_node_for_target_point[NAME_SIZE]; //tail node towards the new target point
  char head_node_for_target_point[NAME_SIZE]; //head node towards the new target point
  double EDD_p = 0; //Expected Delivery Delay (EDD) from this stationary node to the target point p
  double EAD_p = 0; //Expected Arrival Delay (EAD) from the destination vehicle's current position to the target point p
  char packet_src_vertex[NAME_SIZE]; //packet's source vertex name
  char packet_dst_vertex[NAME_SIZE]; //packet's destination vertex name

  boolean dst_vehicle_flag = FALSE; //flag to indicate whether the destination vehicle is within the communication range of this stationary node

  /*@For debugging */
  //if(current_time >= 3929 && intersection_id == 33 && next_carrier->id == 2)
  //  printf("VADD_Forward_Packet_From_Stationary_Node_To_Next_Carrier(): current_time=%.2f, intersection_id=%d, next_carrier->id=%d\n", current_time, intersection_id, next_carrier->id);
  /*****************/

  /** get the stationary node for intersection_id */
  if(intersection_id > param->vanet_table.Gr_size)
  {
    printf("VADD_Forward_Packet_From_Stationary_Node_To_Next_Carrier(): Error: intersection_id(%d) > param->vanet_table.Gr_size(%d)\n", intersection_id, param->vanet_table.Gr_size);
    exit(1);
  }
  stationary_node = param->vanet_table.Gr[intersection_id-1].ptr_stationary_node;

  /** check whether are packets in the stationary node */
  size = stationary_node->packet_queue.size;
  if(size == 0)
    return;

  /*@ for debugging */
  //if(next_carrier->id == param->vanet_table.dst_vnode->id)
  //{
  //  printf("VADD_Forward_Packet_From_Stationary_Node_To_Next_Carrier_VERSION_3(): for debugging\n");
  //}
  /******************/

  /** compute the new target point for the fully dynamic forwarding */
/*   if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_FULLY_DYNAMIC_FORWARDING) */
/*   { */
/*     /\* check whether this stationary node is on the vehicle trajectory or not *\/ */
/*     flag = Is_Vertex_On_VehicleTrajectory_With_New_Destination(param, current_time, stationary_node->latest_packet_ptr, intersection_id, &new_dst, &packet_earlier_arrival_flag); */

/*     if(flag == FALSE) */
/*     { //the case where the stationary node is not on the destination vehicle's trajectory; we compute a new target point */
/*         new_target_point = GetTargetPoint_For_StationaryNode(param, current_time, stationary_node, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory */
/*     } */
/*     else */
/*     { //the case where the stationary node is on the destination vehicle's trajectory */
/*         new_target_point = new_dst; */
/*     } */
/*   } */

  /** check whether the destination vehicle is within the communication range of the stationary node or not; if so, the stationary node forwards its packets to the destination node rather than the next carrier moving towards the intended edge */
  dst_vehicle_flag = is_vehicle_within_communication_range_of_point(param, param->vanet_table.dst_vnode, &(stationary_node->gnode->coordinate));
  //dst_vehicle_flag = is_vehicle_within_communication_range_of_point(param, stationary_node->latest_packet_ptr->dst_vnode, &(stationary_node->gnode->coordinate));

  for(i = 0; i < size; i++) //for-1
  {
    pPacketNode = (packet_queue_node_t*) Dequeue((queue_t*)&(stationary_node->packet_queue)); //dequeue a packet placed at AP's packet queue front

    /* compute the packet's lifetime to check whether the packet expires */
    lifetime = current_time - pPacketNode->generation_time; //compute the packet's lifetime

    /** check whether packet's lifetime is greater than the predefined packet TTL */
    if(lifetime > pPacketNode->ttl) //if-1.1
    //if(lifetime > param->communication_packet_ttl) //if-1.1
    {
      VADD_Discard_Expired_Packet(param, current_time, pPacketNode, VANET_NODE_SNODE, (void*)stationary_node, packet_delivery_stat,pPacketNode->ttl); //discard the expired packet by destroying the memory of the packet while the correponding global packet queue node is dequeued and destroyed 

      continue;
    } //end of if-1.1

    /** if the destination vehicle is within the communication range of the stationary node, the stationary node directly sends its packets to the destination vehicle */
    if(dst_vehicle_flag == TRUE)
    {
        /* set up packet's fields */
        pPacketNode->carry_src_id = stationary_node->intersection_id; //update the packet's carry_src_vid with stationary_node->intersection_id
        pPacketNode->carry_dst_id = next_carrier->id; //update the packet's carry_dst_vid with destination_vehicle's id
		pPacketNode->packet_transmission_count++;
  
        /* set up pointers to the carrier vehicle in both the packet queue node and the global packet queue node corresponding to pPacketNode */
        pPacketNode->carrier_vnode = next_carrier; //pointer to the destination vehicle
        pPacketNode->global_packet->carrier_vnode = next_carrier; //pointer to the destination vehicle

        pPacketNode->last_receive_time = current_time; //last packet receive time
        pPacketNode->destination_arrival_time = current_time; //packet's destination arrival time means the arrival time to the destination vehicle

//#ifdef __LOG_LEVEL_VANET_PACKET_DESTINATION_VEHICLE_ARRIVAL__
        log_vanet(VANET_LOG_PACKET_DESTINATION_VEHICLE_ARRIVAL, current_time, pPacketNode, packet_delivery_stat); //log the data forwarding to the destination vehicle on VANET into VANET logging file
//#endif

#ifdef  __LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__
        /** enqueue the packet carrier trace entry into packet's carrier trace queue */
        Enqueue_CarrierTraceEntry(param, current_time, pPacketNode, VANET_NODE_VEHICLE, (void*)next_carrier);

        /** log packet carrier trace for this packet into carrier_trace_file */
        log_vanet_packet_carrier_trace(VANET_LOG_PACKET_DESTINATION_VEHICLE_ARRIVAL, current_time, pPacketNode);
#endif

        /* dequeue global packet queue node, destroy global packet queue node and then free the memory of global packet queue node */
        DestroyGlobalPacketQueueNode(pPacketNode->global_packet);   

        /* destroy packet queue node along with vehicle trajectory queue and carrier trace queue, and then free the memory of packet queue node */
        DestroyPacketQueueNode(pPacketNode);

        /*@ Don't forget to put "continue" here to go back to the loop! */
        continue;
    }

    /*@For debugging */
    //if(pPacketNode->id == 302)
    //if(pPacketNode->id == 302 && stationary_node->intersection_id == 28)
    //  printf("VADD_Forward_Packet_FromStationary_Node_To_Next_Carrier(): current_time=%.2f, pPacketNode->id=%d\n", current_time, pPacketNode->id);
    /*****************/

    /** replace the packet's vehicle trajectory and packet trajectory with those of the latest packet for the fully dynamic forwardign */
/*     if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_FULLY_DYNAMIC_FORWARDING) */
/*     { */
/*         if(pPacketNode->id != stationary_node->latest_packet_ptr->id) */
/*         { */
/*             Copy_Vehicle_Trajectory(param, current_time, stationary_node->latest_packet_ptr, pPacketNode); //replace the packet's vehicle trajectory with the latest packet's vehicle trajectory */

/*             Copy_Packet_Trajectory(param, current_time, stationary_node->latest_packet_ptr, pPacketNode);//replace the packet's packet trajectory with the latest packet's packet trajectory */
/*         } */
/*     } */
    
    /** check whether packet's target point is equal to stationary node's intersection id; if so, check whether next_carrier is the packet's destination vehicle or not */
    if(pPacketNode->target_point_id == stationary_node->intersection_id) //if-1.2
    { //the case where the packet has already arrived at the packet's target point and so is waiting for the destination vehicle
		/* for the partial deployment of stationary nodes, let the packet stay at the target point without the reverse traversal along the vehicle trajectory */
		if((param->communication_multiple_SN_flag == TRUE) || (param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_STATIC_FORWARDING)) //if-1.2.2.1
		//if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_STATIC_FORWARDING) //if-1.2.2.1
        { //let the packet stay at the target point intersection
          Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue

          continue;
        } //end of if-1.2.2.1
        else if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_PARTIALLY_DYNAMIC_FORWARDING) //else if-1.2.2.2
        { //let the packet compute a new target point intersection using the destination vehicle's trajectory for partially dynamic forwarding
          /** check whether the packet is traversing reversely along the destination vehicle's trajectory */
          if(pPacketNode->reverse_traversal_mode_flag == TRUE) //if-1.2.2.2.1
          { //the case where the packet is being delivered under the reverse traversal mode

            /* check the completion of the reverse traverse in the vehicle trajectory; if so, skip the remaining part and wait for the destination vehicle until its TTL */
            if(pPacketNode->reverse_traversal_completion_flag == TRUE)
            { //the case where the installation of a new packet trajectory towards a new destination node due to the expiration of the original packet trajectory; if so, let the packet stay at this stationary node
                Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue
              
                continue;
            } 

            /* get the next hop of the current intersection on the reverse path of the vehicle trajectory */
            if(pPacketNode->reverse_traversal_next_hop_flag == FALSE) //if-1.2.2.2.1.1
            { //the case where the packet's next hop is not set yet for its reverse traversal

              /* check whether there exists a next hop for the packet's reverse traversal */
              flag = Is_There_NextHop_In_PacketReverseTraversal(pPacketNode, stationary_node->intersection_id, &new_dst);

              /* set up the packet_src_vertex and packet_dst_vertex */
              itoa(stationary_node->intersection_id, packet_src_vertex);
              itoa(new_dst, packet_dst_vertex);
               
              if(flag == TRUE) //if-1.2.2.2.1.1.1
              {
                flag = Install_PacketTrajectory_Into_Packet_With_Edge(param, current_time, param->vanet_table.Gr, param->vanet_table.Gr_size, packet_src_vertex, packet_dst_vertex, pPacketNode);
          
                if(flag == FALSE) //if-1.2.2.2.1.1.1.1
                { //the case where the installation of a new packet trajectory towards a new destination node due to the expiration of the original packet trajectory; if so, let the packet stay at this stationary node
                  Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue
              
                  continue;
                } //end of if-1.2.2.2.1.1.1.1
              } //end of if-1.2.2.2.1.1.1
              else //else-1.2.2.2.1.1.2
              { //In the case where the packet has reached the start point of the vehicle trajectory, but it couldn't meet the destination vehicle, we let this packet wait for the destination vehicle until its TTL expires.
                  //VADD_Discard_Expired_Packet(param, current_time, pPacketNode, VANET_NODE_SNODE, (void*)stationary_node, packet_delivery_stat); //discard the expired packet by destroying the memory of the packet while the correponding global packet queue node is dequeued and destroyed 

                  Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue, letting the packet stay at the stationary node in order to make the performance of the partially dynamic forwarding have the same delivery probability (or delivery ratio) with the static forwarding

                  continue;
              } //end of else-1.2.2.2.1.1.2
            } //end of if-1.2.2.2.1.1 
          } //end of if-1.2.2.2.1
          ////////////////////////////////////////////////////////////////////////////////////
          else //else-1.2.2.2.2
          { //the case where the packet is being delivered under the non-reverse traverse mode

            /* check whether the packet has arrived at the target point on the destination vehicle's trajectory earlier than the destination vehicle and also determine the new destination intersection */
            flag = Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination(param, current_time, pPacketNode, &new_dst);

            /* set up the packet_src_vertex and packet_dst_vertex */
            itoa(stationary_node->intersection_id, packet_src_vertex);
            itoa(new_dst, packet_dst_vertex);
          
            if(stationary_node->intersection_id == new_dst) //if-1.2.2.2.2.1
            { //In the case where the stationary node's id is equal to new destination intersection id, we let the packet stay at this stationary node
              Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue
              
              continue;
            } //end of if-1.2.2.2.2.1
            else if(flag == TRUE) //if-1.2.2.2.2.2
            {
              flag = Install_PacketTrajectory_Into_Packet_With_Edge(param, current_time, param->vanet_table.Gr, param->vanet_table.Gr_size, packet_src_vertex, packet_dst_vertex, pPacketNode);
          
              if(flag == FALSE) //if-1.2.2.2.2.2.1
              { //the case where the installation of a new packet trajectory towards a new destination node due to the expiration of the original packet trajectory; if so, let the packet stay at this stationary node
                Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue
              
                continue;
              } //end of if-1.2.2.2.2.2.1
            } //end of if-1.2.2.2.2.2
            else //else-1.2.2.2.2.3
            {
              flag = Install_PacketTrajectory_Into_Packet(param, current_time, param->vanet_table.Gr, param->vanet_table.Gr_size, packet_src_vertex, packet_dst_vertex, pPacketNode);
              if(flag == FALSE) //if-1.2.2.2.3.1
              { //the case where the installation of a new packet trajectory towards a new target point due to the expiration of the original packet trajectory; if so, let the packet stay at this stationary node
                Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue

                continue;
              } //end of if-1.2.2.2.2.3.1
            } //end of else-1.2.2.2.2.3
          } //end of else-1.2.2.2.2
        } //end of if-1.2.2.2
        else if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_FULLY_DYNAMIC_FORWARDING) //else if-1.2.2.3
        { //let the packet compute a new target point intersection using the destination vehicle's trajectory for fully dynamic forwarding

          /** check whether the packet is traversing reversely along the destination vehicle's trajectory */
          if(pPacketNode->reverse_traversal_mode_flag == TRUE) //if-1.2.2.3.1
          { //the case where the packet is being delivered under the reverse traversal mode

            /* check the completion of the reverse traverse in the vehicle trajectory; if so, skip the remaining part and wait for the destination vehicle until its TTL */
            if(pPacketNode->reverse_traversal_completion_flag == TRUE)
            { //the case where the installation of a new packet trajectory towards a new destination node due to the expiration of the original packet trajectory; if so, let the packet stay at this stationary node
                Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue
              
                continue;
            } 

            /* get the next hop of the current intersection on the reverse path of the vehicle trajectory */
            if(pPacketNode->reverse_traversal_next_hop_flag == FALSE) //if-1.2.2.3.1.1
            { //the case where the packet's next hop is not set yet for its reverse traversal
              /* check whether there exists a next hop for the packet's reverse traversal given the vehicle trajectory and the current intersection id */
              flag = Is_There_NextHop_In_PacketReverseTraversal(pPacketNode, stationary_node->intersection_id, &new_dst);

              /* set up the packet_src_vertex and packet_dst_vertex */
              itoa(stationary_node->intersection_id, packet_src_vertex);
              itoa(new_dst, packet_dst_vertex);
               
              if(flag == TRUE) //if-1.2.2.3.1.1.1
              {
                flag = Install_PacketTrajectory_Into_Packet_With_Edge(param, current_time, param->vanet_table.Gr, param->vanet_table.Gr_size, packet_src_vertex, packet_dst_vertex, pPacketNode);
          
                if(flag == FALSE) //if-1.2.2.3.1.1.1.1
                { //the case where the installation of a new packet trajectory towards a new destination node due to the expiration of the original packet trajectory; if so, let the packet stay at this stationary node
                  Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue
              
                  continue;
                } //end of if-1.2.2.3.1.1.1.1
              } //end of if-1.2.2.3.1.1.1
              else //else-1.2.2.3.1.1.2
              { //In the case where the packet has reached the start point of the vehicle trajectory, but it couldn't meet the destination vehicle, we discard this packet since the packet missed the destination vehicle!
                  //VADD_Discard_Expired_Packet(param, current_time, pPacketNode, VANET_NODE_SNODE, (void*)stationary_node, packet_delivery_stat); //discard the expired packet by destroying the memory of the packet while the correponding global packet queue node is dequeued and destroyed 

                  Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue, letting the packet stay at the stationary node in order to make the performance of the partially dynamic forwarding have the same delivery probability (or delivery ratio) with the static forwarding

                  continue;
              } //end of else-1.2.2.3.1.1.2
            } //end of if-1.2.2.3.1.1 
          } //end of if-1.2.2.3.1
          ////////////////////////////////////////////////////////////////////////////////////
          else //else-1.2.2.3.2
          { //the case where the packet is being delivered under the non-reverse traverse mode

            /* check whether the packet is on the destination vehicle's trajectory and also determine the new destination intersection */
            flag = Is_Vertex_On_VehicleTrajectory_With_New_Destination(param, current_time, pPacketNode, stationary_node->intersection_id, &new_dst, &packet_earlier_arrival_flag);

            /* set up the packet_src_vertex and packet_dst_vertex */
            itoa(stationary_node->intersection_id, packet_src_vertex);
            itoa(new_dst, packet_dst_vertex);

            if(stationary_node->intersection_id == new_dst) //if-1.2.2.3.2.1
            { //In the case where the stationary node's id is equal to new destination intersection id, we let the packet stay at this stationary node
              Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue
              
              continue;
            } //end of if-1.2.2.3.2.1
            else if(flag == TRUE && pPacketNode->reverse_traversal_mode_flag == TRUE) //if-1.2.2.3.2.2
            { //In the case where the packet is on the vehicle trajectory and has arrived earlier than the destination vehicle, we let the packet traverse reversely along the vehicle trajectory
              flag = Install_PacketTrajectory_Into_Packet_With_Edge(param, current_time, param->vanet_table.Gr, param->vanet_table.Gr_size, packet_src_vertex, packet_dst_vertex, pPacketNode);
          
              if(flag == FALSE) //if-1.2.2.3.2.2.1
              { //the case where the installation of a new packet trajectory towards a new destination node due to the expiration of the original packet trajectory; if so, let the packet stay at this stationary node
                Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue
              
                continue;
              } //end of if-1.2.2.3.2.2.1
            } //end of if-1.2.2.3.2.2
            else if(flag == TRUE && pPacketNode->reverse_traversal_mode_flag == FALSE) //if-1.2.2.3.2.3
            { //In the case where the packet is on the vehicle trajectory and has arrived later than the destination vehicle, we let the packet go towards the new target point returned by Is_Vertex_On_VehicleTrajectory_With_New_Destination() through new_dst
              flag = Install_PacketTrajectory_Into_Packet(param, current_time, param->vanet_table.Gr, param->vanet_table.Gr_size, packet_src_vertex, packet_dst_vertex, pPacketNode);
          
              if(flag == FALSE) //if-1.2.2.3.2.3.1
              { //the case where the installation of a new packet trajectory towards a new destination node due to the expiration of the original packet trajectory; if so, let the packet stay at this stationary node
                Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue
              
                continue;
              } //end of if-1.2.2.3.2.3.1
            } //end of if-1.2.2.3.2.3
            else //else-1.2.2.3.2.4
            {
              /* select a new target point */
              new_target_point = GetTargetPoint_For_Packet(param, current_time, pPacketNode, stationary_node->intersection_id, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory
              //new_target_point = GetTargetPoint_For_StationaryNode(param, current_time, stationary_node, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory

              /* update packet_dst_vertex with new_target point */
              itoa(new_target_point, packet_dst_vertex);

              flag = Install_PacketTrajectory_Into_Packet(param, current_time, param->vanet_table.Gr, param->vanet_table.Gr_size, packet_src_vertex, packet_dst_vertex, pPacketNode);
              if(flag == FALSE) //if-1.2.2.3.2.4.1
              { //the case where the installation of a new packet trajectory towards a new target point due to the expiration of the original packet trajectory; if so, let the packet stay at this stationary node
                Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue

                continue;
              } //end of if-1.2.2.3.2.4.1
            } //end of else-1.2.2.3.2.4
          } //end of else-1.2.2.3.2
        } //end of else if-1.2.2.3
        else //else-1.2.2.4
        { //For the other vehicle_vanet_target_point_computation_methods, let the packet stay at the stationary node corresponding to the target point
            //printf("VADD_Forward_Packet_From_Stationary_Node_To_Next_Carrier(): Error: param->vehicle_vanet_target_point_computation_method(%d) is not supported\n", param->vehicle_vanet_target_point_computation_method);
            //exit(1);

            /* let the packet stay at the target point intersection */
            Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue

            continue;
        } //end of else-1.2.2.4
    } //end of if-1.2
    else //else-1.3
    {
      /** for the fully dynamic forwarding, let the packet select a new target point */
      if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_FULLY_DYNAMIC_FORWARDING) //if-1.3.1
      { //let the packet compute a new target point intersection using the destination vehicle's trajectory for fully dynamic forwarding

        /** check whether the packet is traversing reversely along the destination vehicle's trajectory */
        if(pPacketNode->reverse_traversal_mode_flag == TRUE) //if-1.3.1.1
        { //the case where the packet is being delivered under the reverse traversal mode

          /* check the completion of the reverse traverse in the vehicle trajectory; if so, skip the remaining part and wait for the destination vehicle until its TTL */
          if(pPacketNode->reverse_traversal_completion_flag == TRUE)
          { //the case where the installation of a new packet trajectory towards a new destination node due to the expiration of the original packet trajectory; if so, let the packet stay at this stationary node
              Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue
              
              continue;
          } 

          /* get the next hop of the current intersection on the reverse path of the vehicle trajectory */
          if(pPacketNode->reverse_traversal_next_hop_flag == FALSE) //if-1.3.1.1.1
          { //the case where the packet's next hop is not set yet for its reverse traversal
            /* check whether there exists a next hop for the packet's reverse traversal */
            flag = Is_There_NextHop_In_PacketReverseTraversal(pPacketNode, stationary_node->intersection_id, &new_dst);

            /* set up the packet_src_vertex and packet_dst_vertex */
            itoa(stationary_node->intersection_id, packet_src_vertex);
            itoa(new_dst, packet_dst_vertex);
               
            if(flag == TRUE) //if-1.3.1.1.1.1
            {
              flag = Install_PacketTrajectory_Into_Packet_With_Edge(param, current_time, param->vanet_table.Gr, param->vanet_table.Gr_size, packet_src_vertex, packet_dst_vertex, pPacketNode);
          
              if(flag == FALSE) //if-1.3.1.1.1.1.1
              { //the case where the installation of a new packet trajectory towards a new destination node due to the expiration of the original packet trajectory; if so, let the packet stay at this stationary node
                Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue
            
                continue;
              } //end of if-1.3.1.1.1.1.1
            } //end of if-1.3.1.1.1.1
            else //else-1.3.1.1.1.2
            { //In the case where the packet has reached the start point of the vehicle trajectory, but it couldn't meet the destination vehicle, we discard this packet since the packet missed the destination vehicle!
                //VADD_Discard_Expired_Packet(param, current_time, pPacketNode, VANET_NODE_SNODE, (void*)stationary_node, packet_delivery_stat); //discard the expired packet by destroying the memory of the packet while the correponding global packet queue node is dequeued and destroyed 

                Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue, letting the packet stay at the stationary node in order to make the performance of the partially dynamic forwarding have the same delivery probability (or delivery ratio) with the static forwarding

                continue;
            } //end of else-1.3.1.1.1.2
          } //end of if-1.3.1.1.1 
        } //end of if-1.3.1.1
        ////////////////////////////////////////////////////////////////////////////////////
        else //else-1.3.1.2
        { //the case where the packet is being delivered under the non-reverse traverse mode
          /* check whether the packet is on the destination vehicle's trajectory and also determine the new destination intersection */
          flag = Is_Vertex_On_VehicleTrajectory_With_New_Destination(param, current_time, pPacketNode, stationary_node->intersection_id, &new_dst, &packet_earlier_arrival_flag);

          /* set up the packet_src_vertex and packet_dst_vertex */
          itoa(stationary_node->intersection_id, packet_src_vertex);
          itoa(new_dst, packet_dst_vertex);

          if(stationary_node->intersection_id == new_dst) //if-1.3.1.2.1
          { //In the case where the stationary node's id is equal to new destination intersection id, we let the packet stay at this stationary node
            Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue
              
            continue;
          } //end of if-1.3.1.2.1
          else if(flag == TRUE && pPacketNode->reverse_traversal_mode_flag == TRUE) //if-1.3.1.2.2
          { //In the case where the packet is on the vehicle trajectory and has arrived earlier than the destination vehicle, we let the packet traverse reversely along the vehicle trajectory
            flag = Install_PacketTrajectory_Into_Packet_With_Edge(param, current_time, param->vanet_table.Gr, param->vanet_table.Gr_size, packet_src_vertex, packet_dst_vertex, pPacketNode);
          
            if(flag == FALSE) //if-1.3.1.2.2.1
            { //the case where the installation of a new packet trajectory towards a new destination node due to the expiration of the original packet trajectory; if so, let the packet stay at this stationary node
              Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue
              
              continue;
            } //end of if-1.3.1.2.2.1
          } //end of if-1.3.1.2.2
          else if(flag == TRUE && pPacketNode->reverse_traversal_mode_flag == FALSE) //if-1.3.1.2.3
          { //In the case where the packet is on the vehicle trajectory and has arrived later than the destination vehicle, we let the packet go towards the new target point returned by Is_Vertex_On_VehicleTrajectory_With_New_Destination() through new_dst
            flag = Install_PacketTrajectory_Into_Packet(param, current_time, param->vanet_table.Gr, param->vanet_table.Gr_size, packet_src_vertex, packet_dst_vertex, pPacketNode);
        
            if(flag == FALSE) //if-1.3.1.2.3.1
            { //the case where the installation of a new packet trajectory towards a new destination node due to the expiration of the original packet trajectory; if so, let the packet stay at this stationary node
              Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue
              
              continue;
            } //end of if-1.3.1.2.3.1
          } //end of if-1.3.1.2.3
          else //else-1.3.1.2.4
          {
            /* select a new target point */
            new_target_point = GetTargetPoint_For_Packet(param, current_time, pPacketNode, stationary_node->intersection_id, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory
            //new_target_point = GetTargetPoint_For_StationaryNode(param, current_time, stationary_node, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory

            /* update packet_dst_vertex with new_target point */
            itoa(new_target_point, packet_dst_vertex);

            flag = Install_PacketTrajectory_Into_Packet(param, current_time, param->vanet_table.Gr, param->vanet_table.Gr_size, packet_src_vertex, packet_dst_vertex, pPacketNode);
            if(flag == FALSE) //if-1.3.1.2.4.1
            { //the case where the installation of a new packet trajectory towards a new target point due to the expiration of the original packet trajectory; if so, let the packet stay at this stationary node
              Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue
               continue;
            } //end of if-1.3.1.2.4.1
          } //end of else-1.3.1.2.4
        } //end of else if-1.3.1.2
      } //end of else if-1.3.1 
    } //end of else-1.3

    //we check the case where the packet is in the middle of the delivery towards the packet's target point, so check whether next carrier is a right next hop moving to the intended direction towards the target point

    /** check whether the packet's packet trajectory has the vaild number of trajectory queue nodes, such as more than one; otherwise, let the packet stay at this stationary node  */
    if(pPacketNode->packet_trajectory.size <= 1)
    {
        Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue
        continue;
    }

    /** check whether next_carrier is moving the packet trajectory or not */
    flag = VADD_Is_Vehicle_Moving_On_Packet_Trajectory(next_carrier, &(pPacketNode->packet_trajectory));
    if(flag == FALSE) //if-1.4
    {
      Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue back the packet into the stationary node's packet queue

      continue;
    } //end of if-1.4

    /** check whether the reverse_traversal_mode_flag is turned on under the dynamic forwarding */
    if(pPacketNode->reverse_traversal_mode_flag == TRUE)
    {
      /* turn off pPacketNode->reverse_traversal_next_hop_flag to let the next-hop stationary node set the mnext hop on the reverse traversal for the vehicle trajectory */
      pPacketNode->reverse_traversal_next_hop_flag = FALSE;
    }

    /* set up packet's fields */
    pPacketNode->last_receive_time = current_time; //last packet receive time
    pPacketNode->carry_src_id = stationary_node->intersection_id; //stationary node's id
    pPacketNode->carry_dst_id = next_carrier->id; //next packet carrier's id
	pPacketNode->packet_transmission_count++;

    /* set up pointers to the carrier vehicle in both the packet queue node and the global packet queue node corresponding to pPacketNode */
    pPacketNode->carrier_vnode = next_carrier; //pointer to the next packet carrier vehicle
    pPacketNode->global_packet->carrier_vnode = next_carrier; //pointer to the next packet carrier vehicle
    
    Enqueue_With_QueueNodePointer((queue_t*)next_carrier->packet_queue, (queue_node_t*)pPacketNode); //enqueue the packet into next_carrier's packet queue rear

#ifdef  __LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__
    /** enqueue the packet carrier trace entry into packet's carrier trace queue */
    Enqueue_CarrierTraceEntry(param, current_time, pPacketNode, VANET_NODE_VEHICLE, (void*)next_carrier);
#endif

    /** update target point information */
    if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD) //if-1.5
    {
      if((next_carrier->latest_packet_ptr == NULL) || (next_carrier->latest_packet_seq <= pPacketNode->seq)) //if-1.5.1
        /* (i) "vehicle->latest_packet_ptr == NULL" means that vehicle has no packet, so updates its latest packet information
           (ii) "== case" is that next_carrier has previously carried the packet and passed it to another vehicle. Now next_carrier receives the packet again
        */
        //if(next_carrier->latest_packet_seq <= pPacketNode->seq) //= allows next_carrier to set the actual packet with the same seq to its latest packet information
      {
	next_carrier->target_point_id = pPacketNode->target_point_id; //update a target point with the new one
	next_carrier->latest_packet_seq = pPacketNode->seq;
	next_carrier->latest_packet_receive_time = current_time;
	next_carrier->latest_packet_ptr = pPacketNode;
      } //end of if-1.5.1
    } //end of if-1.5
  } //end of for-1

  /** reset the latest packet information if stationary node has no packet to hold with it */
  /*@ Note: for multiple destination vehicles, we must maintain the latest packet information per destination vehicle */
  if(stationary_node->packet_queue.size == 0)
  {
    stationary_node->latest_packet_seq = 0; //reset the latest packet seq number
    stationary_node->latest_packet_receive_time = 0;
    stationary_node->latest_packet_ptr = NULL;
    stationary_node->target_point_id = 0; //reset the target point id
  }
}

void VADD_Forward_Packet_To_Stationary_Node(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *Gr, int Gr_size, intersection_area_type_t intersection_area_type, int head_intersection_id, packet_delivery_statistics_t *packet_delivery_stat)
{ //vehicle forwards its packet(s) to the stationary node at the heading intersection
  packet_queue_node_t *pPacketNode = NULL; //pointer to a packet node
  int size = 0; //size of packet queue
  int i = 0; //index for for-loop
  double lifetime = 0; //packet lifetime
  int intersection_id = 0; //intersection id
  struct_graph_node *intersection_gnode = NULL; //pointer to a graph node corresponding to the intersection
  stationary_node_queue_node_t *stationary_node; //stationary node queue node
  int src_id = 0; //source intersection id
  int dst_id = 0; //destination intersection id
  double EDD = 0; //Expected Delivery Delay
  double EDD_SD = 0; // Delivery Delay Standard Deviation  
  char packet_src_vertex[NAME_SIZE]; //packet's source vertex name
  char packet_dst_vertex[NAME_SIZE]; //packet's destination vertex name
  struct_vehicle_t *dst_vehicle = NULL; //destination vehicle
  boolean dst_vehicle_flag = FALSE; //flag to indicate whether the destination vehicle is within the communication range of this stationary node

  /** check whether intersection_type is INTERSECTION_AREA_HEAD_NODE or not; Only for the head node, vehicle forwards its packets to the stationary node at the head node. */
  if(intersection_area_type == INTERSECTION_AREA_HEAD_NODE || intersection_area_type == INTERSECTION_AREA_BOTH_NODES)
    intersection_id = head_intersection_id;
  else
    return;

  /** find the stationary node in the graph node corresponding to intersection_id */
  if(intersection_id > Gr_size)
  {
    printf("VADD_Forward_Packet_To_Stationary_Node(): intersection_id(%d) > Gr_size(%d)\n", intersection_id, Gr_size);
    exit(1);
  }

  intersection_gnode = &(Gr[intersection_id-1]); //get the graph node corresponding to intersection_id

  stationary_node = intersection_gnode->ptr_stationary_node; //get the stationary node placed at intersection_gnode

  /** forward vehicle's packets to next_carrier */
  size = vehicle->packet_queue->size;
  if(size == 0)
    return;

  /**@ for debugging */
  //if(current_time > 7468)
  //{
  //  printf("VADD_Forward_Packet_To_Stationary_Node(): time=%.2f, debugging\n", current_time);    
  //}
  /*******************/

  for(i = 0; i < size; i++) //for-1
  {
    pPacketNode = (packet_queue_node_t*) Dequeue((queue_t*)vehicle->packet_queue); //dequeue a packet placed at vehicle's packet queue front

    /**@ for debugging */
    //if(current_time > 7200 && pPacketNode->id == 102)
    //{
    //  printf("VADD_Forward_Packet_To_Stationary_Node(): time=%.2f, debugging\n", current_time);    
    //}
    /*******************/

    /* compute the packet's lifetime to check whether the packet expires */
    lifetime = current_time - pPacketNode->generation_time; //compute the packet's lifetime

    /** check whether packet's lifetime is greater than the predefined packet TTL */
    if(lifetime > pPacketNode->ttl) //if-1.1
    //if(lifetime > param->communication_packet_ttl) //if-1.1
    {
      VADD_Discard_Expired_Packet(param, current_time, pPacketNode, VANET_NODE_SNODE, (void*)stationary_node, packet_delivery_stat,pPacketNode->ttl); //discard the expired packet by destroying the memory of the packet while the correponding global packet queue node is dequeued and destroyed 

      continue;
    } //end of if-1.1

    /* set up packet's fields */
    pPacketNode->last_receive_time = current_time; //last packet receive time
    pPacketNode->carry_src_id = vehicle->id; //current packet carrier's id
    pPacketNode->carry_dst_id = stationary_node->intersection_id; //next packet carrier's id is stationary node's intersection id, indicating that the next receiver is a stationary node
	pPacketNode->packet_transmission_count++;

    /* set up pointers to the carrier vehicle in both the packet queue node and the global packet queue node corresponding to pPacketNode */
    pPacketNode->carrier_vnode = NULL; //pointer to the next packet carrier vehicle
    pPacketNode->global_packet->carrier_vnode = NULL; //pointer to the next packet carrier vehicle
    /*@ Observe what impact happens on these NULL assignment to carrier_vnode! @*/

    /* update the packet's current packet position with the next packet position */
    pPacketNode->packet_trajectory.current_packet_position = pPacketNode->packet_trajectory.current_packet_position->next;
    pPacketNode->packet_trajectory.current_packet_intersection_id = pPacketNode->packet_trajectory.current_packet_position->intersection_id;
    pPacketNode->packet_trajectory.order++;

    /* update the packet's current-hop vehicle trajectory queue node under the reverse traversal mode for the dynamic forwardingg */
    if(pPacketNode->reverse_traversal_mode_flag == TRUE)
    {
        Update_CurrentHop_VehicleTrajectoryQueueNode_In_PacketReverseTraversal(pPacketNode);
        //update the current-hop vehicle trajectory queue node in the packet reverse traversal
    }
         
    /* check whether the packet's current packet position is the same as the stationary intersection */
    if(pPacketNode->packet_trajectory.current_packet_intersection_id != stationary_node->intersection_id)
    {
      printf("VADD_Forward_Packet_To_Stationary_Node(): Error: at time=%.2f, vehicle's id=%d, pPacketNode->seq(%d): current_packet_intersection_id(%d) is different from stationary_node_intersection_id(%d)\n", (float)current_time, vehicle->id, pPacketNode->seq, pPacketNode->packet_trajectory.current_packet_intersection_id, stationary_node->intersection_id);

      /* for debugging, show the carrier trace in packet's carrier trace queue */
      Show_CarrierTraceQueue(pPacketNode);

      exit(1);
    }

    Enqueue_With_QueueNodePointer((queue_t*)&(stationary_node->packet_queue), (queue_node_t*)pPacketNode); //enqueue the packet into next_carrier's packet queue rear

#if defined(__LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__) || defined(__LOG_LEVEL_VANET_PACKET_CARRIER_TRACE_FOR_STATIONARY_NODE__)
    /** enqueue the packet carrier trace entry into packet's carrier trace queue */
    Enqueue_CarrierTraceEntry(param, current_time, pPacketNode, VANET_NODE_SNODE, (void*)stationary_node);
#endif

#if defined(__LOG_LEVEL_VANET_PACKET_CARRIER_TRACE_FOR_STATIONARY_NODE__)
    /** log the packet carrier trace for the stationary node */
    if((param->packet_delay_measurement_flag == TRUE) && (param->packet_delay_measurement_target_point == stationary_node->intersection_id))
    {
      log_vanet_packet_carrier_trace(VANET_LOG_PACKET_DESTINATION_VEHICLE_ARRIVAL, current_time, pPacketNode);        
    }
#endif

    /** update target point information */
    if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD) //if-1
    {
      if((stationary_node->latest_packet_ptr == NULL) || (stationary_node->latest_packet_seq <= pPacketNode->seq)) 
	/* (i) "vehicle->latest_packet_ptr == NULL" means that vehicle has no packet, so updates its latest packet information
	   (ii) "== case" is that next_carrier has previously carried the packet and passed it to another vehicle. Now next_carrier receives the packet again
	 */
      {
	stationary_node->target_point_id = pPacketNode->target_point_id; //update a target point with the new one
	stationary_node->latest_packet_seq = pPacketNode->seq;
	stationary_node->latest_packet_receive_time = current_time;
	stationary_node->latest_packet_ptr = pPacketNode;
      }
    } //end of if-1

/*     /\* update the packet's trajectory for the fully dynamic forwarding; on the other hand, for the other forwarding *\/ */
/*     if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_FULLY_DYNAMIC_FORWARDING) //if-2 */
/*     { //this update lets the current packet position be the first one between two packet trajectory queue nodes; this allows the function VADD_Forward_Packet_From_Stationary_Node_To_Next_Carrier() to check whether or not this packet can be carried by a vehicle using the function VADD_Is_Vehicle_Moving_On_Packet_Trajectory(). Note that without this update, there happens an error in VADD_Is_Vehicle_Moving_On_Packet_Trajectory(). */
/*         /\* for order = 1, update the packet trajectory so that order = 0 in the packet trajectory of length 2 *\/ */
/*         if(pPacketNode->packet_trajectory.order == 1) */
/*         { */
/*             itoa(stationary_node->intersection_id, packet_src_vertex); */
/*             itoa(pPacketNode->target_point_id, packet_dst_vertex); */
/*             Install_PacketTrajectory_Into_Packet(param, current_time, param->vanet_table.Gr, param->vanet_table.Gr_size, packet_src_vertex, packet_dst_vertex, pPacketNode); */
/*         } */
/*     } //end of if-2 */

  } //end of for-1

  /** reset the latest packet information if vehicle has no packet to carry with it */
  if(vehicle->packet_queue->size == 0)
  {
    vehicle->latest_packet_seq = 0; //reset the latest packet seq number
    vehicle->latest_packet_receive_time = 0;
    vehicle->latest_packet_ptr = NULL;
  }

  /** check whether the destination vehicle is within the communication range of this stationary node */
  /** check whether are packets in the stationary node */
  size = stationary_node->packet_queue.size;
  if(size == 0)
    return;

  dst_vehicle_flag = is_vehicle_within_communication_range_of_point(param, param->vanet_table.dst_vnode, &(stationary_node->gnode->coordinate));
  //dst_vehicle_flag = is_vehicle_within_communication_range_of_point(param, stationary_node->latest_packet_ptr->dst_vnode, &(stationary_node->gnode->coordinate));
  if(dst_vehicle_flag)
  { //since the destination vehicle is within the communication range of the stationary node, the stationary node forwards its packets to the destination vehicle
      //printf("VADD_Forward_Packet_To_Stationary_Node(): Implement the packet forwarding!\n");
      VADD_Forward_Packet_From_Stationary_Node_To_Next_Carrier(param, current_time, stationary_node->intersection_id, stationary_node->latest_packet_ptr->dst_vnode, packet_delivery_stat);
  }
}

void VADD_Forward_Packet_From_Stationary_Node_To_Destination_Vehicle(parameter_t *param, double current_time, int intersection_id, struct_vehicle_t *destination_vehicle, packet_delivery_statistics_t *packet_delivery_stat)
{ //The stationary node corresponding to intersection_id forwards its packet(s) to the destination vehicle pointed by destination_vehicle and the log for the packet(s) is written into the packet logging file.
  packet_queue_node_t *pPacketNode = NULL; //pointer to a packet node
  int size = 0; //size of packet queue
  int i = 0; //index for for-loop
  double lifetime = 0; //packet lifetime
  stationary_node_queue_node_t *stationary_node = NULL; //pointer to the stationary node corresponding to intersection_id

  /** get the stationary node for intersection_id */
  if(intersection_id > param->vanet_table.Gr_size)
  {
    printf("VADD_Forward_Packet_From_Stationary_Node_To_Destination_Vehicle(): Error: intersection_id(%d) > param->vanet_table.Gr_size(%d)\n", intersection_id, param->vanet_table.Gr_size);
    exit(1);
  }
  stationary_node = param->vanet_table.Gr[intersection_id-1].ptr_stationary_node;

  /** check whether are packets in the stationary node */
  size = stationary_node->packet_queue.size;
  if(size == 0)
    return;

  for(i = 0; i < size; i++) //for-1
  {
    pPacketNode = (packet_queue_node_t*) Dequeue((queue_t*)&(stationary_node->packet_queue)); //dequeue a packet placed at AP's packet queue front

    /* compute the packet's lifetime to check whether the packet expires */
    lifetime = current_time - pPacketNode->generation_time; //compute the packet's lifetime

    /** check whether packet's lifetime is greater than the predefined packet TTL */
    if(lifetime > pPacketNode->ttl) //if-1.1
    //if(lifetime > param->communication_packet_ttl) //if-1.1
    {
      VADD_Discard_Expired_Packet(param, current_time, pPacketNode, VANET_NODE_SNODE, (void*)stationary_node, packet_delivery_stat,pPacketNode->ttl); //discard the expired packet by destroying the memory of the packet while the correponding global packet queue node is dequeued and destroyed 

      continue;
    } //end of if-1.1

    /* set up packet's fields */
    pPacketNode->carry_src_id = stationary_node->intersection_id; //update the packet's carry_src_vid with stationary_node->intersection_id
    pPacketNode->carry_dst_id = destination_vehicle->id; //update the packet's carry_dst_vid with destination_vehicle's id
	pPacketNode->packet_transmission_count++;
  
    /* set up pointers to the carrier vehicle in both the packet queue node and the global packet queue node corresponding to pPacketNode */
    pPacketNode->carrier_vnode = destination_vehicle; //pointer to the destination vehicle
    pPacketNode->global_packet->carrier_vnode = destination_vehicle; //pointer to the destination vehicle

    pPacketNode->last_receive_time = current_time; //last packet receive time
    pPacketNode->destination_arrival_time = current_time; //packet's destination arrival time means the arrival time to the destination vehicle

//#ifdef __LOG_LEVEL_VANET_PACKET_DESTINATION_VEHICLE_ARRIVAL__
    log_vanet(VANET_LOG_PACKET_DESTINATION_VEHICLE_ARRIVAL, current_time, pPacketNode, packet_delivery_stat); //log the data forwarding to the destination vehicle on VANET into VANET logging file
//#endif

#if defined(__LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__) || defined(__LOG_LEVEL_VANET_PACKET_CARRIER_TRACE_FOR_STATIONARY_NODE__)
    /** enqueue the packet carrier trace entry into packet's carrier trace queue */
    Enqueue_CarrierTraceEntry(param, current_time, pPacketNode, VANET_NODE_VEHICLE, (void*)destination_vehicle);

    /** log packet carrier trace for this packet into carrier_trace_file */
    log_vanet_packet_carrier_trace(VANET_LOG_PACKET_DESTINATION_VEHICLE_ARRIVAL, current_time, pPacketNode);
#endif

    /* dequeue global packet queue node, destroy global packet queue node and then free the memory of global packet queue node */
    DestroyGlobalPacketQueueNode(pPacketNode->global_packet);   

    /* destroy packet queue node along with vehicle trajectory queue and carrier trace queue, and then free the memory of packet queue node */
    DestroyPacketQueueNode(pPacketNode);
  } //end of for-1

  /** reset the latest packet information if stationary node has no packet to hold with it */
  /*@ Note: for multiple destination vehicles, we must maintain the latest packet information per destination vehicle */
  if(stationary_node->packet_queue.size == 0)
  {
    stationary_node->latest_packet_seq = 0; //reset the latest packet seq number
    stationary_node->latest_packet_receive_time = 0;
    stationary_node->latest_packet_ptr = NULL;
    stationary_node->target_point_id = 0; //reset the target point id
  }
}

void VADD_Discard_Expired_Packet(parameter_t *param, double current_time, packet_queue_node_t *pPacketNode, vanet_node_type_t node_type, void *vanet_node, packet_delivery_statistics_t *packet_delivery_stat,int lineNumber)
{ //discard the expired packet by destroying the memory of the packet while the correponding global packet queue node is dequeued and destroyed 
  
  //printf("Packet Discarded %d!\n",lineNumber);
  /* set the last receive time to current time in order to count this discarded packet into the accumulated delay with the packet TTL */
  pPacketNode->last_receive_time = current_time; //last packet receive time  

//#ifdef __LOG_LEVEL_VANET_PACKET_DROP__
  log_vanet(VANET_LOG_PACKET_DROP, current_time, pPacketNode, packet_delivery_stat);
//log the packet drop due to TTL expiration into VANET logging file
//#endif

#if defined( __LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__)
//#if defined( __LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__) || defined(__LOG_LEVEL_VANET_PACKET_CARRIER_TRACE_FOR_STATIONARY_NODE__)
  /** enqueue the packet carrier trace entry into packet's carrier trace queue */
  Enqueue_CarrierTraceEntry(param, current_time, pPacketNode, node_type, vanet_node);

  /** log packet carrier trace for this dropped packet into carrier_trace_file */
  log_vanet_packet_carrier_trace(VANET_LOG_PACKET_DROP, current_time, pPacketNode);
#endif

  /* dequeue global packet queue node, destroy global packet queue node and then free the memory of global packet queue node */
  DestroyGlobalPacketQueueNode(pPacketNode->global_packet);

  /* destroy packet queue node along with vehicle trajectory queue and carrier trace queue, and then free the memory of packet queue node */
  DestroyPacketQueueNode(pPacketNode);   
}

double VADD_Update_Vehicle_EDD(double update_time, parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table)
{ //compute vehicle's EDD using the vehicle's offset in directional edge along with real graph G and AP table 
  double per_vehicle_EDD = 0; //EDD computed by Per-vehicle model
  double per_intersection_EDD = 0; //EDD computed by Per-intersection model
  double alpha = 0 ;//alpha is a function of vehicular traffic density for a hybrid EDD

  switch(param->vehicle_vanet_edd_model)
  {
  case VANET_EDD_PER_VEHICLE_MODEL:

    /** compute the vehicle's path EDD according to the vehicle's trajectory */
    vehicle->EDD = VADD_Compute_TBD_Based_EDD(param, vehicle, G, G_size, ap_table);

    break;

  case VANET_EDD_PER_INTERSECTION_MODEL:

    /** compute vehicle's EDD from directional edge's EDD and the offset of directional edge delay */
    vehicle->EDD = VADD_Compute_VADD_Based_EDD(param, vehicle, G, G_size, ap_table);

    break;

  case VANET_EDD_HYBRID_MODEL: 

    /** EDD is computed by a weighted linear combination of Per-vehicle EDD and Per-intersection EDD; note that the weights of two EDDs for the linear combination are adjusted by the density of vehicular traffic in the given road network */

    /* compute the Per-vehicle EDD */
    per_vehicle_EDD = VADD_Compute_TBD_Based_EDD(param, vehicle, G, G_size, ap_table);

    /* compute the Per-intersection EDD */
    per_intersection_EDD = VADD_Compute_VADD_Based_EDD(param, vehicle, G, G_size, ap_table);

    /* combine per_vehicle_EDD and per_intersection_EDD into a linear combination */
    vehicle->EDD = MIN(per_vehicle_EDD, per_intersection_EDD); 
    //vehicle->EDD = alpha*per_vehicle_EDD + (1-alpha)*per_intersection_EDD; //alpha is a function of vehicular traffic density

#ifdef __DEBUG_LEVEL_EDD_COMPUTATION__
    if(per_vehicle_EDD < per_intersection_EDD)
      printf("per_vehicle_EDD(%f) is less than per_intersection_EDD(%f)\n", per_vehicle_EDD, per_intersection_EDD);
#endif

    break;

  default:
    printf("VADD_Update_Vehicle_EDD(): Error: param->vehicle_vanet_edd_model(%d) is not supported yet!\n", param->vehicle_vanet_edd_model);
    exit(1);
  }

  /* update vehicle's EDD update time */
  vehicle->EDD_update_time = update_time;

  return vehicle->EDD;
}

void VADD_Update_Vehicle_EDD_And_EDD_SD(double update_time, parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table)
{ //compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge along with real graph G and AP table 
	double per_vehicle_EDD = 0; //EDD computed by Per-vehicle model
	double per_intersection_EDD = 0; //EDD computed by Per-intersection model
	double alpha = 0 ;//alpha is a function of vehicular traffic density for a hybrid EDD
	double per_vehicle_EDD_SD = 0; //EDD_SD computed by Per-vehicle model
	double per_intersection_EDD_SD = 0; //EDD_SD computed by Per-intersection model

	switch(param->vehicle_vanet_edd_model)
	{
		case VANET_EDD_PER_VEHICLE_MODEL:

			/** compute the vehicle's path EDD and EDD_SD according to the vehicle's trajectory */
			VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(param, vehicle, G, G_size, ap_table, &(vehicle->EDD), &(vehicle->EDD_SD));

			break;

		case VANET_EDD_PER_INTERSECTION_MODEL:

			/** compute vehicle's EDD and EDD_SD from directional edge's EDD/EDD_SD and the offset of directional edge */
			VADD_Compute_EDD_And_EDD_SD_Based_On_VADD(param, vehicle, G, G_size, ap_table, &(vehicle->EDD), &(vehicle->EDD_SD));

			break;

		case VANET_EDD_HYBRID_MODEL: 

			/** EDD is computed by a weighted linear combination of Per-vehicle EDD and Per-intersection EDD; note that the weights of two EDDs for the linear combination are adjusted by the density of vehicular traffic in the given road network */

			/* compute the Per-vehicle EDD and EDD_SD */
			VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(param, vehicle, G, G_size, ap_table, &(per_vehicle_EDD), &(per_vehicle_EDD_SD));

			/* compute the Per-intersection EDD and EDD_SD */
			VADD_Compute_EDD_And_EDD_SD_Based_On_VADD(param, vehicle, G, G_size, ap_table, &(per_intersection_EDD), &(per_intersection_EDD_SD));

			/* combine per_vehicle_EDD and per_intersection_EDD into a linear combination */
			vehicle->EDD = MIN(per_vehicle_EDD, per_intersection_EDD); 
			//vehicle->EDD = alpha*per_vehicle_EDD + (1-alpha)*per_intersection_EDD; //alpha is a function of vehicular traffic density

			/* combine per_vehicle_EDD_SD and per_intersection_EDD_SD into a linear combination */
			vehicle->EDD_SD = MIN(per_vehicle_EDD_SD, per_intersection_EDD_SD); 
			//vehicle->EDD = alpha*per_vehicle_EDD_SD + (1-alpha)*per_intersection_EDD_SD; //alpha is a function of vehicular traffic density

#ifdef __DEBUG_LEVEL_EDD_COMPUTATION__
			if(per_vehicle_EDD < per_intersection_EDD)
				printf("%s:%d: per_vehicle_EDD(%f) is less than per_intersection_EDD(%f)\n", 
						__FUNCTION__, __line__,
						per_vehicle_EDD, per_intersection_EDD);
#endif

			break;

		default:
			printf("%s:%d: Error: param->vehicle_vanet_edd_model(%d) is not supported yet!\n", 
					__FUNCTION__, __LINE__,
					param->vehicle_vanet_edd_model);
			exit(1);
	}

	/* update vehicle's EDD update time */
	vehicle->EDD_update_time = update_time;
}

void VADD_Update_Vehicle_EDD_And_EDD_SD_For_V2V_Data_Delivery(double update_time, parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, int source_intersection_id)
{ //compute vehicle's EDD and EDD_SD from source intersection to the target point for the road network graph G for V2V data delivery, using the vehicle's offset in directional edge in the graph G 
	double per_vehicle_EDD = 0; //EDD computed by Per-vehicle model
	double per_intersection_EDD = 0; //EDD computed by Per-intersection model
	double alpha = 0 ;//alpha is a function of vehicular traffic density for a hybrid EDD
	double per_vehicle_EDD_SD = 0; //EDD_SD computed by Per-vehicle model
	double per_intersection_EDD_SD = 0; //EDD_SD computed by Per-intersection model

	switch(param->vehicle_vanet_edd_model)
	{
		case VANET_EDD_PER_VEHICLE_MODEL:

			/** compute the vehicle's path EDD and EDD_SD according to the vehicle's trajectory */
			VADD_Compute_EDD_And_EDD_SD_Based_On_TBD_For_V2V_Data_Delivery(param, vehicle, G, G_size, source_intersection_id, &(vehicle->EDD_for_V2V), &(vehicle->EDD_SD_for_V2V));

			break;

		case VANET_EDD_PER_INTERSECTION_MODEL:

			/** compute vehicle's EDD and EDD_SD from directional edge's EDD/EDD_SD and the offset of directional edge */
			VADD_Compute_EDD_And_EDD_SD_Based_On_VADD_For_V2V_Data_Delivery(param, vehicle, G, G_size, source_intersection_id, &(vehicle->EDD_for_V2V), &(vehicle->EDD_SD_for_V2V));

			break;

		case VANET_EDD_HYBRID_MODEL: 

			/** EDD is computed by a weighted linear combination of Per-vehicle EDD and Per-intersection EDD; note that the weights of two EDDs for the linear combination are adjusted by the density of vehicular traffic in the given road network */

			/* compute the Per-vehicle EDD and EDD_SD */
			VADD_Compute_EDD_And_EDD_SD_Based_On_TBD_For_V2V_Data_Delivery(param, vehicle, G, G_size, source_intersection_id, &(per_vehicle_EDD), &(per_vehicle_EDD_SD));

			/* compute the Per-intersection EDD and EDD_SD */
			VADD_Compute_EDD_And_EDD_SD_Based_On_VADD_For_V2V_Data_Delivery(param, vehicle, G, G_size, source_intersection_id, &(per_intersection_EDD), &(per_intersection_EDD_SD));

			/* combine per_vehicle_EDD and per_intersection_EDD into a linear combination */
			vehicle->EDD_for_V2V = MIN(per_vehicle_EDD, per_intersection_EDD); 
			//vehicle->EDD = alpha*per_vehicle_EDD + (1-alpha)*per_intersection_EDD; //alpha is a function of vehicular traffic density

			/* combine per_vehicle_EDD_SD and per_intersection_EDD_SD into a linear combination */
			vehicle->EDD_SD_for_V2V = MIN(per_vehicle_EDD_SD, per_intersection_EDD_SD); 
			//vehicle->EDD = alpha*per_vehicle_EDD_SD + (1-alpha)*per_intersection_EDD_SD; //alpha is a function of vehicular traffic density

#ifdef __DEBUG_LEVEL_EDD_COMPUTATION__
			if(per_vehicle_EDD < per_intersection_EDD)
				printf("%s:%d: per_vehicle_EDD(%f) is less than per_intersection_EDD(%f)\n", 
						__FUNCTION__, __line__,
						per_vehicle_EDD, per_intersection_EDD);
#endif

			break;

		default:
			printf("%s:%d: Error: param->vehicle_vanet_edd_model(%d) is not supported yet!\n", 
					__FUNCTION__, __LINE__,
					param->vehicle_vanet_edd_model);
			exit(1);
	}

	/* update vehicle's EDD update time */
	vehicle->EDD_update_time = update_time;
}
double VADD_Compute_TBD_Based_EDD(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table)
{ //compute the TBD-Model-Based EDD (i.e., Per-vehicle EDD) using vehicle's trajectory in the vehicle's current position on the current directional edge
  double path_EDD = 0; //vehicle's path EDD
  double edge_EDD = 0; //vehicle's edge EDD
  int i, j; //indices for for-loops
  struct_path_node *pPathNode = NULL; //pointer to path node
  char *path_edge_tail = NULL; //pointer to the tail vertex of the edge on the path
  char *path_edge_head = NULL; //pointer to the head vertex of the edge on the path
  struct_graph_node *pTailNode = NULL; //pointer to the tail graph node of an edge
  struct_graph_node *pHeadNode = NULL; //pointer to the head graph node of an edge
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge node of <pTailNode,pHeadNode>
  double P_ac = 0; //accumulative carry probability along the trajectory
  double P_ic = 0; //carry probability at each intersection
  double P_branch = 0; //branch probability that the vehicle will branch to a neighboring edge after the arrival at its destination
  double M = 0; //carry delay in an edge
  int node_id = 0; //node id
  int remaining_hop_count = 0; //remaining hop count from the vehicle's current position to its destination
  double offset_in_directional_edge = 0; //vehicle's offset in the directional edge where the vehicle is moving
  double directional_edge_delay_for_remaining_edge_length = 0; //directional edge delay for vehicle's remaining edge length
  double remaining_edge_length = 0; //remaining edge length for the vehicle's current directional edge
  double directional_edge_delay_for_remaining_edge = 0; //directional edge delay for the remaining edge where the vehicle is moving
  double directional_edge_delay_for_offset = 0; //directional edge delay for the offset where the vehicle has moved
  double current_vehicle_EDD = 0; //the vehicle's EDD at a future position on its trajectory 
  boolean flag = FALSE; //flag to indicate that the vehicle's trajectory meets one of access points 
  boolean flag_for_destination = FALSE; //flag for the EDD processing at the destination for the case where the vehicle cannot forward its packets to the next carrier, so must carry them with itself according to its new trajectory
  double accumulated_carry_delay = 0; //the accumulated carry delay for the carry length from the starting position to the current intersection on the trajectory that the original packet carrier has carried the packets without forwarding them.

  pPathNode = vehicle->path_ptr;
  P_ac = 1; //initialize P_ac
  remaining_hop_count = vehicle->path_hop_count - vehicle->path_current_hop; //compute the remaining hop count from the vehicle's current position to its destination

  /**@ for debugging */
  //if(vehicle->id == 1 && vehicle->state_time >= 3599)
  //{
  //  printf("VADD_Compute_TBD_Based_EDD(): at time=%.1f, vehicle(id=%d) is traced\n", (float)vehicle->state_time, vehicle->id);
  //}

  if(remaining_hop_count == 0)
  {
    printf("VADD_Compute_TBD_Based_EDD(): remaining_hop_count is zero!\n");
    fgetc(stdin);
  }
  /*******************/

  for(i = 0; i <= remaining_hop_count; i++) //for-1
  {
    if(i < remaining_hop_count) //if-1
    {
      path_edge_tail = pPathNode->vertex;
      path_edge_head = pPathNode->next->vertex;

      /* obtain the pointer to the direaction edge of <tail_node,head_node> */
      pEdgeNode = FastLookupDirectionalEdgeQueue(G, path_edge_tail, path_edge_head);
      if(pEdgeNode == NULL) //if-1.1
      {
	printf("VADD_Compute_TBD_Based_EDD(): pEdgeNode for <%s,%s> is NULL\n", path_edge_tail, path_edge_head);
	exit(1);
      } //end of if-1.1

      /* set the pointer to tail node of the outgoing edge at the intersection corresponding to intermediate intersection, pointed by pEdgeNode->tail_gnode->vertex */
      pTailNode = pEdgeNode->tail_gnode;

      /* check the computation type of trajectory-based EDD */
      if(i != 0 && param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH) //if-1.2: Note that the condition (i != 0) lets the vehicle moving from one of APs have positive EDD
      //if(param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH) //if-1.2
      //if edd computation type is partial-path and the tail node is one of APs, then finish the EDD computation here
      {
        /* check whether pTailNode->vertex is one of APs; if so, escape from this for-loop since the EDD at access point must be zero; that is, we consider the EDD of the rest of the path just after the AP to be zero. */

        flag = IsVertexInTrafficTable(ap_table, pTailNode->vertex);
        if(flag)
	  break;
      } //end of if-1.2

      /* obtain the edge EDD and compute the vehicle movement time for the remaining edge */
      if(i == 0) //if-1.3
      //if this is the first hop
      {
	/* obtain the vehicle's edge EDD */
	edge_EDD = pEdgeNode->head_gnode->EDD;

	/* obtain the carry delay by vehicle movement in the case where a vehicle cannot forward its packet to appropriate next carrier */
	offset_in_directional_edge = vehicle->current_pos_in_digraph.offset;
	remaining_edge_length = vehicle->edge_length - offset_in_directional_edge;
	M = remaining_edge_length/vehicle->speed; //movement time for the remaining edge length

        /** compute the estimated directional edge delay for the remaining edge length */
        //directional_edge_delay_for_remaining_edge = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, remaining_edge_length); 
        
	/* Note that the above gives the very similar performance as the below (i.e., the same packet loss, but a little longer delay with 1 second), but logically the below makes sense since the vehicle should carry packets in order to compute path_EDD related to the vehicle's path */
	//directional_edge_delay_for_remaining_edge = M; //for version v1.3.9

	/** compute the estimated directional edge delay for the current offset on the directional edge */
       	directional_edge_delay_for_offset = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, offset_in_directional_edge);
      } //end of if-1.3
      else //else-1.4
      //otherwise
      {
	/* obtain the vehicle's edge EDD */
	edge_EDD = pEdgeNode->head_gnode->EDD;

	/* obtain the carry delay by vehicle movement in the case where a vehicle cannot forward its packet to appropriate next carrier */
	M = pEdgeNode->weight/vehicle->speed;
      } //end of else-1.4
    } //end of if-1
    else //else-2
    //handling of EDD at the destination node where path_edge_head->vertex is NULL
    {
      /* obtain the pointer to the destination graph node */
      path_edge_tail = path_edge_head;
      node_id = atoi(path_edge_tail);
      pTailNode = &(G[node_id-1]);

      /* check the computation type of trajectory-based EDD */
      if(param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH) //if-2.1
      //if edd computation type is partial-path and the tail node is one of APs, then finish the EDD computation here; that is, we consider the EDD of the rest of the path just after the AP to be zero
      {
        /* check whether pTailNode->vertex is one of APs; if so, escape from this for-loop since the EDD at access point must be zero */

        flag = IsVertexInTrafficTable(ap_table, pTailNode->vertex);

        if(flag)
	  break;
      } //end of if-2.1

      /* set the vehicle's edge EDD to INFINITE since the vehicle has arrived at its destination, so it has no next edge on its trajectory */
      edge_EDD = INF;

      /* set the carry delay by vehicle movement to zero since the vehicle has arrived at its destination, so it does not move any more according to its trajectory */
      M = 0;

      /* set the flag for the EDD processing at the destination for the case where the vehicle cannot forward its packets to the next carrier, so must carry them with itself according to its new trajectory */
      flag_for_destination = TRUE;
      
    } //end of else-2

    if(i == 0) //if this is the first hop
    { //@Note: for the first hop, we consider two cases: (i) the case of forwarding the packet and (ii) the case of carrying the packet.
      /* update accumulated_carry_delay with M */
      accumulated_carry_delay = M;

      path_EDD = M; //modified on 11/23/08, carry delay for the remaining edge length; 
      //path_EDD = directional_edge_delay_for_remaining_edge; //@commented on 11/23/08
      //path_EDD = edge_EDD - directional_edge_delay_for_remaining_edge;

      current_vehicle_EDD = edge_EDD - directional_edge_delay_for_offset;
    }
    else //else-1
    {
      /* update accumulated_carry_delay with M */
      accumulated_carry_delay += M;

      current_vehicle_EDD = edge_EDD;

      P_ic = 1; //initialize P_ic

      /* set the pointer to head node of the outgoing edge at the intersection corresponding to pTailNode->vertex */
      pHeadNode = pTailNode;
      for(j = 0; j < pTailNode->weight; j++) //for-2
      {
	pHeadNode = pHeadNode->next;

        /** NOTE: is it correct that we use edge_EDD instead of using the vehicle's actual EDD at the intersection corresponding to pTailNode->vertex */
	//check whether the branch's EDD is greater than the EDD of the branch on the trajectory
	if(pHeadNode->EDD > current_vehicle_EDD)
	  continue; //in this case, we don't use this branch for packet forwarding

	//check whether the branch's EDD is greater than the path_EDD according to the trajectory
	//if(pHeadNode->EDD > path_EDD)
	//  continue; //in this case, we don't use this branch for packet forwarding

	//add the portion of EDD for the forwarding into this outgoing branch to path_EDD
	path_EDD += P_ac * pHeadNode->P_prime_pure * pHeadNode->EDD; //use pure forwarding probability for TBD
	//path_EDD += P_ac * pHeadNode->P_pure * pHeadNode->EDD; //use pure forwarding probability
	//path_EDD += P_ac * pHeadNode->P * pHeadNode->EDD; //use forwarding probability
	//path_EDD += P_ac * pHeadNode->CP * pHeadNode->EDD;

	//update the carry probability at the intersection
	P_ic -= pHeadNode->P_prime_pure; //use pure forwarding probability for TBD
	//P_ic -= pHeadNode->P_pure; //use pure forwarding probability
	//P_ic -= pHeadNode->P; //use forwarding probability
	//P_ic *= (1 - pHeadNode->CP); //(1 - pHeadNode->CP) is the probability that the vehicle cannot meet a next carrier moving on the edge <pTailNode->vertex, pHeadNode->vertex>
      } //end of for-2

      /* update the accumulative carry probability P_ac with intersection carry probability P_ic */
      P_ac *= P_ic;

      /* update path_EDD with accumulated_carry_delay */
      path_EDD += P_ac*accumulated_carry_delay; //@modified on 11/23/08; Note that path_EDD takes the movement time for the vehicle's carry length from its starting position to the current edge after updating P_ac

      //path_EDD += P_ac*M; //@modified on 10/5/08; Note that path_EDD takes the movement time for the vehicle's current edge after updating P_ac

      /* process path_EDD at the destination for the case where the vehicle cannot forward its packets to a moving neighboring carrier */
      if(flag_for_destination)
      {
	/* compute the EDD in the case where the vehicle cannot forward its packets at the desination; that is, after its new trajectory is determined, it must carry them with itself */

        P_branch = 1/pTailNode->weight; //branch probability of the vehicle towards the next intersection after its new trajectory is determined

        /* set the pointer to head node of the outgoing edge at the intersection corresponding to pTailNode->vertex */
        pHeadNode = pTailNode;
        for(j = 0; j < pTailNode->weight; j++) //for-2.2.1
        {
	  pHeadNode = pHeadNode->next;

	  //add the portion of EDD for the forwarding into this outgoing branch to path_EDD
	  path_EDD += P_ac * P_branch * pHeadNode->EDD; //use the same branch probability for each directional edge from the tail node
        } //end of for-2.2.1
      }
    } //end of else-1  

    /* let pPathNode point the head node of the next hop */
    pPathNode = pPathNode->next;

    /* check the computation type of trajectory-based EDD */
    if(param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_ONE_HOP_AHEAD && i == 1) //if edd computation type is one-hop-ahead, then finish the EDD computation here
      break;
  } //end of for-1
  
  return path_EDD;
}

double VADD_Compute_VADD_Based_EDD(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table)
{ //compute the VADD-Model-Based EDD (i.e., Per-intersection EDD) in vehicle's current position on the current directional edge
  double vehicle_EDD = 0; //vehicle's EDD at the given position in the current directional edge
  double directional_edge_EDD = 0; //directional edge's EDD
  char *tail_node = vehicle->path_ptr->vertex; //tail node of the directional edge where vehicle is moving
  char *head_node = vehicle->path_ptr->next->vertex; //head node of the directional edge where vehicle is moving
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
  double offset_in_undirectional_edge = 0; //vehicle's offset on the undirectional edge
  double offset_in_directional_edge = 0; //vehicle's offset on the directional edge
  double remaining_edge_length = 0; //remaining edge length for the vehicle
  double directional_edge_delay = 0; //directional edge delay

  /** obtain the pointer to the direaction edge of <tail_node,head_node> */
  pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
  if(pEdgeNode == NULL)
  {
    printf("VADD_Compute_VADD_Based_EDD(): pEdgeNode for <%s,%s> is NULL\n", tail_node, head_node);
    exit(1);
  }

  /** obtain the directional edge's EDD */
  directional_edge_EDD = pEdgeNode->head_gnode->EDD;

  /** compute the offset in directional edge */
  offset_in_undirectional_edge = vehicle->current_pos_in_Gr.offset;
  if(vehicle->move_type == MOVE_FORWARD)
  {
    offset_in_directional_edge = offset_in_undirectional_edge;
  }    
  else if(vehicle->move_type == MOVE_BACKWARD)
  {
    offset_in_directional_edge = vehicle->edge_length - offset_in_undirectional_edge;
  }
  else
  {
    printf("VADD_Compute_VADD_Based_EDD(): vehicle->move_type() is invalid\n", vehicle->move_type);
    exit(1);
  }

  /** compute the estimated directional edge delay for the vehicle's moving distance on the edge */
  directional_edge_delay = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, offset_in_directional_edge);

  /** adjust vehicle's EDD from directional edge's EDD */
  vehicle_EDD = MAX(0, directional_edge_EDD - directional_edge_delay);

  return vehicle_EDD;
}

void VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD)
{ //compute the EDD and EDD_SD based on TBD Model (i.e., Per-vehicle Model) in vehicle's current position on the current directional edge; note that this version is for TBD journal submission
  VADD_Compute_EDD_And_EDD_SD_Based_On_TBD_VERSION_4(param, vehicle, G, G_size, ap_table, vehicle_EDD, vehicle_EDD_SD);
}

void VADD_Compute_EDD_And_EDD_SD_Based_On_TBD_VERSION_1(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD)
{ //compute the EDD and EDD_SD based on TBD Model (i.e., Per-vehicle Model) in vehicle's current position on the current directional edge
  double R = param->communication_range; //communication range
  double path_EDD = 0; //vehicle's path EDD
  double edge_EDD = 0; //vehicle's edge EDD
  double path_EDD_VAR = 0; //vehicle's path EDD_VAR
  double edge_EDD_VAR = 0; //vehicle's edge EDD_VAR
  int i, j; //indices for for-loops
  struct_path_node *pPathNode = NULL; //pointer to path node
  char *path_edge_tail = NULL; //pointer to the tail vertex of the edge on the path
  char *path_edge_head = NULL; //pointer to the head vertex of the edge on the path
  struct_graph_node *pTailNode = NULL; //pointer to the tail graph node of an edge
  struct_graph_node *pHeadNode = NULL; //pointer to the head graph node of an edge
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge node of <pTailNode,pHeadNode>
  double P_ac = 0; //accumulative carry probability along the trajectory
  double P_ic = 0; //carry probability at each intersection
  double P_branch = 0; //branch probability that the vehicle will branch to a neighboring edge after the arrival at its destination
  double M = 0; //carry delay in an edge
  int node_id = 0; //node id
  int remaining_hop_count = 0; //remaining hop count from the vehicle's current position to its destination
  double offset_in_directional_edge = 0; //vehicle's offset in the directional edge where the vehicle is moving
  double directional_edge_delay_for_whole_edge = 0; //directional edge delay for the whole edge length
  //double directional_edge_delay_for_remaining_edge_length = 0; //directional edge delay for vehicle's remaining edge length
  double remaining_edge_length = 0; //remaining edge length for the vehicle's current directional edge
  double directional_edge_delay_for_remaining_edge = 0; //directional edge delay for the remaining edge where the vehicle is moving
  double directional_edge_delay_for_offset = 0; //directional edge delay for the offset where the vehicle has moved
  double current_vehicle_EDD = 0; //the vehicle's EDD at a future position on its trajectory 
  double current_vehicle_EDD_VAR = 0; //the vehicle's EDD_VAR at a future position on its trajectory 
  boolean flag = FALSE; //flag to indicate that the vehicle's trajectory meets one of access points 
  boolean flag_for_destination = FALSE; //flag for the EDD processing at the destination for the case where the vehicle cannot forward its packets to the next carrier, so must carry them with itself according to its new trajectory
  double accumulated_carry_delay = 0; //the accumulated carry delay for the carry length from the starting position to the current intersection on the trajectory that the original packet carrier has carried the packets without forwarding them.
  double accumulated_carry_delay_var = 0; //the accumulated carry delay variance for the carry length from the starting position to the current intersection on the trajectory that the original packet carrier has carried the packets without forwarding them.

  pPathNode = vehicle->path_ptr;
 
  P_ac = 1; //initialize P_ac

  remaining_hop_count = vehicle->path_hop_count - vehicle->path_current_hop; //compute the remaining hop count from the vehicle's current position to its destination

  /**@ for debugging */
  //if(vehicle->id == 1 && vehicle->state_time >= 3599)
  //{
  //  printf("VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(): at time=%.1f, vehicle(id=%d) is traced\n", (float)vehicle->state_time, vehicle->id);
  //}

  if(remaining_hop_count == 0)
  {
    printf("VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(): remaining_hop_count is zero!\n");
    fgetc(stdin);
  }
  /*******************/

  for(i = 0; i <= remaining_hop_count; i++) //for-1
  {
    if(i < remaining_hop_count) //if-1
    {
      path_edge_tail = pPathNode->vertex;
      path_edge_head = pPathNode->next->vertex;

      /* obtain the pointer to the direaction edge of <tail_node,head_node> */
      pEdgeNode = FastLookupDirectionalEdgeQueue(G, path_edge_tail, path_edge_head);
      if(pEdgeNode == NULL) //if-1.1
      {
	printf("VADD_Compute_TBD_Based_EDD(): pEdgeNode for <%s,%s> is NULL\n", path_edge_tail, path_edge_head);
	exit(1);
      } //end of if-1.1

      /* set the pointer to tail node of the outgoing edge at the intersection corresponding to intermediate intersection, pointed by pEdgeNode->tail_gnode->vertex */
      pTailNode = pEdgeNode->tail_gnode;

      /* check the computation type of trajectory-based EDD */
      if(i != 0 && param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH) //if-1.2: Note that the condition (i != 0) lets the vehicle moving from one of APs have positive EDD
      //if(param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH) //if-1.2
      //if edd computation type is partial-path and the tail node is one of APs, then finish the EDD computation here
      {
        /* check whether pTailNode->vertex is one of APs; if so, escape from this for-loop since the EDD at access point must be zero; that is, we consider the EDD of the rest of the path just after the AP to be zero. */

        flag = IsVertexInTrafficTable(ap_table, pTailNode->vertex);
        if(flag)
	  break;
      } //end of if-1.2

      /* obtain the edge EDD and compute the vehicle movement time for the remaining edge */
      if(i == 0) //if-1.3
      //if this is the first hop
      {
	/* obtain the vehicle's edge EDD */
	edge_EDD = pEdgeNode->head_gnode->EDD;

	/* obtain the vehicle's edge EDD_VAR */
	edge_EDD_VAR = pEdgeNode->head_gnode->EDD_VAR;

	/*@for debugging: we use temporarily the square root of edge_EDD_VAR for the vehicle's current edge as its vehicle_EDD_SD */
	*vehicle_EDD_SD = sqrt(edge_EDD_VAR);

	/* obtain the carry delay by vehicle movement in the case where a vehicle cannot forward its packet to appropriate next carrier */
	offset_in_directional_edge = vehicle->current_pos_in_digraph.offset;
	remaining_edge_length = vehicle->edge_length - offset_in_directional_edge;
	M = remaining_edge_length/vehicle->speed; //movement time for the remaining edge length

        /* compute the estimated directional edge delay for the whole edge length */
	directional_edge_delay_for_whole_edge = pEdgeNode->head_gnode->edge_delay;
	//directional_edge_delay_for_whole_edge = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, pEdgeNode->weight); //@[03/03/2009] For version iTBD-v2.1.0, this line is added to let the vehicle's EDD correct in one road segment

        /* compute the estimated directional edge delay for the remaining edge length */
        directional_edge_delay_for_remaining_edge = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, remaining_edge_length); //@[03/03/2009] For version iTBD-v2.1.0, uncomment this line to let EDD correct in one road segment
        
	/* Note that the above gives the very similar performance as the below (i.e., the same packet loss, but a little longer delay with 1 second), but logically the below makes sense since the vehicle should carry packets in order to compute path_EDD related to the vehicle's path */
	//directional_edge_delay_for_remaining_edge = M; //for version v1.3.9

	/** compute the estimated directional edge delay for the current offset on the directional edge */
       	//directional_edge_delay_for_offset = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, offset_in_directional_edge); //@[03/03/2009] For version iTBD-v2.1.0, comment this line to let edge EDD be the sum of the delay for the traveled length on the edge and the EDD for the remaining subedge length

	directional_edge_delay_for_offset = directional_edge_delay_for_whole_edge - directional_edge_delay_for_remaining_edge; //@[03/03/2009] Note that this delay for the offset is based only on vehicular traffic statistics on this edge, not real vehicular traffic condition, such as the network component consisting of vehicles for data forwarding
      } //end of if-1.3
      else //else-1.4
      //otherwise
      {
	/* obtain the vehicle's edge EDD */
	edge_EDD = pEdgeNode->head_gnode->EDD;

	/* obtain the carry delay by vehicle movement in the case where a vehicle cannot forward its packet to appropriate next carrier */
	M = pEdgeNode->weight/vehicle->speed;
      } //end of else-1.4
    } //end of if-1
    else //else-2
    //handling of EDD at the destination node where path_edge_head->vertex is NULL
    {
      /* obtain the pointer to the destination graph node */
      path_edge_tail = path_edge_head;
      node_id = atoi(path_edge_tail);
      pTailNode = &(G[node_id-1]);

      /* check the computation type of trajectory-based EDD */
      if(param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH) //if-2.1
      //if edd computation type is partial-path and the tail node is one of APs, then finish the EDD computation here; that is, we consider the EDD of the rest of the path just after the AP to be zero
      {
        /* check whether pTailNode->vertex is one of APs; if so, escape from this for-loop since the EDD at access point must be zero */

        flag = IsVertexInTrafficTable(ap_table, pTailNode->vertex);

        if(flag)
	  break;
      } //end of if-2.1

      /* set the vehicle's edge EDD to INFINITE since the vehicle has arrived at its destination, so it has no next edge on its trajectory */
      edge_EDD = INF;

      /* set the carry delay by vehicle movement to zero since the vehicle has arrived at its destination, so it does not move any more according to its trajectory */
      M = 0;

      /* set the flag for the EDD processing at the destination for the case where the vehicle cannot forward its packets to the next carrier, so must carry them with itself according to its new trajectory */
      flag_for_destination = TRUE;
      
    } //end of else-2


    if((i == 0) && (euclidean_distance2(&(pTailNode->coordinate), &(vehicle->current_pos)) <= R)) //In the case where the vehicle is within the intersection area for the tail node of the directional edge where the vehicle is moving, we consider the forwarding towards all of the directional edges incident to the tail intersection
    { //@Note: for the first hop, we consider two cases: (i) the case of forwarding the packet and (ii) the case of carrying the packet.

      /* compute vehicle's current EDD based on its offset on the directiona edge where it is moving */
      current_vehicle_EDD = edge_EDD - directional_edge_delay_for_offset; //the vehicle's EDD at this point
      
      /* check the validity of current_vehicle_EDD */
      if(current_vehicle_EDD < 0)
      {
	printf("VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(): <Error> current_vehicle_EDD(%.2f) is negative!\n", (float)current_vehicle_EDD);
	exit(1);
      }

      path_EDD = 0; //initialize path_EDD
      P_ac = 1; //initialize P_ac
      P_ic = 1; //initialize P_ic

      /* set the pointer to head node of the outgoing edge at the intersection corresponding to pTailNode->vertex */
      pHeadNode = pTailNode; //set pHeadNode to the pointer to the tail node 
      for(j = 0; j < pTailNode->weight; j++) //for-2
      {
	pHeadNode = pHeadNode->next;

        /** NOTE: is it correct that we use edge_EDD instead of using the vehicle's actual EDD at the intersection corresponding to pTailNode->vertex */
	//check whether the branch's EDD is greater than the EDD of the branch on the trajectory
	if(pHeadNode->EDD > current_vehicle_EDD)
	  continue; //in this case, we don't use this branch for packet forwarding

	//add the portion of EDD for the forwarding into this outgoing branch to path_EDD
	path_EDD += P_ac * pHeadNode->P_prime_pure * pHeadNode->EDD; //use pure forwarding probability for TBD

	//update the carry probability at the intersection
	P_ic -= pHeadNode->P_prime_pure; //use pure forwarding probability for TBD
      } //end of for-2

      /* update the accumulative carry probability P_ac with intersection carry probability P_ic */
      P_ac *= P_ic;

      /* update accumulated_carry_delay with M */
      accumulated_carry_delay = M;
    }
    else if((i == 0) && (euclidean_distance2(&(pTailNode->coordinate), &(vehicle->current_pos)) > R)) //In the case where the vehicle is out of the intersection area for the tail node of the directional edge where the vehicle is moving, we consider the forwarding towards the directional edge where the vehicle is moving
    { //@Note: for the first hop, we consider two cases: (i) the case of forwarding the packet and (ii) the case of carrying the packet.

      /* compute vehicle's current EDD based on its offset on the directiona edge where it is moving */
      current_vehicle_EDD = edge_EDD - directional_edge_delay_for_offset; //the vehicle's EDD at this point
      
      /* check the validity of current_vehicle_EDD */
      if(current_vehicle_EDD < 0)
      {
	printf("VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(): <Error> current_vehicle_EDD(%.2f) is negative!\n", (float)current_vehicle_EDD);
	exit(1);
      }

      P_ac = 1; //initialize P_ac

      pHeadNode = pEdgeNode->head_gnode; //set pHeadNode to the pointer to the head node 

      path_EDD = P_ac * pHeadNode->P_prime_pure * pHeadNode->EDD; //path_EDD for the case where the vehicle can forward its packet towards the edge where it is moving
      //path_EDD = pHeadNode->P_prime_pure*directional_edge_delay_for_remaining_edge; //@[03/06/09], link delay considering both forwarding and carry for the remaining edge length, so we multiply the forwarding probability P_prime_pure; 

      /* update the accumulative carry probability P_ac with intersection carry probability P_ic */
      P_ic = 1 - pHeadNode->P_prime_pure; //P_ic is the carry probability to carry the packet by the head intersection without forwarding
      P_ac *= P_ic; //update tge accumulative carry probability with P_ic

      /* update accumulated_carry_delay with M */
      accumulated_carry_delay = M;
    }
    else //else-1
    {
      /* update path_EDD with accumulated_carry_delay up to this point and the accumulative carry probability P_ac */
      path_EDD += P_ac*accumulated_carry_delay; //@modified on 11/23/08; Note that path_EDD takes the movement time for the vehicle's carry length from its starting position to the current edge after updating P_ac

      /* update accumulated_carry_delay with M towards the head intersection */
      accumulated_carry_delay += M; //the carry delay M is added to accumulated carry delay for the next carry case

      current_vehicle_EDD = edge_EDD; //the vehicle's EDD at this point

      P_ic = 1; //initialize P_ic

      /* set the pointer to head node of the outgoing edge at the intersection corresponding to pTailNode->vertex */
      pHeadNode = pTailNode;
      for(j = 0; j < pTailNode->weight; j++) //for-2
      {
	pHeadNode = pHeadNode->next;

        /** NOTE: is it correct that we use edge_EDD instead of using the vehicle's actual EDD at the intersection corresponding to pTailNode->vertex */
	//check whether the branch's EDD is greater than the EDD of the branch on the trajectory
	if(pHeadNode->EDD > current_vehicle_EDD)
	  continue; //in this case, we don't use this branch for packet forwarding

	//check whether the branch's EDD is greater than the path_EDD according to the trajectory
	//if(pHeadNode->EDD > path_EDD)
	//  continue; //in this case, we don't use this branch for packet forwarding

	//add the portion of EDD for the forwarding into this outgoing branch to path_EDD
	path_EDD += P_ac * pHeadNode->P_prime_pure * pHeadNode->EDD; //use pure forwarding probability for TBD
	//path_EDD += P_ac * pHeadNode->P_pure * pHeadNode->EDD; //use pure forwarding probability
	//path_EDD += P_ac * pHeadNode->P * pHeadNode->EDD; //use forwarding probability
	//path_EDD += P_ac * pHeadNode->CP * pHeadNode->EDD;

	//update the carry probability at the intersection
	P_ic -= pHeadNode->P_prime_pure; //use pure forwarding probability for TBD
	//P_ic -= pHeadNode->P_pure; //use pure forwarding probability
	//P_ic -= pHeadNode->P; //use forwarding probability
	//P_ic *= (1 - pHeadNode->CP); //(1 - pHeadNode->CP) is the probability that the vehicle cannot meet a next carrier moving on the edge <pTailNode->vertex, pHeadNode->vertex>
      } //end of for-2

      /* update the accumulative carry probability P_ac with intersection carry probability P_ic */
      P_ac *= P_ic;

      /* process path_EDD at the destination for the case where the vehicle cannot forward its packets to a moving neighboring carrier */
      if(flag_for_destination)
      {
	/* compute the EDD in the case where the vehicle cannot forward its packets at the desination; that is, after its new trajectory is determined, it must carry them with itself */

        P_branch = 1/pTailNode->weight; //branch probability of the vehicle towards the next intersection after its new trajectory is determined, assuming the Random-Way-Point (RWP) mobility of the vehicle

        /* set the pointer to head node of the outgoing edge at the intersection corresponding to pTailNode->vertex */
        pHeadNode = pTailNode;
        for(j = 0; j < pTailNode->weight; j++) //for-2.2.1
        {
	  pHeadNode = pHeadNode->next;

	  //add the portion of EDD for the forwarding into this outgoing branch to path_EDD
	  path_EDD += P_ac * P_branch * pHeadNode->EDD; //use the same branch probability for each directional edge from the tail node
        } //end of for-2.2.1
      }
    } //end of else-1  

    /* let pPathNode point the head node of the next hop */
    pPathNode = pPathNode->next;

    /* check the computation type of trajectory-based EDD */
    if(param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_ONE_HOP_AHEAD && i == 1) //if edd computation type is one-hop-ahead, then finish the EDD computation here
      break;
  } //end of for-1

  /** update vehicle's EDD and EDD_SD */
  *vehicle_EDD = MIN(path_EDD, INF); //vehicle_EDD is at most INF
  //*vehicle_EDD = path_EDD;

  /* To de done for the computation of vehicle_EDD_SD based on the vehicle trajectory :
     Note that we use the edge's EDD_SD instead. */
  //*vehicle_EDD_SD = sqrt(path_EDD_VAR);

  //return path_EDD;
}

void VADD_Compute_EDD_And_EDD_SD_Based_On_TBD_VERSION_2(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD)
{ //compute the EDD and EDD_SD based on TBD Model (i.e., Per-vehicle Model) in vehicle's current position on the current directional edge
  double R = param->communication_range; //communication range
  double path_EDD = 0; //vehicle's path EDD
  double edge_EDD = 0; //vehicle's edge EDD
  double path_EDD_VAR = 0; //vehicle's path EDD_VAR
  double edge_EDD_VAR = 0; //vehicle's edge EDD_VAR
  int i, j; //indices for for-loops
  struct_path_node *pPathNode = NULL; //pointer to path node
  char *path_edge_tail = NULL; //pointer to the tail vertex of the edge on the path
  char *path_edge_head = NULL; //pointer to the head vertex of the edge on the path
  struct_graph_node *pTailNode = NULL; //pointer to the tail graph node of an edge
  struct_graph_node *pHeadNode = NULL; //pointer to the head graph node of an edge
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge node of <pTailNode,pHeadNode>
  double P_ac = 0; //accumulative carry probability along the trajectory
  double P_ic = 0; //carry probability at each intersection
  double P_branch = 0; //branch probability that the vehicle will branch to a neighboring edge after the arrival at its destination
  double M = 0; //carry delay in an edge
  int node_id = 0; //node id
  int remaining_hop_count = 0; //remaining hop count from the vehicle's current position to its destination
  double offset_in_directional_edge = 0; //vehicle's offset in the directional edge where the vehicle is moving
  double directional_edge_delay_for_whole_edge = 0; //directional edge delay for the whole edge length
  //double directional_edge_delay_for_remaining_edge_length = 0; //directional edge delay for vehicle's remaining edge length
  double remaining_edge_length = 0; //remaining edge length for the vehicle's current directional edge
  double directional_edge_delay_for_remaining_edge = 0; //directional edge delay for the remaining edge where the vehicle is moving
  double directional_edge_delay_for_offset = 0; //directional edge delay for the offset where the vehicle has moved
  double current_vehicle_EDD = 0; //the vehicle's EDD at a future position on its trajectory 
  double current_vehicle_EDD_VAR = 0; //the vehicle's EDD_VAR at a future position on its trajectory 
  boolean flag = FALSE; //flag to indicate that the vehicle's trajectory meets one of access points 
  boolean flag_for_destination = FALSE; //flag for the EDD processing at the destination for the case where the vehicle cannot forward its packets to the next carrier, so must carry them with itself according to its new trajectory
  double accumulated_carry_delay = 0; //the accumulated carry delay for the carry length from the starting position to the current intersection on the trajectory that the original packet carrier has carried the packets without forwarding them.
  double accumulated_carry_delay_var = 0; //the accumulated carry delay variance for the carry length from the starting position to the current intersection on the trajectory that the original packet carrier has carried the packets without forwarding them.

  pPathNode = vehicle->path_ptr;
 
  P_ac = 1; //initialize P_ac

  remaining_hop_count = vehicle->path_hop_count - vehicle->path_current_hop; //compute the remaining hop count from the vehicle's current position to its destination

  /**@ for debugging */
  //if(vehicle->id == 1 && vehicle->state_time >= 3599)
  //{
  //  printf("VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(): at time=%.1f, vehicle(id=%d) is traced\n", (float)vehicle->state_time, vehicle->id);
  //}

  if(remaining_hop_count == 0)
  {
    printf("VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(): remaining_hop_count is zero!\n");
    fgetc(stdin);
  }
  /*******************/

  for(i = 0; i <= remaining_hop_count; i++) //for-1
  {
    if(i < remaining_hop_count) //if-1
    {
      path_edge_tail = pPathNode->vertex;
      path_edge_head = pPathNode->next->vertex;

      /* obtain the pointer to the direaction edge of <tail_node,head_node> */
      pEdgeNode = FastLookupDirectionalEdgeQueue(G, path_edge_tail, path_edge_head);
      if(pEdgeNode == NULL) //if-1.1
      {
	printf("VADD_Compute_TBD_Based_EDD(): pEdgeNode for <%s,%s> is NULL\n", path_edge_tail, path_edge_head);
	exit(1);
      } //end of if-1.1

      /* set the pointer to tail node of the outgoing edge at the intersection corresponding to intermediate intersection, pointed by pEdgeNode->tail_gnode->vertex */
      pTailNode = pEdgeNode->tail_gnode;

      /* check the computation type of trajectory-based EDD */
      if(i != 0 && param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH) //if-1.2: Note that the condition (i != 0) lets the vehicle moving from one of APs have positive EDD
      //if(param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH) //if-1.2
      //if edd computation type is partial-path and the tail node is one of APs, then finish the EDD computation here
      {
        /* check whether pTailNode->vertex is one of APs; if so, escape from this for-loop since the EDD at access point must be zero; that is, we consider the EDD of the rest of the path just after the AP to be zero. */

        flag = IsVertexInTrafficTable(ap_table, pTailNode->vertex);
        if(flag)
	  break;
      } //end of if-1.2

      /* obtain the edge EDD and compute the vehicle movement time for the remaining edge */
      if(i == 0) //if-1.3
      //if this is the first hop
      {
	/* obtain the vehicle's edge EDD */
	edge_EDD = pEdgeNode->head_gnode->EDD;

	/* obtain the vehicle's edge EDD_VAR */
	edge_EDD_VAR = pEdgeNode->head_gnode->EDD_VAR;

	/*@for debugging: we use temporarily the square root of edge_EDD_VAR for the vehicle's current edge as its vehicle_EDD_SD */
	*vehicle_EDD_SD = sqrt(edge_EDD_VAR);

	/* obtain the carry delay by vehicle movement in the case where a vehicle cannot forward its packet to appropriate next carrier */
	offset_in_directional_edge = vehicle->current_pos_in_digraph.offset;
	remaining_edge_length = vehicle->edge_length - offset_in_directional_edge;
	M = remaining_edge_length/vehicle->speed; //movement time for the remaining edge length

        /* compute the estimated directional edge delay for the whole edge length */
	directional_edge_delay_for_whole_edge = pEdgeNode->head_gnode->edge_delay;
	//directional_edge_delay_for_whole_edge = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, pEdgeNode->weight); //@[03/03/2009] For version iTBD-v2.1.0, this line is added to let the vehicle's EDD correct in one road segment

        /* compute the estimated directional edge delay for the remaining edge length */
        directional_edge_delay_for_remaining_edge = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, remaining_edge_length); //@[03/03/2009] For version iTBD-v2.1.0, uncomment this line to let EDD correct in one road segment
        
	/* Note that the above gives the very similar performance as the below (i.e., the same packet loss, but a little longer delay with 1 second), but logically the below makes sense since the vehicle should carry packets in order to compute path_EDD related to the vehicle's path */
	//directional_edge_delay_for_remaining_edge = M; //for version v1.3.9

	/** compute the estimated directional edge delay for the current offset on the directional edge */
       	//directional_edge_delay_for_offset = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, offset_in_directional_edge); //@[03/03/2009] For version iTBD-v2.1.0, comment this line to let edge EDD be the sum of the delay for the traveled length on the edge and the EDD for the remaining subedge length

	directional_edge_delay_for_offset = directional_edge_delay_for_whole_edge - directional_edge_delay_for_remaining_edge; //@[03/03/2009] Note that this delay for the offset is based only on vehicular traffic statistics on this edge, not real vehicular traffic condition, such as the network component consisting of vehicles for data forwarding
      } //end of if-1.3
      else //else-1.4
      //otherwise
      {
	/* obtain the vehicle's edge EDD */
	edge_EDD = pEdgeNode->head_gnode->EDD;

	/* obtain the carry delay by vehicle movement in the case where a vehicle cannot forward its packet to appropriate next carrier */
	M = pEdgeNode->weight/vehicle->speed;
      } //end of else-1.4
    } //end of if-1
    else //else-2
    //handling of EDD at the destination node where path_edge_head->vertex is NULL
    {
      /* obtain the pointer to the destination graph node */
      path_edge_tail = path_edge_head;
      node_id = atoi(path_edge_tail);
      pTailNode = &(G[node_id-1]);

      /* check the computation type of trajectory-based EDD */
      if(param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH) //if-2.1
      //if edd computation type is partial-path and the tail node is one of APs, then finish the EDD computation here; that is, we consider the EDD of the rest of the path just after the AP to be zero
      {
        /* check whether pTailNode->vertex is one of APs; if so, escape from this for-loop since the EDD at access point must be zero */

        flag = IsVertexInTrafficTable(ap_table, pTailNode->vertex);

        if(flag)
	  break;
      } //end of if-2.1

      /* set the vehicle's edge EDD to INFINITE since the vehicle has arrived at its destination, so it has no next edge on its trajectory */
      edge_EDD = INF;

      /* set the carry delay by vehicle movement to zero since the vehicle has arrived at its destination, so it does not move any more according to its trajectory */
      M = 0;

      /* set the flag for the EDD processing at the destination for the case where the vehicle cannot forward its packets to the next carrier, so must carry them with itself according to its new trajectory */
      flag_for_destination = TRUE;
      
    } //end of else-2


    if((i == 0) && (euclidean_distance2(&(pTailNode->coordinate), &(vehicle->current_pos)) <= R)) //In the case where the vehicle is within the intersection area for the tail node of the directional edge where the vehicle is moving, we consider the forwarding towards all of the directional edges incident to the tail intersection
    { //@Note: for the first hop, we consider two cases: (i) the case of forwarding the packet and (ii) the case of carrying the packet.

      /* compute vehicle's current EDD based on its offset on the directiona edge where it is moving */
      current_vehicle_EDD = edge_EDD - directional_edge_delay_for_offset; //the vehicle's EDD at this point
      
      /* check the validity of current_vehicle_EDD */
      if(current_vehicle_EDD < 0)
      {
	printf("VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(): <Error> current_vehicle_EDD(%.2f) is negative!\n", (float)current_vehicle_EDD);
	exit(1);
      }

      path_EDD = 0; //initialize path_EDD
      P_ac = 1; //initialize P_ac
      P_ic = 1; //initialize P_ic

      /* set the pointer to head node of the outgoing edge at the intersection corresponding to pTailNode->vertex */
      pHeadNode = pTailNode; //set pHeadNode to the pointer to the tail node 
      for(j = 0; j < pTailNode->weight; j++) //for-2
      {
	pHeadNode = pHeadNode->next;

        /** NOTE: is it correct that we use edge_EDD instead of using the vehicle's actual EDD at the intersection corresponding to pTailNode->vertex */
	//check whether the branch's EDD is greater than the EDD of the branch on the trajectory
	if(pHeadNode->EDD > current_vehicle_EDD)
	  continue; //in this case, we don't use this branch for packet forwarding

	//add the portion of EDD for the forwarding into this outgoing branch to path_EDD
	path_EDD += P_ac * pHeadNode->P_prime_pure * pHeadNode->EDD; //use pure forwarding probability for TBD

	//update the carry probability at the intersection
	P_ic -= pHeadNode->P_prime_pure; //use pure forwarding probability for TBD
      } //end of for-2

      /* update the accumulative carry probability P_ac with intersection carry probability P_ic */
      P_ac *= P_ic;

      /* update accumulated_carry_delay with M */
      accumulated_carry_delay = M;
    }
    else if((i == 0) && (euclidean_distance2(&(pTailNode->coordinate), &(vehicle->current_pos)) > R)) //In the case where the vehicle is out of the intersection area for the tail node of the directional edge where the vehicle is moving, we consider the forwarding towards the directional edge where the vehicle is moving
    { //@Note: for the first hop, we consider two cases: (i) the case of forwarding the packet and (ii) the case of carrying the packet.

      /* compute vehicle's current EDD based on its offset on the directiona edge where it is moving */
      current_vehicle_EDD = edge_EDD - directional_edge_delay_for_offset; //the vehicle's EDD at this point
      
      /* check the validity of current_vehicle_EDD */
      if(current_vehicle_EDD < 0)
      {
	printf("VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(): <Error> current_vehicle_EDD(%.2f) is negative!\n", (float)current_vehicle_EDD);
	exit(1);
      }

      P_ac = 1; //initialize P_ac

      pHeadNode = pEdgeNode->head_gnode; //set pHeadNode to the pointer to the head node 

      path_EDD = P_ac * pHeadNode->P_prime_pure * pHeadNode->EDD; //path_EDD for the case where the vehicle can forward its packet towards the edge where it is moving
      //path_EDD = pHeadNode->P_prime_pure*directional_edge_delay_for_remaining_edge; //@[03/06/09], link delay considering both forwarding and carry for the remaining edge length, so we multiply the forwarding probability P_prime_pure; 

      /* update the accumulative carry probability P_ac with intersection carry probability P_ic */
      P_ic = 1 - pHeadNode->P_prime_pure; //P_ic is the carry probability to carry the packet by the head intersection without forwarding
      P_ac *= P_ic; //update tge accumulative carry probability with P_ic

      /* update accumulated_carry_delay with M */
      accumulated_carry_delay = M;
    }
    else //else-1
    {
      /* update path_EDD with accumulated_carry_delay up to this point and the accumulative carry probability P_ac */
      path_EDD += P_ac*accumulated_carry_delay; //@modified on 11/23/08; Note that path_EDD takes the movement time for the vehicle's carry length from its starting position to the current edge after updating P_ac

      /* update accumulated_carry_delay with M towards the head intersection */
      accumulated_carry_delay += M; //the carry delay M is added to accumulated carry delay for the next carry case

      current_vehicle_EDD = edge_EDD; //the vehicle's EDD at this point

      P_ic = 1; //initialize P_ic

      /* set the pointer to head node of the outgoing edge at the intersection corresponding to pTailNode->vertex */
      pHeadNode = pTailNode;
      for(j = 0; j < pTailNode->weight; j++) //for-2
      {
	pHeadNode = pHeadNode->next;

        /** NOTE: is it correct that we use edge_EDD instead of using the vehicle's actual EDD at the intersection corresponding to pTailNode->vertex */
	//check whether the branch's EDD is greater than the EDD of the branch on the trajectory
	if(pHeadNode->EDD > current_vehicle_EDD)
	  continue; //in this case, we don't use this branch for packet forwarding

	//check whether the branch's EDD is greater than the path_EDD according to the trajectory
	//if(pHeadNode->EDD > path_EDD)
	//  continue; //in this case, we don't use this branch for packet forwarding

	//add the portion of EDD for the forwarding into this outgoing branch to path_EDD
	path_EDD += P_ac * pHeadNode->P_prime_pure * pHeadNode->EDD; //use pure forwarding probability for TBD
	//path_EDD += P_ac * pHeadNode->P_pure * pHeadNode->EDD; //use pure forwarding probability
	//path_EDD += P_ac * pHeadNode->P * pHeadNode->EDD; //use forwarding probability
	//path_EDD += P_ac * pHeadNode->CP * pHeadNode->EDD;

	//update the carry probability at the intersection
	P_ic -= pHeadNode->P_prime_pure; //use pure forwarding probability for TBD
	//P_ic -= pHeadNode->P_pure; //use pure forwarding probability
	//P_ic -= pHeadNode->P; //use forwarding probability
	//P_ic *= (1 - pHeadNode->CP); //(1 - pHeadNode->CP) is the probability that the vehicle cannot meet a next carrier moving on the edge <pTailNode->vertex, pHeadNode->vertex>
      } //end of for-2

      /* update the accumulative carry probability P_ac with intersection carry probability P_ic */
      P_ac *= P_ic;

      /* process path_EDD at the destination for the case where the vehicle cannot forward its packets to a moving neighboring carrier */
      if(flag_for_destination)
      {
	/* compute the EDD in the case where the vehicle cannot forward its packets at the desination; that is, after its new trajectory is determined, it must carry them with itself */

        P_branch = 1/pTailNode->weight; //branch probability of the vehicle towards the next intersection after its new trajectory is determined, assuming the Random-Way-Point (RWP) mobility of the vehicle

        /* set the pointer to head node of the outgoing edge at the intersection corresponding to pTailNode->vertex */
        pHeadNode = pTailNode;
        for(j = 0; j < pTailNode->weight; j++) //for-2.2.1
        {
	  pHeadNode = pHeadNode->next;

	  //add the portion of EDD for the forwarding into this outgoing branch to path_EDD
	  path_EDD += P_ac * P_branch * pHeadNode->EDD; //use the same branch probability for each directional edge from the tail node
        } //end of for-2.2.1
      }
    } //end of else-1  

    /* let pPathNode point the head node of the next hop */
    pPathNode = pPathNode->next;

    /* check the computation type of trajectory-based EDD */
    if(param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_ONE_HOP_AHEAD && i == 1) //if edd computation type is one-hop-ahead, then finish the EDD computation here
      break;
  } //end of for-1

  /** update vehicle's EDD and EDD_SD */
  *vehicle_EDD = MIN(path_EDD, INF); //vehicle_EDD is at most INF
  //*vehicle_EDD = path_EDD;

  /* To de done for the computation of vehicle_EDD_SD based on the vehicle trajectory :
     Note that we use the edge's EDD_SD instead. */
  //*vehicle_EDD_SD = sqrt(path_EDD_VAR);

  //return path_EDD;
}

void VADD_Compute_EDD_And_EDD_SD_Based_On_TBD_VERSION_3(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD)
{ //[10/06/09] compute the EDD and EDD_SD based on TBD Model (i.e., Per-vehicle Model) in vehicle's current position on the current directional edge
  double R = param->communication_range; //communication range
  double v = param->vehicle_speed; //vehicle speed
  double path_EDD = 0; //vehicle's path EDD
  double edge_EDD = 0; //vehicle's edge EDD
  double path_EDD_VAR = 0; //vehicle's path EDD_VAR
  double edge_EDD_VAR = 0; //vehicle's edge EDD_VAR
  int i, j; //indices for for-loops
  struct_path_node *pPathNode = NULL; //pointer to path node
  char *path_edge_tail = NULL; //pointer to the tail vertex of the edge on the path
  char *path_edge_head = NULL; //pointer to the head vertex of the edge on the path
  struct_graph_node *pTailNode = NULL; //pointer to the tail graph node of an edge
  struct_graph_node *pHeadNode = NULL; //pointer to the head graph node of an edge
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge node of <pTailNode,pHeadNode>
  double P_ac = 0; //accumulative carry probability along the trajectory
  double P_ic = 0; //carry probability at each intersection
  double P_branch = 0; //branch probability that the vehicle will branch to a neighboring edge after the arrival at its destination
  double M = 0; //carry delay in an edge
  int node_id = 0; //node id
  int remaining_hop_count = 0; //remaining hop count from the vehicle's current position to its destination
  double offset_in_directional_edge = 0; //vehicle's offset in the directional edge where the vehicle is moving
  double directional_edge_delay_for_whole_edge = 0; //directional edge delay for the whole edge length
  //double directional_edge_delay_for_remaining_edge_length = 0; //directional edge delay for vehicle's remaining edge length
  double remaining_edge_length = 0; //remaining edge length for the vehicle's current directional edge
  double directional_edge_delay_for_remaining_edge = 0; //directional edge delay for the remaining edge where the vehicle is moving
  double directional_edge_delay_for_offset = 0; //directional edge delay for the offset where the vehicle has moved
  double current_vehicle_EDD = 0; //the vehicle's EDD at a future position on its trajectory 
  double current_vehicle_EDD_VAR = 0; //the vehicle's EDD_VAR at a future position on its trajectory 
  boolean flag = FALSE; //flag to indicate that the vehicle's trajectory meets one of access points 
  boolean flag_for_destination = FALSE; //flag for the EDD processing at the destination for the case where the vehicle cannot forward its packets to the next carrier, so must carry them with itself according to its new trajectory
  double accumulated_carry_delay = 0; //the accumulated carry delay for the carry length from the starting position to the current intersection on the trajectory that the original packet carrier has carried the packets without forwarding them.
  double accumulated_carry_delay_var = 0; //the accumulated carry delay variance for the carry length from the starting position to the current intersection on the trajectory that the original packet carrier has carried the packets without forwarding them.
  double lambda = 0; //vehicle arrival rate into a directional edge
  double a = R/v; //the inter-arrival bound that two consecutive vehicles can be connected with each other through communication range
  double beta = 0; //the probability that the original carrier will meet another carrier on the edge of its trajectory
  struct_graph_node *path_head_gnode = NULL; //pointer to the head graph node of the current path edge (path_edge_tail, path_edge_head) on the path
  double EDD_portion = 0; //the EDD portion for this current edge when the packet can be forwarded to another carrier 
  double carry_delay_portion = 0; //the carry delay portion for this current edge, considering the edge length or the remaining edge length
  
  /* obtain the pointer to the vehicle's current path node */
  pPathNode = vehicle->path_ptr;
 
  P_ac = 1; //initialize P_ac

  remaining_hop_count = vehicle->path_hop_count - vehicle->path_current_hop; //compute the remaining hop count from the vehicle's current position to its destination

  /**@ for debugging */
  //if(vehicle->id == 1 && vehicle->state_time >= 3599)
  //{
  //  printf("VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(): at time=%.1f, vehicle(id=%d) is traced\n", (float)vehicle->state_time, vehicle->id);
  //}

  if(remaining_hop_count == 0)
  {
    printf("VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(): remaining_hop_count is zero!\n");
    fgetc(stdin);
  }
  /*******************/

  for(i = 0; i <= remaining_hop_count; i++) //for-1
  {
    if(i < remaining_hop_count) //if-1
    {
      path_edge_tail = pPathNode->vertex;
      path_edge_head = pPathNode->next->vertex;

      /* obtain the pointer to the direaction edge of <tail_node,head_node> */
      pEdgeNode = FastLookupDirectionalEdgeQueue(G, path_edge_tail, path_edge_head);
      if(pEdgeNode == NULL) //if-1.1
      {
	printf("VADD_Compute_TBD_Based_EDD(): pEdgeNode for <%s,%s> is NULL\n", path_edge_tail, path_edge_head);
	exit(1);
      } //end of if-1.1

      /* set the pointer to tail node of the outgoing edge at the intersection corresponding to intermediate intersection, pointed by pEdgeNode->tail_gnode->vertex */
      pTailNode = pEdgeNode->tail_gnode;

      /* set the pointer to head node of the outgoing edge */
      path_head_gnode = pEdgeNode->head_gnode;

      /* check the computation type of trajectory-based EDD */
      if(i != 0 && param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH) //if-1.2: Note that the condition (i != 0) lets the vehicle moving from one of APs have positive EDD
      //if(param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH) //if-1.2
      //if edd computation type is partial-path and the tail node is one of APs, then finish the EDD computation here
      {
        /* check whether pTailNode->vertex is one of APs; if so, escape from this for-loop since the EDD at access point must be zero; that is, we consider the EDD of the rest of the path just after the AP to be zero. */

        flag = IsVertexInTrafficTable(ap_table, pTailNode->vertex);
        if(flag)
	  break;
      } //end of if-1.2

      /* obtain the edge EDD and compute the vehicle movement time for the remaining edge */
      if(i == 0) //if-1.3
      //if this is the first hop
      {
	/* obtain the vehicle's edge EDD */
	edge_EDD = pEdgeNode->head_gnode->EDD;

	/* obtain the vehicle's edge EDD_VAR */
	edge_EDD_VAR = pEdgeNode->head_gnode->EDD_VAR;

	/*@for debugging: we use temporarily the square root of edge_EDD_VAR for the vehicle's current edge as its vehicle_EDD_SD */
	*vehicle_EDD_SD = sqrt(edge_EDD_VAR);

	/* obtain the carry delay by vehicle movement in the case where a vehicle cannot forward its packet to appropriate next carrier */
	offset_in_directional_edge = vehicle->current_pos_in_digraph.offset;
	remaining_edge_length = vehicle->edge_length - offset_in_directional_edge;
	M = remaining_edge_length/vehicle->speed; //movement time for the remaining edge length

        /* compute the estimated directional edge delay for the whole edge length */
	directional_edge_delay_for_whole_edge = pEdgeNode->head_gnode->edge_delay;
	//directional_edge_delay_for_whole_edge = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, pEdgeNode->weight); //@[03/03/2009] For version iTBD-v2.1.0, this line is added to let the vehicle's EDD correct in one road segment

        /* compute the estimated directional edge delay for the remaining edge length */
        directional_edge_delay_for_remaining_edge = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, remaining_edge_length); //@[03/03/2009] For version iTBD-v2.1.0, uncomment this line to let EDD correct in one road segment
        
	/* Note that the above gives the very similar performance as the below (i.e., the same packet loss, but a little longer delay with 1 second), but logically the below makes sense since the vehicle should carry packets in order to compute path_EDD related to the vehicle's path */
	//directional_edge_delay_for_remaining_edge = M; //for version v1.3.9

	/** compute the estimated directional edge delay for the current offset on the directional edge */
       	//directional_edge_delay_for_offset = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, offset_in_directional_edge); //@[03/03/2009] For version iTBD-v2.1.0, comment this line to let edge EDD be the sum of the delay for the traveled length on the edge and the EDD for the remaining subedge length

	directional_edge_delay_for_offset = directional_edge_delay_for_whole_edge - directional_edge_delay_for_remaining_edge; //@[03/03/2009] Note that this delay for the offset is based only on vehicular traffic statistics on this edge, not real vehicular traffic condition, such as the network component consisting of vehicles for data forwarding
      } //end of if-1.3
      else //else-1.4
      //otherwise
      {
	/* obtain the vehicle's edge EDD */
	edge_EDD = pEdgeNode->head_gnode->EDD;

	/* obtain the carry delay by vehicle movement in the case where a vehicle cannot forward its packet to appropriate next carrier */
	M = pEdgeNode->weight/vehicle->speed;
      } //end of else-1.4
    } //end of if-1
    else //else-2
    //handling of EDD at the destination node where path_edge_head->vertex is NULL
    {
      /* obtain the pointer to the destination graph node */
      path_edge_tail = path_edge_head;
      node_id = atoi(path_edge_tail);
      pTailNode = &(G[node_id-1]);
      path_head_gnode = NULL;

      /* check the computation type of trajectory-based EDD */
      if(param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH) //if-2.1
      //if edd computation type is partial-path and the tail node is one of APs, then finish the EDD computation here; that is, we consider the EDD of the rest of the path just after the AP to be zero
      {
        /* check whether pTailNode->vertex is one of APs; if so, escape from this for-loop since the EDD at access point must be zero */

        flag = IsVertexInTrafficTable(ap_table, pTailNode->vertex);

        if(flag)
	  break;
      } //end of if-2.1

      /* set the vehicle's edge EDD to INFINITE since the vehicle has arrived at its destination, so it has no next edge on its trajectory */
      edge_EDD = INF;

      /* set the carry delay by vehicle movement to zero since the vehicle has arrived at its destination, so it does not move any more according to its trajectory */
      M = 0;

      /* set the flag for the EDD processing at the destination for the case where the vehicle cannot forward its packets to the next carrier, so must carry them with itself according to its new trajectory */
      flag_for_destination = TRUE;
      
    } //end of else-2


/*     if((i == 0) && (euclidean_distance2(&(pTailNode->coordinate), &(vehicle->current_pos)) <= R)) //if-3: In the case where the vehicle is within the intersection area for the tail node of the directional edge where the vehicle is moving, we consider the forwarding towards all of the directional edges incident to the tail intersection */
/*     { //@Note: for the first hop, we consider two cases: (i) the case of forwarding the packet and (ii) the case of carrying the packet. */

/*       /\* compute vehicle's current EDD based on its offset on the directiona edge where it is moving *\/ */
/*       current_vehicle_EDD = edge_EDD - directional_edge_delay_for_offset; //the vehicle's EDD at this point */
      
/*       /\* check the validity of current_vehicle_EDD *\/ */
/*       if(current_vehicle_EDD < 0) */
/*       { */
/* 	printf("VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(): <Error> current_vehicle_EDD(%.2f) is negative!\n", (float)current_vehicle_EDD); */
/* 	exit(1); */
/*       } */

/*       path_EDD = 0; //initialize path_EDD */
/*       P_ac = 1; //initialize P_ac */
/*       P_ic = 1; //initialize P_ic */

/*       /\* process path_EDD at the destination for the case where the vehicle can forward its packets to a moving neighboring carrier *\/ */
/*       pHeadNode = pTailNode; //set pHeadNode to the pointer to the tail node  */
/*       for(j = 0; j < pTailNode->weight; j++) //for-2 */
/*       { */
/* 	pHeadNode = pHeadNode->next; */

/*         /\** NOTE: is it correct that we use edge_EDD instead of using the vehicle's actual EDD at the intersection corresponding to pTailNode->vertex *\/ */
/* 	//check whether the branch's EDD is greater than the EDD of the branch on the trajectory */
/* 	//if((strcmp(pHeadNode->vertex, path_head_gnode->vertex) == 0) && (pHeadNode->EDD > current_vehicle_EDD)) */
/* 	if(pHeadNode->EDD > edge_EDD) */
/* 	  continue; //in this case, we don't use this branch for packet forwarding */

/* 	//add the portion of EDD for the forwarding into this outgoing branch to path_EDD */
/* 	path_EDD += P_ac * pHeadNode->P_prime_pure * pHeadNode->EDD; //use pure forwarding probability for TBD */

/* 	//update the carry probability at the intersection */
/* 	P_ic -= pHeadNode->P_prime_pure; //use pure forwarding probability for TBD */
/*       } //end of for-2 */

/*       /\* compute the contact probability beta to forward packets to another vehicle on the original carrier's trajectory edge *\/ */
/*       //lambda = path_head_gnode->lambda; //obtain the vehicle arrival rate, assuming that lambda was updated before this function call */
/*       //beta = 1 - exp(-1*lambda*a); //this is different from the contact probability at intersection, that is, 1 - exp(-1*lambda*R/v) != 1 - exp(-1*lambda*2R/v) */

/*       /\* compute the EDD portion for this current edge when the packet can be forwarded to another carrier *\/ */
/*       //EDD_portion = P_ac*beta*current_vehicle_EDD; */

/*       /\* update path_EDD with EDD_portion *\/ */
/*       //path_EDD += EDD_portion; */

/*       /\* compute the carry delay portion for this current edge, considering the remaining edge length *\/ */
/*       //carry_delay_portion = P_ac*(1 - beta)*M; */

/*       /\* update path_EDD with carry_delay_portion *\/ */
/*       //path_EDD += carry_delay_portion; */

/*       /\* update the carry probability with beta *\/ */
/*       //P_ic -= beta; */

/*       /\* update the accumulative carry probability P_ac with intersection carry probability P_ic *\/ */
/*       P_ac *= P_ic; */

/*       /\* update accumulated_carry_delay with M *\/ */
/*       accumulated_carry_delay = M; */
/*     } //end of if-3 */
    //else if((i == 0) && (euclidean_distance2(&(pTailNode->coordinate), &(vehicle->current_pos)) > R)) //else-if-4: In the case where the vehicle is out of the intersection area for the tail node of the directional edge where the vehicle is moving, we consider the forwarding towards the directional edge where the vehicle is moving

    if(i == 0)
    { //@Note: for the first hop, we consider two cases: (i) the case of forwarding the packet and (ii) the case of carrying the packet.

      /* compute vehicle's current EDD based on its offset on the directiona edge where it is moving */
      current_vehicle_EDD = edge_EDD - directional_edge_delay_for_offset; //the vehicle's EDD at this point

      path_EDD = 0; //initialize path_EDD
      P_ac = 1; //initialize P_ac
      P_ic = 1; //initialize P_ic

      /** process path_EDD at the destination for the case where the vehicle can forward its packets to a moving neighboring carrier */
      /* compute the contact probability beta to forward packets to another vehicle on the original carrier's trajectory edge */

      lambda = path_head_gnode->lambda; //obtain the vehicle arrival rate, assuming that lambda was updated before this function call

      beta = 1 - exp(-1*lambda*a); //this is different from the contact probability at intersection, that is, 1 - exp(-1*lambda*R/v) != 1 - exp(-1*lambda*2R/v)

      /* compute the EDD portion for this current edge when the packet can be forwarded to another carrier */
      EDD_portion = P_ac*beta*current_vehicle_EDD;

      /* update path_EDD with EDD_portion */
      path_EDD = EDD_portion;

      /* compute the carry delay portion for this current edge, considering the remaining edge length */
      //carry_delay_portion = P_ac*(1 - beta)*M;

      /* update path_EDD with carry_delay_portion */
      //path_EDD += carry_delay_portion;

      /* update the carry probability with beta */
      P_ic -= beta;

      /* update the accumulative carry probability P_ac with intersection carry probability P_ic */
      P_ac *= P_ic;

      /* update accumulated_carry_delay with M */
      accumulated_carry_delay = M;
      //accumulated_carry_delay = carry_delay_portion;

      /* set path_EDD to carry delay M */
      //path_EDD = M; //this value of M determines the preferred carrier if two vehicles have the same vehicle trajectory 
      //path_EDD = directional_edge_delay_for_remaining_edge;
    } //end of else-if-4
    else if(flag_for_destination == FALSE) //else-if-5: the current edge's head node is not the vehicle's moving destination
    {
      /* update path_EDD with accumulated_carry_delay up to this point and the accumulative carry probability P_ac */
      path_EDD += P_ac*accumulated_carry_delay; //@modified on 11/23/08; Note that path_EDD takes the movement time for the vehicle's carry length from its starting position to the current edge after updating P_ac

      current_vehicle_EDD = edge_EDD; //the vehicle's EDD at this point

      P_ic = 1; //initialize P_ic

      /* process path_EDD at the destination for the case where the vehicle can forward its packets to a moving neighboring carrier */
      pHeadNode = pTailNode; //set the pointer to head node of the outgoing edge at the intersection corresponding to pTailNode->vertex
      for(j = 0; j < pTailNode->weight; j++) //for-2
      {
	pHeadNode = pHeadNode->next;

        /** NOTE: is it correct that we use edge_EDD instead of using the vehicle's actual EDD at the intersection corresponding to pTailNode->vertex */
	//check whether the branch's EDD is greater than the EDD of the branch on the trajectory
	if(pHeadNode->EDD > edge_EDD)
	  continue; //in this case, we don't use this branch for packet forwarding
	/** NOTE: When we consider all the edges' EDD, the performance is sometimes better than only the edges whose EDD are smaller than vehicle's edge EDD */

	//add the portion of EDD for the forwarding into this outgoing branch to path_EDD
	path_EDD += P_ac * pHeadNode->P_prime_pure * pHeadNode->EDD; //use pure forwarding probability for TBD
	//path_EDD += P_ac * pHeadNode->P_pure * pHeadNode->EDD; //use pure forwarding probability
	//path_EDD += P_ac * pHeadNode->P * pHeadNode->EDD; //use forwarding probability
	//path_EDD += P_ac * pHeadNode->CP * pHeadNode->EDD;

	//update the carry probability at the intersection
	P_ic -= pHeadNode->P_prime_pure; //use pure forwarding probability for TBD
	//P_ic -= pHeadNode->P_pure; //use pure forwarding probability
	//P_ic -= pHeadNode->P; //use forwarding probability
	//P_ic *= (1 - pHeadNode->CP); //(1 - pHeadNode->CP) is the probability that the vehicle cannot meet a next carrier moving on the edge <pTailNode->vertex, pHeadNode->vertex>
      } //end of for-2

      /* compute the contact probability beta to forward packets to another vehicle on the original carrier's trajectory edge */
      //lambda = path_head_gnode->lambda; //obtain the vehicle arrival rate, assuming that lambda was updated before this function call
      //beta = 1 - exp(-1*lambda*a); //this is different from the contact probability at intersection, that is, 1 - exp(-1*lambda*R/v) != 1 - exp(-1*lambda*2R/v)

      /* compute the EDD portion for this current edge when the packet can be forwarded to another carrier */
      //EDD_portion = P_ac*beta*current_vehicle_EDD;

      /* update path_EDD with EDD_portion */
      //path_EDD += EDD_portion;

      /* compute the carry delay portion for this current edge, considering the remaining edge length */
      //carry_delay_portion = P_ac*(1 - beta)*M;

      /* update path_EDD with carry_delay_portion */
      //path_EDD += carry_delay_portion;

      /* update the carry probability with beta */
      //P_ic -= beta;

      /* update the accumulative carry probability P_ac with intersection carry probability P_ic */
      P_ac *= P_ic;

      /* update accumulated_carry_delay with M towards the head intersection */
      accumulated_carry_delay += M; //the carry delay M is added to accumulated carry delay for the next carry case
    } //end of else-if-5
    else //else-6: this is for handling the last node in the vehicle trajectory
    {
      /* update path_EDD with accumulated_carry_delay up to this point and the accumulative carry probability P_ac */
      path_EDD += P_ac*accumulated_carry_delay; //@modified on 11/23/08; Note that path_EDD takes the movement time for the vehicle's carry length from its starting position to the current edge after updating P_ac

      current_vehicle_EDD = edge_EDD; //the vehicle's EDD at this point

      P_ic = 1; //initialize P_ic

      /** process path_EDD at the destination for the case where the vehicle can forward its packets to a moving neighboring carrier */
      /* set the pointer to head node of the outgoing edge at the intersection corresponding to pTailNode->vertex */
      pHeadNode = pTailNode;
      for(j = 0; j < pTailNode->weight; j++) //for-2
      {
	pHeadNode = pHeadNode->next;

        /** NOTE: is it correct that we use edge_EDD instead of using the vehicle's actual EDD at the intersection corresponding to pTailNode->vertex */
	//check whether the branch's EDD is greater than the EDD of the branch on the trajectory
	if(pHeadNode->EDD > edge_EDD)
	  continue; //in this case, we don't use this branch for packet forwarding

	//add the portion of EDD for the forwarding into this outgoing branch to path_EDD
	path_EDD += P_ac * pHeadNode->P_prime_pure * pHeadNode->EDD; //use pure forwarding probability for TBD

	//update the carry probability at the intersection
	P_ic -= pHeadNode->P_prime_pure; //use pure forwarding probability for TBD
      } //end of for-2

      /* update the accumulative carry probability P_ac with intersection carry probability P_ic */
      P_ac *= P_ic;

      /** process path_EDD at the destination for the case where the vehicle cannot forward its packets to a moving neighboring carrier */
      /* compute the EDD in the case where the vehicle cannot forward its packets at the desination; that is, after its new trajectory is determined, it must carry them with itself */

      P_branch = 1/pTailNode->weight; //branch probability of the vehicle towards the next intersection after its new trajectory is determined, assuming the Random-Way-Point (RWP) mobility of the vehicle
      
      pHeadNode = pTailNode; //set the pointer to head node of the outgoing edge at the intersection corresponding to pTailNode->vertex
      for(j = 0; j < pTailNode->weight; j++) //for-2.2.1
      {
	pHeadNode = pHeadNode->next;

	//add the portion of EDD for the forwarding into this outgoing branch to path_EDD
	path_EDD += P_ac * P_branch * pHeadNode->EDD; //use the same branch probability for each directional edge from the tail node
      } //end of for-2.2.1
    } //end of else-6  

    /* let pPathNode point the head node of the next hop */
    pPathNode = pPathNode->next;

    /* check the computation type of trajectory-based EDD */
    if(param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_ONE_HOP_AHEAD && i == 1) //if edd computation type is one-hop-ahead, then finish the EDD computation here
      break;
  } //end of for-1

  /** update vehicle's EDD and EDD_SD */
  *vehicle_EDD = MIN(path_EDD, INF); //vehicle_EDD is at most INF
  //*vehicle_EDD = path_EDD;

  /* To de done for the computation of vehicle_EDD_SD based on the vehicle trajectory :
     Note that we use the edge's EDD_SD instead. */
  //*vehicle_EDD_SD = sqrt(path_EDD_VAR);

  //return path_EDD;
}

void VADD_Compute_EDD_And_EDD_SD_Based_On_TBD_VERSION_4(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD)
{ //compute the EDD and EDD_SD based on TBD Model (i.e., Per-vehicle Model) in vehicle's current position on the current directional edge; note that this version is for TBD journal submission
	double R = param->communication_range; //communication range
	double v = param->vehicle_speed; //vehicle speed
	double path_EDD = 0; //vehicle's path EDD
	double edge_EDD = 0; //vehicle's edge EDD
	double path_EDD_VAR = 0; //vehicle's path EDD_VAR
	double edge_EDD_VAR = 0; //vehicle's edge EDD_VAR
	int i, j; //indices for for-loops
	struct_path_node *pPathNode = NULL; //pointer to path node
	char *path_edge_tail = NULL; //pointer to the tail vertex of the edge on the path
	char *path_edge_head = NULL; //pointer to the head vertex of the edge on the path
	struct_graph_node *pTailNode = NULL; //pointer to the tail graph node of an edge
	struct_graph_node *pHeadNode = NULL; //pointer to the head graph node of an edge
	directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge node of <pTailNode,pHeadNode>
	double P_ac = 0; //accumulative carry probability along the trajectory
	double P_ic = 0; //carry probability at each intersection
	double P_branch = 0; //branch probability that the vehicle will branch to a neighboring edge after the arrival at its destination
	double M = 0; //carry delay in an edge
	int node_id = 0; //node id
	int remaining_hop_count = 0; //remaining hop count from the vehicle's current position to its destination
	double offset_in_directional_edge = 0; //vehicle's offset in the directional edge where the vehicle is moving
	double directional_edge_delay_for_whole_edge = 0; //directional edge delay for the whole edge length
	double remaining_edge_length = 0; //remaining edge length for the vehicle's current directional edge
	double directional_edge_delay_for_remaining_edge = 0; //directional edge delay for the remaining edge where the vehicle is moving
	double directional_edge_delay_for_offset = 0; //directional edge delay for the offset where the vehicle has moved
	double current_vehicle_EDD = 0; //the vehicle's EDD at a future position on its trajectory 
	double current_vehicle_EDD_VAR = 0; //the vehicle's EDD_VAR at a future position on its trajectory 
	boolean flag = FALSE; //flag to indicate that the vehicle's trajectory meets one of access points 
	boolean flag_for_destination = FALSE; //flag for the EDD processing at the destination for the case where the vehicle cannot forward its packets to the next carrier, so must carry them with itself according to its new trajectory
	double accumulated_carry_delay = 0; //the accumulated carry delay for the carry length from the starting position to the current intersection on the trajectory that the original packet carrier has carried the packets without forwarding them.
	double accumulated_carry_delay_var = 0; //the accumulated carry delay variance for the carry length from the starting position to the current intersection on the trajectory that the original packet carrier has carried the packets without forwarding them.
	double lambda = 0; //vehicle arrival rate into a directional edge
	double a = R/v; //the inter-arrival bound that two consecutive vehicles can be connected with each other through communication range
	double beta = 0; //the probability that the original carrier will meet another carrier on the edge of its trajectory
	struct_graph_node *path_head_gnode = NULL; //pointer to the head graph node of the current path edge (path_edge_tail, path_edge_head) on the path
	double EDD_portion = 0; //the EDD portion for this current edge when the packet can be forwarded to another carrier 
	double carry_delay_portion = 0; //the carry delay portion for this current edge, considering the edge length or the remaining edge length
  
	/* obtain the pointer to the vehicle's current path node */
	pPathNode = vehicle->path_ptr;
 
	P_ac = 1; //initialize P_ac

	remaining_hop_count = vehicle->path_hop_count - vehicle->path_current_hop; //compute the remaining hop count from the vehicle's current position to its destination

	/**@ for debugging */
#if 0 /* [ */
	if(vehicle->id == 1 && vehicle->state_time >= 3599)
	{
		printf("VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(): at time=%.1f, vehicle(id=%d) is traced\n", (float)vehicle->state_time, vehicle->id);
	}
#endif 

	if(remaining_hop_count == 0)
	{
		printf("VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(): remaining_hop_count is zero!\n");
		fgetc(stdin);
	}
	/*******************/

	for(i = 0; i <= remaining_hop_count; i++) //for-1
	{
		if(i < remaining_hop_count) //if-1
		{
			path_edge_tail = pPathNode->vertex;
			path_edge_head = pPathNode->next->vertex;

			/* obtain the pointer to the direaction edge of <tail_node,head_node> */
			pEdgeNode = FastLookupDirectionalEdgeQueue(G, path_edge_tail, path_edge_head);
			if(pEdgeNode == NULL) //if-1.1
			{
				printf("VADD_Compute_TBD_Based_EDD(): pEdgeNode for <%s,%s> is NULL\n", path_edge_tail, path_edge_head);
				exit(1);
			} //end of if-1.1

			/* set the pointer to tail node of the outgoing edge at the intersection corresponding to intermediate intersection, pointed by pEdgeNode->tail_gnode->vertex */
			pTailNode = pEdgeNode->tail_gnode;

			/* set the pointer to head node of the outgoing edge */
			path_head_gnode = pEdgeNode->head_gnode;

			/* check the computation type of trajectory-based EDD */
			if(i != 0 && param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH) //if-1.2: Note that the condition (i != 0) lets the vehicle moving from one of APs have positive EDD
			//if edd computation type is partial-path and the tail node is one of APs, then finish the EDD computation here
			{
				/* check whether pTailNode->vertex is one of APs; if so, escape from this for-loop since the EDD at access point must be zero; that is, we consider the EDD of the rest of the path just after the AP to be zero. */

				flag = IsVertexInTrafficTable(ap_table, pTailNode->vertex);
				if(flag)
					break;
			} //end of if-1.2

			/* obtain the edge EDD and compute the vehicle movement time for the remaining edge */
			if(i == 0) //if-1.3
			//if this is the first hop
			{
				/* obtain the vehicle's edge EDD */
				edge_EDD = pEdgeNode->head_gnode->EDD;

				/* obtain the vehicle's edge EDD_VAR */
				edge_EDD_VAR = pEdgeNode->head_gnode->EDD_VAR;

				/*@for debugging: we use temporarily the square root of edge_EDD_VAR for the vehicle's current edge as its vehicle_EDD_SD */
				*vehicle_EDD_SD = sqrt(edge_EDD_VAR);

				/* obtain the carry delay by vehicle movement in the case where a vehicle cannot forward its packet to appropriate next carrier */
				offset_in_directional_edge = vehicle->current_pos_in_digraph.offset;
				remaining_edge_length = vehicle->edge_length - offset_in_directional_edge;
				M = remaining_edge_length/vehicle->speed; //movement time for the remaining edge length

				/* compute the estimated directional edge delay for the whole edge length */
				directional_edge_delay_for_whole_edge = pEdgeNode->head_gnode->edge_delay;

				/* compute the estimated directional edge delay for the remaining edge length */
				directional_edge_delay_for_remaining_edge = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, remaining_edge_length); //@[03/03/2009] For version iTBD-v2.1.0, uncomment this line to let EDD correct in one road segment
        
				/* Note that the above gives the very similar performance as the below (i.e., the same packet loss, but a little longer delay with 1 second), but logically the below makes sense since the vehicle should carry packets in order to compute path_EDD related to the vehicle's path */
				//directional_edge_delay_for_remaining_edge = M; //for version v1.3.9

				/** compute the estimated directional edge delay for the current offset on the directional edge */
				directional_edge_delay_for_offset = directional_edge_delay_for_whole_edge - directional_edge_delay_for_remaining_edge; //@[03/03/2009] Note that this delay for the offset is based only on vehicular traffic statistics on this edge, not real vehicular traffic condition, such as the network component consisting of vehicles for data forwarding
			} //end of if-1.3
			else //else-1.4
			//otherwise
			{
				/* obtain the vehicle's edge EDD */
				edge_EDD = pEdgeNode->head_gnode->EDD;

				/* obtain the carry delay by vehicle movement in the case where a vehicle cannot forward its packet to appropriate next carrier */
				M = pEdgeNode->weight/vehicle->speed;
			} //end of else-1.4
		} //end of if-1
		else //else-2
		//handling of EDD at the destination node where path_edge_head->vertex is NULL
		{
			/* obtain the pointer to the destination graph node */
			path_edge_tail = path_edge_head;
			node_id = atoi(path_edge_tail);
			pTailNode = &(G[node_id-1]);
			path_head_gnode = NULL;

			/* check the computation type of trajectory-based EDD */
			if(param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH) //if-2.1
			//if edd computation type is partial-path and the tail node is one of APs, then finish the EDD computation here; that is, we consider the EDD of the rest of the path just after the AP to be zero
			{
				/* check whether pTailNode->vertex is one of APs; if so, escape from this for-loop since the EDD at access point must be zero */

				flag = IsVertexInTrafficTable(ap_table, pTailNode->vertex);

				if(flag)
					break;
			} //end of if-2.1
	
			/* set the vehicle's edge EDD to INFINITE since the vehicle has arrived at its destination, so it has no next edge on its trajectory */
			edge_EDD = INF;

			/* set the carry delay by vehicle movement to zero since the vehicle has arrived at its destination, so it does not move any more according to its trajectory */
			M = 0;

			/* set the flag for the EDD processing at the destination for the case where the vehicle cannot forward its packets to the next carrier, so must carry them with itself according to its new trajectory */
			flag_for_destination = TRUE;
      
		} //end of else-2

		if(i == 0)
		{ //@Note: for the first hop, we consider two cases: (i) the case of forwarding the packet and (ii) the case of carrying the packet.

			/* compute vehicle's current EDD based on its offset on the directiona edge where it is moving */
			current_vehicle_EDD = edge_EDD - directional_edge_delay_for_offset; //the vehicle's EDD at this point

			path_EDD = 0; //initialize path_EDD
			P_ac = 1; //initialize P_ac
			P_ic = 1; //initialize P_ic

			/** process path_EDD at the destination for the case where the vehicle can forward its packets to a moving neighboring carrier */
			/* compute the contact probability beta to forward packets to another vehicle on the original carrier's trajectory edge */

			lambda = path_head_gnode->lambda; //obtain the vehicle arrival rate, assuming that lambda was updated before this function call

			beta = 1 - exp(-1*lambda*a); //this is different from the contact probability at intersection, that is, 1 - exp(-1*lambda*R/v) != 1 - exp(-1*lambda*2R/v)

			/* compute the EDD portion for this current edge when the packet can be forwarded to another carrier */
			EDD_portion = P_ac*beta*current_vehicle_EDD;

			/* update path_EDD with EDD_portion */
			path_EDD = EDD_portion;

			/* update the carry probability with beta */
			P_ic -= beta;

			/* update the accumulative carry probability P_ac with intersection carry probability P_ic */
			P_ac *= P_ic;

			/* update accumulated_carry_delay with M */
			accumulated_carry_delay = M;
		} //end of else-if-4
		else if(flag_for_destination == FALSE) //else-if-5: the current edge's head node is not the vehicle's moving destination
		{
			/* update path_EDD with accumulated_carry_delay up to this point and the accumulative carry probability P_ac */
			path_EDD += P_ac*accumulated_carry_delay; //@modified on 11/23/08; Note that path_EDD takes the movement time for the vehicle's carry length from its starting position to the current edge after updating P_ac

			current_vehicle_EDD = edge_EDD; //the vehicle's EDD at this point

			P_ic = 1; //initialize P_ic

			/* process path_EDD at the destination for the case where the vehicle can forward its packets to a moving neighboring carrier */
			pHeadNode = pTailNode; //set the pointer to head node of the outgoing edge at the intersection corresponding to pTailNode->vertex
			for(j = 0; j < pTailNode->weight; j++) //for-2
			{
				pHeadNode = pHeadNode->next;

				/** NOTE: is it correct that we use edge_EDD instead of using the vehicle's actual EDD at the intersection corresponding to pTailNode->vertex */
				//check whether the branch's EDD is greater than the EDD of the branch on the trajectory
				if(pHeadNode->EDD > edge_EDD)
					continue; //in this case, we don't use this branch for packet forwarding
				/** NOTE: When we consider all the edges' EDD, the performance is sometimes better than only the edges whose EDD are smaller than vehicle's edge EDD */

				//add the portion of EDD for the forwarding into this outgoing branch to path_EDD
				path_EDD += P_ac * pHeadNode->P_prime_pure * pHeadNode->EDD; //use pure forwarding probability for TBD
				//path_EDD += P_ac * pHeadNode->P_pure * pHeadNode->EDD; //use pure forwarding probability
				//path_EDD += P_ac * pHeadNode->P * pHeadNode->EDD; //use forwarding probability
				//path_EDD += P_ac * pHeadNode->CP * pHeadNode->EDD;

				//update the carry probability at the intersection
				P_ic -= pHeadNode->P_prime_pure; //use pure forwarding probability for TBD
				//P_ic -= pHeadNode->P_pure; //use pure forwarding probability
				//P_ic -= pHeadNode->P; //use forwarding probability
				//P_ic *= (1 - pHeadNode->CP); //(1 - pHeadNode->CP) is the probability that the vehicle cannot meet a next carrier moving on the edge <pTailNode->vertex, pHeadNode->vertex>
			} //end of for-2

			/* update the accumulative carry probability P_ac with intersection carry probability P_ic */
			P_ac *= P_ic;

			/* update accumulated_carry_delay with M towards the head intersection */
			accumulated_carry_delay += M; //the carry delay M is added to accumulated carry delay for the next carry case
		} //end of else-if-5
		else //else-6: this is for handling the last node in the vehicle trajectory
		{
			/* update path_EDD with accumulated_carry_delay up to this point and the accumulative carry probability P_ac */
			path_EDD += P_ac*accumulated_carry_delay; //@modified on 11/23/08; Note that path_EDD takes the movement time for the vehicle's carry length from its starting position to the current edge after updating P_ac

			current_vehicle_EDD = edge_EDD; //the vehicle's EDD at this point

			P_ic = 1; //initialize P_ic

			/** process path_EDD at the destination for the case where the vehicle can forward its packets to a moving neighboring carrier */
			/* set the pointer to head node of the outgoing edge at the intersection corresponding to pTailNode->vertex */
			pHeadNode = pTailNode;
			for(j = 0; j < pTailNode->weight; j++) //for-2
			{
				pHeadNode = pHeadNode->next;

				/** NOTE: is it correct that we use edge_EDD instead of using the vehicle's actual EDD at the intersection corresponding to pTailNode->vertex */
				//check whether the branch's EDD is greater than the EDD of the branch on the trajectory
				if(pHeadNode->EDD > edge_EDD)
					continue; //in this case, we don't use this branch for packet forwarding

				//add the portion of EDD for the forwarding into this outgoing branch to path_EDD
				path_EDD += P_ac * pHeadNode->P_prime_pure * pHeadNode->EDD; //use pure forwarding probability for TBD

				//update the carry probability at the intersection
				P_ic -= pHeadNode->P_prime_pure; //use pure forwarding probability for TBD
			} //end of for-2

			/* update the accumulative carry probability P_ac with intersection carry probability P_ic */
			P_ac *= P_ic;

			/** process path_EDD at the destination for the case where the vehicle cannot forward its packets to a moving neighboring carrier */
			/* compute the EDD in the case where the vehicle cannot forward its packets at the desination; that is, after its new trajectory is determined, it must carry them with itself */

			P_branch = 1/pTailNode->weight; //branch probability of the vehicle towards the next intersection after its new trajectory is determined, assuming the Random-Way-Point (RWP) mobility of the vehicle
      
			pHeadNode = pTailNode; //set the pointer to head node of the outgoing edge at the intersection corresponding to pTailNode->vertex
			for(j = 0; j < pTailNode->weight; j++) //for-2.2.1
			{
				pHeadNode = pHeadNode->next;

				//add the portion of EDD for the forwarding into this outgoing branch to path_EDD
				path_EDD += P_ac * P_branch * pHeadNode->EDD; //use the same branch probability for each directional edge from the tail node
			} //end of for-2.2.1
		} //end of else-6  

		/* let pPathNode point the head node of the next hop */
		pPathNode = pPathNode->next;

		/* check the computation type of trajectory-based EDD */
		if(param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_ONE_HOP_AHEAD && i == 1) //if edd computation type is one-hop-ahead, then finish the EDD computation here
			break;
	} //end of for-1

	/** update vehicle's EDD and EDD_SD */
	*vehicle_EDD = MIN(path_EDD, INF); //vehicle_EDD is at most INF
}

void VADD_Compute_EDD_And_EDD_SD_Based_On_TBD_For_V2V_Data_Delivery(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, int source_intersection_id, double *vehicle_EDD, double *vehicle_EDD_SD)
{ //compute the EDD and EDD_SD from source intersection to the target point for the road network graph G for V2V data delivery, based on TBD Model (i.e., Per-vehicle Model) in vehicle's current position on the current directional edge; note that this version is for TPD journal submission
	double R = param->communication_range; //communication range
	double v = param->vehicle_speed; //vehicle speed
	double path_EDD = 0; //vehicle's path EDD
	double edge_EDD = 0; //vehicle's edge EDD
	double path_EDD_VAR = 0; //vehicle's path EDD_VAR
	double edge_EDD_VAR = 0; //vehicle's edge EDD_VAR
	int i, j; //indices for for-loops
	struct_path_node *pPathNode = NULL; //pointer to path node
	char *path_edge_tail = NULL; //pointer to the tail vertex of the edge on the path
	char *path_edge_head = NULL; //pointer to the head vertex of the edge on the path
	struct_graph_node *pTailNode = NULL; //pointer to the tail graph node of an edge
	struct_graph_node *pHeadNode = NULL; //pointer to the head graph node of an edge
	directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge node of <pTailNode,pHeadNode>
	double P_ac = 0; //accumulative carry probability along the trajectory
	double P_ic = 0; //carry probability at each intersection
	double P_branch = 0; //branch probability that the vehicle will branch to a neighboring edge after the arrival at its destination
	double M = 0; //carry delay in an edge
	int node_id = 0; //node id
	int remaining_hop_count = 0; //remaining hop count from the vehicle's current position to its destination
	double offset_in_directional_edge = 0; //vehicle's offset in the directional edge where the vehicle is moving
	double directional_edge_delay_for_whole_edge = 0; //directional edge delay for the whole edge length
	double remaining_edge_length = 0; //remaining edge length for the vehicle's current directional edge
	double directional_edge_delay_for_remaining_edge = 0; //directional edge delay for the remaining edge where the vehicle is moving
	double directional_edge_delay_for_offset = 0; //directional edge delay for the offset where the vehicle has moved
	double current_vehicle_EDD = 0; //the vehicle's EDD at a future position on its trajectory 
	double current_vehicle_EDD_VAR = 0; //the vehicle's EDD_VAR at a future position on its trajectory 
	boolean flag = FALSE; //flag to indicate that the vehicle's trajectory meets one of access points 
	boolean flag_for_destination = FALSE; //flag for the EDD processing at the destination for the case where the vehicle cannot forward its packets to the next carrier, so must carry them with itself according to its new trajectory
	double accumulated_carry_delay = 0; //the accumulated carry delay for the carry length from the starting position to the current intersection on the trajectory that the original packet carrier has carried the packets without forwarding them.
	double accumulated_carry_delay_var = 0; //the accumulated carry delay variance for the carry length from the starting position to the current intersection on the trajectory that the original packet carrier has carried the packets without forwarding them.
	double lambda = 0; //vehicle arrival rate into a directional edge
	double a = R/v; //the inter-arrival bound that two consecutive vehicles can be connected with each other through communication range
	double beta = 0; //the probability that the original carrier will meet another carrier on the edge of its trajectory
	struct_graph_node *path_head_gnode = NULL; //pointer to the head graph node of the current path edge (path_edge_tail, path_edge_head) on the path
	double EDD_portion = 0; //the EDD portion for this current edge when the packet can be forwarded to another carrier 
	double carry_delay_portion = 0; //the carry delay portion for this current edge, considering the edge length or the remaining edge length
  
	/* obtain the pointer to the vehicle's current path node */
	pPathNode = vehicle->path_ptr;
 
	P_ac = 1; //initialize P_ac

	remaining_hop_count = vehicle->path_hop_count - vehicle->path_current_hop; //compute the remaining hop count from the vehicle's current position to its destination

	/**@ for debugging */
#if 0 /* [ */
	if(vehicle->id == 1 && vehicle->state_time >= 3599)
	{
		printf("VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(): at time=%.1f, vehicle(id=%d) is traced\n", (float)vehicle->state_time, vehicle->id);
	}
#endif 

	if(remaining_hop_count == 0)
	{
		printf("%s:%d: remaining_hop_count is zero!\n",
				__FUNCTION__, __LINE__);
		//fgetc(stdin);
	}
	/*******************/

	for(i = 0; i <= remaining_hop_count; i++) //for-1
	{
		if(i < remaining_hop_count) //if-1
		{
			path_edge_tail = pPathNode->vertex;
			path_edge_head = pPathNode->next->vertex;

			/* obtain the pointer to the direaction edge of <tail_node,head_node> */
			pEdgeNode = FastLookupDirectionalEdgeQueue(G, path_edge_tail, path_edge_head);
			if(pEdgeNode == NULL) //if-1.1
			{
				printf("%s:%d: pEdgeNode for <%s,%s> is NULL\n", 
						__FUNCTION__, __LINE__, 
						path_edge_tail, path_edge_head);
				exit(1);
			} //end of if-1.1

			/* set the pointer to tail node of the outgoing edge at the intersection corresponding to intermediate intersection, pointed by pEdgeNode->tail_gnode->vertex */
			pTailNode = pEdgeNode->tail_gnode;

			/* set the pointer to head node of the outgoing edge */
			path_head_gnode = pEdgeNode->head_gnode;

#if 0 /* [ */
			/* check the computation type of trajectory-based EDD */
			if(i != 0 && param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH) //if-1.2: Note that the condition (i != 0) lets the vehicle moving from one of APs have positive EDD
			//if edd computation type is partial-path and the tail node is one of APs, then finish the EDD computation here
			{
				/* check whether pTailNode->vertex is one of APs; if so, escape from this for-loop since the EDD at access point must be zero; that is, we consider the EDD of the rest of the path just after the AP to be zero. */
				
				flag = IsVertexInTrafficTable(ap_table, pTailNode->vertex);
				if(flag)
					break;
			} //end of if-1.2
#endif /* ] */

			/* obtain the edge EDD and compute the vehicle movement time for the remaining edge */
			if(i == 0) //if-1.3
			//if this is the first hop
			{
				/* obtain the vehicle's edge EDD */
				edge_EDD = pEdgeNode->head_gnode->EDD;

				/* obtain the vehicle's edge EDD_VAR */
				edge_EDD_VAR = pEdgeNode->head_gnode->EDD_VAR;

				/*@for debugging: we use temporarily the square root of edge_EDD_VAR for the vehicle's current edge as its vehicle_EDD_SD */
				*vehicle_EDD_SD = sqrt(edge_EDD_VAR);

				/* obtain the carry delay by vehicle movement in the case where a vehicle cannot forward its packet to appropriate next carrier */
				offset_in_directional_edge = vehicle->current_pos_in_digraph.offset;
				remaining_edge_length = vehicle->edge_length - offset_in_directional_edge;
				M = remaining_edge_length/vehicle->speed; //movement time for the remaining edge length

				/* compute the estimated directional edge delay for the whole edge length */
				directional_edge_delay_for_whole_edge = pEdgeNode->head_gnode->edge_delay;

				/* compute the estimated directional edge delay for the remaining edge length */
				directional_edge_delay_for_remaining_edge = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, remaining_edge_length); //@[03/03/2009] For version iTBD-v2.1.0, uncomment this line to let EDD correct in one road segment
        
				/* Note that the above gives the very similar performance as the below (i.e., the same packet loss, but a little longer delay with 1 second), but logically the below makes sense since the vehicle should carry packets in order to compute path_EDD related to the vehicle's path */
				//directional_edge_delay_for_remaining_edge = M; //for version v1.3.9

				/** compute the estimated directional edge delay for the current offset on the directional edge */
				directional_edge_delay_for_offset = directional_edge_delay_for_whole_edge - directional_edge_delay_for_remaining_edge; //@[03/03/2009] Note that this delay for the offset is based only on vehicular traffic statistics on this edge, not real vehicular traffic condition, such as the network component consisting of vehicles for data forwarding
			} //end of if-1.3
			else //else-1.4
			//otherwise
			{
				/* obtain the vehicle's edge EDD */
				edge_EDD = pEdgeNode->head_gnode->EDD;

				/* obtain the carry delay by vehicle movement in the case where a vehicle cannot forward its packet to appropriate next carrier */
				M = pEdgeNode->weight/vehicle->speed;
			} //end of else-1.4
		} //end of if-1
		else //else-2
		//handling of EDD at the destination node where path_edge_head->vertex is NULL
		{
			/* obtain the pointer to the destination graph node */
			path_edge_tail = path_edge_head;
			node_id = atoi(path_edge_tail);
			pTailNode = &(G[node_id-1]);
			path_head_gnode = NULL;

#if 0 /* [ */			
			/* check the computation type of trajectory-based EDD */
			if(param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH) //if-2.1
			//if edd computation type is partial-path and the tail node is one of APs, then finish the EDD computation here; that is, we consider the EDD of the rest of the path just after the AP to be zero
			{
				/* check whether pTailNode->vertex is one of APs; if so, escape from this for-loop since the EDD at access point must be zero */

				flag = IsVertexInTrafficTable(ap_table, pTailNode->vertex);

				if(flag)
					break;
			} //end of if-2.1
#endif /* ] */
	
			/* set the vehicle's edge EDD to INFINITE since the vehicle has arrived at its destination, so it has no next edge on its trajectory */
			edge_EDD = INF;
			/**@NOTE: [TPD] It seems like this INF edge_EDD makes EDD have a much larger value than the EDD by VADD. */

			/* set the carry delay by vehicle movement to zero since the vehicle has arrived at its destination, so it does not move any more according to its trajectory */
			M = 0;

			/* set the flag for the EDD processing at the destination for the case where the vehicle cannot forward its packets to the next carrier, so must carry them with itself according to its new trajectory */
			flag_for_destination = TRUE;
      
		} //end of else-2

		if(i == 0)
		{ //@Note: for the first hop, we consider two cases: (i) the case of forwarding the packet and (ii) the case of carrying the packet.

			/* compute vehicle's current EDD based on its offset on the directiona edge where it is moving */
			current_vehicle_EDD = edge_EDD - directional_edge_delay_for_offset; //the vehicle's EDD at this point

			path_EDD = 0; //initialize path_EDD
			P_ac = 1; //initialize P_ac
			P_ic = 1; //initialize P_ic

			/** process path_EDD at the destination for the case where the vehicle can forward its packets to a moving neighboring carrier */
			/* compute the contact probability beta to forward packets to another vehicle on the original carrier's trajectory edge */

			lambda = path_head_gnode->lambda; //obtain the vehicle arrival rate, assuming that lambda was updated before this function call

			beta = 1 - exp(-1*lambda*a); //this is different from the contact probability at intersection, that is, 1 - exp(-1*lambda*R/v) != 1 - exp(-1*lambda*2R/v)

			/* compute the EDD portion for this current edge when the packet can be forwarded to another carrier */
			EDD_portion = P_ac*beta*current_vehicle_EDD;

			/* update path_EDD with EDD_portion */
			path_EDD = EDD_portion;

			/* update the carry probability with beta */
			P_ic -= beta;

			/* update the accumulative carry probability P_ac with intersection carry probability P_ic */
			P_ac *= P_ic;

			/* update accumulated_carry_delay with M */
			accumulated_carry_delay = M;
		} //end of else-if-4
		else if(flag_for_destination == FALSE) //else-if-5: the current edge's head node is not the vehicle's moving destination
		{
			/* update path_EDD with accumulated_carry_delay up to this point and the accumulative carry probability P_ac */
			path_EDD += P_ac*accumulated_carry_delay; //@modified on 11/23/08; Note that path_EDD takes the movement time for the vehicle's carry length from its starting position to the current edge after updating P_ac

			current_vehicle_EDD = edge_EDD; //the vehicle's EDD at this point

			P_ic = 1; //initialize P_ic

			/* process path_EDD at the destination for the case where the vehicle can forward its packets to a moving neighboring carrier */
			pHeadNode = pTailNode; //set the pointer to head node of the outgoing edge at the intersection corresponding to pTailNode->vertex
			for(j = 0; j < pTailNode->weight; j++) //for-2
			{
				pHeadNode = pHeadNode->next;

				/** NOTE: is it correct that we use edge_EDD instead of using the vehicle's actual EDD at the intersection corresponding to pTailNode->vertex */
				//check whether the branch's EDD is greater than the EDD of the branch on the trajectory
				if(pHeadNode->EDD > edge_EDD)
					continue; //in this case, we don't use this branch for packet forwarding
				/** NOTE: When we consider all the edges' EDD, the performance is sometimes better than only the edges whose EDD are smaller than vehicle's edge EDD */

				//add the portion of EDD for the forwarding into this outgoing branch to path_EDD
				path_EDD += P_ac * pHeadNode->P_prime_pure * pHeadNode->EDD; //use pure forwarding probability for TBD
				//path_EDD += P_ac * pHeadNode->P_pure * pHeadNode->EDD; //use pure forwarding probability
				//path_EDD += P_ac * pHeadNode->P * pHeadNode->EDD; //use forwarding probability
				//path_EDD += P_ac * pHeadNode->CP * pHeadNode->EDD;

				//update the carry probability at the intersection
				P_ic -= pHeadNode->P_prime_pure; //use pure forwarding probability for TBD
				//P_ic -= pHeadNode->P_pure; //use pure forwarding probability
				//P_ic -= pHeadNode->P; //use forwarding probability
				//P_ic *= (1 - pHeadNode->CP); //(1 - pHeadNode->CP) is the probability that the vehicle cannot meet a next carrier moving on the edge <pTailNode->vertex, pHeadNode->vertex>
			} //end of for-2

			/* update the accumulative carry probability P_ac with intersection carry probability P_ic */
			P_ac *= P_ic;

			/* update accumulated_carry_delay with M towards the head intersection */
			accumulated_carry_delay += M; //the carry delay M is added to accumulated carry delay for the next carry case
		} //end of else-if-5
		else //else-6: this is for handling the last node in the vehicle trajectory
		{
			/* update path_EDD with accumulated_carry_delay up to this point and the accumulative carry probability P_ac */
			path_EDD += P_ac*accumulated_carry_delay; //@modified on 11/23/08; Note that path_EDD takes the movement time for the vehicle's carry length from its starting position to the current edge after updating P_ac

			current_vehicle_EDD = edge_EDD; //the vehicle's EDD at this point

			P_ic = 1; //initialize P_ic

			/** process path_EDD at the destination for the case where the vehicle can forward its packets to a moving neighboring carrier */
			/* set the pointer to head node of the outgoing edge at the intersection corresponding to pTailNode->vertex */
			pHeadNode = pTailNode;
			for(j = 0; j < pTailNode->weight; j++) //for-2
			{
				pHeadNode = pHeadNode->next;

				/** NOTE: is it correct that we use edge_EDD instead of using the vehicle's actual EDD at the intersection corresponding to pTailNode->vertex */
				//check whether the branch's EDD is greater than the EDD of the branch on the trajectory
				if(pHeadNode->EDD > edge_EDD)
					continue; //in this case, we don't use this branch for packet forwarding

				//add the portion of EDD for the forwarding into this outgoing branch to path_EDD
				path_EDD += P_ac * pHeadNode->P_prime_pure * pHeadNode->EDD; //use pure forwarding probability for TBD

				//update the carry probability at the intersection
				P_ic -= pHeadNode->P_prime_pure; //use pure forwarding probability for TBD
			} //end of for-2

			/* update the accumulative carry probability P_ac with intersection carry probability P_ic */
			P_ac *= P_ic;

			/** process path_EDD at the destination for the case where the vehicle cannot forward its packets to a moving neighboring carrier */
			/* compute the EDD in the case where the vehicle cannot forward its packets at the desination; that is, after its new trajectory is determined, it must carry them with itself */

			P_branch = 1/pTailNode->weight; //branch probability of the vehicle towards the next intersection after its new trajectory is determined, assuming the Random-Way-Point (RWP) mobility of the vehicle
      
			pHeadNode = pTailNode; //set the pointer to head node of the outgoing edge at the intersection corresponding to pTailNode->vertex
			for(j = 0; j < pTailNode->weight; j++) //for-2.2.1
			{
				pHeadNode = pHeadNode->next;

				//add the portion of EDD for the forwarding into this outgoing branch to path_EDD
				path_EDD += P_ac * P_branch * pHeadNode->EDD; //use the same branch probability for each directional edge from the tail node
			} //end of for-2.2.1
		} //end of else-6  

		/* let pPathNode point the head node of the next hop */
		pPathNode = pPathNode->next;

		/* check the computation type of trajectory-based EDD */
		if(param->vehicle_vanet_tbd_edd_computation_type == VANET_EDD_BASED_ON_TBD_WITH_ONE_HOP_AHEAD && i == 1) //if edd computation type is one-hop-ahead, then finish the EDD computation here
			break;
	} //end of for-1

	/** update vehicle's EDD and EDD_SD */
	*vehicle_EDD = MIN(path_EDD, INF); //vehicle_EDD is at most INF
}

/**************************************************************/

void VADD_Compute_EDD_And_EDD_SD_Based_On_VADD(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD)
{ //compute the EDD and EDD_SD based on VADD Model (i.e., Per-intersection Model) in vehicle's current position on the current directional edge
  VADD_Compute_EDD_And_EDD_SD_Based_On_VADD_VERSION_2(param, vehicle, G, G_size, ap_table, vehicle_EDD, vehicle_EDD_SD); //Version 2 gives a more accurate EDD than Version 3
  //VADD_Compute_EDD_And_EDD_SD_Based_On_VADD_VERSION_3(param, vehicle, G, G_size, ap_table, vehicle_EDD, vehicle_EDD_SD);
}

void VADD_Compute_EDD_And_EDD_SD_Based_On_VADD_VERSION_1(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD)
{ //compute the EDD and EDD_SD based on VADD Model (i.e., Per-intersection Model) in vehicle's current position on the current directional edge
  double directional_edge_EDD = 0; //directional edge's EDD
  double directional_edge_EDD_VAR = 0; //directional edge's EDD_VAR
  double vehicle_EDD_VAR = 0; //vehicle's EDD_VAR
  char *tail_node = vehicle->path_ptr->vertex; //tail node of the directional edge where vehicle is moving
  char *head_node = vehicle->path_ptr->next->vertex; //head node of the directional edge where vehicle is moving
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
  double offset_in_undirectional_edge = 0; //vehicle's offset on the undirectional edge
  double offset_in_directional_edge = 0; //vehicle's offset on the directional edge
  double remaining_edge_length = 0; //remaining edge length for the vehicle
  double directional_edge_delay = 0; //directional edge delay
  double directional_edge_delay_sd = 0; //directional edge delay's standard deviation

  /** obtain the pointer to the direaction edge of <tail_node,head_node> */
  pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
  if(pEdgeNode == NULL)
  {
    printf("VADD_Compute_VADD_Based_EDD_SD(): pEdgeNode for <%s,%s> is NULL\n", tail_node, head_node);
    exit(1);
  }

  /** obtain the directional edge's EDD and EDD_VAR */
  directional_edge_EDD = pEdgeNode->head_gnode->EDD;
  directional_edge_EDD_VAR = pEdgeNode->head_gnode->EDD_VAR;

  /** compute the offset in directional edge */
  offset_in_undirectional_edge = vehicle->current_pos_in_Gr.offset;
  if(vehicle->move_type == MOVE_FORWARD)
  {
    offset_in_directional_edge = offset_in_undirectional_edge;
  }    
  else if(vehicle->move_type == MOVE_BACKWARD)
  {
    offset_in_directional_edge = vehicle->edge_length - offset_in_undirectional_edge;
  }
  else
  {
    printf("VADD_Compute_VADD_Based_EDD_SD(): vehicle->move_type() is invalid\n", vehicle->move_type);
    exit(1);
  }

  /** compute the estimated directional edge delay and its standard deviation for the vehicle's moving distance on the edge */
  directional_edge_delay = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, offset_in_directional_edge);
  directional_edge_delay_sd = VADD_Compute_Subedge_Delay_Standard_Deviation(param, pEdgeNode->head_gnode, offset_in_directional_edge);

  /** adjust vehicle's EDD and EDD_SD from directional edge's EDD and EDD_SD, respectively */
  *vehicle_EDD = MAX(0, directional_edge_EDD - directional_edge_delay);
  *vehicle_EDD = MIN(*vehicle_EDD, INF); //[10/26/09] vehicle_EDD is at most INF
  //*vehicle_EDD = MAX(0, directional_edge_EDD - directional_edge_delay);

  vehicle_EDD_VAR = MAX(0, directional_edge_EDD_VAR - pow(directional_edge_delay_sd, 2));
  //the E2E expected delivery delay variance is the expected sum of the edge delay standard deviations on the E2E path
  *vehicle_EDD_SD = sqrt(vehicle_EDD_VAR);
  *vehicle_EDD_SD = MIN(*vehicle_EDD_SD, INF); //[10/26/09] vehicle_EDD_SD is at most INF
}

void VADD_Compute_EDD_And_EDD_SD_Based_On_VADD_VERSION_2(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD)
{ //compute the EDD and EDD_SD based on VADD Model (i.e., Per-intersection Model) in vehicle's current position on the current directional edge; note that this version is for TBD journal submission
	double directional_edge_EDD = 0; //directional edge's EDD
	double directional_edge_EDD_VAR = 0; //directional edge's EDD_VAR
	double vehicle_EDD_VAR = 0; //vehicle's EDD_VAR
	char *tail_node = vehicle->path_ptr->vertex; //tail node of the directional edge where vehicle is moving
	char *head_node = vehicle->path_ptr->next->vertex; //head node of the directional edge where vehicle is moving
	directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
	double offset_in_undirectional_edge = 0; //vehicle's offset on the undirectional edge
	double offset_in_directional_edge = 0; //vehicle's offset on the directional edge
	double remaining_edge_length = 0; //remaining edge length for the vehicle
	double directional_edge_delay = 0; //directional edge delay
	double directional_edge_delay_variance = 0; //directional edge delay variance
	double directional_subedge_delay = 0; //directional subedge delay for the remaining edge length for the vehicle to travel
	double directional_subedge_delay_variance = 0; //directional subedge delay variance for the remaining edge length for the vehicle to travel
	double directional_subedge_delay_sd = 0; //directional subedge delay standard deviation for the remaining edge length for the vehicle to travel
	double edge_delay_difference = 0; //the difference between directional edge delay and directional subedge delay
	double edge_delay_variance_difference = 0; //the difference between directional edge delay variance and directional subedge delay variance

	/** obtain the pointer to the direaction edge of <tail_node,head_node> */
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
	if(pEdgeNode == NULL)
	{
		printf("VADD_Compute_VADD_Based_EDD_SD(): pEdgeNode for <%s,%s> is NULL\n", tail_node, head_node);
		exit(1);
	}

	/** obtain the directional edge's EDD, EDD_VAR, edge_delay, and edge_delay_variance */
	directional_edge_EDD = pEdgeNode->head_gnode->EDD;
	directional_edge_EDD_VAR = pEdgeNode->head_gnode->EDD_VAR;

	directional_edge_delay = pEdgeNode->head_gnode->edge_delay;
	directional_edge_delay_variance = pEdgeNode->head_gnode->edge_delay_variance;

	/** compute the offset in directional edge */
	offset_in_undirectional_edge = vehicle->current_pos_in_Gr.offset;
	if(vehicle->move_type == MOVE_FORWARD)
	{
		offset_in_directional_edge = offset_in_undirectional_edge;
	}    
	else if(vehicle->move_type == MOVE_BACKWARD)
	{
		offset_in_directional_edge = vehicle->edge_length - offset_in_undirectional_edge;
	}
	else
	{
		printf("VADD_Compute_VADD_Based_EDD_SD(): vehicle->move_type() is invalid\n", vehicle->move_type);
		exit(1);
	}

	/** compute the remaining edge length to travel */
	remaining_edge_length = pEdgeNode->weight - offset_in_directional_edge;

	/** compute the estimated directional edge delay and its standard deviation for the vehicle's remaining edge length */
	directional_subedge_delay = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, remaining_edge_length);
	//directional_edge_delay = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, offset_in_directional_edge);

	directional_subedge_delay_sd = VADD_Compute_Subedge_Delay_Standard_Deviation(param, pEdgeNode->head_gnode, remaining_edge_length);
	//directional_edge_delay_sd = VADD_Compute_Subedge_Delay_Standard_Deviation(param, pEdgeNode->head_gnode, offset_in_directional_edge);

	/** compute the delay difference and the delay variance difference */
	edge_delay_difference = directional_edge_delay - directional_subedge_delay;

	directional_subedge_delay_variance = directional_subedge_delay_sd * directional_subedge_delay_sd; 
	edge_delay_variance_difference = directional_edge_delay_variance - directional_subedge_delay_variance;

	/** adjust vehicle's EDD and EDD_SD from directional edge's EDD and EDD_SD, respectively */
	*vehicle_EDD = MAX(0, directional_edge_EDD - edge_delay_difference);
	*vehicle_EDD = MIN(*vehicle_EDD, INF); //[10/26/09] vehicle_EDD is at most INF
	//*vehicle_EDD = MAX(0, directional_edge_EDD - directional_edge_delay);

	vehicle_EDD_VAR = MAX(0, directional_edge_EDD_VAR - edge_delay_variance_difference);
	//vehicle_EDD_VAR = MAX(0, directional_edge_EDD_VAR - pow(directional_edge_delay_sd, 2));
	//the E2E expected delivery delay variance is the expected sum of the edge delay standard deviations on the E2E path
	*vehicle_EDD_SD = sqrt(vehicle_EDD_VAR);
	*vehicle_EDD_SD = MIN(*vehicle_EDD_SD, INF); //[10/26/09] vehicle_EDD_SD is at most INF
}

void VADD_Compute_EDD_And_EDD_SD_Based_On_VADD_VERSION_3(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_traffic_table *ap_table, double *vehicle_EDD, double *vehicle_EDD_SD)
{ //compute the EDD and EDD_SD based on VADD Model (i.e., Per-intersection Model) in vehicle's current position on the current directional edge; [10/17/09] note that this version uses the link delays and link delay deviations from the current road segment and the next intersection
  double directional_edge_EDD = 0; //directional edge's EDD
  double directional_edge_EDD_VAR = 0; //directional edge's EDD_VAR
  double vehicle_EDD_VAR = 0; //vehicle's EDD_VAR
  char *tail_node = vehicle->path_ptr->vertex; //tail node of the directional edge where vehicle is moving
  char *head_node = vehicle->path_ptr->next->vertex; //head node of the directional edge where vehicle is moving
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
  double offset_in_undirectional_edge = 0; //vehicle's offset on the undirectional edge
  double offset_in_directional_edge = 0; //vehicle's offset on the directional edge
  double remaining_edge_length = 0; //remaining edge length for the vehicle
  double link_delay = 0; //link delay for the subedge
  double link_delay_deviation = 0; //link delay standard deviation
  double link_delay_variance = 0; //link delay variance
  double EDD = 0; //E2E delivery delay from head_node to the destination
  double EDD_VAR = 0; //E2E delivery delay variance 

  /** obtain the pointer to the direaction edge of <tail_node,head_node> */
  pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
  if(pEdgeNode == NULL)
  {
    printf("VADD_Compute_VADD_Based_EDD_SD(): pEdgeNode for <%s,%s> is NULL\n", tail_node, head_node);
    exit(1);
  }

  /** obtain the directional edge's EDD, EDD_VAR, edge_delay, and edge_delay_variance */
/*   directional_edge_EDD = pEdgeNode->head_gnode->EDD; */
/*   directional_edge_EDD_VAR = pEdgeNode->head_gnode->EDD_VAR; */

/*   directional_edge_delay = pEdgeNode->head_gnode->edge_delay; */
/*   directional_edge_delay_variance = pEdgeNode->head_gnode->edge_delay_variance; */

  EDD = pEdgeNode->head_gnode->gnode->EDD; //EDD at head_node
  EDD_VAR = pEdgeNode->head_gnode->gnode->EDD_VAR; //EDD_VAR at head_node

  /** compute the offset in directional edge */
  offset_in_undirectional_edge = vehicle->current_pos_in_Gr.offset;
  if(vehicle->move_type == MOVE_FORWARD)
  {
    offset_in_directional_edge = offset_in_undirectional_edge;
  }    
  else if(vehicle->move_type == MOVE_BACKWARD)
  {
    offset_in_directional_edge = vehicle->edge_length - offset_in_undirectional_edge;
  }
  else
  {
    printf("VADD_Compute_VADD_Based_EDD_SD_Based_On_VADD(): vehicle->move_type() is invalid\n", vehicle->move_type);
    exit(1);
  }

  /** compute the remaining edge length to travel */
  remaining_edge_length = pEdgeNode->weight - offset_in_directional_edge;

  /** compute the estimated directional edge delay and its standard deviation for the vehicle's remaining edge length */
  link_delay = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, remaining_edge_length);
  //directional_edge_delay = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, remaining_edge_length);

  link_delay_deviation = VADD_Compute_Subedge_Delay_Standard_Deviation(param, pEdgeNode->head_gnode, remaining_edge_length);
  //directional_edge_delay_sd = VADD_Compute_Subedge_Delay_Standard_Deviation(param, pEdgeNode->head_gnode, remaining_edge_length);

  link_delay_variance = pow(link_delay_deviation, 2); //compute link delay variance

  /** compute the delay difference and the delay variance difference */
  //edge_delay_difference = directional_edge_delay - directional_subedge_delay;

  //directional_subedge_delay_variance = directional_subedge_delay_sd * directional_subedge_delay_sd; 
  //edge_delay_variance_difference = directional_edge_delay_variance - directional_subedge_delay_variance;

  /** set up vehicle's EDD and EDD_VAR with EDD + link delay and EDD_VAR + link delay variance, respectively */
  *vehicle_EDD = EDD + link_delay; 
  *vehicle_EDD = MIN(*vehicle_EDD, INF); //[10/26/09] vehicle_EDD is at most INF
  //*vehicle_EDD = MAX(0, directional_edge_EDD - edge_delay_difference);

  vehicle_EDD_VAR = EDD_VAR + link_delay_variance;
  //vehicle_EDD_VAR = MAX(0, directional_edge_EDD_VAR - edge_delay_variance_difference);
  //the E2E expected delivery delay variance is the expected sum of the edge delay standard deviations on the E2E path

  *vehicle_EDD_SD = sqrt(vehicle_EDD_VAR);
  *vehicle_EDD_SD = MIN(*vehicle_EDD_SD, INF); //[10/26/09] vehicle_EDD_SD is at most INF
}

void VADD_Compute_EDD_And_EDD_SD_Based_On_VADD_For_V2V_Data_Delivery(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, int source_intersection_id, double *vehicle_EDD, double *vehicle_EDD_SD)
{ //compute the EDD and EDD_SD from source intersection to the target point for the road network graph G for V2V data delivery, based on VADD Model (i.e., Per-intersection Model) in vehicle's current position on the current directional edge; note that this version is for TPD journal submission and this function is the same as VADD_Compute_EDD_And_EDD_SD_Based_On_VADD_VERSION_2() except the parameter list
	double directional_edge_EDD = 0; //directional edge's EDD
	double directional_edge_EDD_VAR = 0; //directional edge's EDD_VAR
	double vehicle_EDD_VAR = 0; //vehicle's EDD_VAR
	char *tail_node = vehicle->path_ptr->vertex; //tail node of the directional edge where vehicle is moving
	char *head_node = vehicle->path_ptr->next->vertex; //head node of the directional edge where vehicle is moving
	directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
	double offset_in_undirectional_edge = 0; //vehicle's offset on the undirectional edge
	double offset_in_directional_edge = 0; //vehicle's offset on the directional edge
	double remaining_edge_length = 0; //remaining edge length for the vehicle
	double directional_edge_delay = 0; //directional edge delay
	double directional_edge_delay_variance = 0; //directional edge delay variance
	double directional_subedge_delay = 0; //directional subedge delay for the remaining edge length for the vehicle to travel
	double directional_subedge_delay_variance = 0; //directional subedge delay variance for the remaining edge length for the vehicle to travel
	double directional_subedge_delay_sd = 0; //directional subedge delay standard deviation for the remaining edge length for the vehicle to travel
	double edge_delay_difference = 0; //the difference between directional edge delay and directional subedge delay
	double edge_delay_variance_difference = 0; //the difference between directional edge delay variance and directional subedge delay variance

	/** obtain the pointer to the direaction edge of <tail_node,head_node> */
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
	if(pEdgeNode == NULL)
	{
		printf("VADD_Compute_VADD_Based_EDD_SD(): pEdgeNode for <%s,%s> is NULL\n", tail_node, head_node);
		exit(1);
	}

	/** obtain the directional edge's EDD, EDD_VAR, edge_delay, and edge_delay_variance */
	directional_edge_EDD = pEdgeNode->head_gnode->EDD;
	directional_edge_EDD_VAR = pEdgeNode->head_gnode->EDD_VAR;

	directional_edge_delay = pEdgeNode->head_gnode->edge_delay;
	directional_edge_delay_variance = pEdgeNode->head_gnode->edge_delay_variance;

	/** compute the offset in directional edge */
	offset_in_undirectional_edge = vehicle->current_pos_in_Gr.offset;
	if(vehicle->move_type == MOVE_FORWARD)
	{
		offset_in_directional_edge = offset_in_undirectional_edge;
	}    
	else if(vehicle->move_type == MOVE_BACKWARD)
	{
		offset_in_directional_edge = vehicle->edge_length - offset_in_undirectional_edge;
	}
	else
	{
		printf("VADD_Compute_VADD_Based_EDD_SD(): vehicle->move_type() is invalid\n", vehicle->move_type);
		exit(1);
	}

	/** compute the remaining edge length to travel */
	remaining_edge_length = pEdgeNode->weight - offset_in_directional_edge;

	/** compute the estimated directional edge delay and its standard deviation for the vehicle's remaining edge length */
	directional_subedge_delay = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, remaining_edge_length);
	//directional_edge_delay = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, offset_in_directional_edge);

	directional_subedge_delay_sd = VADD_Compute_Subedge_Delay_Standard_Deviation(param, pEdgeNode->head_gnode, remaining_edge_length);
	//directional_edge_delay_sd = VADD_Compute_Subedge_Delay_Standard_Deviation(param, pEdgeNode->head_gnode, offset_in_directional_edge);

	/** compute the delay difference and the delay variance difference */
	edge_delay_difference = directional_edge_delay - directional_subedge_delay;

	directional_subedge_delay_variance = directional_subedge_delay_sd * directional_subedge_delay_sd; 
	edge_delay_variance_difference = directional_edge_delay_variance - directional_subedge_delay_variance;

	/** adjust vehicle's EDD and EDD_SD from directional edge's EDD and EDD_SD, respectively */
	*vehicle_EDD = MAX(0, directional_edge_EDD - edge_delay_difference);
	*vehicle_EDD = MIN(*vehicle_EDD, INF); //[10/26/09] vehicle_EDD is at most INF
	//*vehicle_EDD = MAX(0, directional_edge_EDD - directional_edge_delay);

	vehicle_EDD_VAR = MAX(0, directional_edge_EDD_VAR - edge_delay_variance_difference);
	//vehicle_EDD_VAR = MAX(0, directional_edge_EDD_VAR - pow(directional_edge_delay_sd, 2));
	//the E2E expected delivery delay variance is the expected sum of the edge delay standard deviations on the E2E path
	*vehicle_EDD_SD = sqrt(vehicle_EDD_VAR);
	*vehicle_EDD_SD = MIN(*vehicle_EDD_SD, INF); //[10/26/09] vehicle_EDD_SD is at most INF
}

void VADD_Compute_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(parameter_t *param, int target_point_id, char *AP_vertex, forwarding_table_queue_t *FTQ, double *AP_EDD, double *AP_EDD_SD)
{ //compute the EDD and EDD_SD for a target point towards a destination vehicle, based on EDD Computation Model (i.e., Stochastic Model or Shortest Path Model) from an intersection having an access point, that is, from AP's gnode.

    /* Note that we compute the EDD and EDD_SD for each intersection only where VANET E2E Delay Model is Stochastic Model, because with VANET E2E Delay Model of Shortest Path Model, the EDD and EDD_SD of each intersection has been computed at VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model_For_Shortest_EDD */
    if(param->vehicle_vanet_edd_computation_model == VANET_EDD_COMPUTATION_BASED_ON_STOCHASTIC_MODEL)
        VADD_Compute_EDD_And_EDD_SD_For_TargetPoint_At_Intersection_Based_On_Stochastic_Model(param, target_point_id, AP_vertex, FTQ, AP_EDD, AP_EDD_SD);
}

void VADD_Compute_EDD_And_EDD_SD_For_TargetPoint_At_Intersection_Based_On_Stochastic_Model(parameter_t *param, int target_point_id, char *AP_vertex, forwarding_table_queue_t *FTQ, double *AP_EDD, double *AP_EDD_SD)
{ //compute the EDD and EDD_SD for a target point towards a destination vehicle, based on VADD Stochastic Model (i.e., Per-intersection Model) from an intersection having an access point, that is, from AP's gnode.
  int AP_vertex_id = atoi(AP_vertex); //id corresponding to the vertex having AP
  double EDD = 0; //EDD at intersection
  double EDD_VAR = 0; //EDD_VAR at intersection
  double EDD_SD = 0; //EDD_SD at intersection
  double forwarding_probability = 0 ; //forwarding probability for an outgoing edge
  double directional_edge_EDD = 0; //directional edge's EDD
  double directional_edge_EDD_VAR = 0; //directional edge's EDD_VAR
  char *tail_node = NULL; //tail node of the directional edge where vehicle is moving
  char *head_node = NULL; //head node of the directional edge where vehicle is moving
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
  struct_graph_node *G = NULL; //pointer to graph G for target_point_id
  int G_size = 0; //size of graph G
  struct_graph_node *intersection_gnode = NULL; //pointer to a graph node corresponding to intersection_id
  struct_graph_node *neighbor_gnode = NULL; //pointer to a neighbor graph node
  int i = 0; //index for for-loop
  int neighbor_number = 0; //number of neighbors for the intersection point having the AP

  /** get the graph for the target point from forwarding table queue FTQ */
  G = FTQ->index_table[target_point_id-1]->G;
  G_size = FTQ->size;

  /** get the pointer to the graph node corresponding to intersection_id */
  if(AP_vertex_id > G_size)
  {
		printf("%s:%d: Error: AP_vertex_id(%d) cannot be greater than size(%d)\n", 
				__FUNCTION__, __LINE__,
				AP_vertex_id, G_size);
		exit(1);
  }
  intersection_gnode = &(G[AP_vertex_id-1]);

  /** get the number of neighbors of the intersection_gnode */
  neighbor_number = (int) intersection_gnode->weight;

  /** compute the EDD and EDD_SD at the intersection */
  tail_node = intersection_gnode->vertex;
  neighbor_gnode = intersection_gnode;
  for(i = 0; i < neighbor_number; i++ )
  {
    /* get the pointer to the i-th neighbor gnode */
    neighbor_gnode = neighbor_gnode->next;

    /* set head_node */
    head_node = neighbor_gnode->vertex;

    /* obtain the pointer to the direaction edge of <tail_node,head_node> */
    pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
    if(pEdgeNode == NULL)
    {
      printf("%s:%d: pEdgeNode for <%s,%s> is NULL\n", 
			  __FUNCTION__, __LINE__,
			  tail_node, head_node);
      exit(1);
    }

    /* obtain the directional edge's EDD, EDD_VAR, and forwarding probability */
    directional_edge_EDD = pEdgeNode->head_gnode->EDD;
    directional_edge_EDD_VAR = pEdgeNode->head_gnode->EDD_VAR;
    forwarding_probability = pEdgeNode->head_gnode->P_prime;
    //@Note that forwarding_probability should be enhanced considering the waiting time in the case where the packet cannot be forwarded to any carrier around this intersection

    /* update EDD and EDD_VAR with directional_edge_EDD, directional_edge_EDD_VAR and forwarding_probability */
    EDD += directional_edge_EDD * forwarding_probability;

    EDD_VAR += directional_edge_EDD_VAR * pow(forwarding_probability, 2);
    //EDD_VAR += directional_edge_EDD_VAR * forwarding_probability;

  }

  /* compute EDD_SD for this intersection */
  EDD_SD = sqrt(EDD_VAR);

  /* set AP_gnode's EDD and EDD_SD to EDD and EDD_SD, respectively */
  intersection_gnode->EDD = EDD;
  intersection_gnode->EDD_SD = EDD_SD;

  /* set AP_EDD and AP_EDD_SD to EDD and EDD_SD, respectively */
  *AP_EDD = EDD;
  *AP_EDD_SD = EDD_SD; 
}

void VADD_Compute_EDD_And_EDD_SD_For_TargetPoint_At_Intersection_For_V2V_Data_Delivery(parameter_t *param, int target_point_id, int source_intersection_id, struct_graph_node *Gr, int Gr_size, double *E2E_EDD, double *E2E_EDD_SD)
{ //compute the E2E EDD and EDD_SD fromfor a target point towards a destination vehicle, based on VADD Stochastic Model (i.e., Per-intersection Model) from an intersection having an access point, that is, from AP's gnode.
	double EDD = 0; //EDD at intersection
	double EDD_VAR = 0; //EDD_VAR at intersection
	double EDD_SD = 0; //EDD_SD at intersection
	double forwarding_probability = 0 ; //forwarding probability for an outgoing edge
	double directional_edge_EDD = 0; //directional edge's EDD
	double directional_edge_EDD_VAR = 0; //directional edge's EDD_VAR
	char *tail_node = NULL; //tail node of the directional edge where vehicle is moving
	char *head_node = NULL; //head node of the directional edge where vehicle is moving
	directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
	struct_graph_node *intersection_gnode = NULL; //pointer to a graph node corresponding to intersection_id
	struct_graph_node *neighbor_gnode = NULL; //pointer to a neighbor graph node
	int i = 0; //index for for-loop
	int neighbor_number = 0; //number of neighbors for the intersection point having the AP

	/** check the validity of Gr and Gr_size */
	if(Gr == NULL)
	{
		printf("%s:%d Gr is NULL!\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}
	else if(Gr_size <= 0 )
	{
		printf("%s:%d Gr_size is not positive!\n",
				__FUNCTION__, __LINE__,
				Gr_size);
		exit(1);
	}

	/** get the pointer to the graph node corresponding to intersection_id */
	if(target_point_id > Gr_size || source_intersection_id > Gr_size)
	{
		printf("%s:%d: target_point_id(%d) or source_intersection_id(%d) cannot be greater than Gr_size(%d)\n", 
				__FUNCTION__, __LINE__,
				target_point_id, source_intersection_id, Gr_size);
		exit(1);
	}
	
	intersection_gnode = &(Gr[source_intersection_id-1]);

	/** get the number of neighbors of the intersection_gnode */
	neighbor_number = (int) intersection_gnode->weight;

	/** compute the EDD and EDD_SD at the intersection */
	tail_node = intersection_gnode->vertex;
	neighbor_gnode = intersection_gnode;
	for(i = 0; i < neighbor_number; i++ )
	{
		/* get the pointer to the i-th neighbor gnode */
		neighbor_gnode = neighbor_gnode->next;

		/* set head_node */
		head_node = neighbor_gnode->vertex;

		/* obtain the pointer to the direaction edge of <tail_node,head_node> */
		pEdgeNode = FastLookupDirectionalEdgeQueue(Gr, tail_node, head_node);
		if(pEdgeNode == NULL)
		{
			printf("%s:%d: pEdgeNode for <%s,%s> is NULL\n", 
					__FUNCTION__, __LINE__,
					tail_node, head_node);
			exit(1);
		}

		/* obtain the directional edge's EDD, EDD_VAR, and forwarding probability */
		directional_edge_EDD = pEdgeNode->head_gnode->EDD;
		directional_edge_EDD_VAR = pEdgeNode->head_gnode->EDD_VAR;
		forwarding_probability = pEdgeNode->head_gnode->P_prime;
		//@Note that forwarding_probability should be enhanced considering the waiting time in the case where the packet cannot be forwarded to any carrier around this intersection

		/* update EDD and EDD_VAR with directional_edge_EDD, directional_edge_EDD_VAR and forwarding_probability */
		EDD += directional_edge_EDD * forwarding_probability;

		EDD_VAR += directional_edge_EDD_VAR * pow(forwarding_probability, 2);
	}

	/* compute EDD_SD for this intersection */
	EDD_SD = sqrt(EDD_VAR);

	/* set AP_gnode's EDD and EDD_SD to EDD and EDD_SD, respectively */
	intersection_gnode->EDD = EDD;
	intersection_gnode->EDD_SD = EDD_SD;

	/* set E2E_EDD and E2E_EDD_SD to EDD and EDD_SD, respectively */
	*E2E_EDD = EDD;
	*E2E_EDD_SD = EDD_SD; 
}

void VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(parameter_t *param, int target_point_id, char *intersection_vertex, forwarding_table_queue_t *FTQ, double *intersection_EDD, double *intersection_EDD_SD)
{ //get the EDD and EDD_SD for a target point towards a destination vehicle, based on VADD Model (i.e., Per-intersection Model) from an intersection having an access point or stationary node.
  int intersection_vertex_id = atoi(intersection_vertex); //id corresponding to the vertex having intersection
  struct_graph_node *G = NULL; //pointer to graph G for target_point_id
  int G_size = 0; //size of graph G
  struct_graph_node *intersection_gnode = NULL; //pointer to a graph node corresponding to intersection_id
  /** get the graph for the target point from forwarding table queue FTQ */
  G = FTQ->index_table[target_point_id-1]->G;
  G_size = FTQ->size;

  /** get the pointer to the graph node corresponding to intersection_id */
  if(intersection_vertex_id > G_size)
  {
    printf("VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(): Error: intersection_vertex_id(%d) cannot be greater than size(%d)\n", intersection_vertex_id, G_size);
  }
  
  intersection_gnode = &(G[intersection_vertex_id-1]);

  /* set intersection_EDD and intersection_EDD_SD to EDD and EDD_SD, respectively */
  *intersection_EDD = intersection_gnode->EDD;
  *intersection_EDD_SD = intersection_gnode->EDD_SD; 
}

void VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection_For_V2V_Data_Delivery(parameter_t *param, int target_point_id, char *intersection_vertex, forwarding_table_queue_t *FTQ, struct_graph_node *Gr, int Gr_size, double *intersection_EDD, double *intersection_EDD_SD)
{ //get the EDD and EDD_SD from AP (denoted by intersection_vertex) to a target point toward a destination vehicle, based on VADD Model (i.e., Per-intersection Model) for the road network graph Gr corresponding to the target_point_id as packet destination.
  int intersection_vertex_id = atoi(intersection_vertex); //id corresponding to the vertex having intersection
  struct_graph_node *G = NULL; //pointer to graph G for target_point_id
  int G_size = 0; //size of graph G

#if 1 /* [ */ 
  struct_graph_node *G2 = NULL; //pointer to graph G for target_point_id
  int G_size2 = 0; //size of graph G
#endif 

  struct_graph_node *intersection_gnode = NULL; //pointer to a graph node corresponding to intersection_id

  /** get the graph for the target point from forwarding table queue FTQ */
  G = Gr;
  G_size = Gr_size;

#if 1 /* [ */ 
  /** get the graph for the target point from forwarding table queue FTQ */
  G2 = FTQ->index_table[target_point_id-1]->G;
  G_size2 = FTQ->size;
#endif /* ] */

  /** get the pointer to the graph node corresponding to intersection_id */
  if(intersection_vertex_id > G_size)
  {
    printf("%s:%d: Error: intersection_vertex_id(%d) cannot be greater than size(%d)\n", 
			__FUNCTION__, __LINE__,
			intersection_vertex_id, G_size);
  }
  
  intersection_gnode = &(G[intersection_vertex_id-1]);

  /* set intersection_EDD and intersection_EDD_SD to EDD and EDD_SD, respectively */
  *intersection_EDD = intersection_gnode->EDD;
  *intersection_EDD_SD = intersection_gnode->EDD_SD; 
}

void VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Carrier(parameter_t *param, int target_point_id, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, double *EDD, double *EDD_SD)
{ //get the EDD and EDD_SD for a target point towards a destination vehicle, based on VADD Model (i.e., Per-intersection Model) from the position of carrier_vehicle, that is, from carrier's edge offset on the road network graph.

  char *pTailNode = carrier_vehicle->path_ptr->vertex; //the tail node of the edge where carrier vehicle is moving
  char *pHeadNode = carrier_vehicle->path_ptr->next->vertex; //id corresponding to the head node of the edge where carrier vehicle is moving
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to a directional edge node

  struct_graph_node *G = NULL; //pointer to graph G for target_point_id
  int G_size = 0; //size of graph G
  struct_graph_node *intersection_gnode = NULL; //pointer to a graph node corresponding to intersection_id

  double subedge_length = 0; //the length of the subedge for edge (pTailNode, pHeadNode) and the carrier's offset on the edge, that is, the edge's weight - the carrier's offset
  double link_delay = 0; //link delay for the subedge
  double link_delay_variance = 0; //link delay variance
  double link_delay_deviation = 0; //link delay standard deviation
  double EDD_VAR = 0; //E2E delivery delay variance

  /** get the graph for the target point from forwarding table queue FTQ */
  G = FTQ->index_table[target_point_id-1]->G;
  G_size = FTQ->size;

  /** get the pointer to the directed edge corresponding to (pTailNode, pHeadNode) */
  pEdgeNode = FastLookupDirectionalEdgeQueue(G, pTailNode, pHeadNode);
  if(pEdgeNode == NULL)
  {
    printf("VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Carrier(): Error: pEdgeNode for edge (%s,%s) is NULL\n", pTailNode, pHeadNode);
    exit(1);
  }
  
  /** set up the carrier's EDD and EDD_SD */

  //*EDD = pEdgeNode->head_gnode->EDD;
  //*EDD_SD = pEdgeNode->head_gnode->EDD_SD; 

  /* Note that we need to consider the carrier's offset on the road segment of (pTailNode, pHeadNode). */
  subedge_length = pEdgeNode->head_gnode->weight - carrier_vehicle->current_pos_in_digraph.offset;

  /* Also, note that the current packet carrier is moving on the subedge, so we don't need to consider the waiting time at intersection pTailNode. Thus, we can use VADD_Compute_Subedge_Delay() rather than VADD_Compute_Edge_Delay() even when the current packet carrier is placed at intersection pTailNode. */
  link_delay = VADD_Compute_Subedge_Delay(param, pEdgeNode->head_gnode, subedge_length);
  link_delay_deviation = VADD_Compute_Subedge_Delay_Standard_Deviation(param, pEdgeNode->head_gnode, subedge_length);
  link_delay_variance = pow(link_delay_deviation, 2);

  /* compute the EDD and EDD_SD */
  *EDD = pEdgeNode->head_gnode->gnode->EDD + link_delay;

  EDD_VAR = pEdgeNode->head_gnode->gnode->EDD_VAR; 
  *EDD_SD = sqrt(EDD_VAR + link_delay_variance); 
}

double VADD_Get_EDD_And_EDD_SD_In_Shortest_Path_Model(parameter_t *param, int src_id, int dst_id, double *EDD, double *EDD_SD)
{ //get the EDD and EDD_SD from src_id to dst_id in the road network in the shortest path model
    double delay = 0; //E2E delivery delay
    double delay_variance = 0; //E2E delivery delay variance 
    double delay_deviation = 0; //E2E delivery delay standard deviation
    
    /* check the metric type to get the EDD and EDD_SD */
    if(param->vehicle_vanet_metric_type == VANET_METRIC_EDD)
    {
        delay = param->vanet_table.Dr_edd[src_id-1][dst_id-1];
        delay_variance = param->vanet_table.Sr_edd[src_id-1][dst_id-1];
        delay_deviation = sqrt(delay_variance);
    }
    else if(param->vehicle_vanet_metric_type == VANET_METRIC_EDD_VAR)
    {
        delay = param->vanet_table.Sr_edd[src_id-1][dst_id-1];
        delay_variance = param->vanet_table.Dr_edd[src_id-1][dst_id-1];
        delay_deviation = sqrt(delay_variance);
    }
    else
    {
        printf("VADD_Get_EDD_And_EDD_SD_In_Shortest_Path_Model(): vehicle_vanet_metric_type(%d) is not supported!\n", param->vehicle_vanet_metric_type);
        exit(1);
    }

    /* set EDD and EDD_SD to delay and delay_deviation, respectively */
    *EDD = delay;
    *EDD_SD = delay_deviation;
}

/** EDD Computation for Download Mode from AP to Vehicle */
void VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download(double update_time, parameter_t *param, struct_vehicle_t *vehicle, forwarding_table_queue_t *FTQ)
{ //For download mode, compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge with forwarding table queue FTQ and update the vehicle's target point under dynamic target point selection
  double per_vehicle_EDD = 0; //EDD computed by Per-vehicle model
  double per_intersection_EDD = 0; //EDD computed by Per-intersection model
  double alpha = 0 ;//alpha is a function of vehicular traffic density for a hybrid EDD
  double per_vehicle_EDD_SD = 0; //EDD_SD computed by Per-vehicle model
  double per_intersection_EDD_SD = 0; //EDD_SD computed by Per-intersection model
  struct_graph_node *G = NULL; //pointer to graph G corresponding to the vehicle's target point
  int G_size = 0; //size of graph G
  char target_point[NAME_SIZE]; //buffer for target point  
  double EDD_p = 0; //Expected Delivery Delay (EDD) for a target point based on VADD model
  double EAD_p = 0; //Expected Arrival Delay (EAD) of Destination Vehicle for a target point

  /** If there is no packet, return EDD and EDD_SD of infinite */
  if((param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_VEHICLE) && (vehicle->packet_queue->size == 0))
  {
    vehicle->EDD_for_download = INF;
    vehicle->EDD_SD_for_download = INF;
    vehicle->EDD_update_time_for_download = update_time;
    vehicle->target_point_id = 0;
    return;
  }
  else if((param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY) && (vehicle->packet_queue->size == 0) && (vehicle->ptr_convoy_queue_node->leader_vehicle->packet_queue->size == 0))
  //else if((param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY) && (vehicle->ptr_convoy_queue_node->leader_vehicle->packet_queue->size == 0))
  {
    vehicle->EDD_for_download = INF;
    vehicle->EDD_SD_for_download = INF;
    vehicle->EDD_update_time_for_download = update_time;
    vehicle->target_point_id = 0;
    return;
  }

  /**@ for debugging */
  //if(update_time > 4473 && vehicle->ptr_convoy_queue_node->leader_vehicle->id == 75)
  //{
  //  printf("VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download(): time=%.2f, debugging\n", update_time);    
  //}
  /*******************/

  /*@for debugging */
  //if(update_time >= 29192 && vehicle->id == 94)
  //  printf("VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download(): time=%.2f, vid=%d is traced!\n", update_time, vehicle->id);
  /*****************/

  /** find the new target point under dynamic target point selection type */
  if(param->vehicle_vanet_target_point_selection_type == VANET_TARGET_POINT_SELECTION_DYNAMIC_TARGET_POINT)
  { //for dynamic target point selection, let only the convoy leader compute a new target point based on the destination vehicle. Thus, other convoy members refer to the leader's target point
    if(vehicle == vehicle->ptr_convoy_queue_node->leader_vehicle)
    { //the case where vehicle is the convoy leader: update the target point

      vehicle->target_point_id = GetTargetPoint_For_Carrier(param, update_time, vehicle, FTQ, &EDD_p, &EAD_p);
      if(vehicle->target_point_id == 0)
	{ //the vehicle trajectory is not valid any more because the packet delivery time is greater than the travel time of the destination vehicle over its trajectory
        vehicle->EDD_for_download = INF;
        vehicle->EDD_SD_for_download = INF;
        vehicle->EDD_update_time_for_download = update_time;
	return;
      }

      /** Note: Consider whether the new target point is accepted or not, considering that the new target point is within the update circle whose center is the previous target point with the radius of the destination vehicle movement distance during the moving time */

    }
    else if(vehicle->ptr_convoy_queue_node->leader_vehicle->packet_queue->size == 0)
    { //the case where the convoy leader has no packet: defer the target point update until the convoy leader receives the packets of vehicle
      vehicle->EDD_for_download = INF;
      vehicle->EDD_SD_for_download = INF;
      vehicle->EDD_update_time_for_download = update_time;
      vehicle->target_point_id = 0;
      return;
    }
    else
    { //the case where vehicle is not the convoy leader that has packets: use the convoy leader's target point in order to update the EDD and EDD_SD of this vehicle
      //vehicle->target_point_id = vehicle->ptr_convoy_queue_node->leader_vehicle->target_point_id;
      if(vehicle->ptr_convoy_queue_node->leader_vehicle->target_point_id == 0)
      { //the case where the vehicle's target point is zero, use the leader's latest packet's target point id; note that this will not happen because when vehicle receives packets or when the leader has not updated its target point id with the recently received packets, it sets up this target point id with the latest packet's target point id
        vehicle->target_point_id = vehicle->ptr_convoy_queue_node->leader_vehicle->latest_packet_ptr->target_point_id;
      }
      else
      {
        vehicle->target_point_id = vehicle->ptr_convoy_queue_node->leader_vehicle->target_point_id;
      }
    }
  }
  else
  { //for the convoy members under static target point selection or for the convoy members under adaptive target point selection
    if(vehicle == vehicle->ptr_convoy_queue_node->leader_vehicle)
    { //the case where vehicle is the convoy header, use the latest packet's target point as its target point
      if(vehicle->target_point_id == 0)
      { //the case where the vehicle's target point is zero, use the latest packet's target point id; note that this will not happen because when vehicle receives packets, it sets up this target point id with the latest packet's target point id
        vehicle->target_point_id = vehicle->latest_packet_ptr->target_point_id;
      }
    }
    else if(vehicle->ptr_convoy_queue_node->leader_vehicle->packet_queue->size == 0)
    { //the case where the convoy leader has no packet: defer the target point update until the convoy leader receives the packets of vehicle
      vehicle->EDD_for_download = INF;
      vehicle->EDD_SD_for_download = INF;
      vehicle->EDD_update_time_for_download = update_time;
      vehicle->target_point_id = 0;
      return;
    }
    else
    { //the case where vehicle is not the convoy leader that has packets: use the convoy leader's target point in order to update the EDD and EDD_SD of this vehicle
      if(vehicle->ptr_convoy_queue_node->leader_vehicle->target_point_id == 0)
      { //the case where the vehicle's target point is zero, use the leader's latest packet's target point id; note that this will not happen because when vehicle receives packets or when the leader has not updated its target point id with the recently received packets, it sets up this target point id with the latest packet's target point id
        vehicle->target_point_id = vehicle->ptr_convoy_queue_node->leader_vehicle->latest_packet_ptr->target_point_id;
      }
      else
      {
        vehicle->target_point_id = vehicle->ptr_convoy_queue_node->leader_vehicle->target_point_id;
      }
    }
  }

  /* set up G, G_size and tp_table */
  G = FTQ->index_table[vehicle->target_point_id-1]->G;
  G_size = FTQ->size;
    
  /* set up target point table with vehicle's target point id */
  itoa(vehicle->target_point_id, target_point);
  SetTargetPoint_In_TafficTable(&(vehicle->tp_table), target_point);

  /** compute EDD and EDD_SD */
  switch(param->vehicle_vanet_edd_model)
  {
  case VANET_EDD_PER_VEHICLE_MODEL:

    /** compute the vehicle's path EDD and EDD_SD according to the vehicle's trajectory */
    VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(param, vehicle, G, G_size, &(vehicle->tp_table), &(vehicle->EDD_for_download), &(vehicle->EDD_SD_for_download));

    break;

  case VANET_EDD_PER_INTERSECTION_MODEL:

    /** compute vehicle's EDD and EDD_SD from directional edge's EDD/EDD_SD and the offset of directional edge */
    VADD_Compute_EDD_And_EDD_SD_Based_On_VADD(param, vehicle, G, G_size, &(vehicle->tp_table), &(vehicle->EDD_for_download), &(vehicle->EDD_SD_for_download));

    break;

  default:
    printf("VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download(): Error: param->vehicle_vanet_edd_model(%d) is not supported yet!\n", param->vehicle_vanet_edd_model);
    exit(1);
  }

  /* update vehicle's EDD update time */
  vehicle->EDD_update_time_for_download = update_time;
}

void VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download_With_Given_TargetPoint_And_SequenceNumber(double update_time, parameter_t *param, struct_vehicle_t *vehicle, forwarding_table_queue_t *FTQ, int target_point_id, unsigned int seq)
{ //Given a target point and the packet sequence number (to check the freshness of the destination vehicle information), compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge with forwarding table queue FTQ
  double per_vehicle_EDD = 0; //EDD computed by Per-vehicle model
  double per_intersection_EDD = 0; //EDD computed by Per-intersection model
  double alpha = 0 ;//alpha is a function of vehicular traffic density for a hybrid EDD
  double per_vehicle_EDD_SD = 0; //EDD_SD computed by Per-vehicle model
  double per_intersection_EDD_SD = 0; //EDD_SD computed by Per-intersection model
  struct_graph_node *G = NULL; //pointer to graph G corresponding to the vehicle's target point
  int G_size = 0; //size of graph G
  char target_point[NAME_SIZE]; //buffer for target point  
  double EDD_p = 0; //Expected Delivery Delay (EDD) for a target point based on VADD model
  double EAD_p = 0; //Expected Arrival Delay (EAD) of Destination Vehicle for a target point

  /** check the validness of target point id and seq */
  if(target_point_id == 0)
  {
    printf("VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download_With_Given_TargetPoint_And_SequenceNumber(): Error: time=%.2f, vid=%d, target_point_id is 0\n", update_time, vehicle->id);
    exit(1);
  }
  else if(seq == 0)
  {
    printf("VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download_With_Given_TargetPoint_And_SequenceNumber(): Error: time=%.2f, vid=%d, seq is 0\n", update_time, vehicle->id);
    exit(1);
  }

  /** check whether the given target point id is a newer one than vehicle's through the sequence number comparison; 
      Note that vehicle's target point is zero, replace its target point id with the given target point id */
  if(vehicle->target_point_id == 0 || vehicle->latest_packet_seq <= seq) //without ==, the segmentation error happens!
  //if(vehicle->latest_packet_seq < seq)
  {
    vehicle->target_point_id = target_point_id;
    vehicle->latest_packet_seq = seq;
  }

  /* set up G, G_size and tp_table */
  G = FTQ->index_table[vehicle->target_point_id-1]->G;
  G_size = FTQ->size;
    
  /* set up target point table with vehicle's target point id */
  itoa(vehicle->target_point_id, target_point);
  SetTargetPoint_In_TafficTable(&(vehicle->tp_table), target_point);

  /** compute EDD and EDD_SD */
  switch(param->vehicle_vanet_edd_model)
  {
  case VANET_EDD_PER_VEHICLE_MODEL:

    /** compute the vehicle's path EDD and EDD_SD according to the vehicle's trajectory */
    VADD_Compute_EDD_And_EDD_SD_Based_On_TBD(param, vehicle, G, G_size, &(vehicle->tp_table), &(vehicle->EDD_for_download), &(vehicle->EDD_SD_for_download));

    break;

  case VANET_EDD_PER_INTERSECTION_MODEL:

    /** compute vehicle's EDD and EDD_SD from directional edge's EDD/EDD_SD and the offset of directional edge */
    VADD_Compute_EDD_And_EDD_SD_Based_On_VADD(param, vehicle, G, G_size, &(vehicle->tp_table), &(vehicle->EDD_for_download), &(vehicle->EDD_SD_for_download));

    break;

  default:
    printf("VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download_With_Given_TargetPoint_And_SequenceNumber(): Error: param->vehicle_vanet_edd_model(%d) is not supported yet!\n", param->vehicle_vanet_edd_model);
    exit(1);
  }

  /* update vehicle's EDD update time */
  vehicle->EDD_update_time_for_download = update_time;
}

void VADD_Update_VehicleTargetPoint_Along_With_EDD_And_EDD_SD_For_Download(double update_time, parameter_t *param, struct_vehicle_t *vehicle, forwarding_table_queue_t *FTQ, intersection_area_type_t input_intersection_area_type)
{ //For download mode, compute vehicle's target point for the destination vehicle and then recompute the EDD_for_download/EDD_SD_download with forwarding table queue FTQ; for a convoy-based forwarding, update the target point and the EDD_for_download/EDD_SD_download of vehicle's leader 
  boolean flag = FALSE; //flag
  int target_point_id_for_tail = 0; //target point id towards the destination vehicle at the current edge's tail node to which vehicle is close 
  int target_point_id_for_head = 0; //target point id towards the destination vehicle at the current edge's head node to which vehicle is close 
  int target_point_id = 0; //a new target point id towards destination vehicle 
  double difference_for_tail = 0; //difference of EDD_p and EAD_p at tail node
  double difference_for_head = 0; //difference of EDD_p and EAD_p at head node
  intersection_area_type_t output_intersection_area_type; //type to check whether the intersection area(s) having vehicle has an intersection on the destination vehicle

  /*@for debugging */
  //int previous_target_point = vehicle->target_point_id;
  //if(update_time >= 3814 && vehicle->id == 116)
  //  printf("VADD_Update_VehicleTargetPoint_Along_With_EDD_And_EDD_SD_For_Download(): time=%.2f, vid=%d is traced!\n", update_time, vehicle->id);
  /*****************/
  
  /** when the target point selection is adaptive, perform the remaining parts of this function */
  if(param->vehicle_vanet_target_point_selection_type != VANET_TARGET_POINT_SELECTION_ADAPTIVE_TARGET_POINT)
    return;

  /** find a new target point to which vehicle is closest from the intersection area containing vehicle and which is on the destination vehicle trajectory */
  target_point_id = Find_NewTargetPoint_Where_Vehicle_Is_Close_To_DestinationVehicleTrajectory(param, update_time, vehicle, FTQ, input_intersection_area_type);

  /** if there is a new target point greater than 0, return immediately without EDD and EDD_SD updates */
  if(target_point_id == 0)
    return;

  /** update EDD and EDD_SD for download */
  vehicle->target_point_id = target_point_id; //update vehicle's target point id
  VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download(update_time, param, vehicle, FTQ);

  /************************************************************/
  /** update the target point of the latest receive packet */
  //vehicle->latest_packet_ptr->target_point_id = target_point_id;
  //vehicle->latest_packet_ptr->target_point_id_update_time = update_time;
  /************************************************************/

  /*@for debugging */
  //if(previous_target_point != vehicle->target_point_id && vehicle->id == 116)
  //  printf("VADD_Update_VehicleTargetPoint_Along_With_EDD_And_EDD_SD_For_Download(): time=%.2f, vid=%d is traced!\n", update_time, vehicle->id);
  /*****************/
}
/*********************************************************/

boolean VADD_Is_Within_Intersection_Area(parameter_t *param, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, intersection_area_type_t *intersection_area_type, int *tail_intersection_id, int *head_intersection_id)
{ //check whether this vehicle is within the Intersection areas of either the tail node or the head node of the current directed edge for the data forwarding to other vehicles moving on other road segments
  boolean flag = FALSE;
  double R = param->communication_range; //RF communication range
  char *tail = NULL; //tail node of the current directed edge where vehicle is moving
  char *head = NULL; //head node of the current directed edge where vehicle is moving
  struct_graph_node *tail_gnode_in_node_array = NULL; //pointer to the graph node corresponding to the tail node in the node array of G
  struct_graph_node *head_gnode_in_node_array = NULL; //pointer to the graph node corresponding to the head node in the node array of G
  struct_coordinate1_t p1; //the coordinate of the tail node of the directed edge (tail, head) where the vehicle is moving
  struct_coordinate1_t p2; //the coordinate of the head node of the directed edge (tail, head) where the vehicle is moving
  struct_coordinate1_t p;  //the coordinate of the vehicle's position
  double d1 = 0; //distance between vehicle and the intersection of tail node on the road segment
  double d2 = 0; //distance between vehicle and the intersection of head node on the road segment
 
  if(IsEndOfTravel(vehicle->path_list, vehicle->path_ptr) == TRUE)
  {
    printf("VADD_Is_Within_Intersection_Area(): vehicle is on the last vertex on its path, so we cannot make position update vector!\n");
    exit(1);
  }

  /* get tail and head with the edge where vehicle is moving */
  tail = vehicle->path_ptr->vertex;
  head = vehicle->path_ptr->next->vertex;

  /* get tail_intersectionId and head_intersection_id with tail and head, respectively */
  *tail_intersection_id = atoi(tail);
  *head_intersection_id = atoi(head);

  /* get the pointers to the graph nodes of tail and head */
  tail_gnode_in_node_array = LookupGraph(G, G_size, tail);
  head_gnode_in_node_array = LookupGraph(G, G_size, head);
   
  /* obtain the coordinates of the tail and head nodes */
  memcpy(&p1, &(tail_gnode_in_node_array->coordinate), sizeof(p1));
  memcpy(&p2, &(head_gnode_in_node_array->coordinate), sizeof(p2));

  /* obtain the vehicle's coordinate */
  memcpy(&p, &(vehicle->current_pos), sizeof(p));

  /* compute the distance between vehicle and tail intersection */
  d1 = euclidean_distance2(&p, &p1);

  /* compute the distance between vehicle and head intersection */
  d2 = euclidean_distance2(&p, &p2);

  /* determine intersection_area_type with d1 and d2 */
  if(d1 <= R && d2 <= R)
  {
    *intersection_area_type = INTERSECTION_AREA_BOTH_NODES;
    flag = TRUE;
  }
  else if(d1 <= R)
  {
    *intersection_area_type = INTERSECTION_AREA_TAIL_NODE;
    flag = TRUE;
  }
  else if(d2 <= R)
  {
    *intersection_area_type = INTERSECTION_AREA_HEAD_NODE;
    flag = TRUE;
  }
  else
  {
    *intersection_area_type = INTERSECTION_AREA_NONE;
    flag = FALSE;
  }

  return flag;
}

boolean VADD_Is_There_Next_Carrier_At_Both_Intersection_Areas(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, forwarding_table_queue_t *FTQ, intersection_area_type_t intersection_area_type, struct_vehicle_t **next_carrier)
{ //check whether this vehicle can forward packets to the other vehicle as next carrier around both the tail intersection area and the head intersection area
  boolean result = FALSE;
  boolean flag1 = FALSE, flag2 = FALSE;
  struct_vehicle_t *next_carrier_for_tail = NULL; //next carrier for the tail intersection area
  struct_vehicle_t *next_carrier_for_head = NULL; //next carrier for the head intersection area

  /** reset *next_carrier to NULL */
  *next_carrier = NULL;

  /** check whether vehicle has packets to forward to a next carrier */
  if(vehicle->packet_queue->size == 0)
  {
    return FALSE;
  }

  /** find the next carrier for the tail intersection area */
  if(intersection_area_type == INTERSECTION_AREA_TAIL_NODE || intersection_area_type == INTERSECTION_AREA_BOTH_NODES)
    flag1 = VADD_Is_There_Next_Carrier_At_Intersection_Area(param, current_time, vehicle, G, G_size, FTQ, INTERSECTION_AREA_TAIL_NODE, &next_carrier_for_tail);

  /** find the next carrier for the head intersection area */
  if(intersection_area_type == INTERSECTION_AREA_HEAD_NODE || intersection_area_type == INTERSECTION_AREA_BOTH_NODES)
    flag2 = VADD_Is_There_Next_Carrier_At_Intersection_Area(param, current_time, vehicle, G, G_size, FTQ, INTERSECTION_AREA_HEAD_NODE, &next_carrier_for_head);
  
  /** select the best next carrier in terms of minimum EDD */
  if(intersection_area_type == INTERSECTION_AREA_BOTH_NODES) //if-1
  {
    if(flag1 && flag2) //if-1.1
    {
      if(next_carrier_for_tail->EDD < next_carrier_for_head->EDD)
        *next_carrier = next_carrier_for_tail;
      else
	*next_carrier = next_carrier_for_head;

      result = TRUE;
    } //end of if-1.1
  } //end of if-1
  else if(intersection_area_type == INTERSECTION_AREA_TAIL_NODE) //else if-2
  {
    if(flag1)
    {
      *next_carrier = next_carrier_for_tail;      
      result = TRUE;
    }
  } //end of else if-2
  else if(intersection_area_type == INTERSECTION_AREA_HEAD_NODE) //else if-3
  {
    if(flag2)
    {
      *next_carrier = next_carrier_for_head;
      result = TRUE;
    }
  } //end of else if-3

  return result;
}

boolean VADD_Is_There_Next_Carrier_At_Intersection_Area(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, forwarding_table_queue_t *FTQ, intersection_area_type_t intersection_area_type, struct_vehicle_t **next_carrier)
{ //check whether this vehicle can forward packets to the other vehicle as next carrier around either the tail intersection area or the head intersection area
  boolean result = FALSE; //return value
  boolean flag = FALSE; //flag to indicate there exists a next carrier candidate
  //boolean ap_flag = FALSE; //flag to check whether the tail node is one of APs; ap_flag is used to determine next carrier

  struct_graph_node *pGraphNode = NULL; //pointer to a graph node
  char *tail_node = vehicle->path_ptr->vertex; //tail node of the directional edge where vehicle is moving
  char *head_node = vehicle->path_ptr->next->vertex; //head node of the directional edge where vehicle is moving
  //char *tail_node_for_next_edge = NULL; //tail node of the directional edge where vehicle will be moving
  //char *head_node_for_next_edge = NULL; //head node of the directional edge where vehicle will be moving
  char *tail_node_for_next_forwarding_edge = NULL; //tail node of the next directional edge for forwarding
  char *head_node_for_next_forwarding_edge = NULL; //head node of the next directional edge for forwarding
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
  //directional_edge_queue_node_t *pNextEdgeNode = NULL; //pointer to the directional edge taken by this vehicle
  int size = 0; //size of intersection EDD queue
  int i = 0; //index for for-loop
  //int vehicle_id = 0; //vehicle id
  //double vehicle_offset = 0; //vehicle's offset on the next directional edge
  //double vehicle_EDD = 0; //vehicle's EDD
  double min_next_carrier_EDD = INF; //minimum value of next carrier's EDD
  double max_next_carrier_offset = -1; //maximum value of next carrier's offset
  struct_vehicle_t *next_carrier_candidate = NULL; //pointer to the next carrier candidate
  directional_edge_type_t edge_type = OUTGOING_EDGE; //directional edge type for tail_node
  struct_graph_node *intersection_gnode = NULL; //pointer to the graph node of the intersection where the vehicle is arriving
  struct_graph_node *neighboring_intersection_gnode = NULL; //pointer to the graph node of the neighboring intersection for the intersection where the vehicle is arriving

  /** reset *next_carrier to NULL */
  *next_carrier = NULL;

  /** check whether vehicle has packets to forward to a next carrier */
  if(vehicle->packet_queue->size == 0)
  {
    return FALSE;
  }

  ///** check whether vehicle has vaild target point and sequence number under download mode */
  //if(vehicle->target_point_id == 0 || vehicle->latest_packet_seq == 0)
  //{
  //  return FALSE;
  //}

  /*@[11/11/09] Note: I made a mistake that I omitted the following condition "if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)", so I prevented the forwarding at intersections */
  if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
  {
      /** check whether vehicle has vaild target point and sequence number under download mode */
    if(vehicle->target_point_id == 0 || vehicle->latest_packet_seq == 0)
    {
      return FALSE;
    }
  }

  /** search for a best next carrier in the order of the smallest EDDs assigned to directional edges incident to the current intersect where the vehicle has reached */

  /* obtain the pointer to the direaction edge of <tail_node,head_node> */
  pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
  if(pEdgeNode == NULL)
  {
    printf("VADD_Is_There_Next_Carrier_At_Intersection_Area(): pEdgeNode for <%s,%s> is NULL\n", tail_node, head_node);
    exit(1);
  }

  /** Next carrier selection rule: We select a vehicle with the shortest vehicle EDD regardless of branch EDDs: Note that in Per-Intersection model we select a farther vehicle on the best branch with the shortest branch EDD */  

  /* determine the intersection node according to intersection_area_type */
  if(intersection_area_type == INTERSECTION_AREA_HEAD_NODE)
    intersection_gnode = pEdgeNode->head_gnode->gnode;
  else if(intersection_area_type == INTERSECTION_AREA_TAIL_NODE)
    intersection_gnode = pEdgeNode->tail_gnode;
  else
  {
    printf("VADD_Is_There_Next_Carrier_At_Intersection_Area(): intersection_area_type(%d) is invalid!\n", intersection_area_type);
    exit(1);
  }

  neighboring_intersection_gnode = intersection_gnode;
  size = (int)intersection_gnode->weight;
  for(i = 0; i < size; i++) //for-1
  {
    neighboring_intersection_gnode = neighboring_intersection_gnode->next;
    
    /** search a next carrier candidate in the outgoing edge of <intersection, neighboring_intersection> */
    edge_type = OUTGOING_EDGE;
    tail_node_for_next_forwarding_edge = intersection_gnode->vertex;
    head_node_for_next_forwarding_edge = neighboring_intersection_gnode->vertex;
    
    /* check whether the tail node is one of APs; ap_flag is used to determine next carrier */
    //ap_flag = IsVertexInTrafficTable(ap_table, tail_node_for_next_forwarding_edge);

    flag = VADD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_Area(param, vehicle, tail_node_for_next_forwarding_edge, head_node_for_next_forwarding_edge, edge_type, G, G_size, &next_carrier_candidate);
    //determine whether to forward its packets to next carrier moving on the road segment incident to an intersection corresponding to tail_node and return the pointer to the next carrier through *next_carrier

    if(flag) //if-1
    {    
      /* select a vehicle with the minimum next carrier EDD as a next carrier */
      switch(param->vehicle_vanet_forwarding_type) //switch-1
      {
      case VANET_FORWARDING_BASED_ON_VEHICLE:
	if(next_carrier_candidate->EDD < min_next_carrier_EDD)
	{
	  *next_carrier = next_carrier_candidate;
	  min_next_carrier_EDD = next_carrier_candidate->EDD;
          result = flag; //set result to TRUE since there exists next carrier candidate
	}
	break;

      case VANET_FORWARDING_BASED_ON_CONVOY:
	if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
	{
          if((next_carrier_candidate->EDD < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD) 
          && (next_carrier_candidate->EDD < min_next_carrier_EDD))
          //if((next_carrier_candidate->ptr_convoy_queue_node->leader_vehicle->EDD < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD) 
          //&& (next_carrier_candidate->ptr_convoy_queue_node->leader_vehicle->EDD < min_next_carrier_EDD))
	  {
	    *next_carrier = next_carrier_candidate;
	    min_next_carrier_EDD = next_carrier_candidate->EDD; //set min_next_carrier_EDD to the next carrier candidate's EDD
            //min_next_carrier_EDD = next_carrier_candidate->ptr_convoy_queue_node->leader_vehicle->EDD; //set min_next_carrier_EDD to the next carrier candidate's convoy leader's EDD
            result = flag; //set result to TRUE since there exists next carrier candidate
	  }
	}
	else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
	{
          /** let next_carrier_candidate compute its new EDD_for_download with AP's target point */   
          VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download_With_Given_TargetPoint_And_SequenceNumber(current_time, param, next_carrier_candidate, FTQ, vehicle->target_point_id, vehicle->latest_packet_seq);

          if((next_carrier_candidate->EDD_for_download < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD_for_download) 
          && (next_carrier_candidate->EDD_for_download < min_next_carrier_EDD))
	  {
	    *next_carrier = next_carrier_candidate;
	    min_next_carrier_EDD = next_carrier_candidate->EDD_for_download; //set min_next_carrier_EDD to the next carrier candidate's EDD
            result = flag; //set result to TRUE since there exists next carrier candidate
	  }
	}
	break;

      default:
	printf("VADD_Is_There_Next_Carrier_At_Intersection_Area(): param->vehicle_vanet_forwarding_type(%d) is not supported!\n", param->vehicle_vanet_forwarding_type);
	exit(1);
      } //end of switch-1
    } //end of if-1


    /** search a next carrier candidate in the outgoing edge of <intersection, neighboring_intersection> */
    edge_type = INCOMING_EDGE;
    tail_node_for_next_forwarding_edge = neighboring_intersection_gnode->vertex;
    head_node_for_next_forwarding_edge = intersection_gnode->vertex;
    
    flag = VADD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_Area(param, vehicle, tail_node_for_next_forwarding_edge, head_node_for_next_forwarding_edge, edge_type, G, G_size, &next_carrier_candidate);
    //determine whether to forward its packets to next carrier moving on the road segment incident to an intersection corresponding to tail_node and return the pointer to the next carrier through *next_carrier

    if(flag) //if-2
    {    
      /* select a vehicle with the minimum next carrier EDD as a next carrier */
      switch(param->vehicle_vanet_forwarding_type) //switch-2
      {
      case VANET_FORWARDING_BASED_ON_VEHICLE:
	if(next_carrier_candidate->EDD < min_next_carrier_EDD)
	{
	  *next_carrier = next_carrier_candidate;
	  min_next_carrier_EDD = next_carrier_candidate->EDD;
          result = flag; //set result to TRUE since there exists next carrier candidate
	}
	break;

      case VANET_FORWARDING_BASED_ON_CONVOY:
	if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
	{
          if((next_carrier_candidate->EDD < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD) 
          && (next_carrier_candidate->EDD < min_next_carrier_EDD))
          //if((next_carrier_candidate->ptr_convoy_queue_node->leader_vehicle->EDD < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD) 
          //&& (next_carrier_candidate->ptr_convoy_queue_node->leader_vehicle->EDD < min_next_carrier_EDD))
	  {
	    *next_carrier = next_carrier_candidate;
	    min_next_carrier_EDD = next_carrier_candidate->EDD; //set min_next_carrier_EDD to the next carrier candidate's EDD
            //min_next_carrier_EDD = next_carrier_candidate->ptr_convoy_queue_node->leader_vehicle->EDD; //set min_next_carrier_EDD to the next carrier candidate's convoy leader's EDD
	    result = flag; //set result to TRUE since there exists next carrier candidate
	  }	
	}
	else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
	{
          if((next_carrier_candidate->EDD_for_download < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD_for_download) 
          && (next_carrier_candidate->EDD_for_download < min_next_carrier_EDD))
	  {
	    *next_carrier = next_carrier_candidate;
	    min_next_carrier_EDD = next_carrier_candidate->EDD_for_download; //set min_next_carrier_EDD to the next carrier candidate's EDD
	    result = flag; //set result to TRUE since there exists next carrier candidate
	  }	
	}

	break;

      default:
	printf("VADD_Is_There_Next_Carrier_At_Intersection_Area(): param->vehicle_vanet_forwarding_type(%d) is not supported!\n", param->vehicle_vanet_forwarding_type);
	exit(1);
      } //end of switch-2
    } //end of if-2
  } //end of for-1

  /** [03/18/09] set the next_carrier to the vehicle's convoy leader when there exists no next carrier candidate in a different convoy with less EDD */
  if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY)
  {
    if((result == FALSE) && (vehicle->id != vehicle->ptr_convoy_queue_node->leader_vehicle->id))
    {
      /*
      if(vehicle->EDD < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD)
	{ //@Note: due to the order of the update of EDD according to id, leader_vehicle's EDD might be greater than the vehicle's EDD
	printf("VADD_Is_There_Next_Carrier_At_Intersection(): Error: vehicle's EDD(%.2f) is less than vehicle's convoy leader's EDD(%.2f) where vehicle id is %d and the leader's id is %d\n", vehicle->EDD, vehicle->ptr_convoy_queue_node->leader_vehicle->EDD, vehicle->id, vehicle->ptr_convoy_queue_node->leader_vehicle->id);
	exit(1);
      }
      */

      *next_carrier = vehicle->ptr_convoy_queue_node->leader_vehicle;

      result = TRUE;
    }
  }

  return result;
}

boolean VADD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_Area(parameter_t *param, struct_vehicle_t *vehicle, char *tail_node_for_next_forwarding_edge, char *head_node_for_next_forwarding_edge, directional_edge_type_t edge_type, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier)
{ //determine whether to forward its packets to next carrier moving on the road segment incident to an intersection area corresponding to either the tail intersection or the head intersection and return the pointer to the next carrier through *next_carrier
  boolean result = FALSE;
  boolean flag = FALSE; //flag to indicate whether a next carrier is determined or not
  double R = param->communication_range; //communication range
  double distance1 = 0; //distance between the next carrier candidate and the intersection
  double distance2 = 0; //distance between vehicle and another vehicle moving on the same directional edge
  double max_neighbor_offset = -1; //neighbor's offset such the distance between vehicle and preceding neighbor vehicle is maximum
  double min_neighbor_EDD = INF; //neighbor vehicle or neighbor convoy with a minimum EDD
  char *tail_node = tail_node_for_next_forwarding_edge; //tail node of the next directional edge for forwarding
  char *head_node = head_node_for_next_forwarding_edge; //head node of the next directional edge for forwarding
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
  vehicle_movement_queue_node_t *pMoveNode = NULL; //pointer to the vehicle movement queue
  int i = 0; //index for for-loop
  int size = 0; //size of vehicle movement queue
  double offset_in_directional_edge = 0; //vehicle's offset on the directional edge; NOTE that this offset will be -R/2 where R is the communication range.

  /** reset *next_carrier to NULL */
  *next_carrier = NULL;

  /** check whether vehicle has packets to forward to a next carrier */
  if(vehicle->packet_queue->size == 0)
  {
    return FALSE;
  }

  /** obtain the pointer to the direaction edge of <tail_node,head_node> */
  pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);

  if(pEdgeNode == NULL)
  {
    if(edge_type == OUTGOING_EDGE) //outgoing edge for tail_node
      printf("VADD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_Area: pEdgeNode for <%s,%s> is NULL\n", tail_node, head_node);
    else
      printf("VADD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_Area: pEdgeNode for <%s,%s> is NULL\n", head_node, tail_node);

    exit(1);
  }

  size = pEdgeNode->vehicle_movement_list.size;
  pMoveNode = &(pEdgeNode->vehicle_movement_list.head);
  for(i = 0; i < size && flag == FALSE; i++) //[03/18/09] for-1
  //for(i = 0; i < size; i++) //for-1
  {
    pMoveNode = pMoveNode->next;
    if(vehicle->id == pMoveNode->vid)
      continue;

    /* compute the distance between the vehicle's approaching intersection and the next carrier candidate */
    if(edge_type == OUTGOING_EDGE) //for the outgoing edge of tail_node
      distance1 = pMoveNode->offset;
    else //for the incoming edge of tail_node
      distance1 = pEdgeNode->weight - pMoveNode->offset;

    /* compute the distance between the vehicle and the next carrier candidate */
    distance2 = euclidean_distance2(&(vehicle->current_pos), &(pMoveNode->vnode->current_pos));

    if(distance1 > R || distance2 > R) //two vehicles cannot communicate with each other
      continue;

    /* Next carrier selection rule 1: We select a vehicle on the best branch with the shortest branch EDD regardless of vehicles' EDD */
    //if((distance <= param->communication_range) && (pMoveNode->offset > offset_in_directional_edge) && (pMoveNode->offset > max_neighbor_offset))

    /* Next carrier selection rule 2: We select a vehicle with the shortest vehicle EDD */  

    /* select a next vehicle according to Forwarding type */
    switch(param->vehicle_vanet_forwarding_type) //switch-1
    {
      case VANET_FORWARDING_BASED_ON_VEHICLE:
	if((pMoveNode->vnode->EDD < vehicle->EDD) && (pMoveNode->vnode->EDD < min_neighbor_EDD))
        { //we also check vehicles' EDDs
	  min_neighbor_EDD = pMoveNode->vnode->EDD;
          *next_carrier = pMoveNode->vnode; //update the pointer of neighbor vehicle node with smaller EDD; Note in VADD, a vehicle with farther offset is chosen as next carrier.
        }
	break;

      case VANET_FORWARDING_BASED_ON_CONVOY:
	if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
	{
	  if((pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid) && (pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle->EDD < vehicle->ptr_convoy_queue_node->leader_vehicle->EDD)) 
	  {
	    *next_carrier = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
	    flag = TRUE; //[03/18/09] set flag to let the for-loop terminate

	    //@ Note: we need to consider the communication range between the packet source vehicle and the next_carrier since the next_carrier is connected to the leader vehicle with the minimum EDD withing the connected component including the next_carrier

	    //return TRUE;
	  }
	}
	else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
	{
	  if((distance2 <= param->communication_range) && (pMoveNode->vnode->ptr_convoy_queue_node->cid != vehicle->ptr_convoy_queue_node->cid))
	  {
	    *next_carrier = pMoveNode->vnode->ptr_convoy_queue_node->leader_vehicle;
	    flag = TRUE; //[03/18/09] set flag to let the for-loop terminate
	  }
	}
	
	break;

      default:
	printf("VADD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_Area(): vanet forwarding type (%d) is not known!\n", param->vehicle_vanet_forwarding_type);
	exit(1);
    } //end of switch-1
  } //end of for-1

  /** check whether a next carrier vehicle exists */
  if(*next_carrier != NULL)
    result = TRUE;

  return result;
}

boolean VADD_Is_Vehicle_Moving_On_Packet_Trajectory(struct_vehicle_t *vehicle, packet_trajectory_queue_t *packet_trajectory)
{ //check whether vehicle is moving the packet trajectory or not in order to determine the next packet carrier 
  boolean result = FALSE; //result to show the existence of a vehicle moving on the packet trajectory
  int vehicle_tail_intersection_id = 0; //intersection id of the tail node of the edge where vehicle is moving
  int vehicle_head_intersection_id = 0; //intersection id of the head node of the edge where vehicle is moving
  int packet_tail_intersection_id = 0; //intersection id of the tail node of the edge where packet should be forwarded
  int packet_head_intersection_id = 0; //intersection id of the head node of the edge where packet should be forwarded

  /** set vehicle_tail_intersection_id and vehicle_head_intersection_id to the tail and head nodes of the vehicle's current edge */
  if(vehicle->path_ptr == NULL)
  {
    printf("VADD_Is_Vehicle_Moving_On_Packet_Trajectory: Error: vehicle->path_ptr == NULL\n");
    exit(1);
  }
  else if(vehicle->path_ptr->next == NULL)
  {
    printf("VADD_Is_Vehicle_Moving_On_Packet_Trajectory(): Error: vehicle->path_ptr->next == NULL\n");
    exit(1);
  }
  else
  {
    vehicle_tail_intersection_id = atoi(vehicle->path_ptr->vertex);
    vehicle_head_intersection_id = atoi(vehicle->path_ptr->next->vertex);
  }

  /** set packet_tail_intersection_id and packet_head_intersection_id to the tail and head nodes of the packet's current edge */
  if(packet_trajectory->order < packet_trajectory->size && packet_trajectory->current_packet_position != NULL)
  {
    packet_tail_intersection_id = packet_trajectory->current_packet_position->intersection_id;
  }
  else
  {
    printf("VADD_Is_Vehicle_Moving_On_Packet_Trajectory: Error: packet_trajectory->order(%d) >= packet_trajectory->size(%d) or packet_trajectory->current_packet_position != NULL\n", packet_trajectory->order, packet_trajectory->size);
    exit(1);
  }

  if(packet_trajectory->order + 1 < packet_trajectory->size && packet_trajectory->current_packet_position->next != NULL)
  {
    packet_head_intersection_id = packet_trajectory->current_packet_position->next->intersection_id;
  }
  else
  {
    printf("VADD_Is_Vehicle_Moving_On_Packet_Trajectory: Error: (packet_trajectory->order + 1)(%d) >= packet_trajectory->size(%d) or packet_trajectory->current_packet_position->next != NULL\n", packet_trajectory->order + 1, packet_trajectory->size);
    exit(1);
  }

  /** check whether the vehicle's moving edge is the same as the packet's current edge */
  if(vehicle_tail_intersection_id == packet_tail_intersection_id && vehicle_head_intersection_id == packet_head_intersection_id)
  {
    result = TRUE;
  }

  return result;
}

double VADD_Get_Initial_Minimum_Neighbor_EDD(parameter_t *param, struct_vehicle_t *vehicle)
{ //get the initial minimum neighboring EDD for vehicle according to param's data_forwarding_mode and vehicle_vanet_forwarding_type
	double min_neighbor_EDD = INF; //minimum neighbor EDD

	if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
	{
		if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_VEHICLE)
		{
			min_neighbor_EDD = vehicle->EDD;			
		}
		else if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY)
		{
			min_neighbor_EDD = vehicle->ptr_convoy_queue_node->leader_vehicle->EDD; 
		}
		else
		{
			printf("%s:%d: param->vehicle_vanet_forwarding_type(%d) is not supported!\n",
					__FUNCTION__, __LINE__,
					param->vehicle_vanet_forwarding_type);
			exit(1);
		}
	}
	else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
	{
		if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_VEHICLE)
		{
			min_neighbor_EDD = vehicle->EDD_for_download;			
		}
		else if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY)
		{
			min_neighbor_EDD = vehicle->ptr_convoy_queue_node->leader_vehicle->EDD_for_download; 
		}
		else
		{
			printf("%s:%d: param->vehicle_vanet_forwarding_type(%d) is not supported!\n",
					__FUNCTION__, __LINE__,
					param->vehicle_vanet_forwarding_type);
			exit(1);
		}
	}
	else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
	{
		if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_VEHICLE)
		{
			min_neighbor_EDD = vehicle->EDD_for_V2V;			
		}
		else if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY)
		{
			min_neighbor_EDD = vehicle->ptr_convoy_queue_node->leader_vehicle->EDD_for_V2V; 
		}
		else
		{
			printf("%s:%d: param->vehicle_vanet_forwarding_type(%d) is not supported!\n",
					__FUNCTION__, __LINE__,
					param->vehicle_vanet_forwarding_type);
			exit(1);
		}
	}
	else
	{
		printf("%s:%d: param->data_forwarding_mode(%d) is not supported!\n",
				__FUNCTION__, __LINE__,
				param->data_forwarding_mode);
		exit(1);
	}

	return min_neighbor_EDD;
}

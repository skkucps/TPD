/**
 * File: tpd.c
 * Description: implement the data structures and operations for TPD data forwarding.
 * Origin Date: 07/15/2013
 * Update Date: 07/17/2013
 * Maker: Jaehoon Paul Jeong, pauljeong@skku.edu
 */

#include "tpd.h"
#include "util.h"
#include "gsl-util.h"

/* taehwan 20140802 */
int g_margin_time = 0;

void TPD_Set_Margin_Time(int margin)
{
	g_margin_time = margin;
}

/* taehwna 20140731 */
double g_predicted_encounter_time[5000]={0,};

double TPD_Get_Predicted_Encounter_Time(int vehicle_id)
{
	return g_predicted_encounter_time[vehicle_id];
}


/** The Operations of Predicted Encounter Graph */

int TPD_Allocate_Predicted_Encounter_Graph(parameter_t *param, struct_vehicle_t *vehicle)
{ //allocate the memory of a predicted encounter graph for vehicle

	if(vehicle == NULL)
	{
		printf("%s:%d vehicle is NULL!\n", 
				__FUNCTION__, __LINE__);
		exit(1);
	}

	/* allocate the memory of vehicle's predicted_encounter_graph */
	vehicle->predicted_encounter_graph = (predicted_encounter_graph_t*) calloc(1,
											sizeof(predicted_encounter_graph_t));
	assert_memory(vehicle->predicted_encounter_graph);

	/* set predicted_encounter_graph's root to vehicle */
	vehicle->predicted_encounter_graph->root = vehicle;

	/* initialize vehicle's predicted_encounter_graph's minimum priority queue */
	InitQueue((queue_t*)&(vehicle->predicted_encounter_graph->Q), QTYPE_MINIMUM_PRIORITY); 

	/* initialize vehicle's predicted_encounter_graph's adjacency list queue */
	InitQueue((queue_t*)&(vehicle->predicted_encounter_graph->G), QTYPE_ADJACENCY_LIST); 
	vehicle->predicted_encounter_graph->G.bitmap_size = param->vehicle_maximum_number;
	
	vehicle->predicted_encounter_graph->G.bitmap = (boolean*)calloc(param->vehicle_maximum_number, sizeof(boolean));
	assert_memory(vehicle->predicted_encounter_graph->G.bitmap);

	vehicle->predicted_encounter_graph->G.bitmap_gnodes = (adjacency_list_queue_node_t**)calloc(param->vehicle_maximum_number, sizeof(adjacency_list_queue_node_t*));
	assert_memory(vehicle->predicted_encounter_graph->G.bitmap_gnodes);

	return 0;
}

int TPD_Allocate_Predicted_Encounter_Graph_For_Packet(packet_queue_node_t* packet)
{ //allocate the memory of a predicted encounter graph for packet 

	if(packet == NULL)
	{
		printf("%s:%d packet is NULL!\n", 
				__FUNCTION__, __LINE__);
		exit(1);
	}

	/* allocate the memory of packet's predicted_encounter_graph */
	packet->predicted_encounter_graph = (predicted_encounter_graph_t*) calloc(1,
											sizeof(predicted_encounter_graph_t));
	assert_memory(packet->predicted_encounter_graph);

	/* set predicted_encounter_graph's root to NULL because the root vehicle is not known yet */
	packet->predicted_encounter_graph->root = NULL;

	/* initialize packet's predicted_encounter_graph's minimum priority queue */
	InitQueue((queue_t*)&(packet->predicted_encounter_graph->Q), QTYPE_MINIMUM_PRIORITY); 

	/* initialize packet's predicted_encounter_graph's adjacency list queue */
	InitQueue((queue_t*)&(packet->predicted_encounter_graph->G), QTYPE_ADJACENCY_LIST); 
	packet->predicted_encounter_graph->G.bitmap_size = packet->vehicle_maximum_number;

	packet->predicted_encounter_graph->G.bitmap = (boolean*)calloc(packet->vehicle_maximum_number, sizeof(boolean));
	assert_memory(packet->predicted_encounter_graph->G.bitmap);

	packet->predicted_encounter_graph->G.bitmap_gnodes = (adjacency_list_queue_node_t**)calloc(packet->vehicle_maximum_number, sizeof(adjacency_list_queue_node_t*));
	assert_memory(packet->predicted_encounter_graph->G.bitmap_gnodes);

	return 0;
}

int TPD_Free_Predicted_Encounter_Graph(struct_vehicle_t *vehicle)
{ //free the memory of a predicted encounter graph for vehicle

	if(vehicle == NULL)
	{
		printf("%s:%d vehicle is NULL!\n", 
				__FUNCTION__, __LINE__);
		exit(1);
	}

	/* destroy vehicle's predicted_encounter_graph's minimum priority queue */
	DestroyQueue((queue_t*)&(vehicle->predicted_encounter_graph->Q)); 

	/* destroy vehicle's predicted_encounter_graph's adjacency list queue */
	DestroyQueue((queue_t*)&(vehicle->predicted_encounter_graph->G)); 
	
	/* free the memory of vehicle's predicted_encounter_graph */
	free(vehicle->predicted_encounter_graph);

	return 0;
}

int TPD_Free_Predicted_Encounter_Graph_For_Packet(packet_queue_node_t* packet)
{ //free the memory of a predicted encounter graph for packet 

	if(packet == NULL)
	{
		printf("%s:%d packet is NULL!\n", 
				__FUNCTION__, __LINE__);
		exit(1);
	}

	/* destroy packet's predicted_encounter_graph's minimum priority queue */
	DestroyQueue((queue_t*)&(packet->predicted_encounter_graph->Q)); 

	/* destroy packet's predicted_encounter_graph's adjacency list queue */
	DestroyQueue((queue_t*)&(packet->predicted_encounter_graph->G)); 
	
	/* free the memory of packet's predicted_encounter_graph */
	free(packet->predicted_encounter_graph);

	return 0;
}

int TPD_Reset_Queues_In_Predicted_Encounter_Graph(struct_vehicle_t *vehicle)
{ //reset two queues Q and G in the predicted encounter graph for vehicle

	if(vehicle == NULL)
	{
		printf("%s:%d vehicle is NULL!\n", 
				__FUNCTION__, __LINE__);
		exit(1);
	}

	/* reset vehicle's predicted_encounter_graph's minimum priority queue */
	ResetQueue((queue_t*)&(vehicle->predicted_encounter_graph->Q)); 

	/* reset vehicle's predicted_encounter_graph's adjacency list queue */
	ResetQueue((queue_t*)&(vehicle->predicted_encounter_graph->G)); 
	
	return 0;
}

int TPD_Reset_Queues_In_Predicted_Encounter_Graph_For_Packet(packet_queue_node_t* packet)
{ //reset two queues Q and G in the predicted encounter graph for packet 

	if(packet == NULL)
	{
		printf("%s:%d packet is NULL!\n", 
				__FUNCTION__, __LINE__);
		exit(1);
	}

	/* reset packet's predicted_encounter_graph's minimum priority queue */
	ResetQueue((queue_t*)&(packet->predicted_encounter_graph->Q)); 

	/* reset packet's predicted_encounter_graph's adjacency list queue */
	ResetQueue((queue_t*)&(packet->predicted_encounter_graph->G)); 
	
	return 0;
}

double TPD_Compute_EDR_and_EDD(double current_time,
		parameter_t *param,
		struct_vehicle_t *src_vehicle,
		struct_vehicle_t *dst_vehicle,
		boolean encounter_graph_display_flag)
{ //compute the EDR (Expected Delivery Ratio) and EDD (Expected Delivery Delay) of src_vehicle for dst_vehicle
	double EDR = 0; //Expected Delivery Ratio (EDR)

	/* compute EDR and EDD without constructing an encounter graph for greedy routing */
	EDR = TPD_Compute_EDR_and_EDD_For_Greedy_Routing(current_time,
				param,
				src_vehicle,
				dst_vehicle,
				encounter_graph_display_flag);

	return EDR;
}

double TPD_Compute_EDR_and_EDD_For_Greedy_Routing(double current_time,
		parameter_t *param,
		struct_vehicle_t *src_vehicle,
		struct_vehicle_t *dst_vehicle,
		boolean encounter_graph_display_flag)
{ //compute the EDR (Expected Delivery Ratio) and EDD (Expected Delivery Delay) of src_vehicle for dst_vehicle for Greedy Routing
	double EDR = 0; //Expected Delivery Ratio (EDR)

#if TPD_VEHICLE_TRAJECTORY_PRINT_FLAG /* [ */
	//if (current_time > 7221 && current_time < 7222)
	//if (current_time > 23598 && current_time < 23630)
		show_trajectory_and_arrival_time_for_all_vehicles();
	//show the vehicle trajectory along with the arrival time per path node along the vehicle trajectory for all the vehicles in vehicle_list
#endif /* ] */

#if TPD_VEHICLE_TRAJECTORY_STORE_FLAG /* [ */
	store_trajectory_and_arrival_time_for_all_vehicles();
	//store the vehicle trajectory along with the arrival time per path node along the vehicle trajectory for all the vehicles in vehicle_list
#endif /* ] */

	/* construct a predicted encounter graph of src_vehicle toward dst_vehicle */
	EDR = TPD_Construct_Predicted_Encounter_Graph(current_time, param, src_vehicle, dst_vehicle, encounter_graph_display_flag);
	printf("========> EDR = %.2f\n",EDR);
	return EDR;
}

double TPD_Compute_EDR_and_EDD_For_Source_Routing(double current_time,
		parameter_t *param,
		packet_queue_node_t* packet,
		struct_vehicle_t *src_vehicle,
		struct_vehicle_t *dst_vehicle,
		boolean encounter_graph_display_flag)
{ //compute the EDR (Expected Delivery Ratio) and EDD (Expected Delivery Delay) of src_vehicle for dst_vehicle with constructing an encounter graph to deliver packet from src_vehicle to dst_vehicle by Source Routing
	double EDR = 0; //Expected Delivery Ratio (EDR)

#if TPD_VEHICLE_TRAJECTORY_PRINT_FLAG /* [ */
	show_trajectory_and_arrival_time_for_all_vehicles();
	//show the vehicle trajectory along with the arrival time per path node along the vehicle trajectory for all the vehicles in vehicle_list
#endif /* ] */

#if TPD_VEHICLE_TRAJECTORY_STORE_FLAG /* [ */
	store_trajectory_and_arrival_time_for_all_vehicles();
	//store the vehicle trajectory along with the arrival time per path node along the vehicle trajectory for all the vehicles in vehicle_list
#endif /* ] */

	/* construct a predicted encounter graph of src_vehicle toward dst_vehicle for packet */
	EDR = TPD_Construct_Predicted_Encounter_Graph_For_Packet(current_time, param, packet, src_vehicle, dst_vehicle, encounter_graph_display_flag);

	return EDR;
}

double TPD_Construct_Predicted_Encounter_Graph(double current_time,
		parameter_t *param, 
		struct_vehicle_t *src_vehicle, 
		struct_vehicle_t *dst_vehicle,
		boolean encounter_graph_display_flag)
{ //construct a predicted encounter graph from src_vehicle to dst_vehicle and return Expected Delivery Ratio (EDR).
	minimum_priority_queue_t *Q = NULL; //pointer to the minimum priority queue
	minimum_priority_queue_node_t *pQueueNode = NULL; //pointer to a minimum priority queue node
	minimum_priority_queue_node_t *pChildQueueNode = NULL; //pointer to a minimum priority queue node for a child graph node
	minimum_priority_queue_node_t mpq_qnode; //queue node for minimum priority queue
	adjacency_list_queue_t *G = NULL; //pointer to adjacency list queue
	adjacency_list_queue_node_t *pGraphNode = NULL; //pointer to a graph node in adjacency list queue
	adjacency_list_queue_node_t *pParentGraphNode = NULL; //pointer to a parent graph node in adjacency list queue
	adjacency_list_queue_node_t graph_qnode; //queue node for a graph node in adjacency list queue
	neighbor_list_queue_node_t neighbor_qnode; //queue node for a neighbor list queue node
	parent_list_queue_node_t parent_qnode; //queue node for a parent list queue node
	struct struct_vehicle* vehicle_list = NULL; //pointer to vehicle_list 
	struct struct_vehicle* current_vehicle = NULL; //pointer to the current vehicle dequeued from Q
	struct struct_vehicle* vehicle = NULL; //pointer to vehicle node
	boolean flag = FALSE; //flag
	int tail_vertex = 0; //tail vertex for encounter edge
	int head_vertex = 0; //head vertex for encounter edge
	double edge_length = 0; //the length of the edge (tail_vertex, head_vertex)

	double T_threshold = 0; //the encounter time of vehicle and its parent vehicle that is used to determine the valid encounter of two vehicles during the selection of child vehicles in the predicted encounter graph 

	double P_encounter = 0; //encounter probability
	double T_encounter = 0; //encounter time
	double D_encounter = 0; //travel time (i.e., delay) for encounter
	double O_encounter = 0; //offset in the encountered edge where src_vehicle and vehicle encounter
	boolean destination_flag = FALSE; //flag to indicate whether the graph node for dst_vehicle exists in the graph G
	double EDR = 0; //Expected Delivery Ratio (EDR)

#if 0 /* [ */
	if(current_time > 8012 && src_vehicle->id == 94)
	{
		printf("%s:%d [%.0f] src_vehicle(%d) is traced\n",
				__FUNCTION__, __LINE__, 
				(float)current_time, 
				src_vehicle->id);
	}
#endif /* ] */

	if(src_vehicle == NULL)
	{
		printf("%s:%d src_vehicle is NULL\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}
	else if (dst_vehicle == NULL)
	{
		printf("%s:%d dst_vehicle is NULL\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}

	/* check whether src_vehicle is dst_vehicle or not. 
	 * If so, return EDR = 1. */
	if(src_vehicle->id == dst_vehicle->id)
	{
		printf("\n**** src vehicle == dst vehicle %d ***\n\n",src_vehicle->id);
		EDR = 1;
		return EDR;
	}

	/* set up Q and G with src_vehicle's predicted_encounter_graph */
	Q = &(src_vehicle->predicted_encounter_graph->Q);
	G = &(src_vehicle->predicted_encounter_graph->G);

	/* enqueue src_vehicle into src_vehicle's predicted_encounter_graph's queue Q */
	memset(&mpq_qnode, 0, sizeof(minimum_priority_queue_node_t));
	mpq_qnode.id = src_vehicle->id;
	mpq_qnode.key = current_time; //the encounter time with itself is current_time
	mpq_qnode.object_type = QUEUE_NODE_OBJECT_VEHICLE;
	mpq_qnode.object = src_vehicle;
	Enqueue((queue_t*)Q, (queue_node_t*)&mpq_qnode);
 
	/** GRAPH CONSTRUCTION PHASE: construct an encounter graph */
	/* Step 1: Exit the loop if the heap Q is empty or all of the vehicles are 
	 *			visited. */

	/* Step 2: Dequeue the root from Q and set it to the current vehicle. */

	/* Step 3: If a graph node for current_vehicle does not exist, 
	 *		create a graph node for the current vehicle and insert it 
	 *		into G. Let the parent graph node point to this graph node as a 
	 *		child graph node.
	 *		Otherwise, delete this queue node and go to Step 1. */

	/* Step 4: Check whether current_vehicle is dst_vehicle or not:
	 *		If current_vehicle is dst_vehicle, go to Step 1.
	 *		Otherwise, go to Step 5.
	 *		*/

	/* Step 5: Search the encountered vehicles met by the current vehicle
	 *		with a certain encounter probability (>= threshold (e.g., 
	 *		0.7)) as child vehicles that are inserted into Q. Perform the following: 
	 *			- let the current vehicle point to its parent vehicle.
	 *			- let the parent vehicle point to the current vehicle.
	 *		After the search, go to Step 1. 
	 *		*/
int tempX = 1;
	do //do-while-1	
	{
		/* Step 1: Exit the loop if the heap Q is empty or all of the vehicles are 
		 *			visited. */
		if(Q->size == 0)
		{
			//printf("         taehwan11291020 TPD Construct Q empty src_vehicle = %d\n",src_vehicle->id);
			break;
		}

		/* Step 2: Dequeue the root from Q and set it to the current vehicle. */
		pQueueNode = (minimum_priority_queue_node_t*)Dequeue((queue_t*)Q);
		current_vehicle = (struct_vehicle_t*)pQueueNode->object;
		//printf("         taehwan11291011 TPD Construct current_vehicle:%d\n",current_vehicle->id);
#if 0 /* [ */
			if(src_vehicle->id == 325 && current_time >= 7239 && current_vehicle->id == 155)
			{
				printf("%s:%d [%0.f] current_vehicle(%d) is traced - 0.\n", 
						__FUNCTION__, __LINE__,
						current_time,
						current_vehicle->id);
			}
#endif /* ] */

		/* Step 3: If a graph node for current_vehicle does not exist, 
		 *		insert a graph node for the current vehicle into G and
		 *		let the parent graph node point to this graph node as a 
		 *		child graph node.
		 *		If a graph node for current_vehicle exist, but current_vehicle
		 *		is dst_vehicle, let the parent graph node point to this graph
		 *		node as a child graph node.
		 *		Otherwise, delete this queue node and go to Step 1. */

		if((G->bitmap[current_vehicle->id-1] == TRUE) &&
				(current_vehicle->id != dst_vehicle->id))
		{ //since the graph node for current_vehicle already exists, delete this queue node and move on the next vehicle in Q
			/* delete pQueueNode */
			//printf("         taehwan11291028 TPD Construct DestroyQueue %d\n",current_vehicle->id);
			DestroyQueueNode(Q->type, (queue_node_t*)pQueueNode); 
			continue;
		}

		if(G->bitmap[current_vehicle->id-1] == FALSE)
		{
			/* enqueue current_vehicle into src_vehicle's predicted_encounter_graph's queue G */
			memset(&graph_qnode, 0, sizeof(adjacency_list_queue_node_t));
			graph_qnode.id = current_vehicle->id;
			graph_qnode.object_type = QUEUE_NODE_OBJECT_VEHICLE;
			graph_qnode.object = current_vehicle;
			pGraphNode = (adjacency_list_queue_node_t*)Enqueue((queue_t*)G, (queue_node_t*)&graph_qnode);

			/* set up bitmap variables */
			G->bitmap[current_vehicle->id-1] = TRUE;
			G->bitmap_gnodes[current_vehicle->id-1] = pGraphNode;

			/* check whether this graph node is source vehicle or destination vehicle */
			if(current_vehicle->id == src_vehicle->id)
			{
				/* let G->src_vehicle_gnode point to the graph node for source vehicle. */
				G->src_vehicle_gnode = pGraphNode;
			}
			else if(current_vehicle->id == dst_vehicle->id)
			{
				/* let G->dst_vehicle_gnode point to the graph node for destination vehicle. */
				G->dst_vehicle_gnode = pGraphNode;
			}
		}
		else if((G->bitmap[current_vehicle->id-1] == TRUE) &&
				(current_vehicle->id == dst_vehicle->id))
		{
			/* find the pointer to the graph node for dst_vehicle. */
			pGraphNode = G->dst_vehicle_gnode;
			if(pGraphNode == NULL)
			{	/* an unexpected situation happened. */ 
    			printf("%s:%d an unexpected case happened.\n",
						__FUNCTION__, __LINE__);
				exit(1);
			}
		}
		else
		{
			/* an unexpected case happened. */
			printf("%s:%d an unexpected case happened.\n",
					__FUNCTION__, __LINE__);
			exit(1);
		}

		/* let the parent graph node of this graph node point to this graph
		 * as a child graph node (i.e., neighbor node) and let this graph point
		 * to its parent graph node in its parent_list. */
		if(pQueueNode->parent_list.size > 0)
		{
			/* let the parent graph node of current_vehicle point to the
			 * graph node of current_vehicle in the parent graph node's
			 * neighbor_list.
			 * Note: vehicle(i.e., parent) will encounter current_vehicle (i.e., child) 
			 * in its edge (tail_vertex, head_vertex: edge_offset) at time T_encounter
			 * where vehicle is parent and current_vehicle is child. */
			memset(&neighbor_qnode, 0, sizeof(neighbor_list_queue_node_t));
			neighbor_qnode.id = current_vehicle->id;
			neighbor_qnode.weight = pQueueNode->parent_list.head.next->weight; //current_vehicle (as child node) will encounter its parent vehicle after the delay of pQueueNode->parent_list.head.next->weight 
			neighbor_qnode.T_encounter = pQueueNode->parent_list.head.next->T_encounter; //current_vehicle (as child node) will encounter its parent vehicle at the time of pQueueNode->parent_list.head.next->T_encounter
			neighbor_qnode.P_encounter = pQueueNode->parent_list.head.next->P_encounter; //current_vehicle (as child node) will encounter its parent vehicle with the probability of pQueueNode->parent_list.head.next->P_encounter
			neighbor_qnode.tail_vertex = pQueueNode->parent_list.head.next->head_vertex; //tail_vertex for the encounter edge
			neighbor_qnode.head_vertex = pQueueNode->parent_list.head.next->tail_vertex; //head_vertex for the encounter edge
			neighbor_qnode.edge_length = pQueueNode->parent_list.head.next->edge_length; //length of the encounter edge
			neighbor_qnode.edge_offset = neighbor_qnode.edge_length - pQueueNode->parent_list.head.next->edge_offset; //offset for the encounter edge

			neighbor_qnode.object_type = QUEUE_NODE_OBJECT_VEHICLE;
			neighbor_qnode.object = current_vehicle;
			neighbor_qnode.graph_qnode = pGraphNode;
			pParentGraphNode = pQueueNode->parent_list.head.next->graph_qnode; //let pParentGraphNode point to the parent graph node pointed by pQueueNode->parent_list.head.next->parent_qnode
			Enqueue((queue_t*)&(pParentGraphNode->neighbor_list), (queue_node_t*)&neighbor_qnode);

			/* let the graph node of current_vehicle point to the parent graph
			 * node of current_vehicle in the graph node's parent_list. */
			Enqueue((queue_t*)&(pGraphNode->parent_list), (queue_node_t*)(pQueueNode->parent_list.head.next));
		}

		/* set T_threshold to the encounter time of vehicle and its parent vehicle 
		 * for filtering the road segments infeasible in term of encounter time
		 * in the search of an encountered road segment in 
		 * TPD_Compute_Encounter_Probability(). That is, The encounter time in an 
		 * encountered road segment should be later than T_threshold. */ 
		T_threshold = pQueueNode->key; 

		/* delete pQueueNode */
		DestroyQueueNode(Q->type, (queue_node_t*)pQueueNode); 

		/* Step 4: Check whether current_vehicle is dst_vehicle or not:
		 *		If current_vehicle is dst_vehicle, go to Step 1.
		 *		Otherwise, go to Step 5.
		 *		*/
		if(current_vehicle->id == dst_vehicle->id)
		{
			continue;
		}

		/* Step 5: Search the encountered vehicles met by the current vehicle
		 *		with a certain encounter probability (>= threshold (e.g., 
		 *		0.7)) as child vehicles into Q. Perform the following: 
		 *			- let the current vehicle point to its parent vehicle.
		 *			- let the parent vehicle point to the current vehicle.
		 *		After the search, go to Step 1. 
		 *		*/
		//T_threshold = pQueueNode->key; //the encounter time of vehicle and its parent vehicle  
		vehicle_list = get_vehicle_list(); //get the pointer to vehicle_list defined in util.c
		
		vehicle = vehicle_list->next;
	/*	printf("%.2f] taehwan11272130 TPD Construct ",current_time);

		while(vehicle != vehicle_list)
		{
			if (vehicle->id == current_vehicle->id)
			{
				vehicle = vehicle->next;
				continue;
			}
			printf("%s%d",(vehicle == vehicle_list->next?"":","),vehicle->id);
			vehicle = vehicle->next;
		}
		printf(")\n");
	*/	
		vehicle = vehicle_list->next;
		//printf("         taehwan11241602 TPD Construct T_Threshold=%.2f Start current_id = %d src_id = %d QSize = %d\n",T_threshold,current_vehicle->id,src_vehicle->id,Q->size);
		//TPD_Print_Vehicle_Trajectory(current_vehicle);
		while(vehicle != vehicle_list) //while-1
		{
			if(vehicle->id == current_vehicle->id)
			{
				vehicle = vehicle->next; //move on the next vehicle in vehicle_list
				continue;
			}

#if 0 /* [ */			
			flag = TPD_Do_Vehicles_Encounter(current_time, param, current_vehicle, vehicle, &tail_vertex, &head_vertex, &edge_length, &P_encounter, &T_encounter, &D_encounter, &O_encounter); //check whether src_vehicle and vehicle encounter at the offset O_encounter in an edge (tail_vertex, head_vertex) with P_encounter of at least encounter_probability_threshold at time T_encounter after the travel time (i.e., delay) of D_encounter.
#endif /* ] */
			
			flag = TPD_Compute_Encounter_Probability(param, 
					vehicle, 
					current_vehicle,
					T_threshold,
					&tail_vertex, 
					&head_vertex, 
					&edge_length, 
					&P_encounter, 
					&T_encounter, 
					&D_encounter, 
					&O_encounter); //check whether src_vehicle and vehicle encounter at the offset O_encounter in an edge (tail_vertex, head_vertex) with P_encounter of at least encounter_probability_threshold at time T_encounter after the travel time (i.e., delay) of D_encounter.
		        //printf("%.2f] taehwan11241602 TPD Construct (v1,v2):(%d,%d) (tail,head):(%d,%d) EDR:%.2f\n",current_time,vehicle->id,current_vehicle->id,tail_vertex,head_vertex,P_encounter);
			if(flag == FALSE)
			{
				vehicle = vehicle->next; //move on the next vehicle in vehicle_list
				continue;
			}
			//printf("         taehwan11241602 TPD Construct %3d->%3d(%.2f %s-%s) at t=%.2f %.2f/%.2f %2d-%2d is %.4f %s\n",current_vehicle->id,vehicle->id,vehicle->path_current_edge_offset,vehicle->current_pos_in_digraph.tail_node,vehicle->current_pos_in_digraph.head_node,T_encounter,O_encounter,edge_length,tail_vertex,head_vertex,P_encounter,(vehicle->id==1?"*":""));
			//if (current_time >= 8473 && current_time < 8473.60) 
			//	TPD_Print_Vehicle_Trajectory(vehicle);
#if 0 /* [ */
			if(vehicle->id == dst_vehicle->id)
			{
				printf("%s:%d vehicle(%d) for current_vehicle(%d) is dst_vehicle.\n", 
						__FUNCTION__, __LINE__,
						vehicle->id, current_vehicle->id);
			}
#endif /* ] */

			/* enqueue vehicle into src_vehicle's predicted_encounter_graph's queue Q in the ascending order of the encounter time T_encounter */
           		memset(&mpq_qnode, 0, sizeof(minimum_priority_queue_node_t));
            		mpq_qnode.id = vehicle->id;
            		mpq_qnode.key = T_encounter; //the encounter time of current_vehicle and vehicle 
            		mpq_qnode.object_type = QUEUE_NODE_OBJECT_VEHICLE;
            		mpq_qnode.object = vehicle;
            		pChildQueueNode = (minimum_priority_queue_node_t*)Enqueue_By_KeyAscendingOrder((queue_t*)Q, (queue_node_t*)&mpq_qnode);

			/* let vehicle's mpq_qnode have the pointer to its parent graph node for 
			 * current_vehicle pointed by pGraphNode.
			 * Note: vehicle(i.e., child) will encounter current_vehicle (i.e., parent) 
			 * in its edge (tail_vertex, head_vertex: edge_offset) at time T_encounter
			 * where vehicle is child and current_vehicle is parent. */
			memset(&parent_qnode, 0, sizeof(parent_list_queue_node_t));
			parent_qnode.id = current_vehicle->id;
			parent_qnode.weight = D_encounter; //current_vehicle (as parent) will encounter vehicle after the delay of D_encounter
			parent_qnode.T_encounter = T_encounter; //encounter time
			parent_qnode.P_encounter = P_encounter; //encounter probability
			parent_qnode.tail_vertex = tail_vertex; //tail_vertex for the encounter edge
			parent_qnode.head_vertex = head_vertex; //head_vertex for the encounter edge
			parent_qnode.edge_length = edge_length; //length of the encounter edge
			parent_qnode.edge_offset = O_encounter; //offset for the encounter edge
			parent_qnode.object_type = QUEUE_NODE_OBJECT_VEHICLE;
			parent_qnode.object = current_vehicle;
			parent_qnode.graph_qnode = pGraphNode; //let graph_qnode point to the graph node in adjacency list queue pointed by pGraphNode
			Enqueue((queue_t*)&(pChildQueueNode->parent_list), (queue_node_t*)&parent_qnode);

			/* taehwan 20140731 */
			//printf("Encounter Time %d = %.2f\n",current_vehicle->id,T_encounter);
			g_predicted_encounter_time[current_vehicle->id] = T_encounter;
			//printf("%d = %.2f\n",current_vehicle->id,T_encounter);
			
			/* take the next vehicle */
			vehicle = vehicle->next;
		} //end of while-1

	} while(1); //end of do-while-1

	if(G->dst_vehicle_gnode != NULL)
	{
		
#if 0 /* [ */
			if(src_vehicle->id == 153)
			{
				printf("%s:%d src_vehicle(%d) is traced.\n", 
						__FUNCTION__, __LINE__,
						src_vehicle->id);
			}
#endif /* ] */

		/* GRAPH PRUNING PHASE: remove the paths from graph nodes that cannot 
		 * reach the destination vehicle node to the src_vehicle. */
		TPD_Prune_Encounter_Graph(G);

		/* EDR COMPUTATION PHASE: compute Expected Delivery Ratio (EDR) with 
		 * the final predicted encounter graph */
		if(param->tpd_encounter_graph_optimization_flag)
		{
			src_vehicle->EDR_for_V2V = TPD_Compute_EDR_For_Encounter_Graph_By_DP(G);
		}
		else
		{
			src_vehicle->EDR_for_V2V = TPD_Compute_EDR_For_Encounter_Graph(G);
		}

		/* EDD COMPUTATION PHASE: compute Expected Delivery Delay (EDD) with 
		 * the final predicted encounter graph */
		src_vehicle->EDD_for_V2V = TPD_Compute_EDD_For_Encounter_Graph(G);

#if TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FLAG /* [ */
		TPD_Perform_BFS_For_Encounter_Graph(G, encounter_graph_display_flag);
		/* perform the Breadth-First-Search (BFS) for the given encounter graph G from source vehicle to destination vehicle in G */
#endif /* ] */
	}
	else
	{
		//if (src_vehicle->id == 193)
		//printf("%.2f]G->dst_node is NULL!!! %d but EDR is %.2f\n",current_time,src_vehicle->id,src_vehicle->EDR_for_V2V);
		// taehwan 20140826
		// EDR MUST be 0 when dst_vehicle is NULL.
		src_vehicle->EDR_for_V2V = 0;
	}

	/* reset Q and G in predicted_encounter_graph by emptying the queue nodes. */
	TPD_Reset_Queues_In_Predicted_Encounter_Graph(src_vehicle);

	//if (src_vehicle->id == 193)
	//	printf("\n%.2f] EDR %d is %.2f\n\n",current_time,src_vehicle->id,src_vehicle->EDR_for_V2V);
	
	return src_vehicle->EDR_for_V2V;
}

double TPD_Construct_Predicted_Encounter_Graph_For_Packet(double current_time,
		parameter_t *param, 
		packet_queue_node_t *packet,
		struct_vehicle_t *src_vehicle, 
		struct_vehicle_t *dst_vehicle,
		boolean encounter_graph_display_flag)
{ //construct a predicted encounter graph from src_vehicle to dst_vehicle for packet and return Expected Delivery Ratio (EDR).
	minimum_priority_queue_t *Q = NULL; //pointer to the minimum priority queue
	minimum_priority_queue_node_t *pQueueNode = NULL; //pointer to a minimum priority queue node
	minimum_priority_queue_node_t *pChildQueueNode = NULL; //pointer to a minimum priority queue node for a child graph node
	minimum_priority_queue_node_t mpq_qnode; //queue node for minimum priority queue
	adjacency_list_queue_t *G = NULL; //pointer to adjacency list queue
	adjacency_list_queue_node_t *pGraphNode = NULL; //pointer to a graph node in adjacency list queue
	adjacency_list_queue_node_t *pParentGraphNode = NULL; //pointer to a parent graph node in adjacency list queue
	adjacency_list_queue_node_t graph_qnode; //queue node for a graph node in adjacency list queue
	neighbor_list_queue_node_t neighbor_qnode; //queue node for a neighbor list queue node
	parent_list_queue_node_t parent_qnode; //queue node for a parent list queue node
	struct struct_vehicle* vehicle_list = NULL; //pointer to vehicle_list 
	struct struct_vehicle* current_vehicle = NULL; //pointer to the current vehicle dequeued from Q
	struct struct_vehicle* vehicle = NULL; //pointer to vehicle node
	boolean flag = FALSE; //flag
	int tail_vertex = 0; //tail vertex for encounter edge
	int head_vertex = 0; //head vertex for encounter edge
	double edge_length = 0; //the length of the edge (tail_vertex, head_vertex)

	double T_threshold = 0; //the encounter time of vehicle and its parent vehicle that is used to determine the valid encounter of two vehicles during the selection of child vehicles in the predicted encounter graph 

	double P_encounter = 0; //encounter probability
	double T_encounter = 0; //encounter time
	double D_encounter = 0; //travel time (i.e., delay) for encounter
	double O_encounter = 0; //offset in the encountered edge where src_vehicle and vehicle encounter
	boolean destination_flag = FALSE; //flag to indicate whether the graph node for dst_vehicle exists in the graph G
	double EDR = 0; //Expected Delivery Ratio (EDR)

	//printf("TPD_Construct_Predicted_Encounter_Graph_For_Packet %.2f\n",current_time);

#if 1 /* [ */
	if(current_time > 7336 && src_vehicle->id == 300)
	{
		printf("%s:%d [%.0f] src_vehicle(%d) is traced\n",
				__FUNCTION__, __LINE__, 
				(float)current_time, 
				src_vehicle->id);
	}
#endif /* ] */

	if(packet == NULL)
	{
		printf("%s:%d packet is NULL\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}
	else if(src_vehicle == NULL)
	{
		printf("%s:%d src_vehicle is NULL\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}
	else if (dst_vehicle == NULL)
	{
		printf("%s:%d dst_vehicle is NULL\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}

	/* check whether src_vehicle is dst_vehicle or not. 
	 * If so, return EDR = 1. */
	if(src_vehicle->id == dst_vehicle->id)
	{
		EDR = 1;
		return EDR;
	}

	/* set up Q and G with packet's predicted_encounter_graph */
	Q = &(packet->predicted_encounter_graph->Q);
	G = &(packet->predicted_encounter_graph->G);

	/* enqueue src_vehicle into packet's predicted_encounter_graph's queue Q */
	memset(&mpq_qnode, 0, sizeof(minimum_priority_queue_node_t));
	mpq_qnode.id = src_vehicle->id;
	mpq_qnode.key = current_time; //the encounter time with itself is current_time
	mpq_qnode.object_type = QUEUE_NODE_OBJECT_VEHICLE;
	mpq_qnode.object = src_vehicle;
	Enqueue((queue_t*)Q, (queue_node_t*)&mpq_qnode);

	//printf("TPD_Construct_Predicted_Encounter_Graph_For_Packet\n");


	/** GRAPH CONSTRUCTION PHASE: construct an encounter graph */
	/* Step 1: Exit the loop if the heap Q is empty or all of the vehicles are 
	 *			visited. */

	/* Step 2: Dequeue the root from Q and set it to the current vehicle. */

	/* Step 3: If a graph node for current_vehicle does not exist, 
	 *		create a graph node for the current vehicle and insert it 
	 *		into G. Let the parent graph node point to this graph node as a 
	 *		child graph node.
	 *		Otherwise, delete this queue node and go to Step 1. */

	/* Step 4: Check whether current_vehicle is dst_vehicle or not:
	 *		If current_vehicle is dst_vehicle, go to Step 1.
	 *		Otherwise, go to Step 5.
	 *		*/

	/* Step 5: Search the encountered vehicles met by the current vehicle
	 *		with a certain encounter probability (>= threshold (e.g., 
	 *		0.7)) as child vehicles into Q. Perform the following: 
	 *			- let the current vehicle point to its parent vehicle.
	 *			- let the parent vehicle point to the current vehicle.
	 *		After the search, go to Step 1. 
	 *		*/

//printf("\n$$$$$$\n Test\n$$$$$$\n");
	do //do-while-1	
	{
		/* Step 1: Exit the loop if the heap Q is empty or all of the vehicles are 
		 *			visited. */
		if(Q->size == 0)
		{
			break;
		}

		/* Step 2: Dequeue the root from Q and set it to the current vehicle. */
		pQueueNode = (minimum_priority_queue_node_t*)Dequeue((queue_t*)Q);
		current_vehicle = (struct_vehicle_t*)pQueueNode->object;
//printf("   CurrentVehicle %d\n",current_vehicle->id);
		/* Step 3: If a graph node for current_vehicle does not exist, 
		 *		insert a graph node for the current vehicle into G and
		 *		let the parent graph node point to this graph node as a 
		 *		child graph node.
		 *		If a graph node for current_vehicle exist, but current_vehicle
		 *		is dst_vehicle, let the parent graph node point to this graph
		 *		node as a child graph node.
		 *		Otherwise, delete this queue node and go to Step 1. */

		if((G->bitmap[current_vehicle->id-1] == TRUE) &&
				(current_vehicle->id != dst_vehicle->id))
		{ //since the graph node for current_vehicle already exists, delete this queue node and move on the next vehicle in Q
			/* delete pQueueNode */
			DestroyQueueNode(Q->type, (queue_node_t*)pQueueNode); 
			continue;
		}

		if(G->bitmap[current_vehicle->id-1] == FALSE)
		{
			/* enqueue current_vehicle into packet's predicted_encounter_graph's queue G */
			memset(&graph_qnode, 0, sizeof(adjacency_list_queue_node_t));
			graph_qnode.id = current_vehicle->id;
			graph_qnode.object_type = QUEUE_NODE_OBJECT_VEHICLE;
			graph_qnode.object = current_vehicle;
			pGraphNode = (adjacency_list_queue_node_t*)Enqueue((queue_t*)G, (queue_node_t*)&graph_qnode);

			/* set up bitmap variables */
			G->bitmap[current_vehicle->id-1] = TRUE;
			G->bitmap_gnodes[current_vehicle->id-1] = pGraphNode;

			/* check whether this graph node is source vehicle or destination vehicle */
			if(current_vehicle->id == src_vehicle->id)
			{
				/* let G->src_vehicle_gnode point to the graph node for source vehicle. */
				G->src_vehicle_gnode = pGraphNode;
			}
			else if(current_vehicle->id == dst_vehicle->id)
			{
				/* let G->dst_vehicle_gnode point to the graph node for destination vehicle. */
				G->dst_vehicle_gnode = pGraphNode;
			}
		}
		else if((G->bitmap[current_vehicle->id-1] == TRUE) &&
				(current_vehicle->id == dst_vehicle->id))
		{
			/* find the pointer to the graph node for dst_vehicle. */
			pGraphNode = G->dst_vehicle_gnode;
			if(pGraphNode == NULL)
			{	/* an unexpected situation happened. */ 
    			printf("%s:%d an unexpected case happened.\n",
						__FUNCTION__, __LINE__);
				exit(1);
			}
		}
		else
		{
			/* an unexpected case happened. */
			printf("%s:%d an unexpected case happened.\n",
					__FUNCTION__, __LINE__);
			exit(1);
		}

		/* let the parent graph node of this graph node point to this graph
		 * as a child graph node (i.e., neighbor node) and let this graph point
		 * to its parent graph node in its parent_list. */
		if(pQueueNode->parent_list.size > 0)
		{
			/* let the parent graph node of current_vehicle point to the
			 * graph node of current_vehicle in the parent graph node's
			 * neighbor_list.
			 * Note: vehicle(i.e., parent) will encounter current_vehicle (i.e., child) 
			 * in its edge (tail_vertex, head_vertex: edge_offset) at time T_encounter
			 * where vehicle is parent and current_vehicle is child. */
			memset(&neighbor_qnode, 0, sizeof(neighbor_list_queue_node_t));
			neighbor_qnode.id = current_vehicle->id;
			neighbor_qnode.weight = pQueueNode->parent_list.head.next->weight; //current_vehicle (as child node) will encounter its parent vehicle after the delay of pQueueNode->parent_list.head.next->weight 
			neighbor_qnode.T_encounter = pQueueNode->parent_list.head.next->T_encounter; //current_vehicle (as child node) will encounter its parent vehicle at the time of pQueueNode->parent_list.head.next->T_encounter
			neighbor_qnode.P_encounter = pQueueNode->parent_list.head.next->P_encounter; //current_vehicle (as child node) will encounter its parent vehicle with the probability of pQueueNode->parent_list.head.next->P_encounter
			neighbor_qnode.tail_vertex = pQueueNode->parent_list.head.next->head_vertex; //tail_vertex for the encounter edge
			neighbor_qnode.head_vertex = pQueueNode->parent_list.head.next->tail_vertex; //head_vertex for the encounter edge
			neighbor_qnode.edge_length = pQueueNode->parent_list.head.next->edge_length; //length of the encounter edge
			neighbor_qnode.edge_offset = neighbor_qnode.edge_length - pQueueNode->parent_list.head.next->edge_offset; //offset for the encounter edge

			neighbor_qnode.object_type = QUEUE_NODE_OBJECT_VEHICLE;
			neighbor_qnode.object = current_vehicle;
			neighbor_qnode.graph_qnode = pGraphNode;
			pParentGraphNode = pQueueNode->parent_list.head.next->graph_qnode; //let pParentGraphNode point to the parent graph node pointed by pQueueNode->parent_list.head.next->parent_qnode
			Enqueue((queue_t*)&(pParentGraphNode->neighbor_list), (queue_node_t*)&neighbor_qnode);

			/* let the graph node of current_vehicle point to the parent graph
			 * node of current_vehicle in the graph node's parent_list. */
			Enqueue((queue_t*)&(pGraphNode->parent_list), (queue_node_t*)(pQueueNode->parent_list.head.next));
		}

		/* set T_threshold to the encounter time of vehicle and its parent vehicle 
		 * for filtering the road segments infeasible in term of encounter time
		 * in the search of an encountered road segment in 
		 * TPD_Compute_Encounter_Probability(). That is, the encounter time in an 
		 * encountered road segment should be later than T_threshold. */ 
		T_threshold = pQueueNode->key; 

		/* delete pQueueNode */
		DestroyQueueNode(Q->type, (queue_node_t*)pQueueNode); 

		/* Step 4: Check whether current_vehicle is dst_vehicle or not:
		 *		If current_vehicle is dst_vehicle, go to Step 1.
		 *		Otherwise, go to Step 5.
		 *		*/
		if(current_vehicle->id == dst_vehicle->id)
		{
			continue;
		}

		/* Step 5: Search the encountered vehicles met by the current vehicle
		 *		with a certain encounter probability (>= threshold (e.g., 
		 *		0.7)) as child vehicles into Q. Perform the following: 
		 *			- let the current vehicle point to its parent vehicle.
		 *			- let the parent vehicle point to the current vehicle.
		 *		After the search, go to Step 1. 
		 *		*/
		//T_threshold = pQueueNode->key; //the encounter time of vehicle and its parent vehicle  
		vehicle_list = get_vehicle_list(); //get the pointer to vehicle_list defined in util.c
		vehicle = vehicle_list->next;
		while(vehicle != vehicle_list) //while-1
		{
			if(vehicle->id == current_vehicle->id)
			{
				vehicle = vehicle->next; //move on the next vehicle in vehicle_list
				continue;
			}

//printf("       next %d\n",vehicle->id);
#if 1 /* [ */
			if(current_vehicle->id == 283 && vehicle->id == 1)
			{
				printf("%s:%d current_vehicle(%d) and vehicle(%d) are traced.\n", 
						__FUNCTION__, __LINE__,
						current_vehicle->id, vehicle->id);
			}
#endif /* ] */

#if 0 /* [ */			
			flag = TPD_Do_Vehicles_Encounter(current_time, param, current_vehicle, vehicle, &tail_vertex, &head_vertex, &edge_length, &P_encounter, &T_encounter, &D_encounter, &O_encounter); //check whether src_vehicle and vehicle encounter at the offset O_encounter in an edge (tail_vertex, head_vertex) with P_encounter of at least encounter_probability_threshold at time T_encounter after the travel time (i.e., delay) of D_encounter.
#endif /* ] */
			flag = TPD_Compute_Encounter_Probability(param,
					vehicle, 
					current_vehicle,
					T_threshold,
					&tail_vertex, 
					&head_vertex, 
					&edge_length, 
					&P_encounter, 
					&T_encounter, 
					&D_encounter, 
					&O_encounter); //check whether src_vehicle and vehicle encounter at the offset O_encounter in an edge (tail_vertex, head_vertex) with P_encounter of at least encounter_probability_threshold at time T_encounter after the travel time (i.e., delay) of D_encounter.
			if(flag == FALSE)
			{
				vehicle = vehicle->next; //move on the next vehicle in vehicle_list
				continue;
			}

#if 0 /* [ */
			if(vehicle->id == dst_vehicle->id)
			{
				printf("%s:%d vehicle(%d) for current_vehicle(%d) is dst_vehicle.\n", 
						__FUNCTION__, __LINE__,
						vehicle->id, current_vehicle->id);
			}
#endif /* ] */
			
			/* enqueue vehicle into packet's predicted_encounter_graph's queue Q in the ascending order of the encounter time T_encounter */
            memset(&mpq_qnode, 0, sizeof(minimum_priority_queue_node_t));
            mpq_qnode.id = vehicle->id;
            mpq_qnode.key = T_encounter; //the encounter time of current_vehicle and vehicle 
            mpq_qnode.object_type = QUEUE_NODE_OBJECT_VEHICLE;
            mpq_qnode.object = vehicle;
            pChildQueueNode = (minimum_priority_queue_node_t*)Enqueue_By_KeyAscendingOrder((queue_t*)Q, (queue_node_t*)&mpq_qnode);

			/* let vehicle's mpq_qnode have the pointer to its parent graph node for 
			 * current_vehicle pointed by pGraphNode.
			 * Note: vehicle(i.e., child) will encounter current_vehicle (i.e., parent) 
			 * in its edge (tail_vertex, head_vertex: edge_offset) at time T_encounter
			 * where vehicle is child and current_vehicle is parent. */
			memset(&parent_qnode, 0, sizeof(parent_list_queue_node_t));
			parent_qnode.id = current_vehicle->id;
			parent_qnode.weight = D_encounter; //current_vehicle (as parent) will encounter vehicle after the delay of D_encounter
			parent_qnode.T_encounter = T_encounter; //encounter time
			parent_qnode.P_encounter = P_encounter; //encounter probability
			parent_qnode.tail_vertex = tail_vertex; //tail_vertex for the encounter edge
			parent_qnode.head_vertex = head_vertex; //head_vertex for the encounter edge
			parent_qnode.edge_length = edge_length; //length of the encounter edge
			parent_qnode.edge_offset = O_encounter; //offset for the encounter edge
			parent_qnode.object_type = QUEUE_NODE_OBJECT_VEHICLE;
			parent_qnode.object = current_vehicle;
			parent_qnode.graph_qnode = pGraphNode; //let graph_qnode point to the graph node in adjacency list queue pointed by pGraphNode
			Enqueue((queue_t*)&(pChildQueueNode->parent_list), (queue_node_t*)&parent_qnode);

			/* take the next vehicle */
			vehicle = vehicle->next;
		} //end of while-1

	} while(1); //end of do-while-1

	if(G->dst_vehicle_gnode != NULL)
	{
#if 0 /* [ */
			if(src_vehicle->id == 153)
			{
				printf("%s:%d src_vehicle(%d) is traced.\n", 
						__FUNCTION__, __LINE__,
						src_vehicle->id);
			}
#endif /* ] */

		/* GRAPH PRUNING PHASE: remove the paths from graph nodes that cannot 
		 * reach the destination vehicle node to the src_vehicle. */
		TPD_Prune_Encounter_Graph(G);

		/* EDR COMPUTATION PHASE: compute Expected Delivery Ratio (EDR) with 
		 * the final predicted encounter graph */
		if(param->tpd_encounter_graph_optimization_flag)
		{
			src_vehicle->EDR_for_V2V = TPD_Compute_EDR_For_Encounter_Graph_By_DP(G);
		}
		else
		{
			src_vehicle->EDR_for_V2V = TPD_Compute_EDR_For_Encounter_Graph(G);
		}

		/* EDD COMPUTATION PHASE: compute Expected Delivery Delay (EDD) with 
		 * the final predicted encounter graph */
		src_vehicle->EDD_for_V2V = TPD_Compute_EDD_For_Encounter_Graph(G);

#if TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FLAG /* [ */
		TPD_Perform_BFS_For_Encounter_Graph(G, encounter_graph_display_flag);
		/* perform the Breadth-First-Search (BFS) for the given encounter graph G from source vehicle to destination vehicle in G */
#endif /* ] */
	}

#if 0 /* [ */
	/* reset Q and G in predicted_encounter_graph by emptying the queue nodes. */
	TPD_Reset_Queues_In_Predicted_Encounter_Graph(src_vehicle);
#endif /* ] */

	/* let G->carrier_vehicle_gnode point to G->src_vehicle_gnode for source vehicle. */
	G->carrier_vehicle_gnode = G->src_vehicle_gnode;

	/* set up packet's EDR_for_V2V and EDD_for_V2V */
	packet->EDR_for_V2V = src_vehicle->EDR_for_V2V;
	packet->EDD_for_V2V = src_vehicle->EDD_for_V2V;

	return packet->EDR_for_V2V;
}

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
		double *O_encounter)
{ //check whether vehicle1 and vehicle2 encounter at the offset O_encounter in an edge (tail_vertex, head_vertex) with P_encounter of at least encounter_probability_threshold at time T_encounter after the travel time (i.e., delay) of D_encounter.
	double encounter_probability_theshold = param->tpd_encounter_probability_threshold; //encounter probability threshold
	boolean flag = FALSE;
	int tail_1 = 0, head_1 = 0; //the tail vertex and head vertex of the edge visited by vehicle1
	int tail_2 = 0, head_2 = 0; //the tail vertex and head vertex of the edge visited by vehicle2
	double T_tail_1 = 0, T_head_1 = 0; //the average arrival times for the tail vertex and head vertex of the edge visited by vehicle1
	double T_tail_2 = 0, T_head_2 = 0; //the average arrival times for the tail vertex and head vertex of the edge visited by vehicle2
	struct_path_node *path_ptr1 = NULL; //pointer to the tail vertex of the current edge along the path of vehicle1
	struct_path_node *path_ptr2 = NULL; //pointer to the tail vertex of the current edge along the path of vehicle2

	for(path_ptr1 = vehicle1->path_ptr; path_ptr1 != vehicle1->path_list->prev;)
	{
		tail_1 = atoi(path_ptr1->vertex);
		head_1 = atoi(path_ptr1->next->vertex);
		T_tail_1 = path_ptr1->expected_arrival_time;
		T_head_1 = path_ptr1->next->expected_arrival_time;
		for(path_ptr2 = vehicle2->path_ptr; path_ptr2 != vehicle2->path_list->prev;)
		{
			tail_2 = atoi(path_ptr2->vertex);
			head_2 = atoi(path_ptr2->next->vertex);
			T_tail_2 = path_ptr2->expected_arrival_time;
			T_head_2 = path_ptr2->next->expected_arrival_time;

			/* check whether vehicle1 and vehicle2 encounter in edge (tail_1, head_1) */
			//if((tail_1 == head_2) && (head_1 == tail_2))
			if((tail_1 == head_2) && (head_1 == tail_2) && (T_tail_1 <= T_head_2) && (T_head_1 >= T_tail_2))
			{
				*edge_length = path_ptr1->next->weight;
				*T_encounter = (*edge_length + vehicle1->speed*T_tail_1 + vehicle2->speed*T_tail_2)/(vehicle1->speed + vehicle2->speed);
				*D_encounter = *T_encounter - current_time;
				*O_encounter = (*T_encounter - T_tail_1)*vehicle1->speed;
				*tail_vertex = tail_1;
				*head_vertex = head_1;

				if(*D_encounter >= 0)
				{
#if TPD_ENCOUNTER_TRACE_FLAG /* [ */
					printf("%s:%d vehicle1(%d) and vehicle2(%d) encounter at the offset(%.0f) in the edge(%d, %d) of length(%.0f): vehicle1[%.0f,%.0f] and vehicle2[%.0f,%.0f] with current_time(%.0f), encounter time(%.0f), encounter delay(%.0f)\n", __FUNCTION__, __LINE__, vehicle1->id, vehicle2->id, *O_encounter, tail_1, head_1, *edge_length, T_tail_1, T_head_1, T_tail_2, T_head_2, current_time, *T_encounter, *D_encounter);
#endif /* ] */
					return TRUE;
				}
			}

			path_ptr2 = path_ptr2->next;
		}

		path_ptr1 = path_ptr1->next;
	}

	return flag;
}

int TPD_Prune_Encounter_Graph(adjacency_list_queue_t *G)
{ //prune the encounter graph G by removing graph nodes that are not used as intermediate nodes for the data forwarding from src_vehicle to dst_vehicle
	int i = 0; //index
	parent_list_queue_t *Q = NULL; //pointer to the parent list queue
	int parent_number = 0; //the number of parents of the dst_vehicle_gnode
	parent_list_queue_node_t *p = NULL; //pointer to a parent queue node in the parent_list of dst_vehicle_gnode
	parent_list_queue_node_t *q = NULL; //pointer to a parent queue node in the parent_list of an intermediate vehicle graph node
	adjacency_list_queue_node_t *v = NULL; //pointer to an adjacency list queue node called v
	adjacency_list_queue_node_t *u = NULL; //pointer to an adjacency list queue node called u
	adjacency_list_pointer_queue_t DQ; //the deletion queue of nodes to delete in the encounter graph G
	adjacency_list_pointer_queue_node_t qnode; //a queue node to point to a graph queue node in the adjacency list
	adjacency_list_pointer_queue_node_t *w = NULL; //pointer to an adjacency list pointer queue node
	boolean flag = FALSE; //flag to check whether the edge deletion is successful or not

	/* check the validity of G and G's dst_vehicle_gnode. */
	if(G == NULL)
	{
		printf("%s:%d G is NULL.\n",
				__FUNCTION__, __LINE__);
		return 1;
	}
	else if(G->dst_vehicle_gnode == NULL)
	{
		printf("%s:%d G->dst_vehicle_gnode is NULL.\n",
				__FUNCTION__, __LINE__);
		return 1;
	}
	else if(G->dst_vehicle_gnode->parent_list.size == 0)
	{
		printf("%s:%d G->dst_vehicle_gnode->parent_list.size is 0.\n",
				__FUNCTION__, __LINE__);
		return 1;
	}
	else
	{
		Q = &(G->dst_vehicle_gnode->parent_list);
		parent_number = Q->size;
	}

	/* increase the count of graph nodes as intermediate nodes from the
	 * dst_vehicle_graph toward the root of the graph that is src_vehicle. */
	p = &(Q->head);
	G->dst_vehicle_gnode->count = 0; //reset dst_vehicle_gnode's count to zero
	for(i = 0; i < parent_number; i++)
	{
		p = p->next;
		G->dst_vehicle_gnode->count++;
		v = p->graph_qnode;
		v->count = 0; //reset v's count to zero
		do
		{
			v->count++;

			if(v->parent_list.size > 0)
			{
				q = v->parent_list.head.next;
				v = q->graph_qnode;
			}
			else
			{ /* the search has reached the src_vehicle graph node that has 
				no parent node */
				break;
			}
		} while(1);
	}

	/* delete the graph nodes with count zero because they are not used
	 * as intermediate node for the data forwarding from src_vehicle to
	 * dst_vehicle. */

	/* Step 1: The Creation of a Node List of Nodes with Count Zero.
	 *		- make a graph node list of nodes with its count zero
	 *		to delete by scanning the adjacency list queue in G. */
	InitQueue((queue_t*)&DQ, QTYPE_ADJACENCY_LIST_POINTER); //initialize the queue DQ

	v = &(G->head);
	for(i = 0; i < G->size; i++)
	{
		v = v->next;
		if(v->count == 0)
		{
			memset(&qnode, 0, sizeof(qnode));
			qnode.graph_qnode = v;
			Enqueue((queue_t*)&DQ, (queue_node_t*)&qnode);
		}
	}

	/* Step 2: The Deletion of Edges.
	 *		- delete each edge whose head node is the deleted node for
	 *		its parent graph node. */
	w = &(DQ.head);
	for(i = 0; i < DQ.size; i++)
	{
		w = w->next;
		v = w->graph_qnode;

		/* check the validity of parenet_list */
		if(v->parent_list.size == 0)
		{
			printf("%s:%d parent_list.size(vid=%d) is 0.\n",
					__FUNCTION__, __LINE__,
					v->id);
			exit(1);
		}

		p = v->parent_list.head.next;
		u = p->graph_qnode;

		/* delete the edge (u, v) from u's neighbor_list */
		flag = Delete_Edge_In_NeighborList(u, v);
		if(flag == FALSE)
		{
			printf("%s:%d the edge (%d, %d) cannot be deleted!\n",
					__FUNCTION__, __LINE__,
					u->id, v->id);
			exit(1);
		}
	}

	/* Step 3: The Deletion of Nodes.
	 *		- delete graph nodes in the adjacency list G with the node list 
	 *		of deleted nodes. */
	w = &(DQ.head);
	for(i = 0; i < DQ.size; i++)
	{
		w = w->next;

		/* delete the graph node pointed by w from the encounter graph G */
		flag = Delete_Node_In_AdjacencyList(w->graph_qnode);
		if(flag == FALSE)
		{
			printf("%s:%d the graph node (%d) cannot be deleted!\n",
					__FUNCTION__, __LINE__,
					w->graph_qnode->id);
			exit(1);
		}
		else
		{
			/* reset bitmap variables for the deleted graph node */
			G->bitmap[w->graph_qnode->id-1] = FALSE;
			G->bitmap_gnodes[w->graph_qnode->id-1] = NULL;
		}
	}

	DestroyQueue((queue_t*)&DQ); //destroy the queue DQ
	return 0;
}

boolean TPD_Compute_FWD_Probability(
		parameter_t *param,	
		struct_vehicle_t *vehicle1, 
		struct_vehicle_t *vehicle2, 
		double T_threshold,
		int *tail_vertex, 
		int *head_vertex,
		double *edge_length,
		double *P_encounter, 
		double *T_encounter,
		double *D_encounter,
		double *O_encounter)
{ /* compute the encounter probability (called P_encounter) of vehicle1 and 
	 vehicle2 that are expected to encounter in the edge (tail_vertex, head_vertex)
	 of length edge_length and return the boolean flag to indicate whether these 
	 two vehicles will encounter with some probability.
	 Note: T_threshold is the encounter time of vehicle and its parent vehicle 
	 that is used to determine the valid encounter of two vehicles during the 
	 selection of child vehicles in the predicted encounter graph. */
	int tail_1 = 0, head_1 = 0; //the tail vertex and head vertex of the edge visited by vehicle1
	int tail_2 = 0, head_2 = 0; //the tail vertex and head vertex of the edge visited by vehicle2
	double T_tail_1 = 0, T_head_1 = 0; //the average arrival times for the tail vertex and head vertex of the edge visited by vehicle1
	double T_tail_2 = 0, T_head_2 = 0; //the average arrival times for the tail vertex and head vertex of the edge visited by vehicle2
	struct_path_node *path_ptr1 = NULL; //pointer to the tail vertex of the current edge along the path of vehicle1
	struct_path_node *path_ptr2 = NULL; //pointer to the tail vertex of the current edge along the path of vehicle2
	double T_tail_1_std = 0, T_head_1_std = 0; //the standard deviations of the arrival times for the tail vertex and head vertex of the edge visited by vehicle1
	double T_tail_2_std = 0, T_head_2_std = 0; //the standard deviations of the arrival times for the tail vertex and head vertex of the edge visited by vehicle2
	boolean flag1 = TRUE, flag2 = TRUE; //flags to indicate the current edge
	double length1 = 0, length2 = 0; //edge lengths

#if 0 /* [ */
	double unit_length_travel_time = param->vehicle_unit_length_mean_travel_time; //travel time for the unit length
	double unit_length_travel_time_variance = param->vehicle_unit_length_travel_time_variance; //travel time variance for the unit length
#else
	double unit_length_travel_time_1 = vehicle1->unit_length_mean_travel_time; //vehicle1's travel time for the unit length
	double unit_length_travel_time_variance_1 = vehicle1->unit_length_travel_time_variance; //vehicle1's travel time variance for the unit length
	double unit_length_travel_time_2 = vehicle2->unit_length_mean_travel_time; //vehicle2's travel time for the unit length
	double unit_length_travel_time_variance_2 = vehicle2->unit_length_travel_time_variance; //vehicle2's travel time variance for the unit length
#endif /* ] */

	double D_tail_1 = 0, D_tail_2 = 0; //the average travel times for vehicle1 and vehicle2 from their current position to their tail vertex along their trajectory
	double D_head_1 = 0, D_head_2 = 0; //the average travel times for vehicle1 and vehicle2 from their current position to their head vertex along their trajectory 
	double D_tail_var_1 = 0, D_tail_var_2 = 0; //the travel time variances for vehicle1 and vehicle2 from their current position to their tail vertex along their trajectory
	double D_head_var_1 = 0, D_head_var_2 = 0; //the travel time variances for vehicle1 and vehicle2 from their current position to their head vertex along their trajectory
	double D_tail_std_1 = 0, D_tail_std_2 = 0; //the travel time standard deviations for vehicle1 and vehicle2 from their current position to their tail vertex along their trajectory
	double D_head_std_1 = 0, D_head_std_2 = 0; //the travel time standard deviations for vehicle1 and vehicle2 from their current position to their head vertex along their trajectory
	double t_1 = 0; //the travel time in the encountered edge of vehicle1
	double t_2 = 0; //the travel time in the encountered edge of vehicle2
	int vehicle1_path_current_edge_tail = atoi(vehicle1->path_ptr->vertex); //the tail id of the current edge of vehicle1
	int vehicle2_path_current_edge_tail = atoi(vehicle2->path_ptr->vertex); //the tail id of the current edge of vehicle2

	for(path_ptr1 = vehicle1->path_ptr; path_ptr1 != vehicle1->path_list->prev;)
	{
		tail_1 = atoi(path_ptr1->vertex);
		head_1 = atoi(path_ptr1->next->vertex);
		T_tail_1 = path_ptr1->expected_arrival_time;
		T_head_1 = path_ptr1->next->expected_arrival_time;

		/* check whether encounter time is later than T_threhold */
		if(T_tail_1 < T_threshold)
		{
			path_ptr1 = path_ptr1->next;
			continue;
		}

		//if(flag1 == TRUE)
		if(vehicle1_path_current_edge_tail == tail_1)
		{
			length1 = path_ptr1->next->weight - vehicle1->path_current_edge_offset;
			flag1 = FALSE;
			D_tail_1 = 0;
			D_tail_var_1 = 0;
			D_head_1 = length1*unit_length_travel_time_1;
			D_head_var_1 = length1*length1*unit_length_travel_time_variance_1;
		}
		else
		{
			length1 = path_ptr1->next->weight;
			D_tail_1 = D_head_1;
			D_tail_var_1 = D_head_var_1;
			D_head_1 += length1*unit_length_travel_time_1;
			D_head_var_1 += length1*length1*unit_length_travel_time_variance_1;
		}

		D_tail_std_1 = sqrt(D_tail_var_1);
		D_head_std_1 = sqrt(D_head_var_1);

		t_1 = length1*unit_length_travel_time_1;
		flag2 = TRUE;
		for(path_ptr2 = vehicle2->path_ptr; path_ptr2 != vehicle2->path_list->prev;)
		{
			tail_2 = atoi(path_ptr2->vertex);
			head_2 = atoi(path_ptr2->next->vertex);
			T_tail_2 = path_ptr2->expected_arrival_time;
			T_head_2 = path_ptr2->next->expected_arrival_time;

			/* check whether encounter time is later than T_threhold */
			if(T_tail_2 < T_threshold)
			{
				path_ptr2 = path_ptr2->next;
				continue;
			}

			//if(flag2 == TRUE)
			if(vehicle2_path_current_edge_tail == tail_2)
			{
				length2 = path_ptr2->next->weight - vehicle2->path_current_edge_offset;
				flag2 = FALSE;
				D_tail_2 = 0;
				D_tail_var_2 = 0;
				D_head_2 = length2*unit_length_travel_time_2;
				D_head_var_2 = length2*length2*unit_length_travel_time_variance_2;
			}
			else
			{
				length2 = path_ptr2->next->weight;
				D_tail_2 = D_head_2;
				D_tail_var_2 = D_head_var_2;
				D_head_2 += length2*unit_length_travel_time_2;
				D_head_var_2 += length2*length2*unit_length_travel_time_variance_2;
			}

			D_tail_std_2 = sqrt(D_tail_var_2);
			D_head_std_2 = sqrt(D_head_var_2);

			//t_2 = length1*unit_length_travel_time; //Error: length1 must be replaced with length2.
			t_2 = length2*unit_length_travel_time_2;
			
			/* check whether vehicle1 and vehicle2 encounter in edge (tail_1, head_1) */
			//if((tail_1 == head_2) && (head_1 == tail_2) && (T_tail_1 <= T_head_2) && (T_head_1 >= T_tail_2))
			/* taehwan 20140726 */
			int margin_time = g_margin_time;
			// Apply Margin for preventing missing case
		    //if((tail_1 == head_2) && (head_1 == tail_2) && (T_tail_1+ margin_time <= T_head_2) && (T_head_1 >= T_tail_2+ margin_time))
			if((tail_1 == head_2) && (head_1 == tail_2) && ((T_tail_1+ margin_time <= T_head_2) && (T_head_1 >= T_tail_2+ margin_time)))
				{
					*edge_length = path_ptr1->next->weight;
					*T_encounter = (*edge_length + vehicle1->speed*T_tail_1 + vehicle2->speed*T_tail_2)/(vehicle1->speed + vehicle2->speed);
					//*D_encounter = *T_encounter - current_time;
					*D_encounter = *T_encounter - T_threshold;
					*O_encounter = (*T_encounter - T_tail_1)*vehicle1->speed;
					*tail_vertex = tail_1;
					*head_vertex = head_1;
	
					if((*D_encounter >= 0) && (*T_encounter >= T_threshold))
					{
#if TPD_ENCOUNTER_TRACE_FLAG /* [ */
						printf("%s:%d vehicle1(%d) and vehicle2(%d) encounter at the offset(%.0f) in the edge(%d, %d) of length(%.0f): vehicle1[%.0f,%.0f] and vehicle2[%.0f,%.0f] with T_threshold(%.0f), encounter time(%.0f), encounter delay(%.0f)\n", __FUNCTION__, __LINE__, vehicle1->id, vehicle2->id, *O_encounter, tail_1, head_1, *edge_length, T_tail_1, T_head_1, T_tail_2, T_head_2, T_threshold, *T_encounter, *D_encounter);
#endif /* ] */
						/* perform the double integral for encounter probability. */
						*P_encounter = GSL_TPD_Encounter_Probability_For_Gamma_Distribution(
								D_head_2,
								D_head_std_2,
								D_tail_1,
								D_tail_std_1,
								0,
								param->communication_packet_ttl,
								t_1,
								t_2);
						if(*P_encounter >= param->tpd_encounter_probability_threshold)
						{
							return TRUE;
						}
					}
			}

			path_ptr2 = path_ptr2->next;
		}

		path_ptr1 = path_ptr1->next;
	}

	return FALSE;
}

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
		double *O_encounter)
{ /* compute the encounter probability (called P_encounter) of vehicle1 and 
	 vehicle2 that are expected to encounter in the edge (tail_vertex, head_vertex)
	 of length edge_length and return the boolean flag to indicate whether these 
	 two vehicles will encounter with some probability.
	 Note: T_threshold is the encounter time of vehicle and its parent vehicle 
	 that is used to determine the valid encounter of two vehicles during the 
	 selection of child vehicles in the predicted encounter graph. */	
	int tail_1 = 0, head_1 = 0; //the tail vertex and head vertex of the edge visited by vehicle1	
	int tail_2 = 0, head_2 = 0; //the tail vertex and head vertex of the edge visited by vehicle2	
	double T_tail_1 = 0, T_head_1 = 0; //the average arrival times for the tail vertex and head vertex of the edge visited by vehicle1	
	double T_tail_2 = 0, T_head_2 = 0; //the average arrival times for the tail vertex and head vertex of the edge visited by vehicle2	
	struct_path_node *path_ptr1 = NULL; //pointer to the tail vertex of the current edge along the path of vehicle1
	struct_path_node *path_ptr2 = NULL; //pointer to the tail vertex of the current edge along the path of vehicle2
	double T_tail_1_std = 0, T_head_1_std = 0; //the standard deviations of the arrival times for the tail vertex and head vertex of the edge visited by vehicle1	
	double T_tail_2_std = 0, T_head_2_std = 0; //the standard deviations of the arrival times for the tail vertex and head vertex of the edge visited by vehicle2
	boolean flag1 = TRUE, flag2 = TRUE; //flags to indicate the current edge
	double length1 = 0, length2 = 0; //edge lengths

#if 0 /* [ */
	double unit_length_travel_time = param->vehicle_unit_length_mean_travel_time; //travel time for the unit length
	double unit_length_travel_time_variance = param->vehicle_unit_length_travel_time_variance; //travel time variance for the unit length
#else
	double unit_length_travel_time_1 = vehicle1->unit_length_mean_travel_time; //vehicle1's travel time for the unit length
	double unit_length_travel_time_variance_1 = vehicle1->unit_length_travel_time_variance; //vehicle1's travel time variance for the unit length
	double unit_length_travel_time_2 = vehicle2->unit_length_mean_travel_time; //vehicle2's travel time for the unit length
	double unit_length_travel_time_variance_2 = vehicle2->unit_length_travel_time_variance; //vehicle2's travel time variance for the unit length
#endif /* ] */


	double D_tail_1 = 0, D_tail_2 = 0; //the average travel times for vehicle1 and vehicle2 from their current position to their tail vertex along their trajectory
	double D_head_1 = 0, D_head_2 = 0; //the average travel times for vehicle1 and vehicle2 from their current position to their head vertex along their trajectory 
	double D_tail_var_1 = 0, D_tail_var_2 = 0; //the travel time variances for vehicle1 and vehicle2 from their current position to their tail vertex along their trajectory
	double D_head_var_1 = 0, D_head_var_2 = 0; //the travel time variances for vehicle1 and vehicle2 from their current position to their head vertex along their trajectory
	double D_tail_std_1 = 0, D_tail_std_2 = 0; //the travel time standard deviations for vehicle1 and vehicle2 from their current position to their tail vertex along their trajectory
	double D_head_std_1 = 0, D_head_std_2 = 0; //the travel time standard deviations for vehicle1 and vehicle2 from their current position to their head vertex along their trajectory

	double t_1 = 0; //the travel time in the encountered edge of vehicle1
	double t_2 = 0; //the travel time in the encountered edge of vehicle2
	int vehicle1_path_current_edge_tail = atoi(vehicle1->path_ptr->vertex); //the tail id of the current edge of vehicle1
	int vehicle2_path_current_edge_tail = atoi(vehicle2->path_ptr->vertex); //the tail id of the current edge of vehicle2

	for(path_ptr1 = vehicle1->path_ptr; path_ptr1 != vehicle1->path_list->prev;)
	{
		tail_1 = atoi(path_ptr1->vertex);
		head_1 = atoi(path_ptr1->next->vertex);
		T_tail_1 = path_ptr1->expected_arrival_time;						
		T_head_1 = path_ptr1->next->expected_arrival_time;					
																			
		/* check whether encounter time is later than T_threhold */
		if(T_tail_1 < T_threshold)				
		{
			path_ptr1 = path_ptr1->next;		
			continue;
		}

		//if(flag1 == TRUE)
		if(vehicle1_path_current_edge_tail == tail_1)		
		{
			length1 = path_ptr1->next->weight - vehicle1->path_current_edge_offset;		
			flag1 = FALSE;
			D_tail_1 = 0;										
			D_tail_var_1 = 0;
			D_head_1 = length1*unit_length_travel_time_1;			
			D_head_var_1 = length1*length1*unit_length_travel_time_variance_1;	
		}
		else
		{
			length1 = path_ptr1->next->weight;						
			D_tail_1 = D_head_1;								
			D_tail_var_1 = D_head_var_1;							
			D_head_1 += length1*unit_length_travel_time_1;		
			D_head_var_1 += length1*length1*unit_length_travel_time_variance_1;	
		}

		D_tail_std_1 = sqrt(D_tail_var_1);		
		D_head_std_1 = sqrt(D_head_var_1);

		t_1 = length1*unit_length_travel_time_1;	
		flag2 = TRUE;
		for(path_ptr2 = vehicle2->path_ptr; path_ptr2 != vehicle2->path_list->prev;)
		{
			tail_2 = atoi(path_ptr2->vertex);
			head_2 = atoi(path_ptr2->next->vertex);										
			T_tail_2 = path_ptr2->expected_arrival_time;
			T_head_2 = path_ptr2->next->expected_arrival_time;

			/* check whether encounter time is later than T_threhold */
			if(T_tail_2 < T_threshold) 
			{
				path_ptr2 = path_ptr2->next;
				continue;
			}

			//if(flag2 == TRUE)
			if(vehicle2_path_current_edge_tail == tail_2)
			{
				length2 = path_ptr2->next->weight - vehicle2->path_current_edge_offset;	
				flag2 = FALSE;
				D_tail_2 = 0;
				D_tail_var_2 = 0;
				D_head_2 = length2*unit_length_travel_time_2;
				D_head_var_2 = length2*length2*unit_length_travel_time_variance_2;
			}
			else
			{
				length2 = path_ptr2->next->weight;			
				D_tail_2 = D_head_2;
				D_tail_var_2 = D_head_var_2;
				D_head_2 += length2*unit_length_travel_time_2;
				D_head_var_2 += length2*length2*unit_length_travel_time_variance_2;
			}

			D_tail_std_2 = sqrt(D_tail_var_2);
			D_head_std_2 = sqrt(D_head_var_2);		

			//t_2 = length1*unit_length_travel_time; //Error: length1 must be replaced with length2.
			t_2 = length2*unit_length_travel_time_2;
			
			/* check whether vehicle1 and vehicle2 encounter in edge (tail_1, head_1) */
			//if((tail_1 == head_2) && (head_1 == tail_2) && (T_tail_1 <= T_head_2) && (T_head_1 >= T_tail_2))
			/* taehwan 20140726 */
			int margin_time = g_margin_time;	
			// Apply Margin for preventing missing case
		    //if((tail_1 == head_2) && (head_1 == tail_2) && (T_tail_1+ margin_time <= T_head_2) && (T_head_1 >= T_tail_2+ margin_time))
		//	if((tail_1 == head_2) && (head_1 == tail_2) && ((T_tail_1+ margin_time <= T_head_2) && (T_head_1 >= T_tail_2+ margin_time)))
			if ((tail_1 == head_2) && (head_1 == tail_2) && ((D_tail_1 <= D_head_2) && (D_head_1 >= D_tail_2)))	
			{									
				*edge_length = path_ptr1->next->weight;	
				*T_encounter = (*edge_length + vehicle1->speed*T_tail_1 + vehicle2->speed*T_tail_2)/(vehicle1->speed + vehicle2->speed);
				//*D_encounter = *T_encounter - current_time;
				*D_encounter = *T_encounter - T_threshold;	
				*O_encounter = (*T_encounter - T_tail_1)*vehicle1->speed;	
				*tail_vertex = tail_1;			
				*head_vertex = head_1;			
	
				if((*D_encounter >= 0) && (*T_encounter >= T_threshold))
				{
#if TPD_ENCOUNTER_TRACE_FLAG /* [ */
					printf("%s:%d vehicle1(%d) and vehicle2(%d) encounter at the offset(%.0f) in the edge(%d, %d) of length(%.0f): vehicle1[%.0f,%.0f] and vehicle2[%.0f,%.0f] with T_threshold(%.0f), encounter time(%.0f), encounter delay(%.0f)\n", __FUNCTION__, __LINE__, vehicle1->id, vehicle2->id, *O_encounter, tail_1, head_1, *edge_length, T_tail_1, T_head_1, T_tail_2, T_head_2, T_threshold, *T_encounter, *D_encounter);
#endif /* ] */
					/* perform the double integral for encounter probability. */
					//D_tail_std_1 = D_tail_std_1 / 20;
					//D_head_std_2 = D_head_std_2 / 20;
					//if (D_tail_std_1 < 2.5)
					//	D_tail_std_1 = 2.5;
					//if (D_head_std_2 < 2.5)
					//	D_head_std_2 = 2.5;

				//	*P_encounter = GSL_TPD_Encounter_Probability_For_Gamma_Distribution(
				//			D_head_2, 
				//			D_head_std_2,
				//			D_tail_1,/*D_head_2,*/
				//			D_tail_std_1,
				//			0,
				//			param->communication_packet_ttl,
				//			t_1,						
				//			t_2);

					*P_encounter = GSL_TPD_Encounter_Probability_For_Gamma_Distribution(
						D_head_2,
						D_head_std_2,
						D_head_1,/*D_head_2,*/										
						D_head_std_1,
						0,
						param->communication_packet_ttl,
						t_1,						
						t_2);

					//if (vehicle1->id==1) 
					#ifdef TPD_FORWARD_LOG
						printf("         taehwan01022157 TPD Encounter Probability %d->%d (s2=%.2f s1=%.2f) P=%.4f %s\n",vehicle2->id,vehicle1->id,D_head_std_2,D_tail_std_1,*P_encounter,(vehicle1->id==1?"*":""));
						printf("  [%d]: D_tail_2 = %0.f D_head_2 = %0.f T_tail_2 = %0.f T_head_2 = %0.f  [%d]: D_tail_1 = %0.f D_head_1 = %0.f T_tail_1 = %0.f T_head_1 = %0.f EncounterTime = %0.f\n", vehicle2->id, D_tail_2, D_head_2, T_tail_2, T_head_2, vehicle1->id, D_tail_1, D_head_1, T_tail_1, T_head_1, *T_encounter);
					#endif
					
					//	printf(" head2-tail2 = %.2f       t_2 = %.2f\n", head_2 - tail_2, t_2);
					//	TPD_Print_Vehicle_Trajectory(vehicle2);
					//	TPD_Print_Vehicle_Trajectory(vehicle1);
					//	TPD_Print_Vehicle_Encounter_Area_Compared(vehicle1, vehicle2);
						//*P_encounter = 1;		

			//printf("         taehwan12012020 TPD Probability %d->%d time=%.2f P=%.2f\n",vehicle2->id,vehicle1->id,*T_encounter,*P_encounter);
				        //printf("         taehwan12012105 TPD Probability ttl=%.2f\n",param->communication_packet_ttl);
					// taehwan 20141226 pass
					//return TRUE;
					if(*P_encounter >= param->tpd_encounter_probability_threshold)
					{
						return TRUE;
					}
				}
			}
			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			if ((tail_1 == tail_2) && (head_1 == head_2) && ((T_tail_1 + margin_time <= T_head_2) && (T_head_1 >= T_tail_2 + margin_time)))
		//	if ((tail_1 == head_2) && (head_1 == tail_2) && ((D_tail_1 <= D_head_2) && (D_head_1 >= D_tail_2)))		
			{								
				*edge_length = path_ptr1->next->weight;	
				*T_encounter = (*edge_length + vehicle1->speed*T_tail_1 + vehicle2->speed*T_tail_2) / (vehicle1->speed + vehicle2->speed);	
				//*D_encounter = *T_encounter - current_time;
				*D_encounter = *T_encounter - T_threshold;		
				*O_encounter = (*T_encounter - T_tail_1)*vehicle1->speed;
				*tail_vertex = tail_1;			
				*head_vertex = head_1;			

				if ((*D_encounter >= 0) && (*T_encounter >= T_threshold))
				{
#if TPD_ENCOUNTER_TRACE_FLAG /* [ */
					printf("%s:%d vehicle1(%d) and vehicle2(%d) encounter at the offset(%.0f) in the edge(%d, %d) of length(%.0f): vehicle1[%.0f,%.0f] and vehicle2[%.0f,%.0f] with T_threshold(%.0f), encounter time(%.0f), encounter delay(%.0f)\n", __FUNCTION__, __LINE__, vehicle1->id, vehicle2->id, *O_encounter, tail_1, head_1, *edge_length, T_tail_1, T_head_1, T_tail_2, T_head_2, T_threshold, *T_encounter, *D_encounter);
#endif /* ] */
					/* perform the double integral for encounter probability. */
					//D_tail_std_1 = D_tail_std_1 / 20;
					//D_head_std_2 = D_head_std_2 / 20;
					//if (D_tail_std_1 < 2.5)
					//	D_tail_std_1 = 2.5;
					//if (D_head_std_2 < 2.5)
					//	D_head_std_2 = 2.5;
					*P_encounter = GSL_TPD_Encounter_Probability_For_Gamma_Distribution(
						D_head_2,
						D_head_std_2,
						D_head_1,/*D_head_2,*/
						D_head_std_1,
						0,
						param->communication_packet_ttl,
						t_1,
						t_2);
					//if (vehicle1->id==1) 

				#ifdef TPD_FORWARD_LOG
					printf("        jinyong TPD Forwarding Probability %d->%d (s2=%.2f s1=%.2f) P=%.4f %s\n", vehicle2->id, vehicle1->id, D_head_std_2, D_head_std_1, *P_encounter, (vehicle1->id == 1 ? "*" : ""));
					printf("  [%d]: D_tail_2 = %0.f D_head_2 = %0.f T_tail_2 = %0.f T_head_2 = %0.f  [%d]: D_tail_1 = %0.f D_head_1 = %0.f T_tail_1 = %0.f T_head_1 = %0.f\n", vehicle2->id, D_tail_2, D_head_2, T_tail_2, T_head_2, vehicle1->id, D_tail_1, D_head_1, T_tail_1, T_head_1);
				#endif

				//	TPD_Print_Vehicle_Trajectory(vehicle2);
				//	TPD_Print_Vehicle_Trajectory(vehicle1);
				//	TPD_Print_Vehicle_Forward_Area_Compared(vehicle1, vehicle2);
					//*P_encounter = 1;		

					//printf("         taehwan12012020 TPD Probability %d->%d time=%.2f P=%.2f\n",vehicle2->id,vehicle1->id,*T_encounter,*P_encounter);
					//printf("         taehwan12012105 TPD Probability ttl=%.2f\n",param->communication_packet_ttl);
					// taehwan 20141226 pass
					//return TRUE;
					if (*P_encounter >= param->tpd_encounter_probability_threshold)
					{
						return TRUE;
					}
				}
			}




			path_ptr2 = path_ptr2->next;
		}

		path_ptr1 = path_ptr1->next;
	}

	return FALSE;
}

boolean TPD_Compute_Forward_Probability(parameter_t *param,
	struct_vehicle_t *vehicle1,
	struct_vehicle_t *vehicle2,
	double T_threshold,
	int *tail_vertex,
	int *head_vertex,
	double *edge_length,
	double *P_encounter,
	double *T_encounter,
	double *D_encounter,
	double *O_encounter)
{ /* compute the encounter probability (called P_encounter) of vehicle1 and
  vehicle2 that are expected to encounter in the edge (tail_vertex, head_vertex)
  of length edge_length and return the boolean flag to indicate whether these
  two vehicles will encounter with some probability.
  Note: T_threshold is the encounter time of vehicle and its parent vehicle
  that is used to determine the valid encounter of two vehicles during the
  selection of child vehicles in the predicted encounter graph. */
	int tail_1 = 0, head_1 = 0; //the tail vertex and head vertex of the edge visited by vehicle1	
	int tail_2 = 0, head_2 = 0; //the tail vertex and head vertex of the edge visited by vehicle2	
	double T_tail_1 = 0, T_head_1 = 0; //the average arrival times for the tail vertex and head vertex of the edge visited by vehicle1	
	double T_tail_2 = 0, T_head_2 = 0; //the average arrival times for the tail vertex and head vertex of the edge visited by vehicle2
	struct_path_node *path_ptr1 = NULL; //pointer to the tail vertex of the current edge along the path of vehicle1
	struct_path_node *path_ptr2 = NULL; //pointer to the tail vertex of the current edge along the path of vehicle2
	double T_tail_1_std = 0, T_head_1_std = 0; //the standard deviations of the arrival times for the tail vertex and head vertex of the edge visited by vehicle1	€
	double T_tail_2_std = 0, T_head_2_std = 0; //the standard deviations of the arrival times for the tail vertex and head vertex of the edge visited by vehicle2
	boolean flag1 = TRUE, flag2 = TRUE; //flags to indicate the current edge
	double length1 = 0, length2 = 0; //edge lengths

#if 0 /* [ */
	double unit_length_travel_time = param->vehicle_unit_length_mean_travel_time; //travel time for the unit length
	double unit_length_travel_time_variance = param->vehicle_unit_length_travel_time_variance; //travel time variance for the unit length
#else
	double unit_length_travel_time_1 = vehicle1->unit_length_mean_travel_time; //vehicle1's travel time for the unit length
	double unit_length_travel_time_variance_1 = vehicle1->unit_length_travel_time_variance; //vehicle1's travel time variance for the unit length
	double unit_length_travel_time_2 = vehicle2->unit_length_mean_travel_time; //vehicle2's travel time for the unit length
	double unit_length_travel_time_variance_2 = vehicle2->unit_length_travel_time_variance; //vehicle2's travel time variance for the unit length
#endif /* ] */


	double D_tail_1 = 0, D_tail_2 = 0; //the average travel times for vehicle1 and vehicle2 from their current position to their tail vertex along their trajectory
	double D_head_1 = 0, D_head_2 = 0; //the average travel times for vehicle1 and vehicle2 from their current position to their head vertex along their trajectory 
	double D_tail_var_1 = 0, D_tail_var_2 = 0; //the travel time variances for vehicle1 and vehicle2 from their current position to their tail vertex along their trajectory
	double D_head_var_1 = 0, D_head_var_2 = 0; //the travel time variances for vehicle1 and vehicle2 from their current position to their head vertex along their trajectory
	double D_tail_std_1 = 0, D_tail_std_2 = 0; //the travel time standard deviations for vehicle1 and vehicle2 from their current position to their tail vertex along their trajectory
	double D_head_std_1 = 0, D_head_std_2 = 0; //the travel time standard deviations for vehicle1 and vehicle2 from their current position to their head vertex along their trajectory
	double t_1 = 0; //the travel time in the encountered edge of vehicle1
	double t_2 = 0; //the travel time in the encountered edge of vehicle2
	int vehicle1_path_current_edge_tail = atoi(vehicle1->path_ptr->vertex); //the tail id of the current edge of vehicle1
	int vehicle2_path_current_edge_tail = atoi(vehicle2->path_ptr->vertex); //the tail id of the current edge of vehicle2

	for (path_ptr1 = vehicle1->path_ptr; path_ptr1 != vehicle1->path_list->prev;)
	{
		tail_1 = atoi(path_ptr1->vertex);
		head_1 = atoi(path_ptr1->next->vertex);
		T_tail_1 = path_ptr1->expected_arrival_time;					
		T_head_1 = path_ptr1->next->expected_arrival_time;					
	
		/* check whether encounter time is later than T_threhold */
		if (T_tail_1 < T_threshold)				
		{
			path_ptr1 = path_ptr1->next;
			continue;
		}

		//if(flag1 == TRUE)
		if (vehicle1_path_current_edge_tail == tail_1)			
		{
			length1 = path_ptr1->next->weight - vehicle1->path_current_edge_offset;	
			flag1 = FALSE;
			D_tail_1 = 0;										
			D_tail_var_1 = 0;
			D_head_1 = length1*unit_length_travel_time_1;		
			D_head_var_1 = length1*length1*unit_length_travel_time_variance_1;	
        }
		else
		{
			length1 = path_ptr1->next->weight;					
			D_tail_1 = D_head_1;								
			D_tail_var_1 = D_head_var_1;						
			D_head_1 += length1*unit_length_travel_time_1;		
			D_head_var_1 += length1*length1*unit_length_travel_time_variance_1;	
		}

		D_tail_std_1 = sqrt(D_tail_var_1);		//?œì??¸ì°¨
		D_head_std_1 = sqrt(D_head_var_1);

		t_1 = length1*unit_length_travel_time_1;		
		flag2 = TRUE;
		for (path_ptr2 = vehicle2->path_ptr; path_ptr2 != vehicle2->path_list->prev;)
		{
			tail_2 = atoi(path_ptr2->vertex);
			head_2 = atoi(path_ptr2->next->vertex);										
			T_tail_2 = path_ptr2->expected_arrival_time;
			T_head_2 = path_ptr2->next->expected_arrival_time;

			/* check whether encounter time is later than T_threhold */
			if (T_tail_2 < T_threshold)												
			{
				path_ptr2 = path_ptr2->next;
				continue;
			}

			//if(flag2 == TRUE)
			if (vehicle2_path_current_edge_tail == tail_2)
			{
				length2 = path_ptr2->next->weight - vehicle2->path_current_edge_offset;	
				flag2 = FALSE;
				D_tail_2 = 0;
				D_tail_var_2 = 0;
				D_head_2 = length2*unit_length_travel_time_2;
				D_head_var_2 = length2*length2*unit_length_travel_time_variance_2;
			}
			else
			{
				length2 = path_ptr2->next->weight;				
				D_tail_2 = D_head_2;
				D_tail_var_2 = D_head_var_2;
				D_head_2 += length2*unit_length_travel_time_2;
				D_head_var_2 += length2*length2*unit_length_travel_time_variance_2;
			}

			D_tail_std_2 = sqrt(D_tail_var_2);
			D_head_std_2 = sqrt(D_head_var_2);		

			//t_2 = length1*unit_length_travel_time; //Error: length1 must be replaced with length2.
			t_2 = length2*unit_length_travel_time_2;

			/* check whether vehicle1 and vehicle2 encounter in edge (tail_1, head_1) */
			//if((tail_1 == head_2) && (head_1 == tail_2) && (T_tail_1 <= T_head_2) && (T_head_1 >= T_tail_2))
			/* taehwan 20140726 */
			int margin_time = g_margin_time;		
			// Apply Margin for preventing missing case
			//if((tail_1 == head_2) && (head_1 == tail_2) && (T_tail_1+ margin_time <= T_head_2) && (T_head_1 >= T_tail_2+ margin_time))
			if ((tail_1 == tail_2) && (head_1 == head_2) && ((T_tail_1 + margin_time <= T_head_2) && (T_head_1 >= T_tail_2 + margin_time)))
			{								
				*edge_length = path_ptr1->next->weight;	
				*T_encounter = (*edge_length + vehicle1->speed*T_tail_1 + vehicle2->speed*T_tail_2) / (vehicle1->speed + vehicle2->speed);	
				//*D_encounter = *T_encounter - current_time;
				*D_encounter = *T_encounter - T_threshold;	
				*O_encounter = (*T_encounter - T_tail_1)*vehicle1->speed;
				*tail_vertex = tail_1;		
				*head_vertex = head_1;		

				if ((*D_encounter >= 0) && (*T_encounter >= T_threshold))
				{
#if TPD_ENCOUNTER_TRACE_FLAG /* [ */
					printf("%s:%d vehicle1(%d) and vehicle2(%d) encounter at the offset(%.0f) in the edge(%d, %d) of length(%.0f): vehicle1[%.0f,%.0f] and vehicle2[%.0f,%.0f] with T_threshold(%.0f), encounter time(%.0f), encounter delay(%.0f)\n", __FUNCTION__, __LINE__, vehicle1->id, vehicle2->id, *O_encounter, tail_1, head_1, *edge_length, T_tail_1, T_head_1, T_tail_2, T_head_2, T_threshold, *T_encounter, *D_encounter);
#endif /* ] */
					/* perform the double integral for encounter probability. */
					//D_tail_std_1 = D_tail_std_1 / 20;
					//D_head_std_2 = D_head_std_2 / 20;
					//if (D_tail_std_1 < 2.5)
					//	D_tail_std_1 = 2.5;
					//if (D_head_std_2 < 2.5)
					//	D_head_std_2 = 2.5;
					*P_encounter = GSL_TPD_Encounter_Probability_For_Gamma_Distribution(
						D_head_2,
						D_head_std_2,
						D_tail_1,/*D_head_2,*/
						D_tail_std_1,
						0,
						param->communication_packet_ttl,
						t_1,
						t_2);
					//if (vehicle1->id==1) 
					printf("         taehwan01022157 TPD Probability %d->%d (s2=%.2f s1=%.2f) P=%.4f %s\n", vehicle2->id, vehicle1->id, D_head_std_2, D_tail_std_1, *P_encounter, (vehicle1->id == 1 ? "*" : ""));
					//*P_encounter = 1;		

					//printf("         taehwan12012020 TPD Probability %d->%d time=%.2f P=%.2f\n",vehicle2->id,vehicle1->id,*T_encounter,*P_encounter);
					//printf("         taehwan12012105 TPD Probability ttl=%.2f\n",param->communication_packet_ttl);
					// taehwan 20141226 pass
					//return TRUE;
					if (*P_encounter >= param->tpd_encounter_probability_threshold)
					{
						return TRUE;
					}
				}
			}

			path_ptr2 = path_ptr2->next;
		}

		path_ptr1 = path_ptr1->next;
	}

	return FALSE;
}


void test_gamma2()
{
	double prob = 0;
	prob = GSL_TPD_Encounter_Probability_For_Gamma_Distribution(209.20,22.96,278.95,24.08,0,3000,117.45,119.55); printf("TEST = %.4f / 0.0143\n",prob);
	prob = GSL_TPD_Encounter_Probability_For_Gamma_Distribution(146.82,/*15.53*/2.5,105.75,/*15.25*/2.5,0,3000,80.07,88.09); printf("TEST = %.4f / 0.9712\n",prob);
	//prob = GSL_TPD_Encounter_Probability_For_Gamma_Distribution(226.00,20.88,495.85,33.33,0,3000,61.98,75.33); printf("TEST = %.4f / 0\n",prob);
	prob = GSL_TPD_Encounter_Probability_For_Gamma_Distribution(509.20,2.5,504.44,2.5,0,3000,117.45,117.45); printf("TEST = %.4f / 000000\n",prob);
	//prob = GSL_TPD_Encounter_Probability_For_Gamma_Distribution(129.11,14.73,106.76,15.39,0,3000,80.07,77.47); printf("TEST = %.4f / 0.8573\n",prob);

}


double TPD_Compute_EDR_For_Encounter_Graph(adjacency_list_queue_t *G)
{ /* compute the EDR (Expected Delivery Ratio) for the given encounter graph G 
	 by Breadth First Search (BFS) from destination vehicle to source vehicle in G */ 
	adjacency_list_pointer_queue_t Q; //queue for adjacency list pointers
	adjacency_list_pointer_queue_node_t qnode; //queue node for the pointer to a graph node
	adjacency_list_pointer_queue_node_t *p = NULL; //pointer to the pointer to a graph node
	adjacency_list_pointer_queue_node_t *pNewNode = NULL; //pointer to the pointer to a graph node
	adjacency_list_queue_node_t *pQueueNode = NULL; //a pointer to a graph node
	parent_list_queue_node_t *q = NULL; //pointer to a parent queue node
	neighbor_list_queue_node_t *r = NULL; //pointer to a child queue node
	int i = 0; //index
	double P_forward = 0; //forwarding probability
	int count = 0; //count to check whether pQueueNode->EDR is computed completely

	/** initialize Q */
	InitQueue((queue_t*)&Q, QTYPE_ADJACENCY_LIST_POINTER);

	/** search the graph G from leaf dst_vehicle_gnode toward root 
	 * src_vehicle_gnode in order to compute EDR */
	memset(&qnode, 0, sizeof(qnode));
	qnode.graph_qnode = G->dst_vehicle_gnode;
	pNewNode = (adjacency_list_pointer_queue_node_t*)Enqueue((queue_t*)&Q, (queue_node_t*)&qnode);
	pNewNode->graph_qnode->queueing_flag = TRUE; //set queueing_flag to TRUE

	do //do-while-1
	{
		if(Q.size == 0)
		{
			break;
		}
		
		p = (adjacency_list_pointer_queue_node_t*)Dequeue((queue_t*)&Q);
		p->graph_qnode->queueing_flag = FALSE; //reset queueing_flag to FALSE
		pQueueNode = p->graph_qnode;

		/* compute EDR for the vehicle for p */
		if(pQueueNode->id == G->dst_vehicle_gnode->id)
		{ //if p is the graph node for destination vehicle, the EDR is set to 1.
			pQueueNode->EDR = 1;
			pQueueNode->EDR_flag = TRUE;
		}
		else
		{
			/* Note: We assume that pQueueNode has at least one child node in neighbor_list
			 * because it is not the destination vehicle. */

			/* We assume that its child nodes are sorted in the ascending 
			 * order of the encounter time for the calculation of EDR 
			 * at this graph node */
			pQueueNode->EDR = 0;

			/* check what child nodes are not resolved (i.e., not having 
			 * their EDR computed yet). The unresolved child nodes
			 * are enqueued into Q */
			count = 0;
			r = &(pQueueNode->neighbor_list.head);
			for(i = 0; i < pQueueNode->neighbor_list.size; i++)
			{
				r = r->next;

				if(r->graph_qnode->EDR_flag == FALSE)
				{
					/* The graph node pointed by r is resolved for EDR */
#if TPD_EDR_EDD_COMPUTATION_TRACE_FLAG /* [ */ 
					printf("%s:%d graph_node(%d) is not resolved for EDR\n",
							__FUNCTION__, __LINE__,
							r->graph_qnode->id);
#endif /* ] */

					/* enqueue the pointer to this graph node into Q */
					/* Note: We defer the resolution of the node having unresolved 
					 *		child node(s) by enqueueing the node into the queue Q.
					 *		 This deferring allows the unresolved child node(s) to  
					 *		be resolved earlier than the unresolved parent node.
					 */
					/* check whether the pointer to r->graph_qnode is already queued in Q or not */
					if(pQueueNode->queueing_flag == FALSE)
					{
						memset(&qnode, 0, sizeof(qnode));
						qnode.graph_qnode = pQueueNode;
						pNewNode = (adjacency_list_pointer_queue_node_t*)Enqueue((queue_t*)&Q, (queue_node_t*)&qnode);
						pNewNode->graph_qnode->queueing_flag = TRUE; //set queueing_flag to TRUE
					}
					break;
				}
				else
				{
					/* compute the forwarding probability that this graph node
					 * pointed by pQueueNode can forward its packets to the
					 * child node pointed by r */
					P_forward = TPD_Compute_Forwarding_Probability(&(pQueueNode->neighbor_list), i);
					pQueueNode->EDR += P_forward * r->graph_qnode->EDR;
					count++;
				}
			}
		}

		/* check whether pQueueNode->EDR is computed completely with 
		 * count == pQueueNode->neighbor_list.size, which means that
		 * the EDRs of all the child nodes are resolved */
		if(count == pQueueNode->neighbor_list.size)
		{
			pQueueNode->EDR_flag = TRUE;	

			/* enqueue the parent nodes for p into Q for Breadth First Search (BFS) */
			q = &(pQueueNode->parent_list.head);
			for(i = 0; i < pQueueNode->parent_list.size; i++)
			{
				q = q->next;

				/* check whether the pointer to q->graph_qnode is already queued in Q or not */
				if(q->graph_qnode->queueing_flag == FALSE)
				{
					memset(&qnode, 0, sizeof(qnode));
					qnode.graph_qnode = q->graph_qnode;
					pNewNode = (adjacency_list_pointer_queue_node_t*)Enqueue((queue_t*)&Q, (queue_node_t*)&qnode);
					pNewNode->graph_qnode->queueing_flag = TRUE; //set queueing_flag to TRUE
				}
			}
		}

		/* delete p because the memory for p must be returned */
		DestroyQueueNode(Q.type, (queue_node_t*)p);
	} while(1);

	/** destroy Q */
	DestroyQueue((queue_t*)&Q);

	return G->src_vehicle_gnode->EDR;
}

double TPD_Compute_EDR_For_Encounter_Graph_By_DP(adjacency_list_queue_t *G)
{ /* compute the EDR (Expected Delivery Ratio) for the given encounter graph G 
	 by Breadth First Search (BFS) from destination vehicle to source vehicle in G 
	 and Dynamic Programming (DP) for an optimal forwarding subsequence */ 
	adjacency_list_pointer_queue_t Q; //queue for adjacency list pointers
	adjacency_list_pointer_queue_node_t qnode; //queue node for the pointer to a graph node
	adjacency_list_pointer_queue_node_t *p = NULL; //pointer to the pointer to a graph node
	adjacency_list_pointer_queue_node_t *pNewNode = NULL; //pointer to the pointer to a graph node
	adjacency_list_queue_node_t *pQueueNode = NULL; //a pointer to a graph node
	parent_list_queue_node_t *q = NULL; //pointer to a parent queue node
	neighbor_list_queue_node_t *r = NULL; //pointer to a child queue node
	neighbor_list_queue_node_t *pFirstPosition = NULL; //the pointer to the first node in the optimal forwarding subsequence in the neighbor-list

	int i = 0, j = 0, k = 0; //loop indices
	double P_forward = 0; //forwarding probability
	int count = 0; //count to check whether pQueueNode->EDR is computed completely
	int n = 0; //the number of child vehicles in neihghbor_list
	double maximum_EDR = 0; //maximum EDR
	boolean loop_exit_flag = FALSE;

	/** initialize Q */
	InitQueue((queue_t*)&Q, QTYPE_ADJACENCY_LIST_POINTER);

	/** search the graph G from leaf dst_vehicle_gnode toward root 
	 * src_vehicle_gnode in order to compute EDR */
	memset(&qnode, 0, sizeof(qnode));
	qnode.graph_qnode = G->dst_vehicle_gnode;
	pNewNode = (adjacency_list_pointer_queue_node_t*)Enqueue((queue_t*)&Q, (queue_node_t*)&qnode);
	pNewNode->graph_qnode->queueing_flag = TRUE; //set queueing_flag to TRUE

	do //do-while-1
	{
		if(Q.size == 0)
		{
			break;
		}
		
		p = (adjacency_list_pointer_queue_node_t*)Dequeue((queue_t*)&Q);
		p->graph_qnode->queueing_flag = FALSE; //reset queueing_flag to FALSE
		pQueueNode = p->graph_qnode;

		/* compute EDR for the vehicle for p */
		if(pQueueNode->id == G->dst_vehicle_gnode->id) //if-1.1
		{ //if p is the graph node for destination vehicle, the EDR is set to 1.
			pQueueNode->EDR = 1;
			pQueueNode->EDR_flag = TRUE;
		} //end of if-1.1
		else //else-1.2
		{
			/* Note: We assume that pQueueNode has at least one child node in neighbor_list
			 * because it is not the destination vehicle. */

			/* We assume that its child nodes are sorted in the ascending 
			 * order of the encounter time for the calculation of EDR 
			 * at this graph node */

			/* We compute an optimal forwarding subsequence for the current graph node
			 * while checking what child nodes are not resolved (i.e., not having 
			 * their EDR computed yet). The unresolved child node are enqueued into Q */
			count = 0;
			n = pQueueNode->neighbor_list.size;
			maximum_EDR = 0;
			loop_exit_flag = FALSE;

			/* add the last vehicle in neighbor_list to an optimal forwarding
			 * subsequence as a last resort */
			pQueueNode->neighbor_list.head.prev->optimal_subsequence_flag = TRUE;

			for(k = 1; k <= n; k++) //for-1.2.1
			{
				r = &(pQueueNode->neighbor_list.head);

				pQueueNode->EDR = 0; //reset pQueueNode->EDR
				
				/* position r as the first vehicle in the current forwarding subsequence for k.
				 * r currently points to the head node in pQueueNode->neighbor_list. 
				 * move r from the head node in the backward direction by k-1.
				 */
				for(i = 1; i <= k; i++)
				{
					r = r->prev;	
				}

				/* Set pFirstPosition to the pointer to the first node pointed by r in
				 * the optimal forwarding subsequence in  the neighbor-list */
				pFirstPosition = r; 

				/* determine whether the vehicle pointed by pFirstPosition must be added
				 * into the optimal forwarding subsequence as the first forwarding vehicle 
				 * where the optimal forwarding subsequence may be 
				 * (v_{n-(k-1)}, v_{n-(k-2)}, ..., v_{n})
				 * when the new first vehicl v_{n-(k-1)} is added. 
				 * Note that the forwarding sequence is (v_{1}, v_{2}, ..., v_{n-k}, v_{n-(k-1)}, 
				 * v_{n-(k-2)}, ..., v_{n}).
				 * Refer to Appendix in TPD Journal Paper, that is, Optimal Forwarding Path
				 * Computation. */
				for(i = n-(k-1); i <= n; i++, r = r->next) //for-1.2.1.2
				{
					/* check whether this vehicle belongs to an optimal forwarding 
					 * subsequence; note that the first forwarding candidate for i = n-(k-1) must be
					 * considered for a larger optimal forwarding subsequence; 
					 * otherwise, we need to check whether this vehicle is already in the current
					 * optimal forwarding subsequence by optimal_subsequence_flag. */
					if((i > n-(k-1)) && (r->optimal_subsequence_flag == FALSE))
					{
						continue;
					}

					if(r->graph_qnode->EDR_flag == FALSE)
					{
						/* The graph node pointed by r is resolved for EDR */
#if TPD_EDR_EDD_COMPUTATION_TRACE_FLAG /* [ */ 
						printf("%s:%d graph_node(%d) is not resolved for EDR\n",
								__FUNCTION__, __LINE__,
								r->graph_qnode->id);
#endif /* ] */
	
						/* enqueue the pointer to this graph node into Q */
						/* Note: We defer the resolution of the node having unresolved 
						 *		child node(s) by enqueueing the node into the queue Q.
						 *		 This deferring allows the unresolved child node(s) to  
						 *		be resolved earlier than the unresolved parent node.
						 */
						/* check whether the pointer to r->graph_qnode is already queued in 
						 * Q or not */
						if(pQueueNode->queueing_flag == FALSE)
						{
							memset(&qnode, 0, sizeof(qnode));
							qnode.graph_qnode = pQueueNode;
							pNewNode = (adjacency_list_pointer_queue_node_t*)Enqueue((queue_t*)&Q, (queue_node_t*)&qnode);
							pNewNode->graph_qnode->queueing_flag = TRUE; 
							//set queueing_flag to TRUE
						}

						loop_exit_flag = TRUE;
						break;
					}
					else
					{
						/* compute the forwarding probability that this graph node
						 * pointed by pQueueNode can forward its packets to the
						 * child node pointed by r */
						P_forward = TPD_Compute_Forwarding_Probability_For_Optimal_Forwarding_Subsequence(pFirstPosition, n-(k-1), i, n);
						pQueueNode->EDR += P_forward * r->graph_qnode->EDR;
						//count++;
					}
				} //end of for-1.2.1.2

				if(loop_exit_flag)
				{
					break;
				}

				/* determine whether the vehicle pointed by r must be included for an optimal
				 * forwarding subsequence as the first forwarder candidate along with the 
				 * update of maximum_EDR */
				if(pQueueNode->EDR > maximum_EDR)
				{
					pFirstPosition->optimal_subsequence_flag = TRUE; //add the vehicle pointed by pFirstPosition into an optimal forwarding subsequence for a larger optimal forwarding subsequence
					maximum_EDR = pQueueNode->EDR;
				}
				
				count++; //increase count to show how many neighbor vehicles are checked
			} //end of for-1.2.1

			/* set pQueueNode->EDR to maximum_EDR */
			if(loop_exit_flag == FALSE)
			{
				pQueueNode->EDR = maximum_EDR;
			}
		} //end of else-1.2

		/* check whether pQueueNode->EDR is computed completely with 
		 * count == pQueueNode->neighbor_list.size, which means that
		 * the EDRs of all the child nodes are resolved */
		if(count == pQueueNode->neighbor_list.size)
		{
			pQueueNode->EDR_flag = TRUE;	

			/* enqueue the parent nodes for p into Q for Breadth First Search (BFS) */
			q = &(pQueueNode->parent_list.head);
			for(i = 0; i < pQueueNode->parent_list.size; i++)
			{
				q = q->next;

				/* check whether the pointer to q->graph_qnode is already queued in Q or not */
				if(q->graph_qnode->queueing_flag == FALSE)
				{
					memset(&qnode, 0, sizeof(qnode));
					qnode.graph_qnode = q->graph_qnode;
					pNewNode = (adjacency_list_pointer_queue_node_t*)Enqueue((queue_t*)&Q, (queue_node_t*)&qnode);
					pNewNode->graph_qnode->queueing_flag = TRUE; //set queueing_flag to TRUE
				}
			}
		}

		/* delete p because the memory for p must be returned */
		DestroyQueueNode(Q.type, (queue_node_t*)p);
	} while(1);

	/** destroy Q */
	DestroyQueue((queue_t*)&Q);

	return G->src_vehicle_gnode->EDR;
}

double TPD_Compute_EDD_For_Encounter_Graph(adjacency_list_queue_t *G)
{ /* compute the EDD (Expected Delivery Delay) for the given encounter graph G
	 by Breadth First Search (BFS) from destination vehicle to source vehicle in G */
	adjacency_list_pointer_queue_t Q; //queue for adjacency list pointers
	adjacency_list_pointer_queue_node_t qnode; //queue node for the pointer to a graph node
	adjacency_list_pointer_queue_node_t *p = NULL; //pointer to the pointer to a graph node
	adjacency_list_pointer_queue_node_t *pNewNode = NULL; //pointer to the pointer to a graph node
	adjacency_list_queue_node_t *pQueueNode = NULL; //pointer to a graph node
	parent_list_queue_node_t *q = NULL; //pointer to a parent queue node
	neighbor_list_queue_node_t *r = NULL; //pointer to a child queue node
	int i = 0; //index
	double P_forward = 0; //forwarding probability
	double Q_forward = 0; //conditional delivery probability that the packet from node e is successfully delivered to the destination s via the i-th forwarder of node e under the precondition that the packet from e is successfully delivered to the destination node s
	int count = 0; //count to check whether pQueueNode->EDD is computed completely

	/** initialize Q */
	InitQueue((queue_t*)&Q, QTYPE_ADJACENCY_LIST_POINTER);

	/** search the graph G from leaf dst_vehicle_gnode toward root 
	 * src_vehicle_gnode in order to compute EDD */
	memset(&qnode, 0, sizeof(qnode));
	qnode.graph_qnode = G->dst_vehicle_gnode;
	pNewNode = (adjacency_list_pointer_queue_node_t*)Enqueue((queue_t*)&Q, (queue_node_t*)&qnode);
	pNewNode->graph_qnode->queueing_flag = TRUE; //set queueing_flag to TRUE

	do //do-while-1
	{
		if(Q.size == 0)
		{
			break;
		}
		
		p = (adjacency_list_pointer_queue_node_t*)Dequeue((queue_t*)&Q);
		p->graph_qnode->queueing_flag = FALSE; //reset queueing_flag to FALSE
		pQueueNode = p->graph_qnode;

		/* compute EDD for the vehicle for p */
		if(pQueueNode->id == G->dst_vehicle_gnode->id)
		{ //if p is the graph node for destination vehicle, the EDD is set to 0.
			pQueueNode->EDD = 0;
			pQueueNode->EDD_flag = TRUE;
		}
		else
		{
			/* Note: We assume that pQueueNode has at least one child node in neighbor_list
			 * because it is not the destination vehicle. */

			/* We assume that its child nodes are sorted in the ascending 
			 * order of the encounter time for the calculation of EDD 
			 * at this graph node */
			pQueueNode->EDD = 0;

			/* check what child nodes are not resolved (i.e., not having 
			 * their EDD computed yet). The unresolved child nodes
			 * are enqueued into Q_unresolved */
			count = 0;
			r = &(pQueueNode->neighbor_list.head);
			for(i = 0; i < pQueueNode->neighbor_list.size; i++)
			{
				r = r->next;

				if(r->graph_qnode->EDD_flag == FALSE)
				{
					/* The graph node pointed by r is resolved for EDD */
#if TPD_EDR_EDD_COMPUTATION_TRACE_FLAG /* [ */ 					
					printf("%s:%d graph_node(%d) is not resolved for EDD\n",
							__FUNCTION__, __LINE__,
							r->graph_qnode->id);
#endif /* ] */

					/* enqueue the pointer to this graph node into Q */
					/* Note: We defer the resolution of the node having unresolved 
					 *		child node(s) by enqueueing the node into the queue Q.
					 *		 This deferring allows the unresolved child node(s) to  
					 *		be resolved earlier than the unresolved parent node.
					 */
					/* check whether the pointer to r->graph_qnode is already queued in Q or not */
					if(pQueueNode->queueing_flag == FALSE)
					{
						memset(&qnode, 0, sizeof(qnode));
						qnode.graph_qnode = pQueueNode;
						pNewNode = (adjacency_list_pointer_queue_node_t*)Enqueue((queue_t*)&Q, (queue_node_t*)&qnode);
						pNewNode->graph_qnode->queueing_flag = TRUE; //set queueing_flag to TRUE
					}
					break;
				}
				else
				{
					/* compute the forwarding probability that this graph node
					 * pointed by pQueueNode can forward its packets to the
					 * child node pointed by r */
					P_forward = TPD_Compute_Forwarding_Probability(&(pQueueNode->neighbor_list), i);
					
					if(pQueueNode->EDR <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
					{
						Q_forward = 0;
					}
					else
					{
						Q_forward = P_forward * (r->graph_qnode->EDR / pQueueNode->EDR);
					}

					pQueueNode->EDD += Q_forward * (r->weight + r->graph_qnode->EDD);
					//pQueueNode->EDR += P_forward * r->graph_qnode->EDR;
					count++;
				}
			}
		}

		/* check whether pQueueNode->EDD is computed completely with 
		 * count == pQueueNode->neighbor_list.size, which means that
		 * the EDDs of all the child nodes are resolved */
		if(count == pQueueNode->neighbor_list.size)
		{
			pQueueNode->EDD_flag = TRUE;	
			/* enqueue the parent nodes for p into Q for Breadth First Search (BFS) */
			q = &(pQueueNode->parent_list.head);
			for(i = 0; i < pQueueNode->parent_list.size; i++)
			{
				q = q->next;

				memset(&qnode, 0, sizeof(qnode));
				qnode.graph_qnode = q->graph_qnode;
				Enqueue((queue_t*)&Q, (queue_node_t*)&qnode);

				/* check whether the pointer to q->graph_qnode is already queued in Q or not */
				if(q->graph_qnode->queueing_flag == FALSE)
				{
					memset(&qnode, 0, sizeof(qnode));
					qnode.graph_qnode = q->graph_qnode;
					pNewNode = (adjacency_list_pointer_queue_node_t*)Enqueue((queue_t*)&Q, (queue_node_t*)&qnode);
					pNewNode->graph_qnode->queueing_flag = TRUE; //set queueing_flag to TRUE
				}
			}
		}

		/* delete p because the memory for p must be returned */
		DestroyQueueNode(Q.type, (queue_node_t*)p);
	} while(1);

	/** destroy Q */
	DestroyQueue((queue_t*)&Q);

	if (G->src_vehicle_gnode->id == 193)
		printf("\n\n)))))))) %.2f\n",G->src_vehicle_gnode->EDD);
	return G->src_vehicle_gnode->EDD;
}

double TPD_Compute_Forwarding_Probability(neighbor_list_queue_t *Q, 
		int i)
{ /* compute the forwarding probability that a packet carrier vehicle can
	forward its packets to the vehicle corresponding to index i (i.e.,
	the i-th vehicle) in Q when the packet carrier vehicle fails to encounter
	the vehicles before the i-th vehicle. */
	double P_forward = 1; //forwarding probability, initialized with 1
	int j = 0; //index
	neighbor_list_queue_node_t *pQueueNode = NULL; //pointer to a neighbor list queue node

	/* check the validity of i */
	if(i >= Q->size)
	{
		printf("%s:%d i(%d) >= Q->size(%d)\n",
				__FUNCTION__, __LINE__,
				i, Q->size);
		return 0;
	}

	pQueueNode = &(Q->head);
	for(j = 0; j < Q->size; j++)
	{
		pQueueNode = pQueueNode->next;

		if(j == i)
		{
			P_forward *= pQueueNode->P_encounter;
			break; //Note: include "break" to terminate the P_forward computation in Eq. (22) in TPD Paper
		}
		else
		{
			P_forward *= (1 - pQueueNode->P_encounter);	
		}
	}

	return P_forward;
}

double TPD_Compute_Forwarding_Probability_For_Optimal_Forwarding_Subsequence(neighbor_list_queue_node_t *pFirstPosition, 
		int first_position, 
		int candidate_position,	
		int last_position)
{ /* compute the forwarding probability that a packet carrier vehicle 
	candidate_position can forward its packets to the vehicle corresponding to 
	candidate_position in the neighbor-list when the packet carrier vehicle fails 
	to encounter the vehicles from first_position to candidate_position - 1. */
	double P_forward = 1; //forwarding probability, initialized with 1
	int j = 0; //index
	neighbor_list_queue_node_t *pQueueNode = NULL; //pointer to a neighbor list queue node

	/* check the validity of pFirstPosition, first_position, candidate_position, and 
	 * last_position */
	if(pFirstPosition == NULL)
	{
		printf("%s:%d pFirstPosition is NULL\n",
				__FUNCTION__, __LINE__);
		return 0;
	}
	else if(first_position <= 0)
	{
		printf("%s:%d first_position(%d) must be positive!\n",
				__FUNCTION__, __LINE__,
				first_position);
		return 0;
	}
	else if(candidate_position < first_position)
	{
		printf("%s:%d candidate_position(%d) < first_position(%d)!\n",
				__FUNCTION__, __LINE__,
				candidate_position, first_position);
		return 0;
	}
	else if(last_position < candidate_position)
	{
		printf("%s:%d last_position(%d) < candidate_position(%d)!\n",
				__FUNCTION__, __LINE__,
				last_position, candidate_position);
		return 0;
	}

	pQueueNode = pFirstPosition; //set pQueueNode to the first vehicle pointed by pFirstPosition 
	for(j = first_position; j <= candidate_position; j++, pQueueNode = pQueueNode->next)
	{
		if(j == candidate_position)
		{ /* We assume that the vehicle for candidate_position is already in the optimal 
		   * forwarding subsequence, that is, with optimal_subsequence_flag TRUE.
		   * As an exception, when candidate_position vehicle is first_position vehicle,
		   * we allow this candidate_position vehicle as a valid one. This is because
		   * the first_position vehicle is being considered to be included into the 
		   * optimal forwarding subsequence. */
			if((pQueueNode->optimal_subsequence_flag == TRUE) || (candidate_position == first_position))
			{
				P_forward *= pQueueNode->P_encounter;
				break; //Note: include "break" to terminate the P_forward computation in Eq. (22) in TPD Paper
			}
			else
			{
#if 1 /* [ */
				printf("%s:%d the vehicle(%d) for candidate_position(%d) must have TRUE for optimal_subsequence_flag(%d)\n",
						__FUNCTION__, __LINE__,
						pQueueNode->id, 
						candidate_position,	
						pQueueNode->optimal_subsequence_flag);
#endif /* ] */
				P_forward = 0;
				break;
			}
		}
		else
		{
			/* check whether the vehicle pointed by pQueueNode belongs to the optimal forwarding
			 * subsequence except for the first vehicle pointed by pFirstPosition. This is 
			 * because the first vehicle is inspected to check whether it must be added to
			 * the optimal forwarding subsequence. */
			if((j == first_position) || (pQueueNode->optimal_subsequence_flag == TRUE))
			{
				P_forward *= (1 - pQueueNode->P_encounter);	
			}
		}
	}

	return P_forward;
}

boolean TPD_Is_There_Next_Carrier_At_Intersection_For_AP(parameter_t *param, 
		double current_time, 
		struct_access_point_t *AP, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier)
{ //Under V2V mode, determine whether to forward AP's packets to next carrier moving on the other road segment with the highest EDR at intersection and return the pointer to the next carrier through *next_carrier
	boolean result = FALSE; //return value
	boolean flag = FALSE; //flag to indicate there exists a next carrier candidate
	struct_graph_node *pGraphNode = NULL; //pointer to a graph node
	char *tail_node_for_next_forwarding_edge = NULL; //tail node of the next directional edge for forwarding
	char *head_node_for_next_forwarding_edge = NULL; //head node of the next directional edge for forwarding
	int size = 0; //size of intersection EDD queue
	int i = 0; //index for for-loop
	//double max_next_carrier_EDR = 0; //maximum value of next carrier's EDR
	double min_next_carrier_EDD = INF; //minimum value of next carrier's EDD
	struct_vehicle_t *next_carrier_candidate = NULL; //pointer to the next carrier candidate
	directional_edge_type_t edge_type = OUTGOING_EDGE; //directional edge type for tail_node
	struct_graph_node *intersection_gnode = NULL; //pointer to the graph node of the intersection where the vehicle is arriving
	struct_graph_node *neighboring_intersection_gnode = NULL; //pointer to the graph node of the neighboring intersection for the intersection where the vehicle is arriving

	/** reset *next_carrier to NULL */
	*next_carrier = NULL;

	/** check whether AP has packets to forward to a next carrier; 
	 * if there is no packet, return FALSE without further checking 
	 * for a next carrier */
	if(AP->packet_queue->size == 0)
	{
		return FALSE;
	}

	/** search for a best next carrier in the order of the highest EDRs assigned
	 * to directional edges incident to the current intersect where the vehicle 
	 * has reached. */
	intersection_gnode = AP->gnode; //let AP's gnode become intersection gnode
	neighboring_intersection_gnode = intersection_gnode;
	size = (int)intersection_gnode->weight;
	//printf("**** %d\n",size);
	
	for(i = 0; i < size; i++) //for-1
	{
		neighboring_intersection_gnode = neighboring_intersection_gnode->next;
    
		/** search a next carrier candidate in the outgoing edge of 
		 * <intersection, neighboring_intersection> */
		edge_type = OUTGOING_EDGE;
		tail_node_for_next_forwarding_edge = intersection_gnode->vertex;
		head_node_for_next_forwarding_edge = neighboring_intersection_gnode->vertex;
  
	    //printf("TPD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_For_AP\n");
		/* check whether the tail node is one of APs; ap_flag is used to determine next carrier */
		flag = TPD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_For_AP(param, 
				current_time,
				AP, 
				tail_node_for_next_forwarding_edge, 
				head_node_for_next_forwarding_edge, 
				edge_type, 
				G, 
				G_size, 
				&next_carrier_candidate);
				/* determine whether to forward its packets to next carrier moving 
				 * on the road segment incident to an intersection corresponding to 
				 * tail_node for access point AP and return the pointer to the next 
				 * carrier through *next_carrier */
     	// taehwan 20140714
		/*if ( current_time > 7221 && current_time < 7222) 
		{
			printf(">>> (%s,%s)=%s\n",
					tail_node_for_next_forwarding_edge,
					head_node_for_next_forwarding_edge,
					flag==TRUE?"TRUE":"FALSE");
		}*/


		if(flag) //if-1
		{
			/* select a vehicle with the maximum next carrier EDR as a next carrier */
			switch(param->vehicle_vanet_forwarding_type) //switch-1
			{
				case VANET_FORWARDING_BASED_ON_VEHICLE:
				case VANET_FORWARDING_BASED_ON_CONVOY:
					if((next_carrier_candidate->EDR_for_V2V >= param->tpd_delivery_probability_threshold) && (next_carrier_candidate->EDD_for_V2V < min_next_carrier_EDD))
					//if(next_carrier_candidate->EDR_for_V2V > max_next_carrier_EDR)
					{
						*next_carrier = next_carrier_candidate;
						min_next_carrier_EDD = next_carrier_candidate->EDD_for_V2V;
						//max_next_carrier_EDR = next_carrier_candidate->EDR_for_V2V;
						result = flag; /* set result to TRUE since there exists next 
										  carrier candidate */
					}
					break;

				default:
					printf("%s:%d: \
							param->vehicle_vanet_forwarding_type(%d) is not supported!\n", 
							__FUNCTION__, __LINE__,
							param->vehicle_vanet_forwarding_type);
					exit(1);
			} //end of switch-1
		} //end of if-1
	} //end of for-1
	
	/* taehwan 20140715 */
	/*
	if  (current_time > 7221 && current_time < 7222 )
	{
		printf("**** %s\n",(result==TRUE?"TRUE":"FALSE"));
	}*/
	/* check whether next_carrier's EDR is greater than the tpd_delivery_probability_threshold */
	if(result == TRUE)
	{
		printf("%.2f] TPD Is There Next Carrier At Intersection For AP %d\n"
				,current_time
				,(*next_carrier)->id);

		if((*next_carrier)->EDR_for_V2V > param->tpd_delivery_probability_threshold)
		{
#if TPD_AP_FORWARDING_TRACE_FLAG /* [*/
			printf("<I> %s:%d [%0.f] AP forwards packets to next_carrier(%d) with EDR(%0.3f) and EDD(%0.f): next_carrier(%s->%s: %0.f), destination_vehicle(%s->%s: %0.f).\n",
					__FUNCTION__, __LINE__,
					(float)current_time,
					(*next_carrier)->id,
					(*next_carrier)->EDR_for_V2V,
					(*next_carrier)->EDD_for_V2V,
					(*next_carrier)->current_pos_in_digraph.tail_node,
					(*next_carrier)->current_pos_in_digraph.head_node,
					(*next_carrier)->current_pos_in_digraph.offset,
					param->vanet_table.dst_vnode->current_pos_in_digraph.tail_node,
    				param->vanet_table.dst_vnode->current_pos_in_digraph.head_node,
    				param->vanet_table.dst_vnode->current_pos_in_digraph.offset);
#endif /* ] */

			return result;
		}
		result = FALSE;
	}

	return result;
}

boolean TPD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_For_AP(
		parameter_t *param, 
		double current_time, 
		struct_access_point_t *AP, 
		char *tail_node_for_next_forwarding_edge, 
		char *head_node_for_next_forwarding_edge, 
		directional_edge_type_t edge_type, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier)
{ //Under V2V mode, determine whether to forward AP's packets to next carrier moving on the road segment incident to an intersection corresponding to tail_node_for_next_edge and return the pointer to the next carrier through *next_carrier
	boolean result = FALSE;
	boolean flag = FALSE; //flag to indicate whether a next carrier is determined or not
	double distance = 0; //distance between vehicle and another vehicle moving on the same directional edge
	//double max_neighbor_EDR = 0; //neighbor vehicle with the maximum EDR
	double min_neighbor_EDD = INF; //neighbor vehicle with the minimum EDD
	char *tail_node = tail_node_for_next_forwarding_edge; //tail node of the next directional edge for forwarding
	char *head_node = head_node_for_next_forwarding_edge; //head node of the next directional edge for forwarding
	directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
	vehicle_movement_queue_node_t *pMoveNode = NULL; //pointer to the vehicle movement queue
	int i = 0; //index for for-loop
	int size = 0; //size of vehicle movement queue

	/** reset *next_carrier to NULL */
	*next_carrier = NULL;

	/** obtain the pointer to the direaction edge of <tail_node,head_node> */
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);

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
		
#ifdef TPD_FORWARD_LOG
	if (size > 0)
	{   
		printf("%.2f] taehwan11241605 TPD IsDareNext %s:%s size:%d (",current_time,tail_node,head_node,size);
   		for(i = 0; i < size ; i++)
		{
			pMoveNode = pMoveNode->next;
			printf("%s%d",(i==0?"":","),pMoveNode->vnode->id);
		}
		printf(")\n");
	}
#endif

	pMoveNode = &(pEdgeNode->vehicle_movement_list.head);
	for(i = 0; i < size && flag == FALSE; i++) //for-1
	{
		pMoveNode = pMoveNode->next;
		if(edge_type == OUTGOING_EDGE) //for the outgoing edge of the intersection (i.e., tail_node)
			distance = pMoveNode->offset;
		else //for the incoming edge of the intersection (i.e., head_node)
			distance = fabs(pEdgeNode->weight - pMoveNode->offset);
 
		/* check the distance between two vehicles */
		if(distance > param->communication_range)
		{
			continue;
		}

		printf("         taehwan11291022 TPD IsDareNext  %d in range at %.2f(%s-%s)\n",pMoveNode->vnode->id,pMoveNode->vnode->path_current_edge_offset,pMoveNode->vnode->current_pos_in_digraph.tail_node,pMoveNode->vnode->current_pos_in_digraph.head_node);
		//TPD_Print_Vehicle_Trajectory(pMoveNode->vnode);
		/** Next carrier selection rule: We select a vehicle with the highest EDR as
		 * a next carrier */
#if TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FOR_AP_FLAG /* [ */
		TPD_Compute_EDR_and_EDD(current_time, 
				param,
				pMoveNode->vnode,
				param->vanet_table.dst_vnode,
				TRUE);
#else
		TPD_Compute_EDR_and_EDD(current_time, 
				param,
				pMoveNode->vnode,
				param->vanet_table.dst_vnode,
				FALSE);
#endif /* ] */
		/* taehwan 20140714 */
		/*
		if (current_time > 7221 && current_time < 7222)
			printf("%.2f < %.2f : %.2f >= %.2f && %.2f < %.2f\n",
					distance,param->communication_range,
					pMoveNode->vnode->EDR_for_V2V,
					param->tpd_delivery_probability_threshold,
					pMoveNode->vnode->EDD_for_V2V,
					min_neighbor_EDD);
		*/
         	//printf("%.2f] taehwan11241605 TPD IsDareNext ___ num:%d/%d veh_id:%3d EDR:%2.2f %s\n",current_time,i,size,pMoveNode->vnode->id,pMoveNode->vnode->EDR_for_V2V,(pMoveNode->vnode->EDR_for_V2V?"<-----":""));
		/* select a next vehicle that has a higher EDR */
		switch(param->vehicle_vanet_forwarding_type) //switch-1
		{
			case VANET_FORWARDING_BASED_ON_VEHICLE:
			case VANET_FORWARDING_BASED_ON_CONVOY:
				if((pMoveNode->vnode->EDR_for_V2V >= param->tpd_delivery_probability_threshold) && (pMoveNode->vnode->EDD_for_V2V < min_neighbor_EDD))
				//if(pMoveNode->vnode->EDR_for_V2V > max_neighbor_EDR)
				{ //we also check vehicles' EDRs
					//printf("taehwan11252101 TPD IsDareNext ___ more than threshold:%2.2f\n",param->tpd_delivery_probability_threshold);
					min_neighbor_EDD = pMoveNode->vnode->EDD_for_V2V;
					//max_neighbor_EDR = pMoveNode->vnode->EDR_for_V2V;
					*next_carrier = pMoveNode->vnode; /* update the pointer of neighbor 
							vehicle node with higher EDR */
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

boolean TPD_Is_There_Next_Carrier_At_Intersection(parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier)
{ //determine whether to be able to forward its packets to next carrier at the intersection according to the forwarding criteria, depending on param's tpd_encounter_graph_source_routing_flag and return the pointer to the next carrier through *next_carrier
	boolean result = FALSE; //return value

	if(param->tpd_encounter_graph_source_routing_flag)
	{
		/* determine whether to be able to forward its packets to next carrier that 
		 * is one of the child vehicles of the current packet carrier vehicle in
		 * vehicle's predicted encounter graph at the intersection and return the 
		 * pointer to the next carrier through *next_carrier. */
		result = TPD_Is_There_Next_Carrier_At_Intersection_For_Source_Routing(param, 
				current_time, 
				vehicle, 
				G, 
				G_size, 
				next_carrier);
	}
	else
	{
		/* determine whether to forward its packets to next carrier with the highest 
		 * EDR, moving on the other road segment of the intersection and return the 
		 * pointer to the next carrier through *next_carrier. */
		result = TPD_Is_There_Next_Carrier_At_Intersection_For_Greedy_Routing(param, 
				current_time, 
				vehicle, 
				G, 
				G_size, 
				next_carrier);
	}

	return result;
}

boolean TPD_Is_There_Next_Carrier_At_Intersection_For_Greedy_Routing(parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier)
{ //determine whether to be able to forward its packets to next carrier with the highest EDR, moving on the other road segment of the intersection and return the pointer to the next carrier through *next_carrier
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
	//double max_next_carrier_EDR = 0; //maximum value of next carrier's EDR
	//double max_next_carrier_offset = -1; //maximum value of next carrier's offset
	double min_next_carrier_EDD = INF; //minimum value of next carrier's EDD
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

	/** search for a best next carrier in the order of the smallest EDDs assigned to directional edges incident to the current intersect where the vehicle has reached */

	/* obtain the pointer to the direaction edge of <tail_node,head_node> */
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
	if(pEdgeNode == NULL)
	{
		printf("%s:%d: pEdgeNode for <%s,%s> is NULL\n", 
				__FUNCTION__, __LINE__,
				tail_node, head_node);
		exit(1);
	}

	/** Next carrier selection rule: We select a vehicle with the shortest vehicle EDD regardless of branch EDDs: Note that in Per-Intersection model we select a farther vehicle on the best branch with the shortest branch EDD */  
	intersection_gnode = pEdgeNode->head_gnode->gnode;
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
		flag = TPD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_For_Greedy_Routing(param, 
				current_time,
				vehicle, 
				tail_node_for_next_forwarding_edge, 
				head_node_for_next_forwarding_edge, 
				edge_type, 
				G, 
				G_size, 
				&next_carrier_candidate);
				//determine whether to forward its packets to next carrier moving on the road segment incident to an intersection corresponding to tail_node and return the pointer to the next carrier through *next_carrier

		if(flag) //if-1
		{
			/* select a vehicle with the maximum next carrier EDR as a next carrier */
			switch(param->vehicle_vanet_forwarding_type) //switch-1
			{
				case VANET_FORWARDING_BASED_ON_VEHICLE:
				case VANET_FORWARDING_BASED_ON_CONVOY:
					if((next_carrier_candidate->EDR_for_V2V >= param->tpd_delivery_probability_threshold) && (next_carrier_candidate->EDD_for_V2V) < min_next_carrier_EDD)
					//if(next_carrier_candidate->EDR_for_V2V > max_next_carrier_EDR)
					{
						*next_carrier = next_carrier_candidate;
						min_next_carrier_EDD = next_carrier_candidate->EDD_for_V2V;
						//max_next_carrier_EDR = next_carrier_candidate->EDR_for_V2V;
						result = flag; //set result to TRUE since there exists next carrier candidate
					}
					break;

				default:
					printf("%s:%d: param->vehicle_vanet_forwarding_type(%d) is not supported!\n", 
							__FUNCTION__, __LINE__,
							param->vehicle_vanet_forwarding_type);
					exit(1);
			} //end of switch-1
		} //end of if-1

		/** search a next carrier candidate in the outgoing edge of <intersection, neighboring_intersection> */
		edge_type = INCOMING_EDGE;
		tail_node_for_next_forwarding_edge = neighboring_intersection_gnode->vertex;
		head_node_for_next_forwarding_edge = intersection_gnode->vertex;
    
		flag = TPD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_For_Greedy_Routing(param, 
				current_time,	
				vehicle, 
				tail_node_for_next_forwarding_edge, 
				head_node_for_next_forwarding_edge, 
				edge_type, 
				G, 
				G_size, 
				&next_carrier_candidate);
				//determine whether to forward its packets to next carrier moving on the road segment incident to an intersection corresponding to tail_node and return the pointer to the next carrier through *next_carrier

		if(flag) //if-2
		{    
			/* select a vehicle with the maximum next carrier EDR as a next carrier */
			switch(param->vehicle_vanet_forwarding_type) //switch-2
			{
				case VANET_FORWARDING_BASED_ON_VEHICLE:
				case VANET_FORWARDING_BASED_ON_CONVOY:
					if((next_carrier_candidate->EDR_for_V2V >= param->tpd_delivery_probability_threshold) && (next_carrier_candidate->EDD_for_V2V) < min_next_carrier_EDD)
					//if(next_carrier_candidate->EDR_for_V2V > max_next_carrier_EDR)
					{
						*next_carrier = next_carrier_candidate;
						min_next_carrier_EDD = next_carrier_candidate->EDD_for_V2V;
						//max_next_carrier_EDR = next_carrier_candidate->EDR_for_V2V;
						result = flag; //set result to TRUE since there exists next carrier candidate
					}
					break;

				default:
					printf("%s:%d: param->vehicle_vanet_forwarding_type(%d) is not supported!\n", 
							__FUNCTION__, __LINE__,
							param->vehicle_vanet_forwarding_type);
					exit(1);
			} //end of switch-2
		} //end of if-2
	} //end of for-1

	/** check whether a next carrier vehicle exists */
	if(*next_carrier != NULL)
	{
		/* check whether next_carrier's EDR is greater than the current packet carrier (vehicle)'s EDR */
#if TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FOR_INTERSECTION_FLAG /* [ */
		TPD_Compute_EDR_and_EDD(current_time, 
				param,
				vehicle,
				param->vanet_table.dst_vnode,
				TRUE); //recompute vehicle's EDR and EDD 
#else
		TPD_Compute_EDR_and_EDD(current_time, 
				param,
				vehicle,
				param->vanet_table.dst_vnode,
				FALSE); //recompute vehicle's EDR and EDD
#endif /* ] */
		if((*next_carrier)->EDD_for_V2V < vehicle->EDD_for_V2V)
		//if((*next_carrier)->EDR_for_V2V > vehicle->EDR_for_V2V)
		{
			result = TRUE;
#if TPD_GREEDY_ROUTING_INTERSECTION_FORWARDING_TRACE_FLAG /* [*/
			printf("<I> %s:%d [%0.f] EDR(%0.3f) and EDD(%0.f) of next_carrier(%d) is better than EDR(%0.3f) and EDD(%0.f) of current_carrier(%d): current_carrier(%s->%s: %0.f), next_carrier(%s->%s: %0.f), destination_vehicle(%s->%s: %0.f).\n",
					__FUNCTION__, __LINE__,
					(float)current_time,
					(*next_carrier)->EDR_for_V2V, (*next_carrier)->EDD_for_V2V, (*next_carrier)->id,
					vehicle->EDR_for_V2V, vehicle->EDD_for_V2V, vehicle->id,
					vehicle->current_pos_in_digraph.tail_node,
					vehicle->current_pos_in_digraph.head_node,
					vehicle->current_pos_in_digraph.offset,
					(*next_carrier)->current_pos_in_digraph.tail_node,				
					(*next_carrier)->current_pos_in_digraph.head_node,				
					(*next_carrier)->current_pos_in_digraph.offset,   				
					param->vanet_table.dst_vnode->current_pos_in_digraph.tail_node,			
    				param->vanet_table.dst_vnode->current_pos_in_digraph.head_node,
    				param->vanet_table.dst_vnode->current_pos_in_digraph.offset);
#endif /* ] */
		}
		else
		{
			result = FALSE;
		}
	}

	return result;
}

boolean TPD_Is_There_Next_Carrier_At_Intersection_For_Source_Routing(parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier)
{ //determine whether to be able to forward its packets to next carrier that is one of the child vehicles of the current packet carrier vehicle in vehicle's predicted encounter graph at the intersection and return the pointer to the next carrier through *next_carrier
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

	/** search for a best next carrier in the order of the smallest EDDs assigned to directional edges incident to the current intersect where the vehicle has reached */

	/* obtain the pointer to the direaction edge of <tail_node,head_node> */
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
	if(pEdgeNode == NULL)
	{
		printf("%s:%d: pEdgeNode for <%s,%s> is NULL\n", 
				__FUNCTION__, __LINE__,
				tail_node, head_node);
		exit(1);
	}

	/** Next carrier selection rule: We select a vehicle with the shortest vehicle EDD regardless of branch EDDs: Note that in Per-Intersection model we select a farther vehicle on the best branch with the shortest branch EDD */  
	intersection_gnode = pEdgeNode->head_gnode->gnode;
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
		flag = TPD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_For_Source_Routing(param, 
				current_time,
				vehicle, 
				tail_node_for_next_forwarding_edge, 
				head_node_for_next_forwarding_edge, 
				edge_type, 
				G, 
				G_size, 
				&next_carrier_candidate);
				//determine whether to forward its packets to next carrier moving on the road segment incident to an intersection corresponding to tail_node and return the pointer to the next carrier through *next_carrier

		if(flag) //if-1
		{
			*next_carrier = next_carrier_candidate;
			result = TRUE;
			break;
		} //end of if-1

		/** search a next carrier candidate in the outgoing edge of <intersection, neighboring_intersection> */
		edge_type = INCOMING_EDGE;
		tail_node_for_next_forwarding_edge = neighboring_intersection_gnode->vertex;
		head_node_for_next_forwarding_edge = intersection_gnode->vertex;
    
		flag = TPD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_For_Source_Routing(param, 
				current_time,	
				vehicle, 
				tail_node_for_next_forwarding_edge, 
				head_node_for_next_forwarding_edge, 
				edge_type, 
				G, 
				G_size, 
				&next_carrier_candidate);
				//determine whether to forward its packets to next carrier moving on the road segment incident to an intersection corresponding to tail_node and return the pointer to the next carrier through *next_carrier

		if(flag) //if-2
		{
			*next_carrier = next_carrier_candidate;
			result = TRUE;
			break;
		} //end of if-2
	} //end of for-1

	if(flag)
	{
#if TPD_SOURCE_ROUTING_INTERSECTION_FORWARDING_TRACE_FLAG /* [*/
		printf("<I> %s:%d [%0.f] current_carrier(%d) in (%s->%s: %0.f) encounters next_carrier(%d) in (%s->%s: %0.f) for destination_vehicle(%s->%s: %0.f).\n",
				__FUNCTION__, __LINE__,
				(float)current_time,
				vehicle->id,
				vehicle->current_pos_in_digraph.tail_node,
				vehicle->current_pos_in_digraph.head_node,
				vehicle->current_pos_in_digraph.offset,
				(*next_carrier)->id,
				(*next_carrier)->current_pos_in_digraph.tail_node,
				(*next_carrier)->current_pos_in_digraph.head_node,
				(*next_carrier)->current_pos_in_digraph.offset,
				param->vanet_table.dst_vnode->current_pos_in_digraph.tail_node,			
   				param->vanet_table.dst_vnode->current_pos_in_digraph.head_node,
   				param->vanet_table.dst_vnode->current_pos_in_digraph.offset);
#endif /* ] */
	}

	return result;
}

boolean TPD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_For_Greedy_Routing(
		parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		char *tail_node_for_next_forwarding_edge, 
		char *head_node_for_next_forwarding_edge, 
		directional_edge_type_t edge_type, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier)
{ //determine whether to be able to forward its packets to next carrier with the highest EDR, moving on the road segment incident to an intersection corresponding to tail_node_for_next_edge and return the pointer to the next carrier through *next_carrier
	boolean result = FALSE;
	boolean flag = FALSE; //flag to indicate whether a next carrier is determined or not
	double distance = 0; //distance between vehicle and another vehicle moving on the same directional edge
	//double max_neighbor_EDR = 0; //neighbor vehicle with the maximum EDR
	double min_neighbor_EDD = INF; //neighbor vehicle with the minimum EDD
	char *tail_node = tail_node_for_next_forwarding_edge; //tail node of the next directional edge for forwarding
	char *head_node = head_node_for_next_forwarding_edge; //head node of the next directional edge for forwarding
	directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
	vehicle_movement_queue_node_t *pMoveNode = NULL; //pointer to the vehicle movement queue
	int i = 0; //index for for-loop
	int size = 0; //size of vehicle movement queue
	double EDR = 0; //EDR

	/** reset *next_carrier to NULL */
	*next_carrier = NULL;

	/** obtain the pointer to the direaction edge of <tail_node,head_node> */
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);

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
	for(i = 0; i < size && flag == FALSE; i++) //for-1
	{
		pMoveNode = pMoveNode->next;
		if(vehicle->id == pMoveNode->vid)
			continue;

		if(edge_type == OUTGOING_EDGE) //for the outgoing edge of the intersection (i.e., tail_node)
			distance = pMoveNode->offset;
		else //for the incoming edge of the intersection (i.e., head_node)
			distance = fabs(pEdgeNode->weight - pMoveNode->offset);

		/* check the distance between two vehicles */
		if(distance > param->communication_range)
		{
			continue;
		}

		/** Next carrier selection rule: We select a vehicle with the highest EDR as
		 * a next carrier */ 
#if TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FOR_INTERSECTION_ROAD_SEGMENT_FLAG /* [ */		
		EDR = TPD_Compute_EDR_and_EDD(current_time, 
				param,
				pMoveNode->vnode,
				param->vanet_table.dst_vnode,
				TRUE);
#else
		EDR = TPD_Compute_EDR_and_EDD(current_time, 
				param,
				pMoveNode->vnode,
				param->vanet_table.dst_vnode,
				FALSE);
#endif
		if(EDR > 0)
		{
#if 0 /* [ */
			printf("***%s:%d [%0.f] EDR(%0.3f) of vehicle(%d) is positive.***\n",
					__FUNCTION__, __LINE__,
					(float)current_time,
					EDR, pMoveNode->vnode->id);
#endif /* ] */
		}
		else
		{
			continue;
		}

		/* select a next vehicle that has a higher EDR */
		switch(param->vehicle_vanet_forwarding_type) //switch-1
		{
			case VANET_FORWARDING_BASED_ON_VEHICLE:
			case VANET_FORWARDING_BASED_ON_CONVOY:
				if((pMoveNode->vnode->EDR_for_V2V >= param->tpd_delivery_probability_threshold) && (pMoveNode->vnode->EDD_for_V2V < min_neighbor_EDD))
				//if(pMoveNode->vnode->EDR_for_V2V > max_neighbor_EDR)
				{ //we also check vehicles' EDRs
					min_neighbor_EDD = pMoveNode->vnode->EDD_for_V2V;
					//max_neighbor_EDR = pMoveNode->vnode->EDR_for_V2V;
					*next_carrier = pMoveNode->vnode; //update the pointer of neighbor vehicle node with higher EDR
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
	{
		result = TRUE;
	}

	return result;
}

boolean TPD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_For_Source_Routing(
		parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		char *tail_node_for_next_forwarding_edge, 
		char *head_node_for_next_forwarding_edge, 
		directional_edge_type_t edge_type, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier)
{ //determine whether to be able to forward its packets to next carrier (as a child vehicle of vehicle in vehicle's predicted encounter graph), moving on the road segment incident to an intersection corresponding to tail_node_for_next_edge and return the pointer to the next carrier through *next_carrier
	boolean result = FALSE;
	boolean flag = FALSE; //flag to indicate whether a next carrier is determined or not
	double distance = 0; //distance between vehicle and another vehicle moving on the same directional edge
	char *tail_node = tail_node_for_next_forwarding_edge; //tail node of the next directional edge for forwarding
	char *head_node = head_node_for_next_forwarding_edge; //head node of the next directional edge for forwarding
	directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
	vehicle_movement_queue_node_t *pMoveNode = NULL; //pointer to the vehicle movement queue
	int i = 0; //index for for-loop
	int size = 0; //size of vehicle movement queue

	/** reset *next_carrier to NULL */
	*next_carrier = NULL;

	/** obtain the pointer to the direaction edge of <tail_node,head_node> */
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);

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
	for(i = 0; i < size && flag == FALSE; i++) //for-1
	{
		pMoveNode = pMoveNode->next;
		if(vehicle->id == pMoveNode->vid)
			continue;

		if(edge_type == OUTGOING_EDGE) //for the outgoing edge of the intersection (i.e., tail_node)
			distance = pMoveNode->offset;
		else //for the incoming edge of the intersection (i.e., head_node)
			distance = fabs(pEdgeNode->weight - pMoveNode->offset);

		/* check the distance between two vehicles */
		if(distance > param->communication_range)
		{
			continue;
		}

#if 0 /* [ */
		if(current_time > 9684 && vehicle->id == 74 && pMoveNode->vnode->id == 60)
		{
			printf("%s:%d [%0.f] current_vehicle(%d) is checked to encounter next_carrier_vehicle(%d)\n",
					__FUNCTION__, __LINE__,
					current_time, vehicle->id, pMoveNode->vnode->id);
		}
#endif /* ] */

		/** Next carrier selection rule: We select one of the current carrier's 
		 * child vehicles as the next carrier. */
		flag = TPD_Check_Child_Vehicle_In_Encounter_Graph(current_time, vehicle, pMoveNode->vnode); //check whether pMoveNode->vnode is one of vehicle's child vehicles in vehicle's encounter graph or not
		if(flag == FALSE)
		{
			continue;
		}
		else
		{
			*next_carrier = pMoveNode->vnode;
			result = TRUE;
			break;
		}
	} //end of for-1

	/** check whether a next carrier vehicle exists */
	if(flag)
	{
#if 1 /* [*/
		printf("###%s:%d [%0.f] current_carrier(%d) in (%s->%s: %0.f) encounters next_carrier(%d) in (%s->%s: %0.f) for destination_vehicle(%s->%s: %0.f).###\n",
				__FUNCTION__, __LINE__,
				(float)current_time,
				vehicle->id,
				vehicle->current_pos_in_digraph.tail_node,
				vehicle->current_pos_in_digraph.head_node,
				vehicle->current_pos_in_digraph.offset,
				(*next_carrier)->id,
				(*next_carrier)->current_pos_in_digraph.tail_node,
				(*next_carrier)->current_pos_in_digraph.head_node,
				(*next_carrier)->current_pos_in_digraph.offset,
				param->vanet_table.dst_vnode->current_pos_in_digraph.tail_node,			
   				param->vanet_table.dst_vnode->current_pos_in_digraph.head_node,
   				param->vanet_table.dst_vnode->current_pos_in_digraph.offset);
#endif /* ] */
	}

	return result;
}

int TPD_Perform_BFS_For_Encounter_Graph(adjacency_list_queue_t *G, boolean display_flag)
{ /* perform the Breadth-First-Search (BFS) for the given encounter graph G 
	from source vehicle to destination vehicle in G; trajectory_flag is used to determine
	whether to show the trajectory of each vehicle in G */ 
	adjacency_list_pointer_queue_t Q; //queue for adjacency list pointers
	adjacency_list_pointer_queue_node_t qnode; //queue node for the pointer to a graph node
	adjacency_list_pointer_queue_node_t *p = NULL; //pointer to the pointer to a graph node
	adjacency_list_pointer_queue_node_t *pNewNode = NULL; //pointer to the pointer to a graph node
	adjacency_list_queue_node_t *pQueueNode = NULL; //a pointer to a graph node
	neighbor_list_queue_node_t *q = NULL; //pointer to a child queue node
	parent_list_queue_node_t *u = NULL; //pointer to a parent queue node
	int i = 0; //index
	struct_vehicle_t *vehicle = NULL; //pointer to a vehicle
	struct_vehicle_t *parent_vehicle = NULL; //pointer to a parent vehicle
	struct_vehicle_t *dst_vehicle = NULL; //pointer to the destination vehicle
	double current_time = smpl_time(); //current simultation time
    
	//printf("TPD_Perform_BFS_For_Encounter_Graph\n");

	/** initialize Q */
	InitQueue((queue_t*)&Q, QTYPE_ADJACENCY_LIST_POINTER);

	/** perform Breadth-First-Search (BFS) for the graph G from root src_vehicle_gnode 
	 * toward leaf dst_vehicle_gnode in order to display the trajectories of vehicles in G */
	memset(&qnode, 0, sizeof(qnode));

	/* check the validity of pointers in G */
	if(G->src_vehicle_gnode == NULL)
	{
		printf("%s:%d G->src_vehicle_gnode is NULL\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}
	else if(G->dst_vehicle_gnode == NULL)
	{
		printf("%s:%d G->dst_vehicle_gnode is NULL\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}

	qnode.graph_qnode = G->src_vehicle_gnode;
	vehicle = (struct_vehicle_t*)G->src_vehicle_gnode->object;
	dst_vehicle = (struct_vehicle_t*)G->dst_vehicle_gnode->object;

	pNewNode = (adjacency_list_pointer_queue_node_t*)Enqueue((queue_t*)&Q, (queue_node_t*)&qnode);
	pNewNode->graph_qnode->queueing_flag = TRUE; //set queueing_flag to TRUE

	if(display_flag)
	{
		/* print the predicted encounter graph for vehicle */
		printf("\n<<Predicted Encounter Graph for vehicle(%d) at t=%.0f>>\n",
				vehicle->id, (float)current_time);
	}

	do //do-while-1
	{
		if(Q.size == 0)
		{
			break;
		}
		
		p = (adjacency_list_pointer_queue_node_t*)Dequeue((queue_t*)&Q);
		pQueueNode = p->graph_qnode;

		/* print the vehicle and its trajectory */
		vehicle = (struct_vehicle_t*)pQueueNode->object;
		parent_vehicle = (struct_vehicle_t*)pQueueNode->parent_list.head.next->object;

		if(parent_vehicle == NULL)
		{
			if(display_flag)
			{
				printf("\n(vehicle, parent)=(%d, -) with (t=%0.f, p=%0.3f, EDR=%0.3f, EDD=%0.f): pos=<%s, %s: %0.f>\n",
						vehicle->id, 
						current_time,
						pQueueNode->parent_list.head.next->P_encounter,
						pQueueNode->EDR,
						pQueueNode->EDD,
						vehicle->current_pos_in_digraph.tail_node,
						vehicle->current_pos_in_digraph.head_node,
						vehicle->current_pos_in_digraph.offset);

				TPD_Print_Vehicle_Trajectory(vehicle);	
			}
		}
		else
		{
			if(vehicle->id != dst_vehicle->id)
			{
				if(display_flag)
				{
					printf("\n(vehicle, parent)=(%d, %d) with (t=%0.f, p=%0.3f) and <%d, %d: %0.f>: pos=<%s, %s: %0.f>\n",
							vehicle->id, 
							parent_vehicle->id,
							pQueueNode->parent_list.head.next->T_encounter,
							pQueueNode->parent_list.head.next->P_encounter,
							pQueueNode->parent_list.head.next->tail_vertex,
							pQueueNode->parent_list.head.next->head_vertex,
							pQueueNode->parent_list.head.next->edge_offset,
							vehicle->current_pos_in_digraph.tail_node,
							vehicle->current_pos_in_digraph.head_node,
							vehicle->current_pos_in_digraph.offset); 
					/* Note: vehicle (i.e., child) will encounter parent_vehicle (i.e., parent)
					 * at the edge of (tail_vertex, head_vertex: edge_offset) at time T_encounter
					 * with probability P_encounter. */ 

					TPD_Print_Vehicle_Trajectory(vehicle);	
				}
			}
			else
			{
				u = &(pQueueNode->parent_list.head);
				for(i = 0; i < pQueueNode->parent_list.size; i++)
				{
					u = u->next;
					parent_vehicle = (struct_vehicle_t*)u->graph_qnode->object;

					if(display_flag)
					{
						printf("\n(vehicle, parent)=(%d, %d) with (t=%0.f, p=%0.3f) and <%d, %d: %0.f>: pos=<%s, %s: %0.f>\n",
								vehicle->id, 
								parent_vehicle->id,
								u->T_encounter,
								u->P_encounter,
								u->tail_vertex,
								u->head_vertex,
								u->edge_offset,
								vehicle->current_pos_in_digraph.tail_node,
								vehicle->current_pos_in_digraph.head_node,
								vehicle->current_pos_in_digraph.offset);
						/* Note: vehicle (i.e., child) will encounter parent_vehicle (i.e., parent)
						 * at the edge of (tail_vertex, head_vertex: edge_offset) at time T_encounter
						 * with probability P_encounter. */ 

						TPD_Print_Vehicle_Trajectory(vehicle);	
					}
				}
			}
		}

		/* enqueue the  nodes for p into Q for Breadth First Search (BFS) */
		q = &(pQueueNode->neighbor_list.head);
		for(i = 0; i < pQueueNode->neighbor_list.size; i++)
		{
			q = q->next;

			/* check whether the pointer to q->graph_qnode is already queued in Q or not */
			if(q->graph_qnode->queueing_flag == FALSE)
			{
				memset(&qnode, 0, sizeof(qnode));
				qnode.graph_qnode = q->graph_qnode;
				pNewNode = (adjacency_list_pointer_queue_node_t*)Enqueue((queue_t*)&Q, (queue_node_t*)&qnode);
				pNewNode->graph_qnode->queueing_flag = TRUE; //set queueing_flag to TRUE
			}
		}

		/* delete p because the memory for p must be returned */                                                  		/* compute EDR for the vehicle for p */
		DestroyQueueNode(Q.type, (queue_node_t*)p);
	} while(1);

	/* print the trajectory of the destination vehicle */
	if(display_flag)
	{
		printf("\n##(dst_vehicle)=(%d) with (t=%0.f): pos=<%s, %s: %0.f>##\n",
				dst_vehicle->id, 
				current_time,
				dst_vehicle->current_pos_in_digraph.tail_node,
            	dst_vehicle->current_pos_in_digraph.head_node,
				dst_vehicle->current_pos_in_digraph.offset);

		TPD_Print_Vehicle_Trajectory(dst_vehicle);	
	}

	/** destroy Q */
	DestroyQueue((queue_t*)&Q);

	return 0;
}

int TPD_Print_Vehicle_Trajectory(struct_vehicle_t *vehicle)
{ //print vehicle's trajectoy (i.e., path_list from the current tail node to the last node) along with arrival time
	struct_path_node *path_ptr = NULL; //pointer to the tail vertex of an edge along the path of vehicle
	int tail = 0, head = 0; //the tail vertex and head vertex of the edge visited by vehicle
	double T_tail = 0, T_head = 0; //the average arrival times for the tail vertex and head vertex of the edge visited by vehicle
	double length = 0; //edge length
	double prev_edge_delay = 0; //travel delay for the previous edge

	//return 0;
	for(path_ptr = vehicle->path_ptr; path_ptr != vehicle->path_list->prev;)
	{
		tail = atoi(path_ptr->vertex);
		head = atoi(path_ptr->next->vertex);
		T_tail = path_ptr->expected_arrival_time;
		T_head = path_ptr->next->expected_arrival_time;					
		length = path_ptr->next->weight;
		printf("->[%d: t=%0.f, d=%0.f]", tail, T_tail, prev_edge_delay);

		prev_edge_delay = T_head - T_tail;
		path_ptr = path_ptr->next;
	}

	printf("->[%d: t=%0.f, d=%0.f]", head, T_head, prev_edge_delay);
	printf("\n");
	return 0;
}
void TPD_Print_Vehicle_Forward_Area_Compared(struct_vehicle_t *comparision_vehicle,struct_vehicle_t *current_vehicle)
{
	struct_path_node *path_ptr1 = NULL; //pointer to the tail vertex of an edge along the path of comparision_vehicle
	struct_path_node *path_ptr2 = NULL; //pointer to the tail vertex of an edge along the path of current_vehicle
	int tail_1 = 0, head_1 = 0; //the tail vertex and head vertex of the edge visited by comparision_vehicle
	int tail_2 = 0, head_2 = 0; //the tail vertex and head vertex of the edge visited by comparision_vehicle

	for (path_ptr1 = comparision_vehicle->path_ptr; path_ptr1 != comparision_vehicle->path_list->prev;)
	{
		tail_1 = atoi(path_ptr1->vertex);
		head_1 = atoi(path_ptr1->next->vertex);
		for (path_ptr2 = current_vehicle->path_ptr; path_ptr2 != current_vehicle->path_list->prev;)
		{
			tail_2 = atoi(path_ptr2->vertex);
			head_2 = atoi(path_ptr2->next->vertex);
			if ((tail_1 == tail_2) && (head_1 == head_2))
			{			
				printf("jinyong Vehicle:[%d, %d] TPD Vehicle_Forward_Area %d->%d \n", current_vehicle->id, comparision_vehicle->id, tail_1, head_1);
			}
			path_ptr2 = path_ptr2->next;
		}
		path_ptr1 = path_ptr1->next;
	}
}
void TPD_Print_Vehicle_Encounter_Area_Compared(struct_vehicle_t *comparision_vehicle, struct_vehicle_t *current_vehicle)
{
	struct_path_node *path_ptr1 = NULL; //pointer to the tail vertex of an edge along the path of comparision_vehicle
	struct_path_node *path_ptr2 = NULL; //pointer to the tail vertex of an edge along the path of current_vehicle
	int tail_1 = 0, head_1 = 0; //the tail vertex and head vertex of the edge visited by comparision_vehicle
	int tail_2 = 0, head_2 = 0; //the tail vertex and head vertex of the edge visited by comparision_vehicle
	for (path_ptr1 = comparision_vehicle->path_ptr; path_ptr1 != comparision_vehicle->path_list->prev;)
	{
		tail_1 = atoi(path_ptr1->vertex);
		head_1 = atoi(path_ptr1->next->vertex);
		for (path_ptr2 = current_vehicle->path_ptr; path_ptr2 != current_vehicle->path_list->prev;)
		{
			tail_2 = atoi(path_ptr2->vertex);
			head_2 = atoi(path_ptr2->next->vertex);
			if ((tail_1 == head_2) && (head_1 == tail_2))
			{
				printf("jinyong Vehicle:[%d, %d] TPD Vehicle_Encounter_Area %d->%d \n", current_vehicle->id, comparision_vehicle->id, tail_2, head_2);
			}
			path_ptr2 = path_ptr2->next;
		}
		path_ptr1 = path_ptr1->next;
	}




}

boolean TPD_Is_There_Next_Carrier_On_Road_Segment(parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier)
{ //determine whether to forward its packets to next carrier with a better EDR, moving on the current road segment and return the pointer to the next carrier through *next_carrier. 
	boolean result = FALSE;

	/** select a data forwarding method according to two-way forwarding flag */
	if(param->data_forwarding_two_way_forwarding_flag)
	{ /* Data forwarding for two-way road segment */
		if(param->tpd_encounter_graph_source_routing_flag)
		{
			result = TPD_Is_There_Next_Carrier_On_Two_Way_Road_Segment_For_Source_Routing(param, current_time, vehicle, G, G_size, next_carrier);
		}
		else
		{
			result = TPD_Is_There_Next_Carrier_On_Two_Way_Road_Segment_For_Greedy_Routing(param, current_time, vehicle, G, G_size, next_carrier);
		}
	}
	else
	{ /* Data forwarding for one-way road segment */
		if(param->tpd_encounter_graph_source_routing_flag)
		{
			result = TPD_Is_There_Next_Carrier_On_One_Way_Road_Segment_For_Source_Routing(param, current_time, vehicle, G, G_size, next_carrier);
		}
		else
		{
			result = TPD_Is_There_Next_Carrier_On_One_Way_Road_Segment_For_Greedy_Routing(param, current_time, vehicle, G, G_size, next_carrier);
		}
	}

	return result;
}

boolean TPD_Is_There_Next_Carrier_On_One_Way_Road_Segment_For_Greedy_Routing(parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier)
{ //For one-way road segment, determine whether to forward its packets to next carrier with a better EDR, moving on the current road segment and return the pointer to the next carrier through *next_carrier.
	boolean result = FALSE;
	boolean flag = FALSE; //flag to indicate whether a next carrier is determined or not
	double distance = 0; //distance between vehicle and another vehicle moving on the same directional edge
	//double max_neighbor_EDR = 0; //neighbor vehicle with the maximum EDR
	double min_neighbor_EDD = INF; //neighbor vehicle with the minimum EDD
	char *tail_node = vehicle->path_ptr->vertex; //tail node of the directional edge where vehicle is moving
	char *head_node = vehicle->path_ptr->next->vertex; //head node of the directional edge where vehicle is moving
	directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directional edge of <tail_node,head_node>
	vehicle_movement_queue_node_t *pMoveNode = NULL; //pointer to the vehicle movement queue
	int i = 0; //index for for-loop
	int size = 0; //size of vehicle movement queue
	double offset_in_undirectional_edge = 0; //vehicle's offset on the undirectional edge
	double offset_in_directional_edge = 0; //vehicle's offset on the directional edge
	double EDR = 0; //EDR (Expected Delivery Ratio)

	/** reset *next_carrier to NULL */
	*next_carrier = NULL;

	/**@ Search the next carrier on the directional edge where vehicle is moving **/

	/** obtain the pointer to the directional edge of <tail_node,head_node> */
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
	if(pEdgeNode == NULL)
	{
		printf("%s:%d pEdgeNode for <%s,%s> is NULL\n", 
				__FUNCTION__, __LINE__,
				tail_node, head_node);
		exit(1);
	}

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
		printf("%s:%d vehicle->move_type(%d) is invalid\n", 
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

		/* check the distance between two vehicles */
		if(distance > param->communication_range)
		{
			continue;
		}

		/** Next carrier selection rule: We select a vehicle with the highest EDR as
		 * a next carrier */ 
#if TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FOR_ONEWAY_ROAD_SEGMENT_FLAG /* [ */
		EDR = TPD_Compute_EDR_and_EDD(current_time, 
				param,
				pMoveNode->vnode,
				param->vanet_table.dst_vnode,
				TRUE);
#else
		EDR = TPD_Compute_EDR_and_EDD(current_time, 
				param,
				pMoveNode->vnode,
				param->vanet_table.dst_vnode,
				FALSE);
#endif
		if(EDR > 0)
		{
#if 0 /* [ */
			printf("***%s:%d EDR(%0.3f) of vehicle(%d) is positive.***\n",
					__FUNCTION__, __LINE__,
					EDR, pMoveNode->vnode->id);
#endif /* ] */
		}
		else
		{
			continue;
		}

	    /* choose a next carrier with smaller EDD according to forwording type */
		switch(param->vehicle_vanet_forwarding_type) //switch-1
	    {
			case VANET_FORWARDING_BASED_ON_VEHICLE:
			case VANET_FORWARDING_BASED_ON_CONVOY:
				if((pMoveNode->vnode->EDR_for_V2V >= param->tpd_delivery_probability_threshold) && (pMoveNode->vnode->EDD_for_V2V < min_neighbor_EDD))
				//if(pMoveNode->vnode->EDR_for_V2V > max_neighbor_EDR)
				{ //we also check vehicles' EDRs
					min_neighbor_EDD = pMoveNode->vnode->EDD_for_V2V;
					//max_neighbor_EDR = pMoveNode->vnode->EDR_for_V2V;
					*next_carrier = pMoveNode->vnode; //update the pointer of neighbor vehicle node with higher EDR 
				}
				break;

			default:
				printf("%s:%d vanet forwarding type (%d) is not known!\n", 
						__FUNCTION__, __LINE__,
						param->vehicle_vanet_forwarding_type);
				exit(1);
		} //end of switch-1
	} //end of for-1
   
	/** check whether a next carrier vehicle exists */
	if(*next_carrier != NULL)
	{
		/* check whether next_carrier's EDR is greater than the current packet carrier (vehicle)'s EDR */
#if TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FOR_ONEWAY_ROAD_SEGMENT_FLAG /* [ */
		TPD_Compute_EDR_and_EDD(current_time, 
				param,
				vehicle,
				param->vanet_table.dst_vnode,
				TRUE); //recompute vehicle's EDR and EDD
#else
		TPD_Compute_EDR_and_EDD(current_time,
				param,
				vehicle,
				param->vanet_table.dst_vnode,
				FALSE); //recompute vehicle's EDR and EDD
#endif /* ] */
		if((*next_carrier)->EDD_for_V2V < vehicle->EDD_for_V2V)
		//if((*next_carrier)->EDR_for_V2V > vehicle->EDR_for_V2V)
		{
			result = TRUE;
#if 1 /* [*/
			printf("###%s:%d EDR(%0.3f) and EDD(%0.f) of next_carrier(%d) is better than EDR(%0.3f) and EDD(%0.f) of current_carrier(%d).###\n",
					__FUNCTION__, __LINE__,
					(*next_carrier)->EDR_for_V2V, 
					(*next_carrier)->EDD_for_V2V,
					(*next_carrier)->id,
					vehicle->EDR_for_V2V, 
					vehicle->EDD_for_V2V, 				
					vehicle->id);
#endif /* ] */
		}
		else
		{
			result = FALSE;
		}
	}

	return result;
}

boolean TPD_Is_There_Next_Carrier_On_One_Way_Road_Segment_For_Source_Routing(parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier)
{ //For one-way road segment, determine whether to forward its packets to next carrier in the predicted encounter graph for source routing and return the pointer to the next carrier through *next_carrier.
	boolean result = FALSE;

	/** reset *next_carrier to NULL */
	*next_carrier = NULL;

	/* TBD */
	printf("%s:%d To Be Done!\n",
			__FUNCTION__, __LINE__);

	return result;
}

boolean TPD_Is_There_Next_Carrier_On_Two_Way_Road_Segment_For_Greedy_Routing(parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier)
{ //For two-way road segment, determine whether to forward its packets to next carrier with a better EDR, moving on the current road segment and return the pointer to the next carrier through *next_carrier.
	boolean result = FALSE;
	boolean flag = FALSE; //flag to indicate whether a next carrier is determined or not
	double distance = 0; //distance between vehicle and another vehicle moving on the same directional edge
	//double max_neighbor_EDR = 0; //neighbor vehicle with the maximum EDR
	double min_neighbor_EDD = INF; //neighbor vehicle with the minimum EDD
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
	double EDR = 0; //EDR (Expected Delivery Ratio)

	/** reset *next_carrier to NULL */
	*next_carrier = NULL;

	/**@ Search the next carrier on the directional edge where vehicle is moving **/

	/** obtain the pointer to the directional edge of <tail_node,head_node> */
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
	if(pEdgeNode == NULL)
	{
		printf("%s:%d pEdgeNode for <%s,%s> is NULL\n", 
				__FUNCTION__, __LINE__,
				tail_node, head_node);
		exit(1);
	}

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
		printf("%s:%d vehicle->move_type(%d) is invalid\n", 
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

		/* check the distance between two vehicles */
		if(distance > param->communication_range)
		{
			continue;
		}

		/** Next carrier selection rule: We select a vehicle with the highest EDR as
		 * a next carrier */
#if TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FOR_TWOWAY_ROAD_SEGMENT_FLAG /* [ */
		EDR = TPD_Compute_EDR_and_EDD(current_time, 
				param,
				pMoveNode->vnode,
				param->vanet_table.dst_vnode,
				TRUE);
#else
		EDR = TPD_Compute_EDR_and_EDD(current_time, 
				param,
				pMoveNode->vnode,
				param->vanet_table.dst_vnode,
				FALSE);
#endif /* ] */
		if(EDR > 0)
		{
#if 0 /* [ */
			printf("***%s:%d EDR(%0.3f) of vehicle(%d) is positive.***\n",
					__FUNCTION__, __LINE__,
					EDR, pMoveNode->vnode->id);
#endif /* ] */
		}
		else
		{
			continue;
		}

		/* choose a next carrier with smaller EDD according to forwording type */
		switch(param->vehicle_vanet_forwarding_type) //switch-1
		{
			case VANET_FORWARDING_BASED_ON_VEHICLE:
			case VANET_FORWARDING_BASED_ON_CONVOY:
				if((pMoveNode->vnode->EDR_for_V2V >= param->tpd_delivery_probability_threshold) && (pMoveNode->vnode->EDD_for_V2V < min_neighbor_EDD))
				//if(pMoveNode->vnode->EDR_for_V2V > max_neighbor_EDR)
				{ //we also check vehicles' EDRs
					min_neighbor_EDD = pMoveNode->vnode->EDD_for_V2V;
					//max_neighbor_EDR = pMoveNode->vnode->EDR_for_V2V;
					next_carrier_candidate_1 = pMoveNode->vnode; //update the pointer of neighbor vehicle node with higher EDR
				}
				break;

			default:
				printf("%s:%d vanet forwarding type (%d) is not known!\n", 
						__FUNCTION__, __LINE__,
						param->vehicle_vanet_forwarding_type);
				exit(1);
		} //end of switch-1
	} //end of for-1
 
	/******************************************************************************/

	/**@ Search the next carrier on the opposite directional edge of the directional edge where vehicle is moving **/
	/* initialize the variables for the searching of next carrier */
	//max_neighbor_EDR = 0; //neighbor vehicle with the maximum EDR
	min_neighbor_EDD = INF; //neighbor vehicle with the minimum EDD
	flag = FALSE; //flag is set to FALSE to indicate that a next carrier is determined

	/** obtain the pointer to the directional edge of <head_node,tail_node> */
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, head_node, tail_node);
	if(pEdgeNode == NULL)
	{
		printf("%s:%d pEdgeNode for <%s,%s> is NULL\n", 
				__FUNCTION__, __LINE__,
				head_node, tail_node);
		exit(1);
	}

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
		printf("%s:%d vehicle->move_type(%d) is invalid\n", 
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

		/* check the distance between two vehicles */
		if(distance > param->communication_range)
		{
			continue;
		}

		/** Next carrier selection rule: We select a vehicle with the highest EDR as
		 * a next carrier */
#if TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FOR_TWOWAY_ROAD_SEGMENT_FLAG /* [ */
		EDR = TPD_Compute_EDR_and_EDD(current_time, 
				param,
				pMoveNode->vnode,
				param->vanet_table.dst_vnode,
				TRUE);
#else
		EDR = TPD_Compute_EDR_and_EDD(current_time,
				param,
				pMoveNode->vnode,
				param->vanet_table.dst_vnode,
				FALSE);
#endif /* ] */
		if(EDR > 0)
		{
#if 0 /* [ */
			printf("***%s:%d [%0.f] EDR(%0.3f) of vehicle(%d) is positive.***\n",
					__FUNCTION__, __LINE__,
					(float)current_time,
					EDR, pMoveNode->vnode->id);
#endif /* ] */
		}
		else
		{
			continue;
		}

		/* choose a next carrier with smaller EDD according to forwording type */
		switch(param->vehicle_vanet_forwarding_type) //switch-1
		{
			case VANET_FORWARDING_BASED_ON_VEHICLE:
			case VANET_FORWARDING_BASED_ON_CONVOY:
				if((pMoveNode->vnode->EDR_for_V2V >= param->tpd_delivery_probability_threshold) &&
						(pMoveNode->vnode->EDD_for_V2V < min_neighbor_EDD))
				//if(pMoveNode->vnode->EDR_for_V2V > max_neighbor_EDR)
				{ //we also check vehicles' EDDs
					min_neighbor_EDD = pMoveNode->vnode->EDD_for_V2V;	
					//max_neighbor_EDR = pMoveNode->vnode->EDR_for_V2V;
					next_carrier_candidate_2 = pMoveNode->vnode; //update the pointer of neighbor vehicle node with higher EDR 
				}
				break;

			default:
				printf("%s:%d vanet forwarding type (%d) is not known!\n", 
						__FUNCTION__, __LINE__,
						param->vehicle_vanet_forwarding_type);
				exit(1);
		} //end of switch-1
	} //end of for-1
   
	/******************************************************************************/
  
	/** select the best carrier between next_carrier_candidate_1 and next_carrier_candidate_2 */
	if((next_carrier_candidate_1 != NULL) && (next_carrier_candidate_2 != NULL))
	{
		if(next_carrier_candidate_1->EDD_for_V2V <= next_carrier_candidate_2->EDD_for_V2V)
		//if(next_carrier_candidate_1->EDR_for_V2V >= next_carrier_candidate_2->EDR_for_V2V)
		{
			*next_carrier = next_carrier_candidate_1;
		}
		else
		{
			*next_carrier = next_carrier_candidate_2;
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

	/** check whether a next carrier vehicle exists */
	if(*next_carrier != NULL)
	{
		/* check whether next_carrier's EDR is greater than the current packet carrier (vehicle)'s EDR */
#if TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FOR_TWOWAY_ROAD_SEGMENT_FLAG /* [ */
		TPD_Compute_EDR_and_EDD(current_time, 
				param,
				vehicle,
				param->vanet_table.dst_vnode,
				TRUE); //recompute vehicle's EDR??
#else
		TPD_Compute_EDR_and_EDD(current_time, 
				param,
				vehicle,
				param->vanet_table.dst_vnode,
				FALSE); //recompute vehicle's EDR??
#endif /* ] */
		if((*next_carrier)->EDD_for_V2V < vehicle->EDD_for_V2V)
		//if((*next_carrier)->EDR_for_V2V > vehicle->EDR_for_V2V)
		{
			result = TRUE;
#if TPD_GREEDY_ROUTING_ROAD_SEGMENT_FORWARDING_TRACE_FLAG /* [*/
			printf("<R> %s:%d [%0.f] EDR(%0.3f) and EDD(%0.f) of next_carrier(%d) is better than EDR(%0.3f) and EDD(%0.f) of current_carrier(%d): current_carrier(%s->%s: %0.f), next_carrier(%s->%s: %0.f), destination_vehicle(%s->%s: %0.f).\n",
					__FUNCTION__, __LINE__,
					(float)current_time,
					(*next_carrier)->EDR_for_V2V, (*next_carrier)->EDD_for_V2V, (*next_carrier)->id,
					vehicle->EDR_for_V2V, vehicle->EDD_for_V2V, vehicle->id,
					vehicle->current_pos_in_digraph.tail_node,
					vehicle->current_pos_in_digraph.head_node,
					vehicle->current_pos_in_digraph.offset,
					(*next_carrier)->current_pos_in_digraph.tail_node,				
					(*next_carrier)->current_pos_in_digraph.head_node,				
					(*next_carrier)->current_pos_in_digraph.offset,   				
					param->vanet_table.dst_vnode->current_pos_in_digraph.tail_node,			
    				param->vanet_table.dst_vnode->current_pos_in_digraph.head_node,
    				param->vanet_table.dst_vnode->current_pos_in_digraph.offset);
#endif /* ] */
		}
		else
		{
			result = FALSE;
		}
	}

	return result;
}

boolean TPD_Is_There_Next_Carrier_On_Two_Way_Road_Segment_For_Source_Routing(parameter_t *param, 
		double current_time, 
		struct_vehicle_t *vehicle, 
		struct_graph_node *G, 
		int G_size, 
		struct_vehicle_t **next_carrier)
{ //For two-way road segment, determine whether to forward its packets to next carrier in the predicted encounter graph for source routing and return the pointer to the next carrier through *next_carrier.
	boolean result = FALSE;
	boolean flag = FALSE; //flag to indicate whether a next carrier is determined or not
	double distance = 0; //distance between vehicle and another vehicle moving on the same directional edge
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
	double EDR = 0; //EDR (Expected Delivery Ratio)

	/** reset *next_carrier to NULL */
	*next_carrier = NULL;

	/**@ Search the next carrier on the directional edge where vehicle is moving **/

	/** obtain the pointer to the directional edge of <tail_node,head_node> */
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
	if(pEdgeNode == NULL)
	{
		printf("%s:%d pEdgeNode for <%s,%s> is NULL\n", 
				__FUNCTION__, __LINE__,
				tail_node, head_node);
		exit(1);
	}

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
		printf("%s:%d vehicle->move_type(%d) is invalid\n", 
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

		/* check the distance between two vehicles */
		if(distance > param->communication_range)
		{
			continue;
		}

		/** Next carrier selection rule: We select one of the current carrier's 
		 * child vehicles as the next carrier. */
		flag = TPD_Check_Child_Vehicle_In_Encounter_Graph(current_time, vehicle, pMoveNode->vnode); //check whether pMoveNode->vnode is one of vehicle's child vehicles in vehicle's encounter graph or not
		if(flag == FALSE)
		{
			continue;
		}
		else
		{
			*next_carrier = pMoveNode->vnode;
			result = TRUE;
			break;
		}
	}

	if(flag)
	{
#if TPD_SOURCE_ROUTING_ROAD_SEGMENT_FORWARDING_TRACE_FLAG /* [*/
		printf("<R1> %s:%d [%0.f] current_carrier(%d) in (%s->%s: %0.f) encounters next_carrier(%d) in (%s->%s: %0.f) for destination_vehicle(%s->%s: %0.f).\n",
				__FUNCTION__, __LINE__,
				(float)current_time,
				vehicle->id,
				vehicle->current_pos_in_digraph.tail_node,
				vehicle->current_pos_in_digraph.head_node,
				vehicle->current_pos_in_digraph.offset,
				(*next_carrier)->id,
				(*next_carrier)->current_pos_in_digraph.tail_node,
				(*next_carrier)->current_pos_in_digraph.head_node,
				(*next_carrier)->current_pos_in_digraph.offset,
				param->vanet_table.dst_vnode->current_pos_in_digraph.tail_node,			
   				param->vanet_table.dst_vnode->current_pos_in_digraph.head_node,
   				param->vanet_table.dst_vnode->current_pos_in_digraph.offset);
#endif /* ] */
		return result;
	}

	/******************************************************************************/

	/**@ Search the next carrier on the opposite directional edge of the directional edge where vehicle is moving **/

	/** obtain the pointer to the directional edge of <head_node,tail_node> */
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, head_node, tail_node);
	if(pEdgeNode == NULL)
	{
		printf("%s:%d pEdgeNode for <%s,%s> is NULL\n", 
				__FUNCTION__, __LINE__,
				head_node, tail_node);
		exit(1);
	}

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
		printf("%s:%d vehicle->move_type(%d) is invalid\n", 
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

		/* check the distance between two vehicles */
		if(distance > param->communication_range)
		{
			continue;
		}

		/** Next carrier selection rule: We select one of the current carrier's 
		 * child vehicles as the next carrier. */
		flag = TPD_Check_Child_Vehicle_In_Encounter_Graph(current_time, vehicle, pMoveNode->vnode); //check whether pMoveNode->vnode is one of vehicle's child vehicles in vehicle's encounter graph or not
		if(flag == FALSE)
		{
			continue;
		}
		else
		{
			*next_carrier = pMoveNode->vnode;
			result = TRUE;
			break;
		}
	}

	if(flag)
	{
#if TPD_SOURCE_ROUTING_ROAD_SEGMENT_FORWARDING_TRACE_FLAG /* [*/
		printf("<R2> %s:%d [%0.f] current_carrier(%d) in (%s->%s: %0.f) encounters next_carrier(%d) in (%s->%s: %0.f) for destination_vehicle(%s->%s: %0.f).\n",
				__FUNCTION__, __LINE__,
				(float)current_time,
				vehicle->id,
				vehicle->current_pos_in_digraph.tail_node,
				vehicle->current_pos_in_digraph.head_node,
				vehicle->current_pos_in_digraph.offset,
				(*next_carrier)->id,
				(*next_carrier)->current_pos_in_digraph.tail_node,
				(*next_carrier)->current_pos_in_digraph.head_node,
				(*next_carrier)->current_pos_in_digraph.offset,
				param->vanet_table.dst_vnode->current_pos_in_digraph.tail_node,			
   				param->vanet_table.dst_vnode->current_pos_in_digraph.head_node,
   				param->vanet_table.dst_vnode->current_pos_in_digraph.offset);
#endif /* ] */
		return result;
	}

	return result;
}

boolean TPD_Check_Child_Vehicle_In_Encounter_Graph(double current_time, struct_vehicle_t *vehicle, struct_vehicle_t *candidate)
{//check whether candidate is one of vehicle's child vehicles in vehicle's encounter graph or not
	boolean result = FALSE;
	adjacency_list_queue_t *G = NULL; //pointer to adjacency list queue
	neighbor_list_queue_t *Q = NULL; //pointer to neighbor list queue
	neighbor_list_queue_node_t *ptr = NULL; //pointer to a neighbor list queue node
	int i = 0; //index

	/* check the validity of vehicle and candidate */
	if(vehicle == NULL)
	{
		printf("%s:%d [%0.f] vehicle is NULL\n",
				__FUNCTION__, __LINE__,
				current_time);
		exit(1);
	}
	else if(candidate == NULL)
	{
		printf("%s:%d [%0.f] candidate is NULL\n",
				__FUNCTION__, __LINE__,
				current_time);
		exit(1);
	}

	//G = &(vehicle->predicted_encounter_graph->G);
	/* check whether vehicle has packet(s) with an encounter graph */
	if(vehicle->packet_queue->size > 0)
	{
		if(vehicle->packet_queue->head.next->predicted_encounter_graph == NULL)
		{
			printf("%s:%d vehicle(%d) has NULL for vehicle->packet_queue->head.next->predicted_encounter_graph\n",
					__FUNCTION__, __LINE__,
					vehicle->id);
			exit(1);
		}

		G = &(vehicle->packet_queue->head.next->predicted_encounter_graph->G);
	}
	else
	{
		printf("%s:%d vehicle(%d) has no packet\n",
				__FUNCTION__, __LINE__,
				vehicle->id);
		exit(1);
	}

	/* check the validity of G->carrier_vehicle_gnode */
	if(G->carrier_vehicle_gnode == NULL)
	{
		printf("%s:%d [%0.f] vehicle(%d) has NULL value for its's G->carrier_vehicle_gnode\n",
				__FUNCTION__, __LINE__,
				current_time,
				vehicle->id);
		exit(1);                                		
	} 
	else if(G->carrier_vehicle_gnode->id != vehicle->id)
	{
		printf("%s:%d [%0.f] vehicle's id(%d) is different from G->carrier_vehicle_gnode's id(%d)\n",
				__FUNCTION__, __LINE__,
				current_time,
				vehicle->id, G->carrier_vehicle_gnode->id);
		exit(1);                                		
	}
	Q = &(G->carrier_vehicle_gnode->neighbor_list);
	ptr = &(Q->head);
	for(i = 0; i< Q->size; i++)
	{
		ptr = ptr->next;
		if(ptr->id == candidate->id)
		{
			//G->carrier_vehicle_gnode = ptr->graph_qnode;
			result = TRUE;
			break;
		}
	}

	return result;
}

adjacency_list_queue_node_t* TPD_Find_GraphNode_In_EncounterGraph(adjacency_list_queue_t *G,
		struct_vehicle_t *vehicle)
{ //find the pointer to a graph node in the encounter graph G corresponding to vehicle
	adjacency_list_queue_node_t *gnode = NULL; //pointer to a graph node in the encounter graph corresponding to vehicle

	if((G->bitmap[vehicle->id-1] == TRUE) && (G->bitmap_gnodes[vehicle->id-1] != NULL))
	{
		gnode = G->bitmap_gnodes[vehicle->id-1];
	}

	return gnode;
}







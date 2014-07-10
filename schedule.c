/**
 *  File: schedule.c
 *	Description: operations for scheduling for surveillance
 *	Date: 08/23/2007	
 *	Maker: Jaehoon Jeong
 */

#include "stdafx.h"
#include <stdlib.h> //calloc()
#include "schedule.h"
#include "queue.h"
#include "util.h"
#include "graph-data-struct.h"
#include "shortest-path.h"
#include "all-pairs-shortest-paths.h"
#include "mst.h"

void InitTable(schedule_table_t *T, struct_graph_node *Gv, int Gv_size, struct_graph_node *Gr, edge_queue_t *Er)
{ /* initialize table T by building the same number of entries as the number of edges in Gv
     and link T's entries with Er's subedge list entries each other */

	schedule_table_node_t node; //node for schedule table
	struct_graph_node *p = NULL, *q = NULL;//pointers to graph nodes
	char u[NAME_SIZE], v[NAME_SIZE]; //vertices
	int i, j;
	int neighbor_num; //number of neighbor vetices
	boolean flip_flag = FALSE; //flag to indicate if the order of vertices in node matches 
	//that of an entry in schedule table; in this function, this is not used.
	schedule_table_node_t *pTableNode = NULL; //pointer to a schedule table node
	edge_queue_node_t *pEdgeNode = NULL; //pointer to an edge queue node
	subedge_queue_node_t *pSubedgeNode = NULL; //pointer to an subedge queue node
	enum_edge_direction_t direction = EDGE_DIRECTION_FORWARD; //virtual edge direction for the physical edge direction

	assert_memory(T);

	memset(&(T->head), 0 ,sizeof(T->head));
	T->head.next = T->head.prev = &(T->head);
	T->size = 0;
	T->sequence_number = 0; //edge id
	
	/* set the virtual graph information */
	//T->Gv = G; //pointer to the virtual graph G
	//T->Gv_size = G_size; //size of the virtual graph G

	if(Gv_size == 0)
		return;
	
	for(i = 0; i < Gv_size; i++) //for-1
	{
		p = &(Gv[i]);
		strcpy(u, p->vertex);
		neighbor_num = (int)p->weight;
		q = p;
		for(j = 0; j < neighbor_num; j++) //for-2
		{
			q = q->next;
			strcpy(v, q->vertex);
			
			if(LookupTable(Gv, u, v, &flip_flag) == NULL) //if
			{
				memset(&node, 0, sizeof(node));
				//node.eid = eid++; //set the edge id
				//node.eid = ++(T->sequence_number); //set the edge id
				//node.eid = 0; //set the edge id //node.eid is set by Entable
				node.weight = q->weight; //set the edge's weight

				/*************************/
				//node.density = q->density; //set the sensor density
				pEdgeNode = FastLookupEdgeQueue(Gr, u, v, &flip_flag); //look up edge node from real graph Gr, not virtual graph Gv
				if(pEdgeNode == NULL)
				{
				  printf("InitTable(): pEdgeNode is NULL\n");
				  exit(1);
				}
				else
				{
				  node.density = pEdgeNode->density; //set the sensor density fom the edge queue node
				}
				/*************************/
				
				strcpy(node.tail_node, u);
				strcpy(node.head_node, v);
				node.direction = direction; //set the virtual edge's direction
				//Entable(T, &node); //enqueue the clone of node into T and increase T->size by 1.
				pTableNode = Entable(T, &node); //enqueue the clone of node into T and increase T->size by 1.
		                /* register the pointer to the table node in virtual graph Gv */
				RegisterTableNodeIntoVirtualGraph(Gv, Gv_size, pTableNode);
			} //end of if
		} //end of for-1
	} //end of for-2

	/* link T's entries with Er's subedge list entries each other */
	pTableNode = &(T->head);
	pEdgeNode = &(Er->head);
	for(i = 0; i < T->size; i++)
	{
		pTableNode = pTableNode->next;
		pEdgeNode = pEdgeNode->next;
		pSubedgeNode = pEdgeNode->subedge_list.head.next;

		/* let pSubedgeNode->schedule_table_entry point to pTableNode since pSubedgeNode->schedule_table_entry is not set up at ConstructEdgeQueue() */
		pSubedgeNode->schedule_table_entry = pTableNode;

		/* link pTableNode with pSubedgeNode in order to deal with initial sensing holes */
		pTableNode->subedge = pSubedgeNode;
		//pSubedgeNode->schedule_table_entry = pTableNode;
	}
}

void ConstructScheduleTable(schedule_table_t *T, parameter_t *param, struct_graph_node *G, int G_size, struct_traffic_table *src_table, struct_traffic_table *dst_table)
{ //construct schedule table T for scan-scheduling for intersection nodes in G with param and graph_file	
	//schedule_table_node_t table_node; //node for schedule table
	schedule_table_node_t *pTableNode = NULL; //pointer to schedule table node
	schedule_queue_t Q; //schedule queue
	schedule_queue_node_t queue_node; //node for schedule queue
	schedule_queue_node_t *pQueueNode = NULL; //pointer to schedule queue node
	char exit_node[NAME_SIZE]; //exit vertex
	char u[NAME_SIZE], v[NAME_SIZE]; //vertices
    struct_graph_node *graph_node = NULL, *pGraphNode = NULL; //pointers to graph nodes
	int neighbor_num = 0; //number of adjacent nodes
	int i; //index of for-loop
	boolean flip_flag = FALSE; //flag to indicate whether the vertices u and v for a directional edge 
	//are flipped in schedule table entry or not
	double edge_distance = 0; //edge distance
	double r = param->sensor_sensing_range; //sensing range (or sensing radius)
	double w = param->sensor_work_time; //minimum sensor work time for detection
	double v_s = 2*r/w; //scan_speed
	int n = 0; //number of sensors located at an edge
	
	if(dst_table->number <= 0)
	{
		printf("ConstructScheduleTable(); There is no exit in road network: that is, dst_table->number(%d) <= 0\n", dst_table->number);
#ifdef __DEBUG_INTERACTIVE_MODE__
                fgetc(stdin);
#endif
		exit(1);
	}

	/** reset all of the fields related to sensing scheduling, such as flags and arrival_times, departure_times */
	ResetScheduleTable(T);

	/** initialize schedule queue Q */
	InitQueue((queue_t *)&Q, QTYPE_SCHEDULE);

	/** initialize queue_node and enqueue it into Q */
	for(i = 0; i < dst_table->number; i++)
	{
		//strcpy(exit_node, dst_table->list[i]);
		strcpy(exit_node, dst_table->list[i].vertex);

		memset(&queue_node, 0, sizeof(queue_node));
		strcpy(queue_node.head_node, exit_node);
		Enqueue((queue_t *)&Q, (queue_node_t *)&queue_node);
	}

	/** perform the tree expansion for sensor scheduling using Breadth First Search (BFS) */
	while(SizeofQueue((queue_t *)&Q) > 0) //while
	{
		pQueueNode = (schedule_queue_node_t *)Dequeue((queue_t *)&Q);
		graph_node = LookupGraph(G, G_size, pQueueNode->head_node); 
		//look up graph G for the pointer to graph node corresponding to p->tail_node's name
		if(graph_node == NULL)
                {
		        printf("ConstructScheduleTable(): graph_node for pQueueNode->head_node(%s) is NULL\n", pQueueNode->head_node);
#ifdef __DEBUG_INTERACTIVE_MODE__
                        fgetc(stdin);
#endif
                        exit(1);
                }

		strcpy(u, pQueueNode->head_node); 
		//u becomes a tail node of directional edges incident to it

		neighbor_num = (int)graph_node->weight;
		pGraphNode = graph_node;
		for(i = 0; i < neighbor_num; i++) //for
		{
			pGraphNode = pGraphNode->next;
			edge_distance = pGraphNode->weight;
			strcpy(v, pGraphNode->vertex);
			//v becomes a head node of directional edges incident to node u

			/* check whether v is the pQueueNode's tail node or not; for the tail node, 
			   we don't need to expand our searching tree due to the direction of the scanning */
			if(strcmp(v, pQueueNode->tail_node) == 0)
				continue;
            
			/* initialize queue node */
			memset(&queue_node, 0, sizeof(queue_node));
			strcpy(queue_node.tail_node, u);
			strcpy(queue_node.head_node, v);
			queue_node.arrival_time = pQueueNode->departure_time;

			/**@For debugging */
			//if(strcmp(u, "146") == 0)
			//	printf("vertex 146 here!\n");
			/******************/

			/* compute the scan departure time for the edge by computing the scanning time 
			according to sensor_scan_type */
			switch(param->sensor_scan_type)
			{
			case SCAN_CONSTANT_SPEED_WITH_NONOPTIMAL_SLEEPING:
			case SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING:
			case SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
				queue_node.departure_time = queue_node.arrival_time + edge_distance/v_s;
				break;
				
			case SCAN_VARIABLE_SPEED_WITH_NONOPTIMAL_SLEEPING:
			case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING:
			case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
			case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING:
			case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING_AND_WITH_SENSING_HOLE_HANDLING:
			case SCAN_VARIABLE_SPEED_WITH_VARIABLE_SLEEPING_TIME_AND_WITH_SENSING_HOLE_HANDLING:
				 pTableNode = LookupTable(G, u, v, &flip_flag);
				//return the pointer to schedule table entry corresponding to the edge consisting of vertices u and v
				n = pTableNode->sensor_list.live_sensor_number;
				//n = pTableNode->sensor_list.size;
				queue_node.departure_time = queue_node.arrival_time + n*w;
				break;

			case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_VARIABLE_WORKING_TIME:
				printf("ConstructScheduleTable(): param->sensor_scan_type(%d) is not implemented yet\n", param->sensor_scan_type);
				break;
				
			default:
				printf("ConstructScheduleTable(): param->sensor_scan_type(%d) is not defined\n", param->sensor_scan_type);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                fgetc(stdin);
#endif
				exit(1);
			}

			/* determine the expansion of searching tree for queue_node with schedule table T */
			if(IsExpandable(G, &queue_node))
				Enqueue((queue_t *)&Q, (queue_node_t *)&queue_node); //enqueue queue_node into Q		
		} //end of for

		free(pQueueNode); //free the memory allocated to pQueueNode
	} //end of while
}

void ResetScheduleTable(schedule_table_t *T)
{ //reset all of the fields related to sensing scheduling, such as status, flags and arrival_times, departure_times
	int i; //index for for-loop
	schedule_table_node_t *pTableNode = NULL;

	/* reset all of the fields relevant to sensing scheduling */
	pTableNode = &(T->head);
	for(i = 0; i < T->size; i++)
	{
		pTableNode = pTableNode->next;
		pTableNode->status = STATUS_UNKNOWN;
		pTableNode->flag1 = FALSE;
		pTableNode->flag2 = FALSE;
		pTableNode->arrival_time1 = 0;
		pTableNode->arrival_time2 = 0;
		pTableNode->departure_time1 = 0;
		pTableNode->departure_time2 = 0;
	}
}


schedule_table_node_t* Entable(schedule_table_t *T, schedule_table_node_t *node)
{ //enqueue node into table T
	schedule_table_node_t *p = NULL;

	assert_memory(T);
	assert_memory(node);

	/* set up the eid with sequence number of schedule table T */
	node->eid = ++(T->sequence_number); //increase sequence_number

	p = (schedule_table_node_t*) calloc(1, sizeof(schedule_table_node_t));
	assert_memory(p);

	memcpy(p, node, sizeof(schedule_table_node_t));

	/* initialize node's sensor list that is a queue */
	InitQueue((queue_t*) &(p->sensor_list), QTYPE_SENSOR);

	/* insert node before head to let node be the last node in the queue */
	p->prev = T->head.prev;
	p->next = &(T->head);
	T->head.prev->next = p;
	T->head.prev = p;

	T->size++;

	return p; //return the pointer to the node newly added to table T
}

schedule_table_node_t* Detable(schedule_table_t *T)
{ //dequeue node from table T
	//static schedule_table_node_t node;
	schedule_table_node_t *p = NULL;

	assert_memory(T);

	if(T->size == 0)
		return NULL;

	p = T->head.next;
	T->head.next->next->prev = &(T->head);
	T->head.next = T->head.next->next;

	T->size--;

	//memcpy(&node, p, sizeof(node));
	//free(p);

	//return &node;

	return p;
	/** Note: we need to free the memory pointed by p later */
}

void DeleteTableEntry(schedule_table_t *T, schedule_table_node_t *pTableNode)
{ //delete the entry for pTableEntry from table T
	schedule_table_node_t *p = NULL, *q = NULL;

	assert_memory(T);

	if(T->size == 0)
		return;

	p = pTableNode->prev;
	q = pTableNode->next;

	/* link two table nodes adjacent to the node indicated by pTableNode */
	p->next = q;
	q->prev = p;

	T->size--;

	free(pTableNode);
}

void DestroyScheduleTable(schedule_table_t *T)
{ //destory schedule table T
	schedule_table_node_t *p = NULL, *q = NULL;

	assert_memory(T);

	if(T->size == 0)
		return;

	for(p = T->head.next; p != &(T->head);)
	{
		q = p;
		p = p->next;
		DestroyQueue((queue_t*) &(q->sensor_list)); //destory sensor list that is a queue
		free(q);
	}

	T->size = 0;
}

void DestroySensorTable(struct_sensor_table *S)
{ //destory sensor table S
	free(S->list);
	S->list = NULL;
	S->number = 0;
}

/* schedule_table_node_t* LookupTable(schedule_table_t *T, char *u, char *v, boolean *flip_flag) */
/* { //return the pointer to schedule table entry corresponding to the edge consisting of vertices u and v */
/* 	schedule_table_node_t *p = NULL; //table node */
/* 	schedule_table_node_t *node = NULL; //table node corresponding to the edge consisting of vertices u and v */

/* 	if(T->size == 0) */
/* 		return NULL; */

/* 	for(p = T->head.next; p != &(T->head); p = p->next) */
/* 	{ */
/* 		if(strcmp(p->tail_node, u) == 0 && strcmp(p->head_node, v) == 0) */
/* 		{ */
/* 			*flip_flag = FALSE; */
/* 			node = p; */
/* 			break; */
/* 		} */
/* 		else if(strcmp(p->tail_node, v) == 0 && strcmp(p->head_node, u) == 0) */
/* 		{ */
/* 			*flip_flag = TRUE; */
/* 			node = p; */
/* 			break;		 */
/* 		} */
/* 	}; */

/* 	return node; */
/* } */

schedule_table_node_t* LookupTable(struct_graph_node *Gv, char *u, char *v, boolean *flip_flag)
{ //return the pointer to schedule table entry corresponding to the edge consisting of vertices u and v
  schedule_table_node_t *node = NULL; //table node corresponding to the edge consisting of vertices u and v
  //struct_graph_node *Gv = T->Gv; //pointer to the virtual graph Gv
  int node_id = atoi(u);
  struct_graph_node *ptr = NULL; //pointer to graph node

  if(Gv == NULL)
  {
    printf("LookupTable(): Gv is NULL\n");
    exit(1);
  }

  ptr = &(Gv[node_id-1]); //pointer to graph node
	
  while(ptr != NULL)
  {
    ptr = ptr->next;
    if(ptr == NULL)
      break;

    if(strcmp(ptr->vertex, v) == 0)
    {
      node = ptr->ptr_table_node;
      break;
    }
  }

  if(node == NULL)
    return NULL;

  if(strcmp(node->tail_node, u) == 0 && strcmp(node->head_node, v) == 0)
  {
    *flip_flag = FALSE;
  }
  else if(strcmp(node->tail_node, v) == 0 && strcmp(node->head_node, u) == 0)
  {
    *flip_flag = TRUE;
  }
	
  return node;
}

int SizeofTable(schedule_table_t *T)
{ //return the size of table T
	return T->size;
}

/*
schedule_table_node_t* GetTableNode(schedule_table_t *T, int index)
{ //return the table node corresponding to index; the index of the first node is 0.
	schedule_table_node_t* p = NULL;
	int size = T->size; //size of table T, i.e., the number of valid table nodes
	int i; //for-loop index

	if(index >= T->size)
	{
		printf("GetTableNode(): Error: index(%d) >= T->size(%d)\n", index, T->size);
		exit(1);
	}

	for(i = 0, p = &(T->head); i <= index; i++)
		p = p->next;

	return p;
}
*/

schedule_table_node_t* GetTableNodeByEID(schedule_table_t *T, int eid)
{ //return the table node corresponding to eid; the eid starts from 1.
	schedule_table_node_t* p = NULL;
	int i; //for-loop index

	if(eid > T->sequence_number)
	{
		printf("GetTableNodeByEID(): Error: eid(%d) > T->sequence_number(%d)\n", eid, T->sequence_number);
#ifdef __DEBUG_INTERACTIVE_MODE__
                fgetc(stdin);
#endif
		exit(1);
	}

	for(i = 0, p = &(T->head); i < T->size; i++)
	{
		p = p->next;
		if(p->eid == eid)
			break;
	}

	if(p == &(T->head) || i == T->size)
		return NULL;
	else
		return p;
}

boolean IsExpandable(struct_graph_node *G, schedule_queue_node_t *node)
{ //determine the expansion of searching tree for queue_node with schedule table and update the sensing scheduling for intersection nodes in G
	//boolean result = FALSE;
	boolean flip_flag = FALSE; //flag to indicate if the order of vertices in node matches 
	//that of an entry in schedule table
	schedule_table_node_t *p = NULL; //pointer to schedule table node

	//int i;

	/* 	for(i = 0; i < T->size; i++) */ //=>useless for-loop???
/* 	{ */
/* 		p = LookupTable(T, node->tail_node, node->head_node, &flip_flag); */
/* 		if(p != NULL) */
/* 			break; */
/* 	} */

	/* look up the pointer p to the table entry for the virtual edge (tail_node,head_node) */
	p =  LookupTable(G, node->tail_node, node->head_node, &flip_flag);

	if(p == NULL)
	{
		printf("IsExpandable(): there is no entry corresponding to node(tail=%s , head=%s) in schedule table\n",\
			node->tail_node, node->head_node);
#ifdef __DEBUG_INTERACTIVE_MODE__
                fgetc(stdin);
#endif
		exit(1);
	}

	/** update the schedule times according to the direction of the scan */
	if(flip_flag == FALSE)
	{//the case where the schedule for the first directional scan (tail => head) is set at first
        if(p->flag1 == FALSE)
		{
			p->arrival_time1 = node->arrival_time;
			p->departure_time1 = node->departure_time;
			p->flag1 = TRUE;
		}
		else
		{
			if(p->arrival_time1 > node->arrival_time) 
			{ //update the schedule since the new schedule in node can be performed eariler than the schedule in schedule table
				p->arrival_time1 = node->arrival_time;
				p->departure_time1 = node->departure_time;
			}
			else
			{
				return FALSE;
			}
		}
	}
	else if(flip_flag == TRUE)
	{//the case where the schedule for the second directional scan (head => tail) is set at first
		if(p->flag2 == FALSE)
		{
			p->arrival_time2 = node->arrival_time;
			p->departure_time2 = node->departure_time;
			p->flag2 = TRUE;
		}
		else
		{
			if(p->arrival_time2 > node->arrival_time) 
			{ //update the schedule since the new schedule in node can be performed eariler than the schedule in schedule table
				p->arrival_time2 = node->arrival_time;
				p->departure_time2 = node->departure_time;
			}
			else
			{
				return FALSE;
			}
		}
	}

	/** adjust the status of schedule intervals for two directional scans */
	if(p->flag1 == TRUE && p->flag2 == TRUE)
	{
		if(((p->arrival_time1 >= p->arrival_time2) && (p->arrival_time1 <= p->departure_time2)) ||
		   ((p->arrival_time2 >= p->arrival_time1) && (p->arrival_time2 <= p->departure_time1)))
		{
			p->status = STATUS_OVERLAP;
		}
		else if(p->departure_time1 < p->arrival_time2)
		{
			p->status = STATUS_FORWARD;
		}
		else if(p->departure_time2 < p->arrival_time1)
		{
			p->status = STATUS_BACKWARD;
		}
		/*
		else
		{
			printf("IsExpandable(): We cannot determine the status of scanning\n");
			return FALSE;
		}
		*/
	}
	else if(p->flag1 == TRUE)
	{
		p->status = STATUS_FORWARD;
	}
	else if(p->flag2 == TRUE)
	{
		p->status = STATUS_BACKWARD;
	}
	
	return TRUE;
}

int DeploySensorNetwork(parameter_t *param, schedule_table_t *T, struct_sensor_table *S, edge_queue_t *Q)
{ /* deploy sensor nodes into the road network by generating sensor nodes with sensor density,
   selecting their locations, and registering them with schedule table T, sensor table S, and edge queue Q */

	schedule_table_node_t *pTableNode = NULL; //pointer to schedule table node
	sensor_queue_node_t queue_node; //node for sensor queue
	sensor_queue_node_t *pQueueNode = NULL; //pointer to sensor queue node
	edge_queue_node_t *pEdgeNode = NULL; //pointer to edge queue node
	location_queue_node_t sensor_location_node; //sensor location node
	//double density = param->sensor_density; //sensor density: #sensors per meter
	int edge_num = T->size; //number of edges in the graph in the road network
	int sensor_num = 0; //number of sensor nodes for an edge according to the edge's weight (i.e., length)
	int total_sensor_num = 0; //total number of sensor nodes deployed on the road network
	int i, j; //indexed for for-loops
	double weight; //edge weight
	double offset; //relative position from the edge's tail
	int id = 0; //sensor id
	int order = 0; //location order for sensor placed on an edge
	double grid_space = 0; //grid space for uniform distribution
	double r = param->sensor_sensing_range; //sensor sensing radius

	/** Pseudo code:

	Given d = sensor density = #sensors/meter, m = #edges in graph G,

	for i = 0 to m-1 do
        l = weight(e(i)); // weight means the length of the edge
	   a = d*l;
	   for j = 0 to a-1 do
          s(i,j) = uniform(0, l); //s(i,j): sensor node placed on edge id (eid) i
	   end
	end
		
	*/
	
	for(i = 0, pTableNode = &(T->head); i < edge_num; i++)
	{
		pTableNode = pTableNode->next;
		weight = pTableNode->weight;
		sensor_num = (int) (pTableNode->density * weight / (2*r));
		//sensor_num = (int) (pTableNode->density * weight);
		//sensor_num = (int) (density * weight);	

		if(param->sensor_deployment_distribtuion == EQUAL)
			grid_space = weight/sensor_num; //grid space for uniform distribution

		for(j = 0; j < sensor_num; j++)
		{
			if(param->sensor_deployment_distribtuion == EQUAL)
			{
				//offset = grid_space*j + 0.5;
				//offset = grid_space*j + 1/2;
				//offset = grid_space*j + 1;
				offset = grid_space*j;
				//offset = grid_space*j + grid_space/2;
			}
			else if(param->sensor_deployment_distribtuion == UNIFORM)
			{
				offset = uniform(0.1, weight-0.1);
				//offset = smpl_random(0, weight);
				//offset = smpl_random(1, weight-1);
				//we use the range of 0.1 and weight-0.1 to prevent the sensors from being located at the intersection points;
				//this is because we can make it easy to compute scanning time for each edge in the graph for the road network.
			}
			else
			{
				printf("DeploySensorNetwork(): sensor_deployment_distribtuion of type %d is not supported!\n", param->sensor_deployment_distribtuion);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                fgetc(stdin);
#endif
				exit(1);
			}

#ifdef __DEBUG__
			printf("edge(%s,%s) with weight %d: offset=%d\n", pTableNode->tail_node, pTableNode->head_node, 
				pTableNode->weight, offset);
#endif
			memset(&queue_node, 0, sizeof(queue_node));
			queue_node.info.id = 0; //unknown sensor id; later, it will be assigned a unique number.

			/* position in real graph Gr */
			queue_node.info.pos_in_Gr.eid = pTableNode->eid;
			queue_node.info.pos_in_Gr.offset = offset;

			/* position in virtual graph Gv */
			queue_node.info.pos_in_Gv.eid = pTableNode->eid;
			queue_node.info.pos_in_Gv.offset = offset;


			//queue_node.info.energy = param->sensor_energy; //consider heterogenous energy
			if(param->sensor_energy_distribution == NORMAL)
			{   /* bound sensor energy using param->sensor_energy_maximum_deviation */
				do
				{
					queue_node.info.energy = dist_func(param->sensor_energy_distribution, 
					param->sensor_energy, param->sensor_energy_standard_deviation);

					if(fabs(queue_node.info.energy - param->sensor_energy) < param->sensor_energy_maximum_deviation)
					{
						queue_node.info.initial_energy = queue_node.info.energy;
						break;
					}
				}while(1);
			}
			else if(param->sensor_energy_distribution == UNIFORM)
			{   
				queue_node.info.energy = dist_func(param->sensor_energy_distribution, 
					param->sensor_energy - param->sensor_energy_maximum_deviation, 
					param->sensor_energy + param->sensor_energy_maximum_deviation);
				queue_node.info.initial_energy = queue_node.info.energy;
			}
			else if(param->sensor_energy_distribution == EQUAL)
			{
				queue_node.info.energy = param->sensor_energy;
				queue_node.info.initial_energy = queue_node.info.energy;
			}
			else
			{
				printf("DeploySensorNetwork(): param->sensor_energy_distribution(%d) is not supported\n", 
					param->sensor_energy_distribution);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                fgetc(stdin);
#endif
				exit(1);
			}

			queue_node.info.energy_consumption_rate = param->sensor_energy_consumption_rate;
			queue_node.info.warm_up_time = param->sensor_warm_up_time; //warm-up time for sensing devices
			queue_node.info.turn_on_energy_consumption = param->sensor_turn_on_energy_consumption; //turn_on_energy_consumption
			queue_node.info.sensing_range = param->sensor_sensing_range;
			queue_node.info.reschedule_flag = FALSE;

			/* enqueue queue_node into queue list */
			Enqueue((queue_t*) &(pTableNode->sensor_list), (queue_node_t*) &queue_node);
			pTableNode->sensor_list.live_sensor_number++; //increase the number of live sensors
		}

		/* sort sensor queue nodes in ascending order according to offset */
		SortSensorQueue(&(pTableNode->sensor_list));

		total_sensor_num += sensor_num;
	}

	/* generate sensor table containing pointers to all of the sensor nodes in schedule table */
	S->list = (sensor_queue_node_t**) calloc(total_sensor_num, sizeof(sensor_queue_node_t*));
	assert_memory(S->list);
	S->number = total_sensor_num;

	id = 1;
	pTableNode = &(T->head);
	pEdgeNode = &(Q->head);
	for(i = 0; i < edge_num; i++)
	{
		pTableNode = pTableNode->next;
		sensor_num = pTableNode->sensor_list.size;
		order = 0;
		pEdgeNode = pEdgeNode->next;
		
		/* register the total number of sensors (and number of live sensors) on the edge corresponding to pEdgeNode */
		pEdgeNode->total_sensor_number = sensor_num;
		pEdgeNode->live_sensor_number = sensor_num;

		for(j = 0; j < sensor_num; j++)
		{
			pQueueNode = (sensor_queue_node_t*) GetQueueNode((queue_t*) &(pTableNode->sensor_list), j);
			S->list[id-1] = pQueueNode;
			S->list[id-1]->info.id = id;
			S->list[id-1]->info.duty_cycle_number = 0;
			S->list[id-1]->pEdgeQueueNode = pEdgeNode; //pointer to the edge queue node corresponding to the edge where the sensor is located
	
			/* let pQueueNode->pSensorListEntry point to &(S->list[id-1]) in order to allow S->list[id-1] to point to the updated pQueueNode with the same sensor id at the handling of sensing hole */
			pQueueNode->pSensorListEntry = &(S->list[id-1]); //pQueueNode->pSensorListEntry points to the memory cell to point to pQueueNode

			/* let pLocationTableNode's one queue node point to the sensor table entry of S->list[id-1] 
			   in order to access the sensor when vehicles check whether they are detected by sensors or not */
			memset(&sensor_location_node, 0, sizeof(sensor_location_node));
			sensor_location_node.id = id; //sensor id that can be the index in sensor table S to access S[id-1] that is the entry corresponding to the sensor.
			sensor_location_node.order = order++; //location order in the edge starting from 0
			sensor_location_node.sensor = S->list[id-1]; //sensor points to the sensor node entry containing sensor information, such as position.
			Enqueue((queue_t*) &(pEdgeNode->sensor_location_list), (queue_node_t*) &sensor_location_node); //enqueue the location of the sensor indicated by (id-1) in sensor table S into sensor location list

			id++;
		}		
	}

	return total_sensor_num;
}

void ComputeSensingSchedule(parameter_t *param, schedule_table_t *T, struct_sensor_table *S)
{ //compute sensing schedule for each sensor node using schedule table T and sensor table S
	//double sensing_range = param->sensor_sensing_range; //sensing range
	int i; //for-loop index
	int total_sensor_num = S->number;
	sensor_queue_node_t *pSensor = NULL; //pointer to sensor node in sensor table
	schedule_table_node_t *pTableNode = NULL; //pointer to schedule table node
	int eid; //edge id
	double sensor_work_time = param->sensor_work_time; //minimum sensor work time for detection
	//double vehicle_maximum_speed = param->vehicle_maximum_speed; //vehicle maximum speed
	sensor_scan_type_t scan_type = param->sensor_scan_type; //scan type

	for(i = 0; i < total_sensor_num; i++)
	{
		pSensor = S->list[i];

	        //if(pSensor->info.id == 630)
		//  printf("sensor(id=%d) is checked\n", i+1);

		eid = pSensor->info.pos_in_Gv.eid;
		pTableNode = GetTableNodeByEID(T, eid);
		ComputeSensorDutyCycle(pSensor, pTableNode, sensor_work_time, scan_type); //compute sensor duty cycle

		/*@For debugging */
		//if(pSensor->info.id >= 983 && pSensor->info.id <= 995)
		//	printf("sensor(id=%d) is checked for its working schedule [%f,%f]\n", pSensor->info.id, (float)pSensor->info.relative_sensing_start_time, (float)pSensor->info.relative_sensing_end_time);
		/*****/
	}
}

void ComputeSensorDutyCycle(sensor_queue_node_t *pSensor, schedule_table_node_t *pTableNode, double sensor_work_time, sensor_scan_type_t scan_type)
{ //compute sensor duty cycle, such as sensing start time and sensing end time
	enum_status_t status_type = pTableNode->status; //status type for scan direction in edge
	double x = pSensor->info.pos_in_Gv.offset; //location from the tail of the edge eid; unit [m] 

	//int order = pSensor->order; //order of sensor from the tail of the edge
	int order = pSensor->order_for_live_sensors; //order of live sensor from the tail of the edge

	double r = pSensor->info.sensing_range; //sensing range (or sensing radius)
	double w = sensor_work_time; //minimum sensor work time for detection
	double v = 2*r/w; //scan_speed
	double d = pTableNode->weight; //edge distance
	double d_m = 0; //merge position of two scans from the tail of the edge
	//int n = pTableNode->sensor_list.size; //number of sensors whose offset is unique on the edge
	int n = pTableNode->sensor_list.live_sensor_number; //number of live sensors whose offset is unique on the edge
	int order_m = 0; //the order of the sensor where two scans are merged from the tail of the edge 
	double t_a = 0; //arrival time of scan 1 (i.e., forward scan)
	double t_b = 0; //arrival time of scan 2 (i.e., backward scan)
	int sensor_num = 0; //number of sensors in front of the sensor pointed by pSensor for the scan's arrival node of the edge
	
	switch(status_type)
	{
	case STATUS_FORWARD:
		t_a = pTableNode->arrival_time1;
		
		switch(scan_type)
		{
		case SCAN_TURN_ON_ALL:
		case SCAN_NO_USE:
		case SCAN_NO_USE_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
			pSensor->info.relative_sensing_start_time = 0;
			pSensor->info.relative_sensing_end_time = w;
			break;

		case SCAN_CONSTANT_SPEED_WITH_NONOPTIMAL_SLEEPING:
		case SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING:
		case SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
			pSensor->info.relative_sensing_start_time = t_a + MAX((x-r)/v, 0);
			pSensor->info.relative_sensing_end_time = pSensor->info.relative_sensing_start_time + w; //sensor must work at least during sensor_work_time w in order to detect vehicles
			//pSensor->info.relative_sensing_end_time = t_a + MIN((x+r)/v, d/v);
			break;

		case SCAN_VARIABLE_SPEED_WITH_NONOPTIMAL_SLEEPING:
		case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING:
		case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
		case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING:
		case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING_AND_WITH_SENSING_HOLE_HANDLING:
		case SCAN_VARIABLE_SPEED_WITH_VARIABLE_SLEEPING_TIME_AND_WITH_SENSING_HOLE_HANDLING:
			//sensor_num = CountSensorNumberByOffset(pTableNode, status_type, x);
			sensor_num = order; //order indicates the number of sensors before this sensor
			//count the number of sensors in front of offset x from the scan's arrival node of the edge

			pSensor->info.relative_sensing_start_time = t_a + sensor_num*w;
			pSensor->info.relative_sensing_end_time = pSensor->info.relative_sensing_start_time + w;
			break;

		case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_VARIABLE_WORKING_TIME:
			printf("ComputeSensorDutyCycle(): scan_type(%d) is not implemented yet\n", scan_type);
			break;

		default:
			printf("ComputeSensorDutyCycle(): scan_type(%d) is unknown\n", scan_type);
#ifdef __DEBUG_INTERACTIVE_MODE__
                        fgetc(stdin);
#endif
			exit(1);
		}
		break;

	case STATUS_BACKWARD:
		t_b = pTableNode->arrival_time2;
		
		/*@ for debugging */
		//if(pSensor->info.pos_in_Gv.eid == 2 && pSensor->info.pos_in_Gv.offset > 75.876)
		//	printf("debug\n");
		/*********/

		switch(scan_type)
		{
		case SCAN_TURN_ON_ALL:
		case SCAN_NO_USE:
		case SCAN_NO_USE_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
			pSensor->info.relative_sensing_start_time = 0;
			pSensor->info.relative_sensing_end_time = w;
			break;

		case SCAN_CONSTANT_SPEED_WITH_NONOPTIMAL_SLEEPING:
		case SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING:
		case SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
			pSensor->info.relative_sensing_start_time = t_b + MAX(((d-x)-r)/v, 0);
			pSensor->info.relative_sensing_end_time = pSensor->info.relative_sensing_start_time + w; //sensor must work at least during sensor_work_time w in order to detect vehicles
			//pSensor->info.relative_sensing_end_time = t_b + MIN(((d-x)+r)/v, d/v);
			break;

		case SCAN_VARIABLE_SPEED_WITH_NONOPTIMAL_SLEEPING:
		case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING:
		case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
		case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING:
		case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING_AND_WITH_SENSING_HOLE_HANDLING:
		case SCAN_VARIABLE_SPEED_WITH_VARIABLE_SLEEPING_TIME_AND_WITH_SENSING_HOLE_HANDLING:
			//sensor_num = CountSensorNumberByOffset(pTableNode, status_type, x);
			sensor_num = (n-1) - order;
			//count the number of sensors in front of offset x from the scan's arrival node of the edge
			/* original sensor orders:   0, 1, 2, ..., i, ..., (n-3), (n-2), (n-1).
                           converted sensor orders: (n-1), (n-2), (n-3), ..., (n-1)-i, ..., 2, 1, 0.
			*/

			pSensor->info.relative_sensing_start_time = t_b + sensor_num*w;
			pSensor->info.relative_sensing_end_time = pSensor->info.relative_sensing_start_time + w;
			break;

		case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_VARIABLE_WORKING_TIME:
			printf("ComputeSensorDutyCycle(): scan_type(%d) is not implemented yet\n", scan_type);
			break;

		default:
			printf("ComputeSensorDutyCycle(): scan_type(%d) is unknown\n", scan_type);
#ifdef __DEBUG_INTERACTIVE_MODE__
                        fgetc(stdin);
#endif
			exit(1);
		}
		break;

	case STATUS_OVERLAP:
		/* three cases for STATUS_OVERLAP */
		t_a = pTableNode->arrival_time1;
		t_b = pTableNode->arrival_time2;

		switch(scan_type)
		{
		case SCAN_TURN_ON_ALL:
		case SCAN_NO_USE:
		case SCAN_NO_USE_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
			pSensor->info.relative_sensing_start_time = 0;
			pSensor->info.relative_sensing_end_time = w;
			break;

		case SCAN_CONSTANT_SPEED_WITH_NONOPTIMAL_SLEEPING:
		case SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING:
		case SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
			/* determine the merge position of two scans */
			if(t_a == t_b)
				d_m = (int)(d/2);
			else if(t_a < t_b)
				d_m = (int)((d + v*(t_b-t_a))/2);
			else if(t_a > t_b)
				d_m = (int)((d - v*(t_a-t_b))/2);
		
			/* compute the schedule according to the sensor's position for the merge position d_m */
			if(x <= d_m) /* schedule is determined by scan 1 (i.e., forward scan) */
			{
				pSensor->info.relative_sensing_start_time = t_a + MAX((x-r)/v, 0);
				pSensor->info.relative_sensing_end_time = pSensor->info.relative_sensing_start_time + w; //sensor must work at least during sensor_work_time w in order to detect vehicles
				//pSensor->info.relative_sensing_end_time = t_a + MIN((x+r)/v, d/v);
			}
			else /* schedule is determined by scan 2 (i.e., backward scan) */
			{
				pSensor->info.relative_sensing_start_time = t_b + MAX(((d-x)-r)/v, 0);
				pSensor->info.relative_sensing_end_time = pSensor->info.relative_sensing_start_time + w; //sensor must work at least during sensor_work_time w in order to detect vehicles
				//pSensor->info.relative_sensing_end_time = t_b + MIN(((d-x)+r)/v, d/v);
			}
			break;

		case SCAN_VARIABLE_SPEED_WITH_NONOPTIMAL_SLEEPING:
		case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING:
		case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
		case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING:
		case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING_AND_WITH_SENSING_HOLE_HANDLING:
		case SCAN_VARIABLE_SPEED_WITH_VARIABLE_SLEEPING_TIME_AND_WITH_SENSING_HOLE_HANDLING:
			/* determine the order of the sensor where two scans are merged */
			if(t_a == t_b)
				order_m = (int)ceil(n/2) - 1; 
			    //number of sensors: (a) order set of odd number (e.g., 0, *1, 2) and  
			    //(b) order set of even number (e.g., 0, *1, 2, 3)
			    //where * indicates the merge sensor of two scans.
			else if(t_a < t_b)
				order_m = (int)ceil((t_b-t_a)/w + ceil((n*w - (t_b-t_a))/(2*w))) - 1;
			else if(t_a > t_b)
				order_m = (n-1) - ((int)ceil((t_a-t_b)/w + ceil((n*w - (t_a-t_b))/(2*w))) - 1);

			/* compute the schedule according to the sensor's position for the merge order order_m */
			if(order <= order_m) /* schedule is determined by scan 1 (i.e., forward scan) */
			{
				sensor_num = order;
				pSensor->info.relative_sensing_start_time = t_a + sensor_num*w;
				pSensor->info.relative_sensing_end_time = pSensor->info.relative_sensing_start_time + w;
			}
			else /* schedule is determined by scan 2 (i.e., backward scan) */
			{
				sensor_num = (n-1) - order;
				pSensor->info.relative_sensing_start_time = t_b + sensor_num*w;
				pSensor->info.relative_sensing_end_time = pSensor->info.relative_sensing_start_time + w;
			}
			break;

		case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_VARIABLE_WORKING_TIME:
			printf("ComputeSensorDutyCycle(): scan_type(%d) is not implemented yet\n", scan_type);
			break;

		default:
			printf("ComputeSensorDutyCycle(): scan_type(%d) is unknown\n", scan_type);
#ifdef __DEBUG_INTERACTIVE_MODE__
                        fgetc(stdin);
#endif
			exit(1);
		}
		break;

	default:
		if(scan_type == SCAN_TURN_ON_ALL || scan_type == SCAN_NO_USE || scan_type == SCAN_NO_USE_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING)
		{
			pSensor->info.relative_sensing_start_time = 0;
			pSensor->info.relative_sensing_end_time = w;
			break;
		}
		else
		{
			printf("ComputeSensorDutyCycle(): status_type(%d) is unknown\n", status_type);
#ifdef __DEBUG_INTERACTIVE_MODE__
                        fgetc(stdin);
#endif
			exit(1);
		}
	}

	/* compute sensing interval in duty cycle */
	pSensor->info.sensing_interval = pSensor->info.relative_sensing_end_time - pSensor->info.relative_sensing_start_time;
}

int CountSensorNumberByOffset(schedule_table_node_t *pTableNode, enum_status_t status_type, double x)
{ //count the number of sensors in front of offset x from the scan's arrival node of the edge corresponding to pTableNode
	int sensor_num = 0; //number of sensors in front of the sensor pointed by pSensor for the scan's arrival node of the edge
	int i; //index for for-loop
	sensor_queue_node_t *pSensor = NULL;
	//double edge_length = pTableNode->weight;

	for(i = 0; i < pTableNode->sensor_list.size; i++)
	{
		pSensor = (sensor_queue_node_t*) GetQueueNode((queue_t*) &(pTableNode->sensor_list), i);
		switch(status_type)
		{
		case STATUS_FORWARD:
			if(pSensor->info.pos_in_Gv.offset < x)
				sensor_num++;
			break;

		case STATUS_BACKWARD:
			if(pSensor->info.pos_in_Gv.offset > x)
				sensor_num++;
			break;

		case STATUS_OVERLAP:
			break;
		}
	}

	return sensor_num;
}

void ComputeSleepingSchedule(parameter_t *param, schedule_table_t *T, struct_sensor_table *S, struct_traffic_table *src_table, struct_traffic_table *dst_table, int G_size, double **D_move, int **M_move, int **D_scan, int **M_scan, double *movement_time, double *scanning_time, double *sleeping_time)
{ //compute sleeping schedule for each sensor node using schedule table T and sensor table S
        double shortest_path_length = INF; //length of the shortest path from Enter nodes to Exit nodes
	int shortest_path_sensor_num = INF; //number of sensors on the shortest path from destination to source
	char shortest_path_src[NAME_SIZE]; //source of the shortest path from Enter nodes to Exit nodes
	char shortest_path_dst[NAME_SIZE]; //destination of the shortest path from Enter nodes to Exit nodes
	int src = 0; //virtual scan's arrival vetex
	int dst1 = 0; //virtual scan's departure vertex 
	int dst2 = 0; //vehicle's arrival vertex
	//double v_max = param->vehicle_maximum_speed;
	double v_max = (param->vehicle_speed_distribution==EQUAL ? param->vehicle_speed : param->vehicle_maximum_speed); //maximum vehicle speed
	double r = param->sensor_sensing_range; //sensing range (or sensing radius)
	double w = param->sensor_work_time; //minimum sensor work time for detection
	double v_s = 2*r/w; //scan_speed
	double move_time = INF; //vehicle's movement time
	double scan_time = INF; //sensor scanning time
	double sleep_time = INF; //sensor sleeping time
	int total_sensor_num = S->number; //total number of sensors deployed into the road network
	int sensor_num = 0; //number of sensors on the shortest path
	int i, j, k; //indices for for-loops
	sensor_queue_node_t *pSensor = NULL; //pointer to sensor node in sensor table
	int index_i = -1, index_k = -1, index_j = -1; //indices of source and destinations to achieve the minimum sleeping time
    
	switch(param->sensor_scan_type)
	{
	case SCAN_TURN_ON_ALL:
		*movement_time = 0;
		*scanning_time = 0;
		*sleeping_time = *movement_time + *scanning_time;
		break;

	case SCAN_NO_USE:
	case SCAN_NO_USE_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
		/** Compute sleeping time for sensors */
		/* 1. compute the length of the shortest path from Enter nodes to Exit nodes along with source and destination */
		shortest_path_length = FindShortestPath(src_table, dst_table, G_size, D_move, shortest_path_src, shortest_path_dst);

		/* 2. compute the sleeping time as the travel time for the shortest path by dividing the distance by the maximum vehicle speed */
		*movement_time = shortest_path_length/v_max;

		/* 3. scanning time is set to zero since the scan is not used */
		*scanning_time = 0;

		/* 4. actual sleeping interval of sensors, i.e., sleeping time + scanning time */
		*sleeping_time = *movement_time + *scanning_time;
		break;

	case SCAN_CONSTANT_SPEED_WITH_NONOPTIMAL_SLEEPING:
		/** Compute sleeping time for sensors */
		/* 1. compute the length of the shortest path from Enter nodes to Exit nodes along with source and destination */
		shortest_path_length = FindShortestPath(src_table, dst_table, G_size, D_move, shortest_path_src, shortest_path_dst);

		/* 2. compute the sleeping time as the travel time for the shortest path by dividing the distance by the maximum vehicle speed */
		*movement_time = shortest_path_length/v_max;

		/* 3. compute the scanning time as the scan time for the shortest path by dividing the distance by the scan speed */
		*scanning_time = shortest_path_length/v_s;

		/* 4. actual sleeping interval of sensors, i.e., sleeping time + scanning time */
		*sleeping_time = *movement_time + *scanning_time;
		break;

	case SCAN_VARIABLE_SPEED_WITH_NONOPTIMAL_SLEEPING:
		/** Compute sleeping time for sensors */
		/* 1. compute the length of the shortest path from Enter nodes to Exit nodes along with source and destination */
		shortest_path_length = FindShortestPath(src_table, dst_table, G_size, D_move, shortest_path_src, shortest_path_dst);

		/* 2. compute the sleeping time as the travel time for the shortest path by dividing the distance by the maximum vehicle speed */
		*movement_time = shortest_path_length/v_max;

		/* 3. compute the scanning time as the minimum scan arrival time among scan arrival times for sources (entrances) */
		*scanning_time = GetEarliestScanArrivalTimeForSources(src_table, T);

		/* 4. actual sleeping interval of sensors, i.e., sleeping time + scanning time */
		*sleeping_time = *movement_time + *scanning_time;
		break;

	case SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING:
	case SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
		/* we choose a pair of *movement_time and *scanning_time for optimal sleeping time */
		*sleeping_time = INF;
		
		for(i = 0; i < dst_table->number; i++) //for-1
		{
			//dst1 = atoi(dst_table->list[i]) - 1;
			dst1 = atoi(dst_table->list[i].vertex) - 1;
			for(k = 0; k < src_table->number; k++) //for-2
			{
				//src = atoi(src_table->list[k]) - 1;
				src = atoi(src_table->list[k].vertex) - 1;

				if(src == dst1) //when src and dst1 are the same, skip it
				{
					printf("ComputeSleepingSchedule(): Error: src(%d) is equal to dst1(%d)\n", src, dst1);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                        fgetc(stdin);
#endif
					exit(1);
					//continue;
				}

				for(j = 0; j < dst_table->number; j++) //for-3
				{
					//dst2 = atoi(dst_table->list[j]) - 1;
					dst2 = atoi(dst_table->list[j].vertex) - 1;
				
					if(src == dst2) //when src and dst2 are the same, skip it
					{
						printf("ComputeSleepingSchedule(): Error: src(%d) is equal to dst2(%d)\n", src, dst2);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                                fgetc(stdin);
#endif
						exit(1);
						//continue;
					}

					shortest_path_length = D_move[dst1][src];
					scan_time = shortest_path_length/v_s; //sensor scanning time from destination 1 to source

					shortest_path_length = D_move[src][dst2];
					move_time = shortest_path_length/v_max; //vehicle's movement time from source to destination 2

					if(*sleeping_time > move_time + scan_time)
					{
						*movement_time = move_time;
						*scanning_time = scan_time;
						*sleeping_time = *movement_time + *scanning_time;
						index_i = i;
						index_k = k;
						index_j = j;

#ifdef __DEBUG_LEVEL_SLEEPING_TIME__
						if(index_i != index_j)
							printf("The scan starting point index_i(%d) is different from the vehicle arrival point index_j(%d)\n", index_i, index_j);
#endif
					}
				} //end of for-3
			} //end of for-2
		} //end of for-1
		
		/*
		for(i = 0; i < dst_table->number; i++)
		{
			dst = atoi(dst_table->list[i]) - 1;
			for(j = 0; j < src_table->number; j++)
			{
				src = atoi(src_table->list[j]) - 1;

				if(src == dst) //when src and dst are the same, skip it
					break;

				shortest_path_length = D_move[dst][src];
				move_time = shortest_path_length/v_max; //vehicle's movement time from source to destination
				scan_time = shortest_path_length/v_s; //sensor scanning time from destination to source

				if(*sleeping_time > move_time + scan_time)
				{
					*movement_time = move_time;
					*scanning_time = scan_time;
					*sleeping_time = *movement_time + *scanning_time;
				}
			}
		}
		*/

		break;

	case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING:
	case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
		/* we choose a pair of *movement_time and *scanning_time for optimal sleeping time */
		*sleeping_time = INF;
		for(i = 0; i < dst_table->number; i++) //for-1
		{
			//dst1 = atoi(dst_table->list[i]) - 1;
			dst1 = atoi(dst_table->list[i].vertex) - 1;
			for(k = 0; k < src_table->number; k++) //for-2
			{
				//src = atoi(src_table->list[k]) - 1;
				src = atoi(src_table->list[k].vertex) - 1;

				if(src == dst1) //when src and dst1 are the same, skip it
				{
					printf("ComputeSleepingSchedule(): Error: src(%d) is equal to dst1(%d)\n", src, dst1);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                        fgetc(stdin);
#endif
					exit(1);
					//continue;
				}

				for(j = 0; j < dst_table->number; j++) //for-3
				{
					//dst2 = atoi(dst_table->list[j]) - 1;
					dst2 = atoi(dst_table->list[j].vertex) - 1;

					if(src == dst2) //when src and dst2 are the same, skip it
					{
						printf("ComputeSleepingSchedule(): Error: src(%d) is equal to dst2(%d)\n", src, dst2);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                                fgetc(stdin);
#endif
						exit(1);
						//continue;
					}

					shortest_path_sensor_num = D_scan[dst1][src];
					scan_time = shortest_path_sensor_num * w; //sensor scanning time from destination 1 to source

					shortest_path_length = D_move[src][dst2];
					move_time = shortest_path_length/v_max; //vehicle's movement time from source to destination 2

					if(*sleeping_time > move_time + scan_time)
					{
						*movement_time = move_time;
						*scanning_time = scan_time;
						*sleeping_time = *movement_time + *scanning_time;
						index_i = i;
						index_k = k;
						index_j = j;

#ifdef __DEBUG_LEVEL_SLEEPING_TIME__
						if(index_i != index_j)
							printf("The scan starting point index_i(%d) is different from the vehicle arrival point index_j(%d)\n", index_i, index_j);
#endif
					}
				} //end of for-3
			} //end of for-2
		} //end of for-1

		/*
		for(i = 0; i < dst_table->number; i++)
		{
			dst = atoi(dst_table->list[i]) - 1;
			for(j = 0; j < src_table->number; j++)
			{
				src = atoi(src_table->list[j]) - 1;

				if(src == dst) //when src and dst are the same, skip it
					break;

				shortest_path_length = D_move[dst][src];
				move_time = shortest_path_length/v_max; //vehicle's movement time from source to destination
				shortest_path_sensor_num = D_scan[dst][src];
				scan_time = shortest_path_sensor_num * w; //sensor scanning time from destination to source

				if(*sleeping_time > move_time + scan_time)
				{
					*movement_time = move_time;
					*scanning_time = scan_time;
					*sleeping_time = *movement_time + *scanning_time;
				}
			}
		}
		*/

		break;

	case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING:
	case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING_AND_WITH_SENSING_HOLE_HANDLING:
		/* we choose *scanning_time for optimal sleeping time */
		*sleeping_time = INF;
		*movement_time = 0;
		for(i = 0; i < dst_table->number; i++) //for-1
		{
			//dst1 = atoi(dst_table->list[i]) - 1;
			dst1 = atoi(dst_table->list[i].vertex) - 1;
			for(k = 0; k < src_table->number; k++) //for-2
			{
				//src = atoi(src_table->list[k]) - 1;
				src = atoi(src_table->list[k].vertex) - 1;

				if(src == dst1) //when src and dst1 are the same, skip it
				{
					printf("ComputeSleepingSchedule(): Error: src(%d) is equal to dst1(%d)\n", src, dst1);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                        fgetc(stdin);
#endif
					exit(1);
					//continue;
				}

				for(j = 0; j < dst_table->number; j++) //for-3
				{
					//dst2 = atoi(dst_table->list[j]) - 1;
					dst2 = atoi(dst_table->list[j].vertex) - 1;

					if(src == dst2) //when src and dst2 are the same, skip it
					{
						printf("ComputeSleepingSchedule(): Error: src(%d) is equal to dst2(%d)\n", src, dst2);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                                fgetc(stdin);
#endif
						exit(1);
						//continue;
					}

					shortest_path_sensor_num = D_scan[dst1][src];
					scan_time = shortest_path_sensor_num * w; //sensor scanning time from destination 1 to source

					if(*sleeping_time > scan_time)
					{
						*scanning_time = scan_time;
						*sleeping_time = *scanning_time;
						index_i = i;
						index_k = k;
						index_j = j;

#ifdef __DEBUG_LEVEL_SLEEPING_TIME__
						if(index_i != index_j)
							printf("The scan starting point index_i(%d) is different from the vehicle arrival point index_j(%d)\n", index_i, index_j);
#endif
					}
				} //end of for-3
			} //end of for-2
		} //end of for-1
		break;

	case SCAN_VARIABLE_SPEED_WITH_VARIABLE_SLEEPING_TIME_AND_WITH_SENSING_HOLE_HANDLING:
		/* we choose a pair of *movement_time and *scanning_time for optimal sleeping time */
		*sleeping_time = INF;
		for(i = 0; i < dst_table->number; i++) //for-1
		{
			//dst1 = atoi(dst_table->list[i]) - 1;
			dst1 = atoi(dst_table->list[i].vertex) - 1;
			for(k = 0; k < src_table->number; k++) //for-2
			{
				//src = atoi(src_table->list[k]) - 1;
				src = atoi(src_table->list[k].vertex) - 1;

				if(src == dst1) //when src and dst1 are the same, skip it
				{
					printf("ComputeSleepingSchedule(): Error: src(%d) is equal to dst1(%d)\n", src, dst1);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                        fgetc(stdin);
#endif
					exit(1);
					//continue;
				}

				for(j = 0; j < dst_table->number; j++) //for-3
				{
					//dst2 = atoi(dst_table->list[j]) - 1;
					dst2 = atoi(dst_table->list[j].vertex) - 1;

					if(src == dst2) //when src and dst2 are the same, skip it
					{
						printf("ComputeSleepingSchedule(): Error: src(%d) is equal to dst2(%d)\n", src, dst2);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                                fgetc(stdin);
#endif
						exit(1);
						//continue;
					}

					shortest_path_sensor_num = D_scan[dst1][src];
					scan_time = shortest_path_sensor_num * w; //sensor scanning time from destination 1 to source

					shortest_path_length = D_move[src][dst2];
					//move_time = shortest_path_length/v_max; //vehicle's movement time from source to destination 2
					move_time = shortest_path_length/v_max*param->sensor_movement_time_percentage/100; //we just use the time corresponding to the required percentage (sensor_movement_time_percentage) of vehicle's movement time from source to destination 2

					if(*sleeping_time > move_time + scan_time)
					{
						*movement_time = move_time;
						*scanning_time = scan_time;
						*sleeping_time = *movement_time + *scanning_time;
						index_i = i;
						index_k = k;
						index_j = j;

#ifdef __DEBUG_LEVEL_SLEEPING_TIME__
						if(index_i != index_j)
							printf("The scan starting point index_i(%d) is different from the vehicle arrival point index_j(%d)\n", index_i, index_j);
#endif
					}
				} //end of for-3
			} //end of for-2
		} //end of for-1
		break;

	case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_VARIABLE_WORKING_TIME:
		printf("ComputeSleepingSchedule(): sensor_scan_type(%d) is not implemented yet\n", param->sensor_scan_type);
		break;

	default:
		printf("ComputeSleepingSchedule(): sensor_scan_type(%d) is unknown\n", param->sensor_scan_type);
		break;
	}

	/* 5. set the travel time to the sleeping time for each sensor */
	if(*sleeping_time < 0)
	{
		printf("ComputeSleepingSchedule(): *sleeping_time(%f) < 0\n", (float) *sleeping_time);
#ifdef __DEBUG_INTERACTIVE_MODE__
                fgetc(stdin);
#endif
		exit(1);
	}

	sleep_time = *sleeping_time;
	for(i = 0; i < total_sensor_num; i++) //for
	{
		pSensor = S->list[i];

		//pSensor->info.sleeping_interval = sleeping_time;
		
		/* determine sleeping interval according to scan type */
		switch(param->sensor_scan_type) //switch
		{
		case SCAN_TURN_ON_ALL:
		case SCAN_NO_USE:
		case SCAN_NO_USE_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
			pSensor->info.sleeping_interval = *sleeping_time;
			break;
			
		/* adjust sleeping interval considering each sensor's sensing interval */
                /* sleeing_time = movement_time + (scanning_time - sensor_work_time ) */
		case SCAN_CONSTANT_SPEED_WITH_NONOPTIMAL_SLEEPING:
		case SCAN_VARIABLE_SPEED_WITH_NONOPTIMAL_SLEEPING:
		case SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING:
		case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING:
		case SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
		case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
		case SCAN_VARIABLE_SPEED_WITH_VARIABLE_SLEEPING_TIME_AND_WITH_SENSING_HOLE_HANDLING:
		case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_VARIABLE_WORKING_TIME:
		case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING:
		case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING_AND_WITH_SENSING_HOLE_HANDLING:
			pSensor->info.sleeping_interval = MAX(sleep_time - pSensor->info.sensing_interval, 0);
			/*
			pSensor->info.sleeping_interval = sleep_time - pSensor->info.sensing_interval;
			if(pSensor->info.sleeping_interval < 0)
			{
				printf("ComputeSleepingSchedule(): pSensor->info.sleeping_interval(%f) < 0\n", (float) *sleeping_time);
				exit(1);
			}
			*/
			break;

		default:
			printf("ComputeSleepingSchedule(): sensor_scan_type(%d) is unknown\n", param->sensor_scan_type);
			break;
		} //end of switch
	} //end of for

	/* adjust sleeping interval considering sensor's sensing interval for scan types using virtual scanning */
	switch(param->sensor_scan_type) //switch
	{
		/* adjust sleeping interval considering each sensor's sensing interval */
                /* sleeing_time = movement_time + (scanning_time - sensor_work_time ) */
		case SCAN_CONSTANT_SPEED_WITH_NONOPTIMAL_SLEEPING:
		case SCAN_VARIABLE_SPEED_WITH_NONOPTIMAL_SLEEPING:
		case SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING:
		case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING:
		case SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
		case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
	        case SCAN_VARIABLE_SPEED_WITH_VARIABLE_SLEEPING_TIME_AND_WITH_SENSING_HOLE_HANDLING:
		case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_VARIABLE_WORKING_TIME:
		case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING:
		case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING_AND_WITH_SENSING_HOLE_HANDLING:
			*sleeping_time = MAX(sleep_time - param->sensor_work_time, 0);
			break;

		default:
			break;
		
	}//end of switch

/* 	/\** Compute movement time over the physically shortest path for the initial sleeping time for sensors *\/ */
/* 	/\* 1. compute the length of the shortest path from Enter nodes to Exit nodes along with source and destination *\/ */
/* 	shortest_path_length = FindShortestPath(src_table, dst_table, G_size, D_move, shortest_path_src, shortest_path_dst); */

/* 	/\* 2. compute the sleeping time as the travel time for the shortest path by dividing the distance by the maximum vehicle speed *\/ */
/* 	*movement_time = shortest_path_length/v_max; */
}


double GetEarliestScanArrivalTimeForSources(struct_traffic_table *src_table, schedule_table_t *T)
{ //scanning_time is chosen as the minimum scan arrival time among scan arrival times for sources (entrances)
	int source_num = src_table->number; //number of sources (i.e., entrances)
	int i, j; //indices of for-loop
	double min_arrival_time = INF; //minimum arrival time of scan for sources
	double arrival_time = INF; //arrival time of scan for source
	char *src = NULL; //source node name
	schedule_table_node_t *pTableNode = NULL; //pointer to table node
	enum_endpoint_t type = ENDPOINT_UNKNOWN; //endpoint type for vertices in an edge

	for(i = 0; i < source_num; i++) //for-1
	{
		//src = src_table->list[i];
		src = src_table->list[i].vertex;
		pTableNode = &(T->head);
		arrival_time = INF;
		for(j = 0; j < T->size; j++) //for-2
		{
			pTableNode = pTableNode->next;

			if(strcmp(src, pTableNode->tail_node) == 0)
				type = ENDPOINT_TAIL;
			else if(strcmp(src, pTableNode->head_node) == 0)
				type = ENDPOINT_HEAD;
			else
				type = ENDPOINT_UNKNOWN;

			if(type == ENDPOINT_TAIL || type == ENDPOINT_HEAD) //if-1
			{

				switch(pTableNode->status) //switch
				{
				case STATUS_FORWARD:
					if(type == ENDPOINT_TAIL)
						arrival_time = pTableNode->arrival_time1;
					else
						arrival_time = pTableNode->departure_time1;
					break;

				case STATUS_BACKWARD:
					if(type == ENDPOINT_TAIL)
						arrival_time = pTableNode->departure_time2;
					else
						arrival_time = pTableNode->arrival_time2;
					break;

				case STATUS_OVERLAP:
					if(type == ENDPOINT_TAIL)
						arrival_time = pTableNode->arrival_time1;
					else
						arrival_time = pTableNode->arrival_time2;
					break;
				} //end of switch

				/* update min_arrival_time */
				if(min_arrival_time > arrival_time) //if-2
					min_arrival_time = arrival_time;

				break;
			} //end of if-1
		} //end of for-2
	} //end of for-1

	return min_arrival_time;
}

boolean UpdateScheduleTableForSensorDeath(schedule_table_node_t *pTableNode, sensor_queue_node_t *pSensorNode)
{ //update schedule table T along with sensor_list by updating sensors' offsets
	sensor_queue_node_t *ptr_queue_node = NULL; //pointer to sensor queue node
	int order = 0; //order for live sensors in sensor list
	int count = 0; //count for reading as many as sensor_list.size
	int old_live_sensor_number = pTableNode->sensor_list.live_sensor_number; //number of live sensors before the death of the sensor pointed by pSensorNode
	int expected_live_sensor_number = old_live_sensor_number - 1; //expected number of live sensors before the death of the sensor pointed by pSensorNode

	ptr_queue_node = pTableNode->sensor_list.head.next;
	order = 0;
	pTableNode->sensor_list.live_sensor_number = 0; //reset the number of live sensors
	while(count < pTableNode->sensor_list.size)
	{
		/* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		if(ptr_queue_node->info.state != SENSOR_DIE)
		{
			ptr_queue_node->order_for_live_sensors = order++;
			pTableNode->sensor_list.live_sensor_number++;
		}
		else
			ptr_queue_node->order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		ptr_queue_node = ptr_queue_node->next;
		count++;
	}

#ifdef __DEBUG__
	printf("expected live_sensor_number=%d and actual live_sensor_number=%d\n", expected_live_sensor_number, pTableNode->sensor_list.live_sensor_number);
#endif

	return TRUE;
}

void ComputeSurveillanceSchedule(parameter_t *param, schedule_table_t *T, struct_sensor_table *S, struct_traffic_table *src_table, struct_traffic_table *dst_table, struct_graph_node *G, int G_size, double **D_move, int **M_move, int **D_scan, int **M_scan, double *movement_time, double *scanning_time, double *sleeping_time)
{ //perform the schedule for surveillance such as sensing schedule and sleeping schedule

	/** construct schedule table T for scan-scheduling for intersection nodes with param and graph_file */
	if(param->sensor_scan_type != SCAN_TURN_ON_ALL && param->sensor_scan_type != SCAN_NO_USE && param->sensor_scan_type != SCAN_NO_USE_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING)
		ConstructScheduleTable(T, param, G, G_size, src_table, dst_table);

	/** compute sensing schedule for each sensor node using schedule table T and sensor table S */
	ComputeSensingSchedule(param, T, S);

	/** compute sleeping schedule for each sensor node using schedule table T and sensor table S */
	ComputeSleepingSchedule(param, T, S, src_table, dst_table, G_size, D_move, M_move, D_scan, M_scan, movement_time, scanning_time, sleeping_time);
}

void UpdateSurveillanceSchedule(parameter_t *param, schedule_table_t *T, struct_sensor_table *S, struct_traffic_table *src_table, struct_traffic_table *dst_table, struct_graph_node *G, int G_size, double **D_move, int **M_move, int **D_scan, int **M_scan, double *movement_time, double *scanning_time, double *sleeping_time)
{ //update the schedule for surveillance such as sensing schedule and sleeping schedule
	/** perform scheduling with traffic source table and traffic destination table to allow the maximum sleeping time */
	ComputeSurveillanceSchedule(param, T, S, src_table, dst_table, G, G_size, D_move, M_move, D_scan, M_scan, movement_time, scanning_time, sleeping_time);
}

void PerformSensingSchedule(parameter_t *param, struct_sensor_table *S, STATE state, double current_time, int dying_sensor_id)
{ //perform the sensing schedule to schedule sensors for surveillance
	double delay = 0.0; //delay for simulation schedule
	int i;
        double working_time = 0; //sensor's working time before this reschedule
	double energy_consumption = 0; //sensor's energy consumed for working_time

	double min_sensor_lifetime = INF; //minimum sensor lifetime
	int min_sensor_id = -1; //sensor id with minimum lifetime 
	double min_sensor_delay = 0; //delay for the event for min_sensor_id

	for(i = 0; i < S->number; i++) //for-1
	{
	        if(state == SENSOR_BORN) //if-1
		{
		        if(param->sensor_schedule_mode == SENSOR_SCHEDULE_MODE_LAZY_UPDATE) //if-2
			{
				//@for debugging
				//if((i == (491-1)) || (i == (1262-1)) || (i == (1399-1)) || (i == (1401-1)))
			        //if(i == (491-1))
				//  printf("sensor(id=%d) is checkd\n", i+1);
				////////////////

				S->list[i]->info.birth_time = current_time; //the birth time of a sensor
				S->list[i]->info.state = SENSOR_ESTIMATE;
				/* sensor starts sensing after the sleeping time for the vehicle movement over the shortest path (i.e., param->vehicle_initial_arrival_time) plus its sensing starting time considering the simulation starting time of current_time */
				delay = (param->vehicle_initial_arrival_time - current_time) + S->list[i]->info.relative_sensing_start_time;

				/* set initial sleeping interval before the first scheduling */
				S->list[i]->info.initial_sleeping_interval = delay;

				/* set sleeping_start_time and sleeping_end_time with delay; 
				   Note that our scheduling starts from sleeping period */
				S->list[i]->info.sleeping_start_time = current_time;
				S->list[i]->info.sleeping_end_time = current_time + delay;
				
				/* set the surveillance start/restart time 1*/
				S->list[i]->info.surveillance_start_time = current_time;
				S->list[i]->info.surveillance_restart_time = current_time;

				/* For vehicle detection, we update sensing_start_time and sensing_end_time such that sensing_start_time is 
				   set to the ending time of sleeping of delay */
				/*
				S->list[i]->info.sensing_start_time = current_time + delay;
				S->list[i]->info.sensing_end_time = current_time + delay + S->list[i]->info.sensing_interval;
				*/

                                /* schedule the event of SENSOR_DIE with the lifetime corresponding to the remaining energy budget */
				delay = get_sensor_lifetime(&(S->list[i]->info), param);
                                //get sensor lifetime based on its energy budget along with its schedule starting from the sleeping time corresponding to the movement time over the shortest path

				//@for debugging
			        //if(i == (491-1))
				//{
				//  printf("sensor(id=%d) will die at time %f\n", i+1, (float)(delay+current_time));
				//  fgetc(stdin);
				//}
				////////////////

				/* set up the flag and time for the handling of state SENSOR_DIE */
				S->list[i]->info.flag_for_state_sensor_die = FALSE;
				S->list[i]->info.time_for_state_sensor_die = current_time + delay;

				/* For vehicle detection, we update the last sensing_start_time and sensing_end_time just before the death time such that sensing_start_time is set to the ending time of sleeping of delay */
				S->list[i]->info.sensing_start_time = S->list[i]->info.time_for_state_sensor_die - S->list[i]->info.sensing_interval;
				S->list[i]->info.sensing_end_time = S->list[i]->info.time_for_state_sensor_die;

				//schedule(SENSOR_DIE, delay, i+1); //moved to the bottom of this function
				
				if(S->list[i]->info.time_for_state_sensor_die < min_sensor_lifetime) //if-3
				{
				  min_sensor_lifetime = S->list[i]->info.time_for_state_sensor_die;
				  //min_sensor_id = i+1;
                                  min_sensor_id = S->list[i]->info.id;
                                  min_sensor_delay = delay;
				} //end of if-3		
			} //end of if-2
			else //sensor_schedule_mode=SENSOR_SCHEDULE_MODE_EAGER_UPDATE //else-2
			{
			  delay = current_time;
			  schedule(state, delay, S->list[i]->info.id);
			} //end of else-2
		} //end of if-1
		else if(state == SENSOR_RESCHEDULE) //end of else-if-1
		{
		        if(param->sensor_schedule_mode == SENSOR_SCHEDULE_MODE_LAZY_UPDATE) //if-4
		        {
				//@for debugging			        
			        //if(dying_sensor_id == 1260 && current_time > 108312 && i == (630-1))
				//if(current_time > 108270 && i == (1260-1))
				//  printf("sensor(id=%d) is checked\n", i+1);
				////////////////

			        /* perform reschedule for live sensors */
				if(S->list[i]->info.state == SENSOR_DIE)
				  continue;

				S->list[i]->info.reschedule_time = current_time; //the birth time of a sensor
				//S->list[i]->info.state = SENSOR_ESTIMATE;
				S->list[i]->info.reschedule_flag = FALSE; //turn reschedule flag off

				/* adjust the sensor energy budget by the energy consumed by the number of duty cycles from the last restart time to current_time */
				adjust_sensor_energy_budget(current_time, &(S->list[i]->info));

				/* sensing start time is reset by sensing schedule and is set to the 
				relative sensing start time from the beginning of virtual scanning */
				delay = S->list[i]->info.relative_sensing_start_time; 

				/* set initial sleeping interval after rescheduling */
				S->list[i]->info.initial_sleeping_interval = delay;

				/* set surveillance restart time */
				S->list[i]->info.surveillance_restart_time = current_time;

				/* For vehicle detection, we update sensing_start_time and sensing_end_time such that sensing_start_time is 
				set to the ending time of sleeping of delay */
				/*
				S->list[i]->info.sensing_start_time = current_time + delay;
				S->list[i]->info.sensing_end_time = current_time + delay + S->list[i]->info.sensing_interval;
				*/

				/* schedule the event of SENSOR_DIE with the lifetime corresponding to the remaining energy budget */
                                delay = get_sensor_lifetime(&(S->list[i]->info), param);
                                //get sensor lifetime based on its energy budget along with its schedule starting from the sleeping time corresponding to the movement time over the shortest path
				//@NOTE that sensor of id (i+1) can live longer than its original schedule since it can skip its duty cycle due to the reschedule.

				/* set up the flag and time for the handling of state SENSOR_DIE */
				//S->list[i]->info.flag_for_state_sensor_die = FALSE; //this is set to FALSE in state SENSOR_BORN in main.c.
				S->list[i]->info.time_for_state_sensor_die = current_time + delay;

				/* For vehicle detection, we update the last sensing_start_time and sensing_end_time just before the death time such that sensing_start_time is set to the ending time of sleeping of delay */
				S->list[i]->info.sensing_start_time = S->list[i]->info.time_for_state_sensor_die - S->list[i]->info.sensing_interval;
				S->list[i]->info.sensing_end_time = S->list[i]->info.time_for_state_sensor_die;

				if((S->list[i]->info.id != dying_sensor_id) && (S->list[i]->info.time_for_state_sensor_die < min_sensor_lifetime)) //if-7
				{
				  min_sensor_lifetime = S->list[i]->info.time_for_state_sensor_die;
				  //min_sensor_id = i+1;
				  min_sensor_id = S->list[i]->info.id;
				  min_sensor_delay = delay;
				} //end of if-7

				//schedule(SENSOR_DIE, delay, S->list[i]->info.id);
				//if(S->list[i]->info.id != dying_sensor_id) //this reschedule is caused by this dying sensor
				  //schedule(SENSOR_DIE, delay, S->list[i]->info.id);
			} //end of if-4
			else //sensor_schedule_mode=SENSOR_SCHEDULE_MODE_EAGER_UPDATE //else-4
			{
                                /* reduce the energy consumed before this reschedule is issued. This allows the sensor with state SENSOR_SENSE to adjust its energy for one reschedule event even though multiple reschedule events happen at the same time. */
		                if(S->list[i]->info.state == SENSOR_SENSE) //if-5
				{
				  /* calculate energy_consumption considering sensor's working time and sensing range */
				  working_time = current_time - S->list[i]->info.sensing_start_time_for_reschedule;
				  energy_consumption = estimate_energy_consumption(working_time, S->list[i]->info.sensing_range, S->list[i]->info.energy_consumption_rate);
				  S->list[i]->info.energy -= energy_consumption;
				} //end of if-5

				/* perform reschedule for live sensors */
				if(S->list[i]->info.state == SENSOR_DIE)
				  continue;

				/* set reschedule information for sensor */
				// S->list[i]->info.reschedule_flag = TRUE;
				S->list[i]->info.state_before_reschedule = S->list[i]->info.state;
				S->list[i]->info.state = state;
				S->list[i]->info.reschedule_time = current_time; //the birth time of a sensor
				S->list[i]->info.reschedule_flag = FALSE; //turn reschedule flag off
				S->list[i]->info.state = SENSOR_ESTIMATE;

				/* sensing start time is reset by sensing schedule and is set to the 
				relative sensing start time from the beginning of virtual scanning */
				delay = S->list[i]->info.relative_sensing_start_time; 

				/* set initial sleeping interval after rescheduling */
				S->list[i]->info.initial_sleeping_interval = delay;

				/* set surveillance restart time */
				S->list[i]->info.surveillance_restart_time = current_time;

				/* For vehicle detection, we update sensing_start_time and sensing_end_time such that sensing_start_time is 
				set to the ending time of sleeping of delay */
				S->list[i]->info.sensing_start_time = current_time + delay;
				S->list[i]->info.sensing_end_time = current_time + delay + S->list[i]->info.sensing_interval;

				/* set up the flag and time for the handling of state SENSOR_ESTIMATE */
				S->list[i]->info.flag_for_state_sensor_estimate = FALSE;
				S->list[i]->info.time_for_state_sensor_estimate = current_time + delay;

				schedule(SENSOR_ESTIMATE, delay, S->list[i]->info.id);
		        } //end of else-4
		} //end of else-if-1
		else //else-1
		{
			printf("PerformSensingSchedule(): we cannot perform sensing scheduling in state()\n", state);
#ifdef __DEBUG_INTERACTIVE_MODE__
                        fgetc(stdin);
#endif
			exit(1);
		} //else-2
	}

	/** schedule an earliest event when param->sensor_schedule_mode is SENSOR_SCHEDULE_MODE_LAZY_UPDATE */       
	if(param->sensor_schedule_mode == SENSOR_SCHEDULE_MODE_LAZY_UPDATE)
	{
	        if(state == SENSOR_BORN || state == SENSOR_RESCHEDULE)
		{
		  //S->list[min_sensor_id-1]->info.state = SENSOR_DIE;
		  schedule(SENSOR_DIE, min_sensor_delay, min_sensor_id);

		  //@for debugging
		  //printf("[%f] The dead sensor id is %d and the next dead sensor id is %d after time %f\n\n", (float)current_time, dying_sensor_id, min_sensor_id, (float)min_sensor_delay);
		  /////////////////
		}
	} 
}

/* void UpdateSensorState(parameter_t *param, struct_sensor_table *S, double current_time) */
/* { //update sensors' states with their energy consumption by current_time */
/* 	double delay = 0.0; //delay for simulation schedule */
/* 	int i; */
/*         double working_time = 0; //sensor's working time before this reschedule */
/* 	double energy_consumption = 0; //sensor's energy consumed for working_time */
/* 	STATE state = 0; */

/* 	for(i = 0; i < S->number; i++) */
/* 	{ */
/* 		if(state == SENSOR_BORN) */
/* 		{ */
/* 		        delay = current_time; */
/* 			schedule(state, delay, S->list[i]->info.id); */
/* 		} */
/* 		else if(state == SENSOR_RESCHEDULE) */
/* 		{ */
/*                         /\* reduce the energy consumed before this reschedule is issued. This allows the sensor with state SENSOR_SENSE to adjust its energy for one reschedule event even though multiple reschedule events happen at the same time. *\/ */
/* 		        if(S->list[i]->info.state == SENSOR_SENSE)  */
/* 			{ */
/*                                 /\* calculate energy_consumption considering sensor's working time and sensing range *\/ */
/*                                 working_time = current_time - S->list[i]->info.sensing_start_time_for_reschedule; */
/*                                 energy_consumption = estimate_energy_consumption(working_time, S->list[i]->info.sensing_range, S->list[i]->info.energy_consumption_rate); */
/*                                 S->list[i]->info.energy -= energy_consumption; */
/* 			} */

/* 			/\* perform reschedule for live sensors *\/ */
/* 		        if(S->list[i]->info.state != SENSOR_DIE) //if-1 */
/* 			{ */
/*                                 /\* set reschedule information for sensor *\/ */
/* 				S->list[i]->info.reschedule_flag = TRUE; */
/* 				S->list[i]->info.state_before_reschedule = S->list[i]->info.state; */
/* 				S->list[i]->info.state = state; */
/* 				//schedule(state, delay, S->list[i]->info.id); */

/* 				if(param->sensor_schedule_mode == SENSOR_SCHEDULE_MODE_EAGER_UPDATE) //if-2 */
/* 				{ */
/* 				  S->list[i]->info.reschedule_time = current_time; //the birth time of a sensor */
/* 				  S->list[i]->info.state = SENSOR_ESTIMATE; */
/* 				  S->list[i]->info.reschedule_flag = FALSE; //turn reschedule flag off */

/* 				  /\* sensing start time is reset by sensing schedule and is set to the  */
/* 				     relative sensing start time from the beginning of virtual scanning *\/ */
/* 				  delay = S->list[i]->info.relative_sensing_start_time;  */

/* 				  /\* set initial sleeping interval after rescheduling *\/ */
/* 				  S->list[i]->info.initial_sleeping_interval = delay; */

/* 				  /\* set surveillance restart time *\/ */
/* 				  S->list[i]->info.surveillance_restart_time = current_time; */

/* 				  /\* For vehicle detection, we update sensing_start_time and sensing_end_time such that sensing_start_time is  */
/* 				     set to the ending time of sleeping of delay *\/ */
/* 				  S->list[i]->info.sensing_start_time = current_time + delay; */
/* 				  S->list[i]->info.sensing_end_time = current_time + delay + S->list[i]->info.sensing_interval; */

/* 				  /\* set up the flag and time for the handling of state SENSOR_ESTIMATE *\/ */
/* 				  S->list[i]->info.flag_for_state_sensor_estimate = FALSE; */
/* 				  S->list[i]->info.time_for_state_sensor_estimate = current_time + delay; */

/* 				  schedule(SENSOR_ESTIMATE, delay, S->list[i]->info.id); */
/* 				} //end of if-2 */
/* 				else if(param->sensor_schedule_mode == SENSOR_SCHEDULE_MODE_LAZY_UPDATE) //else if-3 */
/* 				{ */
/* 				  S->list[i]->info.reschedule_time = current_time; //the birth time of a sensor */
/* 				  S->list[i]->info.state = SENSOR_ESTIMATE; */
/* 				  S->list[i]->info.reschedule_flag = FALSE; //turn reschedule flag off */

/* 				  /\* adjust the sensor energy budget by the energy consumed by the number of duty cycles from the last restart time to current_time *\/ */
/* 				  adjust_sensor_energy_budget(current_time, &(S->list[i]->info)); */

/* 				  /\* sensing start time is reset by sensing schedule and is set to the  */
/* 				     relative sensing start time from the beginning of virtual scanning *\/ */
/* 				  delay = S->list[i]->info.relative_sensing_start_time;  */

/* 				  /\* set initial sleeping interval after rescheduling *\/ */
/* 				  S->list[i]->info.initial_sleeping_interval = delay; */

/* 				  /\* set surveillance restart time *\/ */
/* 				  S->list[i]->info.surveillance_restart_time = current_time; */

/* 				  /\* For vehicle detection, we update sensing_start_time and sensing_end_time such that sensing_start_time is  */
/* 				     set to the ending time of sleeping of delay *\/ */
/* 				  S->list[i]->info.sensing_start_time = current_time + delay; */
/* 				  S->list[i]->info.sensing_end_time = current_time + delay + S->list[i]->info.sensing_interval; */

/* 				  /\* schedule the event of SENSOR_DIE with the lifetime corresponding to the remaining energy budget *\/ */
/*                                   delay = get_sensor_lifetime(&(S->list[i]->info), param); */
/*                                   //get sensor lifetime based on its energy budget along with its schedule starting from the sleeping time corresponding to the movement time over the shortest path */

/* 				  /\* set up the flag and time for the handling of state SENSOR_DIE *\/ */
/* 				  S->list[i]->info.flag_for_state_sensor_die = FALSE; */
/* 				  S->list[i]->info.time_for_state_sensor_die = current_time + delay; */

/* 				  schedule(SENSOR_DIE, delay, S->list[i]->info.id); */
/* 				} //end of else if-3 */
/* 			} //end of if-1 */
/* 		} */
/* 		else */
/* 		{ */
/* 			printf("PerformSensingSchedule(): we cannot perform sensing scheduling in state()\n", state); */
/* #ifdef __DEBUG_INTERACTIVE_MODE__ */
/*                         fgetc(stdin); */
/* #endif */
/* 			exit(1); */
/* 		} */
/* 	} */
/*}*/

void TakeActionForSensingHole(parameter_t *param, struct_graph_node **Gv, int *Gv_size, schedule_table_t *T, edge_queue_t *Er, struct_sensor_table *S, schedule_table_node_t *pTableNode, double left_hole_offset, double right_hole_offset, double ***Dv_move, int ***Mv_move, int *matrix_size_for_movement_in_Gv, int ***Dv_scan, int ***Mv_scan, int *matrix_size_for_scanning_in_Gv, struct_traffic_table *src_table_for_Gr, struct_traffic_table *dst_table_for_Gr, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *hole_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time, double current_time, int dying_sensor_id)
{ /* take action for sensing hole; update virtual graph and schedule table, 
     compute sensing schedule and sleeping schedule, and perform reschedule for sensors. */

	struct_traffic_table hole_table_for_new_holes; //table for candidate nodes for either traffic sources or traffic destinations in virtual graph Gv for sensing scheduling
	struct_traffic_table hole_table_for_deleted_holes; //table for nodes for deleted holes from the hole set (i.e., hole_table_for_Gv) due to the hole mergence
	struct_traffic_table *src_or_dst_table = NULL; //pointer to traffic table containing holes to be labeled
	struct_traffic_table *src_table = NULL; //pointer to source traffic table
	struct_traffic_table *dst_table = NULL; //pointer to destination traffic table
	boolean flag = FALSE; //flag to determine whether to proceed with hole handling or not

	/** initialize traffic table for source/destination nodes (or enter/exit nodes) */
	InitTrafficTable(&hole_table_for_new_holes);

	/** initialize traffic table for deleted holes */
	InitTrafficTable(&hole_table_for_deleted_holes);

	/** update virtual graph Gv, schedule table T, and edge queue Er using pTableNode, left_hole_offset and right_hole_offset; update schedule table T along with sensor_list by updating sensors' offsets */

	flag = Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(Gv, Gv_size, T, pTableNode, left_hole_offset, right_hole_offset, Er, &hole_table_for_new_holes, &hole_table_for_deleted_holes);
	//if(flag == FALSE)
	//  return;
       	
	//UpdateScheduleTableAndEdgeQueueForSensingHole(T, Er, pTableNode, *Gv, *Gv_size, old_Gv_size, left_hole_offset, right_hole_offset, &hole_table_for_new_holes);

	/** set src_table, dst_table and src_or_dst_table according to hole handling mode */
	if(param->sensor_hole_handling_mode == HOLE_MODE_RESHUFFLE_LABELING)
	{
	  /* merge src_or_dst_table_for_Gv including sensing holes to hole_table_for_Gv */
	  MergeTrafficTable(hole_table_for_Gv, &hole_table_for_new_holes);

	  /* delete hole endpoint nodes in hole_table_for_deleted_holes from hole_table_for_Gv */
	  SubtractTrafficTable(hole_table_for_Gv, &hole_table_for_deleted_holes);

	  /* delete the tables of src_table_for_Gv and dst_table_for_Gv, setting them to src_table_for_Gr and dst_table_for_Gr, respectively */
	  CopyTrafficTable(src_table_for_Gv, src_table_for_Gr);
	  CopyTrafficTable(dst_table_for_Gv, dst_table_for_Gr);

	  /* set table pointers */
	  src_or_dst_table = hole_table_for_Gv;
	  src_table = src_table_for_Gv;
	  dst_table = dst_table_for_Gv;
	}
	else //HOLE_MODE_INCREMENTAL_LABELING
	{
	  /* delete hole endpoint nodes in hole_table_for_deleted_holes from src_table_for_Gv */
	  SubtractTrafficTable(src_table_for_Gv, &hole_table_for_deleted_holes);

	  /* delete hole endpoint nodes in hole_table_for_deleted_holes from dst_table_for_Gv */
	  SubtractTrafficTable(dst_table_for_Gv, &hole_table_for_deleted_holes);

	  /* set table pointers */
	  src_or_dst_table = &hole_table_for_new_holes;
	  src_table = src_table_for_Gv;
	  dst_table = dst_table_for_Gv;
	}

	/** update the shortest path weight matrix Dv_move and predecessor matrix Mv_move in terms of vehicle movement */
	Floyd_Warshall_Construct_Matrices_For_Movement(*Gv, *Gv_size, Dv_move, Mv_move, matrix_size_for_movement_in_Gv);
	
	/** update the shortest path weight matrix Dv_scan and predecessor matrix Mv_scan in terms of
	the number of sensors for sensor scanning; Dv_scan will be used for computing sleeping time 
	in sensor scheduling */
	Floyd_Warshall_Construct_Matrices_For_Scanning(*Gv, *Gv_size, Dv_scan, Mv_scan, matrix_size_for_scanning_in_Gv);

	/** get an optimal labeling of sensing holes that assigns each sensing hole a role of either 
        entrance node or protection node using the exhaustive search or MST-based searching and then
        update the schedule for surveillance such as sensing schedule and sleeping schedule */	

	switch(param->sensor_hole_handling_algorithm)
	{
	case HOLE_HANDLING_NO_HANDLING: //no handling
		break;

	case HOLE_HANDLING_EXHAUSTIVE_SEARCH_ALGORITHM: //exhaustive search algorithm
		HandleSensingHoles_With_ExhaustiveSearchAlgorithm(param, *Gv, *Gv_size, T, S, *Dv_move, *Mv_move, *Dv_scan, *Mv_scan, src_table, dst_table, src_or_dst_table, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
		break;

	case HOLE_HANDLING_GREEDY_ALGORITHM_BASED_MINIMAL_SPANNING_TREE: //greedy algorithm based on minimal spanning tree
		HandleSensingHoles_With_MinimalSpanningTreeAlgorithm(param, *Gv, *Gv_size, T, S, *Dv_move, *Mv_move, *Dv_scan, *Mv_scan, src_table, dst_table, src_or_dst_table, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
		break;

	case HOLE_HANDLING_GREEDY_ALGORITHM_BASED_HEURISTICS:

		break;

	case HOLE_HANDLING_ALL_ENTRANCE_POINTS:
		HandleSensingHoles_With_AllEntrancePointsAlgorithm(param, *Gv, *Gv_size, T, S, *Dv_move, *Mv_move, *Dv_scan, *Mv_scan, src_table, dst_table, src_or_dst_table, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
		break;

	case HOLE_HANDLING_ALL_PROTECTION_POINTS:
		HandleSensingHoles_With_AllProtectionPointsAlgorithm(param, *Gv, *Gv_size, T, S, *Dv_move, *Mv_move, *Dv_scan, *Mv_scan, src_table, dst_table, src_or_dst_table, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
		break;

	case HOLE_HANDLING_RANDOM_LABELING:
		HandleSensingHoles_With_RandomLabelingAlgorithm(param, *Gv, *Gv_size, T, S, *Dv_move, *Mv_move, *Dv_scan, *Mv_scan, src_table, dst_table, src_or_dst_table, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
		break;
		
	default:
		printf("TakeActionForSensingHole: param->sensor_hole_handling_algorithm(%d) is not supported yet!\n", param->sensor_hole_handling_algorithm);
#ifdef __DEBUG_INTERACTIVE_MODE__
                fgetc(stdin);
#endif
		exit(1);
	}//end of switch

	/** perform the sensing schedule to schedule sensors for surveillance */
	if(param->sensor_hole_handling_algorithm != HOLE_HANDLING_NO_HANDLING)
	  PerformSensingSchedule(param, S, SENSOR_RESCHEDULE, current_time, dying_sensor_id);

	Free_Traffic_Table(&hole_table_for_new_holes); //release the memory occupied by the traffic source/destination table for the new holes in virtual graph Gv

	Free_Traffic_Table(&hole_table_for_deleted_holes); //release the memory occupied by the table for the deleted holes in Gv
}

void TakeActionForSensorDeath(parameter_t *param, struct_graph_node *Gv, int Gv_size, schedule_table_t *T, struct_sensor_table *S, schedule_table_node_t *pTableNode, sensor_queue_node_t *pSensorNode, double ***Dv_move, int ***Mv_move, int *matrix_size_for_movement_in_Gv, int ***Dv_scan, int ***Mv_scan, int *matrix_size_for_scanning_in_Gv, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time, double current_time, int dying_sensor_id)
{ /* take action for sensor's death in the scheduling based on variable scan speed; update schedule table, 
     compute sensing schedule and sleeping schedule, and perform reschedule for sensors. */

	/** update schedule table T along with sensor_list by updating sensors' offsets */
	UpdateScheduleTableForSensorDeath(pTableNode, pSensorNode);

	switch(param->sensor_scan_type)
	{
	case SCAN_NO_USE_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
	case SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
	case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
	case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING_AND_WITH_SENSING_HOLE_HANDLING:
	case SCAN_VARIABLE_SPEED_WITH_VARIABLE_SLEEPING_TIME_AND_WITH_SENSING_HOLE_HANDLING:

	        /** check the hole handling algorithm type */
	        if(param->sensor_hole_handling_algorithm == HOLE_HANDLING_NO_HANDLING) //no handling
		  break;	    

		/** update the shortest path weight matrix Dv_scan and predecessor matrix Mv_scan in terms of
		the number of sensors for sensor scanning; Dv_scan will be used for computing sleeping time 
		in sensor scheduling */

		Floyd_Warshall_Construct_Matrices_For_Scanning(Gv, Gv_size, Dv_scan, Mv_scan, matrix_size_for_scanning_in_Gv);

		/** compute the schedule for surveillance such as sensing schedule and sleeping schedule */
		ComputeSurveillanceSchedule(param, T, S, src_table_for_Gv, dst_table_for_Gv, Gv, Gv_size, *Dv_move, *Mv_move, *Dv_scan, *Mv_scan, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);

		/** perform the sensing schedule to schedule sensors for surveillance */
		PerformSensingSchedule(param, S, SENSOR_RESCHEDULE, current_time, dying_sensor_id);
		break;

	default:
		break;
	}
}

sensor_queue_node_t* GetSensorQueueNodeFromLocationQueue(location_queue_t *Q, int index, struct_sensor_table *S)
{ //return the sensor queue node pointed by the sensor table entry corresponding to index; the index of the first node is 0.
	location_queue_node_t *p = NULL;
	sensor_queue_node_t *pSensorNode = NULL; //pointer to sensor queue node corresponding to the location of Q's index
	int size = Q->size; //size of queue Q, i.e., the number of valid queue nodes
	int i; //for-loop index

	if(index >= Q->size)
	{
		printf("GetSensorQueueNodeFromLocationQueue(): Error: index(%d) >= Q->size(%d)\n", index, Q->size);
#ifdef __DEBUG_INTERACTIVE_MODE__
                fgetc(stdin);
#endif
		exit(1);
	}

	for(i = 0, p = &(Q->head); i <= index; i++)
		p = p->next;

	pSensorNode = S->list[p->id - 1];

	return pSensorNode;
}

int MakeSurveillanceScheduleWithInitialSensingHoles(parameter_t *param, struct_graph_node **Gv, int *Gv_size, schedule_table_t *T, struct_sensor_table *S, double ***Dv_move, int ***Mv_move, int *matrix_size_for_movement_in_Gv, int ***Dv_scan, int ***Mv_scan, int *matrix_size_for_scanning_in_Gv, struct_traffic_table *src_table_for_Gr, struct_traffic_table *dst_table_for_Gr, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *hole_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time, edge_queue_t *Er)
{ /* make surveillance schedule by handling initial sensing holes by determining the roles of sensing holes as virtual entrances or virtual exits */
	int total_hole_segment_num = 0; //number of initial sensing hole segments in virtual graph Gv

	/** find initial sensing hole segments in virtual graph that is initially equal to real graph */
	total_hole_segment_num = FindSensingHoles(T, S, Er);

	/** handle the initial sensing holes that are represented as hole segments with two end-points */
	if(total_hole_segment_num > 0)
	{
		switch(param->sensor_initial_hole_handling_algorithm)
		{
			case HOLE_HANDLING_EXHAUSTIVE_SEARCH_ALGORITHM:
			case HOLE_HANDLING_GREEDY_ALGORITHM_BASED_MINIMAL_SPANNING_TREE:
			case HOLE_HANDLING_GREEDY_ALGORITHM_BASED_HEURISTICS:
			case HOLE_HANDLING_ALL_ENTRANCE_POINTS:
			case HOLE_HANDLING_ALL_PROTECTION_POINTS:
			case HOLE_HANDLING_RANDOM_LABELING:
				HandleInitialSensingHoles(Er, param, Gv, Gv_size, T, S, Dv_move, Mv_move, matrix_size_for_movement_in_Gv, Dv_scan, Mv_scan, matrix_size_for_scanning_in_Gv, src_table_for_Gr, dst_table_for_Gr, src_table_for_Gv, dst_table_for_Gv, hole_table_for_Gv, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);

				break;

			case HOLE_HANDLING_NO_HANDLING:
				/** construct the shortest path weight matrix Dv_move and predecessor matrix Mv_move in terms of vehicle movement */
				Floyd_Warshall_Construct_Matrices_For_Movement(*Gv, *Gv_size, Dv_move, Mv_move, matrix_size_for_movement_in_Gv);
	
				/** construct the shortest path weight matrix Dv_scan and predecessor matrix Mv_scan in terms of
					the number of sensors for sensor scanning; Dv_scan will be used for computing sleeping time 
					in sensor scheduling */
				Floyd_Warshall_Construct_Matrices_For_Scanning(*Gv, *Gv_size, Dv_scan, Mv_scan, matrix_size_for_scanning_in_Gv);

				/** perform the schedule for surveillance such as sensing schedule and sleeping schedule
					1. construct schedule table T for scan-scheduling for intersection nodes with param and graph_file.
					2. compute sensing schedule for each sensor node using schedule table T and sensor table S.
					3. compute sleeping schedule for each sensor node using schedule table T and sensor table S. */
				ComputeSurveillanceSchedule(param, T, S, src_table_for_Gv, dst_table_for_Gv, *Gv, *Gv_size, *Dv_move, *Mv_move, *Dv_scan, *Mv_scan, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
				break;

			default:
				printf("MakeSurveillanceScheduleWithInitialSensingHoles(): param->sensor_initial_hole_handling_algorithm(%d) is not supported!\n", param->sensor_initial_hole_handling_algorithm);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                fgetc(stdin);
#endif
				exit(1);
		}		
	}
	else
	{
		/** construct the shortest path weight matrix Dv_move and predecessor matrix Mv_move in terms of vehicle movement */
		Floyd_Warshall_Construct_Matrices_For_Movement(*Gv, *Gv_size, Dv_move, Mv_move, matrix_size_for_movement_in_Gv);
	
		/** construct the shortest path weight matrix Dv_scan and predecessor matrix Mv_scan in terms of
			the number of sensors for sensor scanning; Dv_scan will be used for computing sleeping time 
			in sensor scheduling */
		Floyd_Warshall_Construct_Matrices_For_Scanning(*Gv, *Gv_size, Dv_scan, Mv_scan, matrix_size_for_scanning_in_Gv);

		/** perform the schedule for surveillance such as sensing schedule and sleeping schedule
			1. construct schedule table T for scan-scheduling for intersection nodes with param and graph_file.
			2. compute sensing schedule for each sensor node using schedule table T and sensor table S.
			3. compute sleeping schedule for each sensor node using schedule table T and sensor table S. */

		ComputeSurveillanceSchedule(param, T, S, src_table_for_Gv, dst_table_for_Gv, *Gv, *Gv_size, *Dv_move, *Mv_move, *Dv_scan, *Mv_scan, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
	}

#ifdef __STORE_FILE_FOR_DATA_STRUCT__
	/** store initial virtual graph including sensing hole endpoints into file */
	Store_Graph_Into_File_As_AdjacencyList(*Gv, *Gv_size, INITIAL_VIRTUAL_GRAPH_FILE, 0);
	//store the adjacency list of graph G into a file in the form of adjacency matrix
#endif

	return total_hole_segment_num;
}

int FindSensingHoles(schedule_table_t *T, struct_sensor_table *S, edge_queue_t *Er)
{ //find initial sensing holes in road network Gr and return H containing sensing holes
	int total_hole_segment_num = 0; //number of initial sensing hole segments in real graph Gr
	int i, j; //indices for for-loops
	schedule_table_node_t *pTableNode = NULL; //pointer to a schedule table node
	edge_queue_node_t *pEdgeNode = NULL; //pointer to an edge queue node
	sensor_queue_node_t *pSensorNode = NULL; //pointer to a sensor queue node
	hole_segment_queue_node_t node; //hole segment queue node
	int n; //number
	boolean result = FALSE; //result of sensing hole checking
	double left_hole_offset, right_hole_offset; //offsets of left end-point and right end-point of a sensing hole segment
	int order = 0; //order of sensing hole segment in an edge

	pTableNode = &(T->head);
	pEdgeNode = &(Er->head);
	for(i = 0; i < T->size; i++) //for-1
	{
		pTableNode = pTableNode->next;
		pEdgeNode = pEdgeNode->next;

		n = pTableNode->sensor_list.size;
		if(n == 0) //there is no sensor on this edge
		{
			left_hole_offset = 0;
			right_hole_offset = pTableNode->weight;
			order = 0;

			memset(&node, 0, sizeof(node));
			node.order = order;
			node.left_hole_offset = left_hole_offset;
			node.right_hole_offset = right_hole_offset;
			Enqueue((queue_t*) &(pEdgeNode->sensing_hole_segment_list), (queue_node_t*) &node); //enqueue hole segment queue node into the sensing hole segment list of the edge			
			total_hole_segment_num++;
		}
		else //else
		{
			pSensorNode = &(pTableNode->sensor_list.head);
			order = 0;
			for(j = 0; j <= n; j++) //for-2: NOTE: we check n+1 places (each sensor's left side and the last sensor's right side)
			{
				if(j < n) 
					pSensorNode = pSensorNode->next; //if j == n (which means that j indicates the head node of the edge), then we use pSensorNode corresponding to the sensor of order n-1
				
				result = is_initial_sensing_hole(j, pSensorNode, pTableNode, &left_hole_offset, &right_hole_offset);
				//check whether or not there is a sensing hole segment to the left of the sensor, but the last sensor also checks whether or not there is a righ hole segment
				if(result == TRUE)
				{
					memset(&node, 0, sizeof(node));
					node.order = order++;
					node.left_hole_offset = left_hole_offset;
					node.right_hole_offset = right_hole_offset;
					Enqueue((queue_t*) &(pEdgeNode->sensing_hole_segment_list), (queue_node_t*) &node); //enqueue hole segment queue node into the sensing hole segment list of the edge
					total_hole_segment_num++;
				}			
			} //end of for-2
		} //end of else
	} //end of for-1

	return total_hole_segment_num;
}

void HandleInitialSensingHoles(edge_queue_t *Er, parameter_t *param, struct_graph_node **Gv, int *Gv_size, schedule_table_t *T, struct_sensor_table *S, double ***Dv_move, int ***Mv_move, int *matrix_size_for_movement_in_Gv, int ***Dv_scan, int ***Mv_scan, int *matrix_size_for_scanning_in_Gv, struct_traffic_table *src_table_for_Gr, struct_traffic_table *dst_table_for_Gr, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *hole_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time)
{ //handle the initial sensing holes that are represented as hole segments with two end-points using the sensing hole handling algorithm

	struct_traffic_table hole_table_for_new_holes; //table for candidate nodes for either traffic sources or traffic destinations in virtual graph Gv for sensing scheduling
	struct_traffic_table hole_table_for_deleted_holes; //table for nodes for deleted holes from the hole set (i.e., hole_table_for_Gv) due to the hole mergence
	struct_traffic_table *src_or_dst_table = NULL; //pointer to traffic table containing holes to be labeled
	struct_traffic_table *src_table = NULL; //pointer to source traffic table
	struct_traffic_table *dst_table = NULL; //pointer to destination traffic table

	double left_hole_offset = -1, right_hole_offset = -1; //offsets of a sensing hole segment in virtual graph Gv
	schedule_table_node_t *pTableNode = NULL; //pointer to a schedule table node
	edge_queue_node_t *pEdgeNode = NULL; //pointer to an edge queue node
	hole_segment_queue_node_t *pHoleSegmentNode = NULL; //pointer to a hole segment queue node
	int i, j; //indices of for-loops
	int eid; //edge id
	FILE *fp = NULL; //pointer to the file that will contain the initial sensing hole information

	/** initialize traffic table for source/destination nodes (or enter/exit nodes) */
	InitTrafficTable(&hole_table_for_new_holes);

	/** initialize traffic table for deleted holes */
	InitTrafficTable(&hole_table_for_deleted_holes);

	/** update virtual graph Gv, edge queue Er, and schedule table T with sensing hole list for each edge */
	pEdgeNode = &(Er->head);
	for(i = 0; i < Er->size; i++)
	{
		pEdgeNode = pEdgeNode->next;
		eid = pEdgeNode->eid;
		pHoleSegmentNode = &(pEdgeNode->sensing_hole_segment_list.head);
		for(j = 0; j < pEdgeNode->sensing_hole_segment_list.size; j++)
		{
			pHoleSegmentNode = pHoleSegmentNode->next;

			/** NOTE: get the pointer to the exact schedule table node linked to the subedge containing the sensing hole segment from the updated schedule table T
			    and get the offsets of the sensing hole segment in the virtual graph Gv */
			pTableNode = GetTableNodeFromSubedgeList(&(pEdgeNode->subedge_list), pHoleSegmentNode, &left_hole_offset, &right_hole_offset);
			if(pTableNode == NULL)
			{ 
			  printf("HandleInitialSensingHoles(): pTableNode is NULL\n");
			  exit(1);
			}

			/** update graph Gv and Gv_size using the sensing hole list of each edge */
			//UpdateVirtualGraph(Gv, Gv_size, T, pTableNode, left_hole_offset, right_hole_offset);

			/** update schedule table T along with sensor_list by updating sensors' offsets */
			//UpdateScheduleTableAndEdgeQueueForSensingHole(T, Er, pTableNode, *Gv, *Gv_size, old_Gv_size, left_hole_offset, right_hole_offset, &hole_table_for_new_holes);

			/** update virtual graph Gv, schedule table T, and edge queue Er using pTableNode, left_hole_offset and right_hole_offset; update schedule table T along with sensor_list by updating sensors' offsets */
			Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(Gv, Gv_size, T, pTableNode, left_hole_offset, right_hole_offset, Er, &hole_table_for_new_holes, &hole_table_for_deleted_holes);
		}
	}

	/** set src_table, dst_table and src_or_dst_table according to hole handling mode */
	if(param->sensor_hole_handling_mode == HOLE_MODE_RESHUFFLE_LABELING)
	{
	  /* merge src_or_dst_table_for_Gv including sensing holes to hole_table_for_Gv */
	  MergeTrafficTable(hole_table_for_Gv, &hole_table_for_new_holes);

	  /* delete hole endpoint nodes in hole_table_for_deleted_holes from hole_table_for_Gv */
	  SubtractTrafficTable(hole_table_for_Gv, &hole_table_for_deleted_holes);

	  /* delete the tables of src_table_for_Gv and dst_table_for_Gv, setting them to src_table_for_Gr and dst_table_for_Gr, respectively */
	  CopyTrafficTable(src_table_for_Gv, src_table_for_Gr);
	  CopyTrafficTable(dst_table_for_Gv, dst_table_for_Gr);

	  /* set table pointers */
	  src_or_dst_table = hole_table_for_Gv;
	  src_table = src_table_for_Gv;
	  dst_table = dst_table_for_Gv;
	}
	else //HOLE_MODE_INCREMENTAL_LABELING
	{
	  /* delete hole endpoint nodes in hole_table_for_deleted_holes from hole_table_for_new_holes */
	  SubtractTrafficTable(&hole_table_for_new_holes, &hole_table_for_deleted_holes);

	  /* set table pointers */
	  src_or_dst_table = &hole_table_for_new_holes;
	  src_table = src_table_for_Gv;
	  dst_table = dst_table_for_Gv;
	}

	/** construct the shortest path weight matrix Dv_move and predecessor matrix Mv_move in terms of vehicle movement */
	Floyd_Warshall_Construct_Matrices_For_Movement(*Gv, *Gv_size,  Dv_move, Mv_move, matrix_size_for_movement_in_Gv);
	
	/** construct the shortest path weight matrix Dv_scan and predecessor matrix Mv_scan in terms of
	the number of sensors for sensor scanning; Dv_scan will be used for computing sleeping time 
	in sensor scheduling */
	Floyd_Warshall_Construct_Matrices_For_Scanning(*Gv, *Gv_size, Dv_scan, Mv_scan, matrix_size_for_scanning_in_Gv);

	/** cluster sensing holes into two clusters of entrance node set and protection node set */
	switch(param->sensor_initial_hole_handling_algorithm)
	//switch(param->sensor_hole_handling_algorithm)
	  {//@ NOTE that for the initial hole handling, we perform MST-based labeling regardless of hole handling algorithm for the fairness of comparison
	case HOLE_HANDLING_NO_HANDLING: //no handling
		HandleSensingHoles_With_MinimalSpanningTreeAlgorithm(param, *Gv, *Gv_size, T, S, *Dv_move, *Mv_move, *Dv_scan, *Mv_scan, src_table, dst_table, src_or_dst_table, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
		break;

	case HOLE_HANDLING_EXHAUSTIVE_SEARCH_ALGORITHM: //exhaustive search algorithm
		HandleSensingHoles_With_ExhaustiveSearchAlgorithm(param, *Gv, *Gv_size, T, S, *Dv_move, *Mv_move, *Dv_scan, *Mv_scan, src_table, dst_table, src_or_dst_table, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
		break;

	case HOLE_HANDLING_GREEDY_ALGORITHM_BASED_MINIMAL_SPANNING_TREE: //greedy algorithm based on minimal spanning tree
		HandleSensingHoles_With_MinimalSpanningTreeAlgorithm(param, *Gv, *Gv_size, T, S, *Dv_move, *Mv_move, *Dv_scan, *Mv_scan, src_table, dst_table, src_or_dst_table, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
		break;

	case HOLE_HANDLING_GREEDY_ALGORITHM_BASED_HEURISTICS:

		break;

	case HOLE_HANDLING_ALL_ENTRANCE_POINTS:
	        //HandleSensingHoles_With_AllEntrancePointsAlgorithm(param, *Gv, *Gv_size, T, S, *Dv_move, *Mv_move, *Dv_scan, *Mv_scan, src_table, dst_table, src_or_dst_table, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
		break;

	case HOLE_HANDLING_ALL_PROTECTION_POINTS:
	        //HandleSensingHoles_With_AllProtectionPointsAlgorithm(param, *Gv, *Gv_size, T, S, *Dv_move, *Mv_move, *Dv_scan, *Mv_scan, src_table, dst_table, src_or_dst_table, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
		HandleSensingHoles_With_MinimalSpanningTreeAlgorithm(param, *Gv, *Gv_size, T, S, *Dv_move, *Mv_move, *Dv_scan, *Mv_scan, src_table, dst_table, src_or_dst_table, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
		break;

	case HOLE_HANDLING_RANDOM_LABELING:
	        //HandleSensingHoles_With_RandomLabelingAlgorithm(param, *Gv, *Gv_size, T, S, *Dv_move, *Mv_move, *Dv_scan, *Mv_scan, src_table, dst_table, src_or_dst_table, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
		HandleSensingHoles_With_MinimalSpanningTreeAlgorithm(param, *Gv, *Gv_size, T, S, *Dv_move, *Mv_move, *Dv_scan, *Mv_scan, src_table, dst_table, src_or_dst_table, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
		break;

	default:
		printf("HandleInitialSensingHoles(): param->sensor_hole_handling_algorithm(%d) is not supported yet!\n", param->sensor_hole_handling_algorithm);
#ifdef __DEBUG_INTERACTIVE_MODE__
                fgetc(stdin);
#endif
		exit(1);
	}//end of switch

	/** release the memory occupied by the traffic source/destination table for the new holes in virtual graph Gv */
	Free_Traffic_Table(&hole_table_for_new_holes);
}

void HandleSensingHoles_With_ExhaustiveSearchAlgorithm(parameter_t *param, struct_graph_node *Gv, int Gv_size, schedule_table_t *T, struct_sensor_table *S, double **Dv_move, int **Mv_move, int **Dv_scan, int **Mv_scan, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *src_or_dst_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time)
{ //handle the sensing holes that are represented as hole segments with two end-points using the exhaustive search algorithm

	/** perform exhaustive search to find an optimal labeling for sensing holes as either entrance node or protection node */
	PerformClusteringWithExhaustiveSearch(param, T, S, src_table_for_Gv, dst_table_for_Gv, src_or_dst_table_for_Gv, Gv, Gv_size, Dv_move, Mv_move, Dv_scan, Mv_scan);

	/** update the schedule for surveillance such as sensing schedule and sleeping schedule */
	UpdateSurveillanceSchedule(param, T, S, src_table_for_Gv, dst_table_for_Gv, Gv, Gv_size, Dv_move, Mv_move, Dv_scan, Mv_scan, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
}

void HandleSensingHoles_With_MinimalSpanningTreeAlgorithm(parameter_t *param, struct_graph_node *Gv, int Gv_size, schedule_table_t *T, struct_sensor_table *S, double **Dv_move, int **Mv_move, int **Dv_scan, int **Mv_scan, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *src_or_dst_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time)
{ //handle the sensing holes that are represented as hole segments with two end-points using the MST-based labeling algorithm
	edge_set_queue_t MST_set; //edge set consisting of a minimal spanning tree for virtual topology Tv where vertices of Tv are the set of source nodes, destination nodes, and sensing hole nodes 

	/** perform the clustering for sensing holes (for labeling of entrance node or protection node)
	    based on Minimal Spanning Tree (MST) algorithm to cluster sensing holes */
	MST_PerformClustering(param, src_table_for_Gv, dst_table_for_Gv, src_or_dst_table_for_Gv, Dv_move, Dv_scan, &MST_set);
	
	/** delete MST_set, which can be used for debugging purpose */
	DestroyQueue((queue_t*) &MST_set);

	/** update the schedule for surveillance such as sensing schedule and sleeping schedule */
	UpdateSurveillanceSchedule(param, T, S, src_table_for_Gv, dst_table_for_Gv, Gv, Gv_size, Dv_move, Mv_move, Dv_scan, Mv_scan, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
}

void HandleSensingHoles_With_RandomLabelingAlgorithm(parameter_t *param, struct_graph_node *Gv, int Gv_size, schedule_table_t *T, struct_sensor_table *S, double **Dv_move, int **Mv_move, int **Dv_scan, int **Mv_scan, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *src_or_dst_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time)
{ //handle the sensing holes that are represented as hole segments with two end-points using the random labeling algorithm

	/** perform the clustering for sensing holes (for labeling of entrance node or protection node) based on random labeling algorithm to cluster sensing holes */
	PerformClusteringWithRandomLabeling(src_table_for_Gv, dst_table_for_Gv, src_or_dst_table_for_Gv);
	
	/** update the schedule for surveillance such as sensing schedule and sleeping schedule */
	UpdateSurveillanceSchedule(param, T, S, src_table_for_Gv, dst_table_for_Gv, Gv, Gv_size, Dv_move, Mv_move, Dv_scan, Mv_scan, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
}

void HandleSensingHoles_With_AllEntrancePointsAlgorithm(parameter_t *param, struct_graph_node *Gv, int Gv_size, schedule_table_t *T, struct_sensor_table *S, double **Dv_move, int **Mv_move, int **Dv_scan, int **Mv_scan, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *src_or_dst_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time)
{ //handle the sensing holes that are represented as hole segments with two end-points labeling all of the holes as entrance points

	/** perform the clustering for sensing holes (for labeling of entrance node or protection node) by labeling all of the holes as entrance points */
	PerformClusteringWithAllEntrancePoints(src_table_for_Gv, dst_table_for_Gv, src_or_dst_table_for_Gv);
	
	/** update the schedule for surveillance such as sensing schedule and sleeping schedule */
	UpdateSurveillanceSchedule(param, T, S, src_table_for_Gv, dst_table_for_Gv, Gv, Gv_size, Dv_move, Mv_move, Dv_scan, Mv_scan, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
}

void HandleSensingHoles_With_AllProtectionPointsAlgorithm(parameter_t *param, struct_graph_node *Gv, int Gv_size, schedule_table_t *T, struct_sensor_table *S, double **Dv_move, int **Mv_move, int **Dv_scan, int **Mv_scan, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *src_or_dst_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time)
{ //handle the sensing holes that are represented as hole segments with two end-points labeling all of the holes as protection points

	/** perform the clustering for sensing holes (for labeling of entrance node or protection node) by labeling all of the holes as protection points */
	PerformClusteringWithAllProtectionPoints(src_table_for_Gv, dst_table_for_Gv, src_or_dst_table_for_Gv);
	
	/** update the schedule for surveillance such as sensing schedule and sleeping schedule */
	UpdateSurveillanceSchedule(param, T, S, src_table_for_Gv, dst_table_for_Gv, Gv, Gv_size, Dv_move, Mv_move, Dv_scan, Mv_scan, movement_time_for_sleeping, scanning_time_for_sleeping, sleeping_time);
}

void PerformClusteringWithExhaustiveSearch(parameter_t *param, schedule_table_t *T, struct_sensor_table *S, struct_traffic_table *src_table, struct_traffic_table *dst_table, struct_traffic_table *src_or_dst_table, struct_graph_node *G, int G_size, double **D_move, int **M_move, int **D_scan, int **M_scan)
{ //perform exhaustive search to find an optimal labeling for sensing holes as either entrance node or protection node
	/* We want to find how to allocate the nodes in src_or_dst_table to either src_table or dst_table
	   in order to get the maximal sleeping time to save energy */

	struct_traffic_table src_table_candidate; //candidate of source traffic table
	struct_traffic_table dst_table_candidate; //candidate of destination traffic table

	double *movement_time_candidate = NULL; //sleeping time component for vehicle movement time on the physical shortest path for a pair of entrance node (i.e., source) and protection node (i.e., destination) 
	double *scanning_time_candidate = NULL; //sleeping time component for vehicle movement time on the scan-signal shortest path for a pair of entrance node (i.e., source) and protection node (i.e., destination)
	double *sleeping_time_candidate = NULL; //sleeping time that is the sum of movement_time_for_sleeping and scanning_time_for_sleeping

	double max_sleeping_time = 0; //maximum sleeping time
	int index = 0; //index for entry with the maximum sleeping time among four candidates for sleeping time
	int i; //index for for-loop
	int n = src_or_dst_table->number; //number of sensing holes
	int case_number = (int) pow(2,n); //number of all possible cases in nC2

	boolean *A = NULL; //array for the hole's role that is either source or destination
	int *B = NULL; //array for bitmask
	int counter = 0; //counter for the determination of the roles of holes
	boolean flag = FALSE; //flag to determine whether a vertex belongs to a traffic table or not

	/* NOTE: When the number of sensing hole endpoints is 63, the computer cannot allocate 2^63 arrary for exhaustive search.
		- We need some condition to check the number of initial sensing hole endpoints.
		- We perform exhaustive search when the number of hole endpoints is less than 20.
	*/
	if(n > MAXIMUM_NUMBER_OF_HOLE_ENDPOINTS_FOR_EXHAUSTIVE_SEARCH)
	{
		printf("PerformClusteringWithExhaustiveSearch(): Due to too many sensing hole endpoints (%d), we cannot perform exhaustive searching for labeling.\n", n);
#ifdef __DEBUG_INTERACTIVE_MODE__
                fgetc(stdin);
#endif
		exit(1);
	}

	/** allocate the memory for movement_time_candidate, scanning_time_candidate and sleeping_time_candidate */
	movement_time_candidate = (double*) calloc(case_number, sizeof(double));
	assert_memory(movement_time_candidate);

	scanning_time_candidate = (double*) calloc(case_number, sizeof(double));
	assert_memory(scanning_time_candidate);

	sleeping_time_candidate = (double*) calloc(case_number, sizeof(double));
	assert_memory(sleeping_time_candidate);

	/** allocate the memory for arrary A and B */
	A = (boolean*) calloc(n, sizeof(boolean));
	assert_memory(A);

	B = (int*) calloc(n, sizeof(int));
	assert_memory(B);

	/** initialize the values of bitmask array B so that B[0] = 2^(n-1), B[1] = 2^(n-2), ..., B[n-1] = 2^(n-n) */
	for(i = 0; i < n; i++)
	{
		B[i] = (int) pow(2, n-(i+1));
	}

	/** evaluate sleeping time for src_table_candidate[i] and dst_table_candidate[i] */
	for(counter = 0; counter < case_number; counter++)
	{
		/* initialize traffic source table and destination table */
		InitTrafficTable(&src_table_candidate);
		InitTrafficTable(&dst_table_candidate);

		/* copy traffic source table and traffic destination table */
		CopyTrafficTable(&src_table_candidate, src_table);
		CopyTrafficTable(&dst_table_candidate, dst_table);

		/* determine the roles of the sensing holes */
		for(i = 0; i < n; i++)
		{
			A[i] = (boolean) (((counter & B[i])>>(n-(i+1))) & B[n-1]);

			if(A[i] == FALSE)
			{
				/* check whether the vertex corresponding to A[i] already belongs to dst_table_candidate or not */
				//flag = IsVertexInTrafficTable(&dst_table_candidate, src_or_dst_table->list[i]);
				flag = IsVertexInTrafficTable(&dst_table_candidate, src_or_dst_table->list[i].vertex);
				if(flag == FALSE)
				{
					//AddTrafficTableEntry(&src_table_candidate, src_or_dst_table->list[i]);
					AddTrafficTableEntry(&src_table_candidate, src_or_dst_table->list[i].vertex);
				}
				else
				{
					//printf("src_or_dst_table->list[i] already belong to dst_table_candidate");
					printf("src_or_dst_table->list[i].vertex already belong to dst_table_candidate");
					break;
				}
			}
			else
			{
				/* check whether the vertex corresponding to A[i] already belongs to src_table_candidate or not */
				//flag = IsVertexInTrafficTable(&src_table_candidate, src_or_dst_table->list[i]);
				flag = IsVertexInTrafficTable(&src_table_candidate, src_or_dst_table->list[i].vertex);
				if(flag == FALSE)
				{
					//AddTrafficTableEntry(&dst_table_candidate, src_or_dst_table->list[i]);				
					AddTrafficTableEntry(&dst_table_candidate, src_or_dst_table->list[i].vertex);				
				}
				else
				{
					//printf("src_or_dst_table->list[i] already belong to src_table_candidate");
					printf("src_or_dst_table->list[i].vertex already belong to src_table_candidate");
					break;
				}
			}
		}
		
		/* compute schedule with src_table_candidate and dst_table_candidate */
		if(flag == FALSE)
			ComputeSurveillanceSchedule(param, T, S, &src_table_candidate, &dst_table_candidate, G, G_size, D_move, M_move, D_scan, M_scan, &movement_time_candidate[counter], &scanning_time_candidate[counter], &sleeping_time_candidate[counter]);
		else
			movement_time_candidate[counter] = scanning_time_candidate[counter] = sleeping_time_candidate[counter] = -1;
		
		/* delete tables src_table_candidate and dst_table_candidate */
		Free_Traffic_Table(&src_table_candidate);
		Free_Traffic_Table(&dst_table_candidate);
	}

	/** determine the index with the maximum sleeping time */
	index = 0;
	for(i = 0; i < case_number; i++)
	{
		if(sleeping_time_candidate[i] > max_sleeping_time)
		{
			max_sleeping_time = sleeping_time_candidate[i];
			index = i;
		}
	}

	/** retain the roles of the sensing holes to allow the maximum sleeping time */
	counter = index;

	/* initialize traffic source table and destination table */
	InitTrafficTable(&src_table_candidate);
	InitTrafficTable(&dst_table_candidate);

	/* copy traffic source table and traffic destination table */
	CopyTrafficTable(&src_table_candidate, src_table);
	CopyTrafficTable(&dst_table_candidate, dst_table);

	for(i = 0; i < n; i++)
	{
		A[i] = (boolean) (((counter & B[i])>>(n-(i+1))) & B[n-1]);

		if(A[i] == FALSE)
			//AddTrafficTableEntry(&src_table_candidate, src_or_dst_table->list[i]);	
			AddTrafficTableEntry(&src_table_candidate, src_or_dst_table->list[i].vertex);	
		else
			//AddTrafficTableEntry(&dst_table_candidate, src_or_dst_table->list[i]);
			AddTrafficTableEntry(&dst_table_candidate, src_or_dst_table->list[i].vertex);
	}

	/* update src_table and dst_table by copying src_table_candidate[index]and dst_table_candidate[index] into src_table and dst_table, respectively. */
	CopyTrafficTable(src_table, &src_table_candidate);
	CopyTrafficTable(dst_table, &dst_table_candidate);

	/* delete tables src_table_candidate and dst_table_candidate */
	Free_Traffic_Table(&src_table_candidate);
	Free_Traffic_Table(&dst_table_candidate);

	/* release the memory for movement_time_candidate, scanning_time_candidate and sleeping_time_candidate */
	free(movement_time_candidate);
	free(scanning_time_candidate);
	free(sleeping_time_candidate);

	/* release the memory for arrary A and B */
	free(A);
	free(B);

#ifdef __DEBUG_LEVEL_SORT_TRAFFIC_TABLE__
	/** NOTE: If we want to see the vertex names in ascending order for debugging purpose, 
	    we perform the sorting of src_table and dst_table according to vertex name in asending order. */
	SortTrafficTable(src_table);
	SortTrafficTable(dst_table);
#endif
}

void PerformClusteringWithRandomLabeling(struct_traffic_table *src_table, struct_traffic_table *dst_table, struct_traffic_table *src_or_dst_table)
{ //perform random labeling to find a labeling for sensing holes as either entrance node or protection node
	int i; //index for for-loop
	int n = src_or_dst_table->number; //number of sensing holes
	int Coin = 0; //Result of coin tossing: 0 means head and 1 means tail
	boolean flag1, flag2; //flags to check whether a hole vertex exists in either src_table or dst_table
	static int head_counter = 0; //counter for the number of heads in coin tossing
	static int tail_counter = 0; //counter for the number of tails in coin tossing

	/*
	for(i = 0; i < n; i++) //for-1
	{
	  flag1 = IsVertexInTrafficTable(src_table, src_or_dst_table->list[i].vertex);
	  flag2 = IsVertexInTrafficTable(dst_table, src_or_dst_table->list[i].vertex);
	  if(flag1 || flag2)
	    continue;

	  Coin = smpl_random(0, 1); //fair coin tossing

	  if(Coin == 0) //coin head: entrance point
	  {
	    AddTrafficTableEntry(src_table, src_or_dst_table->list[i].vertex);	
	  }
	  else //coin tail: protection point
	  {
	    AddTrafficTableEntry(dst_table, src_or_dst_table->list[i].vertex);
	  }
	} //end of for-1
	*/

	/* label all of the new holes as the same random label in order to get the locally optimum sleeping time; otherwise, we can get a very short sleeping time when two closest holes have the opposite labels */
        
        Coin = smpl_random(0, 1); //fair coin tossing
	if(Coin == 0)
	  head_counter++;
        else
          tail_counter++;

	for(i = 0; i < n; i++) //for-1
	{
	  flag1 = IsVertexInTrafficTable(src_table, src_or_dst_table->list[i].vertex);
	  flag2 = IsVertexInTrafficTable(dst_table, src_or_dst_table->list[i].vertex);
	  if(flag1 || flag2)
	    continue;

	  //Coin = smpl_random(0, 1); //fair coin tossing

	  if(Coin == 0) //coin head: entrance point
	  {
	    AddTrafficTableEntry(src_table, src_or_dst_table->list[i].vertex);	
	  }
	  else //coin tail: protection point
	  {
	    AddTrafficTableEntry(dst_table, src_or_dst_table->list[i].vertex);
	  }
	} //end of for-1

#ifdef __DEBUG_LEVEL_SORT_TRAFFIC_TABLE__
	/** NOTE: If we want to see the vertex names in ascending order for debugging purpose, 
	    we perform the sorting of src_table and dst_table according to vertex name in asending order. */
	SortTrafficTable(src_table);
	SortTrafficTable(dst_table);
#endif
	return;
}

void PerformClusteringWithAllEntrancePoints(struct_traffic_table *src_table, struct_traffic_table *dst_table, struct_traffic_table *src_or_dst_table)
{ //perform random labeling to find a labeling for sensing holes as entrance points
	int i; //index for for-loop
	int n = src_or_dst_table->number; //number of sensing holes
	int Coin = 0; //Result of coin tossing: 0 means head and 1 means tail
	boolean flag1, flag2; //flags to check whether a hole vertex exists in either src_table or dst_table

	for(i = 0; i < n; i++) //for-1
	{
	  flag1 = IsVertexInTrafficTable(src_table, src_or_dst_table->list[i].vertex);
	  flag2 = IsVertexInTrafficTable(dst_table, src_or_dst_table->list[i].vertex);
	  if(flag1 || flag2)
	    continue;

	  AddTrafficTableEntry(src_table, src_or_dst_table->list[i].vertex);	
	} //end of for-1

#ifdef __DEBUG_LEVEL_SORT_TRAFFIC_TABLE__
	/** NOTE: If we want to see the vertex names in ascending order for debugging purpose, 
	    we perform the sorting of src_table and dst_table according to vertex name in asending order. */
	SortTrafficTable(src_table);
#endif
}

void PerformClusteringWithAllProtectionPoints(struct_traffic_table *src_table, struct_traffic_table *dst_table, struct_traffic_table *src_or_dst_table)
{ //perform random labeling to find a labeling for sensing holes as entrance points
	int i; //index for for-loop
	int n = src_or_dst_table->number; //number of sensing holes
	int Coin = 0; //Result of coin tossing: 0 means head and 1 means tail
	boolean flag1, flag2; //flags to check whether a hole vertex exists in either src_table or dst_table

	for(i = 0; i < n; i++) //for-1
	{
	  flag1 = IsVertexInTrafficTable(src_table, src_or_dst_table->list[i].vertex);
	  flag2 = IsVertexInTrafficTable(dst_table, src_or_dst_table->list[i].vertex);
	  if(flag1 || flag2)
	    continue;

	  AddTrafficTableEntry(dst_table, src_or_dst_table->list[i].vertex);
	} //end of for-1

#ifdef __DEBUG_LEVEL_SORT_TRAFFIC_TABLE__
	/** NOTE: If we want to see the vertex names in ascending order for debugging purpose, 
	    we perform the sorting of src_table and dst_table according to vertex name in asending order. */
	SortTrafficTable(dst_table);
#endif
}

schedule_table_node_t* GetTableNodeFromSubedgeList(subedge_queue_t *Q, hole_segment_queue_node_t *pHoleSegmentNode, double *left_hole_offset_in_Gv, double *right_hole_offset_in_Gv)
{ /* get the pointer to the exact schedule table node linked to the subedge containing the sensing hole segment from
   the updated schedule table T and get the offsets of the sensing hole segment in the virtual graph Gv */

	subedge_queue_node_t *pSubedgeNode = NULL; //pointer to a subedge queue node
	int i;
	double left_endpoint_offset = 0, right_endpoint_offset = 0; //cumulated offsets of the left end-point and the right end-point in a subedge for the offsets in edge queue Er
	schedule_table_node_t* pTableNode = NULL; //pointer to a schedule table node
	double difference_1, difference_2; //difference values

	/* initialize the offsets of left_hole_offset_in_Gv and right_hole_offset_in_Gv */
	*left_hole_offset_in_Gv = pHoleSegmentNode->left_hole_offset;
	*right_hole_offset_in_Gv = pHoleSegmentNode->right_hole_offset;

	pSubedgeNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pSubedgeNode = pSubedgeNode->next;
		left_endpoint_offset = right_endpoint_offset;
		right_endpoint_offset += pSubedgeNode->weight;

		/** we use absolute difference between two offset along with error tolerance rather than direct comparison in order to allow a very small floating-number arithmetic error */
		/*
		if(pHoleSegmentNode->left_hole_offset >= left_endpoint_offset && pHoleSegmentNode->right_hole_offset <= right_endpoint_offset)
		{ //we found the right subedge containing the hole segment
			pTableNode = pSubedgeNode->schedule_table_entry;
			break;
		}
		*/
		
		difference_1 = pHoleSegmentNode->left_hole_offset - left_endpoint_offset;
		difference_2 = right_endpoint_offset - pHoleSegmentNode->right_hole_offset;
		if(difference_1 >= -1*ERROR_TOLERANCE_FOR_REAL_ARITHMETIC && difference_2 >= -1*ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
		{ //we found the right subedge containing the hole segment
			pTableNode = pSubedgeNode->schedule_table_entry;
			break;
		}
	
		/* update left_hole_offset_in_Gv and right_hole_offset_in_Gv */
		*left_hole_offset_in_Gv -= pSubedgeNode->weight;
		*right_hole_offset_in_Gv -= pSubedgeNode->weight;
	}

	return pTableNode;
}

double GetMovementTimeOnPhysicalShortestPath(parameter_t *param, double **D_move, int protection_node, int entrance_node)
{ //return the vehicle movement time on the physical shortest path between a pair of an entrance node and a protection node
	/* the shortest path length on the physical shortest path */
	double move_time = INF; //vehicle's movement time
	double shortest_path_length = INF; //length of the shortest path from Enter nodes to Exit nodes
	//double v_max = param->vehicle_maximum_speed; //maximum vehicle speed
	double v_max = (param->vehicle_speed_distribution==EQUAL ? param->vehicle_speed : param->vehicle_maximum_speed); //maximum vehicle speed

	shortest_path_length = D_move[protection_node][entrance_node];
	move_time = shortest_path_length/v_max; //vehicle's movement time from source to destination

	return move_time;
}

double GetScanningTimeOnScanSignalShortestPath(parameter_t *param, double **D_move, int **D_scan, int protection_node, int entrance_node)
{ //return the sensing scanning time on the scan-signal shortest path between a pair of an entrance node and a protection node
	/* the shortest path length on the scan-signal shortest path */
	double scan_time = INF; //sensor scanning time
    double shortest_path_length = INF; //length of the shortest path from Enter nodes to Exit nodes
	int shortest_path_sensor_num = INF; //number of sensors on the shortest path from destination to source
	double r = param->sensor_sensing_range; //sensing range (or sensing radius)
	double w = param->sensor_work_time; //minimum sensor work time for detection
	double v_s = 2*r/w; //scan_speed

	switch(param->sensor_scan_type)
	{
	case SCAN_TURN_ON_ALL:
	case SCAN_NO_USE:
	case SCAN_NO_USE_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
		scan_time = 0;
		break;

	case SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING:
	case SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
		shortest_path_length = D_move[protection_node][entrance_node];
		scan_time = shortest_path_length/v_s; //sensor scanning time from destination to source
		break;

	case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING:
	case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
	case SCAN_VARIABLE_SPEED_WITH_VARIABLE_SLEEPING_TIME_AND_WITH_SENSING_HOLE_HANDLING:
	case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING:
	case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING_AND_WITH_SENSING_HOLE_HANDLING:
		shortest_path_sensor_num = D_scan[protection_node][entrance_node];
		scan_time = shortest_path_sensor_num * w; //sensor scanning time from destination to source
		break;

	default:
		printf("GetScanningTimeOnScanSignalShortestPath(): sensor_scan_type(%d) is not supported!\n", param->sensor_scan_type);
#ifdef __DEBUG_INTERACTIVE_MODE__
                fgetc(stdin);
#endif
		exit(1);
	}

	return scan_time;
}

void RegisterTableNodeIntoVirtualGraph(struct_graph_node *G, int G_size, schedule_table_node_t *ptr_table_node)
{ //register the pointer to the table node in virtual graph Gv
  char *tail_node = ptr_table_node->tail_node; //tail node of a directional edge
  char *head_node = ptr_table_node->head_node; //head node of a directional edge
  int tail_node_id = atoi(tail_node); //tail node id
  int head_node_id = atoi(head_node); //head node id  
  struct_graph_node *ptr = NULL; //pointer to the graph node
  boolean flag1 = FALSE, flag2 = FALSE;

  /* register ptr_table_node into the neighbor list of tail_node */
  ptr = &(G[tail_node_id-1]);

  while(ptr != NULL)
  {
    ptr = ptr->next;

    if(strcmp(ptr->vertex,head_node) == 0)
    {
      ptr->ptr_table_node = ptr_table_node;
      flag1 = TRUE;
      break;
    }
  }
  
  /* register ptr_table_node into the neighbor list of head_node */
  ptr = &(G[head_node_id-1]);

  while(ptr != NULL)
  {
    ptr = ptr->next;

    if(strcmp(ptr->vertex,tail_node) == 0)
    {
      ptr->ptr_table_node = ptr_table_node;
      flag2 = TRUE;
      break;
    }
  }  

  /* check whether the pointer to the table node is set to both the tail_node's neighbor list and the head_node's neighbor list */
  if(flag1 == FALSE || flag2 == FALSE)
  {
    printf("RegisterTableNodeIntoVirtualGraph(): both flag1(%d) and flag2(%d) must be 1 to indicate the successful registration of the pointer to the table node in virtual graph Gv\n", flag1, flag2);
    exit(1);
  }
}

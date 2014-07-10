/**
 *  File: shortest-path.c
	Description: operations for the single-source all-destination shortest path algorithm 
	 based on Dijkstra's shortest path algorithm
	Date: 09/22/2006	
	Maker: Jaehoon Jeong
*/

#include "stdafx.h"
#include <stdlib.h> //atoi()
#include "all-pairs-shortest-paths.h"
#include "shortest-path.h"
#include "heap.h"
#include "util.h"
#include "quick-sort.h"
#include "queue.h"
#include "vadd.h" //@Note: witout including this header file, VADD_Compute_Average_Convoy_Length() returns a wrong value

#include "gsl-util.h" //@Note: witout including this header file, GSL_Vanet_Delivery_Probability() returns a wrong value

struct_graph_node* Initialize_Graph(parameter_t *param, char *graph_file_name, int *G_size, struct_traffic_table *src_table, struct_traffic_table *dst_table, struct_traffic_table *ap_table, struct_traffic_table *sn_table, double *max_weight)
{ //initialize a adjacent list representing a graph
	FILE *fp; //file pointer
	char input_buf[BUFFER_SIZE]; //input buffer
	char vertex_buf[BUFFER_SIZE]; 
	char neighbor_buf[BUFFER_SIZE];
	char weight_buf[BUFFER_SIZE];  //buffer for weight that is the length of the edge 
	char density_buf[BUFFER_SIZE]; //buffer for density of sensors deployed for unit distance
	char x_coordinate_buf[BUFFER_SIZE]; //buffer for the node's x coordinate
	char y_coordinate_buf[BUFFER_SIZE]; //buffer for the node's y coordinate

	double scale_factor = 1.0; //scale factor for position and distance

	double weight;
	size_t input_buf_size;
	char *token;
	struct_graph_node *G = NULL; //adjacent list representing a graph
	struct_graph_node *node = NULL, *ptr = NULL;
	int node_id = 0; //node id
	int i = 0; //index for G
	int j = 0; //index for src_table or dst_table
	size_t len; //length of node name
	char *char_ptr = NULL; //character pointer
	GRAPH_NODE_TYPE default_type = INTERSECTION_GRAPH_NODE; //graph node type
	GRAPH_NODE_ROLE default_role = ROLE_INTERSECTION_POINT; //graph node's role in scheduling
	GRAPH_NODE_ROLE access_point_role = ROLE_ACCESS_POINT; //access point as role
	GRAPH_NODE_ROLE entrance_role = ROLE_ENTRANCE_POINT; //entrance point in scheduling
	GRAPH_NODE_ROLE protection_role = ROLE_PROTECTION_POINT; //protection point in scheduling

	angle_queue_node_t angle_queue_node; //angle queue node
	intersection_edd_queue_node_t intersection_edd_queue_node; //intersection EDD queue node
		
	*max_weight = 0; //maximum road segment length

	*G_size = 0;
	src_table->number = 0;
	dst_table->number = 0;

	/* initialize tables */
	ap_table->number = 0;
	sn_table->number = 0;

	/* open graph configuration file called graph_file_name */
	fp = fopen(graph_file_name, "r");
	if(!fp)
	{
		fprintf(stderr, "Error: unable to open file \"%s\"\n", graph_file_name);
		exit(1);
	}

	while(fgets(input_buf, sizeof(input_buf), fp) != NULL) //while-1
	{
		input_buf_size = strlen(input_buf);

		token = (char*) strtok(input_buf, ":");

		if(token == NULL)
		{
			printf("Error: the key in the graph configuration file is null!\n");
			exit(1);
		}
		
		if((token[0] == '\n') || (token[0] == '\r') || (token[0] == ' ') || (token[0] == '\t')) //this line is white space
			continue;
		else if(token[0] == '#') //this is comment line
			continue;

		if(strcmp(token, "node_number") == 0) //if-1
		{
			token = (char*) strtok(NULL, " \t");
			if(token == NULL)
			{
				printf("Error: tokeen is null!\n");
				exit(1);
			}

			*G_size = atoi(token);
			G = (struct_graph_node*) calloc(*G_size, sizeof(struct_graph_node));
			assert_memory(G);
		} //end of if-1
		else if(strcmp(token, "traffic_source_number") == 0) //else if-1
		{
			token = (char*) strtok(NULL, " \t");
			if(token == NULL)
			{
				printf("Error: token is null!\n");
				exit(1);
			}

			src_table->number = atoi(token);
			src_table->list = (struct_traffic_node*) calloc(src_table->number, sizeof(struct_traffic_node));
			assert_memory(src_table->list);			
		} //end of else if-1
		else if(strcmp(token, "traffic_destination_number") == 0) //else if-2
		{
			token = (char*) strtok(NULL, " \t");
			if(token == NULL)
			{
				printf("Error: token is null!\n");
				exit(1);
			}

			dst_table->number = atoi(token);
			dst_table->list = (struct_traffic_node*) calloc(dst_table->number, sizeof(struct_traffic_node));
			assert_memory(dst_table->list);
		} //end of else if-2
		else if(strcmp(token, "access_point_number") == 0) //else if-3
		{
			token = (char*) strtok(NULL, " \t");
			if(token == NULL)
			{
				printf("Error: token is null!\n");
				exit(1);
			}

			ap_table->number = atoi(token);
			ap_table->list = (struct_traffic_node*) calloc(ap_table->number, sizeof(struct_traffic_node));
			assert_memory(ap_table->list);			
		} //end of else if-3
		else if(strcmp(token, "stationary_node_number") == 0) //else if-4
		{
			token = (char*) strtok(NULL, " \t");
			if(token == NULL)
			{
				printf("Error: token is null!\n");
				exit(1);
			}

			sn_table->number = atoi(token);
			sn_table->list = (struct_traffic_node*) calloc(sn_table->number, sizeof(struct_traffic_node));
			assert_memory(sn_table->list);			
		} //end of else if-4
		else if(strcmp(token, "traffic_source_list") == 0) //else if-5
		{
			if(src_table->number == 0)
			{
				printf("Error: src_table->number should be set to positive integer beforehand\n");
				exit(1);
			}
			
			j = 0;
			do
			{
				token = (char*) strtok(NULL, " ,\n\r");
				if(token == NULL)
					break;

				len = strlen(token);
				if(len == 0)
				{
					printf("Error: node name is null\n");
					exit(1);
				}

				strcpy(src_table->list[j].vertex, token); //token is the name of one traffic source node
				j++;
			} while(1);		
		} //end of else if-5
		else if(strcmp(token, "traffic_destination_list") == 0) //else if-6
		{
			if(dst_table->number == 0)
			{
				printf("Error: dst_table->number should be set to positive integer beforehand\n");
				exit(1);
			}
			
			j = 0;
			do
			{
				token = (char*) strtok(NULL, " ,\n\r");
				if(token == NULL)
					break;

				len = strlen(token);
				if(len == 0)
				{
					printf("Error: node name is null\n");
					exit(1);
				}

				strcpy(dst_table->list[j].vertex, token); //token is the name of one traffic destination node
				j++;
			} while(1);		
		} //end of else if-6
		else if(strcmp(token, "access_point_list") == 0) //else if-7
		{
			if(ap_table->number == 0)
			{
				printf("Error: ap_table->number should be set to positive integer beforehand\n");
				exit(1);
			}
			
			j = 0;
			do
			{
				token = (char*) strtok(NULL, " ,\n\r");
				if(token == NULL)
					break;

				len = strlen(token);
				if(len == 0)
				{
					printf("Error: node name is null\n");
					exit(1);
				}

				strcpy(ap_table->list[j].vertex, token); //token is the name of one access point node
				j++;
			} while(1);		
		} //end of else if-7
		else if(strcmp(token, "stationary_node_list") == 0) //else if-8
		{
			if(sn_table->number == 0)
			{
				printf("Error: sn_table->number should be set to positive integer beforehand\n");
				exit(1);
			}
			
			j = 0;
			do
			{
				token = (char*) strtok(NULL, " ,\n\r");
				if(token == NULL)
					break;

				len = strlen(token);
				if(len == 0)
				{
					printf("Error: node name is null\n");
					exit(1);
				}

				strcpy(sn_table->list[j].vertex, token); //token is the name of one stationary node
				j++;
			} while(1);		
		} //end of else if-8
		else if(strcmp(token, "scale_factor") == 0) //else if-9
		{
			token = (char*) strtok(NULL, " \t");
			if(token == NULL)
			{
				printf("Error: token is null!\n");
				exit(1);
			}

			scale_factor = atof(token);
			if(scale_factor <= 0)
			{
			  printf("Error: scale_factor(%f) must be positive\n", scale_factor);
			  exit(1);
			}
		} //end of else if-9
		else //else-10
                { //construct the adjacency list
			if(*G_size == 0)
			{
				printf("Error: *G_size should be set to positive integer beforehand\n");
				exit(1);
			}

			ptr = &G[i];
			G[i].gnode = ptr; //gnode points to itself
			strcpy(vertex_buf, token); //vertex name
			strcpy(G[i].vertex, vertex_buf);
			G[i].type = default_type; //graph node type
			G[i].role = default_role; //graph node role
			G[i].weight = 0; //indicate the degree (i.e., number of neighbors)
			G[i].next = NULL;

			/** allocate the memory of angle queue and initialize it */
			G[i].angle_queue = (angle_queue_t*) calloc(1, sizeof(angle_queue_t));
			assert_memory(G[i].angle_queue);
			InitQueue((queue_t*)G[i].angle_queue, QTYPE_ANGLE);

			/** allocate the memory of intersection EDD queue and initialize it */
			G[i].intersection_edd_queue = (intersection_edd_queue_t*) calloc(1, sizeof(intersection_edd_queue_t));
			assert_memory(G[i].intersection_edd_queue);
			InitQueue((queue_t*)G[i].intersection_edd_queue, QTYPE_INTERSECTION_EDD);

			/** obtain the coordinate of the node */
			token = (char*) strtok(NULL, ":"); //remove ':'
			if(token == NULL)
			{
			  printf("Error: Node's coordinate is omitted!\n");
			  exit(1);
			}

			char_ptr = strchr(token, ')'); 
			if(char_ptr != NULL)
			{
			  *char_ptr = '\0'; //remove ')'
			}
			else
			{
			  printf("syntax Error: there is no ')' for coordinate tuple\n");
			  exit(1);
			}
			 
			/* obtain the x-coordinate and y-coordinate of the node */
			//find the position of '(' in front of the coordinate
			char_ptr = strchr(token, '(');
			if(char_ptr != NULL)
			{
			  strcpy(token, char_ptr+1); //shift the string to the left in order to remove '('.
			}
			else
			{
			  printf("syntax Error: there is no '(' for coordinate tuple\n");
			  exit(1);
			}
			  
			//find the position of '(' in front of the coordinate
			char_ptr = strchr(token, ',');
			if(char_ptr != NULL)
			{
			  *char_ptr = '\0';
			  strcpy(x_coordinate_buf, token); //x_coorinate_buf has the node's x coordinate
			  strcpy(token, char_ptr+1); //shift the string to the left in order to remove ','
			  strcpy(y_coordinate_buf, token); //y_coorinate_buf has the node's y coordinate
			}
			else
			{
			  printf("syntax Error: there is no ',' for coordinate tuple\n");
			  exit(1);
			}

			/* set the coordinate of G[i] */
			G[i].coordinate.x = atof(x_coordinate_buf)*scale_factor;
			G[i].coordinate.y = atof(y_coordinate_buf)*scale_factor;
			
			/** obtain the list of neighbor nodes */
			do //do-while-11
			{
				/*
				token = (char*) strtok(NULL, "(\n\r");
				if(token == NULL)
					break;
				*/

				/* neighbor's name */
				token = (char*) strtok(NULL, ", \t\n\r");
				if(token == NULL)
				{
					//printf("Error: neighbor is omitted\n");
					//exit(1);
					break;
				}

				//find the position of '(' in front of neighbor node id
				char_ptr = strchr(token, '(');
				if(char_ptr != NULL)
				{
					strcpy(token, char_ptr+1); //shift the string to the left in order to remove '('.
				}
				else
				{
					printf("syntax Error: there is no '(' for neighbor tuple\n");
					exit(1);
				}

				strcpy(neighbor_buf, token); //neighbor_buf has the neighbor's name
				/***/
                
				/* weight to neighbor */
				//token = (char*) strtok(NULL, ",) \t\n\r");
				token = (char*) strtok(NULL, ", \t\n\r");
				if(token == NULL)
				{
					printf("Error: weight is omitted\n");
					exit(1);
				}

				//check whether there is ')' without density or not
                                char_ptr = strchr(token, ')');
				if(char_ptr != NULL)
				{
					*char_ptr = '\0'; //if there is ')' after weight, replace ')' with '\0'.
					strcpy(weight_buf, token); //weight_buf has the incident edge's weight

					/* set density_buf to param->sensor_density */
#ifdef __DEBUG__
					printf("density for edge (vertex_buf,neighbor_buf) is set to default sensor density(%f)\n", (float) param->sensor_density);
#endif
					sprintf(density_buf, "%f", (float) param->sensor_density);
				}
				else
				{
					strcpy(weight_buf, token); //weight_buf has the incident edge's weight

					/* density of sensors */
					token = (char*) strtok(NULL, ") \t\n\r");
					if(token == NULL)
					{
#ifdef __DEBUG__
						printf("density for edge (vertex_buf,neighbor_buf) is set to default sensor density(%f)\n", (float) param->sensor_density);
#endif
						sprintf(density_buf, "%f", (float) param->sensor_density);
					}
					else
					{
						strcpy(density_buf, token); //density_buf has the density of sensors deployed on the edge	
					}
				}
                                /***/

				node = Make_Graph_Node(neighbor_buf, weight_buf, density_buf, default_type, default_role, scale_factor);
				ptr->next = node;
				ptr = node;
				
				node_id = atoi(node->vertex); //obtain the node id corresponding to node->vertex
				ptr->gnode = &(G[node_id-1]); //let ptr->gnode point to the graph node in G[] corresponding to node_id in order to access the information (such as coordinate) of the graph node corresponding to node_id

			        /** enqueue the angle queue node corresponding to the edge <G[i].vertex,ptr->vertex> into angle queue */
				memset(&angle_queue_node, 0, sizeof(angle_queue_node));
                                strcpy(angle_queue_node.tail_node, G[i].vertex);
                                strcpy(angle_queue_node.head_node, ptr->vertex);
				angle_queue_node.tail_gnode = &(G[i]);
				angle_queue_node.head_gnode = ptr;

		     	       	Enqueue((queue_t*)G[i].angle_queue, (queue_node_t*)&angle_queue_node);

			        /** enqueue the intersection edd queue node corresponding to the edge <G[i].vertex,ptr->vertex> into intersection edd queue */
				memset(&intersection_edd_queue_node, 0, sizeof(intersection_edd_queue_node));
                                intersection_edd_queue_node.type = param->vehicle_vanet_metric_type;
				intersection_edd_queue_node.tail_gnode = &(G[i]);
				intersection_edd_queue_node.head_gnode = ptr;

		     	       	Enqueue((queue_t*)G[i].intersection_edd_queue, (queue_node_t*)&intersection_edd_queue_node);

				G[i].weight++; //increase the degree of node G[i]

				/* update max_weight */
				weight = atof(weight_buf)*scale_factor;
				//weight = atof(weight_buf)*EDGE_SCALE_FACTOR;

				if(*max_weight < weight)
					*max_weight = weight;
			} while(1); //end of do-while-1
                  
			i++;
		} // end of else-10
	} //end of while-1

	///** assign each graph node's role using src and dst traffic tables */
	//AssignTypeAndRoleToGraphNode(G, *G_size, src_table, dst_table);

	/** assign each graph node's role using access point table */
	AssignRoleToGraphNode(G, *G_size, ap_table, access_point_role);

        /** construct as many conditional probability queue nodes as the neighboring edges per the edge where G[i].vertex is the tail node */
        ConstructConditionalForwardingProbabilityQueueNodes_Per_DirectionalEdge(G, *G_size);

	/* close file descriptor for graph configuration file called graph_file_name */
	fclose(fp);

	return G;
}

void AssignTypeAndRoleToGraphNode(struct_graph_node *G, int G_size, struct_traffic_table *src_table, struct_traffic_table *dst_table)
{//assign each graph node's type and role using src and dst traffic tables
  int i; //indices for for-loops
  int node_id = 0; //node id

  /* set the entrance node to entrance role */
  for(i = 0; i < src_table->number; i++)
  {
    node_id = atoi(src_table->list[i].vertex);
    G[node_id-1].type = ENTRANCE_GRAPH_NODE;
    G[node_id-1].role = ROLE_ENTRANCE_POINT;
  }

  /* set the protection node to protection role */
  for(i = 0; i < dst_table->number; i++)
  {
    node_id = atoi(dst_table->list[i].vertex);
    G[node_id-1].type = PROTECTION_GRAPH_NODE;
    G[node_id-1].role = ROLE_PROTECTION_POINT;
  }
}

void AssignRoleToGraphNode(struct_graph_node *G, int G_size, struct_traffic_table *table, GRAPH_NODE_ROLE role)
{//assign each graph node's role using table
  int i; //indices for for-loops
  int node_id = 0; //node id

  /* set each node to role */
  for(i = 0; i < table->number; i++)
  {
    node_id = atoi(table->list[i].vertex);
    G[node_id-1].role = role;
  }
}

void AssociateGraphEdgeWithEdgeEntry(struct_graph_node *G, int G_size, edge_queue_t *E)
{ //associate each pair of two adjacent graph nodes (i.e., physical edge) in graph G with the corresponding edge entry in E

	edge_queue_node_t *pEdgeNode = NULL; //pointer to an edge queue node
	struct_graph_node *p = NULL, *q = NULL;//pointers to graph nodes
	char u[NAME_SIZE], v[NAME_SIZE]; //vertices
	int i, j;
	int neighbor_num; //number of neighbor vetices
	boolean flip_flag = FALSE; //flag to indicate if the order of vertices in node matches 
	//that of an entry in schedule table; in this function, this is not used.

	if(G_size == 0)
		return;
	
	for(i = 0; i < G_size; i++) //for-1
	{
		p = &(G[i]);
		strcpy(u, p->vertex);
		neighbor_num = (int)p->weight;
		q = p;
		for(j = 0; j < neighbor_num; j++) //for-2
		{
			q = q->next;
			strcpy(v, q->vertex);
			pEdgeNode = LookupEdgeQueue(E, u, v, &flip_flag); //get the pointer to the edge entry node from edge queue E through linear searching
			if(pEdgeNode != NULL) //if
			{
			  q->ptr_edge_node = pEdgeNode; //set up the pointer to the edge queue node
			} //end of if
			else
		        {
			  printf("AssociateGraphEdgeWithEdgeEntry(): there is no edge entry for edge (%s,%s)\n", u, v);
			  exit(1);
			}
		} //end of for-1
	} //end of for-2

}

void AssociateGraphEdgeWithDirectionalEdgeEntry(struct_graph_node *G, int G_size, directional_edge_queue_t *E)
{ //associate each pair of two adjacent graph nodes (i.e., physical edge) in graph G with the corresponding directional edge entry in E

	directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to a directional edge queue node
	struct_graph_node *p = NULL, *q = NULL;//pointers to graph nodes
	char u[NAME_SIZE], v[NAME_SIZE]; //vertices where u is tail node and v is head node in edge <u,v>
	int i, j;
	int neighbor_num; //number of neighbor vetices

	if(G_size == 0)
		return;
	
	for(i = 0; i < G_size; i++) //for-1
	{
		p = &(G[i]);
		strcpy(u, p->vertex);
		neighbor_num = (int)p->weight;
		q = p;
		for(j = 0; j < neighbor_num; j++) //for-2
		{
			q = q->next;
			strcpy(v, q->vertex);
			pEdgeNode = LookupDirectionalEdgeQueue(E, u, v); //get the pointer to the edge entry node from edge queue E through linear searching
			if(pEdgeNode != NULL) //if
			{
			  q->ptr_directional_edge_node = pEdgeNode; //set up the pointer to the directional edge queue node
			} //end of if
			else
		        {
			  printf("AssociateGraphEdgeWithDirectionalEdgeEntry(): there is no directional edge entry for edge <%s,%s>\n", u, v);
			  exit(1);
			}
		} //end of for-1
	} //end of for-2
}

struct_graph_node* Make_Graph_Node(char *vertex, char *weight, char *density, GRAPH_NODE_TYPE type, GRAPH_NODE_ROLE role, double scale_factor)
{ //allocate the memory for a graph node
	struct_graph_node* node = NULL;
	size_t l;
	double w; //weight
	double d; //density


	w = atof(weight)*scale_factor;
	//w = atof(weight)*EDGE_SCALE_FACTOR;
	d = atof(density);

        node = (struct_graph_node*) calloc(1, sizeof(struct_graph_node));
	if(node == NULL)
	{
		printf("Make_Graph_Node(): node is NULL\n");
#ifdef __DEBUG_INTERACTIVE_MODE__
                fgetc(stdin);
#endif
	        exit(1);
	}

	/* set vertex name */
	l = strlen(vertex);
	if(l >= NAME_SIZE)
	{
		printf("Make_Graph_Node(): vertex (%s) has a name longer than %d\n", vertex, NAME_SIZE);
		exit(1);
	}

	strcpy(node->vertex, vertex);

	/* set graph node's type */
	node->type = type; //graph node's type

	/* set graph node's role */
	node->role = role; //graph node's role

	/* set graph node's status to USAGE_USED */
	node->status = USAGE_USED;

	/* set weight that is the edge length */
	if(w <= 0)
	{
		printf("Make_Graph_Node(): weight w(%f) is not positive!\n", w);
                //exit(1);
	}
	else
		node->weight = w;

	/* set sensor density for the edge */
	if(d <= 0)
	{
		printf("Make_Graph_Node(): density d(%f) is not positive!\n", d);
		exit(1);
	}
	else
		node->density = d;

	node->next = NULL;

	return node;
}

struct_graph_node* Make_Graph_Node2(char *vertex, double weight, double density, GRAPH_NODE_TYPE type, GRAPH_NODE_ROLE role)
{ //allocate the memory for a graph node for node copy operation
	struct_graph_node* node = NULL;

        node = (struct_graph_node*) calloc(1, sizeof(struct_graph_node));
	if(node == NULL)
	{
		printf("Make_Graph_Node2(): node is NULL\n");
#ifdef __DEBUG_INTERACTIVE_MODE__
                fgetc(stdin);
#endif
	        exit(1);
	}

	/* set vertex name */
	strcpy(node->vertex, vertex);

	/* set graph node's type */
	node->type = type; //graph node's type

	/* set graph node's role */
	node->role = role; //graph node's role

	/* set graph node's status to USAGE_USED */
	node->status = USAGE_USED;

	/* set weight that is the edge length */
	node->weight = weight;

	/* set sensor density for the edge */
	node->density = density;

	node->next = NULL; //let the pointer to the next node be NULL

	return node;
}

void Free_Graph(struct_graph_node *G, int G_size)
{ //release the memory allocated to G
	int i;
	struct_graph_node *ptr1 = NULL, *ptr2 = NULL;

	for(i = 0; i < G_size; i++)
	{
		ptr1 = G[i].next;
		while(ptr1 != NULL)
		{
			ptr2 = ptr1;
			ptr1 = ptr1->next;

                        /* delete the conditional forwarding probability queue for the head node ptr2 and free the memory for conditional forwarding probability queue */
		        DestroyQueue((queue_t*)ptr2->conditional_forwarding_probability_queue);
		        free(ptr2->conditional_forwarding_probability_queue);

                        /* free the memory for the neighboring graph node pointed by ptr2 */
			free(ptr2);
		}
		
		/* delete the angle queue nodes and free the memory for angle queue */
		DestroyQueue((queue_t*)G[i].angle_queue);
		free(G[i].angle_queue);

		/* delete the intersection edd queue nodes and free the memory for intersection EDD queue */
		DestroyQueue((queue_t*)G[i].intersection_edd_queue);
		free(G[i].intersection_edd_queue);
	}

	free(G); //free the node array
}

struct_set_node* Dijkstra(struct_graph_node *G, int G_size, char *src)
{ //run Dijkstra's algorithm to compute the single-source shortest path
	struct_set_node *S = NULL; //set for nodes included in the spanning tree set
	struct_shortest_path_node *u = NULL, *v = NULL;
	struct_graph_node *neighbor = NULL; //neighbor node for a given node u
	struct_shortest_path_node *Q; //Minimum priority queue
	int Q_size; //size of min priority queue Q
	int Q_index; //index for an element in Q
	double weight; //edge weight between nodes u and v

	Q_size = G_size;
	Q = Initialize_MIN_Priority_Queue(G, G_size, src); //initialize a minimum priority queue Q using heap
	S = Set_Init(); //initialize set S

	while(Q_size > 0)
	{
		u = Heap_Extract_Min(Q, Q_size--); //return the node with the minimum distance for source s
		S = Set_Insert(S, u); //insert node u into SET
		for(neighbor = u->gnode->next; neighbor != NULL; neighbor = neighbor->next)
		{
			v = Get_Heap_Node(neighbor->vertex, Q, Q_size, &Q_index); //search the heap node corresponding to vertex name
			if(v == NULL)
				continue; //the neighor node is already in set S, so we don't need to update the distance
			
			weight = neighbor->weight; //edge weight between nodes u and v
			Relax(u, v, weight, Q, Q_size, Q_index, S->prev);
			//Relax(u, v, weight, Q, Q_size, Q_index);
		}
	} //end of while

	return S;
}

struct_shortest_path_node* Initialize_MIN_Priority_Queue(struct_graph_node *G, int G_size, char *src)
{ //initialize a minimum priority queue based on heap for Dijkstra's algorithm
  //src: source node
	struct_shortest_path_node *Q = NULL; //min priority queue
	struct_graph_node *v = NULL; //graph node
	int i;

	//Q = (struct_shortest_path_node*) malloc(sizeof(struct_shortest_path_node)*(G_size+1));
        Q = (struct_shortest_path_node*) calloc(G_size+1, sizeof(struct_shortest_path_node));
	//allocate the memory of heap Q where the size of Q's memory is one more than the actual number
	//of nodes. The reason is that in heap Q, the first node is Q[1].
	if(Q == NULL)
	{
		printf("Initialize_Single_Source(): Q cannot get dynamic memory\n");
		exit(1);
	}
	
	for(i = 1, v = &G[i-1]; i <= G_size; i++, v = &G[i-1])
	{

		if(strcmp(v->vertex, src) == 0)
			Q[i].dist = 0;
		else
			Q[i].dist = INF;

		strcpy(Q[i].vertex, v->vertex);
		memset(Q[i].parent, '\0', sizeof(Q[i].parent));
		Q[i].gnode = v; //gnode indicates the position of graph node v in order to use the pointer in Dijkstra's algorithm.
		Q[i].pnode = NULL; //pnode indicates the position of the neighbor node towards source whose type is struct_set_node*.
	}

	Build_Min_Heap(Q, G_size); //build minimum priority heap

	return Q;
}

void Relax(struct_shortest_path_node *u, struct_shortest_path_node *v, double w, struct_shortest_path_node *Q, int Q_size, int Q_index, struct_set_node *parent_node)
//void Relax(struct_shortest_path_node *u, struct_shortest_path_node *v, int w, struct_shortest_path_node *Q, int Q_size, int Q_index)
{ //update the current node u's cost and parent node towards the source
	double key;

	if(v->dist > u->dist + w)
	{
		//v->dist = u->dist + w;
		key = u->dist + w;
		strcpy(v->parent, u->vertex);
		v->pnode = parent_node;
		Heap_Decrease_Key(Q, Q_size, Q_index, key);
		//decrease the key (i.e., dist) of the node specified by i in heap Q
	}
}

struct_set_node* Set_Init()
{ //initialize the set for shortest path
	struct_set_node *head; //head of doubly linked list for a set including nodes

	//head = (struct_set_node*) malloc(sizeof(struct_set_node));
        head = (struct_set_node*) calloc(1, sizeof(struct_set_node));
	assert_memory(head); //assert the memory allocation
	memset(&(head->node), 0, sizeof(head->node));
	head->next = head;
	head->prev = head;
	return head;
}

void Set_Free(struct_set_node *set)
{ //release the memory occupied by the set for shortest path
	struct_set_node *head = set, *ptr = NULL, *removed_node = NULL;

	for(ptr = head->next; ptr != head;)
	{
		removed_node = ptr;
		ptr = ptr->next;
		free(removed_node);
	}

	free(head);
}

struct_set_node* Set_Insert(struct_set_node *S, struct_shortest_path_node *u)
{ //insert node u into set S
	struct_set_node *set_node; //head of doubly linked list for a set including nodes

	//set_node = (struct_set_node*) malloc(sizeof(struct_set_node));
        set_node = (struct_set_node*) calloc(1, sizeof(struct_set_node));
	assert_memory(set_node); //assert the memory allocation
	memset(set_node, 0, sizeof(struct_set_node));
	memcpy(&(set_node->node), u, sizeof(struct_shortest_path_node));
	set_node->prev = S->prev;
	S->prev->next = set_node;
	S->prev = set_node;
	set_node->next = S;

	return S;
}

struct_shortest_path_node* Get_Heap_Node(char *vertex, struct_shortest_path_node *Q, int Q_size, int *Q_index)
{ //search the heap node corresponding to vertex name
  //@ this searching is linear search, so we need to improve this searching in another efficient way.
	int i;

	for(i = 1; i <= Q_size; i++)
	{
		if(strcmp(Q[i].vertex, vertex) == 0)
		{
			*Q_index = i;
			return &Q[i];
		}
	}

	return NULL;
}

void Free_Traffic_Table(struct_traffic_table *table)
{ //release the memory occupied by the traffic table
	//int i;

	/* 03/02/08: I commented this for-loop since traffic table entry is fixed-sized structure having vertex element of constant size to have the vertex name */
        /*
	for(i = 0; i < table->number; i++)
	{
		free(table->list[i]);
	}	
	*/
        
        /*
	free(table->list);
 	table->number = 0;
	table->list = NULL;
	*/

        /*@Note: check the number of table. If number > 0, then free table->list; otherwise, the memory contamination happens, so other dynamic memory structures are affected! */
        if(table->number > 0)
	{
	  free(table->list);
 	  table->number = 0;
	  table->list = NULL;
	}
}

void Initialize_Path_Table(struct_graph_node *G, int G_size, struct_traffic_table *src_table, struct_path_table *path_table)
{ //intialize path table including the shortest path information for all the sources in src_table
	int i;

	path_table->number = src_table->number;
	//path_table->list = (struct_set_node**) malloc(path_table->number*sizeof(struct_set_node*));
        path_table->list = (struct_set_node**) calloc(path_table->number, sizeof(struct_set_node*));
	for(i = 0; i < src_table->number; i++)
	{
		//path_table->list[i] = Dijkstra(G, G_size, src_table->list[i]);
		path_table->list[i] = Dijkstra(G, G_size, src_table->list[i].vertex);
		//run Dijkstra's algorithm to compute the single-source shortest path		
	}
}

void Free_Path_Table(struct_path_table *table)
{ //release the memory occupied by the path table
	int i;

	for(i = 0; i < table->number; i++)
	{
		Set_Free(table->list[i]);
	}

	if(table->number > 0)
	{
	  free(table->list);
	  table->number = 0;
	  table->list = NULL;
	}
}

struct_path_node* Path_List_Init()
{ //initialize the list for the path
	struct_path_node *head; //head of doubly linked list for a path including nodes

    head = (struct_path_node*) calloc(1, sizeof(struct_path_node));
	assert_memory(head); //assert the memory allocation
	memset(head->vertex, 0, sizeof(head->vertex));
	head->weight = 0;
	head->next = head;
	head->prev = head;
	return head;
}

struct_path_node* Path_List_Reverse_Insert(struct_path_node *path_list, struct_set_node *u)
{ //insert the information of node u into path_list using head node such that node u is the closest to head's next pointer in the head node
	struct_path_node *path_node; //a new path node in the doubly linked list
	struct_graph_node *gnode; //the graph node
	struct_set_node *pnode; //the neighbor node towards source

        path_node = (struct_path_node*) calloc(1, sizeof(struct_path_node));
	assert_memory(path_node); //assert the memory allocation
	strcpy(path_node->vertex, u->node.vertex);
	gnode = u->node.gnode;
	pnode = u->node.pnode;

	if(pnode != NULL)
		path_node->weight = Weight(gnode, pnode->node.vertex);
	else
		path_node->weight = 0;

	/*
	path_node->prev = path_list->prev;
	path_list->prev->next = path_node;
	path_list->prev = path_node;
	path_node->next = path_list;
	*/

	path_node->next = path_list->next;
	path_list->next->prev = path_node;
	path_list->next = path_node;
	path_node->prev = path_list;

	return path_list;
}

struct_path_node* Path_List_Add(struct_path_node *path_list, struct_path_node *u)
{ //add the information of node u to path_list using head node such that node u is the farest to head's next pointer in the head node
	struct_path_node *path_node; //a new path node in the doubly linked list

        path_node = (struct_path_node*) calloc(1, sizeof(struct_path_node));
	assert_memory(path_node); //assert the memory allocation
	strcpy(path_node->vertex, u->vertex);
	
	path_node->prev = path_list->prev;
	path_list->prev->next = path_node;
	path_list->prev = path_node;
	path_node->next = path_list;

	return path_list;
}

double Weight(struct_graph_node *gnode, char *neighbor)
{ //return the edge weight between nodes u and v
	struct_graph_node *ptr = NULL;
	
	for(ptr = gnode; ptr != NULL; ptr = ptr->next)
	{
		if(strcmp(ptr->vertex, neighbor) == 0)
			return ptr->weight;
	}

	return INF; //v is not a neighbor of u
}

struct_path_node* Make_Path_List(struct_path_table *path_table, char *src, char *dst, int *path_hop_count)
{ //make a path list from src to dst using path_table
	struct_set_node *path_set = NULL, *src_ptr = NULL, *dst_ptr = NULL, *ptr = NULL;
	struct_path_node *path_list = NULL;
	int i;
	boolean flag = FALSE; //flag to indicate whether path_table entry for src exists or not

	for(i = 0; i < path_table->number; i++)
	{		
		path_set = path_table->list[i];
		src_ptr = path_set->next;
		if(strcmp(src_ptr->node.vertex, src) == 0)
		{
		        flag = TRUE;
			break;
		}
	}

	if(flag == FALSE)
	//if(src_ptr == NULL)
	{
		printf("Error: there is no shortest path set for src (%s)\n", src);
		exit(1);
	}

	for(ptr = src_ptr->next; ptr != path_set; ptr = ptr->next)
	{
		if(strcmp(ptr->node.vertex, dst) == 0)
		{
			dst_ptr = ptr;
			break;
		}
	}

	if(dst_ptr == NULL)
	{
		printf("Error: there is no shortest path for dst (%s)\n", dst);
		exit(1);
	}

	// make the actual path list
	path_list = Path_List_Init(); //initialize path_list creating a head node
	ptr = dst_ptr;
	*path_hop_count = 0; //initialize path_hop_count with zero
	do
	{
		Path_List_Reverse_Insert(path_list, (struct_set_node*) ptr);
		if(strcmp(ptr->node.vertex, src) == 0)
			break;

		ptr = ptr->node.pnode; //pnode points to the neighbor node towards source src
		(*path_hop_count)++; //increment path_hop_count
	} while(1);

	/* set path_list's length to *path_hop_count + 1 */
	path_list->weight = *path_hop_count + 1;

	return path_list;
}

struct_path_node* Make_Path_List_Before_The_Closest_Protection_Point(struct_path_table *path_table, char *src, char *dst, struct_traffic_table *protection_set, int *path_hop_count)
{ //make a path list from src to dst using path_table before the closest protection point on the path
	struct_set_node *path_set = NULL, *src_ptr = NULL, *dst_ptr = NULL, *ptr = NULL;
	struct_path_node *original_path_list = NULL; //original path list from src to dst
	struct_path_node *new_path_list = NULL; //new path list from src to the closest protection point towards dst
	int i;
	struct_path_node *path_ptr = NULL; //pointer to path node

	for(i = 0; i < path_table->number; i++)
	{		
		path_set = path_table->list[i];
		src_ptr = path_set->next;
		if(strcmp(src_ptr->node.vertex, src) == 0)
			break;
	}

	if(src_ptr == NULL)
	{
		printf("Error: there is no shortest path set for src (%s)\n", src);
		exit(1);
	}

	for(ptr = src_ptr->next; ptr != path_set; ptr = ptr->next)
	{
		if(strcmp(ptr->node.vertex, dst) == 0)
		{
			dst_ptr = ptr;
			break;
		}
	}

	if(dst_ptr == NULL)
	{
		printf("Error: there is no shortest path for dst (%s)\n", dst);
		exit(1);
	}

	/* make the original path list */
	original_path_list = Path_List_Init(); //initialize path_list creating a head node
	ptr = dst_ptr;
	*path_hop_count = 0; //initialize path_hop_count with zero
	do
	{
	  Path_List_Reverse_Insert(original_path_list, (struct_set_node*) ptr);
	  if(strcmp(ptr->node.vertex, src) == 0)
	    break;

	  ptr = ptr->node.pnode; //pnode points to the neighbor node towards source src
	  (*path_hop_count)++; //increment path_hop_count
	} while(1);
	
	/* make the new path list */
	*path_hop_count = 0; //reset path_hop_count to zero for a new path list where there is no protection point in the path except the destination
	new_path_list = Path_List_Init(); //initialize path_list creating a head node	
	path_ptr = original_path_list->next; //path_ptr points to the source vertex
	do
	{
	  Path_List_Add(new_path_list, path_ptr);
	  path_ptr = path_ptr->next;

	  (*path_hop_count)++; //increment path_hop_count
	} while(!IsProtectionPoint(path_ptr->vertex, protection_set));
	Path_List_Add(new_path_list, path_ptr); //add the protection point as the last vertex

	/* delete the original path list */
	Free_Path_List(original_path_list);

	/* set path_list's length to *path_hop_count + 1 */
	new_path_list->weight = *path_hop_count + 1;

	return new_path_list;
}

struct_path_node* Make_Path_List_For_Given_Trajectory(int *trajectory, int trajectory_size, struct_graph_node *G, int *path_hop_count)
{ //make a path list from src to dst using a given trajectory
	struct_path_node *path_list = NULL;
	struct_path_node *path_node = NULL;
	int node_id = 0;
	char node_name[NAME_SIZE];
	struct_graph_node *gnode = NULL; //the graph node
	int i;

	/** make the actual path list */
	path_list = Path_List_Init(); //initialize path_list creating a head node
	*path_hop_count = 0; //initialize path_hop_count with zero
	for(i = 0; i < trajectory_size; i++)
	{
	  path_node = (struct_path_node*) calloc(1, sizeof(struct_path_node));
	  assert_memory(path_node); //assert the memory allocation

	  node_id = trajectory[i];
	  
	  /* obtain node_name from node_id and the weight between two consecutive path nodes */
	  memset(node_name, 0, sizeof(node_name));
	  sprintf(node_name, "%d", node_id);
	  strcpy(path_node->vertex, node_name);
	  if(i == 0)
	    path_node->weight = 0;
	  else
	  {
	    path_node->weight = Weight(gnode, node_name);
	    (*path_hop_count)++; //increment path_hop_count
	  }

	  /* obtain the pointer to the graph node corresponding to node_id for the next path_node's weight */
          gnode = &(G[node_id-1]);

	  /* add path_node to the end of path_list */
	  path_node->prev = path_list->prev;
	  path_list->prev->next = path_node;
	  path_list->prev = path_node;
	  path_node->next = path_list;
	}

	/* set path_list's length to *path_hop_count + 1 */
	path_list->weight = *path_hop_count + 1;

	return path_list;
}

struct_path_node* Make_Forward_Path_List_With_Mobility_List(mobility_queue_t *Q, struct_graph_node *G, int G_size, int *path_hop_count)
{ //make a forward path list with a given mobility list as a new vehicle trajectory
  struct_path_node *path_list = NULL;
  struct_path_node *path_node = NULL;
  int node_id = 0;
  char node_name[NAME_SIZE];
  struct_graph_node *gnode = NULL; //the graph node
  mobility_queue_node_t *pQueueNode = NULL; //pointer to a mobility queue node
  int trajectory_size = Q->size; //trajectory size in mobility list
  int i;

  /** make the actual path list */
  path_list = Path_List_Init(); //initialize path_list creating a head node
  *path_hop_count = 0; //initialize path_hop_count with zero
  for(i = 0, pQueueNode = &(Q->head); i < trajectory_size; i++)
  {
    pQueueNode = pQueueNode->next; //forward direction on the path

    path_node = (struct_path_node*) calloc(1, sizeof(struct_path_node));
    assert_memory(path_node); //assert the memory allocation

    node_id = pQueueNode->intersection_id;
	  
    /* obtain node_name from node_id and the weight between two consecutive path nodes */
    memset(node_name, 0, sizeof(node_name));
    sprintf(node_name, "%d", node_id);
    strcpy(path_node->vertex, node_name);
    if(i == 0)
      path_node->weight = 0;
    else
    {
      path_node->weight = Weight(gnode, node_name);
      (*path_hop_count)++; //increment path_hop_count
    }

    /* obtain the pointer to the graph node corresponding to node_id for the next path_node's weight */
    if(node_id > G_size)
    {
      printf("Make_Forward_Path_List_With_Mobility_List(): Error:  node_id (%d) is greater than G_size (%d)\n", node_id, G_size);
      exit(1);
    }
    gnode = &(G[node_id-1]);

    /* add path_node to the end of path_list */
    path_node->prev = path_list->prev;
    path_list->prev->next = path_node;
    path_list->prev = path_node;
    path_node->next = path_list;
  }

  /* set path_list's length to *path_hop_count + 1 */
  path_list->weight = *path_hop_count + 1;

  return path_list;
}

struct_path_node* Make_Backward_Path_List_With_Mobility_List(mobility_queue_t *Q, struct_graph_node *G, int G_size, int *path_hop_count)
{ //make a backward path list with a given mobility list as a new vehicle trajectory
  struct_path_node *path_list = NULL;
  struct_path_node *path_node = NULL;
  int node_id = 0;
  char node_name[NAME_SIZE];
  struct_graph_node *gnode = NULL; //the graph node
  mobility_queue_node_t *pQueueNode = NULL; //pointer to a mobility queue node
  int trajectory_size = Q->size; //trajectory size in mobility list
  int i;

  /** make the actual path list */
  path_list = Path_List_Init(); //initialize path_list creating a head node
  *path_hop_count = 0; //initialize path_hop_count with zero
  for(i = 0, pQueueNode = &(Q->head); i < trajectory_size; i++)
  {
    pQueueNode = pQueueNode->prev; //backward direction on the path

    path_node = (struct_path_node*) calloc(1, sizeof(struct_path_node));
    assert_memory(path_node); //assert the memory allocation

    node_id = pQueueNode->intersection_id;
	  
    /* obtain node_name from node_id and the weight between two consecutive path nodes */
    memset(node_name, 0, sizeof(node_name));
    sprintf(node_name, "%d", node_id);
    strcpy(path_node->vertex, node_name);
    if(i == 0)
      path_node->weight = 0;
    else
    {
      path_node->weight = Weight(gnode, node_name);
      (*path_hop_count)++; //increment path_hop_count
    }

    /* obtain the pointer to the graph node corresponding to node_id for the next path_node's weight */
    if(node_id > G_size)
    {
      printf("Make_Backward_Path_List_With_Mobility_List(): Error:  node_id (%d) is greater than G_size (%d)\n", node_id, G_size);
      exit(1);
    }
    gnode = &(G[node_id-1]);

    /* add path_node to the end of path_list */
    path_node->prev = path_list->prev;
    path_list->prev->next = path_node;
    path_list->prev = path_node;
    path_node->next = path_list;
  }

  /* set path_list's length to *path_hop_count + 1 */
  path_list->weight = *path_hop_count + 1;

  return path_list;
}

void Free_Path_List(struct_path_node *path_list)
{ //release the memory occupied by the list for the shortest path from source to destination
	struct_path_node *head = path_list, *ptr = NULL, *removed_node = NULL;

	for(ptr = head->next; ptr != head;)
	{
		removed_node = ptr;
		ptr = ptr->next;
		free(removed_node);
	}

	free(head);
}

void Initialize_Schedule_Table(struct_traffic_table *src_table, struct_schedule_table *sched_table) 
{ //initialize the schedule table to generate the traffic according to the source node and the corresponding distribution
	int i;

	sched_table->number = src_table->number;
	//sched_table->entry = (struct_schedule_entry*) malloc(sched_table->number*sizeof(struct_schedule_entry));
        sched_table->entry = (struct_schedule_entry*) calloc(sched_table->number, sizeof(struct_schedule_entry));
	assert_memory(sched_table->entry);

	for(i = 0; i < sched_table->number; i++)
	{
		//strcpy(sched_table->entry[i].vertex, src_table->list[i]);
		strcpy(sched_table->entry[i].vertex, src_table->list[i].vertex);
		sched_table->entry[i].timestamp = 0;
		//sched_table->entry[i].sched_time = 0;
		sched_table->entry[i].sched_time = INF;
	}
}

int Lookup_Schedule_Table_Index(struct_schedule_table *sched_table, char *src)
{ //return the index corresponding to src in sched_table
	int i = -1;

	for(i = 0; i < sched_table->number; i++)
	{
		if(strcmp(sched_table->entry[i].vertex, src) == 0)
			break;
	}

	return i;
}

char* Find_Traffic_Source(struct_schedule_table *sched_table, double current_time)
{ //find the traffic source corresponding to the current time having a new vehicle arrival
	int i;

	for(i = 0; i < sched_table->number; i++)
	{
		if(current_time == sched_table->entry[i].sched_time)
			return sched_table->entry[i].vertex;
	}

	return NULL;
}

int Update_Schedule_Table(struct_schedule_table *sched_table, char *src, double current_time, double delay)
{ //update the time fields corresponding to the traffic source in schedule table
	int i;
	double schedule_time;

	schedule_time = current_time + delay;

	for(i = 0; i < sched_table->number; i++)
	{
		if(strcmp(src, sched_table->entry[i].vertex) == 0)
		{
			sched_table->entry[i].timestamp = current_time;
			sched_table->entry[i].sched_time = schedule_time;
			return 0;
		}
	}

	return -1;
}

void Free_Schedule_Table(struct_schedule_table *table)
{ //release the memory occupied by the schedule table
        if(table->number > 0)
	{
	  free(table->entry);
 	  table->number = 0;	
	  table->entry = NULL;
	}
}

boolean IsEndOfTravel(struct_path_node *path_list, struct_path_node *path_ptr)
{ //check whether the vehicle has arrived at its destination or whether path_ptr->vertex is equal to the destination or not
        if(path_list->prev == path_ptr)
		return TRUE; //path_list->prev indicates the destination vertex
	else
		return FALSE;
}

boolean IsProtectionPoint(char *path_vertex, struct_traffic_table *protection_set)
{ //check whether the vehicle has arrived at one of protection points
  boolean result = FALSE;
  char *pVertex = NULL; //pointer to a vertex that is one of protection points
  int i = 0; //for-loop index

  for(i = 0; i < protection_set->number; i++)
  {
    pVertex = protection_set->list[i].vertex; //get the pointer to the vertex corresponding to the i-th protection point
    if(strcmp(path_vertex, pVertex) == 0)
    {
      result = TRUE;
      break;
    }
  }

  return result;
}

boolean IsNeighbor(int X, int Y, struct_graph_node *G, int G_size)
{ //check whether X is the neighbor of Y or not
	boolean flag = FALSE;
	struct_graph_node *node = &G[X-1];
	int v; //neighbor

	while(node != NULL)
	{
		v = atoi(node->vertex);
		if(v == Y)
		{
			flag = TRUE;
			break;
		}
	}

	return flag;
}

void Store_Graph_Into_File_As_AdjacencyMatrix(struct_graph_node *G, int G_size, char* filename)
{ //store the adjacency list of graph G into a file in the form of adjacency matrix
	FILE *fp; //path-table file
	int i, j;
	double **A = NULL; //adjacency matrix
	char *msg = NULL;
	char *buf = NULL;
	//char msg[MSG_BUF_SIZE]; 
	//Where n is a large number, such as 18, the memory access violation happens.
	//So we need to dynamically allocate the memory for msg and buf according to n.
	//char buf[BUF_SIZE];
	const int number_string_length = NUMBER_STRING_LEN; //length of number string
	int buf_len; //buf length
	int msg_len; //msg length
	int n = G_size; //number of graph nodes

	A = Construct_Adjacency_Matrix(G, G_size);
	//construct an adjacency matrix from the adjacent list G representing the graph
	
	buf_len = sizeof(char)*n*number_string_length;
	//buf = (char*) malloc(buf_len);
	buf = (char*) calloc(buf_len, sizeof(char));
	assert_memory(buf);

	msg_len = sizeof(char)*n*n*number_string_length;
	//msg = (char*) malloc(msg_len);
	msg = (char*) calloc(msg_len, sizeof(char));
	assert_memory(msg);
	memset(msg, 0, msg_len);

	for(i = 0; i < n; i++)
	{
		for(j = 0; j < n; j++)
		{
			sprintf(buf, "%f ", A[i][j]);
			strcat(msg, buf);
		}
		strcat(msg, "\n");
	}

	fp = fopen(filename, "w");
	if(!fp)
	{
		fprintf(stderr, "Error: unable to open file \"%s\"\n", filename);
		exit(1);
	}

#ifdef __DEBUG_LEVEL_1__
	printf(msg);
#endif
	fprintf(fp, msg);
	fclose(fp);

	/* release memory */
	free(buf);
	free(msg);
}

void Store_Graph_Into_File_As_AdjacencyList(struct_graph_node *G, int G_size, char* filename, int indicator)
{ //store the adjacency list of graph G into a file in the form of adjacency list with indicator that determines the title text
	FILE *fp; //path-table file
	int i, j;

	char title1[] = "### Initial Virtual Graph including Sensing Hole Endpoints ###\n\n";
	size_t title1_len = strlen(title1);
	char title2[] = "### Final Virtual Graph including Sensing Hole Endpoints ###\n\n";
	size_t title2_len = strlen(title2);
	char title3[] = "### Intermediate Virtual Graph including Sensing Hole Endpoints ###\n\n";
	size_t title3_len = strlen(title3);
	size_t title_len; //maximum title length among title1_len, title2_len and title3_len

	char *msg = NULL;
	char *buf = NULL;
	//char msg[MSG_BUF_SIZE]; 
	//Where n is a large number, such as 18, the memory access violation happens.
	//So we need to dynamically allocate the memory for msg and buf according to n.
	//char buf[BUF_SIZE];
	const int number_string_length = NUMBER_STRING_LEN; //length of number string
	size_t buf_len; //buf length
	size_t msg_len; //msg length
	int n = G_size; //number of graph nodes
	struct_graph_node *pGraphNode = NULL; //pointer to a graph node
	char *tail_node = NULL; //tail node of an edge
	char *head_node = NULL; //head node of an edge

	/* find the maximum title length among title1_len, title2_len and title3_len */
	title_len = MAX(title1_len, title2_len);
	title_len = MAX(title_len, title3_len);

	buf_len = sizeof(char)*n*number_string_length + title_len;
	//buf = (char*) malloc(buf_len);
	buf = (char*) calloc(buf_len, sizeof(char));
	assert_memory(buf);
	
	//msg_len = sizeof(char)*n*n*number_string_length;
	msg_len = sizeof(char)*n*n*number_string_length + title_len*2;
	//msg = (char*) malloc(msg_len);
	msg = (char*) calloc(msg_len, sizeof(char));
	assert_memory(msg);
	//memset(msg, 0, msg_len);

	if(indicator == 0)
		strcpy(msg, title1);
	else if(indicator == 1)
		strcpy(msg, title2);
	else
		strcpy(msg, title3);

	for(i = 0; i < G_size; i++)
	{
		pGraphNode = &(G[i]);
		tail_node = G[i].vertex;
		sprintf(buf, "vertex %d (%s):\t", i+1, (G[i].type==INTERSECTION_GRAPH_NODE? "intersection" : "hole endpoint"));
		strcat(msg, buf);
		for(j = 0; j < G[i].weight; j++)
		{
			pGraphNode = pGraphNode->next;
			head_node = pGraphNode->vertex;
			if(j < G[i].weight-1)
				sprintf(buf, "(%s,%s):%f, ", tail_node, head_node, (float)pGraphNode->weight);
			else
				sprintf(buf, "(%s,%s):%f.", tail_node, head_node, (float)pGraphNode->weight);

			strcat(msg, buf);
		}
		strcat(msg, "\n\n");
	}

	fp = fopen(filename, "w");
	if(!fp)
	{
		fprintf(stderr, "Error: unable to open file \"%s\"\n", filename);
		exit(1);
	}

#ifdef __DEBUG_LEVEL_1__
	printf(msg);
#endif
	fprintf(fp, msg);
	fclose(fp);

	/* release memory */
	free(buf);
	free(msg);
}

void Store_Graph_Into_File_As_AdjacencyList_With_VANET_Statistics(parameter_t *param, struct_graph_node *G, int G_size, char* filename, int indicator)
{ //store the adjacency list of graph G into a file for VANET statistics in the form of adjacency list with indicator that determines the title text
	FILE *fp; //path-table file
	int i, j;

	char title1[] = "### Road Graph with VANET Type-1 Statistics (Branching Probability) ###\n\n";
	size_t title1_len = strlen(title1);
	char title2[] = "### Road Graph with VANET Type-2 Statistics (Mean Interarrival Time and Average Convoy Length (ACL)) ###\n\n";
	size_t title2_len = strlen(title2);
	char title3[] = "### Road Graph with VANET Type-3 Statistics (Forwarding Probability) ###\n\n";
	size_t title3_len = strlen(title3);
	char title4[] = "### Road Graph with VANET Type-4 Statistics (Expected Delivery Delay (EDD)) ###\n\n";
	size_t title4_len = strlen(title4);
	size_t title_len; //maximum title length among title1_len, title2_len, title3_len and title4_len

	char format1[] = "\n Edge format: {<tail_node,head_node>:length, [number_of_branching, branching_probability]}\n\n";
	size_t format1_len = strlen(format1);
	char format2[] = "\n Edge format: {<tail_node,head_node>:length, [number_of_interarrivals, mean_interarrival_time; expected_average_convoy_length, actual_average_convoy_length]}\n\n";
	size_t format2_len = strlen(format2);
	char format3[] = "\n Edge format: {<tail_node,head_node>:length, [theta, CP, P_prime, P_prime_pure, P]}\n\n";
	size_t format3_len = strlen(format3);
	char format4[] = "\n Edge format: {<tail_node,head_node>:length, [theta, edge_delay, EDD]}\n\n";
	size_t format4_len = strlen(format4);
	size_t format_len; //maximum format length among format1_len, format2_len, format3_len and format4_len

	char *msg = NULL;
	char *buf = NULL;
	//char msg[MSG_BUF_SIZE]; 
	//Where n is a large number, such as 18, the memory access violation happens.
	//So we need to dynamically allocate the memory for msg and buf according to n.
	//char buf[BUF_SIZE];
	const int number_string_length = NUMBER_STRING_LEN; //length of number string
	size_t buf_len; //buf length
	size_t msg_len; //msg length
	int n = G_size; //number of graph nodes
	struct_graph_node *pGraphNode = NULL; //pointer to a graph node
	char *tail_node = NULL; //tail node of an edge
	char *head_node = NULL; //head node of an edge
	int number_of_arrivals = 0; //number of vehicle arrivals on the node
	int margin = 200; //margin for msg_len for tail vertex's information
	size_t actual_msg_len = 0; //actual message length
	double acl_measurement_time = 0; //ACL Measurement Time

	/* find the maximum title length among title1_len, title2_len, title3_len and title4_len */
	title_len = MAX(title1_len, title2_len);
	title_len = MAX(title_len, title3_len);
	title_len = MAX(title_len, title4_len);

	/* find the maximum format length among format1_len, format2_len, format3_len and format4_len */
	format_len = MAX(format1_len, format2_len);
	format_len = MAX(format_len, format3_len);
	format_len = MAX(format_len, format4_len);

	buf_len = sizeof(char)*n*number_string_length + title_len + format_len;
	//buf = (char*) malloc(buf_len);
	buf = (char*) calloc(buf_len, sizeof(char));
	assert_memory(buf);
	
	//msg_len = sizeof(char)*n*n*number_string_length;
	msg_len = sizeof(char)*n*n*number_string_length + title_len*2 + format_len*2 + margin;
	//msg = (char*) malloc(msg_len);
	msg = (char*) calloc(msg_len, sizeof(char));
	assert_memory(msg);
	//memset(msg, 0, msg_len);

	if(indicator == 1) //branching probability
        {
	  strcpy(msg, title1);
	  strcat(msg, format1);

	  //@for debugging
	  actual_msg_len += title3_len + format3_len;
	  ////////////////
	}
	else if(indicator == 2) //mean interarrival time
	{
	  strcpy(msg, title2);
	  strcat(msg, format2);

	  //@for debugging
	  actual_msg_len += title3_len + format3_len;
	  ////////////////
	}
	else if(indicator == 3) //forwarding probability
	{
	  strcpy(msg, title3);
	  strcat(msg, format3);
	  
	  //@for debugging
	  actual_msg_len += title3_len + format3_len;
	  ////////////////
	}
	else if(indicator == 4) //EDD
	{
	  strcpy(msg, title4);
	  strcat(msg, format4);

	  //@for debugging
	  actual_msg_len += title3_len + format3_len;
	  ////////////////
	}
	else
	{
	  printf("Store_Graph_Into_File_As_AdjacencyList_With_VANET_Statistics(): Error: indicator(%d) is not supported yet!\n", indicator);
	  exit(1);
	}

	for(i = 0; i < G_size; i++)
	{
		pGraphNode = &(G[i]);
		tail_node = G[i].vertex;

		/** print the tail node with the number of arrivals */
		number_of_arrivals =  pGraphNode->number_of_arrivals;
		sprintf(buf, "vertex %d (%s) [%d]: ", i+1, (G[i].role==ROLE_INTERSECTION_POINT? "intersection" : "access point"), number_of_arrivals);

		strcat(msg, buf);

		//@for debugging
		actual_msg_len += strlen(buf); 
		////////////////

		/** print the head nodes with either branching probability or mean interarrival time */
		for(j = 0; j < G[i].weight; j++)
		{
			pGraphNode = pGraphNode->next;
			head_node = pGraphNode->vertex;
			if(j < G[i].weight-1)
			{
			  if(indicator == 1) //branching probability
			  {
			    if(number_of_arrivals > 0)
			      sprintf(buf, "{<%s, %s>:%.1f, [%d, %.2f]}, ", tail_node, head_node, (float)pGraphNode->weight, pGraphNode->number_of_branching, (float)pGraphNode->number_of_branching/number_of_arrivals);
			    else
			      sprintf(buf, "{<%s, %s>:%.1f, [%d, %.2f]}, ", tail_node, head_node, (float)pGraphNode->weight, pGraphNode->number_of_branching, (float)0);
			  }
			  else if(indicator == 2) //mean interarrival time and average convoy lengths
			  {
			    /* obtain the expected average convoy length and the actual average convoy length */
			    pGraphNode->expected_average_convoy_length = VADD_Compute_Average_Convoy_Length(param, pGraphNode);
			    //pGraphNode->actual_average_convoy_length = pGraphNode->ptr_directional_edge_node->acl_convoy_area / param->simulation_time;
			    //acl_measurement_time = pGraphNode->ptr_directional_edge_node->acl_convoy_tail_arrival_time;
			    //acl_measurement_time =  param->simulation_time;
                            //acl_measurement_time = MAX(pGraphNode->ptr_directional_edge_node->acl_convoy_tail_arrival_time, pGraphNode->ptr_directional_edge_node->acl_convoy_head_departure_time);
			    acl_measurement_time = pGraphNode->ptr_directional_edge_node->acl_measurement_end_time - pGraphNode->ptr_directional_edge_node->acl_measurement_start_time;
			    pGraphNode->actual_average_convoy_length = pGraphNode->ptr_directional_edge_node->acl_convoy_area / acl_measurement_time;

			    if(pGraphNode->number_of_interarrivals > 0)
			      sprintf(buf, "{<%s, %s>:%.1f, [%d, %.2f; %.2f, %.2f]}, ", tail_node, head_node, (float)pGraphNode->weight, pGraphNode->number_of_interarrivals, (float)pGraphNode->sum_of_interarrival_time/pGraphNode->number_of_interarrivals, (float)pGraphNode->expected_average_convoy_length, (float)pGraphNode->actual_average_convoy_length);
			    else
			      sprintf(buf, "{<%s, %s>:%.1f, [%d, %s; %.2f, %.2f]}, ", tail_node, head_node, (float)pGraphNode->weight, pGraphNode->number_of_interarrivals, "INF", (float)pGraphNode->expected_average_convoy_length, (float)pGraphNode->actual_average_convoy_length);
			  }
			  else if(indicator == 3) //forwarding probability
			  {
			      sprintf(buf, "{<%s, %s>:%.1f, [%.2f, %.2f, %.2f, %.2f, %.2f]}, ", tail_node, head_node, (float)pGraphNode->weight, (float)pGraphNode->theta, (float)pGraphNode->CP, (float)pGraphNode->P_prime, (float)pGraphNode->P_prime_pure, (float)pGraphNode->P);
			  }
			  else if(indicator == 4) //EDD
			  {
			      sprintf(buf, "{<%s, %s>:%.1f, [%.2f, %.2f, %.2f]}, ", tail_node, head_node, (float)pGraphNode->weight, (float)pGraphNode->theta, (float)pGraphNode->edge_delay, (float)pGraphNode->EDD);
			  }
			}
			else
			{
			  if(indicator == 1) //branching probability
			  {
			    if(number_of_arrivals > 0)
			      sprintf(buf, "{<%s, %s>:%.1f, [%d, %.2f]}.", tail_node, head_node, (float)pGraphNode->weight, pGraphNode->number_of_branching, (float)pGraphNode->number_of_branching/number_of_arrivals);
			    else
			      sprintf(buf, "{<%s, %s>:%.1f, [%d, %.2f]}.", tail_node, head_node, (float)pGraphNode->weight, pGraphNode->number_of_branching, (float)0);
			  }
			  else if(indicator == 2) //mean interarrival time and average convoy lengths
			  {
			    /* obtain the expected average convoy length and the actual average convoy length */
			    pGraphNode->expected_average_convoy_length = VADD_Compute_Average_Convoy_Length(param, pGraphNode);
			    //pGraphNode->actual_average_convoy_length = pGraphNode->ptr_directional_edge_node->acl_convoy_area / param->simulation_time;

			    //acl_measurement_time = pGraphNode->ptr_directional_edge_node->acl_convoy_tail_arrival_time;
			    //acl_measurement_time =  param->simulation_time;
			    //acl_measurement_time = MAX(pGraphNode->ptr_directional_edge_node->acl_convoy_tail_arrival_time, pGraphNode->ptr_directional_edge_node->acl_convoy_head_departure_time);
			    acl_measurement_time = pGraphNode->ptr_directional_edge_node->acl_measurement_end_time - pGraphNode->ptr_directional_edge_node->acl_measurement_start_time;
			    pGraphNode->actual_average_convoy_length = pGraphNode->ptr_directional_edge_node->acl_convoy_area / acl_measurement_time;


			    if(pGraphNode->number_of_interarrivals > 0)
			      sprintf(buf, "{<%s,%s>:%.1f, [%d, %.2f; %.2f, %.2f]}.", tail_node, head_node, (float)pGraphNode->weight, pGraphNode->number_of_interarrivals, (float)pGraphNode->sum_of_interarrival_time/pGraphNode->number_of_interarrivals, (float)pGraphNode->expected_average_convoy_length, (float)pGraphNode->actual_average_convoy_length);
			    else
			      sprintf(buf, "{<%s, %s>:%.1f, [%d, %s; %.2f, %.2f]}.", tail_node, head_node, (float)pGraphNode->weight, pGraphNode->number_of_interarrivals, "INF", (float)pGraphNode->expected_average_convoy_length, (float)pGraphNode->actual_average_convoy_length);
			  }
			  else if(indicator == 3) //forwarding probability
			  {
			      sprintf(buf, "{<%s, %s>:%.1f, [%.2f, %.2f, %.2f, %.2f, %.2f]}. ", tail_node, head_node, (float)pGraphNode->weight, (float)pGraphNode->theta, (float)pGraphNode->CP, (float)pGraphNode->P_prime, (float)pGraphNode->P_prime_pure, (float)pGraphNode->P);
			  }
			  else if(indicator == 4) //EDD
			  {
			      sprintf(buf, "{<%s, %s>:%.1f, [%.2f, %.2f, %.2f]}. ", tail_node, head_node, (float)pGraphNode->weight, (float)pGraphNode->theta, (float)pGraphNode->edge_delay, (float)pGraphNode->EDD);
			  }
			}

			strcat(msg, buf);

			//@for debugging
			actual_msg_len += strlen(buf); 
			////////////////
		}
		strcat(msg, "\n\n");

		//@for debugging
		actual_msg_len += 2; 
		////////////////
	}

	fp = fopen(filename, "w");
	if(!fp)
	{
		fprintf(stderr, "Error: unable to open file \"%s\"\n", filename);
		exit(1);
	}

#ifdef __DEBUG_LEVEL_1__
	printf(msg);
#endif
	fprintf(fp, msg);
	fclose(fp);

	/* release memory */
	free(buf);
	free(msg);
}

double** Construct_Adjacency_Matrix(struct_graph_node *G, int G_size)
{ //construct an adjacency matrix from the adjacent list G representing the graph
	double **W = NULL; //adjacency matrix
	int i, j;
	struct_graph_node *ptr = NULL; //pointer to graph node

	W = (double**)calloc(G_size, sizeof(double*));
	assert_memory(W);

	for(i = 0; i < G_size; i++)
	{
		W[i] = (double*)calloc(G_size, sizeof(double));
		assert_memory(W[i]);

		for(j = 0; j < G_size; j++)
			W[i][j] = 0;
	}

	/* initialize the weight matrix W with adjacent list G */
	for(i = 0; i < G_size; i++)
	{
		ptr = G[i].next;
		if(ptr == NULL)
			continue;

		while(ptr != NULL)
		{
			j = atoi(ptr->vertex) - 1; 
			//node id starts from 1, but it should decrease by 1 for weight matrix where the id starts from 0.
			W[i][j] = ptr->weight;
			ptr = ptr->next;
		}
	}

	return W;
}

struct_graph_node* LookupGraph(struct_graph_node *G, int G_size, char *u)
{ //return the pointer to graph node corresponding to vertex u's name
	struct_graph_node *node = NULL;
	int node_id = atoi(u);
      	
	/*
	for(i = 0; i < G_size; i++)
	{
		if(strcmp(u, G[i].vertex) == 0)
		{
			node = &(G[i]);
			break;
		}
	}
	*/

	if(G == NULL)
	{
	  printf("LookupGraph(): G is NULL\n");
	  exit(1);
	}
	else if(node_id > G_size)
	{
	  printf("LookupGraph(): node_id(%d) > G_size(%d)\n", node_id, G_size);
	  exit(1);
	}
	
	node = &(G[node_id-1]);

	if(node->status == USAGE_UNUSED)
	  return NULL;
	else
	  return node;
}

struct_graph_node* LookupGraph_By_NodeID(struct_graph_node *G, int G_size, int node_id)
{ //return the pointer to graph node corresponding to node id
	struct_graph_node *node = NULL;

	if(G == NULL)
	{
	  printf("LookupGraph_By_NodeID(): G is NULL\n");
	  exit(1);
	}
	else if(node_id > G_size)
	{
	  printf("LookupGraph_By_NodeID(): node_id(%d) > G_size(%d)\n", node_id, G_size);
	  exit(1);
	}
	
	node = &(G[node_id-1]);

	if(node->status == USAGE_UNUSED)
	  return NULL;
	else
	  return node;
}

struct_graph_node* GetNeighborGraphNode(struct_graph_node *G, int G_size, char *u, char *v)
{ //return the pointer to graph node corresponding to vertex v that is vertex u's neighbor
	struct_graph_node *node = NULL;
	struct_graph_node *neighbor_node = NULL; //neighbor graph node
	int node_id = atoi(u);
	      	
	if(G == NULL)
	{
	  printf("GetNeighborGraphNode(): G is NULL\n");
	  exit(1);
	}
	else if(node_id > G_size)
	{
	  printf("GetNeighborGraphNode(): node_id(%d) > G_size(%d)\n", node_id, G_size);
	  exit(1);
	}
	
	node = &(G[node_id-1]);

	for(neighbor_node = node->next; neighbor_node != NULL; neighbor_node = neighbor_node->next)
	{
	  if(strcmp(v, neighbor_node->vertex) == 0)
	  {
	    break;
	  }	  
	}

	return neighbor_node;
}

double FindShortestPath(struct_traffic_table *src_table, struct_traffic_table *dst_table, int G_size, double **D, char *shortest_path_src, char *shortest_path_dst)
{ //compute the length of the shortest path from Enter nodes to Exit nodes along with source and destination
	int n = G_size; //size of vertex set; that is, the number of vertices in the graph for the road network
	int i, j; //indices of for-loops
	double min = INF; //minimum distance
	int u, v; //vertices

	for(i = 0; i < src_table->number; i++)
	{
		//u = atoi(src_table->list[i]);
		u = atoi(src_table->list[i].vertex);
		for(j = 0; j < dst_table->number; j++)
		{
			//v = atoi(dst_table->list[j]);
			v = atoi(dst_table->list[j].vertex);
			if(D[u-1][v-1] < min)
			{
				min = D[u-1][v-1];
				//strcpy(shortest_path_src, src_table->list[i]);
				strcpy(shortest_path_src, src_table->list[i].vertex);
				//strcpy(shortest_path_dst, dst_table->list[j]);
				strcpy(shortest_path_dst, dst_table->list[j].vertex);
			}
		}
	}

	return min;
}

int GetSensorNumberForShortestPath(char *src, char *dst, struct_graph_node *G, int n, int **M)
{ //get the number of sensors (that are live or dead) on the shortest path from src to dst
	int path_sensor_num = 0; //number of sensors deployed on the shortest path from src to dst
	path_queue_t *Q = NULL; //pointer to the path queue for the shortest path from src to dst
	path_queue_node_t *pQueueNode = NULL; //pointer to a queue node
	schedule_table_node_t *pTableNode = NULL; //pointer to a schedule table node
	int src_node; //source node
	int dst_node; //destination node
	char tail_node[NAME_SIZE];  //tail node of a directional edge
	char head_node[NAME_SIZE];  //head node of a directional edge	
	int i = 0; //index for for-loop
	boolean flip_flag; //flag for table lookup
	
	src_node = atoi(src);
	dst_node = atoi(dst);

	Q = Floyd_Warshall_Make_Shortest_Path_Queue(M, n, src_node, dst_node);
	//make a queue containing nodes of the shortest path from src_node to dst_node
	
    pQueueNode = (path_queue_node_t*) GetQueueNode((queue_t*)Q, i);
	strcpy(tail_node, pQueueNode->node);
	for(i = 1; i < Q->size; i++)
	{
		pQueueNode = (path_queue_node_t*) GetQueueNode((queue_t*)Q, i);
		strcpy(head_node, pQueueNode->node);
		pTableNode = LookupTable(G, tail_node, head_node, &flip_flag);
		path_sensor_num += pTableNode->sensor_list.size;        
		strcpy(tail_node, head_node); //make head_node be tail_node for retrieving next edge towards src_node
	}

	/* release the dynamic memory allocated to Q's head */
	DestroyQueue((queue_t*)Q);

	return path_sensor_num;
}

boolean Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(struct_graph_node **Gv, int *Gv_size, schedule_table_t *T, schedule_table_node_t *pTableNode, double left_hole_offset, double right_hole_offset, edge_queue_t *Er, struct_traffic_table *added_hole_set, struct_traffic_table *deleted_hole_set)
{//update virtual graph Gv, schedule table T, and edge queue Er using pTableNode, left_hole_offset and right_hole_offset
	char tail_node[NAME_SIZE]; //tail node of a directional edge
	char head_node[NAME_SIZE]; //head node of a directional edge
	int tail_node_id = 0; //tail node id
	int head_node_id = 0; //head node id
	struct_graph_node *ptr_tail_node = NULL; //pointer to the graph node corresponding to tail_node_id
	struct_graph_node *ptr_head_node = NULL; //pointer to the graph node corresponding to head_node_id
	struct_graph_node *ptr_neighbor_graph_node = NULL; //pointer to a neighbor graph node
	struct_graph_node *ptr_neighbor_graph_node_1 = NULL; //pointer to a neighbor graph node
	struct_graph_node *ptr_neighbor_graph_node_2 = NULL; //pointer to a neighbor graph node
	struct_graph_node neighbor_graph_node; //copy for a neighbor graph node
	struct_graph_node neighbor_graph_node_1; //copy for a neighbor graph node
	struct_graph_node neighbor_graph_node_2; //copy for a neighbor graph node

	struct_graph_node *ptr_new_graph_node = NULL; //pointer to a new graph node
	struct_graph_node *ptr_new_graph_node_1 = NULL; //pointer to a new graph node
	struct_graph_node *ptr_new_graph_node_2 = NULL; //pointer to a new graph node
	struct_graph_node new_graph_node; //copy for a new graph node
	struct_graph_node new_graph_node_1; //copy for a new graph node
	struct_graph_node new_graph_node_2; //copy for a new graph node

	double weight = pTableNode->weight; //edge weight; that is, the length of the edge
	double density = pTableNode->density; //density of sensors deployed for unit distance
	GRAPH_NODE_TYPE type = UNKNOWN_GRAPH_NODE_TYPE; //graph node type

	double hole_segment_length = UNKNOWN_GRAPH_NODE_TYPE; //hole segment length
	double segment_length_1, segment_length_2; //road segment length
	schedule_table_node_t *ptr_table_node = NULL; //pointer to table node
	schedule_table_node_t *ptr_table_node_for_hole_segment = NULL; //pointer to the table node corresponding to the hole segment
	schedule_table_node_t *ptr_table_node_for_hole_segment_1 = NULL; //pointer to the table node corresponding to the first hole segment
	schedule_table_node_t *ptr_table_node_for_hole_segment_2 = NULL; //pointer to the table node corresponding to the second hole segment
	schedule_table_node_t *neighbor_table_node = NULL; //pointer to neighbor table node for ptr_table_node
	schedule_table_node_t *neighbor_table_node_1 = NULL; //pointer to neighbor table node for ptr_table_node
	schedule_table_node_t *neighbor_table_node_2 = NULL; //pointer to neighbor table node for ptr_table_node
	schedule_table_node_t table_node; //node for schedule table

	//sensor_queue_t sensor_list_copy; //the copy of the list of sensor nodes located on this edge
	sensor_queue_t new_sensor_list; //list of sensor nodes located on the merged virtual edge
	sensor_queue_node_t *ptr_queue_node = NULL; //pointer to sensor queue node
	sensor_queue_node_t *pQueueNode = NULL; //pointer to the newly added sensor queue node
	sensor_queue_node_t sensor_queue_node; //sensor queue node used for inserting a new queue node into sensor queue list
	int order_for_live_sensors = 0; //order for live sensors in sensor list in the virtual graph Gv
	int order_in_Gv = 0; //order for sensor on the edge in the virtual graph Gv
	int sensor_list_size = pTableNode->sensor_list.size; //number of sensors in sensor_list
	int count = 0; //count for reading as many as sensor_list.size
	subedge_queue_t subedge_queue; //subedge queue for newly subdivided edges that replaces the subedge having a new hole segment
	subedge_queue_node_t subedge_node; //subedge queue node
	subedge_queue_node_t *ptr_subedge_node = NULL; //pointer to a subedge node
	edge_queue_node_t *pEdgeNode = NULL; //pointer to an edge queue node
	double virtual_offset = -1; //offset of a sensing hole endpoint in virtual graph Gv
	boolean flip_flag = FALSE; //flag to check whether the tail node of (tail_node,head_node) in LookupTable() is the actual tail node in schedule table entry
	boolean flip_flag_1 = FALSE; //flag to check whether the tail node of (tail_node,head_node) in LookupTable() is the actual tail node in schedule table entry
	boolean flip_flag_2 = FALSE; //flag to check whether the tail node of (tail_node,head_node) in LookupTable() is the actual tail node in schedule table entry
	GRAPH_NODE_TYPE tail_node_type = UNKNOWN_GRAPH_NODE_TYPE; //graph node type of tail node
	GRAPH_NODE_TYPE head_node_type = UNKNOWN_GRAPH_NODE_TYPE; //graph node type of head node

	GRAPH_NODE_ROLE tail_node_role = ROLE_UNKNOWN; //graph node role of tail node
	GRAPH_NODE_ROLE head_node_role = ROLE_UNKNOWN; //graph node role of head node

	hole_endpoint_queue_node_t left_hole_endpoint_node; //left hole endpoint queue node in a hole segment
	hole_endpoint_queue_node_t right_hole_endpoint_node; //right hole endpoint queue node in a hole segment
	//hole_endpoint_queue_node_t hole_endpoint_node; //hole endpoint queue node in a hole segment
	//@NOTE that the hole_endpoint_node needs to be deleted later.

	hole_endpoint_queue_node_t *left_hole_endpoint = NULL; //pointer to the pointer to the left hole-endpoint of a hole segment
	hole_endpoint_queue_node_t *right_hole_endpoint = NULL; //pointer to the pointer to the right hole-endpoint of a hole segment

	double virtual_offset_of_left_hole = 0; //virtual offset of the left hole-endpoint in virtual graph Gv
	double virtual_offset_of_right_hole = 0; //virtual offset of the right hole-endpoint in virtual graph Gv

	double physical_offset_of_left_hole = 0; //physical offset of the left hole-endpoint in real graph Gr
	double physical_offset_of_right_hole = 0; //physical offset of the right hole-endpoint in real graph Gr
	int hole_number = 0; //number of hole endpoints

	/* initialize tail_node and head_node information with pTableNode */
	strcpy(tail_node, pTableNode->tail_node); //tail node of a directional edge
	strcpy(head_node, pTableNode->head_node); //head node of a directional edge
	tail_node_id = atoi(tail_node); //tail node id
	head_node_id = atoi(head_node); //head node id

	/* initialize subedge queue */
	InitQueue((queue_t*) &subedge_queue, QTYPE_SUBEDGE);

	/* get the pointer to an edge queue node in Er corresponding to the edge pointed by pTableNode */
	pEdgeNode = pTableNode->subedge->edge_queue_entry; //pEdgeNode points to the edge table entry having the subedge list that contains the subedge corresponding to the schedule table entry

	/* check if there is a new sensing hole or not:
	   If so, update virtual graph G, schedule table T, and edge queue Er by subdividing the graph G with hole nodes
         */
	if((fabs(left_hole_offset) <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC) && (fabs(right_hole_offset - weight) <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC))
	{ //The whole virtual edge (tail_node,head_node) is a new hole segment where tail_node and head_node become hole endpoints, that is, virtual enter/exit nodes
		/** NOTE: Dynamic memory problem for (*Gv[i]): refer to item [v2.7.6-2008-3-4] in version.txt 
		    The address values of ptr_tail_node, ptr_head_node, new_graph_node_1 and new_graph_node_2 can change according to the new dynamic memory allocation. So we need to use the structure object copied from &(Gv[i]) rather than the pointer to &(Gv[i]).
		*/

		/* get the pointers to tail_node and head_node in Gv which might be updated by GetUnusedGraphNode() */
		ptr_tail_node = &((*Gv)[tail_node_id-1]); //pointer to the graph node corresponding to tail_node_id
		ptr_head_node = &((*Gv)[head_node_id-1]); //pointer to the graph node corresponding to head_node_id

		/* get the graph node types of tail_node and head_node */
		tail_node_type = ptr_tail_node->type;
		head_node_type = ptr_head_node->type;

		/* get the graph node roles of tail_node and head_node */
		tail_node_role = ptr_tail_node->role;
		head_node_role = ptr_head_node->role;

		//@for debugging
		//if(tail_node_id == 17 || head_node_id == 17)
		//  printf("tail or head node is 17\n");
		////////////////

		/** perform hole segments merging with neighbor hole segment(s) */

	        /** check whether both tail_node and head_node are hole graph nodes or not */
	        if(tail_node_type == HOLE_GRAPH_NODE && head_node_type == HOLE_GRAPH_NODE) //if-1
		{ //@This new hole segment (called Middle hole segment) is located between two incident hole segments: (a) Left hole segment and (b) Right hole segment. => Merge these three hole segments into one hole segment.

		  /* find the other neighbor of tail node different from head node;
		     NOTE that when tail node is a hole graph node, there are only two neighbors. */
		  ptr_neighbor_graph_node_1 = ptr_tail_node->next;
		  do
		  {
		    if(strcmp(ptr_neighbor_graph_node_1->vertex, head_node) != 0)
		      break; //find the neighbor node different from the head node
		    else
		      ptr_neighbor_graph_node_1 = ptr_neighbor_graph_node_1->next;

		    if(ptr_neighbor_graph_node_1 == NULL)
		    {
		      printf("Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(): ptr_neighbor_graph_node_1 is NULL\n");
		      exit(1);
		    }
		  } while(1);

		  memcpy(&neighbor_graph_node_1, ptr_neighbor_graph_node_1, sizeof(neighbor_graph_node_1)); //copy the contents of ptr_neighbor_graph_node_1 to neighbor_graph_node_1

		  /** expand the left-sided hole segment (neighbor_graph_node_1,tail_node) to the right_hole_offset in the virtual edge having a new hole segment from left_hole_segment to right_hole_segment. */

		  /* get the pointer to the table node that is the incident hole segment for tail node */
		  neighbor_table_node_1 = LookupTable(*Gv, neighbor_graph_node_1.vertex, tail_node, &flip_flag_1); //get the pointer to the table entry of the incident edge (neighbor_graph_node_1,tail_node)

                  ///////////////////////////////////////////////////////////////////

		  /* find the other neighbor of head node different from tail node;
		     NOTE that when head node is a hole graph node, there are only two neighbors. */
		  ptr_neighbor_graph_node_2 = ptr_head_node->next;
		  do
		  {
		    if(strcmp(ptr_neighbor_graph_node_2->vertex, tail_node) != 0)
		      break; //find the neighbor node different from the tail node
		    else
		      ptr_neighbor_graph_node_2 = ptr_neighbor_graph_node_2->next;

		    if(ptr_neighbor_graph_node_2 == NULL)
		    {
		      printf("Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(): ptr_neighbor_graph_node_2 is NULL\n");
		      exit(1);
		    }
		  } while(1);

		  memcpy(&neighbor_graph_node_2, ptr_neighbor_graph_node_2, sizeof(neighbor_graph_node_2)); //copy the contents of ptr_neighbor_graph_node_2 to neighbor_graph_node_2

		  /* get the pointer to the table node that is the incident hole segment for head node */
		  neighbor_table_node_2 = LookupTable(*Gv, head_node, neighbor_graph_node_2.vertex, &flip_flag_2); //get the pointer to the table entry of the incident edge (head_node,neighbor_graph_node_2)

                  ///////////////////////////////////////////////////////////////////

		  /*** Procedure of hole segment mergence 
		    1. add the neighbor relationship between neigbhor_graph_node_1 and neighbor_graph_node 2 to Gv.
		    2. make a new table entry table_entry for the merged edge (neighbor_graph_node_1,neighbor_graph_node_2).
		    3. register all of the sensors in the three hole segments into table_entry's sensor_list.
		    4. add the new subedge of (neighbor_graph_node_1,neighbor_graph_node_2) to after the subedge corresponding to the neighbor_table_2's eid in the subedge list of the physical edge entry in edge queue Er.
		    5. delete three subedges of (neighbor_graph_node_1,tail_node), (tail_node,head_node) and (head_node,neighbor_graph_node_2) from the subedge list.
		    6. delete two hole endpoints for the offsets of tail_node and head_node from hole_endpoint list in edge queue Er.		 
		    7. delete the neighbor relationship of (neighbor_graph_node_1,tail_node), (tail_node,head_node) and (head_node,neighbor_graph_node_2) from Gv.
		    8. delete the graph nodes of tail_node and head_node from virtual graph Gv.
		    9. delete the table entries for edges (neighbor_graph_node_1,tail_node), (tail_node,head_node) and (head_node,neighbor_graph_node_2) from T.
		    10. add the hole endpoints of tail_node and head_node (deleted from the hole_set H for hole labeling) to deleted_hole_set.
		  **/

		  ///////////////////////////////////////////////////////////////////

		  /** 1. add the neighbor relationship between neigbhor_graph_node_1 and neighbor_graph_node 2 to Gv. */
		  hole_segment_length = neighbor_table_node_1->weight + pTableNode->weight + neighbor_table_node_2->weight; //the merged hole segment length

		  AddNeighborRelationship(*Gv, neighbor_graph_node_1.vertex, neighbor_graph_node_1.type, neighbor_graph_node_1.role, neighbor_graph_node_2.vertex, neighbor_graph_node_2.type, neighbor_graph_node_2.role, hole_segment_length, density);

                  ///////////////////////////////////////////////////////////////////

		  /** 2. make a new table entry table_entry for the merged edge (neighbor_graph_node_1,neighbor_graph_node_2) */
		  memset(&table_node, 0, sizeof(table_node));
		  table_node.weight = hole_segment_length; //set the edge's weight to the merged hole segment length
		  table_node.density = pTableNode->density; //set the sensor density
		  table_node.type = SEGMENT_TYPE_HOLE_SEGMENT; //set segment type to hole segment

		  /* set the physical offset of tail node */
		  //@NOTE: actually, according to our rule of table node generation, the node relationsip is always the EDGE_DIRECTION_FORWARD such as neighbor_graph_node_1 -> tail_node -> head_node -> neighbor_graph_node_2.
		  if(neighbor_table_node_1->direction == EDGE_DIRECTION_FORWARD)
		    table_node.tail_node_offset_in_Gr = neighbor_table_node_1->tail_node_offset_in_Gr;
		  else
		    table_node.tail_node_offset_in_Gr = neighbor_table_node_1->tail_node_offset_in_Gr - neighbor_table_node_1->weight;

		  strcpy(table_node.tail_node, neighbor_graph_node_1.vertex);
		  strcpy(table_node.head_node, neighbor_graph_node_2.vertex);

		  /* determine the virtual edge's direction */
		  if(pTableNode->direction == EDGE_DIRECTION_FORWARD)
		    table_node.direction = EDGE_DIRECTION_FORWARD;
		  else if(pTableNode->direction == EDGE_DIRECTION_BACKWARD)
		    table_node.direction = EDGE_DIRECTION_BACKWARD;
		  else
		  {
		    printf("Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(): pTableNode->direction(%d) has a wrong value\n", pTableNode->direction);
		    exit(1);
		  }

		  ptr_table_node = Entable(T, &table_node); //enqueue the clone of node into T and increase T->size by 1 and get the pointer to the table node newly added to table T.

		  /* remember the pointer to the table node corresponding to the hole segment */
		  ptr_table_node_for_hole_segment = ptr_table_node;

		  /* register the pointer to the table node in virtual graph Gv */
		  RegisterTableNodeIntoVirtualGraph(*Gv, *Gv_size, ptr_table_node);

                  ///////////////////////////////////////////////////////////////////

		  /** 3. register all of the sensors in the three hole segments into table_entry's sensor_list. */

		  /* initialize the order information for sensors on the new expanded hole segment */
		  order_for_live_sensors = 0;
		  order_in_Gv = 0;

		  if(flip_flag_1 == FALSE) //if-1.1: tail_node is the head node at the table entry pointed by neighbor_table_node_1
		  {
		    if(flip_flag_2 == FALSE) //if-1.1.1: head_node is the tail node at the table entry pointed by neighbor_table_node_2
		    { //@Node Relationship: neighbor_graph_node_1 -> tail_node -> head_node -> neighbor_graph_node_2
		      //@NOTE: actually, according to our rule of table node generation, the node relationsip is always the EDGE_DIRECTION_FORWARD such as neighbor_graph_node_1 -> tail_node -> head_node -> neighbor_graph_node_2.
		      
		      /** add all of the sensor queue nodes of neighbor_table_node_1->sensor_list to ptr_table_node->sensor_list */
		      ptr_queue_node = neighbor_table_node_1->sensor_list.head.next;
		      while(ptr_queue_node != &(neighbor_table_node_1->sensor_list.head))
		      {
			/* initialize sensor_queue_node with ptr_queue_node */
			memcpy(&sensor_queue_node, ptr_queue_node, sizeof(sensor_queue_node));

			/* update the sensor's position in virtual graph, consisting of the eid and offset */
			sensor_queue_node.info.pos_in_Gv.eid = ptr_table_node->eid; //eid is already correct since ptr_queue_node is a queue node of neighbor_table_node's sensor list
			sensor_queue_node.info.pos_in_Gv.offset = ptr_queue_node->info.pos_in_Gv.offset; 

			/* update the sensor order for live sensors and the sensor_list's live_sensor_number */
			if(sensor_queue_node.info.state != SENSOR_DIE)
		        {
			  sensor_queue_node.order_for_live_sensors = order_for_live_sensors++;
			  ptr_table_node->sensor_list.live_sensor_number++;
		        }
			else
			  sensor_queue_node.order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

			/* set the order of sensor on the edge in the virtual graph Gv */
			sensor_queue_node.order_in_Gv = order_in_Gv++;

			pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(ptr_table_node->sensor_list), (queue_node_t *)&sensor_queue_node);

			/* update the pointer of sensor list using ptr_queue_node->pSensorListEntry */
			*(pQueueNode->pSensorListEntry) = pQueueNode;

			ptr_queue_node = ptr_queue_node->next; //go to the next queue node
		      }

		      /** add the appropriate sensor queue nodes of pTableNode->sensor_list to ptr_table_node->sensor_list */
		      hole_segment_length = neighbor_table_node_1->weight; //length of the hole segment merged so far
		      ptr_queue_node = pTableNode->sensor_list.head.next;
		      while(ptr_queue_node != &(pTableNode->sensor_list.head))
		      {
			/* initialize sensor_queue_node with ptr_queue_node */
			memcpy(&sensor_queue_node, ptr_queue_node, sizeof(sensor_queue_node));

			/* update the sensor's position in virtual graph, consisting of the eid and offset */
			sensor_queue_node.info.pos_in_Gv.eid = ptr_table_node->eid;
			sensor_queue_node.info.pos_in_Gv.offset = hole_segment_length + ptr_queue_node->info.pos_in_Gv.offset; 

			/* update the sensor order for live sensors and the sensor_list's live_sensor_number */
			if(sensor_queue_node.info.state != SENSOR_DIE)
		        {
			  sensor_queue_node.order_for_live_sensors = order_for_live_sensors++;
			 ptr_table_node->sensor_list.live_sensor_number++;
		        }
			else
			  sensor_queue_node.order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

			/* set the order of sensor on the edge in the virtual graph Gv */
			sensor_queue_node.order_in_Gv = order_in_Gv++;

			pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(ptr_table_node->sensor_list), (queue_node_t *)&sensor_queue_node);

			/* update the pointer of sensor list using ptr_queue_node->pSensorListEntry in order that sensor table entry in sensor table S can point to the correct sensor queue node for sensor information */
			*(pQueueNode->pSensorListEntry) = pQueueNode;

			ptr_queue_node = ptr_queue_node->next; //go to the next queue node
		      }

		      /** add all of the sensor queue nodes of neighbor_table_node_2->sensor_list to ptr_table_node->sensor_list */
		      hole_segment_length = neighbor_table_node_1->weight + pTableNode->weight; //length of the hole segment merged so far
		      ptr_queue_node = neighbor_table_node_2->sensor_list.head.next;
		      while(ptr_queue_node != &(neighbor_table_node_2->sensor_list.head))
		      {
			/* initialize sensor_queue_node with ptr_queue_node */
			memcpy(&sensor_queue_node, ptr_queue_node, sizeof(sensor_queue_node));

			/* update the sensor's position in virtual graph, consisting of the eid and offset */
			sensor_queue_node.info.pos_in_Gv.eid = ptr_table_node->eid; //eid is already correct since ptr_queue_node is a queue node of neighbor_table_node's sensor list
			sensor_queue_node.info.pos_in_Gv.offset = hole_segment_length + ptr_queue_node->info.pos_in_Gv.offset; 

			/* update the sensor order for live sensors and the sensor_list's live_sensor_number */
			if(sensor_queue_node.info.state != SENSOR_DIE)
		        {
			  sensor_queue_node.order_for_live_sensors = order_for_live_sensors++;
			  ptr_table_node->sensor_list.live_sensor_number++;
		        }
			else
			  sensor_queue_node.order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

			/* set the order of sensor on the edge in the virtual graph Gv */
			sensor_queue_node.order_in_Gv = order_in_Gv++;

			pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(ptr_table_node->sensor_list), (queue_node_t *)&sensor_queue_node);

			/* update the pointer of sensor list using ptr_queue_node->pSensorListEntry */
			*(pQueueNode->pSensorListEntry) = pQueueNode;

			ptr_queue_node = ptr_queue_node->next; //go to the next queue node
		      } 
		    } //end of if-1.1.1
		    else //else-1.1.1
		    { //@Node Relationship: neighbor_graph_node_1 -> tail_node -> head_node <- neighbor_graph_node_2 

		      /** add all of the sensor queue nodes of neighbor_table_node_1->sensor_list to ptr_table_node->sensor_list */
		      ptr_queue_node = neighbor_table_node_1->sensor_list.head.next;
		      while(ptr_queue_node != &(neighbor_table_node_1->sensor_list.head))
		      {
			/* initialize sensor_queue_node with ptr_queue_node */
			memcpy(&sensor_queue_node, ptr_queue_node, sizeof(sensor_queue_node));

			/* update the sensor's position in virtual graph, consisting of the eid and offset */
			sensor_queue_node.info.pos_in_Gv.eid = ptr_table_node->eid; //eid is already correct since ptr_queue_node is a queue node of neighbor_table_node's sensor list
			sensor_queue_node.info.pos_in_Gv.offset = ptr_queue_node->info.pos_in_Gv.offset; 

			/* update the sensor order for live sensors and the sensor_list's live_sensor_number */
			if(sensor_queue_node.info.state != SENSOR_DIE)
		        {
			  sensor_queue_node.order_for_live_sensors = order_for_live_sensors++;
			  ptr_table_node->sensor_list.live_sensor_number++;
		        }
			else
			  sensor_queue_node.order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

			/* set the order of sensor on the edge in the virtual graph Gv */
			sensor_queue_node.order_in_Gv = order_in_Gv++;

			pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(ptr_table_node->sensor_list), (queue_node_t *)&sensor_queue_node);

			/* update the pointer of sensor list using ptr_queue_node->pSensorListEntry */
			*(pQueueNode->pSensorListEntry) = pQueueNode;

			ptr_queue_node = ptr_queue_node->next; //go to the next queue node
		      }

		      /** add the appropriate sensor queue nodes of pTableNode->sensor_list to ptr_table_node->sensor_list */
		      hole_segment_length = neighbor_table_node_1->weight; //length of the hole segment merged so far
		      ptr_queue_node = pTableNode->sensor_list.head.next;
		      while(ptr_queue_node != &(pTableNode->sensor_list.head))
		      {
			/* initialize sensor_queue_node with ptr_queue_node */
			memcpy(&sensor_queue_node, ptr_queue_node, sizeof(sensor_queue_node));

			/* update the sensor's position in virtual graph, consisting of the eid and offset */
			sensor_queue_node.info.pos_in_Gv.eid = ptr_table_node->eid;
			sensor_queue_node.info.pos_in_Gv.offset = hole_segment_length + ptr_queue_node->info.pos_in_Gv.offset; 

			/* update the sensor order for live sensors and the sensor_list's live_sensor_number */
			if(sensor_queue_node.info.state != SENSOR_DIE)
		        {
			  sensor_queue_node.order_for_live_sensors = order_for_live_sensors++;
			 ptr_table_node->sensor_list.live_sensor_number++;
		        }
			else
			  sensor_queue_node.order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

			/* set the order of sensor on the edge in the virtual graph Gv */
			sensor_queue_node.order_in_Gv = order_in_Gv++;

			pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(ptr_table_node->sensor_list), (queue_node_t *)&sensor_queue_node);

			/* update the pointer of sensor list using ptr_queue_node->pSensorListEntry in order that sensor table entry in sensor table S can point to the correct sensor queue node for sensor information */
			*(pQueueNode->pSensorListEntry) = pQueueNode;

			ptr_queue_node = ptr_queue_node->next; //go to the next queue node
		      }

		      /** add all of the sensor queue nodes of neighbor_table_node_2->sensor_list to ptr_table_node->sensor_list */
		      hole_segment_length = neighbor_table_node_1->weight + pTableNode->weight; //length of the hole segment merged so far
		      ptr_queue_node = neighbor_table_node_2->sensor_list.head.prev;
		      while(ptr_queue_node != &(neighbor_table_node_2->sensor_list.head))
		      {
			/* initialize sensor_queue_node with ptr_queue_node */
			memcpy(&sensor_queue_node, ptr_queue_node, sizeof(sensor_queue_node));

			/* update the sensor's position in virtual graph, consisting of the eid and offset */
			sensor_queue_node.info.pos_in_Gv.eid = ptr_table_node->eid; //eid is already correct since ptr_queue_node is a queue node of neighbor_table_node's sensor list
			sensor_queue_node.info.pos_in_Gv.offset = hole_segment_length + (neighbor_table_node_2->weight - ptr_queue_node->info.pos_in_Gv.offset); 

			/* update the sensor order for live sensors and the sensor_list's live_sensor_number */
			if(sensor_queue_node.info.state != SENSOR_DIE)
		        {
			  sensor_queue_node.order_for_live_sensors = order_for_live_sensors++;
			  ptr_table_node->sensor_list.live_sensor_number++;
		        }
			else
			  sensor_queue_node.order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

			/* set the order of sensor on the edge in the virtual graph Gv */
			sensor_queue_node.order_in_Gv = order_in_Gv++;

			pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(ptr_table_node->sensor_list), (queue_node_t *)&sensor_queue_node);

			/* update the pointer of sensor list using ptr_queue_node->pSensorListEntry */
			*(pQueueNode->pSensorListEntry) = pQueueNode;

			ptr_queue_node = ptr_queue_node->prev; //go to the previous queue node
		      } 
		    } //end of else-1.1.1		    
		  } //end of if-1.1
		  else //else-1.2: tail_node is also the head node at the table entry pointed by neighbor_table_node
		  {

		    if(flip_flag_2 == FALSE) //if-1.2.1: head_node is the tail node at the table entry pointed by neighbor_table_node_2
		    { //@Node Relationship: neighbor_graph_node_1 <- tail_node -> head_node -> neighbor_graph_node_2 

		    } //end of if-1.2.1
		    else //else-1.2.2
		    { //@Node Relationship: neighbor_graph_node_1 <- tail_node -> head_node <- neighbor_graph_node_2 

		    } //end of else-1.2.2
		  } //end of else-1.2

		  ///////////////////////////////////////////////////////////////////////////////

		  /** 4. add the new subedge of (neighbor_graph_node_1,neighbor_graph_node_2) to after the subedge corresponding to the neighbor_table_2's eid in the subedge list of the physical edge entry in edge queue Er. */
		  /* set up a new subedge node and enqueue it into the subedge queue */
		  memset(&subedge_node, 0, sizeof(subedge_node));
		  subedge_node.eid = ptr_table_node->eid;
		  strcpy(subedge_node.tail_node, ptr_table_node->tail_node);
		  strcpy(subedge_node.head_node, ptr_table_node->head_node);
		  subedge_node.weight = ptr_table_node->weight;
		  subedge_node.schedule_table_entry = ptr_table_node; //register the pointer to the schedule table entry into schedule_table_entry
		  subedge_node.edge_queue_entry = pEdgeNode; //register the pointer to the edge entry into edge_queue_entry

		  InsertSubedgeAfterEID_With_TableEntryUpdate(&(pEdgeNode->subedge_list), neighbor_table_node_2->eid, &subedge_node);  

		  ///////////////////////////////////////////////////////////////////////////////

		  /** 5. delete three subedges of (neighbor_graph_node_1,tail_node), (tail_node,head_node) and (head_node,neighbor_graph_node_2) from the subedge list. */
		  /* delete the subedge for neighbor_table_node_1->eid from subedge list */
		  DeleteSubedgeWithEID(&(pEdgeNode->subedge_list), neighbor_table_node_1->eid);

		  /* delete the subedge for pTableNode->eid from subedge list */
		  DeleteSubedgeWithEID(&(pEdgeNode->subedge_list), pTableNode->eid);

		  /* delete the subedge for neighbor_table_node_2->eid from subedge list */
		  DeleteSubedgeWithEID(&(pEdgeNode->subedge_list), neighbor_table_node_2->eid);

		  ///////////////////////////////////////////////////////////////////////////////

		  /** 6. delete two hole endpoints for the offsets of tail_node and head_node from hole_endpoint list in edge queue Er. */		    
		  /* delete the hole endpoint for neighbor_table_node_1 from sensing_hole_endpoint_list */
		  DeleteHoleEndpointWithEID(&(pEdgeNode->sensing_hole_endpoint_list), neighbor_table_node_1->eid, tail_node);

		  /* delete the hole endpoint for neighbor_table_node_1 from sensing_hole_endpoint_list */
		  DeleteHoleEndpointWithEID(&(pEdgeNode->sensing_hole_endpoint_list), neighbor_table_node_2->eid, head_node);

		  /** 7. update the eids of the hole endpoints for neighbor_graph_node_1 and neighbor_graph_node_2 with the new table entry's eid (i.e., ptr_table_node->eid) in sensing_hole_endpoint_list */
		  /* update the eid of the hole endpoint for neighbor_table_node_1 in sensing_hole_endpoint_list */
		  UpdateHoleEndpointEID(&(pEdgeNode->sensing_hole_endpoint_list), neighbor_table_node_1->eid, neighbor_graph_node_1.vertex, ptr_table_node->eid);

		  /* update the eid of the hole endpoint for neighbor_table_node_2 in sensing_hole_endpoint_list */
		  UpdateHoleEndpointEID(&(pEdgeNode->sensing_hole_endpoint_list), neighbor_table_node_2->eid, neighbor_graph_node_2.vertex, ptr_table_node->eid); 

		  ///////////////////////////////////////////////////////////////////////////////

		  /** 7. delete the neighbor relationship of (neighbor_graph_node_1,tail_node), (tail_node,head_node) and (head_node,neighbor_graph_node_2) from Gv. */
		  //delete the neighbor relationship between neighbor_graph_node_1 and tail_node
		  DeleteNeighborRelationship(*Gv, neighbor_graph_node_1.vertex, tail_node);

		  //delete the neighbor relationship between tail_node and head_node
		  DeleteNeighborRelationship(*Gv, tail_node, head_node);

		  //delete the neighbor relationship between head_node and neighbor_graph_node_2
		  DeleteNeighborRelationship(*Gv, head_node, neighbor_graph_node_2.vertex);

		  ///////////////////////////////////////////////////////////////////////////////

		  /** 8. delete the graph nodes of tail_node and head_node from virtual graph Gv. */
		  /* delete graph node tail_node from graph G for future reuse for another vertex. */
		  DeleteGraphNode(*Gv, *Gv_size, tail_node);
		  
		  /* delete graph node head_node from graph G for future reuse for another vertex. */
		  DeleteGraphNode(*Gv, *Gv_size, head_node);

		  ///////////////////////////////////////////////////////////////////////////////

		  /** 9. delete the table entries for edges (neighbor_graph_node_1,tail_node), (tail_node,head_node) and (head_node,neighbor_graph_node_2) from T. */
		  //delete the entry for edge (neighbor_graph_node_1,tail_node).
		  DeleteTableEntry(T, neighbor_table_node_1);

		  //delete the entry for edge (tail_node,head_node).
		  DeleteTableEntry(T, pTableNode);

		  //delete the entry for edge (head_node,neighbor_graph_node_2).
		  DeleteTableEntry(T, neighbor_table_node_2);

		  ///////////////////////////////////////////////////////////////////////////////

		  /** 10. add the hole endpoints of tail_node and head_node (deleted from the hole_set H for hole labeling) to deleted_hole_set. */
		  /* add traffic table entry corresponding to tail_node to the traffic table deleted_hole_set */
		    AddTrafficTableEntry(deleted_hole_set, tail_node);

		  /* add traffic table entry corresponding to head_node to the traffic table deleted_hole_set */
		    AddTrafficTableEntry(deleted_hole_set, head_node);

		  ///////////////////////////////////////////////////////////////////////////////


		} //end of if-1
	        /*****************************************************************************************/
		else if(head_node_type == HOLE_GRAPH_NODE) //else-if-2
		{ //@This new hole segment has a hole segment to the right of it (called Right hole segment). => Merge the new hole segment and the Right hole segment into one hole segment.

		} //end of else-if-2
	        /*****************************************************************************************/
	        else if(tail_node_type == HOLE_GRAPH_NODE) //else-if-3    
		{ //@This new hole segment has a hole segment to the left of it (called Left hole segment). => Merge the new hole segment and the Left hole segment into one hole segment.

		} //end of else-if-3
	        /*****************************************************************************************/
	        else //else-1
		{ //The whole physical edge becomes a hole segment. 

		} //end of else-1
	        /*****************************************************************************************/

/* 		/\* add tail_node and head_node to the list of entrance_or_exit? YES *\/ */
/* 		/\** add traffic table entry corresponding to tail_node to the traffic table added_hole_set *\/ */
/* 		  if(tail_node_role != ROLE_ENTRANCE_POINT && tail_node_role != ROLE_PROTECTION_POINT) */
/* 		    AddTrafficTableEntry(added_hole_set, tail_node); */

/* 		  /\** add traffic table entry corresponding to head_node to the traffic table added_hole_set *\/ */
/* 		  if(head_node_role != ROLE_ENTRANCE_POINT && head_node_role != ROLE_PROTECTION_POINT) */
/* 		    AddTrafficTableEntry(added_hole_set, head_node); */

		return FALSE;
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	else if((fabs(left_hole_offset) <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC) && (right_hole_offset > 0 && right_hole_offset < weight))
	{//@HANDLING OF LEFT-SIDED HOLE SEGMENT IN SENSOR SEGMENT: tail_node might become another hole endpoint with a new graph node in the case where hole segments are not merged.
		/** NOTE: Dynamic memory problem for (*Gv[i]): refer to item [v2.7.6-2008-3-4] in version.txt 
		    The address values of ptr_tail_node, ptr_head_node, new_graph_node_1 and new_graph_node_2 can change according to the new dynamic memory allocation. So we need to use the structure object copied from &(Gv[i]) rather than the pointer to &(Gv[i]).
		*/

		/* get the pointers to tail_node and head_node in Gv which might be updated by GetUnusedGraphNode() */
		ptr_tail_node = &((*Gv)[tail_node_id-1]); //pointer to the graph node corresponding to tail_node_id
		ptr_head_node = &((*Gv)[head_node_id-1]); //pointer to the graph node corresponding to head_node_id

		/* get the graph node types of tail_node and head_node */
		tail_node_type = ptr_tail_node->type;
		head_node_type = ptr_head_node->type;

		/* get the graph node roles of tail_node and head_node */
		tail_node_role = ptr_tail_node->role;
		head_node_role = ptr_head_node->role;

	        /** check whether tail_node is hole graph node or not */
	        if(tail_node_type == HOLE_GRAPH_NODE) //if-2
		{
		  /* find the other neighbor of tail node different from head node;
		     NOTE that when tail node is a hole graph node, there are only two neighbors. */
		  ptr_neighbor_graph_node = ptr_tail_node->next;
		  do
		  {
		    if(strcmp(ptr_neighbor_graph_node->vertex, head_node) != 0)
		      break; //find the neighbor node different from the head node
		    else
		      ptr_neighbor_graph_node = ptr_neighbor_graph_node->next;

		    if(ptr_neighbor_graph_node == NULL)
		    {
		      printf("Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(): ptr_neighbor_graph_node is NULL\n");
		      exit(1);
		    }
		  } while(1);

		  memcpy(&neighbor_graph_node, ptr_neighbor_graph_node, sizeof(neighbor_graph_node)); //copy the contents of ptr_neighbor_graph_node to neighbor_graph_node


		  //@for debugging
		  //if(strcmp(neighbor_graph_node.vertex, "23") == 0 || strcmp(neighbor_graph_node.vertex, "24") == 0)
		  //printf("check the neighbor graph node %s\n", neighbor_graph_node.vertex);
		  ////////////////


		  /** expand the left-sided hole segment (neighbor_graph_node,tail_node) to the right_hole_offset in the virtual edge having a new hole segment from left_hole_segment to right_hole_segment. */

		  /* get the pointer to the table node that is the incident hole segment for tail node */
		  neighbor_table_node = LookupTable(*Gv, neighbor_graph_node.vertex, tail_node, &flip_flag); //get the pointer to the table entry of the incident edge (neighbor_graph_node,tail_node)

		  /* make the sensor list for the expanded hole segment with the sensor list of the neighbor table node and the sensor list of the table node located from left_hole_offset to right_hole_offset */
		  InitQueue((queue_t*) &new_sensor_list, QTYPE_SENSOR); //initialize the sensor list queue for the sensors located on the merged virtual edge

		  /* initialize the order information for sensors on the new expanded hole segment */
		  order_for_live_sensors = 0;
		  order_in_Gv = 0;

		  if(flip_flag == FALSE) //if-2.1: tail_node is the head node at the table entry pointed by neighbor_table_node
		  { //@Node Relationship: neighbor_graph_node -> tail_node -> head_node 

		    /** add all of the sensor queue nodes of neighbor_table_node->sensor_list to new_sensor_list */
		    ptr_queue_node = neighbor_table_node->sensor_list.head.next;
		    while(ptr_queue_node != NULL && ptr_queue_node != &(neighbor_table_node->sensor_list.head))
		    {
		      /* initialize sensor_queue_node with ptr_queue_node */
		      memcpy(&sensor_queue_node, ptr_queue_node, sizeof(sensor_queue_node));

		      /* update the sensor's position in virtual graph, consisting of the eid and offset */
		      sensor_queue_node.info.pos_in_Gv.eid = neighbor_table_node->eid; //eid is already correct since ptr_queue_node is a queue node of neighbor_table_node's sensor list
		      sensor_queue_node.info.pos_in_Gv.offset = ptr_queue_node->info.pos_in_Gv.offset; 

		      /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		      if(sensor_queue_node.info.state != SENSOR_DIE)
		      {
			sensor_queue_node.order_for_live_sensors = order_for_live_sensors++;
			new_sensor_list.live_sensor_number++;
		      }
		      else
			sensor_queue_node.order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		      /* set the order of sensor on the edge in the virtual graph Gv */
		      sensor_queue_node.order_in_Gv = order_in_Gv++;

		      pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(new_sensor_list), (queue_node_t *)&sensor_queue_node);

		      /* update the pointer of sensor list using ptr_queue_node->pSensorListEntry */
		      *(pQueueNode->pSensorListEntry) = pQueueNode;

		      ptr_queue_node = ptr_queue_node->next; //go to the next queue node
		    }

		    /** add the appropriate sensor queue nodes of pTableNode->sensor_list to new_sensor_list */
		    ptr_queue_node = GetSensorQueueNodeJustAfterVirtualOffset(&(pTableNode->sensor_list), left_hole_offset);
		    while(ptr_queue_node != NULL && ptr_queue_node->info.pos_in_Gv.offset <= right_hole_offset && ptr_queue_node != &(pTableNode->sensor_list.head))
		    {
		      /* initialize sensor_queue_node with ptr_queue_node */
		      memcpy(&sensor_queue_node, ptr_queue_node, sizeof(sensor_queue_node));

		      /* update the sensor's position in virtual graph, consisting of the eid and offset */
		      sensor_queue_node.info.pos_in_Gv.eid = neighbor_table_node->eid;
		      sensor_queue_node.info.pos_in_Gv.offset = neighbor_table_node->weight + ptr_queue_node->info.pos_in_Gv.offset; 

		      /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		      if(sensor_queue_node.info.state != SENSOR_DIE)
		      {
			sensor_queue_node.order_for_live_sensors = order_for_live_sensors++;
			new_sensor_list.live_sensor_number++;
		      }
		      else
			sensor_queue_node.order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		      /* set the order of sensor on the edge in the virtual graph Gv */
		      sensor_queue_node.order_in_Gv = order_in_Gv++;

		      pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(new_sensor_list), (queue_node_t *)&sensor_queue_node);

		      /* update the pointer of sensor list using ptr_queue_node->pSensorListEntry in order that sensor table entry in sensor table S can point to the correct sensor queue node for sensor information */
		      *(pQueueNode->pSensorListEntry) = pQueueNode;

		      ptr_queue_node = ptr_queue_node->next; //go to the next queue node
		    }

		    /** replace neighbor_table_node's sensor_list with new_sensor_list and reset new_sensor_list since it doesn't have any queue node any more */
		    ReplaceSensorQueue(&(neighbor_table_node->sensor_list), &new_sensor_list);  

		    /** expand the incident hole segment pointed by ptr_neighbor_table_node */
		    segment_length_1 = right_hole_offset - left_hole_offset;
		    segment_length_2 = neighbor_graph_node.weight;
		    hole_segment_length = segment_length_1 + segment_length_2; 
		    //make the length of a new hole segment consisting of neighbor_graph_node and tail_node. 

		    neighbor_table_node->weight = hole_segment_length; //set the edge weight to the new hole segment length

		    /* shrink the weight of the virtual edge (tail_node,head_node) in table entry */
		    pTableNode->weight = weight - right_hole_offset;

		    /* update the tail_node_offset of the table entry pointed by pTableNode */
		    if(neighbor_table_node->direction == EDGE_DIRECTION_FORWARD)
		      pTableNode->tail_node_offset_in_Gr = neighbor_table_node->tail_node_offset_in_Gr + neighbor_table_node->weight;
		    else
		      pTableNode->tail_node_offset_in_Gr = neighbor_table_node->tail_node_offset_in_Gr - neighbor_table_node->weight;

		    /** update the subedges to have the exact virtual edge lengths */
		    /* expand the weight of the subedge of the hole segment (neighbor_graph_node,tail_node) */
		    neighbor_table_node->subedge->weight = neighbor_table_node->weight;

		    /* shrink the weight of the subedge of the virtual edge (tail_node,head_node) */
		    pTableNode->subedge->weight = pTableNode->weight;

		    /** update the weight of the edge (neighbor_graph_node,tail_node) and that of the edge (tail_node,head_node) */
		    //update the weight of the edge (neighbor_graph_node,tail_node)
		    UpdateEdgeWeight(*Gv, neighbor_graph_node.vertex, tail_node, neighbor_table_node->weight);

		    //update the weight of the edge (tail_node,head_node)
		    UpdateEdgeWeight(*Gv, tail_node, head_node, pTableNode->weight);

                    ///////////////////////////////////////////////////
		    /** update the sensor list in the table entry pointed by pTableNode */
                    ///////////////////////////////////////////////////

		    /* delete sensor nodes just before right_hole_offset from the sensor_list */
		    DeleteSensorQueueNodesJustBeforeVirtualOffset(&(pTableNode->sensor_list), right_hole_offset);

		    /* initialize the order information for sensors on the shrunk sensor segment */
		    order_for_live_sensors = 0;
		    order_in_Gv = 0;

		    /* update info.pos_in_Gv.offset, order_in_Gv, order_for_live_sensors and sensor_list.live_sensor_number */
		    pTableNode->sensor_list.live_sensor_number = 0; //reset live_sensor_number
		    ptr_queue_node = GetSensorQueueNodeJustAfterVirtualOffset(&(pTableNode->sensor_list), right_hole_offset);	    
		    while(ptr_queue_node != NULL && ptr_queue_node != &(pTableNode->sensor_list.head))
		    {
		      /* update the sensor's virtual offset */
		      ptr_queue_node->info.pos_in_Gv.offset = ptr_queue_node->info.pos_in_Gv.offset - right_hole_offset; 

		      /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		      if(ptr_queue_node->info.state != SENSOR_DIE)
		      {
			ptr_queue_node->order_for_live_sensors = order_for_live_sensors++;
			pTableNode->sensor_list.live_sensor_number++;
		      }
		      else
			ptr_queue_node->order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		      /* set the order of sensor on the edge in the virtual graph Gv */
		      ptr_queue_node->order_in_Gv = order_in_Gv++;

		      ptr_queue_node = ptr_queue_node->next; //go to the next queue node
		    }
		  } //end of if-2.1
		  else //else-2.1: tail_node is the tail node at the table entry pointed by neighbor_table_node
		  { //@Node Relationship: neighbor_graph_node <- tail_node -> head_node 

		    /** add the appropriate sensor queue nodes of pTableNode->sensor_list to new_sensor_list */
		    ptr_queue_node = GetSensorQueueNodeJustBeforeVirtualOffset(&(pTableNode->sensor_list), right_hole_offset);
		    while(ptr_queue_node != NULL && ptr_queue_node != &(pTableNode->sensor_list.head))
		    {
		      /* initialize sensor_queue_node with ptr_queue_node */
		      memcpy(&sensor_queue_node, ptr_queue_node, sizeof(sensor_queue_node));

		      /* update the sensor's position in virtual graph, consisting of the eid and offset */
		      sensor_queue_node.info.pos_in_Gv.eid = neighbor_table_node->eid;
		      sensor_queue_node.info.pos_in_Gv.offset = right_hole_offset - ptr_queue_node->info.pos_in_Gv.offset; 

		      /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		      if(sensor_queue_node.info.state != SENSOR_DIE)
		      {
			sensor_queue_node.order_for_live_sensors = order_for_live_sensors++;
			new_sensor_list.live_sensor_number++;
		      }
		      else
			sensor_queue_node.order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		      /* set the order of sensor on the edge in the virtual graph Gv */
		      sensor_queue_node.order_in_Gv = order_in_Gv++;

		      pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(new_sensor_list), (queue_node_t *)&sensor_queue_node);

		      /* update the pointer of sensor list using ptr_queue_node->pSensorListEntry in order that sensor table entry in sensor table S can point to the correct sensor queue node for sensor information */
		      *(pQueueNode->pSensorListEntry) = pQueueNode;

		      ptr_queue_node = ptr_queue_node->prev; //go to the previous queue node
		    }

		    /** add all of the sensor queue nodes of neighbor_table_node->sensor_list to new_sensor_list */
		    ptr_queue_node = neighbor_table_node->sensor_list.head.next;
		    while(ptr_queue_node != NULL && ptr_queue_node != &(neighbor_table_node->sensor_list.head))
		    {
		      /* initialize sensor_queue_node with ptr_queue_node */
		      memcpy(&sensor_queue_node, ptr_queue_node, sizeof(sensor_queue_node));

		      /* update the sensor's position in virtual graph, consisting of the eid and offset */
		      sensor_queue_node.info.pos_in_Gv.eid = neighbor_table_node->eid; //eid is already correct since ptr_queue_node is a queue node of neighbor_table_node's sensor list
		      sensor_queue_node.info.pos_in_Gv.offset = right_hole_offset + ptr_queue_node->info.pos_in_Gv.offset; 

		      /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		      if(sensor_queue_node.info.state != SENSOR_DIE)
		      {
			sensor_queue_node.order_for_live_sensors = order_for_live_sensors++;
			new_sensor_list.live_sensor_number++;
		      }
		      else
			sensor_queue_node.order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		      /* set the order of sensor on the edge in the virtual graph Gv */
		      sensor_queue_node.order_in_Gv = order_in_Gv++;

		      pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(new_sensor_list), (queue_node_t *)&sensor_queue_node);

		      /* update the pointer of sensor list using ptr_queue_node->pSensorListEntry */
		      *(pQueueNode->pSensorListEntry) = pQueueNode;

		      ptr_queue_node = ptr_queue_node->next; //go to the next queue node
		    }

		    /** replace neighbor_table_node's sensor_list with new_sensor_list and reset new_sensor_list since it doesn't have any queue node any more */
		    ReplaceSensorQueue(&(neighbor_table_node->sensor_list), &new_sensor_list);  

		    /** expand the incident hole segment pointed by ptr_neighbor_table_node */
		    segment_length_1 = right_hole_offset - left_hole_offset;
		    segment_length_2 = neighbor_graph_node.weight;
		    hole_segment_length = segment_length_1 + segment_length_2; 
		    //make the length of a new hole segment consisting of neighbor_graph_node and tail_node. 

		    neighbor_table_node->weight = hole_segment_length; //set the edge weight to the new hole segment length

		    /* shrink the weight of the virtual edge (tail_node,head_node) in table entry */
		    pTableNode->weight = weight - right_hole_offset;

		    /* update the tail_node_offset of the table entry pointed by pTableNode */
		    if(neighbor_table_node->direction == EDGE_DIRECTION_FORWARD)
		      pTableNode->tail_node_offset_in_Gr = neighbor_table_node->tail_node_offset_in_Gr - right_hole_offset;
		    else
		      pTableNode->tail_node_offset_in_Gr = neighbor_table_node->tail_node_offset_in_Gr + right_hole_offset;

		    /** update the subedges to have the exact virtual edge lengths */
		    /* expand the weight of the subedge of the hole segment (neighbor_graph_node,tail_node) */
		    neighbor_table_node->subedge->weight = neighbor_table_node->weight;

		    /* shrink the weight of the subedge of the virtual edge (tail_node,head_node) */
		    pTableNode->subedge->weight = pTableNode->weight;

		    /** update the weight of the edge (neighbor_graph_node,tail_node) and that of the edge (tail_node,head_node) */
		    //update the weight of the edge (neighbor_graph_node,tail_node)
		    UpdateEdgeWeight(*Gv, neighbor_graph_node.vertex, tail_node, neighbor_table_node->weight);

		    //update the weight of the edge (tail_node,head_node)
		    UpdateEdgeWeight(*Gv, tail_node, head_node, pTableNode->weight);

                    ///////////////////////////////////////////////////
		    /** update the sensor list in the table entry pointed by pTableNode */
                    ///////////////////////////////////////////////////

		    /* delete sensor nodes just before right_hole_offset from the sensor_list */
		    DeleteSensorQueueNodesJustBeforeVirtualOffset(&(pTableNode->sensor_list), right_hole_offset);

		    /* initialize the order information for sensors on the shrunk sensor segment */
		    order_for_live_sensors = 0;
		    order_in_Gv = 0;

		    /* update info.pos_in_Gv.offset, order_in_Gv, order_for_live_sensors and sensor_list.live_sensor_number */
		    pTableNode->sensor_list.live_sensor_number = 0; //reset live_sensor_number
		    ptr_queue_node = GetSensorQueueNodeJustAfterVirtualOffset(&(pTableNode->sensor_list), right_hole_offset);	    
		    while(ptr_queue_node != NULL && ptr_queue_node != &(pTableNode->sensor_list.head))
		    {
		      /* update the sensor's virtual offset */
		      ptr_queue_node->info.pos_in_Gv.offset = ptr_queue_node->info.pos_in_Gv.offset - right_hole_offset; 

		      /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		      if(ptr_queue_node->info.state != SENSOR_DIE)
		      {
			ptr_queue_node->order_for_live_sensors = order_for_live_sensors++;
			pTableNode->sensor_list.live_sensor_number++;
		      }
		      else
			ptr_queue_node->order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		      /* set the order of sensor on the edge in the virtual graph Gv */
		      ptr_queue_node->order_in_Gv = order_in_Gv++;

		      ptr_queue_node = ptr_queue_node->next; //go to the next queue node
		    }
		  } //end of else-2.1
		  
		  /** update the real offsets of two end-points in the expanded hole segment in sensing_hole_endpoint_list on the physical edge including the hole segment */
		  hole_number = GetHoleEndpointsWithHoleSegment(&(neighbor_table_node->subedge->edge_queue_entry->sensing_hole_endpoint_list), neighbor_table_node->eid, &left_hole_endpoint, &right_hole_endpoint);
		  if(hole_number != 2)
		  {
		    printf("Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(): hole_number(%d) must be two!\n", hole_number);
		    exit(1);
		  }
		  
		  virtual_offset_of_left_hole = 0;
		  virtual_offset_of_right_hole = neighbor_table_node->weight;
		  
		  physical_offset_of_left_hole = GetPhysicalOffsetOnSubedgeFromSubedgeList(&(neighbor_table_node->subedge->edge_queue_entry->subedge_list), neighbor_table_node->subedge, virtual_offset_of_left_hole);
		  physical_offset_of_right_hole = GetPhysicalOffsetOnSubedgeFromSubedgeList(&(neighbor_table_node->subedge->edge_queue_entry->subedge_list), neighbor_table_node->subedge, virtual_offset_of_right_hole);

		  left_hole_endpoint->offset = physical_offset_of_left_hole;
		  right_hole_endpoint->offset = physical_offset_of_right_hole;
		} //end of if-2
		///////////////////////////////////////////////////////////////////////////////
		else //@else-2: The case where ptr_tail_node->type == INTERSECTION_GRAPH_NODE | ENTRANCE_GRAPH_NODE | PROTECTION_GRAPH_NODE, so we cannot expand the previous hole segment 
		{
 		  /* get an unused graph node from the array including G for the hole for left_hole_offset */
		  ptr_new_graph_node = GetUnusedGraphNode(Gv, Gv_size);
		  memcpy(&new_graph_node, ptr_new_graph_node, sizeof(new_graph_node));

		  //add the neighbor relationship between tail_node and new_graph_node
		  AddNeighborRelationship(*Gv, tail_node, tail_node_type, tail_node_role, new_graph_node.vertex, new_graph_node.type, new_graph_node.role, right_hole_offset, density);

		  //add the neighbor relationship between new_graph_node and head_node 
		  AddNeighborRelationship(*Gv, head_node, head_node_type, head_node_role, new_graph_node.vertex, new_graph_node.type, new_graph_node.role, weight-right_hole_offset, density);
		  ////////////////////////////////

		  /** insert edge (tail_node, new_graph_node) into table T where new_graph_node corresponds to the hole located at the right_hole_offset */
		  memset(&table_node, 0, sizeof(table_node));
		  //table_node.eid = T->sequence_number + 1; //set the edge id; table_node's eid is set at the function of Entable()
		  table_node.weight = right_hole_offset; //set the edge's weight
		  table_node.density = pTableNode->density; //set the sensor density
		  table_node.type = SEGMENT_TYPE_HOLE_SEGMENT; //set segment type to hole segment
		  table_node.tail_node_offset_in_Gr = pTableNode->tail_node_offset_in_Gr; //set the physical offset of tail node in Gr
		  strcpy(table_node.tail_node, pTableNode->tail_node);
		  strcpy(table_node.head_node, new_graph_node.vertex);

		  /* determine the virtual edge's direction */
		  if(pTableNode->direction == EDGE_DIRECTION_FORWARD)
		    table_node.direction = EDGE_DIRECTION_FORWARD;
		  else if(pTableNode->direction == EDGE_DIRECTION_BACKWARD)
		    table_node.direction = EDGE_DIRECTION_BACKWARD;
		  else
		  {
		    printf("Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(): pTableNode->direction(%d) has a wrong value\n", pTableNode->direction);
		    exit(1);
		  }

		  ptr_table_node = Entable(T, &table_node); //enqueue the clone of node into T and increase T->size by 1 and get the pointer to the table node newly added to table T.

		  /* remember the pointer to the table node corresponding to the hole segment */
		  ptr_table_node_for_hole_segment = ptr_table_node;

		  /* register the pointer to the table node in virtual graph Gv */
		  RegisterTableNodeIntoVirtualGraph(*Gv, *Gv_size, ptr_table_node);

		  /* set up the sensor_list for edge (tail_node, new_graph_node) */
		  ptr_queue_node = pTableNode->sensor_list.head.next;
		  order_for_live_sensors = 0;
		  order_in_Gv = 0;
		  while(ptr_queue_node->info.pos_in_Gv.offset <= right_hole_offset && count < sensor_list_size)
		  {
		    /* update the sensor's position in virtual graph, consisting of the eid and offset */
		    ptr_queue_node->info.pos_in_Gv.eid = ptr_table_node->eid;

		    /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		    if(ptr_queue_node->info.state != SENSOR_DIE)
		    {
		      ptr_queue_node->order_for_live_sensors = order_for_live_sensors++;
		      ptr_table_node->sensor_list.live_sensor_number++;
		    }
		    else
		      ptr_queue_node->order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		    /* set the order of sensor on the edge in the virtual graph Gv */
		    ptr_queue_node->order_in_Gv = order_in_Gv++;

		    pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(ptr_table_node->sensor_list), (queue_node_t *)ptr_queue_node);

		    /* update the pointer of sensor list using ptr_queue_node->pSensorListEntry */
		    *(pQueueNode->pSensorListEntry) = pQueueNode;

		    ptr_queue_node = ptr_queue_node->next;
		    count++;
		  }

		  /* set up a new subedge node and enqueue it into the subedge queue */
		  memset(&subedge_node, 0, sizeof(subedge_node));
		  subedge_node.eid = ptr_table_node->eid;
		  strcpy(subedge_node.tail_node, ptr_table_node->tail_node);
		  strcpy(subedge_node.head_node, ptr_table_node->head_node);
		  subedge_node.weight = ptr_table_node->weight;
		  subedge_node.schedule_table_entry = ptr_table_node; //register the pointer to the schedule table entry into schedule_table_entry
		  subedge_node.edge_queue_entry = pEdgeNode; //register the pointer to the edge entry into edge_queue_entry
		  Enqueue((queue_t *)&subedge_queue, (queue_node_t *)&subedge_node);
		  ///////////////////////////////////////////

		  /** insert edge (new_graph_node, head_node) into table T */
		  memset(&table_node, 0, sizeof(table_node));
		  //table_node.eid = T->sequence_number + 1; //set the edge id
		  table_node.weight = weight - right_hole_offset; //set the edge's weight
		  table_node.density = pTableNode->density; //set the sensor density
		  table_node.type = SEGMENT_TYPE_SENSOR_SEGMENT; //set segment type to sensor segment
		  table_node.tail_node_offset_in_Gr = pTableNode->tail_node_offset_in_Gr + right_hole_offset; //set the physical offset of tail node in Gr
		  strcpy(table_node.tail_node, new_graph_node.vertex);
		  strcpy(table_node.head_node, pTableNode->head_node);

		  /* determine the virtual edge's direction */
		  if(pTableNode->direction == EDGE_DIRECTION_FORWARD)
		    table_node.direction = EDGE_DIRECTION_FORWARD;
		  else if(pTableNode->direction == EDGE_DIRECTION_BACKWARD)
		    table_node.direction = EDGE_DIRECTION_BACKWARD;
		  else
		  {
		    printf("Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(): pTableNode->direction(%d) has a wrong value\n", pTableNode->direction);
		    exit(1);
		  }

		  //Entable(T, &table_node); //enqueue the clone of node into T and increase T->size by 1.
		  //ptr_table_node = GetTableNodeByEID(T, table_node.eid);
		  //return the table node corresponding to eid; the eid starts from 1.
		  ptr_table_node = Entable(T, &table_node); //enqueue the clone of node into T and increase T->size by 1 and get the pointer to the table node newly added to table T.

		  /* register the pointer to the table node in virtual graph Gv */
		  RegisterTableNodeIntoVirtualGraph(*Gv, *Gv_size, ptr_table_node);

		  /* set up the sensor_list for edge (new_graph_node, head_node) */
		  order_for_live_sensors = 0;
		  order_in_Gv = 0;
		  while(ptr_queue_node->info.pos_in_Gv.offset <= weight && count < sensor_list_size)
		  {
		    /* update the sensor's position in virtual graph, consisting of the eid and offset */
		    ptr_queue_node->info.pos_in_Gv.eid = ptr_table_node->eid;
		    ptr_queue_node->info.pos_in_Gv.offset -= right_hole_offset; 

		    /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		    if(ptr_queue_node->info.state != SENSOR_DIE)
		    {
		      ptr_queue_node->order_for_live_sensors = order_for_live_sensors++;
		      ptr_table_node->sensor_list.live_sensor_number++;
		    }
		    else
		      ptr_queue_node->order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		    /* set the order of sensor on the edge in the virtual graph Gv */
		    ptr_queue_node->order_in_Gv = order_in_Gv++;

		    pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(ptr_table_node->sensor_list), (queue_node_t *)ptr_queue_node);

		    /* update the pointer of sensor list using ptr_queue_node->pSensorListEntry */
		    *(pQueueNode->pSensorListEntry) = pQueueNode;

		    ptr_queue_node = ptr_queue_node->next;
		    count++;
		  }

		  /* set up a new subedge node and enqueue it into the subedge queue */
		  memset(&subedge_node, 0, sizeof(subedge_node));
		  subedge_node.eid = ptr_table_node->eid;
		  strcpy(subedge_node.tail_node, ptr_table_node->tail_node);
		  strcpy(subedge_node.head_node, ptr_table_node->head_node);
		  subedge_node.weight = ptr_table_node->weight;
		  subedge_node.schedule_table_entry = ptr_table_node; //register the pointer to the schedule table entry into schedule_table_entry
		  subedge_node.edge_queue_entry = pEdgeNode; //register the pointer to the edge entry into edge_queue_entry
		  Enqueue((queue_t *)&subedge_queue, (queue_node_t *)&subedge_node);
		  ///////////////////////////////////////////

		  /** replace the subedge with the subdivision of the subedge that is the subedge queue and
		      update schedule table entries to let them point to the corresponding subedges */
		  ReplaceSubedgeWithSubdivision(&(pTableNode->subedge), pTableNode->subedge, &subedge_queue);

		  /** insert two endpoints of the new sensing hole segment into the sensing hole endpoint queue */
		  /* register the left hole end-point into pEdgeNode->sensing_hole_endpoint_list */
		  memset(&left_hole_endpoint_node, 0, sizeof(left_hole_endpoint_node));
		  strcpy(left_hole_endpoint_node.vertex, tail_node);
		  left_hole_endpoint_node.eid = ptr_table_node_for_hole_segment->eid; //set the hole endpoint's eid to the eid of the virtual edge containing the hole.

		  virtual_offset_of_left_hole = left_hole_offset; //offset of a sensing hole endpoint in virtual graph Gv; the offset is the offset of the left endpoint of the first subedge in the subdivision in the virtual graph Gv.
		  physical_offset_of_left_hole = GetPhysicalOffsetOnSubedgeFromSubedgeList(&(ptr_table_node_for_hole_segment->subedge->edge_queue_entry->subedge_list), ptr_table_node_for_hole_segment->subedge, virtual_offset_of_left_hole);

		  left_hole_endpoint_node.offset = physical_offset_of_left_hole;
		  left_hole_endpoint_node.edge_queue_entry = pEdgeNode; //register the pointer to the edge entry into edge_queue_entry
		  
		  Enqueue((queue_t *)&(pEdgeNode->sensing_hole_endpoint_list), (queue_node_t *)&left_hole_endpoint_node);

		  /* register the right hole end-point into pEdgeNode->sensing_hole_endpoint_list */
		  memset(&right_hole_endpoint_node, 0, sizeof(right_hole_endpoint_node));
		  strcpy(right_hole_endpoint_node.vertex, new_graph_node.vertex);
		  right_hole_endpoint_node.eid = ptr_table_node_for_hole_segment->eid; //set the hole endpoint's eid to the eid of the virtual edge containing the hole.

		  virtual_offset_of_right_hole = right_hole_offset; //offset of a sensing hole endpoint in virtual graph Gv; the offset is the offset of the right endpoint of the first subedge in the subdivision in the virtual graph Gv.
		  physical_offset_of_right_hole = GetPhysicalOffsetOnSubedgeFromSubedgeList(&(ptr_table_node_for_hole_segment->subedge->edge_queue_entry->subedge_list), ptr_table_node_for_hole_segment->subedge, virtual_offset_of_right_hole);

		  right_hole_endpoint_node.offset = physical_offset_of_right_hole;
		  right_hole_endpoint_node.edge_queue_entry = pEdgeNode; //register the pointer to the edge entry into edge_queue_entry
		  
		  Enqueue((queue_t *)&(pEdgeNode->sensing_hole_endpoint_list), (queue_node_t *)&right_hole_endpoint_node);

		  /** add traffic table entry corresponding to new_graph_node to the traffic table added_hole_set */
		  AddTrafficTableEntry(added_hole_set, new_graph_node.vertex);
		  
		  /** add traffic table entry corresponding to tail_node to the traffic table added_hole_set */
		  if(tail_node_role != ROLE_ENTRANCE_POINT && tail_node_role != ROLE_PROTECTION_POINT)
		    AddTrafficTableEntry(added_hole_set, tail_node);

		  /** delete the neighbor relationship between tail_node and head_node */
		  DeleteNeighborRelationship(*Gv, tail_node, head_node);

		  /** delete the entry for pTableEntry from table T */
		  DeleteTableEntry(T, pTableNode);
		} //end of else-2
	} //end of HANDLING OF LEFT-SIDED HOLE SEGMENT IN SENSOR SEGMENT
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	else if((left_hole_offset > 0 && left_hole_offset < weight) && (fabs(right_hole_offset - weight) <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC))
	{//@HANDLING OF RIGHT-SIDED HOLE SEGMENT IN SENSOR SEGMENT: head_node might become another hole endpoint with a new graph node in the case where hole segments are not merged.
		/** NOTE: Dynamic memory problem for (*Gv[i]): refer to item [v2.7.6-2008-3-4] in version.txt 
		    The address values of ptr_tail_node, ptr_head_node, new_graph_node_1 and new_graph_node_2 can change according to the new dynamic memory allocation. So we need to use the structure object copied from &(Gv[i]) rather than the pointer to &(Gv[i]).
		*/

		/* get the pointers to tail_node and head_node in Gv which might be updated by GetUnusedGraphNode() */
		ptr_tail_node = &((*Gv)[tail_node_id-1]); //pointer to the graph node corresponding to tail_node_id
		ptr_head_node = &((*Gv)[head_node_id-1]); //pointer to the graph node corresponding to head_node_id

		/* get the graph node types of tail_node and head_node */
		tail_node_type = ptr_tail_node->type;
		head_node_type = ptr_head_node->type;

		/* get the graph node roles of tail_node and head_node */
		tail_node_role = ptr_tail_node->role;
		head_node_role = ptr_head_node->role;

	        /** check whether head_node is hole graph node or not */
	        if(head_node_type == HOLE_GRAPH_NODE) //if-1
		{

		  /* find the other neighbor of head node different from tail node;
		     NOTE that when head node is a hole graph node, there are only two neighbors. */
		  ptr_neighbor_graph_node = ptr_head_node->next;
		  do
		  {
		    if(strcmp(ptr_neighbor_graph_node->vertex, tail_node) != 0)
		      break; //find the neighbor node different from the tail node
		    else
		      ptr_neighbor_graph_node = ptr_neighbor_graph_node->next;

		    if(ptr_neighbor_graph_node == NULL)
		    {
		      printf("Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(): ptr_neighbor_graph_node is NULL\n");
		      exit(1);
		    }
		  } while(1);

		  memcpy(&neighbor_graph_node, ptr_neighbor_graph_node, sizeof(neighbor_graph_node)); //copy the contents of ptr_neighbor_graph_node to neighbor_graph_node


		  //@for debugging
		  //if(strcmp(neighbor_graph_node.vertex, "23") == 0 || strcmp(neighbor_graph_node.vertex, "24") == 0)
		  //printf("check the neighbor graph node %s\n", neighbor_graph_node.vertex);
		  ////////////////


		  /** expand the left-sided hole segment (head_node,neighbor_graph_node) to the left_hole_offset in the virtual edge having a new hole segment from left_hole_segment to right_hole_segment. */

		  /* get the pointer to the table node that is the incident hole segment for head node */
		  neighbor_table_node = LookupTable(*Gv, head_node, neighbor_graph_node.vertex, &flip_flag); //get the pointer to the table entry of the incident edge (head_node,neighbor_graph_node)

		  /* make the sensor list for the expanded hole segment with the sensor list of the neighbor table node and the sensor list of the table node located from left_hole_offset to right_hole_offset */
		  InitQueue((queue_t*) &new_sensor_list, QTYPE_SENSOR); //initialize the sensor list queue for the sensors located on the merged virtual edge

		  /* initialize the order information for sensors on the new expanded hole segment */
		  order_for_live_sensors = 0;
		  order_in_Gv = 0;

		  if(flip_flag == FALSE) //if-1.1: head_node is the tail node at the table entry pointed by neighbor_table_node
		  { //@Node Relationship: tail_node -> head_node -> neighbor_graph_node

		    /** add the appropriate sensor queue nodes of pTableNode->sensor_list to new_sensor_list */
		    ptr_queue_node = GetSensorQueueNodeJustAfterVirtualOffset(&(pTableNode->sensor_list), left_hole_offset);
		    while(ptr_queue_node != NULL && ptr_queue_node != &(pTableNode->sensor_list.head))
		    {
		      /* initialize sensor_queue_node with ptr_queue_node */
		      memcpy(&sensor_queue_node, ptr_queue_node, sizeof(sensor_queue_node));

		      /* update the sensor's position in virtual graph, consisting of the eid and offset */
		      sensor_queue_node.info.pos_in_Gv.eid = neighbor_table_node->eid;
		      sensor_queue_node.info.pos_in_Gv.offset = ptr_queue_node->info.pos_in_Gv.offset - left_hole_offset; 

		      /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		      if(sensor_queue_node.info.state != SENSOR_DIE)
		      {
			sensor_queue_node.order_for_live_sensors = order_for_live_sensors++;
			new_sensor_list.live_sensor_number++;
		      }
		      else
			sensor_queue_node.order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		      /* set the order of sensor on the edge in the virtual graph Gv */
		      sensor_queue_node.order_in_Gv = order_in_Gv++;

		      pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(new_sensor_list), (queue_node_t *)&sensor_queue_node);

		      /* update the pointer of sensor list using ptr_queue_node->pSensorListEntry in order that sensor table entry in sensor table S can point to the correct sensor queue node for sensor information */
		      *(pQueueNode->pSensorListEntry) = pQueueNode;

		      ptr_queue_node = ptr_queue_node->next; //go to the next queue node
		    }

		    /** add all of the sensor queue nodes of neighbor_table_node->sensor_list to new_sensor_list */
		    ptr_queue_node = neighbor_table_node->sensor_list.head.next;
		    hole_segment_length = right_hole_offset - left_hole_offset; //hole_segment_length is the length of a new hole segment of a sensor's death
		    while(ptr_queue_node != NULL && ptr_queue_node != &(neighbor_table_node->sensor_list.head))
		    {
		      /* initialize sensor_queue_node with ptr_queue_node */
		      memcpy(&sensor_queue_node, ptr_queue_node, sizeof(sensor_queue_node));

		      /* update the sensor's position in virtual graph, consisting of the eid and offset */
		      sensor_queue_node.info.pos_in_Gv.eid = neighbor_table_node->eid; //eid is already correct since ptr_queue_node is a queue node of neighbor_table_node's sensor list
		      sensor_queue_node.info.pos_in_Gv.offset = hole_segment_length + ptr_queue_node->info.pos_in_Gv.offset; 

		      /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		      if(sensor_queue_node.info.state != SENSOR_DIE)
		      {
			sensor_queue_node.order_for_live_sensors = order_for_live_sensors++;
			new_sensor_list.live_sensor_number++;
		      }
		      else
			sensor_queue_node.order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		      /* set the order of sensor on the edge in the virtual graph Gv */
		      sensor_queue_node.order_in_Gv = order_in_Gv++;

		      pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(new_sensor_list), (queue_node_t *)&sensor_queue_node);

		      /* update the pointer of sensor list using ptr_queue_node->pSensorListEntry */
		      *(pQueueNode->pSensorListEntry) = pQueueNode;

		      ptr_queue_node = ptr_queue_node->next; //go to the next queue node
		    }

		    /** replace neighbor_table_node's sensor_list with new_sensor_list and reset new_sensor_list since it doesn't have any queue node any more */
		    ReplaceSensorQueue(&(neighbor_table_node->sensor_list), &new_sensor_list);  

		    /** expand the incident hole segment pointed by ptr_neighbor_table_node */
		    segment_length_1 = right_hole_offset - left_hole_offset;
		    segment_length_2 = neighbor_graph_node.weight;
		    hole_segment_length = segment_length_1 + segment_length_2; 
		    //make the length of the expanded hole segment consisting of head_node and neighbor_graph_node. 

		    neighbor_table_node->weight = hole_segment_length; //set the edge weight to the new hole segment length

		    /* shrink the weight of the virtual edge (tail_node,head_node) in table entry */
		    pTableNode->weight = left_hole_offset;

		    /* update the tail_node_offset of the table entry pointed by pTableNode */
		    if(neighbor_table_node->direction == EDGE_DIRECTION_FORWARD)
		      neighbor_table_node->tail_node_offset_in_Gr = neighbor_table_node->tail_node_offset_in_Gr - segment_length_1; //where segment_length_1 = right_hole_offset - left_hole_offset
		    else
		      neighbor_table_node->tail_node_offset_in_Gr = neighbor_table_node->tail_node_offset_in_Gr + segment_length_1;

		    /** update the subedges to have the exact virtual edge lengths */
		    /* expand the weight of the subedge of the hole segment (head_node,neighbor_graph_node) */
		    neighbor_table_node->subedge->weight = neighbor_table_node->weight;

		    /* shrink the weight of the subedge of the virtual edge (tail_node,head_node) */
		    pTableNode->subedge->weight = pTableNode->weight;

		    /** update the weight of the edge (head_node,neighbor_graph_node) and that of the edge (tail_node,head_node) */
		    //update the weight of the edge (head_node,neighbor_graph_node)
		    UpdateEdgeWeight(*Gv, head_node, neighbor_graph_node.vertex, neighbor_table_node->weight);

		    //update the weight of the edge (tail_node,head_node)
		    UpdateEdgeWeight(*Gv, tail_node, head_node, pTableNode->weight);

                    ///////////////////////////////////////////////////
		    /** update the sensor list in the table entry pointed by pTableNode */
                    ///////////////////////////////////////////////////

		    /* delete sensor nodes just after left_hole_offset from the sensor_list */
		    DeleteSensorQueueNodesJustAfterVirtualOffset(&(pTableNode->sensor_list), left_hole_offset);

		    /* initialize the order information for sensors on the shrunk sensor segment */
		    order_for_live_sensors = 0;
		    order_in_Gv = 0;

		    /* update info.pos_in_Gv.offset, order_in_Gv, order_for_live_sensors and sensor_list.live_sensor_number */
		    pTableNode->sensor_list.live_sensor_number = 0; //reset live_sensor_number
		    //ptr_queue_node = GetSensorQueueNodeJustAfterVirtualOffset(&(pTableNode->sensor_list), 0);	    
		    ptr_queue_node = pTableNode->sensor_list.head.next;
		    while(ptr_queue_node != NULL && ptr_queue_node != &(pTableNode->sensor_list.head))
		    {
		      /* update the sensor's virtual offset */
		      //ptr_queue_node->info.pos_in_Gv.offset = ptr_queue_node->info.pos_in_Gv.offset; 
		      //The offset is the same as before, so we don't need to update it.

		      /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		      if(ptr_queue_node->info.state != SENSOR_DIE)
		      {
			ptr_queue_node->order_for_live_sensors = order_for_live_sensors++;
			pTableNode->sensor_list.live_sensor_number++;
		      }
		      else
			ptr_queue_node->order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		      /* set the order of sensor on the edge in the virtual graph Gv */
		      ptr_queue_node->order_in_Gv = order_in_Gv++;

		      ptr_queue_node = ptr_queue_node->next; //go to the next queue node
		    }
		  } //end of if-1.1
		  else //else-1.1: head_node is the head node at the table entry pointed by neighbor_table_node
		  { //@Node Relationship: tail_node -> head_node <- neighbor_graph_node

		    /** add all of the sensor queue nodes of neighbor_table_node->sensor_list to new_sensor_list */
		    ptr_queue_node = neighbor_table_node->sensor_list.head.next;
		    while(ptr_queue_node != NULL && ptr_queue_node != &(neighbor_table_node->sensor_list.head))
		    {
		      /* initialize sensor_queue_node with ptr_queue_node */
		      memcpy(&sensor_queue_node, ptr_queue_node, sizeof(sensor_queue_node));

		      /* update the sensor's position in virtual graph, consisting of the eid and offset */
		      sensor_queue_node.info.pos_in_Gv.eid = neighbor_table_node->eid; //eid is already correct since ptr_queue_node is a queue node of neighbor_table_node's sensor list
		      sensor_queue_node.info.pos_in_Gv.offset = ptr_queue_node->info.pos_in_Gv.offset; 

		      /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		      if(sensor_queue_node.info.state != SENSOR_DIE)
		      {
			sensor_queue_node.order_for_live_sensors = order_for_live_sensors++;
			new_sensor_list.live_sensor_number++;
		      }
		      else
			sensor_queue_node.order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		      /* set the order of sensor on the edge in the virtual graph Gv */
		      sensor_queue_node.order_in_Gv = order_in_Gv++;

		      pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(new_sensor_list), (queue_node_t *)&sensor_queue_node);

		      /* update the pointer of sensor list using ptr_queue_node->pSensorListEntry */
		      *(pQueueNode->pSensorListEntry) = pQueueNode;

		      ptr_queue_node = ptr_queue_node->next; //go to the next queue node
		    }

		    /** add the appropriate sensor queue nodes of pTableNode->sensor_list to new_sensor_list */
		    ptr_queue_node = GetSensorQueueNodeJustBeforeVirtualOffset(&(pTableNode->sensor_list), pTableNode->weight);
		    while(ptr_queue_node != NULL && ptr_queue_node->info.pos_in_Gv.offset >= left_hole_offset && ptr_queue_node != &(pTableNode->sensor_list.head))
		    {
		      /* initialize sensor_queue_node with ptr_queue_node */
		      memcpy(&sensor_queue_node, ptr_queue_node, sizeof(sensor_queue_node));

		      /* update the sensor's position in virtual graph, consisting of the eid and offset */
		      sensor_queue_node.info.pos_in_Gv.eid = neighbor_table_node->eid;
		      sensor_queue_node.info.pos_in_Gv.offset = neighbor_table_node->weight + (pTableNode->weight - ptr_queue_node->info.pos_in_Gv.offset); 
                      //sensor_queue_node.info.pos_in_Gv.offset = neighbor_table_node->weight + ptr_queue_node->info.pos_in_Gv.offset; 

		      /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		      if(sensor_queue_node.info.state != SENSOR_DIE)
		      {
			sensor_queue_node.order_for_live_sensors = order_for_live_sensors++;
			new_sensor_list.live_sensor_number++;
		      }
		      else
			sensor_queue_node.order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		      /* set the order of sensor on the edge in the virtual graph Gv */
		      sensor_queue_node.order_in_Gv = order_in_Gv++;

		      pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(new_sensor_list), (queue_node_t *)&sensor_queue_node);

		      /* update the pointer of sensor list using ptr_queue_node->pSensorListEntry in order that sensor table entry in sensor table S can point to the correct sensor queue node for sensor information */
		      *(pQueueNode->pSensorListEntry) = pQueueNode;

		      ptr_queue_node = ptr_queue_node->prev; //go to the previous queue node
		    }

		    /** replace neighbor_table_node's sensor_list with new_sensor_list and reset new_sensor_list since it doesn't have any queue node any more */
		    ReplaceSensorQueue(&(neighbor_table_node->sensor_list), &new_sensor_list);  

		    /** expand the incident hole segment pointed by ptr_neighbor_table_node */
		    segment_length_1 = right_hole_offset - left_hole_offset;
		    segment_length_2 = neighbor_graph_node.weight;
		    hole_segment_length = segment_length_1 + segment_length_2; 
		    //make the length of a new hole segment consisting of neighbor_graph_node and new_graph_node. 

		    neighbor_table_node->weight = hole_segment_length; //set the edge weight to the new hole segment length

		    /* shrink the weight of the virtual edge (tail_node,head_node) in table entry */
		    pTableNode->weight = left_hole_offset;

		    /* update the tail_node_offset of the table entry pointed by pTableNode */
		    if(neighbor_table_node->direction == EDGE_DIRECTION_FORWARD)
		      neighbor_table_node->tail_node_offset_in_Gr = neighbor_table_node->tail_node_offset_in_Gr; //where segment_length_1 = right_hole_offset - left_hole_offset
		    else
		      neighbor_table_node->tail_node_offset_in_Gr = neighbor_table_node->tail_node_offset_in_Gr;

		    /** update the subedges to have the exact virtual edge lengths */
		    /* expand the weight of the subedge of the hole segment (head_node,neighbor_graph_node) */
		    neighbor_table_node->subedge->weight = neighbor_table_node->weight;

		    /* shrink the weight of the subedge of the virtual edge (tail_node,head_node) */
		    pTableNode->subedge->weight = pTableNode->weight;

		    /** update the weight of the edge (head_node,neighbor_graph_node) and that of the edge (tail_node,head_node) */
		    //update the weight of the edge (head_node,neighbor_graph_node)
		    UpdateEdgeWeight(*Gv, head_node, neighbor_graph_node.vertex, neighbor_table_node->weight);

		    //update the weight of the edge (tail_node,head_node)
		    UpdateEdgeWeight(*Gv, tail_node, head_node, pTableNode->weight);

                    ///////////////////////////////////////////////////
		    /** update the sensor list in the table entry pointed by pTableNode */
                    ///////////////////////////////////////////////////

		    /* delete sensor nodes just after right_hole_offset from the sensor_list */
		    DeleteSensorQueueNodesJustAfterVirtualOffset(&(pTableNode->sensor_list), left_hole_offset);

		    /* initialize the order information for sensors on the shrunk sensor segment */
		    order_for_live_sensors = 0;
		    order_in_Gv = 0;

		    /* update info.pos_in_Gv.offset, order_in_Gv, order_for_live_sensors and sensor_list.live_sensor_number */
		    pTableNode->sensor_list.live_sensor_number = 0; //reset live_sensor_number
		    ptr_queue_node = GetSensorQueueNodeJustAfterVirtualOffset(&(pTableNode->sensor_list), 0);	    
		    while(ptr_queue_node != NULL && ptr_queue_node != &(pTableNode->sensor_list.head))
		    {
		      /* update the sensor's virtual offset */
		      //ptr_queue_node->info.pos_in_Gv.offset = ptr_queue_node->info.pos_in_Gv.offset; 
		      //The offset is the same as before, so we don't need to update it.

		      /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		      if(ptr_queue_node->info.state != SENSOR_DIE)
		      {
			ptr_queue_node->order_for_live_sensors = order_for_live_sensors++;
			pTableNode->sensor_list.live_sensor_number++;
		      }
		      else
			ptr_queue_node->order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		      /* set the order of sensor on the edge in the virtual graph Gv */
		      ptr_queue_node->order_in_Gv = order_in_Gv++;

		      ptr_queue_node = ptr_queue_node->next; //go to the next queue node
		    }
		  } //end of else-1.1
		  
		  /** update the real offsets of two end-points in the expanded hole segment in sensing_hole_endpoint_list on the physical edge including the hole segment */
		  hole_number = GetHoleEndpointsWithHoleSegment(&(neighbor_table_node->subedge->edge_queue_entry->sensing_hole_endpoint_list), neighbor_table_node->eid, &left_hole_endpoint, &right_hole_endpoint);
		  if(hole_number != 2)
		  {
		    printf("Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(): hole_number(%d) must be two!\n", hole_number);
		    exit(1);
		  }
		  
		  virtual_offset_of_left_hole = 0;
		  virtual_offset_of_right_hole = neighbor_table_node->weight;
		  
		  physical_offset_of_left_hole = GetPhysicalOffsetOnSubedgeFromSubedgeList(&(neighbor_table_node->subedge->edge_queue_entry->subedge_list), neighbor_table_node->subedge, virtual_offset_of_left_hole);
		  physical_offset_of_right_hole = GetPhysicalOffsetOnSubedgeFromSubedgeList(&(neighbor_table_node->subedge->edge_queue_entry->subedge_list), neighbor_table_node->subedge, virtual_offset_of_right_hole);

		  left_hole_endpoint->offset = physical_offset_of_left_hole;
		  right_hole_endpoint->offset = physical_offset_of_right_hole;
		} //end of if-1
		///////////////////////////////////////////////////////////////////////////////
		else //@else-1: The case where ptr_head_node->type == INTERSECTION_GRAPH_NODE | ENTRANCE_GRAPH_NODE | PROTECTION_GRAPH_NODE, so we cannot expand the previous hole segment 
		{
 		  /* get an unused graph node from the array including G for the hole for left_hole_offset */
		  ptr_new_graph_node = GetUnusedGraphNode(Gv, Gv_size);
		  memcpy(&new_graph_node, ptr_new_graph_node, sizeof(new_graph_node));

		  //add the neighbor relationship between tail_node and new_graph_node
		  AddNeighborRelationship(*Gv, tail_node, tail_node_type, tail_node_role, new_graph_node.vertex, new_graph_node.type, new_graph_node.role, left_hole_offset, density);

		  //add the neighbor relationship between new_graph_node and head_node 
		  AddNeighborRelationship(*Gv, head_node, head_node_type, head_node_role, new_graph_node.vertex, new_graph_node.type, new_graph_node.role, right_hole_offset-left_hole_offset, density);
		  ////////////////////////////////

		  /** insert edge (tail_node, new_graph_node) into table T where new_graph_node corresponds to the hole located at the left_hole_offset */
		  memset(&table_node, 0, sizeof(table_node));
		  table_node.weight = left_hole_offset; //set the edge's weight
		  table_node.density = pTableNode->density; //set the sensor density
		  table_node.type = SEGMENT_TYPE_SENSOR_SEGMENT; //set segment type to sensor segment
		  table_node.tail_node_offset_in_Gr = pTableNode->tail_node_offset_in_Gr; //set the physical offset of tail node in Gr
		  strcpy(table_node.tail_node, pTableNode->tail_node);
		  strcpy(table_node.head_node, new_graph_node.vertex);

		  /* determine the virtual edge's direction */
		  if(pTableNode->direction == EDGE_DIRECTION_FORWARD)
		    table_node.direction = EDGE_DIRECTION_FORWARD;
		  else if(pTableNode->direction == EDGE_DIRECTION_BACKWARD)
		    table_node.direction = EDGE_DIRECTION_BACKWARD;
		  else
		  {
		    printf("Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(): pTableNode->direction(%d) has a wrong value\n", pTableNode->direction);
		    exit(1);
		  }

		  ptr_table_node = Entable(T, &table_node); //enqueue the clone of node into T and increase T->size by 1 and get the pointer to the table node newly added to table T.

		  /* register the pointer to the table node in virtual graph Gv */
		  RegisterTableNodeIntoVirtualGraph(*Gv, *Gv_size, ptr_table_node);

		  /* set up the sensor_list for edge (tail_node, new_graph_node) */
		  ptr_queue_node = pTableNode->sensor_list.head.next;
		  order_for_live_sensors = 0;
		  order_in_Gv = 0;
		  while(ptr_queue_node->info.pos_in_Gv.offset <= left_hole_offset && count < sensor_list_size)
		  {
		    /* update the sensor's position in virtual graph, consisting of the eid and offset */
		    ptr_queue_node->info.pos_in_Gv.eid = ptr_table_node->eid;
	
		    /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		    if(ptr_queue_node->info.state != SENSOR_DIE)
		    {
		      ptr_queue_node->order_for_live_sensors = order_for_live_sensors++;
		      ptr_table_node->sensor_list.live_sensor_number++;
		    }
		    else
		      ptr_queue_node->order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		    /* set the order of sensor on the edge in the virtual graph Gv */
		    ptr_queue_node->order_in_Gv = order_in_Gv++;

		    pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(ptr_table_node->sensor_list), (queue_node_t *)ptr_queue_node);

		    /* update the pointer of sensor list using ptr_queue_node->pSensorListEntry */
		    *(pQueueNode->pSensorListEntry) = pQueueNode;

		    ptr_queue_node = ptr_queue_node->next;
		    count++;
		  }

		  /* set up a new subedge node and enqueue it into the subedge queue */
		  memset(&subedge_node, 0, sizeof(subedge_node));
		  subedge_node.eid = ptr_table_node->eid;
		  strcpy(subedge_node.tail_node, ptr_table_node->tail_node);
		  strcpy(subedge_node.head_node, ptr_table_node->head_node);
		  subedge_node.weight = ptr_table_node->weight;
		  subedge_node.schedule_table_entry = ptr_table_node; //register the pointer to the schedule table entry into schedule_table_entry
		  subedge_node.edge_queue_entry = pEdgeNode; //register the pointer to the edge entry into edge_queue_entry
		  Enqueue((queue_t *)&subedge_queue, (queue_node_t *)&subedge_node);
		  ///////////////////////////////////////////

		  /** insert edge (new_graph_node, head_node) into table T */
		  memset(&table_node, 0, sizeof(table_node));
		  table_node.weight = weight - left_hole_offset; //set the edge's weight
		  table_node.density = pTableNode->density; //set the sensor density
		  table_node.type = SEGMENT_TYPE_HOLE_SEGMENT; //set segment type to hole segment
		  table_node.tail_node_offset_in_Gr = pTableNode->tail_node_offset_in_Gr + left_hole_offset; //set the physical offset of tail node in Gr
		  strcpy(table_node.tail_node, new_graph_node.vertex);
		  strcpy(table_node.head_node, pTableNode->head_node);

		  /* determine the virtual edge's direction */
		  if(pTableNode->direction == EDGE_DIRECTION_FORWARD)
		    table_node.direction = EDGE_DIRECTION_FORWARD;
		  else if(pTableNode->direction == EDGE_DIRECTION_BACKWARD)
		    table_node.direction = EDGE_DIRECTION_BACKWARD;
		  else
		  {
		    printf("Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(): pTableNode->direction(%d) has a wrong value\n", pTableNode->direction);
		    exit(1);
		  }

		  ptr_table_node = Entable(T, &table_node); //enqueue the clone of node into T and increase T->size by 1 and get the pointer to the table node newly added to table T.

		  /* remember the pointer to the table node corresponding to the hole segment */
		  ptr_table_node_for_hole_segment = ptr_table_node;

		  /* register the pointer to the table node in virtual graph Gv */
		  RegisterTableNodeIntoVirtualGraph(*Gv, *Gv_size, ptr_table_node);
		  
		  /* set up the sensor_list for edge (new_graph_node, head_node) */
		  order_for_live_sensors = 0;
		  order_in_Gv = 0;
		  while(ptr_queue_node->info.pos_in_Gv.offset <= weight && count < sensor_list_size)
		  {
		    /* update the sensor's position in virtual graph, consisting of the eid and offset */
		    ptr_queue_node->info.pos_in_Gv.eid = ptr_table_node->eid;
		    ptr_queue_node->info.pos_in_Gv.offset -= left_hole_offset; 

		    /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		    if(ptr_queue_node->info.state != SENSOR_DIE)
		    {
		      ptr_queue_node->order_for_live_sensors = order_for_live_sensors++;
		      ptr_table_node->sensor_list.live_sensor_number++;
		    }
		    else
		      ptr_queue_node->order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		    /* set the order of sensor on the edge in the virtual graph Gv */
		    ptr_queue_node->order_in_Gv = order_in_Gv++;

		    pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(ptr_table_node->sensor_list), (queue_node_t *)ptr_queue_node);

		    /* update the pointer of sensor list using ptr_queue_node->pSensorListEntry */
		    *(pQueueNode->pSensorListEntry) = pQueueNode;

		    ptr_queue_node = ptr_queue_node->next;
		    count++;
		  }

		  /* set up a new subedge node and enqueue it into the subedge queue */
		  memset(&subedge_node, 0, sizeof(subedge_node));
		  subedge_node.eid = ptr_table_node->eid;
		  strcpy(subedge_node.tail_node, ptr_table_node->tail_node);
		  strcpy(subedge_node.head_node, ptr_table_node->head_node);
		  subedge_node.weight = ptr_table_node->weight;
		  subedge_node.schedule_table_entry = ptr_table_node; //register the pointer to the schedule table entry into schedule_table_entry
		  subedge_node.edge_queue_entry = pEdgeNode; //register the pointer to the edge entry into edge_queue_entry
		  Enqueue((queue_t *)&subedge_queue, (queue_node_t *)&subedge_node);
		  ///////////////////////////////////////////

		  /** replace the subedge with the subdivision of the subedge that is the subedge queue and
		      update schedule table entries to let them point to the corresponding subedges */
		  ReplaceSubedgeWithSubdivision(&(pTableNode->subedge), pTableNode->subedge, &subedge_queue);

		  /** insert two endpoints of the new sensing hole segment into the sensing hole endpoint queue */
		  /* register the left hole end-point into pEdgeNode->sensing_hole_endpoint_list */
		  memset(&left_hole_endpoint_node, 0, sizeof(left_hole_endpoint_node));
		  strcpy(left_hole_endpoint_node.vertex, new_graph_node.vertex);
		  left_hole_endpoint_node.eid = ptr_table_node_for_hole_segment->eid; //set the hole endpoint's eid to the eid of the virtual edge containing the hole.

		  virtual_offset_of_left_hole = left_hole_offset; //offset of a sensing hole endpoint in virtual graph Gv; the offset is the offset of the left endpoint of the second subedge in the subdivision in the virtual graph Gv.
		  physical_offset_of_left_hole = GetPhysicalOffsetOnSubedgeFromSubedgeList(&(ptr_table_node_for_hole_segment->subedge->edge_queue_entry->subedge_list), ptr_table_node_for_hole_segment->subedge, virtual_offset_of_left_hole);

		  left_hole_endpoint_node.offset = physical_offset_of_left_hole;
		  left_hole_endpoint_node.edge_queue_entry = pEdgeNode; //register the pointer to the edge entry into edge_queue_entry
		  Enqueue((queue_t *)&(pEdgeNode->sensing_hole_endpoint_list), (queue_node_t *)&left_hole_endpoint_node);

		  /* register the right hole end-point into pEdgeNode->sensing_hole_endpoint_list */
		  memset(&right_hole_endpoint_node, 0, sizeof(right_hole_endpoint_node));
		  strcpy(right_hole_endpoint_node.vertex, head_node);
		  right_hole_endpoint_node.eid = ptr_table_node_for_hole_segment->eid; //set the hole endpoint's eid to the eid of the virtual edge containing the hole.

		  virtual_offset_of_right_hole = right_hole_offset; //offset of a sensing hole endpoint in virtual graph Gv; the offset is the offset of the right endpoint of the second subedge in the subdivision in the virtual graph Gv.
		  physical_offset_of_right_hole = GetPhysicalOffsetOnSubedgeFromSubedgeList(&(ptr_table_node_for_hole_segment->subedge->edge_queue_entry->subedge_list), ptr_table_node_for_hole_segment->subedge, virtual_offset_of_right_hole);

		  right_hole_endpoint_node.offset = physical_offset_of_right_hole;
		  right_hole_endpoint_node.edge_queue_entry = pEdgeNode; //register the pointer to the edge entry into edge_queue_entry
		  
		  Enqueue((queue_t *)&(pEdgeNode->sensing_hole_endpoint_list), (queue_node_t *)&right_hole_endpoint_node);

		  /** add traffic table entry corresponding to new_graph_node to the traffic table added_hole_set */
		  AddTrafficTableEntry(added_hole_set, new_graph_node.vertex);

		  /** add traffic table entry corresponding to head_node to the traffic table added_hole_set */
		  if(head_node_role != ROLE_ENTRANCE_POINT && head_node_role != ROLE_PROTECTION_POINT)
		    AddTrafficTableEntry(added_hole_set, head_node);

		  /** delete the neighbor relationship between tail_node and head_node */
		  DeleteNeighborRelationship(*Gv, tail_node, head_node);

		  /** delete the entry for pTableEntry from table T */
		  DeleteTableEntry(T, pTableNode);
		} //end of else-1
	} //end of HANDLING OF RIGHT-SIDED HOLE SEGMENT IN SENSOR SEGMENT
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	else if((left_hole_offset > 0 && right_hole_offset < weight) && (left_hole_offset < right_hole_offset))
	{//@HANDLING OF MIDDLE HOLE SEGMENT IN SENSOR SEGMENT: //there is one hole segment located between the new left-sided hole segment and the new right-sided hole segment.
		/** NOTE: Dynamic memory problem for (*Gv[i]): refer to item [v2.7.6-2008-3-4] in version.txt 
		    The address values of ptr_tail_node, ptr_head_node, new_graph_node_1 and new_graph_node_2 can change according to the new dynamic memory allocation. So we need to use the structure object copied from &(Gv[i]) rather than the pointer to &(Gv[i]).
		*/

		/* get the pointers to tail_node and head_node in Gv which might be updated by GetUnusedGraphNode() */
		ptr_tail_node = &((*Gv)[tail_node_id-1]); //pointer to the graph node corresponding to tail_node_id
		ptr_head_node = &((*Gv)[head_node_id-1]); //pointer to the graph node corresponding to head_node_id

		/* get the graph node types of tail_node and head_node */
		tail_node_type = ptr_tail_node->type;
		head_node_type = ptr_head_node->type;

		/* get the graph node roles of tail_node and head_node */
		tail_node_role = ptr_tail_node->role;
		head_node_role = ptr_head_node->role;

		//////////////////////////////////////////////////////////////////////////////////////////

 		/* get an unused graph node from the array including G for the hole for left_hole_offset */
		ptr_new_graph_node_1 = GetUnusedGraphNode(Gv, Gv_size);
		memcpy(&new_graph_node_1, ptr_new_graph_node_1, sizeof(new_graph_node_1));

		//////////////////////////////////////////////////////////////////////////////////////////

		/* get an unused graph node from the array including G for the hole for right_hole_offset */
		ptr_new_graph_node_2 = GetUnusedGraphNode(Gv, Gv_size);
		memcpy(&new_graph_node_2, ptr_new_graph_node_2, sizeof(new_graph_node_2));

		//////////////////////////////////////////////////////////////////////////////////////////

		//@for debugging
		//if(strcmp(new_graph_node_1.vertex, "23") == 0 && strcmp(new_graph_node_2.vertex, "24") == 0)
		//  printf("check the virtual edge (%s,%s)\n", new_graph_node_1.vertex, new_graph_node_2.vertex);
		////////////////

		//add the neighbor relationship between tail_node and new_graph_node_1
		AddNeighborRelationship(*Gv, tail_node, tail_node_type, tail_node_role, new_graph_node_1.vertex, new_graph_node_1.type, new_graph_node_1.role, left_hole_offset, density);

		//add the neighbor relationship between new_graph_node_1 and new_graph_node_2 
		AddNeighborRelationship(*Gv, new_graph_node_1.vertex, new_graph_node_1.type, new_graph_node_1.role, new_graph_node_2.vertex, new_graph_node_2.type, new_graph_node_2.role, right_hole_offset-left_hole_offset, density);

		//add the neighbor relationship between new_graph_node_2 and head_node 
		AddNeighborRelationship(*Gv, new_graph_node_2.vertex, new_graph_node_2.type, new_graph_node_2.role, head_node, head_node_type, head_node_role, weight-right_hole_offset, density);

		/** insert edge (tail_node, new_graph_node_1) into table T */
		memset(&table_node, 0, sizeof(table_node));
		table_node.weight = left_hole_offset; //set the edge's weight
		table_node.density = pTableNode->density; //set the sensor density
		table_node.type = SEGMENT_TYPE_SENSOR_SEGMENT; //set segment type to sensor segment
		table_node.tail_node_offset_in_Gr = pTableNode->tail_node_offset_in_Gr; //set the physical offset of tail node in Gr
		strcpy(table_node.tail_node, pTableNode->tail_node);
		strcpy(table_node.head_node, new_graph_node_1.vertex);

	        /* determine the virtual edge's direction */
		if(pTableNode->direction == EDGE_DIRECTION_FORWARD)
		  table_node.direction = EDGE_DIRECTION_FORWARD;
		else if(pTableNode->direction == EDGE_DIRECTION_BACKWARD)
		  table_node.direction = EDGE_DIRECTION_BACKWARD;
		else
		{
		  printf("Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(): pTableNode->direction(%d) has a wrong value\n", pTableNode->direction);
		  exit(1);
		}

		ptr_table_node = Entable(T, &table_node); //enqueue the clone of node into T and increase T->size by 1 and get the pointer to the table node newly added to table T.

		/* register the pointer to the table node in virtual graph Gv */
		RegisterTableNodeIntoVirtualGraph(*Gv, *Gv_size, ptr_table_node);

		/* set up the sensor_list for edge (tail_node, new_graph_node_1) */
		ptr_queue_node = pTableNode->sensor_list.head.next;
		order_for_live_sensors = 0;
		order_in_Gv = 0;
		while(ptr_queue_node->info.pos_in_Gv.offset <= left_hole_offset && count < sensor_list_size)
		{
		  /* update the sensor's position in virtual graph, consisting of the eid and offset */
		  ptr_queue_node->info.pos_in_Gv.eid = ptr_table_node->eid;
		
		  /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		  if(ptr_queue_node->info.state != SENSOR_DIE)
		  {
		    ptr_queue_node->order_for_live_sensors = order_for_live_sensors++;
		    ptr_table_node->sensor_list.live_sensor_number++;
		  }
		  else
		    ptr_queue_node->order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		  /* set the order of sensor on the edge in the virtual graph Gv */
		  ptr_queue_node->order_in_Gv = order_in_Gv++;

		  pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(ptr_table_node->sensor_list), (queue_node_t *)ptr_queue_node);

		  /* update the pointer of sensor list using ptr_queue_node->pSensorListEntry */
		  *(pQueueNode->pSensorListEntry) = pQueueNode;

		  ptr_queue_node = ptr_queue_node->next;
		  count++;
		}

		/* set up a new subedge node and enqueue it into the subedge queue */
		memset(&subedge_node, 0, sizeof(subedge_node));
		subedge_node.eid = ptr_table_node->eid;
		strcpy(subedge_node.tail_node, ptr_table_node->tail_node);
		strcpy(subedge_node.head_node, ptr_table_node->head_node);
		subedge_node.weight = ptr_table_node->weight;
		subedge_node.schedule_table_entry = ptr_table_node; //register the pointer to the schedule table entry into schedule_table_entry
		subedge_node.edge_queue_entry = pEdgeNode; //register the pointer to the edge entry into edge_queue_entry
		Enqueue((queue_t *)&subedge_queue, (queue_node_t *)&subedge_node);
		///////////////////////////////////////////

		/** insert edge (new_graph_node_1, new_graph_node_2) into table T */
		memset(&table_node, 0, sizeof(table_node));
		table_node.weight = right_hole_offset - left_hole_offset; //set the edge's weight
		table_node.density = pTableNode->density; //set the sensor density
		table_node.type = SEGMENT_TYPE_HOLE_SEGMENT; //set segment type to hole segment
		table_node.tail_node_offset_in_Gr = pTableNode->tail_node_offset_in_Gr + left_hole_offset; //set the physical offset of tail node in Gr
		strcpy(table_node.tail_node, new_graph_node_1.vertex);
		strcpy(table_node.head_node, new_graph_node_2.vertex);

	        /* determine the virtual edge's direction */
		if(pTableNode->direction == EDGE_DIRECTION_FORWARD)
		  table_node.direction = EDGE_DIRECTION_FORWARD;
		else if(pTableNode->direction == EDGE_DIRECTION_BACKWARD)
		  table_node.direction = EDGE_DIRECTION_BACKWARD;
		else
		{
		  printf("Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(): pTableNode->direction(%d) has a wrong value\n", pTableNode->direction);
		  exit(1);
		}

		ptr_table_node = Entable(T, &table_node); //enqueue the clone of node into T and increase T->size by 1 and get the pointer to the table node newly added to table T.
		  
		/* remember the pointer to the table node corresponding to the hole segment */
		ptr_table_node_for_hole_segment = ptr_table_node;

		/* register the pointer to the table node in virtual graph Gv */
		RegisterTableNodeIntoVirtualGraph(*Gv, *Gv_size, ptr_table_node);

		/* set up the sensor_list for edge (new_graph_node_1, new_graph_node_2) */
		order_for_live_sensors = 0;
		order_in_Gv = 0;
		while(ptr_queue_node->info.pos_in_Gv.offset <= right_hole_offset && count < sensor_list_size)
	        {
		  /* update the sensor's position in virtual graph, consisting of the eid and offset */
		  ptr_queue_node->info.pos_in_Gv.eid = ptr_table_node->eid;
		  ptr_queue_node->info.pos_in_Gv.offset -= left_hole_offset; 
		  
		  /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		  if(ptr_queue_node->info.state != SENSOR_DIE)
		  {
		    ptr_queue_node->order_for_live_sensors = order_for_live_sensors++;
		    ptr_table_node->sensor_list.live_sensor_number++;
		  }
		  else
		    ptr_queue_node->order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		  /* set the order of sensor on the edge in the virtual graph Gv */
		  ptr_queue_node->order_in_Gv = order_in_Gv++;

		  pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(ptr_table_node->sensor_list), (queue_node_t *)ptr_queue_node);
		
		  /* update the pointer of sensor list using ptr_queue_node->pSensorListEntry */
		  *(pQueueNode->pSensorListEntry) = pQueueNode;
			
		  ptr_queue_node = ptr_queue_node->next;
		  count++;
		}

		/* set up a new subedge node and enqueue it into the subedge queue */
		memset(&subedge_node, 0, sizeof(subedge_node));
		subedge_node.eid = ptr_table_node->eid;
		strcpy(subedge_node.tail_node, ptr_table_node->tail_node);
		strcpy(subedge_node.head_node, ptr_table_node->head_node);
		subedge_node.weight = ptr_table_node->weight;
		subedge_node.schedule_table_entry = ptr_table_node; //register the pointer to the schedule table entry into schedule_table_entry
		subedge_node.edge_queue_entry = pEdgeNode; //register the pointer to the edge entry into edge_queue_entry
		Enqueue((queue_t *)&subedge_queue, (queue_node_t *)&subedge_node);
		///////////////////////////////////////////

		/** insert edge (new_graph_node_2, head_node) into table T */
		memset(&table_node, 0, sizeof(table_node));
		table_node.weight = weight - right_hole_offset; //set the edge's weight
		table_node.density = pTableNode->density; //set the sensor density
		table_node.type = SEGMENT_TYPE_SENSOR_SEGMENT; //set segment type to sensor segment
		table_node.tail_node_offset_in_Gr = pTableNode->tail_node_offset_in_Gr + right_hole_offset; //set the physical offset of tail node in Gr
		strcpy(table_node.tail_node, new_graph_node_2.vertex);
		strcpy(table_node.head_node, pTableNode->head_node);

	        /* determine the virtual edge's direction */
		if(pTableNode->direction == EDGE_DIRECTION_FORWARD)
		  table_node.direction = EDGE_DIRECTION_FORWARD;
		else if(pTableNode->direction == EDGE_DIRECTION_BACKWARD)
		  table_node.direction = EDGE_DIRECTION_BACKWARD;
		else
		{
		  printf("Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(): pTableNode->direction(%d) has a wrong value\n", pTableNode->direction);
		  exit(1);
		}

		ptr_table_node = Entable(T, &table_node); //enqueue the clone of node into T and increase T->size by 1 and get the pointer to the table node newly added to table T.

		/* register the pointer to the table node in virtual graph Gv */
		RegisterTableNodeIntoVirtualGraph(*Gv, *Gv_size, ptr_table_node);

		/* set up the sensor_list for edge (new_graph_node_2, head_node) */
		order_for_live_sensors = 0;
		order_in_Gv = 0;
		while(ptr_queue_node->info.pos_in_Gv.offset <= weight && count < sensor_list_size)
		{
		  /* update the sensor's position in virtual graph, consisting of the eid and offset */
		  ptr_queue_node->info.pos_in_Gv.eid = ptr_table_node->eid;
		  ptr_queue_node->info.pos_in_Gv.offset -= right_hole_offset; 

		  /* update the sensor order for live sensors and the sensor_list's live_sensor_number */
		  if(ptr_queue_node->info.state != SENSOR_DIE)
		  {
		    ptr_queue_node->order_for_live_sensors = order_for_live_sensors++;
		    ptr_table_node->sensor_list.live_sensor_number++;
		  }
		  else
		    ptr_queue_node->order_for_live_sensors = -1; //-1 indicates that this sensor is dead.

		  /* set the order of sensor on the edge in the virtual graph Gv */
		  ptr_queue_node->order_in_Gv = order_in_Gv++;

		  pQueueNode = (sensor_queue_node_t*) Enqueue((queue_t *)&(ptr_table_node->sensor_list), (queue_node_t *)ptr_queue_node);

		  /* update the pointer of sensor list using ptr_queue_node->pSensorListEntry */
		  *(pQueueNode->pSensorListEntry) = pQueueNode;

		  ptr_queue_node = ptr_queue_node->next;
		  count++;
		}

		/* set up a new subedge node and enqueue it into the subedge queue */
		memset(&subedge_node, 0, sizeof(subedge_node));
		subedge_node.eid = ptr_table_node->eid;
		strcpy(subedge_node.tail_node, ptr_table_node->tail_node);
		strcpy(subedge_node.head_node, ptr_table_node->head_node);
		subedge_node.weight = ptr_table_node->weight;
		subedge_node.schedule_table_entry = ptr_table_node; //register the pointer to the schedule table entry into schedule_table_entry
		subedge_node.edge_queue_entry = pEdgeNode; //register the pointer to the edge entry into edge_queue_entry
		Enqueue((queue_t *)&subedge_queue, (queue_node_t *)&subedge_node);
		///////////////////////////////////////////

		/** replace the subedge with the subdivision of the subedge that is the subedge queue and
		    update schedule table entries to let them point to the corresponding subedges */
		ReplaceSubedgeWithSubdivision(&(pTableNode->subedge), pTableNode->subedge, &subedge_queue);		

		/** insert two endpoints of the sensing hole segment into the sensing hole endpoint queue */
		/* register the left hole end-point into pEdgeNode->sensing_hole_endpoint_list */
		memset(&left_hole_endpoint_node, 0, sizeof(left_hole_endpoint_node));
		strcpy(left_hole_endpoint_node.vertex, new_graph_node_1.vertex);
		left_hole_endpoint_node.eid = ptr_table_node_for_hole_segment->eid; //set the hole endpoint's eid to the eid of the virtual edge containing the hole.

		virtual_offset_of_left_hole = left_hole_offset; //offset of a sensing hole endpoint in virtual graph Gv; the offset is the offset of the left endpoint of the second subedge in the subdivision in the virtual graph Gv.
		physical_offset_of_left_hole = GetPhysicalOffsetOnSubedgeFromSubedgeList(&(ptr_table_node_for_hole_segment->subedge->edge_queue_entry->subedge_list), ptr_table_node_for_hole_segment->subedge, virtual_offset_of_left_hole); //@NOTE: This function gives the correct, physical offset since the tail node and the head node of a virtual edge always have the same direction as those of the physical edge

		left_hole_endpoint_node.offset = physical_offset_of_left_hole;
		left_hole_endpoint_node.edge_queue_entry = pEdgeNode; //register the pointer to the edge entry into edge_queue_entry
		  
		Enqueue((queue_t *)&(pEdgeNode->sensing_hole_endpoint_list), (queue_node_t *)&left_hole_endpoint_node);

		/* register the right hole end-point into pEdgeNode->sensing_hole_endpoint_list */
		memset(&right_hole_endpoint_node, 0, sizeof(right_hole_endpoint_node));
		strcpy(right_hole_endpoint_node.vertex, new_graph_node_2.vertex);
		right_hole_endpoint_node.eid = ptr_table_node_for_hole_segment->eid; //set the hole endpoint's eid to the eid of the virtual edge containing the hole.

		virtual_offset_of_right_hole = right_hole_offset; //offset of a sensing hole endpoint in virtual graph Gv; the offset is the offset of the right endpoint of the second subedge in the subdivision in the virtual graph Gv.
		physical_offset_of_right_hole =  GetPhysicalOffsetOnSubedgeFromSubedgeList(&(ptr_table_node_for_hole_segment->subedge->edge_queue_entry->subedge_list), ptr_table_node_for_hole_segment->subedge, virtual_offset_of_right_hole);

		right_hole_endpoint_node.offset = physical_offset_of_right_hole;
		right_hole_endpoint_node.edge_queue_entry = pEdgeNode; //register the pointer to the edge entry into edge_queue_entry
		  
		Enqueue((queue_t *)&(pEdgeNode->sensing_hole_endpoint_list), (queue_node_t *)&right_hole_endpoint_node);

		/** add traffic table entries corresponding to node_id1_name and node_id2_name to the traffic table added_hole_set */
		AddTrafficTableEntry(added_hole_set, new_graph_node_1.vertex);
		AddTrafficTableEntry(added_hole_set, new_graph_node_2.vertex);
		  
	        /** delete the neighbor relationship between tail_node and head_node */
		DeleteNeighborRelationship(*Gv, tail_node, head_node);

		/** delete the entry for pTableEntry from table T */
		DeleteTableEntry(T, pTableNode);
	} //end of HANDLING OF MIDDLE HOLE SEGMENT IN SENSOR SEGMENT
	else
	{
		printf("Update_VirtualGraph_And_ScheduleTable_And_EdgeQueue_For_SensingHole(): left_hole_offset and right_hole_offset are placed out of our handling range\n");
		exit(1);
	}

	return TRUE;
}

void AddNeighborRelationship(struct_graph_node *G, char *node1, GRAPH_NODE_TYPE type1, GRAPH_NODE_ROLE role1, char *node2, GRAPH_NODE_TYPE type2, GRAPH_NODE_ROLE role2, double weight, double density)
{ //add neighbor relationship between node1 and node2 to graph G
  AddNeighborNode(G, node1, node2, weight, density, type2, role2);
  //add the neighnor node2 to node1's entry in G

  AddNeighborNode(G, node2, node1, weight, density, type1, role1);
  //add the neighnor node1 to node2's entry in G
}

void AddNeighborNode(struct_graph_node *G, char *node, char *neighbor_node, double weight, double density, GRAPH_NODE_TYPE type, GRAPH_NODE_ROLE role)
{ //add neighbor node of a given node to adjacency list G
	struct_graph_node *pNewNode = NULL;
	int node_id = atoi(node);
	int neighbor_id = atoi(neighbor_node);
	struct_graph_node *entry = &(G[node_id-1]); //pointer to the entry corresponding to node_id
	struct_graph_node *neighbor_entry = &(G[neighbor_id-1]); //pointer to the entry corresponding to neighbor_id	
	struct_graph_node *ptr = entry->next; //pointer to the entry corresponding to the first neighbor of node_id
	char weight_buf[BUFFER_SIZE];  //buffer for weight that is the length of the edge 
	char density_buf[BUFFER_SIZE]; //buffer for density of sensors deployed for unit distance
	double scale_factor = 1.0; //default scale factor for distance

	sprintf(weight_buf, "%f", (float)weight);
	sprintf(density_buf, "%f", (float)density);

	/* allocate a new graph node */
	pNewNode = Make_Graph_Node(neighbor_node, weight_buf, density_buf, type, role, scale_factor);

	/* let pNodeNode->gnode point to neighbor_entry */
	pNewNode->gnode = neighbor_entry;

	/* insert pNewNode into the neighbor list of entry */
	entry->next = pNewNode;

	/* link pNewNode and ptr */
	pNewNode->next = ptr;

	entry->weight++; //increase the degree of node G[node_id-1]
}

void DeleteNeighborRelationship(struct_graph_node *G, char *node1, char *node2)
{ //delete neighbor relationship between node1 and node2 from graph G
  DeleteNeighborNode(G, node1, node2);
  //delete the neighnor node2 from node1's entry in G

  DeleteNeighborNode(G, node2, node1);
  //delete the neighnor node1 from node2's entry in G
}

void DeleteNeighborNode(struct_graph_node *G, char *node, char *neighbor_node)
{ //delete neighbor node of a given node from adjacency list G
	struct_graph_node *pDeletedNode = NULL;
	int node_id = atoi(node);
	struct_graph_node *entry = &(G[node_id-1]); //pointer to the entry corresponding to node_id
	struct_graph_node *ptr = entry; //pointer to the entry corresponding to the first neighbor of node_id
	struct_graph_node *previous_node = NULL; //pointer to the previous node of the node pointed by ptr
	
	while(ptr != NULL)
	{
		previous_node = ptr;
		ptr = ptr->next;

		if(ptr == NULL)
		{
		  printf("DeleteNeighborNode(): node %s is not the neighbor of node %s, so we cannot delete the edge (%s,%s)\n", neighbor_node, node, node, neighbor_node);
		  exit(1);
		}

		if(strcmp(ptr->vertex, neighbor_node) == 0)
		{
			previous_node->next = ptr->next;
			free(ptr);
			entry->weight--; //decrease the degree of node G[node_id-1]
			break;
		}
	}
}

void AddIntersection_EDD_Queue_Relationship(struct_graph_node *G, int G_size, char *tail_node, char *head_node)
{ //add intersection_edd_queue relationship between virtual_node and tail_node
  AddIntersection_EDD_Queue_Node(G, G_size, tail_node, head_node); //add head_node in tail_node's intersection_edd_queue_node

  AddIntersection_EDD_Queue_Node(G, G_size, head_node, tail_node); //add tail_node in head_node's intersection_edd_queue_node
}

void  AddIntersection_EDD_Queue_Node(struct_graph_node *G, int G_size, char *tail_node, char *head_node)
{ //add head_node in tail_node's intersection_edd_queue_node
  struct_graph_node *tail_gnode = NULL; //pointer to the graph node of tail_node in the node arrary of G
  struct_graph_node *head_gnode = NULL; //pointer to the graph node of tail_node in the node arrary of G
  intersection_edd_queue_t *Q = NULL; //pointer to the intersection queue node of tail_node
  intersection_edd_queue_node_t *p = NULL;
  intersection_edd_queue_node_t *pQueueNode = NULL;
  intersection_edd_queue_node_t queue_node; //intersection_edd_queue node
  char *u = NULL, *v = NULL; //pointers to strings
  int i;

  tail_gnode = LookupGraph(G, G_size, tail_node);

  head_gnode = GetNeighborGraphNode(G, G_size, tail_node, head_node); //obtain the pointer to the graph node corresponding to vertex head_node in the edge (tail_node, head_node)

  Q = tail_gnode->intersection_edd_queue;
  assert_memory(Q);

  /* interserction a new intersection queue node into Q */
  memset(&queue_node, 0, sizeof(queue_node));
  queue_node.tail_gnode = tail_gnode;
  queue_node.head_gnode = head_gnode;

  Enqueue((queue_t*) Q, (queue_node_t*) &queue_node);
}

void DeleteIntersection_EDD_Queue_Relationship(struct_graph_node *G, int G_size, char *tail_node, char *head_node)
{ //delete intersection relationship between tail_node and head_node from graph G in the intersection_edd_queues of both tail_node and head_node
  DeleteIntersection_EDD_Queue_Node(G, G_size, tail_node, head_node); //delete head_node in tail_node's intersection_edd_queue_node

  DeleteIntersection_EDD_Queue_Node(G, G_size, head_node, tail_node); //delete tail_node in head_node's intersection_edd_queue_node
}

void  DeleteIntersection_EDD_Queue_Node(struct_graph_node *G, int G_size, char *tail_node, char *head_node)
{ //delete head_node in tail_node's intersection_edd_queue_node
  struct_graph_node *tail_gnode = NULL; //pointer to the graph node of tail_node in the node arrary of G
  intersection_edd_queue_t *Q = NULL; //pointer to the intersection queue node of tail_node
  intersection_edd_queue_node_t *p = NULL;
  intersection_edd_queue_node_t *pQueueNode = NULL;
  char *u = NULL, *v = NULL; //pointers to strings
  int i;

  tail_gnode = LookupGraph(G, G_size, tail_node);

  Q = tail_gnode->intersection_edd_queue;
  assert_memory(Q);

  if(Q->size == 0)
  {
    printf("DeleteIntersection_EDD_Queue_Node(): Error: Q->size is %d\n", Q->size);
    exit(1);
    //return;
  }

  /* position the intersection_edd_queue node for (tail_node, head_node) */
  pQueueNode = &(Q->head);
  for(i = 0; i < Q->size; i++)
  {
    pQueueNode = pQueueNode->next;
    u = pQueueNode->tail_gnode->vertex;
    v = pQueueNode->head_gnode->vertex;
    if((strcmp(u, tail_node) == 0) && (strcmp(v, head_node) == 0)) 
    {
      p = pQueueNode->prev; //p points to the previous node of the node pointed by pQueueNode;

      /* link the node before pQueueNode's node and the node after pQueueuNode's node each other */
      p->next = pQueueNode->next;
      pQueueNode->next->prev = p;
      break;
    }
  }

  if(i == Q->size)
  {
    printf("DeleteIntersection_EDD_Queue_Node(): Error: there is no intersection_edd_queue node corresponding to edge (%s,%s)\n", tail_node, head_node);
    exit(1);
    //return;
  }

  Q->size--;

  free(pQueueNode);
}

void InitTrafficTable(struct_traffic_table* table)
{ //initialize traffic table
	table->number = 0;
	table->list = NULL;
}

void MakeTrafficTable(struct_traffic_table* table, char *node1, char *node2)
{ //make traffic table with node1 and node2
	int i = 0; //index
	size_t len = 0; //length of node name
	
	if(table->number > 0)
		Free_Traffic_Table(table); //release the memory occupied by the traffic source table

	if(node1 != NULL && node2 == NULL)
	{
		//table->list = (char**) calloc(1, sizeof(char*));
		//assert_memory(table->list);

		/* add node1 to table */
		//len = strlen(node1);
		//table->list[i] = (char*) calloc(1, sizeof(char)*(len+1));
		//assert_memory(table->list[i]);
		//strcpy(table->list[i], node1);

		table->list = (struct_traffic_node*) calloc(1, sizeof(struct_traffic_node));
		assert_memory(table->list);
		strcpy(table->list[i].vertex, node1);

		table->number++;
	}
	else if(node1 == NULL && node2 != NULL)
	{
		//table->list = (char**) calloc(1, sizeof(char*));
		//assert_memory(table->list);

		/* add node2 to table */
		//len = strlen(node2);
		//table->list[i] = (char*) calloc(1, sizeof(char)*(len+1));
		//assert_memory(table->list[i]);
		//strcpy(table->list[i], node2);

		table->list = (struct_traffic_node*) calloc(1, sizeof(struct_traffic_node));
		assert_memory(table->list);
		strcpy(table->list[i].vertex, node2);

		table->number++;
	}
	else if(node1 != NULL && node2 != NULL)
	{
		//table->list = (char**) calloc(2, sizeof(char*));
		//assert_memory(table->list);

		/* add node1 to table */
		//len = strlen(node1);
		//table->list[i] = (char*) calloc(1, sizeof(char)*(len+1));
		//assert_memory(table->list[i]);
		//strcpy(table->list[i], node1);

		table->list = (struct_traffic_node*) calloc(2, sizeof(struct_traffic_node));
		assert_memory(table->list);
		strcpy(table->list[i].vertex, node1);

		table->number++;
		i++;

		/* add node2 to table */
		//len = strlen(node2);
		//table->list[i] = (char*) calloc(1, sizeof(char)*(len+1));
		//assert_memory(table->list[i]);
		//strcpy(table->list[i], node2);

		strcpy(table->list[i].vertex, node2);

		table->number++;
	}
}

void CopyTrafficTable(struct_traffic_table *dst, struct_traffic_table *src)
{ //copy traffic table from src to dst
	int i;
	size_t len = 0; //length of node name

	if(src->number <= 0)
	{
		printf("CopyTrafficTable(): we cannot copy traffic table because src->number(%d) is not greater than 0\n", src->number);
		exit(1);
	}

	if(dst->number > 0)
		Free_Traffic_Table(dst);

	dst->number = src->number;

	//dst->list = (char**) calloc(dst->number, sizeof(char*));
	dst->list = (struct_traffic_node*) calloc(dst->number, sizeof(struct_traffic_node));
	assert_memory(dst->list);

	/* copy table entries from src to dst */
	for(i = 0; i < src->number; i++)
	{
		//len = strlen(src->list[i]);
		//dst->list[i] = (char*) calloc(1, sizeof(char)*(len+1));
		//strcpy(dst->list[i], src->list[i]);

		strcpy(dst->list[i].vertex, src->list[i].vertex);
	}
}

void MergeTrafficTable(struct_traffic_table *dst, struct_traffic_table *src)
{ //merge the traffic table of src to the traffic table of dst
	int i;

	if(src->number == 0)
	  return;

	/* merge the two tables of src and dst into dst */
	for(i = 0; i < src->number; i++)
	{	  
	  if(!IsVertexInTrafficTable(dst, src->list[i].vertex))
	  {
	    AddTrafficTableEntry(dst, src->list[i].vertex);
	  }
	}
}

boolean AddTrafficTableEntry(struct_traffic_table *table, char *node)
{ //add traffic table entry corresponding to node to table
	size_t mem_size = 0; //memory size
	size_t len = 0; //length of node name

	int i = 0; //index
	
	if(node == NULL || strcmp(node, "") == 0)
		return FALSE;

	for(i = 0; i < table->number; i++)
	{
		if(strcmp(node, table->list[i].vertex) == 0)
			return FALSE;
	}

	if(table->number == 0)
	{ //allocate the memory for a new table and a new table entry
		table->list = (struct_traffic_node*) calloc(1, sizeof(struct_traffic_node));
		assert_memory(table->list);
		table->number++; //increase the number of table entries
	}
	else
	{ //reallocate the memory for table and another table entry
		mem_size = table->number*sizeof(struct_traffic_node); 
		table->list = (struct_traffic_node*) realloc(table->list, mem_size + sizeof(struct_traffic_node));
		assert_memory(table->list);
		table->number++; //increase the number of table entries
	}

	//copy node into a new table entry
	//len = strlen(node);
	i = table->number - 1;
	strcpy(table->list[i].vertex, node);

	return TRUE;
}

void SubtractTrafficTable(struct_traffic_table *table, struct_traffic_table *set)
{ //delete nodes in set from table
        int last_index = table->number-1;
	int i;

	if(set->number == 0)
	  return;

	/* delete node from table */
	for(i = 0; i < set->number; i++)
	{	  
	  DeleteTrafficTableEntry(table, set->list[i].vertex);
	  //delete traffic table entry corresponding to node from table	  
	}
}

boolean DeleteTrafficTableEntry(struct_traffic_table *table, char *node)
{ //delete traffic table entry corresponding to node from table
	size_t mem_size = 0; //memory size
	size_t len = 0; //length of node name
	int i = 0; //index
	int deleted_index = -1; //index of deleted node
	int last_index = table->number-1; //index of the last node in table
	
	if(table->number == 0 || node == NULL || strcmp(node, "") == 0)
		return FALSE;

	for(i = 0; i < table->number; i++)
	{
		if(strcmp(node, table->list[i].vertex) == 0)
		{
		  deleted_index = i;
		  break;
		}
	}

	if(deleted_index == -1)
	  return FALSE;
	else if(deleted_index == last_index && table->number == 1)
	{
	        free(table->list);
		table->list = NULL;
		table->number = 0;		
	}
	else if(deleted_index == last_index) //table->number >= 2

	{ //delete the last entry and shrink the memory for the table
	        table->number--; //decrease the number of table entries
		mem_size = table->number*sizeof(struct_traffic_node); 
		table->list = (struct_traffic_node*) realloc(table->list, mem_size);
		assert_memory(table->list);
	}
	else
	{ //delete the last entry and shrink the memory for the table
       	        /* exchange the deleted node with the last node */
	        strcpy(table->list[deleted_index].vertex, table->list[last_index].vertex);

	        table->number--; //decrease the number of table entries
		mem_size = table->number*sizeof(struct_traffic_node); 
		table->list = (struct_traffic_node*) realloc(table->list, mem_size);
		assert_memory(table->list);
	}

	return TRUE;
}

boolean IsVertexInTrafficTable(struct_traffic_table *table, char *node)
{ //check whether node already belongs to table
	int i = 0; //index
	
	if(node == NULL || strcmp(node, "") == 0)
	{
		printf("node is NULL or node is \"\"\n");
#ifdef __DEBUG_INTERACTIVE_MODE__
                fgetc(stdin);
#endif
		//exit(1);
		return FALSE;
	}
	
	for(i = 0; i < table->number; i++)
	{
		//if(strcmp(node, table->list[i]) == 0)
		if(strcmp(node, table->list[i].vertex) == 0)
			return TRUE;
	}

	return FALSE;
}

int GetTrafficTableEntry_ID_With_Index(struct_traffic_table *table, int index)
{ //get the intersection id for a traffic table entry with index
  struct_traffic_node *table_entry = NULL; //pointer to a table entry
  int intersection_id = 0; //intersection id in a road network

  if(index < 0 || index >= table->number)
  {
    printf("GetTrafficTableEntry_With_Index(): index(=%d) should be greater than or equal to 0 and also be less than table->number(%d)\n", index, table->number);
    exit(1);
  }

  table_entry = &(table->list[index]);
  intersection_id = atoi(table_entry->vertex);

  return intersection_id;
}

void Adjust_Maximum_AccessPoint_Number(parameter_t *param, struct_traffic_table *ap_table)
{ //adjust the number of multiple APs according to the number of maximum APs
  int max_ap_number = param->communication_AP_maximum_number; //the number of maximum APs deployed in the road network
  int deleted_ap_number = 0; //number of deleted APs from ap_tabl
  size_t mem_size = 0; //memory size

  /** check the validity of max_ap_number */
  if(param->communication_multiple_AP_flag == FALSE && max_ap_number > 1)
  {
    printf("Adjust_Maximum_AccessPoint_Number(): Error: for param->communication_multiple_AP_flag == FALSE, max_ap_number(%d) must be 1\n", max_ap_number);
    exit(1);
  }
  else 
  {
    if(ap_table->number < max_ap_number)
    {
      printf("Adjust_Maximum_AccessPoint_Number(): Error: ap_table->number(%d) < max_ap_number(%d)\n", ap_table->number, max_ap_number);
      exit(1);
    }
    else
    {
      deleted_ap_number = ap_table->number - max_ap_number;
    }
  }

  /** delete APs from ap_table by deleted_ap_number */
  if(deleted_ap_number == 0)
    return;

  /** shrink the ap_table by deleted_ap_number */
  ap_table->number = max_ap_number; //decrease the number of table entries
  mem_size = ap_table->number*sizeof(struct_traffic_node); 
  ap_table->list = (struct_traffic_node*) realloc(ap_table->list, mem_size);
  assert_memory(ap_table->list);
}

void Store_Traffic_Table_Into_File(FILE *fp, struct_traffic_table *traffic_table)
{ //store traffic table into file pointed by file pointer fp
	int i; //index of for-loop

	if(fp == NULL)
	{
		printf("Store_Traffic_Table_Into_File(): fp is NULL\n");
		exit(1);
	}

	/** sort the traffic table entries with vertex name in ascending order */
	SortTrafficTable(traffic_table);

	/** store the traffic table into file */
	for(i = 0; i < traffic_table->number; i++)
	{
		if(i < traffic_table->number-1)
			//fprintf(fp, "%s, ", traffic_table->list[i]);
			fprintf(fp, "%s, ", traffic_table->list[i].vertex);
		else
			//fprintf(fp, "%s.", traffic_table->list[i]);
			fprintf(fp, "%s.", traffic_table->list[i].vertex);
	}
	fprintf(fp, "\n\n");
}

void Store_Sensing_Hole_Endpoints_And_Labeling_Into_File(edge_queue_t *Er, struct_traffic_table *src_table_for_Gr, struct_traffic_table *dst_table_for_Gr, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, char* filename, int indicator)
{ //store sensing hole endpoints for each edge in the real graph Gr with labeling (i.e., CLUSTER_ENTRANCE or CLUSTER_PROTECTION) into file
	FILE *fp = NULL; //pointer to the file that will contain the initial sensing hole information

	/** open initial_sensing_hole_file */
	fp = fopen(filename, "w");
	if(!fp)
	{
	    fprintf(stderr, "Error: unable to open file \"%s\"\n", filename);
#ifdef __DEBUG_INTERACTIVE_MODE__
            fgetc(stdin);
#endif
	    exit(1);
	}

	if(indicator == 0)
		fprintf(fp, "### Initial Sensing Hole Endpoints ###\n\n");
	else if(indicator == 1)
		fprintf(fp, "### Final Sensing Hole Endpoints ###\n\n");
	else
		fprintf(fp, "### Intermediate Sensing Hole Endpoints ###\n\n");

	/** store entrance node table and protection node table before handling the sensing holes into file */
	fprintf(fp, "Entrance node set (size=%d) before handling the sensing hole endpoints: ", src_table_for_Gr->number);
	Store_Traffic_Table_Into_File(fp, src_table_for_Gr);

	fprintf(fp, "Protection node set (size=%d) before handling the sensing hole endpoints: ", dst_table_for_Gr->number);
	Store_Traffic_Table_Into_File(fp, dst_table_for_Gr);

	/** store entrance node table and protection node table with all the sensing holes into file */
	fprintf(fp, "Entrance node set (size=%d) after the network lifetime: ", src_table_for_Gv->number);
	Store_Traffic_Table_Into_File(fp, src_table_for_Gv);

	fprintf(fp, "Protection node set (size=%d) after the network lifetime: ", dst_table_for_Gv->number);
	Store_Traffic_Table_Into_File(fp, dst_table_for_Gv);

	/** store sensing hole endpoints for each edge in the real graph Gr into file */
	Store_Sensing_Hole_Endpoints_Into_File(fp, Er);

	/** close file */
	fclose(fp);
}

void Store_Sensing_Hole_Endpoints_Into_File(FILE *fp, edge_queue_t *Er)
{ //store initial sensing hole endpoints for each edge in the real graph Gr into file pointed by file pointer fp
	edge_queue_node_t *pEdgeNode = NULL; //pointer to an edge queue node
	hole_endpoint_queue_node_t *pHoleNode = NULL; //pointer to a sensing hole endpoint node
	int i, j; //indices of for-loops
	int total_hole_endpoint_number = 0; //total number of sensing hole endpoints

	if(fp == NULL)
	{
		printf("Store_Traffic_Table_Into_File(): fp is NULL\n");
		exit(1);
	}

	/** sort the sensing hole endpoints of each edge in the ascending order */
	pEdgeNode = &(Er->head);
	for(i = 0; i < Er->size; i++)
	{
		pEdgeNode = pEdgeNode->next;
		SortSensingHoleEndpointQueue(&(pEdgeNode->sensing_hole_endpoint_list));
		//sort the sensing hole endpoints of each edge according to the offsets in the real graph Gr in the ascending order
	}

	/** store the sensing hole endpoints for each edge into file */
	pEdgeNode = &(Er->head);
	for(i = 0; i < Er->size; i++)
	{
		pEdgeNode = pEdgeNode->next;
		fprintf(fp, "%d: Edge(%s,%s) with %d hole endpoints:\t", i+1, pEdgeNode->tail_node, pEdgeNode->head_node, pEdgeNode->sensing_hole_endpoint_list.size);

		pHoleNode = &(pEdgeNode->sensing_hole_endpoint_list.head);
		for(j = 0; j < pEdgeNode->sensing_hole_endpoint_list.size; j++)
		{
			pHoleNode = pHoleNode->next;
			if(j < pEdgeNode->sensing_hole_endpoint_list.size-1)
				fprintf(fp, "(%s, %d, %f), ", pHoleNode->vertex, pHoleNode->hid, pHoleNode->offset);
			else
				fprintf(fp, "(%s, %d, %f).", pHoleNode->vertex, pHoleNode->hid, pHoleNode->offset);		
		}
		total_hole_endpoint_number += pEdgeNode->sensing_hole_endpoint_list.size;
		fprintf(fp, "\n\n");		
	}
	fprintf(fp, "=> Total hole endpoints: %d\n\n", total_hole_endpoint_number);
}

void SortTrafficTable(struct_traffic_table *T)
{ //sort traffic table entries in ascending order according to vertex name
	int entry_num = T->number; //number of entries in traffic table T
	int *A; //array of vertex names represented in integer
	char node[NAME_SIZE]; //vertex name
	int i; //index for for-loop

    /** for testing of quick sort */
	//int B[8] = {5, 3, 2, 6, 4, 1, 5, 7};
	//int B[8] = {5, 3, 2, 6, 4, 1, 3, 7};
	//perform quick sort
	//QuickSort(B, 0, 7);

	/* generate sensor array containing pointers to all of the sensor nodes in schedule table */
	A = (int*) calloc(entry_num, sizeof(int));
	assert_memory(A);

	for(i = 0; i < entry_num; i++)
	{
		//A[i] = atoi(T->list[i]);
		A[i] = atoi(T->list[i].vertex);
	}

	/* sort arrary A in ascending order with vertex name */
	QuickSort(A, 0, entry_num-1);
	//perform quick sort for vertex name array

	/* destroy traffic table and then make traffic table again with the array A */
	Free_Traffic_Table(T);
	for(i = 0; i < entry_num; i++)
	{
	        //_itoa(A[i], node, 10);
                sprintf(node, "%d", A[i]);

		AddTrafficTableEntry(T, node);
		//add traffic table entry corresponding to node to table
	}

	/* release the memory allocated to array A */
	free(A);
}

void SortSensingHoleEndpointQueue(hole_endpoint_queue_t *Q)
{ //sort the sensing hole endpoints of the hole endpoint queue Q according to the offsets in the real graph Gr in the ascending order
	int hole_endpoint_num = Q->size; //number of hole endpoints in the hole endpoint queue
	hole_endpoint_queue_node_t **A; //array of hole endpoint queue node pointers
	hole_endpoint_queue_node_t *pQueueNode = NULL; //pointer to a sensing hole endpoint queue node
	int i; //index for for-loop

	/* generate sensing hole endpoint array containing pointers to all of the nodes in hole endpoint queue */
	A = (hole_endpoint_queue_node_t**) calloc(hole_endpoint_num, sizeof(hole_endpoint_queue_node_t*));
	assert_memory(A);

	pQueueNode = &(Q->head);
	for(i = 0; i < hole_endpoint_num; i++)
	{
		pQueueNode = pQueueNode->next;
		A[i] = pQueueNode;
	}

	/* sort arrary A in ascending order with hole endpoint's offset */
	QuickSortForSensingHoleEndpointArray(A, 0, hole_endpoint_num-1);
	//perform quick sort for sensing hole endpoint array

	/* rearrange the pointers of Q using A in the ascending order */
	RearrangeSensingHoleEndpointQueue(Q, A);

	/* release the memory allocated to array A */
	free(A);
}

void RearrangeSensingHoleEndpointQueue(hole_endpoint_queue_t* Q, hole_endpoint_queue_node_t **A)
{ //rearrange the pointers of sensing hole endpoint queue Q using A in the ascending order
	hole_endpoint_queue_node_t *p = NULL, *q = NULL; //pointers to sensing hole endpoint queue nodes
	int hole_endpoint_num = Q->size; //number of sensing hole endpoints in the queue Q
	int i; //index for for-loop

	/* rearrange the links using A such that A[i]'s offsets are ordered in ascending order */
	q = &(Q->head);
	q->next = &(Q->head);
	q->prev = &(Q->head);
	for(i = 0; i < hole_endpoint_num; i++, p = p->next)
	{
		p = A[i];		
		p->prev = Q->head.prev;
		p->next = &(Q->head);
		Q->head.prev->next = p;
		Q->head.prev = p;

		/* set up the hole endpoint's order */
		p->order = i;
	}
}

struct_graph_node* GetUnusedGraphNode(struct_graph_node **Gv, int *Gv_size)
{ //get an unused graph node from the array including Gv; if needed, update the information for virtual graph in T
  struct_graph_node *G = *Gv; //G points to virtual graph Gv
  int G_size = *Gv_size; //G_size is the size of Gv
  struct_graph_node* ptr_graph_node = NULL;
  size_t G_mem_size = 0;//memory size of graph G
  int i = 0; //for-loop index
  int index = -1; //index for the available graph node in G

  for(i = 0 ; i < G_size; i++)
  {
    if(G[i].status == USAGE_UNUSED)
    {
      index = i;
      break;
    }
  }

  if(index != -1)
  {
    ptr_graph_node = &(G[index]);
  }
  else
  { //since there is no available slot for a new graph node, we expand the virtual graph G by one
    G_mem_size = G_size*sizeof(struct_graph_node);

    /* reallocate the memory for graph G and another struct_graph_node */
    G = (struct_graph_node*) realloc(G, G_mem_size + sizeof(struct_graph_node));
    assert_memory(G);
    (G_size)++;

    index = G_size - 1;

    /* get the pointer to the new graph node */
    ptr_graph_node = &(G[index]);

    /* update the virtual graph Gv and Gv_size */
    *Gv = G;
    *Gv_size = G_size;
  }

  /* initialize the available graph node for the new usage */
  sprintf(ptr_graph_node->vertex, "%d", index+1);
  ptr_graph_node->status = USAGE_USED;
  ptr_graph_node->type = HOLE_GRAPH_NODE; //graph node that is a sensing hole
  ptr_graph_node->role = ROLE_HOLE_ENDPOINT; //graph node role as a sensing hole  
  ptr_graph_node->weight = 0;
  ptr_graph_node->density = 0;
  ptr_graph_node->offset_in_Gr = 0;
  ptr_graph_node->next = NULL;
  ptr_graph_node->ptr_table_node = NULL;

  return ptr_graph_node;
}

void UpdateEdgeWeight(struct_graph_node *G, char *u, char *v, double new_weight)
{ //update the weights of the edge (u,v) and edge (v,u)
  int node_u_id = atoi(u); //node u's id
  int node_v_id = atoi(v); //node v's id
  struct_graph_node *ptr_node_u = &(G[node_u_id-1]); //pointer to graph node u
  struct_graph_node *ptr_node_v = &(G[node_v_id-1]); //pointer to graph node v
  struct_graph_node *ptr = NULL; //pointer to graph node

  if(u == NULL || v == NULL)
  {
    printf("UpdateEdgeWeight(): u or v is NULL\n");
    exit(1);
  }
  
  /* update the weight of the edge (u,v) */
  ptr = ptr_node_u->next;
  while(ptr != NULL)
  {
    if(strcmp(ptr->vertex, v) == 0)
    {
      ptr->weight = new_weight;
      break;
    }

    ptr = ptr->next;
  }

  if(ptr == NULL)
  {
    printf("UpdateEdgeWeight(): v(%s) is not a neighbor of u(%s)\n", v, u);
    exit(1);
  }

  /* update the weight of the edge (v,u) */
  ptr = ptr_node_v->next;
  while(ptr != NULL)
  {
    if(strcmp(ptr->vertex, u) == 0)
    {
      ptr->weight = new_weight;
      break;
    }

    ptr = ptr->next;
  }

  if(ptr == NULL)
  {
    printf("UpdateEdgeWeight(): u(%s) is not a neighbor of v(%s)\n", u, v);
    exit(1);
  }
}

void DeleteGraphNode(struct_graph_node *G, int G_size, char *u)
{ //delete graph node u from graph G for future reuse for another vertex
  int node_u_id = atoi(u); //node u's id
  struct_graph_node *ptr_node_u = NULL;
  struct_graph_node *ptr = NULL; //pointer to graph node

  if(u == NULL)
  {
    printf("DeleteGraphNode(): u is NULL\n");
    exit(1);
  }
  else if(G == NULL)
  {
    printf("DeleteGraphNode(): G is NULL\n");
    exit(1);
  }
  else if(node_u_id > G_size)
  {
    printf("DeleteGraphNode(): node_u_id(%d) > G_size(%d)\n", node_u_id, G_size);
    exit(1);
  }

  /* let ptr_node_u point to the graph node for node_u_id */
  ptr_node_u = &(G[node_u_id-1]);

  if(ptr_node_u->status == USAGE_UNUSED)
  {
    printf("DeleteGraphNode(): graph node(%s) has already been deleted!\n", ptr_node_u->vertex);
    exit(1);
  }
  else if(ptr_node_u->weight > 0)
  {
    printf("DeleteGraphNode(): u has %d neighbor(s)\n", (int)ptr_node_u->weight);
    exit(1);
  }
  
/*   /\* delete the edge (u,v) and the edge (v,u) *\/ */
/*   ptr = ptr_node_u->next; */
/*   while(ptr != NULL) */
/*   { */
/*     DeleteNeighborRelationship(G, u, ptr->vertex); */
/*     //delete neighbor relationship between u and ptr->vertex from graph G */

/*     ptr = ptr->next; */
/*   } */

  memset(ptr_node_u, 0, sizeof(struct_graph_node));
  ptr_node_u->status = USAGE_UNUSED;
}

void SetPhysicalOffsetWithTableEntry(char *node, double virtual_offset, struct_graph_node *G, int G_size, schedule_table_node_t *pTableNode)
{ //set up node's physical offset corresponding to virtual offset right_hole_offset in Gv with the table entry pointed by pTableNode.
  int node_id = atoi(node); //node id
  int tail_node_id = atoi(pTableNode->tail_node); //id of tail node
  int head_node_id = atoi(pTableNode->head_node); //id of head node
  struct_graph_node *ptr_graph_node = NULL; //pointer to the graph node for node
  struct_graph_node *ptr_tail_graph_node = NULL; //pointer to the graph node for tail_node
  struct_graph_node *ptr_head_graph_node = NULL; //pointer to the graph node for head_node
  enum_edge_direction_t direction = pTableNode->direction; //direction for the physical edge = {EDGE_DIRECTION_FORWARD, EDGE_DIRECTION_BACKWARD}
  double segment_length = 0; //the length of the road segment corresponding to the edge (tail_node,head_node)

  /* check the parameters */
  if(node == NULL)
  {
    printf("SetPhysicalOffsetWithTableEntry(): node is NULL\n");
    exit(1);
  }
  else if(G == NULL)
  {
    printf("SetPhysicalOffsetWithTableEntry(): G is NULL\n");
    exit(1);
  }
  else if(node_id > G_size || tail_node_id > G_size || head_node_id > G_size)
  {
    printf("SetPhysicalOffsetWithTableEntry(): node_id(%d) > G_size(%d) or tail_node_id(%d) > G_size(%d) or head_node_id(%d) > G_size(%d)\n", node_id, G_size, tail_node_id, G_size, head_node_id, G_size);
    exit(1);
  }

  /* set the pointer to graph nodes */
  ptr_graph_node = &(G[node_id-1]);
  ptr_tail_graph_node = &(G[tail_node_id-1]);
  ptr_head_graph_node = &(G[head_node_id-1]);

  if(pTableNode->direction == EDGE_DIRECTION_FORWARD)
  { //tail_node is before head_node in real graph Gr: 
    //@POSITION: physical edge tail node -> tail_node => head_node -> physical edge head node

    ptr_graph_node->offset_in_Gr = pTableNode->tail_node_offset_in_Gr + virtual_offset;
  }
  else if(pTableNode->direction == EDGE_DIRECTION_BACKWARD)
  { //tail_node is behind head_node in real graph Gr:
    //@POSITION: physical edge tail node -> head_node <= tail_node -> physical edge head node

    segment_length = pTableNode->weight;
    ptr_graph_node->offset_in_Gr = pTableNode->tail_node_offset_in_Gr + (segment_length - virtual_offset);
  }
  else
  {
    printf("SetPhysicalPosition(): pTableNode->direction(%d) has a wrong value.\n", pTableNode->direction);
    exit(1);
  }
}

double GetPhysicalOffset(char *node, struct_graph_node *G, int G_size)
{ //get node's physical offset in the physical edge in the real graph G.
  int node_id = atoi(node); //node id
  struct_graph_node *ptr_graph_node = NULL; //pointer to the graph node for node
  double offset = 0; //offset of node in the physical edge

  /* check the parameters */
  if(node == NULL)
  {
    printf("GetPhysicalOffset(): node is NULL\n");
    exit(1);
  }
  else if(G == NULL)
  {
    printf("GetPhysicalOffset(): G is NULL\n");
    exit(1);
  }
  else if(node_id > G_size)
  {
    printf("GetPhysicalOffset(): node_id(%d) > G_size(%d)\n", node_id, G_size);
    exit(1);
  }

  /* set the pointer to graph nodes */
  ptr_graph_node = &(G[node_id-1]);

  offset = ptr_graph_node->offset_in_Gr;

  return offset;
}

double SetPhysicalOffset(char *node, double new_offset, struct_graph_node *G, int G_size)
{ //set node's physical offset in the physical edge in the real graph G with new_offset.
  int node_id = atoi(node); //node id
  struct_graph_node *ptr_graph_node = NULL; //pointer to the graph node for node

  /* check the parameters */
  if(node == NULL)
  {
    printf("SetPhysicalOffset(): node is NULL\n");
    exit(1);
  }
  else if(G == NULL)
  {
    printf("SetPhysicalOffset(): G is NULL\n");
    exit(1);
  }
  else if(node_id > G_size)
  {
    printf("SetPhysicalOffset(): node_id(%d) > G_size(%d)\n", node_id, G_size);
    exit(1);
  }

  /* set the pointer to graph nodes */
  ptr_graph_node = &(G[node_id-1]);

  ptr_graph_node->offset_in_Gr = new_offset;

  return new_offset;
}

struct_graph_node* Make_Forwarding_Graph(struct_graph_node* G, int G_size, int* G_new_size)
{ //make a new forwarding graph for data forwarding used by Access Point (AP) based on the road network graph G
  struct_graph_node* G_new = NULL; //forwarding graph including the target point for packet forwarding
  struct_graph_node *src = NULL, *dst = NULL; //pointers to graph nodes 
  int i = 0; //index for for-loop
  int num_of_neighbors = 0; //number of neighboring vertices
  int virtual_node_id = 0; //virtual node's id

  if(G_size <= 0)
  {
    printf("Make_Forwarding_Graph(): G_size(=%d) is less than or equal to zero\n", G_size);
    return NULL;
  }
  
  /** make adjacency list array for the vertices in G_new */
  *G_new_size = G_size;
  G_new = (struct_graph_node*) calloc(*G_new_size, sizeof(struct_graph_node));
  assert_memory(G_new);

  for(i = 0; i < G_size; i++) //for-1
  {
    src = &(G[i]);
    dst = &(G_new[i]);

    CopyGraphNodeAndNeighborList(src, dst, G_new, *G_new_size); //copy src's node information and neighbor list into dst
  } //end of for-1
 
  return G_new;
}

void CopyGraphNodeAndNeighborList(struct_graph_node* src, struct_graph_node* dst, struct_graph_node *G, int G_size)
{ //copy src's node information and neighbor list into dst
  int i = 0; //index for for-loop
  int num_of_neighbors = (int) src->weight; //number of neighboring vertices
  struct_graph_node* ptr_for_src_neighbor = NULL; //pointer to the graph node for a neighboring node of src node
  struct_graph_node* ptr_for_dst_neighbor = NULL; //pointer to the graph node for a neighboring node of dst node
  struct_graph_node* node = NULL; //pointer to a graph node
  angle_queue_node_t angle_queue_node; //angle queue node
  intersection_edd_queue_node_t intersection_edd_queue_node; //intersection EDD queue node

  /** initialize node information */
  dst->gnode = dst; //gnode points to itself
  strcpy(dst->vertex, src->vertex); //vertex name
  dst->type = src->type; //graph node type
  dst->role = src->role; //graph node role
  dst->weight = 0; //number of neighboring vertices
  dst->next = NULL;
  
  dst->coordinate.x = src->coordinate.x;
  dst->coordinate.y = src->coordinate.y;

  /* let dst->ptr_stationary_node point to the stationary node corresponding to this vertex */
  dst->ptr_stationary_node = src->ptr_stationary_node;

  /** allocate the memory of angle queue and initialize it */
  dst->angle_queue = (angle_queue_t*) calloc(1, sizeof(angle_queue_t));
  assert_memory(dst->angle_queue);
  InitQueue((queue_t*)dst->angle_queue, QTYPE_ANGLE);

  /** allocate the memory of intersection EDD queue and initialize it */
  dst->intersection_edd_queue = (intersection_edd_queue_t*) calloc(1, sizeof(intersection_edd_queue_t));
  assert_memory(dst->intersection_edd_queue);
  InitQueue((queue_t*)dst->intersection_edd_queue, QTYPE_INTERSECTION_EDD);

  /** make a neighboring list for neighboring nodes for index */
  ptr_for_src_neighbor = src; 
  ptr_for_dst_neighbor = dst;

  SetGraphNodeStatus(G, G_size, USAGE_USED); //set graph node's status to USAGE_USED in order to let LookupGraph() return the pointer to the graph node correctly
  for(i = 0; i < num_of_neighbors; i++) //for-1
  {
    ptr_for_src_neighbor = ptr_for_src_neighbor->next; //let ptr_for_src_neighbor point to the next neighboring node

    node = Make_Graph_Node2(ptr_for_src_neighbor->vertex, ptr_for_src_neighbor->weight, ptr_for_src_neighbor->density, ptr_for_src_neighbor->type, ptr_for_src_neighbor->role);
    ptr_for_dst_neighbor->next = node; //let ptr_for_dst_neighbor point to the next neighboring node
    ptr_for_dst_neighbor = node; //move ptr_for_dst_neighbor to the next neighbor node
    ptr_for_dst_neighbor->gnode = LookupGraph(G, G_size, node->vertex); //let gnode point to the adjacency list entry for node->vertex

    /** enqueue the angle queue node corresponding to the edge <G[i].vertex,ptr->vertex> into angle queue queue */
    memset(&angle_queue_node, 0, sizeof(angle_queue_node));
    strcpy(angle_queue_node.tail_node, dst->vertex);
    strcpy(angle_queue_node.head_node, ptr_for_dst_neighbor->vertex);
    angle_queue_node.tail_gnode = dst;
    angle_queue_node.head_gnode = ptr_for_dst_neighbor;

    Enqueue((queue_t*)dst->angle_queue, (queue_node_t*)&angle_queue_node);

    /** enqueue the intersection edd queue node corresponding to the edge <G[i].vertex,ptr->vertex> into intersection edd queue */
    memset(&intersection_edd_queue_node, 0, sizeof(intersection_edd_queue_node));
    intersection_edd_queue_node.tail_gnode = dst;
    intersection_edd_queue_node.head_gnode = ptr_for_dst_neighbor;

    Enqueue((queue_t*)dst->intersection_edd_queue, (queue_node_t*)&intersection_edd_queue_node);
    
    dst->weight++; //increase the degree of dst's graph node
  } //end of for-1

  /** construct as many conditional probability queue nodes as the neighboring edges per the edge where G[i].vertex is the tail node */
  ConstructConditionalForwardingProbabilityQueueNodes_Per_DirectionalEdge(G, G_size);
}

void Make_Intersection_EDD_Queue_And_NeighborList_For_VirtualNode(int virtual_node_id, struct_graph_node* dst, struct_graph_node* Gr, int Gr_size, struct_graph_node* Ga, int Ga_size, edge_queue_t *Ea, directional_edge_queue_t *DEa, char *tail_node, char *head_node, double left_edge_length, double right_edge_length)
{ //make the intersection EDD queue and the neighbor list for a virtual node that is the target point for the packet delivery towards a vehicle
  GRAPH_NODE_TYPE default_node_type = INTERSECTION_GRAPH_NODE; //default graph node's type for data forwarding
  GRAPH_NODE_ROLE default_node_role = ROLE_INTERSECTION_POINT; //default graph node's role for data forwarding
  GRAPH_NODE_TYPE virtual_node_type = NONINTERSECTION_GRAPH_NODE; //virtual graph node's type for data forwarding
  GRAPH_NODE_ROLE virtual_node_role = ROLE_NONINTERSECTION_POINT; //virtual graph node's role for data forwarding
  char virtual_node[NAME_SIZE]; //virtual node's name

  int i = 0; //index for for-loop
  int num_of_neighbors = 2; //the number of neighboring vertices is 2 since the virtual node has two neighbors as non-intersection graph node
  struct_graph_node* ptr_for_src_neighbor = NULL; //pointer to the graph node for a neighboring node of src node
  struct_graph_node* ptr_for_dst_neighbor = NULL; //pointer to the graph node for a neighboring node of dst node
  struct_graph_node* node = NULL; //pointer to a graph node
  struct_graph_node *virtual_node_gnode = NULL; //virtual graph node in the node array of the adjacency list for the augmented graph Ga
  struct_graph_node *neighbor_gnode = NULL; //pointer to the graph node of the neighbor graph node
  struct_graph_node *tail_gnode = NULL, *head_gnode = NULL; //pointers to the graph nodes of the tail node and head node in the node array of the adjacency list

  boolean flip_flag = FALSE; //flip flag to indicate whether tail_node is tail node or head node in the edge (tail_node, head_node)

  edge_queue_node_t *ptr_edge_node = NULL; //pointer to the edge node of (virtual_node, tail_node) of (virtual_node, head_node)
  directional_edge_queue_node_t *ptr_directional_edge_node = NULL; //pointer to the directional edge node of (virtual_node, tail_node) of (virtual_node, head_node)


  /** initialize node information */
  itoa(virtual_node_id, virtual_node);
  dst->gnode = dst; //gnode points to itself
  strcpy(dst->vertex, virtual_node); //virtual node name
  dst->type = virtual_node_type; //graph node type
  dst->role = virtual_node_role; //graph node role
  dst->weight = 0; //number of neighboring vertices
  dst->next = NULL;
  
  dst->coordinate.x = -1;
  dst->coordinate.y = -1;

  /** allocate the memory of intersection EDD queue and initialize it */
  dst->intersection_edd_queue = (intersection_edd_queue_t*) calloc(1, sizeof(intersection_edd_queue_t));
  assert_memory(dst->intersection_edd_queue);
  InitQueue((queue_t*)dst->intersection_edd_queue, QTYPE_INTERSECTION_EDD);

  /** delete intersection relationship between tail_node and head_node from graph G in the intersection_edd_queues of both tail_node and head_node; Note that the following function must be called before DeleteNeighborRelationship(), because in the function, the pointer to the graph node of head_node in the edge (tail_node, head_node) is used. */
  DeleteIntersection_EDD_Queue_Relationship(Ga, Ga_size, tail_node, head_node);

  /** delete undirectional edge and directional edges from edge queue E and directional edge queue DE */
  /* delete the undirectional edge (tail_node, head_node) from edge queue E; Note that the following function must be called before DeleteNeighborRelationship(), because in the function, the pointer to the graph node of head_node in the edge (tail_node, head_node) is used. */
  FastDeleteEdge(Ea, tail_node, head_node, Ga);

  /* delete the directional edge (tail_node, head_node) from directional edge queue DE; Note that the following function must be called before DeleteNeighborRelationship(), because in the function, the pointer to the graph node of head_node in the edge (tail_node, head_node) is used. */
  FastDeleteDirectionalEdge(DEa, tail_node, head_node, Ga);

  /* delete the directional edge (head_node, tail_node) from directional edge queue DE */
  FastDeleteDirectionalEdge(DEa, head_node, tail_node, Ga);

  /** delete neighbor relationship between tail_node and head_node from graph G */
  DeleteNeighborRelationship(Ga, tail_node, head_node);

  /** construct neighbor relationship between virtual_node and tail_node/head_node in G;
      Note the adding sequence for tail node virtual_node is from (virtual_node, head_node) to (virtual_node, tail_node) .
      This is because AddNeighborNode() adds a new neighbor node in front of the the first neighbor node in the neighbor list of a tail node. 
  */
  /* add neighbor relationship between virtual_node and head_node to graph G */
  AddNeighborRelationship(Ga, virtual_node, virtual_node_type, virtual_node_role, head_node, default_node_type, default_node_role, right_edge_length, 1);

  /* add neighbor relationship between virtual_node and tail_node to graph G */
  AddNeighborRelationship(Ga, virtual_node, virtual_node_type, virtual_node_role, tail_node, default_node_type, default_node_role, left_edge_length, 1);


  /** construct intersection relationship among tail_node, head_node, and virtual_node from graph G in the intersection_edd_queues; Note that before AddIntersection_EDD_Queue_Relationship(),  AddNeighborRelationship() must be called, because the pointer to the graph node of head_node in the edge (virtual_node, tail_node/head_node) is used.  */
  /* add intersection_edd_queue relationship between virtual_node and tail_node */
  AddIntersection_EDD_Queue_Relationship(Ga, Ga_size, virtual_node, tail_node);

  /* add intersection_edd_queue relationship between virtual_node and tail_node */
  AddIntersection_EDD_Queue_Relationship(Ga, Ga_size, virtual_node, head_node);


  /** update a neighboring list for neighboring nodes for dst */
  ptr_for_dst_neighbor = dst;

  for(i = 0; i < num_of_neighbors; i++) //for-1
  {
    ptr_for_dst_neighbor = ptr_for_dst_neighbor->next; //let ptr_for_dst_neighbor point to the next neighboring node
    ptr_for_dst_neighbor->gnode = LookupGraph(Ga, Ga_size, ptr_for_dst_neighbor->vertex); //let gnode point to the adjacency list entry for node->vertex

    /** insert one undirectional edge and two directional edges into E and DE, respectively;
        Note that the insertion of directional edges must be performed after the neighbor node is added into the neighbor list.
        This is because Insert_DirectionalEdge_Into_DirectionalEdgeQueue() sets the head_node_gnode and tail_node_gnode to point 
        the appropriate graph nodes */
    if(strcmp(ptr_for_dst_neighbor->vertex, tail_node) == 0) //for edge (virtual_node, tail_node)
    {
      /** obtain the pointer to the neighbor node of edge (tail_node, virtual_node)  */
      neighbor_gnode = GetNeighborGraphNode(Ga, Ga_size, tail_node, virtual_node);

      /** insert edge (tail_node, virtual_node) into edge queue E */
      ptr_edge_node = Insert_Edge_Into_EdgeQueue(Ea, Ga, Ga_size, tail_node, virtual_node, left_edge_length);

      /* set ptr_for_dst_neighbor's ptr_edge_node to point to the new edge for edge (tail_node, virtual_node) */
      ptr_for_dst_neighbor->ptr_edge_node = ptr_edge_node;

      /* set neighbor_gnode's ptr_edge_node to point to the new edge for edge (tail_node, virtual_node) */
      neighbor_gnode->ptr_edge_node = ptr_edge_node;

   
      /** insert edge (tail_node, virtual_node) into directional edge queue DE */
      ptr_directional_edge_node = Insert_DirectionalEdge_Into_DirectionalEdgeQueue(DEa, Ga, Ga_size, tail_node, virtual_node, left_edge_length);
      
      /* set neighbor_gnode's ptr_directional_edge_node to point to the new directional edge for edge (tail_node, virtual_node) */
      neighbor_gnode->ptr_directional_edge_node = ptr_directional_edge_node;


      /** insert edge (virtual_node, tail_node) into directional edge queue DE */
      ptr_directional_edge_node = Insert_DirectionalEdge_Into_DirectionalEdgeQueue(DEa, Ga, Ga_size, virtual_node, tail_node, left_edge_length);
     
      /* set ptr_for_dst_neighbor's ptr_directional_edge_node to point to the new directional edge for edge (virtual_node, tail_node) */
      ptr_for_dst_neighbor->ptr_directional_edge_node = ptr_directional_edge_node;
    }
    else if(strcmp(ptr_for_dst_neighbor->vertex, head_node) == 0) //for edge (virtual_node, head_node)
    {
      /** obtain the pointer to the neighbor node of edge (head_node, virtual_node)  */
      neighbor_gnode = GetNeighborGraphNode(Ga, Ga_size, head_node, virtual_node);

      /** insert edge (head_node, virtual_node) into edge queue E */
      ptr_edge_node = Insert_Edge_Into_EdgeQueue(Ea, Ga, Ga_size, head_node, virtual_node, right_edge_length);

      /* set ptr_for_dst_neighbor's ptr_edge_node to point to the new edge for edge (head_node, virtual_node) */
      ptr_for_dst_neighbor->ptr_edge_node = ptr_edge_node;
   
      /* set neighbor_gnode's ptr_edge_node to point to the new edge for edge (head_node, virtual_node) */
      neighbor_gnode->ptr_edge_node = ptr_edge_node;

      /** insert edge (head_node, virtual_node) into directional edge queue DE */
      ptr_directional_edge_node = Insert_DirectionalEdge_Into_DirectionalEdgeQueue(DEa, Ga, Ga_size, head_node, virtual_node, right_edge_length);
      
      /* set neighbor_gnode's ptr_directional_edge_node to point to the new directional edge for edge (head_node, virtual_node) */
      neighbor_gnode->ptr_directional_edge_node = ptr_directional_edge_node;

      /** insert edge (virtual_node, head_node) into directional edge queue DE */
      ptr_directional_edge_node = Insert_DirectionalEdge_Into_DirectionalEdgeQueue(DEa, Ga, Ga_size, virtual_node, head_node, right_edge_length); //insert a new edge node (virtual_node, tail_node) into the end of edge queue E
     
      /* set ptr_for_dst_neighbor's ptr_directional_edge_node to point to the new directional edge for edge (virtual_node, head_node) */
      ptr_for_dst_neighbor->ptr_directional_edge_node = ptr_directional_edge_node;
    }
  } //end of for-1

  /** determine the geographic coordination of the virtual node with the tail_node's and head_node's ones */
  /* obtain the graph node pointers to the tail_node, head_node, and virtual_node in the node array of the adjacency list for the augmented graph G */
  tail_gnode = LookupGraph(Ga, Ga_size, tail_node);
  head_gnode = LookupGraph(Ga, Ga_size, head_node);
  virtual_node_gnode = LookupGraph(Ga, Ga_size, virtual_node);

  Set_IntermediateNode_GeographicCoordinate(virtual_node_gnode, tail_gnode, head_gnode, left_edge_length, right_edge_length);

  /** copy the vehicular traffic information in the directional edges (tail_node, head_node) and (head_node, tail_node) into its subdivided subedges (tail_node, virtual_node), (virtual_node, head_node), (head_node, virtual_node) and (virtual_node, tail_node) in the directional edges */
  CopyVehicularTrafficStatistics_From_DirectionalEdges_To_DirectionalSubedges(Gr, Gr_size, Ga, Ga_size, tail_node, head_node, virtual_node);
}

boolean PickTargetPoint(int *trajectory, int trajectory_size, double target_point_interdistance, int target_point_index, struct_graph_node *G, struct_coordinate3_t *target_point)
{ //pick a target point on the target vehicle's trajectory
  boolean flag = FALSE; //flag to indicate whether a target point is a virtual node (TRUE) or an intersection node (FALSE).
  int i = 0; //index for for-loop
  int num_of_edges = trajectory_size - 1; //number of the edges on the trajectory
  char tail_node[NAME_SIZE], head_node[NAME_SIZE]; //tail node and head node of an edge
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to a directional edge queue node
  double edge_length = 0; //edge length
  double required_distance = target_point_interdistance * target_point_index; //the required distance corresponding to a pair of target point interdistance and target point index
  double accumulated_distance = 0; //the accumulated distance of the edges from the beginning of the trajectory towards the end of the trajectory
  double offset = 0; //offset from the tail node in the edge satisfying the required_distance on the vehicle trajectory

  /** parameter validity checking */
  if(target_point_interdistance <= 0)
  {
    printf("PickTargetPoint(): target_point_interdistance(=%.2f) should be greater than 0\n", target_point_interdistance);
    exit(1);
  }
  else if(target_point_index < 0)
  {
    printf("PickTargetPoint(): target_point_index(=%d) should be a non-negative integer\n", target_point_index);
    exit(1);
  }

  /** search for the edge and offset corresponding to the required distance on the vehicle trajectory */
  for(i = 0; i < num_of_edges; i++) //for-1
  {
    strcpy(tail_node, itoa(trajectory[i], NULL)); //tail node for the directional edge on the vehicle trajectory
    strcpy(head_node, itoa(trajectory[i+1], NULL)); //head node for the directional edge on the vehicle trajectory
    pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
    if(pEdgeNode == NULL)
    {
      printf("PickTargetPoint(): pEdgeNode for <%s,%s> is NULL\n", tail_node, head_node);
      exit(1);
    }

    edge_length = pEdgeNode->weight;
    accumulated_distance += edge_length;

    if(accumulated_distance >= required_distance)
    {
      offset = edge_length - (accumulated_distance - required_distance);
      target_point->eid = pEdgeNode->eid;
      target_point->offset = offset;
      target_point->enode = pEdgeNode;
      flag = TRUE;
      break;
    }
  } //end of for-1

  if(flag == FALSE)
  {
    printf("PickTargetPoint(): Error: There is no target point on the vehicle trajectory satisfying the pair of target_point_interdistance(=%.2f) and target_point_index(=%d)\n", target_point_interdistance, target_point_index);
    exit(1);
  }

  return flag;
}

void AugmentGraph_With_TargetPoint(struct_access_point_t *AP, struct_coordinate3_t *target_point, struct_graph_node *Gr, int Gr_size)
{ //make an augmented graph Ga for data forwarding with the road network graph Gr and the target point;
  //(tail_node, head_node) is subdivied into (tail_node, virtual_node) and (virtual_node, head_node).

  /* data structures of AP */
  struct_graph_node **Ga = &(AP->Ga); //pointer to the augmented graph Ga for data forwarding 
  int *Ga_size = &(AP->Ga_size); //number of nodes in Ga
  edge_queue_t *Ea = &(AP->Ea); //pointer to edge queue Ea
  directional_edge_queue_t *DEa = &(AP->DEa); //pointer to directional edge queue DEa

  int i = 0; //index for for-loop
  int virtual_node_id = 0; //virtual node's id
  char virtual_node[NAME_SIZE]; //virtual node between tain node and head node
  char tail_node[NAME_SIZE], head_node[NAME_SIZE]; //tail node and head node of an edge
  struct_graph_node *neighbor_node_gnode = NULL; //pointer to the graph node of the neighbor node of the virtual node
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to a directional edge queue node
  double edge_length = target_point->enode->weight; //edge length
  double left_edge_length = target_point->offset; //length of the edge (tail_node, target_point)
  double right_edge_length = edge_length - target_point->offset; //length of the edge (tail_node, target_point)
  int num_of_neighbors = 0; //number of neighbor nodes
  boolean flip_flag = FALSE; //flip flag to indicate whether tail_node is tail node or head node in the edge (tail_node, head_node)
  
  /** expand graph Ga by one to accommodate the virtual graph node that is a target point and 
      update the values of the gnode pointers of the neighbor nodes in the neighbor lists */
  Expand_Graph_Memory(Ga, Ga_size); //Since Ga points to the address of AP->Ga, AP->Ga can point to the newly allocated Ga array after this operation.


  /** set up the node names of tail_node, head_node and virtual_node */
  /* set up tail_node's name and head_node's name */
  strcpy(tail_node, target_point->enode->tail_node);
  strcpy(head_node, target_point->enode->head_node);

  /* set virtual node's name to *Ga_size that is the id of the last node in the adjancency list's node array */
  virtual_node_id = *Ga_size;
  strcpy(virtual_node, itoa(virtual_node_id, NULL));


  /** set up the virtual node information and neighboring list for the undetermined target point */
  /* set up the intersection EDD queue and make the neighbor list for a virtual node that is the target point for the packet delivery towards a vehicle */
  Make_Intersection_EDD_Queue_And_NeighborList_For_VirtualNode(virtual_node_id, &((*Ga)[virtual_node_id-1]), Gr, Gr_size, *Ga, *Ga_size, Ea, DEa, tail_node, head_node, left_edge_length, right_edge_length);
 

  /** copy the target point into AP's target point */
  memcpy(&(AP->target_point), target_point, sizeof(*target_point));

  /** make a traffic table entry for the target point as virtual access point, that is, geographic packet desination */
  AddTrafficTableEntry(&(AP->ap_table_for_target_point), virtual_node); //add traffic table entry corresponding to node to table
}

void SetGraphNodeStatus(struct_graph_node *G, int G_size, USAGE_STATUS status)
{ //set graph node's status to status, such as USAGE_USED, in order to let LookupGraph() return the pointer to the graph node correctly
  int i = 0; //index

  for(i = 0; i < G_size; i++)
  {
    G[i].status = status;
  }
}

void Set_IntermediateNode_GeographicCoordinate(struct_graph_node *virtual_node_gnode, struct_graph_node *tail_node_gnode, struct_graph_node *head_node_gnode, double left_edge_length, double right_edge_length)
{ //determine the geographic coordination of the intermedidate node with the tail_node's and head_node's ones
  struct_graph_node *w = virtual_node_gnode; //intermediate node
  struct_graph_node *u = tail_node_gnode; //tail node
  struct_graph_node *v = head_node_gnode; //head node
  double l = left_edge_length; //the length of edge (u,w)
  double r = right_edge_length; //the length of edge (w,v)
  double x_u = u->coordinate.x, y_u = u->coordinate.y; //coordinate of tail node u
  double x_v = v->coordinate.x, y_v = v->coordinate.y; //coordinate of head node v
  double *x_w = &(w->coordinate.x), *y_w = &(w->coordinate.y); //coordinate of virtual node w

  /* check whether the sum of l and r is zero or not */
  if(l+r == 0)
  {
    printf("Set_IntermediateNode_GeographicCoordinate(): the sum of l(=%.2f) and r(=%.2f) is zero\n", l, r);
    exit(1);
  }

  /* determine the x-coordinate of virtual node w */
  if(x_u <= x_v)
  {
    *x_w = r/(l+r)*x_u + l/(l+r)*x_v;
  }
  else
  {
    *x_w = l/(l+r)*x_u + r/(l+r)*x_v;
  }

  /* determine the y-coordinate of virtual node w */
  if(y_u <= y_v)
  {
    *y_w = r/(l+r)*y_u + l/(l+r)*y_v;
  }
  else
  {
    *y_w = l/(l+r)*y_u + r/(l+r)*y_v;
  }
}

void Expand_Graph_Memory(struct_graph_node **G, int *G_size)
{ //expand graph G by one to accommodate another graph node
  size_t G_mem_size = 0;//memory size of graph G

  /* obtain the memory size of graph Ga */
  G_mem_size = *G_size * sizeof(struct_graph_node);

  /* reallocate the memory for graph G and another struct_graph_node */
  *G = (struct_graph_node*) realloc(*G, G_mem_size + sizeof(struct_graph_node));
  assert_memory(G);
  (*G_size)++; //increase G_size by one since a graph node is added into the graph G

  /* update the values of the gnode pointers of the neighbor nodes in the neighbor lists */
  Update_NeighborNode_Gnode(*G, *G_size);

  /* update the values of the gnode pointers of tail_gnode and head_gnode in the intersection_edd_queues for each node in the node array for G */
  Update_Intersection_EDD_QueueNode_Gnode(*G, *G_size);
}

void Update_NeighborNode_Gnode(struct_graph_node *G, int G_size)
{ //update the values of the gnode pointers of the neighbor nodes in the neighbor lists
  int i = 0, j = 0; //indices for for-loops
  struct_graph_node *neighbor = NULL; //pointer to a neighbor graph node
  int num_of_neighbors = 0; //number of neighbor nodes

  for(i = 0; i < G_size; i++)
  {
    neighbor = &(G[i]);
    num_of_neighbors = (int) G[i].weight;
    for(j = 0; j < num_of_neighbors; j++)
    {
      neighbor = neighbor->next;
      neighbor->gnode = LookupGraph(G, G_size, neighbor->vertex); //update the pointer to the graph node corresponding to neighbor vertex's name since the memory location may be changed by realloc() for expanding the graph node array
    }
  }
}

void Update_Intersection_EDD_QueueNode_Gnode(struct_graph_node *G, int G_size)
{ //update the values of the gnode pointers of tail_gnode and head_gnode in the intersection_edd_queues for each node in the node array for G
  int i = 0, j = 0; //indices for for-loops
  struct_graph_node *tail_gnode = NULL; //pointer to a tail graph node
  struct_graph_node *head_gnode = NULL; //pointer to a head graph node
  struct_graph_node *neighbor = NULL; //pointer to a neighbor graph node of tail_gnode
  int num_of_neighbors = 0; //number of neighbor nodes
  intersection_edd_queue_t *Q = NULL; //pointer to the intersection queue node of tail_node
  intersection_edd_queue_node_t *pQueueNode = NULL;

  for(i = 0; i < G_size; i++)
  {
    tail_gnode = &(G[i]);
    Q = tail_gnode->intersection_edd_queue;
    if(Q == NULL)
    { //when Q is equal to NULL, we skip the update of intersection_edd_queue, since the queue is not created yet.
      continue;
    }
    else if(Q->size != (int) tail_gnode->weight)
    {
      printf("Update_Intersection_EDD_QueueNode_Gnode(): Error: Q->size(%d) must be equal to tail_gnode->weight(%d)\n", Q->size, (int) tail_gnode->weight);
      exit(1);
    }

    pQueueNode = &(Q->head);
    num_of_neighbors = (int) G[i].weight;
    neighbor = tail_gnode;
    for(j = 0; j < num_of_neighbors; j++)
    {
      neighbor = neighbor->next;
      pQueueNode = pQueueNode->next;

      head_gnode = neighbor; //obtain the pointer to the graph node corresponding to vertex head_node in the edge (tail_node, head_node)
      pQueueNode->tail_gnode = tail_gnode;
      pQueueNode->head_gnode = head_gnode;
    }
  }
}

void CopyVehicularTrafficStatistics(struct_graph_node *Gr, int Gr_size, struct_graph_node *Ga, int Ga_size)
{ //copy the vehicular traffic statistics of the edges in Gr into that in Ga
  int i = 0; //index for for-loop
  struct_graph_node* src = NULL; //pointer to the graph node in Gr
  struct_graph_node* dst = NULL; //pointer to the graph node in Ga

  if(Gr_size <= 0)
  {
    printf("CopyVehicularTrafficStatistics(): Gr_size(=%d) is less than or equal to zero\n", Gr_size);
    exit(1);
  }
  
  for(i = 0; i < Gr_size; i++) //for-1
  {
    src = &(Gr[i]);
    dst = &(Ga[i]);
        
    CopyVehicularTrafficStatistics_For_NeighborList(src, dst); //copy the traffic information in src's neighbor list into that in dst
  } //end of for-1
}

void CopyVehicularTrafficStatistics_For_NeighborList(struct_graph_node *src, struct_graph_node *dst)
{ //copy the vehicular traffic information in src's neighbor list into that in dst
 
  int i = 0; //index for for-loop
  int num_of_neighbors = (int) src->weight; //number of neighboring vertices
  struct_graph_node* ptr_for_src_neighbor = NULL; //pointer to the graph node for a neighboring node of src node
  struct_graph_node* ptr_for_dst_neighbor = NULL; //pointer to the graph node for a neighboring node of dst node

  /** copy the vehicular traffic information in the tail node of the directional edges */
  dst->mean_interarrival_time = src->mean_interarrival_time; //average interarrival time
  dst->last_arrival_time = src->last_arrival_time; //last arrival time
  dst->sum_of_interarrival_time = src->sum_of_interarrival_time; //sum of interarrival time
  dst->number_of_interarrivals = src->number_of_interarrivals; //number of interarrivals    
  dst->number_of_arrivals = src->number_of_arrivals; //number of arrivals that is used for the denominator of the branch probability
  dst->number_of_branching = src->number_of_branching; //number of branching from the tail node to this node that is a head node where the edge is <tail_node, head_node>; this is equal to number_of_interarrivals+1.  

  /** copy the vehicular traffic information in the head node of each directional edge */
  ptr_for_src_neighbor = src; 
  ptr_for_dst_neighbor = dst;

  for(i = 0; i < num_of_neighbors; i++) //for-1
  {
    ptr_for_src_neighbor = ptr_for_src_neighbor->next; //let ptr_for_src_neighbor point to the next neighboring node
    ptr_for_dst_neighbor = ptr_for_dst_neighbor->next; //let ptr_for_dst_neighbor point to the next neighboring node

    ptr_for_dst_neighbor->mean_interarrival_time = ptr_for_src_neighbor->mean_interarrival_time; //average interarrival time
    ptr_for_dst_neighbor->last_arrival_time = ptr_for_src_neighbor->last_arrival_time; //last arrival time
    ptr_for_dst_neighbor->sum_of_interarrival_time = ptr_for_src_neighbor->sum_of_interarrival_time; //sum of interarrival time
    ptr_for_dst_neighbor->number_of_interarrivals = ptr_for_src_neighbor->number_of_interarrivals; //number of interarrivals    
    ptr_for_dst_neighbor->number_of_arrivals = ptr_for_src_neighbor->number_of_arrivals; //number of arrivals that is used for the denominator of the branch probability
    ptr_for_dst_neighbor->number_of_branching = ptr_for_src_neighbor->number_of_branching; //number of branching from the tail node to this node that is a head node where the edge is <tail_node, head_node>; this is equal to number_of_interarrivals+1.

  } //end of for-1
}

void CopyVehicularTrafficStatistics_From_DirectionalEdges_To_DirectionalSubedges(struct_graph_node *Gr, int Gr_size, struct_graph_node *Ga, int Ga_size, char *tail_node, char *head_node, char *virtual_node)
{ //copy the vehicular traffic information in the directional edges into its subdivided subedges in the directional edges
  struct_graph_node *gnode_Gr = NULL; //pointer to a graph node in road network graph Gr
  struct_graph_node *gnode_Ga = NULL; //pointer to a graph node in augmented graph Ga          
  struct_graph_node *virtual_node_gnode = NULL; //pointer to a graph node for virtual node in node array
  struct_graph_node *head_gnode_for_tail_node = NULL; //pointer to a neighbor graph node for edge (tail_node, head_node)  
  struct_graph_node *tail_gnode_for_head_node = NULL; //pointer to a neighbor graph node for edge (head_node, tail_node)

  /** make the vehicular traffic information (i.e., number of arrivals) in the virtual node for the directional edges 
      (head_node, virtual_node) and (tail_node, virtual_node) */
  virtual_node_gnode = LookupGraph(Ga, Ga_size, virtual_node); //pointer to a graph node for virtual node in node array
  head_gnode_for_tail_node = GetNeighborGraphNode(Gr, Gr_size, tail_node, head_node);
  tail_gnode_for_head_node = GetNeighborGraphNode(Gr, Gr_size, head_node, tail_node);

  virtual_node_gnode->number_of_arrivals = head_gnode_for_tail_node->number_of_branching + tail_gnode_for_head_node->number_of_branching; //number of arrivals at virtual node


  /** copy the traffic statistics of edge (tail_node, head_node) into that of (tail_node, virtual_node) */
  gnode_Gr = GetNeighborGraphNode(Gr, Gr_size, tail_node, head_node); //obtain the pointer to the graph node corresponding to vertex head_node in the edge (tail_node, head_node)

  gnode_Ga = GetNeighborGraphNode(Ga, Ga_size, tail_node, virtual_node); //obtain the pointer to the graph node corresponding to vertex virtual_node in the edge (tail_node, virtual_node)

  gnode_Ga->mean_interarrival_time = gnode_Gr->mean_interarrival_time; //average interarrival time
  gnode_Ga->last_arrival_time = gnode_Gr->last_arrival_time; //last arrival time
  gnode_Ga->sum_of_interarrival_time = gnode_Gr->sum_of_interarrival_time; //sum of interarrival time
  gnode_Ga->number_of_interarrivals = gnode_Gr->number_of_interarrivals; //number of interarrivals    
  gnode_Ga->number_of_arrivals = gnode_Gr->number_of_arrivals; //number of arrivals that is used for the denominator of the branch probability
  gnode_Ga->number_of_branching = gnode_Gr->number_of_branching; //number of branching from the tail node to this node that is a head node where the edge is <tail_node, head_node>; this is equal to number_of_interarrivals+1.


  /** copy the traffic statistics of edge (tail_node, head_node) into that of (virtual_node, head_node) */
  gnode_Ga = GetNeighborGraphNode(Ga, Ga_size, virtual_node, head_node); //obtain the pointer to the graph node corresponding to vertex head_node in the edge (virtual_node, head_node)

  gnode_Ga->mean_interarrival_time = gnode_Gr->mean_interarrival_time; //average interarrival time
  gnode_Ga->last_arrival_time = gnode_Gr->last_arrival_time; //last arrival time
  gnode_Ga->sum_of_interarrival_time = gnode_Gr->sum_of_interarrival_time; //sum of interarrival time
  gnode_Ga->number_of_interarrivals = gnode_Gr->number_of_interarrivals; //number of interarrivals    
  gnode_Ga->number_of_arrivals = gnode_Gr->number_of_arrivals; //number of arrivals that is used for the denominator of the branch probability
  gnode_Ga->number_of_branching = gnode_Gr->number_of_branching; //number of branching from the tail node to this node that is a head node where the edge is <tail_node, head_node>; this is equal to number_of_interarrivals+1.


  /** copy the traffic statistics of edge (head_node, tail_node) into that of (head_node, virtual_node) */
  gnode_Gr = GetNeighborGraphNode(Gr, Gr_size, head_node, tail_node); //obtain the pointer to the graph node corresponding to vertex tail_node in the edge (head_node, tail_node)

  gnode_Ga = GetNeighborGraphNode(Ga, Ga_size, head_node, virtual_node); //obtain the pointer to the graph node corresponding to vertex virtual_node in the edge (head_node, virtual_node)

  gnode_Ga->mean_interarrival_time = gnode_Gr->mean_interarrival_time; //average interarrival time
  gnode_Ga->last_arrival_time = gnode_Gr->last_arrival_time; //last arrival time
  gnode_Ga->sum_of_interarrival_time = gnode_Gr->sum_of_interarrival_time; //sum of interarrival time
  gnode_Ga->number_of_interarrivals = gnode_Gr->number_of_interarrivals; //number of interarrivals    
  gnode_Ga->number_of_arrivals = gnode_Gr->number_of_arrivals; //number of arrivals that is used for the denominator of the branch probability
  gnode_Ga->number_of_branching = gnode_Gr->number_of_branching; //number of branching from the tail node to this node that is a head node where the edge is <tail_node, head_node>; this is equal to number_of_interarrivals+1.


  /** copy the traffic statistics of edge (head_node, tail_node) into that (virtual_node, tail_node) */
  gnode_Ga = GetNeighborGraphNode(Ga, Ga_size, virtual_node, tail_node); //obtain the pointer to the graph node corresponding to vertex tail_node in the edge (virtual_node, tail_node)

  gnode_Ga->mean_interarrival_time = gnode_Gr->mean_interarrival_time; //average interarrival time
  gnode_Ga->last_arrival_time = gnode_Gr->last_arrival_time; //last arrival time
  gnode_Ga->sum_of_interarrival_time = gnode_Gr->sum_of_interarrival_time; //sum of interarrival time
  gnode_Ga->number_of_interarrivals = gnode_Gr->number_of_interarrivals; //number of interarrivals    
  gnode_Ga->number_of_arrivals = gnode_Gr->number_of_arrivals; //number of arrivals that is used for the denominator of the branch probability
  gnode_Ga->number_of_branching = gnode_Gr->number_of_branching; //number of branching from the tail node to this node that is a head node where the edge is <tail_node, head_node>; this is equal to number_of_interarrivals+1.
}

void Set_Forwarding_Information_For_Multiple_APs(parameter_t *param, struct_graph_node *Gr, int Gr_size, struct_graph_node **G_set, int *G_set_size, int ap_number)
{ //set the real graph Gr with the graph set G_set after processing the EDD for each access point
  struct_graph_node *tail = NULL; //pointer to a tail graph node for Gr
  struct_graph_node *head = NULL; //pointer to a head graph node for Gr
  struct_graph_node *ptr = NULL; //pointer to a graph node corresponding to a head node of a graph in G_set
  int i = 0, j = 0; //indices for for-loops
  int neighbor_num = 0; //number of neighbors

  for(i = 0; i < Gr_size; i++) //for-1
  {
    tail = &(Gr[i]);
    head = tail;
    neighbor_num = (int)tail->weight;
    
    for(j = 0; j < neighbor_num; j++) //for-2
    {
      head = head->next;
      
      /* get the pointer to the head node with a minimum EDD among the graphs in the graph set G_set given an edge (tail->vertex, head->vertex) */
      ptr = Get_Minimum_EDD_GraphNode(tail->vertex, head->vertex, G_set, G_set_size, ap_number);
      if(ptr == NULL)
      {
	printf("Set_Forwarding_Infomation_For_Multiple_APs(): Error: Get_Minimum_EDD_GraphNode() returns NULL\n");
	exit(1);
      }

      /* set the forwarding information for the edge (tail->vertex, head->vertex) in Gr */
      head->EDD = ptr->EDD;
      head->EDD_VAR = ptr->EDD_VAR;
      head->EDD_SD = ptr->EDD_SD;
      head->edge_delay = ptr->edge_delay;
      head->edge_delay_variance = ptr->edge_delay_variance;
      head->edge_delay_standard_deviation = ptr->edge_delay_standard_deviation;

      /* [10/26/09] copy the forwarding probability information */
      head->CP = ptr->CP;
      head->theta = ptr->theta;
      head->Q = ptr->Q;
      head->P_prime = ptr->P_prime;
      head->P_prime_pure = ptr->P_prime_pure;
      head->P = ptr->P;
      head->P_pure = ptr->P_pure;

      /* [10/26/09] copy the conditional forwarding probability queue */
      CopyConditionalForwardingProbabilityQueueNodes_For_DirectionalEdge(ptr->conditional_forwarding_probability_queue, head->conditional_forwarding_probability_queue);

    } //end of for-2
  } //end of for-1

  /** compute the forwarding probability based on the EDD for each edge */
  VADD_Recompute_Forwarding_Probability(param, Gr, Gr_size); //compute forwarding probability P per road segment for multiple APs
}

struct_graph_node* Get_Minimum_EDD_GraphNode(char *tail_vertex, char *head_vertex, struct_graph_node **G_set, int *G_set_size, int ap_number)
{ //get the pointer to the head node with a minimum EDD among the graphs in the graph set G_set given an edge (tail->vertex, head->vertex)
  struct_graph_node *ptr = NULL; //pointer to a head graph node
  struct_graph_node *min_EDD_gnode = NULL; //pointer to a head graph node with a minimum EDD
  double min_EDD = INF; //minimum EDD
  int i = 0; //index for for-loop

  for(i = 0; i < ap_number; i++)
  {
    ptr = GetNeighborGraphNode(G_set[i], G_set_size[i], tail_vertex, head_vertex);
    //return the pointer to graph node corresponding to head_vertex that is tail_vertex's neighbor in the edge (tail_vertex, head_vertex)

    /* set the minimum_EDD and the corresponding minimum_EDD_gnode */
    if(ptr->EDD < min_EDD)
    {
      min_EDD = ptr->EDD;
      min_EDD_gnode = ptr;
    }
  }

  /* check whether min_EDD is infinite or not; if so, return the pointer to the graph node in Gr_set[0] as default */
  if(min_EDD == INF)
    min_EDD_gnode = GetNeighborGraphNode(G_set[0], G_set_size[0], tail_vertex, head_vertex);

  return min_EDD_gnode;
}

void SetTargetPoint_In_TafficTable(struct_traffic_table *table, char *target_point)
{ //set a target point in traffice table

  /* release the memory occupied by the traffic table */
  Free_Traffic_Table(table);

  /* add target_point to table */
  AddTrafficTableEntry(table, target_point);
}

int GetTargetPoint_For_Multiple_APs(parameter_t *param, double current_time, access_point_queue_t *APQ, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, double *EDD_p, double *EAD_p, access_point_queue_node_t **transmitter_AP_qnode)
{ //In the case where multiple APs exist, this function returns the pointer to the AP queue node corresponding to the packet-transmitter AP in order to let the AP with the shortest delivery delay transmit this packet towards the destination vehicle
    int target_point_id = 0; //target point id

    //char *AP_vertex = NULL; //AP vertex
    //int size = APQ->size; //the size of the AP queue APQ
    //access_point_queue_node_t *pQueueNode = NULL; //pointer to the AP queue node 

    /* AP selection methods according to the forwarding schemes
       1. TSF: select the AP with the minimum EVD (Expected Vehicle Delay).
       2. RTP (Random Trajectory Point): select the AP that has the shortest delivery delay to the random target point
       3. LTP (Last Trajectory Point): select the AP that has the shortest delivery delay to the last trajectory point of the destination vehicle; this allows the packet to arrive at the last trajectory point earlier than the destination vehicle.
    */

    /* determine the target point (i.e., intersection id) towards which this packet is sent */
    if((param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_STATIC_FORWARDING) || (param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_PARTIALLY_DYNAMIC_FORWARDING) || (param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_FULLY_DYNAMIC_FORWARDING))
    {
        target_point_id = GetTargetPoint_By_PacketTrajectory_For_Multiple_APs(param, current_time, APQ, destination_vehicle, packet, FTQ, EDD_p, EAD_p, transmitter_AP_qnode);
    }
    else if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_END_INTERSECTION)
    {
        target_point_id = GetTargetPoint_By_EndIntersection_For_Multiple_APs(param, current_time, APQ, destination_vehicle, packet, FTQ, EDD_p, EAD_p, transmitter_AP_qnode);
    }
    else if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_RANDOM_INTERSECTION)
    {
        target_point_id = GetTargetPoint_By_RandomIntersection_For_Multiple_APs(param, current_time, APQ, destination_vehicle, packet, FTQ, EDD_p, EAD_p, transmitter_AP_qnode);
    }
    else
    {
        printf("GetTargetPoint_For_Multiple_APs(): Error: param->vehicle_vanet_target_point_selection_type(%d) is not supported yet!\n", param->vehicle_vanet_target_point_selection_type);
        exit(1);
    }

    /** check the validity of target point id */
    if(target_point_id == 0)
    {
        printf("GetTargetPoint_For_Multiple_APs(): for time=%.2f, target_point_id cannot be 0\n", (float) current_time, target_point_id );
        exit(1);
    }

    return target_point_id;    
}

int GetTargetPoint_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, double *EDD_p, double *EAD_p)
{ // get the intersection id of a target point for AP where the target point is on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle
  int target_point_id = 0; //target point id

  /* determine the target point (i.e., intersection id) towards which this packet is sent */
  if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_OPTIMAL_INTERSECTION)
  {
    target_point_id = GetTargetPoint_By_OptimalIntersection_For_AP(param, current_time, AP_vertex, destination_vehicle, FTQ, EDD_p, EAD_p);
  }
  else if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_EARLIEST_ARRIVING_INTERSECTION)
  {
    target_point_id = GetTargetPoint_By_PacketEarliestArrivingIntersection_For_AP(param, current_time, AP_vertex, destination_vehicle, FTQ, EDD_p, EAD_p);
  }
  else if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_HEADING_INTERSECTION)
  {
    target_point_id = GetTargetPoint_By_HeadingIntersection_For_AP(param, current_time, AP_vertex, destination_vehicle, FTQ, EDD_p, EAD_p);
  }
  else if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_END_INTERSECTION)
  {
    target_point_id = GetTargetPoint_By_EndIntersection_For_AP(param, current_time, AP_vertex, destination_vehicle, packet, FTQ, EDD_p, EAD_p);
  }
  else if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_RANDOM_INTERSECTION)
  {
    target_point_id = GetTargetPoint_By_RandomIntersection_For_AP(param, current_time, AP_vertex, destination_vehicle, packet, FTQ, EDD_p, EAD_p);
  }
  else if((param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_STATIC_FORWARDING) || (param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_PARTIALLY_DYNAMIC_FORWARDING) || (param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_FULLY_DYNAMIC_FORWARDING))
  {
    target_point_id = GetTargetPoint_By_PacketTrajectory_For_AP(param, current_time, AP_vertex, destination_vehicle, packet, FTQ, EDD_p, EAD_p);
  }
  else
  {
    printf("GetTargetPoint_For_AP(): Error: param->vehicle_vanet_target_point_selection_type(%d) is not supported yet!\n", param->vehicle_vanet_target_point_selection_type);
    exit(1);
  }

  /** check the validity of target point id */
  if(target_point_id == 0)
  {
      printf("GetTargetPoint_For_AP(): for time=%.2f, target_point_id cannot be 0.\n", (float) current_time);
      exit(1);
  }

  return target_point_id;
}

int GetMultipleTargetPoints_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, target_point_queue_t *TPQ)
{ // get the list of intersection ids of target points for AP where each target point is on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle
  int target_point_number = 0; //the number of target points

  /* initialize TPQ by deleting all queue nodes in TPQ */
  if(TPQ->size > 0)
  {
	DestroyQueue((queue_t*)TPQ);
  }

  /* determine the list of target points (i.e., intersection ids) towards which the copies of this packet are sent */
  if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_RANDOM_INTERSECTION)
  {
    target_point_number = GetMultipleTargetPoints_By_RandomIntersection_For_AP(param, current_time, AP_vertex, destination_vehicle, packet, FTQ, TPQ);
  }
  else if((param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_STATIC_FORWARDING) || (param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_PARTIALLY_DYNAMIC_FORWARDING) || (param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_FULLY_DYNAMIC_FORWARDING))
  {
    target_point_number = GetMultipleTargetPoints_By_PacketTrajectory_For_AP(param, current_time, AP_vertex, destination_vehicle, packet, FTQ, TPQ);
  }
  else
  {
    printf("GetMultipleTargetPoints_For_AP(): Error: param->vehicle_vanet_target_point_selection_type(%d) is not supported yet!\n", param->vehicle_vanet_target_point_selection_type);
    exit(1);
  }

  /** check the validity of target_point_number */
  if(target_point_number == 0)
  {
      printf("GetMultipleTargetPoints_For_AP(): for time=%.2f, there is no set of target points to satisfy the user-required delivery probability (%.2f).\n", (float) current_time, param->communication_packet_delivery_probability_threshold);
      exit(1);
  }

  return target_point_number;
}

int GetTargetPoint_For_Carrier(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p, double *EAD_p)
{ // get the intersection id of a target point for carrier vehicle where the target point is on the destination vehicle trajectory in the carrier's packet
  int target_point_id = 0; //target point id

  /* determine the target point (i.e., intersection id) towards which this packet is sent */
  if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_OPTIMAL_INTERSECTION)
  {
    target_point_id = GetTargetPoint_By_OptimalIntersection_For_Carrier(param, current_time, carrier_vehicle, FTQ, EDD_p, EAD_p);
  }
  else if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_EARLIEST_ARRIVING_INTERSECTION)
  {
    target_point_id = GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier(param, current_time, carrier_vehicle, FTQ, EDD_p, EAD_p);
  }
  else if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_HEADING_INTERSECTION)
  {
    target_point_id = GetTargetPoint_By_HeadingIntersection_For_Carrier(param, current_time, carrier_vehicle, FTQ, EDD_p, EAD_p);
  }
  else if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_RANDOM_INTERSECTION)
  {
    target_point_id = GetTargetPoint_By_RandomIntersection_For_Carrier(param, current_time, carrier_vehicle, FTQ, EDD_p, EAD_p);
  }
  else if((param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_STATIC_FORWARDING) || (param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_PARTIALLY_DYNAMIC_FORWARDING) || (param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_FULLY_DYNAMIC_FORWARDING))
  {
    target_point_id = GetTargetPoint_By_PacketTrajectory_For_Carrier(param, current_time, carrier_vehicle, FTQ, EDD_p, EAD_p);
  }
  else
  {
    printf("GetTargetPoint_For_Carrier(): Error: param->vehicle_vanet_target_point_selection_type(%d) is not supported yet!\n", param->vehicle_vanet_target_point_selection_type);
    exit(1);
  }

  /** check the validity of target point id */
  if(target_point_id == 0)
  {
      printf("GetTargetPoint_For_Carrier(): for time=%.2f, target_point_id cannot be 0\n", (float) current_time, target_point_id );
      exit(1);
  }

  return target_point_id;
}

int GetTargetPoint_For_Carrier_At_TrajectoryIntersection_Before_DestinationVehicle(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, vehicle_trajectory_queue_t *pTrajectory_Queue, vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode, int destination_vehicle_hop, double destination_vehicle_offset, int vertex_hop, double *EDD_p, double *EAD_p)
{ // get the intersection id of a target point for carrier vehicle that is within the communication range of an intersection on the destination vehicle trajectory where the intersection is before the destination vehicle on the trajectory
  int target_point_id = 0; //target point id

  /* determine the target point (i.e., intersection id) towards which this packet is sent */
  if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_OPTIMAL_INTERSECTION)
  {
    target_point_id = GetTargetPoint_By_OptimalIntersection_For_Carrier_At_TrajectoryIntersection_Before_DestinationVehicle(param, current_time, carrier_vehicle, FTQ, pTrajectory_Queue, pCurrent_Trajectory_QNode, destination_vehicle_hop, destination_vehicle_offset, vertex_hop, EDD_p, EAD_p);
  }
  else if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_HEADING_INTERSECTION)
  {
    target_point_id = GetTargetPoint_By_HeadingIntersection_For_Carrier_At_TrajectoryIntersection_Before_DestinationVehicle(param, current_time, carrier_vehicle, FTQ, pTrajectory_Queue, pCurrent_Trajectory_QNode, destination_vehicle_hop, destination_vehicle_offset, vertex_hop, EDD_p, EAD_p);
  }
  else if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_RANDOM_INTERSECTION)
  {
    target_point_id = GetTargetPoint_By_RandomIntersection_For_Carrier_At_TrajectoryIntersection_Before_DestinationVehicle(param, current_time, carrier_vehicle, FTQ, pTrajectory_Queue, pCurrent_Trajectory_QNode, destination_vehicle_hop, destination_vehicle_offset, vertex_hop, EDD_p, EAD_p);
  }
  else
  {
    printf("GetTargetPoint_For_Carrier_At_TrajectoryIntersection_Before_DestinationVehicle(): Error: param->vehicle_vanet_target_point_selection_type(%d) is not supported yet!\n", param->vehicle_vanet_target_point_selection_type);
    exit(1);
  }

  return target_point_id;
}

int GetTargetPoint_For_StationaryNode(parameter_t *param, double current_time, stationary_node_queue_node_t *stationary_node, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ //get the intersection id of a target point for the stationary node where the target point is on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle

  forwarding_table_queue_t *FTQ = param->vanet_table.FTQ; //forwarding table queue
  static int vehicle_trajectory_expiration_count = 0; //count for vehicle trajectory expiration
  int remaining_path_hop_count = 0; //the number of nodes consisting of the path corresponding to the vehicle trajectory from the head node of its current edge to the destination node (i.e., destination position).
  int path_node_number = 0; //the number of nodes on the path to check for an optimal target point, including the tail node of its current edge
  int path_current_hop = 0; //destination vehicle's current hop at this current time

  char *target_point = NULL; //pointer to a vertex corresponding to a target point candidate
  double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
  double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

  double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
  double path_travel_time = 0; //path travel time
  double path_travel_time_deviation = 0; //path travel time deviation

  double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
  double EAD_VAR_p = 0; //EAD_VAR_p is the variance of the destination vehicle's arrival delay to target point p
  double EAD_SD_p = 0; //EAD_SD_p is the standard deviation of the destination vehicle's arrival delay to target point p

  double value = 0 ;//value used for selecting an optimal intersection based on (EDD_p, EDD_SD_p) and (EAD_p, EAD_SD_p)
  double min_value = INF; //minimum value for selecting an optimal intersection
  char min_value_target_point[NAME_SIZE] = ""; //target point with min_value

  int i = 0; //for-loop index
  char current_edge_tail_node[NAME_SIZE] = ""; //tail node of the current edge pointed by vehicle->path_ptr

  /**********************************************************************/
  int target_point_id = 0; //target point id that is an intersection
  int new_target_point_id = 0; //new target point id for the packet with the expired vehicle trajectory such that the destination vehicle has already gone out of the trajectory in the packet.

  /**********************************************************************/
  struct_vehicle_t *destination_vehicle = NULL; //pointer to the destination vehicle for packets
  vehicle_trajectory_queue_t *pTrajectory_Queue = NULL; //pointer to the packet's vehicle trajectory
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to the vehicle trajectory queue node
  vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode = NULL; //pointer to the current trajectory queue node for the edge where the destination vehicle is moving
  double offset_end = 0; //destination vehicle's offset in the current edge on its trajectory for current_time

  double vehicle_speed_variance = 0; //vehicle speed variance
  char stationary_node_vertex[NAME_SIZE]; //vertex name for stationary node

  double max_constraint_value = 0; //maximum constraint value to select a target point in the case where there exists no target point to satisfy the required constraint
  int max_constraint_value_target_point_id = 0; //target point corresponding to the maximum constraint value

  double packet_ttl = 0; //packet's TTL
  /**********************************************************************/

  /** check whether this stationary node has packets or not; if there is no packet, return the stationary node's latest target point */
  if(stationary_node->packet_queue.size == 0)
  {
    new_target_point_id = stationary_node->target_point_id;
    return new_target_point_id;
  }

  /** check whether this stationary node has a valid pointer to the latest packet or not */
  if(stationary_node->latest_packet_ptr == NULL)
  {
    printf("GetTargetPoint_For_StationaryNode(): Error: for time=%.2f, stationary_node->latest_packet_ptr is NULL\n", (float)current_time);
    exit(1);
  }

  /** pointer to the destination vehicle for packets that is used to check whether the trajectory reflects the destination's movement well or not */
  destination_vehicle = stationary_node->latest_packet_ptr->dst_vnode; 

  /** set up vehicle trajectory queue pTrajectory_Queue */
  pTrajectory_Queue = &(stationary_node->latest_packet_ptr->vehicle_trajectory); //pointer to the packet's vehicle trajectory

  /** set packet_ttl to the latest packet's TTL */
  packet_ttl = stationary_node->latest_packet_ptr->ttl;

  /** find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory for this stationary node */
  pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_StationaryNode(param, current_time, stationary_node, &path_current_hop, &offset_end);

  /** check whether the destination vehicle has passed out of its trajectory or not */
  if((param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE) && (pCurrent_Trajectory_QNode == NULL))
  { //choose the stationary node's latest target point. If this packet meets another vehicle with fresh destination vehicle, it is forwarded with the packet with the fresh destination vehicle trajectory.

    vehicle_trajectory_expiration_count++;

    new_target_point_id = stationary_node->target_point_id;
    
    return new_target_point_id;
  }

  /** compute the number of path nodes on the trajectory to check for a target point */
  if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE)
  {
    path_node_number = pTrajectory_Queue->size  - path_current_hop; //the number of nodes on the path to check for an optimap target point, including the tail node of its current edge
  }
  else if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
  {
    path_node_number = pTrajectory_Queue->size; //the number of nodes on the path to check for an optimap target point, including the tail node of its current edge

  }
  else
  {
    printf("GetTargetPoint_For_StationaryNode(): trajectory_length_type(%d) is unknown!\n", param->vehicle_vanet_vehicle_trajectory_length_type);
    exit(1);    
  }

  /** search an optimal target point p such that |EDD_p - EAD_p| is minimum. **/

  /* initialize double *EDD_p_for_optimal_target_point and double *EDD_p_for_optimal_target_point */
  *EDD_p_for_optimal_target_point = 0;
  *EAD_p_for_optimal_target_point = 0;

  /* copy the tail node of the destination vehicle's current edge to current_edge_tail_node */
  strcpy(current_edge_tail_node, pCurrent_Trajectory_QNode->graph_pos.enode->tail_node);

  /* set pTrajectory_QNode to the first queue node to check */
  pTrajectory_QNode = pCurrent_Trajectory_QNode; //Note that the first point to check is the tail node of the current edge


  for(i = 0; i < path_node_number; i++)
  {
    /* For infinite trajectory length type, if pTrajectory_QNode is the queue head, then let pTrajectory_QNode point to head->next */
    if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
    {
      if(pTrajectory_QNode == &(pTrajectory_Queue->head))
        pTrajectory_QNode = pTrajectory_QNode->next;
    }

    target_point =  pTrajectory_QNode->graph_pos.enode->tail_node; //target_point is the tail node of an edge

    /* compute the path distance from the destination vehicle's current position to the target point */
    //path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory(current_time, pTrajectory_Queue, pCurrent_Trajectory_QNode, offset_end, target_point);

    ///* compute the travel time duration */
    //EAD_p = path_distance / param->vehicle_speed; //param->vehicle_speed should be used since it is average vehicle speed
    //EAD_p = path_distance / pTrajectory_Queue->vehicle_speed;

    /* compute the standard deviation of the travel time duration */
    //vehicle_speed_variance = pow(param->vehicle_speed_standard_deviation, 2);
    //vehicle_speed_variance = pow(pTrajectory_Queue->vehicle_speed_standard_deviation, 2);
    //EAD_VAR_p = (i+1)*vehicle_speed_variance;
    //EAD_SD_p = sqrt(EAD_VAR_p);

    /* compute the path distance, the path travel time, and the path travel time deviation from the destination vehicle's current position to the target point */
    path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory_Along_With_PathTravelTime_And_Deviation(param, current_time, pTrajectory_Queue, pCurrent_Trajectory_QNode, offset_end, target_point, &path_travel_time, &path_travel_time_deviation);

    /* compute the mean and standard deviation of the travel time duration 
       @Note: make sure that EAD_p is non-zero for the Gamma distribution */
    if(path_travel_time < 0)
    {
        pTrajectory_QNode = pTrajectory_QNode->next; //pTrajectory_QNode points to the next node on the trajectory

        continue; //In this case, the destination vehicle has already passed this target point
    }
    else
    {
        EAD_p = path_travel_time; //param->vehicle_speed should be used since it is average vehicle speed
        EAD_SD_p = path_travel_time_deviation;
    }   

    /** compute EDD_p for a target point p */    
    /* compute the EDD and EDD_SD for the target point at the intersection having this AP */
    target_point_id = atoi(target_point);

    itoa(stationary_node->intersection_id, stationary_node_vertex);
    VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(param, target_point_id, stationary_node_vertex, FTQ, &EDD_p, &EDD_SD_p);

    /** Filtering: check the difference between EDD_p and EAD_p with ten times of EAD_p */
    if(EDD_p > EAD_p)
    //if(EDD_p >= EAD_p*10)
    {
        pTrajectory_QNode = pTrajectory_QNode->next; //pTrajectory_QNode points to the next node on the trajectory

        continue; //In this case, the destination vehicle has already passed this target point
    }

    /** Compute the probability that the packet will arrive at the target point earlier than the destination vehicle */
    /* Note that the probability is P[T_i^p <= T_i^v] = \int_{0}^{\infy} \int_{0}^{y} {f(x)g(y)}\,dx\,dy */

    /* update min_value and min_value_target_point */ 
    //value = fabs(EDD_p - EAD_p);
    value = Compute_TargetPoint_OptimizationValue(param, target_point_id, EDD_p, EDD_SD_p, EAD_p, EAD_SD_p, packet_ttl, &max_constraint_value, &max_constraint_value_target_point_id); //compute the optimization value for an intersection i that is an optimal target point candidate given the the packet delivery delay distribution (EDD_p, EDD_SD_p) and the destination vehicle arrival delay distribution (EAD_p, EAD_SD_p).

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_STATIONARY_NODE__
    printf("GetTargetPoint_For_StationaryNode(): target point=%s, value=%.2f\n", target_point, value);
#endif 

    if(min_value > value)
    {
      min_value = value;
      strcpy(min_value_target_point, target_point);
      *EDD_p_for_optimal_target_point = EDD_p;
      *EAD_p_for_optimal_target_point = EAD_p;
    }

    pTrajectory_QNode = pTrajectory_QNode->next; //pTrajectory_QNode points to the next node on the trajectory
  }

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_STATIONARY_NODE__
  printf("GetTargetPoint_For_StationaryNode(): optimal target point=%s, min_value=%.2f\n", min_value_target_point, min_value);
  fgetc(stdin);
#endif

  /* convert min_value_target_point into target_point_id */
  if(min_value == INF)
  { //the case where there is no intersection to satisfy the constraint: we choose the last point of the vehicle trajectory
      if(max_constraint_value > 0)
          target_point_id = max_constraint_value_target_point_id;
      else
          target_point_id = atoi(pTrajectory_Queue->head.prev->graph_pos.enode->head_node); //the end of the vehicle trajectory
  }
  else
  {
      target_point_id = atoi(min_value_target_point);
  }

  return target_point_id;
}

int GetTargetPoint_For_Packet(parameter_t *param, double current_time, packet_queue_node_t *packet, int current_intersection_id, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ //for the current intersection holding the packet, get the intersection id of a target point for the packet where the target point is on the packet's vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle

  forwarding_table_queue_t *FTQ = param->vanet_table.FTQ; //forwarding table queue
  static int vehicle_trajectory_expiration_count = 0; //count for vehicle trajectory expiration
  int remaining_path_hop_count = 0; //the number of nodes consisting of the path corresponding to the vehicle trajectory from the head node of its current edge to the destination node (i.e., destination position).
  int path_node_number = 0; //the number of nodes on the path to check for an optimal target point, including the tail node of its current edge
  int path_current_hop = 0; //destination vehicle's current hop at this current time

  char *target_point = NULL; //pointer to a vertex corresponding to a target point candidate
  double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
  double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

  double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
  double path_travel_time = 0; //path travel time
  double path_travel_time_deviation = 0; //path travel time deviation

  double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
  double EAD_VAR_p = 0; //EAD_VAR_p is the variance of the destination vehicle's arrival delay to target point p
  double EAD_SD_p = 0; //EAD_SD_p is the standard deviation of the destination vehicle's arrival delay to target point p

  double value = 0 ;//value used for selecting an optimal intersection based on (EDD_p, EDD_SD_p) and (EAD_p, EAD_SD_p)
  double min_value = INF; //minimum value for selecting an optimal intersection
  char min_value_target_point[NAME_SIZE] = ""; //target point with min_value

  int i = 0; //for-loop index
  char current_edge_tail_node[NAME_SIZE] = ""; //tail node of the current edge pointed by vehicle->path_ptr

  /**********************************************************************/
  int target_point_id = 0; //target point id that is an intersection
  int new_target_point_id = 0; //new target point id for the packet with the expired vehicle trajectory such that the destination vehicle has already gone out of the trajectory in the packet.

  /**********************************************************************/
  struct_vehicle_t *destination_vehicle = NULL; //pointer to the destination vehicle for packets
  vehicle_trajectory_queue_t *pTrajectory_Queue = NULL; //pointer to the packet's vehicle trajectory
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to the vehicle trajectory queue node
  vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode = NULL; //pointer to the current trajectory queue node for the edge where the destination vehicle is moving
  double offset_end = 0; //destination vehicle's offset in the current edge on its trajectory for current_time

  double vehicle_speed_variance = 0; //vehicle speed variance
  char current_intersection_vertex[NAME_SIZE]; //vertex name for the current intersection

  double max_constraint_value = 0; //maximum constraint value to select a target point in the case where there exists no target point to satisfy the required constraint
  int max_constraint_value_target_point_id = 0; //target point corresponding to the maximum constraint value

  double packet_ttl = packet->ttl; //packet's TTL
  /**********************************************************************/

  /** pointer to the destination vehicle for the packet that is used to check whether the trajectory reflects the destination's movement well or not */
  destination_vehicle = packet->dst_vnode; 

  /** set up vehicle trajectory queue pTrajectory_Queue */
  pTrajectory_Queue = &(packet->vehicle_trajectory); //pointer to the packet's vehicle trajectory

  /** find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory for this stationary node */
  pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_Packet(param, current_time, packet, &path_current_hop, &offset_end);

  /** check whether the destination vehicle has passed out of its trajectory or not */
  if((param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE) && (pCurrent_Trajectory_QNode == NULL))
  { //choose the packet's target point as a new target point. If this packet meets another vehicle with fresh destination vehicle, it is forwarded with the packet with the fresh destination vehicle trajectory.

    vehicle_trajectory_expiration_count++;

    new_target_point_id = packet->target_point_id;
    
    return new_target_point_id;
  }

  /** compute the number of path nodes on the trajectory to check for a target point */
  if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE)
  {
    path_node_number = pTrajectory_Queue->size  - path_current_hop; //the number of nodes on the path to check for an optimap target point, including the tail node of its current edge
  }
  else if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
  {
    path_node_number = pTrajectory_Queue->size; //the number of nodes on the path to check for an optimap target point, including the tail node of its current edge

  }
  else
  {
    printf("GetTargetPoint_For_Packet(): trajectory_length_type(%d) is unknown!\n", param->vehicle_vanet_vehicle_trajectory_length_type);
    exit(1);    
  }

  /** search an optimal target point p such that |EDD_p - EAD_p| is minimum. **/

  /* initialize double *EDD_p_for_optimal_target_point and double *EDD_p_for_optimal_target_point */
  *EDD_p_for_optimal_target_point = 0;
  *EAD_p_for_optimal_target_point = 0;

  /* copy the tail node of the destination vehicle's current edge to current_edge_tail_node */
  strcpy(current_edge_tail_node, pCurrent_Trajectory_QNode->graph_pos.enode->tail_node);

  /* set pTrajectory_QNode to the first queue node to check */
  pTrajectory_QNode = pCurrent_Trajectory_QNode; //Note that the first point to check is the tail node of the current edge


  for(i = 0; i < path_node_number; i++)
  {
    /* For infinite trajectory length type, if pTrajectory_QNode is the queue head, then let pTrajectory_QNode point to head->next */
    if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
    {
      if(pTrajectory_QNode == &(pTrajectory_Queue->head))
        pTrajectory_QNode = pTrajectory_QNode->next;
    }

    target_point =  pTrajectory_QNode->graph_pos.enode->tail_node; //target_point is the tail node of an edge

    /* compute the path distance from the destination vehicle's current position to the target point */
    //path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory(current_time, pTrajectory_Queue, pCurrent_Trajectory_QNode, offset_end, target_point);

    ///* compute the travel time duration */
    //EAD_p = path_distance / param->vehicle_speed; //param->vehicle_speed should be used since it is average vehicle speed
    //EAD_p = path_distance / pTrajectory_Queue->vehicle_speed;

    /* compute the standard deviation of the travel time duration */
    //vehicle_speed_variance = pow(param->vehicle_speed_standard_deviation, 2);
    //vehicle_speed_variance = pow(pTrajectory_Queue->vehicle_speed_standard_deviation, 2);
    //EAD_VAR_p = (i+1)*vehicle_speed_variance;
    //EAD_SD_p = sqrt(EAD_VAR_p);

    /* compute the path distance, the path travel time, and the path travel time deviation from the destination vehicle's current position to the target point */
    path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory_Along_With_PathTravelTime_And_Deviation(param, current_time, pTrajectory_Queue, pCurrent_Trajectory_QNode, offset_end, target_point, &path_travel_time, &path_travel_time_deviation);

    /* compute the mean and standard deviation of the travel time duration 
       @Note: make sure that EAD_p is non-zero for the Gamma distribution */
    if(path_travel_time < 0)
    {
        pTrajectory_QNode = pTrajectory_QNode->next; //pTrajectory_QNode points to the next node on the trajectory

        continue; //In this case, the destination vehicle has already passed this target point
    }
    else
    {
        EAD_p = path_travel_time; //param->vehicle_speed should be used since it is average vehicle speed
        EAD_SD_p = path_travel_time_deviation;
    }

    ///* compute the mean and standard deviation of the travel time duration */
    //EAD_p = path_travel_time; //param->vehicle_speed should be used since it is average vehicle speed
    //EAD_SD_p = path_travel_time_deviation;

    /** compute EDD_p for a target point p */    
    /* compute the EDD and EDD_SD for the target point at the intersection having this AP */
    target_point_id = atoi(target_point);

    itoa(current_intersection_id, current_intersection_vertex);
    VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(param, target_point_id, current_intersection_vertex, FTQ, &EDD_p, &EDD_SD_p);

    /** Filtering: check the difference between EDD_p and EAD_p with ten times of EAD_p */
    if(EDD_p > EAD_p)
    //if(EDD_p >= EAD_p*10)
    {
        pTrajectory_QNode = pTrajectory_QNode->next; //pTrajectory_QNode points to the next node on the trajectory

        continue; //In this case, the destination vehicle has already passed this target point
    }

    /** Compute the probability that the packet will arrive at the target point earlier than the destination vehicle */
    /* Note that the probability is P[T_i^p <= T_i^v] = \int_{0}^{\infy} \int_{0}^{y} {f(x)g(y)}\,dx\,dy */

    /* update min_value and min_value_target_point */ 
    //value = fabs(EDD_p - EAD_p);
    value = Compute_TargetPoint_OptimizationValue(param, target_point_id, EDD_p, EDD_SD_p, EAD_p, EAD_SD_p, packet_ttl, &max_constraint_value, &max_constraint_value_target_point_id); //compute the optimization value for an intersection i that is an optimal target point candidate given the the packet delivery delay distribution (EDD_p, EDD_SD_p) and the destination vehicle arrival delay distribution (EAD_p, EAD_SD_p).

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_STATIONARY_NODE__
    printf("GetTargetPoint_For_Packet(): target point=%s, value=%.2f\n", target_point, value);
#endif 

    if(min_value > value)
    {
      min_value = value;
      strcpy(min_value_target_point, target_point);
      *EDD_p_for_optimal_target_point = EDD_p;
      *EAD_p_for_optimal_target_point = EAD_p;
    }

    pTrajectory_QNode = pTrajectory_QNode->next; //pTrajectory_QNode points to the next node on the trajectory
  }

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_STATIONARY_NODE__
  printf("GetTargetPoint_For_Packet(): optimal target point=%s, min_value=%.2f\n", min_value_target_point, min_value);
  fgetc(stdin);
#endif

  /* convert min_value_target_point into target_point_id */
  if(min_value == INF)
  { //the case where there is no intersection to satisfy the constraint: we choose the last point of the vehicle trajectory
      if(max_constraint_value > 0)
          target_point_id = max_constraint_value_target_point_id;
      else
          target_point_id = atoi(pTrajectory_Queue->head.prev->graph_pos.enode->head_node); //the end of the vehicle trajectory
  }
  else
  {
      target_point_id = atoi(min_value_target_point);
  }

  return target_point_id;
}

int GetTargetPoint_By_HeadingIntersection_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ // get the id of a target point for AP, such as intersection id on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle

  int target_point_id = 0;

#ifdef __DEBUG_LEVEL_AP_PACKET_ARRIVE__
  char *target_point = destination_vehicle->current_pos_in_digraph.enode->head_node; //target point
  double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
  double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

  double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
  double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
#endif


  /** determine the target point as the destination vehicle's heading intersection */
  target_point_id = atoi(destination_vehicle->current_pos_in_digraph.enode->head_node);

#ifdef __DEBUG_LEVEL_AP_PACKET_ARRIVE__

  /** compute the travel time duration */
  path_distance = destination_vehicle->edge_length - destination_vehicle->current_pos_in_digraph.offset; //path distance from vehicle's current position to the head node of the vehicle's current edge
  EAD_p = path_distance / destination_vehicle->speed;

  /** get the EDD and EDD_SD for the target point at the intersection having this AP */
  VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(param, target_point_id, AP_vertex, FTQ, &EDD_p, &EDD_SD_p);

  /** set EDD_p_for_optimal_target_point and EAD_p_for_optimal_target_point to EDD_p and EAD_p */
  *EDD_p_for_optimal_target_point = EDD_p;
  *EAD_p_for_optimal_target_point = EAD_p;
#endif

  return target_point_id;
}

int GetTargetPoint_By_EndIntersection_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ // get the id of a target point for AP for the end intersection of destination vehicle's trajectory
  int AP_vertex_id = atoi(AP_vertex); //AP's vertex id, i.e., intersection id
  int id = 0; //intersection id
  int i = 0; //index

  vehicle_trajectory_queue_t *Q = &(packet->vehicle_trajectory); //pointer to the vehicle trajectory queue
  vehicle_trajectory_queue_node_t *pQueueNode = NULL; //pointer to a vehicle trajectory node

  char *target_point = NULL; //the end intersection of the vehicle trajectory is selected as target point
  int target_point_id = -1;
  int trajectory_size = Q->size; //the size of the vehicle trajectory

#ifdef __DEBUG_LEVEL_AP_PACKET_ARRIVE__
  double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
  double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

  double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
  double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
#endif

  /** check the validity of the vehicle trajectory */
  if(trajectory_size == 0)
  {
      printf("GetTargetPoint_By_EndIntersection_For_AP(): Error: the size of vehicle trajectory is zero!\n");
      exit(1);
  }

	/** determine the target point as the destination vehicle's end intersection on its vehicle trajectory */
	if(param->communication_multiple_SN_flag == TRUE)
	{ /* the case where the partial deployment of stationary nodes is performed */
		pQueueNode = &(Q->head);
		for(i = 0; i < Q->size; i++)
		{
			pQueueNode = pQueueNode->prev;
			target_point = pQueueNode->graph_pos.enode->head_node;
			id = atoi(target_point);
		
			/* check the validity of id */
			if(id <= 0 || id > param->vanet_table.Gr_size)
			{
				printf("%s:%d GetTargetPoint_By_EndIntersection_For_AP(): id(=%d) is not valid\n", 
						__FUNCTION__, __LINE__, id);
				exit(1);
			}

			/* check whether the intersection of id has a stationary node and the delay is finite */
			if((param->vanet_table.Gr[id-1].stationary_node_flag == TRUE) && (param->vanet_table.Dr_edd[AP_vertex_id-1][id-1] < INF))
			{
				target_point_id = id;
				break;
			}
		}

		/* check whether target_point_id is valid or not */
		if(target_point_id == -1)
		{
			printf("%s:%d GetTargetPoint_By_EndIntersection_For_AP(): there is no valid target point\n", __FUNCTION__, __LINE__);
			exit(1);
		}
	}
	else
	{ /* the case where the full deployment of stationary nodes is performed */
		target_point = packet->vehicle_trajectory.head.prev->graph_pos.enode->head_node; //the end intersection of the vehicle trajectory is selected as target point

		target_point_id = atoi(target_point);
	}

#ifdef __DEBUG_LEVEL_AP_PACKET_ARRIVE__

  /** compute the travel time duration */
  //path_distance = destination_vehicle->edge_length - destination_vehicle->current_pos_in_digraph.offset; //path distance from vehicle's current position to the head node of the vehicle's current edge
  //EAD_p = path_distance / destination_vehicle->speed;

  /** get the EDD and EDD_SD for the target point at the intersection having this AP */
  VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(param, target_point_id, AP_vertex, FTQ, &EDD_p, &EDD_SD_p);

  /** set EDD_p_for_optimal_target_point and EAD_p_for_optimal_target_point to EDD_p and EAD_p */
  *EDD_p_for_optimal_target_point = EDD_p;
  *EAD_p_for_optimal_target_point = EAD_p;
#endif

  return target_point_id;
}

int GetTargetPoint_By_RandomIntersection_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ // get the id of a target point for AP, such as a random intersection id on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle
  int AP_vertex_id = atoi(AP_vertex); //AP's vertex id, i.e., intersection id
  int id = 0; //intersection id
  int i = 0, j = 0; //for-loop indices
  int trial_count = 0; //trial count to limit the number of random selections

  char *target_point = NULL; //pointer to target pointer vertex name
  int target_point_id = 0;
  int random_target_point_index = 0; //index for a random target point on the destination vehicle trajectory 
  int remaining_path_hop_count = destination_vehicle->path_hop_count - destination_vehicle->path_current_hop; //the number of nodes consisting of the path corresponding to the vehicle trajectory from the head node of its current edge to the destination node (i.e., destination position).
  int path_node_number = remaining_path_hop_count + 1; //the number of nodes on the path to check for an optimal target point, including the tail node of its current edge
  struct_path_node *pPathNode = NULL; //pointer to a path node
  vehicle_trajectory_queue_t *Q = &(packet->vehicle_trajectory);
  int trajectory_size = Q->size; //size of the vehicle trajectory
  vehicle_trajectory_queue_node_t *pQueueNode = NULL; //pointer to the vehicle trajectory queue node

#ifdef __DEBUG_LEVEL_AP_PACKET_ARRIVE__
  char *target_point = NULL; //target point
  double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
  double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

  double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
  double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
#endif

  /** check the validity of the vehicle trajectory */
  if(trajectory_size == 0)
  {
      printf("GetTargetPoint_By_RandomIntersection_For_AP(): Error: the size of vehicle trajectory is zero!\n");
      exit(1);
  }

  /** determine the target point as a random intersection on the destination vehicle's trajectory */
	if(param->communication_multiple_SN_flag == TRUE)
	{ /* the case where the partial deployment of stationary nodes is performed */
		do
		{
			do
			{
				random_target_point_index = smpl_random(0, trajectory_size-1);
			} while(random_target_point_index < 0 || random_target_point_index > trajectory_size-1);

			/** search pQueueNode corresponding to random_target_point **/
			pQueueNode = &(Q->head); //Note that the first point to check is the tail node of the current edge
			for(i = 0; i <= random_target_point_index; i++)
			{
				pQueueNode = pQueueNode->next; //pQueueNode points to an directed edge
			}

			target_point = pQueueNode->graph_pos.enode->tail_node;
			id = atoi(target_point);

			/* check whether the intersection of id has a stationary node and the delay is finite */
			if((param->vanet_table.Gr[id-1].stationary_node_flag == TRUE) && (param->vanet_table.Dr_edd[AP_vertex_id-1][id-1] < INF))
			{
				target_point_id = id;
				break;
			}

			trial_count++;

			/* check the trial number to become over the threshold */
			if(trial_count == (2*Q->size))
			{
				printf("%s:%d GetTargetPoint_By_RandomIntersection_For_AP(): id(=%d) is not valid\n", 
						__FUNCTION__, __LINE__, id);
				exit(1);				
			}
		} while (1);
	}
	else
	{ /* the case where the full deployment of stationary nodes is performed */
		do
		{
			random_target_point_index = smpl_random(0, trajectory_size-1);
		} while(random_target_point_index < 0 || random_target_point_index > trajectory_size-1);

		/** search pQueueNode corresponding to random_target_point **/
		pQueueNode = &(Q->head); //Note that the first point to check is the tail node of the current edge
		for(i = 0; i <= random_target_point_index; i++)
		{
			pQueueNode = pQueueNode->next; //pQueueNode points to an directed edge
		}

		target_point = pQueueNode->graph_pos.enode->tail_node;
		target_point_id = atoi(target_point);
	}

#ifdef __DEBUG_LEVEL_AP_PACKET_ARRIVE__

  /** compute the travel time duration */
  //path_distance = destination_vehicle->edge_length - destination_vehicle->current_pos_in_digraph.offset; //path distance from vehicle's current position to the head node of the vehicle's current edge
  //EAD_p = path_distance / destination_vehicle->speed;

  /** get the EDD and EDD_SD for the target point at the intersection having this AP */
  VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(param, target_point_id, AP_vertex, FTQ, &EDD_p, &EDD_SD_p);

  /** set EDD_p_for_optimal_target_point and EAD_p_for_optimal_target_point to EDD_p and EAD_p */
  *EDD_p_for_optimal_target_point = EDD_p;
  *EAD_p_for_optimal_target_point = EAD_p;
#endif

  return target_point_id;
}


int GetMultipleTargetPoints_By_RandomIntersection_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, target_point_queue_t *TPQ)
{ // get the list of the ids of target points for AP, such as random intersection ids on the vehicle trajectory used to deliver the copies of a packet towards the vehicle that is a destination vehicle
	int target_point_number = 0;

	return target_point_number;
}

int GetTargetPoint_By_OptimalIntersection_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ // get the id of an optimal target point for AP such as intersection point or arbitrary point on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle

  int AP_vertex_id = atoi(AP_vertex); //id corresponding to the the vertex having the AP
  int target_point_id = 0;
  //int remaining_path_hop_count = destination_vehicle->path_hop_count - destination_vehicle->path_current_hop; //the number of nodes consisting of the path corresponding to the vehicle trajectory from the head node of its current edge to the destination node (i.e., destination position).
  //int path_node_number = remaining_path_hop_count + 1; //the number of nodes on the path to check for an optimal target point, including the tail node of its current edge
  int path_node_number = destination_vehicle->path_hop_count + 1; //the number of nodes on the path to check for an optimal target point, including the tail node of its current edge, assuming that the infinite vehicle trajectory is given to AP

  struct_path_node *pPathNode = NULL; //pointer to a path node
  char *target_point = NULL; //pointer to a vertex corresponding to a target point candidate
  double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
  double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

  double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
  double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
  double difference = 0 ;// absolute difference of EDD_p and EAD_p
  double min_difference = INF; //minimum absolute difference of EDD_p and EAD_p
  char min_difference_target_point[NAME_SIZE]; //target point with min difference
  int i = 0; //for-loop index
  char current_edge_tail_node[NAME_SIZE]; //tail node of the current edge pointed by vehicle->path_ptr

  /*@for debugging */
  //if(current_time >= 3623)
  //  printf("GetTargetPoint_By_OptimalIntersection(): time=%.2f, under debugging\n", current_time);
  /*****************/

  /** initialize double *EDD_p_for_optimal_target_point and double *EDD_p_for_optimal_target_point */
  *EDD_p_for_optimal_target_point = 0;
  *EAD_p_for_optimal_target_point = 0;

  /** copy the tail node of vehicle's current edge to current_edge_tail_node */
  strcpy(current_edge_tail_node, destination_vehicle->path_ptr->vertex);

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_AP__
  printf("GetTargetPoint_By_OptimalIntersection_For_AP(): vehicle=%d, graph_pos=(%s,%s:%.2f)\n", destination_vehicle->id, destination_vehicle->current_pos_in_digraph.enode->tail_node, destination_vehicle->current_pos_in_digraph.enode->head_node, destination_vehicle->current_pos_in_digraph.offset);
#endif

  /** search an optimal target point p such that |EDD_p - EAD_p| is minimum. **/
  pPathNode = destination_vehicle->path_ptr; //Note that the first point to check is the tail node of the current edge
  for(i = 0; i < path_node_number; i++)
  //for(i = 0; i < remaining_path_hop_count; i++)
  {
    /* if pPathNode points to the path list's head, let it point to head->next */
    if(pPathNode == destination_vehicle->path_list)
      pPathNode = pPathNode->next->next;

    target_point = pPathNode->vertex; //target_point is the tail node of an edge

    /** compute EAD_p for a target point candidate p */
/*     /\* compute the path distance from the current position to the target point *\/ */
/*     if(strcmp(target_point, current_edge_tail_node) == 0) */
/*     { //the case where vehicle has already passed target_point that is the tail node of the destination_vehicle's current edge */
/*       path_distance = -1*destination_vehicle->current_pos_in_digraph.offset; */
/*     } */
/*     else */
/*     { //the case where target_point is the node that vehicle will visit in future */
/*       path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList(current_time, destination_vehicle, target_point); */
/*     } */

    /* compute the path distance from the destination vehicle's current position to the target point */
    path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList(current_time, destination_vehicle, target_point);

    /* compute the travel time duration */
    EAD_p = path_distance / destination_vehicle->speed;

    /** compute EDD_p for a target point p */    
    /* compute the EDD and EDD_SD for the target point at the intersection having this AP */
    target_point_id = atoi(target_point);

    VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(param, target_point_id, AP_vertex, FTQ, &EDD_p, &EDD_SD_p);

    /* update min_difference and min_difference_target_point */
    difference =  fabs(EDD_p - EAD_p);

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_AP__
    printf("GetTargetPoint_By_OptimalIntersection_For_AP(): target point=%s, difference=%.2f\n", target_point, difference);
#endif 

    if(min_difference > difference)
    {
      min_difference = difference;
      strcpy(min_difference_target_point, target_point);
      *EDD_p_for_optimal_target_point = EDD_p;
      *EAD_p_for_optimal_target_point = EAD_p;
    }

    pPathNode = pPathNode->next; //pPathNode points to the next node on the path
  }


#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_AP__
  printf("GetTargetPoint_By_OptimalIntersection_For_AP(): optimal target point=%s, difference=%.2f\n", min_difference_target_point, min_difference);
  fgetc(stdin);
#endif

  /* convert min_differnce_target_point into target_point_id */
  target_point_id = atoi(min_difference_target_point);

  return target_point_id;
}

int GetTargetPoint_By_PacketEarliestArrivingIntersection_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ // get the id of a target point p for AP such as intersection point on the destination vehicle's trajectory where p is geographically closest to the destination vehicle such that EDD_p <= EAD_p
  int AP_vertex_id = atoi(AP_vertex); //id corresponding to the the vertex having the AP
  int target_point_id = 0; //target point candidate
  int optimal_target_point_id = 0; //optimal target point
  int path_node_number = destination_vehicle->path_hop_count + 1; //the number of nodes on the path to check for an optimal target point, including the tail node of its current edge, assuming that the infinite vehicle trajectory is given to AP

  struct_path_node *pPathNode = NULL; //pointer to a path node
  char *target_point = NULL; //pointer to a vertex corresponding to a target point candidate
  double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
  double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

  double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
  double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
  double difference = 0 ;// absolute difference of EDD_p and EAD_p
  int i = 0; //for-loop index
  char current_edge_tail_node[NAME_SIZE]; //tail node of the current edge pointed by vehicle->path_ptr

  /*@for debugging */
  //if(current_time >= 3623)
  //  printf("GetTargetPoint_By_PacketEarliestArrivingIntersection(): time=%.2f, under debugging\n", current_time);
  /*****************/

  /** initialize double *EDD_p_for_optimal_target_point and double *EDD_p_for_optimal_target_point */
  *EDD_p_for_optimal_target_point = 0;
  *EAD_p_for_optimal_target_point = 0;

  /** copy the tail node of vehicle's current edge to current_edge_tail_node */
  strcpy(current_edge_tail_node, destination_vehicle->path_ptr->vertex);

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_AP__
  printf("GetTargetPoint_By_PacketEarliestArrivingIntersection_For_AP(): vehicle=%d, graph_pos=(%s,%s:%.2f)\n", destination_vehicle->id, destination_vehicle->current_pos_in_digraph.enode->tail_node, destination_vehicle->current_pos_in_digraph.enode->head_node, destination_vehicle->current_pos_in_digraph.offset);
#endif

  /** search a target point p where p is geographically closest to the destination vehicle such that EDD_p <= EAD_p **/
  pPathNode = destination_vehicle->path_ptr; //Note that the first point to check is the tail node of the current edge
  for(i = 0; i < path_node_number; i++)
  {
    /* if pPathNode points to the path list's head, let it point to head->next */
    if(pPathNode == destination_vehicle->path_list)
      pPathNode = pPathNode->next->next;

    target_point = pPathNode->vertex; //target_point is the tail node of an edge

    /** compute EAD_p for a target point candidate p */
    /* compute the path distance from the destination vehicle's current position to the target point */
    path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList(current_time, destination_vehicle, target_point);

    /* compute the travel time duration */
    EAD_p = path_distance / destination_vehicle->speed;

    /** compute EDD_p for a target point p */    
    /* compute the EDD and EDD_SD for the target point at the intersection having this AP */
    target_point_id = atoi(target_point);

    VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(param, target_point_id, AP_vertex, FTQ, &EDD_p, &EDD_SD_p);

    /* check whether the difference of EDD_p - EAD_p is non-positive or not */
    difference =  EDD_p - EAD_p;

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_AP__
    printf("GetTargetPoint_By_PacketEarliestArrivingIntersection_For_AP(): target point=%s, difference=%.2f\n", target_point, difference);
#endif 

    if(difference <= 0)
    {
      optimal_target_point_id = target_point_id;
      *EDD_p_for_optimal_target_point = EDD_p;
      *EAD_p_for_optimal_target_point = EAD_p;
      break;
    }

    pPathNode = pPathNode->next; //pPathNode points to the next node on the path
  }


#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_AP__
  printf("GetTargetPoint_By_PacketEarliestArrivingIntersection_For_AP(): optimal target point=%s, difference=%.2f\n", optimal_target_point, difference);
  fgetc(stdin);
#endif

  /** check whether there exists no intersection p such that EDD_p <= EAD_p or not; if so, find an optimal target point p in terms of mininum |EDD_p - EAD_p| */
  if(optimal_target_point_id == 0)
  {
      //optimal_target_point_id = GetTargetPoint_By_OptimalIntersection_For_AP(param, current_time, AP_vertex, destination_vehicle, FTQ, EDD_p_for_optimal_target_point, EAD_p_for_optimal_target_point);
      printf("GetTargetPoint_By_PacketEarliestArrivingIntersection_For_AP(): Error: There exists no intersection as target point p such that EDD_p <= EAD_p.\n");
      exit(1);
  }

  return optimal_target_point_id;
}

int GetTargetPoint_By_PacketTrajectory_For_Multiple_APs(parameter_t *param, double current_time, access_point_queue_t *APQ, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point, access_point_queue_node_t **transmitter_AP_qnode)
{ //In the Multiple-AP road network, with the packet trajectory from the AP to the destination, return the id of an optimal target point for AP such as intersection point or arbitrary point on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle and also return the pointer to the AP queue node for the actual transmitter AP 

  int AP_vertex_id = 0; //id corresponding to the the vertex having the AP
  int target_point_id = 0; //target point id
  int optimal_target_point_id = 0; //optimal target point id
  
  int size = APQ->size; //the size of AP queue, that is, the number of APs
  access_point_queue_node_t *pQueueNode = NULL; //pointer to an access point queue node
  access_point_queue_node_t *minimum_EAD_AP = NULL; //pointer to the access point queue node with the minimum delivery delay
  double minimum_EAD = INF; //minimum Expected Arrival Delay (EAD), that is, Expected Vehicle Delay
  int i = 0; //for-loop index
  char *AP_vertex = NULL; //the intersection id for the AP
  double EDD_p = 0; //EDD for a target point p
  double EAD_p = 0; //EAD for a target point p

  double optimal_EDD_p = 0; //EDD for an optimal target point p
  double optimal_EAD_p = 0; //EAD for an optimal target point p

  /** find the AP to have the minimum EAD as the actual packet transmitter */
  pQueueNode = &(APQ->head);
  for(i = 0; i < size; i++)
  {
      pQueueNode = pQueueNode->next;

      /* search an optimal target point for the AP pointed by pQueueNode */
      AP_vertex = pQueueNode->vertex;
      target_point_id = GetTargetPoint_By_PacketTrajectory_For_AP(param, current_time, AP_vertex, destination_vehicle, packet, FTQ, &EDD_p, &EAD_p);

      /* check whether this AP has the minimum EAD */
      if(EAD_p < minimum_EAD)
      {
          minimum_EAD = EAD_p;
          minimum_EAD_AP = pQueueNode;
          optimal_target_point_id = target_point_id;
          optimal_EDD_p = EDD_p;
          optimal_EAD_p = EAD_p;
      }
  }

  if(optimal_target_point_id == 0)
  {
      printf("GetTargetPoint_By_PacketTrajectory_For_Multiple_APs(): optimal_target_point_id is 0!\n");
      exit(1);
  }
  else
  {
      /** set transmitter_AP_qnode to the AP queue node with the minimum EAD */
      *transmitter_AP_qnode = minimum_EAD_AP;
      
      /** set up EDD_p_for_optimal_target_point and EAD_p_for_optimal_target_point */
      *EDD_p_for_optimal_target_point = optimal_EDD_p;
      *EAD_p_for_optimal_target_point = optimal_EAD_p;
  }
  
  return optimal_target_point_id;
}

int GetTargetPoint_By_EndIntersection_For_Multiple_APs(parameter_t *param, double current_time, access_point_queue_t *APQ, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point, access_point_queue_node_t **transmitter_AP_qnode)
{ //In the Multiple-AP road network, we select a target point with the last intersection on the vehicle trajectory and select the actual transmitter AP with the one having the shortest delivery delay to the last intersection and also return the pointer to the AP queue node for the actual transmitter AP 

  int AP_vertex_id = 0; //id corresponding to the the vertex having the AP
  char *target_point = NULL; //the end intersection of the vehicle trajectory is selected as target point
  int trajectory_size = packet->vehicle_trajectory.size; //the size of the vehicle trajectory
  int optimal_target_point_id = 0; //optimal target point id
  
  int size = APQ->size; //the size of AP queue, that is, the number of APs
  access_point_queue_node_t *pQueueNode = NULL; //pointer to an access point queue node
  access_point_queue_node_t *minimum_EDD_AP = NULL; //pointer to the access point queue node with the minimum delivery delay
  double minimum_EDD = INF; //minimum Expected Delivery Delay (EDD), that is, Expected Packet Delay
  int i = 0; //for-loop index
  char *AP_vertex = NULL; //the intersection id for the AP
  double EDD_p = 0; //EDD for a target point p
  double EDD_SD_p = 0; //EDD_SD for a target point p
  double EAD_p = 0; //EAD for a target point p

  double optimal_EDD_p = 0; //EDD for an optimal target point p
  double optimal_EAD_p = 0; //EAD for an optimal target point p

  /** check the validity of the vehicle trajectory */
  if(trajectory_size == 0)
  {
      printf("GetTargetPoint_By_EndIntersection_For_Multiple_APs(): Error: the size of vehicle trajectory is zero!\n");
      exit(1);
  }

  /** determine the target point as the destination vehicle's end intersection on its vehicle trajectory */
  target_point = packet->vehicle_trajectory.head.prev->graph_pos.enode->head_node; //the end intersection of the vehicle trajectory is selected as target point

  optimal_target_point_id = atoi(target_point);

  /** find the AP to have the minimum EDD as the actual packet transmitter */
  pQueueNode = &(APQ->head);
  for(i = 0; i < size; i++)
  {
      pQueueNode = pQueueNode->next;

      /** search an AP with the shortest delivery delay to the target point */
      AP_vertex = pQueueNode->vertex;

      /* get the EDD and the EDD_SD for the target point from the AP */
      VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(param, optimal_target_point_id, AP_vertex, FTQ,  &EDD_p, &EDD_SD_p);
      /* check whether this AP has the minimum EDD */
      if(EDD_p < minimum_EDD)
      {
          minimum_EDD = EDD_p;
          minimum_EDD_AP = pQueueNode;
          optimal_EDD_p = EDD_p;
      }
  }

  if(optimal_target_point_id == 0)
  {
      printf("GetTargetPoint_By_EndIntersection_For_Multiple_APs(): optimal_target_point_id is 0!\n");
      exit(1);
  }
  else
  {
      /** set transmitter_AP_qnode to the AP queue node with the minimum EDD */
      *transmitter_AP_qnode = minimum_EDD_AP;
      
      /** set up EDD_p_for_optimal_target_point and EAD_p_for_optimal_target_point */
      *EDD_p_for_optimal_target_point = optimal_EDD_p;
      *EAD_p_for_optimal_target_point = optimal_EAD_p;
  }
  
  return optimal_target_point_id;
}

int GetTargetPoint_By_RandomIntersection_For_Multiple_APs(parameter_t *param, double current_time, access_point_queue_t *APQ, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point, access_point_queue_node_t **transmitter_AP_qnode)
{ //In the Multiple-AP road network, we select a random target point among the intersections on the vehicle trajectory and select the actual transmitter AP with the one having the shortest delivery delay to the target point and also return the pointer to the AP queue node for the actual transmitter AP 

  int AP_vertex_id = 0; //id corresponding to the the vertex having the AP
  char *target_point = NULL; //the end intersection of the vehicle trajectory is selected as target point
  int optimal_target_point_id = 0; //optimal target point id
  int random_target_point_index = 0; //index for a random target point on the destination vehicle trajectory 
  
  vehicle_trajectory_queue_t *Q = &(packet->vehicle_trajectory); //vehicle trajectory queue
  int trajectory_size = Q->size; //size of the vehicle trajectory
  vehicle_trajectory_queue_node_t *pTrajectoryQueueNode = NULL; //pointer to the vehicle trajectory queue node

  int size = APQ->size; //the size of AP queue, that is, the number of APs
  access_point_queue_node_t *pQueueNode = NULL; //pointer to an access point queue node
  access_point_queue_node_t *minimum_EDD_AP = NULL; //pointer to the access point queue node with the minimum delivery delay
  double minimum_EDD = INF; //minimum Expected Delivery Delay (EDD), that is, Expected Packet Delay
  int i = 0; //for-loop index
  char *AP_vertex = NULL; //the intersection id for the AP
  double EDD_p = 0; //EDD for a target point p
  double EDD_SD_p = 0; //EDD_SD for a target point p
  double EAD_p = 0; //EAD for a target point p

  double optimal_EDD_p = 0; //EDD for an optimal target point p
  double optimal_EAD_p = 0; //EAD for an optimal target point p

  /** check the validity of the vehicle trajectory */
  if(trajectory_size == 0)
  {
      printf("GetTargetPoint_By_RandomIntersection_For_Multiple_APs(): Error: the size of vehicle trajectory is zero!\n");
      exit(1);
  }

  /** determine the target point as a random intersection on the destination vehicle's trajectory */
  do
  {
    random_target_point_index = smpl_random(0, trajectory_size-1);
  } while(random_target_point_index < 0 || random_target_point_index > trajectory_size-1);

  /** search pQueueNode corresponding to random_target_point **/
  pTrajectoryQueueNode = &(Q->head); //Note that the first point to check is the tail node of the current edge
  for(i = 0; i <= random_target_point_index; i++)
  {
    pTrajectoryQueueNode = pTrajectoryQueueNode->next; //pTrajectoryQueueNode points to an directed edge
  }

  target_point = pTrajectoryQueueNode->graph_pos.enode->tail_node;
  optimal_target_point_id = atoi(target_point);

  /** find the AP to have the minimum EDD as the actual packet transmitter */
  pQueueNode = &(APQ->head);
  for(i = 0; i < size; i++)
  {
      pQueueNode = pQueueNode->next;

      /** search an AP with the shortest delivery delay to the target point */
      AP_vertex = pQueueNode->vertex;

      /* get the EDD and the EDD_SD for the target point from the AP */
      VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(param, optimal_target_point_id, AP_vertex, FTQ,  &EDD_p, &EDD_SD_p);
      /* check whether this AP has the minimum EDD */
      if(EDD_p < minimum_EDD)
      {
          minimum_EDD = EDD_p;
          minimum_EDD_AP = pQueueNode;
          optimal_EDD_p = EDD_p;
      }
  }

  if(optimal_target_point_id == 0)
  {
      printf("GetTargetPoint_By_RandomIntersection_For_Multiple_APs(): optimal_target_point_id is 0!\n");
      exit(1);
  }
  else
  {
      /** set transmitter_AP_qnode to the AP queue node with the minimum EDD */
      *transmitter_AP_qnode = minimum_EDD_AP;
      
      /** set up EDD_p_for_optimal_target_point and EAD_p_for_optimal_target_point */
      *EDD_p_for_optimal_target_point = optimal_EDD_p;
      *EAD_p_for_optimal_target_point = optimal_EAD_p;
  }
  
  return optimal_target_point_id;
}

int GetTargetPoint_By_PacketTrajectory_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ //With the packet trajectory from the AP to the destination, get the id of an optimal target point for AP such as intersection point or arbitrary point on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle

  int AP_vertex_id = atoi(AP_vertex); //id corresponding to the the vertex having the AP
  int target_point_id = 0;
  //int remaining_path_hop_count = destination_vehicle->path_hop_count - destination_vehicle->path_current_hop; //the number of nodes consisting of the path corresponding to the vehicle trajectory from the head node of its current edge to the destination node (i.e., destination position).
  //int path_node_number = remaining_path_hop_count + 1; //the number of nodes on the path to check for an optimal target point, including the tail node of its current edge
  int path_node_number = destination_vehicle->path_hop_count; //the number of nodes on the path to check for an optimal target point, including the tail node of its current edge, assuming that the infinite vehicle trajectory is given to AP; note that the tail node of its current edge is the node to check first and the travel distance will usually be negative.
  //int path_node_number = destination_vehicle->path_hop_count + 1; //the number of nodes on the path to check for an optimal target point, including the tail node of its current edge, assuming that the infinite vehicle trajectory is given to AP

  struct_path_node *pPathNode = NULL; //pointer to a path node
  char *target_point = NULL; //pointer to a vertex corresponding to a target point candidate
  double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
  double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

  double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
  double path_travel_time = 0; //path travel time
  double path_travel_time_deviation = 0; //path travel time deviation

  double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
  double EAD_VAR_p = 0; //EAD_VAR_p is the variance of the destination vehicle's arrival delay to target point p
  double EAD_SD_p = 0; //EAD_SD_p is the standard deviation of the destination vehicle's arrival delay to target point p

  double value = 0 ;//value used for selecting an optimal intersection based on (EDD_p, EDD_SD_p) and (EAD_p, EAD_SD_p)
  double min_value = INF; //minimum value for selecting an optimal intersection
  char min_value_target_point[NAME_SIZE]; //target point with min_value

  int i = 0; //for-loop index
  char current_edge_tail_node[NAME_SIZE]; //tail node of the current edge pointed by vehicle->path_ptr

  double max_constraint_value = 0; //maximum constraint value to select a target point in the case where there exists no target point to satisfy the required constraint
  int max_constraint_value_target_point_id = 0; //target point corresponding to the maximum constraint value

  double packet_ttl = param->communication_packet_ttl; //packet's TTL
  int last_trajectory_point = 0; //last trajectory point as last resort for target point
  int id = 0; //intersection id
 
  /*@for debugging */
  //if(current_time >= 3623)
  //  printf("GetTargetPoint_By_PacketTrajectory_For_AP(): time=%.2f, under debugging\n", current_time);
  /*****************/

  /** initialize double *EDD_p_for_optimal_target_point and double *EDD_p_for_optimal_target_point */
  *EDD_p_for_optimal_target_point = 0;
  *EAD_p_for_optimal_target_point = 0;

  /** copy the tail node of vehicle's current edge to current_edge_tail_node */
  strcpy(current_edge_tail_node, destination_vehicle->path_ptr->vertex);

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_AP__
  printf("GetTargetPoint_By_PacketTrajectory_For_AP(): vehicle=%d, graph_pos=(%s,%s:%.2f)\n", destination_vehicle->id, destination_vehicle->current_pos_in_digraph.enode->tail_node, destination_vehicle->current_pos_in_digraph.enode->head_node, destination_vehicle->current_pos_in_digraph.offset);
#endif

  /** search an optimal target point p such that |EDD_p - EAD_p| is minimum. **/
  pPathNode = destination_vehicle->path_ptr; //Note that the first point to check is the tail node of the current edge
  for(i = 0; i < path_node_number; i++)
  //for(i = 0; i < remaining_path_hop_count; i++)
  {
    /* if pPathNode points to the path list's head, let it point to head->next */
    if(pPathNode == destination_vehicle->path_list)
      pPathNode = pPathNode->next->next;

    target_point = pPathNode->vertex; //target_point is the tail node of an edge

    ///* compute the path distance from the destination vehicle's current position to the target point */
    //path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList(current_time, destination_vehicle, target_point);

    /* compute the path distance, the path travel time, and the path travel time deviation from the destination vehicle's current position to the target point */
    path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList_Along_With_PathTravelTime_And_Deviation(param, current_time, destination_vehicle, target_point, &path_travel_time, &path_travel_time_deviation);

    /* compute the mean and standard deviation of the travel time duration 
       @Note: make sure that EAD_p is non-zero for the Gamma distribution */
    if(path_travel_time < 0)
    {
        pPathNode = pPathNode->next; //pPathNode points to the next node on the path

        continue; //In this case, the destination vehicle has already passed this target point
    }
    else
    {
        EAD_p = path_travel_time; //param->vehicle_speed should be used since it is average vehicle speed
        EAD_SD_p = path_travel_time_deviation;
    }   

    ///* compute the mean and standard deviation of the travel time duration */
    //EAD_p = path_travel_time; //param->vehicle_speed should be used since it is average vehicle speed
    //EAD_SD_p = path_travel_time_deviation;

    //EAD_p = path_distance / param->vehicle_speed; //param->vehicle_speed should be used since it is average vehicle speed
    //EAD_p = path_distance / destination_vehicle->speed;

    /* compute the standard deviation of the travel time duration */
    //EAD_VAR_p = (i+1)*param->vehicle_speed_variance;
    //EAD_SD_p = sqrt(EAD_VAR_p);

    /** compute EDD_p for a target point p */
    /* compute the EDD and EDD_SD for the target point at the intersection having this AP */
    target_point_id = atoi(target_point);

    VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(param, target_point_id, AP_vertex, FTQ, &EDD_p, &EDD_SD_p);

    /** Filtering: check the difference between EDD_p and EAD_p with ten times of EAD_p */
    if(EDD_p > EAD_p)
    //if(EDD_p >= EAD_p*10)
    {
        pPathNode = pPathNode->next; //pPathNode points to the next node on the path

        continue; //In this case, the destination vehicle has already passed this target point
    }

    /** Compute the probability that the packet will arrive at the target point earlier than the destination vehicle */
    /* Note that the probability is P[T_i^p <= T_i^v] = \int_{0}^{\infy} \int_{0}^{y} {f(x)g(y)}\,dx\,dy */

    /* update min_value and min_value_target_point */ 
    //value = fabs(EDD_p - EAD_p);
    value = Compute_TargetPoint_OptimizationValue(param, target_point_id, EDD_p, EDD_SD_p, EAD_p, EAD_SD_p, packet_ttl, &max_constraint_value, &max_constraint_value_target_point_id); //compute the optimization value for an intersection i that is an optimal target point candidate given the the packet delivery delay distribution (EDD_p, EDD_SD_p) and the destination vehicle arrival delay distribution (EAD_p, EAD_SD_p).

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_AP__
    printf("GetTargetPoint_By_PacketTrajectory_For_AP(): target point=%s, value=%.2f\n", target_point, value);
#endif 

    if(min_value > value)
    {
      min_value = value;
      strcpy(min_value_target_point, target_point);
      *EDD_p_for_optimal_target_point = EDD_p;
      *EAD_p_for_optimal_target_point = EAD_p;
    }

    pPathNode = pPathNode->next; //pPathNode points to the next node on the path
  }

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_AP__
  printf("GetTargetPoint_By_PacketTrajectory_For_AP(): optimal target point=%s, min_value=%.2f\n", min_value_target_point, min_value);
  fgetc(stdin);
#endif

  /* @for debugging */
  //if(packet->id == 33)
  //  printf("GetTargetPoint_By_PacketTrajectory_For_AP(): packet->id=%d\n", packet->id);
  /******************/

  /* convert min_value_target_point into target_point_id */
  if(min_value == INF)
  { //the case where there is no intersection to satisfy the constraint: we choose the last point of the vehicle trajectory
	if(max_constraint_value > 0)
	{
		target_point_id = max_constraint_value_target_point_id;
	}
	else
	{	
	    /* set the last intersection on the vehicle trajectory to the target point as last resort when there is no target point with positive max_constraint_value */
        //last_trajectory_point = atoi(destination_vehicle->path_list->prev->prev->vertex);

		if(param->communication_multiple_SN_flag == TRUE)
		{ /* the case where the partial deployment of stationary nodes is performed */
			/** search the intersection with stationary node from the end of the trajectory */
			pPathNode = destination_vehicle->path_ptr->prev; //Note that the last trajectory point is the intersection just before the tail node of the current edge where the destination vehicle is moving

			/* if pPathNode points to the path list's head, let it point to head->prev */
			if(pPathNode == destination_vehicle->path_list)
				pPathNode = pPathNode->prev;

			do
			{
				id = atoi(pPathNode->vertex);

#if 0 /* [ */
				printf("%s:%d packet_id=%d, target_point=%d\n", __FILE__, __LINE__, packet->id, id);
#endif /* ] */
				/* check whether the intersection of id has a stationary node and the delay is finite */
				if((param->vanet_table.Gr[id-1].stationary_node_flag == TRUE) && (param->vanet_table.Dr_edd[AP_vertex_id-1][id-1] < INF))
				{
					break;
				}
	
				if(pPathNode == destination_vehicle->path_ptr)
				{
					printf("%s:%d GetTargetPoint_By_PacketTrajectory_For_AP(): there is no intersection with stationary node in the trajectory\n", __FILE__, __LINE__);
					exit(1);
				}

				pPathNode = pPathNode->prev; //reverse search

				/* if pPathNode points to the path list's head, let it point to head->prev */
				if(pPathNode == destination_vehicle->path_list)
					pPathNode = pPathNode->prev;

			} while(1);
		}
		else
		{ /* the case where the full deployment of stationary nodes is performed */
			pPathNode = destination_vehicle->path_ptr->prev; //Note that the last trajectory point is the intersection just before the tail node of the current edge where the destination vehicle is moving
			if(pPathNode == destination_vehicle->path_list)
			{
				pPathNode = pPathNode->prev->prev;
			}
		}

		last_trajectory_point = atoi(pPathNode->vertex);

		/* check the validity of last_trajectory_point*/
		if(last_trajectory_point == 0)
		{
			printf("GetTargetPoint_By_PacketTrajectory_For_AP(): Error: last_trajectory_point is 0\n");
			exit(1);
		}

		target_point_id = last_trajectory_point;

#if __DEBUG_LEVEL_TARGET_POINT_SELECTION_TRACE__
        printf("GetTargetPoint_By_PacketTrajectory_For_AP(): last_trajectory_point(%d) is selected as target point\n", last_trajectory_point);	  
#endif
    }

    //Note that in the case where max_constraint_value == 0, we use the last target_point_id as it is.
	//Actually, there is no such case in the simulation.
  }
  else
  {
      target_point_id = atoi(min_value_target_point);
  }

  return target_point_id;
}

int GetTargetPoint_For_AP_For_V2V_Data_Delivery(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, forwarding_table_queue_t *FTQ, int Gr_set_number, struct_graph_node **Gr_set, int *Gr_set_size, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ //get the id of a target point for AP such as an intersection on the destination vehicle's trajectory used to deliver a packet towards the destination vehicle
	int AP_vertex_id = atoi(AP_vertex); //id corresponding to the the vertex having the AP
	int target_point_id = 0;
	int path_node_number = destination_vehicle->path_hop_count; //the number of nodes on the path to check for an optimal target point, including the tail node of its current edge, assuming that the infinite vehicle trajectory is given to AP; note that the tail node of its current edge is the node to check first and the travel distance will usually be negative.

	struct_path_node *pPathNode = NULL; //pointer to a path node
	char *target_point = NULL; //pointer to a vertex corresponding to a target point candidate
	double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
	double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

	double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
	double path_travel_time = 0; //path travel time
	double path_travel_time_deviation = 0; //path travel time deviation

	double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
	double EAD_VAR_p = 0; //EAD_VAR_p is the variance of the destination vehicle's arrival delay to target point p
	double EAD_SD_p = 0; //EAD_SD_p is the standard deviation of the destination vehicle's arrival delay to target point p

	double value = 0 ;//value used for selecting an optimal intersection based on (EDD_p, EDD_SD_p) and (EAD_p, EAD_SD_p)
	double min_value = INF; //minimum value for selecting an optimal intersection
	char min_value_target_point[NAME_SIZE]; //target point with min_value

	int i = 0; //for-loop index
	char current_edge_tail_node[NAME_SIZE]; //tail node of the current edge pointed by vehicle->path_ptr

	double max_constraint_value = 0; //maximum constraint value to select a target point in the case where there exists no target point to satisfy the required constraint
	int max_constraint_value_target_point_id = 0; //target point corresponding to the maximum constraint value

	double packet_ttl = param->communication_packet_ttl; //packet's TTL
	int last_trajectory_point = 0; //last trajectory point as last resort for target point
	int id = 0; //intersection id
 
	/** initialize double *EDD_p_for_optimal_target_point and double *EDD_p_for_optimal_target_point */
	*EDD_p_for_optimal_target_point = 0;
	*EAD_p_for_optimal_target_point = 0;

	/** copy the tail node of vehicle's current edge to current_edge_tail_node */
	strcpy(current_edge_tail_node, destination_vehicle->path_ptr->vertex);

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_AP__
	printf("GetTargetPoint_By_PacketTrajectory_For_AP(): vehicle=%d, graph_pos=(%s,%s:%.2f)\n", destination_vehicle->id, destination_vehicle->current_pos_in_digraph.enode->tail_node, destination_vehicle->current_pos_in_digraph.enode->head_node, destination_vehicle->current_pos_in_digraph.offset);
#endif

	/** search an optimal target point p such that |EDD_p - EAD_p| is minimum. **/
	pPathNode = destination_vehicle->path_ptr; //Note that the first point to check is the tail node of the current edge
	for(i = 0; i < path_node_number; i++)
	{
		/* if pPathNode points to the path list's head, let it point to head->next */
		if(pPathNode == destination_vehicle->path_list)
			pPathNode = pPathNode->next->next;

		target_point = pPathNode->vertex; //target_point is the tail node of an edge

		/* compute the path distance, the path travel time, and the path travel time deviation from the destination vehicle's current position to the target point */
		path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList_Along_With_PathTravelTime_And_Deviation(param, current_time, destination_vehicle, target_point, &path_travel_time, &path_travel_time_deviation);

		/* compute the mean and standard deviation of the travel time duration 
			@Note: make sure that EAD_p is non-zero for the Gamma distribution */
		if(path_travel_time < 0)
		{
			pPathNode = pPathNode->next; //pPathNode points to the next node on the path
			continue; //In this case, the destination vehicle has already passed this target point
		}
		else
		{
			EAD_p = path_travel_time; //param->vehicle_speed should be used since it is average vehicle speed
			EAD_SD_p = path_travel_time_deviation;
		}   
    
		/** compute EDD_p for a target point p */
		/* compute the EDD and EDD_SD for the target point at the intersection having this AP */
		target_point_id = atoi(target_point);

		VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection_For_V2V_Data_Delivery(param, target_point_id, AP_vertex, FTQ, Gr_set[target_point_id-1], Gr_set_size[target_point_id-1], &EDD_p, &EDD_SD_p);
		//VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(param, target_point_id, AP_vertex, FTQ, &EDD_p, &EDD_SD_p);

		/** Filtering: check the difference between EDD_p and EAD_p with ten times of EAD_p */
		if(EDD_p > EAD_p)
		{
			pPathNode = pPathNode->next; //pPathNode points to the next node on the path
			continue; //In this case, the destination vehicle has already passed this target point
		}

		/** Compute the probability that the packet will arrive at the target point earlier than the destination vehicle */
		/* Note that the probability is P[T_i^p <= T_i^v] = \int_{0}^{\infy} \int_{0}^{y} {f(x)g(y)}\,dx\,dy */

		/* update min_value and min_value_target_point */ 
		//value = fabs(EDD_p - EAD_p);
		value = Compute_TargetPoint_OptimizationValue(param, target_point_id, EDD_p, EDD_SD_p, EAD_p, EAD_SD_p, packet_ttl, &max_constraint_value, &max_constraint_value_target_point_id); //compute the optimization value for an intersection i that is an optimal target point candidate given the the packet delivery delay distribution (EDD_p, EDD_SD_p) and the destination vehicle arrival delay distribution (EAD_p, EAD_SD_p).

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_AP__
		printf("GetTargetPoint_By_PacketTrajectory_For_AP(): target point=%s, value=%.2f\n", target_point, value);
#endif 

		if(min_value > value)
		{
			min_value = value;
			strcpy(min_value_target_point, target_point);
			*EDD_p_for_optimal_target_point = EDD_p;
			*EAD_p_for_optimal_target_point = EAD_p;
		}

		pPathNode = pPathNode->next; //pPathNode points to the next node on the path
	}

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_AP__
	printf("GetTargetPoint_By_PacketTrajectory_For_AP(): optimal target point=%s, min_value=%.2f\n", min_value_target_point, min_value);
	fgetc(stdin);
#endif

	/* convert min_value_target_point into target_point_id */
	if(min_value == INF)
	{ //the case where there is no intersection to satisfy the constraint: we choose the last point of the vehicle trajectory
		if(max_constraint_value > 0)
		{
			target_point_id = max_constraint_value_target_point_id;
		}
		else
		{	
			/* set the last intersection on the vehicle trajectory to the target point as last resort when there is no target point with positive max_constraint_value */
			//last_trajectory_point = atoi(destination_vehicle->path_list->prev->prev->vertex);

			if(param->communication_multiple_SN_flag == TRUE)
			{ /* the case where the partial deployment of stationary nodes is performed */
				/** search the intersection with stationary node from the end of the trajectory */
				pPathNode = destination_vehicle->path_ptr->prev; //Note that the last trajectory point is the intersection just before the tail node of the current edge where the destination vehicle is moving

				/* if pPathNode points to the path list's head, let it point to head->prev */
				if(pPathNode == destination_vehicle->path_list)
					pPathNode = pPathNode->prev;

				do
				{
					id = atoi(pPathNode->vertex);

#if 0 /* [ */
					printf("%s:%d packet_id=%d, target_point=%d\n", __FILE__, __LINE__, packet->id, id);
#endif /* ] */

					/* check whether the intersection of id has a stationary node and the delay is finite */
					if((param->vanet_table.Gr[id-1].stationary_node_flag == TRUE) && (param->vanet_table.Dr_edd[AP_vertex_id-1][id-1] < INF))
					{
						break;
					}
	
					if(pPathNode == destination_vehicle->path_ptr)
					{
						printf("%s:%d GetTargetPoint_By_PacketTrajectory_For_AP(): there is no intersection with stationary node in the trajectory\n", __FILE__, __LINE__);
						exit(1);
					}

					pPathNode = pPathNode->prev; //reverse search

					/* if pPathNode points to the path list's head, let it point to head->prev */
					if(pPathNode == destination_vehicle->path_list)
						pPathNode = pPathNode->prev;
				} while(1);
			}
			else
			{ /* the case where the full deployment of stationary nodes is performed */
				pPathNode = destination_vehicle->path_ptr->prev; //Note that the last trajectory point is the intersection just before the tail node of the current edge where the destination vehicle is moving
				if(pPathNode == destination_vehicle->path_list)
				{
					pPathNode = pPathNode->prev->prev;
				}
			}

			last_trajectory_point = atoi(pPathNode->vertex);

			/* check the validity of last_trajectory_point*/
			if(last_trajectory_point == 0)
			{
				printf("GetTargetPoint_By_PacketTrajectory_For_AP(): Error: last_trajectory_point is 0\n");
				exit(1);
			}

			target_point_id = last_trajectory_point;

#if __DEBUG_LEVEL_TARGET_POINT_SELECTION_TRACE__
			printf("GetTargetPoint_By_PacketTrajectory_For_AP(): last_trajectory_point(%d) is selected as target point\n", last_trajectory_point);	  
#endif
		}
		//Note that in the case where max_constraint_value == 0, we use the last target_point_id as it is.
		//Actually, there is no such case in the simulation.
	}
	else
	{
		target_point_id = atoi(min_value_target_point);
	}

	return target_point_id;
}

int GetMultipleTargetPoints_By_PacketTrajectory_For_AP(parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, target_point_queue_t *global_TPQ)
{ //With the packet trajectory from the AP to the destination, get the list of the ids of optimal target points for AP such as intersection points on the vehicle trajectory used to deliver the copies of a packet towards the vehicle that is a destination vehicle
	int target_point_number = 0; //the number of target points
	int n = destination_vehicle->path_hop_count; //the number of nodes on the path to check for an optimal target point, including the tail node of its current edge, assuming that the infinite vehicle trajectory is given to AP
	/* Note: path_hop_count is the number of edges on the vehicle trajectory where 
	 * the start intersection and the end intersection are the same. Thus, path_hop_count 
	 * can represent all of the intersection consisting of the vehicle trajectory. */

	int k = 0; //for-loop index
	struct_path_node *pPathNode = NULL; //pointer to a path node
	int target_point_id = 0; //target point id
	int *w = NULL; /* intersection vector whose element contains the intersection id 
					* that is one of intersections consisting of the destination 
					* vehicle's trajectory
					*/
	int *v = NULL; /* index vector for intersection vector w; 
					* for example, v[1] = 1 and w[v[1]] is the first intersection id
					*/
	double *p = NULL; /* delivery probability whose element contains 
					   * the E2E delivery delay from AP to target point 
					   * where the index i indicates a target point.
					   * For example, d[i] is the delivery delay of target point w[i].
					   */
	double *d = NULL; /* delivery delay whose element contains 
					   * the E2E delivery probability from AP to target point 
					   * where the index indicates a target point.
					   * For example, p[i] is the delivery probability of target point w[i].
					   */
	int max_number = 0; //maximum number of target points allowed for multi-target-point data forwarding
	target_point_queue_node_t qnode; //target point queue node
	double delivery_probability = 0; //delivery probability from AP to target point
	double delivery_delay = 0; //delivery delay from AP to target point

	/* Note: w[0] and v[0] are not used, so index 1 indicates the first item in both w[] and v[]. */

	/* allocate the memory for vectors w, v, p, and d */
	w = (int*) calloc(n+1, sizeof(int));
	assert_memory(w);

	v = (int*) calloc(n+1, sizeof(int));
	assert_memory(v);

	p = (double*) calloc(n+1, sizeof(double));
	assert_memory(p);

	d = (double*) calloc(n+1, sizeof(double));
	assert_memory(d);

	/* copy the intersection ids of the vehicle trajectory into w */
	 pPathNode = destination_vehicle->path_ptr; //Note that the first point to check is the tail node of the current edge
	for(k = 1; k <= n; k++)
	{
		/* if pPathNode points to the path list's head, let it point to head->next */
		if(pPathNode == destination_vehicle->path_list)
			pPathNode = pPathNode->next->next;

		target_point_id = atoi(pPathNode->vertex); //target_point_id is the tail node of an edge
		w[k] = target_point_id;
		pPathNode = pPathNode->next;

		/* compute the delivery delay and the delivery probability for target_point_id */
		delivery_probability = compute_delivery_probability_along_with_delivery_delay_for_target_point(
			target_point_id, param, current_time, AP_vertex, 
			destination_vehicle, FTQ, &delivery_delay);

		p[k] = delivery_probability;
		d[k] = delivery_delay;
	}

	/* search a small number k of target points in order to satisfy the user-required delivery probability */
	for(k = 1; k <= param->data_forwarding_maximum_target_point_number; k++)
	//for(k = 1; k <= n; k++)
   	{
		/* select k target points to satisfy the user-required delivery probability */

		/* if there are such k target points for the requirement, 
		 * stop the searching and return the list of target points 
		 */
		combinations_for_target_points(v, w, 1, n, 1, k, p, d, param, current_time, AP_vertex, destination_vehicle, packet, FTQ, global_TPQ);

		if(global_TPQ->delivery_success_probability > 
			param->communication_packet_delivery_probability_threshold)
		{
			break;
		}

		/* otherwise, increase the target point number k and search k target points */
	}

    /* @for debugging */
    //if(packet->id == 33)
	//  printf("GetMultipleTargetPoints_By_PacketTrajectory_For_AP(): packet->id=%d\n", packet->id);
    /******************/

	/* when global_TPQ is empty due to the non-existence of a target point set to satisfy the user-required delivery probability, the last k intersection points of the destination vehicle trajectory makes a target point set as last resort */
	if(global_TPQ->size == 0)
	{
		/* add the last three intersections of the destination vehicle trajectory to the target point set */
		max_number = param->data_forwarding_maximum_target_point_number;

		//pPathNode = destination_vehicle->path_list->prev;
		for(k = n-max_number+1; k <= n; k++)
		{
		    target_point_id = w[k];

		  	//pPathNode = pPathNode->prev;
			//target_point_id = atoi(pPathNode->vertex);
			
			/* initialize queue node */
			memset(&qnode, 0, sizeof(target_point_queue_node_t));
			qnode.target_point_id = target_point_id;
			Enqueue((queue_t*)global_TPQ, (queue_node_t*)&qnode);
		}
		/* set target_point_number to max_number */
		target_point_number = max_number;
	}
	else
	{
		/* set target_point_number to global_TPQ's size */
		target_point_number = global_TPQ->size;
	}

	/* deallocate the memory for vectors w, v, p, and d */
	free(w);
	free(v);
	free(p);
	free(d);

	return target_point_number;
}

int GetTargetPoint_By_HeadingIntersection_For_Carrier(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ //get the id of a target point for carrier vehicle, such as the destination vehicle's heading intersection on its trajectory, used to deliver a packet towards vehicle that is a destination vehicle

  /**********************************************************************/
  static int vehicle_trajectory_expiration_count = 0; //count for vehicle trajectory expiration

  char *target_point = NULL; //target point
  int target_point_id = 0; //target point id that is an intersection
  int new_target_point_id = 0; //new target point id for the packet with the expired vehicle trajectory such that the destination vehicle has already gone out of the trajectory in the packet.
  int path_current_hop = 0; //destination vehicle's current hop at this current time

  /**********************************************************************/
  struct_vehicle_t *destination_vehicle = NULL; //pointer to the destination vehicle for packets
  vehicle_trajectory_queue_t *pTrajectory_Queue = NULL; //pointer to the packet's vehicle trajectory
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to the vehicle trajectory queue node
  vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode = NULL; //pointer to the current trajectory queue node for the edge where the destination vehicle is moving
  double offset_end = 0; //destination vehicle's offset in the current edge on its trajectory for current_time
  int i = 0; //for-loop index
  /**********************************************************************/

#ifdef __DEBUG_LEVEL_AP_PACKET_ARRIVE__
  double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
  double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

  double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
  double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
#endif

  /* check whether this vehicle has packets or not; if there is no packet, return a random target point */
  if(carrier_vehicle->packet_queue->size == 0)
  {
/*     do */
/*     { */
/*       new_target_point_id = smpl_random(1, FTQ->size); */
/*       if(new_target_point_id == carrier_vehicle->target_point_id) */
/* 	continue; */
/*     } while(new_target_point_id < 1 || new_target_point_id > FTQ->size); */

    new_target_point_id = carrier_vehicle->target_point_id;

    return new_target_point_id;
  }

  /** pointer to the destination vehicle for packets that is used to check whether the trajectory reflects the destination's movement well or not */
  destination_vehicle = carrier_vehicle->latest_packet_ptr->dst_vnode; 

  /** set up vehicle trajectory queue pTrajectory_Queue */
  pTrajectory_Queue = &(carrier_vehicle->latest_packet_ptr->vehicle_trajectory); //pointer to the packet's vehicle trajectory

  /** find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory */
  pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset(param, current_time, carrier_vehicle, &path_current_hop, &offset_end);

  /** check whether the destination vehicle has passed out of its trajectory or not */
  if((param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE) && (pCurrent_Trajectory_QNode == NULL))
  { //choose a random target point among the intersections on the road network graph and set its trajectory selection type as static. If this packet meets another vehicle with fresh destination vehicle, it is forwarded with the packet with the fresh destination vehicle trajectory.

    vehicle_trajectory_expiration_count++;

    /*
    //(i) Option 1: Random target point
    do
    {
      new_target_point_id = smpl_random(1, FTQ->size);
      if(new_target_point_id == carrier_vehicle->target_point_id)
	continue;
    } while(new_target_point_id < 1 || new_target_point_id > FTQ->size);
    */

    //(ii) Option 2: Previous target point (this is the same as the Static target point); this seems better than Option 1
    new_target_point_id = carrier_vehicle->target_point_id;

    //(iii) Option 3: Closest access point; this seems worse than Option 1
    //new_target_point_id = 15;

#ifdef __DEBUG_LEVEL_TRAJECTORY_EXPIRATION__
    /*@Note: Reconsider how to deal with the vehicle trajectory expiration! */
    printf("[time=%.2f] Expire %d: GetTargetPoint_By_OptimalIntersection_For_Carrier(): Note: carrier_vehicle(vid=%d) experiences that the trajectory is expired for this packet(seq=%d)!\n", current_time, vehicle_trajectory_expiration_count, carrier_vehicle->id, carrier_vehicle->latest_packet_ptr->seq);
#endif
    
    return new_target_point_id;
  }

  /** determine the target point as the destination vehicle's heading intersection */
  target_point_id = atoi(pCurrent_Trajectory_QNode->graph_pos.enode->head_node);

#ifdef __DEBUG_LEVEL_AP_PACKET_ARRIVE__

    /** compute the travel time duration */
    path_distance = destination_vehicle->edge_length - destination_vehicle->current_pos_in_digraph.offset; //path distance from vehicle's current position to the head node of the vehicle's current edge
    EAD_p = path_distance / destination_vehicle->speed;

    /** get the EDD and EDD_SD for the target point at the intersection having this carrier vehicle */
    VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Carrier(param, target_point_id, carrier_vehicle, FTQ, &EDD_p, &EDD_SD_p);

    /** set EDD_p_for_optimal_target_point and EAD_p_for_optimal_target_point to EDD_p and EAD_p */
    *EDD_p_for_optimal_target_point = EDD_p;
    *EAD_p_for_optimal_target_point = EAD_p;
#endif

  return target_point_id;
}

int GetTargetPoint_By_RandomIntersection_For_Carrier(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ //get the id of a target point for carrier vehicle, such as a random intersection on the destination vehicle trajectory, used to deliver a packet towards vehicle that is a destination vehicle

  /**********************************************************************/
  static int vehicle_trajectory_expiration_count = 0; //count for vehicle trajectory expiration

  char *target_point = NULL; //target point
  int target_point_id = 0; //target point id that is an intersection
  int new_target_point_id = 0; //new target point id for the packet with the expired vehicle trajectory such that the destination vehicle has already gone out of the trajectory in the packet.
  int random_target_point_index = 0; //index for a random target point on the destination vehicle trajectory 

  int path_node_number = 0; //the number of nodes on the path to check for a target point, including the tail node of its current edge
  int path_current_hop = 0; //destination vehicle's current hop at this current time

  /**********************************************************************/
  struct_vehicle_t *destination_vehicle = NULL; //pointer to the destination vehicle for packets
  vehicle_trajectory_queue_t *pTrajectory_Queue = NULL; //pointer to the packet's vehicle trajectory
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to the vehicle trajectory queue node
  vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode = NULL; //pointer to the current trajectory queue node for the edge where the destination vehicle is moving
  double offset_end = 0; //destination vehicle's offset in the current edge on its trajectory for current_time
  int i = 0; //for-loop index
  /**********************************************************************/

  /* check whether this vehicle has packets or not; if there is no packet, return a random target point */
  if(carrier_vehicle->packet_queue->size == 0)
  {
/*     do */
/*     { */
/*       new_target_point_id = smpl_random(1, FTQ->size); */
/*       if(new_target_point_id == carrier_vehicle->target_point_id) */
/* 	continue; */
/*     } while(new_target_point_id < 1 || new_target_point_id > FTQ->size); */

    new_target_point_id = carrier_vehicle->target_point_id;
    return new_target_point_id;
  }

  /** pointer to the destination vehicle for packets that is used to check whether the trajectory reflects the destination's movement well or not */
  destination_vehicle = carrier_vehicle->latest_packet_ptr->dst_vnode; 

  /** set up vehicle trajectory queue pTrajectory_Queue */
  pTrajectory_Queue = &(carrier_vehicle->latest_packet_ptr->vehicle_trajectory); //pointer to the packet's vehicle trajectory

#ifdef __DEBUG_LEVEL_AP_PACKET_ARRIVE__
  double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
  double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

  double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
  double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
#endif

  /** find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory */
  pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset(param, current_time, carrier_vehicle, &path_current_hop, &offset_end);

  /** check whether the destination vehicle has passed out of its trajectory or not */
  if((param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE) && (pCurrent_Trajectory_QNode == NULL))
  { //choose a random target point among the intersections on the road network graph and set its trajectory selection type as static. If this packet meets another vehicle with fresh destination vehicle, it is forwarded with the packet with the fresh destination vehicle trajectory.

    vehicle_trajectory_expiration_count++;

    /*
    //(i) Option 1: Random target point
    do
    {
      new_target_point_id = smpl_random(1, FTQ->size);
      if(new_target_point_id == carrier_vehicle->target_point_id)
	continue;
    } while(new_target_point_id < 1 || new_target_point_id > FTQ->size);
    */

    //(ii) Option 2: Previous target point (this is the same as the Static target point); this seems better than Option 1
    new_target_point_id = carrier_vehicle->target_point_id;

    //(iii) Option 3: Closest access point; this seems worse than Option 1
    //new_target_point_id = 15;

#ifdef __DEBUG_LEVEL_TRAJECTORY_EXPIRATION__
    /*@Note: Reconsider how to deal with the vehicle trajectory expiration! */
    printf("[time=%.2f] Expire %d: GetTargetPoint_By_OptimalIntersection_For_Carrier(): Note: carrier_vehicle(vid=%d) experiences that the trajectory is expired for this packet(seq=%d)!\n", current_time, vehicle_trajectory_expiration_count, carrier_vehicle->id, carrier_vehicle->latest_packet_ptr->seq);
#endif
    
    return new_target_point_id;
  }

  /** compute the number of path nodes on the trajectory to check for a target point */
  if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE)
  {
    path_node_number = pTrajectory_Queue->size  - path_current_hop; //the number of nodes on the path to check for an optimap target point, including the tail node of its current edge
  }
  else if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
  {
    path_node_number = pTrajectory_Queue->size; //the number of nodes on the path to check for an optimap target point, including the tail node of its current edge

  }
  else
  {
    printf(" GetTargetPoint_By_RandomIntersection_For_Carrier(): trajectory_length_type(%d) is unknown!\n", param->vehicle_vanet_vehicle_trajectory_length_type);
    exit(1);    
  }

  /** select a random index from 0 to path_node_number-1 for a target point */
  do
  {
    random_target_point_index = smpl_random(0, path_node_number-1);
  } while(random_target_point_index < 0 || random_target_point_index > path_node_number-1);

  /** search pPathNode corresponding to random_target_point **/
  pTrajectory_QNode = pCurrent_Trajectory_QNode; //Note that the first point to check is the current trajectory queue node that is the tail node of the current edge where the destination vehicle is moving
  for(i = 0; i < random_target_point_index; i++)
  {
    pTrajectory_QNode =  pTrajectory_QNode->next; // pTrajectory_QNode points to the tail node of an edge

    /* For infinite trajectory length type, if pTrajectory_QNode is the queue head, then let pTrajectory_QNode point to head->next */
    if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
    {
      if(pTrajectory_QNode == &(pTrajectory_Queue->head))
        pTrajectory_QNode = pTrajectory_QNode->next;
    }
  }

  target_point = pTrajectory_QNode->graph_pos.enode->tail_node;
  target_point_id = atoi(target_point);

#ifdef __DEBUG_LEVEL_AP_PACKET_ARRIVE__

    /** compute the travel time duration */
    /* compute the path distance from the destination vehicle's current position to the target point */
    path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory(current_time, pTrajectory_Queue, pCurrent_Trajectory_QNode, offset_end, target_point);
    EAD_p = path_distance / destination_vehicle->speed;

    /** get the EDD and EDD_SD for the target point at the intersection having this carrier vehicle */
    VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Carrier(param, target_point_id, carrier_vehicle, FTQ, &EDD_p, &EDD_SD_p);

    /** set EDD_p_for_optimal_target_point and EAD_p_for_optimal_target_point to EDD_p and EAD_p */
    *EDD_p_for_optimal_target_point = EDD_p;
    *EAD_p_for_optimal_target_point = EAD_p;
#endif

  return target_point_id;
}

int GetTargetPoint_By_OptimalIntersection_For_Carrier(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ // get the id of an optimal target point for carrier vehicle such as intersection point or arbitrary point on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle

  static int vehicle_trajectory_expiration_count = 0; //count for vehicle trajectory expiration
  int remaining_path_hop_count = 0; //the number of nodes consisting of the path corresponding to the vehicle trajectory from the head node of its current edge to the destination node (i.e., destination position).
  int path_node_number = 0; //the number of nodes on the path to check for an optimal target point, including the tail node of its current edge
  int path_current_hop = 0; //destination vehicle's current hop at this current time

  char *target_point = NULL; //pointer to a vertex corresponding to a target point candidate
  double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
  double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

  double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
  double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
  double difference = 0 ;// absolute difference of EDD_p and EAD_p
  double min_difference = INF; //minimum absolute difference of EDD_p and EAD_p
  char min_difference_target_point[NAME_SIZE]; //target point with min difference
  int i = 0; //for-loop index
  char current_edge_tail_node[NAME_SIZE]; //tail node of the current edge pointed by vehicle->path_ptr

  /**********************************************************************/
  int target_point_id = 0; //target point id that is an intersection
  int new_target_point_id = 0; //new target point id for the packet with the expired vehicle trajectory such that the destination vehicle has already gone out of the trajectory in the packet.

  /**********************************************************************/
  struct_vehicle_t *destination_vehicle = NULL; //pointer to the destination vehicle for packets
  vehicle_trajectory_queue_t *pTrajectory_Queue = NULL; //pointer to the packet's vehicle trajectory
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to the vehicle trajectory queue node
  vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode = NULL; //pointer to the current trajectory queue node for the edge where the destination vehicle is moving
  double offset_end = 0; //destination vehicle's offset in the current edge on its trajectory for current_time
  /**********************************************************************/

  /*@for debugging */
  //if(carrier_vehicle->id == 95 && current_time > 5374)
  //  printf("GetTargetPoint_By_OptimalIntersection_For_Carrier(): time=%.2f, vid=%d\n", current_time, carrier_vehicle->id);
  /***********/

  /* check whether this vehicle has packets or not; if there is no packet, return a random target point */
  if(carrier_vehicle->packet_queue->size == 0)
  {
/*     do */
/*     { */
/*       new_target_point_id = smpl_random(1, FTQ->size); */
/*       if(new_target_point_id == carrier_vehicle->target_point_id) */
/* 	continue; */
/*     } while(new_target_point_id < 1 || new_target_point_id > FTQ->size); */

    new_target_point_id = carrier_vehicle->target_point_id;
    return new_target_point_id;
  }

  /** pointer to the destination vehicle for packets that is used to check whether the trajectory reflects the destination's movement well or not */
  destination_vehicle = carrier_vehicle->latest_packet_ptr->dst_vnode; 

  /** set up vehicle trajectory queue pTrajectory_Queue */
  pTrajectory_Queue = &(carrier_vehicle->latest_packet_ptr->vehicle_trajectory); //pointer to the packet's vehicle trajectory

  /** find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory */
  pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset(param, current_time, carrier_vehicle, &path_current_hop, &offset_end);

  /** check whether the destination vehicle has passed out of its trajectory or not */
  if((param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE) && (pCurrent_Trajectory_QNode == NULL))
  { //choose a random target point among the intersections on the road network graph and set its trajectory selection type as static. If this packet meets another vehicle with fresh destination vehicle, it is forwarded with the packet with the fresh destination vehicle trajectory.

    vehicle_trajectory_expiration_count++;

    /*
    //(i) Option 1: Random target point
    do
    {
      new_target_point_id = smpl_random(1, FTQ->size);
      if(new_target_point_id == carrier_vehicle->target_point_id)
	continue;
    } while(new_target_point_id < 1 || new_target_point_id > FTQ->size);
    */

    //(ii) Option 2: Previous target point (this is the same as the Static target point); this seems better than Option 1
    new_target_point_id = carrier_vehicle->target_point_id;

    //(iii) Option 3: Closest access point; this seems worse than Option 1
    //new_target_point_id = 15;

#ifdef __DEBUG_LEVEL_TRAJECTORY_EXPIRATION__
    /*@Note: Reconsider how to deal with the vehicle trajectory expiration! */
    printf("[time=%.2f] Expire %d: GetTargetPoint_By_OptimalIntersection_For_Carrier(): Note: carrier_vehicle(vid=%d) experiences that the trajectory is expired for this packet(seq=%d)!\n", current_time, vehicle_trajectory_expiration_count, carrier_vehicle->id, carrier_vehicle->latest_packet_ptr->seq);
#endif
    
    return new_target_point_id;
  }

  /** compute the number of path nodes on the trajectory to check for a target point */
  if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE)
  {
    path_node_number = pTrajectory_Queue->size  - path_current_hop; //the number of nodes on the path to check for an optimap target point, including the tail node of its current edge
  }
  else if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
  {
    path_node_number = pTrajectory_Queue->size; //the number of nodes on the path to check for an optimap target point, including the tail node of its current edge

  }
  else
  {
    printf(" GetTargetPoint_By_OptimalIntersection_For_Carrier(): trajectory_length_type(%d) is unknown!\n", param->vehicle_vanet_vehicle_trajectory_length_type);
    exit(1);    
  }

  /** search an optimal target point p such that |EDD_p - EAD_p| is minimum. **/

  /* initialize double *EDD_p_for_optimal_target_point and double *EDD_p_for_optimal_target_point */
  *EDD_p_for_optimal_target_point = 0;
  *EAD_p_for_optimal_target_point = 0;

  /* copy the tail node of the destination vehicle's current edge to current_edge_tail_node */
  strcpy(current_edge_tail_node, pCurrent_Trajectory_QNode->graph_pos.enode->tail_node);

  /* set pTrajectory_QNode to the first queue node to check */
  pTrajectory_QNode = pCurrent_Trajectory_QNode; //Note that the first point to check is the tail node of the current edge

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_CARRIER__
  printf("GetTargetPoint_By_OptimalIntersection_For_Carrier():[Carrier Vehicle] carrier_vehicle=%d, graph_pos=(%s,%s:%.2f)\n", carrier_vehicle->id, carrier_vehicle->current_pos_in_digraph.enode->tail_node, carrier_vehicle->current_pos_in_digraph.enode->head_node, carrier_vehicle->current_pos_in_digraph.offset);
  printf("GetTargetPoint_By_OptimalIntersection_For_Carrier():[Trajectory]  vehicle=%d, graph_pos=(%s,%s:%.2f)\n", pTrajectory_Queue->vnode->id, pCurrent_Trajectory_QNode->graph_pos.enode->tail_node, pCurrent_Trajectory_QNode->graph_pos.enode->head_node, offset_end);
  printf("GetTargetPoint_By_OptimalIntersection_For_Carrier():[Destination] vehicle=%d, graph_pos=(%s,%s:%.2f)\n", destination_vehicle->id, destination_vehicle->current_pos_in_digraph.enode->tail_node, destination_vehicle->current_pos_in_digraph.enode->head_node, destination_vehicle->current_pos_in_digraph.offset);
#endif

  for(i = 0; i < path_node_number; i++)
  {
    /* For infinite trajectory length type, if pTrajectory_QNode is the queue head, then let pTrajectory_QNode point to head->next */
    if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
    {
      if(pTrajectory_QNode == &(pTrajectory_Queue->head))
        pTrajectory_QNode = pTrajectory_QNode->next;
    }

    target_point =  pTrajectory_QNode->graph_pos.enode->tail_node; //target_point is the tail node of an edge

    /* compute the path distance from the destination vehicle's current position to the target point */
    path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory(current_time, pTrajectory_Queue, pCurrent_Trajectory_QNode, offset_end, target_point);

    /* compute the travel time duration */
    EAD_p = path_distance / param->vehicle_speed; //param->vehicle_speed should be used since it is average vehicle speed
    //EAD_p = path_distance / pTrajectory_Queue->vehicle_speed;
    //EAD_p = path_distance / destination_vehicle->speed;

    /** compute EDD_p for a target point p */    
    /* compute the EDD and EDD_SD for the target point at the intersection having this AP */
    target_point_id = atoi(target_point);

    VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Carrier(param, target_point_id, carrier_vehicle, FTQ, &EDD_p, &EDD_SD_p);

    /* update min_difference and min_difference_target_point */
    difference =  fabs(EDD_p - EAD_p);

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_CARRIER__
    printf("GetTargetPoint_By_OptimalIntersection_For_Carrier(): target point=%s, difference=%.2f\n", target_point, difference);
#endif 

    if(min_difference > difference)
    {
      min_difference = difference;
      strcpy(min_difference_target_point, target_point);
      *EDD_p_for_optimal_target_point = EDD_p;
      *EAD_p_for_optimal_target_point = EAD_p;
    }

    pTrajectory_QNode = pTrajectory_QNode->next; //pTrajectory_QNode points to the next node on the trajectory
  }

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_CARRIER__
  printf("GetTargetPoint_By_OptimalIntersection_For_Carrier(): optimal target point=%s, difference=%.2f\n", min_difference_target_point, min_difference);
  fgetc(stdin);
#endif

  /* convert min_differnce_target_point into target_point_id */
  target_point_id = atoi(min_difference_target_point);

  return target_point_id;
}

int GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ // get the id of a target point p for carrier vehicle such as intersection point on the destination vehicle's trajectory where p is geographically closest to the destination vehicle such that EDD_p <= EAD_p 

  static int vehicle_trajectory_expiration_count = 0; //count for vehicle trajectory expiration
  int remaining_path_hop_count = 0; //the number of nodes consisting of the path corresponding to the vehicle trajectory from the head node of its current edge to the destination node (i.e., destination position).
  int path_node_number = 0; //the number of nodes on the path to check for an optimal target point, including the tail node of its current edge
  int path_current_hop = 0; //destination vehicle's current hop at this current time

  char *target_point = NULL; //pointer to a vertex corresponding to a target point candidate
  double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
  double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

  double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
  double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
  double difference = 0 ;// absolute difference of EDD_p and EAD_p
  int i = 0; //for-loop index
  char current_edge_tail_node[NAME_SIZE]; //tail node of the current edge pointed by vehicle->path_ptr

  /**********************************************************************/
  int target_point_id = 0; //target point candiate that is an intersection
  int new_target_point_id = 0; //new target point id for the packet with the expired vehicle trajectory such that the destination vehicle has already gone out of the trajectory in the packet.

  /**********************************************************************/
  struct_vehicle_t *destination_vehicle = NULL; //pointer to the destination vehicle for packets
  vehicle_trajectory_queue_t *pTrajectory_Queue = NULL; //pointer to the packet's vehicle trajectory
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to the vehicle trajectory queue node
  vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode = NULL; //pointer to the current trajectory queue node for the edge where the destination vehicle is moving
  double offset_end = 0; //destination vehicle's offset in the current edge on its trajectory for current_time
  /**********************************************************************/

  /*@for debugging */
  //if(carrier_vehicle->id == 95 && current_time > 5374)
  //  printf("GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier(): time=%.2f, vid=%d\n", current_time, carrier_vehicle->id);
  /***********/

  /* check whether this vehicle has packets or not; if there is no packet, return a random target point */
  if(carrier_vehicle->packet_queue->size == 0)
  {
/*     do */
/*     { */
/*       new_target_point_id = smpl_random(1, FTQ->size); */
/*       if(new_target_point_id == carrier_vehicle->target_point_id) */
/* 	continue; */
/*     } while(new_target_point_id < 1 || new_target_point_id > FTQ->size); */

    new_target_point_id = carrier_vehicle->target_point_id;
    return new_target_point_id;
  }

  /** pointer to the destination vehicle for packets that is used to check whether the trajectory reflects the destination's movement well or not */
  destination_vehicle = carrier_vehicle->latest_packet_ptr->dst_vnode; 

  /** set up vehicle trajectory queue pTrajectory_Queue */
  pTrajectory_Queue = &(carrier_vehicle->latest_packet_ptr->vehicle_trajectory); //pointer to the packet's vehicle trajectory

  /** find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory */
  pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset(param, current_time, carrier_vehicle, &path_current_hop, &offset_end);

  /** check whether the destination vehicle has passed out of its trajectory or not */
  if((param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE) && (pCurrent_Trajectory_QNode == NULL))
  { //choose a random target point among the intersections on the road network graph and set its trajectory selection type as static. If this packet meets another vehicle with fresh destination vehicle, it is forwarded with the packet with the fresh destination vehicle trajectory.

    vehicle_trajectory_expiration_count++;

    /*
    //(i) Option 1: Random target point
    do
    {
      new_target_point_id = smpl_random(1, FTQ->size);
      if(new_target_point_id == carrier_vehicle->target_point_id)
	continue;
    } while(new_target_point_id < 1 || new_target_point_id > FTQ->size);
    */

    //(ii) Option 2: Previous target point (this is the same as the Static target point); this seems better than Option 1
    new_target_point_id = carrier_vehicle->target_point_id;

    //(iii) Option 3: Closest access point; this seems worse than Option 1
    //new_target_point_id = 15;

#ifdef __DEBUG_LEVEL_TRAJECTORY_EXPIRATION__
    /*@Note: Reconsider how to deal with the vehicle trajectory expiration! */
    printf("[time=%.2f] Expire %d: GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier(): Note: carrier_vehicle(vid=%d) experiences that the trajectory is expired for this packet(seq=%d)!\n", current_time, vehicle_trajectory_expiration_count, carrier_vehicle->id, carrier_vehicle->latest_packet_ptr->seq);
#endif
    
    return new_target_point_id;
  }

  /** compute the number of path nodes on the trajectory to check for a target point */
  if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE)
  {
    path_node_number = pTrajectory_Queue->size  - path_current_hop; //the number of nodes on the path to check for an optimap target point, including the tail node of its current edge
  }
  else if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
  {
    path_node_number = pTrajectory_Queue->size; //the number of nodes on the path to check for an optimap target point, including the tail node of its current edge

  }
  else
  {
    printf(" GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier(): trajectory_length_type(%d) is unknown!\n", param->vehicle_vanet_vehicle_trajectory_length_type);
    exit(1);    
  }

  /** search a target point p where p is geographically closest to the destination vehicle such that EDD_p <= EAD_p **/

  /* initialize double *EDD_p_for_optimal_target_point and double *EDD_p_for_optimal_target_point */
  *EDD_p_for_optimal_target_point = 0;
  *EAD_p_for_optimal_target_point = 0;

  /* copy the tail node of the destination vehicle's current edge to current_edge_tail_node */
  strcpy(current_edge_tail_node, pCurrent_Trajectory_QNode->graph_pos.enode->tail_node);

  /* set pTrajectory_QNode to the first queue node to check */
  pTrajectory_QNode = pCurrent_Trajectory_QNode; //Note that the first point to check is the tail node of the current edge

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_CARRIER__
  printf("GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier():[Carrier Vehicle] carrier_vehicle=%d, graph_pos=(%s,%s:%.2f)\n", carrier_vehicle->id, carrier_vehicle->current_pos_in_digraph.enode->tail_node, carrier_vehicle->current_pos_in_digraph.enode->head_node, carrier_vehicle->current_pos_in_digraph.offset);
  printf("GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier():[Trajectory]  vehicle=%d, graph_pos=(%s,%s:%.2f)\n", pTrajectory_Queue->vnode->id, pCurrent_Trajectory_QNode->graph_pos.enode->tail_node, pCurrent_Trajectory_QNode->graph_pos.enode->head_node, offset_end);
  printf("GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier():[Destination] vehicle=%d, graph_pos=(%s,%s:%.2f)\n", destination_vehicle->id, destination_vehicle->current_pos_in_digraph.enode->tail_node, destination_vehicle->current_pos_in_digraph.enode->head_node, destination_vehicle->current_pos_in_digraph.offset);
#endif

  for(i = 0; i < path_node_number; i++)
  {
    /* For infinite trajectory length type, if pTrajectory_QNode is the queue head, then let pTrajectory_QNode point to head->next */
    if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
    {
      if(pTrajectory_QNode == &(pTrajectory_Queue->head))
        pTrajectory_QNode = pTrajectory_QNode->next;
    }

    target_point =  pTrajectory_QNode->graph_pos.enode->tail_node; //target_point is the tail node of an edge

    /* compute the path distance from the destination vehicle's current position to the target point */
    path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory(current_time, pTrajectory_Queue, pCurrent_Trajectory_QNode, offset_end, target_point);

    /* compute the travel time duration */
    EAD_p = path_distance / param->vehicle_speed; //param->vehicle_speed should be used since it is average vehicle speed
    //EAD_p = path_distance / pTrajectory_Queue->vehicle_speed;
    //EAD_p = path_distance / destination_vehicle->speed;

    /** compute EDD_p for a target point p */    
    /* compute the EDD and EDD_SD for the target point at the intersection having this AP */
    target_point_id = atoi(target_point);

    VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Carrier(param, target_point_id, carrier_vehicle, FTQ, &EDD_p, &EDD_SD_p);

    /* check whether the difference of EDD_p - EAD_p is non-positive or not */
    difference =  EDD_p - EAD_p;

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_CARRIER__
    printf("GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier(): target point=%s, difference=%.2f\n", target_point, difference);
#endif 

    if(difference <= 0)
    {
      new_target_point_id = target_point_id;
      *EDD_p_for_optimal_target_point = EDD_p;
      *EAD_p_for_optimal_target_point = EAD_p;
      break;
    }

    pTrajectory_QNode = pTrajectory_QNode->next; //pTrajectory_QNode points to the next node on the trajectory
  }

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_CARRIER__
  printf("GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier(): optimal target point=%s, difference=%.2f\n", new_target_point_id, difference);
  fgetc(stdin);
#endif

  /** check whether there exists no intersection p such that EDD_p <= EAD_p or not; if so, find a target point p such that the point p is the geographically closest point on the destination vehicle's trajectory from the carrier's current position */
  if(new_target_point_id == 0)
  {
      //new_target_point_id = GetTargetPoint_By_OptimalIntersection_For_Carrier(param, current_time, carrier_vehicle, FTQ, EDD_p_for_optimal_target_point, EAD_p_for_optimal_target_point);

      new_target_point_id = GetTargetPoint_By_ClosestIntersection_For_Carrier(param, current_time, carrier_vehicle, FTQ, EDD_p_for_optimal_target_point, EAD_p_for_optimal_target_point);
  }

  return new_target_point_id;
}

int GetTargetPoint_By_ClosestIntersection_For_Carrier(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ // get the id of a target point p for carrier vehicle such that the target point p is the geographically closest point on the destination vehicle's trajectory from the carrier's current position 

  static int vehicle_trajectory_expiration_count = 0; //count for vehicle trajectory expiration
  int remaining_path_hop_count = 0; //the number of nodes consisting of the path corresponding to the vehicle trajectory from the head node of its current edge to the destination node (i.e., destination position).
  int path_node_number = 0; //the number of nodes on the path to check for an optimal target point, including the tail node of its current edge
  int path_current_hop = 0; //destination vehicle's current hop at this current time

  char *target_point = NULL; //pointer to a vertex corresponding to a target point candidate
  double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
  double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

  double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
  double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
  double difference = 0 ;// absolute difference of EDD_p and EAD_p
  int i = 0; //for-loop index
  char current_edge_tail_node[NAME_SIZE]; //tail node of the current edge pointed by vehicle->path_ptr

  double distance = 0; //distance from the carrier's heading intersection to an intersection on the destination vehicle's trajectory
  double minimum_distance = INF; //minimum distance from the carrier's heading intersection to the destination vehicle's trajectory
  int minimum_distance_intersection_id = 0; //the intersection id with the minimum distance

  /**********************************************************************/
  int target_point_id = 0; //target point candiate that is an intersection
  int new_target_point_id = 0; //new target point id for the packet with the expired vehicle trajectory such that the destination vehicle has already gone out of the trajectory in the packet.

  /**********************************************************************/
  struct_vehicle_t *destination_vehicle = NULL; //pointer to the destination vehicle for packets
  vehicle_trajectory_queue_t *pTrajectory_Queue = NULL; //pointer to the packet's vehicle trajectory
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to the vehicle trajectory queue node
  vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode = NULL; //pointer to the current trajectory queue node for the edge where the destination vehicle is moving
  double offset_end = 0; //destination vehicle's offset in the current edge on its trajectory for current_time
  /**********************************************************************/

  /*@for debugging */
  //if(carrier_vehicle->id == 95 && current_time > 5374)
  //  printf("GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier(): time=%.2f, vid=%d\n", current_time, carrier_vehicle->id);
  /***********/

  /* check whether this vehicle has packets or not; if there is no packet, return a random target point */
  if(carrier_vehicle->packet_queue->size == 0)
  {
/*     do */
/*     { */
/*       new_target_point_id = smpl_random(1, FTQ->size); */
/*       if(new_target_point_id == carrier_vehicle->target_point_id) */
/* 	continue; */
/*     } while(new_target_point_id < 1 || new_target_point_id > FTQ->size); */

    new_target_point_id = carrier_vehicle->target_point_id;
    return new_target_point_id;
  }

  /** pointer to the destination vehicle for packets that is used to check whether the trajectory reflects the destination's movement well or not */
  destination_vehicle = carrier_vehicle->latest_packet_ptr->dst_vnode; 

  /** set up vehicle trajectory queue pTrajectory_Queue */
  pTrajectory_Queue = &(carrier_vehicle->latest_packet_ptr->vehicle_trajectory); //pointer to the packet's vehicle trajectory

  /** find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory */
  pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset(param, current_time, carrier_vehicle, &path_current_hop, &offset_end);

  /** check whether the destination vehicle has passed out of its trajectory or not */
  if((param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE) && (pCurrent_Trajectory_QNode == NULL))
  { //choose a random target point among the intersections on the road network graph and set its trajectory selection type as static. If this packet meets another vehicle with fresh destination vehicle, it is forwarded with the packet with the fresh destination vehicle trajectory.

    vehicle_trajectory_expiration_count++;

    /*
    //(i) Option 1: Random target point
    do
    {
      new_target_point_id = smpl_random(1, FTQ->size);
      if(new_target_point_id == carrier_vehicle->target_point_id)
	continue;
    } while(new_target_point_id < 1 || new_target_point_id > FTQ->size);
    */

    //(ii) Option 2: Previous target point (this is the same as the Static target point); this seems better than Option 1
    new_target_point_id = carrier_vehicle->target_point_id;

    //(iii) Option 3: Closest access point; this seems worse than Option 1
    //new_target_point_id = 15;

#ifdef __DEBUG_LEVEL_TRAJECTORY_EXPIRATION__
    /*@Note: Reconsider how to deal with the vehicle trajectory expiration! */
    printf("[time=%.2f] Expire %d: GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier(): Note: carrier_vehicle(vid=%d) experiences that the trajectory is expired for this packet(seq=%d)!\n", current_time, vehicle_trajectory_expiration_count, carrier_vehicle->id, carrier_vehicle->latest_packet_ptr->seq);
#endif
    
    return new_target_point_id;
  }

  /** compute the number of path nodes on the trajectory to check for a target point */
  if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE)
  {
    path_node_number = pTrajectory_Queue->size  - path_current_hop; //the number of nodes on the path to check for an optimap target point, including the tail node of its current edge
  }
  else if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
  {
    path_node_number = pTrajectory_Queue->size; //the number of nodes on the path to check for an optimap target point, including the tail node of its current edge

  }
  else
  {
    printf(" GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier(): trajectory_length_type(%d) is unknown!\n", param->vehicle_vanet_vehicle_trajectory_length_type);
    exit(1);    
  }

  /** search a target point p where p is geographically closest to the destination vehicle such that EDD_p <= EAD_p **/

  /* initialize double *EDD_p_for_optimal_target_point and double *EDD_p_for_optimal_target_point */
  *EDD_p_for_optimal_target_point = 0;
  *EAD_p_for_optimal_target_point = 0;

  /* copy the tail node of the destination vehicle's current edge to current_edge_tail_node */
  strcpy(current_edge_tail_node, pCurrent_Trajectory_QNode->graph_pos.enode->tail_node);

  /* set pTrajectory_QNode to the first queue node to check */
  pTrajectory_QNode = pCurrent_Trajectory_QNode; //Note that the first point to check is the tail node of the current edge

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_CARRIER__
  printf("GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier():[Carrier Vehicle] carrier_vehicle=%d, graph_pos=(%s,%s:%.2f)\n", carrier_vehicle->id, carrier_vehicle->current_pos_in_digraph.enode->tail_node, carrier_vehicle->current_pos_in_digraph.enode->head_node, carrier_vehicle->current_pos_in_digraph.offset);
  printf("GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier():[Trajectory]  vehicle=%d, graph_pos=(%s,%s:%.2f)\n", pTrajectory_Queue->vnode->id, pCurrent_Trajectory_QNode->graph_pos.enode->tail_node, pCurrent_Trajectory_QNode->graph_pos.enode->head_node, offset_end);
  printf("GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier():[Destination] vehicle=%d, graph_pos=(%s,%s:%.2f)\n", destination_vehicle->id, destination_vehicle->current_pos_in_digraph.enode->tail_node, destination_vehicle->current_pos_in_digraph.enode->head_node, destination_vehicle->current_pos_in_digraph.offset);
#endif

  for(i = 0; i < path_node_number; i++)
  {
    /* For infinite trajectory length type, if pTrajectory_QNode is the queue head, then let pTrajectory_QNode point to head->next */
    if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
    {
      if(pTrajectory_QNode == &(pTrajectory_Queue->head))
        pTrajectory_QNode = pTrajectory_QNode->next;
    }

    target_point =  pTrajectory_QNode->graph_pos.enode->tail_node; //target_point is the tail node of an edge

    /* compute the path distance from the destination vehicle's current position to the target point */
    path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory(current_time, pTrajectory_Queue, pCurrent_Trajectory_QNode, offset_end, target_point);

    /* compute the travel time duration */
    EAD_p = path_distance / param->vehicle_speed; //param->vehicle_speed should be used since it is average vehicle speed
    //EAD_p = path_distance / pTrajectory_Queue->vehicle_speed;
    //EAD_p = path_distance / destination_vehicle->speed;

    /** compute EDD_p for a target point p */    
    /* compute the EDD and EDD_SD for the target point at the intersection having this AP */
    target_point_id = atoi(target_point);

    VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Carrier(param, target_point_id, carrier_vehicle, FTQ, &EDD_p, &EDD_SD_p);

    /* check whether the difference of EDD_p - EAD_p is non-positive or not */
    difference =  EDD_p - EAD_p;

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_CARRIER__
    printf("GetTargetPoint_By_PacketEarliestArrivingIntersection_For_Carrier(): target point=%s, difference=%.2f\n", target_point, difference);
#endif 

    /*
    if(difference <= 0)
    {
      new_target_point_id = target_point_id;
      *EDD_p_for_optimal_target_point = EDD_p;
      *EAD_p_for_optimal_target_point = EAD_p;
      break;
    }
    */

    pTrajectory_QNode = pTrajectory_QNode->next; //pTrajectory_QNode points to the next node on the trajectory
  }

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_CARRIER__
  printf("GetTargetPoint_By_ClosestIntersection_For_Carrier(): optimal target point=%s, distance=%.2f\n", new_target_point_id, distance);
  fgetc(stdin);
#endif

  return new_target_point_id;
}

int GetTargetPoint_By_PacketTrajectory_For_Carrier(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ //With the packet trajectory from the carrier to the destination, get the id of an optimal target point for carrier vehicle such as intersection point or arbitrary point on the vehicle trajectory used to deliver a packet towards vehicle that is a destination vehicle

  static int vehicle_trajectory_expiration_count = 0; //count for vehicle trajectory expiration
  int remaining_path_hop_count = 0; //the number of nodes consisting of the path corresponding to the vehicle trajectory from the head node of its current edge to the destination node (i.e., destination position).
  int path_node_number = 0; //the number of nodes on the path to check for an optimal target point, including the tail node of its current edge
  int path_current_hop = 0; //destination vehicle's current hop at this current time

  char *target_point = NULL; //pointer to a vertex corresponding to a target point candidate
  double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
  double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

  double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
  double path_travel_time = 0; //path travel time
  double path_travel_time_deviation = 0; //path travel time deviation

  double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
  double EAD_VAR_p = 0; //EAD_VAR_p is the variance of the destination vehicle's arrival delay to target point p
  double EAD_SD_p = 0; //EAD_SD_p is the standard deviation of the destination vehicle's arrival delay to target point p

  double difference = 0 ;// absolute difference of EDD_p and EAD_p
  double min_difference = INF; //minimum absolute difference of EDD_p and EAD_p
  char min_difference_target_point[NAME_SIZE]; //target point with min difference
  int i = 0; //for-loop index
  char current_edge_tail_node[NAME_SIZE]; //tail node of the current edge pointed by vehicle->path_ptr

  /**********************************************************************/
  int target_point_id = 0; //target point id that is an intersection
  int new_target_point_id = 0; //new target point id for the packet with the expired vehicle trajectory such that the destination vehicle has already gone out of the trajectory in the packet.

  /**********************************************************************/
  struct_vehicle_t *destination_vehicle = NULL; //pointer to the destination vehicle for packets
  vehicle_trajectory_queue_t *pTrajectory_Queue = NULL; //pointer to the packet's vehicle trajectory
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to the vehicle trajectory queue node
  vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode = NULL; //pointer to the current trajectory queue node for the edge where the destination vehicle is moving
  double offset_end = 0; //destination vehicle's offset in the current edge on its trajectory for current_time
  /**********************************************************************/

  /*@for debugging */
  //if(carrier_vehicle->id == 95 && current_time > 5374)
  //  printf("GetTargetPoint_By_PacketTrajectory_For_Carrier(): time=%.2f, vid=%d\n", current_time, carrier_vehicle->id);
  /***********/

  /* check whether this vehicle has packets or not; if there is no packet, return a random target point */
  if(carrier_vehicle->packet_queue->size == 0)
  {
/*     do */
/*     { */
/*       new_target_point_id = smpl_random(1, FTQ->size); */
/*       if(new_target_point_id == carrier_vehicle->target_point_id) */
/* 	continue; */
/*     } while(new_target_point_id < 1 || new_target_point_id > FTQ->size); */

    new_target_point_id = carrier_vehicle->target_point_id;
    return new_target_point_id;
  }

  /** pointer to the destination vehicle for packets that is used to check whether the trajectory reflects the destination's movement well or not */
  destination_vehicle = carrier_vehicle->latest_packet_ptr->dst_vnode; 

  /** set up vehicle trajectory queue pTrajectory_Queue */
  pTrajectory_Queue = &(carrier_vehicle->latest_packet_ptr->vehicle_trajectory); //pointer to the packet's vehicle trajectory

  /** find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory */
  pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset(param, current_time, carrier_vehicle, &path_current_hop, &offset_end);

  /** check whether the destination vehicle has passed out of its trajectory or not */
  if((param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE) && (pCurrent_Trajectory_QNode == NULL))
  { //choose a random target point among the intersections on the road network graph and set its trajectory selection type as static. If this packet meets another vehicle with fresh destination vehicle, it is forwarded with the packet with the fresh destination vehicle trajectory.

    vehicle_trajectory_expiration_count++;

    /*
    //(i) Option 1: Random target point
    do
    {
      new_target_point_id = smpl_random(1, FTQ->size);
      if(new_target_point_id == carrier_vehicle->target_point_id)
	continue;
    } while(new_target_point_id < 1 || new_target_point_id > FTQ->size);
    */

    //(ii) Option 2: Previous target point (this is the same as the Static target point); this seems better than Option 1
    new_target_point_id = carrier_vehicle->target_point_id;

    //(iii) Option 3: Closest access point; this seems worse than Option 1
    //new_target_point_id = 15;

#ifdef __DEBUG_LEVEL_TRAJECTORY_EXPIRATION__
    /*@Note: Reconsider how to deal with the vehicle trajectory expiration! */
    printf("[time=%.2f] Expire %d: GetTargetPoint_By_OptimalIntersection_For_Carrier(): Note: carrier_vehicle(vid=%d) experiences that the trajectory is expired for this packet(seq=%d)!\n", current_time, vehicle_trajectory_expiration_count, carrier_vehicle->id, carrier_vehicle->latest_packet_ptr->seq);
#endif
    
    return new_target_point_id;
  }

  /** compute the number of path nodes on the trajectory to check for a target point */
  if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE)
  {
    path_node_number = pTrajectory_Queue->size  - path_current_hop; //the number of nodes on the path to check for an optimap target point, including the tail node of its current edge
  }
  else if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
  {
    path_node_number = pTrajectory_Queue->size; //the number of nodes on the path to check for an optimap target point, including the tail node of its current edge

  }
  else
  {
    printf(" GetTargetPoint_By_PacketTrajectory_For_Carrier(): trajectory_length_type(%d) is unknown!\n", param->vehicle_vanet_vehicle_trajectory_length_type);
    exit(1);    
  }

  /** search an optimal target point p such that |EDD_p - EAD_p| is minimum. **/

  /* initialize double *EDD_p_for_optimal_target_point and double *EDD_p_for_optimal_target_point */
  *EDD_p_for_optimal_target_point = 0;
  *EAD_p_for_optimal_target_point = 0;

  /* copy the tail node of the destination vehicle's current edge to current_edge_tail_node */
  strcpy(current_edge_tail_node, pCurrent_Trajectory_QNode->graph_pos.enode->tail_node);

  /* set pTrajectory_QNode to the first queue node to check */
  pTrajectory_QNode = pCurrent_Trajectory_QNode; //Note that the first point to check is the tail node of the current edge

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_CARRIER__
  printf("GetTargetPoint_By_PacketTrajectory_For_Carrier():[Carrier Vehicle] carrier_vehicle=%d, graph_pos=(%s,%s:%.2f)\n", carrier_vehicle->id, carrier_vehicle->current_pos_in_digraph.enode->tail_node, carrier_vehicle->current_pos_in_digraph.enode->head_node, carrier_vehicle->current_pos_in_digraph.offset);
  printf("GetTargetPoint_By_PacketTrajectory_For_Carrier():[Trajectory]  vehicle=%d, graph_pos=(%s,%s:%.2f)\n", pTrajectory_Queue->vnode->id, pCurrent_Trajectory_QNode->graph_pos.enode->tail_node, pCurrent_Trajectory_QNode->graph_pos.enode->head_node, offset_end);
  printf("GetTargetPoint_By_PacketTrajectory_For_Carrier():[Destination] vehicle=%d, graph_pos=(%s,%s:%.2f)\n", destination_vehicle->id, destination_vehicle->current_pos_in_digraph.enode->tail_node, destination_vehicle->current_pos_in_digraph.enode->head_node, destination_vehicle->current_pos_in_digraph.offset);
#endif

  for(i = 0; i < path_node_number; i++)
  {
    /* For infinite trajectory length type, if pTrajectory_QNode is the queue head, then let pTrajectory_QNode point to head->next */
    if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
    {
      if(pTrajectory_QNode == &(pTrajectory_Queue->head))
        pTrajectory_QNode = pTrajectory_QNode->next;
    }

    target_point =  pTrajectory_QNode->graph_pos.enode->tail_node; //target_point is the tail node of an edge

    ///* compute the path distance from the destination vehicle's current position to the target point */
    //path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory(current_time, pTrajectory_Queue, pCurrent_Trajectory_QNode, offset_end, target_point);

    /* compute the path distance, the path travel time, and the path travel time deviation from the destination vehicle's current position to the target point */
    path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory_Along_With_PathTravelTime_And_Deviation(param, current_time, pTrajectory_Queue, pCurrent_Trajectory_QNode, offset_end, target_point, &path_travel_time, &path_travel_time_deviation);

    /* compute the mean and standard deviation of the travel time duration */
    EAD_p = path_travel_time; //param->vehicle_speed should be used since it is average vehicle speed
    EAD_SD_p = path_travel_time_deviation;

    /* compute the travel time duration */
    //EAD_p = path_distance / param->vehicle_speed; //param->vehicle_speed should be used since it is average vehicle speed
    //EAD_p = path_distance / pTrajectory_Queue->vehicle_speed;
    //EAD_p = path_distance / destination_vehicle->speed;

    /* compute the standard deviation of the travel time duration */
    //EAD_VAR_p = (i+1)*param->vehicle_speed_variance;
    //EAD_SD_p = sqrt(EAD_VAR_p);

    /** Compute the probability that the packet will arrive at the target point earlier than the destination vehicle */
    /* Note that the probability is P[T_i^p <= T_i^v] = \int_{0}^{\infy} \int_{0}^{y} {f(x)g(y)}\,dx\,dy */

    /** compute EDD_p for a target point p */    
    /* compute the EDD and EDD_SD for the target point at the intersection having this AP */
    target_point_id = atoi(target_point);

    VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Carrier(param, target_point_id, carrier_vehicle, FTQ, &EDD_p, &EDD_SD_p);

    /* update min_difference and min_difference_target_point */
    difference =  fabs(EDD_p - EAD_p);

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_CARRIER__
    printf("GetTargetPoint_By_PacketTrajectory_For_Carrier(): target point=%s, difference=%.2f\n", target_point, difference);
#endif 

    if(min_difference > difference)
    {
      min_difference = difference;
      strcpy(min_difference_target_point, target_point);
      *EDD_p_for_optimal_target_point = EDD_p;
      *EAD_p_for_optimal_target_point = EAD_p;
    }

    pTrajectory_QNode = pTrajectory_QNode->next; //pTrajectory_QNode points to the next node on the trajectory
  }

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_CARRIER__
  printf("GetTargetPoint_By_PacketTrajectory_For_Carrier(): optimal target point=%s, difference=%.2f\n", min_difference_target_point, min_difference);
  fgetc(stdin);
#endif

  /* convert min_differnce_target_point into target_point_id */
  target_point_id = atoi(min_difference_target_point);

  return target_point_id;
}

int GetTargetPoint_By_HeadingIntersection_For_Carrier_At_TrajectoryIntersection_Before_DestinationVehicle(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, vehicle_trajectory_queue_t *pTrajectory_Queue, vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode, int destination_vehicle_hop, double destination_vehicle_offset, int vertex_hop, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ //get the id of a target point for carrier vehicle that is the destination vehicle's heading intersection between destination_vehicle_hop and vertex_hop (carrier's position) on the destination vehicle trajectory

  /**********************************************************************/
  static int vehicle_trajectory_expiration_count = 0; //count for vehicle trajectory expiration

  char *target_point = NULL; //target point
  int target_point_id = 0; //target point id that is an intersection
  int new_target_point_id = 0; //new target point id for the packet with the expired vehicle trajectory such that the destination vehicle has already gone out of the trajectory in the packet.

  /**********************************************************************/
  struct_vehicle_t *destination_vehicle = NULL; //pointer to the destination vehicle for packets

  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to the vehicle trajectory queue node
  /**********************************************************************/

#ifdef __DEBUG_LEVEL_AP_PACKET_ARRIVE__
  double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
  double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

  double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
  double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
#endif

  /* check whether this vehicle has packets or not; if there is no packet, return a random target point */
  if(carrier_vehicle->packet_queue->size == 0)
  {
/*     do */
/*     { */
/*       new_target_point_id = smpl_random(1, FTQ->size); */
/*       if(new_target_point_id == carrier_vehicle->target_point_id) */
/* 	continue; */
/*     } while(new_target_point_id < 1 || new_target_point_id > FTQ->size); */

    new_target_point_id = carrier_vehicle->target_point_id;
    return new_target_point_id;
  }

  /** find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory */
  //pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset(param, current_time, carrier_vehicle, &path_current_hop, &offset_end);

  /** check whether the destination vehicle has passed out of its trajectory or not */
  if((param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE) && (pCurrent_Trajectory_QNode == NULL))
  { //choose a random target point among the intersections on the road network graph and set its trajectory selection type as static. If this packet meets another vehicle with fresh destination vehicle, it is forwarded with the packet with the fresh destination vehicle trajectory.

    vehicle_trajectory_expiration_count++;

    /*
    //(i) Option 1: Random target point
    do
    {
      new_target_point_id = smpl_random(1, FTQ->size);
      if(new_target_point_id == carrier_vehicle->target_point_id)
	continue;
    } while(new_target_point_id < 1 || new_target_point_id > FTQ->size);
    */

    //(ii) Option 2: Previous target point (this is the same as the Static target point); this seems better than Option 1
    new_target_point_id = carrier_vehicle->target_point_id;

    //(iii) Option 3: Closest access point; this seems worse than Option 1
    //new_target_point_id = 15;

#ifdef __DEBUG_LEVEL_TRAJECTORY_EXPIRATION__
    /*@Note: Reconsider how to deal with the vehicle trajectory expiration! */
    printf("[time=%.2f] Expire %d: GetTargetPoint_By_OptimalIntersection_For_Carrier(): Note: carrier_vehicle(vid=%d) experiences that the trajectory is expired for this packet(seq=%d)!\n", current_time, vehicle_trajectory_expiration_count, carrier_vehicle->id, carrier_vehicle->latest_packet_ptr->seq);
#endif
    
    return new_target_point_id;
  }

  /** determine the target point as the destination vehicle's heading intersection */
  target_point_id = atoi(pCurrent_Trajectory_QNode->graph_pos.enode->head_node);

#ifdef __DEBUG_LEVEL_AP_PACKET_ARRIVE__

    /** compute the travel time duration */
    destination_vehicle = carrier_vehicle->latest_packet_ptr->dst_vnode; //pointer to the destination vehicle for packets

    path_distance = destination_vehicle->edge_length - destination_vehicle->current_pos_in_digraph.offset; //path distance from vehicle's current position to the head node of the vehicle's current edge

    EAD_p = path_distance / destination_vehicle->speed;

    /** get the EDD and EDD_SD for the target point at the intersection having this carrier vehicle */
    VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Carrier(param, target_point_id, carrier_vehicle, FTQ, &EDD_p, &EDD_SD_p);

    /** set EDD_p_for_optimal_target_point and EAD_p_for_optimal_target_point to EDD_p and EAD_p */
    *EDD_p_for_optimal_target_point = EDD_p;
    *EAD_p_for_optimal_target_point = EAD_p;
#endif

  return target_point_id;
}

int GetTargetPoint_By_RandomIntersection_For_Carrier_At_TrajectoryIntersection_Before_DestinationVehicle(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, vehicle_trajectory_queue_t *pTrajectory_Queue, vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode, int destination_vehicle_hop, double destination_vehicle_offset, int vertex_hop, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ //get the id of a target point for carrier vehicle that is a random intersection point between destination_vehicle_hop and vertex_hop (carrier's position) on the destination vehicle trajectory

  /**********************************************************************/
  static int vehicle_trajectory_expiration_count = 0; //count for vehicle trajectory expiration

  char *target_point = NULL; //target point
  int target_point_id = 0; //target point id that is an intersection
  int new_target_point_id = 0; //new target point id for the packet with the expired vehicle trajectory such that the destination vehicle has already gone out of the trajectory in the packet.
  int random_target_point_index = 0; //index for a random target point on the destination vehicle trajectory 

  int path_node_number = 0; //the number of nodes on the path to check for a target point, including the tail node of its current edge

  /**********************************************************************/
  struct_vehicle_t *destination_vehicle = NULL; //pointer to the destination vehicle for packets

  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to the vehicle trajectory queue node
  int i = 0; //for-loop index
  /**********************************************************************/

#ifdef __DEBUG_LEVEL_AP_PACKET_ARRIVE__
  double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
  double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

  double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
  double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
#endif

  /* check whether this vehicle has packets or not; if there is no packet, return a random target point */
  if(carrier_vehicle->packet_queue->size == 0)
  {
/*     do */
/*     { */
/*       new_target_point_id = smpl_random(1, FTQ->size); */
/*       if(new_target_point_id == carrier_vehicle->target_point_id) */
/* 	continue; */
/*     } while(new_target_point_id < 1 || new_target_point_id > FTQ->size); */

    new_target_point_id = carrier_vehicle->target_point_id;
    return new_target_point_id;
  }

  /** find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory */
  //pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset(param, current_time, carrier_vehicle, &path_current_hop, &offset_end);

  /** check whether the destination vehicle has passed out of its trajectory or not */
  if((param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE) && (pCurrent_Trajectory_QNode == NULL))
  { //choose a random target point among the intersections on the road network graph and set its trajectory selection type as static. If this packet meets another vehicle with fresh destination vehicle, it is forwarded with the packet with the fresh destination vehicle trajectory.

    vehicle_trajectory_expiration_count++;

    /*
    //(i) Option 1: Random target point
    do
    {
      new_target_point_id = smpl_random(1, FTQ->size);
      if(new_target_point_id == carrier_vehicle->target_point_id)
	continue;
    } while(new_target_point_id < 1 || new_target_point_id > FTQ->size);
    */

    //(ii) Option 2: Previous target point (this is the same as the Static target point); this seems better than Option 1
    new_target_point_id = carrier_vehicle->target_point_id;

    //(iii) Option 3: Closest access point; this seems worse than Option 1
    //new_target_point_id = 15;

#ifdef __DEBUG_LEVEL_TRAJECTORY_EXPIRATION__
    /*@Note: Reconsider how to deal with the vehicle trajectory expiration! */
    printf("[time=%.2f] Expire %d: GetTargetPoint_By_OptimalIntersection_For_Carrier(): Note: carrier_vehicle(vid=%d) experiences that the trajectory is expired for this packet(seq=%d)!\n", current_time, vehicle_trajectory_expiration_count, carrier_vehicle->id, carrier_vehicle->latest_packet_ptr->seq);
#endif
    
    return new_target_point_id;
  }

  /** check whether destination_vehicle_hop is greater than vertex_hop; if so, destination_vehicle has already passed vertex on its vehicle trajectory */
  if(destination_vehicle_hop > vertex_hop)
  {
    printf("GetTargetPoint_By_RandomIntersection_For_Carrier_At_TrajectoryIntersection_Before_DestinationVehicle(): destination_vehicle_hop(%d) is greater than vertex_hop(%d)\n", destination_vehicle_hop, vertex_hop);
    exit(1);
  }

  /** compute the number of path nodes on the trajectory to check for a target point */
  path_node_number = vertex_hop - destination_vehicle_hop + 1; //the number of nodes on the path from destination_vehicle_hop to vertex_hop

  /** select a random index from 0 to path_node_number-1 for a target point */
  do
  {
    random_target_point_index = smpl_random(0, path_node_number-1);
  } while(random_target_point_index < 0 || random_target_point_index > path_node_number-1);

  /** search pPathNode corresponding to random_target_point **/
  pTrajectory_QNode = pCurrent_Trajectory_QNode; //Note that the first point to check is the current trajectory queue node that is the tail node of the current edge where the destination vehicle is moving
  for(i = 0; i < random_target_point_index; i++)
  {
     pTrajectory_QNode =  pTrajectory_QNode->next; // pTrajectory_QNode points to the tail node of an edge

    /* For infinite trajectory length type, if pTrajectory_QNode is the queue head, then let pTrajectory_QNode point to head->next */
    if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
    {
      if(pTrajectory_QNode == &(pTrajectory_Queue->head))
        pTrajectory_QNode = pTrajectory_QNode->next;
    }
  }

  target_point = pTrajectory_QNode->graph_pos.enode->tail_node;
  target_point_id = atoi(target_point);

#ifdef __DEBUG_LEVEL_AP_PACKET_ARRIVE__

    /** compute the travel time duration */
    /* compute the path distance from the destination vehicle's current position to the target point */
    path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory(current_time, pTrajectory_Queue, pCurrent_Trajectory_QNode, destination_vehicle_offset, target_point);

    destination_vehicle = carrier_vehicle->latest_packet_ptr->dst_vnode; //pointer to the destination vehicle for packets
    EAD_p = path_distance / destination_vehicle->speed;

    /** get the EDD and EDD_SD for the target point at the intersection having this carrier vehicle */
    VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Carrier(param, target_point_id, carrier_vehicle, FTQ, &EDD_p, &EDD_SD_p);

    /** set EDD_p_for_optimal_target_point and EAD_p_for_optimal_target_point to EDD_p and EAD_p */
    *EDD_p_for_optimal_target_point = EDD_p;
    *EAD_p_for_optimal_target_point = EAD_p;
#endif

  return target_point_id;
}

int GetTargetPoint_By_OptimalIntersection_For_Carrier_At_TrajectoryIntersection_Before_DestinationVehicle(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, forwarding_table_queue_t *FTQ, vehicle_trajectory_queue_t *pTrajectory_Queue, vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode, int destination_vehicle_hop, double destination_vehicle_offset, int vertex_hop, double *EDD_p_for_optimal_target_point, double *EAD_p_for_optimal_target_point)
{ // get the id of an optimal target point for carrier vehicle that is an intersection point between destination_vehicle_hop and vertex_hop (carrier's position) on the destination vehicle trajectory

  static int vehicle_trajectory_expiration_count = 0; //count for vehicle trajectory expiration
  int remaining_path_hop_count = 0; //the number of nodes consisting of the path corresponding to the vehicle trajectory from the head node of its current edge to the destination node (i.e., destination position).
  int path_node_number = 0; //the number of nodes on the path to check for an optimal target point, including the tail node of its current edge
  char *target_point = NULL; //pointer to a vertex corresponding to a target point candidate
  double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
  double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

  double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
  double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
  double difference = 0 ;// absolute difference of EDD_p and EAD_p
  double min_difference = INF; //minimum absolute difference of EDD_p and EAD_p
  char min_difference_target_point[NAME_SIZE]; //target point with min difference
  int i = 0; //for-loop index
  char current_edge_tail_node[NAME_SIZE]; //tail node of the current edge pointed by vehicle->path_ptr

  /**********************************************************************/
  int target_point_id = 0; //target point id that is an intersection

  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to the vehicle trajectory queue node
  int new_target_point_id = 0; //new target point id for the packet with the expired vehicle trajectory such that the destination vehicle has already gone out of the trajectory in the packet.
  /**********************************************************************/

  /* check whether this vehicle has packets or not; if there is no packet, return a random target point */
  if(carrier_vehicle->packet_queue->size == 0)
  {
/*     do */
/*     { */
/*       new_target_point_id = smpl_random(1, FTQ->size); */
/*       if(new_target_point_id == carrier_vehicle->target_point_id) */
/* 	continue; */
/*     } while(new_target_point_id < 1 || new_target_point_id > FTQ->size); */

    new_target_point_id = carrier_vehicle->target_point_id;
    return new_target_point_id;
  }

  /** pointer to the destination vehicle for packets that is used to check whether the trajectory reflects the destination's movement well or not */
  //destination_vehicle = carrier_vehicle->latest_packet_ptr->dst_vnode; 

  /** set up vehicle trajectory queue pTrajectory_Queue */
  //pTrajectory_Queue = &(carrier_vehicle->latest_packet_ptr->vehicle_trajectory); //pointer to the packet's vehicle trajectory

  /** find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory */
  //pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset(param, current_time, carrier_vehicle, &path_current_hop, &offset_end);

  /** check whether the destination vehicle has passed out of its trajectory or not */
  if((param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE) && (pCurrent_Trajectory_QNode == NULL))
  { //choose a random target point among the intersections on the road network graph and set its trajectory selection type as static. If this packet meets another vehicle with fresh destination vehicle, it is forwarded with the packet with the fresh destination vehicle trajectory.

    vehicle_trajectory_expiration_count++;

    /*
    //(i) Option 1: Random target point
    do
    {
      new_target_point_id = smpl_random(1, FTQ->size);
      if(new_target_point_id == carrier_vehicle->target_point_id)
	continue;
    } while(new_target_point_id < 1 || new_target_point_id > FTQ->size);
    */

    //(ii) Option 2: Previous target point (this is the same as the Static target point); this seems better than Option 1
    new_target_point_id = carrier_vehicle->target_point_id;

    //(iii) Option 3: Closest access point; this seems worse than Option 1
    //new_target_point_id = 15;

#ifdef __DEBUG_LEVEL_TRAJECTORY_EXPIRATION__
    /*@Note: Reconsider how to deal with the vehicle trajectory expiration! */
    printf("[time=%.2f] Expire %d: GetTargetPoint_By_OptimalIntersection_For_Carrier(): Note: carrier_vehicle(vid=%d) experiences that the trajectory is expired for this packet(seq=%d)!\n", current_time, vehicle_trajectory_expiration_count, carrier_vehicle->id, carrier_vehicle->latest_packet_ptr->seq);
#endif

    return new_target_point_id;
  }

  /** check whether destination_vehicle_hop is greater than vertex_hop; if so, destination_vehicle has already passed vertex on its vehicle trajectory */
  if(destination_vehicle_hop > vertex_hop)
  {
    printf("GetTargetPoint_By_OptimalIntersection_For_Carrier_At_TrajectoryIntersection_Before_DestinationVehicle(): destination_vehicle_hop(%d) is greater than vertex_hop(%d)\n", destination_vehicle_hop, vertex_hop);
    exit(1);
  }

  /** compute the number of path nodes on the trajectory to check for a target point */
  path_node_number = vertex_hop - destination_vehicle_hop + 1; //the number of nodes on the path from destination_vehicle_hop to vertex_hop

  /** search an optimal target point p such that |EDD_p - EAD_p| is minimum. **/

  /* initialize double *EDD_p_for_optimal_target_point and double *EDD_p_for_optimal_target_point */
  *EDD_p_for_optimal_target_point = 0;
  *EAD_p_for_optimal_target_point = 0;

  /* copy the tail node of the destination vehicle's current edge to current_edge_tail_node */
  strcpy(current_edge_tail_node, pCurrent_Trajectory_QNode->graph_pos.enode->tail_node);

  /* set pTrajectory_QNode to the first queue node to check */
  pTrajectory_QNode = pCurrent_Trajectory_QNode; //Note that the first point to check is the tail node of the current edge

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_CARRIER__
  printf("GetTargetPoint_By_OptimalIntersection_For_Carrier_With_Infinite_VehicleTrajectory():[Carrier Vehicle] carrier_vehicle=%d, graph_pos=(%s,%s:%.2f)\n", carrier_vehicle->id, carrier_vehicle->current_pos_in_digraph.enode->tail_node, carrier_vehicle->current_pos_in_digraph.enode->head_node, carrier_vehicle->current_pos_in_digraph.offset);
  printf("GetTargetPoint_By_OptimalIntersection_For_Carrier_With_Infinite_VehicleTrajectory():[Trajectory]  vehicle=%d, graph_pos=(%s,%s:%.2f)\n", pTrajectory_Queue->vnode->id, pCurrent_Trajectory_QNode->graph_pos.enode->tail_node, pCurrent_Trajectory_QNode->graph_pos.enode->head_node, offset_end);
  printf("GetTargetPoint_By_OptimalIntersection_For_Carrier_With_Infinite_VehicleTrajectory():[Destination] vehicle=%d, graph_pos=(%s,%s:%.2f)\n", destination_vehicle->id, destination_vehicle->current_pos_in_digraph.enode->tail_node, destination_vehicle->current_pos_in_digraph.enode->head_node, destination_vehicle->current_pos_in_digraph.offset);
#endif

  for(i = 0; i < path_node_number; i++)
  {
    /* For infinite trajectory length type, if pTrajectory_QNode is the queue head, then let pTrajectory_QNode point to head->next */
    if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
    {
      if(pTrajectory_QNode == &(pTrajectory_Queue->head))
        pTrajectory_QNode = pTrajectory_QNode->next;
    }

    target_point =  pTrajectory_QNode->graph_pos.enode->tail_node; //target_point is the tail node of an edge

    /* compute the path distance from the destination vehicle's current position to the target point */
    path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory(current_time, pTrajectory_Queue, pCurrent_Trajectory_QNode, destination_vehicle_offset, target_point);

    /* compute the travel time duration */
    EAD_p = path_distance / param->vehicle_speed; //param->vehicle_speed should be used since it is average vehicle speed
    //EAD_p = path_distance / pTrajectory_Queue->vehicle_speed;
    //EAD_p = path_distance / destination_vehicle->speed;

    /** compute EDD_p for a target point p */    
    /* compute the EDD and EDD_SD for the target point at the intersection having this AP */
    target_point_id = atoi(target_point);

    VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Carrier(param, target_point_id, carrier_vehicle, FTQ, &EDD_p, &EDD_SD_p);

    /* update min_difference and min_difference_target_point */
    difference =  fabs(EDD_p - EAD_p);

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_CARRIER__
    printf("GetTargetPoint_By_OptimalIntersection_For_Carrier_With_Infinite_VehicleTrajectory(): target point=%s, difference=%.2f\n", target_point, difference);
#endif 

    if(min_difference > difference)
    {
      min_difference = difference;
      strcpy(min_difference_target_point, target_point);
      *EDD_p_for_optimal_target_point = EDD_p;
      *EAD_p_for_optimal_target_point = EAD_p;
    }

    pTrajectory_QNode = pTrajectory_QNode->next; //pTrajectory_QNode points to the next node on the trajectory
  }

#ifdef __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_CARRIER__
  printf("GetTargetPoint_By_OptimalIntersection_For_Carrier_With_Infinite_VehicleTrajectory(): optimal target point=%s, difference=%.2f\n", min_difference_target_point, min_difference);
  fgetc(stdin);
#endif

  /* convert min_differnce_target_point into target_point_id */
  target_point_id = atoi(min_difference_target_point);

  return target_point_id;
}

double Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList(double current_time, struct_vehicle_t *vehicle, char *target_point)
{ //compute the path distance for the trajectory from the current position to target_point with vehicle's path list
  double path_distance = 0; //path distance

  struct_path_node *pPathNode = NULL; //pointer to a path node
  double remaining_edge_length = 0;

  /** check whether the current edge's tail node is target point or not */
  if(strcmp(target_point, vehicle->current_pos_in_digraph.enode->tail_node) == 0)
  {
    path_distance = -vehicle->current_pos_in_digraph.offset;
    return path_distance;
  }

  /** compute the part of the current edge to travel */
  path_distance = vehicle->edge_length - vehicle->current_pos_in_digraph.offset;
  pPathNode = vehicle->path_ptr->next; //Note that the first point to check is the head node of the current edge
  do
  {
    /* @Note: By our rule of the movement on vehicle's path list, vehicle->path_ptr->next cannot
       point the path list's head, but pPathNode->next can do it. */
    if(pPathNode->next == vehicle->path_list)
      pPathNode = pPathNode->next->next;

    /* check whether target_point is the head node of the next edge */
    if(strcmp(target_point, pPathNode->vertex) == 0)
        break;

    /* check whether there is no target point on the vehicle path list or not */
    if(pPathNode == vehicle->path_ptr)
    {
        if((vehicle->path_hop_count == 1) && (strcmp(target_point, pPathNode->next->vertex) == 0))
        { //the trajectory consists of two vertices
            break;
        }
        else
        {
            printf("Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList(): Error: There is no target point(%s) on the vehicle path list\n", target_point);
            exit(1);
        }
    }

    pPathNode = pPathNode->next;

    path_distance += pPathNode->weight;

  } while(1);


  return path_distance;
}

double Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList_Along_With_PathTravelTime_And_Deviation(parameter_t *param, double current_time, struct_vehicle_t *vehicle, char *target_point, double *travel_time, double *travel_time_deviation)
{ //compute the path distance, the path travel time and the deviation for the trajectory from the current position to target_point with vehicle's path list

  double unit_length = param->vehicle_unit_length; //unit length (i.e., 1 meter) to compute the mean and variance of the travel time with
  double unit_length_travel_time = param->vehicle_unit_length_mean_travel_time; //travel time for the unit length
  double unit_length_travel_time_variance = param->vehicle_unit_length_travel_time_variance; //travel time variance for the unit length
  double unit_length_travel_time_deviation = param->vehicle_unit_length_travel_time_standard_deviation; //travel time standard deviation for the unit length

  double path_distance = 0; //path distance
  double edge_distance = 0; //edge distance

  double edge_travel_time = 0; //edge travel time for a road segment
  double edge_travel_time_deviation = 0; //edge travel time deviation
  double edge_travel_time_variance = 0; //edge travel time variance

  double mu_v = param->vehicle_speed; //vehicle mean speed
  double sigma_v = param->vehicle_speed_standard_deviation; //vehicle speed standard deviation

  double path_travel_time = 0; //path travel time
  double path_travel_time_deviation = 0; //path travel time deviation
  double path_travel_time_variance = 0; //path travel time variance

  struct_path_node *pPathNode = NULL; //pointer to a path node
  double remaining_edge_length = 0;
  
  /** compute the mean and standard deviation of the travel time for the unit length */
  //GSL_Vanet_Compute_TravelTime_And_Deviation(param, unit_length, &unit_length_travel_time, &unit_length_travel_time_deviation);
  //unit_length_travel_time_variance = pow(unit_length_travel_time_deviation, 2); //compute the variance of the travel time for the unit length

  /** check whether the current edge's tail node is target point or not */
  if(strcmp(target_point, vehicle->current_pos_in_digraph.enode->tail_node) == 0)
  {      
      //path_distance = -vehicle->current_pos_in_digraph.offset;
      edge_distance = -vehicle->current_pos_in_digraph.offset; //edge distance
      path_distance += edge_distance; //path distance
  
      /** compute the edge travel time and variance */
      edge_travel_time = edge_distance * unit_length_travel_time; //edge travel time
      edge_travel_time_variance = pow(edge_distance, 2) * unit_length_travel_time_variance; //edge travel time variance
      edge_travel_time_deviation = sqrt(edge_travel_time_variance); //edge travel time deviation

      /** compute the path travel time and variance */
      //path_travel_time = path_distance/mu_v;
      path_travel_time += edge_travel_time;
      path_travel_time_variance += edge_travel_time_variance; //path travel time variance
      path_travel_time_deviation = sqrt(path_travel_time_variance); //path travel time standard deviation

      /** set the path travel time and deviation */
      *travel_time = path_travel_time; //negative path travel time
      *travel_time_deviation = path_travel_time_deviation; //positive path travel time deviation

      return path_distance;
  }

  /** compute the part of the current edge to travel */
  edge_distance = vehicle->edge_length - vehicle->current_pos_in_digraph.offset; //edge distance
  path_distance += edge_distance; //path distance
  
  /** compute the edge travel time and variance */
  edge_travel_time = edge_distance * unit_length_travel_time; //edge travel time
  edge_travel_time_variance = pow(edge_distance, 2) * unit_length_travel_time_variance; //edge travel time variance
  edge_travel_time_deviation = sqrt(edge_travel_time_variance); //edge travel time deviation

  /** compute the path travel time and variance */
  //path_travel_time = path_distance/mu_v;
  path_travel_time += edge_travel_time;
  path_travel_time_variance += edge_travel_time_variance; //path travel time variance
  path_travel_time_deviation = sqrt(path_travel_time_variance); //path travel time standard deviation

  pPathNode = vehicle->path_ptr->next; //Note that the first point to check is the head node of the current edge
  do
  {
    /* @Note: By our rule of the movement on vehicle's path list, vehicle->path_ptr->next cannot
       point the path list's head, but pPathNode->next can do it. */
    if(pPathNode->next == vehicle->path_list)
      pPathNode = pPathNode->next->next;

    /* check whether target_point is the head node of the next edge */
    if(strcmp(target_point, pPathNode->vertex) == 0)
        break;

    /* check whether there is no target point on the vehicle path list or not */
    if(pPathNode == vehicle->path_ptr)
    {
        if((vehicle->path_hop_count == 1) && (strcmp(target_point, pPathNode->next->vertex) == 0))
        { //the trajectory consists of two vertices
            break;
        }
        else
        {
            printf("Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList(): Error: There is no target point(%s) on the vehicle path list\n", target_point);
            exit(1);
        }
    }

    pPathNode = pPathNode->next;

    //path_distance += pPathNode->weight;
    edge_distance = pPathNode->weight; //edge distance
    path_distance += edge_distance; //path distance
  
    /** compute the edge travel time and variance */
    edge_travel_time = edge_distance * unit_length_travel_time; //edge travel time
    edge_travel_time_variance = pow(edge_distance, 2) * unit_length_travel_time_variance; //edge travel time variance
    edge_travel_time_deviation = sqrt(edge_travel_time_variance); //edge travel time deviation

    /** compute the path travel time and variance */
    //path_travel_time = path_distance/mu_v;
    path_travel_time += edge_travel_time;
    path_travel_time_variance += edge_travel_time_variance; //path travel time variance
    path_travel_time_deviation = sqrt(path_travel_time_variance); //path travel time standard deviation
  } while(1);

  /** set the path travel time and deviation */
  *travel_time = path_travel_time;
  *travel_time_deviation = path_travel_time_deviation;

  return path_distance;
}

double Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory(double current_time, vehicle_trajectory_queue_t *pTrajectory_Queue, vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode, double offset_in_current_edge, char *target_point)
{ //compute the path distance for the trajectory from the current position to target_point with destination vehicle trajectory where destination_vehicle_travel_distance is the distance from the trajectory's start position to the destination vehicle's current position
  double path_distance = 0; //path distance
  double target_point_distance = 0; //distance from the trajectory's start position to the target point's position
  double offset_start = 0; //destination vehicle's offset in the first edge on its trajectory
  double edge_length = 0; //edge length
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to a vehicle trajectory queue node
  int i = 0; //for-loop index

  /** check whether the current trajectory queue node's tail node is target point or not */
  if(strcmp(target_point, pCurrent_Trajectory_QNode->graph_pos.enode->tail_node) == 0)
  {
    path_distance = -offset_in_current_edge;
    return path_distance;
  }

  /** compute the part of the current edge to travel */
  path_distance = pCurrent_Trajectory_QNode->graph_pos.enode->weight - offset_in_current_edge;
  pTrajectory_QNode = pCurrent_Trajectory_QNode;
  do
  {
    /* check whether target_point is the head_node of the first trajectory queue node */     
    if(strcmp(target_point, pTrajectory_QNode->graph_pos.enode->head_node) == 0)
      break;

    pTrajectory_QNode = pTrajectory_QNode->next;

    /* if pTrajectory_QNode is the queue head, let it point to head->next */
    if(pTrajectory_QNode == &(pTrajectory_Queue->head))
      pTrajectory_QNode = pTrajectory_QNode->next;

    if(pTrajectory_QNode == pCurrent_Trajectory_QNode)
    {
      printf("Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory(): Error: There is no target point(%s) on the vehicle trajectory\n", target_point);
      exit(1);
    }

    path_distance += pTrajectory_QNode->graph_pos.enode->weight;
  } while(1);

  return path_distance;
}


double Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory_Along_With_PathTravelTime_And_Deviation(parameter_t *param, double current_time, vehicle_trajectory_queue_t *pTrajectory_Queue, vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode, double offset_in_current_edge, char *target_point, double *travel_time, double *travel_time_deviation)
{ //compute the path distance, the path travel time and the deviation for the trajectory from the current position to target_point with destination vehicle trajectory where destination_vehicle_travel_distance is the distance from the trajectory's start position to the destination vehicle's target point

  double unit_length = param->vehicle_unit_length; //unit length (i.e., 1 meter) to compute the mean and variance of the travel time with
  double unit_length_travel_time = param->vehicle_unit_length_mean_travel_time; //travel time for the unit length
  double unit_length_travel_time_variance = param->vehicle_unit_length_travel_time_variance; //travel time variance for the unit length
  double unit_length_travel_time_deviation = param->vehicle_unit_length_travel_time_standard_deviation; //travel time standard deviation for the unit length

  double path_distance = 0; //path distance
  double edge_distance = 0; //edge distance

  double edge_travel_time = 0; //edge travel time for a road segment
  double edge_travel_time_deviation = 0; //edge travel time deviation
  double edge_travel_time_variance = 0; //edge travel time variance

  double mu_v = param->vehicle_speed; //vehicle mean speed
  double sigma_v = param->vehicle_speed_standard_deviation; //vehicle speed standard deviation

  double path_travel_time = 0; //path travel time
  double path_travel_time_deviation = 0; //path travel time deviation
  double path_travel_time_variance = 0; //path travel time variance

  double target_point_distance = 0; //distance from the trajectory's start position to the target point's position
  double offset_start = 0; //destination vehicle's offset in the first edge on its trajectory
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to a vehicle trajectory queue node
  int i = 0; //for-loop index

  /** compute the mean and standard deviation of the travel time for the unit length */
  //GSL_Vanet_Compute_TravelTime_And_Deviation(param, unit_length, &unit_length_travel_time, &unit_length_travel_time_deviation);
  //unit_length_travel_time_variance = pow(unit_length_travel_time_deviation, 2); //compute the variance of the travel time for the unit length

  /** check whether the current trajectory queue node's tail node is target point or not */
  if(strcmp(target_point, pCurrent_Trajectory_QNode->graph_pos.enode->tail_node) == 0)
  {
      //path_distance = -offset_in_current_edge;

      edge_distance = -offset_in_current_edge; //edge distance
      path_distance += edge_distance; //path distance
  
      /** compute the edge travel time and variance */
      edge_travel_time = edge_distance * unit_length_travel_time; //edge travel time
      edge_travel_time_variance = pow(edge_distance, 2) * unit_length_travel_time_variance; //edge travel time variance
      edge_travel_time_deviation = sqrt(edge_travel_time_variance); //edge travel time deviation

      /** compute the path travel time and variance */
      //path_travel_time = path_distance/mu_v;
      path_travel_time += edge_travel_time;
      path_travel_time_variance += edge_travel_time_variance; //path travel time variance
      path_travel_time_deviation = sqrt(path_travel_time_variance); //path travel time standard deviation

      /** set the path travel time and deviation */
      *travel_time = path_travel_time; //negative path travel time
      *travel_time_deviation = path_travel_time_deviation; //positive path travel time deviation

      return path_distance;
  }

  /** compute the part of the current edge to travel */
  edge_distance = pCurrent_Trajectory_QNode->graph_pos.enode->weight - offset_in_current_edge; //edge distance
  path_distance += edge_distance; //path distance
  
  /** compute the edge travel time and variance */
  edge_travel_time = edge_distance * unit_length_travel_time; //edge travel time
  edge_travel_time_variance = pow(edge_distance, 2) * unit_length_travel_time_variance; //edge travel time variance
  edge_travel_time_deviation = sqrt(edge_travel_time_variance); //edge travel time deviation

  /** compute the path travel time and variance */
  //path_travel_time = path_distance/mu_v;
  path_travel_time += edge_travel_time;
  path_travel_time_variance += edge_travel_time_variance; //path travel time variance
  path_travel_time_deviation = sqrt(path_travel_time_variance); //path travel time standard deviation

  pTrajectory_QNode = pCurrent_Trajectory_QNode;
  do
  {
    /* check whether target_point is the head_node of the first trajectory queue node */     
    if(strcmp(target_point, pTrajectory_QNode->graph_pos.enode->head_node) == 0)
      break;

    pTrajectory_QNode = pTrajectory_QNode->next;

    /* if pTrajectory_QNode is the queue head, let it point to head->next */
    if(pTrajectory_QNode == &(pTrajectory_Queue->head))
      pTrajectory_QNode = pTrajectory_QNode->next;

    if(pTrajectory_QNode == pCurrent_Trajectory_QNode)
    {
      printf("Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory(): Error: There is no target point(%s) on the vehicle trajectory\n", target_point);
      exit(1);
    }

    //path_distance += pTrajectory_QNode->graph_pos.enode->weight;

    edge_distance = pTrajectory_QNode->graph_pos.enode->weight; //edge distance
    path_distance += edge_distance; //path distance

    /** compute the edge travel time and variance */
    edge_travel_time = edge_distance * unit_length_travel_time; //edge travel time
    edge_travel_time_variance = pow(edge_distance, 2) * unit_length_travel_time_variance; //edge travel time variance
    edge_travel_time_deviation = sqrt(edge_travel_time_variance); //edge travel time deviation

    /** compute the path travel time and variance */
    //path_travel_time = path_distance/mu_v;
    path_travel_time += edge_travel_time;
    path_travel_time_variance += edge_travel_time_variance; //path travel time variance
    path_travel_time_deviation = sqrt(path_travel_time_variance); //path travel time standard deviation 
  } while(1);

  /** set the path travel time and deviation */
  *travel_time = path_travel_time;
  *travel_time_deviation = path_travel_time_deviation;

  return path_distance;
}

double Compute_PathDistance_From_CurrentPosition_To_EndPosition_With_VehicleTrajectory_Along_With_PathTravelTime_And_Deviation(parameter_t *param, double current_time, packet_queue_node_t *packet, double *travel_time, double *travel_time_deviation)
{ //compute the vehicle's path distance, the path travel time and the deviation for the trajectory from the current position to the end position on the vehicle trajectory where destination_vehicle_travel_distance is the distance from the trajectory's start position to the destination vehicle's end position on its vehicle trajectory

  vehicle_trajectory_queue_t *pTrajectory_Queue = &(packet->vehicle_trajectory); //pointer to the vehicle trajectory queue
  vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode = pTrajectory_Queue->head.next; //pointer to the first vehicle trajectory queue node
  double offset_in_current_edge = packet->dst_vnode->current_pos_in_digraph.offset; //the offset on the vehicle's current edge
  char *target_point = pTrajectory_Queue->head.prev->graph_pos.enode->head_node; //pointer to the last vehicle trajectory queue node

  double unit_length = param->vehicle_unit_length; //unit length (i.e., 1 meter) to compute the mean and variance of the travel time with
  double unit_length_travel_time = param->vehicle_unit_length_mean_travel_time; //travel time for the unit length
  double unit_length_travel_time_variance = param->vehicle_unit_length_travel_time_variance; //travel time variance for the unit length
  double unit_length_travel_time_deviation = param->vehicle_unit_length_travel_time_standard_deviation; //travel time standard deviation for the unit length

  double path_distance = 0; //path distance
  double edge_distance = 0; //edge distance

  double edge_travel_time = 0; //edge travel time for a road segment
  double edge_travel_time_deviation = 0; //edge travel time deviation
  double edge_travel_time_variance = 0; //edge travel time variance

  double mu_v = param->vehicle_speed; //vehicle mean speed
  double sigma_v = param->vehicle_speed_standard_deviation; //vehicle speed standard deviation

  double path_travel_time = 0; //path travel time
  double path_travel_time_deviation = 0; //path travel time deviation
  double path_travel_time_variance = 0; //path travel time variance

  double target_point_distance = 0; //distance from the trajectory's start position to the target point's position
  double offset_start = 0; //destination vehicle's offset in the first edge on its trajectory
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to a vehicle trajectory queue node
  int i = 0; //for-loop index

  /** compute the mean and standard deviation of the travel time for the unit length */
  //GSL_Vanet_Compute_TravelTime_And_Deviation(param, unit_length, &unit_length_travel_time, &unit_length_travel_time_deviation);
  //unit_length_travel_time_variance = pow(unit_length_travel_time_deviation, 2); //compute the variance of the travel time for the unit length

  /** compute the part of the current edge to travel */
  edge_distance = pCurrent_Trajectory_QNode->graph_pos.enode->weight - offset_in_current_edge; //edge distance
  path_distance += edge_distance; //path distance
  
  /** compute the edge travel time and variance */
  edge_travel_time = edge_distance * unit_length_travel_time; //edge travel time
  edge_travel_time_variance = pow(edge_distance, 2) * unit_length_travel_time_variance; //edge travel time variance
  edge_travel_time_deviation = sqrt(edge_travel_time_variance); //edge travel time deviation

  /** compute the path travel time and variance */
  //path_travel_time = path_distance/mu_v;
  path_travel_time += edge_travel_time;
  path_travel_time_variance += edge_travel_time_variance; //path travel time variance
  path_travel_time_deviation = sqrt(path_travel_time_variance); //path travel time standard deviation

  pTrajectory_QNode = pCurrent_Trajectory_QNode;
  do
  {
    /* check whether target_point is the head_node of the first trajectory queue node */     
    if(strcmp(target_point, pTrajectory_QNode->graph_pos.enode->head_node) == 0)
      break;

    pTrajectory_QNode = pTrajectory_QNode->next;

    /* if pTrajectory_QNode is the queue head, let it point to head->next */
    if(pTrajectory_QNode == &(pTrajectory_Queue->head))
      pTrajectory_QNode = pTrajectory_QNode->next;

    if(pTrajectory_QNode == pCurrent_Trajectory_QNode)
    {
      printf("Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehicleTrajectory(): Error: There is no target point(%s) on the vehicle trajectory\n", target_point);
      exit(1);
    }

    //path_distance += pTrajectory_QNode->graph_pos.enode->weight;

    edge_distance = pTrajectory_QNode->graph_pos.enode->weight; //edge distance
    path_distance += edge_distance; //path distance

    /** compute the edge travel time and variance */
    edge_travel_time = edge_distance * unit_length_travel_time; //edge travel time
    edge_travel_time_variance = pow(edge_distance, 2) * unit_length_travel_time_variance; //edge travel time variance
    edge_travel_time_deviation = sqrt(edge_travel_time_variance); //edge travel time deviation

    /** compute the path travel time and variance */
    //path_travel_time = path_distance/mu_v;
    path_travel_time += edge_travel_time;
    path_travel_time_variance += edge_travel_time_variance; //path travel time variance
    path_travel_time_deviation = sqrt(path_travel_time_variance); //path travel time standard deviation 
  } while(1);

  /** set the path travel time and deviation */
  *travel_time = path_travel_time;
  *travel_time_deviation = path_travel_time_deviation;

  return path_distance;
}

double Compute_PathDistance_For_VehicleTrajectory(vehicle_trajectory_queue_t *pTrajectory_Queue)
{ //compute the path distance for the vehicle trajectory from the start position to the end position
  double path_distance = 0; //path distance
  int size = pTrajectory_Queue->size; //queue size
  double edge_length = 0; //edge length
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to a vehicle trajectory queue node
  int i = 0; //for-loop index

  /** compute the part of the current edge to travel */
  pTrajectory_QNode = &(pTrajectory_Queue->head);
  for(i = 0; i < size; i++)
  {
      pTrajectory_QNode = pTrajectory_QNode->next;
      path_distance += pTrajectory_QNode->graph_pos.enode->weight;
  }

  return path_distance;
}

double Compute_TravelTime_For_VehicleTrajectory(parameter_t *param, double current_time, vehicle_trajectory_queue_t *pTrajectory_Queue)
{ //compute the travel time for the path distance for the trajectory from the start position to the end position in the vehicle trajectory. 

  double path_distance = 0; //path distance
  double travel_time = 0; //travel time
  char target_point[NAME_SIZE]; //target point

  /* compute the path distance for the trajectory from the start position to the end position */
  path_distance = Compute_PathDistance_For_VehicleTrajectory(pTrajectory_Queue);

  /* compute the expected travel time */
  travel_time = path_distance/param->vehicle_speed;

  return travel_time;
}

int Find_NewTargetPoint_Where_Vehicle_Is_Close_To_DestinationVehicleTrajectory(parameter_t *param, double current_time, struct_vehicle_t *vehicle, forwarding_table_queue_t *FTQ, intersection_area_type_t input_intersection_area_type)
{ //[NEW] find a new target point to which vehicle is closest from the intersection area containing vehicle and which is on the destination vehicle trajectory
  //[OLD] find a new target point where the intersection area(s) having vehicle has an intersection on the destination vehicle

  int target_point_id = 0; //a new target point id towards destination vehicle 
  boolean flag_for_tail = FALSE, flag_for_head = FALSE; //flags to check whether tail node and head node belong to the destination vehicle trajectory
  char *pTailNode = vehicle->current_pos_in_digraph.enode->tail_node; //pointer to tail node's name of the vehicle's current moving edge
  char *pHeadNode = vehicle->current_pos_in_digraph.enode->head_node; //pointer to head node's name of the vehicle's current moving edge
  vehicle_trajectory_queue_t *pTrajectory_Queue = NULL; //pointer to the packet's vehicle trajectory
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to the vehicle trajectory queue node
  vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode = NULL; //pointer to the current trajectory queue node for the edge where the destination vehicle is moving
  int current_hop = 0; //current hop on the vehicle trajectory
  double current_offset = 0; //offset for the current hop on the trajectory
  int vertex_hop = 0; //the hop of vertex on the destination vehicle trajectory

  int target_point_id_for_tail = 0; //target point for tail node
  double EDD_p_for_tail = 0; //EDD_p of the target point for tail node
  double EAD_p_for_tail = 0; //EAD_p of the target point for tail node
  double difference_for_tail = 0; //absolute difference of EDD_p and EAD_p for tail node

  int target_point_id_for_head = 0; //target point for head node
  double EDD_p_for_head = 0; //EDD_p of the target point for head node
  double EAD_p_for_head = 0; //EAD_p of the target point for head node
  double difference_for_head = 0; //absolute difference of EDD_p and EAD_p for head node

  /** check whether vehicle's packet queue is empty or not; if so, return immediately */
  if(vehicle->packet_queue->size == 0)
  {
    target_point_id = 0;
    return target_point_id;
  }

  /** find the current trajectory queue node along with the current hop and the offset for the current hop on the trajectory */
  pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset(param, current_time, vehicle, &current_hop, &current_offset);

  /* check whether the vehicle's current trajectory queue node is valid or not */
  if(pCurrent_Trajectory_QNode == NULL)
  {
    target_point_id = 0;
    return target_point_id;
  }

  /** check whether the tail intersection area has an intersection on the destination vehicle trajectory */
  if(input_intersection_area_type == INTERSECTION_AREA_TAIL_NODE || input_intersection_area_type == INTERSECTION_AREA_BOTH_NODES)
  {
    flag_for_tail = TRUE; //flag to indicate that target_point_id for tail node is computed

    /* find the hop of tail node on the destination vehicle trajectory */
    vertex_hop = Find_VertexHop_On_VehicleTrajectory(&(vehicle->latest_packet_ptr->vehicle_trajectory), pTailNode);

    /* check whether tail node is before the destination vehicle on the destination vehicle trajectory */
    if(vertex_hop >= current_hop)
    {
      /* get the intersection id of a target point for carrier vehicle that is within the communication range of an intersection on the destination vehicle trajectory where the intersection is before the destination vehicle on the trajectory */
      target_point_id_for_tail = GetTargetPoint_For_Carrier_At_TrajectoryIntersection_Before_DestinationVehicle(param, current_time, vehicle, FTQ, pCurrent_Trajectory_QNode->ptr_queue, pCurrent_Trajectory_QNode, current_hop, current_offset, vertex_hop, &EDD_p_for_tail, &EAD_p_for_tail);

      //target_point_id_for_tail = GetTargetPoint_For_Carrier(param, current_time, vehicle, FTQ, &EDD_p_for_tail, &EAD_p_for_tail);
    }
    else if(vertex_hop >= 0 && vertex_hop < current_hop)
    { //in the case where the packet is behind the destination vehicle, recompute a target point among intersections from the destination vehicle's current edge's tail node to the end of the trajectory
      target_point_id_for_tail = GetTargetPoint_For_Carrier(param, current_time, vehicle, FTQ, &EDD_p_for_tail, &EAD_p_for_tail);
    }
    else
    { //the case where vertex is not close to the destination vehicle's trajectory
      target_point_id = 0;
      return target_point_id;
    }

    difference_for_tail =  fabs(EDD_p_for_tail - EAD_p_for_tail);
  }

  /** check whether the head intersection area has an intersection on the destination vehicle trajectory */
  if(input_intersection_area_type == INTERSECTION_AREA_HEAD_NODE || input_intersection_area_type == INTERSECTION_AREA_BOTH_NODES)
  {
    flag_for_head = TRUE; //flag to indicate that target_point_id for head node is computed

    /* find the hop of head node on the destination vehicle trajectory */
    vertex_hop = Find_VertexHop_On_VehicleTrajectory(&(vehicle->latest_packet_ptr->vehicle_trajectory), pHeadNode);

    /* check whether tail node is before the destination vehicle on the destination vehicle trajectory */
    if(vertex_hop >= current_hop)
    { 
      /* get the intersection id of a target point for carrier vehicle that is within the communication range of an intersection on the destination vehicle trajectory where the intersection is before the destination vehicle on the trajectory */
      target_point_id_for_head = GetTargetPoint_For_Carrier_At_TrajectoryIntersection_Before_DestinationVehicle(param, current_time, vehicle, FTQ, pCurrent_Trajectory_QNode->ptr_queue, pCurrent_Trajectory_QNode, current_hop, current_offset, vertex_hop, &EDD_p_for_head, &EAD_p_for_head);

      //target_point_id_for_head = GetTargetPoint_For_Carrier(param, current_time, vehicle, FTQ, &EDD_p_for_tail, &EAD_p_for_tail);
    }     
    else if(vertex_hop >= 0 && vertex_hop < current_hop)
    { //in the case where the packet is behind the destination vehicle, recompute a target point among intersections from the destination vehicle's current edge's tail node to the end of the trajectory
      target_point_id_for_head = GetTargetPoint_For_Carrier(param, current_time, vehicle, FTQ, &EDD_p_for_tail, &EAD_p_for_tail);
    }
    else
    { //the case where vertex is not close to the destination vehicle's trajectory
      target_point_id = 0;
      return target_point_id;
    }

    difference_for_head =  fabs(EDD_p_for_head - EAD_p_for_head);
  }

  /** determine a new target point */
  if(flag_for_tail == TRUE && flag_for_head == TRUE)
  {
    if(difference_for_tail < difference_for_head)
      target_point_id = target_point_id_for_tail;
    else
      target_point_id = target_point_id_for_head;
  }
  else if(flag_for_tail == TRUE)
  {
    target_point_id = target_point_id_for_tail;
  }
  else if(flag_for_head == TRUE)
  {
    target_point_id = target_point_id_for_head;
  }
  else
  {
    target_point_id = 0;
  }

  return target_point_id;
}

vehicle_trajectory_queue_node_t* Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset(parameter_t *param, double current_time, struct_vehicle_t *carrier_vehicle, int *current_hop, double *current_offset)
{ //find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory for carrier_vehicle
  struct_vehicle_t *destination_vehicle = NULL; //pointer to the destination vehicle for packets
  packet_queue_node_t *pPacket = NULL; //pointer to the latest packet from AP
  vehicle_trajectory_queue_t *pTrajectory_Queue = NULL; //pointer to the packet's vehicle trajectory
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to the vehicle trajectory queue node
  vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode = NULL; //pointer to the current trajectory queue node for the edge where the destination vehicle is moving
  int path_current_hop = 0; //destination vehicle's current hop at this current time
  double delivery_time = 0; //delivery time so far taken for the delivery from AP to this carrier vehicle
  double travel_distance = 0; //destination vehicle's travel distance during delivery_time
  double remaining_distance = 0; //remaining distance to consider path_current_hop
  double offset_start = 0; //destination vehicle's offset in the first edge on its trajectory
  double offset_end = 0; //destination vehicle's offset in the current edge on its trajectory for current_time
  double edge_length = 0; //edge length
  int i = 0; //for-loop index

  /** check whether  carrier_vehicle->latest_packet_ptr is NULL or not */
  if(carrier_vehicle->latest_packet_ptr == NULL)
  {
      printf("Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset(): carrier_vehicle->latest_packet_ptr is NULL!\n");

      /** set the output parameters */
      *current_offset = -1;
      *current_hop = -1;

      return NULL;
  }
  else
  {
      /* pointer to the destination vehicle for packets that is used to check whether the trajectory reflects the destination's movement well or not */
      destination_vehicle = carrier_vehicle->latest_packet_ptr->dst_vnode; 
      pPacket = carrier_vehicle->latest_packet_ptr; //pointer to the latest packet from AP
      pTrajectory_Queue = &(pPacket->vehicle_trajectory); //pointer to the packet's vehicle trajectory
  }

  /* if trajectory queue is empty, return immediately */
  if(pTrajectory_Queue->size == 0)
  {
    /** set the output parameters */
    *current_offset = -1;
    *current_hop = -1;

    return NULL;
  }

  /* compute the delivery time so far taken for the delivery from AP to this carrier vehicle */
  delivery_time = current_time - pPacket->generation_time;

  /* find the position of the destination vehicle on its trajectory in terms of current_hop_count and offset for the directional edge having the destination vehicle */
  travel_distance = delivery_time * param->vehicle_speed; //param->vehicle_speed should be used since it is average vehicle speed
  //travel_distance = delivery_time * pTrajectory_Queue->vehicle_speed;

  pTrajectory_QNode = &(pTrajectory_Queue->head);
 
  while(1)
  // while(remaining_distance >= 0)
  //for(i = 0; i < pTrajectory_Queue->size; i++)
  {
    pTrajectory_QNode = pTrajectory_QNode->next;

    if(i == 0)
    {
      offset_start = pTrajectory_QNode->graph_pos.offset;
      edge_length = pTrajectory_QNode->graph_pos.enode->weight;
      remaining_distance = offset_start + travel_distance - edge_length; 
      path_current_hop = 0; //current path hop

      /* check whether the position for the destination vehicle is determined with remaining_distance or not */
      if(remaining_distance < 0)
      {
        offset_end = offset_start + travel_distance;
	pCurrent_Trajectory_QNode = pTrajectory_QNode;
        break;
      }      
    }
    else if(pTrajectory_QNode == &(pTrajectory_Queue->head))
    { //if pTrajectory_QNode is head, let it point to head->next
      if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
      {
	pTrajectory_QNode = pTrajectory_QNode->next;
      }
      else if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE)
      { //the destination vehicle has passed out of its trajectory
        /** set the output parameters */
        *current_offset = -1;
        *current_hop = -1;
        return NULL;
      }
      else
      {
	printf("Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset(): trajectory_length_type(%d) is unknown!\n", param->vehicle_vanet_vehicle_trajectory_length_type);
	exit(1);
      }
    }
    else
    {
      edge_length = pTrajectory_QNode->graph_pos.enode->weight;
      remaining_distance -= edge_length; 
      path_current_hop++; //current path hop

      /* check whether the position for the destination vehicle is determined with remaining_distance or not */
      if(remaining_distance < 0)
      {
        //offset_end = remaining_distance;  
	offset_end = remaining_distance + edge_length; //roll back to get the offset of the edge where the destination vehicle is moving 

	pCurrent_Trajectory_QNode = pTrajectory_QNode;
        break;
      }    
    }

    i++;
  }

  /** set the output parameters */
  *current_offset = offset_end;
  *current_hop = path_current_hop;
  
  return pCurrent_Trajectory_QNode;
}

vehicle_trajectory_queue_node_t* Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_StationaryNode(parameter_t *param, double current_time, stationary_node_queue_node_t *stationary_node, int *current_hop, double *current_offset)
{ //find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory for stationary_node 
  struct_vehicle_t *destination_vehicle = NULL; //pointer to the destination vehicle for packets
  packet_queue_node_t *pPacket = NULL; //pointer to the latest packet from AP
  vehicle_trajectory_queue_t *pTrajectory_Queue = NULL; //pointer to the packet's vehicle trajectory
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to the vehicle trajectory queue node
  vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode = NULL; //pointer to the current trajectory queue node for the edge where the destination vehicle is moving
  int path_current_hop = 0; //destination vehicle's current hop at this current time
  double delivery_time = 0; //delivery time so far taken for the delivery from AP to this carrier vehicle
  double travel_distance = 0; //destination vehicle's travel distance during delivery_time
  double remaining_distance = 0; //remaining distance to consider path_current_hop
  double offset_start = 0; //destination vehicle's offset in the first edge on its trajectory
  double offset_end = 0; //destination vehicle's offset in the current edge on its trajectory for current_time
  double edge_length = 0; //edge length
  int i = 0; //for-loop index

  /** check whether stationary_node->latest_packet_ptr is NULL or not */
  if(stationary_node->latest_packet_ptr == NULL)
  {
      printf("Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_StationaryNode(): stationary_node->latest_packet_ptr is NULL!\n");

      /** set the output parameters */
      *current_offset = -1;
      *current_hop = -1;

      return NULL;
  }
  else
  {
      /* obtain the pointer to the destination vehicle for packets that is used to check whether the trajectory reflects the destination's movement well or not */
      destination_vehicle = stationary_node->latest_packet_ptr->dst_vnode; 
      pPacket = stationary_node->latest_packet_ptr; //pointer to the latest packet from AP
      pTrajectory_Queue = &(pPacket->vehicle_trajectory); //pointer to the packet's vehicle trajectory
  }

  /* if trajectory queue is empty, return immediately */
  if(pTrajectory_Queue->size == 0)
  {
    /** set the output parameters */
    *current_offset = -1;
    *current_hop = -1;

    return NULL;
  }

  /* compute the delivery time so far taken for the delivery from AP to this carrier vehicle */
  delivery_time = current_time - pPacket->generation_time;

  /* find the position of the destination vehicle on its trajectory in terms of current_hop_count and offset for the directional edge having the destination vehicle */
  travel_distance = delivery_time * param->vehicle_speed; //param->vehicle_speed should be used since it is average vehicle speed
  //travel_distance = delivery_time * pTrajectory_Queue->vehicle_speed;

  pTrajectory_QNode = &(pTrajectory_Queue->head);
 
  while(1)
  // while(remaining_distance >= 0)
  //for(i = 0; i < pTrajectory_Queue->size; i++)
  {
    pTrajectory_QNode = pTrajectory_QNode->next;

    if(i == 0)
    {
      offset_start = pTrajectory_QNode->graph_pos.offset;
      edge_length = pTrajectory_QNode->graph_pos.enode->weight;
      remaining_distance = offset_start + travel_distance - edge_length; 
      path_current_hop = 0; //current path hop

      /* check whether the position for the destination vehicle is determined with remaining_distance or not */
      if(remaining_distance < 0)
      {
        offset_end = offset_start + travel_distance;
	pCurrent_Trajectory_QNode = pTrajectory_QNode;
        break;
      }      
    }
    else if(pTrajectory_QNode == &(pTrajectory_Queue->head))
    { //if pTrajectory_QNode is head, let it point to head->next
      if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
      {
	pTrajectory_QNode = pTrajectory_QNode->next;
      }
      else if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE)
      { //the destination vehicle has passed out of its trajectory
        /** set the output parameters */
        *current_offset = -1;
        *current_hop = -1;
        return NULL;
      }
      else
      {
	printf("Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_StationaryNode(): trajectory_length_type(%d) is unknown!\n", param->vehicle_vanet_vehicle_trajectory_length_type);
	exit(1);
      }
    }
    else
    {
      edge_length = pTrajectory_QNode->graph_pos.enode->weight;
      remaining_distance -= edge_length; 
      path_current_hop++; //current path hop

      /* check whether the position for the destination vehicle is determined with remaining_distance or not */
      if(remaining_distance < 0)
      {
        //offset_end = remaining_distance;  
	offset_end = remaining_distance + edge_length; //roll back to get the offset of the edge where the destination vehicle is moving 

	pCurrent_Trajectory_QNode = pTrajectory_QNode;
        break;
      }    
    }

    i++;
  }

  /** set the output parameters */
  *current_offset = offset_end;
  *current_hop = path_current_hop;
  
  return pCurrent_Trajectory_QNode;
}

vehicle_trajectory_queue_node_t* Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_Packet(parameter_t *param, double current_time, packet_queue_node_t *pPacket, int *current_hop, double *current_offset)
{ //find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the vehicle trajectory contained in packet 
  struct_vehicle_t *destination_vehicle = NULL; //pointer to the destination vehicle for packets
  vehicle_trajectory_queue_t *pTrajectory_Queue = NULL; //pointer to the packet's vehicle trajectory
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to the vehicle trajectory queue node
  vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode = NULL; //pointer to the current trajectory queue node for the edge where the destination vehicle is moving
  int path_current_hop = 0; //destination vehicle's current hop at this current time
  double delivery_time = 0; //delivery time so far taken for the delivery from AP to this carrier vehicle
  double travel_distance = 0; //destination vehicle's travel distance during delivery_time
  double remaining_distance = 0; //remaining distance to consider path_current_hop
  double offset_start = 0; //destination vehicle's offset in the first edge on its trajectory
  double offset_end = 0; //destination vehicle's offset in the current edge on its trajectory for current_time
  double edge_length = 0; //edge length
  int i = 0; //for-loop index

  /** check whether pPacket is NULL or not */
  if(pPacket == NULL)
  {
      printf("Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_Packet(): pPacket is NULL!\n");

      /** set the output parameters */
      *current_offset = -1;
      *current_hop = -1;

      return NULL;
  }
  else
  {
      /* obtain the pointer to the destination vehicle for packets that is used to check whether the trajectory reflects the destination's movement well or not */
      destination_vehicle = pPacket->dst_vnode; 
      pTrajectory_Queue = &(pPacket->vehicle_trajectory); //pointer to the packet's vehicle trajectory
  }

  /* if trajectory queue is empty, return immediately */
  if(pTrajectory_Queue->size == 0)
  {
    /** set the output parameters */
    *current_offset = -1;
    *current_hop = -1;

    return NULL;
  }

  /* compute the delivery time so far taken for the delivery from AP to this carrier vehicle */
  delivery_time = current_time - pPacket->generation_time;

  /* find the position of the destination vehicle on its trajectory in terms of current_hop_count and offset for the directional edge having the destination vehicle */
  travel_distance = delivery_time * param->vehicle_speed; //param->vehicle_speed should be used since it is average vehicle speed
  //travel_distance = delivery_time * pTrajectory_Queue->vehicle_speed;

  pTrajectory_QNode = &(pTrajectory_Queue->head);
 
  while(1)
  // while(remaining_distance >= 0)
  //for(i = 0; i < pTrajectory_Queue->size; i++)
  {
    pTrajectory_QNode = pTrajectory_QNode->next;

    if(i == 0)
    {
      offset_start = pTrajectory_QNode->graph_pos.offset;
      edge_length = pTrajectory_QNode->graph_pos.enode->weight;
      remaining_distance = offset_start + travel_distance - edge_length; 
      path_current_hop = 0; //current path hop

      /* check whether the position for the destination vehicle is determined with remaining_distance or not */
      if(remaining_distance < 0)
      {
        offset_end = offset_start + travel_distance;
	pCurrent_Trajectory_QNode = pTrajectory_QNode;
        break;
      }      
    }
    else if(pTrajectory_QNode == &(pTrajectory_Queue->head))
    { //if pTrajectory_QNode is head, let it point to head->next
      if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE)
      {
	pTrajectory_QNode = pTrajectory_QNode->next;
      }
      else if(param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE)
      { //the destination vehicle has passed out of its trajectory
        /** set the output parameters */
        *current_offset = -1;
        *current_hop = -1;
        return NULL;
      }
      else
      {
	printf("Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_StationaryNode(): trajectory_length_type(%d) is unknown!\n", param->vehicle_vanet_vehicle_trajectory_length_type);
	exit(1);
      }
    }
    else
    {
      edge_length = pTrajectory_QNode->graph_pos.enode->weight;
      remaining_distance -= edge_length; 
      path_current_hop++; //current path hop

      /* check whether the position for the destination vehicle is determined with remaining_distance or not */
      if(remaining_distance < 0)
      {
        //offset_end = remaining_distance;  
	offset_end = remaining_distance + edge_length; //roll back to get the offset of the edge where the destination vehicle is moving 

	pCurrent_Trajectory_QNode = pTrajectory_QNode;
        break;
      }    
    }

    i++;
  }

  /** set the output parameters */
  *current_offset = offset_end;
  *current_hop = path_current_hop;
  
  return pCurrent_Trajectory_QNode;
}

int Find_VertexHop_On_VehicleTrajectory(vehicle_trajectory_queue_t *pTrajectory_Queue, char *vertex)
{ //find the hop of vertex on the destination vehicle trajectory
  int vertex_hop = 0; //the hop of vertex on the destination vehicle trajectory
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to the vehicle trajectory queue node
  int size = pTrajectory_Queue->size; //trajectory queue size
  int i = 0; //for-loop index

  pTrajectory_QNode = &(pTrajectory_Queue->head);
  for(i = 0; i < size; i++)
  {
    pTrajectory_QNode = pTrajectory_QNode->next;

    if(strcmp(vertex, pTrajectory_QNode->graph_pos.enode->tail_node) == 0)
      break;
  }

  /* check whether there exists a queue node corresponding to vertex or not */
  if(i < size)
    vertex_hop = i;
  else
    vertex_hop = -1;

  return vertex_hop;
}

vehicle_trajectory_queue_node_t* Find_VehicleTrajectoryQueueNode_For_VertexHop_On_VehicleTrajectory(vehicle_trajectory_queue_t *pTrajectory_Queue, char *vertex)
{ //find the vehicle trajectory queue node corresponding to the hop of vertex on the destination vehicle trajectory
  int vertex_hop = 0; //the hop of vertex on the destination vehicle trajectory
  vehicle_trajectory_queue_node_t *pTrajectory_QNode = NULL; //pointer to the vehicle trajectory queue node
  int size = pTrajectory_Queue->size; //trajectory queue size
  int i = 0; //for-loop index

  pTrajectory_QNode = &(pTrajectory_Queue->head);
  for(i = 0; i < size; i++)
  {
    pTrajectory_QNode = pTrajectory_QNode->next;

    if(strcmp(vertex, pTrajectory_QNode->graph_pos.enode->tail_node) == 0)
      break;
  }

  /* check whether there exists a queue node corresponding to vertex or not */
  if(i == size)
    pTrajectory_QNode = NULL;

  return pTrajectory_QNode;
}

void Update_LinkDelay_Information(parameter_t *param, struct_graph_node *G, int G_size)
{ //update the link delay, link delay variance, link delay standard deviation for each edge in the graph G with the traffic statistics of vehicular traffic density and vehicle speed
  int i = 0, j = 0; //for-loop indices
  struct_graph_node *ptr = NULL; //pointer to graph node

  for(i = 0; i < G_size; i++)
  {
    ptr = G[i].next;
    if(ptr == NULL)
      continue;

    while(ptr != NULL)
    {
      j = atoi(ptr->vertex) - 1;
      //node id starts from 1, but it should decrease by 1 for weight matrix where the id starts from 0.
      ptr->edge_delay = VADD_Compute_Edge_Delay(param, ptr); //compute the edge delay for the road segment r_ij
      ptr->edge_delay_standard_deviation = VADD_Compute_Edge_Delay_Standard_Deviation(param, ptr); //compute the edge delay standard deviation for the road segment r_ij
      ptr->edge_delay_variance = pow(ptr->edge_delay_standard_deviation, 2); //compute the edge delay variance for the road segment r_ij
      ptr = ptr->next;
    }
  }
}

double Compute_TargetPoint_OptimizationValue(parameter_t *param, int target_point_id, double EDD_p, double EDD_SD_p, double EAD_p, double EAD_SD_p, double packet_ttl, double *max_constraint_value, int *max_constraint_value_target_point_id)
{ //compute the optimization value for an intersection p that is an optimal target point candidate given the the packet delivery delay distribution (EDD_p, EDD_SD_p) and the destination vehicle arrival delay distribution (EAD_p, EAD_SD_p). Note that max_constraint_value and max_constraint_value_target_point_id are used only in the case where there is no target point to satisfy the required constraint, such as delivery probability threshold

    double optimization_value = 0; //value for target point optimization
    double constraint_value = 0; //value for the constraint, such as delivery probability
    double c = 3; //coefficient for delay deviation width;    (mu - c*sigma, mu + c*sigma) covers 99% in the delay distribution supposing that the distribution is Normal.
    double mu_p = EDD_p; //packet delay average
    double sigma_p = EDD_SD_p; //packet delay standard deviation
    double mu_v = EAD_p; //vehicle delay average
    //double sigma_v = EAD_p; //vehicle delay standard deviation
    double sigma_v = EAD_SD_p; //vehicle delay standard deviation; @Note that I made a mistake that sigma_v is set to EAD_p, not EAD_p
    double interval_start = 0; //the start of the integration interval
    //double interval_start = -param->communication_packet_ttl*10; //the start of the integration interval
    //double interval_start = -param->communication_packet_ttl; //the start of the integration interval
    //double interval_end = INF; //the end of the integration interval
    //double interval_end = param->communication_packet_ttl; //the end of the integration interval

    //double interval_end = param->communication_packet_ttl*10; //the end of the integration interval
    //double interval_end = param->communication_packet_ttl; //the end of the integration interval; @Note that this interval_end makes the probability less than the user-defined threshold, so we use the 10 times of TTL as the end of the interval
    //double interval_end = INF; //the end of the integration interval
    double interval_end = packet_ttl; //the end of the integration interval; @Note that this interval_end makes the probability less than the user-defined threshold, so we use the 10 times of TTL as the end of the interval

    switch(param->vehicle_vanet_target_point_optimization_function_type)
    {
        case VANET_TARGET_POINT_OPTIMIZATION_FUNCTION_EDD_AND_EAD_DIFFERENCE:
        //difference of EDD_p and EAD_p
            constraint_value = 0;
            optimization_value = fabs(EDD_p - EAD_p);

            break;

        case VANET_TARGET_POINT_OPTIMIZATION_FUNCTION_EAD_WITH_CONSTRAINT_1:
        //EAD satisfying the constraint 1 that EDD_p + c*EDD_SD_p <= EAD_p - c*EAD_SD_p <===> EAD_p - EDD_p >= c*EAD_SD_p + c*EDD_SD_p, where c is the coefficient for delay deviation width
            /** check whether the packet arrives at the target intersection p earlier than the destination vehicle */
            /* check that the EDD_p is close to zero and the EAD_p is positive; if so, the delivery probability must be 1. */
            if(EDD_p <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC && EAD_p > ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
                constraint_value = 1;
            else
                constraint_value = EAD_p - EDD_p;
            if(constraint_value >= c*EAD_SD_p)
            //if((EAD_p - EDD_p) >= (c*EAD_SD_p + EDD_SD_p))
            //if((EAD_p - EDD_p) >= (c*EAD_SD_p + c*EDD_SD_p))
                optimization_value = EAD_p;
            else
                optimization_value = INF;

            break;

        case VANET_TARGET_POINT_OPTIMIZATION_FUNCTION_EAD_WITH_CONSTRAINT_2:
        //EAD satisfying the constraint 2 that P(DD_p > AD_p) <= epsilon <===> P(DD_p <= AD_p) >= 1 - epsilon, where epsilon is missing probability threshold 
            /* check that the EDD_p is close to zero and the EAD_p is positive; if so, the delivery probability must be 1. */
            if(EDD_p <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC && EAD_p > ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
                constraint_value = 1;
            else
                constraint_value = GSL_Vanet_Delivery_Probability_For_Gamma_Distribution(mu_p, sigma_p, mu_v, sigma_v, interval_start, interval_end); //compute the delivery probability for two Gamma random variables: (i) Packet delay of N(mu_p, sigma_p) and (ii) Vehicle delay of N(mu_v, sigma_v)
                //constraint_value = GSL_Vanet_Delivery_Probability_For_Gaussian_Distribution(mu_p, sigma_p, mu_v, sigma_v, interval_start, interval_end); //compute the delivery probability for two Gaussian random variables: (i) Packet delay of N(mu_p, sigma_p) and (ii) Vehicle delay of N(mu_v, sigma_v)

            /** check whether the packet arrives at the target intersection p earlier than the destination vehicle */
            if(constraint_value >= param->communication_packet_delivery_probability_threshold)
                optimization_value = EAD_p;
            else
                optimization_value = INF;

            break;

        default:
            printf("Compute_TargetPoint_OptimizationValue(): param->vehicle_vanet_target_point_optimization_function_type(%d) is not supported yet!\n", param->vehicle_vanet_target_point_optimization_function_type);
            exit(1);
    }

    /* update the maximum constraint value and the corresponding target point */
    if(constraint_value > *max_constraint_value)
    {
        *max_constraint_value = constraint_value;
        *max_constraint_value_target_point_id = target_point_id;
    }

    return optimization_value;
}

boolean Is_Vertex_On_VehicleTrajectory_With_New_Destination(parameter_t *param, double current_time, packet_queue_node_t *packet, int vertex, int *new_dst, boolean *packet_earlier_arrival_flag)
//check whether vertex is on the destination vehicle's trajectory; if so, this function returns an adjacent vertex closer to the destination vehicle's current position through new_dst
{
    boolean result = FALSE; //result flag

    result = Is_Vertex_On_VehicleTrajectory_With_New_Destination_VERSION_1(param, current_time, packet, vertex, new_dst, packet_earlier_arrival_flag);
    return result;
}

boolean Is_Vertex_On_VehicleTrajectory_With_New_Destination_VERSION_1(parameter_t *param, double current_time, packet_queue_node_t *packet, int vertex, int *new_dst, boolean *packet_earlier_arrival_flag)
{ //Using the estimation of the destination vehicle's position, check whether vertex is on the destination vehicle's trajectory; if so, this function returns an adjacent vertex closer to the destination vehicle's current position through new_dst
    boolean result = FALSE; //result flag
    int vertex_hop = INF; //vertex hop for the destination vehicle's trajectory where the first node on the vehicle trajectory has 0 as its hop
    char vertex_name[NAME_SIZE]; //vertex name
    vehicle_trajectory_queue_t *pTrajectory_Queue = &(packet->vehicle_trajectory); //pointer to the packet's vehicle trajectory
    vehicle_trajectory_queue_node_t *pVertex_Trajectory_QNode = NULL; //pointer to the vehicle trajectory queue node corresponding to vertex
    vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode = NULL; //pointer to the current trajectory queue node for the edge where the destination vehicle is moving

    int path_current_hop = 0; //destination vehicle's current hop at this current time
    double offset_end = 0; //destination vehicle's offset in the current edge on its trajectory for current_time

    stationary_node_queue_node_t *stationary_node = NULL; //pointer to the stationary node corresponding to intersection_id

    double EDD_p = 0; //Expected Delivery Delay (EDD) for target point p
    double EAD_p = 0; //Expected Arrival Delay (EAD) for target point p
    int new_target_point = 0; //new target point considering the destination vehicle's current position

    double destination_vehicle_latest_passing_time = 0; //the destination vehicle's latest passing time for an intersection
    double time_difference = 0; //time difference between two time instants
    double aging_time_threshold = packet->ttl/4; //threshold for the aging time to hold the latest passing time of the destination vehicle in the stationary node placed at the target point intersection
    double communication_range_passing_time_threshold = param->communication_range/param->vehicle_speed; //the time taken for a vehicle to pass from the intersection (having a stationary node) out of the communication range of the stationary node

    /** set packet_earlier_arrival_flag to FALSE */
    *packet_earlier_arrival_flag = FALSE;

    /** get the stationary node for vertex */
    if(vertex > param->vanet_table.Gr_size)
    {
        printf("Is_Vertex_On_VehicleTrajectory_With_New_Destination(): Error: intersection_id(%d) > param->vanet_table.Gr_size(%d)\n", vertex, param->vanet_table.Gr_size);
        exit(1);
    }
    stationary_node = param->vanet_table.Gr[vertex-1].ptr_stationary_node;

    /** check the size of the vehicle trajectory queue */
    if(pTrajectory_Queue->size == 0)
    {
        printf("Is_Vertex_On_VehicleTrajectory_With_New_Destination(): packet's vehicle trajectory size(%d) must greater than 0\n", packet->vehicle_trajectory.size);
        exit(1);
    }

    /** find the hop of vertex on the destination vehicle trajectory */
    itoa(vertex, vertex_name); //convert vertex id into vertex name
    pVertex_Trajectory_QNode = Find_VehicleTrajectoryQueueNode_For_VertexHop_On_VehicleTrajectory(pTrajectory_Queue, vertex_name);
    if(pVertex_Trajectory_QNode == NULL)
    {
        //printf("Is_Vertex_On_VehicleTrajectory_With_New_Destination(): there is no vehicle trajectory queue node corresponding to vertex(%s)\n", vertex_name);
        //return result;

        new_target_point = GetTargetPoint_For_Packet(param, current_time, packet, stationary_node->intersection_id, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory

        *new_dst = new_target_point;
        result = FALSE;

        return result;        
    }

    vertex_hop = pVertex_Trajectory_QNode->order; //get the vertex hop for pVertexTrajectory_QNode

    /** update the reverse_traversal_current_hop_trajectory_qnode with the trajectory queue node whose tail node is the vertex having the packet */
    //packet->reverse_traversal_current_hop_trajectory_qnode = pVertex_Trajectory_QNode;
     
    /** find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory for the vertex holding this packet */
    pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_Packet(param, current_time, packet, &path_current_hop, &offset_end);
    //pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_StationaryNode(param, current_time, stationary_node, &path_current_hop, &offset_end);

    /** check whether the destination vehicle has passed out of its trajectory or not */
    if((param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE) && (pCurrent_Trajectory_QNode == NULL))
    { //choose a random neighboring node of vertex, such that the first neighbor in the adjacency list
        printf("Is_Vertex_On_VehicleTrajectory_With_New_Destination(): the vehicle trajectory has expired, so it cannot be used any mode\n");
        //*new_dst = atoi(param->vanet_table.Gr[vertex-1].next->vertex);
        *new_dst = vertex; //let the packet stay at vertex since the packet's trajectory has expired
        result = TRUE;

        return result;
    }
 
    /** check the destination vehicle's passing time at this stationary node for the debugging purpose */
    destination_vehicle_latest_passing_time = stationary_node->destination_vehicle_latest_passing_time; //latest passing time
    time_difference = current_time - destination_vehicle_latest_passing_time; //time difference that is the interval between the destination vehicle's passing time and the current time

    /** When the packet has arrived at the target point on the destination vehicle's trajectory, we let the packet move reversely along the vehicle trajectory; this allows the same delivery probability with the static forwarding and reduce the overall delivery delay */

    /* choose the name hop for new_dst with vertex_hop and the destination vehicle's  path_current_hop */
	if((vertex_hop - path_current_hop > 0) && (vertex_hop - path_current_hop <= param->communication_packet_reverse_traversal_hop_distance_threshold))
    //if((vertex_hop - path_current_hop > 0) && (vertex_hop - path_current_hop <= REVERSE_TRAVERSAL_HOP_DISTANCE_THRESHOLD))
    { //In the case where vertex is before the destination vehicle on the vehicle trajectory, we let the packet traverse reversely along the destination vehicle's trajectory.
        *new_dst = atoi(pVertex_Trajectory_QNode->prev->graph_pos.enode->tail_node);

        result = TRUE; //indicate that the packet has arrived at the target point on the destination vehicle's trajectory

        /** set packet_earlier_arrival_flag to TRUE */
        *packet_earlier_arrival_flag = TRUE;

        /** update the reverse_traversal_current_hop_trajectory_qnode with the trajectory queue node whose tail node is the vertex having the packet */
        packet->reverse_traversal_mode_flag = TRUE; //enable the reverse traversal mode
        packet->reverse_traversal_current_hop_trajectory_qnode = pVertex_Trajectory_QNode;
    }
    else
    {
        new_target_point = GetTargetPoint_For_Packet(param, current_time, packet, stationary_node->intersection_id, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory

        *new_dst = new_target_point;
        result = TRUE;

        /** set packet_earlier_arrival_flag to FALSE */
        *packet_earlier_arrival_flag = FALSE;

        return result;        
    }

    ///////////////////////////////////////////////////////////////////////////////////

    /** choose the name hop for new_dst with vertex_hop and the destination vehicle's  path_current_hop */
/*     if((vertex_hop - path_current_hop > 0) && (vertex_hop - path_current_hop <= REVERSE_TRAVERSAL_HOP_DISTANCE_THRESHOLD))  */
/*     //In the case where the packet is only before one hop, we enable the reverse traversal mode; otherwise, we let the packet be delivered towards a new target point using the whole road network, not limited by the destination vehicle's trajectory */
/*     //if(vertex_hop > path_current_hop) */
/*     { //In the case where vertex is before the destination vehicle on the vehicle trajectory, we let the packet traverse reversely along the destination vehicle's trajectory. */
/*         *new_dst = atoi(pVertex_Trajectory_QNode->prev->graph_pos.enode->tail_node); */

/*         result = TRUE; */

/*         /\** set packet_earlier_arrival_flag to TRUE *\/ */
/*         *packet_earlier_arrival_flag = TRUE; */

/*         /\** update the reverse_traversal_current_hop_trajectory_qnode with the trajectory queue node whose tail node is the vertex having the packet *\/ */
/*         packet->reverse_traversal_mode_flag = TRUE; //enable the reverse traversal mode */
/*         packet->reverse_traversal_current_hop_trajectory_qnode = pVertex_Trajectory_QNode; */
/*     } */
/*     else if(vertex_hop == path_current_hop) */
/*     { //In the case where vertex is the tail node of the edge where the destination vehicle is moving on the vehicle trajectory, we try to forward the packet over this edge. */
/*         *new_dst = atoi(pVertex_Trajectory_QNode->graph_pos.enode->head_node); */

/*         result = TRUE; //we consider that the packet has arrived earlier at the target point than the destination vehicle even though the packet is a little behind the destination vehicle */

/*         /\** set packet_earlier_arrival_flag to TRUE *\/ */
/*         *packet_earlier_arrival_flag = TRUE; */

/*         /\** update the reverse_traversal_current_hop_trajectory_qnode with the trajectory queue node whose tail node is the vertex having the packet *\/ */
/*         packet->reverse_traversal_mode_flag = TRUE; //enable the reverse traversal mode */
/*         packet->reverse_traversal_current_hop_trajectory_qnode = pVertex_Trajectory_QNode; */
/*     } */
/*     else //vertex_hop < path_current_hop */
/*     { //In the case where vertex is behind the destination vehicle on the vehicle trajectory, we search a new target point and then let the packet go towards the new target point. */
/*         new_target_point = GetTargetPoint_For_Packet(param, current_time, packet, stationary_node->intersection_id, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory */
/*         //new_target_point = GetTargetPoint_For_StationaryNode(param, current_time, stationary_node, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory */

/* /\*         if(new_target_point == vertex) *\/ */
/* /\*             *new_dst = atoi(pVertex_Trajectory_QNode->graph_pos.enode->head_node); //the new destination is the neighboring node towards the destination vehicle *\/ */
/* /\*         else *\/ */
/* /\*             *new_dst = new_target_point; //the new destination is a new target point *\/ */

/*         *new_dst = new_target_point; */

/*         result = TRUE; */

/*         /\** set packet_earlier_arrival_flag to FALSE *\/ */
/*         *packet_earlier_arrival_flag = FALSE; */
/*     } */

    return result;
}

boolean Is_Vertex_On_VehicleTrajectory_With_New_Destination_VERSION_2(parameter_t *param, double current_time, packet_queue_node_t *packet, int vertex, int *new_dst, boolean *packet_earlier_arrival_flag)
{ //Using the destination vehicle passing time for the target point, check whether vertex is on the destination vehicle's trajectory; if so, this function returns an adjacent vertex closer to the destination vehicle's current position through new_dst
    boolean result = FALSE; //result flag
    int vertex_hop = INF; //vertex hop for the destination vehicle's trajectory where the first node on the vehicle trajectory has 0 as its hop
    char vertex_name[NAME_SIZE]; //vertex name
    vehicle_trajectory_queue_t *pTrajectory_Queue = &(packet->vehicle_trajectory); //pointer to the packet's vehicle trajectory
    vehicle_trajectory_queue_node_t *pVertex_Trajectory_QNode = NULL; //pointer to the vehicle trajectory queue node corresponding to vertex
    vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode = NULL; //pointer to the current trajectory queue node for the edge where the destination vehicle is moving

    int path_current_hop = 0; //destination vehicle's current hop at this current time
    double offset_end = 0; //destination vehicle's offset in the current edge on its trajectory for current_time

    stationary_node_queue_node_t *stationary_node = NULL; //pointer to the stationary node corresponding to intersection_id

    double EDD_p = 0; //Expected Delivery Delay (EDD) for target point p
    double EAD_p = 0; //Expected Arrival Delay (EAD) for target point p
    int new_target_point = 0; //new target point considering the destination vehicle's current position

    double destination_vehicle_latest_passing_time = 0; //the destination vehicle's latest passing time for an intersection
    double time_difference = 0; //time difference between two time instants
    double aging_time_threshold = packet->ttl/4; //threshold for the aging time to hold the latest passing time of the destination vehicle in the stationary node placed at the target point intersection
    double communication_range_passing_time_threshold = param->communication_range/param->vehicle_speed; //the time taken for a vehicle to pass from the intersection (having a stationary node) out of the communication range of the stationary node

    /** set packet_earlier_arrival_flag to FALSE */
    *packet_earlier_arrival_flag = FALSE;

    /** get the stationary node for vertex */
    if(vertex > param->vanet_table.Gr_size)
    {
        printf("Is_Vertex_On_VehicleTrajectory_With_New_Destination(): Error: intersection_id(%d) > param->vanet_table.Gr_size(%d)\n", vertex, param->vanet_table.Gr_size);
        exit(1);
    }
    stationary_node = param->vanet_table.Gr[vertex-1].ptr_stationary_node;

    /** check the size of the vehicle trajectory queue */
    if(pTrajectory_Queue->size == 0)
    {
        printf("Is_Vertex_On_VehicleTrajectory_With_New_Destination(): packet's vehicle trajectory size(%d) must greater than 0\n", packet->vehicle_trajectory.size);
        exit(1);
    }

    /** find the hop of vertex on the destination vehicle trajectory */
    itoa(vertex, vertex_name); //convert vertex id into vertex name
    pVertex_Trajectory_QNode = Find_VehicleTrajectoryQueueNode_For_VertexHop_On_VehicleTrajectory(pTrajectory_Queue, vertex_name);
    if(pVertex_Trajectory_QNode == NULL)
    {
        //printf("Is_Vertex_On_VehicleTrajectory_With_New_Destination(): there is no vehicle trajectory queue node corresponding to vertex(%s)\n", vertex_name);
        return result;
    }

    vertex_hop = pVertex_Trajectory_QNode->order; //get the vertex hop for pVertexTrajectory_QNode

    /** update the reverse_traversal_current_hop_trajectory_qnode with the trajectory queue node whose tail node is the vertex having the packet */
    //packet->reverse_traversal_current_hop_trajectory_qnode = pVertex_Trajectory_QNode;
     
    /** find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory for the vertex holding this packet */
    pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_Packet(param, current_time, packet, &path_current_hop, &offset_end);
    //pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_StationaryNode(param, current_time, stationary_node, &path_current_hop, &offset_end);

    /** check whether the destination vehicle has passed out of its trajectory or not */
    if((param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE) && (pCurrent_Trajectory_QNode == NULL))
    { //choose a random neighboring node of vertex, such that the first neighbor in the adjacency list
        printf("Is_Vertex_On_VehicleTrajectory_With_New_Destination(): the vehicle trajectory has expired, so it cannot be used any mode\n");
        //*new_dst = atoi(param->vanet_table.Gr[vertex-1].next->vertex);
        *new_dst = vertex; //let the packet stay at vertex since the packet's trajectory has expired
        result = TRUE;

        return result;
    }
 
    /** check the destination vehicle's passing time at this stationary node */
    destination_vehicle_latest_passing_time = stationary_node->destination_vehicle_latest_passing_time; //latest passing time
    time_difference = current_time - destination_vehicle_latest_passing_time; //time difference that is the interval between the destination vehicle's passing time and the current time
    if((time_difference < aging_time_threshold) && (time_difference > communication_range_passing_time_threshold))
    { //In the case where the destination vehicle has already passed the intersection corresponding to the target point and is out of the communication range of the stationary node of this intersection, we let the packet select a new target point
        /** we need to select another optimal target point */
        new_target_point = GetTargetPoint_For_Packet(param, current_time, packet, stationary_node->intersection_id, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory

        *new_dst = new_target_point;

        result = TRUE;
    }
    else if((time_difference < aging_time_threshold) && (time_difference <= communication_range_passing_time_threshold))
    { //In the case where the destination vehicle has already passed the intersection corresponding to the target point but is within the communication range of the stationary node of this intersection, we let the packet be sent to the heading intersection where the destination vehicle is heading

        *new_dst = atoi(pVertex_Trajectory_QNode->graph_pos.enode->head_node);

        result = TRUE;
    }
    else
    { //the case where the packet has arrived at this intersection earlier than the destination vehicle 
        /** choose the name hop for new_dst with vertex_hop and the destination vehicle's  path_current_hop */
		if((vertex_hop - path_current_hop > 0) && (vertex_hop - path_current_hop <= param->communication_packet_reverse_traversal_hop_distance_threshold)) 
        //if((vertex_hop - path_current_hop > 0) && (vertex_hop - path_current_hop <= REVERSE_TRAVERSAL_HOP_DISTANCE_THRESHOLD)) 
        //if(vertex_hop > path_current_hop)
        { //In the case where vertex is before the destination vehicle on the vehicle trajectory, we let the packet traverse reversely along the destination vehicle's trajectory.
            *new_dst = atoi(pVertex_Trajectory_QNode->prev->graph_pos.enode->tail_node);

            /** update the reverse_traversal_current_hop_trajectory_qnode with the trajectory queue node whose tail node is the vertex having the packet */
            packet->reverse_traversal_mode_flag = TRUE; //enable the reverse traversal mode
            packet->reverse_traversal_current_hop_trajectory_qnode = pVertex_Trajectory_QNode;

            result = TRUE;
 
            /** set packet_earlier_arrival_flag to TRUE */
           *packet_earlier_arrival_flag = TRUE;
        }
        else if(vertex_hop == path_current_hop)
        { //In the case where vertex is the tail node of the edge where the destination vehicle is moving on the vehicle trajectory; we try to forward the packet over this edge, we try to forward the packet over this edge.
            *new_dst = atoi(pVertex_Trajectory_QNode->graph_pos.enode->head_node);

            /** update the reverse_traversal_current_hop_trajectory_qnode with the trajectory queue node whose tail node is the vertex having the packet */
            //packet->reverse_traversal_mode_flag = TRUE; //enable the reverse traversal mode
            //packet->reverse_traversal_current_hop_trajectory_qnode = pVertex_Trajectory_QNode;

            //result = TRUE; //we consider that the packet has arrived earlier at the target point than the destination vehicle even though the packet is a little behind the destination vehicle

            result = TRUE;    

            /** set packet_earlier_arrival_flag to FALSE */
            *packet_earlier_arrival_flag = FALSE;
        }
        else //vertex_hop < path_current_hop
        { //In the case where vertex is behind the destination vehicle on the vehicle trajectory, we search a new target point and then let the packet go towards the new target point.
            new_target_point = GetTargetPoint_For_Packet(param, current_time, packet, stationary_node->intersection_id, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory
            //new_target_point = GetTargetPoint_For_StationaryNode(param, current_time, stationary_node, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory

            /*         if(new_target_point == vertex) */
            /*             *new_dst = atoi(pVertex_Trajectory_QNode->graph_pos.enode->head_node); //the new destination is the neighboring node towards the destination vehicle */
            /*         else */
            /*             *new_dst = new_target_point; //the new destination is a new target point */

            *new_dst = new_target_point;

            result = TRUE;

            /** set packet_earlier_arrival_flag to FALSE */
            *packet_earlier_arrival_flag = FALSE;
        }
    }

    /** choose the name hop for new_dst with vertex_hop and the destination vehicle's  path_current_hop */
   	if((vertex_hop - path_current_hop > 0) && (vertex_hop - path_current_hop <= param->communication_packet_reverse_traversal_hop_distance_threshold)) 
    //if((vertex_hop - path_current_hop > 0) && (vertex_hop - path_current_hop <= REVERSE_TRAVERSAL_HOP_DISTANCE_THRESHOLD)) 
    //In the case where the packet is only before one hop, we enable the reverse traversal mode; otherwise, we let the packet be delivered towards a new target point using the whole road network, not limited by the destination vehicle's trajectory
    //if(vertex_hop > path_current_hop)
    { //In the case where vertex is before the destination vehicle on the vehicle trajectory, we let the packet traverse reversely along the destination vehicle's trajectory.
        *new_dst = atoi(pVertex_Trajectory_QNode->prev->graph_pos.enode->tail_node);

        result = TRUE;

        /** set packet_earlier_arrival_flag to TRUE */
        *packet_earlier_arrival_flag = TRUE;

        /** update the reverse_traversal_current_hop_trajectory_qnode with the trajectory queue node whose tail node is the vertex having the packet */
        packet->reverse_traversal_mode_flag = TRUE; //enable the reverse traversal mode
        packet->reverse_traversal_current_hop_trajectory_qnode = pVertex_Trajectory_QNode;
    }
    else if(vertex_hop == path_current_hop)
    { //In the case where vertex is the tail node of the edge where the destination vehicle is moving on the vehicle trajectory, we try to forward the packet over this edge.
        *new_dst = atoi(pVertex_Trajectory_QNode->graph_pos.enode->head_node);

        result = TRUE; //we consider that the packet has arrived earlier at the target point than the destination vehicle even though the packet is a little behind the destination vehicle

        /** set packet_earlier_arrival_flag to TRUE */
        *packet_earlier_arrival_flag = TRUE;

        /** update the reverse_traversal_current_hop_trajectory_qnode with the trajectory queue node whose tail node is the vertex having the packet */
        packet->reverse_traversal_mode_flag = TRUE; //enable the reverse traversal mode
        packet->reverse_traversal_current_hop_trajectory_qnode = pVertex_Trajectory_QNode;
    }
    else //vertex_hop < path_current_hop
    { //In the case where vertex is behind the destination vehicle on the vehicle trajectory, we search a new target point and then let the packet go towards the new target point.
        new_target_point = GetTargetPoint_For_Packet(param, current_time, packet, stationary_node->intersection_id, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory
        //new_target_point = GetTargetPoint_For_StationaryNode(param, current_time, stationary_node, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory

/*         if(new_target_point == vertex) */
/*             *new_dst = atoi(pVertex_Trajectory_QNode->graph_pos.enode->head_node); //the new destination is the neighboring node towards the destination vehicle */
/*         else */
/*             *new_dst = new_target_point; //the new destination is a new target point */

        *new_dst = new_target_point;

        result = TRUE;

        /** set packet_earlier_arrival_flag to FALSE */
        *packet_earlier_arrival_flag = FALSE;
    }

    return result;
}

boolean Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination(parameter_t *param, double current_time, packet_queue_node_t *packet, int *new_dst)
{ //check whether the packet has already arrived earlier at the target point than the destination vehicle or not; regardless of the result, it returns a new destination node
    boolean result = FALSE; //result flag

    result = Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination_VERSION_1(param, current_time, packet, new_dst);
    return result;
}

boolean Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination_VERSION_1(parameter_t *param, double current_time, packet_queue_node_t *packet, int *new_dst)
{ //Using the estimation of the destination vehicle's position, check whether the packet has already arrived earlier at the target point than the destination vehicle or not; regardless of the result, it returns a new destination node
    boolean result = FALSE; //result
    int vertex = packet->target_point_id; //vertex corresponding to the packet's target point
    int vertex_hop = INF; //vertex hop for the destination vehicle's trajectory where the first node on the vehicle trajectory has 0 as its hop
    char vertex_name[NAME_SIZE]; //vertex name
    vehicle_trajectory_queue_t *pTrajectory_Queue = &(packet->vehicle_trajectory); //pointer to the packet's vehicle trajectory
    vehicle_trajectory_queue_node_t *pVertex_Trajectory_QNode = NULL; //pointer to the vehicle trajectory queue node corresponding to vertex
    vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode = NULL; //pointer to the current trajectory queue node for the edge where the destination vehicle is moving

    int path_current_hop = 0; //destination vehicle's current hop at this current time
    double offset_end = 0; //destination vehicle's offset in the current edge on its trajectory for current_time

    stationary_node_queue_node_t *stationary_node = NULL; //pointer to the stationary node corresponding to intersection_id

    double EDD_p = 0; //Expected Delivery Delay (EDD) for target point p
    double EAD_p = 0; //Expected Arrival Delay (EAD) for target point p
    int new_target_point = 0; //new target point considering the destination vehicle's current position

    double destination_vehicle_latest_passing_time = 0; //the destination vehicle's latest passing time for an intersection
    double time_difference = 0; //time difference between two time instants
    double aging_time_threshold = packet->ttl/4; //threshold for the aging time to hold the latest passing time of the destination vehicle in the stationary node placed at the target point intersection
    double communication_range_passing_time_threshold = param->communication_range/param->vehicle_speed; //the time taken for a vehicle to pass from the intersection (having a stationary node) out of the communication range of the stationary node

    /*@ for debugging */
    //(current_time >= 8615.2468 && packet->id == 34)
    //  printf("Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination_VERSION_1(): for debugging\n");
    /*****************/

    /** get the stationary node for vertex */
    if(vertex > param->vanet_table.Gr_size)
    {
        printf("Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination(): Error: intersection_id(%d) > param->vanet_table.Gr_size(%d)\n", vertex, param->vanet_table.Gr_size);
        exit(1);
    }
    stationary_node = param->vanet_table.Gr[vertex-1].ptr_stationary_node;

    /** check the size of the vehicle trajectory queue */
    if(pTrajectory_Queue->size == 0)
    {
        printf("Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination(): packet's vehicle trajectory size(%d) must greater than 0\n", packet->vehicle_trajectory.size);
        exit(1);
    }

    /** find the hop of vertex on the destination vehicle trajectory */
    itoa(vertex, vertex_name); //convert vertex id into vertex name
    pVertex_Trajectory_QNode = Find_VehicleTrajectoryQueueNode_For_VertexHop_On_VehicleTrajectory(pTrajectory_Queue, vertex_name);
    if(pVertex_Trajectory_QNode == NULL)
    {
        printf("Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination(): there is no vehicle trajectory queue node corresponding to vertex(%s)\n", vertex_name);

        exit(1);
        //return result;
    }

    vertex_hop = pVertex_Trajectory_QNode->order; //get the vertex hop for pVertexTrajectory_QNode

    /** find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory for the vertex holding this packet */
    pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_Packet(param, current_time, packet, &path_current_hop, &offset_end);
    //pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_StationaryNode(param, current_time, stationary_node, &path_current_hop, &offset_end);

    /** check whether the destination vehicle has passed out of its trajectory or not */
    if((param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE) && (pCurrent_Trajectory_QNode == NULL))
    { //choose a random neighboring node of vertex, such that the first neighbor in the adjacency list
        printf("Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination(): the vehicle trajectory has expired, so it cannot be used any mode\n");
        //*new_dst = atoi(param->vanet_table.Gr[vertex-1].next->vertex);
        *new_dst = vertex; //new_dst is the same as vertex, letting the packet stay at vertex

        return result;
    }
 
    /** check the destination vehicle's passing time at this stationary node for the debugging purpose */
    destination_vehicle_latest_passing_time = stationary_node->destination_vehicle_latest_passing_time; //latest passing time
    time_difference = current_time - destination_vehicle_latest_passing_time; //time difference that is the interval between the destination vehicle's passing time and the current time

    /** When the packet has arrived at the target point on the destination vehicle's trajectory, we let the packet move reversely along the vehicle trajectory; this allows the same delivery probability with the static forwarding and reduce the overall delivery delay */

    /*@ for debugging */
    //if(current_time >= 8615.2468 && packet == (packet_queue_node_t *)0x178309e0)
    //    printf("Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination_VERSION_1(): for debugging\n");
    /*****************/
    
    /*@ [05/28/2010] Commented by Paul */
/*     *new_dst = atoi(pVertex_Trajectory_QNode->prev->graph_pos.enode->tail_node); */

/*     result = TRUE; //indicate that the packet has arrived at the target point on the destination vehicle's trajectory */

/*     /\** update the reverse_traversal_current_hop_trajectory_qnode with the trajectory queue node whose tail node is the vertex having the packet *\/ */
/*     packet->reverse_traversal_mode_flag = TRUE; //enable the reverse traversal mode */
/*     packet->reverse_traversal_current_hop_trajectory_qnode = pVertex_Trajectory_QNode; */
    /*****/

    /*@ [05/28/2010] Added by Paul */
    result = TRUE; //indicate that the packet has arrived at the target point on the destination vehicle's trajectory

    if(pVertex_Trajectory_QNode->order == 0)
    { //the case where pVertex_Trajectory_QNode points to the first edge on the vehicle trajectory
        *new_dst = atoi(pVertex_Trajectory_QNode->graph_pos.enode->tail_node);

        /** update the reverse_traversal_current_hop_trajectory_qnode with the trajectory queue node whose tail node is the vertex having the packet */
        packet->reverse_traversal_mode_flag = TRUE; //enable the reverse traversal mode
        packet->reverse_traversal_current_hop_trajectory_qnode = pVertex_Trajectory_QNode;
        packet->reverse_traversal_completion_flag = TRUE; //indicate that the packet has arrived at the source intersection of the destination vehicle's trajectory
    }
    else
    { //the case where pVertex_Trajectory_QNode points to the other edge on the vehicle trajectory
        *new_dst = atoi(pVertex_Trajectory_QNode->prev->graph_pos.enode->tail_node);

        /** update the reverse_traversal_current_hop_trajectory_qnode with the trajectory queue node whose tail node is the vertex having the packet */
        packet->reverse_traversal_mode_flag = TRUE; //enable the reverse traversal mode
        packet->reverse_traversal_current_hop_trajectory_qnode = pVertex_Trajectory_QNode;
    }

    ///////////////////////////////////////////////////////////////////////////////////

/*     /\** choose the name hop for new_dst with vertex_hop and the destination vehicle's  path_current_hop *\/ */
/*     if((vertex_hop - path_current_hop > 0) && (vertex_hop - path_current_hop <= REVERSE_TRAVERSAL_HOP_DISTANCE_THRESHOLD))  */
/*     //if(vertex_hop > path_current_hop) */
/*     { //In the case where vertex is before the destination vehicle on the vehicle trajectory, we let the packet traverse reversely along the destination vehicle's trajectory. */
/*         *new_dst = atoi(pVertex_Trajectory_QNode->prev->graph_pos.enode->tail_node); */

/*         /\** update the reverse_traversal_current_hop_trajectory_qnode with the trajectory queue node whose tail node is the vertex having the packet *\/ */
/*         packet->reverse_traversal_mode_flag = TRUE; //enable the reverse traversal mode */
/*         packet->reverse_traversal_current_hop_trajectory_qnode = pVertex_Trajectory_QNode; */

/*         result = TRUE; */
/*     }         */
/* /\*     else if(vertex_hop == path_current_hop) *\/ */
/* /\*     { //In the case where vertex is the tail node of the edge where the destination vehicle is moving on the vehicle trajectory; we try to forward the packet over this edge, we try to forward the packet over this edge. *\/ */
/* /\*         *new_dst = atoi(pVertex_Trajectory_QNode->graph_pos.enode->head_node); *\/ */
/* /\*         /\\* update the reverse_traversal_current_hop_trajectory_qnode with the trajectory queue node whose tail node is the vertex having the packet *\\/ *\/ */
/* /\*         //packet->reverse_traversal_mode_flag = TRUE; //enable the reverse traversal mode *\/ */
/* /\*         //packet->reverse_traversal_current_hop_trajectory_qnode = pVertex_Trajectory_QNode; *\/ */

/* /\*         //result = TRUE; //we consider that the packet has arrived earlier at the target point than the destination vehicle even though the packet is a little behind the destination vehicle *\/ */

/* /\*         result = FALSE;     *\/ */
/* /\*     } *\/        */
/*     else //vertex_hop < path_current_hop */
/*     { //In the case where vertex is behind the destination vehicle on the vehicle trajectory, we search a new target point and then let the packet go towards the new target point. */
/*         new_target_point = GetTargetPoint_For_Packet(param, current_time, packet, stationary_node->intersection_id, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory */
/*         //new_target_point = GetTargetPoint_For_StationaryNode(param, current_time, stationary_node, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory */

/*         /\*         if(new_target_point == vertex) *\/ */
/*         /\*             *new_dst = atoi(pVertex_Trajectory_QNode->graph_pos.enode->head_node); //the new destination is the neighboring node towards the destination vehicle *\/ */
/*         /\*         else *\/ */
/*         /\*             *new_dst = new_target_point; //the new destination is a new target point *\/ */

/*         *new_dst = new_target_point; */

/*         result = FALSE; */
/*     } */

    return result;
}

boolean Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination_VERSION_2(parameter_t *param, double current_time, packet_queue_node_t *packet, int *new_dst)
{ //Using the destination vehicle passing time for the target point, check whether the packet has already arrived earlier at the target point than the destination vehicle or not; regardless of the result, it returns a new destination node
    boolean result = FALSE; //result
    int vertex = packet->target_point_id; //vertex corresponding to the packet's target point
    int vertex_hop = INF; //vertex hop for the destination vehicle's trajectory where the first node on the vehicle trajectory has 0 as its hop
    char vertex_name[NAME_SIZE]; //vertex name
    vehicle_trajectory_queue_t *pTrajectory_Queue = &(packet->vehicle_trajectory); //pointer to the packet's vehicle trajectory
    vehicle_trajectory_queue_node_t *pVertex_Trajectory_QNode = NULL; //pointer to the vehicle trajectory queue node corresponding to vertex
    vehicle_trajectory_queue_node_t *pCurrent_Trajectory_QNode = NULL; //pointer to the current trajectory queue node for the edge where the destination vehicle is moving

    int path_current_hop = 0; //destination vehicle's current hop at this current time
    double offset_end = 0; //destination vehicle's offset in the current edge on its trajectory for current_time

    stationary_node_queue_node_t *stationary_node = NULL; //pointer to the stationary node corresponding to intersection_id

    double EDD_p = 0; //Expected Delivery Delay (EDD) for target point p
    double EAD_p = 0; //Expected Arrival Delay (EAD) for target point p
    int new_target_point = 0; //new target point considering the destination vehicle's current position

    double destination_vehicle_latest_passing_time = 0; //the destination vehicle's latest passing time for an intersection
    double time_difference = 0; //time difference between two time instants
    double aging_time_threshold = packet->ttl/4; //threshold for the aging time to hold the latest passing time of the destination vehicle in the stationary node placed at the target point intersection
    double communication_range_passing_time_threshold = param->communication_range/param->vehicle_speed; //the time taken for a vehicle to pass from the intersection (having a stationary node) out of the communication range of the stationary node

    /** get the stationary node for vertex */
    if(vertex > param->vanet_table.Gr_size)
    {
        printf("Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination(): Error: intersection_id(%d) > param->vanet_table.Gr_size(%d)\n", vertex, param->vanet_table.Gr_size);
        exit(1);
    }
    stationary_node = param->vanet_table.Gr[vertex-1].ptr_stationary_node;

    /** check the size of the vehicle trajectory queue */
    if(pTrajectory_Queue->size == 0)
    {
        printf("Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination(): packet's vehicle trajectory size(%d) must greater than 0\n", packet->vehicle_trajectory.size);
        exit(1);
    }

    /** find the hop of vertex on the destination vehicle trajectory */
    itoa(vertex, vertex_name); //convert vertex id into vertex name
    pVertex_Trajectory_QNode = Find_VehicleTrajectoryQueueNode_For_VertexHop_On_VehicleTrajectory(pTrajectory_Queue, vertex_name);
    if(pVertex_Trajectory_QNode == NULL)
    {
        printf("Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination(): there is no vehicle trajectory queue node corresponding to vertex(%s)\n", vertex_name);

        exit(1);
        //return result;
    }

    vertex_hop = pVertex_Trajectory_QNode->order; //get the vertex hop for pVertexTrajectory_QNode

    /** find the vehicle trajectory queue node that is the tail node of the edge where the destination vehicle is moving at current_time along with the current hop and the offset for the current hop on the trajectory for the vertex holding this packet */
    pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_Packet(param, current_time, packet, &path_current_hop, &offset_end);
    //pCurrent_Trajectory_QNode = Find_Current_VehicleTrajectoryQueueNode_Along_With_Current_Hop_And_Offset_For_StationaryNode(param, current_time, stationary_node, &path_current_hop, &offset_end);

    /** check whether the destination vehicle has passed out of its trajectory or not */
    if((param->vehicle_vanet_vehicle_trajectory_length_type == VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE) && (pCurrent_Trajectory_QNode == NULL))
    { //choose a random neighboring node of vertex, such that the first neighbor in the adjacency list
        printf("Has_Packet_Arrived_Earlier_At_TargetPoint_Than_Destination_Vehicle_With_New_Destination(): the vehicle trajectory has expired, so it cannot be used any mode\n");
        //*new_dst = atoi(param->vanet_table.Gr[vertex-1].next->vertex);
        *new_dst = vertex; //new_dst is the same as vertex, letting the packet stay at vertex

        return result;
    }
 
    /** check the destination vehicle's passing time at this stationary node */
    destination_vehicle_latest_passing_time = stationary_node->destination_vehicle_latest_passing_time; //latest passing time
    time_difference = current_time - destination_vehicle_latest_passing_time; //time difference that is the interval between the destination vehicle's passing time and the current time
    if((time_difference < aging_time_threshold) && (time_difference > communication_range_passing_time_threshold))
    { //In the case where the destination vehicle has already passed the intersection corresponding to the target point and is out of the communication range of the stationary node of this intersection, we let the packet select a new target point
        /** we need to select another optimal target point */
        new_target_point = GetTargetPoint_For_Packet(param, current_time, packet, stationary_node->intersection_id, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory

        *new_dst = new_target_point;

        result = FALSE;
    }
    else if((time_difference < aging_time_threshold) && (time_difference <= communication_range_passing_time_threshold))
    { //In the case where the destination vehicle has already passed the intersection corresponding to the target point but is within the communication range of the stationary node of this intersection, we let the packet be sent to the heading intersection where the destination vehicle is heading

        *new_dst = atoi(pVertex_Trajectory_QNode->graph_pos.enode->head_node);

        result = FALSE;
    }
    else
    { //the case where the packet has arrived at this intersection earlier than the destination vehicle 
        /** choose the name hop for new_dst with vertex_hop and the destination vehicle's  path_current_hop */
		if((vertex_hop - path_current_hop > 0) && (vertex_hop - path_current_hop <= param->communication_packet_reverse_traversal_hop_distance_threshold)) 
        //if((vertex_hop - path_current_hop > 0) && (vertex_hop - path_current_hop <= REVERSE_TRAVERSAL_HOP_DISTANCE_THRESHOLD)) 
        //if(vertex_hop > path_current_hop)
        { //In the case where vertex is before the destination vehicle on the vehicle trajectory, we let the packet traverse reversely along the destination vehicle's trajectory.
            *new_dst = atoi(pVertex_Trajectory_QNode->prev->graph_pos.enode->tail_node);

            /** update the reverse_traversal_current_hop_trajectory_qnode with the trajectory queue node whose tail node is the vertex having the packet */
            packet->reverse_traversal_mode_flag = TRUE; //enable the reverse traversal mode
            packet->reverse_traversal_current_hop_trajectory_qnode = pVertex_Trajectory_QNode;

            result = TRUE;
        }        
/*         else if(vertex_hop == path_current_hop) */
/*         { //In the case where vertex is the tail node of the edge where the destination vehicle is moving on the vehicle trajectory; we try to forward the packet over this edge, we try to forward the packet over this edge. */
/*             *new_dst = atoi(pVertex_Trajectory_QNode->graph_pos.enode->head_node); */

/*             /\* update the reverse_traversal_current_hop_trajectory_qnode with the trajectory queue node whose tail node is the vertex having the packet *\/ */
/*             //packet->reverse_traversal_mode_flag = TRUE; //enable the reverse traversal mode */
/*             //packet->reverse_traversal_current_hop_trajectory_qnode = pVertex_Trajectory_QNode; */

/*             //result = TRUE; //we consider that the packet has arrived earlier at the target point than the destination vehicle even though the packet is a little behind the destination vehicle */

/*             result = FALSE;     */
/*         } */       
        else //vertex_hop < path_current_hop
        { //In the case where vertex is behind the destination vehicle on the vehicle trajectory, we search a new target point and then let the packet go towards the new target point.
            new_target_point = GetTargetPoint_For_Packet(param, current_time, packet, stationary_node->intersection_id, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory
            //new_target_point = GetTargetPoint_For_StationaryNode(param, current_time, stationary_node, &EDD_p, &EAD_p); //compute a new target point from the stationary node to the destination vehicle with the destination vehicle's trajectory

            /*         if(new_target_point == vertex) */
            /*             *new_dst = atoi(pVertex_Trajectory_QNode->graph_pos.enode->head_node); //the new destination is the neighboring node towards the destination vehicle */
            /*         else */
            /*             *new_dst = new_target_point; //the new destination is a new target point */

            *new_dst = new_target_point;

            result = TRUE;
        }
    }

    return result;
}

boolean Is_There_NextHop_In_PacketReverseTraversal(packet_queue_node_t *packet, int current_intersection_id, int *new_dst)
{ //check whether there exists a next hop for the packet's reverse traversal given the vehicle trajectory and the current intersection id; if so, return the next hop through next_dst
    boolean result = FALSE; //result
    vehicle_trajectory_queue_node_t *current_hop_trajectory_qnode = packet->reverse_traversal_current_hop_trajectory_qnode; //pointer to the vehicle trajectory queue node corresponding to the current hop under the reverse traversal mode
    vehicle_trajectory_queue_node_t *next_hop_trajectory_qnode = NULL; //pointer to the vehicle trajectory queue node corresponding to the next hop under the reverse traversal mode

    if(current_hop_trajectory_qnode->order == 0)
    {
        *new_dst = atoi(current_hop_trajectory_qnode->graph_pos.enode->tail_node); //set new_dst to the current intersection

        result = FALSE;

        /* set packet->reverse_traversal_next_hop_flag to FALSE since there is no next hop in the vehicle trajectory
         */
        packet->reverse_traversal_next_hop_flag = FALSE;

        /* set packet->reverse_traversal_completion_flag to TRUE since the packet has completed the reverse traverse */
        packet->reverse_traversal_completion_flag = TRUE;
    }
    else
    {
        /* set next_hop_trajectory_qnode to the previous queue node of the current hop on the vehicle trajectory for the next hop of the reverse traversal */
        next_hop_trajectory_qnode = current_hop_trajectory_qnode->prev;
        *new_dst = atoi(next_hop_trajectory_qnode->graph_pos.enode->tail_node); //set new_dst to the next hop in the reverse traversal

        result = TRUE;

        /* set packet->reverse_traversal_next_hop_flag to TRUE
           Note that reverse_traversal_nex_hop_flag is turned off when the packet is forwarded to a vehicle moving to the intended edge.
         */
        packet->reverse_traversal_next_hop_flag = TRUE;
    }

    return result;
}

boolean Update_CurrentHop_VehicleTrajectoryQueueNode_In_PacketReverseTraversal(packet_queue_node_t *packet)
{ //update the current-hop vehicle trajectory queue node with the next hop in the packet reverse traversal
    boolean result = FALSE; //result
    vehicle_trajectory_queue_node_t *trajectory_qnode = packet->reverse_traversal_current_hop_trajectory_qnode; //pointer to the vehicle trajectory queue node corresponding to the current hop under the reverse traversal mode
    
    if(trajectory_qnode->order == 0)
    {
        result = FALSE;
    }
    else
    {
        /* move to the previous queue node on the vehicle trajectory for the new current hop of the reverse traversal */
        trajectory_qnode = trajectory_qnode->prev;

        /* update the current-hop trajectory queue node */
        packet->reverse_traversal_current_hop_trajectory_qnode = trajectory_qnode;

        result = TRUE;
    }

    return result;
}

void Update_LinkCost_Information(parameter_t *param, struct_graph_node *G, int G_size)
{ //update the link cost (e.g., link utilization), link cost variance, link cost standard deviation for each edge in the graph G with the traffic statistics of vehicular traffic density, vehicle speed, and communication range
  int i = 0, j = 0; //for-loop indices
  struct_graph_node *ptr = NULL; //pointer to graph node

  for(i = 0; i < G_size; i++)
  {
    ptr = G[i].next;
    if(ptr == NULL)
      continue;

    while(ptr != NULL)
    {
      j = atoi(ptr->vertex) - 1;
      //node id starts from 1, but it should decrease by 1 for weight matrix where the id starts from 0.

#if 0 /* [ */
	  /* @TBD: We need to replace VADD_Compute_Edge_Delay() with VADD_Compute_Edge_Cost() */
      ptr->edge_cost = VADD_Compute_Edge_Delay(param, ptr); //compute the edge cost for the road segment r_ij

	  /* @TBD: We need to replace VADD_Compute_Edge_Delay_Standard_Deviation() with VADD_Compute_Edge_Cost_Standard_Deviation() */
      ptr->edge_cost_standard_deviation = VADD_Compute_Edge_Delay_Standard_Deviation(param, ptr); //compute the edge cost standard deviation for the road segment r_ij

      ptr->edge_cost_variance = pow(ptr->edge_cost_standard_deviation, 2); //compute the edge delay variance for the road segment r_ij
#endif /* ] */

#if 1 /* [[ */
      ptr->edge_cost = VADD_Compute_Average_Convoy_Length(param, ptr)/(param->communication_range); //compute the edge cost for the road segment r_ij as the number of transmission
	  ptr->edge_cost = ceil(ptr->edge_cost);

      ptr->edge_cost_standard_deviation = VADD_Compute_Average_Convoy_Length_Standard_Deviation(param, ptr)/(param->communication_range); //compute the edge cost standard deviation for the road segment r_ij
	  ptr->edge_cost_standard_deviation = ceil(ptr->edge_cost_standard_deviation);

      ptr->edge_cost_variance = pow(ptr->edge_cost_standard_deviation, 2); //compute the edge delay variance for the road segment r_ij
	  ptr->edge_cost_variance = ceil(ptr->edge_cost_variance);
#endif /* ]] */
      ptr = ptr->next;
    }
  }
}

void Init_RoadNetworkGraphNode_StationaryNodeFlag_With_StationaryNodeList(parameter_t *param, struct_graph_node *Gr, int Gr_size, struct_traffic_table *sn_table_for_Gr)
{ /* specify which intersections have their stationary node as temporary packet holder in the road network graph Gr */
	int id = 0; //intersection id
	int i = 0; //index
	struct_graph_node *ptr = NULL; //pointer to a graph node

	/* initialize the adjacency list array */
	for(i = 0; i < sn_table_for_Gr->number; i++)
	{
		/* check the allowed number of stationary nodes for Gr */
	    if(i >= param->communication_SN_maximum_number)
		{
			break;
		}

		id = atoi(sn_table_for_Gr->list[i].vertex);
		/* check the validity of id */
		if(id < 0 || id > Gr_size)
		{
			printf("%s:%d Init_RoadNetworkGraph_With_StationaryNodeList(): id(%d) is invalid where Gr_size=%d\n", __FILE__, __LINE__, id, Gr_size);
			exit(1);
		}

		/* enable stationary_node_flag */
		Gr[id-1].stationary_node_flag = TRUE;
	}

	/* initialize the neighboring head nodes for each tail node */
	for(i = 0; i < Gr_size; i++)
	{
		ptr = Gr[i].next;
		if(ptr == NULL)
			continue;

		while(ptr != NULL)
		{
			/* check whether the intersection j has stationary node or not */
			if(ptr->gnode->stationary_node_flag)
				ptr->stationary_node_flag = TRUE;

			ptr = ptr->next;
  		}
	}
}

void Set_RoadNetworkGraphNode_StationaryNodeFlag(struct_graph_node *Gr, int Gr_size)
{ /* set the stationary node flags of all the graph nodes in Gr */
	int i = 0; //index
	struct_graph_node *ptr = NULL; //pointer to a graph node

	/* initialize the neighboring head nodes for each tail node */
	for(i = 0; i < Gr_size; i++)
	{
		/* set the tail node i's stationary_node_flag to TRUE */
		Gr[i].stationary_node_flag = TRUE;

		ptr = Gr[i].next;
		if(ptr == NULL)
			continue;

		while(ptr != NULL)
		{
			/* set the head node ptr's stationary node to TRUE */
			ptr->stationary_node_flag = TRUE;

			ptr = ptr->next;
  		}
	}
}

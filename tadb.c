
#include "tpd.h"
#include "util.h"
#include "gsl-util.h"

int TADB_Get_RefTime_And_Target_Zone(parameter_t *param, struct_graph_node *Gr, struct_vehicle* receiver_vehicle, int *target_zone, double *refExpectedTime)
{
							
	struct_path_node *path_list = NULL; 
	struct_path_node *path_ptr = NULL;
	struct_set_node *tmp_path_list = NULL;
	path_list = receiver_vehicle->path_list;
	int tmpIntersection = 0;
	int valid_flag = 0;
	double maxThinkTime = 100;				
	double minCost = 99999;								
	double tmpRefExpectedTime =0; 
	int refIntersection;
	char ap_sender[20] = "25";
	int target_zone_intersection_count = 0;
	// Get all intersection on receiver trajectory
	for(path_ptr = path_list->next; path_ptr != path_list;)								
	{
		tmpIntersection = atoi(path_ptr->vertex);				
		printf("%d ",tmpIntersection);
		if (valid_flag == 1)
		{
			// intersection to calculate link cost from src to dst
			// using DEr
			double tmpCost = param->vanet_table.Dr_edc[24][tmpIntersection];
			// tail_node , head_node, move_type
			// calculate expected time Tpi
			if (tmpIntersection >= 0)
			{
				MOVE_TYPE tmpType;
				double tmpLength;
				directional_edge_queue_node_t* tmpNode;
								
				int tmpEdgeID = FastGetEdgeID_MoveType(
						Gr,path_ptr->prev->vertex,path_ptr->vertex, // input arg
						&tmpType,&tmpLength,&tmpNode); // output arg
										
				tmpRefExpectedTime += ( tmpLength / receiver_vehicle->speed ) + maxThinkTime/4;
			}			
			// calculate minimum intersection
			if (minCost > tmpCost)
			{
				minCost = tmpCost;
				refIntersection = tmpIntersection;
				*refExpectedTime = tmpRefExpectedTime;
			}																		
		}										
		if (receiver_vehicle->path_ptr == path_ptr)
		{
			valid_flag = 1; // flag for valid intersection
		}											
		path_ptr = path_ptr->next;		
	}
	printf("\n");
	// refIntersection
	// we get ref intersection = target point
	// minCostExpectedTime
	// we get expected time for min intersection
	// 3.Select the Target Zone ( tp = tv )		
	double tmpMinExpectedTime = 0;
	double tmpMaxExpectedTime = 0;
	double minExpectedTime = 0;
	double maxExpectedTime = 0;
	int maxIntersection;
	int minIntersection;
							
	minIntersection = atoi(receiver_vehicle->path_ptr->vertex);
					
	valid_flag = 0;		
	for(path_ptr = path_list->next; path_ptr != path_list;)								
	{
		tmpIntersection = atoi(path_ptr->vertex);	
								
		if (valid_flag == 1)
		{
			if (tmpIntersection >= 0)
			{
				MOVE_TYPE tmpType;
				double tmpLength;
				directional_edge_queue_node_t* tmpNode;
											
				int tmpEdgeID = FastGetEdgeID_MoveType(
						Gr,path_ptr->prev->vertex,path_ptr->vertex, // input arg
						&tmpType,&tmpLength,&tmpNode); // output arg																	
											
				tmpMinExpectedTime += ( tmpLength / receiver_vehicle->speed ) + maxThinkTime;
				tmpMaxExpectedTime += ( tmpLength / receiver_vehicle->speed );
										
				// get minIntersection  
				if (*refExpectedTime > tmpMinExpectedTime )
				{
					minIntersection = tmpIntersection;
					minExpectedTime = tmpMinExpectedTime;
				}											
				// get maxIntersection
				if (*refExpectedTime > tmpMaxExpectedTime) 
				{
					maxIntersection = tmpIntersection;
					maxExpectedTime = tmpMaxExpectedTime;
				}														
			}
		}
		if (receiver_vehicle->path_ptr == path_ptr)
		{
			valid_flag = 1; // flag for valid intersection
		}
		path_ptr = path_ptr->next;	
	}
								
	// get target zone
	valid_flag = 0;		
	for(path_ptr = path_list->next; path_ptr != path_list;)								
	{
		tmpIntersection = atoi(path_ptr->vertex);	
									
		if (tmpIntersection == minIntersection)
		{
			valid_flag = 1;
		}
									
		if (valid_flag == 1)
		{
			target_zone[target_zone_intersection_count++] = tmpIntersection;
		}
									
		if (tmpIntersection == maxIntersection)
		{
			break;
		}
		path_ptr = path_ptr->next;	
	}
								
	int target_zone_index;
	printf("The current receiver at %s. The intersections of the target zone are ",receiver_vehicle->path_ptr->vertex);
	for(target_zone_index=0;target_zone_index<target_zone_intersection_count;target_zone_index++)
	{
		printf("%d ", target_zone[target_zone_index]);
		if (target_zone_index<target_zone_intersection_count-1)
			printf(",");
	}
	printf("\n");



	return target_zone_intersection_count;
}



boolean TADB_Is_There_Next_Carrier_At_Intersection_For_AP(parameter_t *param,
	double current_time,
	struct_access_point_t *AP,
	struct_graph_node *G,
	int G_size,
	struct_vehicle_t **next_carrier)
{
//Under V2V mode, determine whether to forward AP's packets to next carrier moving on the other road segment with the highest EDR at intersection and return the pointer to the next carrier through *next_carrier
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
	}
}


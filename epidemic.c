/**
 * File: epidemic.c
 * Description: implement the data structures and operations for Epidemic Routing.
 * Origin Date: 01/29/2014
 * Update Date: 01/29/2014
 * Maker: Jaehoon Paul Jeong, pauljeong@skku.edu
 */

#include "epidemic.h"
#include "util.h"
#include "vadd.h"
#include "tpd.h" //TPD_Find_GraphNode_In_EncounterGraph()

//#define EPIDEMIC_20140520
//#define EPIDEMIC_20140602

int EPIDEMIC_Perform_Packet_Dissemination_At_Intersection_For_AP(parameter_t *param,
		double current_time,
		struct_access_point_t *AP,
		struct_graph_node *G,
		int G_size,
		packet_delivery_statistics_t *packet_delivery_stat)
{ //Under V2V mode with Epidemic Routing, perform packet dissemination at an intersection with an AP by letting the AP give its packets to each cluster head for each outgoing edge at the intersection
	int packet_copy_cluster_number = 0; //the number of clusters receiving the packet copies of vehicle for the packet dissemination at this intersection
	int size = 0; //size of intersection EDD queue
	int i = 0; //index for for-loop
	struct_graph_node *intersection_gnode = NULL; //pointer to the graph node of the intersection where the vehicle is arriving
	struct_graph_node *neighboring_intersection_gnode = NULL; //pointer to the graph node of the neighboring intersection for the intersection where the vehicle is arriving
	directional_edge_type_t edge_type = OUTGOING_EDGE; //directed edge type for tail_node
	char *tail_node_for_next_forwarding_edge = NULL; //tail_node of the next directed edge for forwarding
	char *head_node_for_next_forwarding_edge = NULL; //head_node of the next directed edge for forwarding
	struct_vehicle_t *next_carrier = NULL; //pointer to a next carrier for one outgoing directed edge
	boolean flag = FALSE; //flag to see whether there exists a next carrier for an outgoing directed edge
	int discard_count = 0; //the count for the number of packets discarded by packet lifetime expiration
	int forward_count = 0; //the count for the number of packet copies forwarded to next_vehicle
	int total_forward_count = 0; //the total count for the number of packet copies forwarded to all the convoy members
	vehicle_queue_t *Q = NULL; //the pointer to a vehicle queue in one convoy queue node
	vehicle_queue_node_t *pQueueNode = NULL; //the pointer to a vehicle queue node in one convoy queue node
	int j = 0; //index for for-loop

	/** check whether AP has packets to forward to a next carrier; if there is no packet, return FALSE without further checking for a next carrier */
	if(AP->packet_queue->size == 0)
	{
		return -1; //indicate that AP has no packets to send
	}

	/** search for a best next carrier in the order of the smallest EDDs assigned to directional edges incident to the current intersect where the vehicle has reached. */
	intersection_gnode = AP->gnode; //let AP's gnode become intersection gnode
	neighboring_intersection_gnode = intersection_gnode;
	size = (int)intersection_gnode->weight;
	for(i = 0; i < size; i++)
	{
		neighboring_intersection_gnode = neighboring_intersection_gnode->next;

		/* search for a next carrier candidate in the outgoing edge of <intersection,neighboring_intersection> */
		edge_type = OUTGOING_EDGE;
		tail_node_for_next_forwarding_edge = intersection_gnode->vertex;
		head_node_for_next_forwarding_edge = neighboring_intersection_gnode->vertex;

		flag = VADD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection_For_AP(param, current_time, AP, tail_node_for_next_forwarding_edge, head_node_for_next_forwarding_edge, edge_type, G, G_size, &next_carrier);

		if(flag == TRUE && next_carrier != NULL)
		{
			printf("AP forward to %d\n",next_carrier->id);
			/* forward the copies of the packets carried by vehicle to next_carrier */
			packet_copy_cluster_number++;
#if 0 /* [ */
			forward_count = EPIDEMIC_Forward_Packet_Copy_To_Next_Carrier(param, current_time, vehicle, next_carrier, packet_delivery_stat, &discard_count);
#endif /* ] */
			switch(param->vehicle_vanet_forwarding_type)
			{
				case VANET_FORWARDING_BASED_ON_CONVOY:
					/* let vehicle forward its packet copies to each vehicle in the convoy whose leader_vehicle is next_carrier */
					Q = &(next_carrier->ptr_convoy_queue_node->vehicle_list);
					/* check the validity of convoy queue Q */
					if(Q == NULL)
					{
						printf("%s:%d the convoy queue of next_vehicle(%d) is NULL",
								__FUNCTION__, __LINE__,
								next_carrier->id);
						exit(1);
					}

#if 0 /* [ */
					if(Q->size > 1)
					{
						printf("%s:%d the convoy_size for next_carrier(%d) is %d\n",
								__FUNCTION__, __LINE__,
								next_carrier->id, Q->size);
					}
#endif /* ] */

					pQueueNode = &(Q->head);
					for(j = 0; j < Q->size; j++)
					{
						pQueueNode = pQueueNode->next;
						forward_count = EPIDEMIC_Forward_Packet_Copy_From_AP_To_Next_Carrier(param, current_time, AP, pQueueNode->vnode, packet_delivery_stat, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier's convoy head
						total_forward_count += forward_count;
					}
					break;

				case VANET_FORWARDING_BASED_ON_VEHICLE:
					forward_count = EPIDEMIC_Forward_Packet_Copy_From_AP_To_Next_Carrier(param, current_time, AP, next_carrier, packet_delivery_stat, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier's convoy head
					total_forward_count = forward_count;
					break;

				default:
					printf("%s:%d param->vehicle_vanet_forwarding_type(%d) is not supported!\n",
							__FUNCTION__, __LINE__,
							param->vehicle_vanet_forwarding_type);
					exit(1);
			}
		}
	}

#if 1 /* [ */
	if(total_forward_count > 0)
	{
/*
#ifdef EPIDEMIC_20140602
		printf("%.2f Intersection AP(%d) forwards count %d \n",
			current_time,AP->id, total_forward_count);
#else
		printf("%s:%d AP(%d) forwards its packet copies with total_forward_count %d\n",
				__FUNCTION__, __LINE__,
				AP->id, total_forward_count);
#endif
*/
		/* delete the original packets in AP's packet_queue because AP forwarded its packets 
		 * to the neighboring vehicles.
		 * Note that this discard of the original packets is not counted as packet loss. */
		EPIDEMIC_Discard_Original_Packet_In_AP(current_time, AP);
	}
#endif /* ] */

	return packet_copy_cluster_number;
}

int EPIDEMIC_Perform_Packet_Dissemination_At_Intersection(parameter_t *param,
		double current_time,
		struct_vehicle_t *vehicle,
		struct_graph_node *G,
		int G_size,
		packet_delivery_statistics_t *packet_delivery_stat)
{ //Under V2V mode, perform packet dissemination at an intersection by letting vehicle give its packets to each cluster head for each outgoing edge at the intersection
	int packet_copy_cluster_number = 0; //the number of clusters receiving the packet copies of vehicle for the packet dissemination at this intersection
	char *tail_node = vehicle->path_ptr->vertex; //tail node of the directed edge where vehicle is moving
	char *head_node = vehicle->path_ptr->next->vertex; //head node of the directed edge where vehicle is moving
	directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the directed edge of <tail_node,head_node>
	int size = 0; //size of intersection EDD queue
	int i = 0; //index for for-loop
	struct_graph_node *intersection_gnode = NULL; //pointer to the graph node of the intersection where the vehicle is arriving
	struct_graph_node *neighboring_intersection_gnode = NULL; //pointer to the graph node of the neighboring intersection for the intersection where the vehicle is arriving
	directional_edge_type_t edge_type = OUTGOING_EDGE; //directed edge type for tail_node
	char *tail_node_for_next_forwarding_edge = NULL; //tail_node of the next directed edge for forwarding
	char *head_node_for_next_forwarding_edge = NULL; //head_node of the next directed edge for forwarding
	struct_vehicle_t *next_carrier = NULL; //pointer to a next carrier for one outgoing directed edge
	boolean flag = FALSE; //flag to see whether there exists a next carrier for an outgoing directed edge
	int discard_count = 0; //the count for the number of packets discarded by packet lifetime expiration
	int forward_count = 0; //the count for the number of packet copies forwarded to next_vehicle
	int total_forward_count = 0; //the total count for the number of packet copies forwarded to all the convoy members
	vehicle_queue_t *Q = NULL; //the pointer to a vehicle queue in one convoy queue node
	vehicle_queue_node_t *pQueueNode = NULL; //the pointer to a vehicle queue node in one convoy queue node
	int j = 0; //index for for-loop

	/* check whether vehicle has packets to forward to next carriers */
	if(vehicle->packet_queue->size ==0)
	{
		return -1; //there is no packet to send
	}

	/* obtain the pointer to the directed edge of <tail_node,head_node> */
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
	if(pEdgeNode == NULL)
	{
		printf("%s:%d: pEdgeNode for <%s,%s> is NULL!\n",
				__FUNCTION__, __LINE__,
				tail_node, head_node);
		exit(1);
	}

	/* forward one copy of each packet carried by vehicle to the cluster head of the cluster for each outgoing edge that is withing the communication range of vehicle */
	intersection_gnode = pEdgeNode->head_gnode->gnode;
	neighboring_intersection_gnode = intersection_gnode;
	size = (int)intersection_gnode->weight;
	for(i = 0; i < size; i++)
	{
		neighboring_intersection_gnode = neighboring_intersection_gnode->next;

		/* search for a next carrier candidate in the outgoing edge of <intersection,neighboring_intersection> */
		edge_type = OUTGOING_EDGE;
		tail_node_for_next_forwarding_edge = intersection_gnode->vertex;
		head_node_for_next_forwarding_edge = neighboring_intersection_gnode->vertex;

		flag = VADD_Is_There_Next_Carrier_On_Road_Segment_Incident_To_Intersection(param, current_time, vehicle, tail_node_for_next_forwarding_edge, head_node_for_next_forwarding_edge, edge_type, G, G_size, &next_carrier);

		if(flag == TRUE && next_carrier != NULL)
		{
			//printf("Intersection to vehicle(%d)\n",next_carrier->id);
			/* forward the copies of the packets carried by vehicle to next_carrier */
			packet_copy_cluster_number++;
#if 0 /* [ */
			forward_count = EPIDEMIC_Forward_Packet_Copy_To_Next_Carrier(param, current_time, vehicle, next_carrier, packet_delivery_stat, &discard_count);
#endif /* ] */
			switch(param->vehicle_vanet_forwarding_type)
			{
				case VANET_FORWARDING_BASED_ON_CONVOY:
					/* let vehicle forward its packet copies to each vehicle in the convoy whose leader_vehicle is next_carrier */
					Q = &(next_carrier->ptr_convoy_queue_node->vehicle_list);
					/* check the validity of convoy queue Q */
					if(Q == NULL)
					{
						printf("%s:%d the convoy queue of next_vehicle(%d) is NULL",
								__FUNCTION__, __LINE__,
								next_carrier->id);
						exit(1);
					}

#if 1 /* [ */
					if(Q->size > 1)
					{
						//printf("%s:%d the convoy_size for next_carrier(%d) is %d\n",
						//		__FUNCTION__, __LINE__,
						//		next_carrier->id, Q->size);
					}
#endif /* ] */

					pQueueNode = &(Q->head);
					for(j = 0; j < Q->size; j++)
					{
						pQueueNode = pQueueNode->next;
						forward_count = EPIDEMIC_Forward_Packet_Copy_To_Next_Carrier(param, current_time, vehicle, pQueueNode->vnode, packet_delivery_stat, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier's convoy head
						total_forward_count += forward_count;
					}
					break;

				case VANET_FORWARDING_BASED_ON_VEHICLE:
					forward_count = EPIDEMIC_Forward_Packet_Copy_To_Next_Carrier(param, current_time, vehicle, next_carrier, packet_delivery_stat, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier's convoy head
					total_forward_count = forward_count;
					break;

				default:
					printf("%s:%d param->vehicle_vanet_forwarding_type(%d) is not supported!\n",
							__FUNCTION__, __LINE__,
							param->vehicle_vanet_forwarding_type);
					exit(1);
			}
		}
	}

#if 1 /* [ */
	if(total_forward_count > 0)
	{
/*
#ifdef EPIDEMIC_20140602
		printf("Intersection vehicle(%d) forwards count %d pos(%s,%s)\n",	
			vehicle->id, 
			total_forward_count,
			vehicle->current_pos_in_digraph.tail_node,
			vehicle->current_pos_in_digraph.head_node);
#else
		printf("%s:%d vehicle(%d) forwards its packet copies with total_forward_count %d\n",
				__FUNCTION__, __LINE__,
				vehicle->id, total_forward_count);
#endif
*/
	}
#endif /* ] */

	return packet_copy_cluster_number;
}

int EPIDEMIC_Forward_Packet_Copy_From_AP_To_Next_Carrier(parameter_t *param, double current_time, struct_access_point_t *AP, struct_vehicle_t *next_carrier, packet_delivery_statistics_t *packet_delivery_stat, int *discard_count)
{ //AP forwards its packet copies to the next carrier pointed by next_carrier for Epidemic Routing
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
	boolean flag = FALSE; //flag for checking the duplication of a packet

	/* check the validity of data_forwarding_mode and vanet_forwarding_scheme */
	if(param->data_forwarding_mode != DATA_FORWARDING_MODE_V2V)
	{
		printf("%s:%d param->data_forwarding_mode(%d) is not DATA_FORWARDING_MODE_V2V\n",
				__FUNCTION__, __LINE__,
				param->data_forwarding_mode);
		exit(-1);
	}
	else if(param->vanet_forwarding_scheme != VANET_FORWARDING_EPIDEMIC)
	{
		printf("%s:%d param->vanet_forwarding_schme(%d) is not VANET_FORWARDING_EPIDEMIC\n",
				__FUNCTION__, __LINE__,
				param->vanet_forwarding_scheme);
		exit(-1);
	}

	*discard_count = 0; //count for discarded packets

	/** forward AP's packets to next_carrier */
	if(AP->packet_queue->size == 0)
		return 0;

	for(i = 0; i < AP->packet_queue->size; i++) //for-1
	{
		pPacketNode = (packet_queue_node_t*) GetQueueNode((queue_t*)AP->packet_queue, i); //get the pointer to the queue node corresponding to index

		/* compute the packet's lifetime to check whether the packet expires */
		lifetime = current_time - pPacketNode->generation_time; //compute the packet's lifetime

		/** check whether packet's lifetime is greater than the predefined packet TTL */
		if(lifetime > pPacketNode->ttl) //if-1.1
		{
			pPacketNode = (packet_queue_node_t*) Dequeue_With_QueueNodePointer((queue_t*)AP->packet_queue, (queue_node_t*)pPacketNode); //dequeue a packet placed at vehicle's packet queue front

			VADD_Discard_Expired_Packet(param, current_time, pPacketNode, VANET_NODE_VEHICLE, (void*)next_carrier, packet_delivery_stat,pPacketNode->ttl); //discard the expired packet by destroying the memory of the packet while the correponding global packet queue node is dequeued and destroyed 
			(*discard_count)++;

			i--; //decrease index i to check the next queue node because the queue node corresponding to index i is already dequeued and deleted.
			continue;
		} //end of if-1.1

#ifdef EPIDEMIC_20140520
		/* check whether this packet copy is already in the packet queue of next_carrier */
		flag = IsPacketCopy_In_PacketQueue(next_carrier->packet_queue, pPacketNode);
		if (flag)
		{
			continue;
		}
#endif

		/** check packet_depulication_number to limit the number of packet copies in Epidemic Routing */
		if(pPacketNode->packet_duplication_count > param->communication_packet_hop_limit)
		{
			continue;
		}
		else
		{
			/* increase packet's packet_duplication_count to limit the packet copy number */
			pPacketNode->packet_duplication_count++;

			/* increase global_packet's packet_copy_count to see how many packet copies have been generated so far */
			pPacketNode->global_packet->packet_copy_count++;

			/* increase the number of packet copies for packet traffic statistics */
			packet_delivery_stat->generated_packet_copy_number++;
		}
#ifndef EPIDEMIC_20140520
		/* check whether this packet copy is already in the packet queue of next_carrier */
		flag = IsPacketCopy_In_PacketQueue(next_carrier->packet_queue, pPacketNode);
		if (flag)
		{
			continue;
		}
#endif
		/* set up packet's fields */
		pPacketNode->last_receive_time = current_time; //last packet receive time
		pPacketNode->carry_src_id = AP->id; //AP's id
		pPacketNode->carry_dst_id = next_carrier->id; //next packet carrier's id



		/* set up pointers to the carrier vehicle in both the packet queue node and the global packet queue node corresponding to pPacketNode */
		pPacketNode->carrier_vnode = next_carrier; //pointer to the next packet carrier vehicle
		pPacketNode->global_packet->carrier_vnode = next_carrier; //pointer to the next packet carrier vehicle

		/* TBD: add next_carrier to global_packet's carrier_vehicle_list to maintain what vehicles are packet carriers for this packet copy */

		/* update packet_transmission_count */
		distance = euclidean_distance2(&(AP->coordinate), &(next_carrier->current_pos));
		if(R <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
		{
			printf("%s:%d: Error: communication_range(%.2f) is zero!\n", 
					__FUNCTION__, __LINE__,
					(float)R);
			exit(1);
		}

		hop_number = (int)ceil(distance/R);
		pPacketNode->packet_transmission_count += hop_number;

		//Enqueue_With_QueueNodePointer((queue_t*)next_carrier->packet_queue, (queue_node_t*)pPacketNode); //enqueue the packet into next_carrier's packet queue rear
		pPacketNode = (packet_queue_node_t*) Enqueue((queue_t*)next_carrier->packet_queue, (queue_node_t*)pPacketNode); //enqueue the packet into next_carrier's packet queue rear; NOTE: pPacketNode returned by Enqueue() is the pointer to the newly allocated queue node for next_carrier which is different from pPacketNode tossed to Enqueue() as a parameter.
		
		forward_count++; //increase packet count

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

#ifdef  __LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__
		/** enqueue the packet carrier trace entry into packet's carrier trace queue */
		Enqueue_CarrierTraceEntry(param, current_time, pPacketNode, VANET_NODE_VEHICLE, (void*)next_carrier);
#endif
   
	} //end of for-1

	/* change the types of vehicle and nect_carrier */
	if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
	{
		if(forward_count > 0)
		{
			if(next_carrier->type == VEHICLE_CARRIER)
			{
				next_carrier->type = VEHICLE_CURRENT_PACKET_CARRIER;
				//next_carrier becomes a new current packet carrier
			}
		}
	}

	return forward_count;
}

int EPIDEMIC_Forward_Packet_Copy_To_Next_Carrier(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_vehicle_t *next_carrier, packet_delivery_statistics_t *packet_delivery_stat, int *discard_count)
{ //vehicle forwards its packet copies to the next carrier pointed by next_carrier for Epidemic Routing
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
	boolean flag = FALSE; //flag for checking the duplication of a packet

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
	vehicle->packet_queue->size;
	if(vehicle->packet_queue->size == 0)
		return 0;

	for(i = 0; i < vehicle->packet_queue->size; i++) //for-1
	{
		//pPacketNode = (packet_queue_node_t*) Dequeue((queue_t*)vehicle->packet_queue); //dequeue a packet placed at vehicle's packet queue front

		pPacketNode = (packet_queue_node_t*) GetQueueNode((queue_t*)vehicle->packet_queue, i); //get the pointer to the queue node corresponding to index

		/* compute the packet's lifetime to check whether the packet expires */
		lifetime = current_time - pPacketNode->generation_time; //compute the packet's lifetime

		/** check whether packet's lifetime is greater than the predefined packet TTL */
		if(lifetime > pPacketNode->ttl) //if-1.1
		{
			pPacketNode = (packet_queue_node_t*) Dequeue_With_QueueNodePointer((queue_t*)vehicle->packet_queue, (queue_node_t*)pPacketNode); //dequeue a packet placed at vehicle's packet queue front

			VADD_Discard_Expired_Packet(param, current_time, pPacketNode, VANET_NODE_VEHICLE, (void*)next_carrier, packet_delivery_stat,pPacketNode->ttl); //discard the expired packet by destroying the memory of the packet while the correponding global packet queue node is dequeued and destroyed 
			(*discard_count)++;

			i--; //decrease index i to check the next queue node because the queue node corresponding to index i is already dequeued and deleted.
			continue;
		} //end of if-1.1

#ifdef EPIDEMIC_20140520
		/* check whether this packet copy is already in the packet queue of next_carrier */
                flag = IsPacketCopy_In_PacketQueue(next_carrier->packet_queue, pPacketNode);
                if (flag)
		{
			continue;
		}
#endif

		/** check packet_depulication_number to limit the number of packet copies in Epidemic Routing */
		if(pPacketNode->packet_duplication_count > param->communication_packet_hop_limit)
		{
			continue;
		}
		else
		{
			/* increase packet's packet_duplication_count to limit the packet copy number */
			pPacketNode->packet_duplication_count++;

			/* increase global_packet's packet_copy_count to see how many packet copies have been generated so far */
			pPacketNode->global_packet->packet_copy_count++;

			/* increase the number of packet copies for packet traffic statistics */
			packet_delivery_stat->generated_packet_copy_number++;
		}
#ifndef EPIDEMIC_20140520
		/* check whether this packet copy is already in the packet queue of next_carrier */
                flag = IsPacketCopy_In_PacketQueue(next_carrier->packet_queue, pPacketNode);
                if (flag)
		{
			continue;
		}
#endif
		/* set up packet's fields */
		pPacketNode->last_receive_time = current_time; //last packet receive time
		pPacketNode->carry_src_id = vehicle->id; //current packet carrier's id
		pPacketNode->carry_dst_id = next_carrier->id; //next packet carrier's id




		/* set up pointers to the carrier vehicle in both the packet queue node and the global packet queue node corresponding to pPacketNode */
		pPacketNode->carrier_vnode = next_carrier; //pointer to the next packet carrier vehicle
		pPacketNode->global_packet->carrier_vnode = next_carrier; //pointer to the next packet carrier vehicle

		/* TBD: add next_carrier to global_packet's carrier_vehicle_list to maintain what vehicles are packet carriers for this packet copy */

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


		//Enqueue_With_QueueNodePointer((queue_t*)next_carrier->packet_queue, (queue_node_t*)pPacketNode); //enqueue the packet into next_carrier's packet queue rear
		pPacketNode = (packet_queue_node_t*) Enqueue((queue_t*)next_carrier->packet_queue, (queue_node_t*)pPacketNode); //enqueue the packet into next_carrier's packet queue 

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
			if(param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC)
			{
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

				if(next_carrier->type == VEHICLE_CARRIER)
				{
					next_carrier->type = VEHICLE_CURRENT_PACKET_CARRIER;
					//next_carrier becomes a new current packet carrier
				}
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

int EPIDEMIC_Perform_Packet_Dissemination_On_Road_Segment(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, packet_delivery_statistics_t *packet_delivery_stat)
{ //perform packet dissemination on either one-way road segment or two-way road segment by Epidemic Routing and return the number of packet copies forwarded to a neighboring vehicle in a different cluster 
	struct_vehicle_t *next_carrier_1 = NULL; //pointer to the first next_carrier for one outgoing directed edge
	struct_vehicle_t *next_carrier_2 = NULL; //pointer to the second next_carrier for one incoming directed edge
	boolean flag = FALSE; //flag to see whether there exists a next carrier for an outgoing directed edge
	int discard_count = 0; //the count for the number of packets discarded by packet lifetime expiration
	int forward_count = 0; //the count for the number of packet copies forwarded to next_vehicle or a convoy vehicle in the convoy pointed by next_vehicle as leader_vehicle
	int total_forward_count = 0; //the total forward count for next_vehicle or all the convoy members in the convoy pointed by next_vehicle as leader_vehicle
	vehicle_queue_t *Q = NULL; //the pointer to a vehicle queue in one convoy queue node
	vehicle_queue_node_t *pQueueNode = NULL; //the pointer to a vehicle queue node in one convoy queue node
	int i = 0; //index for for-loop

	flag = EPIDEMIC_Is_There_Next_Carrier_On_Road_Segment(param, current_time, vehicle, G, G_size, &next_carrier_1, &next_carrier_2);
	if(flag)
	{
		switch(param->vehicle_vanet_forwarding_type)
		{
			case VANET_FORWARDING_BASED_ON_CONVOY:
				/* let vehicle forward its packet copies to each vehicle in the convoy whose leader_vehicle is next_carrier_1 and next_carrier_2 */
				if(next_carrier_1 != NULL)
				{
					Q = &(next_carrier_1->ptr_convoy_queue_node->vehicle_list);
					/* check the validity of convoy queue Q */
					if(Q == NULL)
					{
						printf("%s:%d the convoy queue of next_vehicle_1(%d) is NULL",
								__FUNCTION__, __LINE__,
								next_carrier_1->id);
						exit(1);
					}

					pQueueNode = &(Q->head);
					for(i = 0; i < Q->size; i++)
					{
						pQueueNode = pQueueNode->next;
						forward_count = EPIDEMIC_Forward_Packet_Copy_To_Next_Carrier(param, current_time, vehicle, pQueueNode->vnode, packet_delivery_stat, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier's convoy head
						total_forward_count += forward_count;
					}
				}

				if(next_carrier_2 != NULL)
				{
					Q = &(next_carrier_2->ptr_convoy_queue_node->vehicle_list);
					/* check the validity of convoy queue Q */
					if(Q == NULL)
					{
						printf("%s:%d the convoy queue of next_vehicle_2(%d) is NULL",
								__FUNCTION__, __LINE__,
								next_carrier_2->id);
						exit(1);
					}

					pQueueNode = &(Q->head);
					for(i = 0; i < Q->size; i++)
					{
						pQueueNode = pQueueNode->next;
						forward_count = EPIDEMIC_Forward_Packet_Copy_To_Next_Carrier(param, current_time, vehicle, pQueueNode->vnode, packet_delivery_stat, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier's convoy head
						total_forward_count += forward_count;
					}
				}
				break;

			case VANET_FORWARDING_BASED_ON_VEHICLE:
				if(next_carrier_1 != NULL)
				{
					forward_count = EPIDEMIC_Forward_Packet_Copy_To_Next_Carrier(param, current_time, vehicle, next_carrier_1, packet_delivery_stat, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier's convoy head
					total_forward_count += forward_count;
				}

				if(next_carrier_2 != NULL)
				{
					forward_count = EPIDEMIC_Forward_Packet_Copy_To_Next_Carrier(param, current_time, vehicle, next_carrier_2, packet_delivery_stat, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier's convoy head
					total_forward_count += forward_count;
				}
				break;

			default:
				printf("%s:%d param->vehicle_vanet_forwarding_type(%d) is not supported!\n",
						__FUNCTION__, __LINE__,
						param->vehicle_vanet_forwarding_type);
				exit(1);
		}
	}

#if 1 /* [ */
	if(total_forward_count > 0)
	{
/*
#ifdef EPIDEMIC_20140602
		printf("time %.2f, vehicle(%d) on road forwards to vehicle(%d) count %d pos(%s,%s)\n",			         current_time,
			vehicle->id, 
			next_carrier_1->id,
			total_forward_count,
			vehicle->current_pos_in_digraph.enode->tail_node, 
			vehicle->current_pos_in_digraph.enode->head_node);
#else
		printf("%s:%d vehicle(%d) forwards its packet copies with total_forward_count %d\n",
				__FUNCTION__, __LINE__,
				vehicle->id, total_forward_count);
#endif
*/
	}
#endif /* ] */

	return total_forward_count;
}

boolean EPIDEMIC_Is_There_Next_Carrier_On_Road_Segment(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier_1, struct_vehicle **next_carrier_2)
{ //determine whether to forward its packets to next carriers moving on the current road segment in Epidemic Routing and return the pointers to the next carriers through *next_carrier; Note that the convoy leaders become the next carriers
	boolean result = FALSE;

	*next_carrier_1 = NULL;
	*next_carrier_2 = NULL;

	/** select a data forwarding method according to two-way forwarding flag */
	if(param->data_forwarding_two_way_forwarding_flag)
	{ /* Data forwarding for two-way road segment */
		result = EPIDEMIC_Is_There_Next_Carrier_On_Two_Way_Road_Segment(param, current_time, vehicle, G, G_size, next_carrier_1, next_carrier_2);
	}
	else
	{ /* Data forwarding for one-way road segment */
		result = EPIDEMIC_Is_There_Next_Carrier_On_One_Way_Road_Segment(param, current_time, vehicle, G, G_size, next_carrier_1);
	}

	return result;
}

boolean EPIDEMIC_Is_There_Next_Carrier_On_One_Way_Road_Segment(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier)
{ //For one-way road segment, determine whether to forward its packets to next carrier moving on the current road segment in Epidemic Routing and return the pointer to the next carrier through *next_carrier; Note that the convoy leader becomes the next carrier 
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

boolean EPIDEMIC_Is_There_Next_Carrier_On_Two_Way_Road_Segment(parameter_t *param, double current_time, struct_vehicle_t *vehicle, struct_graph_node *G, int G_size, struct_vehicle_t **next_carrier_1, struct_vehicle_t **next_carrier_2)
{ //For two-way road segment, determine whether to forward its packets to next carriers moving on the current road segment in Epidemic Routing and return the pointers to the next carriers through *next_carrier_1 and *next_carrier_2; Note that the convoy leaders become the next carriers 
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

	/** reset *next_carrier_1 and *next_carrier_2 to NULL */
	*next_carrier_1 = NULL;
	*next_carrier_2 = NULL;

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
				*next_carrier_1 = next_carrier_candidate_1;
				*next_carrier_2 = next_carrier_candidate_2;
			}
			else
			{
				*next_carrier_1 = next_carrier_candidate_2;
				*next_carrier_2 = next_carrier_candidate_1;
			}
		}
		else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
		{
			if(next_carrier_candidate_1->EDD_for_download <= next_carrier_candidate_2->EDD_for_download)
			{
				*next_carrier_1 = next_carrier_candidate_1;
				*next_carrier_2 = next_carrier_candidate_2;
			}
			else
			{
				*next_carrier_1 = next_carrier_candidate_2;
				*next_carrier_2 = next_carrier_candidate_1;
			}
		}
		else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
		{
			if(next_carrier_candidate_1->EDD_for_V2V <= next_carrier_candidate_2->EDD_for_V2V)
			{
				*next_carrier_1 = next_carrier_candidate_1;
				*next_carrier_2 = next_carrier_candidate_2;
			}
			else
			{
				*next_carrier_1 = next_carrier_candidate_2;
				*next_carrier_2 = next_carrier_candidate_1;
			}
		}

	}
	else if(next_carrier_candidate_1 != NULL)
	{
		*next_carrier_1 = next_carrier_candidate_1;
		*next_carrier_2 = NULL;
	}
	else if(next_carrier_candidate_2 != NULL)
	{
		*next_carrier_1 = next_carrier_candidate_2;
		*next_carrier_2 = NULL;
	}

#if 0 /* [ */	
	/** set the next_carrier to the vehicle's convoy leader when there exists no next carrier candidate in a different convoy with less EDD */
	if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY)
	{
		if((next_carrier_candidate_1 == NULL) && (next_carrier_candidate_2 == NULL) && (vehicle->id != vehicle->ptr_convoy_queue_node->leader_vehicle->id))
		{
			*next_carrier = vehicle->ptr_convoy_queue_node->leader_vehicle;      
		}
	}
#endif /* ] */

	/** check whether either next_carrier_1 or next_carrier_2 is equal to vehicle */
	if(*next_carrier_1 != NULL || *next_carrier_2 != NULL)
	{
		if(vehicle == *next_carrier_1)
		{ //if next_carrier_1 is equal to vehicle, we don't let vehicle send its packets to itself 
			printf("%s:%d: Error: vehicle(id=%d) is the same as next_carrier_1\n", 
					__FUNCTION__, __LINE__,
					vehicle->id);
			exit(1);
		}
		else if(vehicle == *next_carrier_2)
		{ //if next_carrier_2 is equal to vehicle, we don't let vehicle send its packets to itself 
			printf("%s:%d: Error: vehicle(id=%d) is the same as next_carrier_2\n", 
					__FUNCTION__, __LINE__,
					vehicle->id);
			exit(1);
		}
		else
			result = TRUE;
	}

	return result;
}

int EPIDEMIC_Discard_Original_Packet_In_AP(double current_time, 
		struct_access_point_t *AP)
{ //discard original packets in AP's packet queue by releasing the memory of the packets in the packet queue
	int count = 0; //the number of discarded packets
	packet_queue_node_t *pPacketNode = NULL; //pointer to a packet node

	if(AP == NULL)
	{
		printf("%s:%d AP is NULL\n",
				__FUNCTION__, __LINE__);
		return 0;
	}
	else if(AP->packet_queue == NULL)
	{
		printf("%s:%d AP->packet_queue is NULL\n",
				__FUNCTION__, __LINE__);
		return 0;
	}
	else
	{
		count = AP->packet_queue->size;
	}

    /* empty the queue AP->packet_queue by releasing the memory of the packets */
    EmptyQueue((queue_t*) AP->packet_queue);

	return count;
}


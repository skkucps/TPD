/**
 *  File: queue.c
 *	Description: operations for queue that implements the FIFO
 *	Date: 08/21/2007	
 *	Maker: Jaehoon Jeong
 */

#include "stdafx.h"
#include <stdlib.h> //calloc()
#include "queue.h"
#include "util.h"
#include "quick-sort.h"
#include "shortest-path.h" //LookupGraph(), GetNeighborGraphNode()
#include "vadd.h" //VADD_Get_EDD_And_EDD_SD_In_Shortest_Path_Model()
#include "all-pairs-shortest-paths.h" //Floyd_Warshall_Allocate_Matrices_For_EDD()
#include "tpd.h" //TPD_Allocate_Predicted_Encounter_Graph_For_Packet()

/** Note3: whenever we support another queue type, we need to add another case statement for it */
void InitQueue(queue_t *Q, queue_type_t queue_type)
{ //initialize queue Q
	assert_memory(Q);

	switch(queue_type)
	{
	case QTYPE_SCHEDULE: 
		memset(Q, 0, sizeof(schedule_queue_t));
		break;

	case QTYPE_SENSOR: 
		memset(Q, 0, sizeof(sensor_queue_t));
		break;

	case QTYPE_PATH: 
		memset(Q, 0, sizeof(path_queue_t));
		break;

	case QTYPE_EDGE: 
		memset(Q, 0, sizeof(edge_queue_t));
		break;

	case QTYPE_SUBEDGE: 
		memset(Q, 0, sizeof(subedge_queue_t));
		break;

	case QTYPE_LOCATION: 
		memset(Q, 0, sizeof(location_queue_t));
		break;

	case QTYPE_HOLE_SEGMENT: 
		memset(Q, 0, sizeof(hole_segment_queue_t));
		break;

	case QTYPE_HOLE_ENDPOINT: 
		memset(Q, 0, sizeof(hole_endpoint_queue_t));
		break;

	case QTYPE_VERTEX_SET:
		memset(Q, 0, sizeof(vertex_set_queue_t));
		break;

	case QTYPE_EDGE_SET:
		memset(Q, 0, sizeof(edge_set_queue_t));
		break;

	case QTYPE_ANGLE: 
		memset(Q, 0, sizeof(angle_queue_t));
		break;

	case QTYPE_DIRECTIONAL_EDGE: 
		memset(Q, 0, sizeof(directional_edge_queue_t));
		break;

	case QTYPE_DELAY_COMPONENT: 
		memset(Q, 0, sizeof(delay_component_queue_t));
		break;

	case QTYPE_DELAY: 
		memset(Q, 0, sizeof(delay_queue_t));
		break;

	case QTYPE_PACKET: 
		memset(Q, 0, sizeof(packet_queue_t));
		break;

	case QTYPE_VEHICLE: 
		memset(Q, 0, sizeof(vehicle_queue_t));
		break;

	case QTYPE_VEHICLE_MOVEMENT: 
		memset(Q, 0, sizeof(vehicle_movement_queue_t));
		break;

	case QTYPE_INTERSECTION_EDD:
		memset(Q, 0, sizeof(intersection_edd_queue_t));
		break;

	case QTYPE_CONVOY: 
		memset(Q, 0, sizeof(convoy_queue_t));
		break;

	case QTYPE_MOBILITY: 
		memset(Q, 0, sizeof(mobility_queue_t));
		break;

	case QTYPE_DESTINATION_VEHICLE: 
		memset(Q, 0, sizeof(destination_vehicle_queue_t));
		break;

	case QTYPE_ACCESS_POINT: 
		memset(Q, 0, sizeof(access_point_queue_t));
		break;

	case QTYPE_VEHICLE_TRAJECTORY: 
		memset(Q, 0, sizeof(vehicle_trajectory_queue_t));
		break;

	case QTYPE_CARRIER_TRACE: 
		memset(Q, 0, sizeof(carrier_trace_queue_t));		
		break;

	case QTYPE_FORWARDING_TABLE: 
		memset(Q, 0, sizeof(forwarding_table_queue_t));
		break;

	case QTYPE_GLOBAL_PACKET:
		memset(Q, 0, sizeof(global_packet_queue_t));
		break;

	case QTYPE_STATIONARY_NODE:
		memset(Q, 0, sizeof(stationary_node_queue_t));
		break;

	case QTYPE_PACKET_TRAJECTORY:
		memset(Q, 0, sizeof(packet_trajectory_queue_t));
		break;

	case QTYPE_PROBABILITY_AND_STATISTICS:
		memset(Q, 0, sizeof(probability_and_statistics_queue_t));
		break;

	case QTYPE_CONDITIONAL_FORWARDING_PROBABILITY:
		memset(Q, 0, sizeof(conditional_forwarding_probability_queue_t));
		break;

	case QTYPE_TARGET_POINT:
		memset(Q, 0, sizeof(target_point_queue_t));
		((target_point_queue_t*)Q)->minimum_average_delivery_delay = INF;
		((target_point_queue_t*)Q)->delivery_success_probability = 0;
		break;

	case QTYPE_MINIMUM_PRIORITY:
		memset(Q, 0, sizeof(minimum_priority_queue_t));
		break;

	case QTYPE_NEIGHBOR_LIST:
		memset(Q, 0, sizeof(neighbor_list_queue_t));
		break;

	case QTYPE_PARENT_LIST:
		memset(Q, 0, sizeof(parent_list_queue_t));
		break;

	case QTYPE_ADJACENCY_LIST:
		memset(Q, 0, sizeof(adjacency_list_queue_t));
		break;

	case QTYPE_ADJACENCY_LIST_POINTER:
		memset(Q, 0, sizeof(adjacency_list_pointer_queue_t));
		break;

	case QTYPE_PACKET_POINTER: 
		memset(Q, 0, sizeof(packet_pointer_queue_t));
		break;

	default:
		printf("Error: InitQueue(): unknown queue type(%d)\n", queue_type);
		exit(1);
	}

	Q->type = queue_type;
	//Q->size = 0;
	Q->head.next = Q->head.prev = &(Q->head);
}

/** Note3: whenever we support another queue type, we need to add another case statement for it */
queue_node_t* MakeQueueNode(queue_type_t queue_type, queue_node_t *node)
{ //make a queue node corresponding to queue type and copy the data of node into its data portion
	queue_node_t *p = NULL;

	switch(queue_type)
	{
	case QTYPE_SCHEDULE: 
		p = (queue_node_t*) calloc(1, sizeof(schedule_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(schedule_queue_node_t));
		break;

	case QTYPE_SENSOR: 
		p = (queue_node_t*) calloc(1, sizeof(sensor_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(sensor_queue_node_t));
		break;

	case QTYPE_PATH: 
		p = (queue_node_t*) calloc(1, sizeof(path_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(path_queue_node_t));
		break;

	case QTYPE_EDGE: 
		p = (queue_node_t*) calloc(1, sizeof(edge_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(edge_queue_node_t));
		break;

	case QTYPE_SUBEDGE: 
		p = (queue_node_t*) calloc(1, sizeof(subedge_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(subedge_queue_node_t));
		break;

	case QTYPE_LOCATION: 
		p = (queue_node_t*) calloc(1, sizeof(location_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(location_queue_node_t));
		break;

	case QTYPE_HOLE_SEGMENT: 
		p = (queue_node_t*) calloc(1, sizeof(hole_segment_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(hole_segment_queue_node_t));
		break;

	case QTYPE_HOLE_ENDPOINT: 
		p = (queue_node_t*) calloc(1, sizeof(hole_endpoint_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(hole_endpoint_queue_node_t));
		break;

	case QTYPE_VERTEX_SET:
		p = (queue_node_t*) calloc(1, sizeof(vertex_set_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(vertex_set_queue_node_t));
		break;

	case QTYPE_EDGE_SET:
		p = (queue_node_t*) calloc(1, sizeof(edge_set_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(edge_set_queue_node_t));
		break;

	case QTYPE_ANGLE:
		p = (queue_node_t*) calloc(1, sizeof(angle_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(angle_queue_node_t));
		break;

	case QTYPE_DIRECTIONAL_EDGE:
		p = (queue_node_t*) calloc(1, sizeof(directional_edge_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(directional_edge_queue_node_t));
		break;

	case QTYPE_DELAY_COMPONENT:
		p = (queue_node_t*) calloc(1, sizeof(delay_component_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(delay_component_queue_node_t));
		break;

	case QTYPE_DELAY:
		p = (queue_node_t*) calloc(1, sizeof(delay_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(delay_queue_node_t));
		break;

	case QTYPE_PACKET:
		p = (queue_node_t*) calloc(1, sizeof(packet_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(packet_queue_node_t));
		break;

	case QTYPE_VEHICLE:
		p = (queue_node_t*) calloc(1, sizeof(vehicle_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(vehicle_queue_node_t));
		break;

	case QTYPE_VEHICLE_MOVEMENT:
		p = (queue_node_t*) calloc(1, sizeof(vehicle_movement_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(vehicle_movement_queue_node_t));
		break;

	case QTYPE_INTERSECTION_EDD:
		p = (queue_node_t*) calloc(1, sizeof(intersection_edd_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(intersection_edd_queue_node_t));	  
		break;

	case QTYPE_CONVOY:
		p = (queue_node_t*) calloc(1, sizeof(convoy_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(convoy_queue_node_t));
		break;

	case QTYPE_MOBILITY:
		p = (queue_node_t*) calloc(1, sizeof(mobility_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(mobility_queue_node_t));
		break;

	case QTYPE_DESTINATION_VEHICLE:
		p = (queue_node_t*) calloc(1, sizeof(destination_vehicle_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(destination_vehicle_queue_node_t));
		break;

	case QTYPE_ACCESS_POINT:
		p = (queue_node_t*) calloc(1, sizeof(access_point_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(access_point_queue_node_t));
		break;

	case QTYPE_VEHICLE_TRAJECTORY:
		p = (queue_node_t*) calloc(1, sizeof(vehicle_trajectory_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(vehicle_trajectory_queue_node_t));
		break;

	case QTYPE_CARRIER_TRACE:
		p = (queue_node_t*) calloc(1, sizeof(carrier_trace_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(carrier_trace_queue_node_t));
		break;

	case QTYPE_FORWARDING_TABLE:
		p = (queue_node_t*) calloc(1, sizeof(forwarding_table_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(forwarding_table_queue_node_t));
		break;

	case QTYPE_GLOBAL_PACKET:
		p = (queue_node_t*) calloc(1, sizeof(global_packet_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(global_packet_queue_node_t));
	    break;

	case QTYPE_STATIONARY_NODE:
		p = (queue_node_t*) calloc(1, sizeof(stationary_node_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(stationary_node_queue_node_t));
	    break;

	case QTYPE_PACKET_TRAJECTORY:
		p = (queue_node_t*) calloc(1, sizeof(packet_trajectory_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(packet_trajectory_queue_node_t));
	    break;

	case QTYPE_PROBABILITY_AND_STATISTICS:
		p = (queue_node_t*) calloc(1, sizeof(probability_and_statistics_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(probability_and_statistics_queue_node_t));
	    break;

	case QTYPE_CONDITIONAL_FORWARDING_PROBABILITY:
		p = (queue_node_t*) calloc(1, sizeof(conditional_forwarding_probability_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(conditional_forwarding_probability_queue_node_t));
	    break;

	case QTYPE_TARGET_POINT:
		p = (queue_node_t*) calloc(1, sizeof(target_point_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(target_point_queue_node_t));
	    break;

	case QTYPE_MINIMUM_PRIORITY:
		p = (queue_node_t*) calloc(1, sizeof(minimum_priority_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(minimum_priority_queue_node_t));
	    break;

	case QTYPE_NEIGHBOR_LIST:
		p = (queue_node_t*) calloc(1, sizeof(neighbor_list_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(neighbor_list_queue_node_t));
	    break;

	case QTYPE_PARENT_LIST:
		p = (queue_node_t*) calloc(1, sizeof(parent_list_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(parent_list_queue_node_t));
	    break;

	case QTYPE_ADJACENCY_LIST:
		p = (queue_node_t*) calloc(1, sizeof(adjacency_list_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(adjacency_list_queue_node_t));
	    break;

	case QTYPE_ADJACENCY_LIST_POINTER:
		p = (queue_node_t*) calloc(1, sizeof(adjacency_list_pointer_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(adjacency_list_pointer_queue_node_t));
	    break;

	case QTYPE_PACKET_POINTER:
		p = (queue_node_t*) calloc(1, sizeof(packet_pointer_queue_node_t));
		assert_memory(p);
		memcpy(p, node, sizeof(packet_pointer_queue_node_t));
		break;

	default:
		printf("%s:%d: unknown queue type(%d)\n",
				__FUNCTION__, __LINE__, queue_type);
		exit(1);
	}

	return p;
}

queue_node_t* Enqueue(queue_t *Q, queue_node_t *node)
{ //enqueue node into queue Q and return the pointer to the new queue node
	queue_node_t *p = NULL;

	assert_memory(Q);
	assert_memory(node);
	
	p = MakeQueueNode(Q->type, node);
	//make a queue node corresponding to queue type and copy the data of node into its data portion

	/* insert node before head to let node be the last node in the queue */
	p->prev = Q->head.prev;
	p->next = &(Q->head);
	Q->head.prev->next = p;
	Q->head.prev = p;

	Q->size++;

	/* perform work specific to queue type */
	switch(Q->type)
	{
	case QTYPE_HOLE_ENDPOINT: //set the sequence number for a new sensing hole node (i.e., sensing hole endpoint)
		((hole_endpoint_queue_node_t*)p)->hid = ++(((hole_endpoint_queue_t*)Q)->sequence_number);
		break;

	case QTYPE_VERTEX_SET://set the representative information
		((vertex_set_queue_node_t*)p)->representative = (vertex_set_queue_node_t*)Q->head.next;
		((vertex_set_queue_node_t*)p)->cluster_type = ((vertex_set_queue_t*)Q)->cluster_type;
		break;

	case QTYPE_EDGE_SET: //set the representative information
		((edge_set_queue_node_t*)p)->representative = (edge_set_queue_node_t*)Q->head.next;
		break;

	case QTYPE_VEHICLE_MOVEMENT: //let the vehicle_movement_queue_node's ptr_queue point to its vehicle_movement_queue
	        ((vehicle_movement_queue_node_t*)p)->ptr_queue = (vehicle_movement_queue_t*)Q;
	        break;

	case QTYPE_CONVOY:
	        ((convoy_queue_node_t*)p)->ptr_queue = (convoy_queue_t*)Q;
                (((convoy_queue_t*)Q)->sequence_number)++;
                InitQueue((queue_t*) &(((convoy_queue_node_t*)p)->vehicle_list), QTYPE_VEHICLE);
                break;

	case QTYPE_EDGE: //set the eid for a new edge to the next available sequence number
		((edge_queue_node_t*)p)->eid = ++(((edge_queue_t*)Q)->sequence_number);
		break;

	case QTYPE_DIRECTIONAL_EDGE: //set the eid for a new directional edge to the next available sequence number
		((directional_edge_queue_node_t*)p)->eid = ++(((directional_edge_queue_t*)Q)->sequence_number);
		break;

	case QTYPE_DESTINATION_VEHICLE:
	        ((destination_vehicle_queue_node_t*)p)->ptr_queue = (destination_vehicle_queue_t*)Q;
                ((destination_vehicle_queue_node_t*)p)->order =  Q->size - 1;

		/* initialize mobility_list queue for vehicle mobility in the road network */
                InitQueue((queue_t*) &(((destination_vehicle_queue_node_t*)p)->mobility_list), QTYPE_MOBILITY);
                break;

	case QTYPE_MOBILITY:
	        ((mobility_queue_node_t*)p)->ptr_queue = (mobility_queue_t*)Q;
                ((mobility_queue_node_t*)p)->order =  Q->size - 1;
                break;

	case QTYPE_PACKET:
		/* initialize vehicle_trajectory queue for the destination vehicle trajectory */
		InitQueue((queue_t*) &(((packet_queue_node_t*)p)->vehicle_trajectory), QTYPE_VEHICLE_TRAJECTORY);

		/* initialize carrier_trace queue for the packet carrier trace */
		InitQueue((queue_t*) &(((packet_queue_node_t*)p)->carrier_trace), QTYPE_CARRIER_TRACE);

		/* initialize packet_trajectory queue for the packet trajectory */
		InitQueue((queue_t*) &(((packet_queue_node_t*)p)->packet_trajectory), QTYPE_PACKET_TRAJECTORY);
               
		/* allocate the memory of a predicted encounter graph for a packet */
		TPD_Allocate_Predicted_Encounter_Graph_For_Packet((packet_queue_node_t*)p);
		break;

	case QTYPE_VEHICLE_TRAJECTORY:
	        ((vehicle_trajectory_queue_node_t*)p)->ptr_queue = (vehicle_trajectory_queue_t*)Q;
                ((vehicle_trajectory_queue_node_t*)p)->order =  Q->size - 1;

		/* let the current_order_qnode point to the first queue node */
		if(Q->size == 1)
		{
		  ((vehicle_trajectory_queue_t*)Q)->current_order_qnode = (vehicle_trajectory_queue_node_t*)p;
		  ((vehicle_trajectory_queue_t*)Q)->current_order = 0;
		}
		break;

	case QTYPE_CARRIER_TRACE:
	        ((carrier_trace_queue_node_t*)p)->ptr_queue = (carrier_trace_queue_t*)Q;
                ((carrier_trace_queue_node_t*)p)->order =  Q->size - 1;
		break;

	case QTYPE_FORWARDING_TABLE:
	        ((forwarding_table_queue_node_t*)p)->ptr_queue = (forwarding_table_queue_t*)Q;

		/* initialize undirectional edge queue EQ for road network graph G */
                InitQueue((queue_t*) &(((forwarding_table_queue_node_t*)p)->EQ), QTYPE_EDGE);

		/* initialize directional edge queue DEQ for road network graph G */
                InitQueue((queue_t*) &(((forwarding_table_queue_node_t*)p)->DEQ), QTYPE_DIRECTIONAL_EDGE);
		break;

	case QTYPE_GLOBAL_PACKET:
	        ((global_packet_queue_node_t*)p)->ptr_queue = (global_packet_queue_t*)Q;
                ((global_packet_queue_node_t*)p)->order =  Q->size - 1;
		break;

	case QTYPE_STATIONARY_NODE:
	        ((stationary_node_queue_node_t*)p)->ptr_queue = (stationary_node_queue_t*)Q;
                ((stationary_node_queue_node_t*)p)->order =  Q->size - 1;

		/* initialize the stationary node's packet_queue for packets */
                InitQueue((queue_t*) &(((stationary_node_queue_node_t*)p)->packet_queue), QTYPE_PACKET);
		break;

	case QTYPE_PACKET_TRAJECTORY:
	        ((packet_trajectory_queue_node_t*)p)->ptr_queue = (packet_trajectory_queue_t*)Q;
                ((packet_trajectory_queue_node_t*)p)->order =  Q->size - 1;
		break;

	case QTYPE_PROBABILITY_AND_STATISTICS:
		((probability_and_statistics_queue_node_t*)p)->ptr_queue = (probability_and_statistics_queue_t*)Q;
		((probability_and_statistics_queue_node_t*)p)->order =  Q->size - 1;

		/* initialize the probability-and-statistics queue node's conditional forwarding probability queue */
		InitQueue((queue_t*) &(((probability_and_statistics_queue_node_t*)p)->conditional_forwarding_probability_queue), QTYPE_CONDITIONAL_FORWARDING_PROBABILITY);
		break;

	case QTYPE_CONDITIONAL_FORWARDING_PROBABILITY:
	        ((conditional_forwarding_probability_queue_node_t*)p)->ptr_queue = (conditional_forwarding_probability_queue_t*)Q;
                ((conditional_forwarding_probability_queue_node_t*)p)->order =  Q->size - 1;
		break;

	case QTYPE_TARGET_POINT: //let the target_point_queue_node's ptr_queue point to its target_point_queue
		((target_point_queue_node_t*)p)->ptr_queue = (target_point_queue_t*)Q;
		((target_point_queue_node_t*)p)->order = Q->size - 1;
		break;


	case QTYPE_MINIMUM_PRIORITY:
		((minimum_priority_queue_node_t*)p)->ptr_queue = (minimum_priority_queue_t*)Q;
		((minimum_priority_queue_node_t*)p)->order = Q->size - 1;

		InitQueue((queue_t*) &(((minimum_priority_queue_node_t*)p)->parent_list), 
				QTYPE_PARENT_LIST);
		break;

	case QTYPE_NEIGHBOR_LIST:
		((neighbor_list_queue_node_t*)p)->ptr_queue = (neighbor_list_queue_t*)Q;
		((neighbor_list_queue_node_t*)p)->order = Q->size - 1;
		break;

	case QTYPE_PARENT_LIST:
		((parent_list_queue_node_t*)p)->ptr_queue = (parent_list_queue_t*)Q;
		((parent_list_queue_node_t*)p)->order = Q->size - 1;
		break;

	case QTYPE_ADJACENCY_LIST:
		((adjacency_list_queue_node_t*)p)->ptr_queue = (adjacency_list_queue_t*)Q;
		((adjacency_list_queue_node_t*)p)->order = Q->size - 1;

		InitQueue((queue_t*) &(((adjacency_list_queue_node_t*)p)->neighbor_list), 
				QTYPE_NEIGHBOR_LIST);
		InitQueue((queue_t*) &(((adjacency_list_queue_node_t*)p)->parent_list), 
				QTYPE_PARENT_LIST);
		break;

	case QTYPE_ADJACENCY_LIST_POINTER:
		((adjacency_list_pointer_queue_node_t*)p)->ptr_queue = (adjacency_list_pointer_queue_t*)Q;
		((adjacency_list_pointer_queue_node_t*)p)->order = Q->size - 1;
		break;

	case QTYPE_PACKET_POINTER:
		((packet_pointer_queue_node_t*)p)->ptr_queue = (packet_pointer_queue_t*)Q;
		((packet_pointer_queue_node_t*)p)->order =  Q->size - 1;
		break;

	default:
		break;
	}
	
	return p;
}

queue_node_t* Enqueue_With_QueueNodePointer(queue_t *Q, queue_node_t *p)
{ //enqueue queue node pointer p into queue Q without allocating a new queue node
	assert_memory(Q);
	assert_memory(p);
	
	/* insert node before head to let node be the last node in the queue */
	p->prev = Q->head.prev;
	p->next = &(Q->head);
	Q->head.prev->next = p;
	Q->head.prev = p;

	Q->size++;

	/* perform work specific to queue type */
	switch(Q->type)
	{
	case QTYPE_HOLE_ENDPOINT: //set the sequence number for a new sensing hole node (i.e., sensing hole endpoint)
		((hole_endpoint_queue_node_t*)p)->hid = ++(((hole_endpoint_queue_t*)Q)->sequence_number);
		break;

	case QTYPE_VERTEX_SET://set the representative information
		((vertex_set_queue_node_t*)p)->representative = (vertex_set_queue_node_t*)Q->head.next;
		((vertex_set_queue_node_t*)p)->cluster_type = ((vertex_set_queue_t*)Q)->cluster_type;
		break;

	case QTYPE_EDGE_SET: //set the representative information
		((edge_set_queue_node_t*)p)->representative = (edge_set_queue_node_t*)Q->head.next;
		break;

	case QTYPE_VEHICLE_MOVEMENT: //let the vehicle_movement_queue_node's ptr_queue point to its vehicle_movement_queue
	        ((vehicle_movement_queue_node_t*)p)->ptr_queue = (vehicle_movement_queue_t*)Q;
	        break;

	case QTYPE_CONVOY:
	        ((convoy_queue_node_t*)p)->ptr_queue = (convoy_queue_t*)Q;
                (((convoy_queue_t*)Q)->sequence_number)++;
                //InitQueue((queue_t*) &(((convoy_queue_node_t*)p)->vehicle_list), QTYPE_VEHICLE);
                break;

	case QTYPE_EDGE: //set the eid for a new edge to the next available sequence number
		((edge_queue_node_t*)p)->eid = ++(((edge_queue_t*)Q)->sequence_number);
		break;

	case QTYPE_DIRECTIONAL_EDGE: //set the eid for a new directional edge to the next available sequence number
		((directional_edge_queue_node_t*)p)->eid = ++(((directional_edge_queue_t*)Q)->sequence_number);
		break;

	case QTYPE_DESTINATION_VEHICLE:
	        ((destination_vehicle_queue_node_t*)p)->ptr_queue = (destination_vehicle_queue_t*)Q;
                ((destination_vehicle_queue_node_t*)p)->order =  Q->size - 1;

		/* initialize mobility_list queue for vehicle mobility in the road network */
                //InitQueue((queue_t*) &(((destination_vehicle_queue_node_t*)p)->mobility_list), QTYPE_MOBILITY);
                break;

	case QTYPE_MOBILITY:
	        ((mobility_queue_node_t*)p)->ptr_queue = (mobility_queue_t*)Q;
                ((mobility_queue_node_t*)p)->order =  Q->size - 1;
                break;

	case QTYPE_PACKET:
		/* initialize vehicle_trajectory queue for the destination vehicle trajectory */
                //InitQueue((queue_t*) &(((packet_queue_node_t*)p)->vehicle_trajectory), QTYPE_VEHICLE_TRAJECTORY);

		/* initialize carrier_trace queue for the packet carrier trace */
                //InitQueue((queue_t*) &(((packet_queue_node_t*)p)->carrier_trace), QTYPE_CARRIER_TRACE);
		break;

	case QTYPE_VEHICLE_TRAJECTORY:
	        ((vehicle_trajectory_queue_node_t*)p)->ptr_queue = (vehicle_trajectory_queue_t*)Q;
                ((vehicle_trajectory_queue_node_t*)p)->order =  Q->size - 1;

		/* let the current_order_qnode point to the first queue node */
		if(Q->size == 1)
		{
		  ((vehicle_trajectory_queue_t*)Q)->current_order_qnode = (vehicle_trajectory_queue_node_t*)p;
		  ((vehicle_trajectory_queue_t*)Q)->current_order = 0;
		}
		break;

	case QTYPE_CARRIER_TRACE:
	        ((carrier_trace_queue_node_t*)p)->ptr_queue = (carrier_trace_queue_t*)Q;
                ((carrier_trace_queue_node_t*)p)->order =  Q->size - 1;
		break;

	case QTYPE_FORWARDING_TABLE:
	        ((forwarding_table_queue_node_t*)p)->ptr_queue = (forwarding_table_queue_t*)Q;

		/* initialize undirectional edge queue EQ for road network graph G */
                //InitQueue((queue_t*) &(((forwarding_table_queue_node_t*)p)->EQ), QTYPE_EDGE);

		/* initialize directional edge queue DEQ for road network graph G */
                //InitQueue((queue_t*) &(((forwarding_table_queue_node_t*)p)->DEQ), QTYPE_DIRECTIONAL_EDGE);
		break;

	case QTYPE_GLOBAL_PACKET:
	        ((global_packet_queue_node_t*)p)->ptr_queue = (global_packet_queue_t*)Q;
                ((global_packet_queue_node_t*)p)->order =  Q->size - 1;
		break;

	case QTYPE_STATIONARY_NODE:
	        ((stationary_node_queue_node_t*)p)->ptr_queue = (stationary_node_queue_t*)Q;
                ((stationary_node_queue_node_t*)p)->order =  Q->size - 1;

		/* initialize the stationary node's packet_queue for packets */
                //InitQueue((queue_t*) &(((stationary_node_queue_node_t*)p)->packet_queue), QTYPE_PACKET);
		break;

	case QTYPE_PACKET_TRAJECTORY:
	    ((packet_trajectory_queue_node_t*)p)->ptr_queue = (packet_trajectory_queue_t*)Q;
        ((packet_trajectory_queue_node_t*)p)->order =  Q->size - 1;
		break;

	case QTYPE_PROBABILITY_AND_STATISTICS:
	    ((probability_and_statistics_queue_node_t*)p)->ptr_queue = (probability_and_statistics_queue_t*)Q;
        ((probability_and_statistics_queue_node_t*)p)->order =  Q->size - 1;
        break;

	case QTYPE_CONDITIONAL_FORWARDING_PROBABILITY:
		((conditional_forwarding_probability_queue_node_t*)p)->ptr_queue = (conditional_forwarding_probability_queue_t*)Q;
		((conditional_forwarding_probability_queue_node_t*)p)->order =  Q->size - 1;
		break;

	case QTYPE_TARGET_POINT: //let the target_point_queue_node's ptr_queue point to its target_point_queue
		((target_point_queue_node_t*)p)->ptr_queue = (target_point_queue_t*)Q;
		((target_point_queue_node_t*)p)->order = Q->size - 1;
		break;

	case  QTYPE_MINIMUM_PRIORITY:
		((minimum_priority_queue_node_t*)p)->ptr_queue = (minimum_priority_queue_t*)Q;
		((minimum_priority_queue_node_t*)p)->order = Q->size - 1;
		break;

	case QTYPE_NEIGHBOR_LIST:
		((neighbor_list_queue_node_t*)p)->ptr_queue = (neighbor_list_queue_t*)Q;
		((neighbor_list_queue_node_t*)p)->order = Q->size - 1;
		break;

	case QTYPE_PARENT_LIST:
		((parent_list_queue_node_t*)p)->ptr_queue = (parent_list_queue_t*)Q;
		((parent_list_queue_node_t*)p)->order = Q->size - 1;
		break;

	case QTYPE_ADJACENCY_LIST:
		((adjacency_list_queue_node_t*)p)->ptr_queue = (adjacency_list_queue_t*)Q;
		((adjacency_list_queue_node_t*)p)->order = Q->size - 1;
		break;

	case QTYPE_ADJACENCY_LIST_POINTER:
		((adjacency_list_pointer_queue_node_t*)p)->ptr_queue = (adjacency_list_pointer_queue_t*)Q;
		((adjacency_list_pointer_queue_node_t*)p)->order = Q->size - 1;
		break;

	case QTYPE_PACKET_POINTER:
		((packet_pointer_queue_node_t*)p)->ptr_queue = (packet_pointer_queue_t*)Q;
		((packet_pointer_queue_node_t*)p)->order =  Q->size - 1;
		break;

	default:
		break;
	}
	
	return p;
}

queue_node_t* Enqueue_By_KeyAscendingOrder(queue_t *Q, queue_node_t *node)
{ //enqueue node into queue Q by the ascending order of node's key and return the pointer to the new queue node
	queue_node_t *p = NULL; //pointer to a new queue node
	queue_node_t *q = NULL; //pointer to an existing queue node in the queue Q
	int i = 0; //loop-index
	double key_new = 0; //key value of the new node
	double key = 0; //key value of a node in Q

	assert_memory(Q);
	assert_memory(node);

	p = MakeQueueNode(Q->type, node);
	//make a queue node corresponding to queue type and copy the data of node into its data portion

	switch(Q->type)
	{
	case QTYPE_MINIMUM_PRIORITY:
			key_new = ((minimum_priority_queue_node_t*)p)->key;
		break;

	case QTYPE_ADJACENCY_LIST:
			key_new = ((adjacency_list_queue_node_t*)p)->key;
		break;

	default:
		printf("%s:%d queue type(%d) is not supported!\n",
				__FUNCTION__, __LINE__,
				Q->type);
		free(p);
		exit(1);	
	}

	/* find a place to insert the new node p into Q with p's key */
	if(Q->size == 0) //if-1
	{ //insert node before head to let node be the last node in the queue Q
		p->prev = Q->head.prev;
		p->next = &(Q->head);
		Q->head.prev->next = p;
		Q->head.prev = p;
	} //end of if-1
	else //else-2
	{
		q = &(Q->head);
		for(i = 0; i < Q->size; i++)
		{
			q = q->next;

			switch(Q->type)
			{
			case QTYPE_MINIMUM_PRIORITY:
				key =((minimum_priority_queue_node_t*)q)->key;
				break;

			case QTYPE_ADJACENCY_LIST:
				key = ((adjacency_list_queue_node_t*)q)->key;
				break;
			}

			if(key_new <= key)
			{ //insert the new node with key_new in front of the node with key in the queue Q
				p->prev = q->prev;
				p->next = q;
				q->prev->next = p;
				q->prev = p;
				break;
			} 
			else if(i == Q->size - 1 )
			{ //insert node before head to let node be the last node in the queue Q
				p->prev = Q->head.prev;
				p->next = &(Q->head);
				Q->head.prev->next = p;
				Q->head.prev = p;
			}
		}
	} //end of else-2
	
	Q->size++;

	/* perform work specific to queue type */
	switch(Q->type)
	{
	case QTYPE_MINIMUM_PRIORITY:
		((minimum_priority_queue_node_t*)p)->ptr_queue = (minimum_priority_queue_t*)Q; 
		((minimum_priority_queue_node_t*)p)->order = Q->size - 1;

    	InitQueue((queue_t*) &(((minimum_priority_queue_node_t*)p)->parent_list), 
				QTYPE_PARENT_LIST);
		break;

	case QTYPE_ADJACENCY_LIST:
		((adjacency_list_queue_node_t*)p)->ptr_queue = (adjacency_list_queue_t*)Q;
    	((adjacency_list_queue_node_t*)p)->order = Q->size - 1;
    
    	InitQueue((queue_t*) &(((adjacency_list_queue_node_t*)p)->neighbor_list),
    			QTYPE_NEIGHBOR_LIST);
    	InitQueue((queue_t*) &(((adjacency_list_queue_node_t*)p)->parent_list), 
				QTYPE_PARENT_LIST);
		break;
	}

	return p;
}

queue_node_t* Dequeue(queue_t *Q)
{ //dequeue node from queue Q
	//static queue_node_t node;
	queue_node_t *p = NULL, *q = NULL;
	int i; //index of for-loop

	assert_memory(Q);

	if(Q->size == 0)
		return NULL;

	p = Q->head.next;
	Q->head.next->next->prev = &(Q->head);
	Q->head.next = Q->head.next->next;

	Q->size--;

	//memcpy(&node, p, sizeof(node));
	//free(p);

	//return &node;

	/* perform work specific to queue type */
	switch(Q->type)
	{
	case QTYPE_VERTEX_SET: //update the representative information
		q = &(Q->head);
		for(i = 0; i < Q->size; i++)
		{
			q = q->next;
			((vertex_set_queue_node_t*)q)->representative = (vertex_set_queue_node_t*)Q->head.next;
		}
		break;

	case QTYPE_EDGE_SET: //update the representative information
		q = &(Q->head);
		for(i = 0; i < Q->size; i++)
		{
			q = q->next;
			((edge_set_queue_node_t*)q)->representative = (edge_set_queue_node_t*)Q->head.next;
		}
		break;

	case QTYPE_VEHICLE_TRAJECTORY: //update the pointer to the first queue node
		/* let the current_order_qnode point to the first queue node */
		//if(Q->size > 0)
		//{
		//  ((vehicle_trajectory_queue_t*)Q)->current_order_qnode = ((vehicle_trajectory_queue_t*)Q)->head.next;
		//}
		break;
	}

	return p;
	/** Note: we need to free the memory pointed by p later */
}

queue_node_t* Dequeue_With_QueueNodePointer(queue_t *Q, queue_node_t *p)
{ //dequeue node corresponding to pointer p from queue Q
	//static queue_node_t node;
	queue_node_t *q = NULL;
	int i; //index of for-loop

	assert_memory(Q);

	if(Q->size == 0)
		return NULL;

	//p = Q->head.next;
	//Q->head.next->next->prev = &(Q->head);
	//Q->head.next = Q->head.next->next;

	p->prev->next = p->next;
	p->next->prev = p->prev;

	Q->size--;

	//memcpy(&node, p, sizeof(node));
	//free(p);

	//return &node;

	/* perform work specific to queue type */
	switch(Q->type)
	{
	case QTYPE_VERTEX_SET: //update the representative information
		q = &(Q->head);
		for(i = 0; i < Q->size; i++)
		{
			q = q->next;
			((vertex_set_queue_node_t*)q)->representative = (vertex_set_queue_node_t*)Q->head.next;
		}
		break;

	case QTYPE_EDGE_SET: //update the representative information
		q = &(Q->head);
		for(i = 0; i < Q->size; i++)
		{
			q = q->next;
			((edge_set_queue_node_t*)q)->representative = (edge_set_queue_node_t*)Q->head.next;
		}
		break;

	case QTYPE_VEHICLE_TRAJECTORY: //update the pointer to the first queue node
		/* let the current_order_qnode point to the first queue node */
		//if(Q->size > 0)
		//{
		//  ((vehicle_trajectory_queue_t*)Q)->current_order_qnode = ((vehicle_trajectory_queue_t*)Q)->head.next;
		//}
		break;
	}

	return p;
	/** Note: we need to free the memory pointed by p later */
}

void EmptyQueue(queue_t *Q)
{ //empty queue Q by removing the queue nodes in Q
	queue_node_t *p = NULL, *q = NULL;

	if(Q == NULL || Q->size == 0)
		return;

	for(p = Q->head.next; p != &(Q->head);) //for-1
	{
		q = p;
		p = p->next;
		/* destroy queue node q along with its queue(s) as field(s) */
		DestroyQueueNode(Q->type, q);
	} //end of for-1

	/* reset Q->size to 0 */
	Q->size = 0;

	/* reorganize the pointers in Q */
	Q->head.next = Q->head.prev = &(Q->head);
}

void DestroyQueue(queue_t *Q)
{ //destory queue Q
	if(Q == NULL || Q->size == 0)
		return;

	/* perform work specific to queue type */
	switch(Q->type) //switch-1
	{
	case QTYPE_FORWARDING_TABLE: //free the memory of the index table of forwarding table queue
	    if(Q->size > 0)
	    {
	      free(((forwarding_table_queue_t*)Q)->index_table);
	    }
	    break;

	case QTYPE_GLOBAL_PACKET: //free the memory of packet vectors
		if(((global_packet_queue_t*)Q)->physical_vector_size > 0)
		{
		  free(((global_packet_queue_t*)Q)->packet_id_bitmap_vector);	
		  free(((global_packet_queue_t*)Q)->packet_delivery_delay_vector);	
		  free(((global_packet_queue_t*)Q)->packet_deletion_count_vector);

		  ((global_packet_queue_t*)Q)->physical_vector_size = 0;
		}
		break;

	case QTYPE_ADJACENCY_LIST: //free the memory of bitmap if bitmap_size > 0 
		if(((adjacency_list_queue_t*)Q)->bitmap_size > 0)
		{
		  free(((adjacency_list_queue_t*)Q)->bitmap);
		  free(((adjacency_list_queue_t*)Q)->bitmap_gnodes);
		  ((adjacency_list_queue_t*)Q)->bitmap_size = 0;
		}
		break;
	}

	/* empty queue Q by removing the queue nodes in Q and initialize Q */
	EmptyQueue(Q);

	/* initialize the queue Q */
	InitQueue(Q, Q->type);
}

void ResetQueue(queue_t *Q)
{ //reset queue Q by emptying Q and resetting the vector(s) in Q
	if(Q == NULL || Q->size == 0)
		return;

	/* empty queue Q by removing the queue nodes in Q */
	 EmptyQueue(Q);

	/* perform work specific to queue type */
	switch(Q->type) //switch-1
	{
	case QTYPE_ADJACENCY_LIST: //reset the memory of bitmap if bitmap_size > 0 and set dst_vehicle_gnode to NULL
		if(((adjacency_list_queue_t*)Q)->bitmap_size > 0)
		{
		  memset(((adjacency_list_queue_t*)Q)->bitmap, 0, ((adjacency_list_queue_t*)Q)->bitmap_size*sizeof(boolean));
		  //Note: make sure that the size of bitmap is bitmap_size*sizeof(boolean) because the enum type of boolean has four bytes.
		  //memset(((adjacency_list_queue_t*)Q)->bitmap, 0, ((adjacency_list_queue_t*)Q)->bitmap_size);
		  //
		  memset(((adjacency_list_queue_t*)Q)->bitmap_gnodes, 0, ((adjacency_list_queue_t*)Q)->bitmap_size*sizeof(adjacency_list_queue_node_t*));
		}
	
		((adjacency_list_queue_t*)Q)->dst_vehicle_gnode = NULL;
		break;

	default:
		printf("%s:%d Q->type(%d) is not supported!\n",
				__FUNCTION__, __LINE__,
				Q->type);
		exit(1);
	}

	/* reinitialize size and head node */
	Q->size = 0;
	Q->head.next = Q->head.prev = &(Q->head);
}

void DestroyQueueNode(queue_type_t type, queue_node_t *q)
{ //destory queue-node q of queue-type type along with its queue(s) as field(s)
	vertex_set_queue_t *pVertexSetQueue = NULL; //pointer to vertex set queue that is cluster set

	if(q == NULL)
	{
		printf("%s:%d queue node q is NULL\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}

	/* perform work specific to queue type */
	switch(type) //switch-1
	{
	case QTYPE_VERTEX_SET: //delete cluster set queue
		pVertexSetQueue = ((vertex_set_queue_node_t*)q)->cluster_set;
		if(pVertexSetQueue != NULL)
		{
			if(pVertexSetQueue->reference_count > 1)
				pVertexSetQueue->reference_count--; //just decrease reference_count because other vertex_set node(s) refer to this cluster set
			else
				DestroyQueue((queue_t*) pVertexSetQueue); //delete cluster set queue because any other vertex_set node does not refer to this cluster set
		}
		break;

	case QTYPE_EDGE: //delete the related queues
		DestroyQueue((queue_t*) &(((edge_queue_node_t*)q)->sensor_location_list)); //delete sensor location queue
		DestroyQueue((queue_t*) &(((edge_queue_node_t*)q)->subedge_list)); //delete subedge queue
		DestroyQueue((queue_t*) &(((edge_queue_node_t*)q)->sensing_hole_segment_list)); //delete sensing hole segment queue
		DestroyQueue((queue_t*) &(((edge_queue_node_t*)q)->sensing_hole_endpoint_list)); //delete sensing hole endpoint queue
		break;

	case QTYPE_DIRECTIONAL_EDGE: //delete the vehicle movement queue
		DestroyQueue((queue_t*) &(((directional_edge_queue_node_t*)q)->vehicle_movement_list)); //delete vehicle movement queue
		DestroyQueue((queue_t*) &(((directional_edge_queue_node_t*)q)->convoy_list)); //delete convoy queue
		DestroyQueue((queue_t*) &(((directional_edge_queue_node_t*)q)->probability_and_statistics_queue)); //delete probability-and-statistics queue
		break;
			
	case QTYPE_DELAY: //delete the delay component queue
		DestroyQueue((queue_t*) &(((delay_queue_node_t*)q)->delay_component_list)); //delete delay component queue		     
		break;

	case QTYPE_CONVOY: //delete the vehicle queue
		DestroyQueue((queue_t*) &(((convoy_queue_node_t*)q)->vehicle_list)); //delete vehicle queue		     
		break;

	case QTYPE_VEHICLE: //make the pointer to the convoy queue node have NULL since the corresponding convoy for the vehicle is destroyed
		((vehicle_queue_node_t*)q)->vnode->ptr_convoy_queue_node = NULL;
		((vehicle_queue_node_t*)q)->vnode->flag_convoy_registration = FALSE;
		break;

	case QTYPE_VEHICLE_MOVEMENT: //make the pointer to the vehicle movement queue node have NULL since the corresponding vehicle movement node for the vehicle is destroyed
		((vehicle_movement_queue_node_t*)q)->vnode->ptr_vehicle_movement_queue_node = NULL;
		((vehicle_movement_queue_node_t*)q)->vnode->flag_vehicle_movement_queue_registration = FALSE;
		break;

	case QTYPE_DESTINATION_VEHICLE: //delete the mobility queue
		DestroyQueue((queue_t*) &(((destination_vehicle_queue_node_t*)q)->mobility_list)); //delete mobility queue		     
		break;

	case QTYPE_ACCESS_POINT:
		AP_Delete((access_point_queue_node_t*)q); //delete the graph structure and a number of queues for AP q
		break;

	case QTYPE_PACKET: //delete the vehicle trajectory queue
		DestroyQueue((queue_t*) &(((packet_queue_node_t*)q)->vehicle_trajectory)); //delete vehicle trajectory queue		     
		DestroyQueue((queue_t*) &(((packet_queue_node_t*)q)->carrier_trace)); //delete carrier trace queue		     
		DestroyQueue((queue_t*) &(((packet_queue_node_t*)q)->packet_trajectory)); //delete packet trajectory queue
		TPD_Free_Predicted_Encounter_Graph_For_Packet((packet_queue_node_t*)q); //free the predicted encounter graph
		break;

	case QTYPE_FORWARDING_TABLE:
		Free_Graph(((forwarding_table_queue_node_t*)q)->G, ((forwarding_table_queue_node_t*)q)->G_size); //release the memory allocated to real graph Gr
		DestroyQueue((queue_t*) &(((forwarding_table_queue_node_t*)q)->EQ)); //delete undirectional edge queue EQ
		DestroyQueue((queue_t*) &(((forwarding_table_queue_node_t*)q)->DEQ)); //delete directional edge queue DEQ

		/** deallocate the shortest delay matrices */
		Floyd_Warshall_Free_Matrices_For_EDD(&(((forwarding_table_queue_node_t*)q)->Dr_edd), &(((forwarding_table_queue_node_t*)q)->Mr_edd), &(((forwarding_table_queue_node_t*)q)->Sr_edd), &(((forwarding_table_queue_node_t*)q)->matrix_size_for_edd_in_Gr));

		/** deallocate the shortest cost matrices */
		Floyd_Warshall_Free_Matrices_For_EDC(&(((forwarding_table_queue_node_t*)q)->Wr_edc), &(((forwarding_table_queue_node_t*)q)->Dr_edc), &(((forwarding_table_queue_node_t*)q)->Mr_edc), &(((forwarding_table_queue_node_t*)q)->Sr_edc), &(((forwarding_table_queue_node_t*)q)->matrix_size_for_edc_in_Gr));
		break;

	case QTYPE_STATIONARY_NODE:
		DestroyQueue((queue_t*) &(((stationary_node_queue_node_t*)q)->packet_queue)); //delete packet queue		     
		break;             

	case QTYPE_PROBABILITY_AND_STATISTICS:
		DestroyQueue((queue_t*) &(((probability_and_statistics_queue_node_t*)q)->conditional_forwarding_probability_queue)); //delete conditional forwarding probability queue		     
		break;

	case QTYPE_MINIMUM_PRIORITY:
		DestroyQueue((queue_t*) &(((minimum_priority_queue_node_t*)q)->parent_list)); //delete parent list queue
		break;

	case QTYPE_ADJACENCY_LIST: //delete the relate neighbor list queue
		DestroyQueue((queue_t*) &(((adjacency_list_queue_node_t*)q)->neighbor_list)); //delete neighbor list queue
		DestroyQueue((queue_t*) &(((adjacency_list_queue_node_t*)q)->parent_list)); //delete parent list queue
		break;

	case QTYPE_GLOBAL_PACKET: //delete the packet pointer queue
		DestroyQueue((queue_t*)&((global_packet_queue_node_t*)q)->packet_pointer_list);
		//delete packet_pointer_list 
		break;

	default:
		break;
	} //end of switch

	/* free the memory of q */
	free(q);
}

int SizeofQueue(queue_t *Q)
{ //return the size of queue Q
	return Q->size;
}

queue_node_t* GetQueueNode(queue_t *Q, int index)
{ //return the queue node corresponding to index; the index of the first node is 0.
	queue_node_t* p = NULL;
	int size = Q->size; //size of queue Q, i.e., the number of valid queue nodes
	int i; //for-loop index

	if(index >= Q->size)
	{
		printf("GetQueueNode(): Error: index(%d) >= Q->size(%d)\n", index, Q->size);
		exit(1);
	}

	for(i = 0, p = &(Q->head); i <= index; i++)
		p = p->next;

	return p;
}

queue_node_t* GetFrontQueueNode(queue_t *Q)
{ //return the front queue node corresponding to the first enqueued node.
	queue_node_t* p = NULL;
	int size = Q->size; //size of queue Q, i.e., the number of valid queue nodes

	if(size == 0)
	  return p;
	
        p = Q->head.next;

	return p;
}

queue_node_t* GetRearQueueNode(queue_t *Q)
{ //return the rear queue node corresponding to the last enqueued node.
	queue_node_t* p = NULL;
	int size = Q->size; //size of queue Q, i.e., the number of valid queue nodes

	if(size == 0)
	  return p;

        p = Q->head.prev;

	return p;
}

void SortSensorQueue(sensor_queue_t* Q)
{ //sort sensor queue nodes in ascending order according to offset
	int sensor_num = Q->size; //number of sensors deployed on the edge
	sensor_queue_node_t **A; //array of sensor node pointers
	sensor_queue_node_t *pQueueNode = NULL; //pointer to a sensor queue node
	int i; //index for for-loop

    /** for testing of quick sort */
	//int B[8] = {5, 3, 2, 6, 4, 1, 5, 7};
	//int B[8] = {5, 3, 2, 6, 4, 1, 3, 7};
	//perform quick sort
	//QuickSort(B, 0, 7);


	/* generate sensor array containing pointers to all of the sensor nodes in schedule table */
	A = (sensor_queue_node_t**) calloc(sensor_num, sizeof(sensor_queue_node_t*));
	assert_memory(A);

	pQueueNode = &(Q->head);
	for(i = 0; i < sensor_num; i++)
	{
		pQueueNode = pQueueNode->next;
		A[i] = pQueueNode;
	}

	/* sort arrary A in ascending order with sensor position's offset */
	QuickSortForSensorArray(A, 0, sensor_num-1);
	//perform quick sort for sensor array

	/* rearrange the pointers of Q using A in the ascending order */
	RearrangeSensorQueue(Q, A);

	/* release the memory allocated to array A */
	free(A);
}

void RearrangeSensorQueue(sensor_queue_t* Q, sensor_queue_node_t **A)
{ //rearrange the pointers of sensor queue Q using A in the ascending order
	sensor_queue_node_t *p = NULL, *q = NULL; //pointers to sensor queue nodes
	int sensor_num = Q->size; //number of sensors deployed on the edge
	int i; //index for for-loop

	/* rearrange the links using A such that A[i]'s offsets are ordered in ascending order */
	q = &(Q->head);
	q->next = &(Q->head);
	q->prev = &(Q->head);
	for(i = 0; i < sensor_num; i++, p = p->next)
	{
		A[i]->order_in_Gr = i; //specify the sensor's order from the tail node from the edge in the real graph Gr
		A[i]->order_in_Gv = i; //specify the sensor's order from the tail node from the edge in the virtual graph Gv
		A[i]->order_for_live_sensors = i; //specify the live sensor order from the tail node from the edge in the virtual graph Gv

		p = A[i];		
		p->prev = Q->head.prev;
		p->next = &(Q->head);
		Q->head.prev->next = p;
		Q->head.prev = p;
	}
}

void SortEdgeSetQueue(edge_set_queue_t* Q)
{ //sort edge_set queue nodes in ascending order according to weight
	int edge_num = Q->size; //number of edges in the edge_set queue
	edge_set_queue_node_t **A; //array of edge_set node pointers
	edge_set_queue_node_t *pQueueNode = NULL; //pointer to an edge_set queue node
	int i; //index for for-loop

	/* generate edge_set array containing pointers to all of the nodes in edge_set queue */
	A = (edge_set_queue_node_t**) calloc(edge_num, sizeof(edge_set_queue_node_t*));
	assert_memory(A);

	pQueueNode = &(Q->head);
	for(i = 0; i < edge_num; i++)
	{
		pQueueNode = pQueueNode->next;
		A[i] = pQueueNode;
	}

	/* sort arrary A in ascending order with edge's weight */
	QuickSortForEdgeSetArray(A, 0, edge_num-1);
	//perform quick sort for edge_set array

	/* rearrange the pointers of Q using A in the ascending order */
	RearrangeEdgeSetQueue(Q, A);

	/* release the memory allocated to array A */
	free(A);
}

void RearrangeEdgeSetQueue(edge_set_queue_t* Q, edge_set_queue_node_t **A)
{ //rearrange the pointers of edge_set queue Q using A in the ascending order
	edge_set_queue_node_t *p = NULL, *q = NULL; //pointers to edge_set queue nodes
	int edge_num = Q->size; //number of edges in the edge_set queue
	int i; //index for for-loop

	/* rearrange the links using A such that A[i]'s offsets are ordered in ascending order */
	q = &(Q->head);
	q->next = &(Q->head);
	q->prev = &(Q->head);
	for(i = 0; i < edge_num; i++, p = p->next)
	{
		p = A[i];		
		p->prev = Q->head.prev;
		p->next = &(Q->head);
		Q->head.prev->next = p;
		Q->head.prev = p;
	}
}

void SortAngleQueue(angle_queue_t* Q)
{ //sort angle queue nodes in ascending order according to angle
	int neighbor_num = Q->size; //number of neighboring nodes in the angle queue
	angle_queue_node_t **A; //array of angle node pointers
	angle_queue_node_t *pQueueNode = NULL; //pointer to an angle queue node
	int i; //index for for-loop

	/* generate angle queue node array containing pointers to all of the nodes in angle queue */
	A = (angle_queue_node_t**) calloc(neighbor_num, sizeof(angle_queue_node_t*));
	assert_memory(A);

	pQueueNode = &(Q->head);
	for(i = 0; i < neighbor_num; i++)
	{
		pQueueNode = pQueueNode->next;
		A[i] = pQueueNode;
	}

	/* sort arrary A in ascending order with the neighbor's angle */
	QuickSortForAngleArray(A, 0, neighbor_num-1);
	//perform quick sort for angle array

	/* rearrange the pointers of Q using A in the ascending order */
	RearrangeAngleQueue(Q, A);

	/* release the memory allocated to array A */
	free(A);
}

void RearrangeAngleQueue(angle_queue_t* Q, angle_queue_node_t **A)
{ //rearrange the pointers of angle queue Q using A in the ascending order
	angle_queue_node_t *p = NULL, *q = NULL; //pointers to angle queue nodes
	int neighbor_num = Q->size; //number of neighbors in the angle queue
	int i; //index for for-loop

	/* rearrange the links using A such that A[i]'s offsets are ordered in ascending order */
	q = &(Q->head);
	q->next = &(Q->head);
	q->prev = &(Q->head);
	for(i = 0; i < neighbor_num; i++, p = p->next)
	{
		p = A[i];		
		p->prev = Q->head.prev;
		p->next = &(Q->head);
		Q->head.prev->next = p;
		Q->head.prev = p;
	}
}

void SortIntersection_EDD_Queues_In_Graph(parameter_t *param, struct_graph_node *G, int G_size)
{ //sort the intersection edd queue for each intersection in graph G
        int i = 0; //index for for-loop

	for(i = 0; i < G_size; i++)
	{
          SortIntersection_EDD_Queue(param, G[i].intersection_edd_queue);
          //sort intersection EDD queue nodes in ascending order according to EDD
	}
}

void SortIntersection_EDD_Queue(parameter_t *param, intersection_edd_queue_t* Q)
{ //sort intersection EDD queue nodes in ascending order according to EDD
	int neighbor_num = Q->size; //number of neighboring nodes in the intersection EDD queue
	intersection_edd_queue_node_t **A; //array of intersection node pointers
	intersection_edd_queue_node_t *pQueueNode = NULL; //pointer to an intersection EDD queue node
	int i; //index for for-loop

	/* generate intersection EDD queue node array containing pointers to all of the nodes in intersection EDD queue */
	A = (intersection_edd_queue_node_t**) calloc(neighbor_num, sizeof(intersection_edd_queue_node_t*));
	assert_memory(A);

	pQueueNode = &(Q->head);
	for(i = 0; i < neighbor_num; i++)
	{
		pQueueNode = pQueueNode->next;

		/* copy pQueueNode->head_gnode->EDD (or EDD_VAR) to pQueueNode->EDD according to vanet metric type */
                if(param->vehicle_vanet_metric_type == VANET_METRIC_EDD)
                  pQueueNode->EDD = pQueueNode->head_gnode->EDD;
                else if(param->vehicle_vanet_metric_type == VANET_METRIC_EDD_VAR)
                  pQueueNode->EDD = pQueueNode->head_gnode->EDD_VAR; 
                else
                {
                  printf("SortIntersection_EDD_Queue(): Error: Unknown vanet_metric_type=%d\n", param->vehicle_vanet_metric_type);
                  exit(1);
                }

		A[i] = pQueueNode;
	}

	/* sort arrary A in ascending order with the edge's EDD */
	QuickSortForIntersection_EDD_Array(A, 0, neighbor_num-1);
	//perform quick sort for intersection EDD array

	/* rearrange the pointers of Q using A in the ascending order */
	RearrangeIntersection_EDD_Queue(Q, A);

	/* release the memory allocated to array A */
	free(A);
}

void RearrangeIntersection_EDD_Queue(intersection_edd_queue_t* Q, intersection_edd_queue_node_t **A)
{ //rearrange the pointers of intersection EDD queue Q using A in the ascending order
	intersection_edd_queue_node_t *p = NULL, *q = NULL; //pointers to intersection EDD queue nodes
	int neighbor_num = Q->size; //number of neighbors in the intersection EDD queue
	int i; //index for for-loop
	
	/* rearrange the links using A such that A[i]'s offsets are ordered in ascending order */
	q = &(Q->head);
	q->next = &(Q->head);
	q->prev = &(Q->head);
	for(i = 0; i < neighbor_num; i++, p = p->next)
	{
		p = A[i];
	
		/* set the order of the queue node according to its EDD */
		p->order = i;

		p->prev = Q->head.prev;
		p->next = &(Q->head);
		Q->head.prev->next = p;
		Q->head.prev = p;
	}
}

void SortVehicleMovementQueues_In_Graph(struct_graph_node *G, int G_size)
{ //sort the vehicle movement queue for each directional edge in graph G
        int i = 0, j = 0; //IDs for vertices in graph G
	struct_graph_node *ptr = NULL; //pointer to graph node

	/* make adjacency matrix W with adjacent list G */
	for(i = 0; i < G_size; i++)
	{
		ptr = G[i].next;
		if(ptr == NULL)
			continue;

		while(ptr != NULL)
		{
			j = atoi(ptr->vertex) - 1; 
			//node id starts from 1, but it should decrease by 1 for weight matrix where the id starts from 0.
                        SortVehicleMovementQueue(&(ptr->ptr_directional_edge_node->vehicle_movement_list));
                        //sort vehicle movement queue nodes in ascending order according to vehicle's offset in the directional edge <v_i,v_j>

			ptr = ptr->next;
		}
	}
}

void SortVehicleMovementQueue(vehicle_movement_queue_t* Q)
{ //sort vehicle movement queue nodes in ascending order according to offset in the directional edge
	int vehicle_num = Q->size; //number of vehicle moving on the directional edge
	vehicle_movement_queue_node_t **A; //array of vehicle movement queue node pointers
	vehicle_movement_queue_node_t *pQueueNode = NULL; //pointer to a vehicle movement queue node
	int i; //index for for-loop

	/* generate vehicle movement queue node array containing pointers to all of the nodes in vehicle movement queue */
	A = (vehicle_movement_queue_node_t**) calloc(vehicle_num, sizeof(vehicle_movement_queue_node_t*));
	assert_memory(A);

	pQueueNode = &(Q->head);
	for(i = 0; i < vehicle_num; i++)
	{
		pQueueNode = pQueueNode->next;
		A[i] = pQueueNode;
	}

	/* sort arrary A in ascending order with the vehicle's offset */
	QuickSortForVehicleMovementArray(A, 0, vehicle_num-1);
	//perform quick sort for vehicle movement queue node array

	/* rearrange the pointers of Q using A in the ascending order */
	RearrangeVehicleMovementQueue(Q, A);

	/* release the memory allocated to array A */
	free(A);
}

void RearrangeVehicleMovementQueue(vehicle_movement_queue_t* Q, vehicle_movement_queue_node_t **A)
{ //rearrange the pointers of vehicle movement queue Q using A in the ascending order
	vehicle_movement_queue_node_t *p = NULL, *q = NULL; //pointers to vehicle movement queue nodes
	int vehicle_num = Q->size; //number of vehicles moving on the directional edge
	int i; //index for for-loop
	
	/* rearrange the links using A such that A[i]'s offsets are ordered in ascending order */
	q = &(Q->head);
	q->next = &(Q->head);
	q->prev = &(Q->head);
	for(i = 0; i < vehicle_num; i++, p = p->next)
	{
		p = A[i];
	
		/* set the order of the queue node according to its vehicle offset */
		p->order = i;

		p->prev = Q->head.prev;
		p->next = &(Q->head);
		Q->head.prev->next = p;
		Q->head.prev = p;
	}
}

void SortVehicleQueue(vehicle_queue_t* Q)
{ //sort vehicle queue nodes in ascending order according to offset in the directional edge
	int vehicle_num = Q->size; //number of vehicle moving on the directional edge
	vehicle_queue_node_t **A; //array of vehicle queue node pointers
	vehicle_queue_node_t *pQueueNode = NULL; //pointer to a vehicle queue node
	int i; //index for for-loop

	/* generate vehicle queue node array containing pointers to all of the nodes in vehicle queue */
	A = (vehicle_queue_node_t**) calloc(vehicle_num, sizeof(vehicle_queue_node_t*));
	assert_memory(A);

	pQueueNode = &(Q->head);
	for(i = 0; i < vehicle_num; i++)
	{
		pQueueNode = pQueueNode->next;
		pQueueNode->offset = pQueueNode->vnode->current_pos_in_digraph.offset; //set pQueueNode->offset to the offset of the vehicle pointed by pQueueNode on the directional edge
		A[i] = pQueueNode;
	}

	/* sort arrary A in ascending order with the vehicle's offset */
	QuickSortForVehicleArray(A, 0, vehicle_num-1);
	//perform quick sort for vehicle queue node array

	/* rearrange the pointers of Q using A in the ascending order */
	RearrangeVehicleQueue(Q, A);

	/* release the memory allocated to array A */
	free(A);
}

void RearrangeVehicleQueue(vehicle_queue_t* Q, vehicle_queue_node_t **A)
{ //rearrange the pointers of vehicle queue Q using A in the ascending order
	vehicle_queue_node_t *p = NULL, *q = NULL; //pointers to vehicle queue nodes
	int vehicle_num = Q->size; //number of vehicles moving on the directional edge
	int i; //index for for-loop
	
	/* rearrange the links using A such that A[i]'s offsets are ordered in ascending order */
	q = &(Q->head);
	q->next = &(Q->head);
	q->prev = &(Q->head);
	for(i = 0; i < vehicle_num; i++, p = p->next)
	{
		p = A[i];
	
		/* set the order of the queue node according to its vehicle offset */
		p->order = i;
		
		p->prev = Q->head.prev;
		p->next = &(Q->head);
		Q->head.prev->next = p;
		Q->head.prev = p;
	}
}

void SortDirectionalEdgeQueue(directional_edge_queue_t* Q)
{ //sort directional edge queue nodes in ascending order according to eid
	int edge_num = Q->size; //number of edges in the directional edge queue
	directional_edge_queue_node_t **A; //array of directional edge queue node pointers
	directional_edge_queue_node_t *pQueueNode = NULL; //pointer to a directional edge queue node
	int i; //index for for-loop

	/* generate directional edge array containing pointers to all of the nodes in directional edge queue */
	A = (directional_edge_queue_node_t**) calloc(edge_num, sizeof(directional_edge_queue_node_t*));
	assert_memory(A);

	pQueueNode = &(Q->head);
	for(i = 0; i < edge_num; i++)
	{
		pQueueNode = pQueueNode->next;
		A[i] = pQueueNode;
	}

	/* sort arrary A in ascending order with edge's eid */
	QuickSortForDirectionalEdgeArray(A, 0, edge_num-1);
	//perform quick sort for directional_edge_queue array

	/* rearrange the pointers of Q using A in the ascending order */
	RearrangeDirectionalEdgeQueue(Q, A);

	/* release the memory allocated to array A */
	free(A);
}

void RearrangeDirectionalEdgeQueue(directional_edge_queue_t* Q, directional_edge_queue_node_t **A)
{ //rearrange the pointers of directional_edge_queue Q using A in the ascending order of edge id 
	directional_edge_queue_node_t *p = NULL, *q = NULL; //pointers to directional_edge_queue nodes
	int edge_num = Q->size; //number of edges in the directional edge queue
	int i; //index for for-loop

	/* rearrange the links using A such that A[i]'s offsets are ordered in ascending order */
	q = &(Q->head);
	q->next = &(Q->head);
	q->prev = &(Q->head);
	for(i = 0; i < edge_num; i++, p = p->next)
	{
		p = A[i];
		p->order = i; //set the order in directional edge queue according to eid
		p->prev = Q->head.prev;
		p->next = &(Q->head);
		Q->head.prev->next = p;
		Q->head.prev = p;
	}
}

void CopyQueue(queue_t *dst_Q, queue_t *src_Q)
{ //copy the contents of src_Q into those of dst_Q
	int i;
	queue_node_t *pQueueNode = NULL;

	if(dst_Q->type != src_Q->type)
	{
		printf("CopyQueue():we cannot copy because dst_Q->type(%d) and src_Q->type(%d) are different types\n", dst_Q->type, src_Q->type);
		exit(1);
	}

	/* if there are queue nodes in dst_Q, then destroy dst_Q first */
	if(dst_Q->size > 0)
	{
	  DestroyQueue(dst_Q); //destroy the queue dst_Q, and then initialize it by calling InitQueue() within the function
	}

	/* copy the contents of src_Q into dst_Q */
	pQueueNode = &(src_Q->head);
	for(i = 0; i < src_Q->size; i++)
	{
		pQueueNode = pQueueNode->next;
		Enqueue(dst_Q, pQueueNode);
	}
	
	/* A queue of type QTYPE_SENSOR needs live_sensor_number to be copied */
	if(src_Q->type == QTYPE_SENSOR)
	{
		((sensor_queue_t*)dst_Q)->live_sensor_number = ((sensor_queue_t*)src_Q)->live_sensor_number;
		/* update the pointer of sensor list using ptr_queue_node->pSensorListEntry in order that sensor table entry in sensor table S can point to the correct sensor queue node for sensor information */
		pQueueNode = &(dst_Q->head);
		for(i = 0; i < dst_Q->size; i++)
		{
		  pQueueNode = pQueueNode->next;
		  *(((sensor_queue_node_t*)pQueueNode)->pSensorListEntry) = (sensor_queue_node_t*)pQueueNode; //update the pointer of sensor table entry in sensor table S to the sensor queue node
		}
	}
}

void ConstructEdgeQueue(edge_queue_t *Q, struct_graph_node *G, int G_size, parameter_t *param)
{ /* construct edge queue Q by building the same number of entries as the number of edges in G.
     The density of each edge is determined by sensor_density_distribution,
     sensor_density_standard_deviation and sensor_density_maximum_deviation.
     NOTE that the density in struct_graph_node in G is not updated with the new one.
  */

	edge_queue_node_t edge_node; //node for edge queue
	edge_queue_node_t *pEdgeNode = NULL; //pointer to an edge queue node
	subedge_queue_node_t subedge_node; //node for subedge queue
	struct_graph_node *p = NULL, *q = NULL;//pointers to graph nodes
	char u[NAME_SIZE], v[NAME_SIZE]; //vertices
	int i, j;
	int neighbor_num; //number of neighbor vetices
	boolean flip_flag = FALSE; //flag to indicate if the order of vertices in node matches 
	//that of an entry in schedule table; in this function, this is not used.
	double density = 0; //sensor density
	double min_density = 0; //minimum sensor density
	double max_density = 0; //maximum sensor density

	/* check whether G has at least one vertex */
	if(G_size == 0)
		return;

	/* initialize edge queue Q */
	InitQueue((queue_t*) Q, QTYPE_EDGE);
	
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
			if(LookupEdgeQueue(Q, u, v, &flip_flag) == NULL) //if
			{
				memset(&edge_node, 0, sizeof(edge_node));
				//edge_node.eid = eid++; //set the edge id
				//edge_node.eid = Q->size + 1; //set the edge id
				edge_node.weight = q->weight; //set the edge's weight

				/** select density according to sensor_density_distribution */
				if(param->sensor_density_distribution == EQUAL)
				{
				  edge_node.density = q->density;
				}
				else
				{
				  min_density = MAX(q->density - param->sensor_density_maximum_deviation, 0);
				  max_density = MAX(q->density + param->sensor_density_maximum_deviation, 0);
				  do
				  {
				    density = dist_func(param->sensor_density_distribution, q->density, param->sensor_density_standard_deviation*1.0);
				  } while((density < min_density) || (density > max_density)); //keep doing until min_density <= density <= max_density

				  edge_node.density = density; //set the sensor density
				}
				////////////////////////////

				strcpy(edge_node.tail_node, u);
				strcpy(edge_node.head_node, v);
				Enqueue((queue_t*) Q, (queue_node_t*) &edge_node); //enqueue the clone of node into Q and increase Q->size by 1.
			} //end of if
		} //end of for-1
	} //end of for-2

	/** enqueue a subedge corresponding to the edge entry where the subedge is the only edge in the subvision of the edge */
	pEdgeNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pEdgeNode = pEdgeNode->next;

		/* initialize the queues belonging to the edge entry */
		InitQueue((queue_t*) &(pEdgeNode->sensor_location_list), QTYPE_LOCATION); //initialize sensor location queue
		InitQueue((queue_t*) &(pEdgeNode->subedge_list), QTYPE_SUBEDGE); //initialize subedge queue
		InitQueue((queue_t*) &(pEdgeNode->sensing_hole_segment_list), QTYPE_HOLE_SEGMENT); //initialize sensing hole segment queue
		InitQueue((queue_t*) &(pEdgeNode->sensing_hole_endpoint_list), QTYPE_HOLE_ENDPOINT); //initialize sensing hole endpoint queue

		/* enqueue a subedge into the subedge list */
		memset(&subedge_node, 0, sizeof(subedge_node));
		subedge_node.eid = pEdgeNode->eid; //set the edge id
		subedge_node.order = 0; //set the order of the subedge
		subedge_node.weight = pEdgeNode->weight; //set the edge's weight
		subedge_node.edge_queue_entry = pEdgeNode; //register the pointer to the edge entry into edge_queue_entry
		subedge_node.schedule_table_entry = NULL; //schedule_table_entry will be set to the pointer to the corresponding entry in schedule table in InitTable()
		strcpy(subedge_node.tail_node, pEdgeNode->tail_node);
		strcpy(subedge_node.head_node, pEdgeNode->head_node);
		Enqueue((queue_t*) &(pEdgeNode->subedge_list), (queue_node_t*) &subedge_node); //enqueue the clone of node into Q and increase Q->size by 1.
	}
}

void ConstructDirectionalEdgeQueue(parameter_t *param, directional_edge_queue_t *Q, struct_graph_node *G, int G_size)
{ //construct directional edge queue Q by building the same number of entries as the number of edges in G.

  directional_edge_queue_node_t edge_node; //node for directional edge queue
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to an directional edge queue node
  int i = 0, j = 0; //indices of for-loops
  int seq_num = 0; //sequence number for the unique numbering for each road segment r_ij
  struct_graph_node *pTailNode = NULL, *pHeadNode = NULL; //pointers to graph nodes

  /* check whether G has at least one vertex */
  if(G_size == 0)
    return;

  /* initialize directional edge queue Q */
  InitQueue((queue_t*) Q, QTYPE_DIRECTIONAL_EDGE);

  /* assign a unique number to EDD D_ij */
  for(i = 0; i < G_size; i++) //for-1
  {
    pTailNode = &(G[i]);
    pHeadNode = pTailNode;

    for(j = 0; j < pTailNode->weight; j++) //for-2
    {
      pHeadNode = pHeadNode->next;

      /* reset edge_node to zeroes */
      memset(&edge_node, 0, sizeof(edge_node));

      /* initialize edge_node */
      strcpy(edge_node.tail_node, pTailNode->vertex);
      strcpy(edge_node.head_node, pHeadNode->vertex);
      //edge_node.eid = ++seq_num;

      /* set tail_gnode and head_gnode to point to the graph node of the tail_node and that of the head_node, 
         respectively, in the adjacency list where tail_node is in the graph node array and head_node is in 
         the neighbor list of the tail_node */
      edge_node.tail_gnode = pTailNode;
      edge_node.head_gnode = pHeadNode;

      edge_node.weight = pHeadNode->weight;
      pEdgeNode = (directional_edge_queue_node_t*)Enqueue((queue_t*) Q, (queue_node_t*)&edge_node);

      /* initialize vehicle movement list */
      InitQueue((queue_t*) &(pEdgeNode->vehicle_movement_list), QTYPE_VEHICLE_MOVEMENT); 
      pEdgeNode->vehicle_movement_list.eid = edge_node.eid; //set up vehicle_movement_list's eid
      pEdgeNode->vehicle_movement_list.enode = pEdgeNode; //set up vehicle_movement_list's enode

      /* initialize convoy list */
      InitQueue((queue_t*) &(pEdgeNode->convoy_list), QTYPE_CONVOY); 
      pEdgeNode->convoy_list.eid = edge_node.eid; //set up convoy_list's eid
      pEdgeNode->convoy_list.enode = pEdgeNode; //set up convoy_list's enode
      pEdgeNode->convoy_list.sequence_number = 0; //set the initial sequence number for the convoy to one


      /* initialize the probability-and-statistics queue for this directed edge */
      InitQueue((queue_t*) &(pEdgeNode->probability_and_statistics_queue), QTYPE_PROBABILITY_AND_STATISTICS);
      pEdgeNode->probability_and_statistics_queue.eid = pEdgeNode->eid;
      pEdgeNode->probability_and_statistics_queue.enode = pEdgeNode;
      strcpy(pEdgeNode->probability_and_statistics_queue.tail_node, pEdgeNode->tail_node);
      strcpy(pEdgeNode->probability_and_statistics_queue.head_node, pEdgeNode->head_node);

      /* initialize ACL convoy information */
      if(param->vehicle_vanet_acl_measurement_flag)
	pEdgeNode->acl_convoy_threshold = param->communication_range/param->vehicle_speed; //the time to disconnect a convoy from the pEdgeNode->tail_node due to the interarrival time greater than the vehicle movement time corresponding to the communication range

    } //end of for-2
  } //end of for-1
}

edge_queue_node_t* FastLookupEdgeQueue(struct_graph_node *G, char *u, char *v, boolean *flip_flag)
{ //return the pointer to edge queue node corresponding to the edge consisting of vertices u and v using the adjacency list of graph G //@NOTE: [03/07/08] the current simulation version supports the fast lookup for real graph Gr.

  edge_queue_node_t *node = NULL; //edge queue node corresponding to the edge consisting of vertices u and v
  int node_id = atoi(u);
  struct_graph_node *ptr = NULL; //pointer to graph node

  if(G == NULL)
  {
    printf("FastLookupEdgeQueue(): G is NULL\n");
    exit(1);
  }

  ptr = &(G[node_id-1]); //pointer to graph node
	
  while(ptr != NULL)
  {
    ptr = ptr->next;
    if(ptr == NULL)
      break;

    if(strcmp(ptr->vertex, v) == 0)
    {
      node = ptr->ptr_edge_node;
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

directional_edge_queue_node_t* FastLookupDirectionalEdgeQueue(struct_graph_node *G, char *u, char *v)
{ //return the pointer to directional edge queue node corresponding to the edge consisting of vertices u and v using the adjacency list of graph G where u is tail node and v is head node

  directional_edge_queue_node_t *node = NULL; //edge queue node corresponding to the directional edge consisting of vertices u and v
  int node_id = atoi(u);
  struct_graph_node *ptr = NULL; //pointer to graph node

  if(G == NULL)
  {
    printf("FastLookupDirectionalEdgeQueue(): G is NULL\n");
    exit(1);
  }

  ptr = &(G[node_id-1]); //pointer to graph node
	
  while(ptr != NULL)
  {
    ptr = ptr->next;
    if(ptr == NULL)
      break;

    if(strcmp(ptr->vertex, v) == 0)
    {
      node = ptr->ptr_directional_edge_node;
      break;
    }
  }

  return node;
}

edge_queue_node_t* LookupEdgeQueue(edge_queue_t *Q, char *u, char *v, boolean *flip_flag)
{ //return the pointer to edge queue node corresponding to the edge consisting of vertices u and v
	edge_queue_node_t *p = NULL; //edge queue node
	edge_queue_node_t *node = NULL; //edge queue node corresponding to the edge consisting of vertices u and v

	if(Q->size == 0)
		return NULL;

	for(p = Q->head.next; p != &(Q->head); p = p->next)
	{
		if(strcmp(p->tail_node, u) == 0 && strcmp(p->head_node, v) == 0)
		{
			*flip_flag = FALSE;
			node = p;
			break;
		}
		else if(strcmp(p->tail_node, v) == 0 && strcmp(p->head_node, u) == 0)
		{
			*flip_flag = TRUE;
			node = p;
			break;		
		}
	};

	return node;
}

directional_edge_queue_node_t* LookupDirectionalEdgeQueue(directional_edge_queue_t *Q, char *u, char *v)
{ //return the pointer to directional edge queue node corresponding to the directional edge consisting of vertices u and v where u is tail node and v is head node
	directional_edge_queue_node_t *p = NULL; //directional edge queue node
	directional_edge_queue_node_t *node = NULL; //directional edge queue node corresponding to the directional edge consisting of vertices u and v

	if(Q->size == 0)
		return NULL;

	for(p = Q->head.next; p != &(Q->head); p = p->next)
	{
		if(strcmp(p->tail_node, u) == 0 && strcmp(p->head_node, v) == 0)
		{
			node = p;
			break;
		}
	};

	return node;
}

int GetEdgeID_MoveType(edge_queue_t *Q, char *u, char *v, MOVE_TYPE *move_type, double *edge_length)
{ //return the ID (i.e., eid) for the edge consisting of vertices u and v along with move type using the edge queue Q
	edge_queue_node_t *p = NULL; //edge queue node
	int eid = 0; //the ID of the edge consisting of vertices u and v

	*move_type = MOVE_UNKNOWN;

	if(Q->size == 0)
		return -1;

	for(p = Q->head.next; p != &(Q->head); p = p->next)
	{
		if(strcmp(p->tail_node, u) == 0 && strcmp(p->head_node, v) == 0)
		{
			*move_type = MOVE_FORWARD;
			*edge_length = p->weight;
			eid = p->eid;
			break;
		}
		else if(strcmp(p->tail_node, v) == 0 && strcmp(p->head_node, u) == 0)
		{
			*move_type = MOVE_BACKWARD;
			*edge_length = p->weight;
			eid = p->eid;
			break;		
		}
	};

	return eid;
}

int FastGetEdgeID_MoveType(struct_graph_node *G, char *u, char *v, MOVE_TYPE *move_type, double *edge_length, directional_edge_queue_node_t **ptr_directional_edge_node_pointer)
{ //return the ID (i.e., eid) for the edge consisting of vertices u and v along with move type using real graph G along with the pointer to the directed edge in the digraph

  edge_queue_node_t *node = NULL; //edge queue node corresponding to the edge consisting of vertices u and v
  int node_id = atoi(u);
  struct_graph_node *ptr = NULL; //pointer to graph node
  int eid = 0; //the ID of the edge consisting of vertices u and v
  *move_type = MOVE_UNKNOWN;

  if(G == NULL)
  {
    printf("FastGetEdgeID_MoveType(): G is NULL\n");
    return -1;
  }

  ptr = &(G[node_id-1]); //pointer to graph node
	
  while(ptr != NULL)
  {
    ptr = ptr->next;
    if(ptr == NULL)
      break;

    if(strcmp(ptr->vertex, v) == 0)
    {
      node = ptr->ptr_edge_node;
      *ptr_directional_edge_node_pointer = ptr->ptr_directional_edge_node; 
      break;
    }
  }

  if(node == NULL)
    return -1;

  if(strcmp(node->tail_node, u) == 0 && strcmp(node->head_node, v) == 0)
  {
    *move_type = MOVE_FORWARD;
  }
  else if(strcmp(node->tail_node, v) == 0 && strcmp(node->head_node, u) == 0)
  {
    *move_type = MOVE_BACKWARD;
  }

  *edge_length = node->weight;
  eid = node->eid;

  return eid;
}

edge_queue_node_t* GetEdgeNodeByEID(edge_queue_t *Q, int eid)
{ //return the edge queue node corresponding to eid; the eid starts from 1.
	edge_queue_node_t* p = NULL;
	int i; //for-loop index
/*
	if(eid > Q->size)
	{
		printf("GetEdgeNodeByEID(): Error: eid(%d) > Q->size(%d)\n", eid, Q->size);
		exit(1);
	}
*/
	for(i = 0, p = &(Q->head); i < Q->size; i++)
	{
		p = p->next;
		if(p->eid == eid)
			break;
	}

	if(p == &(Q->head) || i == Q->size)
		return NULL;
	else
		return p;
}

void ReplaceSubedgeWithSubdivision(subedge_queue_node_t **subedge_in_schedule_table_entry, subedge_queue_node_t *subedge, subedge_queue_t *subedge_queue)
{ //replace the subedge with the subdivision of the subedge that is the subedge queue and update schedule table entries to let them point to the corresponding subedges

	subedge_queue_t *Q = &((*subedge_in_schedule_table_entry)->edge_queue_entry->subedge_list); //pointer to subedge_list including the subedge corresponding to *subedge_in_schedule_table_entry; edge_queue_entry is the pointer to the edge queue entry containing this subedge list for this subedge queue node.

	subedge_queue_node_t *prev = NULL, *next = NULL; //pointer to the previous subedge of subedge and pointer to the next subedge of subedge
	int eid = subedge->eid; //edge id of the subedge replaced with its subdivision; NOTE that subedge's eid is equal to the eid of the schedule table entry pointing to the subedge
	int i = 0; //index for for-loop
	subedge_queue_node_t *pSubedgeNode = NULL; //pointer to the subedge in subedge_queue
	schedule_table_node_t *pTableNode = NULL; //pointer to a schedule table node

	/* add the new subedges to queue Q:
	NOTE: update pTableNode's subedge to let it point to the correct subedge node in the subedge list of the edge in Er, because pTableNode with the same eid does not have the correct pointer to the subedge wiwth the same eid at this point. */
	pSubedgeNode = &(subedge_queue->head);
	for(i = 0; i < subedge_queue->size; i++)
	{
		pSubedgeNode = pSubedgeNode->next;

		InsertSubedgeAfterEID_With_TableEntryUpdate(Q, eid, pSubedgeNode);
		//insert a new subedge node after the subedge of eid in subedge queue Q along with the update of the table entry to let it point to the correct the pointer to the subedge corresponding to the subedge in the subedge list in the physical edge.

		eid = pSubedgeNode->eid;
	}

	/* //=> Wrong implementation!! Though the previsous implementation works, it updates other table entries alreay having the correct pointer to the subedge node in Q
	pSubedgeNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pSubedgeNode = pSubedgeNode->next;
		pTableNode = pSubedgeNode->schedule_table_entry;
		pTableNode->subedge = pSubedgeNode; //update pTableNode->subedge with pSubedgeNode
	}
	*/

	/* delete the original subedge from Q that will be replaced with its subdivision */
	DeleteSubedgeWithEID(Q, subedge->eid);

	/* update the orders of subedges in the edge */
	pSubedgeNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pSubedgeNode = pSubedgeNode->next;
		pSubedgeNode->order = i;
	}

	/* update the pointer to the starting subedge of the subdivision in pTableNode */
	eid = subedge_queue->head.next->eid; //eid is the edge id of the starting subedge in the subdivision
	pSubedgeNode = GetSubedgeNodeByEID(Q, eid); //return the subedge queue node corresponding to eid; the eid starts from 1.	
	*subedge_in_schedule_table_entry = pSubedgeNode; //update schedule_table_entry's subedge to point to the first subedge in the subdivision of the original subedge; though *subedge_in_schedule_table_entry points to the leading subedge of the subdivision, *subedge_in_schedule_table_entry will be deleted just after this function returns since the subdivision will replace it.
}

subedge_queue_node_t* InsertSubedgeAfterEID_With_TableEntryUpdate(subedge_queue_t *Q, int eid, subedge_queue_node_t *node)
{ //insert a new subedge node after the subedge of eid in subedge queue Q along with the update of the table entry to let it point to the correct the pointer to the subedge corresponding to the subedge in the subedge list in the physical edge.

  subedge_queue_node_t *ptr_actual_subedge = NULL; //pointer to the subedge actually inserted in queue Q
  schedule_table_node_t *pTableNode = NULL; //pointer to a schedule table node

  ptr_actual_subedge = InsertSubedgeAfterEID(Q, eid, node);
  //insert a new subedge node after the subedge of eid in subedge queue Q, returning the acutial pointer to the subedge added to Q
		
  /* update the table node's subedge to point to the actual pointer to the subedge pointed by ptr_actual_subedge */
  pTableNode = ptr_actual_subedge->schedule_table_entry;
  pTableNode->subedge = ptr_actual_subedge; //let pTableNode->subedge point to the subedge with the same eid

  return ptr_actual_subedge;
}

subedge_queue_node_t* InsertSubedgeAfterEID(subedge_queue_t *Q, int eid, subedge_queue_node_t *node)
{ //insert a new subedge node after the subedge of eid in subedge queue Q
	subedge_queue_node_t *p = NULL;
	subedge_queue_node_t *pSubedgeNode = NULL;
	int i;

	assert_memory(Q);
	assert_memory(node);
	
	p = (subedge_queue_node_t*) MakeQueueNode(Q->type, (queue_node_t*)node);
        //make a queue node corresponding to queue type and copy the data of node into its data portion

	/* insert a new subedge node after the subedge node of eid in the queue */
	pSubedgeNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pSubedgeNode = pSubedgeNode->next;
		if(pSubedgeNode->eid == eid)
		{
			p->next = pSubedgeNode->next;
			p->prev = pSubedgeNode;
			pSubedgeNode->next->prev = p;
			pSubedgeNode->next = p;
			break;
		}
	}

	if(i == Q->size)
	{
		printf("EnqueueSubedgeAfterEID(): there is no subedge corresponding to EID=%d\n", eid);
		exit(1);
	}

	Q->size++;

	return p;
}

void DeleteSubedgeWithEID(subedge_queue_t *Q, int eid)
{ //delete the subedge node of eid from subedge queue Q
	subedge_queue_node_t *p = NULL;
	subedge_queue_node_t *pSubedgeNode = NULL;
	int i;

	assert_memory(Q);

	if(Q->size == 0)
		return;

	/* position the subedge node of eid in the queue */
	pSubedgeNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pSubedgeNode = pSubedgeNode->next;
		if(pSubedgeNode->eid == eid)
		{
			p = pSubedgeNode->prev; //p points to the previous node of the node pointed by pSubedgeNode;

			/* link the node before pSubedgeNode's node and the node after pSubedgeNode's node each other */
			p->next = pSubedgeNode->next;
			pSubedgeNode->next->prev = p;
			break;
		}
	}

	if(i == Q->size)
	{
		printf("DeleteSubedgeWithEID(): there is no subedge corresponding to EID=%d\n", eid);
		exit(1);
	}

	Q->size--;

	free(pSubedgeNode);
}

subedge_queue_node_t* GetSubedgeNodeByEID(subedge_queue_t *Q, int eid)
{ //return the subedge queue node corresponding to eid; the eid starts from 1.
	subedge_queue_node_t* p = NULL;
	int i; //for-loop index

	for(i = 0, p = &(Q->head); i < Q->size; i++)
	{
		p = p->next;
		if(p->eid == eid)
			break;
	}

	if(p == &(Q->head) || i == Q->size)
		return NULL;
	else
		return p;
}

sensor_queue_node_t* GetSensorQueueNodeJustBeforeVirtualOffset(sensor_queue_t *Q, double offset)
{ //return the sensor queue node just before or on the given offset in Gv (called virtual offset) in sensor queue Q.
	sensor_queue_node_t* p = NULL;
	int size = Q->size; //size of queue Q, i.e., the number of valid queue nodes
	//int i; //for-loop index

	if(Q->size == 0)
	  return NULL;

	if(offset < 0 || offset > Q->head.prev->info.pos_in_Gv.offset)
	{
	  //printf("GetSensorQueueNodeJustBeforeVirtualOffset(): Error: offset(%f) is out of the valid range [%f,%f])\n", (float)offset, 0, (float)Q->head.prev->info.pos_in_Gv.offset);
	  //exit(1);
	  return NULL;
	}

	p = Q->head.next;
	while(p->info.pos_in_Gv.offset <= offset && p != &(Q->head))
	{
	  p = p->next;
	}

	p = p->prev; //go back to the previous sensor node since it has the closest offset to the given offset and is smaller than or equal to the given offset

	if(p == &(Q->head)) //there is no sensor queue node just before the given offset, since the offset is equal to the last sensor's offset
	  return NULL;
	else
	  return p;
}

sensor_queue_node_t* GetSensorQueueNodeJustAfterVirtualOffset(sensor_queue_t *Q, double offset)
{ //return the sensor queue node just after the given offset in Gv (called virtual offset) in sensor queue Q.
	sensor_queue_node_t* p = NULL;
	int size = Q->size; //size of queue Q, i.e., the number of valid queue nodes
	//int i; //for-loop index

	if(Q->size == 0)
	  return NULL;

	if(offset < 0 || offset > Q->head.prev->info.pos_in_Gv.offset)
	{
	  //printf("GetSensorQueueNodeJustAfterVirtualOffset(): Error: offset(%f) is out of the valid range [%f,%f])\n", (float)offset, 0, (float)Q->head.prev->info.pos_in_Gv.offset);
	  //exit(1);
	  return NULL;
	}

	p = Q->head.prev;
	while(p->info.pos_in_Gv.offset >= offset && p != &(Q->head))
	{
	  p = p->prev;
	}

	p = p->next; //go back to the next sensor node since it has the closest offset to the given offset and is greater than or equal to the given offset

	if(p == &(Q->head)) //there is no sensor queue node just after the given offset, since the offset is equal to the last sensor's offset
	  return NULL;
	else
	  return p;
}

void DeleteSensorQueueNodesJustBeforeVirtualOffset(sensor_queue_t *Q, double offset)
{//delete sensor nodes just before offset from the sensor queue Q
        sensor_queue_node_t *p = NULL, *ptr_delete = NULL, *ptr_ahead = NULL; //pointers to sensor queue nodes
	int size = Q->size; //size of queue Q, i.e., the number of valid queue nodes
	//int i; //for-loop index

	if(Q->size == 0)
	  return;

	if(offset < 0 || offset > Q->head.prev->info.pos_in_Gv.offset)
	{
	  //printf("DeleteSensorQueueNodesJustBeforeVirtualOffset(): Error: offset(%f) is out of the valid range [%f,%f])\n", (float)offset, 0, (float)Q->head.prev->info.pos_in_Gv.offset);
	  //exit(1);
	  return;
	}

	p = Q->head.next;
	while(p->info.pos_in_Gv.offset <= offset && p != &(Q->head))
	{
	  ptr_delete = p; //ptr_delete points to the sensor node deleted soon
	  ptr_ahead = p->next; //ptr_ahead points to the next sensor node

	  p = p->next;
	  
	  ptr_delete->prev->next = ptr_ahead;
	  ptr_ahead->prev = ptr_delete->prev;

	  free(ptr_delete);

	  Q->size--; //decreae queue size
	}
}

void DeleteSensorQueueNodesJustAfterVirtualOffset(sensor_queue_t *Q, double offset)
{//delete sensor nodes just after offset from the sensor queue Q
        sensor_queue_node_t *p = NULL, *ptr_delete = NULL, *ptr_before = NULL; //pointers to sensor queue nodes
	int size = Q->size; //size of queue Q, i.e., the number of valid queue nodes
	//int i; //for-loop index

	if(Q->size == 0)
	  return;

	if(offset < 0 || offset > Q->head.prev->info.pos_in_Gv.offset)
	{
	  //printf("DeleteSensorQueueNodesJustAfterVirtualOffset(): Error: offset(%f) is out of the valid range [%f,%f])\n", (float)offset, 0, (float)Q->head.prev->info.pos_in_Gv.offset);
	  //exit(1);
	  return;
	}

	p = Q->head.prev;
	while(p->info.pos_in_Gv.offset >= offset && p != &(Q->head))
	{
	  ptr_delete = p; //ptr_delete points to the sensor node deleted soon
	  ptr_before = p->prev; //ptr_before points to the previous sensor node

	  p = p->prev;
	  
	  ptr_delete->next->prev = ptr_before;
	  ptr_before->next = ptr_delete->next;

	  free(ptr_delete);

	  Q->size--; //decreae queue size
	}
}

void ReplaceSensorQueue(sensor_queue_t *dst_Q, sensor_queue_t *src_Q)
{//replace sensor list dst_Q with sensor list src_Q by letting dst_Q have sensor queue nodes of src_Q without allocating new sensor queue nodes and copying them
  sensor_queue_node_t *dst_head = &(dst_Q->head);
  sensor_queue_node_t *src_head = &(src_Q->head);

  /** delete sensor_list */
  if(dst_Q->size > 0)
  {
    DestroyQueue((queue_t*)dst_Q); //destroy the queue dst_Q, and then initialize it by calling InitQueue() within the function
  }

  if(src_Q->size == 0)
    return;

  /** put the sensor list of src_Q in dst_Q by adjusting pointers to sensor queue nodes */
  dst_head->next = src_head->next;
  src_head->next->prev = dst_head;

  dst_head->prev = src_head->prev;
  src_head->prev->next = dst_head;

  /** set dst_Q->size tp src_Q->size */
  dst_Q->size = src_Q->size;

  /** set src_Q's size to zero and let src_Q.head's pointers point to src_Q.head itself */
  src_Q->size = 0;
  src_head->next = src_head;
  src_head->prev = src_head;
}

int GetHoleEndpointsWithHoleSegment(hole_endpoint_queue_t *H, int eid, hole_endpoint_queue_node_t **left_hole_endpoint, hole_endpoint_queue_node_t **right_hole_endpoint)
{ //get the pointers of two endpoints of the hole segment corresponding to eid 
  hole_endpoint_queue_node_t *first_hole = NULL, *second_hole = NULL; //pointer to hole-endpoint queue nodes
  hole_endpoint_queue_node_t *ptr = NULL; //pointer to hole-endpoint queue node
  int hole_number = 0;

  if(H->size == 0)
  {
    *left_hole_endpoint = NULL;
    *right_hole_endpoint = NULL;
    return hole_number; //no hole-endpoint is found
  }

  /* find the first hole-endpoint */
  ptr = H->head.next;
  while(ptr != &(H->head))
  {
    if(ptr->eid == eid)
    {
      first_hole = ptr;
      hole_number++;
      ptr = ptr->next; //move ptr to next pointer
      break;
    }
    ptr = ptr->next;
  }

  /* find the second hole-endpoint */
  //ptr = H->head.next;
  while(ptr != &(H->head))
  {
    if(ptr->eid == eid)
    {
      second_hole = ptr;
      hole_number++;
      break;
    }
    ptr = ptr->next;
  }

  /* determine which one is left hole-endpoint and which one is right hole-endpoint */
  if(first_hole->offset <= second_hole->offset)
  {
    *left_hole_endpoint = first_hole;
    *right_hole_endpoint = second_hole;
  }
  else
  {
    *left_hole_endpoint = second_hole;
    *right_hole_endpoint = first_hole;
  }

  return hole_number;
}

void DeleteHoleEndpointWithEID(hole_endpoint_queue_t *H, int eid, char *node)
{ //delete the hole endpoint for eid and node from hole endpoint queue H
	hole_endpoint_queue_node_t *p = NULL;
	hole_endpoint_queue_node_t *pHoleNode = NULL;
	int i;

	assert_memory(H);

	if(H->size == 0)
		return;

	/* position the hole endpoint node of eid in the queue */
	pHoleNode = &(H->head);
	for(i = 0; i < H->size; i++)
	{
		pHoleNode = pHoleNode->next;
		if(pHoleNode->eid == eid && strcmp(pHoleNode->vertex, node) == 0)
		{
			p = pHoleNode->prev; //p points to the previous node of the node pointed by pHoleNode;

			/* link the node before pHoleNode's node and the node after pHoleNode's node each other */
			p->next = pHoleNode->next;
			pHoleNode->next->prev = p;
			break;
		}
	}

	if(i == H->size)
	{
		printf("DeleteHoleEndpointWithEID(): there is no hole endpoint corresponding to EID=%d and node=%s\n", eid, node);
		exit(1);
	}

	H->size--;

	free(pHoleNode);
}

void UpdateHoleEndpointEID(hole_endpoint_queue_t *H, int old_eid, char *node, int new_eid)
{ //replace the hole endpoint's eid (i.e., old_eid) with new_eid
	hole_endpoint_queue_node_t *p = NULL;
	hole_endpoint_queue_node_t *pHoleNode = NULL;
	int i;

	assert_memory(H);

	if(H->size == 0)
		return;

	/* position the hole endpoint node of eid in the queue */
	pHoleNode = &(H->head);
	for(i = 0; i < H->size; i++)
	{
		pHoleNode = pHoleNode->next;
		if(pHoleNode->eid == old_eid && strcmp(pHoleNode->vertex, node) == 0)
		{
		        pHoleNode->eid = new_eid; //update the hole endpoint's eid with new_eid
			break;
		}
	}

	if(i == H->size)
	{
		printf("UpdateHoleEndpointEID(): there is no hole endpoint corresponding to EID=%d and node=%s\n", old_eid, node);
		exit(1);
	}
}

double GetPhysicalOffsetOnSubedgeFromSubedgeList(subedge_queue_t *Q, subedge_queue_node_t *pSubedgeNode, double virtual_offset)
{ /* get the physical offset of virtual_offset in the edge in real graph Gr corresponding to the offset on the subedge from the subedge list 
    that contains the subedge in virtual graph Gv */
	subedge_queue_node_t *p = NULL; //pointer to a subedge queue node
	int i;
	double left_endpoint_offset = 0; //cumulated offset of the left end-point in a subedge for the offset in edge queue Er
	double real_offset = -1; //the offset in Gr (called real offset) corresponding to the offset in Gv (called virtual offset)

	p = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		p = p->next;
		if(pSubedgeNode == p)
		{
			real_offset = left_endpoint_offset + virtual_offset;
			break;
		}

		left_endpoint_offset += p->weight;
	}

	return real_offset;
}

void DeleteVehicleWithVID(vehicle_queue_t *Q, int vid)
{ //delete the vehicle node corresponding to vid from vehicle queue Q
	vehicle_queue_node_t *p = NULL;
	vehicle_queue_node_t *pQueueNode = NULL;
	int i;

	assert_memory(Q);

	if(Q->size == 0)
	{
	        printf("DeleteVehicleWithVID(): Error: Q->size is %d\n", Q->size);
	        //exit(1);
		return;
	}

	/* position the vehicle node of vid in the queue */
	pQueueNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pQueueNode = pQueueNode->next;
		if(pQueueNode->vid == vid)
		{
/* 		        p = pQueueNode->prev; //p points to the previous node of the node pointed by pQueueNode; */

/* 			/\* link the node before pQueueNode's node and the node after pQueueuNode's node each other *\/ */
/* 			p->next = pQueueNode->next; */
/* 			pQueueNode->next->prev = p; */

			/* link the node before pQueueNode's node and the node after pQueueuNode's node each other */
			pQueueNode->prev->next = pQueueNode->next;
			pQueueNode->next->prev = pQueueNode->prev;

			break;
		}
	}

	if(i == Q->size)
	{
		printf("DeleteVehicleWithVID(): Error: there is no vehicle node corresponding to VID=%d\n", vid);
		//exit(1);
		return;
	}

	Q->size--;

	free(pQueueNode);
}

void DeleteVehicleMovementWithVID(vehicle_movement_queue_t *Q, int vid)
{ //delete the vehicle movement node corresponding to vid from vehicle movement queue Q
	vehicle_movement_queue_node_t *p = NULL;
	vehicle_movement_queue_node_t *pQueueNode = NULL;
	int i;

	assert_memory(Q);

	if(Q->size == 0)
	{
	        printf("DeleteVehicleMovementWithVID(): Error: Q->size is %d\n", Q->size);
	        //exit(1);
		return;
	}

	/* position the vehicle movement node of vid in the queue */
	pQueueNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pQueueNode = pQueueNode->next;
		if(pQueueNode->vid == vid)
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
		printf("DeleteVehicleMovementWithVID(): Error: there is no vehicle movement node corresponding to VID=%d\n", vid);
		//exit(1);
		return;
	}

	Q->size--;

	free(pQueueNode);
}

void DeleteConvoyWithCID(convoy_queue_t *Q, int cid)
{ //delete the convoy node corresponding to cid from convoy queue Q
	convoy_queue_node_t *p = NULL;
	convoy_queue_node_t *pQueueNode = NULL;
	int i;

	assert_memory(Q);

	if(Q->size == 0)
	{
	        printf("DeleteConvoyWithCID(): Error: Q->size is %d\n", Q->size);
	        //exit(1);
		return;
	}

	/* position the convoy node of cid in the queue */
	pQueueNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pQueueNode = pQueueNode->next;
		if(pQueueNode->cid == cid)
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
		printf("DeleteConvoyWithCID(): Error: there is no convoy node corresponding to CID=%d\n", cid);
		//exit(1);
		return;
	}

	Q->size--;

	/* delete vehicle queue belonging to this convoy */
	if(pQueueNode->vehicle_list.size > 0)
	  DestroyQueue((queue_t*) &(pQueueNode->vehicle_list));

	free(pQueueNode);
}

void MergeTwoConvoys(convoy_queue_t *Q1, convoy_queue_t *Q2)
{ //merge two convoys moving one the same directional edge into one convoy and delete one between the merged convoys from the convoy queue for the directional edge; note that the new convoy's id is equal to one whose cid is smaller and the convoy with the bigger is deleted

}

void SplitConvoy(convoy_queue_t *Q)
{ //split one convoy into two convoys moving one the same directional edge and register these two convoys into the convoy queue for the directional edge; note that one convoy's id the same as the original one's and another convoy's is set to a unique cid

}

convoy_queue_node_t* GetConvoyClosestToVehicle(parameter_t *param, convoy_queue_t *Q, double vehicle_offset)
{ //get the pointer to the convoy node closest to the vehicle within the communication range
  convoy_queue_node_t *pConvoyNode = NULL; //pointer to a convoy node
  boolean flag = FALSE; //flag to indicate whether there exists a convoy within the vehicle's communication range
  int size = Q->size; //queue size
  double distance1 = 0; //distance between the vehicle and the convoy tail
  double distance2 = 0; //distance between the vehicle and the convoy head
  int i = 0; //index for for-loop


  pConvoyNode = &(Q->head);
  for(i = 0; i < size; i++)
  {
    pConvoyNode = pConvoyNode->next;
   
    //distance1 = fabs(pConvoyNode->tail_vehicle->ptr_vehicle_movement_queue_node->offset - vehicle_offset);
    //distance2 = fabs(pConvoyNode->head_vehicle->ptr_vehicle_movement_queue_node->offset - vehicle_offset);
    distance1 = fabs(pConvoyNode->tail_vehicle->current_pos_in_digraph.offset - vehicle_offset);
    distance2 = fabs(pConvoyNode->head_vehicle->current_pos_in_digraph.offset - vehicle_offset);

    if(distance1 <= param->communication_range || distance2 <= param->communication_range)
    { //the convoy is closest to the vehicle while it is within the vehicle's communication range
      flag = TRUE;
      break;
    }
  }

  if(flag)
    return pConvoyNode;
  else
    return NULL;
}

void Set_NewConvoyHead(convoy_queue_node_t *convoy)
{ //find a vehicle placed at the front of the convoy as the convoy's new head
  vehicle_queue_node_t *pVehicleNode = NULL;
  struct_vehicle_t *head_vehicle = NULL;
  double max_offset = -1; //maximum offset to find the physically first vehicle in the convoy that is closest to the head of the directional edge
  int size = convoy->vehicle_list.size;
  int i = 0; //index for for-loop

  if(size == 0)
    return;

  /** find the new head vehicle with the maximum offset */
  pVehicleNode = &(convoy->vehicle_list.head);
  for(i = 0; i < size; i++)
  {
    pVehicleNode = pVehicleNode->next;
    if(pVehicleNode->vnode->current_pos_in_digraph.offset > max_offset)
    {
      max_offset = pVehicleNode->vnode->current_pos_in_digraph.offset;
      head_vehicle = pVehicleNode->vnode;
    }
  }

  /* adjust the convoy head with head_vehicle */
  convoy->head_vehicle = head_vehicle;
}

void Set_NewConvoyTail(convoy_queue_node_t *convoy)
{ //find a vehicle placed at the end of the convoy as the convoy's new tail
  vehicle_queue_node_t *pVehicleNode = NULL;
  struct_vehicle_t *tail_vehicle = NULL;
  double min_offset = INF; //minimum offset to find the physically last vehicle in the convoy that is closest to the tail of the directional edge
  int size = convoy->vehicle_list.size;
  int i = 0; //index for for-loop

  if(size == 0)
    return;

  /** find the new tail vehicle with the minimum offset */
  pVehicleNode = &(convoy->vehicle_list.head);
  for(i = 0; i < size; i++)
  {
    pVehicleNode = pVehicleNode->next;
    if(pVehicleNode->vnode->current_pos_in_digraph.offset < min_offset)
    {
      min_offset = pVehicleNode->vnode->current_pos_in_digraph.offset;
      tail_vehicle = pVehicleNode->vnode;
    }
  }

  /* adjust the convoy tail with tail_vehicle */
  convoy->tail_vehicle = tail_vehicle;
}

void Set_NewConvoyLeader(parameter_t *param, convoy_queue_node_t *convoy)
{ //find a vehicle with minimum EDD as the convoy's new leader
  vehicle_queue_node_t *pVehicleNode = NULL;
  struct_vehicle_t *leader_vehicle = NULL; //pointer to the leader vehicle
  double min_EDD = INF; //minimum EDD
  int size = convoy->vehicle_list.size;
  int i = 0; //index for for-loop

  if(size == 0)
    return;

  /** find the new leader vehicle with the minimum offset */
  pVehicleNode = &(convoy->vehicle_list.head);
  for(i = 0; i < size; i++)
  {
    pVehicleNode = pVehicleNode->next;

    /* According to data_forwarding_mode, update the convoy's leader vehicle if the vehicle's EDD is shorter than the convoy leader's EDD */
    if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
    {
      if(pVehicleNode->vnode->EDD < min_EDD)
      {
        min_EDD = pVehicleNode->vnode->EDD;
        leader_vehicle = pVehicleNode->vnode;
      }
    }
    else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
    {
      if(pVehicleNode->vnode->EDD < min_EDD)
      {
        min_EDD = pVehicleNode->vnode->EDD;
        leader_vehicle = pVehicleNode->vnode;
      }
    }
  }

  /* adjust the convoy leader with leader_vehicle */
  if(leader_vehicle == NULL)
  {
    /** [10/26/09] initialize leader_vehicle with the first vehicle on the convoy for the case where there is no vehicle with EDD less than INF */
    convoy->leader_vehicle = convoy->vehicle_list.head.next->vnode; //pointer to the leader vehicle
#ifdef __DEBUG_LEVEL_SET_NEW_CONVOY_LEADER__
    printf("Set_NewConvoyLeader(): leader_vehicle is NULL; that is, there is no vehicle with EDD less than INF\n");
    //exit(1);
#endif
  }
  else
  {
    convoy->leader_vehicle = leader_vehicle;
  }
}

struct_vehicle_t* Find_Vehicle_Following_Convoy_Head(convoy_queue_node_t *convoy)
{ //find a vehicle just following the convoy head as the convoy's new head
  vehicle_queue_node_t *pVehicleNode = NULL;
  struct_vehicle_t *head_vehicle = NULL;
  struct_vehicle_t *following_vehicle = NULL;
  double max_offset = -1; //maximum offset to find the physically first vehicle in the convoy that is closest to the head of the directional edge
  int size = convoy->vehicle_list.size;
  int i = 0; //index for for-loop

  if(size == 1)
    return NULL;

  /* obtain the pointer to the convoy head */
  head_vehicle = convoy->head_vehicle;

  /** find the new head vehicle with the maximum offset */
  pVehicleNode = &(convoy->vehicle_list.head);
  for(i = 0; i < size; i++)
  {
    pVehicleNode = pVehicleNode->next;
    if(pVehicleNode->vnode != head_vehicle && pVehicleNode->vnode->current_pos_in_digraph.offset > max_offset)
    {
      max_offset = pVehicleNode->vnode->current_pos_in_digraph.offset;
      following_vehicle = pVehicleNode->vnode;
    }
  }

  /* rerurn the following vehicle */
  return following_vehicle;
}

/** Operations for connected network component */
struct_vehicle_t* Find_Following_Vehicle_Within_Communication_Range_On_Directional_Edge(parameter_t *param, struct_vehicle_t *vehicle, int *acl_convoy_vehicle_number)
{ //find the neighboring vehicle following vehicle within the communication range on the directional edge
  vehicle_movement_queue_t *Q = vehicle->ptr_vehicle_movement_queue_node->ptr_queue; //pointer to the vehicle movement queue
  struct_vehicle_t *following_vehicle = NULL; //following vehicle within the predecessor vehicle's communication range
  vehicle_movement_queue_node_t *pMoveNode = NULL, *pPreviousNode = NULL; //pointers to the vehicle movement queue nodes
  double r = param->communication_range; //communication range
  double margin_of_update_time = param->vehicle_step_time; //margin of movement update time since there are vehicles which have not updated their offsets in the simulation, so we need to consider the margin of update time.
  double margin_of_distance = 0; //margin of the distance between two adjacent vehicles due to the discrepancy of the movement update time 
  double vehicle_speed_for_margin = 0; //vehicle speed for margin of update distance
  double d = 0; //distance between two neighboring vehicle
  int size = Q->size; //size of vehicle movement queue
  int i = 0; //index for for-loop
  
  /* sort the vehicle movement queue in the ascending order of offset in the directional edge */
  SortVehicleMovementQueue(Q);

  /* search the vehicle movement node for vehicle->id */
  pMoveNode = &(Q->head);
  for(i = 0; i < size; i++)
  {
    pMoveNode = pMoveNode->next;
    if(vehicle->id == pMoveNode->vid)
      break;
  }

  /* adjust acl_convoy_vehicle_number considering that more than one vehicle have the same offset as with vehicle's */
  *acl_convoy_vehicle_number = i+1;

  /* find the neighboring vehicle following vehicle */
  if(i == 0 || i == size)
	  following_vehicle = NULL;
  else
  {
	  pPreviousNode = pMoveNode->prev;
	  vehicle_speed_for_margin = MAX(pMoveNode->vnode->speed, pPreviousNode->vnode->speed); //determine vehicle speed for margin
      margin_of_distance = margin_of_update_time * vehicle_speed_for_margin; //determine the margin of distance
      d = fabs(pMoveNode->offset - pPreviousNode->offset);
    
	  if(d <= (r + margin_of_distance))
	  //if(d <= r)
	    following_vehicle = pPreviousNode->vnode;
	  else
        following_vehicle = NULL;
  }
  
  return following_vehicle;
}

struct_vehicle_t* Find_Vehicle_Farthest_Towards_Edge_Head_Node_In_Connected_Network_Component(parameter_t *param, struct_vehicle_t *vehicle)
{ /* find the vehicle farthest from vehicle towards the directional edge's head node within the same connected network component (i.e., convoy), assuming that the vehicle movement queue is sorted in the descending order of vehicle's offset. */
  vehicle_movement_queue_t *Q = vehicle->ptr_vehicle_movement_queue_node->ptr_queue; //pointer to the vehicle movement queue
  struct_vehicle_t *farthest_vehicle = NULL; //the vehicle farthest from the vehicle towards the edge head
  vehicle_movement_queue_node_t *pMoveNode = NULL, *pPreviousNode = NULL; //pointers to the vehicle movement queue nodes
  double r = param->communication_range; //communication range
  double margin_of_update_time = param->vehicle_step_time; //margin of movement update time since there are vehicles which have not updated their offsets in the simulation, so we need to consider the margin of update time.
  double margin_of_distance = 0; //margin of the distance between two adjacent vehicles due to the discrepancy of the movement update time 
  double vehicle_speed_for_margin = 0; //vehicle speed for margin of update distance
  double d = 0; //distance between two neighboring vehicle
  int size = Q->size; //size of vehicle movement queue
  int i = 0; //index for for-loop
  
  /* sort the vehicle movement queue in the ascending order of offset in the directional edge */
  SortVehicleMovementQueue(Q);

  /* search the vehicle movement node for vehicle->id */
  pMoveNode = &(Q->head);
  for(i = 0; i < size; i++)
  {
    pMoveNode = pMoveNode->next;
    if(vehicle->id == pMoveNode->vid)
      break;
  }

  /* search the vehicle farthest from vehicle within the same network component */
  size -= i+1; //adjust the size
  for(i = 0; i < size; i++)
  {
    pPreviousNode = pMoveNode;
    pMoveNode = pMoveNode->next;

    vehicle_speed_for_margin = MAX(pMoveNode->vnode->speed, pPreviousNode->vnode->speed); //determine vehicle speed for margin
    margin_of_distance = margin_of_update_time * vehicle_speed_for_margin; //determine the margin of distance
    d = fabs(pMoveNode->offset - pPreviousNode->offset);
    
    if(d > (r + margin_of_distance))
    //if(d > r)
      break;
  }

  farthest_vehicle = pPreviousNode->vnode;
  
  return farthest_vehicle;
}

/** Queue Operations for iTBD */
edge_queue_node_t* Insert_Edge_Into_EdgeQueue(edge_queue_t *Q, struct_graph_node *G, int G_size, char *tail_node, char *head_node, double weight)
{ //[for iTBD] insert a new edge node (tail_node, head_node) into the end of edge queue Q

  edge_queue_node_t edge_node; //edge queue node
  edge_queue_node_t *pEdgeNode = NULL; //pointer to the subedge actually inserted in queue Q
  char *u = tail_node, *v = head_node; //u is tail_node and v is head_node
  boolean flip_flag = FALSE; //flag to see whether u is tail_node or head_node

  pEdgeNode = FastLookupEdgeQueue(G, u, v, &flip_flag);
  if(pEdgeNode == NULL)
  {
    memset(&edge_node, 0, sizeof(edge_node));
    //edge_node.eid = Q->size + 1; //set the edge id
    edge_node.weight = weight; //set the edge's weight
    strcpy(edge_node.tail_node, u);
    strcpy(edge_node.head_node, v);
    pEdgeNode = (edge_queue_node_t*) Enqueue((queue_t*) Q, (queue_node_t*) &edge_node); //enqueue the clone of node into Q and increase Q->size by 1.
  }

  return pEdgeNode;
}

directional_edge_queue_node_t* Insert_DirectionalEdge_Into_DirectionalEdgeQueue(directional_edge_queue_t *Q, struct_graph_node *G, int G_size, char *tail_node, char *head_node, double weight)
{ //[for iTBD] insert a new directional edge node (tail_node, head_node) into the end of directional edge queue Q

  directional_edge_queue_node_t edge_node; //edge queue node
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to the subedge actually inserted in queue Q
  char *u = tail_node, *v = head_node; //u is tail_node and v is head_node
  boolean flip_flag = FALSE; //flag to see whether u is tail_node or head_node

  pEdgeNode = FastLookupDirectionalEdgeQueue(G, u, v);
  if(pEdgeNode == NULL)
  {
    memset(&edge_node, 0, sizeof(edge_node));
    //edge_node.eid = Q->size + 1; //set the edge id
    edge_node.weight = weight; //set the edge's weight
    strcpy(edge_node.tail_node, u);
    strcpy(edge_node.head_node, v);

    /* set tail_gnode and head_gnode to point to the graph node of the tail_node and that of the head_node, 
       respectively, in the adjacency list where tail_node is in the graph node array and head_node is in 
       the neighbor list of the tail_node */
    edge_node.tail_gnode = LookupGraph(G, G_size, u);
    edge_node.head_gnode = GetNeighborGraphNode(G, G_size, u, v); 

    pEdgeNode = (directional_edge_queue_node_t*) Enqueue((queue_t*) Q, (queue_node_t*) &edge_node); //enqueue the clone of node into Q and increase Q->size by 1.
  }

  return pEdgeNode;
}

void DeleteEdgeWithEID(edge_queue_t *Q, int eid)
{ //delete the edge node of eid from edge queue Q
  edge_queue_node_t *p = NULL;
  edge_queue_node_t *pEdgeNode = NULL;
  int i;

  assert_memory(Q);

  if(Q->size == 0)
    return;

  /* position the edge node of eid in the queue */
  pEdgeNode = &(Q->head);
  for(i = 0; i < Q->size; i++)
  {
    pEdgeNode = pEdgeNode->next;
    if(pEdgeNode->eid == eid)
    {
      p = pEdgeNode->prev; //p points to the previous node of the node pointed by pEdgeNode;

      /* link the node before pEdgeNode's node and the node after pEdgeNode's node each other */
      p->next = pEdgeNode->next;
      pEdgeNode->next->prev = p;
      break;
    }
  }

  if(i == Q->size)
  {
    printf("DeleteEdgeWithEID(): there is no edge corresponding to EID=%d\n", eid);
    exit(1);
  }

  Q->size--;

  free(pEdgeNode);
}

void DeleteDirectionalEdgeWithEID(directional_edge_queue_t *Q, int eid)
{ //delete the edge node of eid from directional edge queue Q
  directional_edge_queue_node_t *p = NULL;
  directional_edge_queue_node_t *pEdgeNode = NULL;
  int i;

  assert_memory(Q);

  if(Q->size == 0)
    return;

  /* position the directional edge node of eid in the queue */
  pEdgeNode = &(Q->head);
  for(i = 0; i < Q->size; i++)
  {
    pEdgeNode = pEdgeNode->next;
    if(pEdgeNode->eid == eid)
    {
      p = pEdgeNode->prev; //p points to the previous node of the node pointed by pEdgeNode;

      /* link the node before pEdgeNode's node and the node after pEdgeNode's node each other */
      p->next = pEdgeNode->next;
      pEdgeNode->next->prev = p;
      break;
    }
  }

  if(i == Q->size)
  {
    printf("DeleteDirectionalEdgeWithEID(): there is no directional edge corresponding to EID=%d\n", eid);
    exit(1);
  }

  Q->size--;

  free(pEdgeNode);
}

void FastDeleteEdge(edge_queue_t *Q, char *tail_node, char *head_node, struct_graph_node *G)
{ //delete the edge node of (tail_node, head_node) from edge queue Q using graph G
  edge_queue_node_t *p = NULL;
  edge_queue_node_t *pEdgeNode = NULL;
  boolean flip_flag = FALSE;
  int i;

  assert_memory(Q);

  if(Q->size == 0)
    return;

  /* position the edge node of (tail_node, head_node) in the queue */
  pEdgeNode = FastLookupEdgeQueue(G, tail_node, head_node, &flip_flag);
  //return the pointer to edge queue node corresponding to the edge consisting of vertices tail_node and head_node using the adjacency list of graph G

  /* link the node before pEdgeNode's node and the node after pEdgeNode's node each other */
  p = pEdgeNode->prev; //p points to the previous node of the node pointed by pEdgeNode;
  p->next = pEdgeNode->next;
  pEdgeNode->next->prev = p;

  Q->size--;

  free(pEdgeNode);
}

void FastDeleteDirectionalEdge(directional_edge_queue_t *Q, char *tail_node, char *head_node, struct_graph_node *G)
{ //delete the directional edge node of (tail_node, head_node) from directional edge queue Q using graph G
  directional_edge_queue_node_t *p = NULL;
  directional_edge_queue_node_t *pEdgeNode = NULL;
  int i;

  assert_memory(Q);

  if(Q->size == 0)
    return;

  /* position the edge node of (tail_node, head_node) in the queue */
  pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
  //return the pointer to edge queue node corresponding to the edge consisting of vertices tail_node and head_node using the adjacency list of graph G

  /* link the node before pEdgeNode's node and the node after pEdgeNode's node each other */
  p = pEdgeNode->prev; //p points to the previous node of the node pointed by pEdgeNode;
  p->next = pEdgeNode->next;
  pEdgeNode->next->prev = p;

  Q->size--;

  free(pEdgeNode);
}

destination_vehicle_queue_node_t* GetDestinationVehicleByVID(destination_vehicle_queue_t *Q, int vid)
{ //return the destination vehicle queue node corresponding to vid; the vid is greater than  1.
  destination_vehicle_queue_node_t* p = NULL;
  int i; //for-loop index

  for(i = 0, p = &(Q->head); i < Q->size; i++)
  {
    p = p->next;
    if(p->vid == vid)
      break;
  }

  //if(p == &(Q->head))
  if(p == &(Q->head) || i == Q->size)
    return NULL;
  else
    return p;
}

destination_vehicle_queue_node_t* GetFirstDestinationVehicle(destination_vehicle_queue_t *Q)
{ //return the first destination vehicle queue node in queue Q
  destination_vehicle_queue_node_t* p = NULL;
  int i; //for-loop index

  if(Q->size > 0)
    p = Q->head.next;

  return p;
}

boolean Is_Destination_Vehicle(destination_vehicle_queue_t *Q, int vid)
{ //check whether or not vid is a destination vehicle with destination vehicle queue Q
  boolean result = FALSE;
  destination_vehicle_queue_node_t *pQueueNode = NULL;

  /* try to obtain the destination vehicle queue node corresponding to vid */
  pQueueNode = GetDestinationVehicleByVID(Q, vid);
  
  if(pQueueNode != NULL)
    result = TRUE;
  else
    result = FALSE;
  
  return result;
}

/** Access Point Queue Operations */
access_point_queue_node_t* GetAccessPointByID(access_point_queue_t *Q, int id)
{ //return the access point structure corresponding to id; the id is greater than 0.
  access_point_queue_node_t* p = NULL;
  int i; //for-loop index

  for(i = 0, p = &(Q->head); i < Q->size; i++)
  {
    p = p->next;
    if(p->id == id)
      break;
  }

  if(p == &(Q->head) || i == Q->size)
    return NULL;
  else
    return p;
}

access_point_queue_node_t* GetFirstAccessPoint(access_point_queue_t *Q)
{ //return the first access point queue node in queue Q
  access_point_queue_node_t* p = NULL;
  int i; //for-loop index

  if(Q->size > 0)
    p = Q->head.next;

  return p;
}

access_point_queue_node_t* GetAccessPointByVertex(access_point_queue_t *Q, char *vertex)
{ //return the access point structure corresponding to vertex
  access_point_queue_node_t* p = NULL;
  int i; //for-loop index

  for(i = 0, p = &(Q->head); i < Q->size; i++)
  {
    p = p->next;
    if(strcmp(p->vertex, vertex) == 0)
      break;
  }

  if(p == &(Q->head) || i == Q->size)
    return NULL;
  else
    return p;
}

void Install_VehicleTrajectory_Into_Packet(parameter_t *param, double current_time, struct_graph_node *G, int G_size, struct_vehicle_t *vehicle, packet_queue_node_t *packet)
{ //set up packet's vehicle trajectory with vehicle's trajectory 
  vehicle_trajectory_queue_node_t trajectory_qnode; //trajectory queue node
  double exposure_degree = param->vehicle_vanet_vehicle_trajectory_exposure_degree; //exposure degree of vehicle trajectory
  int i = 0; //for-loop index
  struct_path_node *pPathNode = NULL; //pointer to a path node
  struct_path_node *pCurrentPathNode = vehicle->path_ptr; //pointer to the tail node of the edge where vehicle is moving
  double current_offset = vehicle->current_pos_in_digraph.offset; //offset of the directional edge where vehicle is moving
  char tail_node[NAME_SIZE]; //tail node of a directed edge
  char head_node[NAME_SIZE]; //head node of a directed edge
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to a directed edge
  double accumulated_path_length = 0; //accumulated path length from the trajectory starting point up to the head node of the edge
  struct_coordinate1_t start_point, end_point, vehicle_point; //Euclidean coordinates

  /** check whether vehicle_trajectory is empty or not; if not, destroy it first */
  if(packet->vehicle_trajectory.size > 0)
    DestroyQueue((queue_t*) &(packet->vehicle_trajectory));

  /** store vehicle's current Graph position and Euclidean position along with the position registration time in order to compare it with target_point */
  packet->vehicle_trajectory.vehicle_pos_register_time = current_time;
  memcpy(&(packet->vehicle_trajectory.vehicle_graph_pos), &(vehicle->current_pos_in_digraph), sizeof(vehicle->current_pos_in_digraph));
  memcpy(&(packet->vehicle_trajectory.vehicle_euclidean_pos), &(vehicle->current_pos), sizeof(vehicle->current_pos));

  /** set up vehicle trajectory according to vehicle trajectory type */
  if(param->vehicle_vanet_vehicle_trajectory_type == VANET_VEHICLE_TRAJECTORY_FULL)
  {
    /* set up the packet's trajectory queue */
    packet->vehicle_trajectory.trajectory_type = param->vehicle_vanet_vehicle_trajectory_type;
    packet->vehicle_trajectory.vehicle_speed = vehicle->speed;
    packet->vehicle_trajectory.vehicle_speed_standard_deviation = vehicle->speed_standard_deviation;
    packet->vehicle_trajectory.vnode = vehicle;
    packet->vehicle_trajectory.target_point_id = packet->target_point_id;
    packet->vehicle_trajectory.target_point_gnode = LookupGraph_By_NodeID(G, G_size, packet->target_point_id);

    /* set up tail_node and head_node for the current directed edge of the vehicle */
    strcpy(tail_node, pCurrentPathNode->vertex);
    strcpy(head_node, pCurrentPathNode->next->vertex); //Note: pCurrentPathNode->next->vertex is at most the end of the path; it is not necessary to check whether head_node is null string or not.

    pPathNode = pCurrentPathNode;
    do
    {
      /* initialize trajectory_qnode */
      memset(&trajectory_qnode, 0, sizeof(trajectory_qnode));

      if(pPathNode == pCurrentPathNode)
      { //process the first edge as exceptional case
	/* set up the graph position on the road network graph */
	memcpy(&trajectory_qnode.graph_pos, &(vehicle->current_pos_in_digraph), sizeof(trajectory_qnode.graph_pos));

	/* set up the Euclidean position of tail_node */
	pEdgeNode = vehicle->current_pos_in_digraph.enode;

	/*@for debugging */
	//printf("Install_VehicleTrajectory_Into_Packet(): edge (%s,%s)\n", pEdgeNode->tail_node, pEdgeNode->head_node);
	/*****************/

	memcpy(&(start_point), &(pEdgeNode->tail_gnode->coordinate), sizeof(start_point));
	memcpy(&(end_point), &(pEdgeNode->head_gnode->gnode->coordinate), sizeof(end_point));
	//Note that for head_node we need to head_gnode->gnode to access the graph node on the graph node array for coordinate the information of head_node

	get_position_on_linear_curve_for_offset(current_offset, &start_point, &end_point, &vehicle_point); //get the vehicle's Euclidean coordinate for offset between start_point and end_point where the point is offset away from start_point

	memcpy(&(trajectory_qnode.euclidean_pos), &(vehicle_point), sizeof(vehicle_point));

	/* compute the estimated arrivel time at tail_node */
	trajectory_qnode.arrival_time = current_time;

	/* compute accumulated path length up to the head node */
	accumulated_path_length = vehicle->edge_length - current_offset;
      }
      else
      { //process the other edges on the trajectory
	/* obtain the directed edge of (tail_node, head_node) */
	pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
	if(pEdgeNode == NULL)
	{
	  printf("Install_VehicleTrajectory_Into_Packet(): Error: pEdgeNode is NULL\n");
	  exit(1);
	}
	
	/*@for debugging */
	//printf("Install_VehicleTrajectory_Into_Packet(): edge (%s,%s)\n", pEdgeNode->tail_node, pEdgeNode->head_node);
	/*****************/

	/* set up the graph position on the road network graph */
	trajectory_qnode.graph_pos.offset = 0;
	trajectory_qnode.graph_pos.eid = pEdgeNode->eid;
	trajectory_qnode.graph_pos.enode = pEdgeNode;

	/* set up the Euclidean position of tail_node */
	memcpy(&(trajectory_qnode.euclidean_pos), &(pEdgeNode->tail_gnode->coordinate), sizeof(trajectory_qnode.euclidean_pos));

	/* compute the estimated arrivel time at tail_node */
	trajectory_qnode.arrival_time = current_time + accumulated_path_length/vehicle->speed;

	/* compute accumulated path length up to the head node */
	accumulated_path_length += pEdgeNode->weight;
      }

      /* enqueue trajectory queue node into vehicle trajectory queue */
      Enqueue((queue_t*) &(packet->vehicle_trajectory), (queue_node_t*) &trajectory_qnode);      

      /* move to the next path node */
      pPathNode = pPathNode->next;

      /* check whether pPathNode points to the head of the path_list; if so, let pPathNode point to the next one */
      if(pPathNode == vehicle->path_list)
	pPathNode = pPathNode->next;

      /* update tail_node and head_node for the next directed edge on the vehicle trajectory */
      strcpy(tail_node, head_node);

      if(pPathNode->next == vehicle->path_list)
      {
        strcpy(head_node, pPathNode->next->next->vertex);

        /* In the case where tail_node is the same as head_node,  */
        if(strcmp(tail_node, head_node) == 0)
	{
	  pPathNode = pPathNode->next->next; //let pPathNode point to path_node for head_node
	  strcpy(head_node, pPathNode->next->vertex);
	}
      }
      else
      {
	strcpy(head_node, pPathNode->next->vertex);
      }

      /* check whether pPathNode points to the last node on the vehicle trajectory
         Note: the packet's vehicle trajectory has its end point as the previous graph node of the trajectory's start point */
      if(pPathNode == pCurrentPathNode)
	break;
    } while(1);
  }
  else if(param->vehicle_vanet_vehicle_trajectory_type == VANET_VEHICLE_TRAJECTORY_PARTIAL)
  {
      /** TBD (To Be Done) */
  }

}

boolean Install_PacketTrajectory_Into_Packet(parameter_t *param, double current_time, struct_graph_node *G, int G_size, char *packet_src_vertex, char *packet_dst_vertex, packet_queue_node_t *packet)
{ //set up packet's packet trajectory to its destination vertex with the shortest path from packet_src_vertex to packet_dst_vertex according to param->vehicle_vanet_target_point_selection_type, such as the stationary-node-based static forwarding or dynamic forwarding
    boolean result = FALSE;

    if((param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_STATIC_FORWARDING) || (param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_PARTIALLY_DYNAMIC_FORWARDING) || (param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_END_INTERSECTION) || (param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_RANDOM_INTERSECTION))
    { //For the stationary-node-based static forwarding or the stationary-node-based partially dynamic forwarding
        result = Install_FullPacketTrajectory_Into_Packet(param, current_time, G, G_size, packet_src_vertex, packet_dst_vertex, packet);
    }
    else if(param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_FULLY_DYNAMIC_FORWARDING)
    { //For the stationary-node-based fully dynamic forwarding
        result = Install_PartialPacketTrajectory_Into_Packet(param, current_time, G, G_size, packet_src_vertex, packet_dst_vertex, packet);
    }
    else
    {
        printf("Install_PacketTrajectory_Into_Packet(): param->vehicle_vanet_target_point_selection_type(%d) is not supported!\n", param->vehicle_vanet_target_point_selection_type);
        exit(1);
    }

    return result;
}

boolean Install_FullPacketTrajectory_Into_Packet(parameter_t *param, double current_time, struct_graph_node *G, int G_size, char *packet_src_vertex, char *packet_dst_vertex, packet_queue_node_t *packet)
{ //set up packet's full packet trajectory to its destination vertex with the shortest path from packet_src_vertex to packet_dst_vertex under the stationary-node-based static forwarding 					
  boolean result = FALSE;
  boolean packet_earlier_arrival_flag = FALSE; //flag to indicate whether the packet has arrived earlier than the destination vehicle at a target point on the destination vehicle's trajectory
  packet_trajectory_queue_node_t trajectory_qnode; //packet trajectory queue node
  int src = atoi(packet_src_vertex); //source vertex's id
  int dst = atoi(packet_dst_vertex); //destination vertex's id
  int new_dst = 0; //new destination vertex's id in the case where src and dst are the same and src is on the destination vehicle's trajectory
  int i = 0, j = 0; //for-loop indices
  int **M = NULL; //predecessor matrix for all-pairs shortest paths in terms of EDD or EDD_VAR in real graph Gr
  int current_hop_node = 0; //current-hop node towards src

  /** check whether packet_trajectory is empty or not; if not, destroy it first */
  if(packet->packet_trajectory.size > 0)
    DestroyQueue((queue_t*) &(packet->packet_trajectory));

  /** check the validity of src and dst */
  if(src > G_size || dst > G_size)
  {
    printf("Install_FullPacketTrajectory_Into_Packet(): src(%d) > G_size(%d) or dst(%d) > G_size(%d)\n", src, G_size, dst, G_size);
    exit(1);
  }
  else if(src == dst)
  {
    /* check whether src is on the destination vehicle's trajectory or not */
    if(Is_Vertex_On_VehicleTrajectory_With_New_Destination(param, current_time, packet, src, &new_dst, &packet_earlier_arrival_flag))
    { //if src is on the destination vehicle's trajectory, we choose an adjacent vertex closer to the destination vehicle's current position
      dst = new_dst;   
    }
    else
    {
      printf("Install_FullPacketTrajectory_Into_Packet(): src(%d) is equal to dst(%d) and we cannot install a new target point\n", src, dst);
      return result;
    }
  }
  
  /** Compute the predecessor matrix M of the EDD (or EDD_SD) shortest path in G */
  M = param->vanet_table.Mr_edd;

  /** construct the packet trajectory from src to dst */
  i = src-1; //node i corresponds to index i-1.
  j = dst-1;

#ifdef __DEBUG_LEVEL_PACKET_TRAJECTORY_CONSTRUCTION___
  printf("%d <- ", j+1);
#endif

  /* enqueue trajectory queue node into packet trajectory queue */
  memset(&trajectory_qnode, 0, sizeof(trajectory_qnode));
  trajectory_qnode.intersection_id = j+1;
  trajectory_qnode.gnode = &(G[j]);
  memcpy(&(trajectory_qnode.euclidean_pos), &(G[j].coordinate), sizeof(G[j].coordinate)); 
  Enqueue((queue_t*) &(packet->packet_trajectory), (queue_node_t*) &trajectory_qnode);

  /* get the previous node towards src */
  j = M[i][j];

  /* check whether j is valid or not */
  if(j == NIL)
  {
	printf("Install_FullPacketTrajectory_Into_Packet(): there is no next hop for dst(%d) towards src(%d)\n", dst, src);
	exit(1);
  }

  while(j != i)
  {
#ifdef __DEBUG_LEVEL_PACKET_TRAJECTORY_CONSTRUCTION___
    printf("%d <- ", j+1);
#endif

    /* enqueue trajectory queue node into packet trajectory queue */
    memset(&trajectory_qnode, 0, sizeof(trajectory_qnode));
    trajectory_qnode.intersection_id = j+1;
    trajectory_qnode.gnode = &(G[j]);
    memcpy(&(trajectory_qnode.euclidean_pos), &(G[j].coordinate), sizeof(G[j].coordinate)); 
    Enqueue((queue_t*) &(packet->packet_trajectory), (queue_node_t*) &trajectory_qnode);

    /* get the previous node towards src */
	current_hop_node = j+1; //current-hop node towards src
    j = M[i][j];

  	/* check whether j is valid or not */
	  if(j == NIL)
  	{
		printf("Install_FullPacketTrajectory_Into_Packet(): there is no next hop for current_hop_node(%d) towards src(%d)\n", current_hop_node, src);
		exit(1);
  	}
  }

#ifdef __DEBUG_LEVEL_PACKET_TRAJECTORY_CONSTRUCTION___
  printf("%d\n", j+1);
#endif

  /* enqueue trajectory queue node into packet trajectory queue */
  memset(&trajectory_qnode, 0, sizeof(trajectory_qnode));
  trajectory_qnode.intersection_id = j+1;
  trajectory_qnode.gnode = &(G[j]);
  memcpy(&(trajectory_qnode.euclidean_pos), &(G[j].coordinate), sizeof(G[j].coordinate)); 
  Enqueue((queue_t*) &(packet->packet_trajectory), (queue_node_t*) &trajectory_qnode);
  /** check whether the packet trajectory has a vaild length */
  if(packet->packet_trajectory.size <= 1)
  {
      printf("Install_FullPacketTrajectory_Into_Packet(): packet_trajectory.size(%d) must be at least 2\n", packet->packet_trajectory.size);
      exit(1);
  }

  /** reverse the order of the queue to make the trajectory be the path from src to dst */
  ReversePacketTrajectoryQueue(&(packet->packet_trajectory), G, G_size);

  /** set the current packet position to the first intersection on the packet trajectory */
  packet->packet_trajectory.current_packet_position = packet->packet_trajectory.head.next;
  packet->packet_trajectory.current_packet_intersection_id = packet->packet_trajectory.head.next->intersection_id;
  packet->packet_trajectory.order = 0;

  /** set or adjust the target point with the packet destination id, i.e., dst */
  packet->target_point_id = dst;

  result = TRUE;
  return result;
}

boolean Install_PartialPacketTrajectory_Into_Packet(parameter_t *param, double current_time, struct_graph_node *G, int G_size, char *packet_src_vertex, char *packet_dst_vertex, packet_queue_node_t *packet)
{ //set up packet's partial packet trajectory (i.e., the current hop and the next hop) to its destination vertex with the shortest path from packet_src_vertex to packet_dst_vertex under the stationary-node-based dynamic forwarding 				
  boolean result = FALSE;
  boolean packet_earlier_arrival_flag = FALSE; //flag to indicate whether the packet has arrived earlier than the destination vehicle at a target point on the destination vehicle's trajectory
  packet_trajectory_queue_t packet_trajectory; //packet trajectory queue
  packet_trajectory_queue_node_t trajectory_qnode; //packet trajectory queue node
  packet_trajectory_queue_node_t *ptr; //pointer to a packet trajectory queue node
  int src = atoi(packet_src_vertex); //source vertex's id
  int dst = atoi(packet_dst_vertex); //destination vertex's id
  int new_dst = 0; //new destination vertex's id in the case where src and dst are the same and src is on the destination vehicle's trajectory
  int i = 0, j = 0; //for-loop indices
  int **M = NULL; //predecessor matrix for all-pairs shortest paths in terms of EDD or EDD_VAR in real graph Gr
  int current_hop_node = 0; //current-hop node towards src

  /*@ for debugging */
  //if(current_time > 8353 && packet->id == 170)
  //    printf("Install_PartialPacketTrajectory_Into_Packet(): for time=%.2f, packet->id=%d\n", (float)current_time, packet->id);
  /******************/

  /** check whether packet_trajectory is empty or not; if not, destroy it first */
  if(packet->packet_trajectory.size > 0)
    DestroyQueue((queue_t*) &(packet->packet_trajectory));

  /** check the validity of src and dst */
  if(src > G_size || dst > G_size)
  {
    printf("Install_PartialPacketTrajectory_Into_Packet(): src(%d) > G_size(%d) or dst(%d) > G_size(%d)\n", src, G_size, dst, G_size);
    exit(1);
  }
  else if(src == dst)
  {
    /* check whether src is on the destination vehicle's trajectory or not */
    if(Is_Vertex_On_VehicleTrajectory_With_New_Destination(param, current_time, packet, src, &new_dst, &packet_earlier_arrival_flag))
    { //if src is on the destination vehicle's trajectory, we choose an adjacent vertex closer to the destination vehicle's current position
      dst = new_dst;   
    }
    else
    {
      printf("Install_PartialPacketTrajectory_Into_Packet(): src(%d) is equal to dst(%d) and we cannot install a new target point\n", src, dst);
      return result;
    }
  }

  /** initialize packet_trajectory queue for the packet trajectory */
  InitQueue((queue_t*) &packet_trajectory, QTYPE_PACKET_TRAJECTORY);                
  
  /** Compute the predecessor matrix M of the EDD (or EDD_SD) shortest path in G */
  M = param->vanet_table.Mr_edd;

  /** construct the packet trajectory from src to dst */
  i = src-1; //node i corresponds to index i-1.
  j = dst-1;

#ifdef __DEBUG_LEVEL_PACKET_TRAJECTORY_CONSTRUCTION___
  printf("%d <- ", j+1);
#endif

  /* enqueue trajectory queue node into packet trajectory queue */
  memset(&trajectory_qnode, 0, sizeof(trajectory_qnode));
  trajectory_qnode.intersection_id = j+1;
  trajectory_qnode.gnode = &(G[j]);
  memcpy(&(trajectory_qnode.euclidean_pos), &(G[j].coordinate), sizeof(G[j].coordinate)); 
  Enqueue((queue_t*) &(packet_trajectory), (queue_node_t*) &trajectory_qnode);

  /* get the previous node towards src */
  j = M[i][j];

  /* check whether j is valid or not */
  if(j == NIL)
  {
	printf("Install_PartialPacketTrajectory_Into_Packet(): there is no next hop for dst(%d) towards src(%d)\n", dst, src);
	exit(1);
  }

  while(j != i)
  {
#ifdef __DEBUG_LEVEL_PACKET_TRAJECTORY_CONSTRUCTION___
    printf("%d <- ", j+1);
#endif

    /* enqueue trajectory queue node into packet trajectory queue */
    memset(&trajectory_qnode, 0, sizeof(trajectory_qnode));
    trajectory_qnode.intersection_id = j+1;
    trajectory_qnode.gnode = &(G[j]);
    memcpy(&(trajectory_qnode.euclidean_pos), &(G[j].coordinate), sizeof(G[j].coordinate)); 
    Enqueue((queue_t*) &(packet_trajectory), (queue_node_t*) &trajectory_qnode);

    /* get the previous node towards src */
	current_hop_node = j+1; //current-hop node towards src
    j = M[i][j];

  	/* check whether j is valid or not */
	  if(j == NIL)
  	{
		printf("Install_PartialPacketTrajectory_Into_Packet(): there is no next hop for current_hop_node(%d) towards src(%d)\n", current_hop_node, src);
		exit(1);
  	}
  }

#ifdef __DEBUG_LEVEL_PACKET_TRAJECTORY_CONSTRUCTION___
  printf("%d\n", j+1);
#endif

  /* enqueue trajectory queue node into packet trajectory queue */
  memset(&trajectory_qnode, 0, sizeof(trajectory_qnode));
  trajectory_qnode.intersection_id = j+1;
  trajectory_qnode.gnode = &(G[j]);
  memcpy(&(trajectory_qnode.euclidean_pos), &(G[j].coordinate), sizeof(G[j].coordinate)); 
  Enqueue((queue_t*) &(packet_trajectory), (queue_node_t*) &trajectory_qnode);

  /** check whether the packet trajectory has a vaild length */
  if(packet_trajectory.size <= 1)
  {
      printf("Install_PartialPacketTrajectory_Into_Packet(): packet_trajectory.size(%d) must be at least 2\n", packet_trajectory.size);
      exit(1);
  }

  /** reverse the order of the queue to make the trajectory be the path from src to dst */
  ReversePacketTrajectoryQueue(&(packet_trajectory), G, G_size);

  /** copy the partial path (i.e., the current hop and the next hop) into packet's packet trajectory, that is, packet->packet_trajectory */

  ptr = &(packet_trajectory.head);

  for(i = 0; i < 2; i++)
  {
    ptr = ptr->next;

    /* enqueue trajectory queue node into packet trajectory queue */
    memset(&trajectory_qnode, 0, sizeof(trajectory_qnode));
    trajectory_qnode.intersection_id = ptr->intersection_id;
    trajectory_qnode.gnode = ptr->gnode;
    memcpy(&(trajectory_qnode.euclidean_pos), &(ptr->euclidean_pos), sizeof(trajectory_qnode.euclidean_pos)); 
    Enqueue((queue_t*) &(packet->packet_trajectory), (queue_node_t*) &trajectory_qnode);
  }
 
  /** destroy packet_trajectory */
  DestroyQueue((queue_t*) &(packet_trajectory));

  /** set the current packet position to the first intersection on the packet trajectory */
  packet->packet_trajectory.current_packet_position = packet->packet_trajectory.head.next;
  packet->packet_trajectory.current_packet_intersection_id = packet->packet_trajectory.head.next->intersection_id;
  packet->packet_trajectory.order = 0;

  /** adjust the target point with the next-hop intersection id */
  //packet->target_point_id = packet->packet_trajectory.head.next->next->intersection_id;

  /** adjust the target point with the packet destination id, i.e., the end of the packet trajectory, not dst */
  packet->target_point_id = packet->packet_trajectory.head.prev->intersection_id;
  //packet->target_point_id = dst;

  result = TRUE;
  return result;
}

boolean Install_PacketTrajectory_Into_Packet_With_Edge(parameter_t *param, double current_time, struct_graph_node *G, int G_size, char *tail_node, char *head_node, packet_queue_node_t *packet)
{ //set up packet's packet trajectory for the edge (tail_node, head_node) under the stationary-node-based dynamic forwarding 					
  boolean result = FALSE;
  packet_trajectory_queue_node_t trajectory_qnode; //packet trajectory queue node
  int tail_node_id = atoi(tail_node); //tail node id
  int head_node_id = atoi(head_node); //head node id
  directional_edge_queue_node_t* pEdgeNode = NULL; //pointer to a directed edge queue node

  /** check whether packet_trajectory is empty or not; if not, destroy it first */
  if(packet->packet_trajectory.size > 0)
    DestroyQueue((queue_t*) &(packet->packet_trajectory));

  /** check the validity of tail_node_id and head_node_id */
  pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
  if(pEdgeNode == NULL)
  {
    printf("Install_PacketTrajectory_Into_Packet_With_Edge(): pEdgeNode for edge (%s,%s) is NULL\n", tail_node, head_node);
    exit(1);
  }

  /** enqueue two queue nodes for edge (tail_node, head_node) */
  /* enqueue the trajectory queue node for tail_node into packet trajectory queue */
  memset(&trajectory_qnode, 0, sizeof(trajectory_qnode));
  trajectory_qnode.intersection_id = tail_node_id;
  trajectory_qnode.gnode = &(G[tail_node_id-1]);
  memcpy(&(trajectory_qnode.euclidean_pos), &(G[tail_node_id-1].coordinate), sizeof(G[tail_node_id-1].coordinate)); 
  Enqueue((queue_t*) &(packet->packet_trajectory), (queue_node_t*) &trajectory_qnode);

  /* enqueue the trajectory queue node for head_node into packet trajectory queue */
  memset(&trajectory_qnode, 0, sizeof(trajectory_qnode));
  trajectory_qnode.intersection_id = head_node_id;
  trajectory_qnode.gnode = &(G[head_node_id-1]);
  memcpy(&(trajectory_qnode.euclidean_pos), &(G[head_node_id-1].coordinate), sizeof(G[head_node_id-1].coordinate)); 
  Enqueue((queue_t*) &(packet->packet_trajectory), (queue_node_t*) &trajectory_qnode);

  /** set the current packet position to the first intersection on the packet trajectory */
  packet->packet_trajectory.current_packet_position = packet->packet_trajectory.head.next;
  packet->packet_trajectory.current_packet_intersection_id = packet->packet_trajectory.head.next->intersection_id;
  packet->packet_trajectory.order = 0;

  /** set or adjust the target point with the packet destination id, i.e., head_node_id */
  packet->target_point_id = head_node_id;

  result = TRUE;
  return result;
}

boolean Install_StaticPacketTrajectory_Into_Packet(parameter_t *param, double current_time, struct_graph_node *G, int G_size, int static_packet_trajectory[], int static_packet_trajectory_size, packet_queue_node_t *packet)
{ //set up packet's packet trajectory with the static packet trajectory 					
  packet_trajectory_queue_node_t trajectory_qnode; //trajectory queue node
  int size = static_packet_trajectory_size;
  int i = 0; //for-loop index
  int j = 0; //array index

  /** check whether packet_trajectory is empty or not; if not, destroy it first */
  if(packet->packet_trajectory.size > 0)
    DestroyQueue((queue_t*) &(packet->packet_trajectory));

  /** construct the packet trajectory from src to dst */
  for(i = 0; i < size; i++)
  {
    /* enqueue trajectory queue node into packet trajectory queue */
    memset(&trajectory_qnode, 0, sizeof(trajectory_qnode));
    trajectory_qnode.intersection_id = static_packet_trajectory[i];
    j = static_packet_trajectory[i] - 1;
    trajectory_qnode.gnode = &(G[j]);
    memcpy(&(trajectory_qnode.euclidean_pos), &(G[j].coordinate), sizeof(G[j].coordinate)); 
    Enqueue((queue_t*) &(packet->packet_trajectory), (queue_node_t*) &trajectory_qnode);
  }

  /** set the current packet position to the first intersection on the packet trajectory */
  packet->packet_trajectory.current_packet_position = packet->packet_trajectory.head.next;
  packet->packet_trajectory.current_packet_intersection_id = packet->packet_trajectory.head.next->intersection_id;
  packet->packet_trajectory.order = 0;

  return TRUE;
}

/** Carrier Trace Queue Operations */
void Enqueue_CarrierTraceEntry(parameter_t *param, double current_time, packet_queue_node_t *packet, vanet_node_type_t node_type, void *vanet_node)
{ //enqueue the packet carrier trace entry into packet's carrier trace queue according to data forwarding mode

  if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
  {
    Enqueue_CarrierTraceEntry_For_Upload(param, current_time, packet, node_type, vanet_node);
  }
  else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
  {
    Enqueue_CarrierTraceEntry_For_Download(param, current_time, packet, node_type, vanet_node);
  }
  else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
  {
    Enqueue_CarrierTraceEntry_For_V2V(param, current_time, packet, node_type, vanet_node);
  }
  else
  {
    printf("Enqueue_CarrierTraceEntry(): param->data_forwarding_mode(%d) is not supportet!\n", param->data_forwarding_mode);
    exit(1);
  }
}

void Enqueue_CarrierTraceEntry_For_Upload(parameter_t *param, double current_time, packet_queue_node_t *packet, vanet_node_type_t node_type, void *vanet_node)
{ //enqueue the packet carrier trace entry into packet's carrier trace queue for upload forwarding mode
  struct_access_point_t *AP = NULL; //pointer to an access point
  stationary_node_queue_node_t *stationary_node = NULL; //pointer to a stationary node
  struct_vehicle_t *carrier_vehicle = NULL; //pointer to a carrier vehicle
  carrier_trace_queue_node_t carrier_trace_qnode; //carrier trace queue node
  char *pTailNode = NULL, *pHeadNode = NULL; //tail node and head node for a directed edge
  directional_edge_queue_node_t* pEdgeNode = NULL; //pointer to a directed edge node
  int target_point_id = packet->target_point_id; //packet's target point id
  int src_id = 0; //source intersection id
  int dst_id = 0; //destination intersection id
  double EDD = 0; //Expected Delivery Delay
  double EDD_SD = 0; // Delivery Delay Standard Deviation
  int intersection_id = 0; //intersection id

  /*@for debugging */
  //if(current_time > 3953 && packet->id == 107)
  //{
  //  printf("Enqueue_CarrierTraceEntry_For_Upload(): packet->id=%d, packet->carrier_vnode->id=%d\n", packet->id, packet->carrier_vnode->id);
  //}
  /*****************/

  /** set up carrier_trace_qnode with the information of current_time, node_type, and vanet_node */
  memset(&carrier_trace_qnode, 0, sizeof(carrier_trace_qnode));
  carrier_trace_qnode.data_forwarding_mode = param->data_forwarding_mode;
  carrier_trace_qnode.node_type = node_type;
  carrier_trace_qnode.receive_time = current_time;

  if(node_type == VANET_NODE_AP)
  {
    AP = (struct_access_point_t*) vanet_node;
    carrier_trace_qnode.carry_src_id = AP->id;
    carrier_trace_qnode.carry_dst_id = AP->id;

    /* set the intersection id to AP->vertex */
    carrier_trace_qnode.intersection_id = atoi(AP->vertex); 

    /* set the Euclidean position to AP's coordinate */
    memcpy(&(carrier_trace_qnode.euclidean_pos), &(AP->coordinate), sizeof(carrier_trace_qnode.euclidean_pos));

    /* check whether the forwarding scheme is one of stationary-based forwarding schemes, such as packet-trajectory-based forwarding */
    if(param->vehicle_vanet_edd_model == VANET_EDD_PER_SNODE_MODEL && param->vehicle_vanet_target_point_selection_type == VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT)
    { //for the case of the packet-trajectory-based forwarding
      /* set the graph_pos to the edge where the packet should be forwarded */
      if(packet->packet_trajectory.order >= packet->packet_trajectory.size )
      {
        printf("Enqueue_CarrierTraceEntry_For_Upload(): at time=%.2f, packet->id=%d: packet->packet_trajectory.order(%d) must be less than packet->packet_trajectory.size(%d)\n", current_time, packet->id, packet->packet_trajectory.order, packet->packet_trajectory.size);
        exit(1);
      }
      pTailNode = packet->packet_trajectory.current_packet_position->gnode->vertex;

      if(packet->packet_trajectory.order+1 >= packet->packet_trajectory.size)
      {
        printf("Enqueue_CarrierTraceEntry_For_Upload(): at time=%.2f, packet->id=%d: (packet->packet_trajectory.order+1)(%d) must be less than packet->packet_trajectory.size(%d)\n", current_time, packet->id, packet->packet_trajectory.order+1, packet->packet_trajectory.size);
        exit(1);
      }

      /* check whether this packet trajectory queue node is the last one or not */
      if(packet->packet_trajectory.order == packet->packet_trajectory.size-1 )
      { //for the last queue node for AP, the head node is an arbitrary adjacent node of the tail node 
        pHeadNode = packet->packet_trajectory.current_packet_position->gnode->next->vertex;
      }
      else
      { //otherwise, the head node is the vertex of the next queue node
        pHeadNode = packet->packet_trajectory.current_packet_position->next->gnode->vertex;
      }

      pEdgeNode = FastLookupDirectionalEdgeQueue(param->vanet_table.Gr, pTailNode, pHeadNode);
      if(pEdgeNode == NULL)
      {
        printf("Enqueue_CarrierTraceEntry_For_Upload(): pEdgeNode is NULL\n");
        exit(1);      
      }
        carrier_trace_qnode.graph_pos.eid = pEdgeNode->eid;
        carrier_trace_qnode.graph_pos.offset = 0;
        carrier_trace_qnode.graph_pos.enode = pEdgeNode;
    }
    else
    { //otherwise
      carrier_trace_qnode.graph_pos.eid = 0;
      carrier_trace_qnode.graph_pos.offset = 0;
      carrier_trace_qnode.graph_pos.enode = 0;
    }

    /* register AP's target point id */
    carrier_trace_qnode.target_point_id = target_point_id;
  }
  else if(node_type == VANET_NODE_VEHICLE)
  {
    carrier_vehicle = (struct_vehicle_t*) vanet_node;
    carrier_trace_qnode.carry_src_id = packet->carry_src_id;
    carrier_trace_qnode.carry_dst_id = packet->carry_dst_id;

    /* set the intersection id to vehicle's tail_node, that is, path_ptr->vertex */
    carrier_trace_qnode.intersection_id = atoi(carrier_vehicle->path_ptr->vertex);

    carrier_trace_qnode.EDD = carrier_vehicle->EDD;
    carrier_trace_qnode.EDD_SD = carrier_vehicle->EDD_SD;

    /* set the euclidean_pos to the current Euclidean position of the vehicle */
    memcpy(&(carrier_trace_qnode.euclidean_pos), &(carrier_vehicle->current_pos), sizeof(carrier_trace_qnode.euclidean_pos));

    /* set the graph_pos to the edge where the packet should be forwarded, that is, where the vehicle is moving */
    memcpy(&(carrier_trace_qnode.graph_pos), &(carrier_vehicle->current_pos_in_digraph), sizeof(carrier_trace_qnode.graph_pos));
    
    /* register carrier_vehicle's target point id */
    carrier_trace_qnode.target_point_id = target_point_id;
  }
  else if(node_type == VANET_NODE_SNODE)
  {
    stationary_node = (stationary_node_queue_node_t*) vanet_node;
    carrier_trace_qnode.carry_src_id = packet->carry_src_id;
    carrier_trace_qnode.carry_dst_id = packet->carry_dst_id;

    /* set the intersection id to stationary noed's intersection id */
    carrier_trace_qnode.intersection_id = stationary_node->intersection_id;

    /* get the EDD and EDD_SD with the stationary node's intersection id and the packet's target point id */
    src_id = atoi(stationary_node->gnode->vertex);
    dst_id = target_point_id;

    VADD_Get_EDD_And_EDD_SD_In_Shortest_Path_Model(param, src_id, dst_id, &EDD, &EDD_SD); //get EDD and EDD_SD from src_id to dst_id

    carrier_trace_qnode.EDD = EDD;
    carrier_trace_qnode.EDD_SD = EDD_SD;
    memcpy(&(carrier_trace_qnode.euclidean_pos), &(stationary_node->gnode->coordinate), sizeof(carrier_trace_qnode.euclidean_pos));
    
    /* check whether the queue node is the last one or not */
    if(packet->packet_trajectory.order == packet->packet_trajectory.size-1)
    { //for the case of the last queue node
      intersection_id = packet->packet_trajectory.current_packet_intersection_id;
      if(intersection_id != packet->target_point_id)
      {
        printf("Enqueue_CarrierTraceEntry_For_Download(): at time=%.2f, packet->id=%d: packet trajectory's last intersection(%d) is different from packet's target point(%d)\n", current_time, packet->id, intersection_id, packet->target_point_id);
        exit(1);
      }
      carrier_trace_qnode.graph_pos.eid = 0;
      carrier_trace_qnode.graph_pos.offset = 0;
      carrier_trace_qnode.graph_pos.enode = 0;
    }
    else
    { //otherwise
      pHeadNode = packet->packet_trajectory.current_packet_position->next->gnode->vertex;

      pEdgeNode = FastLookupDirectionalEdgeQueue(param->vanet_table.Gr, pTailNode, pHeadNode);
      if(pEdgeNode == NULL)
      {
        printf("Enqueue_CarrierTraceEntry_For_Download(): pEdgeNode is NULL\n");
        exit(1);      
      }
      carrier_trace_qnode.graph_pos.eid = pEdgeNode->eid;
      carrier_trace_qnode.graph_pos.offset = 0;
      carrier_trace_qnode.graph_pos.enode = pEdgeNode;
    }

    /* register AP's target point id */
    carrier_trace_qnode.target_point_id = target_point_id;
  }
  else
  {
    printf("Enqueue_CarrierTraceEntry_For_Upload(): Error: node_type(%d) is unknown!\n", node_type);
    exit(1);
  }

  /* enqueue carrier_trace_qnode into packet->carrier_trace queue */
  Enqueue((queue_t*) &(packet->carrier_trace), (queue_node_t*) &carrier_trace_qnode);
}

void Enqueue_CarrierTraceEntry_For_Download(parameter_t *param, double current_time, packet_queue_node_t *packet, vanet_node_type_t node_type, void *vanet_node)
{ //enqueue the packet carrier trace entry into packet's carrier trace queue for download forwarding mode
  struct_access_point_t *AP = NULL; //pointer to an access point
  stationary_node_queue_node_t *stationary_node = NULL; //pointer to a stationary node
  struct_vehicle_t *carrier_vehicle = NULL; //pointer to a carrier vehicle
  struct_vehicle_t *destination_vehicle = NULL; //pointer to a destination vehicle
  carrier_trace_queue_node_t carrier_trace_qnode; //carrier trace queue node
  char *pTailNode = NULL, *pHeadNode = NULL; //tail node and head node for a directed edge
  directional_edge_queue_node_t* pEdgeNode = NULL; //pointer to a directed edge node
  int target_point_id = packet->target_point_id; //packet's target point id
  int src_id = 0; //source intersection id
  int dst_id = 0; //destination intersection id
  double EDD = 0; //Expected Delivery Delay
  double EDD_SD = 0; // Delivery Delay Standard Deviation
  int intersection_id = 0; //intersection id

  /** set up carrier_trace_qnode with the information of current_time, node_type, and vanet_node */
  memset(&carrier_trace_qnode, 0, sizeof(carrier_trace_qnode));
  carrier_trace_qnode.data_forwarding_mode = param->data_forwarding_mode;
  carrier_trace_qnode.node_type = node_type;
  carrier_trace_qnode.receive_time = current_time;

  /** register destination vehicle information */
  destination_vehicle = packet->dst_vnode;
  carrier_trace_qnode.dst_vid = destination_vehicle->id;
  memcpy(&(carrier_trace_qnode.dst_euclidean_pos), &(destination_vehicle->current_pos), sizeof(carrier_trace_qnode.dst_euclidean_pos));
  memcpy(&(carrier_trace_qnode.dst_graph_pos), &(destination_vehicle->current_pos_in_digraph), sizeof(carrier_trace_qnode.dst_graph_pos));


  if(node_type == VANET_NODE_AP)
  {
    AP = (struct_access_point_t*) vanet_node;
    carrier_trace_qnode.carry_src_id = AP->id;
    carrier_trace_qnode.carry_dst_id = AP->id;

    /* set the intersection id to AP->vertex */
    carrier_trace_qnode.intersection_id = atoi(AP->vertex); 

    /* get the EDD and EDD_SD with the AP's vertex and the packet's target point id */
    src_id = atoi(AP->vertex);
    dst_id = target_point_id;

    if(param->vehicle_vanet_target_point_selection_type == VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT)
    { //for the packet-trajectory-based target point selection type, we use the EDD and EDD_SD based on the shortest path model
      VADD_Get_EDD_And_EDD_SD_In_Shortest_Path_Model(param, src_id, dst_id, &EDD, &EDD_SD); //get EDD and EDD_SD from src_id to dst_id

      carrier_trace_qnode.EDD_for_download = EDD;
      carrier_trace_qnode.EDD_SD_for_download = EDD_SD;
    }
    else
    { //for the other target point selection types, we use the EDD and EDD_SD based on the stochastic model
      carrier_trace_qnode.EDD_for_download = AP->EDD_for_download;
      carrier_trace_qnode.EDD_SD_for_download = AP->EDD_SD_for_download;
    }
    /*@ check whether EDD and AP->EDD_for_download are the same or not */

    memcpy(&(carrier_trace_qnode.euclidean_pos), &(AP->coordinate), sizeof(carrier_trace_qnode.euclidean_pos));

    /* check whether the forwarding scheme is one of stationary-based forwarding schemes, such as packet-trajectory-based forwarding */
    if(param->vehicle_vanet_target_point_selection_type == VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT)
    //if(param->vehicle_vanet_edd_model == VANET_EDD_PER_SNODE_MODEL && param->vehicle_vanet_target_point_selection_type == VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT)
    { //for the case of the packet-trajectory-based forwarding

      /* set the graph_pos to the edge where the packet should be forwarded */
      if(packet->packet_trajectory.order >= packet->packet_trajectory.size )
      {
        printf("Enqueue_CarrierTraceEntry_For_Download(): at time=%.2f, packet->id=%d: packet->packet_trajectory.order(%d) must be less than packet->packet_trajectory.size(%d)\n", current_time, packet->id, packet->packet_trajectory.order, packet->packet_trajectory.size);
        exit(1);
      }
      pTailNode = packet->packet_trajectory.current_packet_position->gnode->vertex;

      if(packet->packet_trajectory.order+1 >= packet->packet_trajectory.size )
      {
        printf("Enqueue_CarrierTraceEntry_For_Download(): at time=%.2f, packet->id=%d: (packet->packet_trajectory.order+1)(%d) must be less than packet->packet_trajectory.size(%d)\n", current_time, packet->id, packet->packet_trajectory.order+1, packet->packet_trajectory.size);
        exit(1);
      }

      /* check whether this packet trajectory queue node is the last one or not */
      if(packet->packet_trajectory.order == packet->packet_trajectory.size-1 )
      { //for the last queue node for AP, the head node is an arbitrary adjacent node of the tail node 
        pHeadNode = packet->packet_trajectory.current_packet_position->gnode->next->vertex;
      }
      else
      { //otherwise, the head node is the vertex of the next queue node
        pHeadNode = packet->packet_trajectory.current_packet_position->next->gnode->vertex;
      }

      pEdgeNode = FastLookupDirectionalEdgeQueue(param->vanet_table.Gr, pTailNode, pHeadNode);
      if(pEdgeNode == NULL)
      {
        printf("Enqueue_CarrierTraceEntry_For_Download(): pEdgeNode is NULL\n");
        exit(1);      
      }
      carrier_trace_qnode.graph_pos.eid = pEdgeNode->eid;
      carrier_trace_qnode.graph_pos.offset = 0;
      carrier_trace_qnode.graph_pos.enode = pEdgeNode;
    }
    else
    { //otherwise
      carrier_trace_qnode.graph_pos.eid = 0;
      carrier_trace_qnode.graph_pos.offset = 0;
      carrier_trace_qnode.graph_pos.enode = 0;
    }

    /* register AP's target point id */
    carrier_trace_qnode.target_point_id = target_point_id;
  }
  else if(node_type == VANET_NODE_VEHICLE)
  {
    carrier_vehicle = (struct_vehicle_t*) vanet_node;
    carrier_trace_qnode.carry_src_id = packet->carry_src_id;
    carrier_trace_qnode.carry_dst_id = packet->carry_dst_id;

    /* set the intersection id to vehicle's tail_node, that is, path_ptr->vertex */
    carrier_trace_qnode.intersection_id = atoi(carrier_vehicle->path_ptr->vertex);

    /* get the EDD and EDD_SD with the stationary node's intersection id and the packet's target point id */
    src_id = atoi(carrier_vehicle->path_ptr->vertex);
    dst_id = target_point_id;

    VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Carrier(param, dst_id, carrier_vehicle, param->vanet_table.FTQ, &EDD, &EDD_SD); //get the EDD and EDD_SD for a target point towards a destination vehicle, based on VADD Model (i.e., Per-intersection Model) from the position of carrier_vehicle, that is, from carrier's edge offset on the road network graph.

    carrier_trace_qnode.EDD_for_download = EDD;
    carrier_trace_qnode.EDD_SD_for_download = EDD_SD;

    /* set the euclidean_pos to the current Euclidean position of the vehicle */
    memcpy(&(carrier_trace_qnode.euclidean_pos), &(carrier_vehicle->current_pos), sizeof(carrier_trace_qnode.euclidean_pos));
    
    /* set the graph_pos to the edge where the vehicle is moving */
    memcpy(&(carrier_trace_qnode.graph_pos), &(carrier_vehicle->current_pos_in_digraph), sizeof(carrier_trace_qnode.graph_pos));
    pEdgeNode = carrier_vehicle->current_pos_in_digraph.enode;
    if(pEdgeNode == NULL)
    {
      printf("Enqueue_CarrierTraceEntry_For_Download(): pEdgeNode is NULL\n");
      exit(1);      
    }

    carrier_trace_qnode.graph_pos.type = COORDINATE_DIRECTIONAL_EDGE_TYPE_REGULAR_EDGE;
    carrier_trace_qnode.graph_pos.eid = pEdgeNode->eid;
    carrier_trace_qnode.graph_pos.offset = 0;
    carrier_trace_qnode.graph_pos.enode = pEdgeNode;
    strcpy(carrier_trace_qnode.graph_pos.tail_node, pEdgeNode->tail_node);
    strcpy(carrier_trace_qnode.graph_pos.head_node, pEdgeNode->head_node);
    carrier_trace_qnode.graph_pos.weight = pEdgeNode->weight;

    /* register carrier_vehicle's target point id */
    carrier_trace_qnode.target_point_id = target_point_id;
  }
  else if(node_type == VANET_NODE_SNODE)
  {
    stationary_node = (stationary_node_queue_node_t*) vanet_node;
    carrier_trace_qnode.carry_src_id = packet->carry_src_id;
    carrier_trace_qnode.carry_dst_id = packet->carry_dst_id;

    /* set the intersection id to stationary noed's intersection id */
    carrier_trace_qnode.intersection_id = stationary_node->intersection_id;

    /* get the EDD and EDD_SD with the stationary node's intersection id and the packet's target point id */
    src_id = atoi(stationary_node->gnode->vertex);
    dst_id = target_point_id;

    VADD_Get_EDD_And_EDD_SD_In_Shortest_Path_Model(param, src_id, dst_id, &EDD, &EDD_SD); //get EDD and EDD_SD from src_id to dst_id

    carrier_trace_qnode.EDD_for_download = EDD;
    carrier_trace_qnode.EDD_SD_for_download = EDD_SD;
    memcpy(&(carrier_trace_qnode.euclidean_pos), &(stationary_node->gnode->coordinate), sizeof(carrier_trace_qnode.euclidean_pos));
    
    /* set the graph_pos to the edge where the packet should be forwarded */
    if(packet->packet_trajectory.order >= packet->packet_trajectory.size )
    {
      printf("Enqueue_CarrierTraceEntry_For_Download(): at time=%.2f, packet->id=%d: packet->packet_trajectory.order(%d) must be less than packet->packet_trajectory.size(%d)\n", current_time, packet->id, packet->packet_trajectory.order, packet->packet_trajectory.size);
      exit(1);
    }
    pTailNode = packet->packet_trajectory.current_packet_position->gnode->vertex;

    /* check whether the queue node is the last one or not */
    if(packet->packet_trajectory.order == packet->packet_trajectory.size-1)
    { //for the case of the last queue node
      intersection_id = packet->packet_trajectory.current_packet_intersection_id;
      //if((intersection_id != packet->target_point_id) && (param->vehicle_vanet_target_point_computation_method != VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_FULLY_DYNAMIC_FORWARDING))
      //@Note that for the fully dynamic forwarding, the first forwarding from AP to Stationary Node lets the target point be different from the last node of the packet trajectory
      if(intersection_id != packet->target_point_id)
      {
        printf("Enqueue_CarrierTraceEntry_For_Download(): at time=%.2f, packet->id=%d: packet trajectory's last intersection(%d) is different from packet's target point(%d)\n", current_time, packet->id, intersection_id, packet->target_point_id);
        exit(1);
      }

      carrier_trace_qnode.graph_pos.type = COORDINATE_DIRECTIONAL_EDGE_TYPE_VERTEX_EDGE;
      carrier_trace_qnode.graph_pos.eid = 0;
      carrier_trace_qnode.graph_pos.offset = 0;
      carrier_trace_qnode.graph_pos.enode = 0;
      strcpy(carrier_trace_qnode.graph_pos.tail_node, pTailNode);
      strcpy(carrier_trace_qnode.graph_pos.head_node, pTailNode);
      carrier_trace_qnode.graph_pos.weight = 0;
    }
    else
    { //otherwise
      pHeadNode = packet->packet_trajectory.current_packet_position->next->gnode->vertex;

      pEdgeNode = FastLookupDirectionalEdgeQueue(param->vanet_table.Gr, pTailNode, pHeadNode);
      if(pEdgeNode == NULL)
      {
        printf("Enqueue_CarrierTraceEntry_For_Download(): pEdgeNode is NULL\n");
        exit(1);      
      }

      carrier_trace_qnode.graph_pos.type = COORDINATE_DIRECTIONAL_EDGE_TYPE_REGULAR_EDGE;
      carrier_trace_qnode.graph_pos.eid = pEdgeNode->eid;
      carrier_trace_qnode.graph_pos.offset = 0;
      carrier_trace_qnode.graph_pos.enode = pEdgeNode;
      strcpy(carrier_trace_qnode.graph_pos.tail_node, pEdgeNode->tail_node);
      strcpy(carrier_trace_qnode.graph_pos.head_node, pEdgeNode->head_node);
      carrier_trace_qnode.graph_pos.weight = pEdgeNode->weight;
    }

    /* register AP's target point id */
    carrier_trace_qnode.target_point_id = target_point_id;
  }
  else
  {
    printf("Enqueue_CarrierTraceEntry_For_Download(): Error: node_type(%d) is unknown!\n", node_type);
    exit(1);
  }

  /* enqueue carrier_trace_qnode into packet->carrier_trace queue */
  Enqueue((queue_t*) &(packet->carrier_trace), (queue_node_t*) &carrier_trace_qnode);
}

void Enqueue_CarrierTraceEntry_For_V2V(parameter_t *param, double current_time, packet_queue_node_t *packet, vanet_node_type_t node_type, void *vanet_node)
{ //enqueue the packet carrier trace entry into packet's carrier trace queue for V2V forwarding mode
  struct_access_point_t *AP = NULL; //pointer to an access point
  stationary_node_queue_node_t *stationary_node = NULL; //pointer to a stationary node
  struct_vehicle_t *carrier_vehicle = NULL; //pointer to a carrier vehicle
  struct_vehicle_t *destination_vehicle = NULL; //pointer to a destination vehicle
  carrier_trace_queue_node_t carrier_trace_qnode; //carrier trace queue node
  char *pTailNode = NULL, *pHeadNode = NULL; //tail node and head node for a directed edge
  directional_edge_queue_node_t* pEdgeNode = NULL; //pointer to a directed edge node
  int target_point_id = packet->target_point_id; //packet's target point id
  int src_id = 0; //source intersection id
  int dst_id = 0; //destination intersection id
  double EDD = 0; //Expected Delivery Delay
  double EDD_SD = 0; // Delivery Delay Standard Deviation
  int intersection_id = 0; //intersection id

  /** set up carrier_trace_qnode with the information of current_time, node_type, and vanet_node */
  memset(&carrier_trace_qnode, 0, sizeof(carrier_trace_qnode));
  carrier_trace_qnode.data_forwarding_mode = param->data_forwarding_mode;
  carrier_trace_qnode.node_type = node_type;
  carrier_trace_qnode.receive_time = current_time;

  /** register destination vehicle information */
  destination_vehicle = packet->dst_vnode;
  carrier_trace_qnode.dst_vid = destination_vehicle->id;
  memcpy(&(carrier_trace_qnode.dst_euclidean_pos), &(destination_vehicle->current_pos), sizeof(carrier_trace_qnode.dst_euclidean_pos));
  memcpy(&(carrier_trace_qnode.dst_graph_pos), &(destination_vehicle->current_pos_in_digraph), sizeof(carrier_trace_qnode.dst_graph_pos));


  if(node_type == VANET_NODE_AP)
  {
    AP = (struct_access_point_t*) vanet_node;
    carrier_trace_qnode.carry_src_id = AP->id;
    carrier_trace_qnode.carry_dst_id = AP->id;

    /* set the intersection id to AP->vertex */
    carrier_trace_qnode.intersection_id = atoi(AP->vertex); 

    /* get the EDD and EDD_SD with the AP's vertex and the packet's target point id */
    src_id = atoi(AP->vertex);
    dst_id = target_point_id;

    memcpy(&(carrier_trace_qnode.euclidean_pos), &(AP->coordinate), sizeof(carrier_trace_qnode.euclidean_pos));

    { //otherwise
      carrier_trace_qnode.graph_pos.eid = 0;
      carrier_trace_qnode.graph_pos.offset = 0;
      carrier_trace_qnode.graph_pos.enode = 0;
    }

    /* register AP's target point id */
    carrier_trace_qnode.target_point_id = target_point_id;
  }
  else if(node_type == VANET_NODE_VEHICLE)
  {
    carrier_vehicle = (struct_vehicle_t*) vanet_node;

#if TPD_PACKET_CARRIER_VEHICLE_TRACE_FLAG /* [ */
	printf("%s:%d [%0.f] carrier_vehicle(%d) in (%s->%s: %0.f) is next carrier\n",
			__FUNCTION__, __LINE__,
			current_time,
			carrier_vehicle->id,
			carrier_vehicle->current_pos_in_digraph.tail_node,
			carrier_vehicle->current_pos_in_digraph.head_node,
			carrier_vehicle->current_pos_in_digraph.offset);
#endif /* ] */


    carrier_trace_qnode.carry_src_id = packet->carry_src_id;
    carrier_trace_qnode.carry_dst_id = packet->carry_dst_id;

    /* set the intersection id to vehicle's tail_node, that is, path_ptr->vertex */
    carrier_trace_qnode.intersection_id = atoi(carrier_vehicle->path_ptr->vertex);

    /* get the EDD and EDD_SD with the stationary node's intersection id and the packet's target point id */
    src_id = atoi(carrier_vehicle->path_ptr->vertex);
    dst_id = target_point_id;

    /* set the euclidean_pos to the current Euclidean position of the vehicle */
    memcpy(&(carrier_trace_qnode.euclidean_pos), &(carrier_vehicle->current_pos), sizeof(carrier_trace_qnode.euclidean_pos));
    
    /* set the graph_pos to the edge where the vehicle is moving */
    memcpy(&(carrier_trace_qnode.graph_pos), &(carrier_vehicle->current_pos_in_digraph), sizeof(carrier_trace_qnode.graph_pos));
    pEdgeNode = carrier_vehicle->current_pos_in_digraph.enode;
    if(pEdgeNode == NULL)
    {
      printf("%s:%d pEdgeNode is NULL\n",
			  __FUNCTION__, __LINE__);
      exit(1);      
    }

    carrier_trace_qnode.graph_pos.type = COORDINATE_DIRECTIONAL_EDGE_TYPE_REGULAR_EDGE;
    carrier_trace_qnode.graph_pos.eid = pEdgeNode->eid;
    carrier_trace_qnode.graph_pos.offset = 0;
    carrier_trace_qnode.graph_pos.enode = pEdgeNode;
    strcpy(carrier_trace_qnode.graph_pos.tail_node, pEdgeNode->tail_node);
    strcpy(carrier_trace_qnode.graph_pos.head_node, pEdgeNode->head_node);
    carrier_trace_qnode.graph_pos.weight = pEdgeNode->weight;

    /* register carrier_vehicle's target point id */
    carrier_trace_qnode.target_point_id = target_point_id;
  }
  else if(node_type == VANET_NODE_SNODE)
  {
    stationary_node = (stationary_node_queue_node_t*) vanet_node;
    carrier_trace_qnode.carry_src_id = packet->carry_src_id;
    carrier_trace_qnode.carry_dst_id = packet->carry_dst_id;

    /* set the intersection id to stationary noed's intersection id */
    carrier_trace_qnode.intersection_id = stationary_node->intersection_id;

    memcpy(&(carrier_trace_qnode.euclidean_pos), &(stationary_node->gnode->coordinate), sizeof(carrier_trace_qnode.euclidean_pos));
    
    /* register AP's target point id */
    carrier_trace_qnode.target_point_id = target_point_id;
  }
  else
  {
    printf("%s:%d Error: node_type(%d) is unknown!\n", 
			__FUNCTION__, __LINE__, node_type);
    exit(1);
  }

  /* enqueue carrier_trace_qnode into packet->carrier_trace queue */
  Enqueue((queue_t*) &(packet->carrier_trace), (queue_node_t*) &carrier_trace_qnode);
}

void Show_CarrierTraceQueue(packet_queue_node_t *packet)
{ //show the carrier trace in packet's carrier trace queue according to data forwarding mode
  if(packet->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
  {
    Show_CarrierTraceQueue_For_Upload(packet);
  }
  else if(packet->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
  {
    Show_CarrierTraceQueue_For_Download(packet);
  }
  else
  {
    printf("Show_CarrierTraceQueue(): packet->data_forwarding_mode(%d) is not supportet!\n", packet->data_forwarding_mode);
    exit(1);
  }
}

void Show_CarrierTraceQueue_For_Upload(packet_queue_node_t *packet)
{ //show the carrier trace in packet's carrier trace queue for upload forwarding mode
  carrier_trace_queue_node_t *pQueueNode; //pointer to a carrier trace queue node
  int size = packet->carrier_trace.size; //size of carrier trace queue
  int i = 0; //for-loop index

  pQueueNode = &(packet->carrier_trace.head);
  for(i = 0; i < size; i++) //for-1
  {
    pQueueNode = pQueueNode->next; //get the next queue pointer

    /* print the trace entry pointed by p */
    if(pQueueNode->node_type == VANET_NODE_AP || pQueueNode->node_type == VANET_NODE_VEHICLE || pQueueNode->node_type == VANET_NODE_SNODE) //if-1.1
    {
      if(pQueueNode->node_type == VANET_NODE_SNODE)
      {
        printf("pid=%d: [%d=>(%d=>%d)=>%d: packet tp=%d, trace tp=%d], %d, <EDD=%.2f, EDD_SD=%.2f>, receive_time=%.2f, (%d->%d:%.2f), (%.2f,%.2f)\n", packet->id, packet->src_id, pQueueNode->carry_src_id, pQueueNode->carry_dst_id, packet->dst_id, packet->target_point_id,  pQueueNode->target_point_id, pQueueNode->node_type, pQueueNode->EDD_for_download, pQueueNode->EDD_SD_for_download, pQueueNode->receive_time, pQueueNode->carry_dst_id, pQueueNode->carry_dst_id, 0.0, pQueueNode->euclidean_pos.x, pQueueNode->euclidean_pos.y);
      }
      else
      {
        printf("pid=%d: [%d=>(%d=>%d)=>%d: packet tp=%d, trace tp=%d], %d, <EDD=%.2f, EDD_SD=%.2f>, receive_time=%.2f, (%s->%s:%.2f), (%.2f,%.2f)\n", packet->id, packet->src_id, pQueueNode->carry_src_id, pQueueNode->carry_dst_id, packet->dst_id, packet->target_point_id,  pQueueNode->target_point_id, pQueueNode->node_type, pQueueNode->EDD_for_download, pQueueNode->EDD_SD_for_download, pQueueNode->receive_time, pQueueNode->graph_pos.enode->tail_node, pQueueNode->graph_pos.enode->head_node, pQueueNode->graph_pos.offset, pQueueNode->euclidean_pos.x, pQueueNode->euclidean_pos.y);
      }
    } //end of if-1.1
    else //else-1.2
    {
      printf("Show_CarrierTraceQueue_For_Upload(): node_type(%d) is not supported yet!\n", pQueueNode->node_type);
      exit(1);
    } //end of else-1.2
  } //end of for-1
}

void Show_CarrierTraceQueue_For_Download(packet_queue_node_t *packet)
{ //show the carrier trace in packet's carrier trace queue for download forwarding mode
  carrier_trace_queue_node_t *pQueueNode; //pointer to a carrier trace queue node
  int size = packet->carrier_trace.size; //size of carrier trace queue
  int i = 0; //for-loop index

  pQueueNode = &(packet->carrier_trace.head);
  for(i = 0; i < size; i++) //for-1
  {
    pQueueNode = pQueueNode->next; //get the next queue pointer

    /* print the trace entry pointed by p */
    if(pQueueNode->node_type == VANET_NODE_AP || pQueueNode->node_type == VANET_NODE_VEHICLE || pQueueNode->node_type == VANET_NODE_SNODE) //if-1.1
    {
      if(pQueueNode->node_type == VANET_NODE_SNODE)
      {
        printf("pid=%d: [%d=>(%d=>%d)=>%d: packet tp=%d, trace tp=%d], %d, <EDD=%.2f, EDD_SD=%.2f>, receive_time=%.2f, (%d->%d:%.2f), (%.2f,%.2f) || dst_vehicle: (%s->%s:%.2f), (%.2f,%.2f)\n", packet->id, packet->src_id, pQueueNode->carry_src_id, pQueueNode->carry_dst_id, packet->dst_id, packet->target_point_id,  pQueueNode->target_point_id, pQueueNode->node_type, pQueueNode->EDD_for_download, pQueueNode->EDD_SD_for_download, pQueueNode->receive_time, pQueueNode->carry_dst_id, pQueueNode->carry_dst_id, 0.0, pQueueNode->euclidean_pos.x, pQueueNode->euclidean_pos.y, pQueueNode->dst_graph_pos.enode->tail_node, pQueueNode->dst_graph_pos.enode->head_node, pQueueNode->dst_graph_pos.offset, pQueueNode->dst_euclidean_pos.x, pQueueNode->dst_euclidean_pos.y);
      }
      else
      {
        printf("pid=%d: [%d=>(%d=>%d)=>%d: packet tp=%d, trace tp=%d], %d, <EDD=%.2f, EDD_SD=%.2f>, receive_time=%.2f, (%s->%s:%.2f), (%.2f,%.2f) || dst_vehicle: (%s->%s:%.2f), (%.2f,%.2f)\n", packet->id, packet->src_id, pQueueNode->carry_src_id, pQueueNode->carry_dst_id, packet->dst_id, packet->target_point_id,  pQueueNode->target_point_id, pQueueNode->node_type, pQueueNode->EDD_for_download, pQueueNode->EDD_SD_for_download, pQueueNode->receive_time, pQueueNode->graph_pos.enode->tail_node, pQueueNode->graph_pos.enode->head_node, pQueueNode->graph_pos.offset, pQueueNode->euclidean_pos.x, pQueueNode->euclidean_pos.y, pQueueNode->dst_graph_pos.enode->tail_node, pQueueNode->dst_graph_pos.enode->head_node, pQueueNode->dst_graph_pos.offset, pQueueNode->dst_euclidean_pos.x, pQueueNode->dst_euclidean_pos.y);
      }
    } //end of if-1.1
    else //else-1.2
    {
      printf("Show_CarrierTraceQueue_For_Download(): node_type(%d) is not supported yet!\n", pQueueNode->node_type);
      exit(1);
    } //end of else-1.2
  } //end of for-1
}

/** Packet Queue Operations */
void DestroyPacketQueueNode(packet_queue_node_t *packet)
{ //destroy packet queue node along with vehicle trajectory queue and carrier trace queue, and then free the memory of packet queue node
  if(packet == NULL)
  {
      printf("DestroyPacketQueueNode(): packet is NULL\n");
      exit(1);
  }

  /* destroy the queue node for packet, including vehicle_trajectory, carrier_trace, packet_trajectory, and predicted_encounter_graph */
  DestroyQueueNode(QTYPE_PACKET, (queue_node_t*)packet);
}

/** Forwarding Table Queue Operations */
void InitForwardingTableQueue(forwarding_table_queue_t *FTQ, parameter_t *param, struct_graph_node* Gr, int Gr_size)
{ //initialize the forwarding table for each intersection in the road network graph Gr
  forwarding_table_queue_node_t table_qnode; //forwarding table queue node
  forwarding_table_queue_node_t *pQueueNode = NULL; //pointer to a forwarding table queue node
  int i = 0; //for-loop index

  /** initialize FTQ with its queue type */
  InitQueue((queue_t*) FTQ, QTYPE_FORWARDING_TABLE);
  
  /** allocate an index table to access the forwarding table for each intersection */
  FTQ->index_table = (forwarding_table_queue_node_t**) calloc(Gr_size, sizeof(forwarding_table_queue_node_t*));
  assert_memory(FTQ->index_table);

  for(i = 0; i < Gr_size; i++)
  {
    memset(&table_qnode, 0, sizeof(table_qnode));
  
    pQueueNode = (forwarding_table_queue_node_t*) Enqueue((queue_t*)FTQ, (queue_node_t*)&table_qnode);

    /* set intersection id to i+1 */
    pQueueNode->intersection_id = i+1;

    /** create a graph G, edge queue EQ, and directional edge queue DEQ for data forwarding */
    /* create a graph G for the data forwarding towards a destination intersection */
    pQueueNode->G = Make_Forwarding_Graph(Gr, Gr_size, &(pQueueNode->G_size));

    /** construct edge queue EQ and associate it with graph G */
    /* construct an edge queue EQ containing the information of edges in graph G. */
    ConstructEdgeQueue(&(pQueueNode->EQ), pQueueNode->G, pQueueNode->G_size, param);

    /* associate each pair of two adjacent graph nodes (i.e., physical edge) with the corresponding edge entry in EQ */
    AssociateGraphEdgeWithEdgeEntry(pQueueNode->G, pQueueNode->G_size, &(pQueueNode->EQ));

    /** construct directional edge queue DEQ and associate it with graph Ga */
    /* construct a directional edge queue DEr containing the information of directional edges in graph G */
    ConstructDirectionalEdgeQueue(param, &(pQueueNode->DEQ), pQueueNode->G, pQueueNode->G_size);

    /* associate each pair of two adjacent graph nodes (i.e., physical edge) with the corresponding directional edge entry in DEQ */
    AssociateGraphEdgeWithDirectionalEdgeEntry(pQueueNode->G, pQueueNode->G_size, &(pQueueNode->DEQ));

	/** allocate the shortest delay matrices */
	pQueueNode->matrix_size_for_edd_in_Gr = pQueueNode->G_size;
	Floyd_Warshall_Allocate_Matrices_For_EDD(&(pQueueNode->Dr_edd), &(pQueueNode->Mr_edd), &(pQueueNode->Sr_edd), pQueueNode->matrix_size_for_edd_in_Gr);

	/** allocate the shortest cost matrices */
	pQueueNode->matrix_size_for_edc_in_Gr = pQueueNode->G_size;
	Floyd_Warshall_Allocate_Matrices_For_EDC(&(pQueueNode->Wr_edc), &(pQueueNode->Dr_edc), &(pQueueNode->Mr_edc), &(pQueueNode->Sr_edc), pQueueNode->matrix_size_for_edc_in_Gr);

    /** set up index table entry with the pointer to graph G */
    FTQ->index_table[i] = pQueueNode;
  }
}

void UpdateForwardingTableQueue(forwarding_table_queue_t *FTQ, parameter_t *param, struct_graph_node* Gr, int Gr_size, access_point_queue_t *APQ)
{ //update the forwarding table for each intersection in the road network graph Gr with the vehicular traffic statistics for Gr
  forwarding_table_queue_node_t *pFT_QNode = NULL; //pointer to a forwarding table queue node
  access_point_queue_node_t *pAP_QNode = NULL; //pointer to a forwarding table queue node
  char target_point[NAME_SIZE]; //target point that is an intersection used for packet target
  struct_traffic_table traffic_table_for_target_point; //traffic table for a target point  
  int traffic_table_index_for_target_point = 0; //traffic table index for a target point
  int i = 0, j = 0; //for-loop indices
  double AP_EDD = 0; //EDD at AP
  double AP_EDD_SD = 0; //EDD_SD at AP

  /* initialize traffic_table_for_target_point */
  memset(&traffic_table_for_target_point, 0, sizeof(traffic_table_for_target_point));

  /*@for debugging */
  //printf("Before UpdateForwardingTableQueue()\n");
  /****************/

  pFT_QNode = &(FTQ->head);
  for(i = 0; i < Gr_size; i++)
  {
    pFT_QNode = pFT_QNode->next;

    /** copy the vehicular traffic statistics of the edges in Gr into that in G */
    CopyVehicularTrafficStatistics(Gr, Gr_size, pFT_QNode->G, pFT_QNode->G_size);

    /** compute the Expected Delivery Delay (EDD) per intersection as target point */
    itoa(pFT_QNode->intersection_id, target_point);

    SetTargetPoint_In_TafficTable(&traffic_table_for_target_point, target_point);

    traffic_table_index_for_target_point = 0; //the first target point in the traffic table

    if(param->vehicle_vanet_edd_computation_model == VANET_EDD_COMPUTATION_BASED_ON_STOCHASTIC_MODEL)
	{
      VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model(param, pFT_QNode->G, pFT_QNode->G_size, &(pFT_QNode->DEQ), &traffic_table_for_target_point, traffic_table_index_for_target_point);
	}
    else
	{
      VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model(param, pFT_QNode->G, pFT_QNode->G_size, &(pFT_QNode->DEQ), &traffic_table_for_target_point, traffic_table_index_for_target_point, TRUE);

      VADD_Compute_EDC_And_EDC_SD_Based_On_Shortest_Path_Model(param, pFT_QNode->G, pFT_QNode->G_size, &(pFT_QNode->DEQ), &traffic_table_for_target_point, traffic_table_index_for_target_point, TRUE);
	}

    /** compute the EDDs at intersections having APs;
        Note that we compute the EDD and EDD_SD for each intersection only where VANET E2E Delay Model is Stochastic Model, because with VANET E2E Delay Model of Shortest Path Model, the EDD and EDD_SD of each intersection has been computed at VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model_For_Shortest_EDD */
    if(param->vehicle_vanet_edd_computation_model == VANET_EDD_COMPUTATION_BASED_ON_STOCHASTIC_MODEL)
    {
      pAP_QNode = &(APQ->head);
      for(j = 0; j < APQ->size; j++)
      {
        pAP_QNode = pAP_QNode->next;

        VADD_Compute_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(param, pFT_QNode->intersection_id, pAP_QNode->vertex, FTQ, &AP_EDD, &AP_EDD_SD); //compute the EDDs at the intersection corresponding to pAP_QNode->vertex
      }
    }
  }

  /*@for debugging */
  //printf("After UpdateForwardingTableQueue()\n");
  /****************/
}

/** Global Packet Queue Operations */
void InitGlobalPacketQueue_With_VanetInformationTable(global_packet_queue_t *GPQ, parameter_t *param)
{ //initialize the global packet queue that will contain all of the packet queue nodes in the vehicles in the road network with the VANET Information Table

    /** initialize Global Packet Queue */
    InitQueue((queue_t*)GPQ, QTYPE_GLOBAL_PACKET);

    /** let GPQ->ptr_vanet_table point to the Vanet Information Table */
    GPQ->ptr_vanet_table = &(param->vanet_table);
}

global_packet_queue_node_t* GetGlobalPacketPointerByID(global_packet_queue_t *Q, int id)
{ //return the global packet pointer corresponding to globally unique id; the id is greater than 0.
  global_packet_queue_node_t* p = NULL;
  int i; //for-loop index

  for(i = 0, p = &(Q->head); i < Q->size; i++)
  {
    p = p->next;
    if(p->id == id)
      break;
  }

  if(p == &(Q->head) || i == Q->size)
    return NULL;
  else
    return p;
}

global_packet_queue_node_t* Enqueue_Packet_Into_GlobalPacketQueue(parameter_t *param, double current_time, global_packet_queue_t *Q, packet_queue_node_t *packet)
{ //enqueue packet into global packet queue Q
  global_packet_queue_node_t queue_node; //global packet queue node
  global_packet_queue_node_t *pQueueNode = NULL; //pointer to a global packet queue node

  /** initialize global packet queue node with packet */
  memset(&queue_node, 0, sizeof(queue_node));

  queue_node.id = packet->id;
  queue_node.state = packet->state;
  queue_node.state_time = packet->state_time;
  queue_node.packet = packet;

  /** EDD information */
  queue_node.EDD = packet->expected_delivery_delay;
  queue_node.EDD_SD = packet->expected_delivery_delay_standard_deviation;

  /** Target Point */
  queue_node.target_point_id = packet->target_point_id;

  /** Target Point Recomputation Information: This target point recomputation information is used only in download mode */
  queue_node.target_point_recomputation_interval = 0;
  queue_node.target_point_recomputation_time = 0;

  /** packet_copy_count */
  queue_node.packet_copy_count++;
 
  /** enqueue queue_node into queue Q */
  pQueueNode = (global_packet_queue_node_t*)Enqueue((queue_t*)Q, (queue_node_t*)&queue_node);

  return pQueueNode;
}

global_packet_queue_node_t* Enqueue_Packet_Into_GlobalPacketQueue_With_TargetPoint_Recomputation_Schedule(parameter_t *param, double current_time, global_packet_queue_t *Q, packet_queue_node_t *packet)
{ //enqueue packet into global packet queue Q along with the schedule of target point recomputation
  global_packet_queue_node_t queue_node; //global packet queue node
  global_packet_queue_node_t *pQueueNode = NULL; //pointer to a global packet queue node

  /** initialize global packet queue node with packet */
  memset(&queue_node, 0, sizeof(queue_node));

  queue_node.id = packet->id;
  queue_node.state = packet->state;
  queue_node.state_time = packet->state_time;
  queue_node.packet = packet;

  /** EDD information */
  queue_node.EDD = packet->expected_delivery_delay;
  queue_node.EDD_SD = packet->expected_delivery_delay_standard_deviation;

  /** Target Point */
  queue_node.target_point_id = packet->target_point_id;

  /** Target Point Recomputation Information */
  queue_node.target_point_recomputation_interval = packet->target_point_recomputation_interval;
  queue_node.target_point_recomputation_time = packet->target_point_recomputation_time;

  /** packet_copy_count */
  queue_node.packet_copy_count++;

  /** enqueue queue_node into queue Q */
  pQueueNode = (global_packet_queue_node_t*)Enqueue((queue_t*)Q, (queue_node_t*)&queue_node);

  /** schedule the event of packet where vanet_target_point_selection_type is PROGRESS */
  if(param->vehicle_vanet_target_point_selection_type == VANET_TARGET_POINT_SELECTION_PROGRESS_TARGET_POINT)
  {
    schedule(PACKET_RECOMPUTE_TARGET_POINT, pQueueNode->target_point_recomputation_interval, pQueueNode->id);
  }

  return pQueueNode;
}

void DestroyGlobalPacketQueueNode(global_packet_queue_node_t *global_packet)
{ //dequeue global packet queue node, destroy global packet queue node, and then free the memory of global packet queue node
	global_packet_queue_t *Q = global_packet->ptr_queue; //pointer to global packet queue

	/*@ for debugging */
	//if(global_packet->id ==2423)
	//  printf("DestroyGlobalPacketQueueNode(): for debug, global_packet->id\n", global_packet->id);
	/******************/

	/* decrease global_packet's packet_copy_count */
	global_packet->packet_copy_count--;

	/* if all of the packet copies become deleted, dequeue the global packet queue node */
	if(global_packet->packet_copy_count == 0)
	{
		/* dequeue global_packet from global packet queue Q */
		Dequeue_With_QueueNodePointer((queue_t*)Q, (queue_node_t*)global_packet);

		/* free the memory of the global packet queue node */
		free(global_packet);
	}
}

void ReverseQueue(queue_t *Q)
{ //reverse the order of queue Q, such as packet_trajectory_queue.

  queue_node_t *original_queue_front = NULL; //queue's front in terms of the shortest path
  queue_node_t *p = NULL, *q = NULL; //pointers to queue nodes
  int size = Q->size; //size of queue Q
  int count = 0; //count

  if(Q->size == 0)
      return;

  /* let original queue front point to the last element that is the packet source intersection */
  original_queue_front = Q->head.prev;

  /* reset the information of queue Q for the reversing process */
  Q->head.next = Q->head.prev = &(Q->head);
  Q->size = 0;

  /* reverse the order of the queue list */
  q = original_queue_front; //q points to the first queue node that is the packet source
  do
  {
      p = q; //p is the queue node to be enqueued this time
      q = q->prev; //q is the queue node to be enqueued next time 
      Enqueue_With_QueueNodePointer(Q, p); //enqueue queue node pointer p into queue Q without allocating a new queue node
      count++;
  } while(count < size);
}

void ReversePacketTrajectoryQueue(packet_trajectory_queue_t *Q, struct_graph_node* G, int G_size)
{ //reverse the order of packet trajectory queue Q and set up the distance from the packet source to the vertex

  packet_trajectory_queue_node_t *original_queue_front = NULL; //queue's front in terms of the shortest path
  packet_trajectory_queue_node_t *p = NULL, *q = NULL; //pointers to queue nodes
  int size = Q->size; //size of queue Q
  int count = 0; //count
  double distance = 0; //distance from the packet source to the current vertex
  char tail_vertex[NAME_SIZE]; //tail of an edge
  char head_vertex[NAME_SIZE]; //head of an edge
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to a directional edge queue node

  if(Q->size == 0)
      return;

  /* let original queue front point to the last element that is the packet source intersection */
  original_queue_front = Q->head.prev;

  /* reset the information of queue Q for the reversing process */
  Q->head.next = Q->head.prev = &(Q->head);
  Q->size = 0;

  /* reverse the order of the queue list */
  q = original_queue_front; //q points to the first queue node that is the packet source
  do
  {
      p = q; //p is the queue node to be enqueued this time
      q = q->prev; //q is the queue node to be enqueued next time 

      /* get the tail vertex and the head vertex */
      if(count == 0)
      {
          /* get the tail vertex */
          strcpy(tail_vertex, p->gnode->vertex);
      }
      else
      {
          /* get the head vertex */
          strcpy(head_vertex, p->gnode->vertex);

          /* get the pointer to a directional edge queue node corresponding to (tail_vertex, head_vertex) */
          pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_vertex, head_vertex);
          if(pEdgeNode == NULL)
          {
              printf("ReversePacketTrajectoryQueue(): there is no vaild edge for (%s, %s)\n", tail_vertex, head_vertex);
              exit(1);
          }
          
          /* update distance with the edge's weight */
          distance += pEdgeNode->weight;

          /* update the tail vertex */
          strcpy(tail_vertex, head_vertex);
      }

      /* set the packet trajectory queue node's distance to distance */
      p->distance = distance;

      Enqueue_With_QueueNodePointer((queue_t*)Q, (queue_node_t*)p); //enqueue queue node pointer p into queue Q without allocating a new queue node
      count++;
  } while(count < size);
}

/** Operations for Stationary Node Queue */
void InitStationaryNodeQueue(stationary_node_queue_t *SNQ, struct_graph_node *Gr, int Gr_size)
{ //initialize the stationary node queue SNQ along with the association with the road network graph
    int i = 0; //for-loop index
    struct_graph_node *ptr = NULL; //pointer to a graph node
    stationary_node_queue_node_t qnode; //stationary node queue node 
    stationary_node_queue_node_t *pQueueNode; //stationary node queue node 

    /** initialize the stationary node queue */
    InitQueue((queue_t*) SNQ, QTYPE_STATIONARY_NODE);

    /** construct the stationary node queue */
    for(i = 0; i < Gr_size; i++)
    {
        /* enqueue a stationary node queue node */
        memset(&qnode, 0, sizeof(qnode));
        qnode.intersection_id = atoi(Gr[i].vertex);
        qnode.gnode = &(Gr[i]);

        pQueueNode = (stationary_node_queue_node_t*)Enqueue((queue_t*)SNQ, (queue_node_t*)&qnode);

        /* let the graph node Gr[i] point to the stationary node */
        Gr[i].ptr_stationary_node = pQueueNode;
    }
}

void Update_DestinationVehicle_PassingTime_At_StationaryNode(parameter_t *param, double current_time, struct_vehicle_t *vehicle, char *intersection_vertex)
{ //update the destination vehicle's latest passing time for the intersection vertex
    int intersection_id = atoi(intersection_vertex);
    stationary_node_queue_node_t *stationary_node = NULL; //pointer to the stationary node
    

    /** get the stationary node for intersection_id */
    if(intersection_id > param->vanet_table.Gr_size)
    {
        printf("Update_DestinationVehicle_PassingTime_At_StationaryNode(): Error: intersection_id(%d) > param->vanet_table.Gr_size(%d)\n", intersection_id, param->vanet_table.Gr_size);
        exit(1);
    }
    stationary_node = param->vanet_table.Gr[intersection_id-1].ptr_stationary_node;

    /** update the latest passing time for the destination vehicle */
    stationary_node->destination_vehicle_latest_passing_time = current_time;
}

/** Operations for Probability and Statistics Queue */
void RegisterProbabilityAndStatistics_Per_DirectionalEdge(parameter_t *param, double current_time, struct_graph_node *Gr, int Gr_size, directional_edge_queue_t *DEr)
{ //register the forwarding probability and statistics per directional edge with the corresponding directional edge queue node with the road network graph Gr
    int i = 0; //for-loop index
    int size = DEr->size; //size of the directional edge queue DEr
    directional_edge_queue_node_t *ptr = NULL; //pointer to a directional edge queue node
    probability_and_statistics_queue_t *PSQ = NULL; //pointer to probability-and-statistics queue
    struct_graph_node *tail_gnode = NULL; //pointer to the graph node for the tail node of an edge
    struct_graph_node *head_gnode = NULL; //pointer to the graph node for the head node of an edge
    probability_and_statistics_queue_node_t qnode; //probability-and-statistics queue node
    probability_and_statistics_queue_node_t *pPSQ_QueueNode = NULL; //pointer to a probability-and-statistics queue node

    /** construct the probability-and-statistics queue */
    ptr = &(DEr->head);
    for(i = 0; i < size; i++)
    {
        /* move to the next edge node */
        ptr = ptr->next;

        /* get the pointers to tail graph node and head graph node */
        tail_gnode = ptr->tail_gnode;
        head_gnode = ptr->head_gnode;

        /* get the pointer to a probability-and-statistics queue */
        PSQ = &(ptr->probability_and_statistics_queue); //pointer to the probability-and-statistics queue

        /* copy the forwarding probability and statistics of head gnode into qnode */
        memset(&qnode, 0, sizeof(qnode));
        
        qnode.timestamp = current_time;
        qnode.mean_interarrival_time = head_gnode->mean_interarrival_time;
        qnode.lambda = head_gnode->lambda;
        qnode.contact_probability = head_gnode->CP;
        qnode.forwarding_probability = head_gnode->P_prime;
        qnode.branch_probability = head_gnode->Q;
        qnode.average_forwarding_probability = head_gnode->P;
        qnode.EDD = head_gnode->EDD;
        qnode.EDD_SD = head_gnode->EDD_SD;

        /* enqueue qnode into Q */
        pPSQ_QueueNode = (probability_and_statistics_queue_node_t*) Enqueue((queue_t*)PSQ, (queue_node_t*)&qnode);

        /* copy the conditional forwarding probability queue nodes of queue Q_src to conditional forwarding probability queue Q_dst*/
        CopyConditionalForwardingProbabilityQueueNodes_For_DirectionalEdge(head_gnode->conditional_forwarding_probability_queue, &(pPSQ_QueueNode->conditional_forwarding_probability_queue));
    }
}

void ConstructConditionalForwardingProbabilityQueueNodes_Per_DirectionalEdge(struct_graph_node *G, int G_size)
{ //construct as many conditional probability queue nodes as the neighboring edges per the edge where G[i].vertex is the tail node
  struct_graph_node *tail_gnode = NULL; //pointer to tail graph node
  struct_graph_node *head_gnode = NULL; //pointer to head graph node
  struct_graph_node *neighbor_gnode = NULL; //pointer to a neighboring graph node
  int neighbor_number = 0; //number of neighboring nodes
  conditional_forwarding_probability_queue_node_t qnode; //conditional forwarding probability queue node
  int i = 0, j= 0; //for-loop indices

  for(i = 0; i < G_size; i++)
  {
    tail_gnode = &(G[i]);
    neighbor_number = (int)(tail_gnode->weight);
    neighbor_gnode = tail_gnode;
    for(j = 0; j < neighbor_number; j++)
    {
      neighbor_gnode = neighbor_gnode->next;

      /* make the conditional forwarding probability queue nodes for the edge of (tail_gnode, head_gnode) */
      head_gnode = neighbor_gnode;
      MakeConditionalForwardingProbabilityQueue_For_DirectionalEdge(tail_gnode, head_gnode);
    }
  }
}

void MakeConditionalForwardingProbabilityQueue_For_DirectionalEdge(struct_graph_node *tail_gnode, struct_graph_node *head_gnode)
{ //make the conditional forwarding probability queue for the edge of (tail_gnode, head_gnode)
  struct_graph_node *neighbor_gnode = NULL; //pointer to a neighboring graph node
  int neighbor_number = (int)tail_gnode->weight; //number of neighboring nodes
  conditional_forwarding_probability_queue_t *Q = NULL; //the pointer to the conditional forwarding probability queue
  //conditional_forwarding_probability_queue_t *Q = head_gnode->conditional_forwarding_probability_queue; //the pointer to the conditional forwarding probability queue
  conditional_forwarding_probability_queue_node_t qnode; //conditional forwarding probability queue node
  int i = 0; //for-loop indices

  /* make a conditional forwarding probability queue for the graph node for head_gnode */
  Q = head_gnode->conditional_forwarding_probability_queue = (conditional_forwarding_probability_queue_t*) calloc(1, sizeof(conditional_forwarding_probability_queue_t));
  assert_memory(Q);
  InitQueue((queue_t*)Q, QTYPE_CONDITIONAL_FORWARDING_PROBABILITY);

  /* make placeholders for conditional forwarding probability queue nodes; 
     Note that the fields are set later by the function of CopyConditionalForwardingProbabilityQueueNodes_For_DirectionalEdge() */
  neighbor_gnode = tail_gnode;  
  for(i = 0; i < neighbor_number; i++)
  {
    neighbor_gnode = neighbor_gnode->next;
    
    /** set up a conditional forwarding probability queue node */
    memset(&qnode, 0, sizeof(qnode));

    /* set up the current carrier's edge information */
    strcpy(qnode.tail_node_of_current_carrier_edge, tail_gnode->vertex);
    strcpy(qnode.head_node_of_current_carrier_edge, head_gnode->vertex);

    /* set up the next carrier's edge information */
    strcpy(qnode.tail_node_of_next_carrier_edge, tail_gnode->vertex);
    strcpy(qnode.head_node_of_next_carrier_edge, neighbor_gnode->vertex);

    /* set the conditional forwarding probability to 0 */
    qnode.conditional_forwarding_probability = 0;

    /* enqueue the queue node gnode into queue Q */
    Enqueue((queue_t*)Q, (queue_node_t*)&qnode);
  }
}

void CopyConditionalForwardingProbabilityQueueNodes_For_DirectionalEdge(conditional_forwarding_probability_queue_t *Q_src, conditional_forwarding_probability_queue_t *Q_dst)
{ //copy the conditional forwarding probability queue nodes of queue Q_src to conditional forwarding probability queue Q_dst
  int size = Q_src->size; //size of the conditional forwarding probability queue
  conditional_forwarding_probability_queue_node_t qnode; //conditional forwarding probability queue node
  conditional_forwarding_probability_queue_node_t *pQueueNode = NULL; //pointer to a conditional forwarding probability queue node
  int i = 0; //for-loop indices

  /* delete Q_dst if Q_dst is not empty */
  if(Q_dst->size > 0)
    DestroyQueue((queue_t*)Q_dst);

  pQueueNode = &(Q_src->head);
  for(i = 0; i < size; i++)
  {
    pQueueNode = pQueueNode->next;
    
    /** set up a conditional forwarding probability queue node */
    memset(&qnode, 0, sizeof(qnode));

    /* set up the current carrier's edge information */
    strcpy(qnode.tail_node_of_current_carrier_edge, pQueueNode->tail_node_of_current_carrier_edge);
    strcpy(qnode.head_node_of_current_carrier_edge, pQueueNode->head_node_of_current_carrier_edge);

    /* set up the next carrier's edge information */
    strcpy(qnode.tail_node_of_next_carrier_edge, pQueueNode->tail_node_of_next_carrier_edge);
    strcpy(qnode.head_node_of_next_carrier_edge, pQueueNode->head_node_of_next_carrier_edge);

    /* copy the conditional forwarding probability */
    qnode.conditional_forwarding_probability = pQueueNode->conditional_forwarding_probability;
    
    /* enqueue the queue node gnode into queue Q_dst */
    Enqueue((queue_t*)Q_dst, (queue_node_t*)&qnode);
  }
}

conditional_forwarding_probability_queue_node_t* GetConditionalForwardingProbabilityQueueNode(conditional_forwarding_probability_queue_t *Q, char *tail_node, char *head_node)
{ //get the pointer to a conditional forwarding probability queue node corresponding to the edge (tail_node, head_node) given the current carrier edge with the current carrier edge's conditional forwarding probability queue Q 
  conditional_forwarding_probability_queue_node_t *pQueueNode = NULL; //pointer to the conditional forwarding probability queue node corresponding to the edge (tail_node, head_node)
  conditional_forwarding_probability_queue_node_t *ptr = NULL; //pointer to a conditional forwarding probability queue node
  int size = Q->size; //queue size
  int i = 0; //for-loop index

  ptr = &(Q->head);
  for(i = 0; i < size; i++)
  {
    ptr = ptr->next;

    if(strcmp(ptr->head_node_of_next_carrier_edge, head_node) == 0 && strcmp(ptr->tail_node_of_next_carrier_edge, tail_node) == 0)
    {
      pQueueNode  = ptr;
      break;
    }
  }

  if(pQueueNode == NULL)
  {
    printf("GetConditionalForwardingProbabilityQueueNode(): there is no conditional forwarding probability queue node corresponding to the edge (%s,%s)\n", tail_node, head_node);
    exit(1);
  }
  
  return pQueueNode;
}

void Copy_Vehicle_Trajectory(parameter_t *param, double current_time, packet_queue_node_t *SrcPacket, packet_queue_node_t *DstPacket)
{ //copy SrcPacket's vehicle trajectory into DstPacket's
    vehicle_trajectory_queue_node_t *pQueueNode = NULL; //pointer to the vehicle trajectory queue node
    vehicle_trajectory_queue_node_t *pEnqueuedQueueNode = NULL; //pointer to the vehicle trajectory queue node enqueued into the vehicle trajectory
    vehicle_trajectory_queue_t *pSrcQueue = &(SrcPacket->vehicle_trajectory); //pointer to the SrcPacket's vehicle trajectory queue
    vehicle_trajectory_queue_t *pDstQueue = &(DstPacket->vehicle_trajectory); //pointer to the DstPacket's vehicle trajectory queue
    int size = pSrcQueue->size; //size of SrcPacket's vehicle trajectory queue
    int i = 0; //for-loop index
    int current_order = pSrcQueue->current_order; //the trajectory queue node in the current order to consider this time in SrcQueue
    vehicle_trajectory_queue_node_t *current_order_qnode = NULL; //pointer to the trajectory queue node in the current order in SrcQueue

    /** destroy the DstPacket's vehicle trajectory */
    if(pDstQueue->size > 0)
        DestroyQueue((queue_t*) pDstQueue);

    /** copy queue nodes */
    pQueueNode = &(pSrcQueue->head);
    for(i = 0; i < size; i++)
    {
        pQueueNode = pQueueNode->next;

        pEnqueuedQueueNode = (vehicle_trajectory_queue_node_t*) Enqueue((queue_t*)pDstQueue, (queue_node_t*)pQueueNode);

        /* find the queue node for current_order */
        if(i == current_order)
            current_order_qnode = pEnqueuedQueueNode;
    }

    /** copy queue information */
    pDstQueue->trajectory_type = pSrcQueue->trajectory_type;
    pDstQueue->current_order = pSrcQueue->current_order;
    pDstQueue->current_order_qnode = current_order_qnode;
    pDstQueue->vnode = pSrcQueue->vnode;
    pDstQueue->vehicle_speed = pSrcQueue->vehicle_speed;
    pDstQueue->vehicle_speed_standard_deviation = pSrcQueue->vehicle_speed_standard_deviation;
    pDstQueue->vehicle_pos_register_time = pSrcQueue->vehicle_pos_register_time;

    memcpy(&(pDstQueue->vehicle_graph_pos), &(pSrcQueue->vehicle_graph_pos), sizeof(pDstQueue->vehicle_graph_pos));
    memcpy(&(pDstQueue->vehicle_euclidean_pos), &(pSrcQueue->vehicle_euclidean_pos), sizeof(pDstQueue->vehicle_euclidean_pos));

    pDstQueue->target_point_id = pSrcQueue->target_point_id;
    pDstQueue->target_point_gnode = pSrcQueue->target_point_gnode;
}

void Copy_Packet_Trajectory(parameter_t *param, double current_time, packet_queue_node_t *SrcPacket, packet_queue_node_t *DstPacket)
{ //copy SrcPacket's packet trajectory into DstPacket's
    packet_trajectory_queue_node_t *pQueueNode = NULL; //pointer to the packet trajectory queue node
    packet_trajectory_queue_node_t *pEnqueuedQueueNode = NULL; //pointer to the packet trajectory queue node enqueued into the packet trajectory
    packet_trajectory_queue_t *pSrcQueue = &(SrcPacket->packet_trajectory); //pointer to the SrcPacket's packet trajectory queue
    packet_trajectory_queue_t *pDstQueue = &(DstPacket->packet_trajectory); //pointer to the DstPacket's packet trajectory queue
    int size = pSrcQueue->size; //size of SrcPacket's packet trajectory queue
    int i = 0; //for-loop index
    packet_trajectory_queue_node_t *current_packet_position = NULL; //pointer to the packet trajectory queue node of the intersection corresponding to the tail node of the edge where packet is placed
    int current_packet_intersection_id = pSrcQueue->current_packet_intersection_id; //the packet's current intersection id
    int order = pSrcQueue->order; //the order of the packet trajectory queue node corresponding t

    /** destroy the DstPacket's packet trajectory */
    if(pDstQueue->size > 0)
        DestroyQueue((queue_t*) pDstQueue);

    /** copy queue nodes */
    pQueueNode = &(pSrcQueue->head);
    for(i = 0; i < size; i++)
    {
        pQueueNode = pQueueNode->next;

        pEnqueuedQueueNode = (packet_trajectory_queue_node_t*) Enqueue((queue_t*)pDstQueue, (queue_node_t*)pQueueNode);

        /* find the queue node for order */
        if(i == order)
            current_packet_position = pEnqueuedQueueNode;
    }

    /** copy queue information */
    pDstQueue->current_packet_position = current_packet_position;
    pDstQueue->current_packet_intersection_id = pSrcQueue->current_packet_intersection_id;
    pDstQueue->order =  pSrcQueue->order;
}

void ReplaceTargetPointQueue(target_point_queue_t *dst_Q, target_point_queue_t *src_Q)
{//replace target point list dst_Q with target point list src_Q by letting dst_Q have target point queue nodes of src_Q without allocating new target point queue nodes and copying them
  target_point_queue_node_t *dst_head = &(dst_Q->head);
  target_point_queue_node_t *src_head = &(src_Q->head);

  /** delete target_point_queue dst_Q */
  if(dst_Q->size > 0)
  {
    DestroyQueue((queue_t*)dst_Q); //destroy the queue dst_Q, and then initialize it by calling InitQueue() within the function
  }

  if(src_Q->size == 0)
    return;

  /** put the target point list of src_Q in dst_Q by adjusting pointers to target point queue nodes */
  dst_head->next = src_head->next;
  src_head->next->prev = dst_head;

  dst_head->prev = src_head->prev;
  src_head->prev->next = dst_head;

  /** set dst_Q->size tp src_Q->size */
  dst_Q->size = src_Q->size;

  /** set src_Q's size to zero and let src_Q.head's pointers point to src_Q.head itself */
  src_Q->size = 0;
  src_head->next = src_head;
  src_head->prev = src_head;
}

void Initialize_GlobalPacketQueue_PacketVectors(parameter_t *param, global_packet_queue_t* Q, int vector_size)
{ //initialize packet vectors for packet delivery statistics under multiple packet copy transmission for multiple target points

	/** initialize packet vectors */
	/* allocate the memory of packet_id_bitmap_vector */
	Q->packet_id_bitmap_vector = (boolean*)calloc(vector_size, sizeof(boolean));
	assert_memory(Q->packet_id_bitmap_vector);

	/* allocate the memory of packet_delivery_delay_vector */
	Q->packet_delivery_delay_vector = (double*)calloc(vector_size, sizeof(double));
	assert_memory(Q->packet_delivery_delay_vector);

	/* allocate the memory of packet_deletion_count_vector */
	Q->packet_deletion_count_vector = (int*)calloc(vector_size, sizeof(int));
	assert_memory(Q->packet_deletion_count_vector);

	/* set Q's physical vector size to vector_size */
	Q->physical_vector_size = vector_size; 

	/* set Q's actual vector size to 0 */
	Q->actual_vector_size = 0; 

	/* set packet copy number according to data_forwarding_multiple_target_point_flag */
	if(param->data_forwarding_multiple_target_point_flag)
	{ //@multi-target-point forwarding
		/* set packet copy number to data_forwarding_maximum_target_point_number */
		Q->packet_copy_number = param->data_forwarding_maximum_target_point_number; 
	}
	else
	{//@single-target-point forwarding
		/* set packet copy number to 1 */
		Q->packet_copy_number = 1; 
	}
}

void Reallocate_GlobalPacketQueue_PacketVectors(global_packet_queue_t* Q, int vector_increase_size)
{ //increase the memory of packet vectors by vector_increase_size to accommodate more packets
	size_t mem_size = 0; //memory size

	/* reallocate the memory of packet_id_bitmap_vector */
	mem_size = Q->physical_vector_size*sizeof(boolean); 
	Q->packet_id_bitmap_vector = (boolean*)realloc(Q->packet_id_bitmap_vector, mem_size + sizeof(boolean)*vector_increase_size); //reallocate the memory
	assert_memory(Q->packet_id_bitmap_vector);

	/* reallocate the memory of packet_delivery_delay_vector */
	mem_size = Q->physical_vector_size*sizeof(double); 
	Q->packet_delivery_delay_vector = (double*)realloc(Q->packet_delivery_delay_vector, mem_size + sizeof(double)*vector_increase_size); //reallocate the memory
	assert_memory(Q->packet_delivery_delay_vector);

	/* reallocate the memory of packet_deletion_count_vector */
	mem_size = Q->physical_vector_size*sizeof(int); 
	Q->packet_deletion_count_vector = (int*)realloc(Q->packet_deletion_count_vector, mem_size + sizeof(int)*vector_increase_size); //reallocate the memory
	assert_memory(Q->packet_deletion_count_vector);

	/* increase the number of vector entries */
	Q->physical_vector_size += vector_increase_size; 
}

/** Queue Operations for TPD */
boolean Delete_Edge_In_NeighborList(adjacency_list_queue_node_t *u,
		adjacency_list_queue_node_t *v)
{ //delete the edge (u, v) from u's neighbor_list 
	neighbor_list_queue_t *Q = NULL; //pointer to a neighbor_list queue
	neighbor_list_queue_node_t *p = NULL; //pointer to a neighbor_list queue node called p
	int i = 0; //loop-index
	boolean flag = FALSE; //flag to indicate whether the deletion of edge (u, v) is successful or not

	/* check the validity of u and v */
	if(u == NULL)
	{
		printf("%s:%d u is NULL.\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}
	else if(v == NULL)
	{
		printf("%s:%d v is NULL.\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}

	Q = &(u->neighbor_list);
	p = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		p = p->next;
		if(p->id == v->id)
		{
			/* link p's previous node and p's next node with each other */
			p->prev->next = p->next;
			p->next->prev = p->prev;

			/* delete the queue node pointed by p */
			DestroyQueueNode(Q->type, (queue_node_t*)p);
			Q->size--;
			flag = TRUE;
			break;
		}
	}

	return flag;
}

boolean Delete_Node_In_AdjacencyList(adjacency_list_queue_node_t *v)
{ //delete the graph node v from v's queue that is an adjacency list 
	adjacency_list_queue_t *Q = NULL; //pointer to a neighbor_list queue

	/* check the validity of v and v's ptr_queue */
	if(v == NULL)
	{
		printf("%s:%d v is NULL.\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}
	else if(v->ptr_queue == NULL)
	{
		printf("%s:%d v->ptr_queue is NULL.\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}
	else
	{
		Q = v->ptr_queue;
	}

	/* link v's previous node and v's next node with each other */
	if((v->next == NULL) || (v->prev == NULL) || 
			(v->prev->next == NULL) || (v->next->prev == NULL))
	{
		printf("%s:%d one of v's pointers is NULL.\n",
				__FUNCTION__, __LINE__);
		return FALSE;
	}
	else
	{
		v->prev->next = v->next;
		v->next->prev = v->prev;
	}

	/* delete the queue node pointed by v */
	DestroyQueueNode(Q->type, (queue_node_t*)v);
	Q->size--;

	return TRUE;
}

int ConstructRoadNetworkGraphSet_And_DirectionalEdgeQueueSet_With_SetAssociation(
			parameter_t *param,
			struct_graph_node *Gr, 
			int Gr_size,
			int *pGr_set_number,
			struct_graph_node ***pGr_set, 
			int **pGr_set_size, 
			directional_edge_queue_t **pDEr_set)
{/* construct the set of road network graphs and the set of the corresponding directional edge queues, and then associate these two sets */
	int number = Gr_size; //the number of intersections in the road network graph Gr that will be the number of road network graphs in *pGr_set
	int i = 0; //loop-index

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

	/* if *pGr_set is not NULL, release the memory for the existing *pGr_set */
	if(*pGr_set != NULL)
	{
        printf("%s:%d *pGr_set is not NULL!\n",
                __FUNCTION__, __LINE__);

		if(*pGr_set_size == NULL)
		{
			printf("%s:%d *pGr_set_size is NULL!\n",
					__FUNCTION__, __LINE__);
			exit(1);
		}

		for(i = 0; i < number; i++)
		{
			Free_Graph((*pGr_set)[i], (*pGr_set_size)[i]); //release the memory allocated to graph (*pGr_set)[i]
		}

		free(*pGr_set); //free the allocated memory for *pGr_set
		free(*pGr_set_size); //free the allocated memory for *pGr_set_size
	}

	/* if *pDEr_set is not NULL, release the memory for the existing *pDEr_set */
	if(*pDEr_set != NULL)
	{
		printf("%s:%d *pDEr_set is not NULL!\n",
				__FUNCTION__, __LINE__);

		for(i = 0; i < number; i++)
		{
			DestroyQueue((queue_t*) &((*pDEr_set)[i])); //destory directional edge queue (*pDEr_set)[i]
		}

		free(*pDEr_set); //free the allocated memory for *pDEr_set
	}

	/* set pGr_set_number to number as the number of road network graphs in *pGr_set */
	*pGr_set_number = number;

	/* allocate the memory for *pGr_set */
	*pGr_set = (struct_graph_node**) calloc(number, sizeof(struct_graph_node*)); //allocate the memory for the set of road network graphs
	assert_memory(*pGr_set);

	/* allocate the memory for *pGr_set_size */
	*pGr_set_size = (int*) calloc(number, sizeof(int)); //allocate the memory for the size array for the graphs
	assert_memory(*pGr_set_size);

	/* allocate the memory for *pDEr_set */
	*pDEr_set = (directional_edge_queue_t*) calloc(number, sizeof(directional_edge_queue_t)); //allocate the memory for the set of directional edge queues
	assert_memory(*pDEr_set);

	/* for each index for an intersection, construct a road network graph and the corresponding directional edge queue, and then associate the road network graph with the directional edge queue */
	for(i = 0; i < number; i++)
	{
		(*pGr_set)[i] = Make_Forwarding_Graph(Gr, Gr_size, &((*pGr_set_size)[i]));
		//make a new forwarding graph (*pGr_set)[i] for data forwarding for the intersection (i+1)

		ConstructDirectionalEdgeQueue(param, &((*pDEr_set)[i]), (*pGr_set)[i], (*pGr_set_size)[i]);
		//construct a directional edge queue (*pDEr_set[i]) containing the information of directional edges in graph (*pGr_set)[i]

		AssociateGraphEdgeWithDirectionalEdgeEntry((*pGr_set)[i], (*pGr_set_size)[i], &((*pDEr_set)[i]));
		//associate each pair of two adjacent graph nodes (i.e., physical edge) with the corresponding directional edge entry in DE_set[i]
	}

	return number;
}

int DestroyRoadNetworkGraphSet_And_DirectionalEdgeQueueSet(
			int Gr_set_number,
			struct_graph_node **Gr_set, 
			int *Gr_set_size, 
			directional_edge_queue_t *DEr_set)
{/* destroy the set of road network graphs and the set of the corresponding directional edge queues */
	int number = Gr_set_number; //the number of road network graphs in Gr_set
	int i = 0; //loop-index

	/* check the validity of Gr_size, Gr_set, Gr_set_size, and DEr_set */
	if(Gr_set_number <= 0)
	{
        printf("%s:%d Gr_set_number(%d) is not positive!\n",
                __FUNCTION__, __LINE__,
				Gr_set_number);
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
		printf("%s:%d *pGr_set_size is NULL!\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}
	if(DEr_set == NULL)
	{
		printf("%s:%d DEr_set is NULL!\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}

	/* release the memory of the road network graphs and directional edge queues */
	for(i = 0; i < number; i++)
	{
		Free_Graph(Gr_set[i], Gr_set_size[i]); //release the memory allocated to graph Gr_set[i]

		DestroyQueue((queue_t*) &(DEr_set[i])); //destory directional edge queue DEr_set[i]
	}

	free(Gr_set); //free the allocated memory for Gr_set
	free(Gr_set_size); //free the allocated memory for Gr_set_size
	free(DEr_set); //free the allocated memory for DEr_set

	return 0;
}

boolean IsPacketCopy_In_PacketQueue(packet_queue_t *Q, packet_queue_node_t *pPacketNode)
{ //check whether the packet copy pointed by pPacketNode is already in packet queue Q 
	boolean flag = FALSE;
	packet_queue_node_t *pQueueNode = NULL; //pointer to a packet queue node
	int i = 0; //loop index

	pQueueNode = &(Q->head);
	for(i = 0; i < Q->size; i++)
	{
		pQueueNode = pQueueNode->next;
		if(pPacketNode->id == pQueueNode->id)
		{
			flag = TRUE;
			break;
		}
	}

	return flag;
}

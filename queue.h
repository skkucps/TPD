/**
 *  File: queue.h
 *	Description: operations for queue that implements the FIFO
 *	Origin Date: 08/21/2007
 *	Update Date: 07/15/2013
 *	Maker: Jaehoon Jeong
 */

#ifndef __QUEUE_H__
#define __QUEUE_H__

#include "common.h"
#include "sensor-model.h"
#include "vehicle-model.h"
#include "param.h"

/** Note1: whenever we support another queue type, we need to add another constant for it */
typedef enum _queue_type_t {
	QTYPE_NONE = 0,
	QTYPE_SCHEDULE = 1,      //schedule queue for sensing scheduling
	QTYPE_SENSOR = 2,        //sensor queue for sensors deployed on an edge corresponding to a road segment
	QTYPE_PATH = 3,          //path queue for nodes on the shortest path from source to destination
	QTYPE_EDGE = 4,          //edge queue containing unique edges of the adjacency list for the road network graph
	QTYPE_SUBEDGE = 5,       //subedge queue containing unique edges of the adjacency list for the subdivision graph of the road network graph
	QTYPE_LOCATION = 6,      //sensor location queue containing sensor location order along with the index of sensor table S that is used to access the actual sensor information
	QTYPE_HOLE_SEGMENT = 7,  //sensing hole segment queue containing sensing holes for each edge in real graph Gr
	QTYPE_HOLE_ENDPOINT = 8, //sensing hole segment queue containing sensing holes for each edge in real graph Gr
	QTYPE_VERTEX_SET = 9,    //vertex set queue containing vertices in virtual topology Tv
	QTYPE_EDGE_SET = 10,     //edge set queue containing edges in virtual topoloy Tv
	QTYPE_ANGLE = 11,        //angle queue containing the angle between the vehicle movement vector and the destination vector for computing the forwarding probability P_ij in VADD
	QTYPE_DIRECTIONAL_EDGE = 12,   //edge queue for directional edges corresponding to road segments in VANET
	QTYPE_DELAY = 13,        //delay queue for Expected Delivery Delay (EDD) in VANET
	QTYPE_DELAY_COMPONENT = 14, //delay component queue for delay components for each EDD in VANET
	QTYPE_PACKET = 15,          //packet used in VANET
	QTYPE_PACKET_SCHEDULE = 16, //schedule queue for packet generation
	QTYPE_VEHICLE = 17, //vehicle queue for pointers to vehicle nodes moving on a directional edge
	QTYPE_VEHICLE_MOVEMENT = 18,//vehicle movement queue for packet forwarding on road segment
	QTYPE_INTERSECTION_EDD = 19, //intersection EDD queue containing the pointers to the graph nodes corresponding to edges incident to the intersection in the ascending order of EDD values
	QTYPE_CONVOY = 20, //convoy queue for convoys moving on a direcional edge
	QTYPE_MOBILITY = 21, //mobility queue for the trajectory of destination vehicle(s)
	QTYPE_DESTINATION_VEHICLE = 22, //destination vehicle queue for destination vehicles to receive packets from AP
	QTYPE_ACCESS_POINT = 23, //access point queue for Internet access points
	QTYPE_VEHICLE_TRAJECTORY = 24, //vehicle trajectory queue to estimate the destination vehicle position according to time
	QTYPE_CARRIER_TRACE = 25, //queue for packet carrier trace from packet source to packet destination
	QTYPE_FORWARDING_TABLE = 26, //queue for STBD forwarding tables that are an adjacency list of road network graph for a specific target point or access point
	QTYPE_GLOBAL_PACKET = 27, //global packet queue for pointers to the valid packets whose TTLs do not expire in VANET
	QTYPE_STATIONARY_NODE = 28, //queue of stationary nodes that are attached to intersections as packet buffers
	QTYPE_PACKET_TRAJECTORY = 29, //packet trajectory queue to determine the packet forwarding path
	QTYPE_PROBABILITY_AND_STATISTICS = 30, //queue for the probability and statistics per each edge in the road network
	QTYPE_CONDITIONAL_FORWARDING_PROBABILITY = 31, //queue for the conditional forwarding probability that a packet is forwarded to each incident edge given a vehicle moving edge
	QTYPE_TARGET_POINT = 32, //target point queue containing target points towards which packets will be sent
	QTYPE_MINIMUM_PRIORITY = 33, //minimum priority queue
	QTYPE_NEIGHBOR_LIST = 34, //neighbor list queue of the head vertices for a tail vertex in adjacency list
	QTYPE_PARENT_LIST = 35, //parent list queue of the parent vertices in adjacency list
	QTYPE_ADJACENCY_LIST = 36, //adjacency list queue
	QTYPE_ADJACENCY_LIST_POINTER = 37 //adjacency list pointer queue that contains the pointers to adjacency queue nodes in the adjacency list queue
} queue_type_t;

typedef enum _cluster_type_t {
	CLUSTER_UNKNOWN = 0,    //unknow cluster type
	CLUSTER_ENTRANCE = 1,   //entrance cluster that is a set of entrance nodes
	CLUSTER_PROTECTION = 2, //protection cluster that is a set of protection nodes
	CLUSTER_HOLE = 3        //hole cluster that is a set of sensing hole nodes
} cluster_type_t;

/** enum type of the object pointed by the queue node */
typedef enum _queue_node_object_type_t{
	QUEUE_NODE_OBJECT_UNKNOWN = 0, //unknown object type
	QUEUE_NODE_OBJECT_VEHICLE = 1 //vehicle model object
} queue_node_object_type_t;

/** Note2: whenever we support another queue type, we need to define the specific structure 
of the queue node for the new queue type: 
  - next and prev must be the first two members for queue node */

/** structure for general queue node */
typedef struct _queue_node_t {
    struct _queue_node_t *next; 
	struct _queue_node_t *prev;
} queue_node_t;

/** structure for schedule queue node */
typedef struct _schedule_queue_node_t {
    struct _schedule_queue_node_t *next; 
	struct _schedule_queue_node_t *prev;
	char tail_node[NAME_SIZE];         //tail node of a directional edge
	char head_node[NAME_SIZE];         //head node of a directional edge
	double arrival_time;   //time when the scan arrives at tail node
	double departure_time; //time when the scan leaves head node
} schedule_queue_node_t;

/** structure for sensor node */
typedef struct _sensor_queue_node_t {
	struct _sensor_queue_node_t *next;
	struct _sensor_queue_node_t *prev;
	struct_sensor_t info; //sensor's information
	int order_in_Gr; //order of sensor in the real graph Gr, starting from 0; that is, the number of previous sensors before this sensor from the tail of the edge
	int order_in_Gv; //order of sensor in the virtual graph Gv, starting from 0; that is, the number of previous sensors before this sensor from the tail of the edge
	int order_for_live_sensors; //order of live sensor in the virtual graph Gv, starting from 0; that is, the number of previous live sensors before this sensor from the tail of the edge 
	struct _sensor_queue_node_t **pSensorListEntry; //pointer to the memory of the sensor table entry (pointed by S->list[]) of sensor table S containing all of the sensors; pQueueNode->pSensorListEntry points to the memory cell to point to pQueueNode in order to access to the sensor queue node.
	struct _edge_queue_node_t *pEdgeQueueNode; //pointer to the edge queue node corresponding to the edge where the sensor is located

} sensor_queue_node_t;

/** structure for path node */
typedef struct _path_queue_node_t {
	struct _path_queue_node_t *next;
	struct _path_queue_node_t *prev;
	char node[NAME_SIZE]; //path node
} path_queue_node_t;

/** structure for location node */
typedef struct _location_queue_node_t {
        struct _location_queue_node_t *next; 
	struct _location_queue_node_t *prev;
	int order; //location order in the edge from the tail node of the edge
	int id;    //sensor ID
	struct _sensor_queue_node_t *sensor; //pointer to sensor queue node
} location_queue_node_t;

/** structure for sensing hole segment node */
typedef struct _hole_segment_queue_node_t {
    struct _hole_segment_queue_node_t *next; 
	struct _hole_segment_queue_node_t *prev;
	int order; //location order in the edge from the tail node of the edge
	double left_hole_offset; //offset of left hole in sensing hole area from the tail of the edge
	double right_hole_offset; //offset of right hole in sensing hole area from the tail of the edge
} hole_segment_queue_node_t;

/** structure for sensing hole endpoint node where sensing hole endpoint is one of end-points of a sensing hole segment */
typedef struct _hole_endpoint_queue_node_t {
    struct _hole_endpoint_queue_node_t *next; 
	struct _hole_endpoint_queue_node_t *prev;
	char vertex[NAME_SIZE]; //vertex name for a sensing hole in a virtual graph Gv
	int hid; //sensing hole id based on sequence number of hole endpoint queue, starting from 1
	int order; //location order in the edge from the tail node of the edge
	double offset; //offset in the edge in the real graph Gr
        int eid; //edge id of the virtual edge containing the hole segment having this hole endpoint
	struct _edge_queue_node_t *edge_queue_entry; //pointer to the edge queue entry containing the hole endpoint list for this hole endpoint queue node
} hole_endpoint_queue_node_t;

/** structure for subedge node */
typedef struct _subedge_queue_node_t {
    struct _subedge_queue_node_t *next; 
	struct _subedge_queue_node_t *prev;
	int eid;                    //ID of edge (tail_node, head_node); this indicates the undirectional edge including two directional edges with the same end points of the edge
	int order;                  //order of the subedge in the subdivision of the edge in Er for the real graph Gr
	char tail_node[NAME_SIZE];  //tail node of a directional edge
	char head_node[NAME_SIZE];  //head node of a directional edge
	double weight;              //weight for edge
	struct _schedule_table_node_t *schedule_table_entry; //pointer to the corresponding schedule table entry that is used to locate the table entry
	struct _edge_queue_node_t *edge_queue_entry; //pointer to the edge queue entry containing the subedge list for this subedge queue node
} subedge_queue_node_t;

/** structure for vertex_set node */
typedef struct _vertex_set_queue_node_t {
    struct _vertex_set_queue_node_t *next; 
	struct _vertex_set_queue_node_t *prev;
	char vertex[NAME_SIZE]; //vertex name
	struct _vertex_set_queue_node_t *representative; //pointer to the representive vertex that is used to identify whether two vertices belong to the same set, that is the node just next to the head node
	cluster_type_t cluster_type; //cluster type
	struct _vertex_set_queue_t *cluster_set; //pointer to the cluster set
} vertex_set_queue_node_t;

/** structure for edge_set node */
typedef struct _edge_set_queue_node_t {
    struct _edge_set_queue_node_t *next; 
	struct _edge_set_queue_node_t *prev;
	char tail_node[NAME_SIZE];  //tail node of a directional edge
	char head_node[NAME_SIZE];  //head node of a directional edge
	double weight;              //weight for edge
	struct _edge_set_queue_node_t *representative; //pointer to the representive edge that is used to identify whether two vertices belong to the same set, that is the node just next to the head node
} edge_set_queue_node_t;

/** structure for angle queue node for VADD */
typedef struct _angle_queue_node_t {
        struct _angle_queue_node_t *next; 
	struct _angle_queue_node_t *prev;
	char tail_node[NAME_SIZE];  //tail node of a directional edge
	char head_node[NAME_SIZE];  //head node of a directional edge
        double theta; //angle in degrees between the vehicle movement vector and the destination vector
        double CP; //contacting probability for a packet carrier to meet at least one contact vehicle towards road <tail_node,head_node>, when the carrier moves within the intersection area of tail_node
        struct_graph_node *tail_gnode; //pointer to the graph node corresponding to the tail node in <tail_node, head_node>
        struct_graph_node *head_gnode; //pointer to the graph node corresponding to the head node in <tail_node, head_node>
} angle_queue_node_t;


/** structure for angle queue node for computing the conditional forwarding probability for the incident edges of an edge */
/*
typedef struct _angle_queue_node_t {
        struct _angle_queue_node_t *next; 
	struct _angle_queue_node_t *prev;
	char tail_node[NAME_SIZE];  //tail node of a directional edge
	char head_node[NAME_SIZE];  //head node of a directional edge
        double theta; //angle in degrees between the vehicle movement vector and the destination vector
        double CP; //contacting probability for a packet carrier to meet at least one contact vehicle towards road <tail_node,head_node>, when the carrier moves within the intersection area of tail_node
        double conditional_forwarding_probability; //conditional forwarding probability

        struct_graph_node *tail_gnode; //pointer to the graph node corresponding to the tail node in <tail_node, head_node>
        struct_graph_node *self_gnode; //pointer to the graph node corresponding to the self node in <tail_node, self_node>
        struct_graph_node *neighbor_gnode; //pointer to the graph node corresponding to the neighbor node in <tail_node, neighbor_node>
  //struct_graph_node *gnode; //pointer to the graph node corresponding to the head node in <tail_node, head_node>
} angle_queue_node_t;
*/

//QTYPE_DIRECTIONAL_EDGE = 12,   //edge queue for directional edges corresponding to road segments in VANET
//QTYPE_DELAY = 13,        //delay queue for Expected Delivery Delay (EDD) in VANET
//QTYPE_DELAY_COMPONENT = 14, //delay component queue for delay components for each EDD in VANET


/** structure for delay component queue node for VADD */
typedef struct _delay_component_queue_node_t {
        struct _delay_component_queue_node_t *next; 
	struct _delay_component_queue_node_t *prev;
        int eid; //edge id
        int order; //order according to eid in the increasing order of eid
        double P; //forwarding probability
        double EDD; //expected delivery delay
        struct _directional_edge_queue_node_t *enode; //pointer to the directional edge node corresponding to eid
} delay_component_queue_node_t;

/** structure for vehicle trajectory queue node for a packet destination's trajectory */
typedef struct _vehicle_trajectory_queue_node_t {
        struct _vehicle_trajectory_queue_node_t *next;
        struct _vehicle_trajectory_queue_node_t *prev;
        struct_coordinate3_t graph_pos; //position in road network graph in the format of (eid, offset, enode)
        struct_coordinate1_t euclidean_pos; //position in Euclidean space in the format of (x-coordinate, y-coordinate)       
        double arrival_time; //estimated arrival time of the destination vehicle
        int order; //order of visited landmarks; note that the first's order is zero
        struct _vehicle_trajectory_queue_t *ptr_queue; //pointer to the trajectory queue
} vehicle_trajectory_queue_node_t;

/** structure for vehicle trajectory queue */
typedef struct _vehicle_trajectory_queue_t {
	queue_type_t type; //queue type
	int size;
	vehicle_trajectory_queue_node_t head;

        vanet_vehicle_trajectory_type_t trajectory_type; //vehicle trajectory type
        int current_order; //the trajectory queue node in the current order to consider this time
        vehicle_trajectory_queue_node_t *current_order_qnode; //pointer to the trajectory queue node in the current order

        /* destination vehicle information */
        struct_vehicle_t *vnode; //pointer to the vehicle
        double vehicle_speed; //vehicle speed
        double vehicle_speed_standard_deviation; //vehicle speed standard deviation
        double vehicle_pos_register_time; //register time that vehicle's position is registered
        struct_coordinate3_t vehicle_graph_pos; //position in road network graph in the format of (eid, offset, enode)
        struct_coordinate1_t vehicle_euclidean_pos; //position in Euclidean space in the format of (x-coordinate, y-coordinate)       

        /* target point information */
        int target_point_id; //id of target point determined by AP for the packet-receiving position of the destination vehicle
        struct_graph_node *target_point_gnode; //pointer to the graph node corresponding to target_point_id
} vehicle_trajectory_queue_t;

/** structure for carrier trace queue node for this packet's carrier trace */
typedef struct _carrier_trace_queue_node_t {
        struct _carrier_trace_queue_node_t *next;
        struct _carrier_trace_queue_node_t *prev;

  /** packet information */
        data_forwarding_mode_t data_forwarding_mode; //data forwarding mode = {DATA_FORWARDING_MODE_DOWNLOAD, DATA_FORWARDING_MODE_UPLOAD}

        vanet_node_type_t node_type; //vanet node type = {VANET_NODE_AP, VANET_NODE_VEHICLE, VANET_NODE_SNODE}
  
        int intersection_id; //intersection id

        int carry_src_id; //carrier source id
        int carry_dst_id; //carrier destination id

  /* EDD information for upload forwarding mode */
        double EDD; //EDD from carrier to the AP
        double EDD_SD; //EDD_SD from carrier to the AP
  /************************************************/

  /* EDD information for download forwarding mode */
        double EDD_for_download; //EDD from carrier to the target point
        double EDD_SD_for_download; //EDD_SD from carrier to the target point
  /************************************************/

        struct _access_point_queue_node_t *AP_node; //pointer to the access point
        struct_vehicle_t *vnode; //pointer to the vehicle
        struct_coordinate3_t graph_pos; //position in road network graph in the format of (eid, offset, enode)
        struct_coordinate1_t euclidean_pos; //position in Euclidean space in the format of (x-coordinate, y-coordinate)  
        double receive_time; //packet receive time for this carrier
        int target_point_id; //target point id

  /** destination vehicle information */
        int dst_vid; //destination vehicle id
        struct_coordinate3_t dst_graph_pos; //destination vehicle's position in road network graph in the format of (eid, offset, enode)
        struct_coordinate1_t dst_euclidean_pos; //destination vehicle's position in Euclidean space in the format of (x-coordinate, y-coordinate)  
        
        int order; //order of carriers; note that the first's order is zero
        struct _carrier_trace_queue_t *ptr_queue; //pointer to the carrier trace queue
} carrier_trace_queue_node_t;

/** structure for carrier trace queue */
typedef struct _carrier_trace_queue_t {
	queue_type_t type; //queue type
	int size;
	carrier_trace_queue_node_t head;
} carrier_trace_queue_t;

/** structure for packet trajectory queue node for a packet's forwarding path */
typedef struct _packet_trajectory_queue_node_t {
        struct _packet_trajectory_queue_node_t *next;
        struct _packet_trajectory_queue_node_t *prev;
        int intersection_id; 
        struct_graph_node *gnode; //pointer to the Gr vertex corresponding to intersection_id
        struct_coordinate1_t euclidean_pos; //position in Euclidean space in the format of (x-coordinate, y-coordinate)       
        double distance; //distance from packet source to this vertex
		double edge_length; //the length of the edge between the previous intersection node and this intersection node
		double edge_delay; //link delay of the edge from the previous intersection node to this intersection node
		double edge_delay_variance; //link delay variance of the edge
		double edge_cost; //link cost of the edge from the previous intersection node to this intersection node
		double edge_cost_variance; //link cost variance of the edge
        double arrival_time; //estimated arrival time of the intersection vehicle
        int order; //order of visited intersections; note that the first's order is zero
        struct _packet_trajectory_queue_t *ptr_queue; //pointer to the packet trajectory queue
} packet_trajectory_queue_node_t;

/** structure for packet trajectory queue for a packet's forwarding path */
typedef struct _packet_trajectory_queue_t {
	queue_type_t type; //queue type
	int size;
	packet_trajectory_queue_node_t head;
        packet_trajectory_queue_node_t *current_packet_position; //pointer to the packet trajectory queue node of the intersection corresponding to the tail node of the edge where packet is placed
        int current_packet_intersection_id; //the packet's current intersection id
        int order; //the order of the packet trajectory queue node corresponding to packet_current_position
} packet_trajectory_queue_t;


/** structure for packet queue node */
typedef struct _packet_queue_node_t {
		struct _packet_queue_node_t *next; 
		struct _packet_queue_node_t *prev;
        int id; /* packet id used by SMPL scheduler */
		STATE state; /* packet state */
        double state_time; /* packet state time */

		int vehicle_maximum_number; /* the maximum number of vehicles in the target road network; this is used to construct a predicted encounter graph for TPD data forwarding */
		struct _predicted_encounter_graph_t *predicted_encounter_graph; /* predicted encounter graph used in TPD data forwarding */
		double EDR_for_V2V; //Expected Delivery Ratio (EDR) for V2V in TPD
		double EDD_for_V2V; //Expected Delivery Delay (EDD) for V2V in TPD

		vanet_packet_forwarding_mode_t mode; /* packet forwarding mode = {FORWARDING_MODE_SOURCE_ROUTING=0, FORWARDING_MODE_RANDOM_WALK=1} */

		int expected_packet_transmission_number; //expected packet transmission number
		int packet_transmission_count; //packet transmission count to count the number of transmissions to next carriers, that is, how many carriers have carried this packet

		int packet_duplication_count; //the number of packet duplications to limit the packet copies in Epidemic Routing

        struct _global_packet_queue_node_t *global_packet; /* pointer to the global packet queue node corresponding to this packet */

        data_forwarding_mode_t data_forwarding_mode; /* data forwarding mode: {DATA_FORWARDING_MODE_DOWNLOAD, DATA_FORWARDING_MODE_UPLOAD} */

        vehicle_trajectory_queue_t vehicle_trajectory; /* destination vehicle trajectory under download mode */

        boolean reverse_traversal_mode_flag; //flag to indicate whether or not the packet is under the reverse traversal mode where the packet is delivered along the reverse path of the destination vehicle's trajectory in the partially or fully dynamic forwarding

        boolean reverse_traversal_completion_flag; //flag to undicate whether or not the packet has completed the reverse traverse by reaching the start point of the vehicle trajectory
        boolean reverse_traversal_next_hop_flag; //flag to indicate whether or not the packet's reverse traversal next hop is set as the next destination in the packet trajectory
        vehicle_trajectory_queue_node_t *reverse_traversal_current_hop_trajectory_qnode; //pointer to the vehicle trajectory queue node corresponding to the current hop that is the tail node of the edge having the packet under the reverse traversal mode

        carrier_trace_queue_t carrier_trace; /* queue for this packet's carrier trace */
        packet_trajectory_queue_t packet_trajectory; /* packet trajectory under download mode */

        vanet_node_type_t src_node_type; /* packet source node type = {VANET_NODE_AP, VANET_NODE_VEHICLE} */
        int src_id;  /* packet source id */

        vanet_node_type_t dst_node_type; /* packet destination node type = {VANET_NODE_AP, VANET_NODE_VEHICLE} */
        int dst_id; /* packet destination id */
        int actual_dst_id; /* actual destination id, such as packet receiving AP */

        struct_vehicle_t *dst_vnode; /* pointer to destination vehicle under download mode */

        int carry_src_id; /* carrier source id */
        int carry_dst_id; /* carrier destination id */

        struct_vehicle_t *carrier_vnode; /* pointer to carrier vehicle under both upload mode and download mode */

        int target_point_id; /* Under download mode or V2V mode, this is target point id for the destination vehicle where target point id is the intersection id, not vehicle id */
        //int  initial_target_point_id; //initial target point determined by AP
        //double target_point_id_update_time; //target point update time

        //struct _struct_access_point_t *src_ap; /* pointer to the AP structure corresponding to src_ap_id; this structure's augmented graph Ga is used to compute the carrier candidate vehicle's EDD towards the target point */

		unsigned int seq; /* sequence number */
		MOVE_TYPE move_type; /* movement type: {MOVE_FORWARD, MOVE_BACKWARD} */
        int size;  /* data size in bytes */
        double expected_delivery_delay; /* expected delivery delay (EDD) corresponding to the vehicle's EDD */
        double actual_delivery_delay; /* actual delivery delay from the packet source to the destination AP */
        double expected_delivery_delay_standard_deviation; /* expected delivery delay's standard deviation */
        double actual_delivery_delay_standard_deviation; /* actual delivery delay's standard deviation that is computed by the absolute difference between expected_delivery_delay and actual_delivery_delay */
        double delivery_delay_difference; /* delivery delay difference */
        double ttl; /* Time To Live (TTL): We can set TTL according to vehicle's EDD, such as EDD*2 */
        double actual_lifetime; /* actual lifetime when the packet is discarded */
		double generation_time;    /* time when this packet was generated */
        double last_receive_time; /* last receive time by carry vehicle */
		double destination_arrival_time; /* time when this packet has arrived at the destination, such as access point or destination vehicle */

        /* the recomputation interval and the recomputation time for a new target point */
        double target_point_recomputation_interval; /* the time interval to recompute a new target point during the packet delivery */
        double target_point_recomputation_time; /* the time instant to recompute a new target point during the packet delivery in simulation time */
} packet_queue_node_t;

/** structure for packet schedule queue node */
typedef struct _packet_schedule_queue_node_t {
        struct _packet_schedule_queue_node_t *next;
        struct _packet_schedule_queue_node_t *prev;
        int vid; //vehicle id
        double timestamp; //schedule assignment time
        double sched_time; //absolute schedule time in simulation
} packet_schedule_queue_node_t;

/** structure for global packet queue node */
typedef struct _global_packet_queue_node_t {
        struct _global_packet_queue_node_t *next; 
		struct _global_packet_queue_node_t *prev;

        int id; /* packet id used by SMPL scheduler */
		STATE state; /* packet state */
        double state_time; /* current time for packet state */

        packet_queue_node_t *packet; /* pointer to packet queue node */
        struct_vehicle_t *carrier_vnode; /* pointer to packet carrier vehicle */
        int order; /* order in the global packet queue */

        /** EDD information */
        double EDD; //expected delivery delay
        double EDD_SD; //expected delivery delay standard deviation

        /** Target Point */
        int target_point_id; //target point intersection id

        /** Target Point Recomputation Information: the recomputation interval and the recomputation time for a new target point */
        double target_point_recomputation_interval; /* the time interval to recompute a new target point during the packet delivery */
        double target_point_recomputation_time; /* the time instant to recompute a new target point during the packet delivery in simulation time */

		/* Management of packet copies */
		int packet_copy_count; /* count for packet copies to delete the global packet queue node; NOTE: packet_copy_count increases (or decreases) as one packet copy is generated (or deleted). */

        struct _global_packet_queue_t *ptr_queue; /* pointer to the global packet queue */
} global_packet_queue_node_t;

/** structure for vehicle movement queue node */
typedef struct _vehicle_movement_queue_node_t {
        struct _vehicle_movement_queue_node_t *next;
        struct _vehicle_movement_queue_node_t *prev;
        int vid; //vehicle id
        struct_vehicle_t *vnode; //pointer to vehicle node
        double speed; //vehicle speed
        MOVE_TYPE move_type; //movement type = {MOVE_FORWARD, MOVE_BACKWARD} 
        double registration_time; //registration time of this vehicle movemene queue node
        double arrival_time;  //vehicle arrival time at this edge
        double departure_time; //vehicle departure time from this edge
        double offset; //offset from the tail_node of the edge
        int order; //order in terms of offset on the edge; note that the first vehice's order is zero
        struct _vehicle_movement_queue_t *ptr_queue; //pointer to the vehicle movement queue
} vehicle_movement_queue_node_t;

/** structure for intersection EDD queue node */
typedef struct _intersection_edd_queue_node_t {
        struct _intersection_edd_queue_node_t *next; 
	struct _intersection_edd_queue_node_t *prev;
        vanet_metric_type_t type; //VANET Metric Type = {EDD, EDD_VAR}
        double EDD; //Expected Data Delivery (EDD) Metric
	int order; //ascending order for EDD or EDD_VAR
	struct _struct_graph_node *tail_gnode; //pointer to tail graph node for the directional edge
	struct _struct_graph_node *head_gnode; //pointer to head graph node for the directional edge
} intersection_edd_queue_node_t;

/** structure for vehicle queue node */
typedef struct _vehicle_queue_node_t {
        struct _vehicle_queue_node_t *next;
        struct _vehicle_queue_node_t *prev;
        int vid; //vehicle id
        double offset; //vehicle's offset in the directional edge
        int order; //order of vehicle within the convoy from the least offset; note that the first's order is zero
        struct struct_vehicle *vnode; //pointer to the vehicle node
} vehicle_queue_node_t;

/** structure for mobility queue node for vehicle mobility on the road network */
typedef struct _mobility_queue_node_t {
        struct _mobility_queue_node_t *next;
        struct _mobility_queue_node_t *prev;
        int intersection_id; //intersection id
        int order; //order of visited intersections; note that the first's order is zero
        struct _mobility_queue_t *ptr_queue; //pointer to the mobility queue
} mobility_queue_node_t;

/** structure for target point queue node for a target point as packet destination */
typedef struct _target_point_queue_node_t {
	struct _target_point_queue_node_t *next;
	struct _target_point_queue_node_t *prev;
	int target_point_id; //target intersection id
	double delivery_delay; //delivery delay from AP to target point
	double delivery_probability; //delivery probability	
    //double EPD; //expected packet delay
	//double EPD_SD; //standard deviation of packet delay
	//double EVD; //expected vehicle delay
	//double EVD_SD; //standard deviation of vehicle delay
	int order; //order of target intersections; note that the first's order is zero
	struct _target_point_queue_t *ptr_queue; //pointer to the target point queue
} target_point_queue_node_t;

/////////////////////////////////////////////////////////////////////
/** structure for queue */
typedef struct _queue_t {
	queue_type_t type; //queue type
	int size; //size of queue except for head node
	queue_node_t head; //head node for queue
} queue_t;

/** structure for schedule queue */
typedef struct _schedule_queue_t {
	queue_type_t type; //queue type
	int size;
	schedule_queue_node_t head;
} schedule_queue_t;

/** structure for sensor queue */
typedef struct _sensor_queue_t {
	queue_type_t type; //queue type
	int size; //number of sensors on an edge
	sensor_queue_node_t head;
	int live_sensor_number; //number of live sensors on an edge
} sensor_queue_t;

/** structure for path queue */
typedef struct _path_queue_t {
	queue_type_t type; //queue type
	int size;
	path_queue_node_t head;
} path_queue_t;

/** structure for location queue */
typedef struct _location_queue_t {
	queue_type_t type; //queue type
	int size;
	location_queue_node_t head;
} location_queue_t;

/** structure for sensing hole segment queue */
typedef struct _hole_segment_queue_t {
	queue_type_t type; //queue type
	int size;
	hole_segment_queue_node_t head;
} hole_segment_queue_t;

/** structure for sensing hole endpoint queue */
typedef struct _hole_endpoint_queue_t {
	queue_type_t type; //queue type
	int size;
	hole_endpoint_queue_node_t head;
	int sequence_number; //sequence number that has been allocated recently as an id for a new sensing hole node
} hole_endpoint_queue_t;

/** structure for subedge queue */
typedef struct _subedge_queue_t {
	queue_type_t type; //queue type
	int size;
	subedge_queue_node_t head;
} subedge_queue_t;

/** structure for edge node */
typedef struct _edge_queue_node_t {
        struct _edge_queue_node_t *next; 
	struct _edge_queue_node_t *prev;
	int eid;                    //ID of edge (tail_node, head_node); this indicates the undirectional edge including two directional edges with the same end points of the edge
	char tail_node[NAME_SIZE];  //tail node of a directional edge
	char head_node[NAME_SIZE];  //head node of a directional edge
	double weight;              //weight for edge
	double density;             //sensor density on the edge
	location_queue_t sensor_location_list; //sensor location list for this edge
	int total_sensor_number; //total number of sensors on an edge in real graph Gr 
	int live_sensor_number; //number of live sensors on an edge in real graph Gr 
	subedge_queue_t subedge_list; //subedge list for this edge where subedges are the subdivion of the edge
	hole_segment_queue_t sensing_hole_segment_list; //sensing hole segment list for this edge
	hole_endpoint_queue_t sensing_hole_endpoint_list; //sensing hole endpoint list for this edge that has a list of sensing hole nodes
} edge_queue_node_t;

/** structure for edge queue */
typedef struct _edge_queue_t {
	queue_type_t type; //queue type
	int size;
	edge_queue_node_t head;
	int sequence_number; //sequence number that has been allocated recently as an id for a new edge node
} edge_queue_t;

/** structure for vertex_set queue */
typedef struct _vertex_set_queue_t {
	queue_type_t type; //queue type
	int size;
	vertex_set_queue_node_t head;
	int reference_count; //reference count to indicate how many nodes refer to this queue; when the reference count becomes, we can delete the queue pointed by head
	cluster_type_t cluster_type; //cluster type
} vertex_set_queue_t;

/** structure for edge_set queue */
typedef struct _edge_set_queue_t {
	queue_type_t type; //queue type
	int size;
	edge_set_queue_node_t head;
} edge_set_queue_t;

/** structure for angle queue */
typedef struct _angle_queue_t {
	queue_type_t type; //queue type
	int size;
	angle_queue_node_t head;
} angle_queue_t;

/** structure for delay component queue */
typedef struct _delay_component_queue_t {
	queue_type_t type; //queue type
	int size;
	delay_component_queue_node_t head;
} delay_component_queue_t;

/** structure for delay queue node for VADD */
typedef struct _delay_queue_node_t {
        struct _delay_queue_node_t *next; 
	struct _delay_queue_node_t *prev;
        int eid; //edge id
        int order; //order according to eid in the increasing order of eid
        double edge_delay; //edge delay that a packet needs to be forwarded over this edge through both packet forwarding and vehicle carry
        double edge_delay_standard_deviation; //edge delay standard deviation
        double edge_delay_variance; //edge delay variance
        double edge_value; //edge value that is the constant term in the 2nd moment computation of the delivery delay for an edge
        struct _delay_component_queue_t delay_component_list; //queue for delay components of EDD
        struct _directional_edge_queue_node_t *enode; //pointer to an edge node in directional edge queue corresponding to eid
        double EDD; //expected delivery delay that a packet is forwarded through this edge to the destination
        double delivery_delay_variance; //E2E delivery delay's variance
        double delivery_delay_second_moment; //E2E delivery delay's second_moment
} delay_queue_node_t;

/** structure for delay queue */
typedef struct _delay_queue_t {
	queue_type_t type; //queue type
	int size;
	delay_queue_node_t head;
} delay_queue_t;

/** structure for packet queue */
typedef struct _packet_queue_t {
	queue_type_t type; //queue type
	int size;
	packet_queue_node_t head;
} packet_queue_t;

/** structure for packet schedule queue */
typedef struct _packet_schedule_queue_t {
	queue_type_t type; //queue type
	int size;
	packet_schedule_queue_node_t head;
} packet_schedule_queue_t;


#define INITIAL_PACKET_VECTOR_SIZE 10000
#define PACKET_VECTOR_INCREASE_SIZE 1000

/** structure for global packet queue */
typedef struct _global_packet_queue_t {
	queue_type_t type; //queue type
	int size;
	global_packet_queue_node_t head;
	vanet_information_table_t *ptr_vanet_table; //pointer to the vanet table that contains pointers to data structures used in the data forwarding in VANET; the variable pointed by ptr_vanet_table is one member of parameter_t param

	/* variables for packet delivery delay statistics under 
	 * multiple packet copy transmission */
	boolean *packet_id_bitmap_vector; //packet id bitmap vector where the entry value of 1 indicates that the packet corresponding to index has been received by the destination vehicle 
	double *packet_delivery_delay_vector; //packet delivery delay vector
	int *packet_deletion_count_vector; //packet deletion count vector where each entry counts the number of deleted packets due to TTL expiration; this vector will be used to compute discarded_packet_number later
	int packet_copy_number; //number of packet copies under multi-target-point data forwarding
	int physical_vector_size;
	int actual_vector_size;
} global_packet_queue_t;

/** structure for vehicle queue */
typedef struct _vehicle_queue_t {
        queue_type_t type; 
        int size;
        vehicle_queue_node_t head;
} vehicle_queue_t;

/** structure for vehicle movement queue */
typedef struct _vehicle_movement_queue_t {
	queue_type_t type; //queue type
	int size;
	vehicle_movement_queue_node_t head;
        int eid; //directional edge's id
        struct _directional_edge_queue_node_t *enode; //pointer to the directional edge node
} vehicle_movement_queue_t;

/** structure for convoy queue node */
typedef struct _convoy_queue_node_t {
        struct _convoy_queue_node_t *next;
        struct _convoy_queue_node_t *prev;
        int cid; //convoy id
        //double EDD; //convoy's EDD, that is, the minimum of the vehicle EDDs of the vehicles belonging to this convoy
        struct struct_vehicle *tail_vehicle; //convoy tail vehicle; the vehicle that last joined in the convoy
        struct struct_vehicle *head_vehicle; //convoy head vehicle; the vehicle that created the convoy
        struct struct_vehicle *leader_vehicle; //convoy leader vehicle with the minimum vehicle EDD within the convoy; note that leader's vehicle EDD becomes the convoy EDD
        struct _vehicle_queue_t vehicle_list; //vehicle queue containing the pointers to the vehicles
        struct _convoy_queue_t *ptr_queue; //pointer to the convoy queue
} convoy_queue_node_t;

/** structure for convoy queue */
typedef struct _convoy_queue_t {
        queue_type_t type;
        int size;
        convoy_queue_node_t head;
        int eid; //directional edge id
        struct _directional_edge_queue_node_t *enode; //pointer to the directional edge node
        unsigned int sequence_number; //sequence number for counting the number of created convoys in the directional edge
} convoy_queue_t;

/** structure for conditional forwarding probability queue node for a directed edge */
typedef struct _conditional_forwarding_probability_queue_node_t {
        struct _conditional_forwarding_probability_queue_node_t *next;
        struct _conditional_forwarding_probability_queue_node_t *prev;

        char tail_node_of_current_carrier_edge[NAME_SIZE]; //tail node of current carrier's edge
        char head_node_of_current_carrier_edge[NAME_SIZE]; //head node of current carrier's edge

        char tail_node_of_next_carrier_edge[NAME_SIZE]; //tail node of next carrier's edge
        char head_node_of_next_carrier_edge[NAME_SIZE]; //head node of next carrier's edg

        double conditional_forwarding_probability; //conditional forwarding probability
        struct _probability_and_statistics_queue_node_t *ptr_probability_and_statistics_queue_node; //pointer to the probability and statistics queue node for the carrier moving edge

        int order; //order of edges; note that the first's order is zero
        struct _conditional_forwarding_probability_queue_t *ptr_queue; //pointer to the conditional forwarding probability queue

} conditional_forwarding_probability_queue_node_t;

/** structure for conditional forwarding probability queue for a directed edge */
typedef struct _conditional_forwarding_probability_queue_t {
	queue_type_t type; //queue type
	int size;
	conditional_forwarding_probability_queue_node_t head;
} conditional_forwarding_probability_queue_t;

/** structure for probability and statistics queue node */
typedef struct _probability_and_statistics_queue_node_t {
        struct _probability_and_statistics_queue_node_t *next;
        struct _probability_and_statistics_queue_node_t *prev;
        int eid; //directed edge id
        char tail_node[NAME_SIZE]; //tail vertex
        char head_node[NAME_SIZE]; //head vertex
        struct _directional_edge_queue_node_t *enode; //pointer to the directional edge node

        double timestamp; //timestamp to specify when this information is stored in the simulation

        /** probability and statistics information */
        double mean_interarrival_time; //average interarrival time
        double lambda; //arrival rate per unit time (1 sec) = 1/mean_interarrival_time
        double contact_probability;
        double forwarding_probability;
        conditional_forwarding_probability_queue_t conditional_forwarding_probability_queue;
        double branch_probability;
        double average_forwarding_probability;
        double EDD;
        double EDD_SD;
        /*******************************************/

        int order; //order of edges; note that the first's order is zero
        struct _probability_and_statistics_queue_t *ptr_queue; //pointer to the probability and statistics queue
} probability_and_statistics_queue_node_t;

/** structure for probability and statistics queue */
typedef struct _probability_and_statistics_queue_t {
	queue_type_t type; //queue type
	int size;
	probability_and_statistics_queue_node_t head;

        int eid; //directed edge id
        char tail_node[NAME_SIZE]; //tail vertex
        char head_node[NAME_SIZE]; //head vertex
        struct _directional_edge_queue_node_t *enode; //pointer to the directional edge node
} probability_and_statistics_queue_t;

/** structure for directional edge queue node for VADD */
typedef struct _directional_edge_queue_node_t {
        struct _directional_edge_queue_node_t *next; 
        struct _directional_edge_queue_node_t *prev;
        int eid; //edge id
        int order; //order according to eid in the increasing order of eid
        char tail_node[NAME_SIZE];  //tail node of a directional edge
        char head_node[NAME_SIZE];  //head node of a directional edge
        double weight;              //weight for edge

        /* set tail_gnode and head_gnode to point to the graph node of the tail_node and that of the head_node, 
           respectively, in the adjacency list where tail_node is in the graph node array and head_node is in 
           the neighbor list of the tail_node */
        struct_graph_node *tail_gnode; //pointer to the graph node corresponding to the tail node in <tail_node, head_node>
        struct_graph_node *head_gnode; //pointer to the graph node corresponding to the head node in <tail_node, head_node>

        struct _vehicle_movement_queue_t vehicle_movement_list; //vehicle movement queue
        struct _convoy_queue_t convoy_list; //convoy queue

        struct _probability_and_statistics_queue_t probability_and_statistics_queue; //probability-and-statistics queue

  /** variables for measuring Average Convoy Length (ACL) from the tail node */
        boolean acl_measurement_start_flag; //ACL measurement start flag to indicate that the first arrived vehicle has departed the directional edge, so the ACL measurement can start with the full pipeline of arriving vehicles on the directional edge.
        double acl_measurement_start_time; //ACL measurement start time; that is, the time when the first vehicle arrives at the head of the directional edge        
        double acl_measurement_end_time; //ACL measurement end time; that is, the time when the last sum of area happens during the simulation, which is updated every time the ACL area is summed
        int acl_vehicle_arrival_number; //number of vehicle arrivals at the tail node
        int acl_convoy_vehicle_number; //number of vehicles consisting of a convoy from the tail node
        double acl_interarrival_time; //interarrival time of two consecutive vehicles
        double acl_convoy_head_arrival_time; //arrival time of the convoy head vehicle at the edge
        double acl_convoy_head_departure_time; //departure time of the convoy head vehicle from the edge
        double acl_convoy_tail_arrival_time; //arrival time of the convoy tail vehicle at the edge or the last vehicle arrival time at the edge
        double acl_convoy_tail_departure_time; //departure time of the convoy tail vehicle from the edge
        struct_vehicle_t *acl_convoy_tail_vehicle; //vehicle id arriving last
        struct_vehicle_t *acl_convoy_head_vehicle; //vehicle id corresponding to the convoy head
        double acl_convoy_start_time; //start-time of a convoy
        double acl_convoy_end_time; //end-time of a convoy
        double acl_convoy_start_length; //start-length of a convoy
        double acl_convoy_end_length; //end-length of a convoy
        double acl_convoy_threshold; //the time that a convoy is disconnected from the intersection, that is, param->communication_range/param->vehicle_speed
        double acl_convoy_rectangle_height; //the convoy length when a new convoy starts
        double acl_convoy_triangle_height; //the increased convoy length after a new convoy starts
        double acl_convoy_duration; //the new convoy's duration; that is, the time difference of convoy end time and convoy start time
        double acl_convoy_area; //the accumulated area for ACL during the new convoy's duration
        //boolean acl_flag_of_logging_start; //flag to determine the start of the logging for the convoy length
} directional_edge_queue_node_t;

/** structure for directional edge queue */
typedef struct _directional_edge_queue_t {
	queue_type_t type; //queue type
	int size;
	directional_edge_queue_node_t head;
	int sequence_number; //sequence number that has been allocated recently as an id for a new directional edge node
} directional_edge_queue_t;

/** structure for intersection EDD queue */
typedef struct _intersection_edd_queue_t {
	queue_type_t type; //queue type
	int size;
	intersection_edd_queue_node_t head;
} intersection_edd_queue_t;

/** structure for mobility queue */
typedef struct _mobility_queue_t {
	queue_type_t type; //queue type
	int size;
	mobility_queue_node_t head;
} mobility_queue_t;

/** structure for destination vehicle queue node */
typedef struct _destination_vehicle_queue_node_t {
        struct _destination_vehicle_queue_node_t *next;
        struct _destination_vehicle_queue_node_t *prev;
        int vid; //destination vehicle id
        mobility_type_t mobility_type; //mobility type
        struct struct_vehicle *vnode; //pointer to the vehicle node
        int order; //order of the destination vehicle in the destination vehicle queue
        struct _mobility_queue_t mobility_list; //mobility queue of the destination vehicle
        struct _destination_vehicle_queue_t *ptr_queue; //pointer to the destination vehicle queue
} destination_vehicle_queue_node_t;

/** structure for destination vehicle queue */
typedef struct _destination_vehicle_queue_t {
	queue_type_t type; //queue type
	int size;
	destination_vehicle_queue_node_t head;        
} destination_vehicle_queue_t;

/** structure for Access Point queue node */
typedef struct _access_point_queue_node_t
{
    struct _access_point_queue_node_t* next; /* next AP */
    struct _access_point_queue_node_t* prev; /* previous AP */

    int id;   /* AP id */
    char vertex[NAME_SIZE]; /* the vertex name of the intersection where AP is placed */
    STATE state; /* AP state */
    double state_time; /* current time for AP state */
    struct_coordinate1_t coordinate; /* geographic coordinate in roadmap in the 2-D Cartesian coordinate system */
    struct_graph_node* gnode; /* pointer to the graph node in the road network graph for the intersection where the AP is placed */

    int seq; /* sequence number for packets */
    double packet_generation_time;    /* time when the latest packet is generated by AP */
    double packet_send_time;    /* time when the latest packet is sent to a neighboring vehicle */
    struct _packet_queue_t *packet_queue; /* pointer to packet queue; the memory allocated to packet queue should be released later */

    struct_graph_node* Gr; /* road network graph Gr: this is only the pointer to Gr in order to access Gr, that is, this is not a copy of Gr. Thus, we need not allocate and deallocate Gr for this AP */
    int Gr_size; //number of nodes in Gr

    struct_graph_node* Ga; /* augmented graph Ga including the target point as virtual node */
    int Ga_size; //number of nodes in Ga
    struct_coordinate3_t target_point; /* the position of the target point for the directional edge in Ga */

    edge_queue_t Ea; /* edge queue containing the information of edges in augmented graph Ga */
    directional_edge_queue_t DEa; /* directional edge queue containing the directional edges in augmented graph Ga */

    struct_traffic_table ap_table_for_target_point;  //table for target points that are regarded as the virtual APs (i.e., geographic packet destination) in augmented graph Ga

    int target_point_id; /* AP's current target point for a destination vehicle */
    double EDD_for_download; /* EDD to AP's current target point for a destination vehicle */
    double EDD_SD_for_download; /* EDD_SD to AP's current target point for a destination vehicle */
} access_point_queue_node_t;

/* struct_access_point_t */
typedef access_point_queue_node_t struct_access_point_t;

/** structure for access point queue */
typedef struct _access_point_queue_t {
    queue_type_t type; //queue type
    int size;
    access_point_queue_node_t head;
} access_point_queue_t;

/** struct for forwarding table queue node for STBD data forwarding */
typedef struct _forwarding_table_queue_node_t {
    struct _forwarding_table_queue_node_t *next;
    struct _forwarding_table_queue_node_t *prev;
    int intersection_id; //intersection id
    struct_graph_node *G; //forwarding table that is an adjacency list for a road network graph
    int G_size; //graph size that is the number of nodes
    edge_queue_t EQ; //undirectional edge queue for graph G
    directional_edge_queue_t DEQ; //directional edge queue for graph G

	/* the shortest delay matrices */
	double **Dr_edd; //the shortest delay from start-point to end-point
	int **Mr_edd; //predecessor matrix for Dr_edd
	double **Sr_edd; //supplementary matrix for Dr_edd
	int matrix_size_for_edd_in_Gr; //matrix size of matrices Dr_edd, Mr_edd, and Sr_edd

	/* the shortest cost matrices */
	double **Wr_edc; //the adjacency matrix for edge cost from start-point to end-point
	double **Dr_edc; //the shortest delay from start-point to end-point
	int **Mr_edc; //predecessor matrix for Dr_edd
	double **Sr_edc; //supplementary matrix for Dr_edd
	int matrix_size_for_edc_in_Gr; //matrix size of matrices Dr_edc, Mr_edc, and Sr_edc	

    struct _forwarding_table_queue_t *ptr_queue; //pointer to the forwarding table queue
} forwarding_table_queue_node_t;

/** structure for forwarding table queue for STBD data forwarding */
typedef struct _forwarding_table_queue_t {
    queue_type_t type; //queue type
    int size; //size is the same as the number of nodes in the road network graph
    forwarding_table_queue_node_t head;
    forwarding_table_queue_node_t **index_table; //index table to access a table for intersection id; index_table[0] points to the table corresponding to intersection id == 1
} forwarding_table_queue_t;

/** structure for stationary node queue node */
typedef struct _stationary_node_queue_node_t {
    struct _stationary_node_queue_node_t *next;
    struct _stationary_node_queue_node_t *prev;
    int intersection_id; //intersection id as stationary node id
    int order; //order of stationary nodes; note that the first's order is zero
    struct_graph_node *gnode; //pointer to the Gr vertex corresponding to stationary_node_id
    struct _packet_queue_t packet_queue; /* pointer to packet queue; the memory allocated to packet queue should be released later */

    /** Target Point Information */
    int target_point_id; //target point id of the packet latest generated by AP
    struct_traffic_table tp_table; //target point traffic table
    unsigned int latest_packet_seq; //sequence number of the packet latest generated by AP
    double latest_packet_receive_time; //receive time of the packet latest generated by AP
    struct _packet_queue_node_t *latest_packet_ptr; //pointer to the latest packet from AP among the packets in the stationary node's packet_queue

    /** Destination Vehicle's Passing Time */
    /* Note that the passing time can be cached by caching the pairs of the vehicle ID and the latest passing time */                                                                        
    //int destination_vehicle_id; //destination vehicle id
    double destination_vehicle_latest_passing_time; //the latest passing time of the destination vehicle through the intersection having this stationary node

    struct _stationary_node_queue_t *ptr_queue; //pointer to the stationary node queue
} stationary_node_queue_node_t;

/** structure for stationary node queue */
typedef struct _stationary_node_queue_t {
	queue_type_t type; //queue type
	int size;
	stationary_node_queue_node_t head;
} stationary_node_queue_t;

/** structure for target point queue */
typedef struct _target_point_queue_t {
	queue_type_t type; //queue type
	int size;
	target_point_queue_node_t head;
	double minimum_average_delivery_delay; //minimum average delivery delay
	double delivery_success_probability; //delivery success probability
} target_point_queue_t;

/** structure for packet forwarding tree path queue node for forwarding a packet to a target point */
typedef struct _packet_forwarding_tree_path_queue_node_t {
	struct _packet_forwarding_tree_path_queue_node_t *next;
	struct _packet_forwarding_tree_path_queue_node_t *prev;
  	int target_point_id; //target point id, that is, packet destination id
	packet_trajectory_queue_t PTQ; //packet trajectory queue for packet forwarding
	boolean non_superedge_flag; //flag to indicate that the packet trajectory in PTQ is not a superedge.
	int order; //order of visited intersections; note that the first's order is zero
	struct _packet_forwarding_tree_path_queue_t *ptr_queue; //pointer to the packet forwarding tree path queue
} packet_forwarding_tree_path_queue_node_t;

/** structure for packet forwarding tree path queue for packet forwarding paths to multiple target points in the packet forwarding tree */
typedef struct _packet_forwarding_tree_path_queue_t {
	queue_type_t type; //queue type
	int size;
	packet_forwarding_tree_path_queue_node_t head;
	int source_intersection_id; //the packet's source intersection id
} packet_forwarding_tree_path_queue_t;

/** structure for component_vertex queue node */
typedef struct _component_vertex_queue_node_t {
    struct _component_vertex_queue_node_t *next; 
	struct _component_vertex_queue_node_t *prev;
	int vertex; //vertex id
	int order; //order in the queue
	struct _component_vertex_queue_t *ptr_queue; //pointer to the queue 
} component_vertex_queue_node_t;

/** structure for component_vertex queue */
typedef struct _component_vertex_queue_t {
	queue_type_t type; //queue type
	int size;
	component_vertex_queue_node_t head;
} component_vertex_queue_t;

/** structure for neighbor_list queue node */
typedef struct _neighbor_list_queue_node_t {
	struct _neighbor_list_queue_node_t *next;
	struct _neighbor_list_queue_node_t *prev;
    int id; //id
	boolean optimal_subsequence_flag; //flag to indicate whether this neighbor vehicle belongs to an optimal subsequence.
	double weight; //weight (i.e., encounter delay)
	double T_encounter; //encounter time
	double P_encounter; //encounter probability
	int tail_vertex; //tail vertex for encounter edge
	int head_vertex; //head vertex for encounter edge
	double edge_length; //length of encounter edge
	double edge_offset; //offset for encounter edge
	/* Note: the edge is written in the viewpoint of a parent vehicle. 
	 * That is, the parent vehicle encounters this child vehicle 
	 * at the edge of (tail_vertex, head_vertex: edge_offset). 
	 * For example, for an adjacency_list_queue_node_t node a, node a 
	 * is the parent vehicle and the corresponding neighbor_list_queue_node_t 
	 * node b is the child vehicle.
	 * */

	queue_node_object_type_t object_type; //object type
	void *object; //pointer to the object for the queue node, such as vehicle model
	struct _adjacency_list_queue_node_t *graph_qnode; //pointer to the graph node in the adjacency list of predicted encounter graph
    int order; //order in the queue
	struct _neighbor_list_queue_t *ptr_queue;
} neighbor_list_queue_node_t;

/** structure for neighbor_list queue */
typedef struct _neighbor_list_queue_t {
	queue_type_t type; //queue type
	int	size; 
	neighbor_list_queue_node_t head;
} neighbor_list_queue_t;

/** structure for parent_list queue node */
typedef struct _parent_list_queue_node_t {
	struct _parent_list_queue_node_t *next;
	struct _parent_list_queue_node_t *prev;
    int id; //id
	double weight; //weight (i.e., encounter delay)
	double T_encounter; //encounter time
	double P_encounter; //encounter probability
	int tail_vertex; //tail vertex for encounter edge
	int head_vertex; //head vertex for encounter edge
	double edge_length; //length of encounter edge
	double edge_offset; //offset for encounter edge
	/* Note: the edge is written in the viewpoint of a child vehicle. 
	 * That is, the child vehicle encounters this parent vehicle
	 * at the edge of (tail_vertex, head_vertex: edge_offset).
	 * For example, for an adjacency_list_queue_node_t node a, node a
	 * is the child vehicle and the corresponding parent_list_queue_node_t 
	 * node c is the parent vehicle.
	 * */

	queue_node_object_type_t object_type; //object type
	void *object; //pointer to the object for the queue node, such as vehicle model
	struct _adjacency_list_queue_node_t *graph_qnode; //pointer to the graph node in the adjacency list of predicted encounter graph
    int order; //order in the queue
	struct _parent_list_queue_t *ptr_queue;
} parent_list_queue_node_t;

/** structure for parent_list queue */
typedef struct _parent_list_queue_t {
	queue_type_t type; //queue type
	int	size; 
	parent_list_queue_node_t head;
} parent_list_queue_t;

/** structure for minimum_priority queue node */
typedef struct _minimum_priority_queue_node_t {
	struct _minimum_priority_queue_node_t *next;
	struct _minimum_priority_queue_node_t *prev;
	double key; //key (e.g., encounter time)
	int id; //id (e.g., vehicle id)
	queue_node_object_type_t object_type; //object type
	void *object; //pointer to the object for the queue node, such as vehicle model
	parent_list_queue_t parent_list; //parent list queue
	int order; //order in the queue
	struct _minimum_priority_queue_t *ptr_queue;
} minimum_priority_queue_node_t;

/** structure for minimum_priority queue */
typedef struct _minimum_priority_queue_t {
	queue_type_t type; //queue type
	int size; 
	minimum_priority_queue_node_t head;
} minimum_priority_queue_t;

/** structure for adjacency_list queue node */
typedef struct _adjacency_list_queue_node_t {
	struct _adjacency_list_queue_node_t *next;
	struct _adjacency_list_queue_node_t *prev;
	double key; //key (e.g., id)
    int id; //id (e.g., vertex id)
	queue_node_object_type_t object_type; //object type
	void *object; //pointer to the object for the queue node, such as vehicle model
	neighbor_list_queue_t neighbor_list; //neighbor list queue
	parent_list_queue_t parent_list; //parent list queue
	int count; //usage count as intermediate node from src_vehicle to dst_vehicle
	boolean EDR_flag; //flag to show whether the EDR of this graph node is resoved or not
	boolean EDD_flag; //flag to show whether the EDD of this graph node is resoved or not
	double EDR; //Expected Delivery Ration (EDR)
	double EDD; //Expected Delivery Delay (EDD)
	boolean queueing_flag; //flag to show whether this node or the pointer to this node is queued in some queue, such as adjacency_list_pointer_queue_t Q
    int order; //order in the queue
	struct _adjacency_list_queue_t *ptr_queue;
} adjacency_list_queue_node_t;

/** structure for adjacency_list queue */
typedef struct _adjacency_list_queue_t {
	queue_type_t type; //queue type
	int	size; 
	adjacency_list_queue_node_t head;
	boolean *bitmap; //bitmap to tell whether the graph node for vehicle with vid (index+1) exists in the graph G
	int bitmap_size; //size of bitmap
	adjacency_list_queue_node_t **bitmap_gnodes; //pointers to graph nodes for vehicles
	adjacency_list_queue_node_t *src_vehicle_gnode; //graph node for src_vehicle
	adjacency_list_queue_node_t *dst_vehicle_gnode; //graph node for dst_vehicle
	adjacency_list_queue_node_t *carrier_vehicle_gnode; //graph node for the current carrier vehicle
} adjacency_list_queue_t;

/** structure for adjacency_list_pointer queue node */
typedef struct _adjacency_list_pointer_queue_node_t {
	struct _adjacency_list_pointer_queue_node_t *next;
	struct _adjacency_list_pointer_queue_node_t *prev;
	adjacency_list_queue_node_t *graph_qnode; //pointer to graph queue node in adjacency_list
	int order; //order in the queue
	struct _adjacency_list_pointer_queue_t *ptr_queue;
} adjacency_list_pointer_queue_node_t;

/** structure for adjacency_list_pointer queue */
typedef struct _adjacency_list_pointer_queue_t {
	queue_type_t type; //queue type
	int	size; 
	adjacency_list_pointer_queue_node_t head;
} adjacency_list_pointer_queue_t;

/** Predicted Encounter Graph */
typedef struct _predicted_encounter_graph_t {
	struct_vehicle_t *root; //root vehicle in the graph
	minimum_priority_queue_t Q; //minimum priority queue Q
	adjacency_list_queue_t G; //adjacency list for graph G
} predicted_encounter_graph_t;

/***************************/

/** function declarations **/
void InitQueue(queue_t *Q, queue_type_t queue_type); 
//initialize queue Q

/** Note: whenever we support another queue type, we need to add another case statement for it */
queue_node_t* MakeQueueNode(queue_type_t queue_type, queue_node_t *node);
//make a queue node corresponding to queue type and copy the data of node into its data portion

queue_node_t* Enqueue(queue_t *Q, queue_node_t *node);
//enqueue node into queue Q and return the pointer to the new queue node

queue_node_t* Enqueue_With_QueueNodePointer(queue_t *Q, queue_node_t *p);
//enqueue queue node pointer p into queue Q without allocating a new queue node

queue_node_t* Enqueue_By_KeyAscendingOrder(queue_t *Q, queue_node_t *node);
//enqueue node into queue Q by the ascending order of node's key and return the pointer to the new queue node. Note that node must have key field for this function.

queue_node_t* Dequeue(queue_t *Q);
//dequeue node from queue Q

queue_node_t* Dequeue_With_QueueNodePointer(queue_t *Q, queue_node_t *p);
//dequeue node corresponding to pointer p from queue Q

void EmptyQueue(queue_t *Q);
//empty queue Q by removing the queue nodes in Q

void DestroyQueue(queue_t *Q);
//destory queue Q

void ResetQueue(queue_t *Q);
//reset queue Q by emptying Q and resetting the vector(s) in Q

void DestroyQueueNode(queue_type_t type, queue_node_t *q);
//destory queue-node q of queue-type type along with its queue(s) as field(s)

int SizeofQueue(queue_t *Q);
//return the size of queue Q

queue_node_t* GetQueueNode(queue_t *Q, int index);
//return the queue node corresponding to index; the index of the first node is 0.

queue_node_t* GetFrontQueueNode(queue_t *Q);
//return the front queue node corresponding to the first enqueued node.

queue_node_t* GetRearQueueNode(queue_t *Q);
//return the rear queue node corresponding to the last enqueued node.

void SortSensorQueue(sensor_queue_t* Q);
//sort sensor queue nodes in ascending order according to offset

void RearrangeSensorQueue(sensor_queue_t* Q, sensor_queue_node_t **A);
//rearrange the pointers of sensor queue Q using A in the ascending order

void SortEdgeSetQueue(edge_set_queue_t* Q);
//sort edge_set queue nodes in ascending order according to weight

void RearrangeEdgeSetQueue(edge_set_queue_t* Q, edge_set_queue_node_t **A);
//rearrange the pointers of edge_set queue Q using A in the ascending order

void SortAngleQueue(angle_queue_t* Q);
//sort angle queue nodes in ascending order according to angle

void RearrangeAngleQueue(angle_queue_t* Q, angle_queue_node_t **A);
//rearrange the pointers of angle queue Q using A in the ascending order

void SortIntersection_EDD_Queues_In_Graph(parameter_t *param, struct_graph_node *G, int G_size);
//sort the intersection edd queue for each intersection in graph G

void SortIntersection_EDD_Queue(parameter_t *param, intersection_edd_queue_t* Q);
//sort intersection EDD queue nodes in ascending order according to EDD

void RearrangeIntersection_EDD_Queue(intersection_edd_queue_t* Q, intersection_edd_queue_node_t **A);
//rearrange the pointers of intersection EDD queue Q using A in the ascending order

void SortVehicleMovementQueues_In_Graph(struct_graph_node *G, int G_size);
//sort the vehicle movement queue for each directional edge in graph G

void SortVehicleMovementQueue(vehicle_movement_queue_t* Q);
//sort vehicle movement queue nodes in ascending order according to offset in the directional edge

void RearrangeVehicleMovementQueue(vehicle_movement_queue_t* Q, vehicle_movement_queue_node_t **A);
//rearrange the pointers of vehicle movement queue Q using A in the ascending order

void SortVehicleQueue(vehicle_queue_t* Q);
//sort vehicle queue nodes in ascending order according to offset in the directional edge

void RearrangeVehicleQueue(vehicle_queue_t* Q, vehicle_queue_node_t **A);
//rearrange the pointers of vehicle queue Q using A in the ascending order

void SortDirectionalEdgeQueue(directional_edge_queue_t* Q);
//sort directional edge queue nodes in ascending order according to eid

void RearrangeDirectionalEdgeQueue(directional_edge_queue_t* Q, directional_edge_queue_node_t **A);
//rearrange the pointers of directional_edge_queue Q using A in the ascending order of edge id 

/**********************************************/
void CopyQueue(queue_t *dst_Q, queue_t *src_Q);
//copy the contents of src_Q into those of dst_Q

void ConstructEdgeQueue(edge_queue_t *Q, struct_graph_node *G, int G_size, parameter_t *param);
/* construct edge queue Q by building the same number of entries as the number of edges in G.
   The density of each edge is determined by sensor_density_distribution,
   sensor_density_standard_deviation and sensor_density_maximum_deviation.
   NOTE that the density in struct_graph_node in G is not updated with the new one.
*/

void ConstructDirectionalEdgeQueue(parameter_t *param, directional_edge_queue_t *Q, struct_graph_node *G, int G_size);
//construct directional edge queue Q by building the same number of entries as the number of edges in G.

edge_queue_node_t* FastLookupEdgeQueue(struct_graph_node *G, char *u, char *v, boolean *flip_flag);
//return the pointer to edge queue node corresponding to the edge consisting of vertices u and v using the adjacency list of graph G //@NOTE: [03/07/08] the current simulation version supports the fast lookup for real graph Gr.

directional_edge_queue_node_t* FastLookupDirectionalEdgeQueue(struct_graph_node *G, char *u, char *v);
//return the pointer to directional edge queue node corresponding to the edge consisting of vertices u and v using the adjacency list of graph G where u is tail node and v is head node

edge_queue_node_t* LookupEdgeQueue(edge_queue_t *Q, char *u, char *v, boolean *flip_flag);
//return the pointer to edge queue node corresponding to the edge consisting of vertices u and v through linear searching for edge queue Q

directional_edge_queue_node_t* LookupDirectionalEdgeQueue(directional_edge_queue_t *Q, char *u, char *v);
//return the pointer to directional edge queue node corresponding to the directional edge consisting of vertices u and v where u is tail node and v is head node

int GetEdgeID_MoveType(edge_queue_t *Q, char *u, char *v, MOVE_TYPE *move_type, double *edge_length);
//return the ID (i.e., eid) for the edge consisting of vertices u and v along with move type using the edge queue Q

int FastGetEdgeID_MoveType(struct_graph_node *G, char *u, char *v, MOVE_TYPE *move_type, double *edge_length, directional_edge_queue_node_t **ptr_directional_edge_node_pointer);
//return the ID (i.e., eid) for the edge consisting of vertices u and v along with move type using real graph G along with the pointer to the directed edge in the digraph

edge_queue_node_t* GetEdgeNodeByEID(edge_queue_t *Q, int eid);
//return the edge queue node corresponding to eid; the eid starts from 1.

void ReplaceSubedgeWithSubdivision(subedge_queue_node_t **subedge_in_schedule_table_entry, subedge_queue_node_t *subedge, subedge_queue_t *subedge_queue);
//replace the subedge with the subdivision of the subedge that is the subedge queue and update schedule table entries to let them point to the corresponding subedges

subedge_queue_node_t* InsertSubedgeAfterEID_With_TableEntryUpdate(subedge_queue_t *Q, int eid, subedge_queue_node_t *node);
//insert a new subedge node after the subedge of eid in subedge queue Q along with the update of the table entry to let it point to the correct the pointer to the subedge corresponding to the subedge in the subedge list in the physical edge.

subedge_queue_node_t* InsertSubedgeAfterEID(subedge_queue_t *Q, int eid, subedge_queue_node_t *node);
//insert a new subedge node after the subedge of eid in subedge queue Q

void DeleteSubedgeWithEID(subedge_queue_t *Q, int eid);
//delete the subedge node of eid from subedge queue Q

void DeleteEdgeWithEID(edge_queue_t *Q, int eid);
//delete the edge node of eid from edge queue Q

void DeleteDirectionalEdgeWithEID(directional_edge_queue_t *Q, int eid);
//delete the edge node of eid from directional edge queue Q

void FastDeleteEdge(edge_queue_t *Q, char *tail_node, char *head_node, struct_graph_node *G);
//delete the edge node of (tail_node, head_node) from edge queue Q using graph G

void FastDeleteDirectionalEdge(directional_edge_queue_t *Q, char *tail_node, char *head_node, struct_graph_node *G);
//delete the directional edge node of (tail_node, head_node) from directional edge queue Q using graph G

subedge_queue_node_t* GetSubedgeNodeByEID(subedge_queue_t *Q, int eid);
//return the subedge queue node corresponding to eid; the eid starts from 1.

sensor_queue_node_t* GetSensorQueueNodeJustBeforeVirtualOffset(sensor_queue_t *Q, double offset);
//return the sensor queue node just before or on the given offset in Gv (called virtual offset) in sensor queue Q.

sensor_queue_node_t* GetSensorQueueNodeJustAfterVirtualOffset(sensor_queue_t *Q, double offset);
//return the sensor queue node just after the given offset in Gv (called virtual offset) in sensor queue Q.
void ReplaceSensorQueue(sensor_queue_t *dst_Q, sensor_queue_t *src_Q);
//replace sensor list dst_Q with sensor list src_Q by letting dst_Q have sensor queue nodes of src_Q without allocating new sensor queue nodes and copying them

int GetHoleEndpointsWithHoleSegment(hole_endpoint_queue_t *H, int eid, hole_endpoint_queue_node_t **left_hole_endpoint, hole_endpoint_queue_node_t **right_hole_endpoint);
//get the pointers of two endpoints of the hole segment corresponding to eid 

void DeleteHoleEndpointWithEID(hole_endpoint_queue_t *H, int eid, char *node);
//delete the hole endpoint for eid and node from hole endpoint queue H

void UpdateHoleEndpointEID(hole_endpoint_queue_t *H, int old_eid, char *node, int new_eid);
//replace the hole endpoint's eid (i.e., old_eid) with new_eid

void DeleteSensorQueueNodesJustBeforeVirtualOffset(sensor_queue_t *Q, double offset);
//delete sensor nodes just before offset from the sensor queue Q

void DeleteSensorQueueNodesJustAfterVirtualOffset(sensor_queue_t *Q, double offset);
//delete sensor nodes just after offset from the sensor queue Q

double GetPhysicalOffsetOnSubedgeFromSubedgeList(subedge_queue_t *Q, subedge_queue_node_t *pSubedgeNode, double virtual_offset);
/* get the physical offset of virtual_offset in the edge in real graph Gr corresponding to the offset on the subedge from the subedge list 
   that contains the subedge in virtual graph Gv */

/** functions for VANET */
void DeleteVehicleWithVID(vehicle_queue_t *Q, int vid);
//delete the vehicle node corresponding to vid from vehicle queue Q

void DeleteVehicleMovementWithVID(vehicle_movement_queue_t *Q, int vid);
//delete the vehicle movement node corresponding to vid from vehicle movement queue Q

void DeleteConvoyWithCID(convoy_queue_t *Q, int cid);
//delete the convoy node corresponding to cid from convoy queue Q

void MergeTwoConvoys(convoy_queue_t *Q1, convoy_queue_t *Q2);
//merge two convoys moving one the same directional edge into one convoy and delete one between the merged convoys from the convoy queue for the directional edge; note that the new convoy's id is equal to one whose cid is smaller and the convoy with the bigger is deleted

void SplitConvoy(convoy_queue_t *Q);
//split one convoy into two convoys moving one the same directional edge and register these two convoys into the convoy queue for the directional edge; note that one convoy's id the same as the original one's and another convoy's is set to a unique cid

convoy_queue_node_t* GetConvoyClosestToVehicle(parameter_t *param, convoy_queue_t *Q, double vehicle_offset);
//get the pointer to the convoy node closest to the vehicle within the communication range

void Set_NewConvoyHead(convoy_queue_node_t *convoy);
//find a vehicle placed at the front of the convoy as the convoy's new head

void Set_NewConvoyTail(convoy_queue_node_t *convoy);
//find a vehicle placed at the end of the convoy as the convoy's new tail

void Set_NewConvoyLeader(parameter_t *param, convoy_queue_node_t *convoy);
//find a vehicle with minimum EDD as the convoy's new leader

struct_vehicle_t* Find_Vehicle_Following_Convoy_Head(convoy_queue_node_t *convoy);
//find a vehicle just following the convoy head as the convoy's new head

/** Operations for connected network component */
struct_vehicle_t* Find_Following_Vehicle_Within_Communication_Range_On_Directional_Edge(parameter_t *param, struct_vehicle_t *vehicle, int *acl_convoy_vehicle_number);
//find the neighboring vehicle following vehicle within the communication range on the directional edge

struct_vehicle_t* Find_Vehicle_Farthest_Towards_Edge_Head_Node_In_Connected_Network_Component(parameter_t *param, struct_vehicle_t *vehicle);
/* find the vehicle farthest from vehicle towards the directional edge's head node within the same connected network component (i.e., convoy), assuming that the vehicle movement queue is sorted in the descending order of vehicle's offset. */

/** Queue Operations for TBR */
edge_queue_node_t* Insert_Edge_Into_EdgeQueue(edge_queue_t *Q, struct_graph_node *G, int G_size, char *tail_node, char *head_node, double weight);
//[for TBR] insert a new edge node (tail_node, head_node) into the end of edge queue Q

directional_edge_queue_node_t* Insert_DirectionalEdge_Into_DirectionalEdgeQueue(directional_edge_queue_t *Q, struct_graph_node *G, int G_size, char *tail_node, char *head_node, double weight);
//[for TBR] insert a new directional edge node (tail_node, head_node) into the end of directional edge queue Q

destination_vehicle_queue_node_t* GetDestinationVehicleByVID(destination_vehicle_queue_t *Q, int vid);
//return the destination vehicle queue node corresponding to vid; the vid is greater than 1.

destination_vehicle_queue_node_t* GetFirstDestinationVehicle(destination_vehicle_queue_t *Q);
//return the first destination vehicle queue node in queue Q

boolean Is_Destination_Vehicle(destination_vehicle_queue_t *Q, int vid);
//check whether or not vid is a destination vehicle with destination vehicle queue Q

/** Access Point Queue Operations */
access_point_queue_node_t* GetAccessPointByID(access_point_queue_t *Q, int id);
//return the access point queue node corresponding to id; the id is greater than 0.

access_point_queue_node_t* GetFirstAccessPoint(access_point_queue_t *Q);
//return the first access point queue node in queue Q

access_point_queue_node_t* GetAccessPointByVertex(access_point_queue_t *Q, char *vertex);
//return the access point structure corresponding to vertex

/** Vehicle Trajectory Queue Operations */
void Install_VehicleTrajectory_Into_Packet(parameter_t *param, double current_time, struct_graph_node *G, int G_size, struct_vehicle_t *vehicle, packet_queue_node_t *packet);
//set up packet's vehicle trajectory with vehicle's trajectory 

boolean Install_PacketTrajectory_Into_Packet(parameter_t *param, double current_time, struct_graph_node *G, int G_size, char *packet_src_vertex, char *packet_dst_vertex, packet_queue_node_t *packet);
//set up packet's packet trajectory to its destination vertex with the shortest path from packet_src_vertex to packet_dst_vertex according to param->vehicle_vanet_target_point_selection_type, such as the stationary-node-based static forwarding or dynamic forwarding 

boolean Install_FullPacketTrajectory_Into_Packet(parameter_t *param, double current_time, struct_graph_node *G, int G_size, char *packet_src_vertex, char *packet_dst_vertex, packet_queue_node_t *packet);
//set up packet's full packet trajectory to its destination vertex with the shortest path from packet_src_vertex to packet_dst_vertex under the stationary-node-based static forwarding 	

boolean Install_PartialPacketTrajectory_Into_Packet(parameter_t *param, double current_time, struct_graph_node *G, int G_size, char *packet_src_vertex, char *packet_dst_vertex, packet_queue_node_t *packet);
//set up packet's partial packet trajectory (i.e., the current hop and the next hop) to its destination vertex with the shortest path from packet_src_vertex to packet_dst_vertex under the stationary-node-based dynamic forwarding 					

boolean Install_PacketTrajectory_Into_Packet_With_Edge(parameter_t *param, double current_time, struct_graph_node *G, int G_size, char *tail_node, char *head_node, packet_queue_node_t *packet);
//set up packet's packet trajectory for the edge (tail_node, head_node) under the stationary-node-based dynamic forwarding 					

boolean Install_StaticPacketTrajectory_Into_Packet(parameter_t *param, double current_time, struct_graph_node *G, int G_size, int static_packet_trajectory[], int static_packet_trajectory_size, packet_queue_node_t *packet);
//set up packet's packet trajectory with the static packet trajectory 	

/** Carrier Trace Queue Operations */
void Enqueue_CarrierTraceEntry(parameter_t *param, double current_time, packet_queue_node_t *packet, vanet_node_type_t node_type, void *vanet_node);
//enqueue the packet carrier trace entry into packet's carrier trace queue according to data forwarding mode

void Enqueue_CarrierTraceEntry_For_Upload(parameter_t *param, double current_time, packet_queue_node_t *packet, vanet_node_type_t node_type, void *vanet_node);
//enqueue the packet carrier trace entry into packet's carrier trace queue for upload forwarding mode

void Enqueue_CarrierTraceEntry_For_Download(parameter_t *param, double current_time, packet_queue_node_t *packet, vanet_node_type_t node_type, void *vanet_node);
//enqueue the packet carrier trace entry into packet's carrier trace queue for download forwarding mode

void Enqueue_CarrierTraceEntry_For_V2V(parameter_t *param, double current_time, packet_queue_node_t *packet, vanet_node_type_t node_type, void *vanet_node);
//enqueue the packet carrier trace entry into packet's carrier trace queue for V2V forwarding mode

void Show_CarrierTraceQueue(packet_queue_node_t *packet);
//show the carrier trace in packet's carrier trace queue according to data forwarding mode

void Show_CarrierTraceQueue_For_Upload(packet_queue_node_t *packet);
//show the carrier trace in packet's carrier trace queue for upload forwarding mode

void Show_CarrierTraceQueue_For_Download(packet_queue_node_t *packet);
//show the carrier trace in packet's carrier trace queue for download forwarding mode

/** Packet Queue Operations */
void DestroyPacketQueueNode(packet_queue_node_t *packet);
//destroy packet queue node along with vehicle trajectory queue and carrier trace queue, and then free the memory of packet queue node

/** Forwarding Table Queue Operations */
void InitForwardingTableQueue(forwarding_table_queue_t *FTQ, parameter_t *param, struct_graph_node* Gr, int Gr_size);
//initialize the forwarding table for each intersection in the road network graph Gr

void UpdateForwardingTableQueue(forwarding_table_queue_t *FTQ, parameter_t *param, struct_graph_node* Gr, int Gr_size, access_point_queue_t *APQ);
//update the forwarding table for each intersection in the road network graph Gr with the vehicular traffic statistics for Gr

/** Global Packet Queue Operations */
void InitGlobalPacketQueue_With_VanetInformationTable(global_packet_queue_t *GPQ, parameter_t *param);
//initialize the global packet queue that will contain all of the packet queue nodes in the vehicles in the road network with the VANET Information Table

global_packet_queue_node_t* GetGlobalPacketPointerByID(global_packet_queue_t *Q, int id);
//return the global packet pointer corresponding to globally unique id; the id is greater than 0.

global_packet_queue_node_t* Enqueue_Packet_Into_GlobalPacketQueue(parameter_t *param, double current_time, global_packet_queue_t *Q, packet_queue_node_t *packet);
//enqueue packet into global packet queue Q

global_packet_queue_node_t* Enqueue_Packet_Into_GlobalPacketQueue_With_TargetPoint_Recomputation_Schedule(parameter_t *param, double current_time, global_packet_queue_t *Q, packet_queue_node_t *packet);
//enqueue packet into global packet queue Q along with the schedule of target point recomputation

void DestroyGlobalPacketQueueNode(global_packet_queue_node_t *global_packet);
//dequeue global packet queue node, destroy global packet queue node, and then free the memory of global packet queue node

void ReverseQueue(queue_t *Q);
//reverse the order of queue Q, such as packet_trajectory_queue. 

void ReversePacketTrajectoryQueue(packet_trajectory_queue_t *Q, struct_graph_node* G, int G_size);
//reverse the order of packet trajectory queue Q and set up the distance from the packet source to the vertex

/** Operations for Stationary Node Queue */
void InitStationaryNodeQueue(stationary_node_queue_t *SNQ, struct_graph_node *Gr, int Gr_size);
//initialize the stationary node queue SNQ along with the association with the road network graph

void Update_DestinationVehicle_PassingTime_At_StationaryNode(parameter_t *param, double current_time, struct_vehicle_t *vehicle, char *intersection_vertex);
//update the destination vehicle's latest passing time for the intersection vertex

/** Operations for Probability and Statistics Queue */
void RegisterProbabilityAndStatistics_Per_DirectionalEdge(parameter_t *param, double current_time, struct_graph_node *Gr, int Gr_size, directional_edge_queue_t *DEr);
//register the forwarding probability and statistics per directional edge with the corresponding directional edge queue node with the road network graph Gr

void ConstructConditionalForwardingProbabilityQueueNodes_Per_DirectionalEdge(struct_graph_node *G, int G_size);
//construct as many conditional probability queue nodes as the neighboring edges per the edge where G[i].vertex is the tail node

void MakeConditionalForwardingProbabilityQueue_For_DirectionalEdge(struct_graph_node *tail_gnode, struct_graph_node *head_gnode);
//make the conditional forwarding probability queue for the edge of (tail_gnode, head_gnode)

void CopyConditionalForwardingProbabilityQueueNodes_For_DirectionalEdge(conditional_forwarding_probability_queue_t *Q_src, conditional_forwarding_probability_queue_t *Q_dst);
//copy the conditional forwarding probability queue nodes of queue Q_src to conditional forwarding probability queue Q_dst

conditional_forwarding_probability_queue_node_t* GetConditionalForwardingProbabilityQueueNode(conditional_forwarding_probability_queue_t *Q, char *tail_node, char *head_node);
//get the pointer to a conditional forwarding probability queue node corresponding to the edge (tail_node, head_node) given the current carrier edge with the current carrier edge's conditional forwarding probability queue Q 

void ReplaceTargetPointQueue(target_point_queue_t *dst_Q, target_point_queue_t *src_Q);
//replace target point list dst_Q with target point list src_Q by letting dst_Q have target point queue nodes of src_Q without allocating new target point queue nodes and copying them

void Initialize_GlobalPacketQueue_PacketVectors(parameter_t *param, global_packet_queue_t* Q, int vector_size);
//initialize packet vectors for packet delivery statistics under multiple packet copy transmission for multiple target points

void Reallocate_GlobalPacketQueue_PacketVectors(global_packet_queue_t* Q, int vector_increase_size);
//increase the memory of packet vectors by vector_increase_size to accommodate more packets

/** Queue Operations for TPD */
boolean Delete_Edge_In_NeighborList(adjacency_list_queue_node_t *u,
		adjacency_list_queue_node_t *v);
//delete the edge (u, v) from u's neighbor_list 

boolean Delete_Node_In_AdjacencyList(adjacency_list_queue_node_t *v);
//delete the graph node v from v's queue that is an adjacency list 

int ConstructRoadNetworkGraphSet_And_DirectionalEdgeQueueSet_With_SetAssociation(
		parameter_t *param,
		struct_graph_node *Gr,
		int Gr_size,
		int *pGr_set_number,
		struct_graph_node ***pGr_set,
		int **pGr_set_size,
		directional_edge_queue_t **pDEr_set);
/* construct the set of road network graphs and the set of the corresponding directional edge queues, and then associate these two sets */

int DestroyRoadNetworkGraphSet_And_DirectionalEdgeQueueSet(
		int Gr_set_number,
		struct_graph_node **Gr_set, 
		int *Gr_set_size,
		directional_edge_queue_t *DEr_set);
/* destroy the set of road network graphs and the set of the corresponding directional edge queues */

boolean IsPacketCopy_In_PacketQueue(packet_queue_t *Q, packet_queue_node_t *pPacketNode);
//check whether the packet copy pointed by pPacketNode is already in packet queue Q 

#endif

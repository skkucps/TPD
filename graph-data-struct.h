/**
 *  File: graph-data-struct.h
	Description: Data structures for graph operations
	Date: 09/22/2006	
	Maker: Jaehoon Jeong
*/

#ifndef __GRAPH_DATA_STRUCT_H__
#define __GRAPH_DATA_STRUCT_H__

//#include "common.h"

/** macro constants */

//#define INFINITE 10000
//#define INFINITE 1000000
#define INF 1000000000
//#define INF 0xFFFFFFFF
//#define INF INFINITE
#define NIL -1
#define NAME_SIZE 25
#define BUFFER_SIZE 256
#define GRAPH_FILE_NAME "graph.conf"
#define SOURCE_NAME "A"
#define VIRTUAL_NODE_NEIGHBOR_NAME "-1"

/** graph node type */
typedef enum _GRAPH_NODE_TYPE{
	UNKNOWN_GRAPH_NODE_TYPE = 0,
	INTERSECTION_GRAPH_NODE = 1,    /* intersection node at road graph */
	NONINTERSECTION_GRAPH_NODE = 2, /* virtual graph node for data delivery towards vehicle or nonintersection node at road graph that is the end-point of a sensing hole segment on the road segment */
	ENTRANCE_GRAPH_NODE = 3,        /* entrance node at road graph */
	PROTECTION_GRAPH_NODE = 4,      /* protection node at road graph */
        HOLE_GRAPH_NODE = 5,            /* sensing hole node at road graph */
	ACCESS_POINT_GRAPH_NODE = 6     /* access point node at road graph */
} GRAPH_NODE_TYPE;

typedef enum _GRAPH_NODE_ROLE{
        ROLE_UNKNOWN = 0,
        ROLE_INTERSECTION_POINT = 1,    /* intersection node at virtual graph */
        ROLE_NONINTERSECTION_POINT = 2, /* non-intersection node at virtual graph*/
        ROLE_ENTRANCE_POINT = 3,        /* entrance point at virtual graph */
        ROLE_PROTECTION_POINT = 4,      /* protection point at virtual graph */
        ROLE_HOLE_ENDPOINT = 5,         /* hole endpoint at virtual graph */
        ROLE_ACCESS_POINT = 6           /* access point at virtual graph */

} GRAPH_NODE_ROLE;

/** usage status of graph node */
typedef enum _USAGE_STATUS
{
        USAGE_USED = 0,
        USAGE_UNUSED = 1
} USAGE_STATUS;

/** directional edge type for a graph node */
typedef enum _directional_edge_type_t
{
        OUTGOING_EDGE = 0, //outgoing edge for a vertex
        INCOMING_EDGE = 1 //incoming edge for a vertex
} directional_edge_type_t;

/* coordinate structure */
typedef struct _struct_coordinate1_t
{
	double x;    /* x-coordinate of current position */
	double y;    /* y-coordinate of current position */
} struct_coordinate1_t;

typedef struct _struct_coordinate2_t
{
	int eid;    /* edge id of current position */
	double offset; /* location from the tail of the edge eid; unit [m] */
} struct_coordinate2_t;

/* coordinate directional edge type to specify the location on one diectional edge  */
typedef enum _coordinate_directional_edge_type_t{
        COORDINATE_DIRECTIONAL_EDGE_TYPE_UNKNOWN = 0, //unknown directional edge type
        COORDINATE_DIRECTIONAL_EDGE_TYPE_REGULAR_EDGE = 1, //a regular edge such that tail_node != head_node and weight > 0
        COORDINATE_DIRECTIONAL_EDGE_TYPE_VERTEX_EDGE = 2//a vertex edge such that tail_node == head_node and weight == 0
} coordinate_directional_edge_type_t;

typedef struct _struct_coordinate3_t
{
        coordinate_directional_edge_type_t type; //type of directional edge for coordinate
        int eid; //edge id of current position
        char tail_node[NAME_SIZE]; //tail node of a directional edge
        char head_node[NAME_SIZE]; //head node of a directional edg
        double weight; //weight for edge
        double offset; //location from the tail of the edge eid; unit [m]
        struct _directional_edge_queue_node_t *enode; //pointer to the directional edge queue node corresponding to eid
} struct_coordinate3_t;

struct _schedule_table_node_t; //predeclaration of struct _schedule_table_node_t

struct _edge_queue_node_t; //predeclaration of struct _edge_queue_node_t

/* boolean type */
#ifndef __BOOLEAN__
#define __BOOLEAN__
typedef enum _boolean{
	FALSE = 0,
	TRUE = 1
} boolean;
#endif

typedef struct _struct_graph_node
{
        char vertex[NAME_SIZE]; //vertex name
        USAGE_STATUS status; //status to indicate that this graph node is not used any more, so it can be used for another graph node
        double weight; //the number of edges in the node array or the edge length in the neighbor list
	GRAPH_NODE_TYPE type; //graph node type = {INTERSECTION_GRAPH_NODE, NONINTERSECTION_GRAPH_NODE, ENTRANCE_GRAPH_NODE, PROTECTION_GRAPH_NODE, HOLE_GRAPH_NODE, ACCESS_POINT_GRAPH_NODE};
        GRAPH_NODE_ROLE role; //graph node role = {ROLE_INTERSECTION_POINT, ROLE_NONINTERSECTION_POINT, ROLE_ENTRANCE_POINT, ROLE_PROTECTION_POINT, ROLE_HOLE_ENDPOINT, ROLE_ACCESS_POINT};
        struct_coordinate1_t coordinate; //geographic coordinate in roadmap in the 2-D Cartesian coordinate system
	double density; //density of sensors deployed for unit distance
        double offset_in_Gr; //only for hole graph node, this is the offset at an physical edge in real graph Gr; for other kinds of graph nodes, the offset in this field are set to zero.
		struct _struct_graph_node *next; //neighbor node for vertex
        struct _struct_graph_node *gnode; //pointer to the graph node in graph G array corresponding to this node's vertex in order to access the information (such as coordinate) of the graph node corresponding to this node
        struct _schedule_table_node_t *ptr_table_node; //pointer to the table node of an edge which has this graph node as an endpoint
        struct _edge_queue_node_t *ptr_edge_node; //pointer to the edge node in edge queue Er for real graph Gr
        struct _directional_edge_queue_node_t *ptr_directional_edge_node; //pointer to the directional edge node in directional edge queue DEr for real graph Gr

		boolean stationary_node_flag; //flag to indicate whether this intersection has a working stationary node or not

        struct _stationary_node_queue_node_t *ptr_stationary_node; //pointer to the stationary node in the stationary node queue

        struct _angle_queue_t *angle_queue; //angle queue to enumerate the neighboring nodes in the increasing order of EDD

        struct _intersection_edd_queue_t *intersection_edd_queue; //intersection EDD queue to select an edge with the smallest EDD with a vehicle within the communication range of the intersection corresponding to this graph node

        struct _conditional_forwarding_probability_queue_t *conditional_forwarding_probability_queue; //conditional forwarding probability queue that contains conditional forwarding probability queue nodes for the neighboring edges of the edge where this vertex is the head node

        /** backup information for the recovery into the original edge in real graph Gr */
        char original_vertex[NAME_SIZE]; //original vertex name for the head node that is a virtual node in the augmented graph Ga
        double original_weight; //original weight of the edge before a virtual node subdivides this edge
	GRAPH_NODE_TYPE original_type; 
        GRAPH_NODE_ROLE original_role;
        struct _struct_graph_node *original_gnode;
        //struct _edge_queue_node_t *original_ptr_edge_node;
        //struct _directional_edge_queue_node_t *original_ptr_directional_edge_node;

        /** statistics for vehicular traffic density and branch probability */
        double mean_interarrival_time; //average interarrival time
        double lambda; //arrival rate per unit time (1 sec) = 1/mean_interarrival_time
        double last_arrival_time; //last arrival time
        double sum_of_interarrival_time; //sum of interarrival time
        int number_of_interarrivals; //number of interarrivals    
        int number_of_arrivals; //number of arrivals that is used for the denominator of the branch probability
        int number_of_branching; //number of branching from the tail node to this node that is a head node where the edge is <tail_node, head_node>; this is equal to number_of_interarrivals+1.

        /** variables for computing the forwarding probability P_ij */
        double CP; //contacting probability for a packet carrier to meet at least one contact vehicle towards road r_ij, when the carrier moves within the intersection area of I_i 
        double theta; //angle in degrees between the vehicle movement vector and the destination vector
        double Q; //probability of a vehicle moving from the current intersection I_i towards the next adjacent intersection I_j
        double P_prime; //probability of a packet being forwarded to road r_ij at intersection I_i
        double P_prime_pure; //forwarding probability of a packet being forwarded to road r_ij at intersection I_i in TBD
        double P; //probability that a packet is forwarded through road r_ij at intersection I_i; note that P includes the probability that the packet carrier continues to carry packet without actual forwarding
        double P_pure; //pure forwarding probability that a packet is forwarded into a vehicle moving on the branch by the original packet carrier; note that P_pure is used to compute EDD based on vehicle trajectory.

		/** edge delay statistics */
        double edge_delay; //edge delay consisting of forwarding delay and carry delay on this edge
        double edge_delay_variance; //edge delay variance
        double edge_delay_standard_deviation; //edge delay standard deviation
        double EDD; //Expected Delivery Delay (EDD) through this edge, i.e., the delivery delay's first moment; Note that under download mode, EDD is the EDD from this intersection to the target point
        double EDD_SD; //Expected delivery delay standard deviation; Note that under download mode, EDD is the EDD from this intersection to the target point
        double EDD_VAR; //Expected delivery delay variance; Note that under download mode, EDD is the EDD from this intersection to the target point
        //double EDD_2_Moment; //Expected delivery delay second_moment

		/** edge cost (e.g., link utilization or link delay) statistics */
        double edge_cost; //edge cost on this edge
        double edge_cost_variance; //edge cost variance
        double edge_cost_standard_deviation; //edge cost standard deviation
        double EDC; //Expected Delivery Cost (EDC) through this edge, i.e., the delivery cost's first moment; Note that under download mode, EDC is the EDC from this intersection to the target point
        double EDC_SD; //Expected delivery cost standard deviation; Note that under download mode, EDC is the EDC from this intersection to the target point
        double EDC_VAR; //Expected delivery cost variance; Note that under download mode, EDC is the EDC from this intersection to the target point

        /** variables for expected average convoy length and actual average convoy length */
        double expected_average_convoy_length; //expected average convoy length (ACL)
        double actual_average_convoy_length; //measured average convoy length (ACL)
} struct_graph_node;

struct _struct_set_node; //predeclaration of struct _struct_set_node

typedef struct _struct_shortest_path_node
{
	char vertex[NAME_SIZE];
	double dist;
	char parent[NAME_SIZE];
	struct_graph_node *gnode; //indicate the pointer to the graph node in adjacent list
	//void *pnode; //parent node in set S towards the source whose type is struct_set_node*
	struct _struct_set_node *pnode; //parent node in set S towards the source whose type is struct_set_node*
} struct_shortest_path_node;

typedef struct _struct_set_node
{
	struct_shortest_path_node node;
	struct _struct_set_node *prev;
	struct _struct_set_node *next;
} struct_set_node;

/* structure of traffic node */
typedef struct _struct_traffic_node
{
	char vertex[NAME_SIZE];
} struct_traffic_node;

/* structure of traffic table */
typedef struct _struct_traffic_table
{
	int number; //number of sources or destinations
	struct_traffic_node *list; //list of sources or destinations
} struct_traffic_table;


typedef struct _struct_path_table
{
	int number; //number of shortest path sets
	struct_set_node **list; //list of shortest path sets
} struct_path_table;

/*
typedef struct _struct_sensor_table
{
	int number; //number of table entries
	sensor_queue_node_t **list; //list of sensor node points
} struct_sensor_table;
*/

typedef struct _struct_path_node
{
	char vertex[NAME_SIZE];
	double weight;
	double edge_travel_delay; //the mean travel delay for the edge
	double edge_travel_delay_standard_deviation; //the travel delay standard deviation for the edge
	double expected_arrival_time; //vehicle's expected arrival time to this path node
	double travel_time; //the mean of vehicle's travel time to this path node
	double travel_time_standard_deviation; //the standard deviation of vehicle's travel time to this path node
	struct _struct_path_node *prev; //previous node on the path towards the destination
	struct _struct_path_node *next; //next node on the path towards the destination
} struct_path_node;

/* Schedule Table for Traffic Sources */
typedef struct _struct_schedule_entry
{
	char vertex[NAME_SIZE]; //vertex name indicating intersection ID in road network
	double timestamp; //time when this entry was updated
	double sched_time; //schedule time when a new vehicle arrives from the corresponding source
} struct_schedule_entry;

typedef struct _struct_schedule_table
{
	int number; //number of entries
	struct _struct_schedule_entry *entry;
} struct_schedule_table;

#endif

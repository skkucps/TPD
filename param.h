/** File: param.h
	Description: specify the structure of parameter and the related constants
	Date: 04/07/2005
	Maker: Jaehoon Jeong, jjeong@cs.umn.edu
*/

#ifndef __PARAM_H__
#define __PARAM_H__

#include "common.h"
#include <stdlib.h> //atoi(), atof()
#include <stdio.h>

#include "graph-data-struct.h"

/** Macro Constants */
#define KILO 1000 // KILO = 10^3
#define MEGA 1000000 // MEGA = 10^6
#define GIGA 1000000000 // GIGA = 10^9
#define MILLI 0.001 //MILLI = 10^(-3)
#define BUF_SIZE 255 //buffer size
#define OPERAND_NUM 10 //number of operands

#define INF_FACTOR 10000

/** Enum Types */

/* boolean type */
#ifndef __BOOLEAN__
#define __BOOLEAN__
typedef enum _boolean{
	FALSE = 0,
	TRUE = 1
} boolean;
#endif

/* performance comparison target type: option -a */
typedef  enum _comparison_target_type_t
{ COMPARISON_UNKNOWN = 0,
  /* types for VANET */
  COMPARISON_EDD_AND_LINK_MODEL = 1,
  COMPARISON_EDD_MODEL = 2,
  COMPARISON_TBD_EDD_COMPUTATION_TYPE = 3,
  COMPARISON_EDGE_DELAY_MODEL = 4,
  COMPARISON_INTERSECTION_FORWARDING_TYPE = 5,
  COMPARISON_AVERAGE_CONVOY_LENGTH_ESTIMATION_TYPE = 6,
  COMPARISON_AVERAGE_LINK_DELAY_ESTIMATION_TYPE = 7,
  COMPARISON_TARGET_POINT_SELECTION_TYPE = 8,
  COMPARISON_TARGET_POINT_COMPUTATION_METHOD = 9,
  COMPARISON_TARGET_POINT_NUMBER = 10,

  /* types for sensor network */
  COMPARISON_SCHEDULE_ALGORITHM = 21,
  COMPARISON_HOLE_LABELING_ALGORITHM = 22,
  COMPARISON_HOLE_LABELING_MODE = 23
} comparison_target_type_t;

/* performance evaluation type: option -b */
typedef enum _evaluation_type_t
{ EVALUATION_UNKNOWN = 0,

  /* types for VANET */
  EVALUATION_VEHICLE_MAXIMUM_NUMBER = 1,
  EVALUATION_VEHICLE_PACKET_GENERATING_ENTITY_NUMBER = 2,
  EVALUATION_VEHICLE_SPEED = 3,
  EVALUATION_VEHICLE_SPEED_STANDARD_DEVIATION = 4,
  EVALUATION_VEHICLE_INTERARRIVAL_TIME = 5,
  EVALUATION_COMMUNICATION_PACKET_INTERARRIVAL_TIME = 6,
  EVALUATION_COMMUNICATION_PACKET_TTL = 7,
  EVALUATION_COMMUNICATION_PACKET_DELIVERY_PROBABILITY_THRESHOLD = 8,
  EVALUATION_COMMUNICATION_AP_MAXIMUM_NUMBER = 9,
  EVALUATION_SIMULATION_TIME = 10,
  EVALUATION_VEHICLE_AP_PASSING_ENTITY_PERCENTAGE = 11,
  EVALUATION_COMMUNICATION_SN_MAXIMUM_NUMBER = 12,

  /* types for sensor network */
  EVALUATION_WORK_TIME = 21,
  EVALUATION_ENERGY_VARIATION = 22,
  EVALUATION_SENSOR_DENSITY = 23,
  EVALUATION_SENSOR_DENSITY_VARIATION = 24,
  EVALUATION_PATH_LENGTH_VARIATION = 25
} evaluation_type_t;

/* length unit type */
typedef enum _length_unit_type_t
{ UNKNOWN_LENGTH_UNIT = 0,
  UNIT_METER = 1, //meter length unit
  UNIT_MILE = 2 //mile length unit
} length_unit_type_t; //length unit type

/* sensor schedule mode type in simulation in terms of energy budget update */
typedef enum _sensor_schedule_mode_type_t
{ SENSOR_SCHEDULE_MODE_UNKNOWN = 0,  
  SENSOR_SCHEDULE_MODE_EAGER_UPDATE = 1,
  SENSOR_SCHEDULE_MODE_LAZY_UPDATE = 2
} sensor_schedule_mode_type_t;

/* step mode for vehicle movement */
typedef enum _vehicle_step_mode_type_t
{ UNKNOWN_STEP_MODE = 0,
  STEP_TIME = 1, //vehicle moves by step time
  STEP_EDGE = 2, //vehicle moves by the movement time for the edge corresponding to each road segment	
  STEP_PATH = 3  //vehicle moves by the movement time for the path from the source to the destination	

} vehicle_step_mode_type_t;

/* VANET Vehicular Traffic Model: option -m */
typedef enum _vanet_vehicular_traffic_model_type_t
{
  VANET_VEHICULAR_TRAFFIC_UNKNOWN_MODEL = 0, //Unknown VANET Vehicular Traffic Model
  VANET_VEHICULAR_TRAFFIC_CLOSED_NETWORK = 1, //VANET Closed Network where vehicles arriving at sources move around the road network without leaving the road network
  VANET_VEHICULAR_TRAFFIC_OPEN_NETWORK = 2 //VANET Open Network where vehicles arriving at sources leave the road network when they arrive at their destinations
} vanet_vehicular_traffic_model_type_t;

/* VANET EDD-Link model: option -l */
typedef enum _vanet_edd_and_link_model_type_t
{
  VANET_EDD_AND_LINK_MODEL_UNKNOWN = 0, //Unknown VANET EDD-Link Model
  VANET_EDD_AND_LINK_MODEL_TBD_EDD_AND_TBD_LINK = 1, //TBD EDD + TBD Link Model
  VANET_EDD_AND_LINK_MODEL_TBD_EDD_AND_VADD_LINK = 2, //TBD EDD + VADD Link Model
  VANET_EDD_AND_LINK_MODEL_VADD_EDD_AND_VADD_LINK = 3, //VADD EDD + VADD Link Model
  VANET_EDD_AND_LINK_MODEL_VADD_EDD_AND_TBD_LINK = 4, //VADD EDD + TBD Link Model
  VANET_EDD_AND_LINK_MODEL_STBD_EDD_AND_STBD_LINK = 5 //STBD EDD + STBD Link Model
} vanet_edd_and_link_model_type_t;

/* VANET EDD model: option -e */
typedef enum _vanet_edd_model_type_t
{
  VANET_EDD_UNKNOWN_MODEL = 0, //Unknown VANET EDD Model
  VANET_EDD_PER_VEHICLE_MODEL = 1, //VANET Per-Vehicle Model for EDD Computation: TBD Model
  VANET_EDD_PER_INTERSECTION_MODEL = 2, //VANET Per-Intersection Model for EDD Computation: VADD Model
  VANET_EDD_PER_SNODE_MODEL = 3, //VANET Per-Stationary-Node Model for EDD Computation: STBD Model
  VANET_EDD_HYBRID_MODEL = 4 //VANET Hybrid Model for EDD Computation that is the linear combination of Per-Vehicle EDD and Per-Intersection EDD; note that the weights of two EDDs for the linear combination are adjusted by the density of vehicular traffic in the given road network
} vanet_edd_model_type_t;

/* VANET EDD Computation Model Type: option is not determined yet! */
typedef enum _vanet_edd_computation_model_type_t
{
  VANET_EDD_UNKNOWN_COMPUTATION_MODEL = 0,
  VANET_EDD_COMPUTATION_BASED_ON_STOCHASTIC_MODEL = 1, //EDD computation based on the stochastic model
  VANET_EDD_COMPUTATION_BASED_ON_SHORTEST_PATH_MODEL = 2 //EDD computation based on the shortest path model
} vanet_edd_computation_model_type_t;

/* VANET TBD's EDD Computation Type: option -c */
typedef enum _vanet_tbd_edd_computation_type_t
{
  VANET_EDD_UNKNOWN_COMPUTATION_TYPE = 0,
  VANET_EDD_BASED_ON_TBD_WITH_FULL_PATH = 1, //EDD based on TBD with the path from the current position to the destination
  VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH = 2, //EDD based on TBD with the path from the current position to the first visisted AP
  VANET_EDD_BASED_ON_TBD_WITH_ONE_HOP_AHEAD = 3 //EDD based on TBD with one-hop-ahead intersection
} vanet_tbd_edd_computation_type_t;

/* VANET Edge Delay model: option -d */
typedef enum _vanet_edge_delay_model_type_t
{
  VANET_EDGE_DELAY_UNKNOWN_MODEL = 0, //Unknown VANET Edge Delay Model
  VANET_EDGE_DELAY_TBD_MODEL_FOR_FINITE_ROAD = 1, //VANET TBD Model for Edge Delay Computation
  VANET_EDGE_DELAY_TBD_MODEL_FOR_INFINITE_ROAD = 2, //VANET TBD Model for Edge Delay Computation for Infinite Road
  VANET_EDGE_DELAY_VADD_MODEL = 3, //VANET VADD Model for Edge Delay Computation
  VANET_EDGE_DELAY_TSF_MODEL = 4 //VANET TSF Model for Edge Delay Computation
} vanet_edge_delay_model_type_t;

/* VANET Forwarding Type: option -f */
typedef enum _vanet_forwarding_type_t
{
  VANET_FORWARDING_UNKNOWN_TYPE = 0,
  VANET_FORWARDING_BASED_ON_VEHICLE = 1, //VANET forwarding based on individual vehicle
  VANET_FORWARDING_BASED_ON_CONVOY = 2   //VANET forwarding based on convoy
} vanet_forwarding_type_t;

/* VANET Intersection Forwarding Type: option -h */
typedef enum _vanet_intersection_forwarding_type_t
{
  VANET_INTERSECTION_FORWARDING_UNKNOWN_TYPE = 0,
  VANET_INTERSECTION_FORWARDING_DEFERRED_FORWARDING = 1, //VANET Deferred forwarding at intersection
  VANET_INTERSECTION_FORWARDING_EAGER_FORWARDING = 2   //VANET Eager forwarding at intersection
} vanet_intersection_forwarding_type_t;


/* VANET Target Point Selection Type: option -S */
typedef enum _vanet_target_point_selection_type_t
{
  VANET_TARGET_POINT_SELECTION_UNKNOWN = 0,
  VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT = 1, //optimal target point selection based on the packet trajectory with stationary nodes
  VANET_TARGET_POINT_SELECTION_STATIC_TARGET_POINT = 2, //static target point selection that the target point is determined only by AP
  VANET_TARGET_POINT_SELECTION_CONVERGENT_TARGET_POINT = 3, //convergent target point selection that the target point is getting closer to the destination vehicle
  VANET_TARGET_POINT_SELECTION_REVERSE_PATH_TARGET_POINT = 4, //reverse target point selection that the target point is determined as an intersection from the reverse path on the destination vehicle's trajectory
  VANET_TARGET_POINT_SELECTION_PROGRESS_TARGET_POINT = 5, //PROGRESS target point selection that the intermediate vehicles on the destination vehicle trajectory recomputes a new target point at an interval of EDD(tp(i))/k where tp(i) is the current target point
  VANET_TARGET_POINT_SELECTION_ADAPTIVE_TARGET_POINT = 6, //adaptive target point selection that the intermediate vehicles on the destination vehicle trajectory recomputes a new target point
  VANET_TARGET_POINT_SELECTION_DYNAMIC_TARGET_POINT = 7 //dynamic target point selection that the intermediate vehicles recompute a new target point dynamically during the packet delivery
} vanet_target_point_selection_type_t;

/* VANET Target Point Computation Method: option -C */
typedef enum _vanet_target_point_computation_method_t
{
  VANET_TARGET_POINT_COMPUTATION_METHOD_UNKNOWN = 0,

  VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_PARTIALLY_DYNAMIC_FORWARDING = 1, //target point computation method for an optimal target point, based on the packet trajectory with the partially dynamic forwarding through stationary nodes
  VANET_TARGET_POINT_COMPUTATION_METHOD_END_INTERSECTION = 2, //target point computation method to choose the destination's end intersection in the vehicle trajectory as target point
  VANET_TARGET_POINT_COMPUTATION_METHOD_RANDOM_INTERSECTION = 3, //target point computation method to choose a random intersection among the intersections on the destination vehicle trajectory

  VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_STATIC_FORWARDING = 4, //target point computation method for an optimal target point, based on the packet trajectory with the static forwarding through stationary nodes
  VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_FULLY_DYNAMIC_FORWARDING = 5, //target point computation method for an optimal target point, based on the packet trajectory with the fully dynamic forwarding through stationary nodes
  VANET_TARGET_POINT_COMPUTATION_METHOD_OPTIMAL_INTERSECTION = 6, //target point computation method for an optimal target point 
  VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_EARLIEST_ARRIVING_INTERSECTION = 7, //target point computation method for a target point to choose an intersection where the packet arrives at the intersection earlier than the destination vehicle with the smallest EDD
  VANET_TARGET_POINT_COMPUTATION_METHOD_HEADING_INTERSECTION = 8 //target point computation method to choose the destination's heading intersection as target point
} vanet_target_point_computation_method_t;

/* VANET Target Point Search Type: option -P */
typedef enum _vanet_target_point_search_space_type_t
{
  VANET_TARGET_POINT_SEARCH_SPACE_UNKNOWN = 0,
  VANET_TARGET_POINT_SEARCH_SPACE_VALID_FULL_TRAJECTORY = 1, //the search space is the valid full trajectory from the destination vehicle's current position to the trajectory end position
  VANET_TARGET_POINT_SEARCH_SPACE_VALID_PARTIAL_TRAJECTORY = 2 //the search space is the valid partial trajectory from the destination vehicle's current position to the next target point candidate's position; the actual target point is selected to be closer to the destination vehicle during the packet delivery process. 
} vanet_target_point_search_space_type_t;

/* VANET Vehicle Trajectory Type: option -T */
typedef enum _vanet_vehicle_trajectory_type_t
{
  VANET_VEHICLE_TRAJECTORY_UNKNOWN = 0,
  VANET_VEHICLE_TRAJECTORY_FULL = 1, //the full vehicle trajectory type
  VANET_VEHICLE_TRAJECTORY_PARTIAL = 2 //the partial vehicle trajectory type
} vanet_vehicle_trajectory_type_t;

/* VANET Vehicle Trajectory Length Type: option -D */
typedef enum _vanet_vehicle_trajectory_length_type_t
{
  VANET_VEHICLE_TRAJECTORY_LENGTH_UNKNOWN = 0,
  VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE = 1, //the infinite vehicle trajectory length type
  VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE = 2 //the finite vehicle trajectory length type
} vanet_vehicle_trajectory_length_type_t;

/* VANET Target Point Optimization Function Type */
typedef enum _vanet_target_point_optimization_function_type_t
{
  VANET_TARGET_POINT_OPTIMIZATION_FUNCTION_UNKNOWN = 0,
  VANET_TARGET_POINT_OPTIMIZATION_FUNCTION_EDD_AND_EAD_DIFFERENCE = 1, //difference of EDD and EAD
  VANET_TARGET_POINT_OPTIMIZATION_FUNCTION_EAD_WITH_CONSTRAINT_1 = 2, //EAD satisfying the constraint 1 that EDD + c*EDD_SD <= EAD - c*EAD_SD <===> EAD - EDD >= c*EAD_SD + c*EDD_SD, where c is the coefficient for delay deviation width
  VANET_TARGET_POINT_OPTIMIZATION_FUNCTION_EAD_WITH_CONSTRAINT_2 = 3 //EAD satisfying the constraint 2 that P(DD > AD) <= epsilon <===> P(DD <= AD) >= 1 - epsilon, where epsilon is missing probability threshold 
} vanet_target_point_optimization_function_type_t;

/* vanet metric type */
typedef enum _vanet_metric_type_t
{
  VANET_METRIC_UNKNOWN = 0, //unknown vanet node type
  VANET_METRIC_EDD = 1, //EDD: Expected E2E Delivery Delay
  VANET_METRIC_EDD_VAR = 2 //EDD_VAR: E2E Delivery Delay Variance
} vanet_metric_type_t;

/* vanet packet forwarding mode */
typedef enum _vanet_packet_forwarding_mode_t
{
	FORWARDING_MODE_UNKNOWN = 0,
	FORWARDING_MODE_RANDOM_WALK = 1,
	FORWARDING_MODE_SOURCE_ROUTING = 2
} vanet_packet_forwarding_mode_t;

/* VANET Forwarding Scheme: option -f */
typedef enum _vanet_forwarding_scheme_t
{
  VANET_FORWARDING_UNKNOWN = 0, //unknown vanet forwarding scheme
  VANET_FORWARDING_VADD = 1, //VADD
  VANET_FORWARDING_TBD = 2,  //TBD
  VANET_FORWARDING_TPD = 3,  //TPD
  VANET_FORWARDING_EPIDEMIC = 4, //Epidemic Routing
  VANET_FORWARDING_TSF = 5,  //TSF
  VANET_FORWARDING_TMA = 6,   //TMA
  VANET_FORWARDING_TADB = 7
} vanet_forwarding_scheme_t;

#define NUMBER_OF_VANET_FORWARDING_SCHEME 7 

/* vanet forwarding scheme name */
#define STRING_FOR_VANET_FORWARDING_UNKNOWN "UNKNOWN_SCHEME"
#define STRING_FOR_VANET_FORWARDING_VADD "VADD"
#define STRING_FOR_VANET_FORWARDING_TBD "TBD"
#define STRING_FOR_VANET_FORWARDING_TPD "TPD"
#define STRING_FOR_VANET_FORWARDING_EPIDEMIC "EPIDEMIC"
#define STRING_FOR_VANET_FORWARDING_TSF "TSF"
#define STRING_FOR_VANET_FORWARDING_TMA "TMA"
#define STRING_FOR_VANET_FORWARDING_TADB "TADB"

/* distribution type */
typedef enum _distribution_type_t
{ UNKNOWN_DISTRIBUTION = 0, //unknown distribution
  EQUAL = 1,        //equal distribution of network means that sensors are located by the same insterval each other or other equal distribution makes the input delay be equal to the output delay
  UNIFORM = 2,     //continuous uniform distribution
  NORMAL = 3,      //normal distribution
  EXPONENTIAL = 4, //exponential distribution
  ERLANG = 5,      //k-Erlang distribution
  HYPERX = 6      //2-stage hyperexponential distribution

} distribution_type_t;

#define NUMBER_OF_DISTRIBUTION_TYPE 13

/* distribution type name */
#define STRING_FOR_UNKNOWN_DISTRIBUTION "UNKNOWN_DISTRIBUTION"
#define STRING_FOR_EQUAL "EQUAL"
#define STRING_FOR_UNIFORM "UNIFORM"
#define STRING_FOR_NORMAL "NORMAL"
#define STRING_FOR_EXPONENTIAL "EXPONENTIAL"
#define STRING_FOR_ERLANG "ERLANG"
#define STRING_FOR_HYPERX "HYPERX"

/* sensor scan type */
typedef enum _sensor_scan_type_t
{ SCAN_UNKNOWN = 0,
  SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING = 1,
  SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING = 2, //perform virtual scanning without 
  //the sleeping time corresponding to the movement time over the shortest path
  SCAN_NO_USE = 3, //there is no use of scan to get additional sleeping time; 
  //that is, all-sensor-turn-on after sleeping time based on physically shortest path 
  //from entrance node to exit node
  SCAN_TURN_ON_ALL = 4, //there is no sleeping time using movement time or scanning time
  SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING = 5,
  SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING_AND_WITH_SENSING_HOLE_HANDLING = 6,
  SCAN_NO_USE_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING = 7,
  SCAN_VARIABLE_SPEED_WITH_VARIABLE_SLEEPING_TIME_AND_WITH_SENSING_HOLE_HANDLING = 8,
  SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_VARIABLE_WORKING_TIME = 9,
  SCAN_CONSTANT_SPEED_WITH_NONOPTIMAL_SLEEPING = 10, 
  SCAN_VARIABLE_SPEED_WITH_NONOPTIMAL_SLEEPING = 11,
  SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING = 12, 
  SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING = 13
} sensor_scan_type_t;

#define NUMBER_OF_SCAN_TYPE 14

/* scan type name */
#define STRING_FOR_SCAN_UNKNOWN "SCAN_UNKNOWN"
#define STRING_FOR_SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING "SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING"
#define STRING_FOR_SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING "SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING"
#define STRING_FOR_SCAN_NO_USE "SCAN_NO_USE"
#define STRING_FOR_SCAN_TURN_ON_ALL "SCAN_TURN_ON_ALL"
#define STRING_FOR_SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING "SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING"
#define STRING_FOR_SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING_AND_WITH_SENSING_HOLE_HANDLING "SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING_AND_WITH_SENSING_HOLE_HANDLING"
#define STRING_FOR_SCAN_NO_USE_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING "SCAN_NO_USE_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING"
#define STRING_FOR_SCAN_VARIABLE_SPEED_WITH_VARIABLE_SLEEPING_TIME_AND_WITH_SENSING_HOLE_HANDLING "SCAN_VARIABLE_SPEED_WITH_VARIABLE_SLEEPING_TIME_AND_WITH_SENSING_HOLE_HANDLING"
#define STRING_FOR_SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_VARIABLE_WORKING_TIME "SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_VARIABLE_WORKING_TIME"
#define STRING_FOR_SCAN_CONSTANT_SPEED_WITH_NONOPTIMAL_SLEEPING "SCAN_CONSTANT_SPEED_WITH_NONOPTIMAL_SLEEPING"
#define STRING_FOR_SCAN_VARIABLE_SPEED_WITH_NONOPTIMAL_SLEEPING "SCAN_VARIABLE_SPEED_WITH_NONOPTIMAL_SLEEPING"
#define STRING_FOR_SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING "SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING"
#define STRING_FOR_SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING "SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING"

/* simulation mode type */
typedef enum _simulation_seed_type_t
{ SEED_DEFAULT = 0,
  SEED_TIME = 1	
} simulation_seed_type_t;

typedef enum _simulation_mode_type_t
{ TIME = 0,
  ITERATION = 1
} simulation_mode_type_t;

/* simulation running type */
typedef enum _simulation_run_type_t
{ SIM_SINGLE_SIMULATION = 0,          /* single simulation */
  SIM_SENSOR_TIME_SYNC_ERROR_STD = 1, /* multiple simulation according to sensor time sync error standard deviation (STD) */
  SIM_VEHICLE_SPEED_STD = 2,          /* multiple simulation according to vehicle speed STD */
  SIM_VEHICLE_INTERARRIVAL_TIME = 3,  /* multiple simulation according to vehicle interarrival time */
  SIM_SENSOR_TIME_SYNC_ERROR_STD_VERSUS_VEHICLE_SPEED_STD = 4,  /* multiple simulation according to sensor time sync error STD versus vehicle speed STD */
  SIM_SENSOR_DETECTION_MISSING_PROBABILITY = 5, /* multiple simulation according to sensor detection missing probability */
  SIM_SENSOR_DUPLICATE_DETECTION_PROBABILITY = 6 /* multiple simulation according to sensor duplicate detection probability */
} simulation_run_type_t;

typedef enum _aggregation_type_t
{ AGGREGATION_TYPE_0 = 0, //no use of data aggregation
  AGGREGATION_TYPE_1 = 1  //use of data aggregation	
} aggregation_type_t;

typedef enum _prefilter_type_t
{ PREFILTER_TYPE_0 = 0, //No PREFILTER: mean of Mv's + no use of prefilter based on relative error for Mv's + no use of prefilter based minimum spanning tree
  PREFILTER_TYPE_1 = 1, //PREFILTER: mean of Mv's + no use of prefilter based on relative error for Mv's + prefilter based minimum spanning tree
  PREFILTER_TYPE_2 = 2, //PREFILTER: mean of Mv's + prefilter based on relative error for Mv's + no use of prefilter based minimum spanning tree
  PREFILTER_TYPE_3 = 3  //PREFILTER: mean of Mv's + prefilter based on relative error for Mv's + prefilter based minimum spanning tree
} prefilter_type_t;

/** algorithm type for handling sensing holes */
typedef enum _hole_handling_algorithm_t
{ HOLE_HANDLING_UNKNOWN = 0,
  HOLE_HANDLING_EXHAUSTIVE_SEARCH_ALGORITHM = 1,                  //exhaustive search algorithm
  HOLE_HANDLING_GREEDY_ALGORITHM_BASED_HEURISTICS = 2,            //greedy algorithm based on heuristics
  HOLE_HANDLING_GREEDY_ALGORITHM_BASED_MINIMAL_SPANNING_TREE = 3, //greedy algorithm based on minimal spanning tree
  HOLE_HANDLING_RANDOM_LABELING = 4,                              //all of the holes are randomly labeled as either entrance point or protection point
  HOLE_HANDLING_NO_HANDLING = 5,                                  //no handling
  HOLE_HANDLING_ALL_ENTRANCE_POINTS = 6,                          //all of the holes are labeled as entrance points
  HOLE_HANDLING_ALL_PROTECTION_POINTS = 7                         //all of the holes are labeled as protection points

} hole_handling_algorithm_t;

#define NUMBER_OF_HOLE_HANDLING_TYPE 8

/* sensing hole handling algorithm name */
#define STRING_FOR_HOLE_HANDLING_UNKNOWN "HOLE_HANDLING_UNKNOWN"
#define STRING_FOR_HOLE_HANDLING_EXHAUSTIVE_SEARCH_ALGORITHM "HOLE_HANDLING_EXHAUSTIVE_SEARCH_ALGORITHM"
#define STRING_FOR_HOLE_HANDLING_GREEDY_ALGORITHM_BASED_HEURISTICS "HOLE_HANDLING_GREEDY_ALGORITHM_BASED_HEURISTICS"
#define STRING_FOR_HOLE_HANDLING_GREEDY_ALGORITHM_BASED_MINIMAL_SPANNING_TREE "HOLE_HANDLING_GREEDY_ALGORITHM_BASED_MINIMAL_SPANNING_TREE"
#define STRING_FOR_HOLE_HANDLING_RANDOM_LABELING "HOLE_HANDLING_RANDOM_LABELING"
#define STRING_FOR_HOLE_HANDLING_NO_HANDLING "HOLE_HANDLING_NO_HANDLING"
#define STRING_FOR_HOLE_HANDLING_ALL_ENTRANCE_POINTS "HOLE_HANDLING_ALL_ENTRANCE_POINTS"
#define STRING_FOR_HOLE_HANDLING_ALL_PROTECTION_POINTS "HOLE_HANDLING_ALL_PROTECTION_POINTS"

/** algorithm type for hole handling mode */
typedef enum _hole_handling_mode_t
{ HOLE_MODE_UNKNOWN = 0,
  HOLE_MODE_RESHUFFLE_LABELING = 1,
  HOLE_MODE_INCREMENTAL_LABELING = 2
} hole_handling_mode_t;

#define NUMBER_OF_HOLE_MODE_TYPE 3

/* sensing hole handling algorithm name */
#define STRING_FOR_HOLE_MODE_UNKNOWN "HOLE_MODE_UNKNOWN"
#define STRING_FOR_HOLE_MODE_RESHUFFLE_LABELING "HOLE_MODE_RESHUFFLE_LABELING"
#define STRING_FOR_HOLE_MODE_INCREMENTAL_LABELING "HOLE_MODE_INCREMENTAL_LABELING"

/** performance metric type */
typedef enum _performance_metric_type_t
{ METRIC_NETWORK_LIFETIME = 1,
  METRIC_AVERAGE_DETECTION_TIME = 2
} performance_metric_type_t;

/** data forwarding mode: -F option */
typedef enum _data_forwarding_mode_t
{ DATA_FORWARDING_MODE_UNKNOWN = 0,
  DATA_FORWARDING_MODE_DOWNLOAD = 1, //multhop data forwarding from AP to vehicle
  DATA_FORWARDING_MODE_UPLOAD = 2, //multihop data forwarding from vehicle to AP
  DATA_FORWARDING_MODE_V2V = 3 //multihop data forwarding from vehicle to vehicle
} data_forwarding_mode_t;

#define NUMBER_OF_DATA_FORWARDING_MODE 4

/* data forwarding mode name */
#define STRING_FOR_DATA_FORWARDING_MODE_UNKNOWN "UNKNOWN"
#define STRING_FOR_DATA_FORWARDING_MODE_DOWNLOAD "DOWNLOAD"
#define STRING_FOR_DATA_FORWARDING_MODE_UPLOAD "UPLOAD"
#define STRING_FOR_DATA_FORWARDING_MODE_V2V "V2V"

/** data forwarding link selection */
typedef enum _data_forwarding_link_selection_t
{ FORWARDING_LINK_SELECTION_UNKNOWN = 0,
  FORWARDING_LINK_SELECTION_ANGLE = 1, //selection based on the angle between the outgoing edge's vector and the direction vector towards the destination 
  FORWARDING_LINK_SELECTION_DISTANCE = 2, //selection based on the geographic distance between the current point to the destination point
  FORWARDING_LINK_SELECTION_DELAY = 3 //selection based on the aggregated link delay between the current point to the destination point
} data_forwarding_link_selection_t;

/* VANET Information Table */
typedef struct _vanet_information_table_t
{
    /** road network graph */
    struct_graph_node *Gr; //pointer to road network graph Gr
    int Gr_size; //size of graph Gr in terms of the number of vertices

	/** the set of road network graphs for the data forwarding schemes, such as VADD and TBD */
	

    /** the movement shortest path matrices */
    double **Dr_move; //weight matrix for all-pairs shortest paths in terms of vehicle movement in real graph Gr
    int **Mr_move; //predecessor matrix for all-pairs shortest paths in terms of vehicle movement in real graph Gr
    int matrix_size_for_movement_in_Gr; //matrix size of matrices Dr_move and Mr_move for movement in Gr
    
    /** the EDD shortest path matrices */
	int **Ar_edd; //connectivity adjacency matrix for EDD whose entry value of 1 indicates that the edge has two incident intersections with stationary nodes
    double **Dr_edd; //weight matrix for all-pairs shortest paths in terms of E2E delivery delay or delay variance in real graph Gr
    int **Mr_edd; //predecessor matrix for all-pairs shortest paths in terms of E2E delivery delay or delay variance in real graph Gr
    double **Sr_edd; //supplementary matrix for all-pairs shortest paths in terms of E2E delivery delay or delay variance in real graph Gr
    int matrix_size_for_edd_in_Gr; //matrix size of matrices Dr_edd, Mr_edd, and Sr_edd for E2E delivery delay or delay variance in Gr    

    /** the EDC shortest path matrices */
	double **Wr_edc; //the adjacency matrix of road network graph based on delivery cost mean
    double **Dr_edc; //weight matrix for all-pairs shortest paths in terms of E2E delivery cost or cost variance in real graph Gr
    int **Mr_edc; //predecessor matrix for all-pairs shortest paths in terms of E2E delivery cost or cost variance in real graph Gr
    double **Sr_edc; //supplementary matrix for all-pairs shortest paths in terms of E2E delivery cost or cost variance in real graph Gr
    int matrix_size_for_edc_in_Gr; //matrix size of matrices Dr_edc, Mr_edc, and Sr_edc for E2E delivery cost or cost variance in Gr    

    /** forwarding table queue */
    struct _forwarding_table_queue_t *FTQ; //pointer to the forwarding table queue

    /** pointer to the destination vehicle */
    struct struct_vehicle *dst_vnode; //pointer to the destination vehicle under Download mode
} vanet_information_table_t;

/***************************/

/** Definition of struct parameter */
typedef struct parameter
//typedef struct _parameter_t
{
/* VANET Forwarding Scheme */
		vanet_forwarding_scheme_t vanet_forwarding_scheme; //vanet forwarding scheme

/* VANET Information Table */
        vanet_information_table_t vanet_table; //this table contains the pointers to data structures used in the data forwarding in VANET, such as road network graph Gr, the EDD shortest path matrix Dr_edd, etc.

/* Simulation Logging Options */
        boolean forwarding_probability_and_statistics_flag; //flag to indicate whether the forwarding probability_and_statistics information is logged every EDD update or not


/* Packet Delay Measurement for Fixed Target Point */
		boolean packet_delay_measurement_flag; //flag to indicate whether the End-to-End Packet Delay Measurement should be performed or not

		int packet_delay_measurement_target_point; //target point for the End-to-End Packet Delay Measurement

		double packet_delay_measurement_time; //time for the End-to-End Packet Delay Measurement

/* Infrastructure Node Deployment using Bernoulli process */
		boolean AP_deployment_flag; //AP deployment flag = {0: APs are deployed by graph configuration, 1: APs are deployed by Bernoulli process}
		double AP_deployment_probability; //AP deployment probability p

		boolean SN_deployment_flag; //SN deployment flag = {0: SNs are deployed into all of the intersections, 1: SNs are deployed by Bernoulli process}
		double SN_deployment_probability; //SN deployment probability p

/* Performance Evaluation Type */
        evaluation_type_t evaluation_type;

/* Performance Comparison Target Type */
        comparison_target_type_t comparison_target_type;

/* Length Unit */
        length_unit_type_t length_unit; //length unit type

/* Graph Parameters */
        char graph_file_name[BUF_SIZE]; //graph file name
        int graph_node_number;       //number of nodes in the graph corresponding to the road network

/* Mobility Parameters */
        char mobility_file_name[BUF_SIZE]; //mobility file name

/* Data Forwarding Parameters  */
        data_forwarding_mode_t data_forwarding_mode; //data forwarding mode = {DATA_FORWARDING_MODE_DOWNLOAD, DATA_FORWARDING_MODE_UPLOAD}

        data_forwarding_link_selection_t data_forwarding_link_selection; //data forwarding link selection = {FORWARDING_LINK_SELECTION_ANGLE, FORWARDING_LINK_SELECTION_DISTANCE, FORWARDING_LINK_SELECTION_DELAY}

		boolean data_forwarding_two_way_forwarding_flag; //flag to indicate whether the data forwarding supports two-way forwarding or not 

		boolean data_forwarding_multiple_target_point_flag; //flag to determine whether to perform the data forwarding with multiple target points in order to satisfy the user-required delivery probability

		int data_forwarding_maximum_target_point_number; //maximum number of target points allowed for multi-target-point data forwarding
        
/* Target Point Parameters */
        double target_point_interdistance; //the interdistance between two consecutive target points on the target vehicle's trajectory; unit is meter.
        int target_point_index; //the index to enumerate the target points from the beginning of the vehicle trajectory to the end of the vehicle trajectory

/* Communication Parameters */
        double communication_range; //communication range between the vehicles or the vehicles and access points
        double communication_one_hop_delay; //average one-hop packet transmission delay for the radio communication range: unit is second.
        int communication_data_packet_size; //data packet size: unit is byte
        double communication_packet_ttl; //packet TTL (i.e., lifetime): unit is second
        double communication_packet_ttl_override_flag; //packet TTL override flag to indicate whether the TTL can be replaced with the vehicle trajectory travel time

		int communication_packet_hop_limit; //packet hop limit to limit the number of packet copies that is usually used by Epidemic Routing for controlled broadcast

        double communication_packet_interarrival_time; //packet interarrival time: unit is second
        distribution_type_t communication_packet_interarrival_time_distribution; //packet interarrival time distribution
        double communication_packet_interarrival_time_standard_deviation; //standard deviation of packet interarrival time
        double communication_packet_maximum_interarrival_time_factor; //factor for packet maximum interarrival time
        double communication_packet_maximum_interarrival_time; //packet maximum interarrival time
        int communication_packet_maximum_number; //maximum number of generated packets
        double communication_packet_delivery_probability_threshold; //the threshold of the packet delivery probability that the packet will arrive at the target point (i.e., target intersection) earlier than the destination vehicle
        int communication_packet_reverse_traversal_hop_distance_threshold; //the threshold of the hop distance from the packet and the destination vehicle on the destination vehicle's trajectory in order to determine the packet reverse traversal

        boolean communication_multiple_AP_flag; //flag to indicate that multiple APs are deployed in the road network
        int communication_AP_maximum_number; //maximum number of APs deployed into road network 
        double communication_AP_packet_generation_schedule_time; //AP's packet generation start time

        boolean communication_multiple_SN_flag; //flag to indicate that multiple Stationary Nodes (SNs) are deployed in the road network
        int communication_SN_maximum_number; //maximum number of SNs deployed into road network 
      
/* Sensor Parameters */
        sensor_schedule_mode_type_t sensor_schedule_mode; //sensor schedule mode in simulation in terms of energy budget update
        int sensor_number;       //number of sensors deployed at the sensor network
        double sensor_density;   //number of sensors per unit distance (e.g., 1m)
        distribution_type_t sensor_density_distribution; //sensor density distribution
        double sensor_density_standard_deviation; //standard deviation of sensor density
        double sensor_density_maximum_deviation; //maximum deviation of sensor density
        distribution_type_t sensor_deployment_distribtuion; //sensor deployment distribution
        double sensor_deployment_standard_deviation; //sensor deployment standard deviation
        double sensor_work_time; /* the type of time should be double to work well in smpl simulation */
        double sensor_think_time;
        distribution_type_t sensor_think_time_distribution;
        double sensor_think_time_standard_deviation;
        double sensor_energy;
        distribution_type_t sensor_energy_distribution;
        double sensor_energy_standard_deviation;
        double sensor_energy_maximum_deviation;
        double sensor_energy_consumption_rate; //amount of energy consumption per unit time        
        double sensor_warm_up_time; /* warm-up time for sensing devices */
        double sensor_turn_on_energy_consumption; /* turn_on_energy_consumption */
        double sensor_sensing_range; //sensing range: unit is meter
        distribution_type_t sensor_sensing_range_distribution; 
        double sensor_sensing_range_standard_deviation;
        double sensor_time_sync_max_error; //time synchronization maximum error
        double sensor_time_sync_error; //time synchronization error
        double sensor_time_sync_error_start; //start-value of time synchronization error
        double sensor_time_sync_error_end; //end-value of time synchronization error
        double sensor_time_sync_error_step; //step-value of time synchronization error
        distribution_type_t sensor_time_sync_error_distribution;
        double sensor_time_sync_error_standard_deviation;
        double sensor_time_sync_error_standard_deviation_start;//start-value of time synchronization error standard deviation (STD)
        double sensor_time_sync_error_standard_deviation_end;  //end-value of time synchronization error STD
        double sensor_time_sync_error_standard_deviation_step; //step-value of time synchronization error STD
        double sensor_detection_missing_probability; //sensor detection missing probability
        double sensor_detection_missing_probability_start; //start-value of sensor detection missing probability
        double sensor_detection_missing_probability_end; //end-value of sensor detection missing probability
        double sensor_detection_missing_probability_step; //step-value of sensor detection missing probability
        double sensor_duplicate_detection_probability; //sensor duplicate detection probability
        double sensor_duplicate_detection_probability_start; //start-value of sensor duplicate detection probability
        double sensor_duplicate_detection_probability_end; //end-value of sensor duplicate detection probability
        double sensor_duplicate_detection_probability_step; //step-value of sensor duplicate detection probability
        sensor_scan_type_t sensor_scan_type; //sensor scan type
        hole_handling_algorithm_t sensor_initial_hole_handling_algorithm; //handling algorithm for initial sensing holes
        hole_handling_algorithm_t sensor_hole_handling_algorithm; //hole handling algorithm for sensing holes due to energy depletion
        hole_handling_mode_t sensor_hole_handling_mode; //hole handling mode of incremental labeling or reshuffling labeling

        double sensor_movement_time_percentage; //percentage of movement time used for the sleeping time: e.g., sensor_movement_time_percentage=50 means that the 50% of the shortest movement time is used for the overall sleeping time along with the shortest scanning time

/* Network Parameters */
        double network_width;
        double network_height;
        double network_gap; //distance between two sensors
        distribution_type_t network_sensor_distribution;
        double network_sensor_standard_deviation;

/* Vehicle Parameters */
        boolean vehicle_vanet_target_vehicle_flag; //flag to indicate whether a target vehicle will be used or not

        boolean vehicle_vanet_stationary_vehicle_flag; //flag to indicate whether a stationary vehicle will be used or not
        int vehicle_vanet_stationary_vehicle_id; //stationary vehicle id

        boolean vehicle_vanet_acl_measurement_flag; //flag to indicate whether the ACL measurement per each directional edge will be performed or not
	vanet_vehicular_traffic_model_type_t vehicle_vanet_vehicular_traffic_model; //vehicular traffic model, such as closed network and open network
        vanet_edd_and_link_model_type_t vehicle_vanet_edd_and_link_model; //vehicle's EDD and Link model
        vanet_edd_model_type_t vehicle_vanet_edd_model; //vehicle's EDD model
        vanet_edd_computation_model_type_t vehicle_vanet_edd_computation_model; //vehicle's EDD Computation model: (a) Stochastic model and (b) Shortest Path model
        vanet_tbd_edd_computation_type_t vehicle_vanet_tbd_edd_computation_type; //vehicle's EDD computation type in TBD
        vanet_edge_delay_model_type_t vehicle_vanet_edge_delay_model; //vehicle's Edge Delay model
        vanet_forwarding_type_t vehicle_vanet_forwarding_type; //vehicle's Forwarding type
        vanet_intersection_forwarding_type_t vehicle_vanet_intersection_forwarding_type; //vehicle's Intersection Forwarding type

        vanet_target_point_selection_type_t vehicle_vanet_target_point_selection_type; //target point selection type for destination vehicle in reverse forwarding from AP to destination vehicle

        vanet_target_point_computation_method_t vehicle_vanet_target_point_computation_method; //target point computation method to select a target point for destination vehicle

        vanet_target_point_search_space_type_t vehicle_vanet_target_point_search_space_type; //target point search space type for the destination vehicle's trajectory

        double vehicle_vanet_target_point_recomputation_interval_denominator; //denominator to compute the recomputation interval of a new target point 

        vanet_vehicle_trajectory_type_t vehicle_vanet_vehicle_trajectory_type; //vehicle trajectory type for destination vehicle trajectory
        double vehicle_vanet_vehicle_trajectory_exposure_degree; //degree of vehicle trajectory exposure, such as 1 (=100%) and 0.5 (=50%)

        vanet_vehicle_trajectory_length_type_t vehicle_vanet_vehicle_trajectory_length_type; //vehicle trajectory length type for destination vehicle trajectory

        vanet_metric_type_t vehicle_vanet_metric_type; //vanet metric type = {EDD, EDD_VAR}

        vanet_target_point_optimization_function_type_t vehicle_vanet_target_point_optimization_function_type; //function type to compute a target point optimization value

        int vehicle_maximum_number; //maximum number of vehicles injected into road network
        int vehicle_packet_generating_entity_number; //number of packet generating vehicles
        double vehicle_AP_passing_entity_percentage; //percentage of vehicles passing AP(s); in random-way-point model, the destination is one of APs alternately such as the first destination is an intersection with AP (called AP-intersection) and the next destination is an intersection without AP (non-AP-intersection).
        int vehicle_AP_passing_entity_number; //number of vehicle passing AP(s)
   
        double vehicle_packet_generation_schedule_time; //schedule time when vehicles generate packets
        double vehicle_edd_update_period; //period when the Expected Delivery Delay (EDD) is recalculated based on accumulated vehicular traffic statistics

        vehicle_step_mode_type_t vehicle_step_mode; //step mode for vehicle movement
	double vehicle_step_time; //step time for vehicle's movement
	double vehicle_initial_arrival_time; //the arrival time of the first vehicle
	double vehicle_interarrival_time;
	double vehicle_interarrival_time_start;
	double vehicle_interarrival_time_end;
	double vehicle_interarrival_time_step;
	distribution_type_t vehicle_interarrival_time_distribution;
	double vehicle_interarrival_time_standard_deviation;
	double vehicle_maximum_interarrival_time_factor; //factor to determine vehicle's maximum interarrival time 
	double vehicle_maximum_interarrival_time; //vehicle's maximum interarrival time

	double vehicle_minimum_speed; //vehicle minimum speed: unit is meter/sec
	double vehicle_minimum_speed_in_km_per_hour; //vehicle minimum speed: unit is km/h
	double vehicle_minimum_speed_in_mile_per_hour; //vehicle minimum speed: unit is mph

	double vehicle_maximum_speed; //vehicle maximum speed: unit is meter/sec
	double vehicle_maximum_speed_in_km_per_hour; //vehicle maximum speed: unit is km/h
	double vehicle_maximum_speed_in_mile_per_hour; //vehicle maximum speed: unit is mph

	double vehicle_speed; //vehicle speed: unit is meter/sec
	double vehicle_speed_in_km_per_hour; //unit is km/h
	double vehicle_speed_in_mile_per_hour; //unit is mph

	double vehicle_speed_start; //start-value of vehicle speed
	double vehicle_speed_end; //end-value of vehicle speed
	double vehicle_speed_step; //step-value of vehicle speed
	distribution_type_t vehicle_speed_distribution;

	double vehicle_speed_standard_deviation; //unit is meter/sec
	double vehicle_speed_variance; //the vehicle speed variance that is the square of the vehicle speed standard deviation
	double vehicle_speed_standard_deviation_in_km_per_hour; //unit is km/h
	double vehicle_speed_standard_deviation_in_mile_per_hour; //unit is mph

	double vehicle_speed_bound_coefficient; //coefficient to determine the minimum and maximum speeds, as the multiple of vehicle speed standard deviation
	boolean vehicle_speed_bound_coefficient_flag; //flag to indicate whether vehicle_speed_bound_coefficient should be used to determine the vehicle maximum and minimum speed limits for the vehicle speed generation

	double vehicle_think_time; //waiting time at intersection node
	double vehicle_mean_think_time; //mean waiting time at intersection node due to either stop sign or traffic signal
	double vehicle_think_time_variance; //the variance of waiting time at intersection node due to either stop sign or traffic signal

	double vehicle_think_time_start; //start-value of waiting time at intersection node
	double vehicle_think_time_end; //end-value of waiting time at intersection node
	double vehicle_think_time_step; //step-value of waiting time at intersection node
	distribution_type_t vehicle_think_time_distribution;
	double vehicle_think_time_standard_deviation;

	distribution_type_t vehicle_path_length_distribution; // distribution of path length

	double vehicle_path_length_standard_deviation; //standard deviation of path length
	double vehicle_path_length_standard_deviation_in_km; //standard deviation of path length: unit is kilometer
	double vehicle_path_length_standard_deviation_in_mile; //standard deviation of path length: unit is mile

	double vehicle_path_length_standard_deviation_start; //start-value of standard deviation of path length
	double vehicle_path_length_standard_deviation_end; //end-value of standard deviation of path length
	double vehicle_path_length_standard_deviation_step; //step-value of standard deviation of path length

        int vehicle_path_minimum_hop_count; //minimum hop count in vehicle path, used to allow the vehicle trajectory have at least minimum hop count
    
    /** variables for vehicle travel time statistics */
        double vehicle_unit_length; //unit length for road segment: 1 meter
        double vehicle_unit_length_mean_travel_time; //travel time for the unit length
        double vehicle_unit_length_travel_time_variance; //travel time variance for the unit length
        double vehicle_unit_length_travel_time_standard_deviation; //travel time standard deviation for the unit length        

/* Data Processing Parameters */
	aggregation_type_t data_aggregation_type; //data aggregation type
	int data_aggregation_window_size; //data aggregation window
	int data_aggregation_window_size_start; //start-value of data aggregation window
	int data_aggregation_window_size_end; //end-value of data aggregation window
	int data_aggregation_window_size_step; //stop-value of data aggregation window

	double data_measurement_time; //measurement time for vehicle detections
	int data_number_of_split_measurement_times; //number of split measurement times

	prefilter_type_t data_prefilter_type; //data prefilter type

/* Simulation Time */
	simulation_seed_type_t simulation_seed; //simulation_seed = default or time()
	simulation_mode_type_t simulation_mode; // simulation_mode = time or iteration
	double simulation_time;
	int simulation_iteration_number;
	simulation_run_type_t simulation_run; //simulation running type


/* TPD Parameters */
	double tpd_encounter_probability_threshold; //encounter probability threshold that considers two vehicles to encounter in an edge in a road network graph

	double tpd_delivery_probability_threshold; // E2E delivery probability threshold that lets RSU forward its packets to a next-carrier vehicle within the communication range

	boolean tpd_encounter_graph_optimization_flag; //flag to determine whether to perform Dynamic Programming (DP) for the optimization of the predicted encounter graph for a better EDR

	boolean tpd_encounter_graph_source_routing_flag; //flag to determine whether to perform source routing with the predicted encounter graph
} parameter_t;

/** Declaration of Function */
void init_parameter(parameter_t *param, char* conf_file); 
// initialize simulation parameter, param with conf_file

void update_vehicle_maximum_and_minimum_speeds(parameter_t *param);
//update the vehicle maximum and minimum speed bounds for the speed generation

void create_vanet_information_table_in_parameter(parameter_t *param, struct_graph_node *Gr, int Gr_size, double **Dr_move, int **Mr_move, int matrix_size_for_movement_in_Gr, int **Ar_edd, double **Dr_edd, int **Mr_edd, double **Sr_edd, int matrix_size_for_edd_in_Gr, double **Wr_edc, double **Dr_edc, int **Mr_edc, double **Sr_edc, int matrix_size_for_edc_in_Gr, struct _forwarding_table_queue_t *FTQ);
//create vanet information table containing the data structures used in the data forwarding in the VANET, such as road network graph Gr, the movement shortest path matrices, the EDD shortest path matrices, the EDC shortest path matrices, the forwarding table queue FTQ, etc.
  /*@ Note: Whenever a new data structure is added to vanet information table, the memory for the data structure must be destroyed in destroy_vanet_information_table_in_parameter() */

void destroy_vanet_information_table_in_parameter(parameter_t *param);
//destroy the data structures in vanet information table in param, such as road network graph Gr, the movement shortest path matrices, the EDD shortest path matricesm, the EDC shortest path matrices, the forwarding table queue FTQ, etc.; Note that these data structures can be reallocated memory, so the original pointers may not point to the actual memory.

#endif

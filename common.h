/** File: common.h
	Description: specify the common macro constants, structures, and enum types.
	Date: 07/25/2006
	Maker: Jaehoon Jeong, jjeong@cs.umn.edu
*/

#ifndef __COMMON_H__
#define __COMMON_H__

#include "smpl.h"
#include "param.h" //parameter structure and related constants
#include "rand.h" //type real and random functions
#include <stdlib.h> //atoi(), atof()
#include <stdarg.h> //variable argument list
#include "graph-data-struct.h" //struct_path_node
#include <malloc.h> //_msize()

/* Flag to determine whether to show the trace of vehicle mobility in the road network graph */
#define TPD_VEHICLE_MOBILITY_TRACE_FLAG 0

/* Flag to determine whether to use the actual vehicle speed or the average vehicle speed */
#define TPD_ACTUAL_VEHICLE_SPEED_USE_FLAG 1

/* Flag to determine whether to update vehicle speed per road segment during the travel along its trajectory */
#define TPD_VEHICLE_SPEED_UPDATE_PER_ROAD_SEGMENT_FLAG 0 

/* Flag to determine whether to display the carrier trace during the packet forwarding toward the destination vehicle */
#define TPD_PACKET_CARRIER_VEHICLE_TRACE_FLAG 0

/* Flag to determine whether to display the forwarding trace at an intersection for greedy routing */
#define TPD_GREEDY_ROUTING_INTERSECTION_FORWARDING_TRACE_FLAG 1

/* Flag to determine whether to display the forwarding trace in a road segment for greedy routing */
#define TPD_GREEDY_ROUTING_ROAD_SEGMENT_FORWARDING_TRACE_FLAG 0

/* Flag to determine whether to display the forwarding trace at an intersection for source routing */
#define TPD_SOURCE_ROUTING_INTERSECTION_FORWARDING_TRACE_FLAG 1

/* Flag to determine whether to display the forwarding trace to the destination vehicle for source routing */
#define TPD_SOURCE_ROUTING_DESTINATION_VEHICLE_FORWARDING_TRACE_FLAG 0

/* Flag to determine whether to display the forwarding trace in a road segment for source routing */
#define TPD_SOURCE_ROUTING_ROAD_SEGMENT_FORWARDING_TRACE_FLAG 1

/* Flag to determine whether to display the predicted encounter graph */
#define TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FLAG 1

/* Flag to determine whether to display the predicted encounter graph for a packet */
#define TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FOR_PACKET_FLAG 1

/* Flag to determine whether to display the predicted encounter graph for an AP */
#define TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FOR_AP_FLAG 1

/* Flag to determine whether to display the predicted encounter graph for an intersection */
#define TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FOR_INTERSECTION_FLAG 0

/* Flag to determine whether to display the predicted encounter graph for an intersection road segment that is incident to an intersection */
#define TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FOR_INTERSECTION_ROAD_SEGMENT_FLAG 0

/* Flag to determine whether to display the predicted encounter graph for a one-way road segment */
#define TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FOR_ONEWAY_ROAD_SEGMENT_FLAG 0

/* Flag to determine whether to display the predicted encounter graph for a two-way road segment */
#define TPD_DISPLAY_PREDICTED_ENCOUNTER_GRAPH_FOR_TWOWAY_ROAD_SEGMENT_FLAG 0

/* Flag to determine whether to show the trace messages related to the computation of EDR and EDD in TPD */
#define TPD_EDR_EDD_COMPUTATION_TRACE_FLAG 0

/* Flag to determine whether to show the trace messages related to encounter events in TPD */
#define TPD_ENCOUNTER_TRACE_FLAG 0

/* Flag to determine whether to print the trajectory and arrival time of all the vehicles in vehicle_list */
#define TPD_VEHICLE_TRAJECTORY_PRINT_FLAG 0

/* Flag to determine whether to store the trajectory and arrival time of all the vehicles in vehicle_list */
#define TPD_VEHICLE_TRAJECTORY_STORE_FLAG 0

/* File name for vehicle trajectory information */
#define TPD_TRAJECTORY_FILE_NAME "vehicle-trajectory.txt"

/* Flag to determine whether to print the error messages related to GSL functions */
#define GSL_ERROR_DISPLAY_FLAG 0

//@Library support
#define __GSL_LIBRARY_SUPPORT__
//@GSL library support for GNU scientific library

////////////////////////////////////////////////////
//@ Determine whether to perform breach path checking for network lifetime
//#define __CHECK_BREACH_PATH__
//#define __DEBUG_FOR_BREACH_PATH_IN_DETAIL__

//@ Determine whether to include the DEBUG messages
//#define __DEBUG__

#define __DEBUG_INTERACTIVE_MODE__

//@this macro constant is used to investigate the packet delay distribution
//#define __DEBUG_PACKET_DELAY_DISTRIBUTION__

//#define __DEBUG_SIMULATION_TIME__
//#define __DEBUG_SENSOR_TRACE__
//#define __DEBUG_LEVEL_SENSOR_BORN__
//#define __DEBUG_LEVEL_SENSOR_ESTIMATE__
//#define __DEBUG_LEVEL_SENSOR_SENSE__

//#define __DEBUG_LEVEL_SENSOR_DIE__

//#define __DEBUG_LEVEL_VEHICLE_ARRIVE__
//#define __DEBUG_LEVEL_VEHICLE_RESTART__
//#define __DEBUG_LEVEL_VEHICLE_DETECTED__

//#define __DEBUG_LEVEL_VEHICLE_ESCAPE__

//#define __DEBUG_LEVEL_SLEEPING_TIME__
//#define __DEBUG_LEVEL_0__
//#define __DEBUG_LEVEL_1__
//#define __DEBUG_LEVEL_2__
//#define __DEBUG_TRACE__
//#define __DEBUG_LEVEL_SORT_TRAFFIC_TABLE__

/********************************************************************/
/** VANET DEBUG */
//#define __DEBUG_LEVEL_VANET_PACKET_AP_ARRIVAL__

//#define __DEBUG_LEVEL_VANET_PACKET_DESTINATION_VEHICLE_ARRIVAL__
//@packet arrived at the destination, such as detination vehicle

//#define __DEBUG_LEVEL_VANET_PACKET_DROP__


#define __DEBUG_LEVEL_VANET_EDD_UPDATE__

//#define __DEBUG_LEVEL_EDD_COMPUTATION__

//#define  __DEBUG_LEVEL_AP_PACKET_ARRIVE__
//@debugging for AP packet arrive event

//#define __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_AP__
//@debugging for target point search by AP

//#define __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_CARRIER__
//@debugging for target point search by carrier

//#define __DEBUG_LEVEL_TARGET_POINT_SEARCH_BY_STATIONARY_NODE__
//@debugging for target point search by AP

//#define  __DEBUG_LEVEL_CARRIER_SEARCH_BY_AP__
//@debugging for next carrier search by AP

//#define  __DEBUG_LEVEL_CARRIER_SEARCH_BY_STATIONARY_NODE__
//@debugging for next carrier search by stationary node

//#define __DEBUG_LEVEL_TRAJECTORY_EXPIRATION__
//@debugging for vehicle trajectory expiration

//#define __DEBUG_LEVEL_TARGET_POINT_SELECTION_TRACE__
//@target point selection trace, such as the selection of the last trajectory point as target point

/** DEBUG for Convoy Operations */
//#define __DEBUG_CONVOY_UPDATE__
//#define __DEBUG_CONVOY_JOIN__
//#define __DEBUG_CONVOY_FAST_JOIN__
//#define __DEBUG_CONVOY_LEAVE__
//#define __DEBUG_CONVOY_MERGE__
//#define __DEBUG_CONVOY_SPLIT__


/** DEBUG for Packet Trace in TBD: show packet delivery trace */
//#define __DEBUG_PACKET_TRACE__

/** DEBUG for Packet Trajectory in STBD: show the installation of packet delivery trajectory */
//#define __DEBUG_LEVEL_PACKET_TRAJECTORY_INSTALLATION__



/** DEBUG for Delivery Delay's Standard Deviation */
//#define __DEBUG_DELIVERY_DELAY_STANDARD_DEVIATION__

/** DEBUG for Linear Algebra Operations */
//#define  __DEBUG_LEVEL_GAUSSIAN_ELIMINATION__

/** function debugging */
//#define __DEBUG_LEVEL_SET_NEW_CONVOY_LEADER__
//#define __DEBUG_LEVEL_VADD_COMPUTE_FORWARDING_PROBABILITY__
//#define __DEBUG_LEVEL_VADD_RECOMPUTE_FORWARDING_PROBABILITY__

//@ Debugging for GSL functions
//#define __DEBUG_LEVEL_GSL_FUNCTION__
//#define __DEBUG_LEVEL_GSL_FUNCTION_PROBABILITY__

//@ Debugging for packet trace
#define __DEBUG_LEVEL_VANET_PACKET_CARRIER_TRACE__

//@ Debugging for packet trace in respect of stationary nodes
#define __DEBUG_LEVEL_VANET_PACKET_CARRIER_TRACE_FOR_STATIONARY_NODE__

//@ Debugging for Combinations for Target Points
//#define __DEBUG_COMBINATIONS_FOR_TARGET_POINTS__

//@ [7/24/2010] Debugging for displaying Target Points in Multi-target-point Data Forwarding for TMC Journal Version
//#define __DEBUG_LEVEL_PRINT_TARGET_POINT__

//@ Determine whether to trace the length of a convoy of vehicles on road segment
//#define __LOG_LEVEL_TRACE_OF_VEHICLE_CONVOY_LENGTH__

//@ Determine to what level the events are logged into surveillance log file
//#define __LOG_LEVEL_SENSOR__
//#define __LOG_LEVEL_SENSOR_BORN__
//#define __LOG_LEVEL_SENSOR_RESCHEDULE__
//#define __LOG_LEVEL_SENSOR_ESTIMATE__
//#define __LOG_LEVEL_SENSOR_SENSE__
//#define __LOG_LEVEL_SENSOR_SLEEP__
//#define __LOG_LEVEL_SENSOR_DIE__

//#define __LOG_LEVEL_VEHICLE__
//#define __LOG_LEVEL_VEHICLE_ARRIVE__
//#define __LOG_LEVEL_VEHICLE_CHECK__
//#define __LOG_LEVEL_VEHICLE_MOVE__
//#define __LOG_LEVEL_VEHICLE_DETECTED__

//#define __LOG_LEVEL_VEHICLE_ESCAPE__

//@ Derermine whether the status of sensors on the breach path taken by vehicle is shown or not
//#define __DEBUG_LEVEL_SHOW_STATUS_OF_SENSORS_ON_PATH__
#define __LOG_LEVEL_SHOW_STATUS_OF_SENSORS_ON_PATH__

//@ Determine whether vehicle path is logged or not
//#define __LOG_LEVEL_PATH_LIST__

//@ Determine whether to trace the number of living sensors over surveillance duration
#define __LOG_LEVEL_TRACE_OF_SENSOR_NUMBER__

//@ Determine whether to trace the number of sensing holes over surveillance duration
//#define __LOG_LEVEL_TRACE_OF_HOLE_NUMBER__

//@ Determine whether to trace the sleeping time over surveillance duration
#define __LOG_LEVEL_TRACE_OF_SLEEPING_TIME__

//@ Determine whether to trace the vehicle detection time over surveillance duration
#define __LOG_LEVEL_TRACE_OF_VEHICLE_DETECTION_TIME__

/** LOG for Linear Algebra Operations */
//#define  __LOG_LEVEL_GAUSSIAN_ELIMINATION__

/********************************************************************/
/** VANET LOG */
//@ Determine to what level the events are logged into VANET log file
//#define __LOG_LEVEL_VANET_PACKET_AP_ARRIVAL__

//#define __LOG_LEVEL_VANET_PACKET_DESTINATION_VEHICLE_ARRIVAL__
//@packet arrived at the destination, such as detination vehicle

//#define __LOG_LEVEL_VANET_PACKET_DROP__

//#define __LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__
//@log the packet carrier trace along with the packet receive time and position on the road network

//#define __LOG_LEVEL_VANET_PACKET_CARRIER_TRACE_FOR_STATIONARY_NODE__
//@log the packet carrier trace for the stationary node

//@ Determine whether the vanet road network graphs for vehicular traffic statistics are stored into files or not
//#define __LOG_FILE_FOR_VANET_ROAD_NETWORK_GRAPH__


////////////////////////////////////////////////////

//@ Determine whether the data structures related to scheduling or statistics, such as adjacency matrix and Floyd Warshall shortest path matrix are stored into files or not
//#ifdef  __STORE_FILE_FOR_DATA_STRUCT__


#define PI 3.14159265358979323846

//#define STEP_TIME 10
//#define STEP_TIME 1
//#define STEP_TIME 5
//#define STEP_TIME 0.1
//#define STEP_TIME 0.05
//step time for updating vehicle position

#define STEP_DISTANCE 1
//step distance for updating vehicle position: 1 meter
//@ we need to consider whether this step time is appropriate later

#define VEHICLE_WHEELBASE 2.8
//vehicle's wheelbase: unit is [m]
#define VEHICLE_MAX_TURNING_ANGLE 25
//vehicle's maximum turning angle: unit is [degree]
//#define AVERAGE_TRAJECTORY_DISTANCE 500
#define AVERAGE_TRAJECTORY_DISTANCE 1000
//average distance of vehicle's trajectory: unit is [m]

#define VEHICLE_ZERO_SPEED 0.01


/** length unit macro constants */
#define INCH 0.0254
//1 inch = 2.54cm = 0.0254

#define FEET (12*INCH)
//1 feet = 12 inch

#define MILE (5280*FEET)
//1 mile = 1609.344m = 5280feet

/********************************/

#ifdef _LINUX_

//parameter configuration file
#define CONF_FILE_FOR_DOWNLOAD "./param-configuration/param-for-download.conf"
#define CONF_FILE_FOR_UPLOAD "./param-configuration/param-for-upload.conf"
#define CONF_FILE_FOR_V2V "./param-configuration/param-for-v2v.conf"
//#define CONF_FILE CONF_FILE_FOR_DOWNLOAD
//#define CONF_FILE CONF_FILE_FOR_UPLOAD
#define CONF_FILE CONF_FILE_FOR_V2V

#define CONF_FILE_FOR_AVERAGE_CONVOY_LENGTH "./param-configuration/param-for-acl.conf"
//parameter configuration file for simulation for Average Convoy Length estimation

#define CONF_FILE_FOR_NETWORK_LIFETIME "./param-configuration/param-for-lifetime.conf"
//parameter configuration file for simulation for network lifetime

#define CONF_FILE_FOR_AVERAGE_DETECTION_TIME "./param-configuration/param-for-average-detection-time.conf"
//parameter configuration file for simulation for average detection time

/* graph configuration file */
#define GRAPH_FILE_FOR_NODES_2 "./graph-configuration/graph-node2-for-vanet.conf"
#define GRAPH_FILE_FOR_NODES_3 "./graph-configuration/graph-node3-for-vanet.conf"

//#define GRAPH_FILE_FOR_NODES_4 "./graph-configuration/graph-node4-for-vanet.conf"
//#define GRAPH_FILE_FOR_NODES_4 "./graph-configuration/graph-node4-for-vanet2.conf"
#define GRAPH_FILE_FOR_NODES_4 "./graph-configuration/graph-node4-for-vanet3.conf"
#define GRAPH_FILE_FOR_NODES_5 "./graph-configuration/graph-node5-for-vanet.conf"
#define GRAPH_FILE_FOR_NODES_9 "./graph-configuration/graph-node9-for-vanet.conf"

#define GRAPH_FILE_FOR_NODES_20 "./graph-configuration/graph-node20-for-vanet.conf"
#define GRAPH_FILE_FOR_NODES_20_WITH_MULTIPLE_AP_FOR_DOWNLOAD "./graph-configuration/graph-node20-for-vanet-with-multiple-APs-for-download.conf"
#define GRAPH_FILE_FOR_NODES_20_WITH_MULTIPLE_AP_FOR_UPLOAD "./graph-configuration/graph-node20-for-vanet-with-multiple-APs-for-upload.conf"

#define GRAPH_FILE_FOR_NODES_36 "./graph-configuration/graph-node36-for-vanet.conf"
#define GRAPH_FILE_FOR_NODES_36_WITH_MULTIPLE_AP_FOR_DOWNLOAD "./graph-configuration/graph-node36-for-vanet-with-multiple-APs-for-download.conf"
#define GRAPH_FILE_FOR_NODES_36_WITH_MULTIPLE_AP_FOR_UPLOAD "./graph-configuration/graph-node36-for-vanet-with-multiple-APs-for-upload.conf"

#define GRAPH_FILE_FOR_NODES_49 "./graph-configuration/graph-node49-for-vanet.conf"

#define GRAPH_FILE_FOR_NODES_49_WITH_MULTIPLE_AP_FOR_DOWNLOAD "./graph-configuration/graph-node49-for-vanet-with-multiple-APs-for-download.conf"
#if 0 /* [ For network configuration for the partial deployment of relay nodes */
#define GRAPH_FILE_FOR_NODES_49_WITH_MULTIPLE_AP_FOR_DOWNLOAD "./graph-configuration/graph-node49-for-vanet-with-multiple-APs-for-download-under-partial-stationary-node-deployment.conf"
#endif /* ] */

#define GRAPH_FILE_FOR_NODES_49_WITH_MULTIPLE_AP_FOR_UPLOAD "./graph-configuration/graph-node49-for-vanet-with-multiple-APs-for-upload.conf"

/* mobility configuration file */
#define MOBILITY_FILE_FOR_NODES_2 "./mobility-configuration/mobility-for-graph-node2-for-vanet.conf"
#define MOBILITY_FILE_FOR_NODES_3 "./mobility-configuration/mobility-for-graph-node3-for-vanet.conf"
#define MOBILITY_FILE_FOR_NODES_4 "./mobility-configuration/mobility-for-graph-node4-for-vanet-with-stationary-destination-vehicle.conf"
//#define MOBILITY_FILE_FOR_NODES_4 "./mobility-configuration/mobility-for-graph-node4-for-vanet.conf"
#define MOBILITY_FILE_FOR_NODES_5 "./mobility-configuration/mobility-for-graph-node5-for-vanet.conf"
#define MOBILITY_FILE_FOR_NODES_9 "./mobility-configuration/mobility-for-graph-node9-for-vanet-with-stationary-destination-vehicle.conf"
//#define MOBILITY_FILE_FOR_NODES_9 "./mobility-configuration/mobility-for-graph-node9-for-vanet.conf"

#define MOBILITY_FILE_FOR_NODES_20 "./mobility-configuration/mobility-for-graph-node20-for-vanet.conf"
#define MOBILITY_FILE_FOR_NODES_20_WITH_MULTIPLE_AP "./mobility-configuration/mobility-for-graph-node20-for-vanet.conf"

#define MOBILITY_FILE_FOR_NODES_36 "./mobility-configuration/mobility-for-graph-node36-for-vanet.conf"
#define MOBILITY_FILE_FOR_NODES_36_WITH_MULTIPLE_AP "./mobility-configuration/mobility-for-graph-node36-for-vanet.conf"
#define MOBILITY_FILE_FOR_NODES_49 "./mobility-configuration/mobility-for-graph-node49-for-vanet.conf"
#define MOBILITY_FILE_FOR_NODES_49_WITH_MULTIPLE_AP "./mobility-configuration/mobility-for-graph-node49-for-vanet.conf"

#define OUTPUT_FILE_1 "./output/output.txt"
//simulation output file for simulation results

#define OUTPUT_FILE_2 "./output/output.xls"
//simulation output file for simulation summary

#define SCHEDULE_FILE "./result/schedule.txt"
//sensing schedule file

#define LOCALIZATION_FILE "./result/localization.txt"
//simulation data for timestamps

#define SURVEILLANCE_FILE_SUFFIX ".log"
//surveillance log file

#define VANET_FILE_SUFFIX ".log"
//vanet log file

#define VANET_PACKET_CARRIER_TRACE_FILE_SUFFIX ".pct"
//vanet packet carrier trace log file

#define PATHLIST_FILE "./result/pathlist.txt"
//path-list log file

#define PATHTABLE_FILE_FOR_MOVE "./result/pathtable-for-move.txt"
//path-table file for all-source-all-destination pairs in terms of vehicle movement

#define PATHTABLE_FILE_FOR_SCAN "./result/pathtable-for-scan.txt"
//path-table file for all-source-all-destination pairs in terms of sensor scan

#define ADJACENCY_MATRIX_FILE "./result/adjacency-matrix.txt"
//adjacency matrix file representing the graph of the road network

#define VIRTUAL_TOPOLOGY_FILE_SUFFIX_1 "-Mv1.txt"
//file suffix for virtual topology based on non-aggregation method

#define VIRTUAL_TOPOLOGY_FILE_SUFFIX_2 "-Mv2.txt"
//file suffix for virtual topology based on aggregation method

#define INITIAL_SENSING_HOLE_FILE "./result/initial-sensing-hole.txt"
//initial sensing hole file

#define FINAL_SENSING_HOLE_FILE "./result/final-sensing-hole.txt"
//final sensing hole file

#define INITIAL_VIRTUAL_GRAPH_FILE "./result/initial-virtual-graph.txt"
//initial virtual graph file

#define FINAL_VIRTUAL_GRAPH_FILE "./result/final-virtual-graph.txt"
//final virtual graph file

#define SENSOR_LOCATION_FILE "./result/sensor-location.txt"
//sensor location file

#define TRACE_FILE_OF_VEHICLE_CONVOY_LENGTH "./result/trace-file-of-vehicle-convoy-length.xls"
//trace file of vehicle convoy length

#define TRACE_FILE_OF_SENSOR_NUMBER "./result/trace-file-of-sensor-number.xls"
//trace file of sensor number

#define TRACE_FILE_OF_HOLE_NUMBER "./result/trace-file-of-hole-number.xls"
//trace file of hole number

#define TRACE_FILE_OF_SLEEPING_TIME "./result/trace-file-of-sleeping-time.xls"
//trace file of sleeping time

#define TRACE_FILE_OF_VEHICLE_DETECTION_TIME "./result/trace-file-of-vehicle-detection-time.xls"
//trace file of vehicle detection time

#define VANET_TYPE_1_STATISTICS_ROAD_GRAPH_FILE "./result/vanet-type-1-statistics-road-graph.txt"
//road graph file with VANET type-1 statistics

#define VANET_TYPE_2_STATISTICS_ROAD_GRAPH_FILE "./result/vanet-type-2-statistics-road-graph.txt"
//road graph file with VANET type-2 statistics

#define VANET_TYPE_3_STATISTICS_ROAD_GRAPH_FILE "./result/vanet-type-3-statistics-road-graph.txt"
//road graph file with VANET type-3 statistics

#define VANET_TYPE_4_STATISTICS_ROAD_GRAPH_FILE "./result/vanet-type-4-statistics-road-graph.txt"
//road graph file with VANET type-4 statistics

#else
#define CONF_FILE "param-configuration\\param-for-win.conf"
//parameter configuration file

#define CONF_FILE_FOR_AVERAGE_CONVOY_LENGTH "param-configuration\\param-for-acl-for-win.conf"
//parameter configuration file for simulation for Average Convoy Length estimation

/* graph configuration file */
#define GRAPH_FILE_FOR_NODES_2 ".\\graph-configuration\\graph-node2-for-ivanet.conf"
#define GRAPH_FILE_FOR_NODES_3 ".\\graph-configuration\\graph-node3-for-ivanet.conf"
#define GRAPH_FILE_FOR_NODES_4 ".\\graph-configuration\\graph-node4-for-ivanet.conf"
#define GRAPH_FILE_FOR_NODES_9 ".\\graph-configuration\\graph-node9-for-ivanet.conf"
#define GRAPH_FILE_FOR_NODES_36 ".\\graph-configuration\\graph-node36-for-ivanet.conf"
#define GRAPH_FILE_FOR_NODES_36_WITH_MULTIPLE_AP ".\\graph-configuration\\graph-node36-for-ivanet-with-multiple-APs.conf"
#define GRAPH_FILE_FOR_NODES_49 ".\\graph-configuration\\graph-node49-for-vanet.conf"
#define GRAPH_FILE_FOR_NODES_49_WITH_MULTIPLE_AP ".\\graph-configuration\\graph-node49-for-vanet-with-multiple-APs.conf"

/* mobility configuration file */
#define MOBILITY_FILE_FOR_NODES_2 ".\\mobility-configuration\\mobility-for-graph-node2-for-vanet.conf"
#define MOBILITY_FILE_FOR_NODES_3 ".\\mobility-configuration\\mobility-for-graph-node3-for-vanet.conf"
#define MOBILITY_FILE_FOR_NODES_4 ".\\mobility-configuration\\mobility-for-graph-node4-for-vanet.conf"
#define MOBILITY_FILE_FOR_NODES_5 ".\\mobility-configuration\\mobility-for-graph-node5-for-vanet.conf"
#define MOBILITY_FILE_FOR_NODES_9 ".\\mobility-configuration\\mobility-for-graph-node9-for-vanet.conf"
#define MOBILITY_FILE_FOR_NODES_36 ".\\mobility-configuration\\mobility-for-graph-node36-for-vanet.conf"
#define MOBILITY_FILE_FOR_NODES_36_WITH_MULTIPLE_AP ".\\mobility-configuration\\mobility-for-graph-node36-for-vanet-with-multiple-APs.conf"
#define MOBILITY_FILE_FOR_NODES_49 ".\\mobility-configuration\\mobility-for-graph-node49-for-vanet.conf"
#define MOBILITY_FILE_FOR_NODES_49_WITH_MULTIPLE_AP ".\\mobility-configuration\\mobility-for-graph-node49-for-vanet.conf"

#define OUTPUT_FILE_1 "output\\output.txt"
//simulation output file for simulation results

#define OUTPUT_FILE_2 "output\\output.xls"
//simulation output file for simulation summary

#define SCHEDULE_FILE "result\\schedule.txt"
//sensing schedule file

#define LOCALIZATION_FILE "result\\localization.txt"
//simulation data for timestamps

#define SURVEILLANCE_FILE_SUFFIX ".log"
//surveillance log file

#define VANET_FILE_SUFFIX ".log"
//vanet log file

#define VANET_PACKET_CARRIER_TRACE_FILE_SUFFIX ".pct"
//vanet packet carrier trace log file

#define PATHLIST_FILE "result\\pathlist.txt"
//path-list log file

#define PATHTABLE_FILE_FOR_MOVE "result\\pathtable-for-move.txt"
//path-table file for all-source-all-destination pairs in terms of vehicle movement

#define PATHTABLE_FILE_FOR_SCAN "result\\pathtable-for-scan.txt"
//path-table file for all-source-all-destination pairs in terms of sensor scan

#define ADJACENCY_MATRIX_FILE "result\\adjacency-matrix.txt"
//adjacency matrix file representing the graph of the road network

#define VIRTUAL_TOPOLOGY_FILE_SUFFIX_1 "-Mv1.txt"
//file suffix for virtual topology based on non-aggregation method

#define VIRTUAL_TOPOLOGY_FILE_SUFFIX_2 "-Mv2.txt"
//file suffix for virtual topology based on aggregation method

#define INITIAL_SENSING_HOLE_FILE "result\\initial-sensing-hole.txt"
//initial sensing hole file

#define FINAL_SENSING_HOLE_FILE "result\\final-sensing-hole.txt"
//final sensing hole file

#define INITIAL_VIRTUAL_GRAPH_FILE "result\\initial-virtual-graph.txt"
//initial virtual graph file

#define FINAL_VIRTUAL_GRAPH_FILE "result\\final-virtual-graph.txt"
//final virtual graph file

#define SENSOR_LOCATION_FILE "result\\sensor-location.txt"
//sensor location file

#define TRACE_FILE_OF_VEHICLE_CONVOY_LENGTH "result\\trace-file-of-vehicle-convoy-length.xls"
//trace file of vehicle convoy length

#define TRACE_FILE_OF_SENSOR_NUMBER "result\\trace-file-of-sensor-number.xls"
//trace file of sensor number

#define TRACE_FILE_OF_HOLE_NUMBER "result\\trace-file-of-hole-number.xls"
//trace file of hole number

#define TRACE_FILE_OF_SLEEPING_TIME "result\\trace-file-of-sleeping-time.xls"
//trace file of sleeping time

#define TRACE_FILE_OF_VEHICLE_DETECTION_TIME "result\\trace-file-of-vehicle-detection-time.xls"
//trace file of vehicle detection time

#define VANET_TYPE_1_STATISTICS_ROAD_GRAPH_FILE "result\\vanet-type-1-statistics-road-graph.txt"
//road graph file with VANET type-1 statistics

#define VANET_TYPE_2_STATISTICS_ROAD_GRAPH_FILE "result\\vanet-type-2-statistics-road-graph.txt"
//road graph file with VANET type-2 statistics

#define VANET_TYPE_3_STATISTICS_ROAD_GRAPH_FILE "result\\vanet-type-3-statistics-road-graph.txt"
//road graph file with VANET type-3 statistics

#define VANET_TYPE_4_STATISTICS_ROAD_GRAPH_FILE "result\\vanet-type-4-statistics-road-graph.txt"
//road graph file with VANET type-4 statistics

#endif

/** VANET Constants */
/* threshold of the hop distance to enable the reverse traversal of the packet for the destination vehicle's trajectory */
#define REVERSE_TRAVERSAL_HOP_DISTANCE_THRESHOLD 3
//#define REVERSE_TRAVERSAL_HOP_DISTANCE_THRESHOLD 1

#define BUF_SIZE 255 //buffer size
#define MSG_BUF_SIZE 1000 //message buffer size
#define ENERGY_TOLERANCE 0.001
//energy tolerance for energy for sensing; that is, with this energy, we cannot run sensing device.

#define OFFSET_TOLERANCE 0.001
//offset tolerance for vehicle's offset for movement

#define ERROR_TOLERANCE_FOR_REAL_ARITHMETIC 0.0000001
//error tolerance for real number arithmetic for the equality comparison of two real numbers: 10^-7

#define NUMBER_STRING_LEN 20 //length of number string

//#define DUPLICATE_DETECTION_INTERVAL 0.01
#define DUPLICATE_DETECTION_INTERVAL 0.05
//duplicate detection interval for two successive timestamps due to duplicate detection

/* Constants */
//@ system parameter
#define Communication_range 75
//communication range: 75 [m]

//@ Time constraints
#define T_comm_tx 0.3
//RF transmitting time = 0.02 [sec]

#define T_comm_rx 0.2
//RF receiving time = 0.02 [sec]

#define T_comp 0.02
//computation time = 0.1 [sec]

#define T_warm 0.4
//warming-up time = 0.1 [sec]

#define T_sense 1.0
//minimum working time needed for sensing = 0.2 [sec]

//@ Power constraints
#define P_comp 16.5
//CPU energy consumption rate = 16.5 [mW]

#define P_comm_tx 21
//Radio transmit mode's energy consumption rate = 21 [mW]

#define P_comm_rx 15
//Radio receive mode's energy consumption rate = 15 [mW]

#define P_warm 15
//Sensor's warming-up energy consumption rate = 15 [mW]

#define P_work 10
//Sensor's working energy consumption rate = 10 [mW]

#define SCAN_SPEED 1.0
//scan speed 1 m/sec

#define MAXIMUM_NUMBER_OF_HOLE_ENDPOINTS_FOR_EXHAUSTIVE_SEARCH 20
//maximum number of initial sensing hole endpoints for exhaustive search

#define MATRIX_SIZE_MULTIPLIER 2
//multiplier used to expand the size of the matrix

/** macro constants for VANET */
//#define STATIONARY_VEHICLE_ID 10
//#define STATIONARY_VEHICLE_ID 1
//stationary vehicle's id

#define TARGET_VEHICLE_ID 2
//target vehicle's id

//#define TIME_MARGIN 1
//time margin used to order the events, such as the AP vehicular traffic statistics update after the vehicular traffic statistics for graph Gr

/** enum types */

typedef enum _SENSOR_TYPE{
	SENSOR_TYPE_UNKNOWN = 0,
	INTERSECTION_NODE = 1,   /* intersection node: sensor placed at intersection on road */
	NONINTERSECTION_NODE = 2 /* nonintersection node: sensor placed at non-intersection on road */
} SENSOR_TYPE;

typedef enum _STATE{
	STATE_UNKNOWN = 0,     /* unknown state */

	SENSOR_BORN = 1,       /* sensor born state */
	SENSOR_RESCHEDULE = 2, /* sensor reschedule state */
	SENSOR_ESTIMATE = 3,   /* sensor estimate state */
	SENSOR_SENSE = 4,      /* sensor sense state */
	SENSOR_SLEEP = 5,      /* sensor sleep state */
	SENSOR_DIE = 6,        /* sensor die state */

	VEHICLE_ARRIVE = 7,    /* vehicle arrive state */
	VEHICLE_CHECK = 8,     /* vehicle check state */
	VEHICLE_ESCAPE = 9,    /* vehicle escape state */
	VEHICLE_MOVE = 10,      /* vehicle move state */
	VEHICLE_DETECTED = 11, /* vehicle detected state */
	VEHICLE_RESTART = 12,  /* vehicle restart state */
	VEHICLE_OUT_OF_INTERSECTION = 13, /* vehicle out-of-intersection state: out of the communication range from the intersection */
	
	/** states for VANET */
	VANET_EDD_UPDATE = 14, /* vanet edd update state: build or rebuild the EDD */ 

	/** states for packet */
	PACKET_ARRIVE = 15,       /* packet generation state */
	PACKET_TRANSMITTED = 16,  /* packet transmission state */
	PACKET_RECEIVED = 17,     /* packet receiving state */
	PACKET_RECOMPUTE_TARGET_POINT = 18, /* packet state to recompute its target point */

	/** states for Convoy */
	CONVOY_CREATE = 19,    /* convoy create state */
	CONVOY_UPDATE = 20,    /* convoy update state */
	CONVOY_DELETE = 21,    /* convoy delete state */

	/** states for stationary vehicle */
	VEHICLE_STATIONARY_VEHICLE_START = 22, /* stationary-vehicle-start state where the stationary vehicle starts */
	VEHICLE_STATIONARY_VEHICLE_SEND = 23,  /* stationary-vehicle-send state where the stationary vehicle sends packets to moving vehicles */

	/** states for target vehicle */
	VEHICLE_TARGET_VEHICLE_START = 24,   /* state where a target vehicle starts its movement according to its vehicle trajectory */
	VEHICLE_TARGET_VEHICLE_RESTART = 25, /* state where a target vehicle restarts its movement according to its new vehicle trajectory */
	VEHICLE_TARGET_VEHICLE_RECEIVE = 26, /* state where a target vehicle receives a packet from packet carrier */

	/** states for Internet access point (AP) */
	AP_START = 27,         /* state where the AP starts its data forwarding operation */
	AP_UPDATE = 28,         /* state where the AP updates its vehicular traffic statistics */
	AP_PACKET_ARRIVE = 29, /* state where a packet is generated at the AP */
	AP_PACKET_SEND = 30,   /* state where the AP tries to send its packets to an appropriate carrier vehicle */

	/** states for target tracking */
	VEHICLE_UPDATE = 100,   /* vehicle's mobility update state */
        TRACK_REFRESH = 101     /* tracking area's refresh state */
} STATE;

/* delay type */
typedef enum _DELAY_TYPE{
	DELAY_TYPE_UNKNOWN = 0,
	DELAY_SENSOR = 1,                    /* think time at sensor */
	DELAY_VEHICLE_INTERARRIVAL_TIME = 2, /* vehicle's interarrival time */
	DELAY_VEHICLE_SPEED = 3,             /* vehicle's speed */
	DELAY_VEHICLE_THINK_TIME = 4,        /* vehicle's think time */
	DELAY_PACKET_INTERARRIVAL_TIME = 5   /* packet interarrival time */
} DELAY_TYPE;

/* vehicle movement type */
typedef enum _MOVE_TYPE{
	MOVE_UNKNOWN = 0,
	MOVE_FORWARD = 1, /* move the forward for the direction of edge that is road segment taken by the vehicle */
	MOVE_BACKWARD = 2 /* move the backward for the direction of edge that is road segment taken by the vehicle */
} MOVE_TYPE;

/* trace file type */
typedef enum _TRACE_TYPE{
  TRACE_UNKNOWN = 0,
  TRACE_SENSOR_NUMBER = 1,
  TRACE_HOLE_NUMBER = 2,
  TRACE_SLEEPING_TIME = 3,
  TRACE_VEHICLE_DETECTION_TIME = 4,
  TRACE_VEHICLE_CONVOY_LENGTH = 5
} TRACE_TYPE;

/* VANET log type */
typedef enum _VANET_LOG_TYPE{
	VANET_LOG_UNKNOWN = 0,
	VANET_LOG_PACKET_AP_ARRIVAL = 1, /* packet has successfully reached one of Internet access points */
	VANET_LOG_PACKET_DESTINATION_VEHICLE_ARRIVAL = 2, /* packet has successfully reached one of destination vehicles */
	VANET_LOG_PACKET_DROP = 3 /* packet has been dropped by the expiration of its TTL */
} VANET_LOG_TYPE;

/* Intersection area type */
typedef enum _intersection_area_type_t{
  INTERSECTION_AREA_UNKNOWN = 0, 
  INTERSECTION_AREA_TAIL_NODE = 1,  /* intersection area for the tail node of the directional edge where the vehicle is moving */
  INTERSECTION_AREA_HEAD_NODE = 2,  /* intersection area for the head node of the directional edge where the vehicle is moving */
  INTERSECTION_AREA_BOTH_NODES = 3, /* intersection area for both the tail and head nodes of the directional edge where the vehicle is moving */
  INTERSECTION_AREA_NONE = 4 /* not in the intersection area, that is, in the road segment far away from the communication for the two intersections*/
} intersection_area_type_t;

/* mobility type */
typedef enum _mobility_type_t
{
  MOBILITY_UNKNOWN = 0, //unknown mobility
  MOBILITY_OPEN = 1, //open mobility
  MOBILITY_CLOSED = 2, //closed mobility
  MOBILITY_STATIONARY = 3, //no mobility as stationary node
  MOBILITY_HYBRID = 4 //non-destination vehicle's mobility, that is, Hybrid mobility model of City Section Mobility model and Mahattan Mobility mode;
} mobility_type_t;

/* vanet node type */
typedef enum _vanet_node_type_t
{
  VANET_NODE_UNKNOWN = 0, //unknown vanet node type
  VANET_NODE_AP = 1, //access point
  VANET_NODE_VEHICLE = 2, //vehicle
  VANET_NODE_SNODE = 3 //stationary node for packet buffer at intersection
} vanet_node_type_t;

/** structure data types */
/* statistics for packet delivery */
typedef struct _packet_delivery_statistics_t{
  int generated_packet_number; //number of generated packets
  int generated_packet_copy_number; //number of generated packet copies in Epidemic Routing
  int delivered_packet_number; //number of delivered packets
  int discarded_packet_number; //number of discarded packets due to TTL expiration
  double packet_delivery_ratio; //packet_delivery_ratio = delivered_packet_number/generated_packet_number
  double expected_delivery_delay_sum; //sum of expected packet delivery delays
  double mean_expected_delivery_delay; //mean expected delivery delay
  double expected_delivery_delay_standard_deviation_sum; //sum of expected packet delivery delay standard deviations
  double mean_expected_delivery_delay_standard_deviation; //mean expected delivery delay standard deviation
  double actual_delivery_delay_sum; //sum of actual packet delivery delay
  double ap_arrival_delay_sum; //sum of actual packet delivery delay on ap arrival
  double destination_vehicle_arrival_delay_sum; //sum of actual packet delivery delay on vehicle arrival
  double mean_actual_delivery_delay; //mean actual delivery delay
  double ratio_of_two_delivery_delays; //ratio of mean expected delivery delay to mean actual delivery delay
  double delivery_delay_difference_sum; //sum of the difference from the actual delivery delay to the expected delivery delay
  double mean_delivery_delay_difference; //mean delivery delay difference
  int expected_packet_transmission_number_sum; //sum of expected packet transmission numbers
  int mean_expected_packet_transmission_number; //mean expected packet transmission number
  int actual_packet_transmission_number_sum; //sum of actual packet transmission numbers
  int mean_actual_packet_transmission_number; //mean actual packet transmission number
  int min_delivery_delay_difference; // min delivery delay difference
} packet_delivery_statistics_t;

#endif

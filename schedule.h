/**
 *  File: schedule.h
 *	Description: operations for scheduling for surveillance
 *	Date: 08/23/2007	
 *	Maker: Jaehoon Jeong
 */

#ifndef __SCHEDULE_H__
#define __SCHEDULE_H__

#include "common.h"
#include "queue.h"

/** road segment type */
typedef enum _enum_segment_type_t {
        SEGMENT_TYPE_SENSOR_SEGMENT = 0,
        SEGMENT_TYPE_HOLE_SEGMENT = 1
} enum_segment_type_t;

/** status type for scan direction in edge */
typedef enum _enum_status_t {
	STATUS_UNKNOWN = 0,
	STATUS_FORWARD = 1,  //scan direction: tail => head
	STATUS_BACKWARD = 2, //scan direction: head => tail
	STATUS_OVERLAP = 3   //two scans meet in the middle of the edge
} enum_status_t;

/** endpoint type for vertices in an edge */
typedef enum _enum_endpoint_t {
	ENDPOINT_UNKNOWN = 0,
	ENDPOINT_TAIL = 1, //tail node
	ENDPOINT_HEAD = 2  //head node
} enum_endpoint_t;

/** direction type for virtual edge's direction for the physical edge */
typedef enum _enum_edge_direction_t {
	EDGE_DIRECTION_UNKNOWN = 0,
	EDGE_DIRECTION_FORWARD = 1, //edge direction: physical edge's tail -> tail => head -> physical edge's head
	EDGE_DIRECTION_BACKWARD = 2 //edge direction: physical edge's tail -> head <= tail -> physical edge's head
} enum_edge_direction_t;

/** structure for schedule table node */
typedef struct _schedule_table_node_t {
        /** virtual edge information */
	int eid; //edge id; this indicates the undirectional edge including two directional edges with the same end points of the edge
        enum_segment_type_t type; //type of road segment = {SEGMENT_TYPE_SENSOR_SEGMENT, SEGMENT_TYPE_HOLE_SEGMENT};
	char tail_node[NAME_SIZE]; //tail node of a directional edge
	char head_node[NAME_SIZE]; //head node of a directional edge
        enum_edge_direction_t direction; //direction for the physical edge = {EDGE_DIRECTION_FORWARD, EDGE_DIRECTION_BACKWARD}
        double tail_node_offset_in_Gr; //physical offset of tail_node at the physical edge in Gr including this virtual edge (tail_node,head_node)
	double weight; //edge weight; that is, the length of the edge
	double density; //density of sensors deployed for unit distance

        /** scheduling information */
	enum_status_t status; //status of schedule intervals for two directional scans
	double arrival_time1;   //time when the first scan arrives at tail node
	double departure_time1; //time when the first leaves head node
	boolean flag1;          //flag to indicate whether arrival_time1 & departure_time1 are vaild
	double arrival_time2;   //time when the second scan arrives at head node
	double departure_time2; //time when the second scan leaves tail node
	boolean flag2;          //flag to indicate whether arrival_time2 & departure_time2 are vaild
        struct _schedule_table_node_t *next;
	struct _schedule_table_node_t *prev;
	sensor_queue_t sensor_list; //list of sensor nodes located on this edge
	subedge_queue_node_t *subedge; //pointer to the corresponding subedge in Er that is used to deal with the initial sensing hole handling
} schedule_table_node_t;

typedef struct _schedule_table_t {
	schedule_table_node_t head;
	int size;
	int sequence_number; //sequence number that has been allocated recently as an eid for a new edge
} schedule_table_t;

/** structure for sensor table */
typedef struct _struct_sensor_table
{
	int number; //number of table entries
	sensor_queue_node_t **list; //list of sensor node pointers that are maintained in sensor queue lists in schedule table entries
} struct_sensor_table;

/** function declarations */
void InitTable(schedule_table_t *T, struct_graph_node *Gv, int Gv_size, struct_graph_node *Gr, edge_queue_t *Er); 
 /* initialize table T by building the same number of entries as the number of edges in Gv
    and link T's entries with Er's subedge list entries each other. */


void ConstructScheduleTable(schedule_table_t *T, parameter_t *param, struct_graph_node *G, int G_size, struct_traffic_table *src_table, struct_traffic_table *dst_table);
//construct schedule table Q for scan-scheduling for intersection nodes in G with param and graph_file

void ResetScheduleTable(schedule_table_t *T);
//reset all of the fields related to sensing scheduling, such as flags and arrival_times, departure_times

schedule_table_node_t* Entable(schedule_table_t *T, schedule_table_node_t *node);
//enqueue node into table T

schedule_table_node_t* Detable(schedule_table_t *T);
//dequeue node from table T

void DeleteTableEntry(schedule_table_t *T, schedule_table_node_t *pTableNode);
//delete the entry for pTableEntry from table T

void DestroyScheduleTable(schedule_table_t *T);
//destory schedule table T

void DestroySensorTable(struct_sensor_table *S);
//destory sensor table S

schedule_table_node_t* LookupTable(struct_graph_node *Gv, char *u, char *v, boolean *flip_flag);
//return the pointer to schedule table entry corresponding to the edge consisting of vertices u and v

int SizeofTable(schedule_table_t *T);
//return the size of table T

//schedule_table_node_t* GetTableNode(schedule_table_t *T, int index);
//return the table node corresponding to index; the index of the first node is 0.

schedule_table_node_t* GetTableNodeByEID(schedule_table_t *T, int eid);
//return the table node corresponding to eid; the eid starts from 1.

boolean IsExpandable(struct_graph_node *G, schedule_queue_node_t *node);
//determine the expansion of searching tree for queue_node with schedule table entry (accessed by virtual graph G) and update the sensing scheduling for intersection nodes in G

int DeploySensorNetwork(parameter_t *param, schedule_table_t *T, struct_sensor_table *S, edge_queue_t *Q);
/* deploy sensor nodes into the road network by generating sensor nodes with sensor density,
   selecting their locations, and registering them with schedule table T, sensor table S, and edge queue Q */

void ComputeSensingSchedule(parameter_t *param, schedule_table_t *T, struct_sensor_table *S);
//compute sensing schedule for each sensor node using schedule table T and sensor table S

int CountSensorNumberByOffset(schedule_table_node_t *pTableNode, enum_status_t status_type, double x);
//count the number of sensors in front of offset x from the scan's arrival node of the edge corresponding to pTableNode

void ComputeSensorDutyCycle(sensor_queue_node_t *pSensor, schedule_table_node_t *pTableNode, double sensor_work_time, sensor_scan_type_t scan_type);
//compute sensor duty cycle, such as sensing start time and sensing end time

void ComputeSleepingSchedule(parameter_t *param, schedule_table_t *T, struct_sensor_table *S, struct_traffic_table *src_table, struct_traffic_table *dst_table, int G_size, double **D_move, int **M_move, int **D_scan, int **M_scan, double *movement_time, double *scanning_time, double *sleeping_time);
//compute sleeping schedule for each sensor node using schedule table T and sensor table S

double GetEarliestScanArrivalTimeForSources(struct_traffic_table *src_table, schedule_table_t *T);
//scanning_time is chosen as the minimum scan arrival time among scan arrival times for sources (entrances)

void ComputeSurveillanceSchedule(parameter_t *param, schedule_table_t *T, struct_sensor_table *S, struct_traffic_table *src_table, struct_traffic_table *dst_table, struct_graph_node *G, int G_size, double **D_move, int **M_move, int **D_scan, int **M_scan, double *movement_time, double *scanning_time, double *sleeping_time);
//perform the schedule for surveillance such as sensing schedule and sleeping schedule

void UpdateSurveillanceSchedule(parameter_t *param, schedule_table_t *T, struct_sensor_table *S, struct_traffic_table *src_table, struct_traffic_table *dst_table, struct_graph_node *G, int G_size, double **D_move, int **M_move, int **D_scan, int **M_scan, double *movement_time, double *scanning_time, double *sleeping_time);
//update the schedule for surveillance such as sensing schedule and sleeping schedule

boolean UpdateScheduleTableForSensorDeath(schedule_table_node_t *pTableNode, sensor_queue_node_t *pSensorNode);
//update schedule table T along with sensor_list by updating sensors' offsets

void PerformSensingSchedule(parameter_t *param, struct_sensor_table *S, STATE state, double current_time, int dying_sensor_id);
//perform the sensing schedule to schedule sensors for surveillance

void TakeActionForSensingHole(parameter_t *param, struct_graph_node **Gv, int *Gv_size, schedule_table_t *T, edge_queue_t *Er, struct_sensor_table *S, schedule_table_node_t *pTableNode, double left_hole_offset, double right_hole_offset, double ***Dv_move, int ***Mv_move, int *matrix_size_for_movement_in_Gv, int ***Dv_scan, int ***Mv_scan, int *matrix_size_for_scanning_in_Gv, struct_traffic_table *src_table_for_Gr, struct_traffic_table *dst_table_for_Gr, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *hole_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time, double current_time, int dying_sensor_id);
/* take action for sensing hole; update virtual graph and schedule table, 
   compute sensing schedule and sleeping schedule, and perform reschedule for sensors. */

void TakeActionForSensorDeath(parameter_t *param, struct_graph_node *Gv, int Gv_size, schedule_table_t *T, struct_sensor_table *S, schedule_table_node_t *pTableNode, sensor_queue_node_t *pSensorNode, double ***Dv_move, int ***Mv_move, int *matrix_size_for_movement_in_Gv, int ***Dv_scan, int ***Mv_scan, int *matrix_size_for_scanning_in_Gv, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time, double current_time, int dying_sensor_id);
/* take action for sensor's death in the scheduling based on variable scan speed; update schedule table, 
   compute sensing schedule and sleeping schedule, and perform reschedule for sensors. */

sensor_queue_node_t* GetSensorQueueNodeFromLocationQueue(location_queue_t *Q, int index, struct_sensor_table *S);
//return the sensor queue node pointed by the sensor table entry corresponding to index; the index of the first node is 0.

int MakeSurveillanceScheduleWithInitialSensingHoles(parameter_t *param, struct_graph_node **Gv, int *Gv_size, schedule_table_t *T, struct_sensor_table *S, double ***Dv_move, int ***Mv_move, int *matrix_size_for_movement_in_Gv, int ***Dv_scan, int ***Mv_scan, int *matrix_size_for_scanning_in_Gv, struct_traffic_table *src_table_for_Gr, struct_traffic_table *dst_table_for_Gr, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *hole_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time, edge_queue_t *Er);
/* make surveillance schedule by handling initial sensing holes by determining the roles of sensing holes as virtual entrances or virtual exits */

int FindSensingHoles(schedule_table_t *T, struct_sensor_table *S, edge_queue_t *Er);
//find initial sensing holes in road network Gr and return H containing sensing holes


void HandleInitialSensingHoles(edge_queue_t *Er, parameter_t *param, struct_graph_node **Gv, int *Gv_size, schedule_table_t *T, struct_sensor_table *S, double ***Dv_move, int ***Mv_move, int *matrix_size_for_movement_in_Gv, int ***Dv_scan, int ***Mv_scan, int *matrix_size_for_scanning_in_Gv, struct_traffic_table *src_table_for_Gr, struct_traffic_table *dst_table_for_Gr, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *hole_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time);
//handle the initial sensing holes that are represented as hole segments with two end-points using the sensing hole handling algorithm

void HandleSensingHoles_With_ExhaustiveSearchAlgorithm(parameter_t *param, struct_graph_node *Gv, int Gv_size, schedule_table_t *T, struct_sensor_table *S, double **Dv_move, int **Mv_move, int **Dv_scan, int **Mv_scan, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *src_or_dst_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time);
//handle the sensing holes that are represented as hole segments with two end-points using the exhaustive search algorithm

void HandleSensingHoles_With_MinimalSpanningTreeAlgorithm(parameter_t *param, struct_graph_node *Gv, int Gv_size, schedule_table_t *T, struct_sensor_table *S, double **Dv_move, int **Mv_move, int **Dv_scan, int **Mv_scan, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *src_or_dst_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time);
//handle the sensing holes that are represented as hole segments with two end-points using the MST-based labeling algorithm

void HandleSensingHoles_With_RandomLabelingAlgorithm(parameter_t *param, struct_graph_node *Gv, int Gv_size, schedule_table_t *T, struct_sensor_table *S, double **Dv_move, int **Mv_move, int **Dv_scan, int **Mv_scan, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *src_or_dst_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time);
//handle the sensing holes that are represented as hole segments with two end-points using the random labeling algorithm

void HandleSensingHoles_With_AllEntrancePointsAlgorithm(parameter_t *param, struct_graph_node *Gv, int Gv_size, schedule_table_t *T, struct_sensor_table *S, double **Dv_move, int **Mv_move, int **Dv_scan, int **Mv_scan, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *src_or_dst_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time);
//handle the sensing holes that are represented as hole segments with two end-points labeling all of the holes as entrance points

void HandleSensingHoles_With_AllProtectionPointsAlgorithm(parameter_t *param, struct_graph_node *Gv, int Gv_size, schedule_table_t *T, struct_sensor_table *S, double **Dv_move, int **Mv_move, int **Dv_scan, int **Mv_scan, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *src_or_dst_table_for_Gv, double *movement_time_for_sleeping, double *scanning_time_for_sleeping, double *sleeping_time);
//handle the sensing holes that are represented as hole segments with two end-points labeling all of the holes as protection points

schedule_table_node_t* GetTableNodeFromSubedgeList(subedge_queue_t *Q, hole_segment_queue_node_t *pHoleSegmentNode, double *left_hole_offset_in_Gv, double *right_hole_offset_in_Gv);
/* get the pointer to the exact schedule table node linked to the subedge containing the sensing hole segment from
   the updated schedule table T and get the offsets of the sensing hole segment in the virtual graph Gv */

void PerformClusteringWithExhaustiveSearch(parameter_t *param, schedule_table_t *T, struct_sensor_table *S, struct_traffic_table *src_table, struct_traffic_table *dst_table, struct_traffic_table *src_or_dst_table, struct_graph_node *G, int G_size, double **D_move, int **M_move, int **D_scan, int **M_scan);
//perform exhaustive search to find an optimal labeling for sensing holes as either entrance node or protection node

void PerformClusteringWithRandomLabeling(struct_traffic_table *src_table, struct_traffic_table *dst_table, struct_traffic_table *src_or_dst_table);
//perform random labeling to find a labeling for sensing holes as either entrance node or protection node

void PerformClusteringWithAllEntrancePoints(struct_traffic_table *src_table, struct_traffic_table *dst_table, struct_traffic_table *src_or_dst_table);
//perform random labeling to find a labeling for sensing holes as entrance points

void PerformClusteringWithAllProtectionPoints(struct_traffic_table *src_table, struct_traffic_table *dst_table, struct_traffic_table *src_or_dst_table);
//perform random labeling to find a labeling for sensing holes as entrance points

double GetMovementTimeOnPhysicalShortestPath(parameter_t *param, double **D_move, int protection_node, int entrance_node);
//return the vehicle movement time on the physical shortest path between a pair of an entrance node and a protection node

double GetScanningTimeOnScanSignalShortestPath(parameter_t *param, double **D_move, int **D_scan, int protection_node, int entrance_node);
//return the sensing scanning time on the scan-signal shortest path between a pair of an entrance node and a protection node

void RegisterTableNodeIntoVirtualGraph(struct_graph_node *G, int G_size, schedule_table_node_t *ptr_table_node);
//register the pointer to the table node in virtual graph Gv

#endif

#ifndef __TADB_H__
#define __TADB_H__

#include "queue.h"

int TADB_Get_RefTime_And_Target_Zone(parameter_t *param, struct_graph_node *Gr, struct_vehicle* receiver_vehicle, int *target_zone , double *refExpectedTime);
boolean TADB_Is_There_Next_Carrier_At_Intersection_For_AP(parameter_t *param,
	double current_time,
	struct_access_point_t *AP,
	struct_graph_node *G,
	int G_size,
	struct_vehicle_t **next_carrier);

#endif



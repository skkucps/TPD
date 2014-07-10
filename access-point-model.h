/** File: access-point-model.h
	Description: specify the macro constants, structures, and enum types for Internet access point model.
	Date: 02/24/2009
	Update: 07/12/2009
	Maker: Jaehoon Jeong, jjeong@cs.umn.edu
*/

#ifndef __ACCESS_POINT_MODEL_H__
#define __ACCESS_POINT_MODEL_H__

#include "common.h"
#include "graph-data-struct.h"
#include "queue.h"

/** AP operations */
void AP_Init(struct_access_point_t *AP, parameter_t *param, struct_graph_node* Gr, int Gr_size, char *ap_vertex, int ap_id);
//initialize the AP by setting the location in the road network with G and also by setting AP's vertex and id name with ap_vertex and ap_id, respectively 

void AP_Delete(struct_access_point_t *AP);
//delete the AP by freeing the memory allocated to AP

#endif


/**
 *  File: mst.h
 *	Description: operations for Minimal Spanning Tree (MST)
 *	Date: 10/31/2007
 *	Maker: Jaehoon Jeong
 */

#ifndef __MST_H__
#define __MST_H__

/*                                                     
typedef struct _graph_t
{
	vertex_set_t V; //vertex set
	edge_set_t E; //edge set
} graph_t;
*/

void MST_PerformClustering(parameter_t *param, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *src_or_dst_table_for_Gv, double **Dv_move, int **Dv_scan, edge_set_queue_t *MST_set);
/** perform the clustering for sensing holes (for labeling of entrance node or protection node)
    based on Minimal Spanning Tree (MST) algorithm to cluster sensing holes */

vertex_set_queue_t* MST_MakeClusterSet(vertex_set_queue_node_t* vertex_node, cluster_type_t cluster_type);
//make a cluster set for sensing hole labeling

vertex_set_queue_t* MST_MergeClusterSet(vertex_set_queue_t* set1, vertex_set_queue_t* set2, vertex_set_queue_node_t *pVertexNodeForTailNode, vertex_set_queue_node_t *pVertexNodeForHeadNode, vertex_set_queue_t *entrance_set, vertex_set_queue_t *protection_set, vertex_set_queue_t *hole_set);
//merge two cluster sets set1 and set2 and update the pointers to cluster sets for tail_node and head_node

vertex_set_queue_t* MST_FindClusterSet(char *vertex, vertex_set_queue_t *entrance_set, vertex_set_queue_t *protection_set, vertex_set_queue_t *hole_set);
//find the cluster set containing vertex

vertex_set_queue_node_t* MST_FindVertexSetNode(char *vertex, vertex_set_queue_t *entrance_set, vertex_set_queue_t *protection_set, vertex_set_queue_t *hole_set);
//find the vertex set node corresponding to vertex

void MST_UpdateTrafficTable(struct_traffic_table *traffic_table, vertex_set_queue_t *cluster_set);
//update traffic table using cluster set

#endif


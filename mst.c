/**
 *  File: mst.c
 *	Description: operations for Minimal Spanning Tree (MST)
 *	Date: 10/31/2007
 *	Maker: Jaehoon Jeong
 */

#include "stdafx.h"
#include "common.h"
#include "graph-data-struct.h"
#include "queue.h"
#include "util.h"
#include "shortest-path.h"
#include "schedule.h"
#include "mst.h"

void MST_PerformClustering(parameter_t *param, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, struct_traffic_table *src_or_dst_table_for_Gv, double **Dv_move, int **Dv_scan, edge_set_queue_t *MST_set)
{ /** perform the clustering for sensing holes (for labeling of entrance node or protection node)
      based on Minimal Spanning Tree (MST) algorithm to cluster sensing holes */
	vertex_set_queue_t entrance_set; //entrance node set including sensing holes
	vertex_set_queue_t protection_set; //protection node set including sensing holes
	vertex_set_queue_t hole_set; //sensing hole set in virtual topology Tv

	vertex_set_queue_t Tv_V; //vertex set of virtual topology Tv
	edge_set_queue_t Tv_E; //edge set of virtual topology Tv
	vertex_set_queue_node_t vertex_node; //vertex_set queue node
	edge_set_queue_node_t edge_node; //edge_set queue node
	int i, j; //indices of for-loop
	int index1, index2; //indices for 2x2 matrix, such as Dv_move and Dv_scan
	vertex_set_queue_node_t *p = NULL, *q = NULL; //pointers to vertex set nodes
	int vertex_number = 0; //number of vertices in virtual topology
	int count = 0; //counter of edges in MST
	edge_set_queue_node_t *pEdgeNode = NULL; //pointer to edge_set queue node
	vertex_set_queue_node_t *pVertexNodeForTailNode = NULL; //pointer to the vertex set node corresponding to tail_node of an edge
	vertex_set_queue_node_t *pVertexNodeForHeadNode = NULL; //pointer to the vertex set node corresponding to head_node of an edge
	vertex_set_queue_t *cluster_set_for_tail_node = NULL; //pointer to the cluster set for tail_node of an edge
	vertex_set_queue_t *cluster_set_for_head_node = NULL; //pointer to the cluster set for head_node of an edge

	double move_time = INF; //vehicle's movement time
	double scan_time = INF; //sensor scanning time
	double sleep_time = INF; //sensor sleeping time

	/** initialize vertex sets and edge sets */
	InitQueue((queue_t*) &entrance_set, QTYPE_VERTEX_SET);
	InitQueue((queue_t*) &protection_set, QTYPE_VERTEX_SET);
	InitQueue((queue_t*) &hole_set, QTYPE_VERTEX_SET);
	InitQueue((queue_t*) &Tv_V, QTYPE_VERTEX_SET);
	InitQueue((queue_t*) &Tv_E, QTYPE_EDGE_SET);
	InitQueue((queue_t*) MST_set, QTYPE_EDGE_SET);

	/** construct the vertex set Tv_V of virtual topology Tv along with entrance set, protection set, and sensing hole set */
	/* construct the vertex set Tv_V and entrance set */
	for(i = 0; i < src_table_for_Gv->number; i++)
	{
		memset(&vertex_node, 0, sizeof(vertex_node));
		//strcpy(vertex_node.vertex, src_table_for_Gv->list[i]);
                strcpy(vertex_node.vertex, src_table_for_Gv->list[i].vertex);
		vertex_node.cluster_type = CLUSTER_ENTRANCE; 
		Enqueue((queue_t*)&Tv_V, (queue_node_t*)&vertex_node);
		Enqueue((queue_t*)&entrance_set, (queue_node_t*)&vertex_node);
	}

	/* construct the vertex set Tv_V and protection set */
	for(i = 0; i < dst_table_for_Gv->number; i++)
	{
		memset(&vertex_node, 0, sizeof(vertex_node));
		//strcpy(vertex_node.vertex, dst_table_for_Gv->list[i]);
                strcpy(vertex_node.vertex, dst_table_for_Gv->list[i].vertex);
		vertex_node.cluster_type = CLUSTER_PROTECTION;
		Enqueue((queue_t*)&Tv_V, (queue_node_t*)&vertex_node);
		Enqueue((queue_t*)&protection_set, (queue_node_t*)&vertex_node);
	}

	/* construct the vertex set Tv_V and sensing hole set */
	for(i = 0; i < src_or_dst_table_for_Gv->number; i++)
	{
		memset(&vertex_node, 0, sizeof(vertex_node));
		//strcpy(vertex_node.vertex, src_or_dst_table_for_Gv->list[i]);
                strcpy(vertex_node.vertex, src_or_dst_table_for_Gv->list[i].vertex);
		vertex_node.cluster_type = CLUSTER_HOLE;
		Enqueue((queue_t*)&Tv_V, (queue_node_t*)&vertex_node);
		Enqueue((queue_t*)&hole_set, (queue_node_t*)&vertex_node);
	}

	/** construct the edge set Tv_E of virtual topology Tv */
	p = &(Tv_V.head);
	for(i = 0; i < Tv_V.size; i++)
	{
		p = p->next;
		q = p;
		for(j = i+1; j < Tv_V.size; j++)
		{
			q = q->next;
			memset(&edge_node, 0, sizeof(edge_node));
			strcpy(edge_node.tail_node, p->vertex);
			strcpy(edge_node.head_node, q->vertex);
			index1 = atoi(edge_node.tail_node) - 1;
			index2 = atoi(edge_node.head_node) - 1; 
			//edge_node.weight = Dv_move[index1][index2];
			//edge_node.weight = Dv_scan[index1][index2];
			//edge_node.weight = Dv_scan[index1][index2] + Dv_move[index1][index2];

			move_time = GetMovementTimeOnPhysicalShortestPath(param, Dv_move, index1, index2); //return the vehicle movement time on the physical shortest path between a pair of an entrance node and a protection node
			scan_time = GetScanningTimeOnScanSignalShortestPath(param, Dv_move, Dv_scan, index1, index2); //return the sensing scanning time on the scan-signal shortest path between a pair of an entrance node and a protection node
			sleep_time = scan_time + move_time; //sleeping time between a pair of destination (i.e., protection node) and source (i.e., entrance node)

			edge_node.weight = sleep_time; //edge wight is sleeping time
			Enqueue((queue_t*)&Tv_E, (queue_node_t*)&edge_node);
		}
	}

	/** make an individulal cluster set for each vertex in entrance set */
	p = &(entrance_set.head);
	for(i = 0; i < entrance_set.size; i++)
	{
		p = p->next;
		p->cluster_set = MST_MakeClusterSet(p, CLUSTER_ENTRANCE);
	}

	/** make an individulal cluster set for each vertex in protection set */
	p = &(protection_set.head);
	for(i = 0; i < protection_set.size; i++)
	{
		p = p->next;
		p->cluster_set = MST_MakeClusterSet(p, CLUSTER_PROTECTION);
	}

	/** make an individulal cluster set for each vertex in sensing hole set Tv_H */
	p = &(hole_set.head);
	for(i = 0; i < hole_set.size; i++)
	{
		p = p->next;
		p->cluster_set = MST_MakeClusterSet(p, CLUSTER_HOLE);
	}

	/** sort the edges of Tv_E by nondecreasing weigts */
	SortEdgeSetQueue(&Tv_E);
	
	/** get the number of vertices in virtual topology Tv */
	vertex_number = Tv_V.size;

	/** select MST edges to satisfy our clustering criteria */
	pEdgeNode = &(Tv_E.head);
	while(count < vertex_number - 2)
	//while(count < vertex_number - 1)
	{
		/** pick a shortest edge from edge_set Tv_E that is not used before */
		pEdgeNode = pEdgeNode->next;
		if(pEdgeNode == &(Tv_E.head))
			break; //we search all of the edges in the edge set queue
		
		/** get the vertex_set node pointers for tail_node and head_node of the selected edge */
		pVertexNodeForTailNode = MST_FindVertexSetNode(pEdgeNode->tail_node, &entrance_set, &protection_set, &hole_set);
		pVertexNodeForHeadNode = MST_FindVertexSetNode(pEdgeNode->head_node, &entrance_set, &protection_set, &hole_set);

		/** check the validity to whether this edge can be added int MST;
		    When one end-point of the edge is included in CLUSTER_ENTRANCE and the other end-point of the edge is included in CLUSTER_PROTECTION,
			this edge is not valid for MST, so take the next shortest edge */
		
		if((pVertexNodeForTailNode->cluster_set->cluster_type == CLUSTER_ENTRANCE && pVertexNodeForHeadNode->cluster_set->cluster_type == CLUSTER_PROTECTION) ||
		   (pVertexNodeForTailNode->cluster_set->cluster_type == CLUSTER_PROTECTION && pVertexNodeForHeadNode->cluster_set->cluster_type == CLUSTER_ENTRANCE))
		   continue;
						
		/** get the cluster_set pointers to the vertex_set nodes for tail_node and head_node of the selected edge */
		cluster_set_for_tail_node = pVertexNodeForTailNode->cluster_set;
		cluster_set_for_head_node = pVertexNodeForHeadNode->cluster_set;
		
		if(&(cluster_set_for_tail_node->head) != &(cluster_set_for_head_node->head))
		{ //check whether tail_node and head_node are in the same cluster set

			/* union two cluster sets and update the pointers to cluster sets for tail_node and head_node */
			MST_MergeClusterSet(cluster_set_for_tail_node, cluster_set_for_head_node, pVertexNodeForTailNode, pVertexNodeForHeadNode, &entrance_set, &protection_set, &hole_set);

			/* add a new MST edge into MST_set */
			memset(&edge_node, 0, sizeof(edge_node));
			strcpy(edge_node.tail_node, pEdgeNode->tail_node);
			strcpy(edge_node.head_node, pEdgeNode->head_node);
			index1 = atoi(edge_node.tail_node) - 1;
			index2 = atoi(edge_node.head_node) - 1; 
			//edge_node.weight = Dv_move[index1][index2];
			//edge_node.weight = Dv_scan[index1][index2];
			//edge_node.weight = Dv_scan[index1][index2] + Dv_move[index1][index2];

			move_time = GetMovementTimeOnPhysicalShortestPath(param, Dv_move, index1, index2); //return the vehicle movement time on the physical shortest path between a pair of an entrance node and a protection node
			scan_time = GetScanningTimeOnScanSignalShortestPath(param, Dv_move, Dv_scan, index1, index2); //return the sensing scanning time on the scan-signal shortest path between a pair of an entrance node and a protection node
			//move_time = GetMovementTimeOnPhysicalShortestPath(param, D_move, index2, index1); //return the vehicle movement time on the physical shortest path between a pair of an entrance node and a protection node
			//scan_time = GetScanningTimeOnScanSignalShortestPath(param, D_scan, index2, index1); //return the sensing scanning time on the scan-signal shortest path between a pair of an entrance node and a protection node
			sleep_time = scan_time + move_time; //sleeping time between a pair of destination (i.e., protection node) and source (i.e., entrance node)

			edge_node.weight = sleep_time; //edge wight is sleeping time

			Enqueue((queue_t*)MST_set, (queue_node_t*)&edge_node);

			/* increase count for the number of edges in MST_set */
			count++;
		}
	}

	/** update src_table_for_Gv and dst_table_for_Gv using the entrance cluster set and the protection cluster set */
	MST_UpdateTrafficTable(src_table_for_Gv, entrance_set.head.next->cluster_set); //update traffic table src_table_for_Gv using cluster set
	MST_UpdateTrafficTable(dst_table_for_Gv, protection_set.head.next->cluster_set); //update traffic table dst_table_for_Gv using cluster set

	/** destroy vertex sets and edge sets */
	DestroyQueue((queue_t*) &entrance_set);
	DestroyQueue((queue_t*) &protection_set);
	DestroyQueue((queue_t*) &hole_set);
	DestroyQueue((queue_t*) &Tv_V);
	DestroyQueue((queue_t*) &Tv_E);	

#ifdef __DEBUG_LEVEL_SORT_TRAFFIC_TABLE__
	/** NOTE: If we want to see the vertex names in ascending order for debugging purpose, 
	    we perform the sorting of src_table and dst_table according to vertex name in asending order. */
	SortTrafficTable(src_table_for_Gv);
	SortTrafficTable(dst_table_for_Gv);
#endif
}

vertex_set_queue_t* MST_MakeClusterSet(vertex_set_queue_node_t* vertex_node, cluster_type_t cluster_type)
{ //make a cluster set for sensing hole labeling
	vertex_set_queue_t *cluster_set = NULL;

	cluster_set = (vertex_set_queue_t*) calloc(1, sizeof(vertex_set_queue_t));
	assert_memory(cluster_set);

	/** initialize vertex set for clustering */
	InitQueue((queue_t*) cluster_set, QTYPE_VERTEX_SET);

	Enqueue((queue_t*)cluster_set, (queue_node_t*)vertex_node);
	cluster_set->cluster_type = cluster_type; //set the cluster type of the cluster set to cluster_type
	cluster_set->reference_count = 1; //set reference count to 1
	cluster_set->head.cluster_type = cluster_type; //set the cluster type of the cluster set's head node to cluster_type

	return cluster_set;
}

vertex_set_queue_t* MST_MergeClusterSet(vertex_set_queue_t* set1, vertex_set_queue_t* set2, vertex_set_queue_node_t *pVertexNodeForTailNode, vertex_set_queue_node_t *pVertexNodeForHeadNode, vertex_set_queue_t *entrance_set, vertex_set_queue_t *protection_set, vertex_set_queue_t *hole_set)
{ //merge two cluster sets set1 and set2 and update the pointers to cluster sets for tail_node and head_node
	vertex_set_queue_t *merged_set = NULL; //pointer to the merged cluster set
	vertex_set_queue_t *subset = NULL; //pointer to the smaller set between set1 and set1 in term of size or the set of cluster type CLUSTER_HOLE
	vertex_set_queue_node_t *p = NULL;
	vertex_set_queue_node_t *q = NULL;
	int i; //index of for-loop

	/* select merged_set and small set with set size and set type */
	if(set1->cluster_type == set2->cluster_type)
	{
		if(set1->size >= set2->size)
		{
			merged_set = set1;
			subset = set2;
		}
		else
		{
			merged_set = set2;
			subset = set1;
		}
	}
	else if((set1->cluster_type == CLUSTER_HOLE) || (set2->cluster_type == CLUSTER_HOLE))
	{
		if(set1->cluster_type == CLUSTER_HOLE) //set1's cluster type is CLUSTER_HOLE
		{
			merged_set = set2;
			subset = set1;
		}
		else //set2's cluster type is CLUSTER_HOLE
		{
			merged_set = set1;
			subset = set2;
		}
	}
	else
	{
		printf("MST_MergeClusterSet(): we cannot merge two sets where one of them is CLUSTER_ENTRANCE and one of them is CLUSTER_PROTECTION: set1's cluster type(%d) and set2's cluster type(%d)\n", set1->cluster_type, set2->cluster_type);
		exit(1);
	}

	p = &(subset->head);
	for(i = 0; i < subset->size; i++)
	{
		p = p->next;
		Enqueue((queue_t*) merged_set, (queue_node_t*) p); 

		/* NOTE: update cluster_set pointer of the vertex set node corresponding to p->vertex */
		q = MST_FindVertexSetNode(p->vertex, entrance_set, protection_set, hole_set);
		q->cluster_set = merged_set;
	}

	/*
	if(subset == set1)
	{
		//update the pointers to cluster sets for tail_node after deleting the cluster set pointed by pVertexNodeForTailNode->cluster_set
		//DestroyQueue((queue_t*) pVertexNodeForTailNode->cluster_set);
		//pVertexNodeForTailNode->cluster_set = merged_set;
	}
	else
	{
		//update the pointers to cluster sets for head_node after deleting the cluster set pointed by pVertexNodeForHeadNode->cluster_set
		//DestroyQueue((queue_t*) pVertexNodeForHeadNode->cluster_set);
		//pVertexNodeForHeadNode->cluster_set = merged_set;
	}
	*/

	/** delete the cluset set pointed by subset */
	DestroyQueue((queue_t*) subset);

	merged_set->reference_count++; //increment the reference count of the cluster set pointed by merged_set

	return merged_set;
}

vertex_set_queue_t* MST_FindClusterSet(char *vertex, vertex_set_queue_t *entrance_set, vertex_set_queue_t *protection_set, vertex_set_queue_t *hole_set)
{ //find the cluster set containing vertex
	vertex_set_queue_node_t *p = NULL; //pointer to a vertex_set queue node

	p = MST_FindVertexSetNode(vertex, entrance_set, protection_set, hole_set); //find the vertex set node corresponding to vertex

	if(p == NULL)
		return NULL;
	
	return p->cluster_set;
}

vertex_set_queue_node_t* MST_FindVertexSetNode(char *vertex, vertex_set_queue_t *entrance_set, vertex_set_queue_t *protection_set, vertex_set_queue_t *hole_set)
{ //find the vertex set node corresponding to vertex
	vertex_set_queue_node_t *wanted_node = NULL; //pointer to the wanted vertex set node corresponding to vertex
	vertex_set_queue_node_t *p = NULL; //pointer to a vertex_set queue node
	int i; //index of for-loop

	/* search entrance_set to find vertex */
	p = &(entrance_set->head);
	for(i = 0; i < entrance_set->size; i++)
	{
		p = p->next;
		if(strcmp(vertex, p->vertex) == 0)
		{
			wanted_node = p;
			return wanted_node;
		}
	}

	/* search protection_set to find vertex */
	p = &(protection_set->head);
	for(i = 0; i < protection_set->size; i++)
	{
		p = p->next;
		if(strcmp(vertex, p->vertex) == 0)
		{
			wanted_node = p;
			return wanted_node;
		}
	}

	/* search hole_set to find vertex */
	p = &(hole_set->head);
	for(i = 0; i < hole_set->size; i++)
	{
		p = p->next;
		if(strcmp(vertex, p->vertex) == 0)
		{
			wanted_node = p;
			return wanted_node;
		}
	}

	return wanted_node;
}

void MST_UpdateTrafficTable(struct_traffic_table *traffic_table, vertex_set_queue_t *cluster_set)
{ //update traffic table using cluster set
	vertex_set_queue_node_t *p = NULL; //pointer to vertex set node
	int i; //index of for-loop

	/** destroy traffic table */
	Free_Traffic_Table(traffic_table); 

	p = &(cluster_set->head);
	for(i = 0; i < cluster_set->size; i++)
	{
		p = p->next;
		AddTrafficTableEntry(traffic_table, p->vertex); //add traffic table entry corresponding to node to table
	}
}



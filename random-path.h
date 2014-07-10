/**
 *  File: random-path.h
	Description: operations for the random path from the source to the destination 
	 satisfying the lower bound and the upper bound.
	Date: 11/13/2006	
	Maker: Jaehoon Jeong
*/

#ifndef __RANDOM_PATH_H__
#define __RANDOM_PATH_H__

#include "graph-data-struct.h"
#include "param.h"

struct_path_node* Random_Path_Make_Path_List(int src, int dst, struct_graph_node *G, int G_size, double** D, int** M, parameter_t* param, int *path_hop_count);
//make a random_path between source src and destination dst on graph G along with the all-pairs shortest path information D & M which are shortest distance matrix and predecessor matrix, respectively.

struct_path_node* Random_Path_Make_Path_List_Before_The_Closest_Protection_Point(int src, int dst, struct_graph_node *G, int G_size, double** D, int** M, parameter_t* param, struct_traffic_table *protection_set, int *path_hop_count);
//make a random_path between source src and destination dst on graph G before the closest protection point on the path, along with the all-pairs shortest path information D & M which are shortest distance matrix and predecessor matrix, respectively.


struct_path_node* Random_Path_List_Init();
//initialize the list for a random path

struct_path_node* Random_Path_List_Backward_Insert(struct_path_node* h, struct_path_node *y, int u, double w);
//insert node u into path_list using pointer y towards the source with edge weight w

struct_path_node* Random_Path_List_Forward_Insert(struct_path_node* h, struct_path_node *x, int u, double w);
//insert node u into path_list using pointer x towards the destination with edge weight w

struct_path_node* Random_Path_List_Middle_Insert(struct_path_node* h, struct_path_node *x, struct_path_node *y, struct_graph_node *G, int G_size, int** M, int *path_hop_count);
//complete the random path by inserting the intermediate nodes on the shortest path between x and y

double Random_Path_Get_Weight(int u, int v, struct_graph_node *G, int G_size);
//get the weight of the edge between nodes u and v

#endif

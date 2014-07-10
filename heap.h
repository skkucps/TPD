/**
 *  File: heap.h
	Description: operations for heap that implements the minimum priority queue
	Date: 09/22/2006	
	Maker: Jaehoon Jeong
*/

#ifndef __HEAP_H__
#define __HEAP_H__

#include "graph-data-struct.h"

int Parent(int i); //return the index of the parent node of node i

int Left(int i); //return the index of the left child of node i

int Right(int i); //return the index of the right child of node i

void Min_Heapify(struct_shortest_path_node *Q, int Q_size, int i);
//heapify the current array 

void Exchange(struct_shortest_path_node *x, struct_shortest_path_node *y);
//exchange the values of two nodes

void Build_Min_Heap(struct_shortest_path_node *Q, int Q_size);
//build minimum priority heap

struct_shortest_path_node* Heap_Minimum(struct_shortest_path_node *Q);
//return the node minimum value from heap Q

struct_shortest_path_node* Heap_Extract_Min(struct_shortest_path_node *Q, int Q_size);
//extract the node minimum value from heap Q

void Heap_Decrease_Key(struct_shortest_path_node *Q, int Q_size, int i, double key);
//decrease the key (i.e., dist) of the node specified by i in heap Q

void Min_Heap_Insert(struct_shortest_path_node *Q, int Q_size, struct_shortest_path_node *new_node);
//insert a new node into heap Q

#endif

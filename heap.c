/**
 *  File: heap.c
	Description: operations for heap that implements the minimum priority queue
	Date: 09/22/2006	
	Maker: Jaehoon Jeong
*/

#include "stdafx.h"
#include "heap.h"
//#include "stdio.h"

int Parent(int i)
{ //return the index of the parent node of node i
	return (int) i/2;
}

int Left(int i)
{ //return the index of the left child of node i
	return 2*i;
}

int Right(int i)
{ //return the index of the right child of node i
	return 2*i + 1;
}

void Min_Heapify(struct_shortest_path_node *Q, int Q_size, int i)
{ //heapify the current array 
	int l, r, smallest;

	l = Left(i);
	r = Right(i);

	if((l <= Q_size) && (Q[l].dist < Q[i].dist))
		smallest = l;
	else
		smallest = i;

	if((r <= Q_size) && (Q[r].dist < Q[smallest].dist))
		smallest = r;

	if(smallest != i)
	{
		Exchange(&Q[i], &Q[smallest]);
		Min_Heapify(Q, Q_size, smallest);
	}	
}

void Exchange(struct_shortest_path_node *x, struct_shortest_path_node *y)
{ //exchange the values of two nodes
	struct_shortest_path_node temp;

	memcpy(&temp, y, sizeof(struct_shortest_path_node));
	memcpy(y, x, sizeof(struct_shortest_path_node));
	memcpy(x, &temp, sizeof(struct_shortest_path_node));
}

void Build_Min_Heap(struct_shortest_path_node *Q, int Q_size)
{ //build minimum priority heap
	int i;

	for(i = (int)Q_size/2; i >= 1; i--)
		Min_Heapify(Q, Q_size, i);
}

struct_shortest_path_node* Heap_Minimum(struct_shortest_path_node *Q)
{ //return the node minimum value from heap Q
	return &Q[1];
}

struct_shortest_path_node* Heap_Extract_Min(struct_shortest_path_node *Q, int Q_size)
{ //extract the node minimum value from heap Q
	static struct_shortest_path_node min;

	if(Q_size < 1)
	{
		printf("Heap_Extract_Min(): heap underflow\n");
		return NULL;
	}

	memcpy(&min, &Q[1], sizeof(struct_shortest_path_node));
	memcpy(&Q[1], &Q[Q_size], sizeof(struct_shortest_path_node));
	//Q[1] = Q[Q_size];
	Q_size--;
	Min_Heapify(Q, Q_size, 1);

	return &min;
}

void Heap_Decrease_Key(struct_shortest_path_node *Q, int Q_size, int i, double key)
{ //decrease the key (i.e., dist) of the node specified by i in heap Q
	if(Q_size == 0)
	{
		printf("Heap_Decrease_Key(): Heap size is 0\n");
		return;
	}

	if(key > Q[i].dist)
	{
		printf("Heap_Decrease_Key(): new key is greater than current key\n");
		return;
	}

	Q[i].dist = key;
	while(i > 1 && Q[Parent(i)].dist > Q[i].dist)
	{
		Exchange(&Q[i], &Q[Parent(i)]);
		i = Parent(i);
	}
}

void Min_Heap_Insert(struct_shortest_path_node *Q, int Q_size, struct_shortest_path_node *new_node)
{ //insert a new node into heap Q
	Q_size++;
	memcpy(&Q[Q_size], new_node, sizeof(struct_shortest_path_node));
	Q[Q_size].dist = INF;
	Heap_Decrease_Key(Q, Q_size, Q_size, new_node->dist);
}

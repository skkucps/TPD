/**
 *  File: random-path.c
	Description: operations for the random path from the source to the destination 
	 satisfying the lower bound and the upper bound.
	Date: 11/13/2006	
	Maker: Jaehoon Jeong
*/

#include "stdafx.h"
#include "rand.h"
#include "random-path.h"
#include "util.h"
#include "shortest-path.h" //IsNeighbor()

struct_path_node* Random_Path_Make_Path_List(int src, int dst, struct_graph_node *G, int G_size, double** D, int** M, parameter_t* param, int *path_hop_count)
{ //make a random_path between source src and destination dst on graph G along with the all-pairs shortest path information D & M which are shortest distance matrix and predecessor matrix, respectively.
  
	double LengthFromSrc = 0;
	double LengthFromDst = 0;
	double TotalDistance = 0;
	int X = src; //last node visited from random walk starting at src
	int Y = dst; //last node visited from random walk starting at dst
	int Count = 0; //count indicating the position of a neighbor randomly chosen
	int Z = 0; //next node to take in a random walk
	double Length = 0; //length of a random path
	double Mean = 0; //length of a shortest path
	int Coin = 0; //Result of coin tossing: 0 means head and 1 means tail
	struct_graph_node* Z_Node = NULL; //neighbor node
	struct_path_node* Path_Head = NULL; //path head indicating the random path between src and dst
	struct_path_node* X_Ptr = NULL; //pointer to node X
	struct_path_node* Y_Ptr = NULL; //pointer to node Y
	double Weight = 0; //edge weight

	Path_Head = Random_Path_List_Init(); //make the head node for the random path

	/* randomly select a route length */
	if(src > G_size || dst > G_size)
	{
		printf("Make_Random_Path(): src (%d) > G_size (%d) or dst (%d) > G_size (%d)\n", src, G_size, dst, G_size);
		exit(1);
	}
	else if(src == dst)
	{
		printf("Make_Random_Path(): src (%d) equals dst (%d)\n", src, dst);
		return NULL;
	}

	Mean = D[src-1][dst-1];
	do
	{
		Length = (int) dist_func(param->vehicle_path_length_distribution, Mean*1.0, param->vehicle_path_length_standard_deviation*1.0);
	} while(Length < Mean); //keep doing until go get Length >= Mean

	/* insert nodes X and Y into path list */
	X_Ptr = Y_Ptr = Path_Head;
	Z = X; Weight = 0;
	X_Ptr = Random_Path_List_Forward_Insert(Path_Head, X_Ptr, Z, Weight);
	Z = Y; Weight = 0;
	Y_Ptr = Random_Path_List_Backward_Insert(Path_Head, Y_Ptr, Z, Weight);

	*path_hop_count = 0; //initialize path_hop_count with zero
	do //do-while-1
	{
/*
		if(IsNeighbor(X, Y, G, G_size) == TRUE)
		{
			//update the weight of edge <X,Y>
			Weight = Random_Path_Get_Weight(X, Y, G, G_size);
			Y_Ptr->weight = Weight;
		}
*/

		Coin = smpl_random(0, 1); //fair coin tossing

		if(Coin == 0) //coin head: random walk from src
		{
			//if(src == 6 && dst == 2)
			//	printf("here\n");

		        Count = smpl_random(1, (int)G[X-1].weight); //select the outgoing edge
			if(Count == 0) //smpl_random might return 0
			        Count++;

			/* obtain the pointer to the selected outgoing edge */
			Z_Node = &G[X-1];
			while(Count > 0)
			{
				Z_Node = Z_Node->next;
				Count--;
			}

			Z = atoi(Z_Node->vertex);
			Weight = Random_Path_Get_Weight(X, Z, G, G_size); //obtain the length of edge (X,Z) 
			TotalDistance = Weight + LengthFromSrc + LengthFromDst + D[Z-1][Y-1];
			if((TotalDistance > Length) || (Z == Y))
			{
				Random_Path_List_Middle_Insert(Path_Head, X_Ptr, Y_Ptr, G, G_size, M, path_hop_count);
				break;
			}
			else
			{
				X = Z;
				X_Ptr = Random_Path_List_Forward_Insert(Path_Head, X_Ptr, Z, Weight);
                                LengthFromSrc += Weight;
				
				(*path_hop_count)++; //increment path_hop_count
			}
		}
		else //coin tail: random walk from dst
		{
			Count = smpl_random(1, (int)G[Y-1].weight); //select the outgoing edge
			if(Count == 0) //smpl_random might return 0
			        Count++;

			/* obtain the pointer to the selected outgoing edge */
			Z_Node = &G[Y-1];
			while(Count > 0) //while-2
			{
				Z_Node = Z_Node->next;
				Count--;
			} //end of while-2

			Z = atoi(Z_Node->vertex);
			Weight = Random_Path_Get_Weight(Y, Z, G, G_size);
			TotalDistance = Weight + LengthFromSrc + LengthFromDst + D[Z-1][X-1];
			if((TotalDistance > Length) || (Z == X))
			{
				Random_Path_List_Middle_Insert(Path_Head, X_Ptr, Y_Ptr, G, G_size, M, path_hop_count);
				break;			
			}
			else
			{
				Y = Z;
				Y_Ptr = Random_Path_List_Backward_Insert(Path_Head, Y_Ptr, Z, Weight);
                                LengthFromDst += Weight;

				(*path_hop_count)++; //increment path_hop_count
			}
		} //end of else
	} while(1); //end of do-while-1

	/* set Path_Hist's length to *path_hop_count + 1 */
	Path_Head->weight = *path_hop_count + 1;

	return Path_Head;
}

struct_path_node* Random_Path_Make_Path_List_Before_The_Closest_Protection_Point(int src, int dst, struct_graph_node *G, int G_size, double** D, int** M, parameter_t* param, struct_traffic_table *protection_set, int *path_hop_count)
{ //make a random_path between source src and destination dst on graph G before the closest protection point on the path, along with the all-pairs shortest path information D & M which are shortest distance matrix and predecessor matrix, respectively.
  
	double LengthFromSrc = 0;
	double LengthFromDst = 0;
	double TotalDistance = 0;
	int X = src; //last node visited from random walk starting at src
	int Y = dst; //last node visited from random walk starting at dst
	int Count = 0; //count indicating the position of a neighbor randomly chosen
	int Z = 0; //next node to take in a random walk
	double Length = 0; //length of a random path
	double Mean = 0; //length of a shortest path
	int Coin = 0; //Result of coin tossing: 0 means head and 1 means tail
	struct_graph_node* Z_Node = NULL; //neighbor node
	struct_path_node* X_Ptr = NULL; //pointer to node X
	struct_path_node* Y_Ptr = NULL; //pointer to node Y
	double Weight = 0; //edge weight
	struct_path_node* Path_Head = NULL; //path head indicating the random path between src and dst
	struct_path_node* new_path_list = NULL; //new path list from src to the closest protection point towards dst
	struct_path_node *path_ptr = NULL; //pointer to path node
	
	Path_Head = Random_Path_List_Init(); //make the head node for the random path

	/* randomly select a route length */
	if(src > G_size || dst > G_size)
	{
		printf("Make_Random_Path(): src (%d) > G_size (%d) or dst (%d) > G_size (%d)\n", src, G_size, dst, G_size);
		exit(1);
	}
	else if(src == dst)
	{
		printf("Make_Random_Path(): src (%d) equals dst (%d)\n", src, dst);
		return NULL;
	}

	Mean = D[src-1][dst-1];
	do
	{
		Length = (int) dist_func(param->vehicle_path_length_distribution, Mean*1.0, param->vehicle_path_length_standard_deviation*1.0);
	} while(Length < Mean); //keep doing until go get Length >= Mean

	/* insert nodes X and Y into path list */
	X_Ptr = Y_Ptr = Path_Head;
	Z = X; Weight = 0;
	X_Ptr = Random_Path_List_Forward_Insert(Path_Head, X_Ptr, Z, Weight);
	Z = Y; Weight = 0;
	Y_Ptr = Random_Path_List_Backward_Insert(Path_Head, Y_Ptr, Z, Weight);

	*path_hop_count = 0; //initialize path_hop_count with zero
	do //do-while-1
	{
/*
		if(IsNeighbor(X, Y, G, G_size) == TRUE)
		{
			//update the weight of edge <X,Y>
			Weight = Random_Path_Get_Weight(X, Y, G, G_size);
			Y_Ptr->weight = Weight;
		}
*/

		Coin = smpl_random(0, 1); //fair coin tossing

		if(Coin == 0) //coin head: random walk from src
		{
			//if(src == 6 && dst == 2)
			//	printf("here\n");

			Count = smpl_random(1, (int)G[X-1].weight);
			if(Count == 0)
				Count++;

			Z_Node = &G[X-1];
			while(Count > 0)
			{
				Z_Node = Z_Node->next;
				Count--;
			}

			Z = atoi(Z_Node->vertex);
			Weight = Random_Path_Get_Weight(X, Z, G, G_size);
			TotalDistance = Weight + LengthFromSrc + LengthFromDst + D[Z-1][Y-1];
			if((TotalDistance > Length) || (Z == Y))
			{
				Random_Path_List_Middle_Insert(Path_Head, X_Ptr, Y_Ptr, G, G_size, M, path_hop_count);
				break;
			}
			else
			{
				X = Z;
				X_Ptr = Random_Path_List_Forward_Insert(Path_Head, X_Ptr, Z, Weight);
                                LengthFromSrc += Weight;

				(*path_hop_count)++; //increment path_hop_count
			}
		}
		else //coin tail: random walk from dst
		{
			Count = smpl_random(1, (int)G[Y-1].weight);
			if(Count == 0) //smpl_random might return 0
			        Count++;
			
			Z_Node = &G[Y-1];
			while(Count > 0) //while-2
			{
				Z_Node = Z_Node->next;
				Count--;
			} //end of while-2

			Z = atoi(Z_Node->vertex);
			Weight = Random_Path_Get_Weight(Y, Z, G, G_size);
			TotalDistance = Weight + LengthFromSrc + LengthFromDst + D[Z-1][X-1];
			if((TotalDistance > Length) || (Z == X))
			{
				Random_Path_List_Middle_Insert(Path_Head, X_Ptr, Y_Ptr, G, G_size, M, path_hop_count);
				break;			
			}
			else
			{
				Y = Z;
				Y_Ptr = Random_Path_List_Backward_Insert(Path_Head, Y_Ptr, Z, Weight);
                                LengthFromDst += Weight;

				(*path_hop_count)++; //increment path_hop_count
			}
		} //end of else
	} while(1); //end of do-while-1

	//return Path_Head;

	/* make the new path list */
	*path_hop_count = 0; //reset path_hop_count to zero for a new path list where there is no protection point in the path except the destination
	new_path_list = Path_List_Init(); //initialize path_list creating a head node	
	path_ptr = Path_Head->next; //path_ptr points to the source vertex
	do
	{
	  Path_List_Add(new_path_list, path_ptr);
	  path_ptr = path_ptr->next;

	  (*path_hop_count)++; //increment path_hop_count
	} while(!IsProtectionPoint(path_ptr->vertex, protection_set));
	Path_List_Add(new_path_list, path_ptr); //add the protection point as the last vertex
	
	/* delete the original path list */
	Free_Path_List(Path_Head);

	return new_path_list;
}

struct_path_node* Random_Path_List_Init()
{ //initialize the list for a random path
	struct_path_node *head; //head of doubly linked list for a path including nodes

	head = (struct_path_node*) malloc(sizeof(struct_path_node));
	assert_memory(head); //assert the memory allocation
	memset(head->vertex, 0, sizeof(head->vertex));
	head->weight = 0;
	head->next = head;
	head->prev = head;
	return head;
}

struct_path_node* Random_Path_List_Backward_Insert(struct_path_node* h, struct_path_node *y, int u, double w)
{ //insert node u into path_list using pointer y towards the source with edge weight w
  //input:
  //  h: head of the random path
  //  y: current node towards the source
  //  u: a new node inserted
  //  w: edge weight between current_node and next_node
  //output: a pointer to a node newly inserted

	struct_path_node *path_node; //a new path node in the doubly linked list
	struct_path_node *path_list = y; //indicate the current node pointed by y
	
	path_node = (struct_path_node*) malloc(sizeof(struct_path_node));
	assert_memory(path_node); //assert the memory allocation
	
	//_itoa(u, path_node->vertex, 10);
	sprintf(path_node->vertex, "%d", u);	
	path_node->weight = 0;

	if(h != y) //y indicates a node different from the path list's head h whose weight indicates the number of nodes in the path
		path_list->weight = w; //update the weight of edge <u, y>
	
	path_node->prev = path_list->prev;
	path_list->prev->next = path_node;
	path_list->prev = path_node;
	path_node->next = path_list;

	h->weight++; //increase the number of nodes on the path

	return path_node;
}

struct_path_node* Random_Path_List_Forward_Insert(struct_path_node* h, struct_path_node *x, int u, double w)
{ //insert node u into path_list using pointer x towards the destination with edge weight w
  //input:
  //  h: head of the random path
  //  x: current node towards the destination
  //  u: a new node inserted
  //  w: edge weight between current_node and next_node
  //output: a pointer to a node newly inserted

	struct_path_node *path_node; //a new path node in the doubly linked list
	struct_path_node *path_list = x; //indicate the current node pointed by x
	
	path_node = (struct_path_node*) malloc(sizeof(struct_path_node));
	assert_memory(path_node); //assert the memory allocation
	
	//_itoa(u, path_node->vertex, 10);
	sprintf(path_node->vertex, "%d", u);	
	path_node->weight = w;
	
	path_node->next = path_list->next;
	path_list->next->prev = path_node;
	path_list->next = path_node;
	path_node->prev = path_list;

	h->weight++; //increase the number of nodes on the path

	return path_node;
}

struct_path_node* Random_Path_List_Middle_Insert(struct_path_node* h, struct_path_node *x, struct_path_node *y, struct_graph_node *G, int G_size, int** M, int *path_hop_count)
{ //complete the random path by inserting the intermediate nodes on the shortest path between x and y
  //input:
  //  h: head of the random path
  //  x: current node towards the destination
  //  y: current node towards the source
  //output: head of the random path

	int u, v; //node ID
	int i, j; //node index = node ID - 1
	double w; //edge weight

	i = atoi(x->vertex) - 1;
	j = atoi(y->vertex) - 1;

	if(i == j) //there is no node to insert in the middle of the path from x to y
		return h;
    
	do
	{
		u = j + 1;
		j = M[i][j]; //M[i][j] indicates the previous node towards node i

		if(j == i)
		{
			//update the weight of edge <u,v> 
			v = j + 1;
			u = atoi(y->vertex);
			w = Random_Path_Get_Weight(u, v, G, G_size);
			y->weight = w; //set edge (u,v)'s weight to w

			(*path_hop_count)++; //increment path_hop_count since the final edge (u,v) is added to the path

			break;
		}
		else
		{
			v = j + 1;	//v is the previous node of node j towards the source
			w = Random_Path_Get_Weight(u, v, G, G_size);
			y = Random_Path_List_Backward_Insert(h, y, v, w);

			(*path_hop_count)++; //increment path_hop_count since another node v is added to the path
		}
	} while(1);

	return h;
}

double Random_Path_Get_Weight(int u, int v, struct_graph_node *G, int G_size)
{ //get the weight of the edge between nodes u and v
	struct_graph_node *node;

	if(u > G_size)
	{
		printf("Random_Path_Get_Weight(): u (%d) > G_size (%d)\n", u, G_size);
		exit(1);
	}

	node = &G[u-1];
	while(atoi(node->vertex) != v)
	{
		node = node->next;
	}

	if(node == NULL)
		return -1;
	else
		return node->weight;
}

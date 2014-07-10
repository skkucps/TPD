/**
    File: all-pairs-shortest-paths.h
	Description: operations for the all-pairs shortest-paths using the Floyd-Warshall algorithm.
	Date: 11/13/2006	
	Maker: Jaehoon Jeong
*/

#ifndef __ALL_PAIRS_SHORTEST_PATHS_H__
#define __ALL_PAIRS_SHORTEST_PATHS_H__

#include "graph-data-struct.h"
#include "queue.h"
#include "schedule.h"

void Floyd_Warshall_Allocate_All_Matrices(int G_size, double ***Dr_move, int ***Mr_move, int *matrix_size_for_movement_in_Gr, double ***Dv_move, int ***Mv_move, int *matrix_size_for_movement_in_Gv, int ***Dv_scan, int ***Mv_scan, int *matrix_size_for_scanning_in_Gv, int ***Dr_breach, int ***Mr_breach, int *matrix_size_for_breach_in_Gr);
//allocate all the matrices for all-pairs shortest path in Gr and Gv

void Floyd_Warshall_Free_All_Matrices(double ***Dr_move, int ***Mr_move, int *matrix_size_for_movement_in_Gr, double ***Dv_move, int ***Mv_move, int *matrix_size_for_movement_in_Gv, int ***Dv_scan, int ***Mv_scan, int *matrix_size_for_scanning_in_Gv, int ***Dr_breach, int ***Mr_breach, int *matrix_size_for_breach_in_Gr);
//free all the matrices for all-pairs shortest path in Gr and Gv

double** Floyd_Warshall_Allocate_Matrix_Of_Type_Double(int size);
//allocate the memory of the matrix of type double whose rows and columns correspond to size

int** Floyd_Warshall_Allocate_Matrix_Of_Type_Int(int size);
//allocate the memory of the matrix of type int whose rows and columns correspond to size

double** Floyd_Warshall_Allocate_2D_Matrix_Of_Type_Double(int row_size, int column_size);
//allocate the memory of the 2-dimensional matrix of type double whose row_size is row size and column_size is column size

target_point_queue_node_t*** Floyd_Warshall_Allocate_2D_Matrix_Of_Type_TargetPointQueueNode_Pointer(int row_size, int column_size);
//allocate the memory of the 2-dimensional matrix of type target_point_queue_node_t* whose row_size is row size and column_size is column size

double** Floyd_Warshall_Reallocate_Matrix_Of_Type_Double(double **A, int old_size, int new_size);
//reallocate the memory of the matrix of type double whose rows and columns correspond to two times new_size

int** Floyd_Warshall_Reallocate_Matrix_Of_Type_Int(int **A, int old_size, int new_size);
//reallocate the memory of the matrix of type int whose rows and columns correspond to two times new_size

void Floyd_Warshall_Free_Matrix_Of_Type_Double(double **A, int size);
//free the memory for 2-dimensional martix A of type double

void Floyd_Warshall_Free_Matrix_Of_Type_Int(int **A, int size);
//free the memory for 2-dimensional martix A of type int

void Floyd_Warshall_Free_2D_Matrix_Of_Type_Double(double **A, int row_size, int column_size);
//free the memory for 2-dimensional martix A of type double whose row_size is row size and column_size is column size

void Floyd_Warshall_Free_2D_Matrix_Of_Type_TargetPointQueueNode_Pointer(target_point_queue_node_t ***A, int row_size, int column_size);
//free the memory for 2-dimensional martix A of type target_point_queue_node_t* whose row_size is row size and column_size is column size

int Floyd_Warshall_Allocate_Matrices_For_Movement(double ***D, int ***M, int matrix_size);
//allocate the memory of matrices for movement, i.e., the shortest path matrix D and predecessor matrix M

int Floyd_Warshall_Allocate_Matrices_For_Scanning(int ***D, int ***M, int matrix_size);
//allocate the memory of matrices for scanning, i.e., the shortest path matrix D, and predecessor matrix M

int Floyd_Warshall_Allocate_Matrices_For_Breach(int ***D, int ***M, int matrix_size);
//allocate the memory of matrices for breach path, i.e., the shortest path matrix D and predecessor matrix M

int Floyd_Warshall_Allocate_Matrices_For_EDD(double ***D, int ***M, double ***S, int matrix_size);
//allocate the memory of matrices for Expected Delivery Delay (EDD), i.e., the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S

int Floyd_Warshall_Allocate_Matrices_For_EDD_VAR(double ***D, int ***M, double ***S, int matrix_size);
//allocate the memory of matrices for Delivery Delay Variance (EDD_VAR), i.e., the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S

int Floyd_Warshall_Allocate_Matrices_For_EDC(double ***W, double ***D, int ***M, double ***S, int matrix_size);
//allocate the memory of matrices for Expected Delivery Cost (EDC), i.e., the adjacency matrix W, the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S

int Floyd_Warshall_Allocate_Matrices_For_Hop(int ***D, int ***M, int matrix_size);
//allocate the memory of matrices for hop number, i.e., the shortest path matrix D and predecessor matrix M

int Floyd_Warshall_Allocate_Matrices_For_Mcast(double ***T, double ***D, int ***M, double ***S, int matrix_size);
//allocate the memory of matrices for Multicast based on Expected Delivery Cost (EDC), i.e., the adjacency matrix T, the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S for delay cost variance


int Floyd_Warshall_Reallocate_Matrices_For_Movement(int G_size, double ***D, int ***M, int *matrix_size);
//reallocate the memory of matrices for movement, i.e., the shortest path matrix D and predecessor matrix M

int Floyd_Warshall_Reallocate_Matrices_For_Scanning(int G_size, int ***D, int ***M, int *matrix_size);
//reallocate the memory of matrices for scanning, i.e., the shortest path matrix D and predecessor matrix M

int Floyd_Warshall_Reallocate_Matrices_For_Breach(int G_size, int ***D, int ***M, int *matrix_size);
//reallocate the memory of matrices for breach path, i.e., the shortest path matrix D and predecessor matrix M

int Floyd_Warshall_Reallocate_Matrices_For_EDD(int G_size, double ***D, int ***M, double ***S, int *matrix_size);
//reallocate the memory of matrices for EDD, i.e., the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S

int Floyd_Warshall_Reallocate_Matrices_For_EDD_VAR(int G_size, double ***D, int ***M, double ***S, int *matrix_size);
//reallocate the memory of matrices for EDD_VAR, i.e., the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S

int Floyd_Warshall_Reallocate_Matrices_For_EDC(int G_size, double ***W, double ***D, int ***M, double ***S, int *matrix_size);
//reallocate the memory of matrices for EDC, i.e., the adjacency matrix W, the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S

int Floyd_Warshall_Reallocate_Matrices_For_Hop(int G_size, int ***D, int ***M, int *matrix_size);
//reallocate the memory of matrices for hop number, i.e., the shortest path matrix D and predecessor matrix M

int Floyd_Warshall_Reallocate_Matrices_For_Mcast(int G_size, double ***T, double ***D, int ***M, double ***S, int *matrix_size);
//reallocate the memory of matrices for Multicast based on EDC, i.e., the adjacency matrix T, the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S

int Floyd_Warshall_Free_Matrices_For_Movement(double ***D, int ***M, int *matrix_size);
//free the memory of matrices for movement, i.e., the shortest path matrix D and predecessor matrix M

int Floyd_Warshall_Free_Matrices_For_Scanning(int ***D, int ***M, int *matrix_size);
//free the memory of matrices for scanning, i.e., the shortest path matrix D and predecessor matrix M

int Floyd_Warshall_Free_Matrices_For_Breach(int ***D, int ***M, int *matrix_size);
//free the memory of matrices for breach path, i.e., the shortest path matrix D, and predecessor matrix M

int Floyd_Warshall_Free_Matrices_For_EDD(double ***D, int ***M, double ***S, int *matrix_size);
//free the memory of matrices for EDD, i.e., the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S

int Floyd_Warshall_Free_Matrices_For_EDD_VAR(double ***D, int ***M, double ***S, int *matrix_size);
//free the memory of matrices for EDD_VAR, i.e., the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S

int Floyd_Warshall_Free_Matrices_For_EDC(double ***W, double ***D, int ***M, double ***S, int *matrix_size);
//free the memory of matrices for EDC, i.e., the adjacency matrix, the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S

int Floyd_Warshall_Free_Matrices_For_Hop(int ***D, int ***M, int *matrix_size);
//free the memory of matrices for hop number, i.e., the shortest path matrix D and predecessor matrix M

int Floyd_Warshall_Free_Matrices_For_Mcast(double ***T, double ***D, int ***M, double ***S, int *matrix_size);
//free the memory of matrices for Multicast based on EDC, i.e., the adjacency matrix T, the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S

double** Floyd_Warshall_Make_Weight_Matrix_For_Movement(double **W, int W_size, struct_graph_node *G, int G_size);
//make the adjacency matrix W for movement with graph G

int** Floyd_Warshall_Make_Weight_Matrix_For_Scanning(int **W, int W_size, struct_graph_node *G, int G_size);
//make the adjacency matrix W for scanning with with sensor_list in schedule table T

int** Floyd_Warshall_Make_Weight_Matrix_For_Breach(int **W, int W_size, struct_graph_node *Gr, int Gr_size);
//make the adjacency matrix W for constructing breach path matrix with sensor_list in schedule table T

void Floyd_Warshall_Make_Weight_Matrix_For_EDD(double **D, double **S, int D_size, struct_graph_node *G, int G_size, parameter_t *param);
//make the adjacency matrices D and S for EDD with road graph G

void Floyd_Warshall_Make_Weight_Matrix_For_EDD_VAR(double **D, double **S, int D_size, struct_graph_node *G, int G_size, parameter_t *param);
//make the adjacency matrices D and S for EDD_VAR with road graph G

void Floyd_Warshall_Make_Weight_Matrix_For_EDC(double **W, double **D, double **S, int W_size, struct_graph_node *G, int G_size, parameter_t *param);
//make the adjacency matrices D and S for EDC with road graph G

int** Floyd_Warshall_Make_Weight_Matrix_For_Hop(int **W, int W_size, struct_graph_node *G, int G_size);
//make the adjacency matrix W for hop number with graph G

int Floyd_Warshall_Construct_Matrices_For_Movement(struct_graph_node *G, int G_size, double ***D, int ***M, int *matrix_size);
//construct the shortest path weight matrix D and predecessor matrix M in physical road network

int Floyd_Warshall_Construct_Matrices_For_Scanning(struct_graph_node *G, int G_size, int ***D, int ***M, int *matrix_size);
//construct the shortest path weight matrix D and predecessor matrix M in terms of the number of sensors for sensor scanning

int Floyd_Warshall_Construct_Matrices_For_Breach(struct_graph_node *G, int G_size, int ***D, int ***M, int *matrix_size);
//construct the shortest path weight matrix D and predecessor matrix M for breach path where no-live-sensor edge has 1 and live-sensor-edge has INF.

int Floyd_Warshall_Construct_Matrices_For_EDD(struct_graph_node *G, int G_size, double ***D, int ***M, double ***S, int *matrix_size, parameter_t *param);
//construct the shortest path weight matrix D and predecessor matrix M for EDD

int Floyd_Warshall_Construct_Matrices_For_EDD_VAR(struct_graph_node *G, int G_size, double ***D, int ***M, double ***S, int *matrix_size, parameter_t *param);
//construct the shortest path weight matrix D and predecessor matrix M for EDD_VAR

int Floyd_Warshall_Construct_Matrices_For_EDC(struct_graph_node *G, int G_size, double ***W, double ***D, int ***M, double ***S, int *matrix_size, parameter_t *param);
//construct the adjacency matrix W, the shortest path weight matrix D and predecessor matrix M for EDC

int Floyd_Warshall_Construct_Matrices_For_Hop(struct_graph_node *G, int G_size, int ***D, int ***M, int *matrix_size);
//construct the hop shortest path weight matrix D and predecessor matrix M in physical road network

int Floyd_Warshall_Construct_Shortest_PathInfo_For_Movement(double** D, int** M, int n);
//compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm for vehicle's movement

int Floyd_Warshall_Construct_Shortest_PathInfo_For_Scanning(int** D, int** M, int n);
//compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm for sensor scanning

int Floyd_Warshall_Construct_Shortest_PathInfo_For_Breach(int** D, int** M, int n);
//compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm for breach path

int Floyd_Warshall_Construct_Shortest_PathInfo_For_EDD(double** D, int** M, double** S, int n);
//compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm for EDD

int Floyd_Warshall_Construct_Shortest_PathInfo_For_EDD_VAR(double** D, int** M, double** S, int n);
//compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm for EDD_VAR

int Floyd_Warshall_Construct_Shortest_PathInfo_For_EDC(double** D, int** M, double** S, int n);
//compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm for EDC

int Floyd_Warshall_Construct_Shortest_PathInfo_For_Hop(int** D, int** M, int n);
//compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm for hop number

///////////////////////////////////////////////////////////////////////

void Floyd_Warshall_Get_Shortest_Path(int** M, int n, int src, int dst);
//void Floyd_Warshall_Get_Shortest_Path(int* M, int n, int src, int dst);
//get the shortest path from src to dst using Floyd_Warshall algorithm

path_queue_t* Floyd_Warshall_Make_Shortest_Path_Queue(int** M, int n, int src, int dst);
//make a queue containing the shortest path from dst to src using Floyd_Warshall algorithm

void Floyd_Warshall_Show_Weight_Table(double** A, int n);
//show the two-dimensional nxn weight matrix A

void Floyd_Warshall_Show_Predecessor_Table(int** A, int n);
//show the two-dimensional nxn predecessor matrix A

void Floyd_Warshall_Store_Weight_Table_Into_File(double** A, int n, char* filename);
//store the two-dimensional nxn weight matrix A into a file called filename

void Floyd_Warshall_Store_Predecessor_Table_Into_File(int** A, int n, char* filename);
//store the two-dimensional nxn predecessor matrix A into a file called filename

/** Operations for Trajectory-based Multi-Anycast (TMA) */
void Floyd_Warshall_Make_Weight_Matrix_For_EDC_With_PacketForwardingTreePathQueue(double **D, double **S, int matrix_size, packet_forwarding_tree_path_queue_t *PFTPQ);
/* make the adjacency matrix D for delivery delay and the supplementary matrix S for delivery cost with packet forwarding tree path queue PFTPQ */

void Floyd_Warshall_Make_Weight_Matrix_For_EDD_With_AdjacencyMatrix(double **D, double **S, int D_size, double **T, parameter_t *param);
  /* make the adjacency matrices D and S for EDD with adjacency matrix T
   * Note that we assume that the entry with a finite value (i.e., delivery cost) 
   * in T has a finite value (i.e., delivery delay and delivery delay variance) 
   * in both D and S */

void Floyd_Warshall_Make_Weight_Matrix_For_EDC_With_AdjacencyMatrix(double **D, double **S, int D_size, double **T, parameter_t *param);
  /* make the adjacency matrices D and S for EDC with adjacency matrix T
   * Note that we assume that the entry with a finite value (i.e., delivery cost) 
   * in T has a finite value (i.e., delivery cost and delivery cost variance) 
   * in both D and S */

void Floyd_Warshall_Copy_Matrix_Of_Type_Double(double **M_dst, double **M_src, int matrix_size);
/* copy the source matrix M_src into the destination matrix M_dst where the matrix data type is double and the matrix size is matrix_size */

void Floyd_Warshall_Copy_Matrix_Of_Type_Int(int **M_dst, int **M_src, int matrix_size);
/* copy the source matrix M_src into the destination matrix M_dst where the matrix data type is int and the matrix size is matrix_size */

double Floyd_Warshall_Sum_Matrix_Weights(double **W, int matrix_size);
/* sum the weights of the edges in matrix W of size matrix_size */

void Floyd_Warshall_Make_VirtualGraph_With_Graph_And_ComponentVertexQueues(struct_graph_node *G, int G_size, component_vertex_queue_t *CVQ_src, component_vertex_queue_t *CVQ_tp, double ***A, int A_size);
/* make virtual graph's adjacency matrix A with road network graph G where CVQ_src is the component vertex queue for source node of a superedge and CVQ_tp is the component vertex queue for target point of the superedge */

int** Floyd_Warshall_Make_Connectivity_Adjacency_Matrix_For_Movement(int **W, int W_size, struct_graph_node *G, int G_size);
/* make the connectivity adjacency matrix W for movement with graph G */

int** Floyd_Warshall_Make_Connectivity_Adjacency_Matrix_For_EDD(int **W, int W_size, struct_graph_node *G, int G_size);
/* make the connectivity adjacency matrix W for EDD with graph G */

int** Floyd_Warshall_Init_Connectivity_Adjacency_Matrix_For_EDD(parameter_t *param, struct_graph_node *G, int G_size, struct_traffic_table *sn_table, int ***A, int A_size);
/* initialize the connectivity adjacency matrix A for road network graph G with stationary node list sn_table */

#endif

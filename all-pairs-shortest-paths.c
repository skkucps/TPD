/**
 *  File: all-pairs-shortest-paths.c
	Description: operations for the all-pairs shortest-paths using the Floyd-Warshall algorithm.
	Date: 11/13/2006	
	Maker: Jaehoon Jeong
*/

#include "stdafx.h"
#include "all-pairs-shortest-paths.h"
#include "util.h" //assert_memory()
#include "vadd.h" //VADD_Compute_Edge_Delay()
#include "shortest-path.h" //Update_LinkDelay_Information()

///////////////////////////////////////////////////////////////////
void Floyd_Warshall_Allocate_All_Matrices(int G_size, double ***Dr_move, int ***Mr_move, int *matrix_size_for_movement_in_Gr, double ***Dv_move, int ***Mv_move, int *matrix_size_for_movement_in_Gv, int ***Dv_scan, int ***Mv_scan, int *matrix_size_for_scanning_in_Gv, int ***Dv_breach, int ***Mv_breach, int *matrix_size_for_breach_in_Gv)
{ //allocate all the matrices for all-pairs shortest path in Gr and Gv

  /** check whether graph information is valid or not */
  if(G_size < 0)
  {
    printf("Floyd_Warshall_Allocate_All_Matrices(): G_size(%d) < 0\n", G_size);
    exit(1);
  }

  /** check whether at least one of the matrices have already been allocated or not */
  if(*Dr_move != NULL || *Mr_move != NULL || *Dv_move != NULL || *Mv_move != NULL || *Dv_scan != NULL || *Mv_scan != NULL || *Dv_breach != NULL || *Mv_breach != NULL )
  {
    printf("Floyd_Warshall_Reallocate_Matrices_For_Movement(): we cannot reallocate the matrices since at one of matrices has already been allocated memory!\n");
    exit(1);
  }

  /* allocate the memory of matrices for movement, i.e., shortest path matrix Dr_move, and predecessor matrix Mr_move */
  *matrix_size_for_movement_in_Gr = G_size;
  Floyd_Warshall_Allocate_Matrices_For_Movement(Dr_move, Mr_move, *matrix_size_for_movement_in_Gr);

  /* allocate the memory of matrices for movement, i.e., shortest path matrix Dv_move, and predecessor matrix Mv_move */
  *matrix_size_for_movement_in_Gv = G_size * MATRIX_SIZE_MULTIPLIER;
  Floyd_Warshall_Allocate_Matrices_For_Movement(Dv_move, Mv_move, *matrix_size_for_movement_in_Gv);

  /* allocate the memory of matrices for scanning, i.e., shortest path matrix Dv_scan, and predecessor matrix Mv_scan */
  *matrix_size_for_scanning_in_Gv = G_size * MATRIX_SIZE_MULTIPLIER;
  Floyd_Warshall_Allocate_Matrices_For_Scanning(Dv_scan, Mv_scan, *matrix_size_for_scanning_in_Gv);

  /* allocate the memory of matrices for breach path, i.e., shortest path matrix Dv_breach, and predecessor matrix Mv_breach */
  *matrix_size_for_breach_in_Gv = G_size;
  Floyd_Warshall_Allocate_Matrices_For_Breach(Dv_breach, Mv_breach, *matrix_size_for_breach_in_Gv);
}

void Floyd_Warshall_Free_All_Matrices(double ***Dr_move, int ***Mr_move, int *matrix_size_for_movement_in_Gr, double ***Dv_move, int ***Mv_move, int *matrix_size_for_movement_in_Gv, int ***Dv_scan, int ***Mv_scan, int *matrix_size_for_scanning_in_Gv, int ***Dv_breach, int ***Mv_breach, int *matrix_size_for_breach_in_Gv)
{ //free all the matrices for all-pairs shortest path in Gr and Gv

  /* free matrices for Gr */
  Floyd_Warshall_Free_Matrices_For_Movement(Dr_move, Mr_move, matrix_size_for_movement_in_Gr);

  /* free matrices for Gv */
  Floyd_Warshall_Free_Matrices_For_Movement(Dv_move, Mv_move, matrix_size_for_movement_in_Gv);
 
  Floyd_Warshall_Free_Matrices_For_Scanning(Dv_scan, Mv_scan, matrix_size_for_scanning_in_Gv);

  Floyd_Warshall_Free_Matrices_For_Breach(Dv_breach, Mv_breach, matrix_size_for_breach_in_Gv);
}

///////////////////////////////////////////////////////////////////

double** Floyd_Warshall_Allocate_Matrix_Of_Type_Double(int size)
{ //allocate the memory of the matrix of type double whose rows and columns correspond to size
	double **A = NULL; //matrix
	int i;

	A = (double**)calloc(size, sizeof(double*)); //all of entries are set to zero
	assert_memory(A);

	for(i = 0; i < size; i++)
	{
		A[i] = (double*)calloc(size, sizeof(double));
		assert_memory(A[i]);
	}

	return A;
}

int** Floyd_Warshall_Allocate_Matrix_Of_Type_Int(int size)
{ //allocate the memory of the matrix of type int whose rows and columns correspond to size
	int **A = NULL; //matrix
	int i;

	A = (int**)calloc(size, sizeof(int*)); //all of entries are set to zero
	assert_memory(A);

	for(i = 0; i < size; i++)
	{
		A[i] = (int*)calloc(size, sizeof(int));
		assert_memory(A[i]);
	}

	return A;
}

double** Floyd_Warshall_Allocate_2D_Matrix_Of_Type_Double(int row_size, int column_size)
{ //allocate the memory of the 2-dimensional matrix of type double whose row_size is row size and column_size is column size
	double **A = NULL; //matrix
	int i;

	A = (double**)calloc(row_size, sizeof(double*)); //all of entries are set to zero
	assert_memory(A);

	for(i = 0; i < row_size; i++)
	{
		A[i] = (double*)calloc(column_size, sizeof(double));
		assert_memory(A[i]);
	}

	return A;
}

target_point_queue_node_t*** Floyd_Warshall_Allocate_2D_Matrix_Of_Type_TargetPointQueueNode_Pointer(int row_size, int column_size)
{ //allocate the memory of the 2-dimensional matrix of type target_point_queue_node_t* whose row_size is row size and column_size is column size
	target_point_queue_node_t ***A = NULL; //matrix
	int i;

	A = (target_point_queue_node_t***)calloc(row_size, sizeof(target_point_queue_node_t**)); //all of entries are set to zero
	assert_memory(A);

	for(i = 0; i < row_size; i++)
	{
		A[i] = (target_point_queue_node_t**)calloc(column_size, sizeof(target_point_queue_node_t*));
		assert_memory(A[i]);
	}

	return A;
}

double** Floyd_Warshall_Reallocate_Matrix_Of_Type_Double(double **A, int old_size, int new_size)
{ //reallocate the memory of the matrix of type double whose rows and columns correspond to two times new_size

	if(A != NULL)
	{
	  Floyd_Warshall_Free_Matrix_Of_Type_Double(A, old_size);
	}
  
	/* allocate the memory for matrix A with new_size */
	A = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(new_size);

	return A;
}

int** Floyd_Warshall_Reallocate_Matrix_Of_Type_Int(int **A, int old_size, int new_size)
{ //reallocate the memory of the matrix of type int whose rows and columns correspond to two times new_size

	if(A != NULL)
	{
	  Floyd_Warshall_Free_Matrix_Of_Type_Int(A, old_size);
	}
  
	/* allocate the memory for matrix A with new_size */
	A = Floyd_Warshall_Allocate_Matrix_Of_Type_Int(new_size);

	return A;
}

void Floyd_Warshall_Free_Matrix_Of_Type_Double(double **A, int size)
{ //free the memory for 2-dimensional martix A of type double
	int i;
	
	for(i = 0; i < size; i++)
		free(A[i]);

	free(A);
}

void Floyd_Warshall_Free_Matrix_Of_Type_Int(int **A, int size)
{ //free the memory for 2-dimensional martix A of type int
	int i;
	
	for(i = 0; i < size; i++)
		free(A[i]);

	free(A);
}

void Floyd_Warshall_Free_2D_Matrix_Of_Type_Double(double **A, int row_size, int column_size)
{ //free the memory for 2-dimensional martix A of type double whose row_size is row size and column_size is column size
	int i;
	
	for(i = 0; i < row_size; i++)
		free(A[i]);

	free(A);
}

void Floyd_Warshall_Free_2D_Matrix_Of_Type_TargetPointQueueNode_Pointer(target_point_queue_node_t ***A, int row_size, int column_size)
{ //free the memory for 2-dimensional martix A of type target_point_queue_node_t* whose row_size is row size and column_size is column size
	int i;
	
	for(i = 0; i < row_size; i++)
		free(A[i]);

	free(A);
}

///////////////////////////////////////////////////////////////////

int Floyd_Warshall_Allocate_Matrices_For_Movement(double ***D, int ***M, int matrix_size)
{ //allocate the memory of matrices for movement, i.e., the shortest path matrix D and predecessor matrix M

  if(*D != NULL || *M != NULL)
  {
    printf("Floyd_Warshall_Allocate_Matrices_For_Movement(): we cannot allocate the matrices for movement since one of matrices D and M have already been allocated memory!\n");
    exit(1);
  }

  *D = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(matrix_size);
  //allocate the memory of the shortest path matrix for movement whose rows and columns correspond to matrix_size

  *M = Floyd_Warshall_Allocate_Matrix_Of_Type_Int(matrix_size);
  //allocate the memory of the predecessor matrix for movement whose rows and columns correspond to matrix_size  

  return 0;
}

int Floyd_Warshall_Allocate_Matrices_For_Scanning(int ***D, int ***M, int matrix_size)
{ //allocate the memory of matrices for scanning, i.e., the shortest path matrix D and predecessor matrix M

  if(*D != NULL || *M != NULL)
  {
    printf("Floyd_Warshall_Allocate_Matrices_For_Scanning(): we cannot allocate the matrices for scanning since one of matrices D and M have already been allocated memory!\n");
    exit(1);
  }

  *D = Floyd_Warshall_Allocate_Matrix_Of_Type_Int(matrix_size);
  //allocate the memory of the shortest path matrix for scanning whose rows and columns correspond to matrix_size

  *M = Floyd_Warshall_Allocate_Matrix_Of_Type_Int(matrix_size);
  //allocate the memory of the predecessor matrix for scanning whose rows and columns correspond to matrix_size  

  return 0;
}

int Floyd_Warshall_Allocate_Matrices_For_Breach(int ***D, int ***M, int matrix_size)
{ //allocate the memory of matrices for breach path, i.e., the shortest path matrix D and predecessor matrix M

  if(*D != NULL || *M != NULL)
  {
    printf("Floyd_Warshall_Allocate_Matrices_For_Breach(): we cannot allocate the matrices for breach path since one of matrices D and M have already been allocated memory!\n");
    exit(1);
  }

  *D = Floyd_Warshall_Allocate_Matrix_Of_Type_Int(matrix_size);
  //allocate the memory of the shortest path matrix for breach path whose rows and columns correspond to matrix_size

  *M = Floyd_Warshall_Allocate_Matrix_Of_Type_Int(matrix_size);
  //allocate the memory of the predecessor matrix for breach path whose rows and columns correspond to matrix_size  

  return 0;
}

int Floyd_Warshall_Allocate_Matrices_For_EDD(double ***D, int ***M, double ***S, int matrix_size)
{ //allocate the memory of matrices for Expected Delivery Delay (EDD), i.e., the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S

  if(*D != NULL || *M != NULL || *S != NULL)
  {
    printf("Floyd_Warshall_Allocate_Matrices_For_EDD(): we cannot allocate the matrices for EDD since one of matrices D, M, and S have already been allocated memory!\n");
    exit(1);
  }

  *D = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(matrix_size);
  //allocate the memory of the shortest path matrix for EDD whose rows and columns correspond to matrix_size

  *M = Floyd_Warshall_Allocate_Matrix_Of_Type_Int(matrix_size);
  //allocate the memory of the predecessor matrix for EDD whose rows and columns correspond to matrix_size  

  *S = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(matrix_size);
  //allocate the memory of the matrix of the delay variance for the shortest path whose rows and columns correspond to matrix_size

  return 0;
}

int Floyd_Warshall_Allocate_Matrices_For_EDD_VAR(double ***D, int ***M, double ***S, int matrix_size)
{ //allocate the memory of matrices for Delivery Delay Variance (EDD_VAR), i.e., the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S

  if(*D != NULL || *M != NULL || *S != NULL)
  {
    printf("Floyd_Warshall_Allocate_Matrices_For_EDD_VAR(): we cannot allocate the matrices for EDD_VAR since one of matrices D, M, and S have already been allocated memory!\n");
    exit(1);
  }

  *D = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(matrix_size);
  //allocate the memory of the shortest path matrix for EDD_VAR whose rows and columns correspond to matrix_size

  *M = Floyd_Warshall_Allocate_Matrix_Of_Type_Int(matrix_size);
  //allocate the memory of the predecessor matrix for EDD_VAR whose rows and columns correspond to matrix_size  

  *S = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(matrix_size);
  //allocate the memory of the matrix of the delay variance for the shortest path whose rows and columns correspond to matrix_size

  return 0;
}

int Floyd_Warshall_Allocate_Matrices_For_EDC(double ***W, double ***D, int ***M, double ***S, int matrix_size)
{ //allocate the memory of matrices for Expected Delivery Cost (EDC), i.e., the adjacency matrix W, the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S

  if(*W != NULL || *D != NULL || *M != NULL || *S != NULL)
  {
    printf("Floyd_Warshall_Allocate_Matrices_For_EDC(): we cannot allocate the matrices for EDC since one of matrices W, D, M, and S have already been allocated memory!\n");
    exit(1);
  }

  *W = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(matrix_size);
  //allocate the memory of the adjacency matrix for EDC whose rows and columns correspond to matrix_size

  *D = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(matrix_size);
  //allocate the memory of the shortest path matrix for EDC whose rows and columns correspond to matrix_size

  *M = Floyd_Warshall_Allocate_Matrix_Of_Type_Int(matrix_size);
  //allocate the memory of the predecessor matrix for EDC whose rows and columns correspond to matrix_size  

  *S = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(matrix_size);
  //allocate the memory of the matrix of the cost variance for the shortest path whose rows and columns correspond to matrix_size

  return 0;
}

int Floyd_Warshall_Allocate_Matrices_For_Hop(int ***D, int ***M, int matrix_size)
{ //allocate the memory of matrices for hop number, i.e., the shortest path matrix D and predecessor matrix M

  if(*D != NULL || *M != NULL)
  {
    printf("Floyd_Warshall_Allocate_Matrices_For_Hop(): we cannot allocate the matrices for hop number since one of matrices D and M have already been allocated memory!\n");
    exit(1);
  }

  *D = Floyd_Warshall_Allocate_Matrix_Of_Type_Int(matrix_size);
  //allocate the memory of the shortest path matrix for movement whose rows and columns correspond to matrix_size

  *M = Floyd_Warshall_Allocate_Matrix_Of_Type_Int(matrix_size);
  //allocate the memory of the predecessor matrix for movement whose rows and columns correspond to matrix_size  

  return 0;
}

int Floyd_Warshall_Allocate_Matrices_For_Mcast(double ***T, double ***D, int ***M, double ***S, int matrix_size)
{ //allocate the memory of matrices for Multicast based on Expected Delivery Cost (EDC), i.e., the adjacency matrix T, the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S for delay cost variance

	if(*T != NULL || *D != NULL || *M != NULL || *S != NULL)
	{
		printf("Floyd_Warshall_Allocate_Matrices_For_Mcast(): we cannot allocate the matrices for <ulticast based on EDC since one of matrices T, D, M, and S have already been allocated memory!\n");
		exit(1);
	}

	*T = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(matrix_size);
	//allocate the memory of the adjacency matrix for Multicast whose rows and columns correspond to matrix_size

	*D = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(matrix_size);
	//allocate the memory of the shortest path matrix for Multicast whose rows and columns correspond to matrix_size

	*M = Floyd_Warshall_Allocate_Matrix_Of_Type_Int(matrix_size);
	//allocate the memory of the predecessor matrix for Multicast whose rows and columns correspond to matrix_size  

	*S = Floyd_Warshall_Allocate_Matrix_Of_Type_Double(matrix_size);
	//allocate the memory of the matrix of the cost variance for the shortest path whose rows and columns correspond to matrix_size

	return 0;
}

int Floyd_Warshall_Reallocate_Matrices_For_Movement(int G_size, double ***D, int ***M, int *matrix_size)
{ //reallocate the memory of matrices for movement, i.e., the shortest path matrix D and predecessor matrix M
  int new_matrix_size = *matrix_size + G_size; //new matrix size

  if(*D == NULL || *M == NULL)
  {
    printf("Floyd_Warshall_Reallocate_Matrices_For_Movement(): we cannot reallocate the matrices for movement since one of matrices D and M have not been allocated memory yet!\n");
    exit(1);
  }

  *D = Floyd_Warshall_Reallocate_Matrix_Of_Type_Double(*D, *matrix_size, new_matrix_size);
  //allocate the memory of the shortest path matrix for movement whose rows and columns correspond to matrix_size

  *M = Floyd_Warshall_Reallocate_Matrix_Of_Type_Int(*M, *matrix_size, new_matrix_size);
  //allocate the memory of the predecessor matrix for movement whose rows and columns correspond to matrix_size  

  *matrix_size = new_matrix_size;

  return 0;
}

int Floyd_Warshall_Reallocate_Matrices_For_Scanning(int G_size, int ***D, int ***M, int *matrix_size)
{ //reallocate the memory of matrices for scanning, i.e., the shortest path matrix D and predecessor matrix M
  int new_matrix_size = *matrix_size + G_size; //new matrix size

  if(*D == NULL || *M == NULL)
  {
    printf("Floyd_Warshall_Reallocate_Matrices_For_Scanning(): we cannot reallocate the matrices for scanning since one of matrices D and M have not been allocated memory yet!\n");
    exit(1);
  }

  *D = Floyd_Warshall_Reallocate_Matrix_Of_Type_Int(*D, *matrix_size, new_matrix_size);
  //allocate the memory of the shortest path matrix for scanning whose rows and columns correspond to matrix_size

  *M = Floyd_Warshall_Reallocate_Matrix_Of_Type_Int(*M, *matrix_size, new_matrix_size);
  //allocate the memory of the predecessor matrix for scanning whose rows and columns correspond to matrix_size  

  *matrix_size = new_matrix_size;

  return 0;
}

int Floyd_Warshall_Reallocate_Matrices_For_Breach(int G_size, int ***D, int ***M, int *matrix_size)
{ //reallocate the memory of matrices for breach path, i.e., the shortest path matrix D and predecessor matrix M
  int new_matrix_size = *matrix_size + G_size; //new matrix size

  if(*D == NULL || *M == NULL)
  {
    printf("Floyd_Warshall_Reallocate_Matrices_For_Breach(): we cannot reallocate the matrices for breach path since one of matrices D and M have not been allocated memory yet!\n");
    exit(1);
  }

  *D = Floyd_Warshall_Reallocate_Matrix_Of_Type_Int(*D, *matrix_size, new_matrix_size);
  //allocate the memory of the shortest path matrix for breach path whose rows and columns correspond to matrix_size

  *M = Floyd_Warshall_Reallocate_Matrix_Of_Type_Int(*M, *matrix_size, new_matrix_size);
  //allocate the memory of the predecessor matrix for breach path whose rows and columns correspond to matrix_size  

  *matrix_size = new_matrix_size;

  return 0;
}

int Floyd_Warshall_Reallocate_Matrices_For_EDD(int G_size, double ***D, int ***M, double ***S, int *matrix_size)
{ //reallocate the memory of matrices for EDD, i.e., the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S
  int new_matrix_size = *matrix_size + G_size; //new matrix size

  if(*D == NULL || *M == NULL || *S == NULL)
  {
    printf("Floyd_Warshall_Reallocate_Matrices_For_EDD(): we cannot reallocate the matrices for EDD since one of matrices D, M, and S have not been allocated memory yet!\n");
    exit(1);
  }

  *D = Floyd_Warshall_Reallocate_Matrix_Of_Type_Double(*D, *matrix_size, new_matrix_size);
  //allocate the memory of the shortest path matrix for EDD whose rows and columns correspond to matrix_size

  *M = Floyd_Warshall_Reallocate_Matrix_Of_Type_Int(*M, *matrix_size, new_matrix_size);
  //allocate the memory of the predecessor matrix for EDD whose rows and columns correspond to matrix_size  

  *S = Floyd_Warshall_Reallocate_Matrix_Of_Type_Double(*S, *matrix_size, new_matrix_size);
  //allocate the memory of the matrix of the delay variance for the shortest path whose rows and columns correspond to matrix_size

  *matrix_size = new_matrix_size;

  return 0;
}

int Floyd_Warshall_Reallocate_Matrices_For_EDD_VAR(int G_size, double ***D, int ***M, double ***S, int *matrix_size)
{ //reallocate the memory of matrices for EDD_VAR, i.e., the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S
  int new_matrix_size = *matrix_size + G_size; //new matrix size

  if(*D == NULL || *M == NULL || *S == NULL)
  {
    printf("Floyd_Warshall_Reallocate_Matrices_For_EDD_VAR(): we cannot reallocate the matrices for EDD_VAR since one of matrices D, M, and S have not been allocated memory yet!\n");
    exit(1);
  }

  *D = Floyd_Warshall_Reallocate_Matrix_Of_Type_Double(*D, *matrix_size, new_matrix_size);
  //allocate the memory of the shortest path matrix for EDD_VAR whose rows and columns correspond to matrix_size

  *M = Floyd_Warshall_Reallocate_Matrix_Of_Type_Int(*M, *matrix_size, new_matrix_size);
  //allocate the memory of the predecessor matrix for EDD_VAR whose rows and columns correspond to matrix_size  

  *S = Floyd_Warshall_Reallocate_Matrix_Of_Type_Double(*S, *matrix_size, new_matrix_size);
  //allocate the memory of the matrix of the delay variance for the shortest path whose rows and columns correspond to matrix_size

  *matrix_size = new_matrix_size;

  return 0;
}

int Floyd_Warshall_Reallocate_Matrices_For_EDC(int G_size, double ***W, double ***D, int ***M, double ***S, int *matrix_size)
{ //reallocate the memory of matrices for EDC, i.e., the adjacency matrix W, the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S
  int new_matrix_size = *matrix_size + G_size; //new matrix size

  if(*W == NULL || *D == NULL || *M == NULL || *S == NULL)
  {
    printf("Floyd_Warshall_Reallocate_Matrices_For_EDC(): we cannot reallocate the matrices for EDC since one of matrices W, D, M, and S have not been allocated memory yet!\n");
    exit(1);
  }

  *W = Floyd_Warshall_Reallocate_Matrix_Of_Type_Double(*W, *matrix_size, new_matrix_size);
  //allocate the memory of the adjacency matrix for EDC whose rows and columns correspond to matrix_size

  *D = Floyd_Warshall_Reallocate_Matrix_Of_Type_Double(*D, *matrix_size, new_matrix_size);
  //allocate the memory of the shortest path matrix for EDC whose rows and columns correspond to matrix_size

  *M = Floyd_Warshall_Reallocate_Matrix_Of_Type_Int(*M, *matrix_size, new_matrix_size);
  //allocate the memory of the predecessor matrix for EDC whose rows and columns correspond to matrix_size  

  *S = Floyd_Warshall_Reallocate_Matrix_Of_Type_Double(*S, *matrix_size, new_matrix_size);
  //allocate the memory of the matrix of the cost variance for the shortest path whose rows and columns correspond to matrix_size

  *matrix_size = new_matrix_size;

  return 0;
}

int Floyd_Warshall_Reallocate_Matrices_For_Hop(int G_size, int ***D, int ***M, int *matrix_size)
{ //reallocate the memory of matrices for hop number, i.e., the shortest path matrix D and predecessor matrix M
  int new_matrix_size = *matrix_size + G_size; //new matrix size

  if(*D == NULL || *M == NULL)
  {
    printf("Floyd_Warshall_Reallocate_Matrices_For_Hop(): we cannot reallocate the matrices for hop number since one of matrices D and M have not been allocated memory yet!\n");
    exit(1);
  }

  *D = Floyd_Warshall_Reallocate_Matrix_Of_Type_Int(*D, *matrix_size, new_matrix_size);
  //allocate the memory of the shortest path matrix for movement whose rows and columns correspond to matrix_size

  *M = Floyd_Warshall_Reallocate_Matrix_Of_Type_Int(*M, *matrix_size, new_matrix_size);
  //allocate the memory of the predecessor matrix for movement whose rows and columns correspond to matrix_size  

  *matrix_size = new_matrix_size;

  return 0;
}

int Floyd_Warshall_Reallocate_Matrices_For_Mcast(int G_size, double ***T, double ***D, int ***M, double ***S, int *matrix_size)
{ //reallocate the memory of matrices for Multicast based on EDC, i.e., the adjacency matrix T, the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S
  int new_matrix_size = *matrix_size + G_size; //new matrix size

  if(*T == NULL || *D == NULL || *M == NULL || *S == NULL)
  {
    printf("Floyd_Warshall_Reallocate_Matrices_For_EDC(): we cannot reallocate the matrices for Multicast since one of matrices T, D, M, and S have not been allocated memory yet!\n");
    exit(1);
  }

  *T = Floyd_Warshall_Reallocate_Matrix_Of_Type_Double(*T, *matrix_size, new_matrix_size);
  //allocate the memory of the adjacency matrix for Multicast whose rows and columns correspond to matrix_size

  *D = Floyd_Warshall_Reallocate_Matrix_Of_Type_Double(*D, *matrix_size, new_matrix_size);
  //allocate the memory of the shortest path matrix for Multicast whose rows and columns correspond to matrix_size

  *M = Floyd_Warshall_Reallocate_Matrix_Of_Type_Int(*M, *matrix_size, new_matrix_size);
  //allocate the memory of the predecessor matrix for Multicast whose rows and columns correspond to matrix_size  

  *S = Floyd_Warshall_Reallocate_Matrix_Of_Type_Double(*S, *matrix_size, new_matrix_size);
  //allocate the memory of the matrix of the cost variance for the shortest path whose rows and columns correspond to matrix_size

  *matrix_size = new_matrix_size;

  return 0;
}

int Floyd_Warshall_Free_Matrices_For_Movement(double ***D, int ***M, int *matrix_size)
{ //free the memory of matrices for movement, i.e., the shortest path matrix D and predecessor matrix M

  if(*D == NULL || *M == NULL)
  {
    printf("Floyd_Warshall_Free_Matrices_For_Movement(): we cannot free the matrices for movement since one of matrices D and M have not been allocated memory yet!\n");
    exit(1);
  }

  Floyd_Warshall_Free_Matrix_Of_Type_Double(*D, *matrix_size);
  //allocate the memory of the shortest path matrix for movement whose rows and columns correspond to matrix_size

  Floyd_Warshall_Free_Matrix_Of_Type_Int(*M, *matrix_size);
  //allocate the memory of the predecessor matrix for movement whose rows and columns correspond to matrix_size  

  *D = NULL; //set matrix pointer to NULL
  *M = NULL; //set matrix pointer to NULL
  *matrix_size = 0; //set matrix size to zero

  return 0;
}

int Floyd_Warshall_Free_Matrices_For_Scanning(int ***D, int ***M, int *matrix_size)
{ //free the memory of matrices for scanning, i.e., the shortest path matrix D, and predecessor matrix M

  if(*D == NULL || *M == NULL)
  {
    printf("Floyd_Warshall_Free_Matrices_For_Scanning(): we cannot free the matrices for scanning since one of matrics D and M have not been allocated memory yet!\n");
    exit(1);
  }

  Floyd_Warshall_Free_Matrix_Of_Type_Int(*D, *matrix_size);
  //allocate the memory of the shortest path matrix for scanning whose rows and columns correspond to matrix_size

  Floyd_Warshall_Free_Matrix_Of_Type_Int(*M, *matrix_size);
  //allocate the memory of the predecessor matrix for scanning whose rows and columns correspond to matrix_size  

  *D = *M = NULL; //set matrix pointers to NULL
  *matrix_size = 0; //set matrix size to zero

  return 0;
}

int Floyd_Warshall_Free_Matrices_For_Breach(int ***D, int ***M, int *matrix_size)
{ //free the memory of matrices for breach path, i.e., the shortest path matrix D, and predecessor matrix M

  if(*D == NULL || *M == NULL)
  {
    printf("Floyd_Warshall_Free_Matrices_For_Breach(): we cannot free the matrices for breach path since one of matrics D and M have not been allocated memory yet!\n");
    exit(1);
  }

  Floyd_Warshall_Free_Matrix_Of_Type_Int(*D, *matrix_size);
  //allocate the memory of the shortest path matrix for breach path whose rows and columns correspond to matrix_size

  Floyd_Warshall_Free_Matrix_Of_Type_Int(*M, *matrix_size);
  //allocate the memory of the predecessor matrix for breach path whose rows and columns correspond to matrix_size  

  *D = *M = NULL; //set matrix pointers to NULL
  *matrix_size = 0; //set matrix size to zero

  return 0;
}

int Floyd_Warshall_Free_Matrices_For_EDD(double ***D, int ***M, double ***S, int *matrix_size)
{ //free the memory of matrices for EDD, i.e., the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S

  if(*D == NULL || *M == NULL || *S == NULL)
  {
    printf("Floyd_Warshall_Free_Matrices_For_EDD(): we cannot free the matrices for EDD since one of matrices D, M, and S have not been allocated memory yet!\n");
    exit(1);
  }

  Floyd_Warshall_Free_Matrix_Of_Type_Double(*D, *matrix_size);
  //allocate the memory of the shortest path matrix for EDD whose rows and columns correspond to matrix_size

  Floyd_Warshall_Free_Matrix_Of_Type_Int(*M, *matrix_size);
  //allocate the memory of the predecessor matrix for EDD whose rows and columns correspond to matrix_size  

  Floyd_Warshall_Free_Matrix_Of_Type_Double(*S, *matrix_size);
  //allocate the memory of the matrix of the delay variance for the shortest path whose rows and columns correspond to matrix_size

  *D = NULL; //set matrix pointer to NULL
  *M = NULL; //set matrix pointer to NULL
  *S = NULL; //set matrix pointer to NULL
  *matrix_size = 0; //set matrix size to zero

  return 0;
}

int Floyd_Warshall_Free_Matrices_For_EDD_VAR(double ***D, int ***M, double ***S, int *matrix_size)
{ //free the memory of matrices for EDD_VAR, i.e., the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S 

  if(*D == NULL || *M == NULL || *S == NULL)
  {
    printf("Floyd_Warshall_Free_Matrices_For_EDD_VAR(): we cannot free the matrices for EDD_VAR since one of matrices D, M, and S have not been allocated memory yet!\n");
    exit(1);
  }

  Floyd_Warshall_Free_Matrix_Of_Type_Double(*D, *matrix_size);
  //allocate the memory of the shortest path matrix for EDD_VAR whose rows and columns correspond to matrix_size

  Floyd_Warshall_Free_Matrix_Of_Type_Int(*M, *matrix_size);
  //allocate the memory of the predecessor matrix for EDD_VAR whose rows and columns correspond to matrix_size  

  Floyd_Warshall_Free_Matrix_Of_Type_Double(*S, *matrix_size);
  //allocate the memory of the matrix of the delay variance for the shortest path whose rows and columns correspond to matrix_size

  *D = NULL; //set matrix pointer to NULL
  *M = NULL; //set matrix pointer to NULL
  *S = NULL; //set matrix pointer to NULL
  *matrix_size = 0; //set matrix size to zero

  return 0;
}

int Floyd_Warshall_Free_Matrices_For_EDC(double ***W, double ***D, int ***M, double ***S, int *matrix_size)
{ //free the memory of matrices for EDC, i.e., the adjacency matrix W, the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S

  if(*W == NULL || *D == NULL || *M == NULL || *S == NULL)
  {
    printf("Floyd_Warshall_Free_Matrices_For_EDC(): we cannot free the matrices for EDC since one of matrices W, D, M, and S have not been allocated memory yet!\n");
    exit(1);
  }

  Floyd_Warshall_Free_Matrix_Of_Type_Double(*W, *matrix_size);
  //allocate the memory of the adjacency matrix for EDC whose rows and columns correspond to matrix_size

  Floyd_Warshall_Free_Matrix_Of_Type_Double(*D, *matrix_size);
  //allocate the memory of the shortest path matrix for EDC whose rows and columns correspond to matrix_size

  Floyd_Warshall_Free_Matrix_Of_Type_Int(*M, *matrix_size);
  //allocate the memory of the predecessor matrix for EDC whose rows and columns correspond to matrix_size  

  Floyd_Warshall_Free_Matrix_Of_Type_Double(*S, *matrix_size);
  //allocate the memory of the matrix of the cost variance for the shortest path whose rows and columns correspond to matrix_size

  *W = NULL; //set matrix pointer to NULL
  *D = NULL; //set matrix pointer to NULL
  *M = NULL; //set matrix pointer to NULL
  *S = NULL; //set matrix pointer to NULL
  *matrix_size = 0; //set matrix size to zero

  return 0;
}

int Floyd_Warshall_Free_Matrices_For_Hop(int ***D, int ***M, int *matrix_size)
{ //free the memory of matrices for hop number, i.e., the shortest path matrix D and predecessor matrix M

  if(*D == NULL || *M == NULL)
  {
    printf("Floyd_Warshall_Free_Matrices_For_Hop(): we cannot free the matrices for hop number since one of matrices D and M have not been allocated memory yet!\n");
    exit(1);
  }

  Floyd_Warshall_Free_Matrix_Of_Type_Int(*D, *matrix_size);
  //allocate the memory of the shortest path matrix for movement whose rows and columns correspond to matrix_size

  Floyd_Warshall_Free_Matrix_Of_Type_Int(*M, *matrix_size);
  //allocate the memory of the predecessor matrix for movement whose rows and columns correspond to matrix_size  

  *D = NULL; //set matrix pointer to NULL
  *M = NULL; //set matrix pointer to NULL
  *matrix_size = 0; //set matrix size to zero

  return 0;
}

int Floyd_Warshall_Free_Matrices_For_Mcast(double ***T, double ***D, int ***M, double ***S, int *matrix_size)
{ //free the memory of matrices for Multicast based on EDC, i.e., the adjacency matrix T, the shortest path matrix D, the predecessor matrix M, and the supplementary metric matrix S

  if(*T == NULL || *D == NULL || *M == NULL || *S == NULL)
  {
    printf("Floyd_Warshall_Free_Matrices_For_Mcast(): we cannot free the matrices for Multicast since one of matrices T, D, M, and S have not been allocated memory yet!\n");
    exit(1);
  }

  Floyd_Warshall_Free_Matrix_Of_Type_Double(*T, *matrix_size);
  //allocate the memory of the adjacency matrix for Multicast whose rows and columns correspond to matrix_size

  Floyd_Warshall_Free_Matrix_Of_Type_Double(*D, *matrix_size);
  //allocate the memory of the shortest path matrix for Multicast whose rows and columns correspond to matrix_size

  Floyd_Warshall_Free_Matrix_Of_Type_Int(*M, *matrix_size);
  //allocate the memory of the predecessor matrix for Multicast whose rows and columns correspond to matrix_size  

  Floyd_Warshall_Free_Matrix_Of_Type_Double(*S, *matrix_size);
  //allocate the memory of the matrix of the cost variance for the shortest path whose rows and columns correspond to matrix_size

  *T = NULL; //set matrix pointer to NULL
  *D = NULL; //set matrix pointer to NULL
  *M = NULL; //set matrix pointer to NULL
  *S = NULL; //set matrix pointer to NULL
  *matrix_size = 0; //set matrix size to zero

  return 0;
}

///////////////////////////////////////////////////////////////////

double** Floyd_Warshall_Make_Weight_Matrix_For_Movement(double **W, int W_size, struct_graph_node *G, int G_size)
{ //make the adjacency matrix W for movement with graph G
	int i, j;
	struct_graph_node *ptr = NULL; //pointer to graph node

	if(W_size < G_size)
	{
	  printf("Floyd_Warshall_Make_Weight_Matrix_For_Movement(): W_size(%d) is less than G_size(%d)\n", W_size, G_size);
	  exit(1);
	}

	/* initialize the weight matrix W */
	for(i = 0; i < G_size; i++)
	{
	        W[i][i] = 0;
		for(j = i+1; j < G_size; j++)
		{
		        W[i][j] = INF;
			W[j][i] = INF;
		}
	}

	/* make adjacency matrix W with adjacent list G */
	for(i = 0; i < G_size; i++)
	{
		ptr = G[i].next;
		if(ptr == NULL)
			continue;

		while(ptr != NULL)
		{
			j = atoi(ptr->vertex) - 1; 
			//node id starts from 1, but it should decrease by 1 for weight matrix where the id starts from 0.
			W[i][j] = ptr->weight;
			ptr = ptr->next;
  		}
	}

	return W;
}

int** Floyd_Warshall_Make_Weight_Matrix_For_Scanning(int **W, int W_size, struct_graph_node *G, int G_size)
{ //make the adjacency matrix W for scanning with with sensor_list in schedule table T
	int i, j;
	char u[NAME_SIZE], v[NAME_SIZE]; //vertex names
	schedule_table_node_t *ptr = NULL; //pointer to table entry node
	boolean flip_flag; //flip flag

	if(W_size < G_size)
	{
	  printf("Floyd_Warshall_Make_Weight_Matrix_For_Scanning(): W_size(%d) is less than G_size(%d)\n", W_size, G_size);
	  exit(1);
	}

	/* set the weights to W[i][j] using the number of sensors deployed on each edge and sensor_work_time */
	for(i = 0; i < G_size; i++)
	{
	        W[i][i] = 0;
		for(j = i+1; j < G_size; j++)
		{
			sprintf(u, "%d", i+1);
			sprintf(v, "%d", j+1);
			ptr = LookupTable(G, u, v, &flip_flag);
			if(ptr != NULL)
			        W[i][j] = W[j][i] = ptr->sensor_list.live_sensor_number;
			else
				W[i][j] = W[j][i] = INF;
			
		}
	}

	return W;
}

int** Floyd_Warshall_Make_Weight_Matrix_For_Breach(int **W, int W_size, struct_graph_node *G, int G_size)
{ //make the adjacency matrix W for constructing breach path matrix with sensor_list in schedule table T
	int i, j;
	char u[NAME_SIZE], v[NAME_SIZE]; //vertex names
	schedule_table_node_t *ptr = NULL; //pointer to table entry node
	//edge_queue_node_t *ptr = NULL; //pointer to edge queue node
	boolean flip_flag; //flip flag

	if(W_size < G_size)
	{
	  printf("Floyd_Warshall_Make_Weight_Matrix_For_Breach(): W_size(%d) is less than G_size(%d)\n", W_size, G_size);
	  exit(1);
	}

	/* set the weights to W[i][j] using the number of sensors deployed on each edge and sensor_work_time */
	for(i = 0; i < G_size; i++)
	{
	        W[i][i] = 0;
		for(j = i+1; j < G_size; j++)
		{
			sprintf(u, "%d", i+1);
			sprintf(v, "%d", j+1);
			ptr = LookupTable(G, u, v, &flip_flag);
			//ptr = FastLookupEdgeQueue(G, u, v, &flip_flag);
			if(ptr != NULL)
			{
			  if(ptr->sensor_list.live_sensor_number == 0)
			  //if(ptr->live_sensor_number == 0)
				        W[i][j] = W[j][i] = 1;
				else
				        W[i][j] = W[j][i] = INF;				  
			}
			else
				W[i][j] = W[j][i] = INF;
			
		}
	}

	return W;
}

void Floyd_Warshall_Make_Weight_Matrix_For_EDD(double **D, double **S, int D_size, struct_graph_node *G, int G_size, parameter_t *param)
{ //make the adjacency matrices D and S for EDD with road graph G
	int i, j;
	struct_graph_node *ptr = NULL; //pointer to graph node

	if(D_size < G_size)
	{
	  printf("Floyd_Warshall_Make_Weight_Matrix_For_EDD(): D_size(%d) is less than G_size(%d)\n", D_size, G_size);
	  exit(1);
	}

	/* initialize the weight matrices D and S */
	for(i = 0; i < G_size; i++)
	{
		D[i][i] = 0;
		S[i][i] = 0;
		for(j = i+1; j < G_size; j++)
		{
			D[i][j] = INF;
			D[j][i] = INF;

			S[i][j] = INF;
			S[j][i] = INF;
		}
	}

	/* make adjacency matrices D and S with adjacent list G */
	for(i = 0; i < G_size; i++)
	{
		ptr = G[i].next;
		if(ptr == NULL)
			continue;

		while(ptr != NULL)
		{
			j = atoi(ptr->vertex) - 1; 
			//node id starts from 1, but it should decrease by 1 for weight matrix where the id starts from 0.
			
			/* update the link delay, link delay variance, link delay standard deviation for each edge in the graph G with the traffic statistics of vehicular traffic density and vehicle speed */
			Update_LinkDelay_Information(param, G, G_size);

			/** check whether the link (i,j) is valid for EDD with the Connectivity Adjacency Matrix */
			if(param->vanet_table.Ar_edd[i][j] == 1)
			{
				/* set D[i][j] and S[i][j] to the head_node's edge_delay and edge_delay_variance, respectively */
				D[i][j] = ptr->edge_delay;
				S[i][j] = ptr->edge_delay_variance;
			}
			else
			{
				/* set D[i][j] and S[i][j] to INF because there does not exist at least one stationary node between i and j */
				D[i][j] = INF;
				S[i][j] = INF;
			}

			ptr = ptr->next;
		}
	}
}

void Floyd_Warshall_Make_Weight_Matrix_For_EDD_VAR(double **D, double **S, int D_size, struct_graph_node *G, int G_size, parameter_t *param)
{ //make the adjacency matrices D and S for EDD_VAR with road graph G
	int i, j;
	struct_graph_node *ptr = NULL; //pointer to graph node
        double standard_deviation = 0; //standard deviation for variance

	if(D_size < G_size)
	{
	  printf("Floyd_Warshall_Make_Weight_Matrix_For_EDD_VAR(): D_size(%d) is less than G_size(%d)\n", D_size, G_size);
	  exit(1);
	}

	/* initialize the weight matrices D and S */
	for(i = 0; i < G_size; i++)
	{
	        D[i][i] = 0;
	        S[i][i] = 0;
		for(j = i+1; j < G_size; j++)
		{
		        D[i][j] = INF;
			D[j][i] = INF;

		        S[i][j] = INF;
			S[j][i] = INF;
		}
	}

	/* make adjacency matrices D and S with adjacent list G */
	for(i = 0; i < G_size; i++)
	{
		ptr = G[i].next;
		if(ptr == NULL)
			continue;

		while(ptr != NULL)
		{
			j = atoi(ptr->vertex) - 1; 
			//node id starts from 1, but it should decrease by 1 for weight matrix where the id starts from 0.

			/* update the link delay, link delay variance, link delay standard deviation for each edge in the graph G with the traffic statistics of vehicular traffic density and vehicle speed */
			Update_LinkDelay_Information(param, G, G_size);

			/** check whether the link (i,j) is valid for EDD with the Connectivity Adjacency Matrix */
			if(param->vanet_table.Ar_edd[i][j] == 1)
			{
				/* set D[i][j] and S[i][j] to the head_node's edge_delay_variance and edge_delay, respectively */
				D[i][j] = ptr->edge_delay_variance;
				S[i][j] = ptr->edge_delay;
			}
			else
			{
				/* set D[i][j] and S[i][j] to INF because there does not exist at least one stationary node between i and j */
				D[i][j] = INF;
				S[i][j] = INF;
			}

			ptr = ptr->next;
		}
	}
}

void Floyd_Warshall_Make_Weight_Matrix_For_EDC(double **W, double **D, double **S, int W_size, struct_graph_node *G, int G_size, parameter_t *param)
{ //make the adjacency matrices W, D and S for EDC with road graph G
	int i, j;
	struct_graph_node *ptr = NULL; //pointer to graph node

	if(W_size < G_size)
	{
	  printf("Floyd_Warshall_Make_Weight_Matrix_For_EDC(): W_size(%d) is less than G_size(%d)\n", W_size, G_size);
	  exit(1);
	}

	/* initialize the weight matrices W, D and S */
	for(i = 0; i < G_size; i++)
	{
		W[i][i] = 0;
		D[i][i] = 0;
		S[i][i] = 0;
		for(j = i+1; j < G_size; j++)
		{
			W[i][j] = INF;
			W[j][i] = INF;

			D[i][j] = INF;
			D[j][i] = INF;

			S[i][j] = INF;
			S[j][i] = INF;
		}
	}

	/* make adjacency matrices D and S with adjacent list G */
	for(i = 0; i < G_size; i++)
	{
		ptr = G[i].next;
		if(ptr == NULL)
			continue;

		while(ptr != NULL)
		{
			j = atoi(ptr->vertex) - 1; 
			//node id starts from 1, but it should decrease by 1 for weight matrix where the id starts from 0.
			
			/* update the link cost (e.g., delay or link utilization), link cost variance, link cost standard deviation for each edge in the graph G with the traffic statistics of vehicular traffic density and vehicle speed */
			Update_LinkCost_Information(param, G, G_size);

			/** check whether the link (i,j) is valid for EDD with the Connectivity Adjacency Matrix */
			if(param->vanet_table.Ar_edd[i][j] == 1)
			{
				/* set W[i][j] & D[i][j] and S[i][j] to the head_node's edge_cost and edge_cost_variance, respectively */
				W[i][j] = D[i][j] = ptr->edge_cost;
				S[i][j] = ptr->edge_cost_variance;
			}
			else
			{
				/* set W[i][j] & D[i][j] and S[i][j] to INF because there does not exist at least one stationary node between i and j */
				W[i][j] = D[i][j] = INF;
				S[i][j] = INF;
			}

			ptr = ptr->next;
		}
	}
}

int** Floyd_Warshall_Make_Weight_Matrix_For_Hop(int **W, int W_size, struct_graph_node *G, int G_size)
{ //make the adjacency matrix W for hop number with graph G
	int i, j;
	struct_graph_node *ptr = NULL; //pointer to graph node

	if(W_size < G_size)
	{
	  printf("Floyd_Warshall_Make_Weight_Matrix_For_Hop(): W_size(%d) is less than G_size(%d)\n", W_size, G_size);
	  exit(1);
	}

	/* initialize the weight matrix W */
	for(i = 0; i < G_size; i++)
	{
	        W[i][i] = 0;
		for(j = i+1; j < G_size; j++)
		{
			W[i][j] = INF;
			W[j][i] = INF;
		}
	}

	/* make adjacency matrix W with adjacent list G */
	for(i = 0; i < G_size; i++)
	{
		ptr = G[i].next;
		if(ptr == NULL)
			continue;

		while(ptr != NULL)
		{
			j = atoi(ptr->vertex) - 1; 
			//node id starts from 1, but it should decrease by 1 for weight matrix where the id starts from 0.
			W[i][j] = 1; //Note that 1 indicates the existence of an edge between nodes i and j
			ptr = ptr->next;
  		}
	}

	return W;
}

///////////////////////////////////////////////////////////////////

int Floyd_Warshall_Construct_Matrices_For_Movement(struct_graph_node *G, int G_size, double ***D, int ***M, int *matrix_size)
{ //construct the shortest path weight matrix D and predecessor matrix M

#ifdef __DEBUG__
	int i, j;
#endif

	/** check whether graph information is valid or not */
	if(G_size < 0 || G == NULL)
	{
	  printf("Floyd_Warshall_Construct_Matrices_For_Movement(): G_size(%d) < 0 or G is NULL\n", G_size);
	  exit(1);
	}

	/** check whether G_size is greater than matrix_size. If so, expand the size of the matrices W, D, and M */
	if(G_size > *matrix_size)
	{
	  Floyd_Warshall_Reallocate_Matrices_For_Movement(G_size, D, M, matrix_size);
          //reallocate the memory of matrices for movement, i.e., the shortest path matrix D and predecessor matrix M

#ifdef __DEBUG__	  
	  printf("Floyd_Warshall_Construct_Matrices_For_Movement(): we expand the matrices of D and M\n");
#endif
	}

	*D = Floyd_Warshall_Make_Weight_Matrix_For_Movement(*D, *matrix_size, G, G_size);
	//construct a weight matrix from the adjacent list G representing the graph

	Floyd_Warshall_Construct_Shortest_PathInfo_For_Movement(*D, *M, G_size);
        //compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm

#ifdef __DEBUG__
	for(i = 1; i <= G_size; i++)
	{
		for(j = 1; j <= G_size; j++)
		{
			if(i != j)
				Floyd_Warshall_Get_Shortest_Path(*M, G_size, i, j);
				//Floyd_Warshall_Get_Shortest_Path((int*) M, n, i, j);
		}
	}
#endif
	
	return 0;
}

int Floyd_Warshall_Construct_Matrices_For_Scanning(struct_graph_node *G, int G_size, int ***D, int ***M, int *matrix_size)
{ //construct the shortest path weight matrix D and predecessor matrix M in terms of the number of sensors for sensor scanning

#ifdef __DEBUG__
	int i, j;
#endif

	/** check whether graph information is valid or not */
	if(G_size < 0 || G == NULL)
	{
	  printf("Floyd_Warshall_Construct_Matrices_For_Scanning(): G_size(%d) < 0 or G is NULL\n", G_size);
	  exit(1);
	}

	/** check whether G_size is greater than matrix_size. If so, expand the size of the matrices D and M */
	if(G_size > *matrix_size)
	{
	  Floyd_Warshall_Reallocate_Matrices_For_Scanning(G_size, D, M, matrix_size);
          //reallocate the memory of matrices for scanning, i.e., the shortest path matrix D and predecessor matrix M

#ifdef __DEBUG__
	  printf("Floyd_Warshall_Construct_Matrices_For_Scanning(): we expand the matrices of D and M\n");
#endif
	}

	*D = Floyd_Warshall_Make_Weight_Matrix_For_Scanning(*D, *matrix_size, G, G_size);
	//construct a weight matrix for scanning from the schedule table T for virtual graph Gv

	Floyd_Warshall_Construct_Shortest_PathInfo_For_Scanning(*D, *M, G_size);
        //compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm

#ifdef __DEBUG__
	for(i = 1; i <= G_size; i++)
	{
		for(j = 1; j <= G_size; j++)
		{
			if(i != j)
				Floyd_Warshall_Get_Shortest_Path(*M, G_size, i, j);
				//Floyd_Warshall_Get_Shortest_Path((int*) M, n, i, j);
		}
	}
#endif
	
	return 0;
}

int Floyd_Warshall_Construct_Matrices_For_Breach(struct_graph_node *G, int G_size, int ***D, int ***M, int *matrix_size)
{ //construct the shortest path weight matrix D and predecessor matrix M for breach path where no-live-sensor edge has 1 and live-sensor-edge has INF.

#ifdef __DEBUG__
	int i, j;
#endif

	/** check whether graph information is valid or not */
	if(G_size < 0 || G == NULL)
	{
	  printf("Floyd_Warshall_Construct_Matrices_For_Breach(): G_size(%d) < 0 or G is NULL\n", G_size);
	  exit(1);
	}

	/** check whether G_size is greater than matrix_size. If so, expand the size of the matrices D and M */
	if(G_size > *matrix_size)
	{
	  Floyd_Warshall_Reallocate_Matrices_For_Breach(G_size, D, M, matrix_size);
          //reallocate the memory of matrices for breach path, i.e., the shortest path matrix D and predecessor matrix M
#ifdef __DEBUG__
	  printf("Floyd_Warshall_Construct_Matrices_For_Breach(): we expand the matrices of D and M\n");
#endif
	}

	*D = Floyd_Warshall_Make_Weight_Matrix_For_Breach(*D, *matrix_size, G, G_size);
	//construct a weight matrix for breach path from the sensor list in the schedule table T for virtual graph Gv

	Floyd_Warshall_Construct_Shortest_PathInfo_For_Breach(*D, *M, G_size);
        //compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm

#ifdef __DEBUG__
	for(i = 1; i <= G_size; i++)
	{
		for(j = 1; j <= G_size; j++)
		{
			if(i != j)
				Floyd_Warshall_Get_Shortest_Path(*M, G_size, i, j);
				//Floyd_Warshall_Get_Shortest_Path((int*) M, n, i, j);
		}
	}
#endif
	
	return 0;
}

int Floyd_Warshall_Construct_Matrices_For_EDD(struct_graph_node *G, int G_size, double ***D, int ***M, double ***S, int *matrix_size, parameter_t *param)
{ //construct the shortest path weight matrix D, the predecessor matrix M, and the supplementary metric matrix S for EDD

#ifdef __DEBUG__
	int i, j;
#endif

	/** check whether graph information is valid or not */
	if(G_size < 0 || G == NULL)
	{
	  printf("Floyd_Warshall_Construct_Matrices_For_EDD(): G_size(%d) < 0 or G is NULL\n", G_size);
	  exit(1);
	}

	/** check whether G_size is greater than matrix_size. If so, expand the size of the matrices W, D, M and S */
	if(G_size > *matrix_size)
	{
          Floyd_Warshall_Reallocate_Matrices_For_EDD(G_size, D, M, S, matrix_size);
          //reallocate the memory of matrices for EDD, i.e., the shortest path matrix D, the predecessor matrix M, and the supplementary metric matric S

#ifdef __DEBUG__	  
	  printf("Floyd_Warshall_Construct_Matrices_For_EDD(): we expand the matrices of D, M, and S\n");
#endif
	}

	Floyd_Warshall_Make_Weight_Matrix_For_EDD(*D, *S, *matrix_size, G, G_size, param);
	//construct a weight matrix from the adjacent list G representing the graph

	Floyd_Warshall_Construct_Shortest_PathInfo_For_EDD(*D, *M, *S, G_size);
        //compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm

#ifdef __DEBUG__
	for(i = 1; i <= G_size; i++)
	{
		for(j = 1; j <= G_size; j++)
		{
			if(i != j)
				Floyd_Warshall_Get_Shortest_Path(*M, G_size, i, j);
				//Floyd_Warshall_Get_Shortest_Path((int*) M, n, i, j);
		}
	}
#endif
	
	return 0;
}

int Floyd_Warshall_Construct_Matrices_For_EDD_VAR(struct_graph_node *G, int G_size, double ***D, int ***M, double ***S, int *matrix_size, parameter_t *param)
{ //construct the shortest path weight matrix D, the predecessor matrix M, and the supplementary metric matrix S for EDD_VAR

#ifdef __DEBUG__
	int i, j;
#endif

	/** check whether graph information is valid or not */
	if(G_size < 0 || G == NULL)
	{
	  printf("Floyd_Warshall_Construct_Matrices_For_EDD_VAR(): G_size(%d) < 0 or G is NULL\n", G_size);
	  exit(1);
	}

	/** check whether G_size is greater than matrix_size. If so, expand the size of the matrices W, D, M and S */
	if(G_size > *matrix_size)
	{
          Floyd_Warshall_Reallocate_Matrices_For_EDD_VAR(G_size, D, M, S, matrix_size);
          //reallocate the memory of matrices for EDD_VAR, i.e., the shortest path matrix D, the predecessor matrix M, and the supplementary metric matric S

#ifdef __DEBUG__	  
	  printf("Floyd_Warshall_Construct_Matrices_For_EDD_VAR(): we expand the matrices of D, M, and S\n");
#endif
	}

	Floyd_Warshall_Make_Weight_Matrix_For_EDD_VAR(*D, *S, *matrix_size, G, G_size, param);
	//construct a weight matrix from the adjacent list G representing the graph

	Floyd_Warshall_Construct_Shortest_PathInfo_For_EDD_VAR(*D, *M, *S, G_size);
        //compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm

#ifdef __DEBUG__
	for(i = 1; i <= G_size; i++)
	{
		for(j = 1; j <= G_size; j++)
		{
			if(i != j)
				Floyd_Warshall_Get_Shortest_Path(*M, G_size, i, j);
				//Floyd_Warshall_Get_Shortest_Path((int*) M, n, i, j);
		}
	}
#endif
	
	return 0;
}

int Floyd_Warshall_Construct_Matrices_For_EDC(struct_graph_node *G, int G_size, double ***W, double ***D, int ***M, double ***S, int *matrix_size, parameter_t *param)
{ //construct the adjacency matrix W, the shortest path weight matrix D, the predecessor matrix M, and the supplementary metric matrix S for EDC

#ifdef __DEBUG__
	int i, j;
#endif

	/** check whether graph information is valid or not */
	if(G_size < 0 || G == NULL)
	{
	  printf("Floyd_Warshall_Construct_Matrices_For_EDC(): G_size(%d) < 0 or G is NULL\n", G_size);
	  exit(1);
	}

	/** check whether G_size is greater than matrix_size. If so, expand the size of the matrices W, D, M and S */
	if(G_size > *matrix_size)
	{
	  Floyd_Warshall_Reallocate_Matrices_For_EDC(G_size, W, D, M, S, matrix_size);
          //reallocate the memory of matrices for EDC, i.e., the adjacency matrix W, the shortest path matrix D, the predecessor matrix M, and the supplementary metric matric S

#ifdef __DEBUG__	  
	  printf("Floyd_Warshall_Construct_Matrices_For_EDC(): we expand the matrices of W, D, M, and S\n");
#endif
	}

	Floyd_Warshall_Make_Weight_Matrix_For_EDC(*W, *D, *S, *matrix_size, G, G_size, param);
	//construct a weight matrix from the adjacent list G representing the graph

	Floyd_Warshall_Construct_Shortest_PathInfo_For_EDC(*D, *M, *S, G_size);
	//compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm
#ifdef __DEBUG__
	for(i = 1; i <= G_size; i++)
	{
		for(j = 1; j <= G_size; j++)
		{
			if(i != j)
				Floyd_Warshall_Get_Shortest_Path(*M, G_size, i, j);
				//Floyd_Warshall_Get_Shortest_Path((int*) M, n, i, j);
		}
	}
#endif
	
	return 0;
}

int Floyd_Warshall_Construct_Matrices_For_Hop(struct_graph_node *G, int G_size, int ***D, int ***M, int *matrix_size)
{ //construct the hop shortest path weight matrix D and predecessor matrix M

#ifdef __DEBUG__
	int i, j;
#endif

	/** check whether graph information is valid or not */
	if(G_size < 0 || G == NULL)
	{
	  printf("Floyd_Warshall_Construct_Matrices_For_Hop(): G_size(%d) < 0 or G is NULL\n", G_size);
	  exit(1);
	}

	/** check whether G_size is greater than matrix_size. If so, expand the size of the matrices W, D, and M */
	if(G_size > *matrix_size)
	{
	  Floyd_Warshall_Reallocate_Matrices_For_Hop(G_size, D, M, matrix_size);
	  //reallocate the memory of matrices for hop number, i.e., the shortest path matrix D and predecessor matrix M

#ifdef __DEBUG__	  
	  printf("Floyd_Warshall_Construct_Matrices_For_Hop(): we expand the matrices of D and M\n");
#endif
	}

	*D = Floyd_Warshall_Make_Weight_Matrix_For_Hop(*D, *matrix_size, G, G_size);
	//construct a weight matrix from the adjacent list G representing the graph

	Floyd_Warshall_Construct_Shortest_PathInfo_For_Hop(*D, *M, G_size);
    //compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm

#ifdef __DEBUG__
	for(i = 1; i <= G_size; i++)
	{
		for(j = 1; j <= G_size; j++)
		{
			if(i != j)
				Floyd_Warshall_Get_Shortest_Path(*M, G_size, i, j);
				//Floyd_Warshall_Get_Shortest_Path((int*) M, n, i, j);
		}
	}
#endif
	
	return 0;
}

///////////////////////////////////////////////////////////////////

int Floyd_Warshall_Construct_Shortest_PathInfo_For_Movement(double** D, int** M, int n)
{ //compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm for vehicle's movement
  //where D is regarded as a dynamically allocated 2-dimensional array
  //input:= D: edge weight matrix, n: number of nodes 
  //output:= D: distance matrix, M: predecessor matrix
	int i, j, k;

	/* initialize the precessor matrix M */
	for(i = 0; i < n; i++) //for-1
	{
		M[i][i] = NIL; //there is no predecessor of node j
		for(j = i+1; j < n; j++) //for-2
		{
			/* consider the directed edge (i,j) */
			if(D[i][j] == INF)
			{
				M[i][j] = NIL; //there is no predecessor of node j
			}
			else if(D[i][j] < INF)
			{
				M[i][j] = i; //node i becomes the predecessor of node j
			}
			else
			{
				printf("Floyd_Warshall_Construct_Shortest_PathInfo_For_Movement(): D[%d][%d](=%f) is greater than INF\n", i, j, (float)D[i][j]);
				exit(1);
			}

		  	/* consider the directed edge (j,i) */
			if(D[j][i] == INF)
			{
				M[j][i] = NIL; //there is no predecessor of node i
			}
			else if(D[j][i] < INF)
			{
				M[j][i] = j; //node j becomes the predecessor of node i
			}
			else
			{
				printf("Floyd_Warshall_Construct_Shortest_PathInfo_For_Movement(): D[%d][%d](=%f) is greater than INF\n", j, i, (float)D[j][i]);
				exit(1);
			}
		} //end of for-2
	} //end of for-1

	/** compute all-pairs shortest paths */
	for(k = 0; k < n; k++) //for-1
	{
		for(i = 0; i < n; i++) //for-2
		{
			for(j = 0; j < n; j++) //for-3
			{
				if(D[i][j] > D[i][k] + D[k][j])
				{
					D[i][j] = D[i][k] + D[k][j]; //update the length of path(i,j)
					M[i][j] = M[k][j]; //update the predecessor of node j
				}
			} //end of for-3
  		} //end of for-2
    } //end of for-1

    return 0;
}

int Floyd_Warshall_Construct_Shortest_PathInfo_For_Scanning(int** D, int** M, int n)
{ //compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm for sensor scanning
  //where D is regarded as a dynamically allocated 2-dimensional array
  //input:= D: edge weight matrix, n: number of nodes 
  //output:= D: distance matrix, M: predecessor matrix
	int i, j, k;

	/* initialize the precessor matrix M */
	for(i = 0; i < n; i++) //for-1
	{
		M[i][i] = NIL; //there is no predecessor of node j
		for(j = i+1; j < n; j++) //for-2
		{
			/* consider the directed edge (i,j) */
			if(D[i][j] == INF)
			{
				M[i][j] = NIL; //there is no predecessor of node j
			}
			else if(D[i][j] < INF)
			{
				M[i][j] = i; //node i becomes the predecessor of node j
			}
			else
			{
				printf("Floyd_Warshall_Construct_Shortest_PathInfo_For_Scanning(): D[%d][%d](=%f) is greater than INF\n", i, j, (float)D[i][j]);
				exit(1);
			}

		  	/* consider the directed edge (j,i) */
			if(D[j][i] == INF)
			{
				M[j][i] = NIL; //there is no predecessor of node i
			}
			else if(D[j][i] < INF)
			{
				M[j][i] = j; //node j becomes the predecessor of node i
			}
			else
			{
				printf("Floyd_Warshall_Construct_Shortest_PathInfo_For_Scanning(): D[%d][%d](=%f) is greater than INF\n", j, i, (float)D[j][i]);
				exit(1);
			}
		} //end of for-2
	} //end of for-1

	/** compute all-pairs shortest paths */
	for(k = 0; k < n; k++) //for-1
	{
		for(i = 0; i < n; i++) //for-2
		{
			for(j = 0; j < n; j++) //for-3
			{
				if(D[i][j] > D[i][k] + D[k][j])
				{
					D[i][j] = D[i][k] + D[k][j]; //update the length of path(i,j)
					M[i][j] = M[k][j]; //update the predecessor of node j
				}
			} //end of for-3
  		} //end of for-2
    } //end of for-1

    return 0;
}

int Floyd_Warshall_Construct_Shortest_PathInfo_For_Breach(int** D, int** M, int n)
{ //compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm for breach path
  //where D is regarded as a dynamically allocated 2-dimensional array
  //input:= D: edge weight matrix, n: number of nodes 
  //output:= D: distance matrix, M: predecessor matrix
	int i, j, k;

	/* initialize the precessor matrix M */
	for(i = 0; i < n; i++) //for-1
	{
		M[i][i] = NIL; //there is no predecessor of node j
		for(j = i+1; j < n; j++) //for-2
		{
			/* consider the directed edge (i,j) */
			if(D[i][j] == INF)
			{
				M[i][j] = NIL; //there is no predecessor of node j
			}
			else if(D[i][j] < INF)
			{
				M[i][j] = i; //node i becomes the predecessor of node j
			}
			else
			{
				printf("Floyd_Warshall_Construct_Shortest_PathInfo_For_Breach(): D[%d][%d](=%f) is greater than INF\n", i, j, (float)D[i][j]);
				exit(1);
			}

		  	/* consider the directed edge (j,i) */
			if(D[j][i] == INF)
			{
				M[j][i] = NIL; //there is no predecessor of node i
			}
			else if(D[j][i] < INF)
			{
				M[j][i] = j; //node j becomes the predecessor of node i
			}
			else
			{
				printf("Floyd_Warshall_Construct_Shortest_PathInfo_For_Breach(): D[%d][%d](=%f) is greater than INF\n", j, i, (float)D[j][i]);
				exit(1);
			}
		} //end of for-2
	} //end of for-1

	/** compute all-pairs shortest paths */
	for(k = 0; k < n; k++) //for-1
	{
		for(i = 0; i < n; i++) //for-2
		{
			for(j = 0; j < n; j++) //for-3
			{
				if(D[i][j] > D[i][k] + D[k][j])
				{
					D[i][j] = D[i][k] + D[k][j]; //update the length of path(i,j)
					M[i][j] = M[k][j]; //update the predecessor of node j
				}
			} //end of for-3
  		} //end of for-2
    } //end of for-1

    return 0;
}

int Floyd_Warshall_Construct_Shortest_PathInfo_For_EDD(double** D, int** M, double** S, int n)
{ //compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm for EDD
  //where D is regarded as a dynamically allocated 2-dimensional array
  //input:= D: edge weight matrix, n: number of nodes 
  //output:= D: distance matrix, M: predecessor matrix, S: supplementary metric matrix
	int i, j, k;

	/* initialize the precessor matrix M */
	for(i = 0; i < n; i++) //for-1
	{
		M[i][i] = NIL; //there is no predecessor of node j
		for(j = i+1; j < n; j++) //for-2
		{
			/* consider the directed edge (i,j) */
			if(D[i][j] == INF)
			{
				M[i][j] = NIL; //there is no predecessor of node j
			}
			else if(D[i][j] < INF)
			{
				M[i][j] = i; //node i becomes the predecessor of node j
			}
			else
			{
				printf("Floyd_Warshall_Construct_Shortest_PathInfo_For_EDD(): D[%d][%d](=%f) is greater than INF\n", i, j, (float)D[i][j]);
				exit(1);
			}

		  	/* consider the directed edge (j,i) */
			if(D[j][i] == INF)
			{
				M[j][i] = NIL; //there is no predecessor of node i
			}
			else if(D[j][i] < INF)
			{
				M[j][i] = j; //node j becomes the predecessor of node i
			}
			else
			{
				printf("Floyd_Warshall_Construct_Shortest_PathInfo_For_EDD(): D[%d][%d](=%f) is greater than INF\n", j, i, (float)D[j][i]);
				exit(1);
			}
		} //end of for-2
	} //end of for-1

	/** compute all-pairs shortest paths */
	for(k = 0; k < n; k++) //for-1
	{
		for(i = 0; i < n; i++) //for-2
		{
			for(j = 0; j < n; j++) //for-3
			{
				if(D[i][j] > D[i][k] + D[k][j])
				{
					D[i][j] = D[i][k] + D[k][j]; //update the delay of path(i,j)
					S[i][j] = S[i][k] + S[k][j]; //update the supplementary metric for path(i,j), such as E2E delay variance  
					M[i][j] = M[k][j]; //update the predecessor of node j
				}
			} //end of for-3
  		} //end of for-2
    } //end of for-1

    return 0;
}

int Floyd_Warshall_Construct_Shortest_PathInfo_For_EDD_VAR(double** D, int** M, double** S, int n)
{ //compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm for EDD_VAR
  //where D is regarded as a dynamically allocated 2-dimensional array
  //input:= D: edge weight matrix, n: number of nodes 
  //output:= D: distance matrix, M: predecessor matrix, S: supplementary metric matrix
	int i, j, k;

	/* initialize the precessor matrix M */
	for(i = 0; i < n; i++) //for-1
	{
		M[i][i] = NIL; //there is no predecessor of node j
		for(j = i+1; j < n; j++) //for-2
		{
			/* consider the directed edge (i,j) */
			if(D[i][j] == INF)
			{
				M[i][j] = NIL; //there is no predecessor of node j
			}
			else if(D[i][j] < INF)
			{
				M[i][j] = i; //node i becomes the predecessor of node j
			}
			else
			{
				printf("Floyd_Warshall_Construct_Shortest_PathInfo_For_EDD_VAR(): D[%d][%d](=%f) is greater than INF\n", i, j, (float)D[i][j]);
				exit(1);
			}

		  	/* consider the directed edge (j,i) */
			if(D[j][i] == INF)
			{
				M[j][i] = NIL; //there is no predecessor of node i
			}
			else if(D[j][i] < INF)
			{
				M[j][i] = j; //node j becomes the predecessor of node i
			}
			else
			{
				printf("Floyd_Warshall_Construct_Shortest_PathInfo_For_EDD_VAR(): D[%d][%d](=%f) is greater than INF\n", j, i, (float)D[j][i]);
				exit(1);
			}
		} //end of for-2
	} //end of for-1

	/** compute all-pairs shortest paths */
	for(k = 0; k < n; k++) //for-1
	{
		for(i = 0; i < n; i++) //for-2
		{
			for(j = 0; j < n; j++) //for-3
			{
				if(D[i][j] > D[i][k] + D[k][j])
				{
					D[i][j] = D[i][k] + D[k][j]; //update the length of path(i,j)
                                        S[i][j] = S[i][k] + S[k][j]; //update the supplementary metric for path(i,j), such as E2E delay
					M[i][j] = M[k][j]; //update the predecessor of node j
				}
			} //end of for-3
  		} //end of for-2
    } //end of for-1

    return 0;
}

int Floyd_Warshall_Construct_Shortest_PathInfo_For_EDC(double** D, int** M, double** S, int n)
{ //compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm for EDC
  //where D is regarded as a dynamically allocated 2-dimensional array
  //input:= D: edge weight matrix, n: number of nodes 
  //output:= D: distance matrix, M: predecessor matrix, S: supplementary metric matrix
	int i, j, k;

	/* initialize the precessor matrix M */
	for(i = 0; i < n; i++) //for-1
	{
		M[i][i] = NIL; //there is no predecessor of node j
		for(j = i+1; j < n; j++) //for-2
		{
		  	/* consider the directed edge (i,j) */
			if(D[i][j] == INF)
			{
				M[i][j] = NIL; //there is no predecessor of node j
			}
			else if(D[i][j] < INF)
			{
				M[i][j] = i; //node i becomes the predecessor of node j
			}
			else
			{
				printf("Floyd_Warshall_Construct_Shortest_PathInfo_For_EDC(): D[%d][%d](=%f) is greater than INF\n", i, j, (float)D[i][j]);
				exit(1);
			}

		  	/* consider the directed edge (j,i) */
			if(D[j][i] == INF)
			{
				M[j][i] = NIL; //there is no predecessor of node i
			}
			else if(D[j][i] < INF)
			{
				M[j][i] = j; //node j becomes the predecessor of node i
			}
			else
			{
				printf("Floyd_Warshall_Construct_Shortest_PathInfo_For_EDC(): D[%d][%d](=%f) is greater than INF\n", j, i, (float)D[j][i]);
				exit(1);
			}
		} //end of for-2
	} //end of for-1

	/** compute all-pairs shortest paths */
	for(k = 0; k < n; k++) //for-1
	{
		for(i = 0; i < n; i++) //for-2
		{
			for(j = 0; j < n; j++) //for-3
			{
				if(D[i][j] > D[i][k] + D[k][j])
				{
					D[i][j] = D[i][k] + D[k][j]; //update the cost of path(i,j)
					S[i][j] = S[i][k] + S[k][j]; //update the supplementary metric for path(i,j), such as E2E cost variance  
					M[i][j] = M[k][j]; //update the predecessor of node j
				}
			} //end of for-3
  		} //end of for-2
    } //end of for-1

    return 0;
}

int Floyd_Warshall_Construct_Shortest_PathInfo_For_Hop(int** D, int** M, int n)
{ //compute the matrix for all-pairs shortest-paths using the Floyd-Warshall algorithm for the hop number from source to destination
  //where D is regarded as a dynamically allocated 2-dimensional array
  //input:= D: edge weight matrix, n: number of nodes 
  //output:= D: distance matrix, M: predecessor matrix
	int i, j, k;

	/* initialize the precessor matrix M */
	for(i = 0; i < n; i++) //for-1
	{
	    M[i][i] = NIL; //there is no predecessor of node j
		for(j = i+1; j < n; j++) //for-2
		{
			/* consider the directed edge (i,j) */
			if(D[i][j] == INF)
			{
				M[i][j] = NIL; //there is no predecessor of node j
			}
			else if(D[i][j] < INF)
			{
				M[i][j] = i; //node i becomes the predecessor of node j
			}
			else
			{
				printf("Floyd_Warshall_Construct_Shortest_PathInfo_For_Hop(): D[%d][%d](=%f) is greater than INF\n", i, j, (float)D[i][j]);
				exit(1);
			}

		  	/* consider the directed edge (j,i) */
			if(D[j][i] == INF)
			{
				M[j][i] = NIL; //there is no predecessor of node i
			}
			else if(D[j][i] < INF)
			{
				M[j][i] = j; //node j becomes the predecessor of node i
			}
			else
			{
				printf("Floyd_Warshall_Construct_Shortest_PathInfo_For_Hop(): D[%d][%d](=%f) is greater than INF\n", j, i, (float)D[j][i]);
				exit(1);
			}
		} //end of for-2
	} //end of for-1

	/** compute all-pairs shortest paths */
	for(k = 0; k < n; k++) //for-1
	{
		for(i = 0; i < n; i++) //for-2
		{
			for(j = 0; j < n; j++) //for-3
			{
				if(D[i][j] > D[i][k] + D[k][j])
				{
					D[i][j] = D[i][k] + D[k][j]; //update the length of path(i,j)
					M[i][j] = M[k][j]; //update the predecessor of node j
				}
			} //end of for-3
  		} //end of for-2
    } //end of for-1

    return 0;
}

///////////////////////////////////////////////////////////////////

double Get_Link_Cost_With_Floyd_Warshall_Get_Shortest_Path(int** M, int n, int src, int dst)
{
	int i, j;
	int current_hop_node = 0; //current-hop node towards src
	
	double **Dr = param->vanet_table.Dr_edc; //delivery-cost matrix
	double **Sr = param->vanet_table.Sr_edc; //delivery-cost-variance matrix
	int Dr_size = param->vanet_table.matrix_size_for_edc_in_Gr; //matrix size

	if(src > n || dst > n)
	{
		printf("Floyd_Warshall_Get_Shortest_Path(): src(%d) > n(%d) or dst(%d) > n(%d)\n", src, n, dst, n);
		exit(1);
	}

	i = src-1; //node i corresponds to index i-1.
	j = dst-1;
 
	/* get the current-hop node towards src */
	current_hop_node = j+1;

	/* get the previous node towards src */
	j = M[i][j];
	//j = *(M+n*i+j);
	if(j == NIL)
	{
	    //printf("Floyd_Warshall_Get_Shortest_Path(): there is no next hop for dst(%d) towards src(%d)\n", dst, src);
		return 0;
	}
	else
	{	/* print the current hop node, that is, dst */
		printf("%d <- ", current_hop_node);
	}

	double linkCost = 0;
	while(j != i)
	{
		printf("%d <- ", j+1);		
		
		// calculate link cost from current_hop_node to j+1 
		double tempLinkCost;
		tempLinkCost = 0; // calculate link cost
		linkCost += tempLinkCost + (60)/4;
		
		current_hop_node = j+1; //current-hop ncode towards src
		/* get the previous node towards src */
		j = M[i][j];
		//j = *(M+n*i+j);
		if(j == NIL)
		{
			printf("Floyd_Warshall_Get_Shortest_Path(): there is no next hop for current_hop_node(%d) towards src(%d)\n", current_hop_node, src);
			return 0;
		}
	}
	printf("%d\n", j+1);
	
	return linkCost;
}

void Floyd_Warshall_Get_Shortest_Path(int** M, int n, int src, int dst)
//void Floyd_Warshall_Get_Shortest_Path(int* M, int n, int src, int dst)
{ //get the shortest path from src to dst using Floyd_Warshall algorithm
  //input:= M: predecessor matrix, n: number of nodes , src: source node, dst: destination node
 
	int i, j;
	int current_hop_node = 0; //current-hop node towards src

	if(src > n || dst > n)
	{
		printf("Floyd_Warshall_Get_Shortest_Path(): src(%d) > n(%d) or dst(%d) > n(%d)\n", src, n, dst, n);
		exit(1);
	}

	i = src-1; //node i corresponds to index i-1.
	j = dst-1;
 
	/* get the current-hop node towards src */
	current_hop_node = j+1;

	/* get the previous node towards src */
	j = M[i][j];
	//j = *(M+n*i+j);
	if(j == NIL)
	{
	    //printf("Floyd_Warshall_Get_Shortest_Path(): there is no next hop for dst(%d) towards src(%d)\n", dst, src);
		return;
	}
	else
	{	/* print the current hop node, that is, dst */
		printf("%d <- ", current_hop_node);
	}

	while(j != i)
	{
		printf("%d <- ", j+1);
		current_hop_node = j+1; //current-hop ncode towards src
		/* get the previous node towards src */
		j = M[i][j];
		//j = *(M+n*i+j);
		if(j == NIL)
		{
			printf("Floyd_Warshall_Get_Shortest_Path(): there is no next hop for current_hop_node(%d) towards src(%d)\n", current_hop_node, src);
			return;
		}
	}
	printf("%d\n", j+1);
}

path_queue_t* Floyd_Warshall_Make_Shortest_Path_Queue(int** M, int n, int src, int dst)
{ //make a queue containing the shortest path from dst to src using Floyd_Warshall algorithm
  //input:= M: predecessor matrix, n: number of nodes , src: source node, dst: destination node
 
	static path_queue_t Q; //path queue for the shortest path from src to dst
	path_queue_node_t queue_node; //node for path queue
	int i, j;
	char node[NAME_SIZE]; //node name
	//char buffer[BUF_SIZE]; //buffer to contain integer value as character string

	if(src > n || dst > n)
	{
		printf("Floyd_Warshall_Make_Shortest_Path_Queue(): src(%d) > n(%d) or dst(%d) > n(%d)\n", src, n, dst, n);
		exit(1);
	}

	InitQueue((queue_t*) &Q, QTYPE_PATH); //initialize path queue Q

	i = src - 1; //node i corresponds to index i-1.
	j = dst - 1;

#ifdef __DEBUG__
	printf("%d <- ", j+1);
#endif

	/* enqueue the name of node (j+1) into Q */
	memset(&queue_node, 0, sizeof(queue_node));
	//_itoa(j+1, node, 10);
	sprintf(node, "%d", j+1);
	strcpy(queue_node.node, node);
	Enqueue((queue_t*) &Q, (queue_node_t*) &queue_node);

	/* get the previous node towards src */
	j = M[i][j];
	while(j != i)
	{
#ifdef __DEBUG__
		printf("%d <- ", j+1);
#endif

		/* enqueue the name of node (j+1) into Q */
		memset(&queue_node, 0, sizeof(queue_node));
		//_itoa(j+1, node, 10);
	        sprintf(node, "%d", j+1);
		strcpy(queue_node.node, node);
		Enqueue((queue_t*) &Q, (queue_node_t*) &queue_node);

		/* get the previous node towards src */
		j = M[i][j];
	}

#ifdef __DEBUG__
	printf("%d\n", j+1);
#endif

	/* enqueue the name of node (j+1) into Q */
	memset(&queue_node, 0, sizeof(queue_node));
	//_itoa(j+1, node, 10);
	sprintf(node, "%d", j+1);
	strcpy(queue_node.node, node);
	Enqueue((queue_t*) &Q, (queue_node_t*) &queue_node);

	return &Q;
}

void Floyd_Warshall_Show_Weight_Table(double** A, int n)
{ //show the two-dimensional nxn weight matrix A
	int i, j;
	char *msg = NULL;
	char *buf = NULL;
	//char msg[MSG_BUF_SIZE]; 
	//Where n is a large number, such as 18, the memory access violation happens.
	//So we need to dynamically allocate the memory for msg and buf according to n.
	//char buf[BUF_SIZE];
	const int number_string_length = NUMBER_STRING_LEN; //length of number string
	int buf_len; //buf length
	int msg_len; //msg length

	buf_len = sizeof(char)*n*number_string_length;
	//buf = (char*) malloc(buf_len);
	buf = (char*) calloc(buf_len, sizeof(char));
	assert_memory(buf);

	msg_len = sizeof(char)*n*n*number_string_length;
	//msg = (char*) malloc(msg_len);
	msg = (char*) calloc(msg_len, sizeof(char));
	assert_memory(msg);
	memset(msg, 0, msg_len);
	for(i = 0; i < n; i++)
	{
		for(j = 0; j < n; j++)
		{
			sprintf(buf, "%f ", A[i][j]);
			strcat(msg, buf);
		}
		strcat(msg, "\n");
	}

	printf(msg);

	/* release memory */
	free(buf);
	free(msg);
}

void Floyd_Warshall_Show_Predecessor_Table(int** A, int n)
{ //show the two-dimensional nxn predecessor matrix A
	int i, j;
	char *msg = NULL;
	char *buf = NULL;
	//char msg[MSG_BUF_SIZE]; 
	//Where n is a large number, such as 18, the memory access violation happens.
	//So we need to dynamically allocate the memory for msg and buf according to n.
	//char buf[BUF_SIZE];
	const int number_string_length = NUMBER_STRING_LEN; //length of number string
	int buf_len; //buf length
	int msg_len; //msg length

	buf_len = sizeof(char)*n*number_string_length;
	//buf = (char*) malloc(buf_len);
	buf = (char*) calloc(buf_len, sizeof(char));
	assert_memory(buf);

	msg_len = sizeof(char)*n*n*number_string_length;
	//msg = (char*) malloc(msg_len);
	msg = (char*) calloc(msg_len, sizeof(char));
	assert_memory(msg);
	memset(msg, 0, msg_len);
	for(i = 0; i < n; i++)
	{
		for(j = 0; j < n; j++)
		{
			sprintf(buf, "%d ", A[i][j]);
			strcat(msg, buf);
		}
		strcat(msg, "\n");
	}

	printf(msg);

	/* release memory */
	free(buf);
	free(msg);
}

void Floyd_Warshall_Store_Weight_Table_Into_File(double** A, int n, char* filename)
{ //store the two-dimensional nxn weight matrix A into a file called filename
	FILE *fp; //path-table file
	int i, j;
	char *msg = NULL;
	char *buf = NULL;
	//char msg[MSG_BUF_SIZE]; 
	//Where n is a large number, such as 18, the memory access violation happens.
	//So we need to dynamically allocate the memory for msg and buf according to n.
	//char buf[BUF_SIZE];
	const int number_string_length = NUMBER_STRING_LEN; //length of number string
	int buf_len; //buf length
	int msg_len; //msg length

	buf_len = sizeof(char)*n*number_string_length;
	//buf = (char*) malloc(buf_len);
	buf = (char*) calloc(buf_len, sizeof(char));
	assert_memory(buf);

	msg_len = sizeof(char)*n*n*number_string_length;
	//msg = (char*) malloc(msg_len);
	msg = (char*) calloc(msg_len, sizeof(char));
	assert_memory(msg);
	memset(msg, 0, msg_len);

	for(i = 0; i < n; i++)
	{
		for(j = 0; j < n; j++)
		{
			sprintf(buf, "%f ", A[i][j]);
			strcat(msg, buf);
		}
		strcat(msg, "\n");
	}

	fp = fopen(filename, "w");
	if(!fp)
	{
		fprintf(stderr, "Error: unable to open file \"%s\"\n", filename);
		exit(1);
	}

#ifdef __DEBUG_LEVEL_0__
	printf(msg);
#endif
	fprintf(fp, msg);
	fclose(fp);

	/* release memory */
	free(buf);
	free(msg); //if NUMBER_STRING_LEN is small for numbers, this could make heap error here.
}

void Floyd_Warshall_Store_Predecessor_Table_Into_File(int** A, int n, char* filename)
{ //store the two-dimensional nxn predecessor matrix A into a file called filename
	FILE *fp; //path-table file
	int i, j;
	char *msg = NULL;
	char *buf = NULL;
	//char msg[MSG_BUF_SIZE]; 
	//Where n is a large number, such as 18, the memory access violation happens.
	//So we need to dynamically allocate the memory for msg and buf according to n.
	//char buf[BUF_SIZE];
	const int number_string_length = NUMBER_STRING_LEN; //length of number string
	int buf_len; //buf length
	int msg_len; //msg length

	buf_len = sizeof(char)*n*number_string_length;
	//buf = (char*) malloc(buf_len);
	buf = (char*) calloc(buf_len, sizeof(char));
	assert_memory(buf);

	msg_len = sizeof(char)*n*n*number_string_length;
	//msg = (char*) malloc(msg_len);
	msg = (char*) calloc(msg_len, sizeof(char));
	assert_memory(msg);
	memset(msg, 0, msg_len);

	for(i = 0; i < n; i++)
	{
		for(j = 0; j < n; j++)
		{
			sprintf(buf, "%d ", A[i][j]);
			strcat(msg, buf);
		}
		strcat(msg, "\n");
	}

	fp = fopen(filename, "w");
	if(!fp)
	{
		fprintf(stderr, "Error: unable to open file \"%s\"\n", filename);
		exit(1);
	}

#ifdef __DEBUG_LEVEL_0__
	printf(msg);
#endif
	fprintf(fp, msg);
	fclose(fp);

	/* release memory */
	free(buf);
	free(msg);
}

/** Operations for Trajectory-based Multi-Anycast (TMA) */
void Floyd_Warshall_Make_Weight_Matrix_For_EDC_With_PacketForwardingTreePathQueue(double **D, double **S, int matrix_size, packet_forwarding_tree_path_queue_t *PFTPQ)
{ /* make the adjacency matrix D for delivery delay and the supplementary matrix S for delivery cost with packet forwarding tree path queue PFTPQ */
	int i = 0, j = 0; //for-loop indices
	int u = 0; //tail node
	int v = 0; //head node
	packet_forwarding_tree_path_queue_node_t *pPFTP_QueueNode = NULL; //pointer to a packet forwarding tree path queue node
	packet_trajectory_queue_node_t *pPT_QueueNode = NULL; //pointer to a packet trajectory queue node

	/* initialize the weight matrices D and S */
	for(i = 0; i < matrix_size; i++)
	{
		D[i][i] = 0;
		S[i][i] = 0;
		for(j = i+1; j < matrix_size; j++)
		{
			D[i][j] = INF;
			D[j][i] = INF;

			S[i][j] = INF;
			S[j][i] = INF;
		}
	}

	/* make adjacency matrices D and S with packet forwarding tree path queue PFTPQ */
	pPFTP_QueueNode = &(PFTPQ->head);	
	for(i = 0; i < PFTPQ->size; i++)
	{
		pPFTP_QueueNode = pPFTP_QueueNode->next;		
		pPT_QueueNode = &(pPFTP_QueueNode->PTQ.head);
		for(j = 0; j < pPFTP_QueueNode->PTQ.size; j++)
		{
			pPT_QueueNode = pPT_QueueNode->next;
			if(j == 0)
			{ //u is the first intersection, so there is no previous intersection
				u = pPT_QueueNode->intersection_id;

				/* check the validity of u */
				if(u > matrix_size)
				{
				  printf("Floyd_Warshall_Make_Weight_Matrix_For_EDC_With_PacketForwardingTreePathQueue(): u(%d) is greater than matrix_size(%d)", u, matrix_size);
				  exit(1);
				}

				continue;
			}
			else
			{
				v = pPT_QueueNode->intersection_id;

				/* check the validity of v */
				if(v > matrix_size)
				{
				  printf("Floyd_Warshall_Make_Weight_Matrix_For_EDC_With_PacketForwardingTreePathQueue(): v(%d) is greater than matrix_size(%d)", v, matrix_size);
				  exit(1);
				}
			}
			
			/* set the edge_cost and edge_cost_variance to D[u-1][v-1] and S[u-1][v-1] */
			D[u-1][v-1] = pPT_QueueNode->edge_cost;
			S[u-1][v-1] = pPT_QueueNode->edge_cost_variance;

			/* let the tail node v changed to the head node u */
			u = v;
		}
	}
}

void Floyd_Warshall_Make_Weight_Matrix_For_EDD_With_AdjacencyMatrix(double **D, double **S, int D_size, double **T, parameter_t *param)
{ /* make the adjacency matrices D and S for EDD with adjacency matrix T
   * Note that we assume that the entry with a finite value (i.e., delivery cost) 
   * in T has a finite value (i.e., delivery delay and delivery delay variance) 
   * in both D and S */
	int i, j;
	double **Dr = param->vanet_table.Dr_edd; //delivery-delay matrix
	double **Sr = param->vanet_table.Sr_edd; //delivery-delay-variance matrix
	int Dr_size = param->vanet_table.matrix_size_for_edd_in_Gr; //matrix size

	if(D_size != Dr_size)
	{
	  printf("Floyd_Warshall_Make_Weight_Matrix_For_EDD_With_AdjacencyMatrix(): D_size(%d) is not equal to Dr_size(%d)\n", D_size, Dr_size);
	  //exit(1);
	}

	/* initialize the weight matrices D and S */
	for(i = 0; i < D_size; i++)
	{
		D[i][i] = 0;
		S[i][i] = 0;
		for(j = i+1; j < D_size; j++)
		{
			/* check whether the edge i->j exists in the tree T */
			if(T[i][j] < INF)
			{
				D[i][j] = Dr[i][j];
				S[i][j] = Sr[i][j];
			}
			else
			{
				D[i][j] = INF;
				S[i][j] = INF;
			}

			/* check whether the edge j->i exists in the tree T */
			if(T[j][i] < INF)
			{
				D[j][i] = Dr[j][i];
				S[j][i] = Sr[j][i];
			}
			else
			{
				D[j][i] = INF;
				S[j][i] = INF;
			}
		}
	}
}

void Floyd_Warshall_Make_Weight_Matrix_For_EDC_With_AdjacencyMatrix(double **D, double **S, int D_size, double **T, parameter_t *param)
{ /* make the adjacency matrices D and S for EDC with adjacency matrix T
   * Note that we assume that the entry with a finite value (i.e., delivery cost) 
   * in T has a finite value (i.e., delivery cost and delivery cost variance) 
   * in both D and S */
	int i, j;
	double **Dr = param->vanet_table.Dr_edc; //delivery-cost matrix
	double **Sr = param->vanet_table.Sr_edc; //delivery-cost-variance matrix
	int Dr_size = param->vanet_table.matrix_size_for_edc_in_Gr; //matrix size

	if(D_size != Dr_size)
	{
	  printf("Floyd_Warshall_Make_Weight_Matrix_For_EDC_With_AdjacencyMatrix(): D_size(%d) is not equal to Dr_size(%d)\n", D_size, Dr_size);
	  //exit(1);
	}

	/* initialize the weight matrices D and S */
	for(i = 0; i < D_size; i++)
	{
		D[i][i] = 0;
		S[i][i] = 0;
		for(j = i+1; j < D_size; j++)
		{
			/* check whether the edge i->j exists in the tree T */
			if(T[i][j] < INF)
			{
				D[i][j] = Dr[i][j];
				S[i][j] = Sr[i][j];
			}
			else
			{
				D[i][j] = INF;
				S[i][j] = INF;
			}

			/* check whether the edge j->i exists in the tree T */
			if(T[j][i] < INF)
			{
				D[j][i] = Dr[j][i];
				S[j][i] = Sr[j][i];
			}
			else
			{
				D[j][i] = INF;
				S[j][i] = INF;
			}
		}
	}
}

void Floyd_Warshall_Copy_Matrix_Of_Type_Double(double **M_dst, double **M_src, int matrix_size)
{ /* copy the source matrix M_src into the destination matrix M_dst where the matrix data type is double and the matrix size is matrix_size */
	int i = 0, j = 0; //for-loop indices

	/* copy matrix M_src into matrix M_dst */
	for(i = 0; i < matrix_size; i++)
	{
		M_dst[i][i] = 0;
		for(j = i+1; j < matrix_size; j++)
		{
			M_dst[i][j] = M_src[i][j];
			M_dst[j][i] = M_src[j][i];
		}
	}
}

void Floyd_Warshall_Copy_Matrix_Of_Type_Int(int **M_dst, int **M_src, int matrix_size)
{ /* copy the source matrix M_src into the destination matrix M_dst where the matrix data type is int and the matrix size is matrix_size */
	int i = 0, j = 0; //for-loop indices

	/* copy matrix M_src into matrix M_dst */
	for(i = 0; i < matrix_size; i++)
	{
		M_dst[i][i] = 0;
		for(j = i+1; j < matrix_size; j++)
		{
			M_dst[i][j] = M_src[i][j];
			M_dst[j][i] = M_src[j][i];
		}
	}
}

double Floyd_Warshall_Sum_Matrix_Weights(double **W, int matrix_size)
{ /* sum the weights of the edges in matrix W of size matrix_size */
	int i = 0, j = 0; //for-loop indices
	double sum= 0; //sum of edge weights

	/* sum up the edge weights */
	for(i = 0; i < matrix_size; i++)
	{
		if(W[i][i] < INF)
		{
			sum += W[i][i];
		}
		
		for(j = i+1; j < matrix_size; j++)
		{
			if(W[i][j] < INF)
			{
				sum += W[i][j];
			}

			if(W[j][i] < INF)
			{
				sum += W[j][i];
			}
		}
	}

	return sum;
}

void Floyd_Warshall_Make_VirtualGraph_With_Graph_And_ComponentVertexQueues(struct_graph_node *G, int G_size, component_vertex_queue_t *CVQ_src, component_vertex_queue_t *CVQ_tp, double ***A, int A_size)
{ /* make virtual graph's adjacency matrix A with road network graph G where CVQ_src is the component vertex queue for source node of a superedge and CVQ_tp is the component vertex queue for target point of the superedge */

	int i = 0, j = 0; //graph array indices
	int w = 0; //vertex id
	struct_graph_node *ptr = NULL; //pointer to graph node
	int v1 = G_size + 1; //virtual node v1 for component vertex queue CVQ_src
	int v2 = G_size + 2; //virtual node v2 for component vertex queue CVQ_tp
	component_vertex_queue_node_t *pQueueNode = NULL; //pointer to a component vertex queue node

	if(*A == NULL)
	{
	  printf("Floyd_Warshall_Construct_VirtualGraph_With_Graph(): matrix *A must not be NULL!\n");
	  exit(1);
	}

	if(A_size < G_size)
	{
		printf("Floyd_Warshall_Construct_VirtualGraph_With_Graph_And_ComponentVertexQueues: A_size(%d) is less than G_size(%d)\n", A_size, G_size);
		exit(1);
	}

	/** initialize matrix A with the delivery costs of the edges in graph G */
	/* initialize the weight matrix A */
	for(i = 0; i < A_size; i++)
	{
		(*A)[i][i] = 0;
		for(j = i+1; j < A_size; j++)
		{
			(*A)[i][j] = INF;
			(*A)[j][i] = INF;
		}
	}

	/* make adjacency matrix A with adjacency list G */
	for(i = 0; i < G_size; i++)
	{
		ptr = G[i].next;
		if(ptr == NULL)
			continue;

		while(ptr != NULL)
		{
			j = atoi(ptr->vertex) - 1; 
			//node id starts from 1, but it should decrease by 1 for weight matrix where the id starts from 0.
			
			/* set the head_node's edge_cost to (*A)[i][j] */
			(*A)[i][j] = ptr->edge_cost;

			ptr = ptr->next;
		}
	}

	/* connect virtual node v1 to all vertices w in CVQ_src with weight 0 */
	pQueueNode = &(CVQ_src->head);
	for(i = 0; i < CVQ_src->size; i++)
	{
		pQueueNode = pQueueNode->next;
		w = pQueueNode->vertex;
		(*A)[v1-1][w-1] = 0;
	}

	/* connect all vertices w in CVQ_tp to virtual node v2 with weight 0 */
	pQueueNode = &(CVQ_tp->head);
	for(i = 0; i < CVQ_tp->size; i++)
	{
		pQueueNode = pQueueNode->next;
		w = pQueueNode->vertex;
		(*A)[w-1][v2-1] = 0;
	}
}

int** Floyd_Warshall_Make_Connectivity_Adjacency_Matrix_For_Movement(int **W, int W_size, struct_graph_node *G, int G_size)
{ /* make the connectivity adjacency matrix W for movement with graph G */
	int i, j;
	struct_graph_node *ptr = NULL; //pointer to graph node

	if(W_size < G_size)
	{
	  printf("Floyd_Warshall_Make_Connectivity_Adjacency_Matrix_For_Movement(): W_size(%d) is less than G_size(%d)\n", W_size, G_size);
	  exit(1);
	}

	/* initialize the adjacency matrix W */
	for(i = 0; i < G_size; i++)
	{
		//W[i][i] = 0;
		W[i][i] = 1; //node i is connected to itself
		for(j = i+1; j < G_size; j++)
		{
			W[i][j] = INF;
			W[j][i] = INF;
		}
	}

	/* make adjacency matrix W with adjacent list G */
	for(i = 0; i < G_size; i++)
	{
		ptr = G[i].next;
		if(ptr == NULL)
			continue;

		while(ptr != NULL)
		{
			j = atoi(ptr->vertex) - 1; 
			//node id starts from 1, but it should decrease by 1 for weight matrix where the id starts from 0.
			W[i][j] = 1;
			ptr = ptr->next;
  		}
	}

	return W;
}

int** Floyd_Warshall_Make_Connectivity_Adjacency_Matrix_For_EDD(int **W, int W_size, struct_graph_node *G, int G_size)
{ /* make the connectivity adjacency matrix W for EDD with graph G */
	int i, j;
	struct_graph_node *ptr = NULL; //pointer to graph node

	if(W_size < G_size)
	{
	  printf("Floyd_Warshall_Make_Connectivity_Adjacency_Matrix_For_EDD(): W_size(%d) is less than G_size(%d)\n", W_size, G_size);
	  exit(1);
	}

	/* initialize the adjacency matrix W */
	for(i = 0; i < G_size; i++)
	{
		//W[i][i] = 0;
		W[i][i] = 1; //node i is connected to itself
		for(j = i+1; j < G_size; j++)
		{
			W[i][j] = INF;
			W[j][i] = INF;
		}
	}

	/* make adjacency matrix W with adjacent list G */
	for(i = 0; i < G_size; i++)
	{
		ptr = G[i].next;
		if(ptr == NULL)
			continue;

		while(ptr != NULL)
		{
			j = atoi(ptr->vertex) - 1; 
			//node id starts from 1, but it should decrease by 1 for weight matrix where the id starts from 0.

			/* check whether both intersection i and intersection j have stationary nodes or not */
			if(G[i].stationary_node_flag && ptr->stationary_node_flag)
				W[i][j] = 1;

			ptr = ptr->next;
  		}
	}

	return W;
}

int** Floyd_Warshall_Init_Connectivity_Adjacency_Matrix_For_EDD(parameter_t *param, struct_graph_node *G, int G_size, struct_traffic_table *sn_table, int ***A, int A_size)
{ /* initialize the connectivity adjacency matrix A for road network graph G with stationary node list sn_table */
	int id = 0; //intersection id
	int i = 0; //index

	if(*A != NULL)
	{
		printf("Floyd_Warshall_Make_Connectivity_Adjacency_Matrix_For_EDD(): we cannot allocate the connectivity adjacency matrix A since A has already been allocated memory!\n");
		exit(1);
	}

	*A = Floyd_Warshall_Allocate_Matrix_Of_Type_Int(A_size);
	//allocate the memory of the adjacency whose rows and columns correspond to A_size

	/* initialize A with road network graph G */
	Floyd_Warshall_Make_Connectivity_Adjacency_Matrix_For_EDD(*A, A_size, G, G_size);

	return *A;
}

/**
 *  File: linear-algebra.c
    Description: implementation of Linear Algebraic Systems
    Date: 08/14/2008
    Update Date: 08/14/2008
    Maker: Jaehoon Jeong
*/

#include "stdafx.h"
#include "common.h" //boolean
#include "linear-algebra.h" //parameter structure and related constants
#include "util.h" //assert_memory()

void LA_Test_Gaussian_Elimination_1()
{ //Test-1: test Gaussian Elimination function and Backward Substitution function
  double **A = NULL; //n x (n+1) matrix containing n x n forwarding probability matrix and edge delay vector
  double *x = NULL; //solution vector of size row_size
  int *p = NULL; //permutation vector of size row_size
  int row_size = 5, column_size = 6; //row size and column size of a matrix
  int i = 0; //index of for-loop

  /** allocate the memory of the row_size x column_size matrix A of type double */
  A = LA_Allocate_Matrix_Of_Type_Double(row_size, column_size);

  /** allocate the memory for solution vector x */
  x = (double*)calloc(row_size, sizeof(double));
  assert_memory(x);

  /** allocate the memory for permutation vector p */
  p = (int*)calloc(row_size, sizeof(int));
  assert_memory(p);

  /** initialize the augmented matrix A */
  A[0][0]=2, A[0][1]=-1, A[0][2]=0, A[0][3]=0, A[0][4]=1, A[0][5]=0;
  A[1][0]=-1, A[1][1]=2, A[1][2]=-1, A[1][3]=0, A[1][4]=1, A[1][5]=1;
  A[2][0]=0, A[2][1]=-1, A[2][2]=2, A[2][3]=-1, A[2][4]=1, A[2][5]=1;
  A[3][0]=0, A[3][1]=0, A[3][2]=-1, A[3][3]=2, A[3][4]=1, A[3][5]=0;
  A[4][0]=1, A[4][1]=2, A[4][2]=3, A[4][3]=4, A[4][4]=5, A[4][5]=1;

  /** perform Gaussian elimination for nonsingular case of matrix A where A's column_size is equal to row_size+1, since matrix A is the augmented matrix A = (P | -d) where P is n x n matrix and d is a column vector */
  LA_Perform_Gaussian_Elimination_For_Nonsingular_Case(A, row_size, column_size, p);

  /** perform Backward substitution to obtain the solution x for the augmented matrix A where x is a column vector of dimension row_size */
  LA_Perform_Backward_Substitution(A, row_size, p, x);

  /** print the solution */
  for(i = 0; i < row_size; i++)
  {
    printf("x[%d]=%f\n", i, (float)x[i]);
  }
  printf("\n\n");

  /** free the memory for the matrix A */
  LA_Free_Matrix_Of_Type_Double(A, row_size);

  /** free the memory for solution vector x */
  free(x);

  /** free the memory for permutation vector p */
  free(p);
}

void LA_Test_Gaussian_Elimination_2()
{ //Test-2: test Gaussian Elimination function and Backward Substitution function
  double **A = NULL; //n x (n+1) matrix containing n x n forwarding probability matrix and edge delay vector
  double *x = NULL; //solution vector of size row_size
  int *p = NULL; //permutation vector of size row_size
  int row_size = 2, column_size = 3; //row size and column size of a matrix
  int i = 0; //index of for-loop

  /** allocate the memory of the row_size x column_size matrix A of type double */
  A = LA_Allocate_Matrix_Of_Type_Double(row_size, column_size);

  /** allocate the memory for solution vector x */
  x = (double*)calloc(row_size, sizeof(double));
  assert_memory(x);

  /** allocate the memory for permutation vector p */
  p = (int*)calloc(row_size, sizeof(int));
  assert_memory(p);

  /** initialize the augmented matrix A */
  A[0][0]=0.6, A[0][1]=1, A[0][2]=22;
  A[1][0]=1600, A[1][1]=10, A[1][2]=3210;

  /** perform Gaussian elimination for nonsingular case of matrix A where A's column_size is equal to row_size+1, since matrix A is the augmented matrix A = (P | -d) where P is n x n matrix and d is a column vector */
  LA_Perform_Gaussian_Elimination_For_Nonsingular_Case(A, row_size, column_size, p);

  /** perform Backward substitution to obtain the solution x for the augmented matrix A where x is a column vector of dimension row_size */
  LA_Perform_Backward_Substitution(A, row_size, p, x);

  /** print the solution */
  for(i = 0; i < row_size; i++)
  {
    printf("x[%d]=%f\n", i, (float)x[i]);
  }
  printf("\n\n");

  /** free the memory for the matrix A */
  LA_Free_Matrix_Of_Type_Double(A, row_size);

  /** free the memory for solution vector x */
  free(x);

  /** free the memory for permutation vector p */
  free(p);
}

void LA_Test_Gaussian_Elimination_3()
{ //Test-3: test Gaussian Elimination function and Backward Substitution function
  double **A = NULL; //n x (n+1) matrix containing n x n forwarding probability matrix and edge delay vector
  double *x = NULL; //solution vector of size row_size
  int *p = NULL; //permutation vector of size row_size
  int row_size = 2, column_size = 3; //row size and column size of a matrix
  int i = 0; //index of for-loop

  /** allocate the memory of the row_size x column_size matrix A of type double */
  A = LA_Allocate_Matrix_Of_Type_Double(row_size, column_size);

  /** allocate the memory for solution vector x */
  x = (double*)calloc(row_size, sizeof(double));
  assert_memory(x);

  /** allocate the memory for permutation vector p */
  p = (int*)calloc(row_size, sizeof(int));
  assert_memory(p);

  /** initialize the augmented matrix A */
  A[0][0]=10, A[0][1]=1600, A[0][2]=3210;
  A[1][0]=1, A[1][1]=0.6, A[1][2]=22;

  /** perform Gaussian elimination for nonsingular case of matrix A where A's column_size is equal to row_size+1, since matrix A is the augmented matrix A = (P | -d) where P is n x n matrix and d is a column vector */
  LA_Perform_Gaussian_Elimination_For_Nonsingular_Case(A, row_size, column_size, p);

  /** perform Backward substitution to obtain the solution x for the augmented matrix A where x is a column vector of dimension row_size */
  LA_Perform_Backward_Substitution(A, row_size, p, x);

  /** print the solution */
  for(i = 0; i < row_size; i++)
  {
    printf("x[%d]=%f\n", i, (float)x[i]);
  }
  printf("\n\n");

  /** free the memory for the matrix A */
  LA_Free_Matrix_Of_Type_Double(A, row_size);

  /** free the memory for solution vector x */
  free(x);

  /** free the memory for permutation vector p */
  free(p);
}

void LA_Test_Gaussian_Elimination_4()
{ //Test-1: test Gaussian Elimination function and Backward Substitution function
  double **A = NULL; //n x (n+1) matrix containing n x n forwarding probability matrix and edge delay vector
  double *x = NULL; //solution vector of size row_size
  int *p = NULL; //permutation vector of size row_size
  int row_size = 3, column_size = 4; //row size and column size of a matrix
  int i = 0; //index of for-loop

  /** allocate the memory of the row_size x column_size matrix A of type double */
  A = LA_Allocate_Matrix_Of_Type_Double(row_size, column_size);

  /** allocate the memory for solution vector x */
  x = (double*)calloc(row_size, sizeof(double));
  assert_memory(x);

  /** allocate the memory for permutation vector p */
  p = (int*)calloc(row_size, sizeof(int));
  assert_memory(p);

  /** initialize the augmented matrix A */
  A[0][0]=1, A[0][1]=2, A[0][2]=1, A[0][3]=2;
  A[1][0]=2, A[1][1]=6, A[1][2]=1, A[1][3]=7;
  A[2][0]=1, A[2][1]=1, A[2][2]=4, A[2][3]=3;

  /** perform Gaussian elimination for nonsingular case of matrix A where A's column_size is equal to row_size+1, since matrix A is the augmented matrix A = (P | -d) where P is n x n matrix and d is a column vector */
  LA_Perform_Gaussian_Elimination_For_Nonsingular_Case(A, row_size, column_size, p);

  /** perform Backward substitution to obtain the solution x for the augmented matrix A where x is a column vector of dimension row_size */
  LA_Perform_Backward_Substitution(A, row_size, p, x);

  /** print the solution */
  for(i = 0; i < row_size; i++)
  {
    printf("x[%d]=%f\n", i, (float)x[i]);
  }
  printf("\n\n");

  /** free the memory for the matrix A */
  LA_Free_Matrix_Of_Type_Double(A, row_size);

  /** free the memory for solution vector x */
  free(x);

  /** free the memory for permutation vector p */
  free(p);
}

void LA_Test_Gaussian_Elimination_5()
{ //Test-1: test Gaussian Elimination function and Backward Substitution function
  double **A = NULL; //n x (n+1) matrix containing n x n forwarding probability matrix and edge delay vector
  double *x = NULL; //solution vector of size row_size
  int *p = NULL; //permutation vector of size row_size
  int row_size = 4, column_size = 5; //row size and column size of a matrix
  int i = 0; //index of for-loop

  /** allocate the memory of the row_size x column_size matrix A of type double */
  A = LA_Allocate_Matrix_Of_Type_Double(row_size, column_size);

  /** allocate the memory for solution vector x */
  x = (double*)calloc(row_size, sizeof(double));
  assert_memory(x);

  /** allocate the memory for permutation vector p */
  p = (int*)calloc(row_size, sizeof(int));
  assert_memory(p);

  /** initialize the augmented matrix A */
  A[0][0]=1, A[0][1]=2, A[0][2]=-1, A[0][3]=0, A[0][4]=1;
  A[1][0]=2, A[1][1]=4, A[1][2]=-2, A[1][3]=-1, A[1][4]=-1;
  A[2][0]=-3, A[2][1]=-5, A[2][2]=6, A[2][3]=1, A[2][4]=3;
  A[3][0]=-1, A[3][1]=2, A[3][2]=8, A[3][3]=-2, A[3][4]=0;

  /** perform Gaussian elimination for nonsingular case of matrix A where A's column_size is equal to row_size+1, since matrix A is the augmented matrix A = (P | -d) where P is n x n matrix and d is a column vector */
  LA_Perform_Gaussian_Elimination_For_Nonsingular_Case(A, row_size, column_size, p);

  /** perform Backward substitution to obtain the solution x for the augmented matrix A where x is a column vector of dimension row_size */
  LA_Perform_Backward_Substitution(A, row_size, p, x);

  /** print the solution */
  for(i = 0; i < row_size; i++)
  {
    printf("x[%d]=%f\n", i, (float)x[i]);
  }
  printf("\n\n");

  /** free the memory for the matrix A */
  LA_Free_Matrix_Of_Type_Double(A, row_size);

  /** free the memory for solution vector x */
  free(x);

  /** free the memory for permutation vector p */
  free(p);
}

double** LA_Allocate_Matrix_Of_Type_Double(int row_size, int column_size)
{ //allocate the memory of row_size x column_size matrix of type double
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

void LA_Reset_Matrix_Of_Type_Double(double **A, int row_size, int column_size)
{ //reset the memory of row_size x column_size matrix A of type double
  int i;

  for(i = 0; i < row_size; i++)
  {
    memset(A[i], 0, column_size*sizeof(double));
  }
}

void LA_Free_Matrix_Of_Type_Double(double **A, int row_size)
{ //free the memory for 2-dimensional martix A of type double with dimension row_size x column_size
  int i;
	
  for(i = 0; i < row_size; i++)
    free(A[i]);

  free(A);
}

void LA_Perform_Gaussian_Elimination_For_Nonsingular_Case(double **A, int row_size, int column_size, int *p)
{ //perform Gaussian elimination for nonsingular case of matrix A where A's column_size is equal to row_size+1, since matrix A is the augmented matrix A = (P | -d) where P is n x n matrix and d is a column vector, returning permutation vector p
  int i = 0, j = 0, k = 0; //indices for for-loops
  int n = row_size; //n is row_size
  boolean flag_nonsingular = FALSE; //flag for checking the nonsigularity of matrix A
  double c = 0; //multiple coefficient for variable elimination
  int temp = 0; //temporary variable for value exchange
  double absolute_value = 0; //absolute value
  double max_absolute_value = 0; //maximum absolute value of the pivot candidate
  int max_absolute_value_index = 0; //index for max_absolute_value

  /** initialize permutation vector p */
  for(i = 0; i < n; i++)
    p[i] = i;
  
  /** perform Gaussian elimination */
  for(j = 0; j < n; j++) //for-1: j is column index
  {
    /** check the singularity of matrix A */
    flag_nonsingular = FALSE;
    max_absolute_value = 0;
    for(i = j; i < n; i++) //for all i >= j
    {
      absolute_value = fabs(A[p[i]][j]);
      if(absolute_value > LA_ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
      {
	flag_nonsingular = TRUE;
	if(absolute_value > max_absolute_value)
        {
            max_absolute_value = absolute_value;
            max_absolute_value_index = i;
        }
      }
    }

    if(flag_nonsingular == FALSE)
    {
      printf("LA_Perform_Gaussian_Elimination_For_Nonsingular_Case(): matrix A is singular!\n");
      exit(1);
    }
    else
    {
      i = max_absolute_value_index; //partial pivoting
      if(i != j) //in the case where A[i][i] == 0
      //if(p[i] != p[j]) //in the case where A[i][i] == 0
      {
	/* exchange rows p[i] and p[j] */
	temp = p[i];
	p[i] = p[j];
	p[j] = temp;
      }
    }

    /** elimination of column variables below pivot */
    for(i = j+1; i < n; i++) //for-2
    {
      c = A[p[i]][j]/A[p[j]][j];
      for(k = j; k < n+1; k++) //for-3
      //for(k = j+1; k < n+1; k++) //for-3
      {
	A[p[i]][k] += -c*A[p[j]][k];
      } //end of for-3
    } //end of for-2
  } //end of for-1

#ifdef __DEBUG_LEVEL_GAUSSIAN_ELIMINATION__
  /*@for debugging */
  for(i = 0; i < n; i++)
  {
      for(j = 0; j <= n; j++)
      {
          printf("A[%d][%d]=%.2f ", i, j, A[i][j]);
      }
      printf("\n");
  }
  printf("\n");

  for(i = 0; i < n; i++)
  {
      for(j = 0; j <= n; j++)
      {
          printf("A[p[%d]][%d]=A[%d][%d]=%.2f ", i, j, p[i], j, A[p[i]][j]);
      }
      printf("\n");
  }
  /*****************/
#endif
}

void LA_Perform_Backward_Substitution(double **A, int row_size, int *p, double *x)
{ //perform Backward substitution to obtain the solution x for the augmented matrix A along with permutation vector p where x is a column vector of dimension row_size
  int i = 0, j = 0, k = 0; //indices for for-loops
  int n = row_size; //n is row_size
  double sum = 0; //sum of products of A[i][j]*x[j]

  for(i = n-1; i >= 0; i--) //for-1
  {
    sum = 0;
    for(j = i+1; j < n; j++) //for-2
    {
      sum += A[p[i]][j]*x[j];
      //sum += A[i][j]*x[j];
    } //end of for-2

    x[i] = 1/A[p[i]][i]*(A[p[i]][n] - sum);
    //x[i] = 1/A[i][i]*(A[i][n] - sum);
  } //end of for-1
}

void LA_Log_LinearSystem_And_Solution(char *filename, double **A, int row_size,  double *x)
{ //log the linear system of augmented matrix A and its solution vector x
    char filename_buf[BUF_SIZE]; //log file for linear system
    FILE *fp = NULL; //file pointer
    int i = 0, j = 0; //for-loop indices

    if(filename == NULL)
    {
        filename = filename_buf;
#ifdef _LINUX_
        strcpy(filename, "output/output");
#else
        strcpy(filename, "output\\output");
#endif
        strcat(filename, ".lis");
    }

    /* open log file */
    fp = fopen(filename, "w");
    if(!fp)
    {
        fprintf(stderr, "LA_Log_LinearSystem_And_Solution(): Error: unable to open file \"%s\"\n", filename);
	exit(1);
    }

    /* log the row_size */
    fprintf(fp, "%d\n\n", row_size);

    /* log the linear system A */
    for(i = 0; i < row_size; i++)
    {
        for(j = 0; j <= row_size; j++)
        {
            fprintf(fp, "%f ", A[i][j]);
        }
        fprintf(fp, "\n");
    }
    fprintf(fp, "\n");

    /* log the solution vector x */
    for(i = 0; i < row_size; i++)
    {
        fprintf(fp, "%f ", x[i]);
    }

    /* close the log file */
    fclose(fp);
}

/**  File: linear-algebra.h
    Description: implementation of Linear Algebraic Systems
    Date: 08/14/2008
    Update Date: 08/14/2008
    Maker: Jaehoon Jeong
*/

#ifndef __LINEAR_ALGEBRA_H__
#define __LINEAR_ALGEBRA_H__

/** include header files */

/** macro constants */
#define LA_ERROR_TOLERANCE_FOR_REAL_ARITHMETIC 0.0000001
//error tolerance for real number arithmetic for the equality comparison of two real numbers: 10^-7

/** definition of structures */


/** declaration of functions */

void LA_Test_Gaussian_Elimination_1();
//Test-1: test Gaussian Elimination function and Backward Substitution function

void LA_Test_Gaussian_Elimination_2();
//Test-2: test Gaussian Elimination function and Backward Substitution function

void LA_Test_Gaussian_Elimination_3();
//Test-3: test Gaussian Elimination function and Backward Substitution function

void LA_Test_Gaussian_Elimination_4();
//Test-4: test Gaussian Elimination function and Backward Substitution function

void LA_Test_Gaussian_Elimination_5();
//Test-5: test Gaussian Elimination function and Backward Substitution function

double** LA_Allocate_Matrix_Of_Type_Double(int row_size, int column_size);
//allocate the memory of row_size x column_size matrix of type double

void LA_Reset_Matrix_Of_Type_Double(double **A, int row_size, int column_size);
//reset the memory of row_size x column_size matrix A of type double

void LA_Free_Matrix_Of_Type_Double(double **A, int row_size);
//free the memory for 2-dimensional martix A of type double with dimension row_size x column_size

void LA_Perform_Gaussian_Elimination_For_Nonsingular_Case(double **A, int row_size, int column_size, int *p);
//perform Gaussian elimination for nonsingular case of matrix A where A's column_size is equal to row_size+1, since matrix A is the augmented matrix A = (P | -d) where P is n x n matrix and d is a column vector, returning permutation vector p

void LA_Perform_Backward_Substitution(double **A, int row_size, int *p, double *x);
//perform Backward substitution to obtain the solution x for the augmented matrix A along with permutation vector p where x is a column vector of dimension row_size

void LA_Log_LinearSystem_And_Solution(char *filename, double **A, int row_size,  double *x);
//log the linear system of augmented matrix A and its solution vector x

#endif







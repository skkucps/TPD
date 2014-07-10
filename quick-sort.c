/**
 *  File: quick-sort.c
 *	Description: Quick sort algorithm
 *	Date: 09/13/2007	
 *	Maker: Jaehoon Jeong
 */

#include "stdafx.h"
#include "quick-sort.h"


/* Test of Quick Sort

	int B[8] = {5, 3, 2, 6, 4, 1, 5, 7};
	//int B[8] = {5, 3, 2, 6, 4, 1, 3, 7};

	//perform quick sort
	QuickSort(B, 0, 7);

*/

void QuickSort(int *A, int p, int r)
{ //perform quick sort
	int q; //index for array A

	if(p < r)
	{
		q = Partition(A, p, r);
		QuickSort(A, p, q);
		QuickSort(A, q+1, r);
	}
}

static int Partition(int *A, int p, int r)
{ //find an index to partition the array A for further quick sort
	int x; //pivot for partitioning
	int i, j; //indicies for array A

	x = A[p]; 
	//x is pivot for the partition of A such that the elements less than x
	//are placed at the left side of A and the elements greater than x are
	//placed at the right side of A.

	i = p - 1;
	j = r + 1;

	while(1)
	{
		do
		{
			j = j - 1; //decrease index j; that is, move j to the left			
		} while(A[j] > x);

		do
		{
			i = i + 1; //increase index i; that is, move i to the right			
		} while(A[i] < x);

		if(i < j)
		{
			if(A[i] > A[j])
				ExchangeElements(&A[i], &A[j]); //exchange the array elements of A[i] and A[j]
		}
		else
		{
			return j;
		}
	}
}

static void ExchangeElements(int *ptr1, int *ptr2)
{ //exchange the array elements of *ptr1 (i.e., A[i]) and * ptr2 (i.e., A[j])
	int temp;

	temp = *ptr1;
	*ptr1 = *ptr2;
	*ptr2 = temp;
}

/** Quick Sort for Sensor Array of type sensor_queue_node_t */
void QuickSortForSensorArray(sensor_queue_node_t **A, int p, int r)
{ //perform quick sort for sensor array
	int q; //index for array A

	if(p < r)
	{
		q = PartitionForSensorArray(A, p, r);
		QuickSortForSensorArray(A, p, q);
		QuickSortForSensorArray(A, q+1, r);
	}
}

static int PartitionForSensorArray(sensor_queue_node_t **A, int p, int r)
{ //find an index to partition the array A for further quick sort for sensor array
	double x; //pivot for partitioning
	int i, j; //indicies for array A

	x = A[p]->info.pos_in_Gv.offset; 
	//x is pivot for the partition of A such that the elements less than x
	//are placed at the left side of A and the elements greater than x are
	//placed at the right side of A.

	i = p - 1;
	j = r + 1;

	while(1)
	{
		do
		{
			j = j - 1; //decrease index j; that is, move j to the left			
		} while(A[j]->info.pos_in_Gv.offset > x);

		do
		{
			i = i + 1; //increase index i; that is, move i to the right			
		} while(A[i]->info.pos_in_Gv.offset < x);

		if(i < j)
		{
			if(A[i]->info.pos_in_Gv.offset > A[j]->info.pos_in_Gv.offset)
				ExchangeElementsForSensorArray(&A[i], &A[j]); //exchange the array elements of A[i] and A[j]
		}
		else
		{
			return j;
		}
	}
}

static void ExchangeElementsForSensorArray(sensor_queue_node_t **ptr1, sensor_queue_node_t **ptr2)
{ //exchange the sensor array elements of *ptr1 (i.e., A[i]) and *ptr2 (i.e., A[j])
	sensor_queue_node_t *temp;

	temp = *ptr1;
	*ptr1 = *ptr2;
	*ptr2 = temp;
}

/////////////////////////////////////////////////////////

/** Quick Sort for Edge_Set Array of type edge_set_queue_node_t */
void QuickSortForEdgeSetArray(edge_set_queue_node_t **A, int p, int r)
{ //perform quick sort for edge_set array
	int q; //index for array A

	if(p < r)
	{
		q = PartitionForEdgeSetArray(A, p, r);
		QuickSortForEdgeSetArray(A, p, q);
		QuickSortForEdgeSetArray(A, q+1, r);
	}
}

static int PartitionForEdgeSetArray(edge_set_queue_node_t **A, int p, int r)
{ //find an index to partition the array A for further quick sort for edge_set array
	double x; //pivot for partitioning
	int i, j; //indicies for array A

	x = A[p]->weight; 
	//x is pivot for the partition of A such that the elements less than x
	//are placed at the left side of A and the elements greater than x are
	//placed at the right side of A.

	i = p - 1;
	j = r + 1;

	while(1)
	{
		do
		{
			j = j - 1; //decrease index j; that is, move j to the left			
		} while(A[j]->weight > x);

		do
		{
			i = i + 1; //increase index i; that is, move i to the right			
		} while(A[i]->weight < x);

		if(i < j)
		{
			if(A[i]->weight > A[j]->weight)
				ExchangeElementsForEdgeSetArray(&A[i], &A[j]); //exchange the array elements of A[i] and A[j]
		}
		else
		{
			return j;
		}
	}
}

static void ExchangeElementsForEdgeSetArray(edge_set_queue_node_t **ptr1, edge_set_queue_node_t **ptr2)
{ //exchange the edge_set array elements of *ptr1 (i.e., A[i]) and *ptr2 (i.e., A[j])
	edge_set_queue_node_t *temp;

	temp = *ptr1;
	*ptr1 = *ptr2;
	*ptr2 = temp;
}

/////////////////////////////////////////////////////////

/** Quick Sort for Sensing Hole Endpoint Array of type hole_endpoint_queue_node_t */
void QuickSortForSensingHoleEndpointArray(hole_endpoint_queue_node_t **A, int p, int r)
{ //perform quick sort for sensing hole endpoint array
	int q; //index for array A

	if(p < r)
	{
		q = PartitionForSensingHoleEndpointArray(A, p, r);
		QuickSortForSensingHoleEndpointArray(A, p, q);
		QuickSortForSensingHoleEndpointArray(A, q+1, r);
	}
}

static int PartitionForSensingHoleEndpointArray(hole_endpoint_queue_node_t **A, int p, int r)
{ //find an index to partition the array A for further quick sort for sensing hole endpoint array
	double x; //pivot for partitioning
	int i, j; //indicies for array A

	x = A[p]->offset; 
	//x is pivot for the partition of A such that the elements less than x
	//are placed at the left side of A and the elements greater than x are
	//placed at the right side of A.

	i = p - 1;
	j = r + 1;

	while(1)
	{
		do
		{
			j = j - 1; //decrease index j; that is, move j to the left			
		} while(A[j]->offset > x);

		do
		{
			i = i + 1; //increase index i; that is, move i to the right			
		} while(A[i]->offset < x);

		if(i < j)
		{
			if(A[i]->offset > A[j]->offset)
				ExchangeElementsForSensingHoleEndpointArray(&A[i], &A[j]); //exchange the array elements of A[i] and A[j]
		}
		else
		{
			return j;
		}
	}
}

static void ExchangeElementsForSensingHoleEndpointArray(hole_endpoint_queue_node_t **ptr1, hole_endpoint_queue_node_t **ptr2)
{ //exchange the sensing hole endpoint array elements of *ptr1 (i.e., A[i]) and *ptr2 (i.e., A[j])
	hole_endpoint_queue_node_t *temp;

	temp = *ptr1;
	*ptr1 = *ptr2;
	*ptr2 = temp;
}

/////////////////////////////////////////////////////////

/** Quick Sort for Angle Array of type angle_queue_node_t */
void QuickSortForAngleArray(angle_queue_node_t **A, int p, int r)
{ //perform quick sort for angle array
	int q; //index for array A

	if(p < r)
	{
		q = PartitionForAngleArray(A, p, r);
		QuickSortForAngleArray(A, p, q);
		QuickSortForAngleArray(A, q+1, r);
	}
}

static int PartitionForAngleArray(angle_queue_node_t **A, int p, int r)
{ //find an index to partition the array A for further quick sort for angle array
	double x; //the primary pivot for partitioning
	double y; //the secondary pivot for partitioning
	int i, j; //indicies for array A

	x = A[p]->theta; 
	y = A[p]->CP;
	//Tuple (x,y) is the combination pivot for the partition of A such that the elements whose theta are 
        //less than x and whose CP are greater than y are placed at the left side of A and the elements 
        //whose theta are greater than x and whose CP are less than y are placed at the right side of A.
	

	i = p - 1;
	j = r + 1;

	while(1)
	{
		do
		{
			j = j - 1; //decrease index j; that is, move j to the left
		} while(!((A[j]->theta < x) || ((A[j]->theta == x) && (A[j]->CP >= y))));
		//} while(A[j]->theta > x);

		do
		{
			i = i + 1; //increase index i; that is, move i to the right			
		} while(!((A[i]->theta > x) || ((A[i]->theta == x) && (A[i]->CP <= y))));
		//} while(A[i]->theta < x);

		if(i < j)
		{
		        if((A[i]->theta > A[j]->theta) || ((A[i]->theta == A[j]->theta) && (A[i]->CP < A[j]->CP)))
		        //if((A[i]->theta >= A[j]->theta) && (A[i]->CP < A[j]->CP))
		        //if(A[i]->theta > A[j]->theta)
				ExchangeElementsForAngleArray(&A[i], &A[j]); //exchange the array elements of A[i] and A[j]
		}
		else
		{
			return j;
		}
	}
}

static void ExchangeElementsForAngleArray(angle_queue_node_t **ptr1, angle_queue_node_t **ptr2)
{ //exchange the angle array elements of *ptr1 (i.e., A[i]) and *ptr2 (i.e., A[j])
	angle_queue_node_t *temp;

	temp = *ptr1;
	*ptr1 = *ptr2;
	*ptr2 = temp;
}

/////////////////////////////////////////////////////////

/** Quick Sort for Intersection EDD Array of type intersection_edd_queue_node_t */
void QuickSortForIntersection_EDD_Array(intersection_edd_queue_node_t **A, int p, int r)
{ //perform quick sort for intersection EDD array
	int q; //index for array A

	if(p < r)
	{
		q = PartitionForIntersection_EDD_Array(A, p, r);
		QuickSortForIntersection_EDD_Array(A, p, q);
		QuickSortForIntersection_EDD_Array(A, q+1, r);
	}
}

static int PartitionForIntersection_EDD_Array(intersection_edd_queue_node_t **A, int p, int r)
{ //find an index to partition the array A for further quick sort for intersection EDD array
	double x; //pivot for partitioning
	int i, j; //indicies for array A

	x = A[p]->EDD; 
	//x is pivot for the partition of A such that the elements less than x
	//are placed at the left side of A and the elements greater than x are
	//placed at the right side of A.

	i = p - 1;
	j = r + 1;

	while(1)
	{
		do
		{
			j = j - 1; //decrease index j; that is, move j to the left			
		} while(A[j]->EDD > x);

		do
		{
			i = i + 1; //increase index i; that is, move i to the right			
		} while(A[i]->EDD < x);

		if(i < j)
		{
			if(A[i]->EDD > A[j]->EDD)
				ExchangeElementsForIntersection_EDD_Array(&A[i], &A[j]); //exchange the array elements of A[i] and A[j]
		}
		else
		{
			return j;
		}
	}
}

static void ExchangeElementsForIntersection_EDD_Array(intersection_edd_queue_node_t **ptr1, intersection_edd_queue_node_t **ptr2)
{ //exchange the intersection EDD array elements of *ptr1 (i.e., A[i]) and *ptr2 (i.e., A[j])
	intersection_edd_queue_node_t *temp;

	temp = *ptr1;
	*ptr1 = *ptr2;
	*ptr2 = temp;
}

/////////////////////////////////////////////////////////

/** Quick Sort for VehicleMovement Array of type vehicle_movement_queue_node_t */
void QuickSortForVehicleMovementArray(vehicle_movement_queue_node_t **A, int p, int r)
{ //perform quick sort for vehicle_movement array
	int q; //index for array A

	if(p < r)
	{
		q = PartitionForVehicleMovementArray(A, p, r);
		QuickSortForVehicleMovementArray(A, p, q);
		QuickSortForVehicleMovementArray(A, q+1, r);
	}
}

static int PartitionForVehicleMovementArray(vehicle_movement_queue_node_t **A, int p, int r)
{ //find an index to partition the array A for further quick sort for vehicle_movement array
	double x; //pivot for partitioning
	int i, j; //indicies for array A

	x = A[p]->offset; 
	//x is pivot for the partition of A such that the elements less than x
	//are placed at the left side of A and the elements greater than x are
	//placed at the right side of A.

	i = p - 1;
	j = r + 1;

	while(1)
	{
		do
		{
			j = j - 1; //decrease index j; that is, move j to the left			
		} while(A[j]->offset > x);

		do
		{
			i = i + 1; //increase index i; that is, move i to the right			
		} while(A[i]->offset < x);

		if(i < j)
		{
			if(A[i]->offset > A[j]->offset)
				ExchangeElementsForVehicleMovementArray(&A[i], &A[j]); //exchange the array elements of A[i] and A[j]
		}
		else
		{
			return j;
		}
	}
}

static void ExchangeElementsForVehicleMovementArray(vehicle_movement_queue_node_t **ptr1, vehicle_movement_queue_node_t **ptr2)
{ //exchange the vehicle_movement array elements of *ptr1 (i.e., A[i]) and *ptr2 (i.e., A[j])
	vehicle_movement_queue_node_t *temp;

	temp = *ptr1;
	*ptr1 = *ptr2;
	*ptr2 = temp;
}

/////////////////////////////////////////////////////////

/** Quick Sort for Vehicle Array of type vehicle_queue_node_t */
void QuickSortForVehicleArray(vehicle_queue_node_t **A, int p, int r)
{ //perform quick sort for vehicle array
	int q; //index for array A

	if(p < r)
	{
		q = PartitionForVehicleArray(A, p, r);
		QuickSortForVehicleArray(A, p, q);
		QuickSortForVehicleArray(A, q+1, r);
	}
}

static int PartitionForVehicleArray(vehicle_queue_node_t **A, int p, int r)
{ //find an index to partition the array A for further quick sort for vehicle array
	double x; //pivot for partitioning
	int i, j; //indicies for array A

	x = A[p]->offset; 
	//x is pivot for the partition of A such that the elements less than x
	//are placed at the left side of A and the elements greater than x are
	//placed at the right side of A.

	i = p - 1;
	j = r + 1;

	while(1)
	{
		do
		{
			j = j - 1; //decrease index j; that is, move j to the left			
		} while(A[j]->offset > x);

		do
		{
			i = i + 1; //increase index i; that is, move i to the right			
		} while(A[i]->offset < x);

		if(i < j)
		{
			if(A[i]->offset > A[j]->offset)
				ExchangeElementsForVehicleArray(&A[i], &A[j]); //exchange the array elements of A[i] and A[j]
		}
		else
		{
			return j;
		}
	}
}

static void ExchangeElementsForVehicleArray(vehicle_queue_node_t **ptr1, vehicle_queue_node_t **ptr2)
{ //exchange the vehicle array elements of *ptr1 (i.e., A[i]) and *ptr2 (i.e., A[j])
	vehicle_queue_node_t *temp;

	temp = *ptr1;
	*ptr1 = *ptr2;
	*ptr2 = temp;
}

/////////////////////////////////////////////////////////

/** Quick Sort for Directional_Edge Array of type directional_edge_queue_node_t */
void QuickSortForDirectionalEdgeArray(directional_edge_queue_node_t **A, int p, int r)
{ //perform quick sort for directional edge array
	int q; //index for array A

	if(p < r)
	{
		q = PartitionForDirectionalEdgeArray(A, p, r);
		QuickSortForDirectionalEdgeArray(A, p, q);
		QuickSortForDirectionalEdgeArray(A, q+1, r);
	}
}

static int PartitionForDirectionalEdgeArray(directional_edge_queue_node_t **A, int p, int r)
{ //find an index to partition the array A for further quick sort for directional edge array
	double x; //pivot for partitioning
	int i, j; //indicies for array A

	x = A[p]->eid; 
	//x is pivot for the partition of A such that the elements less than x
	//are placed at the left side of A and the elements greater than x are
	//placed at the right side of A.

	i = p - 1;
	j = r + 1;

	while(1)
	{
		do
		{
			j = j - 1; //decrease index j; that is, move j to the left			
		} while(A[j]->eid > x);

		do
		{
			i = i + 1; //increase index i; that is, move i to the right			
		} while(A[i]->eid < x);

		if(i < j)
		{
			if(A[i]->eid > A[j]->eid)
				ExchangeElementsForDirectionalEdgeArray(&A[i], &A[j]); //exchange the array elements of A[i] and A[j]
		}
		else
		{
			return j;
		}
	}
}

static void ExchangeElementsForDirectionalEdgeArray(directional_edge_queue_node_t **ptr1, directional_edge_queue_node_t **ptr2)
{ //exchange the directional edge array elements of *ptr1 (i.e., A[i]) and *ptr2 (i.e., A[j])
	directional_edge_queue_node_t *temp;

	temp = *ptr1;
	*ptr1 = *ptr2;
	*ptr2 = temp;
}


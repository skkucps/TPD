/**
 *  File: quick-sort.h
 *	Description: Quick sort algorithm
 *	Date: 09/13/2007	
 *	Maker: Jaehoon Jeong
 */

#ifndef __QUICK_SORT_H__
#define __QUICK_SORT_H__

#include "queue.h"

void QuickSort(int *A, int p, int r); 
//perform quick sort

static int Partition(int *A, int p, int r); 
//find an index to partition the array A for further quick sort

static void ExchangeElements(int *ptr1, int *ptr2);
//exchange the array elements of *ptr1 (i.e., A[i]) and * ptr2 (i.e., A[j])

/** Quick Sort for Sensor Array of type sensor_queue_node_t */
void QuickSortForSensorArray(sensor_queue_node_t **A, int p, int r);
//perform quick sort for sensor array

static int PartitionForSensorArray(sensor_queue_node_t **A, int p, int r);
//find an index to partition the array A for further quick sort for sensor array

static void ExchangeElementsForSensorArray(sensor_queue_node_t **ptr1, sensor_queue_node_t **ptr2);
//exchange the sensor array elements of *ptr1 (i.e., A[i]) and * ptr2 (i.e., A[j])

/** Quick Sort for Edge_Set Array of type edge_set_queue_node_t */
void QuickSortForEdgeSetArray(edge_set_queue_node_t **A, int p, int r);
//perform quick sort for edge_set array

static int PartitionForEdgeSetArray(edge_set_queue_node_t **A, int p, int r);
//find an index to partition the array A for further quick sort for edge_set array

static void ExchangeElementsForEdgeSetArray(edge_set_queue_node_t **ptr1, edge_set_queue_node_t **ptr2);
//exchange the edge_set array elements of *ptr1 (i.e., A[i]) and *ptr2 (i.e., A[j])

/** Quick Sort for Sensing Hole Endpoint Array of type hole_endpoint_queue_node_t */
void QuickSortForSensingHoleEndpointArray(hole_endpoint_queue_node_t **A, int p, int r);
//perform quick sort for sensing hole endpoint array

static int PartitionForSensingHoleEndpointArray(hole_endpoint_queue_node_t **A, int p, int r);
//find an index to partition the array A for further quick sort for sensing hole endpoint array

static void ExchangeElementsForSensingHoleEndpointArray(hole_endpoint_queue_node_t **ptr1, hole_endpoint_queue_node_t **ptr2);
//exchange the sensing hole endpoint array elements of *ptr1 (i.e., A[i]) and *ptr2 (i.e., A[j])

/** Quick Sort for Angle Array of type angle_queue_node_t */
void QuickSortForAngleArray(angle_queue_node_t **A, int p, int r);
//perform quick sort for angle array

static int PartitionForAngleArray(angle_queue_node_t **A, int p, int r);
//find an index to partition the array A for further quick sort for angle array

static void ExchangeElementsForAngleArray(angle_queue_node_t **ptr1, angle_queue_node_t **ptr2);
//exchange the angle array elements of *ptr1 (i.e., A[i]) and *ptr2 (i.e., A[j])

/** Quick Sort for Intersection EDD Array of type intersection_edd_queue_node_t */
void QuickSortForIntersection_EDD_Array(intersection_edd_queue_node_t **A, int p, int r);
//perform quick sort for intersection EDD array

static int PartitionForIntersection_EDD_Array(intersection_edd_queue_node_t **A, int p, int r);
//find an index to partition the array A for further quick sort for intersection EDD array

static void ExchangeElementsForIntersection_EDD_Array(intersection_edd_queue_node_t **ptr1, intersection_edd_queue_node_t **ptr2);
//exchange the intersection EDD array elements of *ptr1 (i.e., A[i]) and *ptr2 (i.e., A[j])

/** Quick Sort for VehicleMovement Array of type vehicle_movement_queue_node_t */
void QuickSortForVehicleMovementArray(vehicle_movement_queue_node_t **A, int p, int r);
//perform quick sort for vehicle_movement array

static int PartitionForVehicleMovementArray(vehicle_movement_queue_node_t **A, int p, int r);
//find an index to partition the array A for further quick sort for vehicle_movement array

static void ExchangeElementsForVehicleMovementArray(vehicle_movement_queue_node_t **ptr1, vehicle_movement_queue_node_t **ptr2);
//exchange the vehicle_movement array elements of *ptr1 (i.e., A[i]) and *ptr2 (i.e., A[j])

/** Quick Sort for Vehicle Array of type vehicle_queue_node_t */
void QuickSortForVehicleArray(vehicle_queue_node_t **A, int p, int r);
//perform quick sort for vehicle array

static int PartitionForVehicleArray(vehicle_queue_node_t **A, int p, int r);
//find an index to partition the array A for further quick sort for vehicle array

static void ExchangeElementsForVehicleArray(vehicle_queue_node_t **ptr1, vehicle_queue_node_t **ptr2);
//exchange the vehicle array elements of *ptr1 (i.e., A[i]) and *ptr2 (i.e., A[j])

/** Quick Sort for Directional_Edge Array of type directional_edge_queue_node_t */
void QuickSortForDirectionalEdgeArray(directional_edge_queue_node_t **A, int p, int r);
//perform quick sort for directional edge array

static int PartitionForDirectionalEdgeArray(directional_edge_queue_node_t **A, int p, int r);
//find an index to partition the array A for further quick sort for directional edge array

static void ExchangeElementsForDirectionalEdgeArray(directional_edge_queue_node_t **ptr1, directional_edge_queue_node_t **ptr2);
//exchange the directional edge array elements of *ptr1 (i.e., A[i]) and *ptr2 (i.e., A[j])

#endif

/**
	File: matlab-operation.h
	Description: This file contains the functions of target mobility handling and matlab operations.
	Date: 07/24/2006
	Maker: Jaehoon Jeong, jjeong@cs.umn.edu
	Memo:
*/

#ifndef __MATLAB_OPERATION_H__
#define __MATLAB_OPERATION_H__

#include "engine.h"
//#include "matlab.h"
#include "param.h"

#define MATLAB_BUF_SIZE 256
#define MATLAB_NUMBER_OF_INITIAL_VECTOR_VALUES 4
//number of the initial vector's values
#define MATLAB_NUMBER_OF_MOBILITY_VECTOR_VALUES 3
//number of the mobility vector's values

#define APL_MATLAB_START_COMMAND "-nodisplay"
//matlab start command to depress the matlab GUI desktop window. cf. -nodesktop

/* structure to get the matlab result for target tracking */
typedef struct _struct_matlab_tracking_result
{
	int number_of_working_sensors; //number of working sensors
	int number_full_tx_power; //number of RF receiving sensors by full Tx power
	int number_directional_antenna; //number of RF receiving sensors by directional antenna for full Tx power
	int number_tx_power_control; //number of RF receiving sensors by tx power control for full Tx power
	int number_tx_and_directional; //number of RF receiving sensors by both tx power control and directional antenna for full Tx power
	double energy_full_tx_power; //energy cost by full Tx power
	double energy_directional_antenna; //energy cost by directional antenna for full Tx power
	double energy_tx_power_control; //energy cost by tx power control for full Tx power
	double energy_tx_and_directional; //energy cost by both tx power control and directional antenna for full Tx power
} struct_matlab_tracking_result;

/* structure to get matlab result of prefiltering */
typedef struct _struct_matlab_prefiltering_result
{
	int result; //result of matlab operation for localization
        char road_sensor_network_graph_filename[BUF_SIZE]; //the file name of the road-sensor network graph that contains the topology of the sensor network deployed on the road network, which is used to compute the localization error
        char intersection_vector_filename[BUF_SIZE]; //the file name of the intersection vector 
        char road_network_graph_filename[BUF_SIZE]; //the file name of the road network graph
	char intersection_sensor_network_graph_filename[BUF_SIZE]; //the file name of the intersection sensor network graph that is the permuted one for the road-sensor network graph and whose nodes are intersection sensor nodes
	char sensor_network_graph_filename[BUF_SIZE]; //the file name of the sensor network graph that is the permuted one for the sensor nodes deployed in the road-sensor network graph
        char permutation_vector_filename[BUF_SIZE]; //the file name of the permutation vector
        char intersection_pair_matrix_filename[BUF_SIZE]; //the file name of the intersection pair matrix whose entry consists of (intersection sensor id, the corresponding intersection id)
        char intersection_sensor_id_vector_filename[BUF_SIZE]; //the file name of the intersection sensor id vector
        char nonintersection_sensor_id_vector_filename[BUF_SIZE]; //the file name of the nonintersection sensor id vector
        char sensor_degree_vector_filename[BUF_SIZE]; //the file name of the sensor degree vector where the degree for each sensor is the number of the sensor's neighbors
} struct_matlab_prefiltering_result;

/* structure to get matlab result of localization */
typedef struct _struct_matlab_localization_result
{
	int result; //result of matlab operation for localization
	float error_ratio; //error ratio of localization
	float E_ratio; //ratio indicating the relative difference between Er and Ev
	float M_ratio; //ratio indicating the relative difference between Mr and Mv
} struct_matlab_localization_result;

int matlab_open_file(char *directory, char *filename);
//open filename to prove that the matlab recognizes the input filename

Engine* matlab_start_for_localization();
//perform the initialization for matlab operations; that is, it starts matlab engine.

void matlab_stop_localization(Engine *matlab_ep);
//close the connection with the matlab engine.

struct_matlab_localization_result* matlab_perform_localization(Engine *matlab_ep, char *current_directory, int nodenum, int measurement_number, double speed, double speed_deviation, char *adjacency_matrix_filename, char *virtual_topology_filename_prefix, int data_aggregation_type, int data_prefilter_type);
//	enum aggregation_type data_aggregation_type, enum prefilter_type data_prefilter_type);
//  : these types should be int since they are memory-copied into matlab variable of type double
//perform the localization of sensor nodes along with both prefiltering and graph matching.

struct_matlab_prefiltering_result* matlab_perform_prefiltering(Engine *matlab_ep, char *current_directory, int nodenum, int measurement_number, double speed, double speed_deviation, char *adjacency_matrix_filename, char *intersection_vector_filename, char *virtual_topology_filename_prefix, int data_aggregation_type, int data_prefilter_type);
//perform the prefiltering according to the given prefiltering type

struct_matlab_localization_result* matlab_perform_graph_matching(Engine *matlab_ep, char *current_directory, double speed, char *adjacency_matrix_filename, char *intersection_vector_filename, int data_aggregation_type, int data_prefilter_type, struct_matlab_prefiltering_result* prefiltering_result, boolean nonintersection_sensor_localization_flag);
//perform the graph matching according to the given graph matching type

int matlab_start(char *scenario_file); /* perform the initialization for matlab operations;
that is, it starts matlab engine and opens the scenario file. */

void matlab_stop(); //close the connection with the matlab engine and close the scenario file

int matlab_read_initial_mobility_vector(double *x, double *y, double *speed, double *direction);
//return the initial mobility vector including the vehicle's initial position, speed, and direction.

int matlab_read_mobility_vector(double *steering_angle, double *acceleration, double *duration);
//return the mobility vector including the vehicle's steering angle, acceleration and duration.

double matlab_get_optimal_refresh_time(parameter_t *param); 
//return the optimal refresh time using one-dimensional optimization,i.e., Golden section search.

int matlab_count_neighbor_sensors_based_on_contour(double refresh_time, struct_vehicle_t *vehicle, 
	struct_sensor_t *sensor_list, int total_sensor_number, int *S_ID, double *S_X, double *S_Y, 
	int *p_live_sensor_number, struct_matlab_tracking_result *p_matlab_result);
//estimate the number of neighbor sensors within the current dynamic contour

int matlab_count_neighbor_sensors_based_on_circle(double refresh_time, struct_vehicle_t *vehicle, 
	struct_sensor_t *sensor_list, int total_sensor_number, int *S_ID, double *S_X, double *S_Y, 
	int *p_live_sensor_number, struct_matlab_tracking_result *p_matlab_result);
//estimate the number of neighbor sensors within the current tracking circle

//MATLAB library - Linking MATLAB libraries automatically
#pragma comment(lib, "libeng.lib")
#pragma comment(lib, "libmx.lib")
//#pragma comment(lib, "libmatlb.lib")
#pragma comment(lib, "libmat.lib")
//#pragma comment(lib, "libmmfile.lib")

#endif

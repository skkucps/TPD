/** File: analysis.h
	Description: implement the functions for analyzing simulation data for localization
	Date: 03/20/2007
	Maker: Jaehoon Jeong, jjeong@cs.umn.edu
*/

#ifndef __ANALYSIS_H__
#define __ANALYSIS_H__

#include "matlab-operation.h" //struct_matlab_prefiltering_result

/* header files for subgraph matching */
#include "argloader.h"
#include "allocpool.h"
#include "vf2_sub_state.h"
#include "vf2_mono_state.h"
#include "match.h"
#include "apl-graph-matching.h"

#define RANGE_FACTOR 10
//range factor

//#define FP_DIGITS 2
#define FP_DIGITS 1
//the number of floating point digits

#define FP_SCALE_FACTOR 10
//#define FP_SCALE_FACTOR 1
//the factor to determine floating point part; for example, 10 means the first floating digit

#define MAXNODES 200
//the maximum number of graph nodes

#define WEIGHT_DIFFERENCE_THRESHOLD 5
//#define WEIGHT_DIFFERENCE_THRESHOLD 3
//threshold of edge weight difference

struct_matlab_localization_result* perform_data_analysis_v1(Engine *matlab_ep, parameter_t *param, char *output_file, char *localization_file);
//perform data analysis for localization using the isomorphic graph matching and return the error ratio of the localization

struct_matlab_localization_result* perform_data_analysis_v2(Engine *matlab_ep, parameter_t *param, char *output_file, char *localization_file, char *adjacency_matrix_file, char *intersection_vector_file);
//perform the localization based on the graph matching type, such as the isomorphic graph matching and the subgraph matching and return the error ratio of the localization

//char* get_virtual_topology(parameter_t *param, char *output_file, char *localization_file, int number);
////get a virtual topology (Mv) from simulation data including vehicle detection timestamps

char* get_virtual_topology_based_on_nonaggregation_method(parameter_t *param, char *output_file, char *localization_file, int number);
//get a virtual topology (Mv) from simulation data including vehicle detection timestamps using Non-aggregation Method

char* get_virtual_topology_based_on_aggregation_method(parameter_t *param, char *output_file, char *localization_file, int number);
//get a virtual topology (Mv) from simulation data including vehicle detection timestamps using Aggregation Method

int get_number_of_timestamps(char *output_file);
//get the number of timestamps in localization log file

int get_max_road_segment_length(char *output_file);
//get the maximum road segment length in real graph

void store_matrix_into_file(float** A, int n, char* filename);
//store the two-dimensional nxn matrix A into a file called filename

void divide_localization_data(parameter_t *param, char *localization_file, char *output_file, int nodenum, char **TF, int *TF_number);
//divide traffic measurement data into the number of measurement_number

int binary_search_for_localization(float *TIMESTAMP, int boundary_time, int start_index, int end_index);

char* store_timestamp_info_in_file(char *localization_file, int hour, int *NODE_ID, float *TIMESTAMP, int *EVENT, int *VEHICLE_ID, int start_index, int bound_index, int nodenum);
//store the timestamp information for hour i in a file//find the index for timestamp close to boundary_time using Binary Search

struct_matlab_localization_result* perform_enhanced_localization(Engine *matlab_ep, char *current_directory, int nodenum, int measurement_number, double speed, double speed_deviation, char *adjacency_matrix_filename, char *intersection_vector_filename, char *virtual_topology_filename_prefix, int data_aggregation_type, int data_prefilter_type);
//perform the localization of sensor nodes along with both prefiltering and subgraph matching.	

struct_matlab_localization_result* perform_subgraph_matching(char *current_directory, int nodenum, int measurement_number, double speed, char *adjacency_matrix_filename, char *intersection_vector_filename, char *virtual_topology_filename_prefix, int data_aggregation_type, int data_prefilter_type, struct_matlab_prefiltering_result* prefiltering_result, boolean nonintersection_sensor_localization_flag);
//perform the subgraph matching algorithm

bool my_visitor(int n, node_id ni1[], node_id ni2[], void *user_data);
//the callback function to store a matching into file

int read_permutation_vector_file(ifstream &in_p, node_id perm_list[], int *perm_list_size);
//read permutation vector file containing the pairs of intersection id and sensor id into perm_list

int read_intersection_pair_matrix_file(ifstream &in_i, node_id sensor_id_list[], int *sensor_id_list_size, node_id intersection_id_list[], int *intersection_id_list_size, node_id sensor_index_list[], int *sensor_index_list_size);
/* read the intersection pair matrix consisting of sensor_id_list and intersection_id_list from intersection_pair_matrix_file 
     where vector sensor_id_list is the list of sensor node ids, vector intersection_id_list is the list of the corresponding 
     intersection ids, and vector sensor_index_list is the list of the index for sensor node id in sensor_id_list.
*/

double compute_localization_error_ratio(node_id roadnet_list[], int roadnet_list_size, node_id perm_list[], int perm_list_size);
/* compute the localization error ratio
     where roadnet_list contains the intersection ids in the road network where the corresponding index in roadnet_list is sensor id,
           roadnet_list_size is the size of roadnet_list, that is, the number of sensors,
           perm_list contains the intersection ids in the road network where the corresponding index in perm_list is sensor id, and
           perm_list_size is the size of perm_list.
*/

#endif

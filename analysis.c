/** File: analysis.c
	Description: implement the functions for analyzing simulation data for localization
	Date: 03/20/2007
	Maker: Jaehoon Jeong, jjeong@cs.umn.edu
*/


#include "stdafx.h"

#include "common.h"
//#include "matlab-operation.h"
#include "analysis.h"
#include "util.h"

//#include <malloc.h> //for _msize()
#include <math.h> //for pow()
#include <time.h> //time()

#include <iostream>
#include <fstream>
using namespace std;

/* header files for open() */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#if defined(_WIN32)

#include <windows.h> //for GetCurrentDirectory()
#include <winbase.h>

#else

#include <unistd.h> //for getcwd(): get the current working directionary

#endif

struct_matlab_localization_result* perform_data_analysis_v1(Engine *matlab_ep, parameter_t *param, char *output_file, char *localization_file)
{ //perform data analysis for localization using the isomorphic graph matching and return the error ratio of the localization
	char *virtual_topology = NULL;
	int  measurement_number = param->data_number_of_split_measurement_times; //measurement number
	//int simulation_hours = (int)(param->simulation_time/3600); //simulation hours
	char **TF = NULL; //timestamp file names created from localization file
	int *TF_number = NULL; //number of entries in timestamp files
	char **VF = NULL; //file names including virtual topologies
	int length = BUF_SIZE; //file name length
	int i; //index for for-loop
	time_t start_time, end_time, computation_time; //time variables for measuring computation time
	char adjacency_matrix_filename[BUF_SIZE] = ADJACENCY_MATRIX_FILE; //adjacency matrix file
	char* return_val = NULL; //return value

#if defined(_WIN32)
	TCHAR current_directory[BUF_SIZE]; //buffer for current directory
        DWORD nBufferLength = BUF_SIZE;
        LPTSTR lpBuffer = current_directory;
#else
	char current_directory[BUF_SIZE]; //buffer for current directory
        size_t nBufferLength = BUF_SIZE;
        char *lpBuffer = current_directory;

#endif

	char localization_filename_buf[BUF_SIZE]; //buffer for localization filename
	char virtual_topology_filename_prefix[BUF_SIZE]; //prefix for virtual topology file name
	char *ptr; //pointer to string
	struct_matlab_localization_result *p_localization_result; //pointer to localization result
  
#if defined(_WIN32)
	i = GetCurrentDirectory(nBufferLength , lpBuffer) ;
#else
	return_val = getcwd(lpBuffer, nBufferLength);
	if(return_val == NULL)
	{
	  printf("perform_data_analysis_v2(): Error: getcwd()'s returned alsolute path name needs a longer buffer than lpBuffer of size %d\n", BUF_SIZE);
	  exit(1);
	}
	i = strlen(lpBuffer);

#endif

#if defined( __DEBUG_LEVEL_3__)
        printf("*** CURRENT DIRECTORY: %s, LENGTH: %d ***\n\n" , lpBuffer , i) ;
#endif

	//@ for debugging
	//strcpy(virtual_topology_filename_prefix, "result\\localization");
	//error_ratio = matlab_perform_localization(current_directory, param->sensor_number, simulation_hours, param->vehicle_speed, adjacency_matrix_filename, virtual_topology_filename_prefix);
	////////////////

	/* allocate the memory for file names TF and VF */
	TF = (char**) calloc(measurement_number, sizeof(char*));
	assert_memory(TF);

	TF_number = (int*) calloc(measurement_number, sizeof(int));
	assert_memory(TF_number);

	VF = (char**) calloc(measurement_number, sizeof(char*));
	assert_memory(VF);

	for(i = 0; i < measurement_number; i++)
	{
		TF[i] = (char*) calloc(length, sizeof(char));
		assert_memory(TF[i]);

		VF[i] = (char*) calloc(length, sizeof(char));
		assert_memory(VF[i]);
	}

	/* divide traffic measurement data into the number of measurement_number */
	divide_localization_data(param, localization_file, output_file, param->sensor_number, TF, TF_number);

	/* get a virtual topology (Mv) from simulation data including vehicle detection timestamps */
        for(i = 0; i < measurement_number; i++)
	//for(i = 0; i < simulation_hours; i++)
	{
		/**************************************************/
		start_time = time(NULL);
		printf("\n\n*** get_virtual_topology(): [File %d] start_time=%d ***\n\n", i+1, start_time);
		/**************************************************/

#ifdef __DEBUG_TRACE__
	printf("BEFORE get_virtual_topology()\n");
#endif
	        //virtual_topology = get_virtual_topology(param, output_file, TF[i], TF_number[i]);
	        if(param->data_aggregation_type == AGGREGATION_TYPE_0)
	          virtual_topology = get_virtual_topology_based_on_nonaggregation_method(param, output_file, TF[i], TF_number[i]);
	        else if(param->data_aggregation_type == AGGREGATION_TYPE_1)
	          virtual_topology = get_virtual_topology_based_on_aggregation_method(param, output_file, TF[i], TF_number[i]);
		else
		{
		  printf("perform_data_analysis_v1(): data_aggregation_type(%d) is not supported!\n", param->data_aggregation_type);
		  exit(1);
		}

		strcpy(VF[i], virtual_topology);

#ifdef __DEBUG_TRACE__
	printf("AFTER get_virtual_topology()\n");
#endif
		/**************************************************/
		end_time = time(NULL);
		printf("\n\n*** get_virtual_topology(): [File %d] end_time=%d ***\n\n", i+1, start_time);
		computation_time = end_time - start_time;
		printf("\n\n*** get_virtual_topology(): [File %d] computation_time=%d ***\n\n\n", i+1, computation_time);
		/**************************************************/
	}

	/* get the prefix of localization file */
	strcpy(localization_filename_buf, localization_file);
	/*
	token = strtok(localization_filename_buf, ".");
	if(token == NULL)
	{
		printf("perform_data_analysis(): error: token is NULL\n");
		exit(1);
	}
	*/

	ptr = rindex(localization_filename_buf, '.');
	*ptr = '\0';

	memset(virtual_topology_filename_prefix, 0, sizeof(virtual_topology_filename_prefix));
	strcpy(virtual_topology_filename_prefix, localization_filename_buf);

	/* perform prefiltering and graph matching using Matlab */

#ifdef __DEBUG_TRACE__
	printf("BEFORE matlab_perform_localization()\n");
#endif
	p_localization_result = matlab_perform_localization(matlab_ep, current_directory, param->sensor_number, measurement_number, param->vehicle_speed, param->vehicle_speed_standard_deviation, adjacency_matrix_filename, virtual_topology_filename_prefix, (int)param->data_aggregation_type, (int)param->data_prefilter_type);
	//enum aggregation_type data_aggregation_type, enum prefilter_type data_prefilter_type)
	// : these types should be int since they are memory-copied into matlab variable of type double
	//perform the localization of sensor nodes along with both prefiltering and graph matching.	

#ifdef __DEBUG_TRACE__
	printf("AFTER matlab_perform_localization()\n");
#endif

	/* release the memory for file names TF and VF*/
	for(i = 0; i < measurement_number; i++)
	{
		free(TF[i]);
		free(VF[i]);
	}

	free(TF);
	free(TF_number);
	free(VF);

	return p_localization_result;
}

struct_matlab_localization_result* perform_data_analysis_v2(Engine *matlab_ep, parameter_t *param, char *output_file, char *localization_file, char *adjacency_matrix_file, char *intersection_vector_file)
{ //perform the localization based on the graph matching type, such as the isomorphic graph matching and the subgraph matching and return the error ratio of the localization
	char *virtual_topology = NULL;
	int  measurement_number = param->data_number_of_split_measurement_times; //measurement number
	//int simulation_hours = (int)(param->simulation_time/3600); //simulation hours
	char **TF = NULL; //timestamp file names created from localization file
	int *TF_number = NULL; //number of entries in timestamp files
	char **VF = NULL; //file names including virtual topologies
	int length = BUF_SIZE; //file name length
	int i; //index for for-loop
	time_t start_time, end_time, computation_time; //time variables for measuring computation time
	char* return_val = NULL; //return value

#if defined(_WIN32)
	TCHAR current_directory[BUF_SIZE]; //buffer for current directory
        DWORD nBufferLength = BUF_SIZE;
        LPTSTR lpBuffer = current_directory;
#else
	char current_directory[BUF_SIZE]; //buffer for current directory
        size_t nBufferLength = BUF_SIZE;
        char *lpBuffer = current_directory;

#endif

	char localization_filename_buf[BUF_SIZE]; //buffer for localization filename
	char virtual_topology_filename_prefix[BUF_SIZE]; //prefix for virtual topology file name
	char *ptr; //pointer to string
	struct_matlab_localization_result *p_localization_result; //pointer to localization result
  
#if defined(_WIN32)
	i = GetCurrentDirectory(nBufferLength , lpBuffer) ;
#else
	return_val = getcwd(lpBuffer, nBufferLength);
	if(return_val == NULL)
	{
	  printf("perform_data_analysis_v2(): Error: getcwd()'s returned alsolute path name needs a longer buffer than lpBuffer of size %d\n", BUF_SIZE);
	  exit(1);
	}
	i = strlen(lpBuffer);

#endif

#if defined( __DEBUG_LEVEL_3__)
        printf("*** CURRENT DIRECTORY: %s, LENGTH: %d ***\n\n" , lpBuffer , i) ;
#endif

	//@ for debugging
	//strcpy(virtual_topology_filename_prefix, "result\\localization");
	//error_ratio = matlab_perform_localization(current_directory, param->sensor_number, simulation_hours, param->vehicle_speed, adjacency_matrix_filename, virtual_topology_filename_prefix);
	////////////////

	/* allocate the memory for file names TF and VF */
	TF = (char**) calloc(measurement_number, sizeof(char*));
	assert_memory(TF);

	TF_number = (int*) calloc(measurement_number, sizeof(int));
	assert_memory(TF_number);

	VF = (char**) calloc(measurement_number, sizeof(char*));
	assert_memory(VF);

	for(i = 0; i < measurement_number; i++)
	{
		TF[i] = (char*) calloc(length, sizeof(char));
		assert_memory(TF[i]);

		VF[i] = (char*) calloc(length, sizeof(char));
		assert_memory(VF[i]);
	}

	/* divide traffic measurement data into the number of measurement_number */
	divide_localization_data(param, localization_file, output_file, param->sensor_number, TF, TF_number);

	/* get a virtual topology (Mv) from simulation data including vehicle detection timestamps */
        for(i = 0; i < measurement_number; i++)
	//for(i = 0; i < simulation_hours; i++)
	{
		/**************************************************/
		start_time = time(NULL);
		printf("\n\n*** get_virtual_topology(): [File %d] start_time=%d ***\n\n", i+1, start_time);
		/**************************************************/

#ifdef __DEBUG_TRACE__
	printf("BEFORE get_virtual_topology()\n");
#endif
	        //virtual_topology = get_virtual_topology(param, output_file, TF[i], TF_number[i]);
	        if(param->data_aggregation_type == AGGREGATION_TYPE_0)
	          virtual_topology = get_virtual_topology_based_on_nonaggregation_method(param, output_file, TF[i], TF_number[i]);
	        else if(param->data_aggregation_type == AGGREGATION_TYPE_1)
	          virtual_topology = get_virtual_topology_based_on_aggregation_method(param, output_file, TF[i], TF_number[i]);
		else
		{
		  printf("perform_data_analysis_v2(): data_aggregation_type(%d) is not supported!\n", param->data_aggregation_type);
		  exit(1);
		}

		strcpy(VF[i], virtual_topology);

#ifdef __DEBUG_TRACE__
	printf("AFTER get_virtual_topology()\n");
#endif
		/**************************************************/
		end_time = time(NULL);
		printf("\n\n*** get_virtual_topology(): [File %d] end_time=%d ***\n\n", i+1, start_time);
		computation_time = end_time - start_time;
		printf("\n\n*** get_virtual_topology(): [File %d] computation_time=%d ***\n\n\n", i+1, computation_time);
		/**************************************************/
	}

	/* get the prefix of localization file */
	strcpy(localization_filename_buf, localization_file);
	ptr = rindex(localization_filename_buf, '.');
	*ptr = '\0';

	memset(virtual_topology_filename_prefix, 0, sizeof(virtual_topology_filename_prefix));
	strcpy(virtual_topology_filename_prefix, localization_filename_buf);

	/* perform prefiltering and graph matching using Matlab */

#ifdef __DEBUG_TRACE__
	printf("BEFORE perform_enhanced_localization()\n");
#endif
	p_localization_result = perform_enhanced_localization(matlab_ep, current_directory, param->sensor_number, measurement_number, param->vehicle_speed, param->vehicle_speed_standard_deviation, adjacency_matrix_file, intersection_vector_file, virtual_topology_filename_prefix, (int)param->data_aggregation_type, (int)param->data_prefilter_type);
	//perform the localization of sensor nodes along with both prefiltering and subgraph matching.	

#ifdef __DEBUG_TRACE__
	printf("AFTER perform_enhanced_localization()\n");
#endif

	/* release the memory for file names TF and VF*/
	for(i = 0; i < measurement_number; i++)
	{
		free(TF[i]);
		free(VF[i]);
	}

	free(TF);
	free(TF_number);
	free(VF);

	return p_localization_result;
}

void divide_localization_data(parameter_t *param, char *localization_file, char *output_file, int nodenum, char **TF, int *TF_number)
{ //divide traffic measurement data into the number of measurement_number
	int *NODE_ID = NULL; //vector for node ID
	float *TIMESTAMP = NULL; //vector for timestamp
	int *EVENT = NULL; //event
	int *VEHICLE_ID = NULL; //vehicle ID
	int sensor_id; //sensor node ID
	float timestamp; //timestamp
	int event; //event
	int vehicle_id; //vehicle ID
	FILE *fp = NULL; //file pointer to output file
	int i; //index for for-loop
	int start_index, end_index, bound_index; //index variables
	int boundary_time; //boundary time
	int length = 0; //actual length of arrays, such as NODE_ID
	int number; //number of timestamps in localization file
	char *filename; //file name

	/* get the number of timestamps in localization log file */
	number = get_number_of_timestamps(output_file);

	/* make vectors NODE_ID and TIMESTAMP */
	NODE_ID = (int*) malloc(number*sizeof(int));
	assert_memory(NODE_ID);

	TIMESTAMP = (float*) malloc(number*sizeof(float));
	assert_memory(TIMESTAMP);

	EVENT = (int*) malloc(number*sizeof(int));
	assert_memory(EVENT);

	VEHICLE_ID = (int*) malloc(number*sizeof(int));
	assert_memory(VEHICLE_ID);

	/* open localization_file and store localization_file into arrays, such as NODE_ID */
	fp = fopen(localization_file, "r");
	if(!fp)
	{
	    fprintf(stderr, "error : unable to open file \"%s\"\n", localization_file);
	    exit(1);
	}

	for(i = 0; i < number; i++)
	{
		fscanf(fp, "%d %f %d %d", &sensor_id, &timestamp, &event, &vehicle_id);
		NODE_ID[length] = sensor_id;
		TIMESTAMP[length] = timestamp;			
		EVENT[length] = event;
		VEHICLE_ID[length++] = vehicle_id;
	}

	/* store the contents of arrays into multiple files according to time boundary where unit is hour */
	start_index = 0;
	end_index = number - 1;
	bound_index = 0; //index for the last timestamp within the specified interval
	for(i = 1; i <= param->data_number_of_split_measurement_times; i++)
	//for(i = 1; i <= simulation_hours; i++)
	{
	        boundary_time = (int)(param->data_measurement_time*60*i); //param->data_measurement_time's unit is minute
	        //boundary_time = 3600*i;
	        bound_index = binary_search_for_localization(TIMESTAMP, boundary_time, start_index, end_index); 
		//find the index for timestamp close to boundary_time using Binary Search

		TF_number[i-1] = bound_index - start_index + 1; //number of timestamp entries
                filename = store_timestamp_info_in_file(localization_file, i, NODE_ID, TIMESTAMP, EVENT, VEHICLE_ID, start_index, bound_index, nodenum);
		//store the timestamp information for hour i in a file
		strcpy(TF[i-1], filename);		
		start_index = bound_index+1;
	}
    
	/* release the memory for arrays */
	free(NODE_ID);
	free(TIMESTAMP);
	free(EVENT);
	free(VEHICLE_ID);

	/* I forgot to write "fclose()" here. Due to the omission of fclose(), 
	I cannot open a new file, since file handles are exhausted. */
	fclose(fp);
}

int binary_search_for_localization(float *TIMESTAMP, int boundary_time, int start_index, int end_index)
{ //find the index for timestamp close to boundary_time using Binary Search
	int idx; //index
	int num = boundary_time;
	int mid = start_index;
	int left = start_index;
	int right = end_index;;

	while(left <= right)
	{
		//mid = (int)floor((left+right)/2);    
		mid = (int)floor((left+right)/2.0);    
    
		if(num > TIMESTAMP[mid])
			left = mid + 1;        
		else if(num < TIMESTAMP[mid])
			right = mid - 1;        
		else // num == csum(mid)
		{
			idx = mid;
			return idx;
		}
	}

	idx = right;

	return idx;
}

char* store_timestamp_info_in_file(char *localization_file, int hour, int *NODE_ID, float *TIMESTAMP, int *EVENT, int *VEHICLE_ID, int start_index, int bound_index, int nodenum)
{ //store the timestamp information for hour i in a file
	char filename_suffix[BUF_SIZE] = LOCALIZATION_FILE_SUFFIX; //localization file name suffix
	static char filename[BUF_SIZE] = ""; //file name
	char localization_filename_prefix[BUF_SIZE] = ""; //prefix for localization file name
	char *ptr = NULL; //pointer to string
	char hour_buf[BUF_SIZE] = "";
	FILE *fp = NULL; //file pointer to filename
	int i = 0; //index for for-loop

	/* make a file name to store the timestamp information */
	strcpy(localization_filename_prefix, localization_file);
        /*
	token = strtok(localization_filename_buf, ".");
	if(token == NULL)
	{
		printf("store_timestamp_info_in_file(): error: token is NULL\n");
		exit(1);
	}
	*/

	ptr = rindex(localization_filename_prefix, '.');
	*ptr = '\0';

	memset(filename, 0, sizeof(filename));
	strcpy(filename, localization_filename_prefix);
	strcat(filename, "-"); //separator character
	itoa(hour, hour_buf);
	strcat(filename, hour_buf);
	strcat(filename, filename_suffix);

	/* open filename and store the timestamp information into filename */
	fp = fopen(filename, "w");
	if(!fp)
	{
	    fprintf(stderr, "error : unable to open file \"%s\"\n", filename);
	    exit(1);
	}

	for(i = start_index; i <= bound_index; i++)
		fprintf(fp, "%d %f %d %d\n", NODE_ID[i], TIMESTAMP[i], EVENT[i], VEHICLE_ID[i]);
    
	fclose(fp);

	return filename;
}

/* char* get_virtual_topology(parameter_t *param, char *output_file, char *localization_file, int number) */
/* { //get a virtual topology (Mv) from simulation data including vehicle detection timestamps */
/* 	//int number; //number of timestamps in localization.txt */
/* 	int length = 0; //length of TIMESTAMP including only timestamps for vehicle detection event (or VEHICLE_CHECK)  */
/* 	//size_t size; //allocated memory size */
/* 	//char pathtable_file[BUF_SIZE] = PATHTABLE_FILE; //path-table file */
/* 	//char adjacency_matrix_file[BUF_SIZE] = ADJACENCY_MATRIX_FILE; //adjacency matrix file */
/* 	int nodenum = param->sensor_number; //number of sensor nodes */
/* 	double speed = param->vehicle_speed; //vehicle speed	 */
/* 	double speed_standard_deviation = param->vehicle_speed_standard_deviation;  */
/* 	//standard deviation of vehicle speed */
/* 	int window_size; //window size for aggregation */
/* 	//int window_size_factor = (int) param->data_aggregation_window_size; //window size factor */
/* 	int max_road_segment_length; //maximum road segment length */
/* 	int range; //range for time difference */
/* 	int range_factor; //range factor */
/* 	////////////////////////////////////////////////////////////// */
/* 	int fp_digits; //number of the digits under floating point */
/* 	int fp_factor = 10; //factor to increase the scale of time difference axis according to fp_digits */
/* 	int fp_scale; //scale for time difference which is pow(fp_factor, fp_digits) */
/* 	////////////////////////////////////////////////////////////// */
/* 	int *NODE_ID = NULL; //vector for node ID */
/* 	float *TIMESTAMP = NULL; //vector for timestamp */
/* 	FILE *fp = NULL; //file pointer to output file */
/* 	int i, j, k, h; //index for for-loop */
/* 	int index; //index for vectors */
/* 	float difference; //time difference */
/* 	int sensor_id; //sensor node ID */
/* 	float timestamp; //timestamp */
/* 	int event; //event */
/* 	int vehicle_id; //vehicle ID */
/* 	float **TV = NULL; //time vectors for sensors */
/* 	int *TV_size = NULL; //size of time vector for each sensor */
/* 	float ***E_X1 = NULL; //time difference without aggregation */
/* 	int ***E_Y1 = NULL; //frequency of time difference without aggregation */
/* 	int **E_R1 = NULL; //frequency of time difference greater than range without aggregation */
/* 	float ***E_X2 = NULL; //time difference with aggregation */
/* 	int ***E_Y2 = NULL; //frequency of time difference with aggregation	 */
/* 	int **E_R2 = NULL; //frequency of time difference greater than range with aggregation */
/* 	int **E_X2_size = NULL; //actual size of aggregation vector */
/* 	float **E_X1_max = NULL; //matrix for time difference with maximum frequency for non-aggregation method */
/* 	int **E_Y1_max = NULL; //matrix for maximum frequency for non-aggregation method */
/* 	float **E_X2_max = NULL; //matrix for time difference with maximum frequency for aggregation method */
/* 	int **E_Y2_max = NULL; //matrix for maximum frequency for aggregation method */

/* 	/\* output files for simulation *\/	 */
/* 	char virtual_topology_file_suffix1[BUF_SIZE] = VIRTUAL_TOPOLOGY_FILE_SUFFIX_1;  */

/* 	//file suffix for virtual topology based on non-aggregation method */
/* 	char virtual_topology_file_suffix2[BUF_SIZE] = VIRTUAL_TOPOLOGY_FILE_SUFFIX_2;  */
/* 	//file suffix for virtual topology based on aggregation method */
/* 	static char virtual_topology_file_name1[BUF_SIZE];  */
/* 	//file name for virtual topology based on non-aggregation method */
/* 	static char virtual_topology_file_name2[BUF_SIZE];  */
/* 	//file name for virtual topology based on aggregation method */
/* 	char *ptr; //pointer to string */
/* 	char localization_filename_buf[BUF_SIZE]; //buffer for localization file name */

/* 	/\**@for debugging *\/ */
/* 	/\* */
/* 	static int file_num = 0; */
/* 	file_num++; */

/* 	if(file_num == 11)  */
/* 	    printf("file_num=%d\n", file_num); */
/* 	*\/ */
/* 	/\******************\/ */

/* 	//number = get_number_of_timestamps(output_file); */
/* 	//get the number of timestamps in localization log file */

/* 	max_road_segment_length = get_max_road_segment_length(output_file); */
/* 	//get the maximum road segment length in real graph */

/* 	/\* time difference range *\/ */
/* 	//range_factor = RANGE_FACTOR; */
/* 	//fp_digits = FP_DIGITS; //decide how many floating digits are used */
/* 	//fp_scale = (int) pow(fp_factor, fp_digits); */
/* 	fp_scale = FP_SCALE_FACTOR; //we consider the first floating point */
/* 	//@Note: fp_scale is needed to set considering max time sync error and vehicle speed deviation */

/* 	//range = (int) (max_road_segment_length/speed*range_factor*fp_scale); */
/* 	//range = (int) (max_road_segment_length/speed*1.5); //note: the time difference should be less than 1.5 times the travel time for the maximum road segment */
/* 	//range = (int) (max_road_segment_length/speed*1.5); //note: the time difference should be less than 1.5 times the travel time for the maximum road segment */
/* 	//range = (int) (max_road_segment_length/speed*1.5*fp_scale); //note: the time difference should be less than 1.5 times the travel time for the maximum road segment multipled by fp_scale: why? */

/* 	/\* range is the travel time bound considering the vehicle speed and the vehicle speed deviation *\/ */
/* 	range = (int) (max_road_segment_length/(speed - 3*speed_standard_deviation)); //note: the time difference should be less than range: 3*speed_standard_deviation covers the 99% of possible speed range */

/* 	//VEHICLE_CHECK: detection event */
	
/* 	/\* make vectors NODE_ID and TIMESTAMP *\/ */
/* 	NODE_ID = (int*) calloc(number, sizeof(int)); */
/* 	assert_memory(NODE_ID); */

/* 	TIMESTAMP = (float*) calloc(number, sizeof(float)); */
/* 	assert_memory(TIMESTAMP); */

/* 	/\* open localization_file and store localization_file into arrays, such as NODE_ID *\/ */
/* 	fp = fopen(localization_file, "r"); */
/* 	if(!fp) */
/* 	{ */
/* 	    fprintf(stderr, "error : unable to open file \"%s\"\n", localization_file); */
/* 	    exit(1); */
/* 	} */

/* 	for(i = 0; i < number; i++) */
/* 	{ */
/* 		fscanf(fp, "%d %f %d %d", &sensor_id, &timestamp, &event, &vehicle_id); */
/* 		if(event == VEHICLE_CHECK) */
/* 		{ */
/* 			NODE_ID[length] = sensor_id; */
/* 			TIMESTAMP[length++] = timestamp;			 */
/* 		} */
/* 	} */

/* 	/\* realloc the memory fof NODE_ID and TIMESTAMP *\/ */
/* 	NODE_ID = (int*) realloc(NODE_ID, length*sizeof(int)); */
/* 	assert_memory(NODE_ID); */

/* 	TIMESTAMP = (float*) realloc(TIMESTAMP, length*sizeof(float)); */
/* 	assert_memory(TIMESTAMP); */

/* 	/\* allocate the memory for TV and TV_size *\/ */
/* 	TV = (float**) calloc(param->sensor_number, sizeof(float*)); */
/* 	assert_memory(TV); */

/* 	for(i = 0; i < param->sensor_number; i++) */
/* 	{ */
/* 		TV[i] = (float*) calloc(length, sizeof(float)); */
/* 		assert_memory(TV[i]); */
/* 	} */

/* 	TV_size = (int*) calloc(param->sensor_number, sizeof(int)); */
/* 	//calloc() initializes each element to 0. */
/* 	assert_memory(TV_size); */

/* 	/\* store timestamps corresponding to node ID (i.e., i) into TV[i] *\/ */
	
/* 	for(k = 0; k < length; k++) */
/* 	{ */
/* 		i = NODE_ID[k]-1; */
/* 		j = TV_size[i]++; */
/* 		TV[i][j] = TIMESTAMP[k];		 */
/* 	} */

/* 	/\* allocate the memory for E_X1, E_Y1, E_X2, E_Y2, and so on *\/ */
/* 	E_X1 = (float***) calloc(nodenum, sizeof(float**)); */
/* 	assert_memory(E_X1); */

/* 	E_Y1 = (int***) calloc(nodenum, sizeof(int**)); */
/* 	assert_memory(E_Y1); */

/* 	E_R1 = (int**) calloc(nodenum, sizeof(int*)); */
/* 	assert_memory(E_R1); */

/* 	E_X2 = (float***) calloc(nodenum, sizeof(float**)); */
/* 	assert_memory(E_X2); */

/* 	E_Y2 = (int***) calloc(nodenum, sizeof(int**)); */
/* 	assert_memory(E_Y2); */

/* 	E_R2 = (int**) calloc(nodenum, sizeof(int*)); */
/* 	assert_memory(E_R2); */

/* 	E_X2_size = (int**) calloc(nodenum, sizeof(int*)); */
/* 	assert_memory(E_X2_size); */

/* 	E_X1_max = (float**) calloc(nodenum, sizeof(float*)); */
/* 	assert_memory(E_X1_max);  */

/* 	E_Y1_max = (int**) calloc(nodenum, sizeof(int*)); */
/* 	assert_memory(E_Y1_max);  */

/* 	E_X2_max = (float**) calloc(nodenum, sizeof(float*)); */
/* 	assert_memory(E_X2_max);  */

/* 	E_Y2_max = (int**) calloc(nodenum, sizeof(int*)); */
/* 	assert_memory(E_Y2_max);  */

/* 	for(i = 0; i < nodenum; i++) */
/* 	{ */
/* 		E_X1[i] = (float**) calloc(nodenum, sizeof(float*)); */
/* 		assert_memory(E_X1[i]); */

/* 		E_Y1[i] = (int**) calloc(nodenum, sizeof(int*)); */
/* 		assert_memory(E_Y1[i]); */

/* 		E_R1[i] = (int*) calloc(nodenum, sizeof(int)); */
/* 		assert_memory(E_R1[i]); */

/* 		E_X2[i] = (float**) calloc(nodenum, sizeof(float*)); */
/* 		assert_memory(E_X2[i]); */

/* 		E_Y2[i] = (int**) calloc(nodenum, sizeof(int*)); */
/* 		assert_memory(E_Y2[i]); */

/* 		E_R2[i] = (int*) calloc(nodenum, sizeof(int)); */
/* 		assert_memory(E_R2[i]); */

/* 		E_X2_size[i] = (int*) calloc(nodenum, sizeof(int)); */
/* 		assert_memory(E_X2_size[i]); */

/* 		E_X1_max[i] = (float*) calloc(nodenum, sizeof(float)); */
/* 		assert_memory(E_X1_max[i]); */

/* 		E_Y1_max[i] = (int*) calloc(nodenum, sizeof(int)); */
/* 		assert_memory(E_Y1_max[i]); */

/* 		E_X2_max[i] = (float*) calloc(nodenum, sizeof(float)); */
/* 		assert_memory(E_X2_max[i]); */

/* 		E_Y2_max[i] = (int*) calloc(nodenum, sizeof(int)); */
/* 		assert_memory(E_Y2_max[i]); */
/* 	} */

/* 	for(i = 0; i < nodenum; i++) */
/* 	{ */
/* 		for(j = 0; j < nodenum; j++) */
/* 		{ */
/* 			E_X1[i][j] = (float*) calloc(range, sizeof(float)); */
/* 			assert_memory(E_X1[i][j]); */

/* 			E_Y1[i][j] = (int*) calloc(range, sizeof(int)); */
/* 			assert_memory(E_Y1[i][j]); */

/* 			E_X2[i][j] = (float*) calloc(range, sizeof(float)); */
/* 			assert_memory(E_X2[i][j]); */

/* 			E_Y2[i][j] = (int*) calloc(range, sizeof(int)); */
/* 			assert_memory(E_Y2[i][j]); */
/* 		} */
/* 	} */

/* 	/\** FAST comutation: compute the time difference frequency for non-aggregation method *\/ */
/* 	for(h = 0; h < nodenum; h++) //for-1 */
/* 	{ */
/* 		for(k = h+1; k < nodenum; k++) //for-2 */
/* 		{ */
/* 			//set E_X1[h][k] */
/* 			for(i = 0; i < range; i++) //for-3 */
/* 			{ */
/* 				E_X1[h][k][i] = (float)i; */
/* 				//set the same value in symmetric entry using the propoery of symmetric matrix */
/* 				E_X1[k][h][i] = (float)i; */
/* 			} //end of for-3 */

/* 			//set E_Y1[h][k] and E_R1[h][k] */
/* 			for(i = 0; i < TV_size[h]; i++) //for-4 */
/* 			{ */
/* 				for(j = 0; j < TV_size[k]; j++) //for-5 */
/* 				{ */
/* 				        //difference = (float)ceil(TV[h][i] - TV[k][j]); //@04/30/09: note that the time difference is rounded up to the second */
/* 				        //difference = (float)ceil(TV[h][i]*fp_scale - TV[k][j]*fp_scale); */
/* 					//difference = (float)fabs(difference); */
/*                                         //difference = (ftloat)fabs(TV[h][i] - TV[k][j]); */
/* 				        difference = (float)fabs(TV[h][i]*fp_scale - TV[k][j]*fp_scale); */
/* 					index = (int)round(difference); */
/* 					//index = (int)difference; */
					
/* 					if(index >= range)  */
/* 					{ */
/* 						E_R1[h][k]++; */
/* 						//increase the same values in symmetric entries using the propoery of symmetric matrix */
/* 						E_R1[k][h]++; */
/* 					} */
/* 					else  */
/* 					{ */
/* 						E_Y1[h][k][index]++; */
/* 						//increase the same values in symmetric entries using the propoery of symmetric matrix */
/* 						E_Y1[k][h][index]++; */
/* 					} */
/* 				} //end of for-5 */
/* 			} //end of for-4 */
/* 		} //end of for-2 */
/* 	} //end of for-1 */


/* 	/\** SLOW comutation: compute the time difference frequency for non-aggregation method *\/ */
/* 	/\* */
/* 	for(h = 0; h < nodenum; h++) //for-1 */
/* 	{ */
/* 		for(k = 0; k < nodenum; k++) //for-2 */
/* 		{ */
/* 			//set E_X1[h][k] */
/* 			for(i = 0; i < range; i++) //for-3 */
/* 			{ */
/* 				E_X1[h][k][i] = (float)i; */
/* 			} //end of for-3 */

/* 			if(h == k) continue; */

/* 			//set E_Y1[h][k] and E_R1[h][k] */
/* 			for(i = 0; i < TV_size[h]; i++) //for-4 */
/* 			{ */
/* 				for(j = 0; j < TV_size[k]; j++) //for-5 */
/* 				{ */
/* 					//difference = abs((int)(TV[h][i]*fp_scale - TV[k][j]*fp_scale)); */
/* 					//difference = (float)ceil(TV[h][i]*fp_scale - TV[k][j]*fp_scale); */
/* 					//index = (int)abs(difference); */
/* 					//difference = (float)fabs(difference); */
/* 					//index = (int)difference; */
/* 				        difference = (float)fabs(TV[h][i]*fp_scale - TV[k][j]*fp_scale); */
/* 					index = (int)round(difference); */
					
/* 					if(index >= range) E_R1[h][k]++; */
/* 					else E_Y1[h][k][index]++; */
/* 				} //end of for-5 */
/* 			} //end of for-4 */
/* 		} //end of for-2 */
/* 	} //end of for-1 */
/* 	*\/ */

/* 	/\** compute the time difference frequency for aggregation method *\/ */
/* 	//if(speed_standard_deviation == 0) */
/* 	//	speed_standard_deviation = 1.0; //make sure that speed_standard_deviation cannot be zero due to non-zero window_size. */

/* 	//window_size = (int)ceil(speed_standard_deviation*window_size_factor); //window size for aggregation */
/* 	window_size = param->data_aggregation_window_size; */

/* 	/\** Slow version for aggregation method */
/* 	for(h = 0; h < nodenum; h++) //for-1 */
/* 	{ */
/* 		for(k = 0; k < nodenum; k++) //for-2 */
/* 		{ */
/* 			if(h == k) continue; */

/* 			if(range <= window_size) //if-1 */
/* 			{ */
/* 				//E_X2[h][k][0] = floorf(mean_integer(1, range)); */
/* 				E_X2[h][k][0] = (float)floor(mean_integer(1, range)); //this x value is the representative for two sensors' timestamps */
/* 				E_Y2[h][k][0] = sum_vector(E_Y1[h][k], 0, range-1); */
/* 				E_X2_size[h][k] = 1; */
/* 				break; */
/* 			} //end of if-1 */

/* 			for(i = 0; i < range; i++) //for-3: i is the time difference value and range is the end point where the sliding window can move for moving average */
/* 			{ */
/* 				if(i > range-window_size) //if-2 //equals (range-1) - (window_size-1) */
/* 				{ */
/* 					//E_X2[h][k][i] = ceilf(mean_integer(i, range)); */
/* 					E_X2[h][k][i] = (float)ceil(mean_integer(i, range)); */
/* 					//E_X2[h][k][i] = (int)floor(mean_integer(i, range)); */
/* 					E_Y2[h][k][i] = sum_vector(E_Y1[h][k], i, range-1); */
/* 					E_X2_size[h][k]++; */
/* 				} //end of if-2 */
/* 				else //else-2 */
/* 				{ */
/* 					//E_X2[h][k][i] = ceilf(mean_integer(i, i+window_size-1)); */
/* 					E_X2[h][k][i] = (float)ceil(mean_integer(i, i+window_size-1)); */
/* 					//E_X2[h][k][i] = (int)floor(mean_integer(i, i+window_size-1)); */
/* 					E_Y2[h][k][i] = sum_vector(E_Y1[h][k], i, i+window_size-1); */
/* 					E_X2_size[h][k]++;					 */
/* 				} //end of else-2 */
/* 			} //end of for-3			 */
/* 		} //end of for-2 */
/* 	} //end of for-1 */
/* 	*\/ */

/* 	/\** Fast version for aggregation method */
/* 	for(h = 0; h < nodenum; h++) //for-1 */
/* 	{ */
/* 		for(k = h+1; k < nodenum; k++) //for-2 */
/* 		{ */
/* 			if(range <= window_size) //if-1 */
/* 			{ */
/* 				E_X2[h][k][0] = (float)floor(mean_integer(1, range)); //this x value is the representative for two sensors' timestamps */
/* 				E_Y2[h][k][0] = sum_vector(E_Y1[h][k], 0, range-1); */
/* 				E_X2_size[h][k] = 1; */

/* 				E_X2[k][h][0] = E_X2[h][k][0]; */
/*  				E_Y2[k][h][0] = E_Y2[h][k][0]; */
/* 				E_X2_size[k][h] = E_X2_size[h][k]; */

/* 				break; */
/* 			} //end of if-1 */

/* 			for(i = 0; i < range; i++) //for-3: i is the time difference value and range is the end point where the sliding window can move for moving average */
/* 			{ */
/* 				if(i > range-window_size) //if-2 //equals (range-1) - (window_size-1) */
/* 				{ */
/* 					E_X2[h][k][i] = (float)ceil(mean_integer(i, range)); */
/* 					E_Y2[h][k][i] = sum_vector(E_Y1[h][k], i, range-1); */
/* 					E_X2_size[h][k]++; */

/* 					E_X2[k][h][i] = E_X2[h][k][i]; */
/* 					E_Y2[k][h][i] = E_Y2[h][k][i]; */
/* 					E_X2_size[k][h] = E_X2_size[h][k]; */

/* 				} //end of if-2 */
/* 				else //else-2 */
/* 				{ */
/* 					E_X2[h][k][i] = (float)ceil(mean_integer(i, i+window_size-1)); */
/* 					E_Y2[h][k][i] = sum_vector(E_Y1[h][k], i, i+window_size-1); */
/* 					E_X2_size[h][k]++; */

/* 					E_X2[k][h][i] = E_X2[h][k][i]; */
/* 					E_Y2[k][h][i] = E_Y2[h][k][i]; */
/* 					E_X2_size[k][h] = E_X2_size[h][k]; */
/* 				} //end of else-2 */
/* 			} //end of for-3			 */
/* 		} //end of for-2 */
/* 	} //end of for-1 */

/* 	/\* find the time difference with the maximum frequency for each virtual edge  */
/* 	   for non-aggregation method *\/ */
/* 	for(h = 0; h < nodenum; h++) //for-1 */
/* 	{ */
/* 		for(k = 0; k < nodenum; k++) //for-2 */
/* 		{ */
/* 			for(i = 0; i < range; i++) //for-3 */
/* 			{ */
/* 				if(E_Y1_max[h][k] < E_Y1[h][k][i]) */
/* 				{ */
/* 					E_Y1_max[h][k] = E_Y1[h][k][i]; */
/* 					E_X1_max[h][k] = E_X1[h][k][i]; */
/* 				} */
/* 			} //end of for-3 */
/* 		} //end of for-2 */
/* 	} //end of for-1 */

/* 	/\* find the time difference with the maximum frequency for each virtual edge  */
/* 	   for aggregation method *\/ */
/* 	for(h = 0; h < nodenum; h++) //for-1 */
/* 	{ */
/* 		for(k = 0; k < nodenum; k++) //for-2 */
/* 		{ */
/* 			for(i = 0; i < E_X2_size[h][k]; i++) //for-3 */
/* 			{ */
/* 				if(E_Y2_max[h][k] < E_Y2[h][k][i]) */
/* 				{ */
/* 					E_Y2_max[h][k] = E_Y2[h][k][i]; */
/* 					E_X2_max[h][k] = E_X2[h][k][i]; */
/* 				} */
/* 			} //end of for-3 */
/* 		} //end of for-2 */
/* 	} //end of for-1 */

/* 	/\* rescale E_X1_max and E_X2_max with fp_scale *\/ */
/* 	for(h = 0; h < nodenum; h++) //for-1 */
/* 	{ */
/* 		for(k = 0; k < nodenum; k++) //for-2 */
/* 		{ */
/* 		  E_X1_max[h][k] /= fp_scale; */
/* 		  E_X2_max[h][k] /= fp_scale; */
/* 		} //end of for-2 */
/* 	} //end of for-1 */


/* 	/\* make names for virtual topology files *\/ */
/* 	strcpy(localization_filename_buf, localization_file); */
/* 	/\* */
/* 	token = strtok(localization_filename_buf, "."); */
/* 	if(token == NULL) */
/* 	{ */
/* 		printf("get_virtual_topology(): error: token is NULL\n"); */
/* 		exit(1); */
/* 	} */
/* 	*\/ */

/* 	ptr = rindex(localization_filename_buf, '.'); */
/* 	*ptr = '\0'; */
	
/* 	memset(virtual_topology_file_name1, 0, sizeof(virtual_topology_file_name1)); */
/* 	memset(virtual_topology_file_name2, 0, sizeof(virtual_topology_file_name2)); */

/* 	strcpy(virtual_topology_file_name1, localization_filename_buf); */
/* 	strcat(virtual_topology_file_name1, virtual_topology_file_suffix1); */

/* 	strcpy(virtual_topology_file_name2, localization_filename_buf); */
/* 	strcat(virtual_topology_file_name2, virtual_topology_file_suffix2); */

/* 	/\* store E_X1_max and E_X2_max into files Mv1.txt and Mv2.txt *\/ */
/* 	store_matrix_into_file(E_X1_max, nodenum, virtual_topology_file_name1); */
/* 	//store the two-dimensional matrix E_X1_max into a file called virtual_topology_file_name1 */

/* 	store_matrix_into_file(E_X2_max, nodenum, virtual_topology_file_name2); */
/* 	//store the two-dimensional matrix E_X2_max into a file called virtual_topology_file_name2 */

/* 	/\* release memory allocated in this function *\/ */
/* 	free(NODE_ID); */
/* 	free(TIMESTAMP); */

/* 	for(i = 0; i < param->sensor_number; i++) */
/* 	{ */
/* 		free(TV[i]); */
/* 	} */
/* 	free(TV); */
/* 	free(TV_size); */

/* 	for(i = 0; i < nodenum; i++) */
/* 	{ */
/* 		for(j = 0; j < nodenum; j++) */
/* 		{ */
/* 			free(E_X1[i][j]); */
/* 			free(E_Y1[i][j]); */
/* 			free(E_X2[i][j]); */
/* 			free(E_Y2[i][j]); */
/* 		} */
/* 	} */

/* 	for(i = 0; i < nodenum; i++) */
/* 	{ */
/* 		free(E_X1[i]); */
/* 		free(E_Y1[i]); */
/* 		free(E_R1[i]); */
/* 		free(E_X2[i]); */
/* 		free(E_Y2[i]); */
/* 		free(E_R2[i]); */
/* 		free(E_X2_size[i]); */
/* 		free(E_X1_max[i]); */
/* 		free(E_Y1_max[i]); */
/* 		free(E_X2_max[i]); */
/* 		free(E_Y2_max[i]); */
/* 	} */

/* 	free(E_X1); */
/* 	free(E_Y1); */
/* 	free(E_R1); */
/* 	free(E_X2); */
/* 	free(E_Y2); */
/* 	free(E_R2); */
/* 	free(E_X2_size); */
/* 	free(E_X1_max); */
/* 	free(E_Y1_max); */
/* 	free(E_X2_max); */
/* 	free(E_Y2_max); */

/*     /\* I forgot to write "fclose()" here. Due to the omission of fclose(),  */
/* 	I cannot open a new file, since file handles are exhausted. *\/ */
/* 	fclose(fp); */

/* 	return virtual_topology_file_name2; */
/* } */

char* get_virtual_topology_based_on_nonaggregation_method(parameter_t *param, char *output_file, char *localization_file, int number)
{ //get a virtual topology (Mv) from simulation data including vehicle detection timestamps using Non-aggregation Method
	//int number; //number of timestamps in localization.txt
	int length = 0; //length of TIMESTAMP including only timestamps for vehicle detection event (or VEHICLE_CHECK) 
	//size_t size; //allocated memory size
	//char pathtable_file[BUF_SIZE] = PATHTABLE_FILE; //path-table file
	//char adjacency_matrix_file[BUF_SIZE] = ADJACENCY_MATRIX_FILE; //adjacency matrix file
	int nodenum = param->sensor_number; //number of sensor nodes
	double speed = param->vehicle_speed; //vehicle speed	
	double speed_standard_deviation = param->vehicle_speed_standard_deviation; 
	//standard deviation of vehicle speed
	int window_size; //window size for aggregation
	//int window_size_factor = (int) param->data_aggregation_window_size; //window size factor
	int max_road_segment_length; //maximum road segment length
	int range; //range for time difference
	int range_factor; //range factor
	//////////////////////////////////////////////////////////////
	//int fp_digits; //number of the digits under floating point
	//int fp_factor; //factor to increase the scale of time difference axis according to fp_digits
	int fp_scale; //scale for time difference which is pow(fp_factor, fp_digits)
	//////////////////////////////////////////////////////////////
	int *NODE_ID = NULL; //vector for node ID
	float *TIMESTAMP = NULL; //vector for timestamp
	FILE *fp = NULL; //file pointer to output file
	int i, j, k, h; //index for for-loop
	int index; //index for vectors
	float difference; //time difference
	int sensor_id; //sensor node ID
	float timestamp; //timestamp
	int event; //event
	int vehicle_id; //vehicle ID
	float **TV = NULL; //time vectors for sensors
	int *TV_size = NULL; //size of time vector for each sensor

	float ***E_X1 = NULL; //time difference without aggregation
	int ***E_Y1 = NULL; //frequency of time difference without aggregation
	int **E_R1 = NULL; //frequency of time difference greater than range without aggregation
	float **E_X1_max = NULL; //matrix for time difference with maximum frequency for non-aggregation method
	int **E_Y1_max = NULL; //matrix for maximum frequency for non-aggregation method

	/* output files for simulation */	
	char virtual_topology_file_suffix1[BUF_SIZE] = VIRTUAL_TOPOLOGY_FILE_SUFFIX_1; 
	//file suffix for virtual topology based on non-aggregation method
	static char virtual_topology_file_name1[BUF_SIZE]; 
	//file name for virtual topology based on non-aggregation method
	char *ptr; //pointer to string
	char localization_filename_buf[BUF_SIZE]; //buffer for localization file name

	/**@for debugging */
	/*
	static int file_num = 0;
	file_num++;

	if(file_num == 11) 
	    printf("file_num=%d\n", file_num);
	*/
	/******************/

	//number = get_number_of_timestamps(output_file);
	//get the number of timestamps in localization log file

	max_road_segment_length = get_max_road_segment_length(output_file);
	//get the maximum road segment length in real graph

	/* time difference range */
	//range_factor = RANGE_FACTOR;
	//fp_digits = FP_DIGITS; //decide how many floating digits are used
	//fp_scale = (int) pow(fp_factor, fp_digits);
	fp_scale = FP_SCALE_FACTOR; //we consider the first floating point
	//@Note: fp_scale is needed to set considering max time sync error and vehicle speed deviation

	/* range is the travel time bound considering the vehicle speed and the vehicle speed deviation */
	range = (int) (max_road_segment_length/speed*1.5*fp_scale); //note: the time difference should be less than 1.5 times the travel time for the maximum road segment multipled by fp_scale: why?
	//range = (int) (max_road_segment_length/(speed - 3*speed_standard_deviation)*fp_scale); //note: the time difference should be less than the travel time for the maximum road segment multipled by fp_scale: the unit of range is scaled down by fp_scale: e.g., 1 -> 0.1 sec
	//note: the time difference should be less than range: 3*speed_standard_deviation covers the 99% of possible speed range


	//VEHICLE_CHECK: detection event
	
	/* make vectors NODE_ID and TIMESTAMP */
	NODE_ID = (int*) calloc(number, sizeof(int));
	assert_memory(NODE_ID);

	TIMESTAMP = (float*) calloc(number, sizeof(float));
	assert_memory(TIMESTAMP);

	/* open localization_file and store localization_file into arrays, such as NODE_ID */
	fp = fopen(localization_file, "r");
	if(!fp)
	{
	    fprintf(stderr, "error : unable to open file \"%s\"\n", localization_file);
	    exit(1);
	}

	for(i = 0; i < number; i++)
	{
		fscanf(fp, "%d %f %d %d", &sensor_id, &timestamp, &event, &vehicle_id);
		if(event == VEHICLE_CHECK)
		{
			NODE_ID[length] = sensor_id;
			TIMESTAMP[length++] = timestamp;			
		}
	}

	/* realloc the memory fof NODE_ID and TIMESTAMP */
	NODE_ID = (int*) realloc(NODE_ID, length*sizeof(int));
	assert_memory(NODE_ID);

	TIMESTAMP = (float*) realloc(TIMESTAMP, length*sizeof(float));
	assert_memory(TIMESTAMP);

	/* allocate the memory for TV and TV_size */
	TV = (float**) calloc(param->sensor_number, sizeof(float*));
	assert_memory(TV);

	for(i = 0; i < param->sensor_number; i++)
	{
		TV[i] = (float*) calloc(length, sizeof(float));
		assert_memory(TV[i]);
	}

	TV_size = (int*) calloc(param->sensor_number, sizeof(int));
	//calloc() initializes each element to 0.
	assert_memory(TV_size);

	/* store timestamps corresponding to node ID (i.e., i) into TV[i] */
	
	for(k = 0; k < length; k++)
	{
		i = NODE_ID[k]-1;
		j = TV_size[i]++;
		TV[i][j] = TIMESTAMP[k];		
	}

	/* allocate the memory for E_X1, E_Y1, and so on */
	E_X1 = (float***) calloc(nodenum, sizeof(float**));
	assert_memory(E_X1);

	E_Y1 = (int***) calloc(nodenum, sizeof(int**));
	assert_memory(E_Y1);

	E_R1 = (int**) calloc(nodenum, sizeof(int*));
	assert_memory(E_R1);

	E_X1_max = (float**) calloc(nodenum, sizeof(float*));
	assert_memory(E_X1_max); 

	E_Y1_max = (int**) calloc(nodenum, sizeof(int*));
	assert_memory(E_Y1_max); 

	for(i = 0; i < nodenum; i++)
	{
		E_X1[i] = (float**) calloc(nodenum, sizeof(float*));
		assert_memory(E_X1[i]);

		E_Y1[i] = (int**) calloc(nodenum, sizeof(int*));
		assert_memory(E_Y1[i]);

		E_R1[i] = (int*) calloc(nodenum, sizeof(int));
		assert_memory(E_R1[i]);

		E_X1_max[i] = (float*) calloc(nodenum, sizeof(float));
		assert_memory(E_X1_max[i]);

		E_Y1_max[i] = (int*) calloc(nodenum, sizeof(int));
		assert_memory(E_Y1_max[i]);
	}

	/** Slow version: memory allocation for E_X1[] and E_Y1[] */
/* 	for(i = 0; i < nodenum; i++) */
/* 	{ */
/* 		for(j = 0; j < nodenum; j++) */
/* 		{ */
/* 			E_X1[i][j] = (float*) calloc(range, sizeof(float)); */
/* 			assert_memory(E_X1[i][j]); */

/* 			E_Y1[i][j] = (int*) calloc(range, sizeof(int)); */
/* 			assert_memory(E_Y1[i][j]); */
/* 		} */
/* 	} */

	/** Fast version: memory allocation for E_X1[] and E_Y1[] */
	for(i = 0; i < nodenum; i++)
	{
		E_X1[i][i] = (float*) calloc(range, sizeof(float));
		assert_memory(E_X1[i][i]);

		E_Y1[i][i] = (int*) calloc(range, sizeof(int));
		assert_memory(E_Y1[i][i]);

		for(j = i+1; j < nodenum; j++)
		{
			E_X1[i][j] = (float*) calloc(range, sizeof(float));
			assert_memory(E_X1[i][j]);

			E_Y1[i][j] = (int*) calloc(range, sizeof(int));
			assert_memory(E_Y1[i][j]);

			E_X1[j][i] = (float*) calloc(range, sizeof(float));
			assert_memory(E_X1[j][i]);

			E_Y1[j][i] = (int*) calloc(range, sizeof(int));
			assert_memory(E_Y1[j][i]);
		}
	}

	/** SLOW comutation: compute the time difference frequency for non-aggregation method */
/* 	for(h = 0; h < nodenum; h++) //for-1 */
/* 	{ */
/* 		for(k = 0; k < nodenum; k++) //for-2 */
/* 		{ */
/* 			//set E_X1[h][k] */
/* 			for(i = 0; i < range; i++) //for-3 */
/* 			{ */
/* 				E_X1[h][k][i] = (float)i; */
/* 			} //end of for-3 */

/* 			if(h == k) continue; */

/* 			//set E_Y1[h][k] and E_R1[h][k] */
/* 			for(i = 0; i < TV_size[h]; i++) //for-4 */
/* 			{ */
/* 				for(j = 0; j < TV_size[k]; j++) //for-5 */
/* 				{ */
/* 					//difference = abs((int)(TV[h][i]*fp_scale - TV[k][j]*fp_scale)); */
/* 					//difference = (float)ceil(TV[h][i]*fp_scale - TV[k][j]*fp_scale); */
/* 					//index = (int)abs(difference); */
/* 					//difference = (float)fabs(difference); */
/* 					//index = (int)difference; */
/* 				        difference = (float)fabs(TV[h][i]*fp_scale - TV[k][j]*fp_scale); */
/* 					index = (int)round(difference); */
					
/* 					if(index >= range) E_R1[h][k]++; */
/* 					else E_Y1[h][k][index]++; */
/* 				} //end of for-5 */
/* 			} //end of for-4 */
/* 		} //end of for-2 */
/* 	} //end of for-1 */

	/** FAST comutation: compute the time difference frequency for non-aggregation method */
	for(h = 0; h < nodenum; h++) //for-1
	{
		for(k = h+1; k < nodenum; k++) //for-2
		{
			//set E_X1[h][k]
			for(i = 0; i < range; i++) //for-3
			{
				E_X1[h][k][i] = (float)i;
				//set the same value in symmetric entry using the propoery of symmetric matrix
				E_X1[k][h][i] = (float)i;
			} //end of for-3

			//set E_Y1[h][k] and E_R1[h][k]
			for(i = 0; i < TV_size[h]; i++) //for-4
			{
				for(j = 0; j < TV_size[k]; j++) //for-5
				{
				        //difference = (float)ceil(TV[h][i] - TV[k][j]); //@04/30/09: note that the time difference is rounded up to the second
				        //difference = (float)ceil(TV[h][i]*fp_scale - TV[k][j]*fp_scale);
					//difference = (float)fabs(difference);
                                        //difference = (ftloat)fabs(TV[h][i] - TV[k][j]);
				        difference = (float)fabs(TV[h][i]*fp_scale - TV[k][j]*fp_scale);
					index = (int)round(difference);
					//index = (int)difference;
					
					if(index >= range) 
					{
						E_R1[h][k]++;
						//increase the same values in symmetric entries using the propoery of symmetric matrix
						E_R1[k][h]++;
					}
					else 
					{
						E_Y1[h][k][index]++;
						//increase the same values in symmetric entries using the propoery of symmetric matrix
						E_Y1[k][h][index]++;
					}
				} //end of for-5
			} //end of for-4
		} //end of for-2
	} //end of for-1

	/* Slow version: find the time difference with the maximum frequency for each virtual edge 
	   for non-aggregation method */
/* 	for(h = 0; h < nodenum; h++) //for-1 */
/* 	{ */
/* 		for(k = 0; k < nodenum; k++) //for-2 */
/* 		{ */
/* 			for(i = 0; i < range; i++) //for-3 */
/* 			{ */
/* 				if(E_Y1_max[h][k] < E_Y1[h][k][i]) */
/* 				{ */
/* 					E_Y1_max[h][k] = E_Y1[h][k][i]; */
/* 					E_X1_max[h][k] = E_X1[h][k][i]; */
/* 				} */
/* 			} //end of for-3 */
/* 		} //end of for-2 */
/* 	} //end of for-1 */

	/* Fast version: find the time difference with the maximum frequency for each virtual edge 
	   for non-aggregation method */
	for(h = 0; h < nodenum; h++) //for-1
	{
		for(k = h+1; k < nodenum; k++) //for-2
		{
			for(i = 0; i < range; i++) //for-3
			{
				if(E_Y1_max[h][k] < E_Y1[h][k][i])
				{
					E_Y1_max[h][k] = E_Y1[h][k][i];
					E_X1_max[h][k] = E_X1[h][k][i];

					E_Y1_max[k][h] = E_Y1_max[h][k];
					E_X1_max[k][h] = E_X1_max[h][k];
				}
			} //end of for-3
		} //end of for-2
	} //end of for-1

	/* make names for virtual topology files */
	strcpy(localization_filename_buf, localization_file);
	/*
	token = strtok(localization_filename_buf, ".");
	if(token == NULL)
	{
		printf("get_virtual_topology(): error: token is NULL\n");
		exit(1);
	}
	*/

	ptr = rindex(localization_filename_buf, '.');
	*ptr = '\0';
	
	memset(virtual_topology_file_name1, 0, sizeof(virtual_topology_file_name1));

	strcpy(virtual_topology_file_name1, localization_filename_buf);
	strcat(virtual_topology_file_name1, virtual_topology_file_suffix1);

	/* store E_X1_max into files Mv1.txt */
	store_matrix_into_file(E_X1_max, nodenum, virtual_topology_file_name1);
	//store the two-dimensional matrix E_X1_max into a file called virtual_topology_file_name1

	/* release memory allocated in this function */
	free(NODE_ID);
	free(TIMESTAMP);

	for(i = 0; i < param->sensor_number; i++)
	{
		free(TV[i]);
	}
	free(TV);
	free(TV_size);

	/* Slow version: memory deallocation of E_X1[] and E_Y1[] */
/* 	for(i = 0; i < nodenum; i++) */
/* 	{ */
/* 		for(j = 0; j < nodenum; j++) */
/* 		{ */
/* 			free(E_X1[i][j]); */
/* 			free(E_Y1[i][j]); */
/* 		} */
/* 	} */

	/* Fast version: memory deallocation of E_X1[] and E_Y1[] */
	for(i = 0; i < nodenum; i++)
	{
		free(E_X1[i][i]);
		free(E_Y1[i][i]);
	        
		for(j = i+1; j < nodenum; j++)
		{
			free(E_X1[i][j]);
			free(E_Y1[i][j]);

			free(E_X1[j][i]);
			free(E_Y1[j][i]);
		}
	}

	for(i = 0; i < nodenum; i++)
	{
		free(E_X1[i]);
		free(E_Y1[i]);
		free(E_R1[i]);
		free(E_X1_max[i]);
		free(E_Y1_max[i]);
	}

	free(E_X1);
	free(E_Y1);
	free(E_R1);
	free(E_X1_max);
	free(E_Y1_max);

    /* I forgot to write "fclose()" here. Due to the omission of fclose(), 
	I cannot open a new file, since file handles are exhausted. */
	fclose(fp);

	return virtual_topology_file_name1;
}

char* get_virtual_topology_based_on_aggregation_method(parameter_t *param, char *output_file, char *localization_file, int number)
{ //get a virtual topology (Mv) from simulation data including vehicle detection timestamps using Aggregation Method
	//int number; //number of timestamps in localization.txt
	int length = 0; //length of TIMESTAMP including only timestamps for vehicle detection event (or VEHICLE_CHECK) 
	//size_t size; //allocated memory size
	//char pathtable_file[BUF_SIZE] = PATHTABLE_FILE; //path-table file
	//char adjacency_matrix_file[BUF_SIZE] = ADJACENCY_MATRIX_FILE; //adjacency matrix file
	int nodenum = param->sensor_number; //number of sensor nodes
	double speed = param->vehicle_speed; //vehicle speed	
	double speed_standard_deviation = param->vehicle_speed_standard_deviation; 
	//standard deviation of vehicle speed
	int window_size; //window size for aggregation
	//int window_size_factor = (int) param->data_aggregation_window_size; //window size factor
	int max_road_segment_length; //maximum road segment length
	int range; //range for time difference
	int range_factor; //range factor
	//////////////////////////////////////////////////////////////
	//int fp_digits; //number of the digits under floating point
	//int fp_factor; //factor to increase the scale of time difference axis according to fp_digits
	int fp_scale; //scale for time difference which is pow(fp_factor, fp_digits)
	//////////////////////////////////////////////////////////////
	int *NODE_ID = NULL; //vector for node ID
	float *TIMESTAMP = NULL; //vector for timestamp
	FILE *fp = NULL; //file pointer to output file
	int i, j, k, h; //index for for-loop
	int index; //index for vectors
	float difference; //time difference
	int sensor_id; //sensor node ID
	float timestamp; //timestamp
	int event; //event
	int vehicle_id; //vehicle ID
	float **TV = NULL; //time vectors for sensors
	int *TV_size = NULL; //size of time vector for each sensor

	////////////////////////////////////////////////////////
	float ***E_X1 = NULL; //time difference without aggregation
	int ***E_Y1 = NULL; //frequency of time difference without aggregation
	int **E_R1 = NULL; //frequency of time difference greater than range without aggregation

	////////////////////////////////////////////////////////
	float ***E_X2 = NULL; //time difference with aggregation
	int ***E_Y2 = NULL; //frequency of time difference with aggregation	
	int **E_R2 = NULL; //frequency of time difference greater than range with aggregation
	int **E_X2_size = NULL; //actual size of aggregation vecto
 	float **E_X2_max = NULL; //matrix for time difference with maximum frequency for aggregation method
 	int **E_Y2_max = NULL; //matrix for maximum frequency for aggregation methoda

	/* output files for simulation */	
	char virtual_topology_file_suffix2[BUF_SIZE] = VIRTUAL_TOPOLOGY_FILE_SUFFIX_2; 
	//file suffix for virtual topology based on aggregation method
	static char virtual_topology_file_name2[BUF_SIZE]; 
	//file name for virtual topology based on aggregation method
	char *ptr; //pointer to string
	char localization_filename_buf[BUF_SIZE]; //buffer for localization file name

	/**@for debugging */
	/*
	static int file_num = 0;
	file_num++;

	if(file_num == 11) 
	    printf("file_num=%d\n", file_num);
	*/
	/******************/

	//number = get_number_of_timestamps(output_file);
	//get the number of timestamps in localization log file

	max_road_segment_length = get_max_road_segment_length(output_file);
	//get the maximum road segment length in real graph

	/* time difference range */
	//range_factor = RANGE_FACTOR;
	//fp_digits = FP_DIGITS; //decide how many floating digits are used
	//fp_scale = (int) pow(fp_factor, fp_digits);
	fp_scale = FP_SCALE_FACTOR; //we consider the first floating point
	//@Note: fp_scale is needed to set considering max time sync error and vehicle speed deviation

	/* range is the travel time bound considering the vehicle speed and the vehicle speed deviation */
	range = (int) (max_road_segment_length/speed*1.5*fp_scale); //note: the time difference should be less than 1.5 times the travel time for the maximum road segment multipled by fp_scale: why?
	//range = (int) (max_road_segment_length/(speed - 3*speed_standard_deviation)*fp_scale); //note: the time difference should be less than the travel time for the maximum road segment multipled by fp_scale: the unit of range is scaled down by fp_scale: e.g., 1 -> 0.1 sec
	//note: the time difference should be less than range: 3*speed_standard_deviation covers the 99% of possible speed range

	//VEHICLE_CHECK: detection event
	
	/* make vectors NODE_ID and TIMESTAMP */
	NODE_ID = (int*) calloc(number, sizeof(int));
	assert_memory(NODE_ID);

	TIMESTAMP = (float*) calloc(number, sizeof(float));
	assert_memory(TIMESTAMP);

	/* open localization_file and store localization_file into arrays, such as NODE_ID */
	fp = fopen(localization_file, "r");
	if(!fp)
	{
	    fprintf(stderr, "error : unable to open file \"%s\"\n", localization_file);
	    exit(1);
	}

	for(i = 0; i < number; i++)
	{
		fscanf(fp, "%d %f %d %d", &sensor_id, &timestamp, &event, &vehicle_id);
		if(event == VEHICLE_CHECK)
		{
			NODE_ID[length] = sensor_id;
			TIMESTAMP[length++] = timestamp;			
		}
	}

	/* realloc the memory fof NODE_ID and TIMESTAMP */
	NODE_ID = (int*) realloc(NODE_ID, length*sizeof(int));
	assert_memory(NODE_ID);

	TIMESTAMP = (float*) realloc(TIMESTAMP, length*sizeof(float));
	assert_memory(TIMESTAMP);

	/* allocate the memory for TV and TV_size */
	TV = (float**) calloc(param->sensor_number, sizeof(float*));
	assert_memory(TV);

	for(i = 0; i < param->sensor_number; i++)
	{
		TV[i] = (float*) calloc(length, sizeof(float));
		assert_memory(TV[i]);
	}

	TV_size = (int*) calloc(param->sensor_number, sizeof(int));
	//calloc() initializes each element to 0.
	assert_memory(TV_size);

	/* store timestamps corresponding to node ID (i.e., i) into TV[i] */
	
	for(k = 0; k < length; k++)
	{
		i = NODE_ID[k]-1;
		j = TV_size[i]++;
		TV[i][j] = TIMESTAMP[k];		
	}

	/* allocate the memory for E_X1, E_Y1, and so on */
	E_X1 = (float***) calloc(nodenum, sizeof(float**));
	assert_memory(E_X1);

	E_Y1 = (int***) calloc(nodenum, sizeof(int**));
	assert_memory(E_Y1);

	E_R1 = (int**) calloc(nodenum, sizeof(int*));
	assert_memory(E_R1);

	/* allocate the memory for E_X2, E_Y2, and so on */
	E_X2 = (float***) calloc(nodenum, sizeof(float**));
	assert_memory(E_X2);

	E_Y2 = (int***) calloc(nodenum, sizeof(int**));
	assert_memory(E_Y2);

	E_R2 = (int**) calloc(nodenum, sizeof(int*));
	assert_memory(E_R2);

	E_X2_size = (int**) calloc(nodenum, sizeof(int*));
	assert_memory(E_X2_size);

	E_X2_max = (float**) calloc(nodenum, sizeof(float*));
	assert_memory(E_X2_max); 

	E_Y2_max = (int**) calloc(nodenum, sizeof(int*));
	assert_memory(E_Y2_max); 

	for(i = 0; i < nodenum; i++)
	{
		E_X1[i] = (float**) calloc(nodenum, sizeof(float*));
		assert_memory(E_X1[i]);

		E_Y1[i] = (int**) calloc(nodenum, sizeof(int*));
		assert_memory(E_Y1[i]);

		E_R1[i] = (int*) calloc(nodenum, sizeof(int));
		assert_memory(E_R1[i]);

                ////////////////////////////////////////////////////
		E_X2[i] = (float**) calloc(nodenum, sizeof(float*));
		assert_memory(E_X2[i]);

		E_Y2[i] = (int**) calloc(nodenum, sizeof(int*));
		assert_memory(E_Y2[i]);

		E_R2[i] = (int*) calloc(nodenum, sizeof(int));
		assert_memory(E_R2[i]);

		E_X2_size[i] = (int*) calloc(nodenum, sizeof(int));
		assert_memory(E_X2_size[i]);

		E_X2_max[i] = (float*) calloc(nodenum, sizeof(float));
		assert_memory(E_X2_max[i]);

		E_Y2_max[i] = (int*) calloc(nodenum, sizeof(int));
		assert_memory(E_Y2_max[i]);
	}

	/** Slow version: memory allocation for E_X2[] and E_Y2[] */	
/* 	for(i = 0; i < nodenum; i++) */
/* 	{ */
/* 		for(j = 0; j < nodenum; j++) */
/* 		{ */
/* 			E_X2[i][j] = (float*) calloc(range, sizeof(float)); */
/* 			assert_memory(E_X2[i][j]); */

/* 			E_Y2[i][j] = (int*) calloc(range, sizeof(int)); */
/* 			assert_memory(E_Y2[i][j]); */
/* 		} */
/* 	} */

	/** Fast version: memory allocation for E_X2[] and E_Y2[] */	
	for(i = 0; i < nodenum; i++)
	{
		E_X1[i][i] = (float*) calloc(range, sizeof(float));
		assert_memory(E_X1[i][i]);

		E_Y1[i][i] = (int*) calloc(range, sizeof(int));
		assert_memory(E_Y1[i][i]);

		///////////////////////////////////////////////////
		E_X2[i][i] = (float*) calloc(range, sizeof(float));
		assert_memory(E_X2[i][i]);

		E_Y2[i][i] = (int*) calloc(range, sizeof(int));
		assert_memory(E_Y2[i][i]);

		for(j = i+1; j < nodenum; j++)
		{
			E_X1[i][j] = (float*) calloc(range, sizeof(float));
			assert_memory(E_X1[i][j]);

			E_Y1[i][j] = (int*) calloc(range, sizeof(int));
			assert_memory(E_Y1[i][j]);

			E_X1[j][i] = (float*) calloc(range, sizeof(float));
			assert_memory(E_X1[j][i]);

			E_Y1[j][i] = (int*) calloc(range, sizeof(int));
			assert_memory(E_Y1[j][i]);

			///////////////////////////////////////////////////
			E_X2[i][j] = (float*) calloc(range, sizeof(float));
			assert_memory(E_X2[i][j]);

			E_Y2[i][j] = (int*) calloc(range, sizeof(int));
			assert_memory(E_Y2[i][j]);

			E_X2[j][i] = (float*) calloc(range, sizeof(float));
			assert_memory(E_X2[j][i]);

			E_Y2[j][i] = (int*) calloc(range, sizeof(int));
			assert_memory(E_Y2[j][i]);
		}
	}

	/** compute the time difference frequency for aggregation method */
	//if(speed_standard_deviation == 0)
	//	speed_standard_deviation = 1.0; //make sure that speed_standard_deviation cannot be zero due to non-zero window_size.

	//window_size = (int)ceil(speed_standard_deviation*window_size_factor); //window size for aggregation
	window_size = param->data_aggregation_window_size;

	/** FAST comutation: compute the time difference frequency for non-aggregation method */
	for(h = 0; h < nodenum; h++) //for-1
	{
		for(k = h+1; k < nodenum; k++) //for-2
		{
			//set E_X1[h][k]
			for(i = 0; i < range; i++) //for-3
			{
				E_X1[h][k][i] = (float)i;
				//set the same value in symmetric entry using the propoery of symmetric matrix
				E_X1[k][h][i] = (float)i;
			} //end of for-3

			//set E_Y1[h][k] and E_R1[h][k]
			for(i = 0; i < TV_size[h]; i++) //for-4
			{
				for(j = 0; j < TV_size[k]; j++) //for-5
				{
				        difference = (float)fabs(TV[h][i]*fp_scale - TV[k][j]*fp_scale);
					index = (int)round(difference);
					
					if(index >= range) 
					{
						E_R1[h][k]++;
						//increase the same values in symmetric entries using the propoery of symmetric matrix
						E_R1[k][h]++;
					}
					else 
					{
						E_Y1[h][k][index]++;
						//increase the same values in symmetric entries using the propoery of symmetric matrix
						E_Y1[k][h][index]++;
					}
				} //end of for-5
			} //end of for-4
		} //end of for-2
	} //end of for-1

	/** Slow version for aggregation method */
/* 	for(h = 0; h < nodenum; h++) //for-1 */
/* 	{ */
/* 		for(k = 0; k < nodenum; k++) //for-2 */
/* 		{ */
/* 			if(h == k) continue; */

/* 			if(range <= window_size) //if-1 */
/* 			{ */
/* 				//E_X2[h][k][0] = floorf(mean_integer(1, range)); */
/* 				E_X2[h][k][0] = (float)floor(mean_integer(1, range)); //this x value is the representative for two sensors' timestamps */
/* 				E_Y2[h][k][0] = sum_vector(E_Y1[h][k], 0, range-1); //E_Y1[h][k] is used to compute E_Y2[h][k][0] */
/* 				E_X2_size[h][k] = 1; */
/* 				break; */
/* 			} //end of if-1 */

/* 			for(i = 0; i < range; i++) //for-3: i is the time difference value and range is the end point where the sliding window can move for moving average */
/* 			{ */
/* 				if(i > range-window_size) //if-2 //equals (range-1) - (window_size-1) */
/* 				{ */
/* 					//E_X2[h][k][i] = ceilf(mean_integer(i, range)); */
/* 					E_X2[h][k][i] = (float)ceil(mean_integer(i, range)); */
/* 					//E_X2[h][k][i] = (int)floor(mean_integer(i, range)); */
/* 					E_Y2[h][k][i] = sum_vector(E_Y1[h][k], i, range-1); //E_Y1[h][k] is used to compute E_Y2[h][k][i] */
/* 					E_X2_size[h][k]++; */
/* 				} //end of if-2 */
/* 				else //else-2 */
/* 				{ */
/* 					//E_X2[h][k][i] = ceilf(mean_integer(i, i+window_size-1)); */
/* 					E_X2[h][k][i] = (float)ceil(mean_integer(i, i+window_size-1)); */
/* 					//E_X2[h][k][i] = (int)floor(mean_integer(i, i+window_size-1)); */
/* 					E_Y2[h][k][i] = sum_vector(E_Y1[h][k], i, i+window_size-1); */
/* 					E_X2_size[h][k]++;					 */
/* 				} //end of else-2 */
/* 			} //end of for-3			 */
/* 		} //end of for-2 */
/* 	} //end of for-1 */

	/** Fast version for aggregation method */
	for(h = 0; h < nodenum; h++) //for-1
	{
		for(k = h+1; k < nodenum; k++) //for-2
		{
			if(range <= window_size) //if-1
			{
				E_X2[h][k][0] = (float)floor(mean_integer(1, range)); //this x value is the representative for two sensors' timestamps
				E_Y2[h][k][0] = sum_vector(E_Y1[h][k], 0, range-1); //E_Y1[h][k] is used to compute E_Y2[h][k][0]
				E_X2_size[h][k] = 1;

				E_X2[k][h][0] = E_X2[h][k][0];
 				E_Y2[k][h][0] = E_Y2[h][k][0];
				E_X2_size[k][h] = E_X2_size[h][k];

				break;
			} //end of if-1

			for(i = 0; i < range; i++) //for-3: i is the time difference value and range is the end point where the sliding window can move for moving average
			{
				if(i > range-window_size) //if-2 //equals (range-1) - (window_size-1)
				{
					E_X2[h][k][i] = (float)ceil(mean_integer(i, range));
					E_Y2[h][k][i] = sum_vector(E_Y1[h][k], i, range-1); //E_Y1[h][k] is used to compute E_Y2[h][k][i]
					E_X2_size[h][k]++;

					E_X2[k][h][i] = E_X2[h][k][i];
					E_Y2[k][h][i] = E_Y2[h][k][i];
					E_X2_size[k][h] = E_X2_size[h][k];

				} //end of if-2
				else //else-2
				{
					E_X2[h][k][i] = (float)ceil(mean_integer(i, i+window_size-1));
					E_Y2[h][k][i] = sum_vector(E_Y1[h][k], i, i+window_size-1); //E_Y1[h][k] is used to compute E_Y2[h][k][i]
					E_X2_size[h][k]++;

					E_X2[k][h][i] = E_X2[h][k][i];
					E_Y2[k][h][i] = E_Y2[h][k][i];
					E_X2_size[k][h] = E_X2_size[h][k];
				} //end of else-2
			} //end of for-3			
		} //end of for-2
	} //end of for-1

	/* Slow version: find the time difference with the maximum frequency for each virtual edge 
	   for aggregation method */
/* 	for(h = 0; h < nodenum; h++) //for-1 */
/* 	{ */
/* 		for(k = 0; k < nodenum; k++) //for-2 */
/* 		{ */
/* 			for(i = 0; i < E_X2_size[h][k]; i++) //for-3 */
/* 			{ */
/* 				if(E_Y2_max[h][k] < E_Y2[h][k][i]) */
/* 				{ */
/* 					E_Y2_max[h][k] = E_Y2[h][k][i]; */
/* 					E_X2_max[h][k] = E_X2[h][k][i]; */
/* 				} */
/* 			} //end of for-3 */
/* 		} //end of for-2 */
/* 	} //end of for-1 */

	/* Fast version: find the time difference with the maximum frequency for each virtual edge 
	   for aggregation method */
	for(h = 0; h < nodenum; h++) //for-1
	{
		for(k = h+1; k < nodenum; k++) //for-2
		{
			for(i = 0; i < E_X2_size[h][k]; i++) //for-3
			{
				if(E_Y2_max[h][k] < E_Y2[h][k][i])
				{
					E_Y2_max[h][k] = E_Y2[h][k][i];
					E_X2_max[h][k] = E_X2[h][k][i];

					E_Y2_max[k][h] = E_Y2_max[h][k];
					E_X2_max[k][h] = E_X2_max[h][k];
				}
			} //end of for-3
		} //end of for-2
	} //end of for-1

	/* rescale E_X2_max with fp_scale */
	for(h = 0; h < nodenum; h++) //for-1
	{
		for(k = 0; k < nodenum; k++) //for-2
		{
		  E_X2_max[h][k] /= fp_scale;
		} //end of for-2
	} //end of for-1

	/* make names for virtual topology files */
	strcpy(localization_filename_buf, localization_file);
	/*
	token = strtok(localization_filename_buf, ".");
	if(token == NULL)
	{
		printf("get_virtual_topology_based_on_aggregation_method(): error: token is NULL\n");
		exit(1);
	}
	*/

	ptr = rindex(localization_filename_buf, '.');
	*ptr = '\0';
	
	memset(virtual_topology_file_name2, 0, sizeof(virtual_topology_file_name2));

	strcpy(virtual_topology_file_name2, localization_filename_buf);
	strcat(virtual_topology_file_name2, virtual_topology_file_suffix2);

	/* store E_X2_max into file Mv2.txt */
	store_matrix_into_file(E_X2_max, nodenum, virtual_topology_file_name2);
	//store the two-dimensional matrix E_X2_max into a file called virtual_topology_file_name2

	/* release memory allocated in this function */
	free(NODE_ID);
	free(TIMESTAMP);

	for(i = 0; i < param->sensor_number; i++)
	{
		free(TV[i]);
	}
	free(TV);
	free(TV_size);

	/* Slow version: memory deallocation of E_X2[] and E_Y2[] */
/* 	for(i = 0; i < nodenum; i++) */
/* 	{ */
/* 		for(j = 0; j < nodenum; j++) */
/* 		{ */
/* 			free(E_X2[i][j]); */
/* 			free(E_Y2[i][j]); */
/* 		} */
/* 	} */

	/* Fast version: memory deallocation of E_X2[] and E_Y2[] */
	for(i = 0; i < nodenum; i++)
	{
		free(E_X1[i][i]);
		free(E_Y1[i][i]);

	        ////////////////
		free(E_X2[i][i]);
		free(E_Y2[i][i]);
	        
		for(j = i+1; j < nodenum; j++)
		{
			free(E_X1[i][j]);
			free(E_Y1[i][j]);

			free(E_X1[j][i]);
			free(E_Y1[j][i]);

		        /////////////////
			free(E_X2[i][j]);
			free(E_Y2[i][j]);

			free(E_X2[j][i]);
			free(E_Y2[j][i]);
		}
	}

	for(i = 0; i < nodenum; i++)
	{

		free(E_X1[i]);
		free(E_Y1[i]);
		free(E_R1[i]);

		//////////////
		free(E_X2[i]);
		free(E_Y2[i]);
		free(E_R2[i]);
		free(E_X2_size[i]);
		free(E_X2_max[i]);
		free(E_Y2_max[i]);
	}

	free(E_X1);
	free(E_Y1);
	free(E_R1);

	//////////
	free(E_X2);
	free(E_Y2);
	free(E_R2);
	free(E_X2_size);
	free(E_X2_max);
	free(E_Y2_max);

    /* I forgot to write "fclose()" here. Due to the omission of fclose(), 
	I cannot open a new file, since file handles are exhausted. */
	fclose(fp);

	return virtual_topology_file_name2;
}

int get_number_of_timestamps(char *output_file)
{ //get the number of timestamps in localization log file
	int number = -1; //number of timestamps
	FILE *fp;
	char input_buf[BUF_SIZE];
	size_t input_buf_size;
	char *token;

	/* open an output file indicated by output_name */
	fp = fopen(output_file, "r");
	if(!fp)
	{
		fprintf(stderr, "error: unable to open file \"%s\"\n", output_file);
		exit(1);
	}

	while(fgets(input_buf, sizeof(input_buf), fp) != NULL) //while-1
	{
		input_buf_size = strlen(input_buf);

		token = (char*) strtok(input_buf, "=");

		if(token == NULL)
		{
			printf("error: the key in the output file is null!\n");
			exit(1);
		}
		
		if((token[0] == '\n') || (token[0] == ' ') || (token[0] == '\t')) //this line is white space
			continue;

		if(strcmp(token, "seq_of_localization_log") == 0) //if
		{
			token = (char*) strtok(NULL, " \t");
			if(token == NULL)
			{
				printf("error: token is null!\n");
				exit(1);
			}

			number = atoi(token);
			break;
		} //end of if
	} //end of while

	/* I forgot to write "fclose()" here. Due to the omission of fclose(), 
	I cannot open a new file, since file handles are exhausted. */
	fclose(fp);

	return number;
}

int get_max_road_segment_length(char *output_file)
{ //get the maximum road segment length in real graph
	int length = -1; //number of timestamps
	FILE *fp;
	char input_buf[BUF_SIZE];
	size_t input_buf_size;
	char *token;

	/* open an output file indicated by output_name */
	fp = fopen(output_file, "r");
	if(!fp)
	{
		fprintf(stderr, "error: unable to open file \"%s\"\n", output_file);
		exit(1);
	}

	while(fgets(input_buf, sizeof(input_buf), fp) != NULL) //while-1
	{
		input_buf_size = strlen(input_buf);

		token = (char*) strtok(input_buf, "=");

		if(token == NULL)
		{
			printf("error: the key in the output file is null!\n");
			exit(1);
		}
		
		if((token[0] == '\n') || (token[0] == ' ') || (token[0] == '\t')) //this line is white space
			continue;

		if(strcmp(token, "max_road_segment_length") == 0) //if
		{
			token = (char*) strtok(NULL, " \t");
			if(token == NULL)
			{
				printf("error: token is null!\n");
				exit(1);
			}

			length = atoi(token);
			break;
		} //end of if
	} //end of while

	/* I forgot to write "fclose()" here. Due to the omission of fclose(), 
	I cannot open a new file, since file handles are exhausted. */
	fclose(fp);

	return length;
}

void store_matrix_into_file(float** A, int n, char* filename)
{ //store the two-dimensional nxn matrix A into a file called filename
	FILE *fp; //path-table file
	int i, j;
	char *msg = NULL;
	char *buf = NULL;
	//char msg[MSG_BUF_SIZE]; 
	//Where n is a large number, such as 18, the memory access violation happens.
	//So we need to dynamically allocate the memory for msg and buf according to n.
	//char buf[BUF_SIZE];
	const int number_string_length =  NUMBER_STRING_LENGTH; //length of number string: 20 bytes
	int buf_len; //buf length
	int msg_len; //msg length

	//buf_len = sizeof(char)*n*number_string_length;
	//buf = (char*) malloc(buf_len);

	buf_len = n*number_string_length;
	buf = (char*) calloc(buf_len, sizeof(char));
	assert_memory(buf);

	//msg_len = sizeof(char)*n*n*number_string_length;
	//msg = (char*) malloc(msg_len);
	//assert_memory(msg);
	//memset(msg, 0, msg_len);

	msg_len = n*n*number_string_length;
	msg = (char*) calloc(msg_len, sizeof(char));
	assert_memory(msg);

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
		fprintf(stderr, "error : unable to open file \"%s\"\n", filename);
		exit(1);
	}

	//printf(msg);
	fprintf(fp, msg);
	fclose(fp);

	/* release memory */
	free(buf);
	free(msg);
}

struct_matlab_localization_result* perform_enhanced_localization(Engine *matlab_ep, char *current_directory, int nodenum, int measurement_number, double speed, double speed_deviation, char *adjacency_matrix_filename, char *intersection_vector_filename, char *virtual_topology_filename_prefix, int data_aggregation_type, int data_prefilter_type)
{ //perform the APL localization according to the prefiltering type and the graph-matching algorithm
  struct_matlab_prefiltering_result *prefiltering_result = NULL; //prefiltering result
  struct_matlab_localization_result *localization_result = NULL; //localization result
  boolean nonintersection_sensor_localization_flag = TRUE; //flag for nonintersection sensor localization
   
  /* perform the prefiltering algorithm */
  prefiltering_result = matlab_perform_prefiltering(matlab_ep, current_directory, nodenum, measurement_number, speed, speed_deviation, adjacency_matrix_filename, intersection_vector_filename, virtual_topology_filename_prefix, data_aggregation_type, data_prefilter_type);

  /* perform the graph matching algorithm */
  switch(data_prefilter_type)
  {
  case PREFILTER_TYPE_0: 
  case PREFILTER_TYPE_1: 
  case PREFILTER_TYPE_2: 
  case PREFILTER_TYPE_3: 
    localization_result = matlab_perform_graph_matching(matlab_ep, current_directory, speed, adjacency_matrix_filename, intersection_vector_filename, data_aggregation_type, data_prefilter_type, prefiltering_result, nonintersection_sensor_localization_flag);

    break;

  case PREFILTER_TYPE_4:
    localization_result = perform_subgraph_matching(current_directory, nodenum, measurement_number, speed, adjacency_matrix_filename, intersection_vector_filename, virtual_topology_filename_prefix, data_aggregation_type, data_prefilter_type, prefiltering_result, nonintersection_sensor_localization_flag);

    break;

  default:
    printf("perform_enhanced_localization(): data_prefilter_type(%d) is not supported!\n", data_prefilter_type);
    exit(1);
  }

  return localization_result;
}

struct_matlab_localization_result* perform_subgraph_matching(char *current_directory, int nodenum, int measurement_number, double speed, char *adjacency_matrix_filename, char *intersection_vector_filename, char *virtual_topology_filename_prefix, int data_aggregation_type, int data_prefilter_type, struct_matlab_prefiltering_result* prefiltering_result, boolean nonintersection_sensor_localization_flag)
{ //perform the subgraph matching algorithm
  static struct_matlab_localization_result localization_result; //result of localization
  int result = 0;
  double error_ratio = 0; //error ratio of localization
  double E_ratio = 0; //ratio indicating the relative difference between Er and Ev
  double M_ratio = 0; //ratio indicating the relative difference between Mr and Mv

  int n; //the number of matched node pairs in graph matching
  int i; //index of for-loop

  node_id ni1[MAXNODES], ni2[MAXNODES]; //ni1[] is the list of sensor ids and ni2[] is the list of intersection ids matched with the sensor ids
  node_id perm_list[MAXNODES]; //the list of the sensor ids for the intersection ids, that is, the list of the original sensor ids before the permutation of the sensor ids
  int perm_list_size = 0; //the size of perm_list, that is, the number of intersection nodes
  node_id sensor_id_list[MAXNODES]; //the list of the intersection sensor ids
  int sensor_id_list_size = 0; //the size of sensor_id_list
  node_id intersection_id_list[MAXNODES]; //the list of the intersection ids in the road network
  int intersection_id_list_size = 0; //the size of intersection_id_list
  node_id sensor_index_list[MAXNODES]; //the list of the corresponding indices to the intersection sensors ids in sensor_id_list
  int sensor_index_list_size = 0; //the size of sensor_index_list
  double my_threshold = WEIGHT_DIFFERENCE_THRESHOLD; //threshold in the comparison of edge weights of two graphs

  NullAllocator<Empty> node_allocator; //node allocator for node attribute
  NewAllocator<Weight> edge_allocator; //edge allocator for edge attribute

  /* Open the files graph-1.grp and graph-2.grp */
  //ifstream in_1("graphs/sensornet-1.grp"); //small graph
  //ifstream in_2("graphs/roadnet-1.grp"); //large graph

  ifstream in_1(prefiltering_result->intersection_sensor_network_graph_filename); //small graph
  ifstream in_2(prefiltering_result->road_network_graph_filename); //large graph
  ifstream in_p(prefiltering_result->permutation_vector_filename); //permutation vector file
  ifstream in_i(prefiltering_result->intersection_pair_matrix_filename); //intersection pair matrix file

  /* Create the ARGLoader for graph-1 */
  StreamARGLoader<Empty, Weight> loader_1(&node_allocator, &edge_allocator, in_1);

  /* Create the ARGLoader for graph-2 */
  StreamARGLoader<Empty, Weight> loader_2(&node_allocator, &edge_allocator, in_2);

  /* Build the graph of graph-1 */
  ARGraph<Empty, Weight> graph_1(&loader_1);

  /* Build the graph of graph-2 */
  ARGraph<Empty, Weight> graph_2(&loader_2);

  /* Install the attribute destroyers */
  graph_1.SetEdgeDestroyer(new WeightDestroyer()); //destroy the attribute of class Weight in graph_1
  graph_2.SetEdgeDestroyer(new WeightDestroyer()); //destroy the attributes of class Weight in graph_2

  /* Install the attribute comparator
     Note: this needs to be done only graph_1 */
  graph_1.SetEdgeComparator(new WeightComparator(my_threshold));

  /* Create the initial state of the search space */
  //VF2SubState s0(&graph_1, &graph_2); //subgraph matching where graph_1 is a small graph and graph_2 is a large graph
  VF2MonoState s0(&graph_1, &graph_2); //monomorphism matching where graph_1 is a small graph and graph_2 is a large graph

  /* initialize result for prefiltering */
  localization_result.result = -1;
  localization_result.error_ratio = 1;
  localization_result.E_ratio = -1;
  localization_result.M_ratio = -1;

  /* perform the subgraph matching */
  if(!match(&s0, &n, ni1, ni2))
  {
    printf("No matching found!\n");
    return &localization_result;
  }

  printf("Found a matching with %d nodes:\n", n);

  for(i = 0; i < n; i++)
  {
    printf("\tNode %hd of graph 1 is paired with node %hd of graph 2\n", ni1[i], ni2[i]);
  }

  /* read permutation vector file into perm_list */
  result = read_permutation_vector_file(in_p, perm_list, &perm_list_size);

  /* read the intersection pair matrix consisting of sensor_id_list and intersection_id_list from intersection_pair_matrix_file 
     where vector sensor_id_list is the list of sensor node ids, vector intersection_id_list is the list of the corresponding 
     intersection ids, and vector sensor_index_list is the list of the index for sensor node id in sensor_id_list.
  */
  result = read_intersection_pair_matrix_file(in_i, sensor_id_list, &sensor_id_list_size, intersection_id_list, &intersection_id_list_size, sensor_index_list, &sensor_index_list_size);

  /* compute the localization error ratio */
  error_ratio = compute_localization_error_ratio(ni2, n, intersection_id_list, intersection_id_list_size);
  //error_ratio = compute_localization_error_ratio(ni2, n, perm_list, perm_list_size);

/*   /\* Create the output file *\/ */
/*   FILE *f = NULL; //output file */

/*   f = fopen("output.txt", "w"); */
/*   if(f == NULL) */
/*   { */
/*     cout<<"f is NULL!"<<endl; */
/*     exit(1); */
/*   } */

/*   match(&s0, my_visitor, f); */

/*   fclose(f); */

  /* return result for localization */
  localization_result.result = result;
  localization_result.error_ratio = (float)error_ratio;
  localization_result.E_ratio = (float)E_ratio;
  localization_result.M_ratio = (float)M_ratio;

  return &localization_result;
}

bool my_visitor(int n, node_id ni1[], node_id ni2[], void *user_data)
{ //the callback function to store a matching into file
  FILE *f = (FILE *)user_data;

  /* Print the matched pairs on the file */
  int i;
  for(i = 0; i < n; i++)
  {
    fprintf(f, "(%hd, %hd) ", ni1[i], ni2[i]);
  }
  fprintf(f, "\n");

  /* Return false to search for the next matching */
  return false;
}

int read_permutation_vector_file(ifstream &in_p, node_id perm_list[], int *perm_list_size)
{ //read permutation vector file containing the pairs of intersection id and sensor id into perm_list
  int result = 0;
  int size = 0;
  int i = 0;
  int index = 0; //index of perm_list
  int id = 0; //sensor id

  /* obtain the size of perm_list */
  in_p >> size;

  /* store the permutation vector in the file pointed by in_p into perm_list */
  for(i = 0; i < size; i++)
  {
    in_p >> index >> id;
    perm_list[index] = (node_id) (id+1);
    //perm_list[index] = (node_id)id;
  }

  /* set perm_list_size to size */
  *perm_list_size = size;

  return result;
}

int read_intersection_pair_matrix_file(ifstream &in_i, node_id sensor_id_list[], int *sensor_id_list_size, node_id intersection_id_list[], int *intersection_id_list_size, node_id sensor_index_list[], int *sensor_index_list_size)
{ /* read the intersection pair matrix consisting of sensor_id_list and intersection_id_list from intersection_pair_matrix_file 
     where vector sensor_id_list is the list of sensor node ids, vector intersection_id_list is the list of the corresponding 
     intersection ids, and vector sensor_index_list is the list of the index for sensor node id in sensor_id_list.
  */
  int result = 0;
  int number = 0;
  int i = 0;
  int sensor_id = 0; //intersection sensor id
  int intersection_id = 0; //intersection id
  int index = 0; //index of perm_list
  int id = 0; //sensor id

  /* obtain the number of rows in intersection pair matrix */
  in_i >> number;

  /* restore sensor_id_list and intersection_id_list from the file pointed by in_i along with sensor_index_list */
  for(i = 0; i < number; i++)
  {
    in_i >> sensor_id >> intersection_id;
    sensor_id_list[i] = (node_id) sensor_id;
    intersection_id_list[i] = (node_id) intersection_id;
    sensor_index_list[i] = (node_id) (i+1); //the index corresponds to the virtual sensor id, so it starts from zero for subgraph matching
  }

  /* set each list size to number */
  *sensor_id_list_size = number;
  *intersection_id_list_size = number;
  *sensor_index_list_size = number;

  return result;
}

double compute_localization_error_ratio(node_id roadnet_list[], int roadnet_list_size, node_id perm_list[], int perm_list_size)
{ /* compute the localization error ratio
     where roadnet_list contains the intersection ids in the road network where the corresponding index in roadnet_list is sensor id,
           roadnet_list_size is the size of roadnet_list, that is, the number of sensors,
           perm_list contains the intersection ids in the road network where the corresponding index in perm_list is sensor id, and
           perm_list_size is the size of perm_list.
  */

  double error_ratio = 1; //localization error ratio
  int mismatch_number = 0; //number of mismatches between roadnet_list and perm_list
  int i = 0; //loop index

  /* compare two list sizes */
  if(roadnet_list_size != perm_list_size)
  {
    printf("compute_localization_error_ratio(): roadnet_list_size(%d) does not match perm_list_size(%d)\n", roadnet_list_size, perm_list_size);
    return error_ratio;
  }

  /* find the number of mismatches between two lists */
  for(i = 0; i < roadnet_list_size; i++)
  {
    if((roadnet_list[i]+1) != perm_list[i])
    { //Note: roadnet_list[i]'s id is one less than perm_list[i], so we add one to road_list[i]
#ifdef __SUBGRAPH_MATCHING_DEBUG__
      printf("compute_localization_error_ratio(): Error: roadnet_list[i]+1 (%d) is different from perm_list[i]\n", roadnet_list[i]+1, perm_list[i]);
#endif

      mismatch_number++;
    }
  }

  /* compute the localization error ratio */
  error_ratio = (double)mismatch_number/roadnet_list_size;

  return error_ratio;
}

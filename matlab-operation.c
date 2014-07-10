/**
	File: matlab-operation.c
	Description: This file contains the functions of target mobility handling and matlab operations.
	Date: 07/24/2006
	Maker: Jaehoon Jeong, jjeong@cs.umn.edu
	Memo:
*/

#include "stdafx.h"
#include "matlab-operation.h"
#include "util.h"
#include "common.h"

/* variables for Matlab */
static Engine *g_matlab_ep = NULL; //pointer to the matlab engine instance
static mxArray *g_matlab_T = NULL, *g_matlab_result = NULL; //pointer to matlab array structure
static char g_matlab_buffer[MATLAB_BUF_SIZE]; //buffer for matlab command
static FILE *g_matlab_fp = NULL; //pointer to mobility scenario file

/* declarations for static functions */
static double optimize_refresh_time_golden_search(double v, double d, double density);
//compute an optimal refresh time through Matlab engine and script

Engine* matlab_start_for_localization()
{ //perform the initialization for matlab operations; that is, it starts matlab engine.
        Engine *matlab_ep = NULL; //pointer to the matlab engine instance

	matlab_ep = engOpen("\0");
	//matlab_ep = engOpen(APL_MATLAB_START_COMMAND);
	if(matlab_ep == NULL)
	{
		fprintf(stderr, "error : can't start Matlab engine\n");
		//return 1;
		//exit(1);
		return NULL;
	}

	/* set matlab_ep to matlab_ep */
	g_matlab_ep = matlab_ep;

	return g_matlab_ep;
	//return matlab_ep;
}

void matlab_stop_localization(Engine *matlab_ep)
{ //close the connection with the matlab engine.
	engClose(matlab_ep);
}

int matlab_start(char *scenario_file)
{ /* perform the initialization for matlab operations; that is, it starts matlab engine and 
     opens the scenario file. */
        char startcmd[] = APL_MATLAB_START_COMMAND;

        g_matlab_ep = engOpen("\0");
        //g_matlab_ep = engOpen(startcmd);
	if(g_matlab_ep == NULL)
	{
		fprintf(stderr, "error : can't start Matlab engine\n");
		return 1;
	}

	/* open mobility scenario file */
	g_matlab_fp = fopen(scenario_file, "r");
	if(g_matlab_fp == NULL)
	{
		fprintf(stderr, "error : unable to open file \"%s\"\n", scenario_file);
		return 1;
	}

	return 0;
}

void matlab_stop()
{ //close the connection with the matlab engine and close the scenario file
	/* close the connection with the matlab engine */
	engClose(g_matlab_ep);

	/* close mobility scenario file */
	fclose(g_matlab_fp);
}

int matlab_read_initial_mobility_vector(double *x, double *y, double *direction, double *speed)
{ //return the initial mobility vector including the vehicle's initial position, speed, and direction.
	char input_buf[MATLAB_BUF_SIZE]; //input buffer
	size_t input_buf_size = 0; //size of input buffer
	char key_buf[MATLAB_BUF_SIZE], value_buf[MATLAB_BUF_SIZE]; //key buffer, and value buffer
	char *token; //pointer to token
	char *result_code; //return result as pointer
	int i; //loop index

	while((result_code = fgets(input_buf, sizeof(input_buf), g_matlab_fp)) != NULL) //start of while
	{
		input_buf_size = strlen(input_buf);
		token = (char*) strtok(input_buf, "=");
		strcpy(key_buf, token);
		if(key_buf == NULL)
		{
			printf("matlab_read_initial_mobility_vector(): mobility scenario file has null key!\n");
			return 1;
		}
		else if((key_buf[0] == '\n') || (key_buf[0] == ' ') || (key_buf[0] == '\t')) //this line is white space
			continue;
		else if(key_buf[0] == '#') //this is comment line
			continue;

		if(strcmp(key_buf, "IV") != 0) //identify whether this entry is the Initial Vector (IV) for the vehicle's mobility
		{
			printf("matlab_read_initial_mobility_vector(): key_buf(%s) does not indicate the initial mobility vector\n", key_buf);
			return 1;
		}

		token = (char*) strtok(NULL, "\n");
		strcpy(value_buf, token);
		if(value_buf == NULL)
		{
			printf("matlab_read_initial_mobility_vector(): mobility scenario file has null value!\n");
			return 1;
		}
		else
		{
			token = (char* ) strtok(value_buf, ",");
			if(token == NULL)
			{
				printf("matlab_read_initial_mobility_vector(): token is NULL!\n");
				return 1;
			}

			for(i = 0; i < MATLAB_NUMBER_OF_INITIAL_VECTOR_VALUES; i++)
			{
				switch(i)
				{
				case 0:
					*x = atof(token);
					break;

				case 1:
					*y = atof(token);
					break;

				case 2:
					*direction = atof(token);
					*direction = degree2radian(*direction);
					break;

				case 3:
					*speed = atof(token);
					*speed = km_hour2m_sec(*speed);
					break;
				} //end of switch

				token = (char* ) strtok(NULL, ",");
				if((token == NULL) && (i != MATLAB_NUMBER_OF_INITIAL_VECTOR_VALUES-1))
				{
					printf("matlab_read_initial_mobility_vector(): token is NULL!\n");
					return 1;
				} //end of if
			} //end of for
			break; //exit from while loop
		} //end of else
	} //end of while

	return 0;
}

int matlab_read_mobility_vector(double *steering_angle, double *acceleration, double *duration)
{ //return the mobility vector including the vehicle's steering angle, acceleration and duration.
	char input_buf[MATLAB_BUF_SIZE]; //input buffer
	size_t input_buf_size = 0; //size of input buffer
	char key_buf[MATLAB_BUF_SIZE], value_buf[MATLAB_BUF_SIZE]; //key buffer, and value buffer
	char *token; //pointer to token
	char *result_code; //return result as pointer
	int i; //loop index

	while((result_code = fgets(input_buf, sizeof(input_buf), g_matlab_fp)) != NULL) //start of while
	{
		input_buf_size = strlen(input_buf);
		token = (char*) strtok(input_buf, "\n");
		strcpy(value_buf, token);
		if(key_buf == NULL)
		{
			printf("matlab_read_mobility_vector(): mobility scenario file has null key!\n");
			return 1;
		}
		else if((key_buf[0] == '\n') || (key_buf[0] == ' ') || (key_buf[0] == '\t')) //this line is white space
			continue;
		else if(key_buf[0] == '#') //this is comment line
			continue;
		else
		{
			token = (char* ) strtok(value_buf, ",");
			if(token == NULL)
			{
				printf("token is NULL!\n");
				return 1;
			}

			for(i = 0; i < MATLAB_NUMBER_OF_MOBILITY_VECTOR_VALUES; i++)
			{
				
				switch(i)
				{
				case 0:
					*steering_angle = atof(token); //unit is [degree]
					*steering_angle = degree2radian(*steering_angle); //unit is [radian]
					break;

				case 1:
					*acceleration = atof(token); //unit is [m^2/sec]
					break;

				case 2:
					*duration = atof(token); //unit is [sec]
					break;		
				} //end of switch

				token = (char* ) strtok(NULL, ",");
				if((token == NULL) && (i != MATLAB_NUMBER_OF_MOBILITY_VECTOR_VALUES-1))
				{
					printf("token is NULL!\n");
					return 1;
				}

			} //end of for
			break; //exit from while loop
		} //end of else
	} //end of while
	
	return 0;
}

double matlab_get_optimal_refresh_time(parameter_t *param)
{ //return the optimal refresh time using one-dimensional optimization,i.e., Golden section search.
	double optimal_refresh_time = 0;
        double v; //vehicle speed
	double d; //average trajectory distance used for optimization
	double density; //density of sensor nodes for the surveillance field

	v = km_hour2m_sec(param->vehicle_speed);
	d = AVERAGE_TRAJECTORY_DISTANCE;
	density = param->sensor_number/(param->network_height*param->network_width);

	optimal_refresh_time = optimize_refresh_time_golden_search(v, d, density);
	return optimal_refresh_time;
}

static double optimize_refresh_time_golden_search(double v, double d, double density)
{ //compute an optimal refresh time through Matlab engine and script
	double T_opt = 0; //optimal refresh time
	mxArray *mx_T_opt = NULL, *mx_v = NULL, *mx_d = NULL, *mx_density = NULL;//, *mx_result = NULL;
	void *data = NULL;
	//char output[MATLAB_BUF_SIZE]; //buffer for output display of Matlab operation

	/* set up the buffer of output to get the output message from Matlab */
	//engOutputBuffer(g_matlab_ep, output, MATLAB_BUF_SIZE);

	/* place the variables T_opt, v, d, and density into the Matlab workspace */
	//mx_T_opt = mxCreateDoubleScalar(T_opt);
	mx_v = mxCreateDoubleScalar(v);
	mx_d = mxCreateDoubleScalar(d);
	mx_density = mxCreateDoubleScalar(density);
	
	//engPutVariable(g_matlab_ep, "T_opt", mx_T_opt);
	engPutVariable(g_matlab_ep, "v", mx_v);
	engPutVariable(g_matlab_ep, "d", mx_d);
	engPutVariable(g_matlab_ep, "density", mx_density);

	/* perform optimization function */
	engEvalString(g_matlab_ep, "T_opt = optimize_refresh_time_golden_search(v, d, density)");

	//printf("%s\n",output+2); //output+2 removes prompt ">>" from the output

	mx_T_opt = engGetVariable(g_matlab_ep, "T_opt");
	if(mx_T_opt == NULL)
	{
		printf("optimize_refresh_time_golden_search(): engGetVariable() returns NULL\n");
		return -1;
	}

	data = mxGetData(mx_T_opt);
	memcpy(&T_opt, data, sizeof(T_opt));

	/* free memory of Matlab variables */
	mxDestroyArray(mx_T_opt);
	mxDestroyArray(mx_v);
	mxDestroyArray(mx_d);
	mxDestroyArray(mx_density);

	return T_opt;
}

/*
int matlab_count_neighbor_sensors_based_on_circle(double refresh_time, struct struct_vehicle *vehicle, struct struct_sensor *sensor_list, int total_sensor_number, int *S_ID, double *S_X, double *S_Y, int *p_live_sensor_number, struct_matlab_tracking_result *p_matlab_result)
{ //estimate the number of neighbor sensors within the current dynamic contour
	int count = 0; //number of working sensors within the current dynamic contour
	double X0, Y0; //X0 is the x-coordinate of vehicle and Y0 the y-coordinate.	
	//int i; //loop index
    mxArray *mx_S_X = NULL, *mx_S_Y = NULL;
	double radius; //radius of tracking circle 
	//double energy = 0; //energy consumed by the current circle or contour
	//int neighbor_number_in_comm_range = 0; //number of neighbor sensors within the communication range
	double T_work = 0; //working time of sensor within a contour

	// store the vehicle's current position in (X0, Y0)
	X0 = vehicle->current_pos.x;
	Y0 = vehicle->current_pos.y;

	// call Mablab function to get the number and list of working sensors
	//[n, in] = find_sensors_inside_contour(X0, Y0, S_X, S_Y, refresh_time, max_turning_angle, speed, direction, wheelbase, flag_plot);

	//reduce the energy usage of each working sensor

	// find the number of sensors within the current tracking circle
	radius = vehicle->speed * refresh_time;
	count = count_neighbor_sensors(vehicle->current_pos.x, vehicle->current_pos.y, sensor_list, total_sensor_number, radius);
	p_matlab_result->number_of_working_sensors = count;

	// compute the consumed energy
	T_work = refresh_time;
	p_matlab_result->number_full_tx_power = count_neighbor_sensors(X0, Y0, sensor_list, total_sensor_number, Communication_range);
	p_matlab_result->energy_full_tx_power = (P_comp*T_comp + P_warm*T_warm + P_work*T_sense + P_work*T_work)*count + P_comm_tx*T_comm_tx + P_comm_rx*T_comm_rx*(p_matlab_result->number_full_tx_power);

	return count;
}
*/

/*
struct struct_matlab_tracking_result
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
};
*/


int matlab_count_neighbor_sensors_based_on_circle(double refresh_time, struct_vehicle_t *vehicle, 
	struct_sensor_t *sensor_list, int total_sensor_number, int *S_ID, double *S_X, double *S_Y, 
	int *p_live_sensor_number, struct_matlab_tracking_result *p_matlab_result)
{ //estimate the number of neighbor sensors within the current tracking circle
	int n = 0; //number of working sensors within the current circle
	double X0, Y0; //X0 is the x-coordinate of vehicle and Y0 the y-coordinate.	
	double T_work = 0; //working time of sensor within a contour
	double range; //communication range

	/* store the vehicle's current position in (X0, Y0) */
	X0 = vehicle->current_pos.x;
	Y0 = vehicle->current_pos.y;

	/****************************************************************/
	/** Future work: reduce the energy usage of each working sensor */
	/****************************************************************/

	/**************************************************/
	/* Compute Energy Cost */
	/**************************************************/

	T_work = refresh_time;

	range = refresh_time * vehicle->speed;
	n = p_matlab_result->number_of_working_sensors = count_neighbor_sensors(X0, Y0, sensor_list, total_sensor_number, range);
	//number of working sensors
	
	/** Full Tx power                    */
	range = Communication_range;
	p_matlab_result->number_full_tx_power = count_neighbor_sensors(X0, Y0, sensor_list, total_sensor_number, range);
	//count the number of working sensors within communication cone by only tx power control
	p_matlab_result->energy_full_tx_power = (P_comp*T_comp + P_warm*T_warm + P_work*T_sense + P_work*T_work)*(int)n + P_comm_tx*T_comm_tx + P_comm_rx*T_comm_rx*(p_matlab_result->number_full_tx_power); 

	/** Tx power control                 */
	range = refresh_time * vehicle->speed;
	p_matlab_result->number_tx_power_control = count_neighbor_sensors(X0, Y0, sensor_list, total_sensor_number, range);
	//count the number of working sensors within communication cone by only tx power control
	p_matlab_result->energy_tx_power_control = (P_comp*T_comp + P_warm*T_warm + P_work*T_sense + P_work*T_work)*(int)n + P_comm_tx*T_comm_tx + P_comm_rx*T_comm_rx*(p_matlab_result->number_tx_power_control); 

	/** Directional antenna              */
	p_matlab_result->number_directional_antenna = p_matlab_result->number_full_tx_power;
	//count the number of working sensors within communication cone by only tx power control
	p_matlab_result->energy_directional_antenna = p_matlab_result->energy_full_tx_power;

	/** Combination of Directional antenna and Power control */
	p_matlab_result->number_tx_and_directional = p_matlab_result->number_tx_power_control;
	//count the number of working sensors within communication cone by only tx power control
	p_matlab_result->energy_tx_and_directional = p_matlab_result->energy_tx_power_control;

	return n;
}

int matlab_count_neighbor_sensors_based_on_contour(double refresh_time, struct_vehicle_t *vehicle, 
	struct_sensor_t *sensor_list, int total_sensor_number, int *S_ID, double *S_X, double *S_Y, 
	int *p_live_sensor_number, struct_matlab_tracking_result *p_matlab_result)
{ //estimate the number of neighbor sensors within the current tracking circle
	double n = 0; //number of working sensors within the current circle
	double number_directional_antenna; //number of RF receiving sensors by directional antenna for full Tx power
	double number_tx_and_directional; //number of RF receiving sensors by both tx power control and directional antenna for full Tx power
	//int n = 0; //number of working sensors within the current circle
	double *inside = NULL; //boolean vector indicating which S_ID should work for the current contour
	double *inside_directional_antenna = NULL; //boolean vector indicating which S_ID is located within the communication cone by directional antenna
	double *inside_tx_and_directional = NULL; //boolean vector indicating which S_ID is located within the minimal communication cone by both directional antenna and Tx power control
	//int *inside = NULL; //boolean vector indicating which S_ID should work for the current contour
	double X0, Y0; //X0 is the x-coordinate of vehicle and Y0 the y-coordinate.	
	//int i; //loop index
	mxArray *mx_X0 = NULL, *mx_Y0 = NULL, *mx_refresh_time = NULL, *mx_max_turning_angle = NULL, 
		*mx_speed = NULL, *mx_direction = NULL, *mx_wheelbase = NULL, *mx_flag_plot = NULL;
        mxArray *mx_S_X = NULL, *mx_S_Y = NULL;
	mxArray *mx_n; //Matlab variable for n that is the number of working sensors within the current contour
	//mxArray *mx_inside; //Matlab variable for in that is the boolean vector indicating which S_ID should work for the current contour
	mxArray *mx_number_directional_antenna; //Matlab variable for number of RF receiving sensors by directional antenna for full Tx power
	//mxArray *mx_inside_directional_antenna; //Matlab variable for in that is the boolean vector indicating which S_ID is located within the communication cone by directional antenna
	mxArray *mx_number_tx_and_directional; //Matlab variable for number of RF receiving sensors by directional antenna and Tx power control
	//mxArray *mx_inside_tx_and_directional; //Matlab variable for in that is the boolean vector indicating which S_ID is located within the minimal communication cone by both directional antenna and Tx power control

	int flag_plot = 0;
	//int flag_plot = 1;
	//char output[MATLAB_BUF_SIZE*10]; //buffer for output display of Matlab operation
	//char *data = NULL;
	//double *data = NULL;
	void *data = NULL;
	double angle; //vehicle's maximum turning angle in [radian]
	//int count; //variable to count the number of working sensors
	//int neighbor_number_in_comm_range = 0; //number of neighbor sensors within the communication range
	double T_work = 0; //working time of sensor within a contour
	double range; //communication range

	/* store the vehicle's current position in (X0, Y0) */
	X0 = vehicle->current_pos.x;
	Y0 = vehicle->current_pos.y;

	/* set up the buffer of output to get the output message from Matlab */
	//engOutputBuffer(matlab_ep, output, MATLAB_BUF_SIZE*10);

	/* set up Malab variables */
	mx_X0 = mxCreateDoubleScalar(vehicle->current_pos.x);
	mx_Y0 = mxCreateDoubleScalar(vehicle->current_pos.y);
	mx_refresh_time = mxCreateDoubleScalar(refresh_time);
	angle = degree2radian(vehicle->max_turning_angle); //convert the angle in degree into the angle in radian
	mx_max_turning_angle = mxCreateDoubleScalar(angle);
	mx_speed = mxCreateDoubleScalar(vehicle->speed);
	mx_direction = mxCreateDoubleScalar(vehicle->direction);
	mx_wheelbase = mxCreateDoubleScalar(vehicle->wheelbase);
	mx_flag_plot = mxCreateDoubleScalar(flag_plot);
		
	mx_S_X = mxCreateDoubleMatrix(1, *p_live_sensor_number, mxREAL);
	mx_S_Y = mxCreateDoubleMatrix(1, *p_live_sensor_number, mxREAL);

	memcpy((void*)mxGetPr(mx_S_X), (void*)S_X, sizeof(double)*(*p_live_sensor_number));
	memcpy((void*)mxGetPr(mx_S_Y), (void*)S_Y, sizeof(double)*(*p_live_sensor_number));

	//mx_n = mxCreateDoubleScalar(n);
	//mx_inside = mxCreateDoubleMatrix(1, *p_live_sensor_number, mxREAL);
	//memset((void*)mxGetPr(mx_in), 0, sizeof(double)*(*p_live_sensor_number));

	/* map mx variables to Mablab variables */
	engPutVariable(g_matlab_ep, "X0", mx_X0);
	engPutVariable(g_matlab_ep, "Y0", mx_Y0);
	engPutVariable(g_matlab_ep, "refresh_time", mx_refresh_time);
	engPutVariable(g_matlab_ep, "max_turning_angle", mx_max_turning_angle);
	engPutVariable(g_matlab_ep, "speed", mx_speed);
	engPutVariable(g_matlab_ep, "direction", mx_direction);
	engPutVariable(g_matlab_ep, "wheelbase", mx_wheelbase);
	engPutVariable(g_matlab_ep, "flag_plot", mx_flag_plot);
	engPutVariable(g_matlab_ep, "S_X", mx_S_X);
	engPutVariable(g_matlab_ep, "S_Y", mx_S_Y);
	//engPutVariable(g_matlab_ep, "n", mx_n);
	//engPutVariable(g_matlab_ep, "inside", mx_in);
	
	/* call Mablab function to get the number and list of working sensors */
	engEvalString(g_matlab_ep, "[n, inside, number_directional_antenna, inside_directional_antenna, number_tx_and_directional, inside_tx_and_directional] = find_sensors_inside_contour_and_cones(X0, Y0, S_X, S_Y, refresh_time, max_turning_angle, speed, direction, wheelbase, flag_plot)");
	
	//printf("%s\n",output+2); //output+2 removes prompt ">>" from the output

	/* get the result of n */
	mx_n = engGetVariable(g_matlab_ep, "n");
	if(mx_n == NULL)
	{
		printf("matlab_count_neighbor_sensors_based_on_contour(): engGetVariable() returns NULL for Matlab variable n\n");
		return -1;
	}

	data = mxGetData(mx_n);
	memcpy(&n, data, sizeof(n));
	//memcpy(&n, (int*) data, sizeof(n));

	/* get the result of inside */
/*
	mx_inside = engGetVariable(g_matlab_ep, "inside");
	if(mx_inside == NULL)
	{
		printf("matlab_count_neighbor_sensors_based_on_contour(): engGetVariable() returns NULL for Matlab variable in\n");
		return -1;
	}

	data = mxGetPr(mx_inside);
	//data = mxGetData(mx_inside);
	inside = (double*)malloc(sizeof(double)*(*p_live_sensor_number));
	//inside = (int*)malloc(sizeof(int)*(*p_live_sensor_number));
	if(inside == NULL)
	{
		printf("matlab_count_neighbor_sensors_based_on_contour(): malloc cannot allocate memory to in\n");
		return -1;
	}
	memcpy(inside, data, sizeof(double)*(*p_live_sensor_number));
	//memcpy(inside, (int*) data, sizeof(int)*(*p_live_sensor_number));
*/
	/**************************************************/
	/* get the result of number_directional_antenna */
	mx_number_directional_antenna = engGetVariable(g_matlab_ep, "number_directional_antenna");
	if(mx_number_directional_antenna == NULL)
	{
		printf("matlab_count_neighbor_sensors_based_on_contour(): engGetVariable() returns NULL for Matlab variable n\n");
		return -1;
	}

	data = mxGetData(mx_number_directional_antenna);
	memcpy(&number_directional_antenna, data, sizeof(number_directional_antenna));

	/* get the result of inside__directional_antenna */
/*
	mx_inside_directional_antenna = engGetVariable(g_matlab_ep, "inside_directional_antenna");
	if(mx_inside_directional_antenna == NULL)
	{
		printf("matlab_count_neighbor_sensors_based_on_contour(): engGetVariable() returns NULL for Matlab variable in\n");
		return -1;
	}

	data = mxGetPr(mx_inside_directional_antenna);
	inside_directional_antenna = (double*)malloc(sizeof(double)*(*p_live_sensor_number));
	if(inside_directional_antenna == NULL)
	{
		printf("matlab_count_neighbor_sensors_based_on_contour(): malloc cannot allocate memory to in\n");
		return -1;
	}
	memcpy(inside_directional_antenna, data, sizeof(double)*(*p_live_sensor_number));
*/
	/**************************************************/
	/* get the result of number_tx_and_directional */
	mx_number_tx_and_directional = engGetVariable(g_matlab_ep, "number_tx_and_directional");
	if(mx_number_tx_and_directional == NULL)
	{
		printf("matlab_count_neighbor_sensors_based_on_contour(): engGetVariable() returns NULL for Matlab variable n\n");
		return -1;
	}

	data = mxGetData(mx_number_tx_and_directional);
	memcpy(&number_tx_and_directional, data, sizeof(number_tx_and_directional));
	
	/* get the result of inside_tx_and_directional */
/*
	mx_inside_tx_and_directional = engGetVariable(g_matlab_ep, "inside_tx_and_directional");
	if(mx_inside_tx_and_directional == NULL)
	{
		printf("matlab_count_neighbor_sensors_based_on_contour(): engGetVariable() returns NULL for Matlab variable in\n");
		return -1;
	}

	data = mxGetPr(mx_inside_tx_and_directional);
	inside_tx_and_directional = (double*)malloc(sizeof(double)*(*p_live_sensor_number));
	if(inside_tx_and_directional == NULL)
	{
		printf("matlab_count_neighbor_sensors_based_on_contour(): malloc cannot allocate memory to in\n");
		return -1;
	}

	//memcpy(inside_tx_and_directional, data, sizeof(double)*(*p_live_sensor_number));
*/
	/*@ run time error happens! */


	/****************************************************************/
	/** Future work: reduce the energy usage of each working sensor */
	/****************************************************************/
/*
	count = 0; //count the number of working sensors within contour
	for(i = 0; i < *p_live_sensor_number; i++)
	{
		//if(inside[i] == 1)
		if((int)inside[i] == 1)
		{	
			printf("sensor %d is working sensor in the current contour\n", S_ID[i]);
			count++;
			// compute the sensing  considering warming-up time
			// reduce the energy of sensor S_ID[i] by consumed energy
		}
	}
*/
	/**************************************************/
	/* Compute Energy Cost */
	/**************************************************/

	T_work = refresh_time;
	p_matlab_result->number_of_working_sensors = (int)n; //number of working sensors

	/** Full Tx power                    */
	range = Communication_range;
	p_matlab_result->number_full_tx_power = count_neighbor_sensors(X0, Y0, sensor_list, total_sensor_number, range);
	//count the number of working sensors within communication cone by only tx power control
	p_matlab_result->energy_full_tx_power = (P_comp*T_comp + P_warm*T_warm + P_work*T_sense + P_work*T_work)*(int)n + P_comm_tx*T_comm_tx + P_comm_rx*T_comm_rx*(p_matlab_result->number_full_tx_power); 

	/** Tx power control                 */
	range = refresh_time * vehicle->speed;
	p_matlab_result->number_tx_power_control = count_neighbor_sensors(X0, Y0, sensor_list, total_sensor_number, range);
	//count the number of working sensors within communication cone by only tx power control
	p_matlab_result->energy_tx_power_control = (P_comp*T_comp + P_warm*T_warm + P_work*T_sense + P_work*T_work)*(int)n + P_comm_tx*T_comm_tx + P_comm_rx*T_comm_rx*(p_matlab_result->number_tx_power_control); 

	/** Directional antenna              */
	p_matlab_result->number_directional_antenna = (int) number_directional_antenna;
	//count the number of working sensors within communication cone by only tx power control
	p_matlab_result->energy_directional_antenna = (P_comp*T_comp + P_warm*T_warm + P_work*T_sense + P_work*T_work)*(int)n + P_comm_tx*T_comm_tx + P_comm_rx*T_comm_rx*(p_matlab_result->number_directional_antenna); 

	/** Combination of Directional antenna and Power control */
	p_matlab_result->number_tx_and_directional = (int) number_tx_and_directional;
	//count the number of working sensors within communication cone by only tx power control
	p_matlab_result->energy_tx_and_directional = (P_comp*T_comp + P_warm*T_warm + P_work*T_sense + P_work*T_work)*(int)n + P_comm_tx*T_comm_tx + P_comm_rx*T_comm_rx*(p_matlab_result->number_tx_and_directional); 

	/**************************************************/

	/* free memory of Matlab variables */
	mxDestroyArray(mx_X0);
	mxDestroyArray(mx_Y0);
	mxDestroyArray(mx_refresh_time);
	mxDestroyArray(mx_max_turning_angle);
	mxDestroyArray(mx_speed);
	mxDestroyArray(mx_direction);
	mxDestroyArray(mx_wheelbase);
	mxDestroyArray(mx_flag_plot);
	mxDestroyArray(mx_S_X);
	mxDestroyArray(mx_S_Y);
	mxDestroyArray(mx_n);
//	mxDestroyArray(mx_inside);
	mxDestroyArray(mx_number_directional_antenna);
//	mxDestroyArray(mx_inside_directional_antenna);
	mxDestroyArray(mx_number_tx_and_directional);
//	mxDestroyArray(mx_inside_tx_and_directional);


//	free(inside); //free the memory allocated to inside
//	free(inside_directional_antenna); //free the memory allocated to inside_directional_antenna
//	free(inside_tx_and_directional); //free the memory allocated to inside_tx_and_directional

	return (int)n;
}

int matlab_open_file(char *directory, char *filename)
{ //open filename to prove that the matlab recognizes the input filename

	mxArray *mx_filename = NULL, *mx_directory = NULL, *mx_result = NULL;
	//int result = -1;
	double result = -1;
	//double *data = NULL;
	void *data = NULL;
	//size_t size;
	int length1, length2;
	double filename_buf[BUF_SIZE];
	//char directory[BUF_SIZE] = "c:\\temp";
	double directory_buf[BUF_SIZE];
	int i;

	length1 = (int)strlen(filename);
	for(i = 0; i < length1; i++)
		filename_buf[i] = filename[i];

	length2 = (int)strlen(directory);
	for(i = 0; i < length2; i++)
		directory_buf[i] = directory[i];

	mx_filename = mxCreateDoubleMatrix(1, length1, mxREAL);
	memcpy((void*)mxGetPr(mx_filename), (void*)filename_buf, sizeof(double)*length1);

	mx_directory = mxCreateDoubleMatrix(1, length2, mxREAL);
	memcpy((void*)mxGetPr(mx_directory), (void*)directory_buf, sizeof(double)*length2);

	//mx_length = mxCreateDoubleScalar(length);

	engPutVariable(g_matlab_ep, "filename", mx_filename);
	engPutVariable(g_matlab_ep, "directory", mx_directory);
	//engPutVariable(g_matlab_ep, "length", mx_length);

	//result = engEvalString(g_matlab_ep, "[result] = Open_File(filename)");
	engEvalString(g_matlab_ep, "result = Open_File(directory, filename)");

	/* get the result */
	mx_result = engGetVariable(g_matlab_ep, "result");
	if(mx_result == NULL)
	{
		printf("matlab_open_file(): engGetVariable() returns NULL for Matlab variable n\n");
		return -1;
	}

	data = mxGetData(mx_result);
	memcpy(&result, data, sizeof(result));

	mxDestroyArray(mx_directory);
	mxDestroyArray(mx_filename);
	//mxDestroyArray(mx_length);
	mxDestroyArray(mx_result);

	return (int)result;
}

struct_matlab_localization_result* matlab_perform_localization(Engine *matlab_ep, char *current_directory, int nodenum, int measurement_number, double speed, double speed_deviation, char *adjacency_matrix_filename, char *virtual_topology_filename_prefix, int data_aggregation_type, int data_prefilter_type)
//	enum aggregation_type data_aggregation_type, enum prefilter_type data_prefilter_type)
//  : these types should be int since they are memory-copied into matlab variable of type double
{ //perform the localization of sensor nodes along with both prefiltering and graph matching.
  //return value: error-ratio
	static struct_matlab_localization_result localization_result; //localization result
	int result; //result of matlab main function Perform_Localization()
	double error_ratio = 0; //error ratio of localization
	double E_ratio = 0; //ratio indicating the relative difference between Er and Ev
	double M_ratio = 0; //ratio indicating the relative difference between Mr and Mv
	double current_directory_buf[BUF_SIZE], adjacency_matrix_filename_buf[BUF_SIZE], 
		virtual_topology_filename_prefix_buf[BUF_SIZE];
	int length; //string length
	mxArray *mx_current_directory = NULL, *mx_nodenum = NULL, *mx_measurement_number = NULL,
		*mx_speed = NULL, *mx_speed_deviation = NULL, *mx_adjacency_matrix_filename = NULL,	*mx_virtual_topology_filename_prefix = NULL, 
		*mx_data_aggregation_type = NULL, *mx_data_prefilter_type = NULL, *mx_result = NULL, 
		*mx_error_ratio = NULL, *mx_E_ratio = NULL, *mx_M_ratio = NULL;
	int i; //loop index
	int ret_val;
	//double *data = NULL;
	void *data = NULL;

	/* initialize result for localization */
	localization_result.result = -1;
	localization_result.error_ratio = -1;
	localization_result.E_ratio = -1;
	localization_result.M_ratio = -1;

	/* set up Malab variables */
	length = (int)strlen(current_directory);

#ifdef __DEBUG_LEVEL_1__
	printf("current_directory_buf:\n");
#endif
	for(i = 0; i < length; i++)
	{
		current_directory_buf[i] = current_directory[i];
#ifdef __DEBUG_LEVEL_1__
		printf("%f ", current_directory_buf[i]);
#endif
	}
#ifdef __DEBUG_LEVEL_1__
	printf("\n\n");
#endif

	mx_current_directory = mxCreateDoubleMatrix(1, length, mxREAL);
	memcpy((void*)mxGetPr(mx_current_directory), (void*)current_directory_buf, sizeof(double)*length);
	engPutVariable(matlab_ep, "current_directory", mx_current_directory);

	mx_nodenum = mxCreateDoubleScalar(nodenum);
	engPutVariable(matlab_ep, "nodenum", mx_nodenum);

	mx_measurement_number = mxCreateDoubleScalar(measurement_number);
	engPutVariable(matlab_ep, "measurement_number", mx_measurement_number);

	mx_speed = mxCreateDoubleScalar(speed);
	engPutVariable(matlab_ep, "speed", mx_speed);

	mx_speed_deviation = mxCreateDoubleScalar(speed_deviation);
	engPutVariable(matlab_ep, "speed_deviation", mx_speed_deviation);

	length = (int)strlen(adjacency_matrix_filename);
#ifdef __DEBUG_LEVEL_1__
	printf("adjacency_matrix_filename_buf:\n");
#endif
	for(i = 0; i < length; i++)
	{
		adjacency_matrix_filename_buf[i] = adjacency_matrix_filename[i];
#ifdef __DEBUG_LEVEL_1__
		printf("%f ", adjacency_matrix_filename_buf[i]);
#endif
	}
#ifdef __DEBUG_LEVEL_1__
	printf("\n\n");
#endif

	mx_adjacency_matrix_filename = mxCreateDoubleMatrix(1, length, mxREAL);
	memcpy((void*)mxGetPr(mx_adjacency_matrix_filename), (void*)adjacency_matrix_filename_buf, sizeof(double)*length);
	engPutVariable(matlab_ep, "adjacency_matrix_filename", mx_adjacency_matrix_filename);

	length = (int)strlen(virtual_topology_filename_prefix);
#ifdef __DEBUG_LEVEL_1__
	printf("virtual_topology_filename_prefix_buf:\n");
#endif
	for(i = 0; i < length; i++)
	{
		virtual_topology_filename_prefix_buf[i] = virtual_topology_filename_prefix[i];
#ifdef __DEBUG_LEVEL_1__
		printf("%f ", virtual_topology_filename_prefix_buf[i]);
#endif
	}
#ifdef __DEBUG_LEVEL_1__
	printf("\n\n");
#endif
	mx_virtual_topology_filename_prefix = mxCreateDoubleMatrix(1, length, mxREAL);
	memcpy((void*)mxGetPr(mx_virtual_topology_filename_prefix), (void*)virtual_topology_filename_prefix_buf, sizeof(double)*length);
	engPutVariable(matlab_ep, "virtual_topology_filename_prefix", mx_virtual_topology_filename_prefix);

	mx_data_aggregation_type = mxCreateDoubleScalar(data_aggregation_type);
	engPutVariable(matlab_ep, "data_aggregation_type", mx_data_aggregation_type);

	mx_data_prefilter_type = mxCreateDoubleScalar(data_prefilter_type);
	engPutVariable(matlab_ep, "data_prefilter_type", mx_data_prefilter_type);

	/* perform matlab script using matlab engine */
	ret_val = engEvalString(matlab_ep, "[result, error_ratio, E_ratio, M_ratio] = Perform_APL_Localization(current_directory, nodenum, measurement_number, speed, speed_deviation, adjacency_matrix_filename, virtual_topology_filename_prefix, data_aggregation_type, data_prefilter_type)");
	
	mx_result = engGetVariable(matlab_ep, "result");
	if(mx_result == NULL)
	{
		printf("matlab_perform_localization(): engGetVariable() returns NULL for Matlab variable n\n");
		return &localization_result;
	}
	data = mxGetData(mx_result);
	memcpy(&result, data, sizeof(result));

	if(result == -1)
	{
		printf("matlab_perform_localization(): Perform_Localization returns error due to operation problem, such as newprim7()\n");
		return &localization_result;
	}

	mx_error_ratio = engGetVariable(matlab_ep, "error_ratio");
	if(mx_error_ratio == NULL)
	{
		printf("matlab_perform_localization(): engGetVariable() returns NULL for Matlab variable n\n");
		return &localization_result;
	}
	data = mxGetData(mx_error_ratio);
	memcpy(&error_ratio, data, sizeof(error_ratio));

	mx_E_ratio = engGetVariable(matlab_ep, "E_ratio");
	if(mx_E_ratio == NULL)
	{
		printf("matlab_perform_localization(): engGetVariable() returns NULL for Matlab variable n\n");
		return &localization_result;
	}
	data = mxGetData(mx_E_ratio);
	memcpy(&E_ratio, data, sizeof(E_ratio));

	mx_M_ratio = engGetVariable(matlab_ep, "M_ratio");
	if(mx_M_ratio == NULL)
	{
		printf("matlab_perform_localization(): engGetVariable() returns NULL for Matlab variable n\n");
		return &localization_result;
	}
	data = mxGetData(mx_M_ratio);
	memcpy(&M_ratio, data, sizeof(M_ratio));

	/* release the memory for mx variables */
	mxDestroyArray(mx_current_directory);
	mxDestroyArray(mx_nodenum);
	mxDestroyArray(mx_measurement_number);
	mxDestroyArray(mx_speed);
	mxDestroyArray(mx_speed_deviation);
	mxDestroyArray(mx_adjacency_matrix_filename);
	mxDestroyArray(mx_virtual_topology_filename_prefix);
	mxDestroyArray(mx_data_aggregation_type);
	mxDestroyArray(mx_data_prefilter_type);
	mxDestroyArray(mx_result);
	mxDestroyArray(mx_error_ratio);
	mxDestroyArray(mx_E_ratio);
	mxDestroyArray(mx_M_ratio);

	/* return result for localization */
	localization_result.result = result;
	localization_result.error_ratio = (float)error_ratio;
	localization_result.E_ratio = (float)E_ratio;
	localization_result.M_ratio = (float)M_ratio;

	return &localization_result;
}

struct_matlab_prefiltering_result* matlab_perform_prefiltering(Engine *matlab_ep, char *current_directory, int nodenum, int measurement_number, double speed, double speed_deviation, char *adjacency_matrix_filename, char *intersection_vector_filename, char *virtual_topology_filename_prefix, int data_aggregation_type, int data_prefilter_type)
{ //perform the prefiltering according to the given prefiltering type
        static struct_matlab_prefiltering_result prefiltering_result; //result of prefiltering
	int result; //result of matlab main function Perform_Prefiltering()
	//double current_directory_buf[BUF_SIZE], adjacency_matrix_filename_buf[BUF_SIZE], virtual_topology_filename_prefix_buf[BUF_SIZE], road_network_graph_filename_buf[BUF_SIZE], sensor_network_graph_filename_buf[BUF_SIZE], permutation_vector_filename_buf[BUF_SIZE];
	char road_network_graph_filename[BUF_SIZE], intersection_sensor_network_graph_filename[BUF_SIZE], sensor_network_graph_filename[BUF_SIZE], permutation_vector_filename[BUF_SIZE], intersection_pair_matrix_filename[BUF_SIZE], intersection_sensor_id_vector_filename[BUF_SIZE], nonintersection_sensor_id_vector_filename[BUF_SIZE], sensor_degree_vector_filename[BUF_SIZE];
	int length; //string length
	mxArray *mx_current_directory = NULL, *mx_nodenum = NULL, *mx_measurement_number = NULL, *mx_speed = NULL, *mx_speed_deviation = NULL, *mx_adjacency_matrix_filename = NULL, *mx_intersection_vector_filename = NULL, *mx_virtual_topology_filename_prefix = NULL, *mx_data_aggregation_type = NULL, *mx_data_prefilter_type = NULL, *mx_result = NULL, *mx_road_network_graph_filename = NULL, *mx_intersection_sensor_network_graph_filename = NULL, *mx_sensor_network_graph_filename = NULL, *mx_permutation_vector_filename = NULL, *mx_intersection_pair_matrix_filename = NULL, *mx_intersection_sensor_id_vector_filename = NULL, *mx_nonintersection_sensor_id_vector_filename = NULL, *mx_sensor_degree_vector_filename = NULL;
	int i; //loop index
	int ret_val;
	void *data = NULL;
	char *ptr = NULL;

	/* initialize result for prefiltering */
	prefiltering_result.result = -1;
	memset(prefiltering_result.road_sensor_network_graph_filename, 0, sizeof(prefiltering_result.road_sensor_network_graph_filename));
	memset(prefiltering_result.intersection_vector_filename, 0, sizeof(prefiltering_result.intersection_vector_filename));
	memset(prefiltering_result.road_network_graph_filename, 0, sizeof(prefiltering_result.road_network_graph_filename));
	memset(prefiltering_result.intersection_sensor_network_graph_filename, 0, sizeof(prefiltering_result.intersection_sensor_network_graph_filename));
	memset(prefiltering_result.sensor_network_graph_filename, 0, sizeof(prefiltering_result.sensor_network_graph_filename));
	memset(prefiltering_result.permutation_vector_filename, 0, sizeof(prefiltering_result.permutation_vector_filename));
	memset(prefiltering_result.intersection_pair_matrix_filename, 0, sizeof(prefiltering_result.intersection_pair_matrix_filename));
	memset(prefiltering_result.intersection_sensor_id_vector_filename, 0, sizeof(prefiltering_result.intersection_sensor_id_vector_filename));
	memset(prefiltering_result.nonintersection_sensor_id_vector_filename, 0, sizeof(prefiltering_result.nonintersection_sensor_id_vector_filename));
	memset(prefiltering_result.sensor_degree_vector_filename, 0, sizeof(prefiltering_result.sensor_degree_vector_filename));

	/* set up Malab variables */
/* 	length = (int)strlen(current_directory); */

/* #ifdef __DEBUG_LEVEL_1__ */
/* 	printf("current_directory_buf:\n"); */
/* #endif */
/* 	for(i = 0; i < length; i++) */
/* 	{ */
/* 		current_directory_buf[i] = current_directory[i]; */
/* #ifdef __DEBUG_LEVEL_1__ */
/* 		printf("%f ", current_directory_buf[i]); */
/* #endif */
/* 	} */
/* #ifdef __DEBUG_LEVEL_1__ */
/* 	printf("\n\n"); */
/* #endif */

/* 	mx_current_directory = mxCreateDoubleMatrix(1, length, mxREAL); */
/* 	memcpy((void*)mxGetPr(mx_current_directory), (void*)current_directory_buf, sizeof(double)*length); */

	mx_current_directory = mxCreateString(current_directory); //create 1-by-N string mxArray initialized to specified string
	engPutVariable(matlab_ep, "current_directory", mx_current_directory);

	mx_nodenum = mxCreateDoubleScalar(nodenum);
	engPutVariable(matlab_ep, "nodenum", mx_nodenum);

	mx_measurement_number = mxCreateDoubleScalar(measurement_number);
	engPutVariable(matlab_ep, "measurement_number", mx_measurement_number);

	mx_speed = mxCreateDoubleScalar(speed);
	engPutVariable(matlab_ep, "speed", mx_speed);

	mx_speed_deviation = mxCreateDoubleScalar(speed_deviation);
	engPutVariable(matlab_ep, "speed_deviation", mx_speed_deviation);

	mx_adjacency_matrix_filename = mxCreateString(adjacency_matrix_filename); //create 1-by-N string mxArray initialized to specified string
	engPutVariable(matlab_ep, "adjacency_matrix_filename", mx_adjacency_matrix_filename);

	mx_intersection_vector_filename = mxCreateString(intersection_vector_filename); //create 1-by-N string mxArray initialized to specified string
	engPutVariable(matlab_ep, "intersection_vector_filename", mx_intersection_vector_filename);

	mx_virtual_topology_filename_prefix = mxCreateString(virtual_topology_filename_prefix); //create 1-by-N string mxArray initialized to specified string
	engPutVariable(matlab_ep, "virtual_topology_filename_prefix", mx_virtual_topology_filename_prefix);

	mx_data_aggregation_type = mxCreateDoubleScalar(data_aggregation_type);
	engPutVariable(matlab_ep, "data_aggregation_type", mx_data_aggregation_type);

	mx_data_prefilter_type = mxCreateDoubleScalar(data_prefilter_type);
	engPutVariable(matlab_ep, "data_prefilter_type", mx_data_prefilter_type);

	/* perform matlab script using matlab engine */
	ret_val = engEvalString(matlab_ep, "[result, road_network_graph_filename, intersection_sensor_network_graph_filename, sensor_network_graph_filename, permutation_vector_filename, intersection_pair_matrix_filename, intersection_sensor_id_vector_filename, nonintersection_sensor_id_vector_filename, sensor_degree_vector_filename] = Perform_Prefiltering(current_directory, nodenum, measurement_number, speed, speed_deviation, adjacency_matrix_filename, intersection_vector_filename, virtual_topology_filename_prefix, data_aggregation_type, data_prefilter_type)");
	
	//ret_val = engEvalString(matlab_ep, "[result, road_network_graph_filename, intersection_sensor_network_graph_filename, permutation_vector_filename, intersection_sensor_vector_filename] = Perform_Prefiltering(current_directory, nodenum, measurement_number, speed, adjacency_matrix_filename, intersection_vector_filename, virtual_topology_filename_prefix, data_aggregation_type, data_prefilter_type)");
	
	/* get the result after performing the prefiltering */
	mx_result = engGetVariable(matlab_ep, "result");
	if(mx_result == NULL)
	{
		printf("matlab_perform_prefiltering(): engGetVariable() returns NULL for the Matlab variable called result n\n");
		return &prefiltering_result;
	}
	data = mxGetData(mx_result);
	memcpy(&result, data, sizeof(result));

	if(result == -1)
	{
		printf("matlab_perform_prefiltering(): Perform_Prefiltering returns error due to operation problem, such as newprim7()\n");
		return &prefiltering_result;
	}

	/* get the road network graph file name in the array of characters */
	mx_road_network_graph_filename = engGetVariable(matlab_ep, "road_network_graph_filename");
	if(mx_road_network_graph_filename == NULL)
	{
		printf("matlab_perform_prefiltering(): engGetVariable() returns NULL for the Matlab variable called road_network_graph_filename\n");
		return &prefiltering_result;
	}

	ptr = mxArrayToString(mx_road_network_graph_filename); //convert array to C string
	strcpy(road_network_graph_filename, ptr);

	/* get the intersection sensor network graph file name in the array of characters */
	mx_intersection_sensor_network_graph_filename = engGetVariable(matlab_ep, "intersection_sensor_network_graph_filename");
	if(mx_intersection_sensor_network_graph_filename == NULL)
	{
		printf("matlab_perform_prefiltering(): engGetVariable() returns NULL for the Matlab variable called intersection_sensor_network_graph_filename\n");
		return &prefiltering_result;
	}

	ptr = mxArrayToString(mx_intersection_sensor_network_graph_filename); //convert array to C string
	strcpy(intersection_sensor_network_graph_filename, ptr);

	/* get the sensor network graph file name in the array of characters */
	mx_sensor_network_graph_filename = engGetVariable(matlab_ep, "sensor_network_graph_filename");
	if(mx_sensor_network_graph_filename == NULL)
	{
		printf("matlab_perform_prefiltering(): engGetVariable() returns NULL for the Matlab variable called sensor_network_graph_filename\n");
		return &prefiltering_result;
	}

	ptr = mxArrayToString(mx_sensor_network_graph_filename); //convert array to C string
	strcpy(sensor_network_graph_filename, ptr);

	/* get the permutation vector file name in the array of characters */
	mx_permutation_vector_filename = engGetVariable(matlab_ep, "permutation_vector_filename");
	if(mx_permutation_vector_filename == NULL)
	{
		printf("matlab_perform_prefiltering(): engGetVariable() returns NULL for the Matlab variable called permutation_vector_filename\n");
		return &prefiltering_result;
	}

	ptr = mxArrayToString(mx_permutation_vector_filename); //convert array to C string
	strcpy(permutation_vector_filename, ptr);

	/* get the intersection pair matrix file name in the array of characters */
	mx_intersection_pair_matrix_filename = engGetVariable(matlab_ep, "intersection_pair_matrix_filename");
	if(mx_intersection_pair_matrix_filename == NULL)
	{
		printf("matlab_perform_prefiltering(): engGetVariable() returns NULL for the Matlab variable called intersection_pair_matrix_filename\n");
		return &prefiltering_result;
	}

	ptr = mxArrayToString(mx_intersection_pair_matrix_filename); //convert array to C string
	strcpy(intersection_pair_matrix_filename, ptr);

	/* get the intersection sensor id vector file name in the array of characters */
	mx_intersection_sensor_id_vector_filename = engGetVariable(matlab_ep, "intersection_sensor_id_vector_filename");
	if(mx_intersection_sensor_id_vector_filename == NULL)
	{
		printf("matlab_perform_prefiltering(): engGetVariable() returns NULL for the Matlab variable called intersection_sensor_id_vector_filename\n");
		return &prefiltering_result;
	}

	ptr = mxArrayToString(mx_intersection_sensor_id_vector_filename); //convert array to C string
	strcpy(intersection_sensor_id_vector_filename, ptr);

	/* get the nonintersection sensor id vector file name in the array of characters */
	mx_nonintersection_sensor_id_vector_filename = engGetVariable(matlab_ep, "nonintersection_sensor_id_vector_filename");
	if(mx_nonintersection_sensor_id_vector_filename == NULL)
	{
		printf("matlab_perform_prefiltering(): engGetVariable() returns NULL for the Matlab variable called nonintersection_sensor_id_vector_filename\n");
		return &prefiltering_result;
	}

	ptr = mxArrayToString(mx_nonintersection_sensor_id_vector_filename); //convert array to C string
	strcpy(nonintersection_sensor_id_vector_filename, ptr);

	/* get the sensor degree vector file name in the array of characters */
	mx_sensor_degree_vector_filename = engGetVariable(matlab_ep, "sensor_degree_vector_filename");
	if(mx_sensor_degree_vector_filename == NULL)
	{
		printf("matlab_perform_prefiltering(): engGetVariable() returns NULL for the Matlab variable called sensor_degree_vector_filename\n");
		return &prefiltering_result;
	}

	ptr = mxArrayToString(mx_sensor_degree_vector_filename); //convert array to C string
	strcpy(sensor_degree_vector_filename, ptr);

	/* copy the prefiltering result */
	prefiltering_result.result = result;
	strcpy(prefiltering_result.road_network_graph_filename, road_network_graph_filename);
	strcpy(prefiltering_result.intersection_sensor_network_graph_filename, intersection_sensor_network_graph_filename);
	strcpy(prefiltering_result.sensor_network_graph_filename, sensor_network_graph_filename);
	strcpy(prefiltering_result.permutation_vector_filename, permutation_vector_filename);
	strcpy(prefiltering_result.intersection_pair_matrix_filename, intersection_pair_matrix_filename);
	strcpy(prefiltering_result.intersection_sensor_id_vector_filename, intersection_sensor_id_vector_filename);
	strcpy(prefiltering_result.nonintersection_sensor_id_vector_filename, nonintersection_sensor_id_vector_filename);
	strcpy(prefiltering_result.sensor_degree_vector_filename, sensor_degree_vector_filename);

	/* release the memory for mx variables */
	mxDestroyArray(mx_current_directory);
	mxDestroyArray(mx_nodenum);
	mxDestroyArray(mx_measurement_number);
	mxDestroyArray(mx_speed);
	mxDestroyArray(mx_speed_deviation);
	mxDestroyArray(mx_adjacency_matrix_filename);
	mxDestroyArray(mx_intersection_vector_filename);
	mxDestroyArray(mx_virtual_topology_filename_prefix);
	mxDestroyArray(mx_data_aggregation_type);
	mxDestroyArray(mx_data_prefilter_type);
	mxDestroyArray(mx_result);
	mxDestroyArray(mx_road_network_graph_filename);
	mxDestroyArray(mx_intersection_sensor_network_graph_filename);
	mxDestroyArray(mx_sensor_network_graph_filename);
	mxDestroyArray(mx_permutation_vector_filename);
	mxDestroyArray(mx_intersection_pair_matrix_filename);
	mxDestroyArray(mx_intersection_sensor_id_vector_filename);
	mxDestroyArray(mx_nonintersection_sensor_id_vector_filename);
	mxDestroyArray(mx_sensor_degree_vector_filename);

        return &prefiltering_result;
}

struct_matlab_localization_result* matlab_perform_graph_matching(Engine *matlab_ep, char *current_directory, double speed, char *adjacency_matrix_filename, char *intersection_vector_filename, int data_aggregation_type, int data_prefilter_type, struct_matlab_prefiltering_result* prefiltering_result, boolean nonintersection_sensor_localization_flag)
{ //perform the prefiltering according to the given prefiltering type
        static struct_matlab_localization_result localization_result; //result of localization
	int result; //result of matlab main function Perform_Prefiltering()
	//double current_directory_buf[BUF_SIZE], road_network_graph_filename_buf[BUF_SIZE], sensor_network_graph_filename_buf[BUF_SIZE], permutation_vector_filename_buf[BUF_SIZE];
	int length; //string length
	mxArray *mx_current_directory = NULL, *mx_speed = NULL, *mx_adjacency_matrix_filename = NULL, *mx_road_network_graph_filename = NULL, *mx_intersection_sensor_network_graph_filename = NULL, *mx_sensor_network_graph_filename = NULL, *mx_permutation_vector_filename = NULL, *mx_intersection_pair_matrix_filename = NULL, *mx_intersection_sensor_id_vector_filename = NULL, *mx_nonintersection_sensor_id_vector_filename = NULL, *mx_sensor_degree_vector_filename = NULL, *mx_nonintersection_sensor_localization_flag = NULL, *mx_data_aggregation_type = NULL, *mx_data_prefilter_type = NULL, *mx_result = NULL, *mx_error_ratio = NULL, *mx_E_ratio = NULL, *mx_M_ratio = NULL;
	int i; //loop index
	int ret_val;
	void *data = NULL;
	double error_ratio = 0; //error ratio of localization
	double E_ratio = 0; //ratio indicating the relative difference between Er and Ev
	double M_ratio = 0; //ratio indicating the relative difference between Mr and Mv

	/* initialize result for prefiltering */
	localization_result.result = -1;
	localization_result.error_ratio = -1;
	localization_result.E_ratio = -1;
	localization_result.M_ratio = -1;

	/* set up Malab variables */
/* 	length = (int)strlen(current_directory); */

/* #ifdef __DEBUG_LEVEL_1__ */
/* 	printf("current_directory_buf:\n"); */
/* #endif */
/* 	for(i = 0; i < length; i++) */
/* 	{ */
/* 		current_directory_buf[i] = current_directory[i]; */
/* #ifdef __DEBUG_LEVEL_1__ */
/* 		printf("%f ", current_directory_buf[i]); */
/* #endif */
/* 	} */
/* #ifdef __DEBUG_LEVEL_1__ */
/* 	printf("\n\n"); */
/* #endif */

/* 	mx_current_directory = mxCreateDoubleMatrix(1, length, mxREAL); */
/* 	memcpy((void*)mxGetPr(mx_current_directory), (void*)current_directory_buf, sizeof(double)*length); */

	mx_current_directory = mxCreateString(current_directory); //create 1-by-N string mxArray initialized to specified string
	engPutVariable(matlab_ep, "current_directory", mx_current_directory);

	mx_speed = mxCreateDoubleScalar(speed);
	engPutVariable(matlab_ep, "speed", mx_speed);

	mx_adjacency_matrix_filename = mxCreateString(adjacency_matrix_filename); //create 1-by-N string mxArray initialized to specified string
	engPutVariable(matlab_ep, "adjacency_matrix_filename", mx_adjacency_matrix_filename);

	mx_nonintersection_sensor_localization_flag = mxCreateDoubleScalar(nonintersection_sensor_localization_flag);
	engPutVariable(matlab_ep, "nonintersection_sensor_localization_flag", mx_nonintersection_sensor_localization_flag);

	mx_data_aggregation_type = mxCreateDoubleScalar(data_aggregation_type);
	engPutVariable(matlab_ep, "data_aggregation_type", mx_data_aggregation_type);

	mx_data_prefilter_type = mxCreateDoubleScalar(data_prefilter_type);
	engPutVariable(matlab_ep, "data_prefilter_type", mx_data_prefilter_type);

	/* get road_network_graph_filename */
/* 	length = (int)strlen(prefiltering_result->road_network_graph_filename); */
/* #ifdef __DEBUG_LEVEL_1__ */
/* 	printf("road_network_graph_filename_buf:\n"); */
/* #endif */
/* 	for(i = 0; i < length; i++) */
/* 	{ */
/* 		road_network_graph_filename_buf[i] = prefiltering_result->road_network_graph_filename[i]; */
/* #ifdef __DEBUG_LEVEL_1__ */
/* 		printf("%f ", road_network_graph_filename_buf[i]); */
/* #endif */
/* 	} */
/* #ifdef __DEBUG_LEVEL_1__ */
/* 	printf("\n\n"); */
/* #endif */

/* 	mx_road_network_graph_filename = mxCreateDoubleMatrix(1, length, mxREAL); */
/* 	memcpy((void*)mxGetPr(mx_road_network_graph_filename), (void*)road_network_graph_filename_buf, sizeof(double)*length); */

	mx_road_network_graph_filename = mxCreateString(prefiltering_result->road_network_graph_filename); //create 1-by-N string mxArray initialized to specified string
	engPutVariable(matlab_ep, "road_network_graph_filename", mx_road_network_graph_filename);

	/* get intersection_sensor_network_graph_filename */
	mx_intersection_sensor_network_graph_filename = mxCreateString(prefiltering_result->intersection_sensor_network_graph_filename); //create 1-by-N string mxArray initialized to specified string
	engPutVariable(matlab_ep, "intersection_sensor_network_graph_filename", mx_intersection_sensor_network_graph_filename);

	/* get sensor_network_graph_filename */
	mx_sensor_network_graph_filename = mxCreateString(prefiltering_result->sensor_network_graph_filename); //create 1-by-N string mxArray initialized to specified string
	engPutVariable(matlab_ep, "sensor_network_graph_filename", mx_sensor_network_graph_filename);

	/* get permutation_vector_filename */
	mx_permutation_vector_filename = mxCreateString(prefiltering_result->permutation_vector_filename); //create 1-by-N string mxArray initialized to specified string
	engPutVariable(matlab_ep, "permutation_vector_filename", mx_permutation_vector_filename);

	/* get intersection_pair_matrix_filename */
	mx_intersection_pair_matrix_filename = mxCreateString(prefiltering_result->intersection_pair_matrix_filename); //create 1-by-N string mxArray initialized to specified string
	engPutVariable(matlab_ep, "intersection_pair_matrix_filename", mx_intersection_pair_matrix_filename);

	/* get intersection_sensor_id_vector_filename */
	mx_intersection_sensor_id_vector_filename = mxCreateString(prefiltering_result->intersection_sensor_id_vector_filename); //create 1-by-N string mxArray initialized to specified string
	engPutVariable(matlab_ep, "intersection_sensor_id_vector_filename", mx_intersection_sensor_id_vector_filename);

	/* get nonintersection_sensor_id_vector_filename */
	mx_nonintersection_sensor_id_vector_filename = mxCreateString(prefiltering_result->nonintersection_sensor_id_vector_filename); //create 1-by-N string mxArray initialized to specified string
	engPutVariable(matlab_ep, "nonintersection_sensor_id_vector_filename", mx_nonintersection_sensor_id_vector_filename);

	/* get sensor_degree_vector_filename */
	mx_sensor_degree_vector_filename = mxCreateString(prefiltering_result->sensor_degree_vector_filename); //create 1-by-N string mxArray initialized to specified string
	engPutVariable(matlab_ep, "sensor_degree_vector_filename", mx_sensor_degree_vector_filename);

	/** perform matlab script using matlab engine */
	ret_val = engEvalString(matlab_ep, "[result, error_ratio, E_ratio, M_ratio] = Perform_Graph_Matching(current_directory, speed, adjacency_matrix_filename, road_network_graph_filename, intersection_sensor_network_graph_filename, sensor_network_graph_filename, permutation_vector_filename, intersection_pair_matrix_filename, intersection_sensor_id_vector_filename, nonintersection_sensor_id_vector_filename, sensor_degree_vector_filename, nonintersection_sensor_localization_flag, data_aggregation_type, data_prefilter_type)");

	//ret_val = engEvalString(matlab_ep, "[result, error_ratio, E_ratio, M_ratio] = Perform_Graph_Matching(current_directory, road_network_graph_filename, sensor_network_graph_filename, permutation_vector_filename, data_aggregation_type, data_prefilter_type)");
	
	mx_result = engGetVariable(matlab_ep, "result");
	if(mx_result == NULL)
	{
		printf("matlab_perform_prefiltering(): engGetVariable() returns NULL for the Matlab variable called result n\n");
		return &localization_result;
	}
	data = mxGetData(mx_result);
	memcpy(&result, data, sizeof(result));

	if(result == -1)
	{
		printf("matlab_perform_prefiltering(): Perform_Prefiltering returns error due to operation problem, such as newprim7()\n");
		return &localization_result;
	}

	mx_error_ratio = engGetVariable(matlab_ep, "error_ratio");
	if(mx_error_ratio == NULL)
	{
		printf("matlab_perform_localization(): engGetVariable() returns NULL for Matlab variable n\n");
		return &localization_result;
	}
	data = mxGetData(mx_error_ratio);
	memcpy(&error_ratio, data, sizeof(error_ratio));

	mx_E_ratio = engGetVariable(matlab_ep, "E_ratio");
	if(mx_E_ratio == NULL)
	{
		printf("matlab_perform_localization(): engGetVariable() returns NULL for Matlab variable n\n");
		return &localization_result;
	}
	data = mxGetData(mx_E_ratio);
	memcpy(&E_ratio, data, sizeof(E_ratio));

	mx_M_ratio = engGetVariable(matlab_ep, "M_ratio");
	if(mx_M_ratio == NULL)
	{
		printf("matlab_perform_localization(): engGetVariable() returns NULL for Matlab variable n\n");
		return &localization_result;
	}
	data = mxGetData(mx_M_ratio);
	memcpy(&M_ratio, data, sizeof(M_ratio));

	/* release the memory for mx variables */
	mxDestroyArray(mx_current_directory);
	mxDestroyArray(mx_speed);
	mxDestroyArray(mx_adjacency_matrix_filename);
	mxDestroyArray(mx_road_network_graph_filename);
	mxDestroyArray(mx_intersection_sensor_network_graph_filename);
	mxDestroyArray(mx_sensor_network_graph_filename);
	mxDestroyArray(mx_permutation_vector_filename);
	mxDestroyArray(mx_intersection_pair_matrix_filename);
	mxDestroyArray(mx_intersection_sensor_id_vector_filename);
	mxDestroyArray(mx_nonintersection_sensor_id_vector_filename);
	mxDestroyArray(mx_sensor_degree_vector_filename);
	mxDestroyArray(mx_nonintersection_sensor_localization_flag);
	mxDestroyArray(mx_data_aggregation_type);
	mxDestroyArray(mx_data_prefilter_type);
	mxDestroyArray(mx_result);
	mxDestroyArray(mx_error_ratio);
	mxDestroyArray(mx_E_ratio);
	mxDestroyArray(mx_M_ratio);

	/* return result for localization */
	localization_result.result = result;
	localization_result.error_ratio = (float)error_ratio;
	localization_result.E_ratio = (float)E_ratio;
	localization_result.M_ratio = (float)M_ratio;

	return &localization_result;
}

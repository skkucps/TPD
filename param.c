/**
 *  File: param.c
    Description: initialize the parameters of the system and the workload
    Date: 04/07/2005
    Update Date: 04/07/2005
    Maker: Jaehoon Jeong
*/

#include "stdafx.h"
#include <ctype.h> //isalpha
#include "param.h" //parameter structure and related constants
#include "util.h" //unit conversion funtions, such as convert_mile_per_hour_to_km_per_hour().
/* NOTE: when I didn't include util.h, convert_mile_per_hour_to_km_per_hour() returned 
   a wrong value because it seems that param.c's code is not bound to util.c's code */

#include "gsl-util.h" //GSL_Vanet_Compute_TravelTime_And_Deviation()
#include "shortest-path.h" //Free_Graph()
#include "all-pairs-shortest-paths.h" //Floyd_Warshall_Free_Matrices_For_Movement() and Floyd_Warshall_Free_Matrices_For_EDD()

void init_parameter(struct parameter *param, char* conf_file)
{ // initialize simulation parameter, param with conf_file
	FILE *fp; //destination file
	//char* input_buf = NULL; //input buffer
	char input_buf[BUF_SIZE]; //input buffer
  	char key_buf[BUF_SIZE], value_buf[BUF_SIZE]; //key buffer, and value buffer
	//int input_buf_size = 0;
	size_t input_buf_size = 0; //size of input buffer
	//int result; //return result
	char *result_code; //return result as pointer
	double operand[OPERAND_NUM]; //operands
	double value; //value corresponding to key
	int operand_number; //number of operands of value
	char *token; //pointer to token
	boolean enum_flag; //flag to indentify if the type of the operand is enum
	int i; //loop index

	/** flags for checking if all parameters are set */
	/* VANET Forwarding Scheme */
	boolean flag_vanet_forwarding_scheme = FALSE;

	/* TPD Parameters */
	boolean flag_tpd_encounter_probability_threshold = FALSE;
	boolean flag_tpd_delivery_probability_threshold = FALSE;
	boolean flag_tpd_encounter_graph_optimization_flag = FALSE;
	boolean flag_tpd_encounter_graph_source_routing_flag = FALSE;

	/* Simulation Logging Options */
	boolean flag_forwarding_probability_and_statistics_flag = FALSE; //flag for the flag to indicate whether the forwarding probability_and_statistics information is logged every EDD update or not

	/* Infrastructure Node Deployment */
	boolean flag_AP_deployment_flag = FALSE;
	boolean flag_AP_deployment_probability = FALSE;
	boolean flag_SN_deployment_flag = FALSE;
	boolean flag_SN_deployment_probability = FALSE;

	/* Performance Evaluation Type*/
	boolean flag_evaluation_type = FALSE;

	/* Performance Comparison Target Type */
	boolean flag_comparison_target_type = FALSE;

	/* Length Unit */
	boolean flag_length_unit = FALSE;

	/* Graph Parameters */
	boolean flag_graph_file_name = FALSE;
	boolean flag_graph_node_number = FALSE;

	/* Mobility Parameters */
	boolean flag_mobility_file_name = FALSE;

	/* Data Forwarding Parameters */
	boolean flag_data_forwarding_mode = FALSE;
	boolean flag_data_forwarding_link_selection = FALSE;
	boolean flag_data_forwarding_two_way_forwarding_flag = FALSE;
	boolean flag_data_forwarding_multiple_target_point_flag = FALSE;
	boolean flag_data_forwarding_maximum_target_point_number = FALSE;

	/* Packet Delay Measurement Parameters */
	boolean flag_packet_delay_measurement_flag = FALSE;
	boolean flag_packet_delay_measurement_target_point = FALSE;
	boolean flag_packet_delay_measurement_time = FALSE;

	/* Target Point Parameters */
	boolean flag_target_point_interdistance = FALSE;
	boolean flag_target_point_index = FALSE;

	/* Communication Parameters */
	boolean flag_communication_range = FALSE;
	boolean flag_communication_one_hop_delay = FALSE;
	boolean flag_communication_data_packet_size = FALSE;
	boolean flag_communication_packet_ttl = FALSE;
	boolean flag_communication_packet_ttl_override_flag = FALSE;
	boolean flag_communication_packet_hop_limit = FALSE;
	boolean flag_communication_packet_interarrival_time = FALSE;
	boolean flag_communication_packet_interarrival_time_distribution = FALSE;
	boolean flag_communication_packet_interarrival_time_standard_deviation = FALSE;
	boolean flag_communication_packet_maximum_interarrival_time_factor = FALSE;
	boolean flag_communication_packet_maximum_number = FALSE;
	boolean flag_communication_packet_delivery_probability_threshold = FALSE;
	boolean flag_communication_packet_reverse_traversal_hop_distance_threshold = FALSE;
	boolean flag_communication_multiple_AP_flag = FALSE;
	boolean flag_communication_AP_maximum_number = FALSE;
	boolean flag_communication_AP_packet_generation_schedule_time = FALSE;
	boolean flag_communication_multiple_SN_flag = FALSE;
	boolean flag_communication_SN_maximum_number = FALSE;
	boolean flag_communication_multiple_target_point_flag = FALSE;

	/* Sensor Parameters */
	boolean flag_sensor_schedule_mode = FALSE;
	boolean flag_sensor_number = FALSE;
	boolean flag_sensor_density = FALSE;
	boolean flag_sensor_density_distribution = FALSE;
	boolean flag_sensor_density_standard_deviation = FALSE;
	boolean flag_sensor_density_maximum_deviation = FALSE;
	boolean flag_sensor_deployment_distribtuion = FALSE;
	boolean flag_sensor_deployment_standard_deviation = FALSE;
	boolean flag_sensor_work_time = FALSE;
	boolean flag_sensor_think_time = FALSE;
	boolean flag_sensor_think_time_distribution = FALSE;
	boolean flag_sensor_think_time_standard_deviation = FALSE;
	boolean flag_sensor_energy = FALSE;
	boolean flag_sensor_energy_distribution = FALSE;
	boolean flag_sensor_energy_standard_deviation = FALSE;
	boolean flag_sensor_energy_maximum_deviation = FALSE;
	boolean flag_sensor_energy_consumption_rate = FALSE;
	boolean flag_sensor_warm_up_time = FALSE;
	boolean flag_sensor_turn_on_energy_consumption = FALSE;
	boolean flag_sensor_sensing_range = FALSE;
	boolean flag_sensor_sensing_range_distribution = FALSE;
	boolean flag_sensor_sensing_range_standard_deviation = FALSE;
	boolean flag_sensor_time_sync_max_error = FALSE;
	boolean flag_sensor_time_sync_error = FALSE;
	boolean flag_sensor_time_sync_error_start = FALSE;
	boolean flag_sensor_time_sync_error_end = FALSE;
	boolean flag_sensor_time_sync_error_step = FALSE;
	boolean flag_sensor_time_sync_error_distribution = FALSE;
	boolean flag_sensor_time_sync_error_standard_deviation = FALSE;
	boolean flag_sensor_time_sync_error_standard_deviation_start = FALSE;
	boolean flag_sensor_time_sync_error_standard_deviation_end = FALSE;
	boolean flag_sensor_time_sync_error_standard_deviation_step = FALSE;
	boolean flag_sensor_detection_missing_probability = FALSE;
	boolean flag_sensor_detection_missing_probability_start = FALSE;
	boolean flag_sensor_detection_missing_probability_end = FALSE;
	boolean flag_sensor_detection_missing_probability_step = FALSE;
	boolean flag_sensor_duplicate_detection_probability = FALSE;
	boolean flag_sensor_duplicate_detection_probability_start = FALSE;
	boolean flag_sensor_duplicate_detection_probability_end = FALSE;
	boolean flag_sensor_duplicate_detection_probability_step = FALSE;
	boolean flag_sensor_scan_type = FALSE;
	boolean flag_sensor_initial_hole_handling_algorithm = FALSE;
	boolean flag_sensor_hole_handling_algorithm = FALSE;
	boolean flag_sensor_hole_handling_mode = FALSE;
	boolean flag_sensor_movement_time_percentage = FALSE;

	/* Network Parameters */
	boolean flag_network_width = FALSE;
	boolean flag_network_height = FALSE;
	boolean flag_network_gap = FALSE;
	boolean flag_network_sensor_distribution = FALSE;
	boolean flag_network_sensor_standard_deviation = FALSE;

	/* Vehicle Parameters */
	boolean flag_vehicle_vanet_target_vehicle_flag = FALSE;
	boolean flag_vehicle_vanet_stationary_vehicle_flag = FALSE;
	boolean flag_vehicle_vanet_acl_measurement_flag = FALSE;
	boolean flag_vehicle_vanet_vehicular_traffic_model = FALSE;
	boolean flag_vehicle_vanet_edd_model = FALSE;
	boolean flag_vehicle_vanet_edd_computation_model = FALSE;
	boolean flag_vehicle_vanet_tbd_edd_computation_type = FALSE;
	boolean flag_vehicle_vanet_edge_delay_model = FALSE;
	boolean flag_vehicle_vanet_forwarding_type = FALSE;
	boolean flag_vehicle_vanet_intersection_forwarding_type = FALSE;
	boolean flag_vehicle_vanet_target_point_selection_type = FALSE;
	boolean flag_vehicle_vanet_target_point_computation_method = FALSE;
	boolean flag_vehicle_vanet_target_point_search_space_type = FALSE;
	boolean flag_vehicle_vanet_target_point_recomputation_interval_denominator = FALSE;
	boolean flag_vehicle_vanet_vehicle_trajectory_exposure_degree = FALSE;
	boolean flag_vehicle_vanet_vehicle_trajectory_length_type = FALSE;
	boolean flag_vehicle_vanet_metric_type = FALSE;
	boolean flag_vehicle_vanet_target_point_optimization_function_type = FALSE;

	boolean flag_vehicle_maximum_number = FALSE;
	boolean flag_vehicle_packet_generating_entity_number = FALSE;
	boolean flag_vehicle_AP_passing_entity_percentage = FALSE;
	boolean flag_vehicle_packet_generation_schedule_time = FALSE;
	boolean flag_vehicle_edd_update_period = FALSE;
	boolean flag_vehicle_step_mode = FALSE;
	boolean flag_vehicle_step_time = FALSE;
	boolean flag_vehicle_initial_arrival_time = FALSE;
	boolean flag_vehicle_interarrival_time = FALSE;
	boolean flag_vehicle_interarrival_time_start = FALSE;
	boolean flag_vehicle_interarrival_time_end = FALSE;
	boolean flag_vehicle_interarrival_time_step = FALSE;
	boolean flag_vehicle_interarrival_time_distribution = FALSE;
	boolean flag_vehicle_interarrival_time_standard_deviation = FALSE;
	boolean flag_vehicle_maximum_interarrival_time_factor = FALSE;

	boolean flag_vehicle_minimum_speed_in_mile_per_hour = FALSE;
	boolean flag_vehicle_maximum_speed_in_mile_per_hour = FALSE;
	boolean flag_vehicle_speed_in_mile_per_hour = FALSE;
	
	boolean flag_vehicle_speed_bound_coefficient = FALSE;

	boolean flag_vehicle_speed_distribution = FALSE;
	boolean flag_vehicle_speed_standard_deviation_in_mile_per_hour = FALSE;

	boolean flag_vehicle_speed_standard_deviation_start = FALSE;
	boolean flag_vehicle_speed_standard_deviation_end = FALSE;
	boolean flag_vehicle_speed_standard_deviation_step = FALSE;

	boolean flag_vehicle_think_time = FALSE;
	boolean flag_vehicle_think_time_start = FALSE;
	boolean flag_vehicle_think_time_end = FALSE;
	boolean flag_vehicle_think_time_step = FALSE;
	boolean flag_vehicle_think_time_distribution = FALSE;
	boolean flag_vehicle_think_time_standard_deviation = FALSE;

	boolean flag_vehicle_path_length_distribution = FALSE;
	boolean flag_vehicle_path_length_standard_deviation_in_mile = FALSE; 

	boolean flag_vehicle_path_length_standard_deviation_start = FALSE;
	boolean flag_vehicle_path_length_standard_deviation_end = FALSE;
	boolean flag_vehicle_path_length_standard_deviation_step = FALSE;

	boolean flag_vehicle_path_minimum_hop_count = FALSE;

/* Data Processing Parameters */
	boolean flag_data_aggregation_type = FALSE;
	boolean flag_data_aggregation_window_size = FALSE;
	boolean flag_data_aggregation_window_size_start = FALSE;
	boolean flag_data_aggregation_window_size_end = FALSE;
	boolean flag_data_aggregation_window_size_step = FALSE;

	boolean flag_data_measurement_time = FALSE;
	boolean flag_data_number_of_split_measurement_times = FALSE;

	boolean flag_data_prefilter_type = FALSE;

	/* Simulation Time */
	boolean flag_simulation_seed = FALSE;
	boolean flag_simulation_mode = FALSE;
	boolean flag_simulation_time = FALSE;
	boolean flag_simulation_iteration_number = FALSE;
	boolean flag_simulation_run = FALSE;

	/* initialize param with zeroes */
	memset(param, 0, sizeof(*param));

	/* open conf_file */
	fp = fopen(conf_file, "r");
	if(!fp)
	{
		fprintf(stderr, "Error: unable to open file \"%s\"\n", conf_file);
		exit(1);
	}

	/* set each field of parameter with conf_file information */
	//while((result = getline(&input_buf, &input_buf_size, fp)) != -1)
	while((result_code = fgets(input_buf, sizeof(input_buf), fp)) != NULL)
	//@ getline allocates a memory block for input_buf dynamically and expands it if needed.
	//while(fscanf(fp, "%s", input_buf) == 1)
	{
		input_buf_size = strlen(input_buf); //2006-7-21
                ////////////////////////////////////

		token = (char*) strtok(input_buf, "=");

		strcpy(key_buf, token);
		if(key_buf == NULL)
		{
			printf("Error: system configuration file has null key!\n");
			exit(1);
		}
		else if((key_buf[0] == '\n') || (key_buf[0] == '\r') || (key_buf[0] == ' ') || (key_buf[0] == '\t')) //this line is white space
			continue;
		else if(key_buf[0] == '#') //this is comment line
			continue;

		token = (char*) strtok(NULL, "\n\r");
		/*
		strcpy(value_buf, token);
		if(value_buf == NULL)
		{
			printf("Error: system configuration file has null value!\n");
			exit(1);
		}
		*/
		if(token == NULL)
		{
			printf("Error: system configuration file has null value!\n");
			exit(1);
		}
		else if(strcmp(key_buf, "graph_file_name") == 0)
		{
			strcpy(value_buf, token);			
		}
		else if(strcmp(key_buf, "mobility_file_name") == 0)
		{
			strcpy(value_buf, token);			
		}
		else //else-1
		{
			strcpy(value_buf, token);

			enum_flag = FALSE;
			i = operand_number = 0;

			token = (char*) strtok(value_buf, "*");
			if(token == NULL)
			{
				printf("token is NULL!\n");
				exit(1);
			}

			do
			{	
				if(strcmp(token, "KILO") == 0)
				{
					operand[i++] = KILO;
				}
				else if(strcmp(token, "MEGA") == 0)
				{
					operand[i++] = MEGA;
				}
				else if(strcmp(token, "GIGA") == 0)
				{
					operand[i++] = GIGA;
				}
				else if(strcmp(token, "MILLI") == 0)
				{
					operand[i++] = MILLI;
				}
				else if(strcmp(token, "EVALUATION_UNKNOWN") == 0)
				{
					value = EVALUATION_UNKNOWN;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "EVALUATION_WORK_TIME") == 0)
				{
					value = EVALUATION_WORK_TIME;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "EVALUATION_ENERGY_VARIATION") == 0)
				{
					value = EVALUATION_ENERGY_VARIATION;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "EVALUATION_SENSOR_DENSITY") == 0)
				{
					value = EVALUATION_SENSOR_DENSITY;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "EVALUATION_PATH_LENGTH_VARIATION") == 0)
				{
					value = EVALUATION_PATH_LENGTH_VARIATION;
					enum_flag = TRUE;
					break;
				}
				/** VANET Evaluation Types */
				else if(strcmp(token, "EVALUATION_VEHICLE_MAXIMUM_NUMBER") == 0)
				{
					value = EVALUATION_VEHICLE_MAXIMUM_NUMBER;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "EVALUATION_VEHICLE_PACKET_GENERATING_ENTITY_NUMBER") == 0)
				{
					value = EVALUATION_VEHICLE_PACKET_GENERATING_ENTITY_NUMBER;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "EVALUATION_VEHICLE_SPEED") == 0)
				{
					value = EVALUATION_VEHICLE_SPEED;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "EVALUATION_VEHICLE_SPEED_STANDARD_DEVIATION") == 0)
				{
					value = EVALUATION_VEHICLE_SPEED_STANDARD_DEVIATION;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "EVALUATION_VEHICLE_INTERARRIVAL_TIME") == 0)
				{
					value = EVALUATION_VEHICLE_INTERARRIVAL_TIME;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "EVALUATION_COMMUNICATION_PACKET_INTERARRIVAL_TIME") == 0)
				{
					value = EVALUATION_COMMUNICATION_PACKET_INTERARRIVAL_TIME;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "EVALUATION_COMMUNICATION_PACKET_TTL") == 0)
				{
					value = EVALUATION_COMMUNICATION_PACKET_TTL;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "EVALUATION_COMMUNICATION_PACKET_DELIVERY_PROBABILITY_THRESHOLD") == 0)
				{
					value = EVALUATION_COMMUNICATION_PACKET_DELIVERY_PROBABILITY_THRESHOLD;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "EVALUATION_COMMUNICATION_AP_MAXIMUM_NUMBER") == 0)
				{
					value = EVALUATION_COMMUNICATION_AP_MAXIMUM_NUMBER;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "EVALUATION_SIMULATION_TIME") == 0)
				{
					value = EVALUATION_SIMULATION_TIME;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "EVALUATION_VEHICLE_AP_PASSING_ENTITY_PERCENTAGE") == 0)
				{
					value = EVALUATION_VEHICLE_AP_PASSING_ENTITY_PERCENTAGE;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "EVALUATION_COMMUNICATION_SN_MAXIMUM_NUMBER") == 0)
				{
					value = EVALUATION_COMMUNICATION_SN_MAXIMUM_NUMBER;
					enum_flag = TRUE;
					break;
				}
				/** VANET Comparison Types */
				else if(strcmp(token, "COMPARISON_EDD_AND_LINK_MODEL") == 0)
				{
					value = COMPARISON_EDD_AND_LINK_MODEL;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "COMPARISON_EDD_MODEL") == 0)
				{
					value = COMPARISON_EDD_MODEL;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "COMPARISON_TBD_EDD_COMPUTATION_TYPE") == 0)
				{
					value = COMPARISON_TBD_EDD_COMPUTATION_TYPE;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "COMPARISON_EDGE_DELAY_MODEL") == 0)
				{
					value = COMPARISON_EDGE_DELAY_MODEL;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "COMPARISON_INTERSECTION_FORWARDING_TYPE") == 0)
				{
					value = COMPARISON_INTERSECTION_FORWARDING_TYPE;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "COMPARISON_AVERAGE_CONVOY_LENGTH_ESTIMATION_TYPE") == 0)
				{
					value = COMPARISON_AVERAGE_CONVOY_LENGTH_ESTIMATION_TYPE;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "COMPARISON_AVERAGE_LINK_DELAY_ESTIMATION_TYPE") == 0)
				{
					value = COMPARISON_AVERAGE_LINK_DELAY_ESTIMATION_TYPE;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "COMPARISON_TARGET_POINT_SELECTION_TYPE") == 0)
				{
					value = COMPARISON_TARGET_POINT_SELECTION_TYPE;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "COMPARISON_TARGET_POINT_NUMBER") == 0)
				{
					value = COMPARISON_TARGET_POINT_NUMBER;
					enum_flag = TRUE;
					break;
				}
				/*************************/
				/** WSN Comparison Types */
				else if(strcmp(token, "COMPARISON_SCHEDULE_ALGORITHM") == 0)
				{
					value = COMPARISON_SCHEDULE_ALGORITHM;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "COMPARISON_HOLE_LABELING_ALGORITHM") == 0)
				{
					value = COMPARISON_HOLE_LABELING_ALGORITHM;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "STEP_TIME") == 0)
				{
					value = STEP_TIME;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "STEP_EDGE") == 0)
				{
					value = STEP_EDGE;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "STEP_PATH") == 0)
				{
					value = STEP_PATH;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "UNIT_METER") == 0)
				{
					value = UNIT_METER;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "UNIT_MILE") == 0)
				{
					value = UNIT_MILE;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "INF") == 0)
				{
					value = INF;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "INF_FACTOR") == 0)
				{
					value = INF_FACTOR;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "EXPONENTIAL") == 0)
				{
					value = EXPONENTIAL;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "UNIFORM") == 0)
				{
					value = UNIFORM;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "NORMAL") == 0)
				{
					value = NORMAL;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "ERLANG") == 0)
				{
					value = ERLANG;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "HYPERX") == 0)
				{
					value = HYPERX;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "EQUAL") == 0)
				{
					value = EQUAL;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "SENSOR_SCHEDULE_MODE_UNKNOWN") == 0)
				{
					value = SENSOR_SCHEDULE_MODE_UNKNOWN;
					enum_flag = TRUE;
					break;
				}				
				else if(strcmp(token, "SENSOR_SCHEDULE_MODE_EAGER_UPDATE") == 0)
				{
					value = SENSOR_SCHEDULE_MODE_EAGER_UPDATE;
					enum_flag = TRUE;
					break;
				}				
				else if(strcmp(token, "SENSOR_SCHEDULE_MODE_LAZY_UPDATE") == 0)
				{
					value = SENSOR_SCHEDULE_MODE_LAZY_UPDATE;
					enum_flag = TRUE;
					break;
				}				
				else if(strcmp(token, "SCAN_TURN_ON_ALL") == 0)
				{
					value = SCAN_TURN_ON_ALL;
					enum_flag = TRUE;
					break;
				}				
				else if(strcmp(token, "SCAN_NO_USE") == 0)
				{
					value = SCAN_NO_USE;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "SCAN_CONSTANT_SPEED_WITH_NONOPTIMAL_SLEEPING") == 0)
				{
					value = SCAN_CONSTANT_SPEED_WITH_NONOPTIMAL_SLEEPING;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "SCAN_VARIABLE_SPEED_WITH_NONOPTIMAL_SLEEPING") == 0)
				{
					value = SCAN_VARIABLE_SPEED_WITH_NONOPTIMAL_SLEEPING;
					enum_flag = TRUE;
					break;
				}

				else if(strcmp(token, "SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING") == 0)
				{
					value = SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING") == 0)
				{
					value = SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_VARIABLE_WORKING_TIME") == 0)
				{
					value = SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_VARIABLE_WORKING_TIME;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "SCAN_NO_USE_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING") == 0)
				{
					value = SCAN_NO_USE_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING") == 0)
				{
					value = SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING;
					enum_flag = TRUE;
					break;
				}

				else if(strcmp(token, "SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING") == 0)
				{
					value = SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING") == 0)
				{
					value = SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING_AND_WITH_SENSING_HOLE_HANDLING") == 0)
				{
					value = SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING_AND_WITH_SENSING_HOLE_HANDLING;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "SCAN_VARIABLE_SPEED_WITH_VARIABLE_SLEEPING_TIME_AND_WITH_SENSING_HOLE_HANDLING") == 0)
				{
					value = SCAN_VARIABLE_SPEED_WITH_VARIABLE_SLEEPING_TIME_AND_WITH_SENSING_HOLE_HANDLING;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "HOLE_HANDLING_NO_HANDLING") == 0)
				{
					value = HOLE_HANDLING_NO_HANDLING;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "HOLE_HANDLING_EXHAUSTIVE_SEARCH_ALGORITHM") == 0)
				{
					value = HOLE_HANDLING_EXHAUSTIVE_SEARCH_ALGORITHM;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "HOLE_HANDLING_GREEDY_ALGORITHM_BASED_MINIMAL_SPANNING_TREE") == 0)
				{
					value = HOLE_HANDLING_GREEDY_ALGORITHM_BASED_MINIMAL_SPANNING_TREE;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "HOLE_HANDLING_GREEDY_ALGORITHM_BASED_HEURISTICS") == 0)
				{
					value = HOLE_HANDLING_GREEDY_ALGORITHM_BASED_HEURISTICS;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "HOLE_HANDLING_ALL_ENTRANCE_POINTS") == 0)
				{
					value = HOLE_HANDLING_ALL_ENTRANCE_POINTS;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "HOLE_HANDLING_ALL_PROTECTION_POINTS") == 0)
				{
					value = HOLE_HANDLING_ALL_PROTECTION_POINTS;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "HOLE_HANDLING_RANDOM_LABELING") == 0)
				{
					value = HOLE_HANDLING_RANDOM_LABELING;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "HOLE_MODE_RESHUFFLE_LABELING") == 0)
				{
					value = HOLE_MODE_RESHUFFLE_LABELING;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "HOLE_MODE_INCREMENTAL_LABELING") == 0)
				{
					value = HOLE_MODE_INCREMENTAL_LABELING;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "AGGREGATION_TYPE_0") == 0)
				{
					value = AGGREGATION_TYPE_0;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "AGGREGATION_TYPE_1") == 0)
				{
					value = AGGREGATION_TYPE_1;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "PREFILTER_TYPE_0") == 0)
				{
					value = PREFILTER_TYPE_0;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "PREFILTER_TYPE_1") == 0)
				{
					value = PREFILTER_TYPE_1;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "PREFILTER_TYPE_2") == 0)
				{
					value = PREFILTER_TYPE_2;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "PREFILTER_TYPE_3") == 0)
				{
					value = PREFILTER_TYPE_3;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "SEED_DEFAULT") == 0)
				{
					value = SEED_DEFAULT;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "SEED_TIME") == 0)
				{
					value = SEED_TIME;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "TIME") == 0)
				{
					value = TIME;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "ITERATION") == 0)
				{
					value = ITERATION;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "SIM_SINGLE_SIMULATION") == 0)
				{
					value = SIM_SINGLE_SIMULATION;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "SIM_SENSOR_TIME_SYNC_ERROR_STD") == 0)
				{
					value = SIM_SENSOR_TIME_SYNC_ERROR_STD;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "SIM_VEHICLE_SPEED_STD") == 0)
				{
					value = SIM_VEHICLE_SPEED_STD;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "SIM_VEHICLE_INTERARRIVAL_TIME") == 0)
				{
					value = SIM_VEHICLE_INTERARRIVAL_TIME;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "SIM_SENSOR_TIME_SYNC_ERROR_STD_VERSUS_VEHICLE_SPEED_STD") == 0)
				{
					value = SIM_SENSOR_TIME_SYNC_ERROR_STD_VERSUS_VEHICLE_SPEED_STD;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_EDD_PER_VEHICLE_MODEL") == 0)
				{
					value = VANET_EDD_PER_VEHICLE_MODEL;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_EDD_PER_INTERSECTION_MODEL") == 0)
				{
					value = VANET_EDD_PER_INTERSECTION_MODEL;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_EDD_PER_SNODE_MODEL") == 0)
				{
					value = VANET_EDD_PER_SNODE_MODEL;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_EDD_HYBRID_MODEL") == 0)
				{
					value = VANET_EDD_HYBRID_MODEL;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_EDGE_DELAY_TBD_MODEL_FOR_FINITE_ROAD") == 0)
				{
					value = VANET_EDGE_DELAY_TBD_MODEL_FOR_FINITE_ROAD;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_EDGE_DELAY_TBD_MODEL_FOR_INFINITE_ROAD") == 0)
				{
					value = VANET_EDGE_DELAY_TBD_MODEL_FOR_INFINITE_ROAD;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_EDGE_DELAY_VADD_MODEL") == 0)
				{
					value = VANET_EDGE_DELAY_VADD_MODEL;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_EDGE_DELAY_TSF_MODEL") == 0)
				{
					value = VANET_EDGE_DELAY_TSF_MODEL;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_EDD_BASED_ON_TBD_WITH_FULL_PATH") == 0)
				{
					value = VANET_EDD_BASED_ON_TBD_WITH_FULL_PATH;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH") == 0)
				{
					value = VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_EDD_BASED_ON_TBD_WITH_ONE_HOP_AHEAD") == 0)
				{
					value = VANET_EDD_BASED_ON_TBD_WITH_ONE_HOP_AHEAD;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_EDD_COMPUTATION_BASED_ON_STOCHASTIC_MODEL") == 0)
				{
					value = VANET_EDD_COMPUTATION_BASED_ON_STOCHASTIC_MODEL;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_EDD_COMPUTATION_BASED_ON_SHORTEST_PATH_MODEL") == 0)
				{
					value = VANET_EDD_COMPUTATION_BASED_ON_SHORTEST_PATH_MODEL;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_FORWARDING_BASED_ON_VEHICLE") == 0)
				{
					value = VANET_FORWARDING_BASED_ON_VEHICLE;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_FORWARDING_BASED_ON_CONVOY") == 0)
				{
					value = VANET_FORWARDING_BASED_ON_CONVOY;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_INTERSECTION_FORWARDING_DEFERRED_FORWARDING") == 0)
				{
					value = VANET_INTERSECTION_FORWARDING_DEFERRED_FORWARDING;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_INTERSECTION_FORWARDING_EAGER_FORWARDING") == 0)
				{
					value = VANET_INTERSECTION_FORWARDING_EAGER_FORWARDING;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_VEHICULAR_TRAFFIC_CLOSED_NETWORK") == 0)
				{
					value = VANET_VEHICULAR_TRAFFIC_CLOSED_NETWORK;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_VEHICULAR_TRAFFIC_OPEN_NETWORK") == 0)
				{
					value = VANET_VEHICULAR_TRAFFIC_OPEN_NETWORK;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_SELECTION_STATIC_TARGET_POINT") == 0)
				{
					value = VANET_TARGET_POINT_SELECTION_STATIC_TARGET_POINT;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_SELECTION_CONVERGENT_TARGET_POINT") == 0)
				{
					value = VANET_TARGET_POINT_SELECTION_CONVERGENT_TARGET_POINT;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_SELECTION_REVERSE_PATH_TARGET_POINT") == 0)
				{
					value = VANET_TARGET_POINT_SELECTION_REVERSE_PATH_TARGET_POINT;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_SELECTION_PROGRESS_TARGET_POINT") == 0)
				{
					value = VANET_TARGET_POINT_SELECTION_PROGRESS_TARGET_POINT;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_SELECTION_ADAPTIVE_TARGET_POINT") == 0)
				{
					value = VANET_TARGET_POINT_SELECTION_ADAPTIVE_TARGET_POINT;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_SELECTION_DYNAMIC_TARGET_POINT") == 0)
				{
					value = VANET_TARGET_POINT_SELECTION_DYNAMIC_TARGET_POINT;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT") == 0)
				{
					value = VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_COMPUTATION_METHOD_OPTIMAL_INTERSECTION") == 0)
				{
					value = VANET_TARGET_POINT_COMPUTATION_METHOD_OPTIMAL_INTERSECTION;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_EARLIEST_ARRIVING_INTERSECTION") == 0)
				{
					value = VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_EARLIEST_ARRIVING_INTERSECTION;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_COMPUTATION_METHOD_HEADING_INTERSECTION") == 0)
				{
					value = VANET_TARGET_POINT_COMPUTATION_METHOD_HEADING_INTERSECTION;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_COMPUTATION_METHOD_END_INTERSECTION") == 0)
				{
					value = VANET_TARGET_POINT_COMPUTATION_METHOD_END_INTERSECTION;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_COMPUTATION_METHOD_RANDOM_INTERSECTION") == 0)
				{
					value = VANET_TARGET_POINT_COMPUTATION_METHOD_RANDOM_INTERSECTION;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_STATIC_FORWARDING") == 0)
				{
					value = VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_STATIC_FORWARDING;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_PARTIALLY_DYNAMIC_FORWARDING") == 0)
				{
					value = VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_PARTIALLY_DYNAMIC_FORWARDING;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_FULLY_DYNAMIC_FORWARDING") == 0)
				{
					value = VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_FULLY_DYNAMIC_FORWARDING;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_SEARCH_SPACE_VALID_FULL_TRAJECTORY") == 0)
				{
					value = VANET_TARGET_POINT_SEARCH_SPACE_VALID_FULL_TRAJECTORY;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_SEARCH_SPACE_VALID_PARTIAL_TRAJECTORY") == 0)
				{
					value = VANET_TARGET_POINT_SEARCH_SPACE_VALID_PARTIAL_TRAJECTORY;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE") == 0)
				{
					value = VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE") == 0)
				{
					value = VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_METRIC_EDD") == 0)
				{
					value = VANET_METRIC_EDD;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_METRIC_EDD_VAR") == 0)
				{
					value = VANET_METRIC_EDD_VAR;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_OPTIMIZATION_FUNCTION_EDD_AND_EAD_DIFFERENCE") == 0)
				{
					value = VANET_TARGET_POINT_OPTIMIZATION_FUNCTION_EDD_AND_EAD_DIFFERENCE;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_OPTIMIZATION_FUNCTION_EAD_WITH_CONSTRAINT_1") == 0)
				{
					value = VANET_TARGET_POINT_OPTIMIZATION_FUNCTION_EAD_WITH_CONSTRAINT_1;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "VANET_TARGET_POINT_OPTIMIZATION_FUNCTION_EAD_WITH_CONSTRAINT_2") == 0)
				{
					value = VANET_TARGET_POINT_OPTIMIZATION_FUNCTION_EAD_WITH_CONSTRAINT_2;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "DATA_FORWARDING_MODE_DOWNLOAD") == 0)
				{
					value = DATA_FORWARDING_MODE_DOWNLOAD;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "DATA_FORWARDING_MODE_UPLOAD") == 0)
				{
					value = DATA_FORWARDING_MODE_UPLOAD;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "DATA_FORWARDING_MODE_V2V") == 0)
				{
					value = DATA_FORWARDING_MODE_V2V;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "FORWARDING_LINK_SELECTION_ANGLE") == 0)
				{
					value = FORWARDING_LINK_SELECTION_ANGLE;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "FORWARDING_LINK_SELECTION_DISTANCE") == 0)
				{
					value = FORWARDING_LINK_SELECTION_DISTANCE;
					enum_flag = TRUE;
					break;	
				}
				else if(strcmp(token, "FORWARDING_LINK_SELECTION_DELAY") == 0)
				{
					value = FORWARDING_LINK_SELECTION_DELAY;
					enum_flag = TRUE;
					break;	
				}
				/* VANET Forwarding Schemes */
				else if(strcmp(token, "VANET_FORWARDING_VADD") == 0)
				{
					value = VANET_FORWARDING_VADD;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "VANET_FORWARDING_TBD") == 0)
				{
					value = VANET_FORWARDING_TBD;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "VANET_FORWARDING_TPD") == 0)
				{
					value = VANET_FORWARDING_TPD;
					enum_flag = TRUE;
					break;
				}
				else if (strcmp(token, "VANET_FORWARDING_TADB") == 0)
				{
					value = VANET_FORWARDING_TADB;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "VANET_FORWARDING_EPIDEMIC") == 0)
				{
					value = VANET_FORWARDING_EPIDEMIC;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "VANET_FORWARDING_TSF") == 0)
				{
					value = VANET_FORWARDING_TSF;
					enum_flag = TRUE;
					break;
				}
				else if(strcmp(token, "VANET_FORWARDING_TMA") == 0)
				{
					value = VANET_FORWARDING_TMA;
					enum_flag = TRUE;
					break;
				}
				//////////////////////////////
				else if(isalpha(token[0]) != 0)
				{
					printf("Error: %s is a wrong macro-constant!\n", token);
					exit(1);
				}
				else
				{
					operand[i++] = atof(token);
				}

				operand_number++;
				token = (char*) strtok(NULL, "*\n\r");
			} while(token); //end of do-while

			if(enum_flag == FALSE)
			{
				value = 1;
				for(i = 0; i < operand_number; i++)
					value *= operand[i];
			}
		} //end of else-1

		if(strcmp(key_buf, "vanet_forwarding_scheme") == 0)
		{
			param->vanet_forwarding_scheme = (vanet_forwarding_scheme_t)value;
			flag_vanet_forwarding_scheme = TRUE;
		}
		else if(strcmp(key_buf, "tpd_encounter_probability_threshold") == 0)
		{
			param->tpd_encounter_probability_threshold = value;
			flag_tpd_encounter_probability_threshold = TRUE;
		}
		else if(strcmp(key_buf, "tpd_delivery_probability_threshold") == 0)
		{
			param->tpd_delivery_probability_threshold = value;
			flag_tpd_delivery_probability_threshold = TRUE;
		}
		else if(strcmp(key_buf, "tpd_encounter_graph_optimization_flag") == 0)
		{
			param->tpd_encounter_graph_optimization_flag = (boolean)value;
			flag_tpd_encounter_graph_optimization_flag = TRUE;
		}
		else if(strcmp(key_buf, "tpd_encounter_graph_source_routing_flag") == 0)
		{
			param->tpd_encounter_graph_source_routing_flag = (boolean)value;
			flag_tpd_encounter_graph_source_routing_flag = TRUE;
		}
		else if(strcmp(key_buf, "forwarding_probability_and_statistics_flag") == 0)
		{
			param->forwarding_probability_and_statistics_flag = (boolean)value;		
			flag_forwarding_probability_and_statistics_flag = TRUE;		
		}
		else if(strcmp(key_buf, "AP_deployment_flag") == 0)
		{
			param->AP_deployment_flag = (boolean)value;		
			flag_AP_deployment_flag = TRUE;		
		}
		else if(strcmp(key_buf, "AP_deployment_probability") == 0)
		{
			param->AP_deployment_probability = value;		
			flag_AP_deployment_probability = TRUE;		
		}
		else if(strcmp(key_buf, "SN_deployment_flag") == 0)
		{
			param->SN_deployment_flag = (boolean)value;		
			flag_SN_deployment_flag = TRUE;		
		}
		else if(strcmp(key_buf, "SN_deployment_probability") == 0)
		{
			param->SN_deployment_flag = (boolean)value;		
			flag_AP_deployment_flag = TRUE;		
		}
		else if(strcmp(key_buf, "evaluation_type") == 0)
		{
			param->evaluation_type = (evaluation_type_t)value;		
			flag_evaluation_type = TRUE;		
		}
		else if(strcmp(key_buf, "comparison_target_type") == 0)
		{
			param->comparison_target_type = (comparison_target_type_t)value;
  	                flag_comparison_target_type = TRUE;		
		}
		else if(strcmp(key_buf, "length_unit") == 0)
		{
			param->length_unit = (length_unit_type_t)value;		
			flag_length_unit = TRUE;		
		}
		else if(strcmp(key_buf, "graph_file_name") == 0)
		{
			strcpy(param->graph_file_name, value_buf);		
			flag_graph_file_name = TRUE;		
		}
		else if(strcmp(key_buf, "graph_node_number") == 0)
		{
			param->graph_node_number = (int) value;		
			flag_graph_node_number = TRUE;		
		}
		else if(strcmp(key_buf, "mobility_file_name") == 0)
		{
			strcpy(param->mobility_file_name, value_buf);		
			flag_mobility_file_name = TRUE;		
		}
		else if(strcmp(key_buf, "mobility_file_name") == 0)
		{
			strcpy(param->mobility_file_name, value_buf);		
			flag_mobility_file_name = TRUE;		
		}
		else if(strcmp(key_buf, "data_forwarding_mode") == 0)
		{
			param->data_forwarding_mode = (data_forwarding_mode_t)value;		
			flag_data_forwarding_mode = TRUE;		
		}
		else if(strcmp(key_buf, "data_forwarding_link_selection") == 0)
		{
			param->data_forwarding_link_selection = (data_forwarding_link_selection_t)value;		
			flag_data_forwarding_link_selection = TRUE;		
		}
		else if(strcmp(key_buf, "data_forwarding_two_way_forwarding_flag") == 0)
		{
			param->data_forwarding_two_way_forwarding_flag = (boolean)value;		
			flag_data_forwarding_two_way_forwarding_flag = TRUE;		
		}
		else if(strcmp(key_buf, "data_forwarding_multiple_target_point_flag") == 0)
		{
			param->data_forwarding_multiple_target_point_flag = (boolean)value;		
			flag_data_forwarding_multiple_target_point_flag = TRUE;		
		}
		else if(strcmp(key_buf, "data_forwarding_maximum_target_point_number") == 0)
		{
			param->data_forwarding_maximum_target_point_number = (int)value;		
			flag_data_forwarding_maximum_target_point_number = TRUE;		
		}
		else if(strcmp(key_buf, "packet_delay_measurement_flag") == 0)
		{
			param->packet_delay_measurement_flag = (boolean) value;		
			flag_packet_delay_measurement_flag = TRUE;		
		}
		else if(strcmp(key_buf, "packet_delay_measurement_target_point") == 0)
		{
			param->packet_delay_measurement_target_point = (int) value;		
			flag_packet_delay_measurement_target_point = TRUE;		
		}
		else if(strcmp(key_buf, "packet_delay_measurement_time") == 0)
		{
			param->packet_delay_measurement_time = value;		
			flag_packet_delay_measurement_time = TRUE;		
		}
		else if(strcmp(key_buf, "target_point_interdistance") == 0)
		{
			param->target_point_interdistance = value;		
			flag_target_point_interdistance = TRUE;		
		}
		else if(strcmp(key_buf, "target_point_index") == 0)
		{
			param->target_point_index = (int) value;		
			flag_target_point_index = TRUE;		
		}
		else if(strcmp(key_buf, "communication_range") == 0)
		{
			param->communication_range = (int) value;		
			flag_communication_range = TRUE;		
		}
		else if(strcmp(key_buf, "communication_one_hop_delay") == 0)
		{
			param->communication_one_hop_delay = value;		
			flag_communication_one_hop_delay = TRUE;		
		}
		else if(strcmp(key_buf, "communication_data_packet_size") == 0)
		{
			param->communication_data_packet_size = (int) value;		
			flag_communication_data_packet_size = TRUE;		
		}
		else if(strcmp(key_buf, "communication_packet_ttl") == 0)
		{
			param->communication_packet_ttl = (int) value;		
			flag_communication_packet_ttl = TRUE;		
		}
		else if(strcmp(key_buf, "communication_packet_ttl_override_flag") == 0)
		{
			param->communication_packet_ttl_override_flag = (boolean)value;		
			flag_communication_packet_ttl_override_flag = TRUE;		
		}
		else if(strcmp(key_buf, "communication_packet_hop_limit") == 0)
		{
			param->communication_packet_hop_limit = (int) value;
			flag_communication_packet_hop_limit = TRUE;
		}
		else if(strcmp(key_buf, "communication_packet_interarrival_time") == 0)
		{
			param->communication_packet_interarrival_time = value;		
			flag_communication_packet_interarrival_time = TRUE;		
		}
		else if(strcmp(key_buf, "communication_packet_interarrival_time_distribution") == 0)
		{
			param->communication_packet_interarrival_time_distribution = (distribution_type_t)value;		
			flag_communication_packet_interarrival_time_distribution = TRUE;		
		}
		else if(strcmp(key_buf, "communication_packet_interarrival_time_standard_deviation") == 0)
		{
			param->communication_packet_interarrival_time_standard_deviation = value;		
			flag_communication_packet_interarrival_time_standard_deviation = TRUE;		
		}
		else if(strcmp(key_buf, "communication_packet_maximum_interarrival_time_factor") == 0)
		{
			param->communication_packet_maximum_interarrival_time_factor = value;		
			flag_communication_packet_maximum_interarrival_time_factor = TRUE;		
			if(flag_communication_packet_interarrival_time == FALSE)
			{
				printf("Error: communication_packet_interarrival_time must be set to determine param->communication_packet_maximum_interarrival_time along with communication_packet_maximum_interarrival_time_factor.\n");
				exit(1);
			}
			else
			{
				param->communication_packet_maximum_interarrival_time = param->communication_packet_maximum_interarrival_time_factor * param->communication_packet_interarrival_time;
			}
		}
		else if(strcmp(key_buf, "communication_packet_maximum_number") == 0)
		{
			param->communication_packet_maximum_number = (int) value;		
			flag_communication_packet_maximum_number = TRUE;		
		}
		else if(strcmp(key_buf, "communication_packet_delivery_probability_threshold") == 0)
		{
			param->communication_packet_delivery_probability_threshold = value;		
			flag_communication_packet_delivery_probability_threshold = TRUE;	
	
                        /* check the validity of the delivery probability threshold */
                        if((param->communication_packet_delivery_probability_threshold < 0) || (param->communication_packet_delivery_probability_threshold > 1))
                        {
                            printf("init_parameter(): param->communication_packet_delivery_probability_threshold(%.4f) must be between 0 and 1\n", (float)param->communication_packet_delivery_probability_threshold);
                            exit(1);
                        }
		}
		else if(strcmp(key_buf, "communication_packet_reverse_traversal_hop_distance_threshold") == 0)
		{
			param->communication_packet_reverse_traversal_hop_distance_threshold = (int) value;		
			flag_communication_packet_reverse_traversal_hop_distance_threshold = TRUE;		
		}
		else if(strcmp(key_buf, "communication_multiple_AP_flag") == 0)
		{
			param->communication_multiple_AP_flag = (boolean) value;		
			flag_communication_multiple_AP_flag = TRUE;		
		}
		else if(strcmp(key_buf, "communication_AP_maximum_number") == 0)
		{
			param->communication_AP_maximum_number = (int) value;		
			flag_communication_AP_maximum_number = TRUE;		
		}
		else if(strcmp(key_buf, "communication_multiple_SN_flag") == 0)
		{
			param->communication_multiple_SN_flag = (boolean) value;		
			flag_communication_multiple_SN_flag = TRUE;		
		}
		else if(strcmp(key_buf, "communication_SN_maximum_number") == 0)
		{
			param->communication_SN_maximum_number = (int) value;		
			flag_communication_SN_maximum_number = TRUE;		
		}
		else if(strcmp(key_buf, "communication_AP_packet_generation_schedule_time") == 0)
		{
			param->communication_AP_packet_generation_schedule_time = value;		
			flag_communication_AP_packet_generation_schedule_time = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_schedule_mode") == 0)
		{
			param->sensor_schedule_mode = (sensor_schedule_mode_type_t)value;		
			flag_sensor_schedule_mode = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_number") == 0)
		{
			param->sensor_number = (int) value;		
			flag_sensor_number = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_density") == 0)
		{
			param->sensor_density = value;		
			flag_sensor_density = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_density_distribution") == 0)
		{
			param->sensor_density_distribution = (distribution_type_t)value;		
			flag_sensor_density_distribution = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_density_standard_deviation") == 0)
		{
			param->sensor_density_standard_deviation = value;		
			flag_sensor_density_standard_deviation = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_density_maximum_deviation") == 0)
		{
			param->sensor_density_maximum_deviation = value;		
			flag_sensor_density_maximum_deviation = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_deployment_distribtuion") == 0)
		{
			param->sensor_deployment_distribtuion = (distribution_type_t)value;		
			flag_sensor_deployment_distribtuion = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_deployment_standard_deviation") == 0)
		{
			param->sensor_deployment_standard_deviation = value;		
			flag_sensor_deployment_standard_deviation = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_work_time") == 0)
		{
			param->sensor_work_time = value;		
			flag_sensor_work_time = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_think_time") == 0)
		{
			param->sensor_think_time = value;		
			flag_sensor_think_time = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_think_time_distribution") == 0)
		{
			param->sensor_think_time_distribution = (distribution_type_t)value;		
			flag_sensor_think_time_distribution = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_think_time_standard_deviation") == 0)
		{
			param->sensor_think_time_standard_deviation = value;		
			flag_sensor_think_time_standard_deviation = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_energy") == 0)
		{
			param->sensor_energy = value;		
			flag_sensor_energy = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_energy_distribution") == 0)
		{
			param->sensor_energy_distribution = (distribution_type_t)value;		
			flag_sensor_energy_distribution = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_energy_standard_deviation") == 0)
		{
			param->sensor_energy_standard_deviation = value;		
			flag_sensor_energy_standard_deviation = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_energy_maximum_deviation") == 0)
		{
			param->sensor_energy_maximum_deviation = value;		
			flag_sensor_energy_maximum_deviation = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_energy_consumption_rate") == 0)
		{
			param->sensor_energy_consumption_rate = value;		
			flag_sensor_energy_consumption_rate = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_warm_up_time") == 0)
		{
			param->sensor_warm_up_time = value;		
			flag_sensor_warm_up_time = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_turn_on_energy_consumption") == 0)
		{
			param->sensor_turn_on_energy_consumption = value;		
			flag_sensor_turn_on_energy_consumption = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_sensing_range") == 0)
		{
			/*
			if(flag_length_unit == FALSE)
			{
				printf("length_unit must be set to either UNIT_METER or UNIT_MILE at first!\n");
#ifdef __DEBUG_INTERACTIVE_MODE__
                                fgetc(stdin);
#endif
				exit(1);
			}
			*/
			
			param->sensor_sensing_range = value;		
			flag_sensor_sensing_range = TRUE;		
		}
		else if(strcmp(key_buf, "sensor_sensing_range_distribution") == 0)
		{
			param->sensor_sensing_range_distribution = (distribution_type_t)value;		
			flag_sensor_sensing_range_distribution = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_sensing_range_standard_deviation") == 0)
		{
			param->sensor_sensing_range_standard_deviation = value;		
			flag_sensor_sensing_range_standard_deviation = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_time_sync_max_error") == 0)
		{
			param->sensor_time_sync_max_error = value;		
			flag_sensor_time_sync_max_error = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_time_sync_error") == 0)
		{
			param->sensor_time_sync_error = value;		
			flag_sensor_time_sync_error = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_time_sync_error_start") == 0)
		{
			param->sensor_time_sync_error_start = value;		
			flag_sensor_time_sync_error_start = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_time_sync_error_end") == 0)
		{
			param->sensor_time_sync_error_end = value;		
			flag_sensor_time_sync_error_end = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_time_sync_error_step") == 0)
		{
			param->sensor_time_sync_error_step = value;		
			flag_sensor_time_sync_error_step = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_time_sync_error_distribution") == 0)
		{
			param->sensor_time_sync_error_distribution = (distribution_type_t)value;
			flag_sensor_time_sync_error_distribution = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_time_sync_error_standard_deviation") == 0)
		{
			param->sensor_time_sync_error_standard_deviation = value;		
			flag_sensor_time_sync_error_standard_deviation = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_time_sync_error_standard_deviation_start") == 0)
		{
			param->sensor_time_sync_error_standard_deviation_start = value;		
			flag_sensor_time_sync_error_standard_deviation_start = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_time_sync_error_standard_deviation_end") == 0)
		{
			param->sensor_time_sync_error_standard_deviation_end = value;		
			flag_sensor_time_sync_error_standard_deviation_end = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_time_sync_error_standard_deviation_step") == 0)
		{
			param->sensor_time_sync_error_standard_deviation_step = value;		
			flag_sensor_time_sync_error_standard_deviation_step = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_detection_missing_probability") == 0)
		{
			param->sensor_detection_missing_probability = value;		
			flag_sensor_detection_missing_probability = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_detection_missing_probability_start") == 0)
		{
			param->sensor_detection_missing_probability_start = value;		
			flag_sensor_detection_missing_probability_start = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_detection_missing_probability_end") == 0)
		{
			param->sensor_detection_missing_probability_end = value;		
			flag_sensor_detection_missing_probability_end = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_detection_missing_probability_step") == 0)
		{
			param->sensor_detection_missing_probability_step = value;		
			flag_sensor_detection_missing_probability_step = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_duplicate_detection_probability") == 0)
		{
			param->sensor_duplicate_detection_probability = value;		
			flag_sensor_duplicate_detection_probability = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_duplicate_detection_probability_start") == 0)
		{
			param->sensor_duplicate_detection_probability_start = value;		
			flag_sensor_duplicate_detection_probability_start = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_duplicate_detection_probability_end") == 0)
		{
			param->sensor_duplicate_detection_probability_end = value;		
			flag_sensor_duplicate_detection_probability_end = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_duplicate_detection_probability_step") == 0)
		{
			param->sensor_duplicate_detection_probability_step = value;		
			flag_sensor_duplicate_detection_probability_step = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_scan_type") == 0)
		{
			param->sensor_scan_type = (sensor_scan_type_t)value;		
			flag_sensor_scan_type = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_initial_hole_handling_algorithm") == 0)
		{
			param->sensor_initial_hole_handling_algorithm = (hole_handling_algorithm_t)value;		
			flag_sensor_initial_hole_handling_algorithm = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_hole_handling_algorithm") == 0)
		{
			param->sensor_hole_handling_algorithm = (hole_handling_algorithm_t)value;		
			flag_sensor_hole_handling_algorithm = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_hole_handling_mode") == 0)
		{
			param->sensor_hole_handling_mode = (hole_handling_mode_t)value;		
			flag_sensor_hole_handling_mode = TRUE;	
		}
		else if(strcmp(key_buf, "sensor_movement_time_percentage") == 0)
		{
			param->sensor_movement_time_percentage = value;		
			flag_sensor_movement_time_percentage = TRUE;	
		}
		else if(strcmp(key_buf, "network_width") == 0)
		{
			param->network_width = value;		
			flag_network_width = TRUE;		
		}
		else if(strcmp(key_buf, "network_height") == 0)
		{
			param->network_height = value;		
			flag_network_height = TRUE;		
		}
		else if(strcmp(key_buf, "network_gap") == 0)
		{
			param->network_gap = value;		
			flag_network_gap = TRUE;		
		}
		else if(strcmp(key_buf, "network_sensor_distribution") == 0)
		{
			param->network_sensor_distribution = (distribution_type_t)value;		
			flag_network_sensor_distribution = TRUE;	
		}
		else if(strcmp(key_buf, "network_sensor_standard_deviation") == 0)
		{
			param->network_sensor_standard_deviation = value;		
			flag_network_sensor_standard_deviation = TRUE;	
		}
		else if(strcmp(key_buf, "vehicle_vanet_target_vehicle_flag") == 0)
		{
			param->vehicle_vanet_target_vehicle_flag = (boolean) value;		
			flag_vehicle_vanet_target_vehicle_flag = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_vanet_stationary_vehicle_flag") == 0)
		{
			param->vehicle_vanet_stationary_vehicle_flag = (boolean) value;		
			flag_vehicle_vanet_stationary_vehicle_flag = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_vanet_acl_measurement_flag") == 0)
		{
			param->vehicle_vanet_acl_measurement_flag = (boolean) value;		
			flag_vehicle_vanet_acl_measurement_flag = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_vanet_vehicular_traffic_model") == 0)
		{
			param->vehicle_vanet_vehicular_traffic_model = (vanet_vehicular_traffic_model_type_t)value;		
			flag_vehicle_vanet_vehicular_traffic_model = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_vanet_edd_model") == 0)
		{
			param->vehicle_vanet_edd_model = (vanet_edd_model_type_t)value;		
			flag_vehicle_vanet_edd_model = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_vanet_edd_computation_model") == 0)
		{
			param->vehicle_vanet_edd_computation_model = (vanet_edd_computation_model_type_t)value;		
			flag_vehicle_vanet_edd_computation_model = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_vanet_tbd_edd_computation_type") == 0)
		{
			param->vehicle_vanet_tbd_edd_computation_type = (vanet_tbd_edd_computation_type_t)value;		
			flag_vehicle_vanet_tbd_edd_computation_type = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_vanet_edge_delay_model") == 0)
		{
			param->vehicle_vanet_edge_delay_model = (vanet_edge_delay_model_type_t)value;		
			flag_vehicle_vanet_edge_delay_model = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_vanet_forwarding_type") == 0)
		{
			param->vehicle_vanet_forwarding_type = (vanet_forwarding_type_t)value;		
			flag_vehicle_vanet_forwarding_type = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_vanet_intersection_forwarding_type") == 0)
		{
			param->vehicle_vanet_intersection_forwarding_type = (vanet_intersection_forwarding_type_t)value;		
			flag_vehicle_vanet_intersection_forwarding_type = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_vanet_target_point_selection_type") == 0)
		{
			param->vehicle_vanet_target_point_selection_type = (vanet_target_point_selection_type_t)value;		
			flag_vehicle_vanet_target_point_selection_type = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_vanet_target_point_computation_method") == 0)
		{
			param->vehicle_vanet_target_point_computation_method = (vanet_target_point_computation_method_t)value;		
			flag_vehicle_vanet_target_point_computation_method = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_vanet_target_point_search_space_type") == 0)
		{
			param->vehicle_vanet_target_point_search_space_type = (vanet_target_point_search_space_type_t)value;		
			flag_vehicle_vanet_target_point_search_space_type = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_vanet_target_point_recomputation_interval_denominator") == 0)
		{
			param->vehicle_vanet_target_point_recomputation_interval_denominator = value;		
			flag_vehicle_vanet_target_point_recomputation_interval_denominator = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_vanet_vehicle_trajectory_exposure_degree") == 0)
		{
			param->vehicle_vanet_vehicle_trajectory_exposure_degree = value;
			if(value < (1.0 - ERROR_TOLERANCE_FOR_REAL_ARITHMETIC))
			  param->vehicle_vanet_vehicle_trajectory_type = VANET_VEHICLE_TRAJECTORY_PARTIAL;
			else
			  param->vehicle_vanet_vehicle_trajectory_type = VANET_VEHICLE_TRAJECTORY_FULL;

			flag_vehicle_vanet_vehicle_trajectory_exposure_degree = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_vanet_vehicle_trajectory_length_type") == 0)
		{
			param->vehicle_vanet_vehicle_trajectory_length_type = (vanet_vehicle_trajectory_length_type_t)value;
			flag_vehicle_vanet_vehicle_trajectory_length_type = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_vanet_metric_type") == 0)
		{
			param->vehicle_vanet_metric_type = (vanet_metric_type_t)value;
			flag_vehicle_vanet_metric_type = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_vanet_target_point_optimization_function_type") == 0)
		{
			param->vehicle_vanet_target_point_optimization_function_type = (vanet_target_point_optimization_function_type_t)value;
			flag_vehicle_vanet_target_point_optimization_function_type = TRUE;
		}
		else if(strcmp(key_buf, "vehicle_maximum_number") == 0)
		{
			param->vehicle_maximum_number = (int) value;		
			flag_vehicle_maximum_number = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_packet_generating_entity_number") == 0)
		{
			param->vehicle_packet_generating_entity_number = (int) value;		
			flag_vehicle_packet_generating_entity_number = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_AP_passing_entity_percentage") == 0)
		{
			/* check the vality of the value */
			if((value < 0) || (value > 100))
			{
			  printf("init_parameter(): Error: vehicle_AP_passing_entity_percentage(%.2f) must be between 0 and 100\n", (float) value);
			  exit(1);
			}

			param->vehicle_AP_passing_entity_percentage = value;
			flag_vehicle_AP_passing_entity_percentage = TRUE;

			if(param->vehicle_maximum_number > 0)
			{
			  param->vehicle_AP_passing_entity_number = (int)ceil(param->vehicle_AP_passing_entity_percentage/100.0*param->vehicle_maximum_number);
			}
		}
		else if(strcmp(key_buf, "vehicle_packet_generation_schedule_time") == 0)
		{
			param->vehicle_packet_generation_schedule_time = value;		
			flag_vehicle_packet_generation_schedule_time = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_edd_update_period") == 0)
		{
			param->vehicle_edd_update_period = value;		
			flag_vehicle_edd_update_period = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_step_mode") == 0)
		{
			param->vehicle_step_mode = (vehicle_step_mode_type_t)value;		
			flag_vehicle_step_mode = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_step_time") == 0)
		{
			param->vehicle_step_time = value;		
			flag_vehicle_step_time = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_initial_arrival_time") == 0)
		{
			param->vehicle_initial_arrival_time = value;		
			flag_vehicle_initial_arrival_time = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_interarrival_time") == 0)
		{
			param->vehicle_interarrival_time = value;		
			flag_vehicle_interarrival_time = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_interarrival_time_start") == 0)
		{
			param->vehicle_interarrival_time_start = value;		
			flag_vehicle_interarrival_time_start = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_interarrival_time_end") == 0)
		{
			param->vehicle_interarrival_time_end = value;		
			flag_vehicle_interarrival_time_end = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_interarrival_time_step") == 0)
		{
			param->vehicle_interarrival_time_step = value;		
			flag_vehicle_interarrival_time_step = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_interarrival_time_distribution") == 0)
		{
			param->vehicle_interarrival_time_distribution = (distribution_type_t)value;		
			flag_vehicle_interarrival_time_distribution = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_interarrival_time_standard_deviation") == 0)
		{
			param->vehicle_interarrival_time_standard_deviation = value;		
			flag_vehicle_interarrival_time_standard_deviation = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_maximum_interarrival_time_factor") == 0)
		{
			param->vehicle_maximum_interarrival_time_factor = value;		
			flag_vehicle_maximum_interarrival_time_factor = TRUE;		
			if(flag_vehicle_interarrival_time == FALSE)
			{
				printf("Error: vehicle_interarrival_time must be set to determine param->vehicle_maximum_interarrival_time along with vehicle_maximum_interarrival_time_factor.\n");
				exit(1);
			}
			else
			{
				param->vehicle_maximum_interarrival_time = param->vehicle_maximum_interarrival_time_factor * param->vehicle_interarrival_time;
			}
		}
		else if(strcmp(key_buf, "vehicle_minimum_speed_in_mile_per_hour") == 0)
		{// 1mile = 1609m
			param->vehicle_minimum_speed_in_mile_per_hour = value;
			param->vehicle_minimum_speed_in_km_per_hour = convert_mile_per_hour_to_km_per_hour(value);			
			param->vehicle_minimum_speed = convert_mile_per_hour_to_meter_per_sec(value);
			flag_vehicle_minimum_speed_in_mile_per_hour = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_maximum_speed_in_mile_per_hour") == 0)
		{
			param->vehicle_maximum_speed_in_mile_per_hour = value;
			param->vehicle_maximum_speed_in_km_per_hour = convert_mile_per_hour_to_km_per_hour(value);			
			param->vehicle_maximum_speed = convert_mile_per_hour_to_meter_per_sec(value);
			flag_vehicle_maximum_speed_in_mile_per_hour = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_speed_in_mile_per_hour") == 0)
		{
			param->vehicle_speed_in_mile_per_hour = value;
			param->vehicle_speed_in_km_per_hour = convert_mile_per_hour_to_km_per_hour(value);			
			param->vehicle_speed = convert_mile_per_hour_to_meter_per_sec(value);
			flag_vehicle_speed_in_mile_per_hour = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_speed_distribution") == 0)
		{
			param->vehicle_speed_distribution = (distribution_type_t)value;		
			flag_vehicle_speed_distribution = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_speed_standard_deviation_in_mile_per_hour") == 0)
		{
			param->vehicle_speed_standard_deviation_in_mile_per_hour = value;
			param->vehicle_speed_standard_deviation_in_km_per_hour = convert_mile_per_hour_to_km_per_hour(value);			
			param->vehicle_speed_standard_deviation = convert_mile_per_hour_to_meter_per_sec(value);
			param->vehicle_speed_variance = pow(param->vehicle_speed_standard_deviation, 2);
			flag_vehicle_speed_standard_deviation_in_mile_per_hour = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_speed_bound_coefficient") == 0)
		{
			param->vehicle_speed_bound_coefficient = (int) value;
                        param->vehicle_speed_bound_coefficient_flag = TRUE;
			flag_vehicle_speed_bound_coefficient = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_think_time") == 0)
		{
			param->vehicle_think_time = value;

			/* set vehicle_mean_think_time to the average of vehicle_think_time */
			if(param->vehicle_think_time >= 0)
			{ /* Assume that the distribution of param->vehicle_think_time is UNIFORM. */
				param->vehicle_mean_think_time = param->vehicle_think_time/2;
				param->vehicle_think_time_variance = pow(param->vehicle_think_time, 2)/12;
			}
			else
			{
				printf("%s:%d param->vehicle_think_time(%.f) is negative\n",
						__FUNCTION__, __LINE__,
						param->vehicle_think_time);
				exit(1);
			}

			flag_vehicle_think_time = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_think_time_start") == 0)
		{
			param->vehicle_think_time_start = value;		
			flag_vehicle_think_time_start = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_think_time_end") == 0)
		{
			param->vehicle_think_time_end = value;		
			flag_vehicle_think_time_end = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_think_time_step") == 0)
		{
			param->vehicle_think_time_step = value;		
			flag_vehicle_think_time_step = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_think_time_distribution") == 0)
		{
			param->vehicle_think_time_distribution = (distribution_type_t)value;		
			flag_vehicle_think_time_distribution = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_think_time_standard_deviation") == 0)
		{
			param->vehicle_think_time_standard_deviation = value;		
			flag_vehicle_think_time_standard_deviation = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_path_length_distribution") == 0)
		{
			param->vehicle_path_length_distribution = (distribution_type_t)value;		
			flag_vehicle_path_length_distribution = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_path_length_standard_deviation_in_mile") == 0)
		{
		        param->vehicle_path_length_standard_deviation_in_mile = value;
		        param->vehicle_path_length_standard_deviation_in_km = convert_mile_to_km(value);
			param->vehicle_path_length_standard_deviation = convert_mile_to_meter(value);
			flag_vehicle_path_length_standard_deviation_in_mile = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_path_length_standard_deviation_start") == 0)
		{
			param->vehicle_path_length_standard_deviation_start = value;		
			flag_vehicle_path_length_standard_deviation_start = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_path_length_standard_deviation_end") == 0)
		{
			param->vehicle_path_length_standard_deviation_end = value;		
			flag_vehicle_path_length_standard_deviation_end = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_path_length_standard_deviation_step") == 0)
		{
			param->vehicle_path_length_standard_deviation_step = value;		
			flag_vehicle_path_length_standard_deviation_step = TRUE;		
		}
		else if(strcmp(key_buf, "vehicle_path_minimum_hop_count") == 0)
		{
		        param->vehicle_path_minimum_hop_count = (int)value;
		        flag_vehicle_path_minimum_hop_count = TRUE;
		}
		else if(strcmp(key_buf, "data_aggregation_type") == 0)
		{
			param->data_aggregation_type = (aggregation_type_t)value;		
			flag_data_aggregation_type = TRUE;
		}
		else if(strcmp(key_buf, "data_aggregation_window_size") == 0)
		{
			param->data_aggregation_window_size = (int)value;		
			flag_data_aggregation_window_size = TRUE;
		}
		else if(strcmp(key_buf, "data_aggregation_window_size_start") == 0)
		{
			param->data_aggregation_window_size_start = (int)value;		
			flag_data_aggregation_window_size_start = TRUE;
		}
		else if(strcmp(key_buf, "data_aggregation_window_size_end") == 0)
		{
			param->data_aggregation_window_size_end = (int)value;		
			flag_data_aggregation_window_size_end = TRUE;
		}
		else if(strcmp(key_buf, "data_aggregation_window_size_step") == 0)
		{
			param->data_aggregation_window_size_step = (int)value;		
			flag_data_aggregation_window_size_step = TRUE;
		}
		else if(strcmp(key_buf, "data_measurement_time") == 0)
		{
			param->data_measurement_time = value;		
			flag_data_measurement_time = TRUE;
		}
		else if(strcmp(key_buf, "data_number_of_split_measurement_times") == 0)
		{
			param->data_number_of_split_measurement_times = (int) value;		
			flag_data_number_of_split_measurement_times = TRUE;
		}
		else if(strcmp(key_buf, "data_prefilter_type") == 0)
		{
			param->data_prefilter_type = (prefilter_type_t)value;		
			flag_data_prefilter_type = TRUE;
		}
		else if(strcmp(key_buf, "simulation_seed") == 0)
		{
			param->simulation_seed = (simulation_seed_type_t)value;		
			flag_simulation_seed = TRUE;
		}
		else if(strcmp(key_buf, "simulation_mode") == 0)
		{
			param->simulation_mode = (simulation_mode_type_t)value;		
			flag_simulation_mode = TRUE;
		}
		else if(strcmp(key_buf, "simulation_time") == 0)
		{
			param->simulation_time = value;		
			flag_simulation_time = TRUE;
		}
		else if(strcmp(key_buf, "simulation_iteration_number") == 0)
		{
			param->simulation_iteration_number = (int) value;		
			flag_simulation_iteration_number = TRUE;
		}
		else if(strcmp(key_buf, "simulation_run") == 0)
		{
			param->simulation_run = (simulation_run_type_t)value;		
			flag_simulation_run = TRUE;
		}
	} //end of while

	/**  check if all parameters are set */
	if(flag_vanet_forwarding_scheme == FALSE)
	{       
		printf("Error: flag_vanet_forwarding_scheme is not set\n");
		exit(1);
	}   
	else if(flag_tpd_encounter_probability_threshold == FALSE)
	{
		printf("Error: flag_tpd_encounter_probability_threshold is not set\n");
		exit(1);
	}
	else if(flag_tpd_delivery_probability_threshold == FALSE)
	{
		printf("Error: flag_tpd_delivery_probability_threshold is not set\n");
		exit(1);
	}
	else if(flag_tpd_encounter_graph_optimization_flag == FALSE)
	{
		printf("Error: flag_tpd_encounter_graph_optimization_flag is not set\n");
		exit(1);
	}
	else if(flag_tpd_encounter_graph_source_routing_flag == FALSE)
	{
		printf("Error: flag_tpd_encounter_graph_source_routing_flag is not set\n");
		exit(1);
	}
	else if(flag_forwarding_probability_and_statistics_flag == FALSE)
	{
		printf("Error: flag_forwarding_probability_and_statistics_flag is not set\n");
		exit(1);
	}
	else if(flag_evaluation_type == FALSE)
	{
		printf("Error: flag_evaluation_type is not set\n");
		exit(1);
	}
	else if(flag_comparison_target_type == FALSE)
	{
		printf("Error: flag_comparison_target_type is not set\n");
		exit(1);
	}
	else if(flag_length_unit == FALSE)
	{
		printf("Error: flag_length_unit is not set\n");
		exit(1);
	}
	else if(flag_graph_file_name == FALSE)
	{
		printf("Error: graph_file_name is not set\n");
		exit(1);
	}
	else if(flag_graph_node_number == FALSE)
	{
		printf("Error: graph_node_number is not set\n");
		exit(1);
	}
	else if(flag_data_forwarding_mode == FALSE)
	{
		printf("Error: data_forwarding_mode is not set\n");
		exit(1);
	}
	else if(flag_data_forwarding_link_selection == FALSE)
	{
		printf("Error: data_forwarding_link_selection is not set\n");
		exit(1);
	}
	else if(flag_data_forwarding_two_way_forwarding_flag == FALSE)
	{
		printf("Error: data_forwarding_two_way_forwarding_flag is not set\n");
		exit(1);
	}
	else if(flag_data_forwarding_multiple_target_point_flag == FALSE)
	{
		printf("Error: data_forwarding_multiple_target_point_flag is not set\n");
		exit(1);
	}
	else if(flag_data_forwarding_maximum_target_point_number == FALSE)
	{
		printf("Error: data_forwarding_maximum_target_point_number is not set\n");
		exit(1);
	}
	else if(flag_packet_delay_measurement_flag == FALSE)
	{
		printf("Error: packet_delay_measurement_flag is not set\n");
		exit(1);
	}
	else if(flag_packet_delay_measurement_target_point == FALSE)
	{
		printf("Error: packet_delay_measurement_target_point is not set\n");
		exit(1);
	}
	else if(flag_packet_delay_measurement_time == FALSE)
	{
		printf("Error: packet_delay_measurement_time is not set\n");
		exit(1);
	}
	else if(flag_target_point_interdistance == FALSE)
	{
		printf("Error: target_point_interdistance is not set\n");
		exit(1);
	}
	else if(flag_target_point_index == FALSE)
	{
		printf("Error: target_point_index is not set\n");
		exit(1);
	}
	else if(flag_communication_range == FALSE)
	{
		printf("Error: communication_range is not set\n");
		exit(1);
	}
	else if(flag_communication_one_hop_delay == FALSE)
	{
		printf("Error: communication_one_hop_delay is not set\n");
		exit(1);
	}
	else if(flag_communication_packet_interarrival_time == FALSE)
	{
		printf("Error: communication_packet_interarrival_time is not set\n");
		exit(1);
	}
	else if(flag_communication_data_packet_size == FALSE)
	{
		printf("Error: communication_data_packet_size is not set\n");
		exit(1);
	}
	else if(flag_communication_packet_ttl == FALSE)
	{
		printf("Error: communication_packet_ttl is not set\n");
		exit(1);
	}
	else if(flag_communication_packet_ttl_override_flag == FALSE)
	{
		printf("Error: communication_packet_ttl_override_flag is not set\n");
		exit(1);
	}
	else if(flag_communication_packet_hop_limit == FALSE)
	{
		printf("Error: communication_packet_hop_limit is not set\n");
		exit(1);
	}
	else if(flag_communication_packet_maximum_number == FALSE)
	{
		printf("Error: communication_packet_maximum_number is not set\n");
		exit(1);
	}
	else if(flag_communication_packet_delivery_probability_threshold == FALSE)
	{
		printf("Error: communication_packet_delivery_probability_threshold is not set\n");
		exit(1);
	}
	else if(flag_communication_packet_reverse_traversal_hop_distance_threshold == FALSE)
	{
		printf("Error: communication_packet_reverse_traversal_hop_distance_threshold is not set\n");
		exit(1);
	}
	else if(flag_communication_multiple_AP_flag == FALSE)
	{
		printf("Error: communication_multiple_AP_flag is not set\n");
		exit(1);
	}
	else if(flag_communication_AP_maximum_number == FALSE)
	{
		printf("Error: communication_AP_maximum_number is not set\n");
		exit(1);
	}
	else if(flag_communication_multiple_SN_flag == FALSE)
	{
		printf("Error: communication_multiple_SN_flag is not set\n");
		exit(1);
	}
	else if(flag_communication_SN_maximum_number == FALSE)
	{
		printf("Error: communication_SN_maximum_number is not set\n");
		exit(1);
	}
	else if(flag_communication_AP_packet_generation_schedule_time == FALSE)
	{
		printf("Error: communication_AP_packet_generation_schedule_time is not set\n");
		exit(1);
	}
	else if(flag_sensor_schedule_mode == FALSE)
	{
		printf("Error: sensor_schedule_mode is not set\n");
		exit(1);
	}
	else if(flag_sensor_number == FALSE)
	{
		printf("Error: sensor_number is not set\n");
		exit(1);
	}
	else if(flag_sensor_density == FALSE)
	{
		printf("Error: sensor_density is not set\n");
		exit(1);
	}
	else if(flag_sensor_density_distribution == FALSE)
	{
		printf("Error: sensor_density_distribution is not set\n");
		exit(1);
	}
	else if(flag_sensor_density_standard_deviation == FALSE)
	{
		printf("Error: sensor_density_standard_deviation is not set\n");
		exit(1);
	}
	else if(flag_sensor_density_maximum_deviation == FALSE)
	{
		printf("Error: sensor_density_maximum_deviation is not set\n");
		exit(1);
	}
	else if(flag_sensor_deployment_distribtuion == FALSE)
	{
		printf("Error: sensor_deployment_distribtuion is not set\n");
		exit(1);
	}
	else if(flag_sensor_deployment_standard_deviation == FALSE)
	{
		printf("Error: sensor_deployment_standard_deviation is not set\n");
		exit(1);
	}
	else if(flag_sensor_work_time == FALSE)
	{
		printf("Error: sensor_work_time is not set\n");
		exit(1);
	}
	else if(flag_sensor_think_time == FALSE)
	{
		printf("Error: sensor_think_time is not set\n");
		exit(1);
	}
	else if(flag_sensor_think_time_distribution == FALSE)
	{
		printf("Error: sensor_think_time_distribution is not set\n");
		exit(1);
	}
	else if(flag_sensor_think_time_standard_deviation == FALSE)
	{
		printf("Error: sensor_think_time_standard_deviation is not set\n");
		exit(1);
	}
	else if(flag_sensor_energy == FALSE)
	{
		printf("Error: sensor_energy is not set\n");
		exit(1);
	}
	else if(flag_sensor_energy_distribution == FALSE)
	{
		printf("Error: sensor_energy_distribution is not set\n");
		exit(1);
	}
	else if(flag_sensor_energy_standard_deviation == FALSE)
	{
		printf("Error: sensor_energy_standard_deviation is not set\n");
		exit(1);
	}
	else if(flag_sensor_energy_maximum_deviation == FALSE)
	{
		printf("Error: sensor_energy_maximum_deviation is not set\n");
		exit(1);
	}
	else if(flag_sensor_energy_consumption_rate == FALSE)
	{
		printf("Error: sensor_energy_consumption_rate is not set\n");
		exit(1);
	}
	else if(flag_sensor_warm_up_time == FALSE)
	{
		printf("Error: sensor_warm_up_time is not set\n");
		exit(1);
	}
	else if(flag_sensor_turn_on_energy_consumption == FALSE)
	{
		printf("Error: sensor_turn_on_energy_consumption is not set\n");
		exit(1);
	}
	else if(flag_sensor_sensing_range == FALSE)
	{
		printf("Error: sensor_sensing_range is not set\n");
		exit(1);
	}
	else if(flag_sensor_sensing_range_distribution == FALSE)
	{
		printf("Error: sensor_sensing_range_distribution is not set\n");
		exit(1);
	}
	else if(flag_sensor_sensing_range_standard_deviation == FALSE)
	{
		printf("Error: sensor_sensing_range_standard_deviation is not set\n");
		exit(1);
	}
	else if(flag_sensor_time_sync_max_error == FALSE)
	{
		printf("Error: sensor_time_sync_max_error is not set\n");
		exit(1);
	}
	else if(flag_sensor_time_sync_error == FALSE)
	{
		printf("Error: sensor_time_sync_error is not set\n");
		exit(1);
	}
	else if(flag_sensor_time_sync_error_start == FALSE)
	{
		printf("Error: sensor_time_sync_error_start is not set\n");
		exit(1);
	}
	else if(flag_sensor_time_sync_error_end == FALSE)
	{
		printf("Error: sensor_time_sync_error_end is not set\n");
		exit(1);
	}
	else if(flag_sensor_time_sync_error_step == FALSE)
	{
		printf("Error: sensor_time_sync_error_step is not set\n");
		exit(1);
	}
	else if(flag_sensor_time_sync_error_distribution == FALSE)
	{
		printf("Error: sensor_time_sync_error_distribution is not set\n");
		exit(1);
	}
	else if(flag_sensor_time_sync_error_standard_deviation == FALSE)
	{
		printf("Error: sensor_time_sync_error_standard_deviation is not set\n");
		exit(1);
	}
	else if(flag_sensor_time_sync_error_standard_deviation_start == FALSE)
	{
		printf("Error: sensor_time_sync_error_standard_deviation_start is not set\n");
		exit(1);
	}
	else if(flag_sensor_time_sync_error_standard_deviation_end == FALSE)
	{
		printf("Error: sensor_time_sync_error_standard_deviation_end is not set\n");
		exit(1);
	}
	else if(flag_sensor_time_sync_error_standard_deviation_step == FALSE)
	{
		printf("Error: sensor_time_sync_error_standard_deviation_step is not set\n");
		exit(1);
	}
	else if(flag_sensor_detection_missing_probability == FALSE)
	{
		printf("Error: sensor_detection_missing_probability is not set\n");
		exit(1);
	}
	else if(flag_sensor_detection_missing_probability_start == FALSE)
	{
		printf("Error: sensor_detection_missing_probability_start is not set\n");
		exit(1);
	}
	else if(flag_sensor_detection_missing_probability_end == FALSE)
	{
		printf("Error: sensor_detection_missing_probability_end is not set\n");
		exit(1);
	}
	else if(flag_sensor_detection_missing_probability_step == FALSE)
	{
		printf("Error: sensor_detection_missing_probability_step is not set\n");
		exit(1);
	}
	else if(flag_sensor_duplicate_detection_probability == FALSE)
	{
		printf("Error: sensor_duplicate_detection_probability is not set\n");
		exit(1);
	}
	else if(flag_sensor_duplicate_detection_probability_start == FALSE)
	{
		printf("Error: sensor_duplicate_detection_probability_start is not set\n");
		exit(1);
	}
	else if(flag_sensor_duplicate_detection_probability_end == FALSE)
	{
		printf("Error: sensor_duplicate_detection_probability_end is not set\n");
		exit(1);
	}
	else if(flag_sensor_duplicate_detection_probability_step == FALSE)
	{
		printf("Error: sensor_duplicate_detection_probability_step is not set\n");
		exit(1);
	}
	else if(flag_sensor_scan_type == FALSE)
	{
		printf("Error: sensor_scan_type is not set\n");
		exit(1);
	}
	else if(flag_sensor_initial_hole_handling_algorithm == FALSE)
	{
		printf("Error: sensor_initial_hole_handling_algorithm is not set\n");
		exit(1);
	}
	else if(flag_sensor_hole_handling_algorithm == FALSE)
	{
		printf("Error: sensor_hole_handling_algorithm is not set\n");
		exit(1);
	}
	else if(flag_sensor_hole_handling_mode == FALSE)
	{
		printf("Error: sensor_hole_handling_mode is not set\n");
		exit(1);
	}
	else if(flag_sensor_movement_time_percentage == FALSE)
	{
		printf("Error: sensor_movement_time_percentage is not set\n");
		exit(1);
	}
	else if(flag_network_width == FALSE)
	{
		printf("Error: network_width is not set\n");
		exit(1);
	}
	else if(flag_network_height == FALSE)
	{
		printf("Error: network_height is not set\n");
		exit(1);
	}
	else if(flag_network_gap == FALSE)
	{
		printf("Error: network_gap is not set\n");
		exit(1);
	}
	else if(flag_network_sensor_distribution == FALSE)
	{
		printf("Error: network_sensor_distribution is not set\n");
		exit(1);
	}
	else if(flag_network_sensor_standard_deviation == FALSE)
	{
		printf("Error: network_sensor_standard_deviation is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_target_vehicle_flag == FALSE)
	{
		printf("Error: vehicle_vanet_target_vehicle_flag is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_stationary_vehicle_flag == FALSE)
	{
		printf("Error: vehicle_vanet_stationary_vehicle_flag is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_acl_measurement_flag == FALSE)
	{
		printf("Error: vehicle_vanet_acl_measurement_flag is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_vehicular_traffic_model == FALSE)
	{
		printf("Error: vehicle_vanet_vehicular_traffic_model is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_edd_model == FALSE)
	{
		printf("Error: vehicle_vanet_edd_model is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_edd_computation_model == FALSE)
	{
		printf("Error: vehicle_vanet_edd_computation_model is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_tbd_edd_computation_type == FALSE)
	{
		printf("Error: vehicle_vanet_tbd_edd_computation_type is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_edge_delay_model == FALSE)
	{
		printf("Error: vehicle_vanet_edge_delay_model is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_forwarding_type == FALSE)
	{
		printf("Error: vehicle_vanet_forwarding_type is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_intersection_forwarding_type == FALSE)
	{
		printf("Error: vehicle_vanet_intersection_forwarding_type is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_target_point_selection_type == FALSE)
	{
		printf("Error: vehicle_vanet_target_point_selection_type is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_target_point_computation_method == FALSE)
	{
		printf("Error: vehicle_vanet_target_point_computation_method is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_target_point_search_space_type == FALSE)
	{
		printf("Error: vehicle_vanet_target_point_search_space_type is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_target_point_recomputation_interval_denominator == FALSE)
	{
		printf("Error: vehicle_vanet_target_point_recomputation_interval_denominator is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_vehicle_trajectory_exposure_degree == FALSE)
	{
		printf("Error: vehicle_vanet_vehicle_trajectory_exposure_degree is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_vehicle_trajectory_length_type == FALSE)
	{
		printf("Error: vehicle_vanet_vehicle_trajectory_length_type is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_metric_type == FALSE)
	{
		printf("Error: vehicle_vanet_metric_type is not set\n");
		exit(1);
	}
	else if(flag_vehicle_vanet_target_point_optimization_function_type == FALSE)
	{
		printf("Error: vehicle_vanet_target_point_optimization_function_type is not set\n");
		exit(1);
	}
	else if(flag_vehicle_maximum_number == FALSE)
	{
		printf("Error: vehicle_maximum_number is not set\n");
		exit(1);
	}
	else if(flag_vehicle_packet_generating_entity_number == FALSE)
	{
		printf("Error: vehicle_packet_generating_entity_number is not set\n");
		exit(1);
	}
	else if(flag_vehicle_packet_generation_schedule_time == FALSE)
	{
		printf("Error: vehicle_packet_generation_schedule_time is not set\n");
		exit(1);
	}
	else if(flag_vehicle_edd_update_period == FALSE)
	{
		printf("Error: vehicle_edd_update_period is not set\n");
		exit(1);
	}
	else if(flag_vehicle_step_mode == FALSE)
	{
		printf("Error: vehicle_step_mode is not set\n");
		exit(1);
	}
	else if(flag_vehicle_step_time == FALSE)
	{
		printf("Error: vehicle_step_time is not set\n");
		exit(1);
	}
	else if(flag_vehicle_initial_arrival_time == FALSE)
	{
		printf("Error: vehicle_initial_arrival_time is not set\n");
		exit(1);
	}
	else if(flag_vehicle_interarrival_time == FALSE)
	{
		printf("Error: vehicle_interarrival_time is not set\n");
		exit(1);
	}
	else if(flag_vehicle_interarrival_time_start == FALSE)
	{
		printf("Error: vehicle_interarrival_time_start is not set\n");
		exit(1);
	}
	else if(flag_vehicle_interarrival_time_end == FALSE)
	{
		printf("Error: vehicle_interarrival_time_end is not set\n");
		exit(1);
	}
	else if(flag_vehicle_interarrival_time_step == FALSE)
	{
		printf("Error: vehicle_interarrival_time_step is not set\n");
		exit(1);
	}
	else if(flag_vehicle_interarrival_time_distribution == FALSE)
	{
		printf("Error: vehicle_interarrival_time_distribution is not set\n");
		exit(1);
	}
	else if(flag_vehicle_interarrival_time_standard_deviation == FALSE)
	{
		printf("Error: vehicle_interarrival_time_standard_deviation is not set\n");
		exit(1);
	}
	else if(flag_vehicle_maximum_interarrival_time_factor == FALSE)
	{
		printf("Error: vehicle_maximum_interarrival_time_factor is not set\n");
		exit(1);
	}
	else if(flag_vehicle_minimum_speed_in_mile_per_hour == FALSE)
	{
		printf("Error: vehicle_minimum_speed_in_mile_per_hour is not set\n");
		exit(1);
	}
	else if(flag_vehicle_maximum_speed_in_mile_per_hour == FALSE)
	{
		printf("Error: vehicle_maximum_speed_in_mile_per_hour is not set\n");
		exit(1);
	}
	else if(flag_vehicle_speed_in_mile_per_hour == FALSE)
	{
		printf("Error: vehicle_speed_in_mile_per_hour is not set\n");
		exit(1);
	}
	else if(flag_vehicle_speed_distribution == FALSE)
	{
		printf("Error: vehicle_speed_distribution is not set\n");
		exit(1);
	}
	else if(flag_vehicle_speed_standard_deviation_in_mile_per_hour == FALSE)
	{
		printf("Error: vehicle_speed_standard_deviation_in_mile_per_hour is not set\n");
		exit(1);
	}
	/*
	else if(flag_vehicle_speed_bound_coefficient == FALSE)
	{
		printf("Error: vehicle_speed_bound_coefficient is not set\n");
		exit(1);
	}
	*/
	else if(flag_vehicle_think_time == FALSE)
	{
		printf("Error: vehicle_think_time is not set\n");
		exit(1);
	}
	else if(flag_vehicle_think_time_start == FALSE)
	{
		printf("Error: vehicle_think_time_start is not set\n");
		exit(1);
	}
	else if(flag_vehicle_think_time_end == FALSE)
	{
		printf("Error: vehicle_think_time_end is not set\n");
		exit(1);
	}
	else if(flag_vehicle_think_time_step == FALSE)
	{
		printf("Error: vehicle_think_time_step is not set\n");
		exit(1);
	}
	else if(flag_vehicle_think_time_distribution == FALSE)
	{
		printf("Error: vehicle_think_time_distribution is not set\n");
		exit(1);
	}
	else if(flag_vehicle_think_time_standard_deviation == FALSE)
	{
		printf("Error: vehicle_think_time_standard_deviation is not set\n");
		exit(1);
	}
	else if(flag_vehicle_path_length_distribution == FALSE)
	{
		printf("Error: vehicle_path_length_distribution is not set\n");
		exit(1);
	}
	else if(flag_vehicle_path_length_standard_deviation_in_mile == FALSE)
	{
		printf("Error: vehicle_path_length_standard_deviation_in_mile is not set\n");
		exit(1);
	}
	else if(flag_vehicle_path_length_standard_deviation_start == FALSE)
	{
		printf("Error: vehicle_path_length_standard_deviation_start is not set\n");
		exit(1);
	}
	else if(flag_vehicle_path_length_standard_deviation_end == FALSE)
	{
		printf("Error: vehicle_path_length_standard_deviation_end is not set\n");
		exit(1);
	}
	else if(flag_vehicle_path_length_standard_deviation_step == FALSE)
	{
		printf("Error: vehicle_path_length_standard_deviation_step is not set\n");
		exit(1);
	}
	else if(flag_vehicle_path_minimum_hop_count == FALSE)
	{
		printf("Error: vehicle_path_minimum_hop_count is not set\n");
		exit(1);
	}
	else if(flag_data_aggregation_type == FALSE)
	{
		printf("Error: data_aggregation_type is not set\n");
		exit(1);
	}
	else if(flag_data_aggregation_window_size == FALSE)
	{
		printf("Error: data_aggregation_window_size is not set\n");
		exit(1);
	}
	else if(flag_data_aggregation_window_size_start == FALSE)
	{
		printf("Error: data_aggregation_window_size_start is not set\n");
		exit(1);
	}
	else if(flag_data_aggregation_window_size_end == FALSE)
	{
		printf("Error: data_aggregation_window_size_end is not set\n");
		exit(1);
	}
	else if(flag_data_aggregation_window_size_step == FALSE)
	{
		printf("Error: data_aggregation_window_size_step is not set\n");
		exit(1);
	}
	else if(flag_data_measurement_time == FALSE)
	{
		printf("Error: data_measurement_time is not set\n");
		exit(1);
	}
	else if(flag_data_number_of_split_measurement_times == FALSE)
	{
		printf("Error: data_number_of_split_measurement_times is not set\n");
		exit(1);
	}
	else if(flag_data_prefilter_type == FALSE)
	{
		printf("Error: data_prefilter_type is not set\n");
		exit(1);
	}
	else if(flag_simulation_seed == FALSE)
	{
		printf("Error: simulation_seed is not set\n");
		exit(1);
	}
	else if(flag_simulation_mode == FALSE)
	{
		printf("Error: simulation_mode is not set\n");
		exit(1);
	}
	else if(param->simulation_mode == TIME)
	{
		if(flag_simulation_time == FALSE)
		{
			printf("Error: cross_traffic_interval is not set\n");
			exit(1);
		}
	}
	else if(param->simulation_mode == ITERATION)
	{
		if(flag_simulation_iteration_number == FALSE)
		{
			printf("Error: cross_traffic_interval is not set\n");
			exit(1);
		}
	}
	else if(flag_simulation_run == FALSE)
	{
		printf("Error: simulation_run is not set\n");
		exit(1);
	}

	/** set the vehicle min and max speed deviations */
	if(param->vehicle_speed_bound_coefficient_flag)
	{
		update_vehicle_maximum_and_minimum_speeds(param); //update the vehicle maximum and minimum speed bounds for the speed generation
	}

	/** set up the variables for vehicle travel time statistics for the unit length of 1 meter */
	param->vehicle_unit_length = 1; //1-meter unit length
        
	/* compute the mean and standard deviation of the travel time for the unit length */
	GSL_Vanet_Compute_TravelTime_And_Deviation(param, param->vehicle_unit_length, &(param->vehicle_unit_length_mean_travel_time), &(param->vehicle_unit_length_travel_time_standard_deviation));

	param->vehicle_unit_length_travel_time_variance = pow(param->vehicle_unit_length_travel_time_standard_deviation, 2); //compute the variance of the travel time for the unit length

	/* I forgot to write "fclose()" here. Due to the omission of fclose(), 
	I cannot open a new file, since file handles are exhausted. */
	fclose(fp);

	/*
	if(input_buf) //free the memory allocated by getline()
		free(input_buf);	
	*/
} //end of init_parameter()

/** functions related to parameter handling */
void create_vanet_information_table_in_parameter(parameter_t *param, struct_graph_node *Gr, int Gr_size, double **Dr_move, int **Mr_move, int matrix_size_for_movement_in_Gr, int **Ar_edd, double **Dr_edd, int **Mr_edd, double **Sr_edd, int matrix_size_for_edd_in_Gr, double **Wr_edc, double **Dr_edc, int **Mr_edc, double **Sr_edc, int matrix_size_for_edc_in_Gr, struct _forwarding_table_queue_t *FTQ)
{ //create vanet information table containing the data structures used in the data forwarding in the VANET, such as road network graph Gr, the movement shortest path matrices, the EDD shortest path matrices, the EDC shortest path matrices, the forwarding table queue FTQ, etc.
  /*@ Note: Whenever a new data structure is added to vanet information table, the memory for the data structure must be destroyed in destroy_vanet_information_table_in_parameter() */

    /** road network graph */
    param->vanet_table.Gr = Gr;
    param->vanet_table.Gr_size = Gr_size;

    /** the movement shortest path matrices */
    param->vanet_table.Dr_move = Dr_move;
    param->vanet_table.Mr_move = Mr_move;
    param->vanet_table.matrix_size_for_movement_in_Gr = matrix_size_for_movement_in_Gr;

    /** the EDD shortest path matrices */
    param->vanet_table.Ar_edd = Ar_edd;
    param->vanet_table.Dr_edd = Dr_edd;
    param->vanet_table.Mr_edd = Mr_edd;
    param->vanet_table.Sr_edd = Sr_edd;
    param->vanet_table.matrix_size_for_edd_in_Gr = matrix_size_for_edd_in_Gr;

    /** the EDC shortest path matrices */
    param->vanet_table.Wr_edc = Wr_edc;
    param->vanet_table.Dr_edc = Dr_edc;
    param->vanet_table.Mr_edc = Mr_edc;
    param->vanet_table.Sr_edc = Sr_edc;
    param->vanet_table.matrix_size_for_edc_in_Gr = matrix_size_for_edc_in_Gr;

    /** forwarding table queue */
    param->vanet_table.FTQ = FTQ; //pointer to the forwarding table queue
}

void destroy_vanet_information_table_in_parameter(parameter_t *param)
{ //destroy the data structures in vanet information table in param, such as road network graph Gr, the movement shortest path matrices, the EDD shortest path matricesm, the EDC shortest path matrices, the forwarding table queue FTQ, etc.; Note that these data structures can be reallocated memory, so the original pointers may not point to the actual memory.

  /* release the memory allocated to real graph Gr */
  Free_Graph(param->vanet_table.Gr, param->vanet_table.Gr_size); 
  param->vanet_table.Gr = NULL; 
  param->vanet_table.Gr_size = 0;

  /* free matrices for movement in Gr */
  Floyd_Warshall_Free_Matrices_For_Movement(&(param->vanet_table.Dr_move), &(param->vanet_table.Mr_move), &(param->vanet_table.matrix_size_for_movement_in_Gr));

  /* free matrices for EDD in Gr */
  Floyd_Warshall_Free_Matrix_Of_Type_Int(param->vanet_table.Ar_edd, param->vanet_table.matrix_size_for_edd_in_Gr);
  param->vanet_table.Ar_edd = NULL;

  Floyd_Warshall_Free_Matrices_For_EDD(&(param->vanet_table.Dr_edd), &(param->vanet_table.Mr_edd), &(param->vanet_table.Sr_edd), &(param->vanet_table.matrix_size_for_edd_in_Gr));

  /* free matrices for EDC in Gr */
  Floyd_Warshall_Free_Matrices_For_EDC(&(param->vanet_table.Wr_edc), &(param->vanet_table.Dr_edc), &(param->vanet_table.Mr_edc), &(param->vanet_table.Sr_edc), &(param->vanet_table.matrix_size_for_edc_in_Gr));

  /* destory forwarding table queue FTQ */
  DestroyQueue((queue_t*)param->vanet_table.FTQ);
}

void update_vehicle_maximum_and_minimum_speeds(parameter_t *param)
{ //update the vehicle maximum and minimum speed bounds for the speed generation

  /* min speed bound */
  param->vehicle_minimum_speed_in_mile_per_hour = param->vehicle_speed_in_mile_per_hour - (param->vehicle_speed_standard_deviation_in_mile_per_hour * param->vehicle_speed_bound_coefficient);
  if(param->vehicle_minimum_speed_in_mile_per_hour <= 0)
  {
    printf("update_vehicle_maximum_and_minimum_speeds(): error: param->vehicle_minimum_speed_in_mile_per_hour(%.3f) must be greater than zero\n", param->vehicle_minimum_speed_in_mile_per_hour);
    exit(1);
  }
  param->vehicle_minimum_speed_in_km_per_hour = convert_mile_per_hour_to_km_per_hour(param->vehicle_minimum_speed_in_mile_per_hour);
  param->vehicle_minimum_speed = convert_mile_per_hour_to_meter_per_sec(param->vehicle_minimum_speed_in_mile_per_hour);
	
  /* max speed bound */
  param->vehicle_maximum_speed_in_mile_per_hour = param->vehicle_speed_in_mile_per_hour + (param->vehicle_speed_standard_deviation_in_mile_per_hour * param->vehicle_speed_bound_coefficient);
  param->vehicle_maximum_speed_in_km_per_hour = convert_mile_per_hour_to_km_per_hour(param->vehicle_maximum_speed_in_mile_per_hour);
  param->vehicle_maximum_speed = convert_mile_per_hour_to_meter_per_sec(param->vehicle_maximum_speed_in_mile_per_hour);
}

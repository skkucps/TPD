/**
	fILE: main.c
	Description: This simulation model is for (i) Travel Prediction-based Data Forwarding (TPD) for Vehicle-to-Vehicle Data Delivery, (ii) Trajectory-based Statistical Forwarding (TSF) for Infrastructure-to-Vehicle Data Delivery, and (iii) Trajectory-Based Data Forwarding (TBD) for Vehicle-to-Infrastructure Data Delivery.
	Update Date: 07/13/2013
	Maker: Jaehoon (Paul) Jeong, pauljeong@skku.edu
	Memo:
	1. the variable of double type should be set to point number, e.g., 0.0 
	2. this version is run in both Visual C++.NET and Linux.
	3. [07/14/2010] this version supports the data forwarding in the middle of road segments among vehicles moving in the opposite directions.

*/
 
#include "stdafx.h"
#include "main.h"
#include "util.h"
#include "shortest-path.h"
#include "all-pairs-shortest-paths.h"
#include "random-path.h"
#include "schedule.h"
#include "vadd.h"
#include "linear-algebra.h"
#include "access-point-model.h"
#include "mobility.h"
#include "tpd.h"
#include "epidemic.h"
#include "steiner-tree.h"

#include <time.h>

//#define BSMA_20150227 0

#ifdef __GSL_LIBRARY_SUPPORT__
#include "gsl-util.h" //gsl utility funtions
#endif

#ifdef _LINUX_
#include <unistd.h> //getopt(), opterr
#endif

#include <stdlib.h> //srand48()

/* taehwan 20140723 */
#include <fcntl.h>
#include <sys/stat.h>

/* taehwan 20140719 */
#define VEHICLE_COUNT_MAX 3000
#define INTERSECTION_COUNT 50
int g_vehicle_current_segment_location[VEHICLE_COUNT_MAX]; // [vehicle_id]{intersection_head}
int g_vehicle_color[VEHICLE_COUNT_MAX];
int g_segment_visit_count[INTERSECTION_COUNT][4];
int g_intersection_visit_count[INTERSECTION_COUNT]={0,};
double g_current_time;
int g_gnuplot_option=0;
int g_color_flag = 1;

void toggle_forward_color()
{
	if (g_color_flag == 1)
		g_color_flag = 2;
	else
		g_color_flag = 1;
}

int* get_intersection_visit_count()
{
	int i,j;
	for(i=1;i<INTERSECTION_COUNT;i++)
	{
		g_intersection_visit_count[i] = 0;
		for(j=0;j<4;j++)
		{
		
			g_intersection_visit_count[i] += g_segment_visit_count[i][j];
		}
	}
	return g_intersection_visit_count;
}

FILE* g_gnuplotpipe = NULL;


void print_intersection_visit_count()
{
	int* visit_count = NULL;
	visit_count = get_intersection_visit_count();
	
	if (visit_count == NULL)
		return;

	int i;
	for(i=1;i<INTERSECTION_COUNT;i++)
	{
		printf("%7d ",visit_count[i]);
		if ( i%7 == 0)
			printf("\n");
	}

	fprintf(g_gnuplotpipe,"set output\n");

	pclose(g_gnuplotpipe);
}

int g_write_interval = 500;
int g_write_count = 0;


/* taehwan 20140723 */
void gnuplot_intersection_visit_count()
{

	g_write_count++;			

	int* visit_count = NULL;
	visit_count = get_intersection_visit_count();

	int max = 0;

	if (visit_count == NULL)
		return;
	int i;

	for (i=1;i<INTERSECTION_COUNT;i++)
	{
		if (visit_count[i] > max)
			max = visit_count[i];
	}

	if (g_write_count == 1) 
	{
		g_gnuplotpipe = popen("gnuplot -p","w");
	}

	if (g_gnuplotpipe != NULL)
	{
		if (g_write_count == 1)
		{
			fprintf(g_gnuplotpipe,"unset key\n");
		    fprintf(g_gnuplotpipe,"set title \"Intersection Visit Count\"\n");
			fprintf(g_gnuplotpipe,"set tic scale 0\n");
			fprintf(g_gnuplotpipe,"set palette rgbformula -7,2,-7\n");
			fprintf(g_gnuplotpipe,"set cblabel \"Visit Count\"\n");
			fprintf(g_gnuplotpipe,"set xrange[-0.5:6.5]\n");
			fprintf(g_gnuplotpipe,"set yrange[-0.5:6.5]\n");
			fprintf(g_gnuplotpipe,"set view map\n");
		}
		fprintf(g_gnuplotpipe,"set cbrange [0:%d]\n",max);
			

		if (g_write_count % g_write_interval != 0)
			return;
		fprintf(g_gnuplotpipe,"splot '-' matrix with image\n");
	
		for(i=1;i<INTERSECTION_COUNT;i++)
		{
			fprintf(g_gnuplotpipe,"%d ",visit_count[i]);

			if ( i%7 == 0)
				fprintf(g_gnuplotpipe,"\n");
		}	

		fprintf(g_gnuplotpipe,"e\ne\n");
	}
}

double g_x;
double g_y;
int g_direction;

/* taehwan 20140724 */
void convertDigraphToCoordinate(int head,int tail,double offset,double length)
{
	// head to tail range 1~49
	// 1) coordinate
	//      a]x = ((head-1) % 7) + 1
	//      b]x = floor( (head+6) / 7 )
	// 2) d = offset / edge_length 
	//      a]head - tail == -1 : x += d
	//      b]head - tail == 1  : x -= d
	//      c]head - tail < -1  : y += d
	//      d]head - tail > 1   : y -= d
	g_x = ( (head-1) % 7 ) + 1;
	g_y = floor( (head+6) / 7 );

	double d = offset / length;

	if (head-tail == -1)
	{
		g_direction = 1;
		g_x += d;
	}
	else if (head-tail == 1)
	{
		g_direction = 3;
		g_x -= d;
	}
	else if (head-tail < - 1)
	{
		g_direction = 2;
		g_y += d;
	}
	else if (head-tail > 1)
	{
		g_direction = 4;
		g_y -= d;
	}
	else
		printf("error %d,%d\n",head,tail);
}

double g_vehicle_point[VEHICLE_COUNT_MAX][3]={0,};
int g_vehicle_have_packet[VEHICLE_COUNT_MAX]={0,};

/* taehwan 20140728 */
boolean g_is_packet_forwarding = FALSE;
boolean g_gnuplot_init = FALSE;

int g_gnuplot_packet_forwarding_delay = 1000;

/* taehwan 20140725 */
void gnuplot_vehicle_point(double currenttime)
{
	int i;

	g_write_count++;

	char buf[28];

	int interval = g_write_interval;

	if (g_write_count == 1) 
	{
		g_gnuplotpipe = popen("gnuplot -p","w");
	}

//	if (g_current_time < 8400 || g_current_time > 9200)
//		return;

	if (g_gnuplotpipe != NULL)
	{
		if (g_gnuplot_init == FALSE)
		{
			g_gnuplot_init = TRUE;

			printf("Init GNUPlot\n");
			/* make gif routine. annotate below if you want realtime visualizatoin.*/
			//fprintf(g_gnuplotpipe,"set term gif animate\n");
			//fprintf(g_gnuplotpipe,"set output \"trace_log.gif\"\n");
			/* end of gif routine */
			fprintf(g_gnuplotpipe,"reset\n");
			fprintf(g_gnuplotpipe,"set xrange[0:8]\n");
			fprintf(g_gnuplotpipe,"set yrange[0:8]\n");
			fprintf(g_gnuplotpipe,"set grid\n");
			fprintf(g_gnuplotpipe,"set title \"TPD\"\n");
			fprintf(g_gnuplotpipe,"unset key\n");
			fprintf(g_gnuplotpipe,"unset colorbox\n");
			fprintf(g_gnuplotpipe,"set palette model RGB\n");
			fprintf(g_gnuplotpipe,"set palette model RGB defined (0 \"black\", 1 \"blue\", 2 \"green\", 3 \"red\")\n");

		}

		if (g_is_packet_forwarding == TRUE)
			interval = g_gnuplot_packet_forwarding_delay;

		if (g_write_count % interval != 0)
			return;

		
		fprintf(g_gnuplotpipe,"set title \"TPD t=%.0f\"\n",g_current_time);
		fprintf(g_gnuplotpipe,"plot '-' using 2:3:1:4 with labels textcolor palette\n");

		g_vehicle_color[1] = 3;

		g_vehicle_have_packet[1] = 3;
/*
		g_vehicle_have_packet[111] = 2;
		g_vehicle_have_packet[50] = 2;
		g_vehicle_have_packet[107] = 2;
		g_vehicle_have_packet[142] = 2;
*/
		//g_vehicle_have_packet[26] = 2;
		//g_vehicle_have_packet[156] = 2;
		//g_vehicle_have_packet[43] = 2;
		//g_vehicle_have_packet[95] = 2;

		boolean isPacketForwarding = FALSE;

		for(i=1;i<VEHICLE_COUNT_MAX;i++)
		{

			int temp_color;
			
			temp_color = g_vehicle_color[i];
			
			//if (g_vehicle_point[i][0] == 4 && g_vehicle_point[i][1] == 4)
			//{
			//	g_vehicle_color[i] = 1;
			//}

			temp_color = g_vehicle_color[i];

			if (g_vehicle_have_packet[i] == 1)
			{
				temp_color = g_color_flag;
			}
			else
			{
				temp_color = g_vehicle_have_packet[i];
			}			

			if (g_vehicle_point[i][0] == 0)
			{
				break;
			}
			
			double x = 0;
			double y = 0;

			x = g_vehicle_point[i][0];
			y = g_vehicle_point[i][1];

			double ROAD_OFFSET = -0.1;
			switch((int)g_vehicle_point[i][2])
			{
				case 1:
					y += ROAD_OFFSET;
					break;
				case 2:
					x -= ROAD_OFFSET;
					break;
				case 3:
					y -= ROAD_OFFSET;
					break;
				case 4:
					x += ROAD_OFFSET;
					break;
			}

			fprintf(g_gnuplotpipe,"%d %2.8f %2.8f %d\n",
				i,
				x,
				y,
				temp_color);

			if (g_vehicle_have_packet[i] == 1)
			{
				isPacketForwarding = TRUE;
				//if (i != 26) g_vehicle_have_packet[26] = 2; 	
                //if (i != 156) g_vehicle_have_packet[156] = 2;
				//if (i != 43) g_vehicle_have_packet[43] = 2; 
                //if (i != 95) g_vehicle_have_packet[95] = 2;
			}
		}
		if (isPacketForwarding == TRUE)
			g_is_packet_forwarding = TRUE;
		else
			g_is_packet_forwarding = FALSE;

		fprintf(g_gnuplotpipe,"e\n");
	}
}

/** run simulation */

int main(int argc, char** argv)
{
	//test_gamma2();
	char graph_file[BUF_SIZE]; //graph configuration file is specified in conf_file
	char output_file[BUF_SIZE]; //output file name
	char output_file_1[BUF_SIZE] = OUTPUT_FILE_1; //output file of simulation results in the format of text (.txt)
	char output_file_2[BUF_SIZE] = OUTPUT_FILE_2; //output file of simulation summary in the format of excel (.xls)
	char schedule_file[BUF_SIZE] = SCHEDULE_FILE; //schedule file
	char vanet_file[BUF_SIZE]; //vanet log file
	char vanet_file_suffix[BUF_SIZE] = VANET_FILE_SUFFIX; //vanet log file suffix
	char vanet_packet_carrier_trace_file[BUF_SIZE]; //vanet packet carrier trace log file
	char vanet_packet_carrier_trace_file_suffix[BUF_SIZE] = VANET_PACKET_CARRIER_TRACE_FILE_SUFFIX; //vanet packet carrier trace log file suffix
	char pathlist_file[BUF_SIZE] = PATHLIST_FILE; //path-list log file
	char trace_file_of_vehicle_convoy_length[BUF_SIZE] = TRACE_FILE_OF_VEHICLE_CONVOY_LENGTH; //trace file of vehicle convoy length 
	char trace_file_of_sensor_number[BUF_SIZE] = TRACE_FILE_OF_SENSOR_NUMBER; //trace file of sensor number over surveillance duration
	char trace_file_of_hole_number[BUF_SIZE] = TRACE_FILE_OF_HOLE_NUMBER; //trace file of hole number over surveillance duration
	char trace_file_of_sleeping_time[BUF_SIZE] = TRACE_FILE_OF_SLEEPING_TIME; //trace file of sleeping time over surveillance duration
	char trace_file_of_vehicle_detection_time[BUF_SIZE] = TRACE_FILE_OF_VEHICLE_DETECTION_TIME; //trace file of vehicle detection time over surveillance duration

	struct parameter param; //simulation parameter
	unsigned int seed = 1; //seed for initializing rand(): this gives one hole initially
	//unsigned int seed = 2; //seed for initializing rand(): this guarantees no hole initially
	time_t start_time, end_time, computation_time; //time variables for measuring computation time
	int result = 0;
	int trajectory_for_graph_with_2_nodes[] = {1, 2}; //trajectory for stationary vehicle in a graph with 2 nodes
	int trajectory_for_graph_with_3_nodes[] = {1, 2, 3}; //trajectory for stationary vehicle in a graph with 3 nodes
	int trajectory_for_graph_with_4_nodes[] = {1, 2, 4}; //trajectory for stationary vehicle in a graph with 4 nodes
	int trajectory_for_graph_with_5_nodes[] = {1, 2, 3, 4, 5}; //trajectory for stationary vehicle in a graph with 5 nodes
	int trajectory_for_graph_with_9_nodes[] = {1, 2, 3, 6, 9}; //trajectory for stationary vehicle in a graph with 9 nodes
	int trajectory_for_graph_with_20_nodes[] = {12, 13, 14, 15}; //trajectory for stationary vehicle in a graph with 9 nodes
	int trajectory_for_graph_with_36_nodes[] = {15, 9, 3, 2, 1}; //trajectory for stationary vehicle in a graph with 36 nodes where the AP is located at intersection 1
	//int trajectory_for_graph_with_36_nodes[] = {1, 2, 3, 9, 15}; //trajectory for stationary vehicle in a graph with 36 nodes where the AP is located at intersection 15
	int trajectory_for_graph_with_49_nodes[] = {1, 2, 3, 4, 11, 18, 25}; //trajectory for stationary vehicle in a graph with 49 nodes where the AP is located at intersection 25

	int target_vehicle_trajectory_for_graph_with_20_nodes[] = {1, 2, 3, 4, 5}; //target vehicle's trajectory in a graph with 20 nodes
	int target_vehicle_trajectory_for_graph_with_36_nodes[] = {1, 2, 3, 4, 5, 6}; //target vehicle's trajectory in a graph with 36 nodes
	int target_vehicle_trajectory_for_graph_with_49_nodes[] = {1, 2, 3, 4, 5, 6, 7}; //target vehicle's trajectory in a graph with 36 nodes
	int *trajectory = NULL; //trajectory for a vehicle in a graph
	int trajectory_size = 0; //trajectory size

#ifdef _LINUX_
	int c; //option character
#endif
	
	char *ptr = NULL; //pointer to buffer

	/** parameters passed from argument list argv */
	boolean forwarding_probability_and_statistics_flag = FALSE; //flag to indicate whether the forwarding probability_and_statistics information is logged every EDD update or not
	boolean flag_forwarding_probability_and_statistics_flag = FALSE; //flag to indicate whether forwarding_probability_and_statistics_flag is passed from argv 

	boolean packet_delay_measurement_flag = FALSE; //flag to indicate whether the packet delay measurement is performed or not
	boolean flag_packet_delay_measurement_flag = FALSE;

	int packet_delay_measurement_target_point = 0; //target point for the packet delay measurement
	boolean flag_packet_delay_measurement_target_point = FALSE;

	double packet_delay_measurement_time = 0; //packet delay measurement time
	boolean flag_packet_delay_measurement_time = FALSE;

	comparison_target_type_t comparison_target_type = COMPARISON_UNKNOWN; 
	boolean flag_comparison_target_type = FALSE; //flag to indicate whether comparison_target_type is passed from argv 

	evaluation_type_t evaluation_type = EVALUATION_UNKNOWN; 
	boolean flag_evaluation_type = FALSE; //flag to indicate whether evaluation_type is passed from argv 

	char param_conf_file[BUF_SIZE] = CONF_FILE; //parameter configuration file
	boolean flag_param_conf_file = FALSE; //flag to indicate whether param_conf_file is passed from argv 

	distribution_type_t vehicle_speed_distribution = NORMAL; //vehicle_speed_distribution
	boolean flag_vehicle_speed_distribution = FALSE; //flag to indicate whether vehicle_speed_distribution is passed from argv
	
	double vehicle_speed_in_mile_per_hour = 0; //unit of speed is mile/hour [MPH]
	double vehicle_speed_in_km_per_hour = 0; //unit of speed is km/hour [km/h]
	double vehicle_speed = 0; //unit of speed is meter/sec [m/s]
	boolean flag_vehicle_speed = FALSE; //flag to indicate whether vehicle_speed is passed from argv 

	double vehicle_speed_standard_deviation_in_mile_per_hour = 0; //unit of speed variation is mile/hour [MPH]
	double vehicle_speed_standard_deviation_in_km_per_hour = 0; //unit of speed variation is km/hour [km/h]
	double vehicle_speed_standard_deviation = 0; //unit of speed variation is km/hour [km/h]
	boolean flag_vehicle_speed_standard_deviation = FALSE; //flag to indicate whether vehicle_speed_standard_deviation is passed from argv 

	distribution_type_t vehicle_path_length_distribution = EQUAL; //vehicle path length distribution = {EQUAL, NORMAL}
	boolean flag_vehicle_path_length_distribution = FALSE;

	double vehicle_path_length_standard_deviation_in_mile = 0; //unit is mile
	double vehicle_path_length_standard_deviation_in_km = 0; //unit is kilometer
	double vehicle_path_length_standard_deviation = 0; //unit is meter
	boolean flag_vehicle_path_length_standard_deviation = FALSE; //flag to indicate whether vehicle_path_length_standard_deviation is passed from argv 

	double vehicle_interarrival_time = 0; //unit is seconds
	boolean flag_vehicle_interarrival_time = FALSE; //flag to indicate whether vehicle_interarrival_time is passed from argv

	double vehicle_think_time = 0; //unit is seconds
	boolean flag_vehicle_think_time = FALSE; //flag to indicate whether vehicle_think_time is passed from argv

	performance_metric_type_t metric_type = METRIC_NETWORK_LIFETIME; //performance metric type = {METRIC_NETWORK_LIFETIME, METRIC_AVERAGE_DETECTION_TIME};
	boolean flag_metric_type = FALSE;

	double simulation_time = 0; //simulation time
	boolean flag_simulation_time = FALSE;

	data_forwarding_mode_t data_forwarding_mode = DATA_FORWARDING_MODE_UNKNOWN;
	boolean flag_data_forwarding_mode = FALSE;
	
	data_forwarding_link_selection_t data_forwarding_link_selection = FORWARDING_LINK_SELECTION_ANGLE; //data forwarding link selection = {angle, distance, delay}
	boolean flag_data_forwarding_link_selection = FALSE;

	boolean data_forwarding_two_way_forwarding_flag = FALSE;
	boolean flag_data_forwarding_two_way_forwarding_flag = FALSE;

	boolean data_forwarding_multiple_target_point_flag = FALSE;
	boolean flag_data_forwarding_multiple_target_point_flag = FALSE;

	int data_forwarding_maximum_target_point_number = 0;
	boolean flag_data_forwarding_maximum_target_point_number = FALSE;

	vanet_edd_and_link_model_type_t vehicle_vanet_edd_and_link_model = VANET_EDD_AND_LINK_MODEL_UNKNOWN; //VANET EDD and Link model type
	boolean flag_vehicle_vanet_edd_and_link_model = FALSE;

	vanet_edd_model_type_t edd_model_type = VANET_EDD_PER_VEHICLE_MODEL; //VANET EDD model type
	boolean flag_edd_model_type = FALSE;

	vanet_edd_computation_model_type_t edd_computation_model = VANET_EDD_COMPUTATION_BASED_ON_STOCHASTIC_MODEL; //VANET EDD computation model
	boolean flag_edd_computation_model = FALSE;

	vanet_tbd_edd_computation_type_t tbd_edd_computation_type = VANET_EDD_BASED_ON_TBD_WITH_PARTIAL_PATH; //VANET TBD's EDD computation type
	boolean flag_tbd_edd_computation_type = FALSE;

	vanet_edge_delay_model_type_t edge_delay_model_type = VANET_EDGE_DELAY_UNKNOWN_MODEL; //VANET Edge Delay model type
	boolean flag_edge_delay_model_type = FALSE;

	vanet_forwarding_type_t forwarding_type = VANET_FORWARDING_BASED_ON_VEHICLE; //VANET Forwarding Type
        boolean flag_forwarding_type = FALSE;

	vanet_intersection_forwarding_type_t intersection_forwarding_type = VANET_INTERSECTION_FORWARDING_DEFERRED_FORWARDING; //VANET Intersection Forwarding Type
	boolean flag_intersection_forwarding_type = FALSE;

	int vehicle_maximum_number = 0; //maximum number of vehicles moving on the road network; unit is vehicles
	boolean flag_vehicle_maximum_number = FALSE; //flag to indicate whether vehicle_maximum_number is passed from argv

	boolean vehicle_vanet_stationary_vehicle_flag = FALSE; //VANET Stationary Vehicle Flag
    boolean flag_vehicle_vanet_stationary_vehicle_flag = FALSE;

	int vehicle_packet_generating_entity_number = 0; //number of packet generating vehicles
	boolean flag_vehicle_packet_generating_entity_number = FALSE; //flag to indicate whether vehicle_packet_generating_entity_number is passed from argv

	double vehicle_AP_passing_entity_percentage = 0; //percentage of vehicles passing AP(s)
	boolean flag_vehicle_AP_passing_entity_percentage = FALSE;

	vanet_vehicular_traffic_model_type_t vehicle_vanet_vehicular_traffic_model = VANET_VEHICULAR_TRAFFIC_UNKNOWN_MODEL; //VANET Vehicular Traffic model
	boolean flag_vehicle_vanet_vehicular_traffic_model = FALSE;

	boolean vehicle_vanet_acl_measurement_flag = FALSE; //Average Convoy Length (ACL) measurement flag
	boolean flag_vehicle_vanet_acl_measurement_flag = FALSE;

	double communication_packet_interarrival_time = 0; //packet interarrival time; unit is seconds
	boolean flag_communication_packet_interarrival_time = FALSE; //flag to indicate whether communication_packet_interarrival_time is passed from argv

	double communication_range = 0; //communication range; unit is meters
	boolean flag_communication_range = FALSE; //flag to indicate whether communication_range is passed from argv

	double communication_packet_ttl = 0; //packet TTL; unit is seconds
	boolean flag_communication_packet_ttl = FALSE; //flag to indicate whether communication_packet_ttl is passed from argv

	boolean communication_packet_ttl_override_flag = FALSE; //packet TTL override flag
	boolean flag_communication_packet_ttl_override_flag = FALSE; //flag to indicate whether communication_packet_ttl_override_flag is passed from argv

	int communication_packet_hop_limit = 0; //packet hop limit; unit is number
	boolean flag_communication_packet_hop_limit = FALSE;

	int communication_packet_maximum_number = 0; //maximum number of generated packets
	boolean flag_communication_packet_maximum_number = FALSE; //flag to indicate whether communication_packet_maximum_number is passed from argv

	boolean communication_multiple_AP_flag = FALSE; //flag to indicate that multiple APs are deployed in the road network
	boolean flag_communication_multiple_AP_flag = FALSE; //flag to indicate whether communication_multiple_AP_flag is passed from argv

	int communication_AP_maximum_number = 0; //maximum number of APs deployed in the road network
	boolean flag_communication_AP_maximum_number = FALSE; //flag to indicate whether communication_AP_maximum_number is passed from argv

	int communication_SN_maximum_number = 0; //maximum number of Stationary Nodes (SNs) deployed in the road network
	boolean flag_communication_SN_maximum_number = FALSE;

	double communication_packet_delivery_probability_threshold = 0; //the threshold of the packet delivery probability that the packet will arrive at the target point (i.e., target intersection) earlier than the destination vehicle
	boolean flag_communication_packet_delivery_probability_threshold = FALSE; //flag to indicate whether communication_packet_delivery_probability_threshold is passed from argv

	int communication_packet_reverse_traversal_hop_distance_threshold = 1; //the threshold of the hop distance from the packet and the destination vehicle on the destination vehicle's trajectory in order to determine the packet reverse traversal
	boolean flag_communication_packet_reverse_traversal_hop_distance_threshold = FALSE;

	int graph_node_number = 0; //graph node number in road network
	boolean flag_graph_node_number =  FALSE; //flag to indicate whether graph_node_number is passed from argv

	vanet_target_point_selection_type_t vehicle_vanet_target_point_selection_type = VANET_TARGET_POINT_SELECTION_UNKNOWN;
	boolean flag_vehicle_vanet_target_point_selection_type = FALSE;

	vanet_target_point_computation_method_t vehicle_vanet_target_point_computation_method = VANET_TARGET_POINT_COMPUTATION_METHOD_UNKNOWN;
	boolean flag_vehicle_vanet_target_point_computation_method = FALSE;

	vanet_target_point_search_space_type_t vehicle_vanet_target_point_search_space_type = VANET_TARGET_POINT_SEARCH_SPACE_UNKNOWN; //target point search space type for the destination vehicle's trajectory
	boolean flag_vehicle_vanet_target_point_search_space_type = FALSE;

	double vehicle_vanet_target_point_recomputation_interval_denominator = 1.0;
	boolean flag_vehicle_vanet_target_point_recomputation_interval_denominator = FALSE;

	vanet_vehicle_trajectory_type_t vehicle_vanet_vehicle_trajectory_type = VANET_VEHICLE_TRAJECTORY_UNKNOWN;
	boolean flag_vehicle_vanet_vehicle_trajectory_type = FALSE;

	vanet_vehicle_trajectory_length_type_t vehicle_vanet_vehicle_trajectory_length_type = VANET_VEHICLE_TRAJECTORY_LENGTH_UNKNOWN;
	boolean flag_vehicle_vanet_vehicle_trajectory_length_type = FALSE;

	boolean tpd_encounter_graph_optimization_flag = FALSE; //flag to determine whether to perform Dynamic Programming (DP) for the optimization of the predicted encounter graph for a better EDR
	boolean flag_tpd_encounter_graph_optimization_flag = FALSE;

	boolean tpd_encounter_graph_source_routing_flag = FALSE; //flag to determine whether to perform source routing with the predicted encounter graph
	boolean flag_tpd_encounter_graph_source_routing_flag = FALSE;

	vanet_forwarding_scheme_t vanet_forwarding_scheme = VANET_FORWARDING_UNKNOWN; //vanet forwarding scheme
	boolean flag_vanet_forwarding_scheme = FALSE;

	/***********************************************************************/

	/**@for debugging */

	//test_get_position_on_linear_curve();
        //test whether get_position_on_linear_curve() works well or not

	/******************/

#ifdef _LINUX_
	opterr = 0; //we don't want getopt() to write an error message to stderr.

	while((c = getopt(argc, argv, "#:@:*:a:b:c:d:e:f:g:h:i:j:k:l:m:n:o:p:q:r:s:t:u:v:w:x:y:z:A:B:C:D:E:F:G:H:I:J:K:L:M:N:O:P:Q:R:S:T:U:V:W:X:Y:Z:")) != -1) //c: means that option character c has an argument
	{
	    switch(c)
	    {
			case '#':
				g_gnuplot_packet_forwarding_delay = atoi(optarg);
				break;
			case '@':
				g_write_interval = atoi(optarg); 
				break;
			case '*': //gnuplot
				// 0 - no gnuplot
				// 1 - visit count
				// 2 - vehicle movement
				g_gnuplot_option = atoi(optarg);
				break;
		    case 'a': //Comparison target type
                comparison_target_type = (comparison_target_type_t)atoi(optarg);
                flag_comparison_target_type = TRUE;
		        break;

		    case 'b': //Evaluation type
                evaluation_type = (evaluation_type_t)atoi(optarg);
                flag_evaluation_type = TRUE;
	    	    break;

		//////////////////////////////////////////
		//@Other Parameters
            case 'c': //VANET TBD's EDD computation type
                tbd_edd_computation_type = (vanet_tbd_edd_computation_type_t)atoi(optarg);
                flag_tbd_edd_computation_type = TRUE;
                break;

            case 'd': //VANET Edge Delay model type
                edge_delay_model_type = (vanet_edge_delay_model_type_t)atoi(optarg);
                flag_edge_delay_model_type = TRUE;
                break;

		    case 'e': //VANET EDD model type
                edd_model_type = (vanet_edd_model_type_t)atoi(optarg);
                flag_edd_model_type = TRUE;
                break;

		    case 'f': //VANET forwarding scheme
	    	    vanet_forwarding_scheme = (vanet_forwarding_scheme_t)atoi(optarg);
                flag_vanet_forwarding_scheme = TRUE;
	        	break;

            case 'g': //Number of packet generating vehicles
                vehicle_packet_generating_entity_number = atoi(optarg);
                flag_vehicle_packet_generating_entity_number = TRUE;
                break;

            case 'h': //Average Convoy Length (ACL) measurement flag
                vehicle_vanet_acl_measurement_flag = (boolean)atoi(optarg);
                flag_vehicle_vanet_acl_measurement_flag = TRUE;
                break;

		    case 'i': //packet interarrival time
		        communication_packet_interarrival_time = atof(optarg);
	    	    flag_communication_packet_interarrival_time = TRUE;
	        	break;

		    case 'j': //maximum number of generated packets
		        communication_packet_maximum_number = atoi(optarg);
	    	    flag_communication_packet_maximum_number = TRUE;
	        	break;

		    case 'k': //graph node number in road network
		        graph_node_number = atoi(optarg); 
	    	    flag_graph_node_number =  TRUE;
                break;

		    case 'l':
		        vehicle_vanet_edd_and_link_model = (vanet_edd_and_link_model_type_t)atoi(optarg);
	    	    flag_vehicle_vanet_edd_and_link_model = TRUE;
                break;

            case 'm': //VANET Vehicular Traffic model
                vehicle_vanet_vehicular_traffic_model = (vanet_vehicular_traffic_model_type_t)atoi(optarg);
                flag_vehicle_vanet_vehicular_traffic_model = TRUE;
                break;

            case 'n': //Maximum number of vehicles
                vehicle_maximum_number = atoi(optarg);
                flag_vehicle_maximum_number = TRUE;
                break;

		    case 'o': //VANET Stationary Vehicle Flag
		        vehicle_vanet_stationary_vehicle_flag = (boolean)atoi(optarg);
	    	    flag_vehicle_vanet_stationary_vehicle_flag = TRUE;
	        	break;

		    case 'p':
                strcpy(param_conf_file, optarg);
                flag_param_conf_file = TRUE;
                break;

            case 'q':
		        vehicle_speed_distribution = (distribution_type_t)atoi(optarg);
	    	    flag_vehicle_speed_distribution = TRUE;
	        	break;

		    case 'r': //communication range
		        communication_range = atof(optarg);
	    	    flag_communication_range = TRUE;
	        	break;

		    case 's':
		        seed = atoi(optarg);
                break;

		    case 't': //packet TTL
		        communication_packet_ttl = atof(optarg);
	    	    flag_communication_packet_ttl = TRUE;
	        	break;

		    case 'u':
    	        simulation_time = atof(optarg);
	    	    flag_simulation_time = TRUE;
                break;

		    case 'v':
                vehicle_speed_in_mile_per_hour = atof(optarg);    
                vehicle_speed_in_km_per_hour = convert_mile_per_hour_to_km_per_hour(vehicle_speed_in_mile_per_hour);		    
                vehicle_speed = convert_mile_per_hour_to_meter_per_sec(vehicle_speed_in_mile_per_hour);
                flag_vehicle_speed = TRUE;
	        break;

		    case 'w':
                vehicle_speed_standard_deviation_in_mile_per_hour = atof(optarg);    
                vehicle_speed_standard_deviation_in_km_per_hour = convert_mile_per_hour_to_km_per_hour(vehicle_speed_standard_deviation_in_mile_per_hour);		    
                vehicle_speed_standard_deviation = convert_mile_per_hour_to_meter_per_sec(vehicle_speed_standard_deviation_in_mile_per_hour);
                flag_vehicle_speed_standard_deviation = TRUE;
	        break;

		    case 'x':
		        strcpy(output_file_1, optarg);
	    	    break;

	   	    case 'y':
				strcpy(output_file_2, optarg);
		        break;

		    case 'z':
		        vehicle_interarrival_time = atof(optarg);
	    	    flag_vehicle_interarrival_time = TRUE;
	        	break;

		    case 'A':
		        vehicle_AP_passing_entity_percentage = atof(optarg);
	    	    flag_vehicle_AP_passing_entity_percentage = TRUE;
	        	break;

		    case 'B': //packet TTL override flag
		        communication_packet_ttl_override_flag = (boolean)atoi(optarg);
	    	    flag_communication_packet_ttl_override_flag = TRUE;
	        	break;

		    case 'C':
		        vehicle_vanet_target_point_computation_method = (vanet_target_point_computation_method_t)atoi(optarg);
				flag_vehicle_vanet_target_point_computation_method = TRUE;
				break;

	   	    case 'D':
				vehicle_vanet_vehicle_trajectory_length_type = (vanet_vehicle_trajectory_length_type_t)atoi(optarg);
				flag_vehicle_vanet_vehicle_trajectory_length_type = TRUE;
				break;

            case 'E':
                communication_packet_delivery_probability_threshold = atof(optarg);
                flag_communication_packet_delivery_probability_threshold = TRUE;
                break;

		    case 'F':
		        data_forwarding_mode = (data_forwarding_mode_t)atoi(optarg);
	    	    flag_data_forwarding_mode = TRUE;
	        	break;

			case 'G':
				vehicle_path_length_standard_deviation_in_mile = atof(optarg);
				vehicle_path_length_standard_deviation_in_km = convert_mile_to_km(vehicle_path_length_standard_deviation_in_mile);
				vehicle_path_length_standard_deviation = convert_mile_to_meter(vehicle_path_length_standard_deviation_in_mile);
				flag_vehicle_path_length_standard_deviation = TRUE;
	        	break;

            case 'H': //packet hop limit
                communication_packet_hop_limit = atoi(optarg);
                flag_communication_packet_hop_limit = TRUE;
                break;

		    case 'I':
		        vehicle_vanet_target_point_recomputation_interval_denominator = atof(optarg);
	    	    flag_vehicle_vanet_target_point_recomputation_interval_denominator = TRUE;
	        	break;

			case 'J':
				communication_packet_reverse_traversal_hop_distance_threshold = atoi(optarg);
				flag_communication_packet_reverse_traversal_hop_distance_threshold = TRUE;
				break;

			case 'K':
				data_forwarding_two_way_forwarding_flag = (boolean)atoi(optarg);
				flag_data_forwarding_two_way_forwarding_flag = TRUE;
				break;

		    case 'L':
		        data_forwarding_link_selection = (data_forwarding_link_selection_t)atoi(optarg);
	    	    flag_data_forwarding_link_selection = TRUE;
	        	break;
	
		    case 'M':
		        communication_multiple_AP_flag = (boolean)atoi(optarg);
                flag_communication_multiple_AP_flag = TRUE;
				break;

		    case 'N': //maximum number of APs deployed in the road network
		        communication_AP_maximum_number = atoi(optarg);
	    	    flag_communication_AP_maximum_number = TRUE;
	        	break;

		    case 'O':
		        tpd_encounter_graph_optimization_flag = (boolean)atoi(optarg);
	    	    flag_tpd_encounter_graph_optimization_flag = TRUE;
	        	break;

			case 'Q':
				TPD_Set_Margin_Time(atoi(optarg));
				break;
		    case 'P':
		        vehicle_vanet_target_point_search_space_type = (vanet_target_point_search_space_type_t)atoi(optarg);
				flag_vehicle_vanet_target_point_search_space_type = TRUE;
		        break;

            case 'R':
                forwarding_probability_and_statistics_flag = (boolean)atoi(optarg);
                flag_forwarding_probability_and_statistics_flag = TRUE;
                break;

		    case 'S':
	    	    tpd_encounter_graph_source_routing_flag = (boolean)atoi(optarg);
				flag_tpd_encounter_graph_source_routing_flag= TRUE;
				break;

		    case 'T':
		        vehicle_vanet_vehicle_trajectory_type = (vanet_vehicle_trajectory_type_t)atoi(optarg);
	    	    flag_vehicle_vanet_vehicle_trajectory_type = TRUE;
	        	break;

			case 'U':
				communication_SN_maximum_number = atoi(optarg);
				flag_communication_SN_maximum_number = TRUE;
				break;

		    case 'V':
		        vehicle_path_length_distribution = (distribution_type_t)atoi(optarg);
	    	    flag_vehicle_path_length_distribution = TRUE;
	        	break;

			case 'W':
				data_forwarding_multiple_target_point_flag = (boolean)atoi(optarg);
				flag_data_forwarding_multiple_target_point_flag = TRUE;
				break;

            case 'X':
                packet_delay_measurement_flag = (boolean)atoi(optarg);
                flag_packet_delay_measurement_flag = TRUE;
                break;

            case 'Y':
                packet_delay_measurement_target_point = atoi(optarg);
                flag_packet_delay_measurement_target_point = TRUE;
                break;

            case 'Z':
                packet_delay_measurement_time = atof(optarg);
                flag_packet_delay_measurement_time = TRUE;
                break;

	    default:
                printf("main(): option of %c is not supported yet!\n", c);
                exit(1);
	    }
	}
#endif

	printf("param_conf_file = \"%s\"\n",param_conf_file);
	/** check the data forwarding mode to select parameter configuration file */
	if(flag_data_forwarding_mode == TRUE && flag_param_conf_file == FALSE)
	{
		if(data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
		{
			strcpy(param_conf_file, CONF_FILE_FOR_DOWNLOAD);
		}
		else if(data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
		{
			strcpy(param_conf_file, CONF_FILE_FOR_UPLOAD);
		}
		else if(data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
		{
			strcpy(param_conf_file, CONF_FILE_FOR_V2V);
		}
		else
		{
			printf("%s:%d: data_forwarding_mode(%d) is not supported!\n", 
					__FUNCTION__, __LINE__,
					data_forwarding_mode);
			exit(1);
		}
	}
         
	/** initialize param with conf_file */
	init_parameter(&param, param_conf_file);	

	/** update param with the passed parameters */
	if(flag_vanet_forwarding_scheme)
		param.vanet_forwarding_scheme = vanet_forwarding_scheme;

	printf("Parameter for Forwarding Scheme = %d(%s)\n", 
		param.vanet_forwarding_scheme, 
		get_vanet_forwarding_scheme_name(param.vanet_forwarding_scheme));

	if(flag_tpd_encounter_graph_optimization_flag)
	  param.tpd_encounter_graph_optimization_flag = tpd_encounter_graph_optimization_flag;

	if(flag_tpd_encounter_graph_source_routing_flag)
	  param.tpd_encounter_graph_source_routing_flag = tpd_encounter_graph_source_routing_flag;

    if(flag_forwarding_probability_and_statistics_flag)
      param.forwarding_probability_and_statistics_flag = forwarding_probability_and_statistics_flag;

	if(flag_comparison_target_type)
	  param.comparison_target_type = comparison_target_type;

	if(flag_evaluation_type)
	  param.evaluation_type = evaluation_type;

	/* Data forwarding mode = {Download, Upload} */
	if(flag_data_forwarding_mode)
	  param.data_forwarding_mode = data_forwarding_mode;

	/* Data forwarding link selection = {angle, distance, delay} */
	if(flag_data_forwarding_link_selection)
	  param.data_forwarding_link_selection = data_forwarding_link_selection;

	/* Data forwarding two-way flag */
	if(flag_data_forwarding_two_way_forwarding_flag)
	  param.data_forwarding_two_way_forwarding_flag = data_forwarding_two_way_forwarding_flag;

	/* Data forwarding multiple-target-point flag */
	if(flag_data_forwarding_multiple_target_point_flag)
	  param.data_forwarding_multiple_target_point_flag = data_forwarding_multiple_target_point_flag;

	/* The maximum number of target points allowed for multi-target-point data forwarding */
	if(flag_data_forwarding_maximum_target_point_number)
	  param.data_forwarding_maximum_target_point_number = data_forwarding_maximum_target_point_number;

    /* Packet Delay Measurement Parameters */
    if(flag_packet_delay_measurement_flag)
      param.packet_delay_measurement_flag = packet_delay_measurement_flag;

    if(flag_packet_delay_measurement_target_point)
      param.packet_delay_measurement_target_point = packet_delay_measurement_target_point;

    if(flag_packet_delay_measurement_time)
      param.packet_delay_measurement_time = packet_delay_measurement_time;

    /* Vehicle Parameters */
	if(flag_edd_model_type) //VANET EDD model type
	  param.vehicle_vanet_edd_model = edd_model_type;

	if(flag_edd_computation_model) //VANET EDD computation model
	  param.vehicle_vanet_edd_computation_model = edd_computation_model;

	if(flag_tbd_edd_computation_type) //VANET TBD's EDD computation type
	  param.vehicle_vanet_tbd_edd_computation_type = tbd_edd_computation_type;

	if(flag_edge_delay_model_type) //VANET Edge Delay model type
	  param.vehicle_vanet_edge_delay_model = edge_delay_model_type;

	if(flag_forwarding_type) //VANET Forwarding Type
	  param.vehicle_vanet_forwarding_type = forwarding_type;

	if(flag_intersection_forwarding_type) //VANET Intersection Forwarding Type
	  param.vehicle_vanet_intersection_forwarding_type = intersection_forwarding_type;

	if(flag_vehicle_vanet_target_point_selection_type) //VANET Target Point Selection Type
	  param.vehicle_vanet_target_point_selection_type = vehicle_vanet_target_point_selection_type;

	if(flag_vehicle_vanet_target_point_computation_method) //VANET Target Point Computation Method
	  param.vehicle_vanet_target_point_computation_method = vehicle_vanet_target_point_computation_method;

	if(flag_vehicle_vanet_target_point_search_space_type) //VANET Target Point Search Space Type
	  param.vehicle_vanet_target_point_search_space_type = vehicle_vanet_target_point_search_space_type;

	if(flag_vehicle_vanet_target_point_recomputation_interval_denominator) //VANET Target Point Recomputation Interval Denominator
	  param.vehicle_vanet_target_point_recomputation_interval_denominator = vehicle_vanet_target_point_recomputation_interval_denominator;

	if(flag_vehicle_vanet_vehicle_trajectory_type) //VANET Vehicle Trajectory Type
	  param.vehicle_vanet_vehicle_trajectory_type = vehicle_vanet_vehicle_trajectory_type;

	if(flag_vehicle_vanet_vehicle_trajectory_length_type) //VANET Vehicle Trajectory Length Type
	  param.vehicle_vanet_vehicle_trajectory_length_type = vehicle_vanet_vehicle_trajectory_length_type;

	/* set VANET EDD and Link Model type and update the EDD model and the edge delay model */
	if(flag_vehicle_vanet_edd_and_link_model)
	{
	  param.vehicle_vanet_edd_and_link_model = vehicle_vanet_edd_and_link_model;

	  switch(vehicle_vanet_edd_and_link_model)
	  {
	  case VANET_EDD_AND_LINK_MODEL_TBD_EDD_AND_TBD_LINK:
	    param.vehicle_vanet_edd_model = VANET_EDD_PER_VEHICLE_MODEL;
	    param.vehicle_vanet_edge_delay_model = VANET_EDGE_DELAY_TBD_MODEL_FOR_FINITE_ROAD;
	    break; 

	  case VANET_EDD_AND_LINK_MODEL_TBD_EDD_AND_VADD_LINK:
	    param.vehicle_vanet_edd_model = VANET_EDD_PER_VEHICLE_MODEL;
	    param.vehicle_vanet_edge_delay_model = VANET_EDGE_DELAY_VADD_MODEL;
	    break; 

	  case VANET_EDD_AND_LINK_MODEL_VADD_EDD_AND_VADD_LINK:
	    param.vehicle_vanet_edd_model = VANET_EDD_PER_INTERSECTION_MODEL;
	    param.vehicle_vanet_edge_delay_model = VANET_EDGE_DELAY_VADD_MODEL;
	    break; 

	  case VANET_EDD_AND_LINK_MODEL_VADD_EDD_AND_TBD_LINK:
	    param.vehicle_vanet_edd_model = VANET_EDD_PER_INTERSECTION_MODEL;
	    param.vehicle_vanet_edge_delay_model = VANET_EDGE_DELAY_TBD_MODEL_FOR_FINITE_ROAD;
	    break; 

	  default:
	    printf("main(): Error: vehicle_vanet_edd_and_link_model (%d) is not supported!\n", vehicle_vanet_edd_and_link_model);
	    exit(1);
	  }
	}

	/** update param.vehicle_vanet_edd_model accoring to param.vanet_forwarding_scheme */
	switch(param.vanet_forwarding_scheme)
	{
		case VANET_FORWARDING_VADD:
		case VANET_FORWARDING_EPIDEMIC:	
		case VANET_FORWARDING_TADB:
			param.vehicle_vanet_edd_model = VANET_EDD_PER_INTERSECTION_MODEL;
			break;

		case VANET_FORWARDING_TBD:
			param.vehicle_vanet_edd_model = VANET_EDD_PER_VEHICLE_MODEL;
			break;

		default:
			break;
	}

	/* select graph configuration file */
	if(flag_graph_node_number) //graph node number
	{
	  param.graph_node_number = graph_node_number;
	}

	if(flag_vehicle_maximum_number) //Maximum number of vehicles
	  param.vehicle_maximum_number = vehicle_maximum_number;

        /* set param.vehicle_speed_distribution to vehicle_speed_distribution */
	if(flag_vehicle_speed_distribution)
	  param.vehicle_speed_distribution = vehicle_speed_distribution;

	if(flag_vehicle_speed)
	{  
	  param.vehicle_speed = vehicle_speed;
	  param.vehicle_speed_in_mile_per_hour = vehicle_speed_in_mile_per_hour;
	  param.vehicle_speed_in_km_per_hour = vehicle_speed_in_km_per_hour;
	}

        /* update the vehicle speed standard deviation along with the update of the maximum and minimum vehicle speed */
	if(flag_vehicle_speed_standard_deviation)
	{
	  param.vehicle_speed_standard_deviation = vehicle_speed_standard_deviation;
	  param.vehicle_speed_standard_deviation_in_km_per_hour = vehicle_speed_standard_deviation_in_km_per_hour;
	  param.vehicle_speed_standard_deviation_in_mile_per_hour = vehicle_speed_standard_deviation_in_mile_per_hour;
          /* update the vehicle maximum and minimum speed bounds for the speed generation */
          if(param.vehicle_speed_bound_coefficient_flag)
              update_vehicle_maximum_and_minimum_speeds(&param);
	}

	if(flag_vehicle_path_length_distribution)
	{
	  param.vehicle_path_length_distribution = vehicle_path_length_distribution;
	}
      
	if(flag_vehicle_path_length_standard_deviation)
	{
	  /* set vehicle_path_length_distribution to NORMAL */
	  //param.vehicle_path_length_distribution = NORMAL;
	  param.vehicle_path_length_standard_deviation = vehicle_path_length_standard_deviation;
	  param.vehicle_path_length_standard_deviation_in_mile = vehicle_path_length_standard_deviation_in_mile;
	  param.vehicle_path_length_standard_deviation_in_km = vehicle_path_length_standard_deviation_in_km;
	}

	if(flag_vehicle_interarrival_time)
	{
	  /* set vehicle_interarrival_time_distribution to EXPONENTIAL */
          param.vehicle_interarrival_time_distribution = EXPONENTIAL;
	  param.vehicle_interarrival_time = vehicle_interarrival_time;
	}

        /* set vehicle's think time (i.e., waiting time) at intersections */
	if(flag_vehicle_think_time)
	{
	  /* set vehicle_think_time_distribution to UNIFORM */
          param.vehicle_think_time_distribution = UNIFORM;
	  param.vehicle_think_time = vehicle_think_time;
	}

	if(flag_vehicle_packet_generating_entity_number)
	  param.vehicle_packet_generating_entity_number = vehicle_packet_generating_entity_number;

	/* set vehicle_AP_passing_vehicle_percentage and vehicle_AP_passing_vehicle_number */
	if(flag_vehicle_AP_passing_entity_percentage)
	{
	  /* check the vality of the value */
	  if((vehicle_AP_passing_entity_percentage < 0) || (vehicle_AP_passing_entity_percentage > 100))
	  {
            printf("main(): Error: vehicle_AP_passing_entity_percentage(%.2f) must be between 0 and 100\n", (float) vehicle_AP_passing_entity_percentage);
	    exit(1);
	  }

	  param.vehicle_AP_passing_entity_percentage = vehicle_AP_passing_entity_percentage;

	  if(param.vehicle_maximum_number > 0)
	  {
	    param.vehicle_AP_passing_entity_number = (int)ceil(param.vehicle_AP_passing_entity_percentage/100.0*param.vehicle_maximum_number);
	  }
	}

	if(flag_vehicle_vanet_stationary_vehicle_flag)
	  param.vehicle_vanet_stationary_vehicle_flag = vehicle_vanet_stationary_vehicle_flag;

	/* VANET Vehicular Traffic model */
	if(flag_vehicle_vanet_vehicular_traffic_model)
          param.vehicle_vanet_vehicular_traffic_model = vehicle_vanet_vehicular_traffic_model;

	/* Average Convoy Length (ACL) measurement flag */
	if(flag_vehicle_vanet_acl_measurement_flag)
	  param.vehicle_vanet_acl_measurement_flag = vehicle_vanet_acl_measurement_flag; 

	if(flag_communication_range)
	  param.communication_range = communication_range;

	if(flag_communication_packet_ttl)
	  param.communication_packet_ttl = communication_packet_ttl;

	if(flag_communication_packet_ttl_override_flag)
	  param.communication_packet_ttl_override_flag = communication_packet_ttl_override_flag;

    if(flag_communication_packet_hop_limit)
		param.communication_packet_hop_limit = communication_packet_hop_limit;

	if(flag_communication_packet_maximum_number)
	  param.communication_packet_maximum_number = communication_packet_maximum_number;

	if(flag_communication_packet_interarrival_time)
	  param.communication_packet_interarrival_time = communication_packet_interarrival_time;

	if(flag_communication_multiple_AP_flag)
		param.communication_multiple_AP_flag = communication_multiple_AP_flag;

	if(flag_communication_AP_maximum_number)
		param.communication_AP_maximum_number = communication_AP_maximum_number;

	if(flag_communication_packet_delivery_probability_threshold)
	{
		param.communication_packet_delivery_probability_threshold = communication_packet_delivery_probability_threshold;
		/* check the validity of the delivery probability threshold */
		if((param.communication_packet_delivery_probability_threshold < 0) || (param.communication_packet_delivery_probability_threshold > 1))
		{
			printf("main(): param.communication_packet_delivery_probability_threshold(%.4f) must be between 0 and 1\n", (float)param.communication_packet_delivery_probability_threshold);
			exit(1);
		}
	}

	if(flag_communication_SN_maximum_number)
	{
		param.communication_SN_maximum_number = communication_SN_maximum_number;
		param.communication_multiple_SN_flag = TRUE; //the flag is automatically set to TRUE since SN_maximum_number was specified

		//param.vehicle_vanet_target_point_computation_method = VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_STATIC_FORWARDING; //let the packet stay at the target point without reverse traverse along the destination vehicle's trajectory
	}

	if(flag_simulation_time)
	  param.simulation_time = simulation_time;


	/** select the vehicle's trajectory for either a stationary vehicle or a target vehicle according to graph node number and data forwarding mode */
	switch(param.graph_node_number)
	{
	  case 2:
	    strcpy(param.graph_file_name, GRAPH_FILE_FOR_NODES_2);
	    strcpy(param.mobility_file_name, MOBILITY_FILE_FOR_NODES_2);
	    trajectory = trajectory_for_graph_with_2_nodes;
	    trajectory_size = sizeof(trajectory_for_graph_with_2_nodes)/sizeof(int);
            break;

	  case 3:
	    strcpy(param.graph_file_name, GRAPH_FILE_FOR_NODES_3);
	    strcpy(param.mobility_file_name, MOBILITY_FILE_FOR_NODES_3);
	    trajectory = trajectory_for_graph_with_3_nodes;
	    trajectory_size = sizeof(trajectory_for_graph_with_3_nodes)/sizeof(int);
            break;

	  case 4:
	    strcpy(param.graph_file_name, GRAPH_FILE_FOR_NODES_4);
	    strcpy(param.mobility_file_name, MOBILITY_FILE_FOR_NODES_4);
	    trajectory = trajectory_for_graph_with_4_nodes;
	    trajectory_size = sizeof(trajectory_for_graph_with_4_nodes)/sizeof(int);
	    break;

	  case 5:
	    strcpy(param.graph_file_name, GRAPH_FILE_FOR_NODES_5);
	    strcpy(param.mobility_file_name, MOBILITY_FILE_FOR_NODES_5);
	    trajectory = trajectory_for_graph_with_5_nodes;
	    trajectory_size = sizeof(trajectory_for_graph_with_5_nodes)/sizeof(int);
	    break;

	  case 9:
	    strcpy(param.graph_file_name, GRAPH_FILE_FOR_NODES_9);
	    strcpy(param.mobility_file_name, MOBILITY_FILE_FOR_NODES_9);
	    trajectory = trajectory_for_graph_with_9_nodes;
	    trajectory_size = sizeof(trajectory_for_graph_with_9_nodes)/sizeof(int);
	    break;

	  case 20:
	    if(param.communication_multiple_AP_flag)
            {
                /* copy graph configuration file name */
                if(param.data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
                {
                    strcpy(param.graph_file_name, GRAPH_FILE_FOR_NODES_20_WITH_MULTIPLE_AP_FOR_DOWNLOAD);
                }
                else if(param.data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
                {
                    strcpy(param.graph_file_name, GRAPH_FILE_FOR_NODES_20_WITH_MULTIPLE_AP_FOR_UPLOAD);
                }
                else if(param.data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
                {
                    strcpy(param.graph_file_name, GRAPH_FILE_FOR_NODES_20_WITH_MULTIPLE_AP_FOR_UPLOAD);
                }
                else
                {
					printf("%s:%d: data_forwarding_mode(%d) is not supported!\n", 
							__FUNCTION__, __LINE__,
							data_forwarding_mode);
                    exit(1);
                }
                
                /* copy mobility configuration file name */
				strcpy(param.mobility_file_name, MOBILITY_FILE_FOR_NODES_20_WITH_MULTIPLE_AP);
	    }
	    else
            {
	      strcpy(param.graph_file_name, GRAPH_FILE_FOR_NODES_20);
	      strcpy(param.mobility_file_name, MOBILITY_FILE_FOR_NODES_20);
	    }
	    
	    /** select a vehicle trajectory according to data forwarding mode */
		if(param.data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
	    { //packet download from AP to vehicles
	      trajectory = target_vehicle_trajectory_for_graph_with_20_nodes;
	      trajectory_size = sizeof(target_vehicle_trajectory_for_graph_with_20_nodes)/sizeof(int);
	    }
	    else if(param.data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
	    { //packet upload from vehicles to AP
	      trajectory = trajectory_for_graph_with_20_nodes;
	      trajectory_size = sizeof(trajectory_for_graph_with_20_nodes)/sizeof(int);
	    }
		else if(param.data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
	    { //packet download from AP to vehicles
	      trajectory = target_vehicle_trajectory_for_graph_with_20_nodes;
	      trajectory_size = sizeof(target_vehicle_trajectory_for_graph_with_20_nodes)/sizeof(int);
	    }
		break;

	  case 36:
	    if(param.communication_multiple_AP_flag)
            {
                /* copy graph configuration file name */
                if(param.data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
                {
                    strcpy(param.graph_file_name, GRAPH_FILE_FOR_NODES_36_WITH_MULTIPLE_AP_FOR_DOWNLOAD);
                }
                else if(param.data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
                {
                    strcpy(param.graph_file_name, GRAPH_FILE_FOR_NODES_36_WITH_MULTIPLE_AP_FOR_UPLOAD);
                }
				else if(param.data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
                {
                    strcpy(param.graph_file_name, GRAPH_FILE_FOR_NODES_36_WITH_MULTIPLE_AP_FOR_UPLOAD);
                }
                else
                {
					printf("%s:%d: data_forwarding_mode(%d) is not supported!\n", 
							__FUNCTION__, __LINE__,
							data_forwarding_mode);
                    exit(1);
                }
                
                /* copy mobility configuration file name */
  	        strcpy(param.mobility_file_name, MOBILITY_FILE_FOR_NODES_36_WITH_MULTIPLE_AP);
	    }
	    else
            {
	      strcpy(param.graph_file_name, GRAPH_FILE_FOR_NODES_36);
	      strcpy(param.mobility_file_name, MOBILITY_FILE_FOR_NODES_36);
	    }
	    
	    /** select a vehicle trajectory according to data forwarding mode */
		if(param.data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
	    { //packet download from AP to vehicles
	      trajectory = target_vehicle_trajectory_for_graph_with_36_nodes;
	      trajectory_size = sizeof(target_vehicle_trajectory_for_graph_with_36_nodes)/sizeof(int);
	    }
	    else if(param.data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
	    { //packet upload from vehicles to AP
	      trajectory = trajectory_for_graph_with_36_nodes;
	      trajectory_size = sizeof(trajectory_for_graph_with_36_nodes)/sizeof(int);
	    }
		else if(param.data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
	    { //packet download from AP to vehicles
	      trajectory = target_vehicle_trajectory_for_graph_with_36_nodes;
	      trajectory_size = sizeof(target_vehicle_trajectory_for_graph_with_36_nodes)/sizeof(int);
	    }

	    break;

	  case 49:
	    if(param.communication_multiple_AP_flag)
            {
                /* copy graph configuration file name */
                if(param.data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
                {
                    strcpy(param.graph_file_name, GRAPH_FILE_FOR_NODES_49_WITH_MULTIPLE_AP_FOR_DOWNLOAD);
                }
                else if(param.data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
                {
                    strcpy(param.graph_file_name, GRAPH_FILE_FOR_NODES_49_WITH_MULTIPLE_AP_FOR_UPLOAD);
                }
		else if(param.data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
                {
                    strcpy(param.graph_file_name, GRAPH_FILE_FOR_NODES_49_WITH_MULTIPLE_AP_FOR_DOWNLOAD);
                }
                else
                {
					printf("%s:%d: data_forwarding_mode(%d) is not supported!\n", 
							__FUNCTION__, __LINE__,
							data_forwarding_mode);
                    exit(1);
                }
                
                /* copy mobility configuration file name */
  	        strcpy(param.mobility_file_name, MOBILITY_FILE_FOR_NODES_49_WITH_MULTIPLE_AP);
	    }
	    else
            {
	      strcpy(param.graph_file_name, GRAPH_FILE_FOR_NODES_49);
	      strcpy(param.mobility_file_name, MOBILITY_FILE_FOR_NODES_49);
	    }
	    
	    /** select a vehicle trajectory according to data forwarding mode */
		if(param.data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
	    { //packet download from AP to vehicles
	      trajectory = target_vehicle_trajectory_for_graph_with_49_nodes;
	      trajectory_size = sizeof(target_vehicle_trajectory_for_graph_with_49_nodes)/sizeof(int);
	    }
	    else if(param.data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
	    { //packet upload from vehicles to AP
	      trajectory = trajectory_for_graph_with_49_nodes;
	      trajectory_size = sizeof(trajectory_for_graph_with_49_nodes)/sizeof(int);
	    }
		else if(param.data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
	    { //packet download from AP to vehicles
	      trajectory = target_vehicle_trajectory_for_graph_with_49_nodes;
	      trajectory_size = sizeof(target_vehicle_trajectory_for_graph_with_49_nodes)/sizeof(int);
	    }

	    break;

	  default:
	    printf("main(): there is no graph configuration file for graph_node_number=%d\n", graph_node_number);
	    exit(1);
	}

	/** get graph file name from param */
	strcpy(graph_file, param.graph_file_name);

	/** get the prefix for log files from output_file_1 */
	strcpy(output_file, output_file_1);
#ifdef _LINUX_
	ptr = rindex(output_file, '.');
#else
	ptr = right_index(output_file, '.');
#endif
	*ptr = '\0';

	/* vanet log file */
	memset(vanet_file, 0, sizeof(vanet_file));
	strcpy(vanet_file, output_file);
	strcat(vanet_file, vanet_file_suffix);

	/* vanet packet carrier trace log file */
	memset(vanet_packet_carrier_trace_file, 0, sizeof(vanet_packet_carrier_trace_file));
	strcpy(vanet_packet_carrier_trace_file, output_file);
	strcat(vanet_packet_carrier_trace_file, vanet_packet_carrier_trace_file_suffix);

#if defined(__DEBUG_INTERACTIVE_MODE__) || defined(__DEBUG_SIMULATION_TIME__)
	start_time = time(NULL);
	printf("\n*** Simulation: start_time=%d ***\n", (int)start_time);
#endif

	/**@for Normal distribution test */
        //GSL_test();
	/*************/

	/**@for Gamma distribution test */
        //GSL_test_for_gamma();
	/*************/

	/**@for test */
	//LA_Test_Gaussian_Elimination_1();
	/*************/

	/**@for test */
	//LA_Test_Gaussian_Elimination_2();
	/*************/

	/** run simulation according to a given scenario */
	run(seed, &param, graph_file, schedule_file, vanet_file, pathlist_file, output_file_1, output_file_2, trace_file_of_vehicle_convoy_length, trajectory, trajectory_size, vanet_packet_carrier_trace_file);
	
#if defined(__DEBUG_INTERACTIVE_MODE__) || defined(__DEBUG_SIMULATION_TIME__)
	end_time = time(NULL);
        printf("\n*** Simulation: start_time=%d ***\n", (int)start_time);
	printf("\n*** Simulation: end_time=%d ***\n", (int)end_time);
	computation_time = end_time - start_time;
	printf("*** Simulation: computation_time=%d ***\n\n", (int)computation_time);

	//printf("\nHit return to finish the simulation\n");
        //fgetc(stdin);
#endif
	/* taehwan 20140719 */
	/*
	int i,j,k;
	for(i=49;i>=7;i=i-7)
	{
		for(k=0;k<3;k++)
		{
			for(j=i-6;j<=i;j++)
			{
				if (k==0)
					printf("%5d %5d %5d ",0,g_segment_visit_count[j][0],0);
				else if (k==2)
					printf("%5d %5d %5d ",0,g_segment_visit_count[j][2],0);
				else 
					printf("%5d %5d %5d ",g_segment_visit_count[j][3],j,g_segment_visit_count[j][1]);
			}
			printf("\n");
		}
		printf("\n");
	}
	*/
	
	/* taehwan 20140722 */
	//print_intersection_visit_count();

	return 0;
}

/** run simulation */
int run(unsigned int seed, struct parameter *param, char *graph_file, char *schedule_file, char *vanet_file, char *pathlist_file, char *output_file_1, char *output_file_2, char *trace_file_of_vehicle_convoy_length, int *trajectory, int trajectory_size, char *vanet_packet_carrier_trace_file)
{ //run simulation according to a given scenario

#ifdef __LOG_LEVEL_TRACE_OF_VEHICLE_CONVOY_LENGTH__
  /** variables for VANET */
        int convoy_vehicle_number = 0; //number of vehicles consisting of a convoy
	double convoy_interarrival_time = 0; //interarrival time of two consecutive vehicles
	double convoy_last_arrival_time = 0; //arrival time of the last vehicle
	int convoy_last_vehicle = -1; //vehicle id arriving last
	int convoy_head = -1; //vehicle id corresponding to the convoy head
	struct_vehicle_t *convoy_ptr_to_the_following_vehicle = NULL; //pointer to the folloing vehicle in the convoy
	double convoy_start_time = 0; //start-time of a convoy
	double convoy_end_time = 0; //end-time of a convoy
	double convoy_start_length = 0; //start-length of a convoy
	double convoy_end_length = 0; //end-length of a convoy
	double convoy_threshold = param->communication_range/param->vehicle_speed; //the time that a convoy is disconnected from the intersection
	boolean flag_of_vanet_logging_start = FALSE; //flag to determine the start of the logging for the convoy length
	//boolean flag_of_vanet_logging_start = TRUE; //flag to determine the start of the logging for the convoy length
#endif

  /** variables for real graphs and virtual graphs */
	struct_graph_node *Gr = NULL; //adjacency list for a real graph representing a road network
	int Gr_size = 0; //size of real graph Gr
	struct_graph_node *pGraphNode = NULL; //pointer to graph node
	int neighbor_node_id = 0; //neighbor node id

	edge_queue_t Er; //edge queue containing the information of edges in real graph Gr
	edge_queue_node_t *pEdgeNode = NULL; //pointer to edge queue node

	directional_edge_queue_t DEr; //directional edge queue containing the directional edges in real graph Gr
	directional_edge_queue_node_t *ptr_directional_edge_node = NULL; //pointer to the directional edge node in directional edge queue DEr for real graph Gr

	/* road network data structures for the extended VADD and TBD */
	struct_graph_node **Gr_set = NULL; //the set of road network graphs where the index is the packet destination intersection from any other intersection in the target road network graph
	int *Gr_set_size = NULL; //the size of each road network graph corresponding to the index
	int Gr_set_number = 0; //the number of road network graphs in Gr_set
	directional_edge_queue_t *DEr_set = NULL; //the set of directoinal edge queues corresponding to the road network graphs

	char adjacency_matrix_file[BUF_SIZE] = ADJACENCY_MATRIX_FILE; //adjacency matrix file representing the graph of the road network
	char pathtable_file_for_move[BUF_SIZE] = PATHTABLE_FILE_FOR_MOVE; //path-table file for all-source-all-destination pair in terms of vehicle movement
	char pathtable_file_for_scan[BUF_SIZE] = PATHTABLE_FILE_FOR_SCAN; //path-table file for all-source-all-destination pair in terms of sensor scan
	struct_traffic_table src_table_for_Gr; //table for traffic sources where the vehicles arrive in real graph Gr
	struct_traffic_table dst_table_for_Gr; //table for traffic destinations where the vehicles leave in real graph Gr
	struct_traffic_table ap_table_for_Gr;  //table for access points that are placed at intersections in real graph Gr
	int ap_table_index_for_Gr = 0; //index for access point table ap_table_for_Gr; 0 indicates the first AP in the ap table
	struct_traffic_table sn_table_for_Gr;  //table for stationary nodes where play the role of relay node as temporary packet holder

	struct_access_point_t AP; //Internet Access Point (AP) for the data forwarding of download
	struct_access_point_t *pAP = NULL; //pointer to an AP
	int ap_table_index_for_target_point = 0; //index for access point table AP.ap_table_for_target_point; 0 indicates the first AP (i.e., destination vehicle) in the ap table
	access_point_queue_t APQ; //Queue of Internet Access Points
	access_point_queue_node_t *actual_transmitter_AP_qnode = NULL; //pointer to the actual transmitter AP queue node under the multiple-AP scenario
	int actual_transmitter_AP_id  = 0; //the actual transmitter AP's id, not vertex id
	char actual_transmitter_AP_vertex[NAME_SIZE]; //the actual transmitter AP's vertex name
	
	double **Dr_move = NULL; //weight matrix for all-pairs shortest paths in terms of vehicle movement in real graph Gr
	int **Mr_move = NULL; //predecessor matrix for all-pairs shortest paths in terms of vehicle movement in real graph Gr
	int matrix_size_for_movement_in_Gr = 0; //matrix size of matrices Dr_move and Mr_move for movement in Gr

	int **Ar_edd = NULL; //adjacency matrix for the connectivity based on stationary node such that one intersection with stationary node and another intersection with stationary node have the connectivity whose value is 1. If two adjacent intersections have no connectivity, the edge value is INF.
	double **Dr_edd = NULL; //weight matrix for all-pairs shortest paths in terms of E2E delivery delay or delay variance in real graph Gr
	int **Mr_edd = NULL; //predecessor matrix for all-pairs shortest paths in terms of E2E delivery delay or delay variance in real graph Gr
	double **Sr_edd = NULL; //supplementary matrix for all-pairs shortest paths in terms of E2E delivery delay or delay variance in real graph Gr
	int matrix_size_for_edd_in_Gr = 0; //matrix size of matrices Dr_edd, Mr_edd, and Sr_edd for E2E delivery delay or delay variance in Gr

	double **Wr_edc = NULL; //adjacency matrix for edge cost in road network graph
	double **Dr_edc = NULL; 
	/* weight matrix for all-pairs shortest paths in terms of 
	 * expected E2E delivery cost (e.g., delay or channel utilization, 
	   that is, the transmission number for forwarding in a road segment,
	   considering forwarding length according to vehicle arrival rate \lambda)
	   or cost variance in real graph Gr */
	int **Mr_edc = NULL; //predecessor matrix for all-pairs shortest paths in terms of expected E2E delivery cost or cost variance in real graph Gr
	double **Sr_edc = NULL; //supplementary matrix for all-pairs shortest paths in terms of expected E2E delivery cost or cost variance in real graph Gr
	int matrix_size_for_edc_in_Gr = 0; //matrix size of matrices Dr_edc, Mr_edc, and Sr_edc for expected E2E delivery cost or cost variance in Gr

#if 0 /* [ */
	/** Matrices for Partial Stationary-Node Deployment */
	double **Wr_edd_psn = NULL; //adjacency matrix for edge delay in road network graph Gr under the partial SN deployment
	double **Dr_edd_psn = NULL; //weight matrix for all-pairs shortest paths in terms of E2E delivery delay or delay variance in real graph Gr 
	int **Mr_edd_psn = NULL; //predecessor matrix for all-pairs shortest paths in terms of E2E delivery delay or delay variance in real graph Gr
	double **Sr_edd_psn = NULL; //supplementary matrix for all-pairs shortest paths in terms of E2E delivery delay or delay variance in real graph Gr
	int matrix_size_for_edd_psn_in_Gr = 0; //matrix size of matrices Dr_edd, Mr_edd, and Sr_edd for E2E delivery delay or delay variance in Gr

	double **Wr_edc_psn = NULL; //adjacency matrix for edge cost in road network graph Gr under the partial SN deployment
	double **Dr_edc_psn = NULL; 
	/* weight matrix for all-pairs shortest paths in terms of 
	 * expected E2E delivery cost (e.g., delay or channel utilization, 
	   that is, the transmission number for forwarding in a road segment,
	   considering forwarding length according to vehicle arrival rate \lambda)
	   or cost variance in real graph Gr */
	int **Mr_edc_psn = NULL; //predecessor matrix for all-pairs shortest paths in terms of expected E2E delivery cost or cost variance in real graph Gr
	double **Sr_edc_psn = NULL; //supplementary matrix for all-pairs shortest paths in terms of expected E2E delivery cost or cost variance in real graph Gr
	int matrix_size_for_edc_psn_in_Gr = 0; //matrix size of matrices Dr_edc, Mr_edc, and Sr_edc for expected E2E delivery cost or cost variance in Gr	
	/*************/
#endif /* ] */

	double max_road_segment_length = 0; //maximum road segment length
	double average_length_of_shortest_paths = 0; //average length of the shortest paths between the arbitrary pair of source and destination

	/* variables for edge information */
	int eid = 0; //edge id
	double offset = 0; //relative position from the tail node of an edge
	char *tail_node = NULL; //pointer to tail node name
	char *head_node = NULL; //pointer to head node name
	char tail_node_buf[NAME_SIZE]; //buffer for tail node name
	char head_node_buf[NAME_SIZE]; //buffer for head node name

	MOVE_TYPE move_type; //movement type for vehicle
	double edge_length = 0; //the length of the edge

	int tail_intersection_id = 0; //tail intersection id for an edge
	int head_intersection_id = 0; //head intersection id for an edge

	int physical_eid = 0; //physical edge id
	double physical_offset = 0; //relative position from the tail node of an edge
	char *physical_tail_node = NULL; //pointer to tail node name
	char *physical_head_node = NULL; //pointer to head node name

	int virtual_eid = 0; //virtual edge id
	double virtual_offset = 0; //relative position from the tail node of an edge
	char *virtual_tail_node = NULL; //pointer to tail node name
	char *virtual_head_node = NULL; //pointer to head node name

	FILE *fp_1 = NULL; //file pointer to output file 1 for simulation summary
	FILE *fp_2 = NULL; //file pointer to output file 2 for performance data after simulation

	FILE *fp_trace_file_of_vehicle_convoy_length = NULL; //file pointer to the trace file of vehicle convoy length

	int iteration_counter = 0; //number of the iterations of all the client packets
	double surveillance_start_time = 0; //the starting time of surveillance which is the starting point of sensor schedule, consisting of movement_time and scanning_time.
	double latest_death_time = 0; //the latest death time used to compute network lifetime in the case where all of sensors die
	double simulation_current_point; //simulation_current_point: time() or the value of iteration_counter
	double simulation_end_point; //simulation_end_point: param.simulation_time or param.simulation_iteration_number
	int id; //vehicle id or convoy id
	unsigned int new_simulation_node_id = 0; //a new simulation node's id, such as access point, vehicle and packet 
        boolean vehicle_id_override_flag = FALSE; //the flag to let the vehicle id replaced by new_simulation_node_id or greater numbers

	struct_vehicle_t *vehicle = NULL; //pointer to vehicle node
	struct_vehicle_t *next_carrier = NULL; //pointer to vehicle node
	struct_vehicle_t *destination_vehicle = NULL; //pointer to destination vehicle node

	double vehicle_arrival_duration = 0; //duration when vehicles arrive at road network
	double arrival_time = 0; //vehicle arrival time
	int number_of_vehicles = 0; //number of vehicles arriving at one entrance
	STATE event; //event indicating state
	STATE state; //state of sensor or vehicle
	double delay; //delay for simulation schedule
	int result = 0; //function result
	double current_time; //current simulation time
	//double working_time; //working time for one working period
	//double available_working_time; //available working time for one working period that is based on the remaining energy
	double movement_time; //movement time of vehicle
	double movement_distance; //movement distance of vehicle

	int i, j; //index for for-loop

	double offset_tolerance = OFFSET_TOLERANCE; //offset tolerance for vehicle's offset for movement
	int index; //index for table
	double diff; //difference between two double values

	struct_path_node *path_list = NULL; //list of vertices on the path from source to destination

	/* variable for scheduling vehicular traffic according to source */
	struct_schedule_table sched_table; //vehicle schedule table
	char *traffic_source = NULL; //traffic source
	char *traffic_destination = NULL; //traffic destination
	int src = 0; //integer corresponding to traffic source
	int dst = 0; //integer corresponding to traffic destination
	double distance = 0; //distance of a road segment: unit is [m]
	struct_path_table path_table; //table for shortest path sets

	/* variables for statistics */
	double *interarrival_time_sum = NULL; //accumlative interarrival time for each source
	int *arrival_number = NULL; //number of arrivals for each source
	double *previous_arrival_time = NULL; //previous arrival time for each source
	double *average_interarrival_time = NULL; //average interarrival time for each source

	int number_of_arrived_vehicles = 0; //number of vehicles that arrived at the road network

	/* statistics for vehicle speed */
        double speed_sum = 0; //sum of speeds for vehicle's speed mean in the speed statistics
	double speed_square_sum = 0; //sum of speed squares for vehicle's speed 2nd moment in the speed statistics
	double speed_mean = 0; //speed mean
	double speed_2nd_moment = 0; //the 2nd moment of speed
	double speed_variance = 0; //speed variance
	double speed_standard_deviation = 0; //speed standard deviation

	/* statistics for vehicle movement time */
	double movement_time_sum = 0; //sum for vehicle's movement time mean in the movement time statistics
	double movement_time_square_sum = 0; //sum of speed squares for vehicle's speed 2nd moment in the speed statistics
	double movement_time_mean = 0; //movement time mean
	double movement_time_2nd_moment = 0; //the 2nd moment of movement time
	double movement_time_variance = 0; //movement time variance
	double movement_time_standard_deviation = 0; //movement time standard deviation

	/* statistics for packet delivery statistics */
	packet_delivery_statistics_t packet_delivery_statistics;

	/* variable in logging for detection missing or duplicate detection */
	int log_increment = 0; //number of new logged entries: 0: detection error, 1: normal detection, and 2: duplicate detection

	int seq_of_surveillance_log = 0; //sequence number of localization log 
	boolean flag = FALSE; //flag for propositional statement
	boolean simulation_termination_flag = FALSE; //flag to terminate the simulation because all of sensors are out of energy.

	/* variables for VADD simulation */
	double angle = 0; //angle
	int packet_vehicle_count = 0; //count for the number of packet generating vehicles
	int number_of_generated_packets = 0; //number of generated packets
	packet_queue_node_t packet; //packet for data delivery
	packet_queue_node_t *pPacket = NULL; //pointer to a packet queue node

	global_packet_queue_node_t global_packet; //global packet to forwarding management of multiple packet copies
	global_packet_queue_node_t *pGlobalPacket = NULL; //pointer to a global packet queue

#if GLOBAL_PACKET_TRACE_FLAG
	global_packet_queue_node_t *pTrace_GlobalPacket = NULL; //pointer to a global packet queue
#endif

	struct_graph_node *ap_graph_node = NULL; //pointer to the graph node corresponding to an Internet access point
	int ap_id = 0; //Internet access point's ID; note that in this version, we assume that there exists only one AP, however, later we will accommodate the multiple-AP scenario.
	boolean flag_packet_log = FALSE; //flag for letting vehicle perform packet forwarding

	convoy_queue_node_t *convoy = NULL; //pointer to a convoy queue node
	boolean flag_first_convoy_member = FALSE; //flag to indicate whether the vehicle is the first vehicle in its convoy

	/* variables for TSF simulation */
	//struct_coordinate3_t target_point; //target point to deliver a packet to a target vehicle
	int target_point_id; //id of target point (i.e., intersection id on the vehicle trajectory) used to deliver a packet towards a target vehicle
	char target_point[NAME_SIZE]; //string of target point id
	//int target_vehicle_id; //packet destination vehicle
	double 	delta_x = 0; //the change in x-coordinate
	double 	delta_y = 0; //the change in y-coordinate
	struct_graph_node *tail_gnode_in_node_array = NULL; //pointer to the graph node corresponding to the tail node in the node array of Gr
	intersection_area_type_t intersection_area_type = INTERSECTION_AREA_UNKNOWN; //Intersection area type
	int AP_vehicle_number = 0; //the number of vehicles passing AP aperiodically

	destination_vehicle_queue_t DVQ; //destination vehicle queue
	destination_vehicle_queue_node_t *pDestinationVehicleQueueNode = NULL; //pointer to a destination vehicle queue node

	double EDD_p = 0; //packet's Expected Delivery Delay (EDD) for target point p
	double EAD_p = 0 ; //destination vehicle's Expected Arrival Delay (EAD) for target point p

	forwarding_table_queue_t FTQ; //Forwarding Table Queue that has a table for forwarding towards each intersection in the road network graph Gr

	global_packet_queue_t GPQ; //Global Packet Queue that contains the valid packets delivered towards destination vehicle whose TTLs do not expire

	char packet_src_vertex[NAME_SIZE]; //packet source vertex
	char packet_dst_vertex[NAME_SIZE]; //packet destination vertex

	stationary_node_queue_t SNQ; //Stationary Node Queue that contains the stationary nodes placed at intersections

	int static_packet_trajectory[] = {25, 18, 11, 4}; //static packet trajectory for investigating the delay distribution (per-hop distribution and per-path distribution) in a graph with 49 nodes

	int static_packet_trajectory_size = sizeof(static_packet_trajectory)/sizeof(int); //size of static packet trajectory

	int destination_vehicle_id = 0; //destination vehicle id

	double travel_distance = 0; //the travel distance of the destination vehicle on its vehicle trajectory
	double travel_time = 0; //the mean travel time of the destination vehicle on its vehicle trajectory
	double travel_time_deviation = 0; //the travel time deviation of the destination vehicle on its vehicle trajectory

	boolean intersection_visit_flag = FALSE; //flat to indicate that vehicle arrices at an intersection

	int target_point_number = 0; //the number of target points
	target_point_queue_t TPQ; //target point queue containing target points towards which packets will be sent
	
#ifdef BSMA_20150227
	// taehwan 20150227 
	target_point_queue_t FTPQ; //feasible target point queue containing target points towards which packets will be sent to destination vehicles in multicast
	edge_set_queue_t ESQ; //edge set queue for multicast tree T after filtering out redundant target points
	/** Initialize Feasible Target Point Queue FTPQ */
	InitQueue((queue_t*) &FTPQ, QTYPE_TARGET_POINT);

	/** Initialize edge set queue ESQ containing edges in a multicast tree */
	InitQueue((queue_t*)&ESQ, QTYPE_EDGE_SET);
#endif

	target_point_queue_node_t *pTargetPoint = NULL; //pointer to a target point queue node
	int global_packet_id = 0; //a globally unique packet id set to packet's id

	/* variables for TPD simulation */
	int forward_count = 0; //count for forwarded packets
	int discard_count = 0; //count for discarded packets

	/* taehwan 20140712 checking range flag */
	bool isInRange = FALSE;
	/*************************************************************************************************/

	/** @for debugging */
	//LA_Test_Gaussian_Elimination_1();
	//Test-1: test Gaussian Elimination function and Backward Substitution function

	//LA_Test_Gaussian_Elimination_2();
	//Test-2: test Gaussian Elimination function and Backward Substitution function

	//LA_Test_Gaussian_Elimination_3();
	//Test-3: test Gaussian Elimination function and Backward Substitution function

	/** set the seed for rand() */
	if(param->simulation_seed == SEED_TIME)
	  srand48((unsigned)time(NULL));
	else if(param->simulation_seed == SEED_DEFAULT)
	  srand48(seed);
	else
	{
		printf("run(): param->simulation_seed of type %d is not supported!\n", param->simulation_seed);
#ifdef __DEBUG_INTERACTIVE_MODE__
                fgetc(stdin);
#endif
		exit(1);
	}

	/** open log and trace files **/
	/** open output_file_1 for simulation summary */
	fp_1 = fopen(output_file_1, "w");
	if(!fp_1)
	{
	    fprintf(stderr, "Error: unable to open file \"%s\"\n", output_file_1);
#ifdef __DEBUG_INTERACTIVE_MODE__
	    fgetc(stdin);
#endif
	    exit(1);
	}
	else
	    sendto(fp_1); //redirect simulation messages to output file

        /** open output_file_2 for simulation summary */
	fp_2 = fopen(output_file_2, "w");
	if(!fp_2)
	{
	    fprintf(stderr, "Error: unable to open file \"%s\"\n", output_file_2);
#ifdef __DEBUG_INTERACTIVE_MODE__
	    fgetc(stdin);
#endif
	    exit(1);
	}

#if defined(__LOG_LEVEL_VANET_PACKET_DROP__) || defined(__LOG_LEVEL_VANET_PACKET_AP_ARRIVAL__) || defined(__LOG_LEVEL_VANET_PACKET_DESTINATION_VEHICLE_ARRIVAL__)
	/* open a file pointer for VANET log file */
	open_vanet_file(vanet_file);
#endif

#if defined(__LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__) || defined(__LOG_LEVEL_VANET_PACKET_CARRIER_TRACE_FOR_STATIONARY_NODE__)
	/* open a file pointer for VANET packet carrier trace log file */
	open_vanet_packet_carrier_trace_file(vanet_packet_carrier_trace_file);
#endif
	
#ifdef __LOG_LEVEL_PATH_LIST__
	/* open a file pointer for path-list log file */
	open_path_list_file(pathlist_file);
#endif

	/** open trace files for observing system behavior */
#ifdef __LOG_LEVEL_TRACE_OF_VEHICLE_CONVOY_LENGTH__
	fp_trace_file_of_vehicle_convoy_length = open_trace_file(trace_file_of_vehicle_convoy_length);
#endif


        /** initialize packet delivery statistics */
        memset(&packet_delivery_statistics, 0, sizeof(packet_delivery_statistics));

/* 	/\** initialize vehicle mobility for destination vehicle(s) under download mode *\/ */
/* 	if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD) */
/* 	{ //initialize queue DVQ for the packet download from AP to vehicles */
/* 	  init_mobility(param->mobility_file_name, &DVQ); */
/* 	} */
/* 	else */
/* 	{ //initialize queue DVQ; [7/23/2009] Up to now, DVQ is not used under upload mode */
/* 	  InitQueue((queue_t*) &DVQ, QTYPE_DESTINATION_VEHICLE); */
/* 	} */

	/** initialize an adjacency list representing a real graph */
	Gr = Initialize_Graph(param, graph_file, &Gr_size, &src_table_for_Gr, &dst_table_for_Gr, &ap_table_for_Gr, &sn_table_for_Gr, &max_road_segment_length);

	/** specify which intersections have their stationary node as temporary packet holder in the road network graph Gr */
	if(param->communication_multiple_SN_flag)
	{
		Init_RoadNetworkGraphNode_StationaryNodeFlag_With_StationaryNodeList(param, Gr, Gr_size, &sn_table_for_Gr);
	}
	else
	{ //set the stationary_node_flags of graph nodes to TRUE
		Set_RoadNetworkGraphNode_StationaryNodeFlag(Gr, Gr_size);
	}

	/** adjust the number of multiple APs according to the number of maximum APs */
	if(param->communication_multiple_AP_flag)
	{
		Adjust_Maximum_AccessPoint_Number(param, &ap_table_for_Gr);
	}

	/** construct an edge queue Er containing the information of edges in real graph Gr. When sensor_density is not uniform, the density of each edge is determined by sensor_density_distribution, sensor_density_standard_deviation and sensor_density_maximum_deviation. */
	ConstructEdgeQueue(&Er, Gr, Gr_size, param);

	/** associate each pair of two adjacent graph nodes (i.e., physical edge) with the corresponding edge entry in Er */
	AssociateGraphEdgeWithEdgeEntry(Gr, Gr_size, &Er);

	/** construct a directional edge queue DEr containing the information of directional edges in real graph Gr */
	ConstructDirectionalEdgeQueue(param, &DEr, Gr, Gr_size);

	/** associate each pair of two adjacent graph nodes (i.e., physical edge) with the corresponding directional edge entry in DEr */
	AssociateGraphEdgeWithDirectionalEdgeEntry(Gr, Gr_size, &DEr);

	/** construct the set of road network graphs and the set of the corresponding directional edge queues, and then associate these two sets */
	ConstructRoadNetworkGraphSet_And_DirectionalEdgeQueueSet_With_SetAssociation(param, Gr, Gr_size, &Gr_set_number, &Gr_set, &Gr_set_size, &DEr_set);


#ifdef  __STORE_FILE_FOR_DATA_STRUCT__
	Store_Graph_Into_File_As_AdjacencyMatrix(Gr, Gr_size, adjacency_matrix_file);
	//store the adjacency list of graph Gr into a file in the form of adjacency matrix
#endif


	/** allocate the matrices for movement in Gr */
	matrix_size_for_movement_in_Gr = Gr_size;
	Floyd_Warshall_Allocate_Matrices_For_Movement(&Dr_move, &Mr_move, matrix_size_for_movement_in_Gr);

	/** construct the shortest path weight matrix Dr_move and predecessor matrix Mr_move in terms of vehicle movement */
	Floyd_Warshall_Construct_Matrices_For_Movement(Gr, Gr_size, &Dr_move, &Mr_move, &matrix_size_for_movement_in_Gr);

#ifdef  __STORE_FILE_FOR_DATA_STRUCT__
	Floyd_Warshall_Store_Weight_Table_Into_File(Dr_move, Gr_size, pathtable_file_for_move);
	//store the two-dimensional matrix D_move into a file called pathtable.dat
#endif

	/* allocate the matrices for Expected Delivery Delay (EDD) in G */
	matrix_size_for_edd_in_Gr = Gr_size;
	Floyd_Warshall_Allocate_Matrices_For_EDD(&Dr_edd, &Mr_edd, &Sr_edd, matrix_size_for_edd_in_Gr);

	/* make the connectivity adjacency matrix Ar_edd for Gr */
	Floyd_Warshall_Init_Connectivity_Adjacency_Matrix_For_EDD(param, Gr, Gr_size, &sn_table_for_Gr, &Ar_edd, matrix_size_for_edd_in_Gr);

	/** allocate the matrices for Expected Delivery Cost (EDC) in G */
	matrix_size_for_edc_in_Gr = Gr_size;
	Floyd_Warshall_Allocate_Matrices_For_EDC(&Wr_edc, &Dr_edc, &Mr_edc, &Sr_edc, matrix_size_for_edc_in_Gr);

	/* intialize path table including the shortest path information for all the sources in src_table */
	Initialize_Path_Table(Gr, Gr_size, &src_table_for_Gr, &path_table);

	/** Initialize Stationary Node Queue. 
		Note that InitStationaryNodeQueue() should be called before InitForwardingTableQueue() 
		in order that the pointers to stationary nodes can be set to graph nodes. */
	InitStationaryNodeQueue(&SNQ, Gr, Gr_size);
	
	/** Initialize Forwarding Table Queue */
	InitForwardingTableQueue(&FTQ, param, Gr, Gr_size);
	//initialize the forwarding table for each intersection in the road network graph Gr

	/** Initialize Global Packet Queue */
	/* initialize Global Packet Queue with Vanet Information Table */
	InitGlobalPacketQueue_With_VanetInformationTable(&GPQ, param);

	/* initialize packet vectors for packet delivery statistics under multiple packet copy transmission for multiple target points */
	Initialize_GlobalPacketQueue_PacketVectors(param, &GPQ, INITIAL_PACKET_VECTOR_SIZE);

	/** Create the vanet information table containing the data structures used in the data forwarding in the VANET, such as road network graph Gr, the movement shortest path matrices, the EDD shortest path matricesm, the EDC shortest path matrices, the forwarding table queue FTQ, etc. */
	create_vanet_information_table_in_parameter(param, Gr, Gr_size, Dr_move, Mr_move, matrix_size_for_movement_in_Gr, Ar_edd, Dr_edd, Mr_edd, Sr_edd, matrix_size_for_edd_in_Gr, Wr_edc, Dr_edc, Mr_edc, Sr_edc, matrix_size_for_edc_in_Gr, &FTQ);

	/** Initialize Target Point Queue TPQ */
	InitQueue((queue_t*) &TPQ, QTYPE_TARGET_POINT);

	/*** Simulation Parts ***/

	/** initialize the statistics */
        interarrival_time_sum = (double*) calloc(src_table_for_Gr.number, sizeof(double)); //accumlative interarrival time for each source
	assert_memory(interarrival_time_sum);

	arrival_number = (int*) calloc(src_table_for_Gr.number, sizeof(int)); //number of arrivals for each source
	assert_memory(arrival_number);

	previous_arrival_time = (double*) calloc(src_table_for_Gr.number, sizeof(double)); //previous arrival time for each source
	assert_memory(previous_arrival_time);

        average_interarrival_time = (double*) calloc(src_table_for_Gr.number, sizeof(double)); //average interarrival time for each source
	assert_memory(average_interarrival_time);

	/** initialize SMPL simulator */
	smpl(0, "VANET Model");

	/** turn tracing on with level 2 */
#ifdef _TRACE_
	trace(2);
#endif


	/** initialize vehicle_list by let vehicle_list.next and vehicle_list.prev pointing to itself */
	init_vehicle_list();

	/** initialize vehicle schedule table  */
	Initialize_Schedule_Table(&src_table_for_Gr, &sched_table);

	/** schedule the event to build the Expected Delivery Delay (EDD) */
	id = 0;
	delay = param->vehicle_edd_update_period;
	schedule(VANET_EDD_UPDATE, delay, id);

	/** set the initial vehicle id according to the data forwarding mode;
	    if the data forwaring mode is DATA_FORWARDING_MODE_DOWNLOAD, schedule the AP_START event.
        */
	if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD ||
			param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
	{ //packet download from AP to vehicles
	  /* initialize access point queue APQ */
	  InitQueue((queue_t*) &APQ, QTYPE_ACCESS_POINT);
	  
	  /* initialize access point queue node AP */
	  memset(&AP, 0, sizeof(access_point_queue_node_t));

      ap_id = 1; //access point id

	  /* initialize AP structure */
	  for(i = 0; i < ap_table_for_Gr.number; i++)
	  {
	    /* enqueue the access point queue node for AP into queue APQ */
	    pAP = (access_point_queue_node_t*) Enqueue((queue_t*) &APQ, (queue_node_t*) &AP);

	    AP_Init(pAP, param, Gr, Gr_size, ap_table_for_Gr.list[i].vertex, ap_id);
	    //initialize the AP by setting the location in the road network with G and also by setting AP's vertex and id name with ap_vertex and ap_id, respectively 
 
	    /* schedule AP_START for the data forwarding operation in AP */
            delay = param->communication_AP_packet_generation_schedule_time; //Just after the edd update period, perform the event AP_START
	    //delay = param->vehicle_edd_update_period; //Just after the edd update period, perform the event AP_START
	    schedule(AP_START, delay, ap_id++);	    
	  }

	  //new_simulation_node_id = ap_id; //vehicle id for a new vehicle arrival
	}
	else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
	{ //packet upload from vehicles to AP
	  //new_simulation_node_id = 1; //vehicle id for a new vehicle arrival

	  /* initialize access point queue APQ */
	  InitQueue((queue_t*) &APQ, QTYPE_ACCESS_POINT);
	  
	  ap_id = 1; //access point id

	  /* initialize AP structure */
	  for(i = 0; i < ap_table_for_Gr.number; i++)
	  {
	    /* enqueue the access point queue node for AP into queue APQ */
	    pAP = (access_point_queue_node_t*) Enqueue((queue_t*) &APQ, (queue_node_t*) &AP);

	    AP_Init(pAP, param, Gr, Gr_size, ap_table_for_Gr.list[i].vertex, ap_id);
	    //initialize the AP by setting the location in the road network with G and also by setting AP's vertex and id name with ap_vertex and ap_id, respectively 
 	    ap_id++;	    
	  }

	  //new_simulation_node_id = ap_id; //vehicle id for a new vehicle arrival
          
          /** set param->vehicle_vanet_stationary_vehicle_id if param->vehicle_vanet_stationary_vehicle_flag is set */
          if(param->vehicle_vanet_stationary_vehicle_flag)
              param->vehicle_vanet_stationary_vehicle_id = new_simulation_node_id;
	}

	/** initialize vehicle mobility for destination vehicle(s) under download mode and let the destination vehicle's id start from the available lowest id for new_simulation_node_id */
	new_simulation_node_id = 1; //let the first destination vehicle have vid of 1.
	if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD ||
			param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
	{ //initialize queue DVQ for the packet download from AP to vehicles
          vehicle_id_override_flag = TRUE; //let the vehicle id replaced by new_simulation_node_id or greater numbers
	  init_mobility(param->mobility_file_name, vehicle_id_override_flag, new_simulation_node_id, &DVQ);
	}
	else
	{ //initialize queue DVQ; [7/23/2009] Up to now, DVQ is not used under upload mode
	  InitQueue((queue_t*) &DVQ, QTYPE_DESTINATION_VEHICLE);
	}

	/* setup of param->vehicle_initial_arrival_time:
           If param->vehicle_initial_arrival_time is equal to -1, then it is set from the vehicle movement time for the shortest path. */
	if(param->vehicle_initial_arrival_time == -1)
	  param->vehicle_initial_arrival_time = 0; //vehicle_initial_arrival_time is set to zero

	/** register vehicles into SMPL scheduler */
	/* register suveillance_start_time with param->vehicle_initial_arrival_time */
	surveillance_start_time = param->vehicle_initial_arrival_time; //the starting time of surveillance which is the starting point of sensor schedule, consisting of movement_time and scanning_time.

	for(i = 0; (i < src_table_for_Gr.number) && (number_of_arrived_vehicles < param->vehicle_maximum_number); i++)
	{
		//set the initial vehicle arrival time
		//delay = param->vehicle_initial_arrival_time;
		//@let a vehicle arrive at the initial sleeping time
		//delay = initial_sleeping_time;
		//delay = delay_func(param, DELAY_VEHICLE_INTERARRIVAL_TIME);

		if(param->vehicle_interarrival_time_distribution == UNIFORM)
		{
			vehicle_arrival_duration = MAX(param->simulation_time - param->vehicle_initial_arrival_time, 0);
			number_of_vehicles = (int) (vehicle_arrival_duration/param->vehicle_interarrival_time); //number of vehicles arriving at one entrance
			//traffic_source = src_table_for_Gr.list[i];
			traffic_source = src_table_for_Gr.list[i].vertex;

			for(j = 0; j < number_of_vehicles; j++)
			{
				/* determine vehicle arrival time from uniform distribution */
				arrival_time = dist_func(param->vehicle_interarrival_time_distribution, param->vehicle_initial_arrival_time, param->simulation_time);

				/* register a new vehicle to vehicle list */
				register_vehicle(new_simulation_node_id, traffic_source, arrival_time, &dst_table_for_Gr, &ap_table_for_Gr, param, &path_table, Gr, Gr_size, &Er, Dr_move, Mr_move);

				/* set up the first vehicle arrival time on an entrance in schedule table entry corresponding to the entrance */
				//if(sched_table.entry[i].sched_time > delay)
				//	sched_table.entry[i].sched_time = delay;

				//sched_table.entry[i].sched_time = arrival_time;
				schedule(VEHICLE_ARRIVE, arrival_time, new_simulation_node_id);

				new_simulation_node_id++; //increment new_simulation_node_id
			}
		}
		else
		{
			/* select the vehicle's start intersection randomly among the source intersections */
			index = smpl_random(0, src_table_for_Gr.number-1);
			traffic_source = src_table_for_Gr.list[index].vertex;
			//traffic_source = src_table_for_Gr.list[i].vertex;

			/* set the initial vehicle arrival time */
			arrival_time = param->vehicle_initial_arrival_time;

			/* register a new vehicle to vehicle list */
			register_vehicle(new_simulation_node_id, traffic_source, arrival_time, &dst_table_for_Gr, &ap_table_for_Gr, param, &path_table, Gr, Gr_size, &Er, Dr_move, Mr_move);

			sched_table.entry[i].sched_time = arrival_time;
			schedule(VEHICLE_ARRIVE, arrival_time, new_simulation_node_id);		
			new_simulation_node_id++;
			number_of_arrived_vehicles++;
		}
	}

	/** initialize statistics variables */
	
	if(param->simulation_mode == TIME)
	{
	    simulation_end_point = param->simulation_time;
	}
	else
	{
	    iteration_counter = 0; //number of the iterations of all the client packets
	    simulation_end_point = param->simulation_iteration_number;
	}

	printf("\nSIMULATION STARTED TO %f (TIME MODE)\n",simulation_end_point);

	/** actual simulation procedure including task scheduling */
	do
	{
		/* taehwan 20140828 terminate when packet delivery number >= packet count */
		if (param->communication_packet_maximum_number <= 
				packet_delivery_statistics.delivered_packet_number + packet_delivery_statistics.discarded_packet_number) 
		{
			printf("********************************************************\n");
			printf("Simulation Terminate : All Packet Forwarding COMPLETED!!\n");
			printf("********************************************************\n");
			break;
		} 
		
		//printf("SIMULATION ITERATION NUMBER is %d\n",iteration_counter);
		if(param->simulation_mode == TIME)
		{
			simulation_current_point = smpl_time();
			if((simulation_termination_flag == TRUE) || (simulation_current_point >= simulation_end_point))
			{
				printf("\nSIMULATION TERMINATED AT %f (TIME MODE)\n",simulation_end_point);
				break; // main loop break
			}
		}
		else
		{
			simulation_current_point = (real) iteration_counter;
			if((int) simulation_current_point >= (int) simulation_end_point)
			{
				printf("\nSIMULATION TERMINATED AT %f (COUNT MODE)\n",simulation_end_point);
				break; // main loop break
			}
		}
		//printf("SIMULATION END is %f and CURRENT is %f\n",simulation_end_point,simulation_current_point);
		iteration_counter++;

		result = cause((int*)&event, &id);
		if(result != 0)
		{
			printf("\nSIMULATION TERMINATED (NO MORE EVENT)\n\n");
			break; // main loop break
		}

		/**@for debugging */
		/*
		if((simulation_current_point > 4244.79 && event == PACKET_ARRIVE) || (simulation_current_point > 4248 && event == PACKET_ARRIVE))
		  printf("debugging point: time=%f\n", (float)simulation_current_point);
		*/

		//if(simulation_current_point > 3702.9 && id == 2)
		//  printf("debugging point: time=%f\n", (float)simulation_current_point);
		/******************/


		switch(event)
		{ // do not put release() and request() together in an event handler

/********** VANET STATES ***********/
			case VANET_EDD_UPDATE: 
				/** process the vehicular traffic statistics for building both the forwarding probability and the Expected Delivery Delay (EDD) */
				current_time = smpl_time();			
			

#ifdef __DEBUG_LEVEL_VANET_EDD_UPDATE__
                                printf("[%.2f] VANET_EDD_UPDATE: EDD Table has been updated!\n", (float)current_time);
#endif
				/** update the forwarding table according to data_forwarding_mode */
				if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD || param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD || param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
				{ /* Update forwarding table Gr under upload mode */
					if(param->vehicle_vanet_edd_computation_model == VANET_EDD_COMPUTATION_BASED_ON_STOCHASTIC_MODEL)
					{
						printf("VANET_EDD_COMPUTATION_BASED_ON_STOCHASTIC_MODEL\n");
						/*@ Note that for the multiple-AP network, the EDDs for the multiple APs are computed only under the Upload forwarding mode. */
						//if((param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD) && (ap_table_for_Gr.number > 1))
						if(ap_table_for_Gr.number > 1) //the case where the number of APs is more than 1
							VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model_For_Multiple_APs(param, Gr, Gr_size, &DEr, &ap_table_for_Gr); //compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) based on the stochastic model with graph Gr and directional edge queue DEr.
						else //the case where the number of APs is one
							VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model(param, Gr, Gr_size, &DEr, &ap_table_for_Gr, 0); //compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) based on the stochastic model with graph Gr and directional edge queue DEr.
					}
					else
					{
						printf("VANET_EDD_COMPUTATION_BASED_ON_SHORTEST_PATH_MODEL\n");
						//printf("\n*******  VANET_EDD_EDR_UPDATE : AP number = %d********* \n\n",ap_table_for_Gr.number);
						if(ap_table_for_Gr.number > 1) //the case where the number of APs is more than 1						
						{ //Even for multiple APs, it is enough to update the EDDs for one AP because the update is related to all-pairs-shortest path tables D, S, and M in param.vanet_information_table; this update lets the individual vehicles be able to construct a convoy on the same road segment by letting them set their EDD towards the intersection having the AP of index 0 
							VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model(param, Gr, Gr_size, &DEr, &ap_table_for_Gr, 0, FALSE); //compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) based on the shortest path model with graph Gr and directional edge queue DEr.
							//VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model_For_Multiple_APs(param, Gr, Gr_size, &DEr, &ap_table_for_Gr); //For the Multiple-AP road network, compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) based on the shortest path model with graph Gr and directional edge queue DEr.

							VADD_Compute_EDC_And_EDC_SD_Based_On_Shortest_Path_Model(param, Gr, Gr_size, &DEr, &ap_table_for_Gr, 0, FALSE); //compute the Expected Delivery Cost (EDC) and Expected Delivery Cost Standard Deviation (EDC_SD) based on the stochastic model with graph Gr and directional edge queue DEr.

							//printf("run(): Error: VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model_For_Multiple_APs() is not implemented yet!\n");
							//exit(1);

						}
						else
						{ //the case where the number of APs is one						
							
							VADD_Compute_EDD_And_EDD_SD_Based_On_Shortest_Path_Model(param, Gr, Gr_size, &DEr, &ap_table_for_Gr, 0, FALSE); //compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) based on the shortest path model with graph Gr and directional edge queue DEr; this update lets the individual vehicles be able to construct a convoy on the same road segment by letting them set their EDD towards the intersection having the AP of index 0 

							VADD_Compute_EDC_And_EDC_SD_Based_On_Shortest_Path_Model(param, Gr, Gr_size, &DEr, &ap_table_for_Gr, 0, FALSE); //compute the Expected Delivery Cost (EDC) and Expected Delivery Cost Standard Deviation (EDC_SD) based on the stochastic model with graph Gr and directional edge queue DEr.
						}
					}
					
					/** compute EDD and EDD_SD for VADD, TBD, Epidemic Routing so that they can work for the V2V data delivery */
					if((param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V) && (param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD || param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC || param->vanet_forwarding_scheme == VANET_FORWARDING_TADB))
					{
						VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model_For_V2V_Data_Delivery(param, Gr, Gr_size, Gr_set, Gr_set_size, DEr_set);
					}

					/** register the probability and statistics related to the data forwarding with the probability-and-statistics queue per directed edge in DEr */
					if(param->forwarding_probability_and_statistics_flag)
					{
						RegisterProbabilityAndStatistics_Per_DirectionalEdge(param, current_time, Gr, Gr_size, &DEr);
					}
                                  
					/** update the vehicles' EDDs and EDD_SDs */
					if(flag_packet_log == FALSE)
					{ //first-time update of EDD
						if(param->vehicle_vanet_acl_measurement_flag != TRUE)
							register_all_vehicle_movement(param, current_time, Gr); //register all vehicles' movement into vehicle movement lists in directional edge queue and sort each vehicle movement list in the descending order of offset

						update_all_vehicle_edd_and_edd_sd(current_time, param, Gr, Gr_size, &ap_table_for_Gr); //update all vehicles' EDDs and EDD_SDs

						/* construct a convoy of size one with this vehicle */
						if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY)
						{
							convoy_all_vehicle_join(param, current_time, Gr, Gr_size, &packet_delivery_statistics); //join all vehicles into its own convoys
						}

						flag_packet_log = TRUE; //set flag_packet_log to TRUE in order to let vehicle perform packet forwarding
					}
					else
					{
						update_all_vehicle_edd_and_edd_sd(current_time, param, Gr, Gr_size, &ap_table_for_Gr); //update all vehicles' EDDs and EDD_SDs
					}
				}
	
				/*******************************************************************/
				if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD || param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
				//if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
				{ /* Update forwarding table queue FTQ under download mode */
					UpdateForwardingTableQueue(&FTQ, param, Gr, Gr_size, &APQ);
					//update the forwarding table for each intersection in the road network graph Gr with the vehicular traffic statistics for Gr
				}

				delay = param->vehicle_edd_update_period;
				schedule(VANET_EDD_UPDATE, delay, id);


#ifdef  __LOG_FILE_FOR_VANET_ROAD_NETWORK_GRAPH__
				Store_Graph_Into_File_As_AdjacencyList_With_VANET_Statistics(param, Gr, Gr_size, VANET_TYPE_1_STATISTICS_ROAD_GRAPH_FILE, 1); //store the adjacency list of graph G into a file for VANET type-1 statistics (i.e., branching probability) in the form of adjacency list with indicator that determines the title text

				Store_Graph_Into_File_As_AdjacencyList_With_VANET_Statistics(param, Gr, Gr_size, VANET_TYPE_2_STATISTICS_ROAD_GRAPH_FILE, 2); //store the adjacency list of graph G into a file for VANET type-2 statistics (i.e., mean interarrival time) in the form of adjacency list with indicator that determines the title text

				Store_Graph_Into_File_As_AdjacencyList_With_VANET_Statistics(param, Gr, Gr_size, VANET_TYPE_3_STATISTICS_ROAD_GRAPH_FILE, 3); //store the adjacency list of graph G into a file for VANET type-3 statistics (i.e., mean interarrival time) in the form of adjacency list with indicator that determines the title text

				Store_Graph_Into_File_As_AdjacencyList_With_VANET_Statistics(param, Gr, Gr_size, VANET_TYPE_4_STATISTICS_ROAD_GRAPH_FILE, 4); //store the adjacency list of graph G into a file for VANET type-4 statistics (i.e., mean interarrival time) in the form of adjacency list with indicator that determines the title text
#endif

			break;

/********** PACKET STATES ***********/
		case PACKET_ARRIVE:
				current_time = smpl_time();

				/** control the number of generated packets */
				if((param->communication_packet_maximum_number != INF) && (number_of_generated_packets == param->communication_packet_maximum_number))
				{
				  //printf("at time=%f, the packet generation from vid=%d stops!\n", (float)current_time, id);
				  break;
				}

				vehicle = vehicle_search(id);
				if(vehicle == NULL)
				{
				  if(param->vehicle_vanet_vehicular_traffic_model == VANET_VEHICULAR_TRAFFIC_OPEN_NETWORK) //when the vehicular traffic model is open, vehicle exits the road network, so the packet generation related to this vehicle stops.
				  {
				    break;
				  }
				  else
				  {
				    printf("case PACKET_ARRIVE: there is no vehicle node corresponding to id %d\n", id);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                    fgetc(stdin);
#endif
				    exit(1);
				  }
				}

				/* obtain the pointer to the first access point */
				pAP =  GetFirstAccessPoint(&APQ);
				if(pAP == NULL)
				{
				  printf("After GetFirstAccessPoint(), pAP is NULL!\n");
				  exit(1);
				}

				/* update the packet delivery statistics */
			    number_of_generated_packets++;
				packet_delivery_statistics.generated_packet_number++;
				packet_delivery_statistics.generated_packet_copy_number++;

				/* initialize a packet and insert it into the vehicle's packet queue  */
				memset(&packet, '\0', sizeof(packet));

				packet.id = new_simulation_node_id++; //set a globally unique id to packet's id
				packet.state = PACKET_ARRIVE;
				packet.state_time = current_time;
                                
				packet.data_forwarding_mode = DATA_FORWARDING_MODE_UPLOAD;
				packet.seq = ++(vehicle->seq); //increase the sequence number for a new packet generated by this vehicle
				packet.size = param->communication_data_packet_size;

				packet.src_node_type = VANET_NODE_VEHICLE;
				packet.src_id = id;
				packet.dst_node_type = VANET_NODE_AP;
				packet.dst_id = pAP->id;
				packet.carry_src_id = id;
				packet.carry_dst_id = id;
				//packet.target_dst_ap_id = atoi(ap_table_for_Gr.list[0].vertex);

                                /* determine the packet's target point */
				packet.target_point_id = atoi(pAP->vertex); //target point is the AP's intersection id
                                /* set packet.carrier_vnode to the packet carrier, i.e., vehicle */
                                packet.carrier_vnode = vehicle;

				packet.expected_delivery_delay = vehicle->EDD; //set vehicle's EDD to packet's EDD
				packet.expected_delivery_delay_standard_deviation = vehicle->EDD_SD; //set vehicle's expected delivery delay standard deviation (EDD_SD) to packet's expected delivery delay standard deviation
                                packet.actual_delivery_delay = 0;

				packet.ttl = param->communication_packet_ttl;
				packet.generation_time = current_time;
				packet.last_receive_time = current_time;
				pPacket = (packet_queue_node_t*) Enqueue((queue_t*)vehicle->packet_queue, (queue_node_t*)&packet);

                                /******************************************************/
#ifdef  __LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__
				/** enqueue the packet carrier trace entry into packet's carrier trace queue */
				Enqueue_CarrierTraceEntry(param, current_time, pPacket, VANET_NODE_VEHICLE, (void*)vehicle);
#endif

				/** enqueue packet into global packet queue GPQ */
				pGlobalPacket = Enqueue_Packet_Into_GlobalPacketQueue(param, current_time, &GPQ, pPacket);

				/** let packet's global_packet point to pGlobalPacket */
				pPacket->global_packet = pGlobalPacket;

                                /******************************************************/

				/** schedule next packet for the vehicle with id */
				delay = delay_func(param, DELAY_PACKET_INTERARRIVAL_TIME);
				schedule(PACKET_ARRIVE, delay, id);
				
/* 				/\** let the packet holder vehicle check whether to be able to send its packets to a neighboring vehicle: Note that this probing can save the waiting time for packet transimission to the next carrier *\/ */
/* 				flag = VADD_Is_There_Next_Carrier_On_Road_Segment(param, current_time, vehicle, Gr, Gr_size, &next_carrier); */
/*                                 if(flag) //if-1 */
/*                                 { */
/*                                   VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier, &packet_delivery_statistics); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier */
/*                                 } //end of if-1 */

				/* The packet holder vehicle (i.e., vehicle) tries to send its packets including the new packet to a better carrier. Also, iteratively, these packets are transmitted to the next carrier until the packets cannot be forwarded to the next carrier. */

				VADD_Iterative_Forward_Packet_To_Next_Carrier_On_Road_Segment(param, current_time, vehicle, Gr, Gr_size, &FTQ, &packet_delivery_statistics); 
				
				break;

                        case PACKET_RECOMPUTE_TARGET_POINT: /* packet state to recompute its target point */
				current_time = smpl_time();
				
				/** check whether data forwarding mode is download or not. If not, skip processing this event for a new target point */
				if(param->data_forwarding_mode != DATA_FORWARDING_MODE_DOWNLOAD)
				  break;

				pGlobalPacket = GetGlobalPacketPointerByID(&GPQ, id); //get the pointer to the global packet in global packet queue GPQ
				if(pGlobalPacket == NULL)
				{ //the packet corresponding to pGlobalPacket is either delivered to the destination vehicle or dropped due to TTL expiration
				  break;
				}

				/*@ for debugging */
				if(current_time > 7105 && pGlobalPacket->carrier_vnode->id == 121)
				  printf("current_time=%.2f, id=%d\n", current_time, id);
				/******************/
				
				/** check whether this packet is the carrier's latest packet; if so, update the target point and schedule another target point recomputation event; otherwise, ignore this event. */
				if(pGlobalPacket->carrier_vnode->latest_packet_ptr->id != id)
				  break;

				/** recompute the target point of this packet */
				pGlobalPacket->target_point_id = GetTargetPoint_For_Carrier(param, current_time, pGlobalPacket->carrier_vnode, &FTQ, &EDD_p, &EAD_p);
				
				/** update the packet's target point */
				pGlobalPacket->packet->target_point_id = pGlobalPacket->target_point_id;

				/** update the target point of the carrier vehicle holding this packet */
				pGlobalPacket->carrier_vnode->target_point_id = pGlobalPacket->target_point_id;

				/** update the packet carrier's EDD, EDD_SD and EDD_update_time for download */
				VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download(current_time, param, pGlobalPacket->carrier_vnode, &FTQ);				
			        break;

/********** VEHICLE STATES ***********/

			case VEHICLE_ARRIVE:
				//number_of_arrived_vehicles++;
				current_time = smpl_time();

				vehicle = vehicle_search(id);
				if(vehicle == NULL)
				{
					printf("case VEHICLE_ARRIVE: there is no vehicle node corresponding to id %d\n", id);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                        fgetc(stdin);
#endif
					exit(1);
				}
				
				/**@for debugging */
				//if(id == 158)
				//if(id == 158 && current_time >= 3600)
				//  printf("at time=%.1f, vehicle(id=%d) is tracked\n", (float)current_time, id);
				/******************/

				/* update vehicle's state and state_time */
				vehicle->state = VEHICLE_ARRIVE;
				vehicle->state_time = current_time;

				/* check whether data forwarding mode is download or upload */
				if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD ||
						param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
			        {
				  /* check whether vehicle is a destination vehicle or not; if so, replace its trajectory witg the trajectory in destination vehicle queue node's mobility_list */
				  pDestinationVehicleQueueNode =  GetDestinationVehicleByVID(&DVQ, vehicle->id);
				  if(pDestinationVehicleQueueNode)
				  {
					/* set up fields related to vehicle's type and mobility */
				    vehicle->type = VEHICLE_DESTINATION;
				    vehicle->mobility_type = pDestinationVehicleQueueNode->mobility_type;

				    /* replace vehicle trajectory with mobility list */
				    replace_vehicle_trajectory_with_mobility_list(current_time, param, vehicle, pDestinationVehicleQueueNode, Gr, Gr_size);

				    /* set up the information for the destination vehicle queue node (e.g., vnode) with vehicle */
				    pDestinationVehicleQueueNode->vnode = vehicle;

					/* register the destination vehicle into destination_vehicle */
					destination_vehicle_id = vehicle->id;
					destination_vehicle = vehicle;

					/* register the pointer to the destination vehicle into param->dst_vnode */
					param->vanet_table.dst_vnode = vehicle;
				   
				  }
				  else
				  { //non-destination vehicle
					vehicle->type = VEHICLE_CARRIER;
				  }
				}
				else //param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD
				{
				  vehicle->type = VEHICLE_CARRIER;
				}

				/* update the accumulated values of speed_sum and speed_square_sum */
				speed_sum += vehicle->speed; //speed_sum for speed mean in the speed statistics
				speed_square_sum += pow(vehicle->speed, 2); //speed_square_sum for speed variance in the speed statistics
				//vehicle's speed in [m/s]

				/* determine the role of vehicle if the vehicular traffic model is closed network */
				if(param->vehicle_vanet_vehicular_traffic_model == VANET_VEHICULAR_TRAFFIC_CLOSED_NETWORK)
				{ 
				  if(AP_vehicle_number < param->vehicle_AP_passing_entity_number)
				  {
				    AP_vehicle_number++;
				    vehicle->role = VEHICLE_ROLE_AP_VEHICLE; 
				  }
				  else
				  {
				    vehicle->role = VEHICLE_ROLE_NON_AP_VEHICLE; 
				  }
				}

				/**@ Note: the vehicle movement registration can be performed at state VANET_EDD_UPDATE just after the schedule of the arrival of this vehicle. Thus, we need to check whether the movement registration of this vehicle has been performed or not. */

				/** register the vehicle's movement into the directional edge queue's vehicle_movement_list */
                if((param->vehicle_vanet_acl_measurement_flag == TRUE) && (vehicle->flag_vehicle_movement_queue_registration == FALSE))
					register_vehicle_movement(param, vehicle, current_time, Gr);

                if(flag_packet_log)
				{
                    if((param->vehicle_vanet_acl_measurement_flag != TRUE) && (vehicle->flag_vehicle_movement_queue_registration == FALSE))
				    register_vehicle_movement(param, vehicle, current_time, Gr);

					/* check whether data forwarding mode is download or upload */
					if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD ||
						  param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
					{
						/* obtain the pointer to the first access point */
						pAP =  GetFirstAccessPoint(&APQ);
						if(pAP == NULL)
						{
							printf("After GetFirstAccessPoint(), pAP is NULL!\n");
							exit(1);
						}
						//compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge along with real graph Gr and AP table				      				  
						VADD_Update_Vehicle_EDD_And_EDD_SD(current_time, param, vehicle, pAP->Ga, pAP->Ga_size, &(pAP->ap_table_for_target_point)); 
					}
					else
					{
						//compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge along with real graph Gr and AP table
						VADD_Update_Vehicle_EDD_And_EDD_SD(current_time, param, vehicle, Gr, Gr_size, &ap_table_for_Gr);				    
					}

					if((param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY) && (vehicle->flag_convoy_registration == FALSE))
					{
						//join vehicle into the convoy preceding the vehicle on the directional edge if the vehicle is within the communication range of the convoy
						convoy_join(param, vehicle, current_time, &packet_delivery_statistics); 
					}
				}
				///* transit to VEHICLE_CHECK state */
				//delay = 0.0;
				//schedule(VEHICLE_CHECK, delay, id);

				/** perform packet generation schedule */
				if(param->comparison_target_type != COMPARISON_AVERAGE_LINK_DELAY_ESTIMATION_TYPE) //if-1
				{ //@Note: for the comparison target of link delay estimation, after the 1st vehicular traffic measurement period, each vehicle arriving at the road segment generates one packet.

				  /** transit to one state among VEHICLE_TARGET_VEHICLE_START, VEHICLE_STATIONARY_VEHICLE_START and VEHICLE_CHECK */
/* 				  if(param->vehicle_vanet_target_vehicle_flag && (id == TARGET_VEHICLE_ID)) //if-1.1 */
/* 				  { */
/* 				    delay = MAX(param->vehicle_packet_generation_schedule_time - current_time, 0); */
/* 				    schedule(VEHICLE_TARGET_VEHICLE_START, delay, id); */

/* 				    packet_vehicle_count++; */
/* 				    //break; //skip the rest of the codes in this event */
/* 				  } //end of if-1.1 */

				  if((param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD) && (param->vehicle_vanet_stationary_vehicle_flag == TRUE) && (id == param->vehicle_vanet_stationary_vehicle_id)) //if-1.2
				  {
				    /** delete the stationary vehicle's vehicle movement node from the edge's vehicle movement list */
				    if(param->vehicle_vanet_acl_measurement_flag)
				      delete_vehicle_movement(param, vehicle, current_time, Gr);

				    delay = MAX(param->vehicle_packet_generation_schedule_time - current_time, 0);
				    schedule(VEHICLE_STATIONARY_VEHICLE_START, delay, id);

				    packet_vehicle_count++;
				    //break; //skip the rest of the codes in this event

				  } //end of if-1.2
				  else //else-1.3
				  {
                                    /** schedule the vehicle's movement */
				    delay = 0.0;
				    schedule(VEHICLE_CHECK, delay, id);
				    //perform the rest of the codes in this event
				    /** schedule the packet generation in packet generating vehicles */
				    if((param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD) && (packet_vehicle_count < param->vehicle_packet_generating_entity_number))
				    {
				      delay = MAX(param->vehicle_packet_generation_schedule_time - current_time, 0);
				      schedule(PACKET_ARRIVE, delay, id);
				      packet_vehicle_count++;
				    }
				  } //end of else-1.3
				} //end of if-1
				else if((param->comparison_target_type == COMPARISON_AVERAGE_LINK_DELAY_ESTIMATION_TYPE) && (flag_packet_log == FALSE)) //else if-2
				{
                                  /** schedule the vehicle's movement */
				  delay = 0.0;
				  schedule(VEHICLE_CHECK, delay, id);
				  //perform the rest of the codes in this event
				} //end of else if-2
				else if((param->comparison_target_type == COMPARISON_AVERAGE_LINK_DELAY_ESTIMATION_TYPE) && (flag_packet_log == TRUE)) //else if-3
				{ //the case where a vehicle arriving at the road segment generates one packet to be delivered to the other end of the road segment

				  /* update the packet delivery statistics */
				  number_of_generated_packets++;
				  packet_delivery_statistics.generated_packet_number++;
				  packet_delivery_statistics.generated_packet_copy_number++;

				  /* initialize a packet and insert it into the vehicle's packet queue */
				  memset(&packet, '\0', sizeof(packet));
				  packet.data_forwarding_mode = DATA_FORWARDING_MODE_UPLOAD;
				  packet.src_node_type = VANET_NODE_VEHICLE;
				  packet.src_id = id;

				  /* obtain the pointer to the first access point */
				  pAP =  GetFirstAccessPoint(&APQ);
				  if(pAP == NULL)
				  {
				    printf("After GetFirstAccessPoint(), pAP is NULL!\n");
				    exit(1);
				  }				  
				  packet.dst_node_type = VANET_NODE_AP;
				  packet.dst_id = pAP->id;
				  //packet.dst_id = atoi(ap_table_for_Gr.list[0].vertex);

				  packet.carry_src_id = id;
				  packet.carry_dst_id = 0;

				  packet.seq = ++(vehicle->seq); //increase the sequence number for a new packet generated by this vehicle
				  packet.size = param->communication_data_packet_size;
				  packet.expected_delivery_delay = vehicle->EDD; //set vehicle's EDD to packet's EDD
				  packet.expected_delivery_delay_standard_deviation = vehicle->EDD_SD; //set vehicle's expected delivery delay standard deviation (EDD_SD) to packet's expected delivery delay standard deviation
                                  packet.actual_delivery_delay = 0;

			 	  packet.ttl = param->communication_packet_ttl;
				  packet.generation_time = current_time;
				  packet.last_receive_time = current_time;
				  Enqueue((queue_t*)vehicle->packet_queue, (queue_node_t*)&packet);

				  /* increase the number of packet carrier vehicles */
				  packet_vehicle_count++;

                                  /** schedule the vehicle's movement */
				  delay = 0.0;
				  schedule(VEHICLE_CHECK, delay, id);
				  //perform the rest of the codes in this event
  				} //end of else if-3

				/* schedule the arrival of another vehicle for the current traffic source */
				if(param->vehicle_interarrival_time_distribution != UNIFORM)
				{
					if((param->vehicle_vanet_vehicular_traffic_model == VANET_VEHICULAR_TRAFFIC_OPEN_NETWORK) || 
					   ((param->vehicle_vanet_vehicular_traffic_model == VANET_VEHICULAR_TRAFFIC_CLOSED_NETWORK) &&
					    (number_of_arrived_vehicles < param->vehicle_maximum_number)))
					{
					  /* select traffic source for a new vehicle */
					  traffic_source = Find_Traffic_Source(&sched_table, current_time);

					  /* get an interarrivel time from the current vehicle and the new vehicle */
					  delay = delay_func(param, DELAY_VEHICLE_INTERARRIVAL_TIME);

					  /* compute the arrivel time of the new vehicle */
					  arrival_time = current_time + delay;

					  /* register a new vehicle to vehicle list */
					  register_vehicle(new_simulation_node_id, traffic_source, arrival_time, &dst_table_for_Gr, &ap_table_for_Gr, param, &path_table, Gr, Gr_size, &Er, Dr_move, Mr_move);

					  /* update schedule table for a new vehicle arrival for the traffic_source */
					  Update_Schedule_Table(&sched_table, traffic_source, current_time, delay);

					  schedule(VEHICLE_ARRIVE, delay, new_simulation_node_id);
					  new_simulation_node_id++;
					  number_of_arrived_vehicles++;
					}
				}
								
				/* update the statistics for arrival */
				index = Lookup_Schedule_Table_Index(&sched_table, traffic_source);
				//return the index corresponding to traffic_source in sched_table


				/* include the interarrival time from the 2nd arrival since the 1st arrival cannot be used for interarrival time */
				if(arrival_number[index] > 0)
				{
					interarrival_time_sum[index] = interarrival_time_sum[index] + (current_time - previous_arrival_time[index]);

#ifdef __LOG_LEVEL_TRACE_OF_VEHICLE_CONVOY_LENGTH__
					/*@VANET: get the interarrival time */
					convoy_interarrival_time = current_time - previous_arrival_time[index];
#endif
				}
				else
				{
#ifdef __LOG_LEVEL_TRACE_OF_VEHICLE_CONVOY_LENGTH__
					/*@VANET: set the first interarrival time for the first vehicle to zero */
				        convoy_interarrival_time = 0;
#endif
				}
			
				arrival_number[index]++;
				previous_arrival_time[index] = current_time;

				// when distribution is EXPONENTIAL and delay is 0, the returned delay may be negative.

				/* log the path list of a new vehicle */
				///////////////////////////////////////////////////////////////////
#ifdef __LOG_LEVEL_PATH_LIST__
				log_path_list(traffic_source, traffic_destination, current_time, vehicle->id, path_list); //log the path list
#endif
				///////////////////////////////////////////////////////////////////

				/* log the status of vehicle */
				///////////////////////////////////////////////////////////////////
#ifdef __LOG_LEVEL_VEHICLE__
				log_increment = log_surveillance_for_vehicle(vehicle->id, current_time, event, &(vehicle->current_pos_in_Gr), vehicle->move_type, &Er);
				seq_of_surveillance_log += log_increment;
#endif
				///////////////////////////////////////////////////////////////////

#ifdef __DEBUG_LEVEL_VEHICLE_ARRIVE__
				printf("vehicle(id=%d) has arrived at time %f\n", id, (float)current_time);
#endif


				/** For the branch probability computation, we increment the visiting count of the vertex corresponding to vehicle->path_ptr->prev */
				pGraphNode = LookupGraph(Gr, Gr_size, vehicle->path_ptr->vertex);
				pGraphNode->number_of_arrivals++;
                                /*******************************/

/* 				/\** update the statistics for branch probability and vehicular traffic densidy *\/							 //index = atoi(vehicle->path_ptr->vertex) - 1; //index for the graph node G[index] corresponding to the current intersection node */
/* 				pGraphNode = LookupGraph(Gr, Gr_size, vehicle->path_ptr->vertex); //return the pointer to graph node corresponding to vertex vehicle->path_ptr->vertex */
				
/* 				if(pGraphNode->number_of_branching == 0) */
/* 				{ */
/* 				  pGraphNode->last_arrival_time = current_time; */
/* 				} */
/* 				else */
/* 				{ */
/* 				  pGraphNode->number_of_interarrivals++; */
/* 				  pGraphNode->sum_of_interarrival_time += current_time - pGraphNode->last_arrival_time; */
/* 				  pGraphNode->last_arrival_time = current_time; */
/* 				} */
	
/* 				pGraphNode->number_of_branching++; //increment the number of branching to count the number of new vehicle arrivals on this node */
/* 				/\*******************************************\/ */


#ifdef __LOG_LEVEL_TRACE_OF_VEHICLE_CONVOY_LENGTH__

				/*@VANET: check whether the escape of this vehicle has happened for the initialization phase or not:
				  if this escape has happened, start the logging for the convoy length */
				//if(flag_of_vanet_logging_start == FALSE)
				//  break;

				/* make a schedule to trigger the VEHICLE_OUT_OF_INTERSECTION event */
				delay = param->communication_range/vehicle->speed;
				schedule(VEHICLE_OUT_OF_INTERSECTION, delay, id);

				/* perform the work for the convoy length computation */
				convoy_threshold = param->communication_range/vehicle->speed;
				convoy_last_arrival_time = current_time;
				convoy_last_vehicle = id;
				if(convoy_vehicle_number == 0)
				{
				  convoy_head = id;
				  convoy_start_time = current_time;
				  convoy_end_time = 0;
				  convoy_start_length = 0;
				  convoy_end_length = 0;
				  convoy_vehicle_number++;
				}
				else if(convoy_interarrival_time <= convoy_threshold)
				{
				  convoy_vehicle_number++;
				}
				
#endif
			        break;


#ifdef __LOG_LEVEL_TRACE_OF_VEHICLE_CONVOY_LENGTH__

		        case VEHICLE_OUT_OF_INTERSECTION: //vehicle goes out of the communication range from the intersection
			        current_time = smpl_time();

				vehicle = vehicle_search(id);
				if(vehicle == NULL)
				{
				  printf("case VEHICLE_OUT_OF_INTERSECTION: there is no vehicle node corresponding to id %d\n", id);
#ifdef __DEBUG_INTERACTIVE_MODE__
	                          fgetc(stdin);
#endif
				  exit(1);
				}

				/* update vehicle's state and state_time */
				vehicle->state = VEHICLE_OUT_OF_INTERSECTION;
				vehicle->state_time = current_time;

				/* compute the vehicle interarrival time at the intersection */
				if((flag_of_vanet_logging_start == FALSE) && (convoy_last_vehicle == id))
				{ //the case where the first arrived vehicle has not departed the road segment; so the logging has not started
				  convoy_vehicle_number = 0; //reset the convoy-vehicle-number to zero
				  convoy_start_time = current_time; //set the convoy-start-time to the current time
				  convoy_start_length = 0; //reset the convoy-start-length to zero
				  convoy_head = -1; //reset convoy-head to -1
				}
				else if((flag_of_vanet_logging_start == TRUE) && (convoy_last_vehicle == id))
				{
				  convoy_end_time = current_time;
				  convoy_end_length = convoy_start_length + (convoy_end_time - convoy_start_time)*vehicle->speed;
				  if(convoy_end_length > vehicle->edge_length)
				  {
                                    printf("main(): Error: at time=%.1f, for vehicle(id=%d), convoy_end_length(%.0f) > vehicle->edge_length(%.0f)\n", (float)current_time, vehicle->id, (float)convoy_end_length, (float)vehicle->edge_length);
				    exit(1);
                                  }

				  /* log the the convoy length of vehicles */
				  log_trace_file(fp_trace_file_of_vehicle_convoy_length, TRACE_VEHICLE_CONVOY_LENGTH, current_time, convoy_start_time, convoy_start_length, convoy_end_time, convoy_end_length, vehicle->speed, convoy_vehicle_number);

				  convoy_vehicle_number = 0; //reset the convoy-vehicle-number to zero
				  convoy_start_time = current_time; //set the convoy-start-time to the current time
				  convoy_start_length = 0; //reset the convoy-start-length to zero

				  convoy_head = -1; //reset convoy-head to -1
				}

			        break;
#endif


			case VEHICLE_RESTART:
				current_time = smpl_time();

				vehicle = vehicle_search(id);
				if(vehicle == NULL)
				{
					printf("case VEHICLE_RESTART: there is no vehicle node corresponding to id %d\n", id);
#ifdef __DEBUG_INTERACTIVE_MODE__
                    fgetc(stdin);
#endif
					exit(1);
				}

				/**@for debugging */
				//if(id == 158)
				//  printf("at time=%.1f, vehicle(id=%d) is tracked\n", (float)current_time, id);
				/******************/


				/** update vehicle's state and state_time */
				vehicle->state = VEHICLE_RESTART;
				vehicle->state_time = current_time;

				/** set up a new path for the vehicle's movement */
				update_vehicle_trajectory(vehicle, current_time, &dst_table_for_Gr, &ap_table_for_Gr, param, &path_table, Gr, Gr_size, &Er, Dr_move, Mr_move);

				/* taehwan 20140829 */
				g_vehicle_color[vehicle->id] = 0;

				/** register the vehicle's movement into the directional edge queue's vehicle_movement_list */
				if(param->vehicle_vanet_acl_measurement_flag)
				  register_vehicle_movement(param, vehicle, current_time, Gr);

                if(flag_packet_log)
				{
				  if(param->vehicle_vanet_acl_measurement_flag != TRUE)
				    register_vehicle_movement(param, vehicle, current_time, Gr);

				  /* check whether data forwarding mode is download or upload */
				  if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD ||
						  param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
				  {
				    /* obtain the pointer to the first access point */
				    pAP =  GetFirstAccessPoint(&APQ);
				    if(pAP == NULL)
				    {
				      printf("After GetFirstAccessPoint(), pAP is NULL!\n");
				      exit(1);
				    }

				    VADD_Update_Vehicle_EDD_And_EDD_SD(current_time, param, vehicle, pAP->Ga, pAP->Ga_size, &(pAP->ap_table_for_target_point)); //compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge along with real graph Gr and AP table				      				  
				  }
				  else
				  {
				    VADD_Update_Vehicle_EDD_And_EDD_SD(current_time, param, vehicle, Gr, Gr_size, &ap_table_for_Gr);
				    //compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge along with real graph Gr and AP table
				  }

				  if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY)
				  {
				    /**@for debugging */
				    //if(vehicle->id == 710)
				    //  printf("vehicle(id=%d) is tracked\n", vehicle->id);
				    /******************/

				    convoy_join(param, vehicle, current_time, &packet_delivery_statistics); //join vehicle into the convoy preceding the vehicle on the directional edge if the vehicle is within the communication range of the convoy
				    //delay = 0.0;
				    //schedule(CONVOY_UPDATE, delay, id);
				  }
				}
				
                                /** determine think time (i.e., waiting time) at an intersection according to the think time distribution */
                                //let the vehicle move to the next state after a random time at the this state
                                //delay =  delay_func(param, DELAY_VEHICLE_THINK_TIME);
                                if(param->vehicle_think_time > 0)
                                    delay =  delay_func(param, DELAY_VEHICLE_THINK_TIME);
                                else
                                    delay = 0; //let vehicle wait for a random time before going to the next state

				/** transit to VEHICLE_CHECK state */
				schedule(VEHICLE_CHECK, delay, id);

				/* log the path list of a new vehicle */
				///////////////////////////////////////////////////////////////////
#ifdef __LOG_LEVEL_PATH_LIST__
				log_path_list(traffic_source, traffic_destination, current_time, vehicle->id, path_list); //log the path list
#endif
				///////////////////////////////////////////////////////////////////

				/* log the status of vehicle */
				///////////////////////////////////////////////////////////////////
#ifdef __LOG_LEVEL_VEHICLE__
				log_increment = log_surveillance_for_vehicle(vehicle->id, current_time, event, &(vehicle->current_pos_in_Gr), vehicle->move_type, &Er);
				seq_of_surveillance_log += log_increment;
#endif
				///////////////////////////////////////////////////////////////////

#ifdef __DEBUG_LEVEL_VEHICLE_RESTART__
				printf("vehicle(id=%d) has restarted at time %f\n", id, (float)current_time);
#endif

                                break;


			case VEHICLE_CHECK:
				current_time = smpl_time();

				vehicle = vehicle_search(id);
				if(vehicle == NULL)
				{
				  printf("case VEHICLE_CHECK: there is no vehicle node corresponding to id %d\n", id);
#ifdef __DEBUG_INTERACTIVE_MODE__
				  fgetc(stdin);
#endif
				  exit(1);
				}

				/* update vehicle's state and state_time */
				vehicle->state = VEHICLE_CHECK;
				vehicle->state_time = current_time;

				/*@For debugging */
				//if(vehicle->id == param->vehicle_vanet_stationary_vehicle_id)
				//  printf("vehicle(id=%d) is tracked\n", vehicle->id);
				/*****/

			    /* log the status of vehicle */
				///////////////////////////////////////////////////////////////////
#ifdef __LOG_LEVEL_VEHICLE__
				log_increment = log_surveillance_for_vehicle(vehicle->id, current_time, event, &(vehicle->current_pos_in_Gr), vehicle->move_type, &Er);
				seq_of_surveillance_log += log_increment;
#endif
				///////////////////////////////////////////////////////////////////

				/* check whether the vehicle has arrived at its destination */			
                if(vehicle->mobility_type == MOBILITY_STATIONARY)
                { //for stationary node, do not schedule any event for it any more.
                  break;
                }
                else if(IsEndOfTravel(vehicle->path_list, vehicle->path_ptr) == TRUE) //if-1       
				//if(IsEndOfTravel(vehicle->path_list, vehicle->path_ptr) == TRUE) //if-1
				{ //This block is performed only if the source is equal to the destination.					
				  printf("VEHICLE_CHECK: IsEndOfTravel() returns TRUE!\n");
				  exit(1);
				  //state = VEHICLE_ESCAPE;
				  //state = VEHICLE_RESTART;
				  //delay = 0.0;
				  //schedule(state, delay, id);
				  //break; //Note: don't omit "break" to skip the remaining code in this case block!
				} //end of if-1
				else //else-1
				{
				  /* determine the vehicle step time based on vehicle_step_mode */
				  if(param->vehicle_step_mode == STEP_TIME)
				  {
				    /* determine movement interval using edge distance and offset */
					if(vehicle->move_type == MOVE_FORWARD) //if-2
					{
					  diff = vehicle->edge_length - vehicle->current_pos_in_Gr.offset;
					} //end of if-2
					else if(vehicle->move_type == MOVE_BACKWARD) //if-3
					{
					  diff = vehicle->current_pos_in_Gr.offset;
					} //end of if-3
					else //else-2
					{
					  printf("vehicle->move_type(%d) is invalid\n", vehicle->move_type);
#ifdef __DEBUG_INTERACTIVE_MODE__
					  fgetc(stdin);
#endif
					  exit(1);
					} //end of else-2

					movement_time = diff/vehicle->speed;
					delay = MIN(movement_time, param->vehicle_step_time);
				  }
				  else if(param->vehicle_step_mode == STEP_EDGE)
					delay = vehicle->edge_length/vehicle->speed;
				  else if(param->vehicle_step_mode == STEP_PATH)
					delay = vehicle->path_length/vehicle->speed;
				  else
				  {
					printf("run(): param->vehicle_step_mode(%d) is not supported!\n", param->vehicle_step_mode);
#ifdef __DEBUG_INTERACTIVE_MODE__
					fgetc(stdin);
#endif
					exit(1);
				  }

			  	  vehicle->move_start_time = current_time;
				  vehicle->move_end_time = current_time + delay;
				  vehicle->move_interval = delay;
				  state = VEHICLE_MOVE;
				} //end of else-1

				/** let vehicle go to the state of VEHICLE_MOVE */
				schedule(state, delay, id);

				break;


			case VEHICLE_MOVE:

				current_time = smpl_time();

				vehicle = vehicle_search(id);
				if(vehicle == NULL)
				{
				  printf("case VEHICLE_MOVE: there is no vehicle node corresponding to id %d\n", id);
#ifdef __DEBUG_INTERACTIVE_MODE__
				  fgetc(stdin);
#endif
				  exit(1);
				}
#if TPD_VEHICLE_MOBILITY_TRACE_FLAG
				/** TRACE vehicle mobility */
				if((current_time > 9781) && (vehicle->id == 86 || vehicle->id == 168))
				//if((current_time > 12620) && (vehicle->id == 88 || vehicle->id == 29))
				{
					printf("%s:%d: TRACE [%.2f] vehicle(%d)'s position is <%s, %s: %0.f>\n",
							__FUNCTION__, __LINE__,
							(float)current_time,
							vehicle->id, 
							vehicle->current_pos_in_digraph.tail_node,
							vehicle->current_pos_in_digraph.head_node,
							vehicle->current_pos_in_digraph.offset);
				}
				/***************************/
#endif
				/*if ( (vehicle->id == 22 || vehicle->id == 1) &&
						(current_time > 23680 && current_time < 23730 ) )
				{
					setLogOnOff(TRUE);
				} else {
					setLogOnOff(FALSE);
				}*/

				/** reset intersection_flag to FALSE when vehicle is leaving from an intersection */
				intersection_visit_flag = FALSE;

				/* update vehicle's state and state_time */
				vehicle->state = VEHICLE_MOVE;
				vehicle->state_time = current_time;

				/**@for debugging */
				//if(vehicle->id == 10 && current_time >= 3910)
				//  printf("VEHICLE_MOVE: at time=%.1f, vid = %d\n", (float)current_time, vehicle->id);
				/******************/

				/* log the status of vehicle */
				///////////////////////////////////////////////////////////////////
#ifdef __LOG_LEVEL_VEHICLE__
				log_increment = log_surveillance_for_vehicle(vehicle->id, current_time, event, &(vehicle->current_pos_in_Gr), vehicle->move_type, &Er);
				seq_of_surveillance_log += log_increment;
#endif
				///////////////////////////////////////////////////////////////////

				/* update the vehicle's offset in both real graph Gr (i.e., undirectional graph) and vehicle_movement_list (i.e., directional graph) */
				if(vehicle->move_type == MOVE_FORWARD) //if-1
				{
					movement_time = vehicle->move_interval;
					movement_distance = movement_time*vehicle->speed;
					vehicle->current_pos_in_Gr.offset += movement_distance; //update the offset in real graph Gr
					vehicle->path_current_edge_offset += movement_distance; //increase the edge offset

					/* update the geometric location in the 2-D Cartesian coordinate system */
					delta_x = movement_time * vehicle->pos_update_vector.x; //change in x-coordinate
					delta_y = movement_time * vehicle->pos_update_vector.y; //change in y-coordinate
				
					vehicle->current_pos.x += delta_x;
					vehicle->current_pos.y += delta_y;

					/* After the vehicular traffic measurement phase, if vehicle is registered in a vehicle movement queue, update the vehicle's offset in the vehicle movement queue */

					if(param->vehicle_vanet_acl_measurement_flag)
					{
						vehicle->ptr_vehicle_movement_queue_node->offset = vehicle->current_pos_in_Gr.offset; //update the offset in vehicle movement list
						vehicle->current_pos_in_digraph.offset = vehicle->ptr_vehicle_movement_queue_node->offset; //update the offset in vehicle's current_pos_in_digraph
					}

					if(flag_packet_log)
					{
						if(param->vehicle_vanet_acl_measurement_flag != TRUE)
						{
							vehicle->ptr_vehicle_movement_queue_node->offset = vehicle->current_pos_in_Gr.offset; //update the offset in vehicle movement list
							vehicle->current_pos_in_digraph.offset = vehicle->ptr_vehicle_movement_queue_node->offset; //update the offset in vehicle's current_pos_in_digraph
						}

						if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
						{
							VADD_Update_Vehicle_EDD_And_EDD_SD(current_time, param, vehicle, Gr, Gr_size, &ap_table_for_Gr); //compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge along with real graph Gr and AP table
						}
						else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
						{
							VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download(current_time, param, vehicle, &FTQ);
						}
					}
				} //end of if-1
				else if(vehicle->move_type == MOVE_BACKWARD) //else-if-2
				{
					movement_time = vehicle->move_interval;
					movement_distance = movement_time*vehicle->speed;
					vehicle->current_pos_in_Gr.offset -= movement_distance;	//update the offset in real graph Gr
					vehicle->path_current_edge_offset += movement_distance; //increase the edge offset

					/* update the geometric location in the 2-D Cartesian coordinate system */
					delta_x = movement_time * vehicle->pos_update_vector.x; //change in x-coordinate
					delta_y = movement_time * vehicle->pos_update_vector.y; //change in y-coordinate
				
					vehicle->current_pos.x += delta_x;
					vehicle->current_pos.y += delta_y;

					/* After the vehicular traffic measurement phase, if vehicle is registered in a vehicle movement queue, update the vehicle's offset in the vehicle movement queue */

					if(param->vehicle_vanet_acl_measurement_flag)
					{
						vehicle->ptr_vehicle_movement_queue_node->offset = vehicle->edge_length - vehicle->current_pos_in_Gr.offset; //update the offset in vehicle movement list
						vehicle->current_pos_in_digraph.offset = vehicle->ptr_vehicle_movement_queue_node->offset; //update the offset in vehicle's current_pos_in_digraph
					}

					if(flag_packet_log)
					{
						if(param->vehicle_vanet_acl_measurement_flag != TRUE)
						{
							vehicle->ptr_vehicle_movement_queue_node->offset = vehicle->edge_length - vehicle->current_pos_in_Gr.offset; //update the offset in vehicle movement list
							vehicle->current_pos_in_digraph.offset = vehicle->ptr_vehicle_movement_queue_node->offset; //update the offset in vehicle's current_pos_in_digraph
						}

						if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
						{
							VADD_Update_Vehicle_EDD_And_EDD_SD(current_time, param, vehicle, Gr, Gr_size, &ap_table_for_Gr); //compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge along with real graph Gr and AP table
						}
						else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
						{
							VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download(current_time, param, vehicle, &FTQ);
						}
					}
				} //end of else-of-2
				else //else-3
				{		
					printf("vehicle->move_type(%d) is invalid\n", vehicle->move_type);
#ifdef __DEBUG_INTERACTIVE_MODE__
					fgetc(stdin);
#endif
					exit(1);
				} //end of else-3

				/****************************************************************************/
				/*##########################################################################*/
				/****************************************************************************/
				/** only when flag_packet_log is set to TRUE, perform the packet forwarding to another vehicle or AP */
				if(flag_packet_log) //if-4				
				{
					/** check whether this vehicle is within the Intersection areas of either the tail node or the head node of the current directed edge for the data forwarding to other vehicles moving on other road segments*/
					flag = VADD_Is_Within_Intersection_Area(param, vehicle, Gr, Gr_size, &intersection_area_type, &tail_intersection_id, &head_intersection_id);
					if(flag) //if-4.1
					{
						/** Under download mode, check if vehicle arrives at the destination vehicle trajectory in its packet or not */
						if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD) //if-4.1.1
						{
							/* For download mode, compute vehicle's target point for the destination vehicle and then recompute the EDD_for_download/EDD_SD_download with forwarding table queue FTQ; for a convoy-based forwarding, update the target point and the EDD_for_download/EDD_SD_download of vehicle's leader */
							if(param->vehicle_vanet_target_point_selection_type == VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT) //if-4.1.1.1
							{ //forward the vehicle's packets to the stationary node at the intersectio
								/* if the intersection is the tail node, vehicle checks whether there are packets waiting for a vehicle moving this edge. */
								if(intersection_area_type == INTERSECTION_AREA_TAIL_NODE || intersection_area_type == INTERSECTION_AREA_BOTH_NODES) //if-4.1.1.1.1
								{
									/* forward to vehicle the packets needed to go to the edge where vehicle is moving */
									VADD_Forward_Packet_From_Stationary_Node_To_Next_Carrier(param, current_time, tail_intersection_id, vehicle, &packet_delivery_statistics);

									/* update the vehicle's EDD_for_download with the packets forwarded from the stationary node for tail_intersection_id */
									VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download(current_time, param, vehicle, &FTQ);
								} //end of if-4.1.1.1.1

								/* if the intersection is the head node, vehicle forwards its packets to the stationary node at the intersection. */
								if(intersection_area_type == INTERSECTION_AREA_HEAD_NODE || intersection_area_type == INTERSECTION_AREA_BOTH_NODES) //if-4.1.1.1.2
								{
									/* check whether vehicle is the destination vehicle or not. If so, the stationary node forwards its packets to the destination vehicle. */
									if(is_destination_vehicle(param, vehicle)) //if-4.1.1.1.2.1
									{
										/* forward the packets of the stationary node to the destination vehicle */
										VADD_Forward_Packet_From_Stationary_Node_To_Destination_Vehicle(param, current_time, head_intersection_id, vehicle, &packet_delivery_statistics);
									} //end of if-4.1.1.1.2.1
									else //else-4.1.1.1.2.2
									{
										/* check whether the vehicle's convoy (i.e., the vehicle's convoy leader) has packets or not */
										if(does_vehicle_convoy_have_packet(vehicle, param)) //if-4.1.1.1.2.2.1
										{
											VADD_Forward_Packet_To_Stationary_Node(param, current_time, vehicle->ptr_convoy_queue_node->leader_vehicle, Gr, Gr_size, intersection_area_type, head_intersection_id, &packet_delivery_statistics); //vehicle's convoy leader forwards its packet(s) to the stationary node at the heading intersection

											/* update the vehicle convoy leader's EDD_for_download with the packets forwarded from the stationary node for tail_intersection_id */
											VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download(current_time, param, vehicle->ptr_convoy_queue_node->leader_vehicle, &FTQ);
										} //end of if-4.1.1.1.2.2.1

										/* check whether the vehicle has packets or not */
										if(does_vehicle_have_packet(vehicle)) //if-4.1.1.1.2.2.2
										{
											VADD_Forward_Packet_To_Stationary_Node(param, current_time, vehicle, Gr, Gr_size, intersection_area_type, head_intersection_id, &packet_delivery_statistics); //vehicle forwards its packet(s) to the stationary node at the heading intersection

											/* update the vehicle's EDD_for_download with the packets forwarded from the stationary node for tail_intersection_id */
											VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download(current_time, param, vehicle, &FTQ);
										} //end of if-4.1.1.1.2.2.2
									} //end of else-4.1.1.1.2.2
								} //end of if-4.1.1.1.2
							} //end of if-4.1.1.1
							else //update the target point for other target point selection schemes except for packet-trajectory-based target point selection //else-4.1.1.2
							{							
								VADD_Update_VehicleTargetPoint_Along_With_EDD_And_EDD_SD_For_Download(current_time, param, vehicle, &FTQ, intersection_area_type);
							} //end of else-4.1.1.2
						} //end of if-4.1.1
						/** if there is a better carrier in this intersection area, it passes its packet to the carrier */
						else if((param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD) || (param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD && param->vehicle_vanet_target_point_selection_type != VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT)) //else if-4.1.2
						{
							flag = VADD_Is_There_Next_Carrier_At_Both_Intersection_Areas(param, current_time, vehicle, Gr, Gr_size, &FTQ, intersection_area_type, &next_carrier); //check whether this vehicle can forward with the communication to the other vehicle as next carrier around the upcoming intersection
							if(flag) //if-4.1.2.1
							{
								/* check whether the vehicle's convoy (i.e., the vehicle's convoy leader) has packets or not */
								if(does_vehicle_convoy_have_packet(vehicle, param)) //if-4.1.2.1.1
								{

									VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle->ptr_convoy_queue_node->leader_vehicle, next_carrier, &packet_delivery_statistics, &discard_count); //vehicle's convoy leader forwards its packet(s) to the next carrier pointed by next_carrier

									if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD) //if-4.1.2.1.1.1
									{
										VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download(current_time, param, vehicle->ptr_convoy_queue_node->leader_vehicle, &FTQ);
										/* update the vehicle convoy leader's EDD_for_download with the packets forwarded from the intersection */
									} //end of if-4.1.2.1.1.1
								} //end of if-4.1.2.1.1

								/* check whether the vehicle's convoy (i.e., the vehicle's convoy leader) has packets or not */
								if(does_vehicle_have_packet(vehicle)) //if-4.1.2.1.2
								{
									VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
	
									if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD) //if-4.1.2.1.2.1
									{
										VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download(current_time, param, vehicle, &FTQ);
										/* update the vehicle's EDD_for_download with the packets forwarded from the intersection */
									} //end of if-4.1.2.1.2.1
								} //end of if-4.1.2.1.2
							} //end of if-4.1.2.1
						} //end of else if-4.1.2
						else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V) //else if-4.1.3
						{

						} //end of else if-4.1.3
					} //end of if-4.1

					/** check whether this vehicle reaches an Internet access point and returns the pointer to the graph node corresponding to the access point through *destination_ap; Note that when the vehicle arrives at the location of the Internet access point, it can forward its packets, not using the remote transmission within the communication range; this is because we want to make the actual link delay in one segment close to the expected delivery delay */

					flag = VADD_Is_Within_AP_Communication_Range(param, vehicle, Gr, Gr_size, &ap_table_for_Gr, &ap_graph_node); //@Note: This line should be modified in order to support multiple APs
					//if (vehicle->id==1 || vehicle->id==22)
					//	printf("Range Check : vehicle id=%d, %d\n",vehicle->id,flag);
					if(flag) //if-4.2
					{
						if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD) //if-4.2.1
						{
							/* check whether the vehicle's convoy (i.e., the vehicle's convoy leader) has packets or not */
							if(does_vehicle_convoy_have_packet(vehicle, param)) //if-4.2.1.1
							{
								pAP = GetAccessPointByVertex(&APQ, ap_graph_node->vertex); //return the access point structure corresponding to vertex

								VADD_Forward_Packet_To_AP(param, current_time, vehicle->ptr_convoy_queue_node->leader_vehicle, pAP, &packet_delivery_statistics); //vehicle lets its convoy leader forward its packet(s) to the access point pointed by destination_ap and the log for the packet(s) is written into the packet logging file.
							} //end of if-4.2.1.1

							/* check whether the vehicle's convoy (i.e., the vehicle's convoy leader) has packets or not */
							if(does_vehicle_have_packet(vehicle)) //if-4.2.1.2
							{
								pAP = GetAccessPointByVertex(&APQ, ap_graph_node->vertex); //return the access point structure corresponding to vertex

								VADD_Forward_Packet_To_AP(param, current_time, vehicle, pAP, &packet_delivery_statistics); //vehicle forwards its packet(s) to the access point pointed by destination_ap and the log for the packet(s) is written into the packet logging file.
							} //end of if-4.2.1.2
						} //end of if-4.2.1
						/*********************************/
						else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD) //if-4.2.2
						{
							/* obtain the pointer to the access point placed at ap_graph_node's vertex */
							pAP =  GetAccessPointByVertex(&APQ, ap_graph_node->vertex);
							if(pAP == NULL)
							{
								printf("After GetAccessPointByVertex(), pAP for vertex %s is NULL!\n", ap_graph_node->vertex);
								exit(1);
							}		      

							/* try to find a better neighboring vehicle than this vehicle to send the AP's packets to a next carrier */
							if(param->vehicle_vanet_target_point_selection_type != VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT)
							{
								flag = VADD_Is_There_Next_Carrier_At_Intersection_For_AP(param, current_time, pAP, Gr, Gr_size, &FTQ, &next_carrier);
								if(flag == TRUE)
								{
									VADD_Forward_Packet_From_AP_To_Next_Carrier(param, current_time, pAP, next_carrier, &packet_delivery_statistics, &discard_count); //AP forwards its packet(s) to the neighboring vehicle and the log for the packet(s) is written into the packet logging file.
									
									/* taehwan 20140730 */
									// check packet ttl has expired
									if (discard_count > 0)
									{
										printf("TTL EXPIRED AT AP ! - %d %d\n",vehicle->id,discard_count);
									}

									/* update the next_carrier's EDD_for_download with the packets forwarded from the AP for pAP */
									VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download(current_time, param, next_carrier, &FTQ);
								}
							}
						} //end of if-4.2.2
						/*********************************/
						else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V) //if-4.2.3
						{
							/* obtain the pointer to the access point placed at ap_graph_node's vertex */
							pAP =  GetAccessPointByVertex(&APQ, ap_graph_node->vertex);
							if(pAP == NULL) //if-4.2.3.1
							{
								printf("After GetAccessPointByVertex(), pAP for vertex %s is NULL!\n", ap_graph_node->vertex);
								exit(1);
							} //end of if-4.2.3.1		      

							/* try to find a better neighboring vehicle than this vehicle to send the AP's packets to a next carrier */
							if(param->vanet_forwarding_scheme == VANET_FORWARDING_TPD) //if-4.2.3.2
							{							
								flag = TPD_Is_There_Next_Carrier_At_Intersection_For_AP(param, current_time, pAP, Gr, Gr_size, &next_carrier);
								
								if(flag == TRUE) //if-4.2.3.2.1
								{
									//AP forwards its packet(s) to the neighboring vehicle and the log for the packet(s) is written into the packet logging file.
									forward_count = VADD_Forward_Packet_From_AP_To_Next_Carrier(param, current_time, pAP, next_carrier, &packet_delivery_statistics, &discard_count); 
																		
									if (discard_count > 0)
									{
										printf("TPD) TTL EXPIRED AT AP! - %d %d\n",vehicle->id,discard_count);
									}
									
									if(next_carrier == NULL) //if-4.2.3.2.1.1
									{
										printf("%s:%d VADD_Forward_Packet_From_AP_To_Next_Carrier() returns no next_carrier along with positive forward_count(%d) with discard_count(%d)\n",
												__FUNCTION__, __LINE__,
												forward_count, discard_count);
									} //end of if-4.2.3.2.1.1
									else 
									{
										printf("\n%.2f] %d TPD find next carrier (%d) at intersection for ap , arrived at(%.2f)\n\n"
												,current_time, vehicle->id
												,next_carrier->id
												,next_carrier->arrival_time);
									}
								} //end of if-4.2.3.2.1
							} //end of if-4.2.3.2
							else if(param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD
							|| param->vanet_forwarding_scheme == VANET_FORWARDING_TADB) //else if-4.2.3.3 
							{
#if 1 /* [ */
								target_point_id = GetTargetPoint_For_AP_For_V2V_Data_Delivery(param, current_time, pAP->vertex, pDestinationVehicleQueueNode->vnode, &FTQ, Gr_set_number, Gr_set, Gr_set_size, &EDD_p, &EAD_p);

								/* set AP's target point to target_point_id again */
								pAP->target_point_id = target_point_id;
#endif /* ] */
//printf("@@@@@@\n");
								flag = VADD_Is_There_Next_Carrier_At_Intersection_For_AP(param, current_time, pAP, Gr_set[target_point_id-1], Gr_set_size[target_point_id-1], &FTQ, &next_carrier);
								if(flag == TRUE) //if-4.2.3.3.1
								{
									//AP forwards its packet(s) to the neighboring vehicle and the log for the packet(s) is written into the packet logging file.
									forward_count = VADD_Forward_Packet_From_AP_To_Next_Carrier(param, current_time, pAP, next_carrier, &packet_delivery_statistics, &discard_count); 
									
									printf("target_point_id = %d\n",target_point_id);
									
									if(next_carrier == NULL) //if-4.2.3.3.1.1
									{
										printf("%s:%d VADD_Forward_Packet_From_AP_To_Next_Carrier() returns no next_carrier along with positive forward_count(%d) with discard_count(%d)\n",
												__FUNCTION__, __LINE__,
												forward_count, discard_count);
									} //end of if-4.2.3.3.1.1
								} //end of if-4.2.3.3.1
							} //end of else if-4.2.3.3
							else if(param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC) //else if-4.2.3.4 
							{
#if 1 /* [ */
								target_point_id = GetTargetPoint_For_AP_For_V2V_Data_Delivery(param, current_time, pAP->vertex, pDestinationVehicleQueueNode->vnode, &FTQ, Gr_set_number, Gr_set, Gr_set_size, &EDD_p, &EAD_p);

								/* set AP's target point to target_point_id again */
								pAP->target_point_id = target_point_id;
#endif /* ] */

								EPIDEMIC_Perform_Packet_Dissemination_At_Intersection_For_AP(param, current_time, pAP, Gr_set[pAP->target_point_id-1], Gr_set_size[pAP->target_point_id-1], &packet_delivery_statistics);

#if 0 /* [ */
								flag = EPIDEMIC_Is_There_Next_Carrier_At_Intersection_For_AP(param, current_time, pAP, Gr_set[target_point_id-1], Gr_set_size[target_point_id-1], &FTQ, &next_carrier);
								if(flag == TRUE) //if-4.2.3.3.1
								{
									//AP forwards its packet(s) to the neighboring vehicle and the log for the packet(s) is written into the packet logging file.
									forward_count = VADD_Forward_Packet_From_AP_To_Next_Carrier(param, current_time, pAP, next_carrier, &packet_delivery_statistics, &discard_count); 
									
									if(next_carrier == NULL) //if-4.2.3.3.1.1
									{
										printf("%s:%d VADD_Forward_Packet_From_AP_To_Next_Carrier() returns no next_carrier along with positive forward_count(%d) with discard_count(%d)\n",
												__FUNCTION__, __LINE__,
												forward_count, discard_count);
									} //end of if-4.2.3.3.1.1
								} //end of if-4.2.3.3.1
#endif /* ] */
							} //end of else if-4.2.3.4
						} //end of else-if-4.2.3
					} //end of if-4.2
				} //end of if-4
	
				/**************************************************************/

				/* For flag_packet_log == TRUE and param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD, check whether this vehicle is within the communication range with one of destination vehicles or not */
				
				//bool isInRange = FALSE;
				/* taehwan 20140722 */
				/*
				if (current_time < 7201 && current_time > 7200)
				    show_trajectory_and_arrival_time_for_all_vehicles();
				*/
				if((flag_packet_log == TRUE) && (param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD || param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)) //if-1
				{
					flag = VADD_Is_Within_Destination_Vehicle_Communication_Range(param, vehicle, Gr, Gr_size, &DVQ, &destination_vehicle);
					isInRange = flag;
					if(flag) //if-1.1
					{


						/* check whether the vehicle's convoy (i.e., the vehicle's convoy leader) has packets or not */
						if(does_vehicle_convoy_have_packet(vehicle, param)) //if-1.1.1
						{
							/* taehwan 20140731 */
							/*
							int vehicle_id = vehicle->ptr_convoy_queue_node->leader_vehicle->id;
							double predicted_time = TPD_Get_Predicted_Encounter_Time(vehicle_id);
	
							printf("Predicted Time of %d is %.2f\n",vehicle_id,predicted_time);
	
							if (predicted_time + 100 < current_time)
								printf("\nEXPIRED MEETING!!!!!!!\n\n");
							*/

							/* taehwan 20140712 checking forwarding */
							//printf("does_vehicle_convoy_have_packet == TRUE\n");
							VADD_Forward_Packet_To_Destination_Vehicle(param, current_time, vehicle->ptr_convoy_queue_node->leader_vehicle, destination_vehicle, &packet_delivery_statistics); //vehicle forwards its packet(s) to the destination vehicle
							
							if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
							{
								/* update the vehicle convoy leader's EDD_for_download after forwarding */
								VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download(current_time, param, vehicle->ptr_convoy_queue_node->leader_vehicle, &FTQ);
							}
						} //end of if-1.1.1

						/* check whether the vehicle's convoy (i.e., the vehicle's convoy leader) has packets or not */
						if(does_vehicle_have_packet(vehicle)) //if-1.1.2
						{
							VADD_Forward_Packet_To_Destination_Vehicle(param, current_time, vehicle, destination_vehicle, &packet_delivery_statistics); //vehicle forwards its packet(s) to the destination vehicle
							/* taehwan 20140725 */
							if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
							{
								/* update the vehicle's EDD_for_download after forwarding */
								VADD_Update_Vehicle_EDD_And_EDD_SD_For_Download(current_time, param, vehicle, &FTQ);
							}
						} //end of if-1.1.2
					} //end of if-1.1	  
				} //end of if-1

				/**************************************************************/
				/*############################################################*/
				/**************************************************************/
				/* determine movement interval using edge distance and offset */
				if(vehicle->move_type == MOVE_FORWARD) //if-1
				{
					if(vehicle->current_pos_in_Gr.offset >= vehicle->edge_length - offset_tolerance) //if-1.1
					{ //the case where the vehicle arrives at the intersection
#if TPD_VEHICLE_MOBILITY_TRACE_FLAG
						/** TRACE vehicle mobility */
						if((current_time > 7200) && (vehicle->id == 86))
						{
							printf("%s:%d: TRACE [%.2f] vehicle(%d)'s position is <%s, %s: %0.f> and its speed is %0.f\n",
									__FUNCTION__, __LINE__,
									(float)current_time,
									vehicle->id, 
									vehicle->current_pos_in_digraph.tail_node,
									vehicle->current_pos_in_digraph.head_node,
									vehicle->current_pos_in_digraph.offset,
									vehicle->speed);
						}
				/***************************/
#endif

						/** set intersection_flag to TRUE to indicate that vehicle arrices at an intersection */
						intersection_visit_flag = TRUE;

					    /* update vehicle's speed at the intersection using the speed distribution considering vehicle minimum speed and vehicle maximum speed */
#if TPD_VEHICLE_SPEED_UPDATE_PER_ROAD_SEGMENT_FLAG
					    set_vehicle_speed(param, vehicle);
#endif
	
					    /* only when flag_packet_log is set to TRUE, perform the packet forwarding */
						if(flag_packet_log && does_vehicle_have_packet(vehicle)) //if-1.1.1
						//if(flag_packet_log && does_vehicle_convoy_have_packet(vehicle, param)) //if-1.1.1
						{
							/** determine whether to forward its packets to the best next carrier moving on the other road segment of the intersection and return the pointer to the next carrier through next_carrier */
							if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD || param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD) //if-1.1.1.1
							{
								flag = VADD_Is_There_Next_Carrier_At_Intersection(param, current_time, vehicle, Gr, Gr_size, &FTQ, &next_carrier);
								if(flag) //if-1.1.1.1.1
								{
									if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY && vehicle->flag_convoy_registration == TRUE) //if-1.1.1.1.1.1
									{ 
										VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle->ptr_convoy_queue_node->leader_vehicle, next_carrier->ptr_convoy_queue_node->leader_vehicle, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
									} //end of if-1.1.1.1.1.1
									else //else-1.1.1.1.1.2
									{
										VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
									} //end of else-1.1.1.1.1.2
								} //end of if-1.1.1.1.1
							} //end of if-1.1.1.1
							else if((param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V) && (vehicle->type == VEHICLE_CURRENT_PACKET_CARRIER)) //else if-1.1.1.2
							{
								if(param->vanet_forwarding_scheme == VANET_FORWARDING_TPD && does_vehicle_have_packet(vehicle)) //if-1.1.1.2.1
								{
									//printf("@@@@@@\n");
									flag = TPD_Is_There_Next_Carrier_At_Intersection(param, current_time, vehicle, Gr, Gr_size, &next_carrier);
									if(flag) //if-1.1.1.2.1.1
									{
										printf("TPD find next carrier (%d) at intersection, arrived at(%.2f)\n"
												,next_carrier->id
												,next_carrier->arrival_time);
										
										VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
							
									/* taehwan 20140730 */
									// check packet ttl has expired
									if (discard_count > 0)
									{
										printf("TTL EXPIRED AT Intersection 3416 ! - %d %d\n",vehicle->id,discard_count);
									}


									} //end of if-1.1.1.2.1.1
									
								} //end of if-1.1.1.2.1
								else if((param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD || param->vanet_forwarding_scheme == VANET_FORWARDING_TADB) && does_vehicle_convoy_have_packet(vehicle, param)) //if-1.1.1.2.2
								{
									flag = VADD_Is_There_Next_Carrier_At_Intersection(param, current_time, vehicle, Gr_set[vehicle->ptr_convoy_queue_node->leader_vehicle->latest_packet_ptr->target_point_id-1], Gr_set_size[vehicle->ptr_convoy_queue_node->leader_vehicle->latest_packet_ptr->target_point_id-1], &FTQ, &next_carrier);
									if(flag) //if-1.1.1.2.2.1
									{
										if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY && vehicle->flag_convoy_registration == TRUE) //if-1.1.1.2.2.1.1
										{ 
											VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle->ptr_convoy_queue_node->leader_vehicle, next_carrier->ptr_convoy_queue_node->leader_vehicle, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
										} //end of if-1.1.1.2.2.1.1
										else //else-1.1.1.2.2.1.2
										{
											VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
										} //end of else-1.1.1.2.2.1.2
									} //end of if-1.1.1.2.2.1
								} //end of if-1.1.1.2.2
								else if(param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC) //if-1.1.1.2.3
								{
									if(does_vehicle_convoy_have_packet(vehicle, param)) //if-1.1.1.2.3.1
									{
										/* let vehicle's convoy head give its packet copy to the convoy head of each convoy outgoing from this intersection */
										EPIDEMIC_Perform_Packet_Dissemination_At_Intersection(param, current_time, vehicle, Gr_set[vehicle->ptr_convoy_queue_node->leader_vehicle->latest_packet_ptr->target_point_id-1], Gr_set_size[vehicle->ptr_convoy_queue_node->leader_vehicle->latest_packet_ptr->target_point_id-1], &packet_delivery_statistics);
									} //end of if-1.1.1.2.3.1
									
									if(does_vehicle_have_packet(vehicle)) //if-1.1.1.2.3.2
									{
										/* let vehicle give its packet copy to the convoy head of each convoy outgoing from this intersection */
										EPIDEMIC_Perform_Packet_Dissemination_At_Intersection(param, current_time, vehicle, Gr_set[vehicle->latest_packet_ptr->target_point_id-1], Gr_set_size[vehicle->latest_packet_ptr->target_point_id-1], &packet_delivery_statistics);
	
									} //end of if-1.1.1.2.3.2
								} //end of if-1.1.1.2.3
							} //end of else if-1.1.1.2
						} //end of if-1.1.1
				
						/** delete the vehicle's movement from the directional edge queue's vehicle_movement_list */
						if(flag_packet_log) //if-1.1.2
						{
							if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY && vehicle->flag_convoy_registration == TRUE)
							{
								convoy_leave(param, vehicle, current_time); //let the vehicle leave from the convoy on the current directional edge
							}

							if(param->vehicle_vanet_acl_measurement_flag != TRUE)
								delete_vehicle_movement(param, vehicle, current_time, Gr);
						} //end of if-1.1.2

						if(param->vehicle_vanet_acl_measurement_flag)
							delete_vehicle_movement(param, vehicle, current_time, Gr);

						/** update the statistics for branch probability and vehicular traffic densidy */
						pGraphNode = GetNeighborGraphNode(Gr, Gr_size, vehicle->path_ptr->vertex, vehicle->path_ptr->next->vertex); //return the pointer to graph node corresponding to vertex vehicle->path_ptr->next->vertex that is vehicle->path_ptr->vertex's neighbor

						if(pGraphNode->number_of_branching == 0)
						{
							pGraphNode->last_arrival_time = current_time;
						}
						else
						{
							pGraphNode->number_of_interarrivals++;
							pGraphNode->sum_of_interarrival_time += current_time - pGraphNode->last_arrival_time;
							pGraphNode->last_arrival_time = current_time;
						}

						pGraphNode->number_of_branching++; //increment the number of branching from the tail node to the head node in this edge
						/*******************************************/

						/* vehicle moves to the next vertex on the path by updating the path position and increase path_current_hop by one */
						vehicle->path_ptr = vehicle->path_ptr->next;
						vehicle->path_current_hop++;

						/** For the branch probability computation, we increment the visiting count of the vertex corresponding to vehicle->path_ptr->prev */
						pGraphNode = LookupGraph(Gr, Gr_size, vehicle->path_ptr->vertex);
						pGraphNode->number_of_arrivals++;
						/*******************************/

						/* Under the Download mode, update the destination vehicle's latest passing time for the current intersection */
						if((param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD) && (vehicle->id == destination_vehicle_id))
						{
							Update_DestinationVehicle_PassingTime_At_StationaryNode(param, current_time, vehicle, vehicle->path_ptr->vertex);
						}

						/* check whether the vehicle has arrived at its destination */
						if(IsEndOfTravel(vehicle->path_list, vehicle->path_ptr) == TRUE) //if-3
						{
							/**@for debugging */
							//if(id == 1)
							//  printf("at time=%.1f, vehicle(id=%d) is tracked\n", (float)current_time, id);
							/******************/

							if(param->vehicle_vanet_vehicular_traffic_model == VANET_VEHICULAR_TRAFFIC_OPEN_NETWORK)
								state = VEHICLE_ESCAPE;
							else if(param->vehicle_vanet_vehicular_traffic_model == VANET_VEHICULAR_TRAFFIC_CLOSED_NETWORK) 
							{
								/* we select vehicle restart state according to data forwarding mode and target vehicle's id */
								if((param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD || param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V) && Is_Destination_Vehicle(&DVQ, vehicle->id))
								{	
									state = VEHICLE_TARGET_VEHICLE_RESTART;
								}
								else 
								{
									state = VEHICLE_RESTART;
								}
							}
							else
							{
								printf("main(): Error: Unknown vehicular traffic model (%d)\n", param->vehicle_vanet_vehicular_traffic_model);
								exit(1);
							}

							/* At the next state, we determine think time (i.e., waiting time) at an intersection according to the think time distribution */
							delay = 0.0;

							schedule(state, delay, id);
							break; //Note: don't omit "break" to skip the remaining code in this case block!
						} //end of if-3
						else //else-3
						{
							/** update the current position and movement type */
							tail_node = vehicle->path_ptr->vertex;
							head_node = vehicle->path_ptr->next->vertex;
							vehicle->current_pos_in_Gr.eid = FastGetEdgeID_MoveType(Gr, tail_node, head_node, &move_type, &edge_length, &ptr_directional_edge_node);
							vehicle->move_type = move_type;
							vehicle->edge_length = edge_length;
							vehicle->path_current_edge_offset = 0; //reset path_current_edge_offset

							if(move_type == MOVE_FORWARD)
								vehicle->current_pos_in_Gr.offset = 0;
							else if(move_type == MOVE_BACKWARD)
								vehicle->current_pos_in_Gr.offset = edge_length;
							else
							{
								printf("move_type(%d) is invalid\n", move_type);
#ifdef __DEBUG_INTERACTIVE_MODE__
								fgetc(stdin);
#endif
								exit(1); 
							}

							/* update vehicle's current position in vehicle's digraph */
							strcpy(vehicle->current_pos_in_digraph.tail_node, tail_node);
							strcpy(vehicle->current_pos_in_digraph.head_node, head_node);
							vehicle->current_pos_in_digraph.offset = 0;
							if(ptr_directional_edge_node == NULL)
							{
								printf("%s:%d ptr_directional_edge_node for edge (%s,%s) is NULL\n",
										__FUNCTION__, __LINE__,
										tail_node, head_node);
								exit(1);
							}
							else
							{
								vehicle->current_pos_in_digraph.enode = ptr_directional_edge_node;
								vehicle->current_pos_in_digraph.eid = ptr_directional_edge_node->eid;
							}

							/* set the vehicle's current position in the 2-D Cartesian coordinate system */
							tail_gnode_in_node_array = LookupGraph(Gr, Gr_size, tail_node);

							vehicle->current_pos.x = tail_gnode_in_node_array->coordinate.x;
							vehicle->current_pos.y = tail_gnode_in_node_array->coordinate.y;

							/** compute the position update vector to update the vehicle's geometric position by movement time, such as STEP_TIME */
							get_position_update_vector(vehicle, Gr, Gr_size);

							/** register the vehicle's movement into the directional edge queue's vehicle_movement_list */
							if(param->vehicle_vanet_acl_measurement_flag)
								register_vehicle_movement(param, vehicle, current_time, Gr);

							if(flag_packet_log)
							{
								if(param->vehicle_vanet_acl_measurement_flag != TRUE)
									register_vehicle_movement(param, vehicle, current_time, Gr);

								/* check whether data forwarding mode is download or upload */
								if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
								{
									/* obtain the pointer to the first access point */
									pAP =  GetFirstAccessPoint(&APQ);
									if(pAP == NULL)
									{
										printf("After GetFirstAccessPoint(), pAP is NULL!\n");
										exit(1);
									}

									VADD_Update_Vehicle_EDD_And_EDD_SD(current_time, param, vehicle, pAP->Ga, pAP->Ga_size, &(pAP->ap_table_for_target_point)); //compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge along with real graph Gr and AP table				      				  
								}
								else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
								{
									VADD_Update_Vehicle_EDD_And_EDD_SD(current_time, param, vehicle, Gr, Gr_size, &ap_table_for_Gr);
									//compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge along with real graph Gr and AP table
								}
								else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
								{
									if(param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD || param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC
									|| param->vanet_forwarding_scheme == VANET_FORWARDING_TADB)
									{
										VADD_Update_Vehicle_EDD_And_EDD_SD(current_time, param, vehicle, Gr, Gr_size, &ap_table_for_Gr);
	
									}
								}

								if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY)
{
									convoy_join(param, vehicle, current_time, &packet_delivery_statistics); //join vehicle into the convoy preceding the vehicle on the directional edge if the vehicle is within the communication range of the convoy
								}
							}
						} //end of else-3
					} //end of if-2
					else //else-2
					{
						if(flag_packet_log) //if-2.1
						{
							/** update the convoys close to the vehicle */
							if(param->vehicle_speed_distribution != EQUAL)
								convoy_update(param, vehicle, current_time);

							/** check whether the vehicle has packets to send */
							if(does_vehicle_have_packet(vehicle)) //if-2.1.1
							{
								/** determine whether to forward its packets to next carrier moving on the same road segment  and return the pointer to the next carrier through next_carrier */
								if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD || param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD) //if-2.1.1.1
								{
									flag = VADD_Is_There_Next_Carrier_On_Road_Segment(param, current_time, vehicle, Gr, Gr_size, &next_carrier);
									if(flag) //if-2.1.1.1.1
									{
										/* taehwan 20140730 */
										printf("Hi %d~!!!!\n",next_carrier->id);
								    	
										if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY) //if-2.1.1.1.1.1
										{
											VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier->ptr_convoy_queue_node->leader_vehicle, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier's convoy head
										} //end of if-2.1.1.1.1.1
										else //else-2.1.1.1.1.2
										{
											VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
										} //end of else-2.1.1.1.1.2
									} //end of 2.1.1.1.1
								} //end of if-2.1.1.1
								else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V) //else if-2.1.1.2
								{
									if(param->vanet_forwarding_scheme == VANET_FORWARDING_TPD) //if-2.1.1.1.1
									{
									        //printf("@@@ TPD_NCOR\n");
										flag = TPD_Is_There_Next_Carrier_On_Road_Segment(param, current_time, vehicle, Gr, Gr_size, &next_carrier);
										if(flag) //if-2.1.1.1.1.1
										{

											/* taehwan 20140730 */
											printf("Hi %d~!!!!\n",next_carrier->id);
								    		printf("TPD find next carrier (%d) on road segment, arrived at(%.2f)\n"
												,next_carrier->id
												,next_carrier->arrival_time);
											
											VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
									
											/* taehwan 20140730 */
											// check packet ttl has expired
											if (discard_count > 0)
											{
												printf("TTL EXPIRED AT AP On road 3676! - %d %d\n",vehicle->id,discard_count);
											}


										} //end of if-2.1.1.1.1.1
									} //end of if-2.1.1.1.1
									else if(param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD
									|| param->vanet_forwarding_scheme == VANET_FORWARDING_TADB) //else if-2.1.1.1.2
									{
										flag = VADD_Is_There_Next_Carrier_On_Road_Segment(param, current_time, vehicle, Gr, Gr_size, &next_carrier);
										if(flag) //if-2.1.1.1.2.1
										{
											printf("VADD_Forward_Packet_To_Next_Carrier : %3d (%2s->%2s) -> %3d (%2s->%2s) : %s\n",
												vehicle->id,vehicle->current_pos_in_digraph.tail_node,vehicle->current_pos_in_digraph.head_node,
												next_carrier->id,next_carrier->current_pos_in_digraph.tail_node,next_carrier->current_pos_in_digraph.head_node,
												(vehicle->current_pos_in_digraph.eid == next_carrier->current_pos_in_digraph.eid ? "Forward":"Encounter"));
											toggle_forward_color();	
											if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY) //if-2.1.1.1.2.1.1
											{
												VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier->ptr_convoy_queue_node->leader_vehicle, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier's convoy head
											} //end of if-2.1.1.1.2.1.1
											else //else-2.1.1.1.2.1.2
											{
												VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
											} //end of else-2.1.1.1.2.1.2
										} //end of if-2.1.1.1.2.1
									} //end of else if-2.1.1.1.2
									else if(param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC) //else if-2.1.1.1.3

									{
										/* Packet Dissemination in Road Segment by Epidemic Routing */
										if(does_vehicle_convoy_have_packet(vehicle, param)) //if-2.1.1.1.3.1
										{
											EPIDEMIC_Perform_Packet_Dissemination_On_Road_Segment(param, current_time, vehicle, Gr_set[vehicle->ptr_convoy_queue_node->leader_vehicle->latest_packet_ptr->target_point_id-1], Gr_set_size[vehicle->ptr_convoy_queue_node->leader_vehicle->latest_packet_ptr->target_point_id-1], &packet_delivery_statistics);
										} //end of if-2.1.1.1.3.1
										
										if(does_vehicle_have_packet(vehicle)) //if-2.1.1.1.3.2
										{
											EPIDEMIC_Perform_Packet_Dissemination_On_Road_Segment(param, current_time, vehicle, Gr_set[vehicle->latest_packet_ptr->target_point_id-1], Gr_set_size[vehicle->latest_packet_ptr->target_point_id-1], &packet_delivery_statistics);
										} //end of if-2.1.1.1.3.2
									} //end of else if-2.1.1.1.3
								} //end of else if-2.1.1.2
							} //end of if-2.1.1
						} //end of if-2.1
					} //end of else-2
				} //end if-1
				/******************************************************/
				/*####################################################*/
				/******************************************************/
				else if(vehicle->move_type == MOVE_BACKWARD) //else if-2
				{
					if(vehicle->current_pos_in_Gr.offset <= offset_tolerance) //if-2.1
					/* if(vehicle->current_pos_in_Gr.offset <= 0) //if-4 => "<= 0" makes an infinite loop between VEHICLE_CHECK and VEHICLE_MOVE.; refer to version.txt: [v1.1.1-2007-9-20]'s bullet 2. */
					{ //the case where the vehicle arrives at the intersection
#if TPD_VEHICLE_MOBILITY_TRACE_FLAG
						/** TRACE vehicle mobility */
						if((current_time > 7200) && (vehicle->id == 86))
						{
							printf("%s:%d: TRACE [%.2f] vehicle(%d)'s position is <%s, %s: %0.f> and its speed is %0.f\n",
									__FUNCTION__, __LINE__,
									(float)current_time,
									vehicle->id, 
									vehicle->current_pos_in_digraph.tail_node,
									vehicle->current_pos_in_digraph.head_node,
									vehicle->current_pos_in_digraph.offset,
									vehicle->speed);
						}
				/***************************/
#endif

						/** set intersection_flag to TRUE to indicate that vehicle arrices at an intersection */
						intersection_visit_flag = TRUE;

						/* update vehicle's speed at the intersection using the speed distribution considering vehicle minimum speed and vehicle maximum speed */
#if TPD_VEHICLE_SPEED_UPDATE_PER_ROAD_SEGMENT_FLAG
						set_vehicle_speed(param, vehicle);
#endif

						/* only when flag_packet_log is set to TRUE, perform the packet forwarding */
						if(flag_packet_log && (does_vehicle_convoy_have_packet(vehicle, param) || does_vehicle_have_packet(vehicle))) //if-2.1.1

						//if(flag_packet_log && does_vehicle_convoy_have_packet(vehicle, param)) //if-2.1.1
						{
							/** determine whether to forward its packets to the best next carrier moving on the other road segment of the intersection and return the pointer to the next carrier through next_carrier */
							if((param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD || param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD) && does_vehicle_convoy_have_packet(vehicle, param)) //if-2.1.1.1
							{
								flag = VADD_Is_There_Next_Carrier_At_Intersection(param, current_time, vehicle, Gr, Gr_size, &FTQ, &next_carrier);
								if(flag) //if-2.1.1.1.1
								{
									if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY && vehicle->flag_convoy_registration == TRUE) //if-2.1.1.1.1.1
									{
										VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle->ptr_convoy_queue_node->leader_vehicle, next_carrier->ptr_convoy_queue_node->leader_vehicle, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
									} //end of if-2.1.1.1.1.1
									else //else-2.1.1.1.1.2
									{
										VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
									} //end of else-2.1.1.1.1.2
								} //end of if-2.1.1.1.1
							} //end of if-2.1.1.1
							else if((param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V) && (vehicle->type == VEHICLE_CURRENT_PACKET_CARRIER)) //else if-2.1.1.2
							{
								if(param->vanet_forwarding_scheme == VANET_FORWARDING_TPD && does_vehicle_have_packet(vehicle)) //if-2.1.1.2.1
								{
									//printf("@@@ TPDISTNCAI\n");
									flag = TPD_Is_There_Next_Carrier_At_Intersection(param, current_time, vehicle, Gr, Gr_size, &next_carrier);
									if(flag) //if-2.1.1.2.1.1
									{

										printf("TPD find next carrier (%d) at intersection, arrived at(%.2f)\n"
												,next_carrier->id
												,next_carrier->arrival_time);
																				
										VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
											
										/* taehwan 20140730 */
										// check packet ttl has expired
										if (discard_count > 0)
										{
											printf("TTL EXPIRED AT Intersection 3790 ! - %d %d\n",vehicle->id,discard_count);
										}


									} //end of if-2.1.1.2.1.1
								} //end of if-2.1.1.2.1
								else if((param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD || param->vanet_forwarding_scheme == VANET_FORWARDING_TADB) && does_vehicle_convoy_have_packet(vehicle, param)) //if-2.1.1.2.2
								{
									flag = VADD_Is_There_Next_Carrier_At_Intersection(param, current_time, vehicle, Gr_set[vehicle->ptr_convoy_queue_node->leader_vehicle->latest_packet_ptr->target_point_id-1], Gr_set_size[vehicle->ptr_convoy_queue_node->leader_vehicle->latest_packet_ptr->target_point_id-1], &FTQ, &next_carrier);
									if(flag) //if-2.1.1.2.2.1
									{
										if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY && vehicle->flag_convoy_registration == TRUE) //if-2.1.1.2.2.1.1
										{ 
											VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle->ptr_convoy_queue_node->leader_vehicle, next_carrier->ptr_convoy_queue_node->leader_vehicle, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
										} //end of if-2.1.1.2.2.1.1
										else //else-2.1.1.2.2.1.2
										{
											VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
										} //end of else-2.1.1.2.2.1.2
									} //end of if-2.1.1.2.2.1
								} //end of if-2.1.1.2.2
								else if(param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC) //if-2.1.1.2.3
								{
									if(does_vehicle_convoy_have_packet(vehicle, param)) //if-2.1.1.2.3.1
									{
										/* let vehicle's convoy head give its packet copy to the convoy head of each convoy outgoing from this intersection */
										EPIDEMIC_Perform_Packet_Dissemination_At_Intersection(param, current_time, vehicle, Gr_set[vehicle->ptr_convoy_queue_node->leader_vehicle->latest_packet_ptr->target_point_id-1], Gr_set_size[vehicle->ptr_convoy_queue_node->leader_vehicle->latest_packet_ptr->target_point_id-1], &packet_delivery_statistics);
									} //end of if-2.1.1.2.3.1
									else //else-2.1.1.2.3.2
									{
										/* let vehicle give its packet copy to the convoy head of each convoy outgoing from this intersection */
										EPIDEMIC_Perform_Packet_Dissemination_At_Intersection(param, current_time, vehicle, Gr_set[vehicle->latest_packet_ptr->target_point_id-1], Gr_set_size[vehicle->latest_packet_ptr->target_point_id-1], &packet_delivery_statistics);
	
									} //end of else-2.1.1.2.3.2
								} //end of if-2.1.1.2.3
							} //end of else if-2.1.1.2
						} //end of if-2.1.1

						/** delete the vehicle's movement from the directional edge queue's vehicle_movement_list */
						if(flag_packet_log) //if-2.1.2
						{ 
							if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY && vehicle->flag_convoy_registration == TRUE)
							{
								convoy_leave(param, vehicle, current_time); //let the vehicle leave from the convoy on the current directional edge
							}
                          
							if(param->vehicle_vanet_acl_measurement_flag != TRUE)
								delete_vehicle_movement(param, vehicle, current_time, Gr);
						} //end of if-2.1.2

						if(param->vehicle_vanet_acl_measurement_flag)
							delete_vehicle_movement(param, vehicle, current_time, Gr);

						/** update the statistics for branch probability and vehicular traffic densidy */
						pGraphNode = GetNeighborGraphNode(Gr, Gr_size, vehicle->path_ptr->vertex, vehicle->path_ptr->next->vertex); //return the pointer to graph node corresponding to vertex vehicle->path_ptr->next->vertex that is vehicle->path_ptr->vertex's neighbor

						if(pGraphNode->number_of_branching == 0)
						{
							pGraphNode->last_arrival_time = current_time;
						}
						else
						{
							pGraphNode->number_of_interarrivals++;
							pGraphNode->sum_of_interarrival_time += current_time - pGraphNode->last_arrival_time;
							pGraphNode->last_arrival_time = current_time;
						}
						
						pGraphNode->number_of_branching++; //increment the number of branching from the tail node to the head node in this edge
						
						/*******************************************/

						/* vehicle moves to the next vertex on the path by updating the path position and increase path_current_hop by one */
						vehicle->path_ptr = vehicle->path_ptr->next;
						vehicle->path_current_hop++;

						/** For the branch probability computation, we increment the visiting count of the vertex corresponding to vehicle->path_ptr->prev */
						pGraphNode = LookupGraph(Gr, Gr_size, vehicle->path_ptr->vertex);
						pGraphNode->number_of_arrivals++;
						/*******************************/

						/* Under the Download mode, update the destination vehicle's latest passing time for the current intersection */
						if((param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD) && (vehicle->id == destination_vehicle_id))
						{
							Update_DestinationVehicle_PassingTime_At_StationaryNode(param, current_time, vehicle, vehicle->path_ptr->vertex);
						}
						
						/* check whether the vehicle has arrived at its destination */
						if(IsEndOfTravel(vehicle->path_list, vehicle->path_ptr) == TRUE) //if-5
						{
							if(param->vehicle_vanet_vehicular_traffic_model == VANET_VEHICULAR_TRAFFIC_OPEN_NETWORK)
								state = VEHICLE_ESCAPE;
							else if(param->vehicle_vanet_vehicular_traffic_model == VANET_VEHICULAR_TRAFFIC_CLOSED_NETWORK) 
							{
								/* we select vehicle restart state according to data forwarding mode and target vehicle's id */
								if((param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD || param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V) && Is_Destination_Vehicle(&DVQ, vehicle->id))
									state = VEHICLE_TARGET_VEHICLE_RESTART;
								else
									state = VEHICLE_RESTART;
							}
							else
							{
								printf("main(): Error: Unknown vehicular traffic model (%d)\n", param->vehicle_vanet_vehicular_traffic_model);
								exit(1);
							}

							/* At the next state, we determine think time (i.e., waiting time) at an intersection according to the think time distribution */
							delay = 0.0; //let the vehicle move to the next state, letting the vehicle wait for a random time at the next state

							schedule(state, delay, id);
							break; //Note: don't omit "break" to skip the remaining code in this case block!
						} //end of if-5
						else //else-5
						{
							/** update the current position and movement type */
							tail_node = vehicle->path_ptr->vertex;
							head_node = vehicle->path_ptr->next->vertex;
							vehicle->current_pos_in_Gr.eid = FastGetEdgeID_MoveType(Gr, tail_node, head_node, &move_type, &edge_length, &ptr_directional_edge_node);
							vehicle->move_type = move_type;
							vehicle->edge_length = edge_length;
							vehicle->path_current_edge_offset = 0; //reset path_current_edge_offset

							if(move_type == MOVE_FORWARD)
								vehicle->current_pos_in_Gr.offset = 0;
							else if(move_type == MOVE_BACKWARD)
								vehicle->current_pos_in_Gr.offset = edge_length;
							else
							{
								printf("move_type(%d) is invalid\n", move_type);
#ifdef __DEBUG_INTERACTIVE_MODE__
								fgetc(stdin);
#endif
								exit(1); 
							}

							/* update vehicle's current position in vehicle's digraph */
							strcpy(vehicle->current_pos_in_digraph.tail_node, tail_node);
							strcpy(vehicle->current_pos_in_digraph.head_node, head_node);
							vehicle->current_pos_in_digraph.offset = 0;
							if(ptr_directional_edge_node == NULL)
							{
								printf("%s:%d ptr_directional_edge_node for edge (%s,%s) is NULL\n",
										__FUNCTION__, __LINE__,
										tail_node, head_node);
								exit(1);
							}
							else
							{
								vehicle->current_pos_in_digraph.enode = ptr_directional_edge_node;
								vehicle->current_pos_in_digraph.eid = ptr_directional_edge_node->eid;
							}

							/* set the vehicle's current position in the 2-D Cartesian coordinate system */
							tail_gnode_in_node_array = LookupGraph(Gr, Gr_size, tail_node);

							vehicle->current_pos.x = tail_gnode_in_node_array->coordinate.x;
							vehicle->current_pos.y = tail_gnode_in_node_array->coordinate.y;

							/** compute the position update vector to update the vehicle's geometric position by movement time, such as STEP_TIME */
							get_position_update_vector(vehicle, Gr, Gr_size);

							/** register the vehicle's movement into the directional edge queue's vehicle_movement_list */
							if(param->vehicle_vanet_acl_measurement_flag)
								register_vehicle_movement(param, vehicle, current_time, Gr);

							if(flag_packet_log)
							{
								if(param->vehicle_vanet_acl_measurement_flag != TRUE)
									register_vehicle_movement(param, vehicle, current_time, Gr);

								/* check whether data forwarding mode is download or upload */
								if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
								{
									/* obtain the pointer to the first access point */
									pAP =  GetFirstAccessPoint(&APQ);
									if(pAP == NULL)
									{
										printf("After GetFirstAccessPoint(), pAP is NULL!\n");
										exit(1);
									}

									VADD_Update_Vehicle_EDD_And_EDD_SD(current_time, param, vehicle, pAP->Ga, pAP->Ga_size, &(pAP->ap_table_for_target_point)); //compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge along with real graph Gr and AP table				      				  
								}
								else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
								{
									VADD_Update_Vehicle_EDD_And_EDD_SD(current_time, param, vehicle, Gr, Gr_size, &ap_table_for_Gr);
									//compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge along with real graph Gr and AP table
								}
								else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
								{
									if(param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD || param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC
									|| param->vanet_forwarding_scheme == VANET_FORWARDING_TADB)
									{
										VADD_Update_Vehicle_EDD_And_EDD_SD(current_time, param, vehicle, Gr, Gr_size, &ap_table_for_Gr);
									}
								}

								if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY)
								{
									convoy_join(param, vehicle, current_time, &packet_delivery_statistics); //join vehicle into the convoy preceding the vehicle on the directional edge if the vehicle is within the communication range of the convoy
								}
							}
						} //end of else-5
					} //end of if-4
					else //else-4
					{
						if(flag_packet_log) //if-4.1
						{
							/** update the convoys close to the vehicle */
							if(param->vehicle_speed_distribution != EQUAL)
								convoy_update(param, vehicle, current_time);

							/** check whether the vehicle has packets to send */
							if(does_vehicle_have_packet(vehicle)) //if-4.1.1
							{
								/** determine whether to forward its packets to next carrier moving on the same road segment  and return the pointer to the next carrier through next_carrier */
								if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD || param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD) //if-4.1.1.1
								{
									flag = VADD_Is_There_Next_Carrier_On_Road_Segment(param, current_time, vehicle, Gr, Gr_size, &next_carrier);
									if(flag) //if-4.1.1.1.1
									{

										/* taehwan 20140730 */
										printf("Hi %d~!!!!\n",next_carrier->id);
								    	
										if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY) //if-4.1.1.1.1.1
										{
											VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier->ptr_convoy_queue_node->leader_vehicle, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier's convoy head
										} //end of if-4.1.1.1.1.1
										else //else-4.1.1.1.1.2
										{
											VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
										} //end of else-4.1.1.1.1.2
									} //end of if-4.1.1.1.1
								} //end of if-4.1.1.1
								else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V) //else if-4.1.1.2
								{
									if(param->vanet_forwarding_scheme == VANET_FORWARDING_TPD) //if-4.1.1.1.1
									{
										//printf("@@@@@ ONROAD\n");
										flag = TPD_Is_There_Next_Carrier_On_Road_Segment(param, current_time, vehicle, Gr, Gr_size, &next_carrier);
										if(flag) //if-4.1.1.1.1.1
										{

											/* taehwan 20140730 */
											printf("Hi %d~!!!!\n",next_carrier->id);
								    	
											printf("TPD find next carrier (%d) on road segment, arrived at(%.2f)\n"
												,next_carrier->id
												,next_carrier->arrival_time);
																					
											//vehicle forwards its packet(s) to the next carrier pointed by next_carrier
											VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier, &packet_delivery_statistics, &discard_count); 
											
										} //end of if-4.1.1.1.1.1
									} //end of if-4.1.1.1.1
									else if(param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD ||
									        param->vanet_forwarding_scheme == VANET_FORWARDING_TADB) //else if-4.1.1.1.2
									{
										flag = VADD_Is_There_Next_Carrier_On_Road_Segment(param, current_time, vehicle, Gr, Gr_size, &next_carrier);
										if(flag) //if-4.1.1.1.2.1
										{
											printf("VADD_Forward_Packet_To_Next_Carrier : %3d (%2s->%2s) -> %3d (%2s->%2s) : %s\n",
												vehicle->id,vehicle->current_pos_in_digraph.tail_node,vehicle->current_pos_in_digraph.head_node,
												next_carrier->id,next_carrier->current_pos_in_digraph.tail_node,next_carrier->current_pos_in_digraph.head_node,
												(vehicle->current_pos_in_digraph.eid == next_carrier->current_pos_in_digraph.eid ? "Forward":"Encounter"));
											toggle_forward_color();
											if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY) //if-4.1.1.1.2.1.1
											{
												VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier->ptr_convoy_queue_node->leader_vehicle, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier's convoy head
											} //end of if-4.1.1.1.2.1.1
											else //else-4.1.1.1.2.1.2
											{
												VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
											} //end of else-4.1.1.1.2.1.2
										} //end of if-4.1.1.1.2.1
									} //end of else if-4.1.1.1.2
									else if(param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC) //else if-4.1.1.1.3
									{
										/* Packet Dissemination in Road Segment by Epidemic Routing */
										if(does_vehicle_convoy_have_packet(vehicle, param)) //if-4.1.1.1.3.1
										{
											EPIDEMIC_Perform_Packet_Dissemination_On_Road_Segment(param, current_time, vehicle, Gr_set[vehicle->ptr_convoy_queue_node->leader_vehicle->latest_packet_ptr->target_point_id-1], Gr_set_size[vehicle->ptr_convoy_queue_node->leader_vehicle->latest_packet_ptr->target_point_id-1], &packet_delivery_statistics);
										} //end of if-4.1.1.1.3.1
										else //else-4.1.1.1.3.2
										{
											EPIDEMIC_Perform_Packet_Dissemination_On_Road_Segment(param, current_time, vehicle, Gr_set[vehicle->latest_packet_ptr->target_point_id-1], Gr_set_size[vehicle->latest_packet_ptr->target_point_id-1], &packet_delivery_statistics);
										} //end of else-4.1.1.1.3.2
									} //end of else if-4.1.1.1.3
								} //end of else if-4.1.1.2
							} //end of if-4.1.1
						} //end of if-4.1
					} //end of else-4
				} //end of else-if-1
				else //else-1
				{
					printf("vehicle->move_type(%d) is invalid\n", vehicle->move_type);
#ifdef __DEBUG_INTERACTIVE_MODE__
					fgetc(stdin);
#endif
					exit(1);
				} //end of else-1

				/* vehicle completes the movement to the next vertex on the path by updating the path position */
				//vehicle->path_ptr = vehicle->path_ptr->next;

				/* according to intersection_visit_flag, we prepare an appropriate waiting time at the intersection or at the road segment */
				if(intersection_visit_flag)
				{
					if(param->vehicle_think_time > 0)
						delay =  delay_func(param, DELAY_VEHICLE_THINK_TIME);
					else
						delay = 0; //let vehicle wait for a random time before going to the state VEHICLE_CHECK

					/** reset intersection_flag to FALSE when vehicle is leaving from an intersection */
					intersection_visit_flag = FALSE;
				}	
				else
				{
					delay = 0; //let vehicle go to the state VEHICLE_CHECK
				}

				{
				int tT = atoi(vehicle->current_pos_in_digraph.tail_node);
				int tH = atoi(vehicle->current_pos_in_digraph.head_node);

				/* taehwan 20140724 */
				/* Update Vehicle Point */
				g_x=0;
				g_y=0;

				convertDigraphToCoordinate(
						tT,
						tH,
						vehicle->current_pos_in_digraph.offset,
						vehicle->edge_length);

				g_vehicle_point[vehicle->id][0] = g_x;
				g_vehicle_point[vehicle->id][1] = g_y;
                g_vehicle_point[vehicle->id][2] = g_direction;

				if ( does_vehicle_have_packet(vehicle) == TRUE)
				{
					g_vehicle_have_packet[vehicle->id] = 1;
				} else
				{
					g_vehicle_have_packet[vehicle->id] = 0;				
				}

				g_current_time = current_time;
				
				
				if (g_gnuplot_option == 2 && vehicle->current_pos_in_digraph.offset > 0)
					gnuplot_vehicle_point(current_time);

				/* taehwan 20140719 */
				if (vehicle->id != 1) {	
				
					if (vehicle->id < VEHICLE_COUNT_MAX && vehicle->id >= 0)
					{
						// increase visit count only when visiting new intersection 
						if (g_vehicle_current_segment_location[vehicle->id] != tH)
						{
							g_vehicle_current_segment_location[vehicle->id] == tH;
							if (tH+7==tT)
							{
								g_segment_visit_count[tH][0]++;
							} else if (tH+1==tT)
							{
								g_segment_visit_count[tH][1]++;
							} else if (tH-7==tT)
							{
								g_segment_visit_count[tH][2]++;
							} else if (tH-1==tT)
							{
								g_segment_visit_count[tH][3]++;
							}

							if (g_gnuplot_option == 1)
								gnuplot_intersection_visit_count();
						}

					}	
				}
				}
				// taehwan 20140711
				/*if ((vehicle->id == 22 || vehicle->id == 1) 
						&& (current_time > 23680 && current_time <23730))
				{
					printf("%.2f,%d,%s,%s,%0.f,%0.f,%d\n",
						current_time,
						vehicle->id,
						vehicle->current_pos_in_digraph.tail_node,
						vehicle->current_pos_in_digraph.head_node,
						vehicle->current_pos_in_digraph.offset,
						vehicle->edge_length,
						isInRange);
				}*/
				
				/* taehwan 20140714 print current pos of vehicle */
				/*
				if ((vehicle->id == 71 || vehicle->id == 77 || vehicle->id == 1) &&
					(current_time < 7900 && current_time > 7200))
				{
					printf("%.2f,%d,%s,%s,%0.f,%0.f,%d\n",
						current_time,
						vehicle->id,
						vehicle->current_pos_in_digraph.tail_node,
						vehicle->current_pos_in_digraph.head_node,
						vehicle->current_pos_in_digraph.offset,
						vehicle->edge_length,
						isInRange);
				}
				*/
				setLogOnOff(FALSE);				
				/* transit to VEHICLE_CHECK state */
				schedule(VEHICLE_CHECK, delay, id);

				break;
/***************************************************/
/*#################################################*/
/***************************************************/

			case VEHICLE_ESCAPE:
			        //number_of_escaped_vehicles++;
				current_time = smpl_time();

				vehicle = vehicle_search(id);
				if(vehicle == NULL)
				{
					printf("case VEHICLE_ESCAPE: there is no vehicle node corresponding to id %d\n", id);
#ifdef __DEBUG_INTERACTIVE_MODE__
					fgetc(stdin);
#endif
					exit(1);
				}

				/* update vehicle's state and state_time */
				vehicle->state = VEHICLE_ESCAPE;
				vehicle->state_time = current_time;

				/* make statistics information, such as addition of movement time */
				movement_time = current_time - vehicle->restart_time;

/*@ for debugging */
#ifdef __DEBUG_LEVEL_VEHICLE_ESCAPE__
				//printf("[time: %f] vehicle %d with restart time %f and speed %f exits with its movement time %f without detection!\n", (float) current_time, id, (float) vehicle->restart_time, (float) vehicle->speed, (float) movement_time);
				fprintf(fp_1, "[time: %f] vehicle %d with restart time %f and speed %f exits with its movement time %f without detection!\n", (float) current_time, id, (float) vehicle->restart_time, (float) vehicle->speed, (float) movement_time);
#endif

#ifdef __LOG_LEVEL_TRACE_OF_VEHICLE_CONVOY_LENGTH__

				/*@VANET: check whether the escape of this vehicle is first or not:
				  if this escape is first, start the logging for the convoy length */
			  
				/*@VANET: update convoy information */
				if((flag_of_vanet_logging_start == FALSE) && (convoy_head == id))
				{ //vehicle is the first arrived vehicle and the convoy head

				  /** find the vehicle with the greatest offset for the next convoy head */
				  /* sort the vehicle movement list of the directional edge <1,2> */

				  /* obtain the pointer to the vehicle at the rear of the vehicle movement list */
				  
				  /* change the convoy_head with the following vehicle id */
				  convoy_head = id+1;

				  /* adjust the convoy_start_time with the following vehicle's arrival time */
				  vehicle = vehicle_search(convoy_head);
				  if(vehicle == NULL)
				  {
				    printf("case VEHICLE_ESCAPE: vehicle(id=%d) following convoy_head(id=%d) has not arrived yet\n", convoy_head, id);
				    break;
				  }

				  convoy_vehicle_number--; //decrease the convoy-vehicle-number
				  convoy_start_time = current_time; //set the convoy-start-time to the current time
				  convoy_start_length = (current_time - vehicle->arrival_time)*vehicle->speed;
				  //vehicle's current position becomes the new convoy's length

				  /* set flag_of_vanet_logging_start to TRUE since the first arrived vehicle has departed this road segment */
				  flag_of_vanet_logging_start = TRUE;
				}
				else if((flag_of_vanet_logging_start == FALSE) && (convoy_head != id))
				{ //vehicle is the first arrived vehicle but is not the convoy head
				  /* set flag_of_vanet_logging_start to TRUE since the first arrived vehicle has departed this road segment */
				  flag_of_vanet_logging_start = TRUE;
				}
				else if((flag_of_vanet_logging_start == TRUE) && (convoy_head == id))
				{
				  convoy_end_time = current_time;
				  convoy_end_length = convoy_start_length + (convoy_end_time - convoy_start_time)*vehicle->speed;

				  /* log the the convoy length of vehicles */
				  log_trace_file(fp_trace_file_of_vehicle_convoy_length, TRACE_VEHICLE_CONVOY_LENGTH, current_time, convoy_start_time, convoy_start_length, convoy_end_time, convoy_end_length, vehicle->speed, convoy_vehicle_number);

				  /** find the vehicle with the greatest offset for the next convoy head */
				  /* sort the vehicle movement list of the directional edge <1,2> */

				  /* obtain the pointer to the vehicle at the rear of the vehicle movement list */
		
				  /* change the convoy_head with the following vehicle id */
				  convoy_head = id+1;

				  
/* 				  convoy_ptr_to_the_following_vehicle = Find_Following_Vehicle_Within_Communication_Range_On_Directional_Edge(param, vehicle); //set the convoy head vehicle to vehicle */

/* 				  if(convoy_ptr_to_the_following_vehicle == NULL) */
/* 				  { */
/* 				    printf("main(): convoy_ptr_to_the_following_vehicle == NULL with convoy_vehicle_number=%d\n", convoy_vehicle_number); */
/* 				    exit(1); */
/* 				  } */

/* 				  convoy_head = convoy_ptr_to_the_following_vehicle->id; */

				  /* adjust the convoy_start_time with the following vehicle's arrival time */
				  vehicle = vehicle_search(convoy_head);
				  if(vehicle == NULL)
				  {
				    printf("case VEHICLE_ESCAPE: there is no vehicle node corresponding to convoy_head(id=%d)\n", convoy_head);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                    fgetc(stdin);
#endif
				    exit(1);
				  }

				  convoy_vehicle_number--; //decrease the convoy-vehicle-number
				  convoy_start_time = current_time; //set the convoy-start-time to the current time
				  convoy_start_length = (current_time - vehicle->arrival_time)*vehicle->speed;
				  //vehicle's current position becomes the new convoy's length
				}
#endif				

				/* free the memory occupied by vehicle */
				vehicle_delete(id);

				break;

/** Stationary Vehicle States */
			case VEHICLE_STATIONARY_VEHICLE_START:			       
				current_time = smpl_time();

				vehicle = vehicle_search(id);
				if(vehicle == NULL)
				{
					printf("case VEHICLE_STATIONARY_VEHICLE_START: there is no vehicle node corresponding to id %d\n", id);
#ifdef __DEBUG_INTERACTIVE_MODE__
                    fgetc(stdin);
#endif
					exit(1);
				}

				/* update vehicle's state and state_time */
				vehicle->state = VEHICLE_STATIONARY_VEHICLE_START;
				vehicle->state_time = current_time;

				/** set up the vehicle's trajectory */
				set_vehicle_trajectory(vehicle, current_time, trajectory, trajectory_size, Gr, &Er);

				/** check whether data forwarding mode is download or upload */
				if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD ||
						param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
				{
				  /* obtain the pointer to the first access point */
				  pAP =  GetFirstAccessPoint(&APQ);
				  if(pAP == NULL)
				  {
				    printf("After GetFirstAccessPoint(), pAP is NULL!\n");
				    exit(1);
				  }

				  VADD_Update_Vehicle_EDD_And_EDD_SD(current_time, param, vehicle, pAP->Ga, pAP->Ga_size, &(pAP->ap_table_for_target_point)); //compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge along with real graph Gr and AP table				      				  
				}
				else
				{
				  VADD_Update_Vehicle_EDD_And_EDD_SD(current_time, param, vehicle, Gr, Gr_size, &ap_table_for_Gr);
				  //compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge along with real graph Gr and AP table
				}

				/** schedule packet generation */
				delay = 0;
				schedule(PACKET_ARRIVE, delay, id);

				/** schedule packet transmission */
				delay = param->vehicle_step_time;
				schedule(VEHICLE_STATIONARY_VEHICLE_SEND, delay, id);

			        break;

			case VEHICLE_STATIONARY_VEHICLE_SEND:
				current_time = smpl_time();

		                /**@for debugging */
				//if((current_time > 4244.7) || (current_time > 4248))
				//  printf("debugging point: time=%f\n", (float)current_time);
				/******************/

				vehicle = vehicle_search(id);
				if(vehicle == NULL)
				{
					printf("case VEHICLE_STATIONARY_VEHICLE_SEND: there is no vehicle node corresponding to id %d\n", id);
#ifdef __DEBUG_INTERACTIVE_MODE__
                    fgetc(stdin);
#endif
					exit(1);
				}

				/* update vehicle's state and state_time */
				vehicle->state = VEHICLE_STATIONARY_VEHICLE_SEND;
				vehicle->state_time = current_time;

				/** check whether there are packets to send and if there are packets to send, try to send them to a neighboring vehicle */
				if(vehicle->packet_queue->size > 0)
				{
					/**@for debugging */
					//if(vehicle->packet_queue->head.next->seq == 11)
					//  printf("main(): VEHICLE_STATIONARY_VEHICLE_SEND: packet(seq=%d) is traced\n", vehicle->packet_queue->head.next->seq);
					/******************/

					/* try to send the vehicle's packets to the neighboring vehicle */
				    flag = VADD_Is_There_Next_Carrier_On_Road_Segment(param, current_time, vehicle, Gr, Gr_size, &next_carrier);
                    //@Note: next_carrier is the convoy tail for the source vehicle of vid=1; on the other hand, for the other vehicles, next_carrier is the convoy leader.
                    if(flag) //if-1
                    {
						/* taehwan 20140730 */
					    printf("Hi %d~!!!!\n",next_carrier->id);
								    	
						VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, next_carrier, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier

					} //end of if-1.1
				}
                                
				/** schedule packet transmission */
				delay = param->vehicle_step_time;
                                schedule(VEHICLE_STATIONARY_VEHICLE_SEND, delay, id);
                                
			        break;

/** Convoy States */
			case CONVOY_UPDATE:
				current_time = smpl_time();

				vehicle = vehicle_search(id);
				if(vehicle == NULL)
				{
					printf("case CONVOY_UPDATE: there is no vehicle node corresponding to id %d\n", id);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                        fgetc(stdin);
#endif
					exit(1);
				}

				/* update vehicle's state and state_time */
				vehicle->state = CONVOY_UPDATE;
				vehicle->state_time = current_time;

				/* obtain the pointer to the convoy queue node pointed by this vehicle */
				convoy = vehicle->ptr_convoy_queue_node;

				/* forward the tail vehicle's packets to the leader vehicle considering the distance between tail vehicle and leader vehicle for communication delay.
				 @NOTE: we need to consider the communication delay between vehicle and convoy->head_vehicle later. */
				//VADD_Transfer_Packets(param, convoy, vehicle, convoy->head_vehicle);
				VADD_Forward_Packet_To_Next_Carrier(param, current_time, vehicle, convoy->head_vehicle, &packet_delivery_statistics, &discard_count);
				break;


/** states for target vehicle */
                        case VEHICLE_TARGET_VEHICLE_START:
				current_time = smpl_time();

				vehicle = vehicle_search(id);
				if(vehicle == NULL)
				{
					printf("case VEHICLE_TARGET_VEHICLE_START: there is no vehicle node corresponding to id %d\n", id);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                        fgetc(stdin);
#endif
					exit(1);
				}

				/* update vehicle's state and state_time */
				vehicle->state = VEHICLE_TARGET_VEHICLE_START;
				vehicle->state_time = current_time;

				/* set up the vehicle's trajectory */
				set_vehicle_trajectory(vehicle, current_time, trajectory, trajectory_size, Gr, &Er);

				/* adjust the vehicle's current position with trajectory */
				set_vehicle_current_position(param, vehicle, current_time, trajectory, trajectory_size, Gr, &Er);

				/* go to the state of VEHICLE_CHECK as the normal vehicles */
				delay = 0;
                                schedule(VEHICLE_CHECK, delay, id);
	
                                break;

			case VEHICLE_TARGET_VEHICLE_RESTART:
				current_time = smpl_time();

				vehicle = vehicle_search(id);
				if(vehicle == NULL)
				{
					printf("case VEHICLE_TARGET_VEHICLE_RESTART: there is no vehicle node corresponding to id %d\n", id);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                        fgetc(stdin);
#endif
					exit(1);
				}

				/* update vehicle's state and state_time */
				vehicle->state = VEHICLE_TARGET_VEHICLE_RESTART;
				vehicle->state_time = current_time;

				/* increase path_traverse_count that is the count for how many times vehicle passed its vehicle trajectory */
				vehicle->path_traverse_count++;

				/* set vehicle's current trajectory with pQueueNode's mobility_list as a new vehicle trajectory along with mobility type of either MOBILITY_OPEN or MOBILITY_CLOSED */
				pDestinationVehicleQueueNode = GetDestinationVehicleByVID(&DVQ, vehicle->id);
				if(pDestinationVehicleQueueNode == NULL)
				{
				  printf("%s:%d pDestinationVehicleQueueNode is NULL!\n",
						  __FUNCTION__, __LINE__);
				  exit(1);
				}

				update_vehicle_trajectory_with_mobility_list(current_time, param, vehicle, pDestinationVehicleQueueNode, Gr, Gr_size); //update vehicle trajectory according to vehicle's mobility type and path direction

				//set_vehicle_current_position(param, vehicle, current_time, trajectory, trajectory_size, Gr, &Er);

				/** register the vehicle's movement into the directional edge queue's vehicle_movement_list */
				if(param->vehicle_vanet_acl_measurement_flag)
				  register_vehicle_movement(param, vehicle, current_time, Gr);

                                if(flag_packet_log)
				{
				  if(param->vehicle_vanet_acl_measurement_flag != TRUE)
				    register_vehicle_movement(param, vehicle, current_time, Gr);

				  /* check whether data forwarding mode is download or upload */
				  if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD ||
						  param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
				  {
				    /* obtain the pointer to the first access point */
				    pAP =  GetFirstAccessPoint(&APQ);
				    if(pAP == NULL)
				    {
				      printf("After GetFirstAccessPoint(), pAP is NULL!\n");
				      exit(1);
				    }

				    VADD_Update_Vehicle_EDD_And_EDD_SD(current_time, param, vehicle, pAP->Ga, pAP->Ga_size, &(pAP->ap_table_for_target_point)); //compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge along with real graph Gr and AP table				      				  
				  }
				  else
				  {
				    VADD_Update_Vehicle_EDD_And_EDD_SD(current_time, param, vehicle, Gr, Gr_size, &ap_table_for_Gr);
				    //compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge along with real graph Gr and AP table
				  }

				  if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY)
				  {
				    convoy_join(param, vehicle, current_time, &packet_delivery_statistics); //join vehicle into the convoy preceding the vehicle on the directional edge if the vehicle is within the communication range of the convoy
				    //delay = 0.0;
				    //schedule(CONVOY_UPDATE, delay, id);
				  }
				}
				
				
                                /** determine think time (i.e., waiting time) at an intersection according to the think time distribution */
			        //delay = 0.0;
                                //delay =  delay_func(param, DELAY_VEHICLE_THINK_TIME);
                                if(param->vehicle_think_time > 0)
                                    delay =  delay_func(param, DELAY_VEHICLE_THINK_TIME);
                                else
                                    delay = 0; //let vehicle wait for a random time before going to the next state


                                /** go to the state of VEHICLE_CHECK as the normal vehicles */
                                schedule(VEHICLE_CHECK, delay, id);
			
                                break;

                        case VEHICLE_TARGET_VEHICLE_RECEIVE:

				break;


/** AP States */
		        case AP_START:
				current_time = smpl_time();
		
				/* obtain the pointer to the access point corresponding to id */
				pAP =  GetAccessPointByID(&APQ, id);
				if(pAP == NULL)
				{
					printf("case AP_START: there is no access point corresponding to id %d\n", id);
#ifdef __DEBUG_INTERACTIVE_MODE__
                    fgetc(stdin);
#endif
					exit(1);
				}

                                /* set state to AP_START */
                                pAP->state = AP_START;
				pAP->state_time = current_time;

				/* copy the vehicular traffic statistics of the edges in Gr into that in Ga */
				CopyVehicularTrafficStatistics(Gr, Gr_size, pAP->Ga, pAP->Ga_size);

				/* pick a target point on the target vehicle's trajectory */
				//PickTargetPoint(trajectory, trajectory_size, param->target_point_interdistance, param->target_point_index, Gr, &target_point);
				
				/* make an augmented graph Ga for data forwarding with the road network graph Gr and the target point */
				//AugmentGraph_With_TargetPoint(pAP, &target_point, Gr, Gr_size);

				/*@For debugging */
				//VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model(param, Gr, Gr_size, &DEr, &ap_table_for_Gr, ap_table_index_for_Gr); //compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) based on the stochastic model with graph Gr and directional edge queue DEr.
				///////////////////

				/* compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) with graph Ga including the target point as virual AP (i.e., packet destination). */
				//VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model(param, pAP->Ga, pAP->Ga_size, &(pAP->DEa), &(pAP->ap_table_for_target_point), ap_table_index_for_target_point);

				/* schedule an event of packet arrival */
				delay = 0;
				schedule(AP_PACKET_ARRIVE, delay, id);

				/* schedule an event of packet transmission */
				//delay = param->vehicle_step_time;
				//schedule(AP_PACKET_SEND, delay, id);

				/* schedule an event of AP update */
				delay = param->vehicle_edd_update_period;
				schedule(AP_UPDATE, delay, id);

				break;

			case AP_UPDATE:
				current_time = smpl_time();
		
				/* obtain the pointer to the access point corresponding to id */
				pAP =  GetAccessPointByID(&APQ, id);
				if(pAP == NULL)
				{
					printf("case AP_UPDATE: there is no access point corresponding to id %d\n", id);
#ifdef __DEBUG_INTERACTIVE_MODE__
                    fgetc(stdin);
#endif
					exit(1);
				}

				/* set state to AP_UPDATE */
				pAP->state = AP_UPDATE;
				pAP->state_time = current_time;

				/* copy the vehicular traffic statistics of the edges in Gr into that in Ga */
				CopyVehicularTrafficStatistics(Gr, Gr_size, pAP->Ga, pAP->Ga_size);

				/* schedule an event of AP update */
				delay = param->vehicle_edd_update_period;
				schedule(AP_UPDATE, delay, id);

				break;

			case AP_PACKET_ARRIVE:
				current_time = smpl_time();

				/* obtain the pointer to the access point corresponding to id */
				pAP =  GetAccessPointByID(&APQ, id);
				if(pAP == NULL)
				{
					printf("case AP_PACKET_ARRIVE: there is no access point corresponding to id %d\n", id);
#ifdef __DEBUG_INTERACTIVE_MODE__
                    fgetc(stdin);
#endif
					exit(1);
				}

				/* set state to AP_PACKET_ARRIVE */
				pAP->state = AP_PACKET_ARRIVE;
		        pAP->state_time = current_time;

				/** control the number of generated packets */
				if((param->communication_packet_maximum_number != INF) && (number_of_generated_packets == param->communication_packet_maximum_number))
				{
				  //printf("at time=%f, the packet generation from AP=%d stops!\n", (float)current_time, id);
				  break;
				}
				
				/* update the packet delivery statistics */
		        number_of_generated_packets++;
				packet_delivery_statistics.generated_packet_number++;
				packet_delivery_statistics.generated_packet_copy_number++;

				/* update the packet generation information at AP */
				pAP->packet_generation_time = current_time;

				/** initialize a global packet and insert it into the global packet queue */
				memset(&global_packet, '\0', sizeof(global_packet));
				global_packet.id = global_packet_id++; //set a globally unique packet id to packet's id
				global_packet.state = AP_PACKET_ARRIVE;
				global_packet.state_time = current_time;
				/* enqueue global_packet into global packet queue */
				pGlobalPacket = (global_packet_queue_node_t*) Enqueue((queue_t*)&GPQ, (queue_node_t*)&global_packet);
				/* Note that this Enqueue() does not allocate the memory of multicast forwarding table whose entry number will be Gr_size. The allocation will be done in Create_Multicast_Forwarding_Table_For_GlobalPacket() */

#if GLOBAL_PACKET_TRACE_FLAG
				if(global_packet.id == 14)
				{
					printf("[%.2f] run():\n global_packet.id=%d is traced\n", (float)current_time, global_packet.id);
					pTrace_GlobalPacket = pGlobalPacket;
				}
#endif

				/** initialize a packet and insert it into the vehicle's packet queue */
				memset(&packet, '\0', sizeof(packet));
				
				packet.id = global_packet_id; //set a globally unique packet id to packet's id
				 
				printf("global_packet_id %d\n",global_packet_id);

				//packet.id = new_simulation_node_id++; //set a globally unique id to packet's id
				packet.state = AP_PACKET_ARRIVE;
				packet.state_time = current_time;
				
				packet.data_forwarding_mode = DATA_FORWARDING_MODE_DOWNLOAD;
				packet.seq = ++(pAP->seq); //increase the sequence number for a new packet generated by this vehicle
				packet.size = param->communication_data_packet_size;

				packet.src_node_type = VANET_NODE_AP;
				packet.src_id = pAP->id;

				/*@for debugging */
				//if(packet.id == 240)
				//    printf("run(): for time=%.2f, packet.id=%d and packet.seq=%d\n", packet.id, packet.seq);
				/*****************/

				/* get the pointer to the first destination vehicle */
				pDestinationVehicleQueueNode = GetFirstDestinationVehicle(&DVQ);
				if(pDestinationVehicleQueueNode == NULL)
				{ 
					printf("%s:%d pDestinationVehicleQueueNode is NULL!\n",
							__FUNCTION__, __LINE__);
					exit(1);
				}				
				packet.dst_node_type = VANET_NODE_VEHICLE;
				packet.dst_id = pDestinationVehicleQueueNode->vid; //packet destination vehicle       
				packet.dst_vnode = pDestinationVehicleQueueNode->vnode; //pointer to destination vehicle

				packet.carry_src_id = pAP->id;
				packet.carry_dst_id = 0;


				/* the maximum number of vehicles in the target road network; this is used to construct a predicted encounter graph for TPD data forwarding */
				packet.vehicle_maximum_number = param->vehicle_maximum_number;

				/* enqueue packet into packet queue */
				pPacket = (packet_queue_node_t*) Enqueue((queue_t*)pAP->packet_queue, (queue_node_t*)&packet);
				
				/** set up packet's target point or vehicle trajectory with vehicle's trajectory according to target point selection type */
				if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
				//@START OF DATA_FORWARDING_MODE_DOWNLOAD
				/*****************************************/
				{
					if((param->vehicle_vanet_target_point_selection_type == VANET_TARGET_POINT_SELECTION_ADAPTIVE_TARGET_POINT) || (param->vehicle_vanet_target_point_selection_type == VANET_TARGET_POINT_SELECTION_DYNAMIC_TARGET_POINT) || (param->vehicle_vanet_target_point_selection_type == VANET_TARGET_POINT_SELECTION_PROGRESS_TARGET_POINT) || (param->vehicle_vanet_target_point_selection_type == VANET_TARGET_POINT_SELECTION_REVERSE_PATH_TARGET_POINT) || (param->vehicle_vanet_target_point_selection_type == VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT) || (param->vehicle_vanet_target_point_selection_type == VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT && (param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_PARTIALLY_DYNAMIC_FORWARDING || param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_FULLY_DYNAMIC_FORWARDING || param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_END_INTERSECTION || param->vehicle_vanet_target_point_computation_method == VANET_TARGET_POINT_COMPUTATION_METHOD_RANDOM_INTERSECTION)))
					{
						Install_VehicleTrajectory_Into_Packet(param, current_time, Gr, Gr_size, pDestinationVehicleQueueNode->vnode, pPacket);

						/* update the packet TTL according to TTL override flag */
						if(param->communication_packet_ttl_override_flag)
						{
							//param->communication_packet_ttl =  Compute_TravelTime_For_VehicleTrajectory(param, current_time, &(pPacket->vehicle_trajectory)); //compute the travel time for the path distance for the trajectory from the start position to the end position in the vehicle trajectory. 
							printf("Compute_PathDistance_From_CurrentPosition_To_EndPosition_With_VehicleTrajectory_Along_With_PathTravelTime_And_Deviation\n");
							travel_distance = Compute_PathDistance_From_CurrentPosition_To_EndPosition_With_VehicleTrajectory_Along_With_PathTravelTime_And_Deviation(param, current_time, pPacket, &travel_time, &travel_time_deviation);
							//compute the vehicle's path distance, the path travel time and the deviation for the trajectory from the current position to the end position on the vehicle trajectory where destination_vehicle_travel_distance is the distance from the trajectory's start position to the destination vehicle's end position on its vehicle trajectory

							/* set the packet's TTL to the sum of travel time and travel time deviation */
							//param->communication_packet_ttl = travel_time + travel_time_deviation;
							param->communication_packet_ttl = travel_time + 3*travel_time_deviation; //this TTL covers 99% of the vehicle delay
						}
					}

					/* obtain the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) with graph G for the target point (i.e., packet destination). */

					/** determine a target point according to the measurement flag the packet delay */
					if(param->packet_delay_measurement_flag == TRUE)
					{
						/* set the AP's target point to the given target point */
						pAP->target_point_id = param->packet_delay_measurement_target_point;
					}
					else
					{
						/* set the AP's target point(s) to optimal target point(s) considering the destination vehicle's trajectory */
						if(param->data_forwarding_multiple_target_point_flag)
						{ //In the case where multiple target points are used for data delivery
							/* @Note: For the multiple-target-point transmission, we consider one AP for simple implementation even though there are multiple APs available in the road network. */
							/* transmit multiple copies of a packet to the destination vehicle */
							target_point_number = GetMultipleTargetPoints_For_AP(param, current_time, pAP->vertex, pDestinationVehicleQueueNode->vnode, pPacket, &FTQ, &TPQ);

							/* set up the actual_transmitter_AP_id and actual_transmitter_AP_vertex */
							actual_transmitter_AP_id = pAP->id;
							strcpy(actual_transmitter_AP_vertex, pAP->vertex);
							actual_transmitter_AP_qnode = pAP;
						}
						else if((param->communication_multiple_AP_flag == TRUE) && (param->communication_AP_maximum_number > 1))
						{ //In the case where multiple APs exist, let the AP with the shortest delivery delay transmit this packet towards the destination vehicle
							/*
								1. TSF: select the AP with the minimum EVD (Expected Vehicle Delay).
								2. RTP (Random Trajectory Point): select the AP whose location is geographically closest to the destination vehicle's current location
								3. LTP (Last Trajectory Point): select the AP whose location is geographically closest to the last trajectory point of the destination vehicle
							*/
							target_point_id = GetTargetPoint_For_Multiple_APs(param, current_time, &APQ, pDestinationVehicleQueueNode->vnode, pPacket, &FTQ, &EDD_p, &EAD_p, &actual_transmitter_AP_qnode);

							/* set up the actual_transmitter_AP_id and actual_transmitter_AP_vertex */
							actual_transmitter_AP_id = actual_transmitter_AP_qnode->id;
							strcpy(actual_transmitter_AP_vertex, actual_transmitter_AP_qnode->vertex);
						}
						else
						{ //In the case where a single AP exists
							target_point_id = GetTargetPoint_For_AP(param, current_time, pAP->vertex, pDestinationVehicleQueueNode->vnode, pPacket, &FTQ, &EDD_p, &EAD_p);

							/* set up the actual_transmitter_AP_id and actual_transmitter_AP_vertex */
							actual_transmitter_AP_id = pAP->id;
							strcpy(actual_transmitter_AP_vertex, pAP->vertex);
							actual_transmitter_AP_qnode = pAP;
						}
                                    
						/**@for debugging */
						//if(pAP->target_point_id == 10)
						//    printf("run(): for time=%.2f, packet.id = %d\n", (float)current_time, packet.id);
						/******************/                                 
					}
                
					/** transmit packet(s) according to the policy by the number of target points */               
					if(param->data_forwarding_multiple_target_point_flag) //@start of multiple packet transmission
					{ /** In the case of multiple packet transmission towards multiple target points */
#ifdef __DEBUG_LEVEL_PRINT_TARGET_POINT__
						/* print out target points */
						pTargetPoint = &(TPQ.head);
					 	printf("pid=%d target points: ", packet.id);
					  	for(i = 0; i < TPQ.size; i++)
						{
							pTargetPoint = pTargetPoint->next;
							printf("%d ", pTargetPoint->target_point_id);
						}
						printf("\n");
#endif

						/* transmit one copy of the packet towards each target point */
						pTargetPoint = &(TPQ.head);
						for(i = 0; i < TPQ.size; i++) //@start of for-loop
						{
							pTargetPoint = pTargetPoint->next;

							pAP->target_point_id = pTargetPoint->target_point_id;
							pPacket->target_point_id = pTargetPoint->target_point_id;

							/* get the EDD and the EDD_SD for the target point from the AP */
							VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(param, pPacket->target_point_id, actual_transmitter_AP_vertex, &FTQ,  &(pAP->EDD_for_download), &(pAP->EDD_SD_for_download));
				
							pPacket->expected_delivery_delay = pAP->EDD_for_download;
							pPacket->expected_delivery_delay_standard_deviation = pAP->EDD_SD_for_download;				

							pPacket->actual_delivery_delay = 0;
							pPacket->actual_delivery_delay_standard_deviation = 0; //later, set the carry vehicle's expected delivery delay standard deviation (EDD_SD) to packet's expected delivery delay standard deviation
							pPacket->delivery_delay_difference = 0;
	
							pPacket->ttl = param->communication_packet_ttl;
							pPacket->generation_time = current_time;
							pPacket->last_receive_time = current_time;

							/* set the recomputation interval and the recomputation time for a new target point */
							pPacket->target_point_recomputation_interval = pPacket->expected_delivery_delay/param->vehicle_vanet_target_point_recomputation_interval_denominator;
							pPacket->target_point_recomputation_time = pPacket->generation_time + pPacket->target_point_recomputation_interval;

#ifdef __DEBUG_LEVEL_AP_PACKET_ARRIVE__
							printf("AP_PACKET_ARRIVE: seq=%d, target_point=%d for destination vehicle on (%s,%s:%.2f); EDD_p=%.2f, EAD_p=%.2f, |EDD_p-EAD_p|=%.2f\n", pPacket->seq, pPacket->target_point_id, pDestinationVehicleQueueNode->vnode->current_pos_in_digraph.enode->tail_node, pDestinationVehicleQueueNode->vnode->current_pos_in_digraph.enode->head_node, pDestinationVehicleQueueNode->vnode->current_pos_in_digraph.offset, EDD_p, EAD_p, fabs(EDD_p - EAD_p));
				
#endif

							/** set up packet trajectory from the access point to the intersection corresponding to the target point.
Note that for the static forwarding, the path from the AP to the target point intersection is inserted into the packet; on the other hand, for the dynamic forwarding, the next hop from the AP among the path from the AP to the target point intersection is inserted into the packet */
							if(param->vehicle_vanet_target_point_selection_type == VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT)
							{
								strcpy(packet_src_vertex, actual_transmitter_AP_vertex); //packet source vertex
								itoa(pPacket->target_point_id, packet_dst_vertex); //packet destination vertex

#ifdef __DEBUG_PACKET_DELAY_DISTRIBUTION__
								Install_StaticPacketTrajectory_Into_Packet(param, current_time, Gr, Gr_size, static_packet_trajectory, static_packet_trajectory_size, pPacket);
#else
								Install_PacketTrajectory_Into_Packet(param, current_time, Gr, Gr_size, packet_src_vertex, packet_dst_vertex, pPacket);
#endif
							}

							/* expected packet transmission number */
							src = atoi(actual_transmitter_AP_vertex);
							dst = pPacket->target_point_id;

							if((src < 0) || (src > Gr_size))
							{
								printf("%s:%d run(): src(%d) is invalid\n", __FILE__, __LINE__, src);
								exit(1);
							} 
							else if((dst < 0) || (dst > Gr_size))
							{
								printf("%s:%d run(): dst(%d) is invalid\n", __FILE__, __LINE__, dst);
								exit(1);
							}

							pPacket->expected_packet_transmission_number = param->vanet_table.Dr_edc[src-1][dst-1] + (pPacket->packet_trajectory.size-1)*2;
							/* Note: Count the number of transmissions by stationary nodes; note that the first stationary node will only transmit the packet and the last stationary node will only receive the packet. On the other hand, the intermediate stationary nodes will receive the packet as well as transmit the packet. */
							/***************************************/

							/** handover the packet from the current AP to the actual transmitter AP */
							if(pAP->id != actual_transmitter_AP_id)
							{
								/* dequeue the packet pointed by pPacket from pAP's packet queue */
								Dequeue_With_QueueNodePointer((queue_t*)pAP->packet_queue, (queue_node_t*)pPacket);

								/* enqueue the packet pointed by pPacket from pAP's packet queue */
								Enqueue_With_QueueNodePointer((queue_t*)actual_transmitter_AP_qnode->packet_queue, (queue_node_t*)pPacket);

								/* update the packet's field according to the actual transmitter AP */
								pPacket->src_id = actual_transmitter_AP_qnode->id;
					   			pPacket->carry_src_id = actual_transmitter_AP_qnode->id;                    
							}
									
#ifdef  __LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__
							/** enqueue the packet carrier trace entry into packet's carrier trace queue */
							Enqueue_CarrierTraceEntry(param, current_time, pPacket, VANET_NODE_AP, (void*)actual_transmitter_AP_qnode);
#endif

							/** enqueue packet into global packet queue GPQ along with the schedule of target point recomputation */
							pGlobalPacket = Enqueue_Packet_Into_GlobalPacketQueue_With_TargetPoint_Recomputation_Schedule(param, current_time, &GPQ, pPacket);

							/** let packet's global_packet point to pGlobalPacket */
							pPacket->global_packet = pGlobalPacket;

							/* check whether there is another target point to send a copy of the packet towards; if so, we make a copy of the packet by enqueuing pPacket into pAP's packet queue */
							if(i+1 < TPQ.size)
							{
								/* make a new copy of the packet and enqueue it into pAP's packet queue */
								pPacket = (packet_queue_node_t*) Enqueue((queue_t*)pAP->packet_queue, (queue_node_t*)&packet);
							
								/* install vehicle trajectory into the new packet */
								Install_VehicleTrajectory_Into_Packet(param, current_time, Gr, Gr_size, pDestinationVehicleQueueNode->vnode, pPacket);
							}
						} //@end of for-loop

						/** forward the packet to the stationary node at the intersection of AP for the packet-trajectory-based forwarding; @Note that this forwarding must be performed after enqueueing the packet pointer into the global packet queue. */
						if(param->vehicle_vanet_target_point_selection_type == VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT)
						{
							/* forward the packet from AP to the stationary node */
							VADD_Forward_Packet_From_AP_To_Stationary_Node(param, current_time, actual_transmitter_AP_qnode, atoi(actual_transmitter_AP_qnode->vertex), &packet_delivery_statistics);
						}
					} //@end of multiple packet transmission
					else //@start of single packet transmission
					{ /** In the case of a single packet transmission towards a single target point */

#ifdef __DEBUG_LEVEL_PRINT_TARGET_POINT__
						/* print out target points */
					 	printf("pid=%d target points: %d\n", packet.id, target_point_id);
#endif

						/* determine AP's target point (i.e., intersection id) towards which this packet is sent */
						pAP->target_point_id = target_point_id;
						pPacket->target_point_id = target_point_id;

						/* get the EDD and the EDD_SD for the target point from the AP */
						VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(param, pPacket->target_point_id, actual_transmitter_AP_vertex, &FTQ,  &(pAP->EDD_for_download), &(pAP->EDD_SD_for_download));
				
						pPacket->expected_delivery_delay = pAP->EDD_for_download;
						pPacket->expected_delivery_delay_standard_deviation = pAP->EDD_SD_for_download;				

						pPacket->actual_delivery_delay = 0;
						pPacket->actual_delivery_delay_standard_deviation = 0; //later, set the carry vehicle's expected delivery delay standard deviation (EDD_SD) to packet's expected delivery delay standard deviation
						pPacket->delivery_delay_difference = 0;

						pPacket->ttl = param->communication_packet_ttl;
						pPacket->generation_time = current_time;
						pPacket->last_receive_time = current_time;

						/* set the recomputation interval and the recomputation time for a new target point */
						pPacket->target_point_recomputation_interval = pPacket->expected_delivery_delay/param->vehicle_vanet_target_point_recomputation_interval_denominator;
						pPacket->target_point_recomputation_time = pPacket->generation_time + pPacket->target_point_recomputation_interval;

#ifdef __DEBUG_LEVEL_AP_PACKET_ARRIVE__
						printf("AP_PACKET_ARRIVE: seq=%d, target_point=%d for destination vehicle on (%s,%s:%.2f); EDD_p=%.2f, EAD_p=%.2f, |EDD_p-EAD_p|=%.2f\n", pPacket->seq, pPacket->target_point_id, pDestinationVehicleQueueNode->vnode->current_pos_in_digraph.enode->tail_node, pDestinationVehicleQueueNode->vnode->current_pos_in_digraph.enode->head_node, pDestinationVehicleQueueNode->vnode->current_pos_in_digraph.offset, EDD_p, EAD_p, fabs(EDD_p - EAD_p));
				
#endif

						/** set up packet trajectory from the access point to the intersection corresponding to the target point.
Note that for the static forwarding, the path from the AP to the target point intersection is inserted into the packet; on the other hand, for the dynamic forwarding, the next hop from the AP among the path from the AP to the target point intersection is inserted into the packet */
						if(param->vehicle_vanet_target_point_selection_type == VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT)
						{
							strcpy(packet_src_vertex, actual_transmitter_AP_vertex); //packet source vertex
							itoa(pPacket->target_point_id, packet_dst_vertex); //packet destination vertex
                                  

#ifdef __DEBUG_PACKET_DELAY_DISTRIBUTION__
							Install_StaticPacketTrajectory_Into_Packet(param, current_time, Gr, Gr_size, static_packet_trajectory, static_packet_trajectory_size, pPacket);
#else
							Install_PacketTrajectory_Into_Packet(param, current_time, Gr, Gr_size, packet_src_vertex, packet_dst_vertex, pPacket);
#endif
						}

						/* expected packet transmission number */
						src = atoi(actual_transmitter_AP_vertex);
						dst = pPacket->target_point_id;

						if((src < 0) || (src > Gr_size))
						{
							printf("%s:%d run(): src(%d) is invalid\n", __FILE__, __LINE__, src);
							exit(1);
						} 
						else if((dst < 0) || (dst > Gr_size))
						{
							printf("%s:%d run(): dst(%d) is invalid\n", __FILE__, __LINE__, dst);
							exit(1);
						}

						pPacket->expected_packet_transmission_number = param->vanet_table.Dr_edc[src-1][dst-1] + (pPacket->packet_trajectory.size-1)*2;
						/* Note: Count the number of transmissions by stationary nodes; note that the first stationary node will only transmit the packet and the last stationary node will only receive the packet. On the other hand, the intermediate stationary nodes will receive the packet as well as transmit the packet. */
						/***************************************/

						/** handover the packet from the current AP to the actual transmitter AP */
						if(pAP->id != actual_transmitter_AP_id)
						{
							/* dequeue the packet pointed by pPacket from pAP's packet queue */
							Dequeue_With_QueueNodePointer((queue_t*)pAP->packet_queue, (queue_node_t*)pPacket);


							/* enqueue the packet pointed by pPacket from pAP's packet queue */
							Enqueue_With_QueueNodePointer((queue_t*)actual_transmitter_AP_qnode->packet_queue, (queue_node_t*)pPacket);

							/* update the packet's field according to the actual transmitter AP */
							pPacket->src_id = actual_transmitter_AP_qnode->id;
					   		pPacket->carry_src_id = actual_transmitter_AP_qnode->id;                    
						}
								
#ifdef  __LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__
						/** enqueue the packet carrier trace entry into packet's carrier trace queue */
						Enqueue_CarrierTraceEntry(param, current_time, pPacket, VANET_NODE_AP, (void*)actual_transmitter_AP_qnode);
#endif

						/** enqueue packet into global packet queue GPQ along with the schedule of target point recomputation */
						pGlobalPacket = Enqueue_Packet_Into_GlobalPacketQueue_With_TargetPoint_Recomputation_Schedule(param, current_time, &GPQ, pPacket);

						/** let packet's global_packet point to pGlobalPacket */
						pPacket->global_packet = pGlobalPacket;

						/** forward the packet to the stationary node at the intersection of AP for the packet-trajectory-based forwarding; @Note that this forwarding must be performed after enqueueing the packet pointer into the global packet queue. */
						if(param->vehicle_vanet_target_point_selection_type == VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT)
						{
							/* forward the packet from AP to the stationary node */
							VADD_Forward_Packet_From_AP_To_Stationary_Node(param, current_time, actual_transmitter_AP_qnode, atoi(actual_transmitter_AP_qnode->vertex), &packet_delivery_statistics);
						}
					} //@end of single packet transmission
				} //@END OF DATA_FORWARDING_MODE_DOWNLOAD
				/*****************************************/
				else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
				//@START OF DATA_FORWARDING_MODE_V2V
				/*****************************************/
				{
					/* compute a target point for VADD, TBD or Epidemic Routing as vanet forwarding scheme */
					if(param->vanet_forwarding_scheme == VANET_FORWARDING_VADD || param->vanet_forwarding_scheme == VANET_FORWARDING_TBD || param->vanet_forwarding_scheme == VANET_FORWARDING_EPIDEMIC 
					|| param->vanet_forwarding_scheme == VANET_FORWARDING_TADB)
					{
						target_point_id = GetTargetPoint_For_AP_For_V2V_Data_Delivery(param, current_time, pAP->vertex, pDestinationVehicleQueueNode->vnode, &FTQ, Gr_set_number, Gr_set, Gr_set_size, &EDD_p, &EAD_p);

						/* set packet's target point and AP's target point to target_point_id */
						pAP->target_point_id = target_point_id;
						pPacket->target_point_id = target_point_id;
					}

					/* set up packet fields, such as ttl and generation_time */
					pPacket->ttl = param->communication_packet_ttl;
					pPacket->generation_time = current_time;
					pPacket->last_receive_time = current_time;

					/** enqueue packet into global packet queue GPQ */
					pGlobalPacket = Enqueue_Packet_Into_GlobalPacketQueue(param, current_time, &GPQ, pPacket);

					/** let packet's global_packet point to pGlobalPacket */
					pPacket->global_packet = pGlobalPacket;
				}
				/*****************************************/

				/** schedule next packet for the vehicle with id */
				delay = delay_func(param, DELAY_PACKET_INTERARRIVAL_TIME);
				schedule(AP_PACKET_ARRIVE, delay, id);
				
				break;

			case AP_PACKET_SEND:
				current_time = smpl_time();

		                /**@for debugging */
				//if((current_time > 4244.7) || (current_time > 4248))
				//  printf("debugging point: time=%f\n", (float)current_time);
				/******************/

				/* obtain the pointer to the access point corresponding to id */
				pAP =  GetAccessPointByID(&APQ, id);
				if(pAP == NULL)
				{
				        printf("case AP_PACKET_SEND: there is no access point corresponding to id %d\n", id);
#ifdef __DEBUG_INTERACTIVE_MODE__
                    fgetc(stdin);
#endif
					exit(1);
				}

				/* update vehicle's state and state_time */
				pAP->state = AP_PACKET_SEND;
				pAP->state_time = current_time;

				/** check whether there are packets to send and if there are packets to send, try to send them to a neighboring vehicle */
				if(pAP->packet_queue->size > 0)
				{
				  /**@for debugging */
				  //if(pAP->packet_queue->head.next->seq == 11)
				  //  printf("main(): AP_PACKET_SEND: packet(seq=%d) is traced\n", pAP->packet_queue->head.next->seq);
				  /******************/

				  /** pick a target point on the target vehicle's trajectory
                                      The target point is the intersection where the destination vehicle is heading.
				  */
				  //PickTargetPoint(trajectory, trajectory_size, param->target_point_interdistance, param->target_point_index, Gr, &target_point);

/* 				  /\* get the pointer to the first destination vehicle *\/ */
/* 				  pDestinationVehicleQueueNode = GetFirstDestinationVehicle(&DVQ); */
/* 				  if(pDestinationVehicleQueueNode == NULL) */
/* 				  { */
/* 				    printf("pDestinationVehicleQueueNode is NULL!\n"); */
/* 				    exit(1); */
/* 				  } */

/* 				  /\* set the target point with the destination vehicle's head vertex *\/ */
/* 				  target_point_id = GetTargetPoint_With_VehicleMobility(current_time, pDestinationVehicleQueueNode->vnode); */
/* 				  itoa(target_point_id, target_point); */
/* 				  SetTargetPoint_In_TafficTable(&(pAP->ap_table_for_target_point), target_point); */
				
/* 				  /\** make an augmented graph Ga for data forwarding with the road network graph Gr and the target point *\/ */
/* 				  //AugmentGraph_With_TargetPoint(&AP, &target_point, Gr, Gr_size); */

/* 				  /\** compute the Expected Delivery Delay (EDD) and Expected Delivery Delay Standard Deviation (EDD_SD) with graph Ga including the target point as virual AP (i.e., packet destination). *\/ */
/* 				  ap_table_index_for_target_point = 0; //the first target point in the AP traffic table */
/* 				  VADD_Compute_EDD_And_EDD_SD_Based_On_Stochastic_Model(param, pAP->Ga, pAP->Ga_size, &(pAP->DEa), &(pAP->ap_table_for_target_point), ap_table_index_for_target_point); */

				  /** update all vehicles' EDDs and EDD_SDs */
                                  //update_all_vehicle_edd_and_edd_sd(current_time, param, pAP->Ga, pAP->Ga_size, &(pAP->ap_table_for_target_point)); 

				  /** send the packet(s) to an appropriate next carrier vehicle */
                                  /* try to send the AP's packets to the neighboring vehicle */
				  flag = VADD_Is_There_Next_Carrier_At_Intersection_For_AP(param, current_time, pAP, Gr, Gr_size, &FTQ, &next_carrier);
				  
                                  //@Note: next_carrier is the convoy tail for the source vehicle of vid=1; on the other hand, for the other vehicles, next_carrier is the convoy leader.
                                  if(flag) //if-1
                                  {
                                    VADD_Forward_Packet_From_AP_To_Next_Carrier(param, current_time, pAP, next_carrier, &packet_delivery_statistics, &discard_count); //vehicle forwards its packet(s) to the next carrier pointed by next_carrier
                                  } //end of if-1.1
				}
                                
				/** schedule packet transmission */
				delay = param->vehicle_step_time;
                                schedule(AP_PACKET_SEND, delay, id);
                                
			        break;

			default:
				printf("Unknown state(%d)!\n", event);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                fgetc(stdin);
#endif
				exit(1);
		} //end of switch
	} while(1); //end of do-while

	/** simulation summary **/
	/** compute packet delivery statistics */
        //@Version 1 for the performance evaluation
        /*
	packet_delivery_statistics.mean_expected_delivery_delay = packet_delivery_statistics.expected_delivery_delay_sum/packet_delivery_statistics.delivered_packet_number;
	packet_delivery_statistics.mean_actual_delivery_delay = packet_delivery_statistics.actual_delivery_delay_sum/packet_delivery_statistics.delivered_packet_number;
	packet_delivery_statistics.ratio_of_two_delivery_delays = packet_delivery_statistics.mean_expected_delivery_delay/packet_delivery_statistics.mean_actual_delivery_delay;
	packet_delivery_statistics.packet_delivery_ratio = (double)packet_delivery_statistics.delivered_packet_number/packet_delivery_statistics.generated_packet_number;

	packet_delivery_statistics.mean_expected_delivery_delay_standard_deviation = packet_delivery_statistics.expected_delivery_delay_standard_deviation_sum/packet_delivery_statistics.delivered_packet_number;
	packet_delivery_statistics.mean_delivery_delay_difference = packet_delivery_statistics.delivery_delay_difference_sum/packet_delivery_statistics.delivered_packet_number;
        */

        //@Version 2 for the performance evaluation        
	packet_delivery_statistics.mean_expected_delivery_delay = packet_delivery_statistics.expected_delivery_delay_sum/packet_delivery_statistics.delivered_packet_number;
/*	packet_delivery_statistics.mean_actual_delivery_delay = (packet_delivery_statistics.actual_delivery_delay_sum + packet_delivery_statistics.discarded_packet_number * param->communication_packet_ttl
)/(packet_delivery_statistics.delivered_packet_number + packet_delivery_statistics.discarded_packet_number);*/

	packet_delivery_statistics.mean_actual_delivery_delay = packet_delivery_statistics.actual_delivery_delay_sum /packet_delivery_statistics.delivered_packet_number;

        printf("packet_delivery_statistics.actual_delivery_delay_sum = %d\n",(int)packet_delivery_statistics.actual_delivery_delay_sum);
        printf("packet_delivery_statistics.discarded_packet_number = %d\n",packet_delivery_statistics.discarded_packet_number);
        printf("param->communication_packet_ttl = %d\n",(int)param->communication_packet_ttl);
        printf("ap_arrival_delay_sum = %d\n",(int)packet_delivery_statistics.ap_arrival_delay_sum);
        printf("destination_vehicle_arrival_delay_sum = %d\n",(int)packet_delivery_statistics.destination_vehicle_arrival_delay_sum);

	packet_delivery_statistics.ratio_of_two_delivery_delays = packet_delivery_statistics.mean_expected_delivery_delay/packet_delivery_statistics.mean_actual_delivery_delay;
	packet_delivery_statistics.packet_delivery_ratio = (double)packet_delivery_statistics.delivered_packet_number/packet_delivery_statistics.generated_packet_number;

	packet_delivery_statistics.mean_expected_delivery_delay_standard_deviation = packet_delivery_statistics.expected_delivery_delay_standard_deviation_sum/packet_delivery_statistics.delivered_packet_number;
	packet_delivery_statistics.mean_delivery_delay_difference = packet_delivery_statistics.delivery_delay_difference_sum/packet_delivery_statistics.delivered_packet_number;

	/* delivery cost stats */
	if((packet_delivery_statistics.delivered_packet_number + packet_delivery_statistics.discarded_packet_number) > 0)
	{
		packet_delivery_statistics.mean_expected_packet_transmission_number = packet_delivery_statistics.expected_packet_transmission_number_sum/(packet_delivery_statistics.delivered_packet_number + packet_delivery_statistics.discarded_packet_number); 
		packet_delivery_statistics.mean_actual_packet_transmission_number = packet_delivery_statistics.actual_packet_transmission_number_sum/(packet_delivery_statistics.delivered_packet_number + packet_delivery_statistics.discarded_packet_number); 
	}
	else
	{
		packet_delivery_statistics.mean_expected_packet_transmission_number = 0;
		packet_delivery_statistics.mean_actual_packet_transmission_number = 0;
	}
	/** store summary into output file 1 */
	fprintf(fp_1, "\n/*** simulation results ***/\n");

	fprintf(fp_1, "data_forwarding_type (option -F)=%s\n", get_data_forwarding_mode_name(param->data_forwarding_mode));
	fprintf(fp_1, "vanet_forwarding_scheme (option -f)=%s\n", get_vanet_forwarding_scheme_name(param->vanet_forwarding_scheme));

	fprintf(fp_1, "tpd_encounter_graph_source_routing_flag (option -S)=%d\n", param->tpd_encounter_graph_source_routing_flag);
	fprintf(fp_1, "tpd_encounter_graph_optimization_flag (option -O)=%d\n", param->tpd_encounter_graph_optimization_flag);
	fprintf(fp_1, "tpd_encounter_probability_threshold=%.2f\n", param->tpd_encounter_probability_threshold);
	fprintf(fp_1, "tpd_delivery_probability_threshold=%.2f\n", param->tpd_delivery_probability_threshold);

	fprintf(fp_1, "seed=%d\n", seed);
	fprintf(fp_1, "simulation_time=%.1f [sec]\n", param->simulation_time);
	fprintf(fp_1, "graph_node_number (option -k)=%d\n", param->graph_node_number);
    fprintf(fp_1, "graph_file_name=%s\n", param->graph_file_name);
    fprintf(fp_1, "data_forwarding_two_way_forwarding_flag (option -K)=%d\n", param->data_forwarding_two_way_forwarding_flag);
    fprintf(fp_1, "data_forwarding_multiple_target_point_flag (option -W)=%d\n", param->data_forwarding_multiple_target_point_flag);
    fprintf(fp_1, "data_forwarding_maximum_target_point_number=%d\n", param->data_forwarding_maximum_target_point_number);
    fprintf(fp_1, "forwarding_probability_and_statistics_flag=%d\n", (int)param->forwarding_probability_and_statistics_flag);
    fprintf(fp_1, "packet_delay_measurement_flag (option -X)=%d\n", (int)param->packet_delay_measurement_flag);
    fprintf(fp_1, "packet_delay_measurement_target_point (option -Y)=%d\n", param->packet_delay_measurement_target_point);
    fprintf(fp_1, "packet_delay_measurement_time (option -Z)=%.1f\n", param->packet_delay_measurement_time);
	fprintf(fp_1, "vehicle_vanet_target_point_selection_type=%d\n", param->vehicle_vanet_target_point_selection_type);
	fprintf(fp_1, "vehicle_vanet_target_point_computation_method (option -C)=%d\n", param->vehicle_vanet_target_point_computation_method);
	fprintf(fp_1, "vehicle_vanet_target_point_search_space_type (option -P)=%d\n", param->vehicle_vanet_target_point_search_space_type);
	fprintf(fp_1, "vehicle_vanet_target_point_recomputation_interval_denominator (option -I)=%.1f\n", param->vehicle_vanet_target_point_recomputation_interval_denominator);
	fprintf(fp_1, "vehicle_vanet_vehicle_trajectory_type (option -T)=%d\n", param->vehicle_vanet_vehicle_trajectory_type);
	fprintf(fp_1, "vehicle_vanet_vehicle_trajectory_length_type (option -D)=%d\n", param->vehicle_vanet_vehicle_trajectory_length_type);
	fprintf(fp_1, "vehicle_vanet_vehicular_traffic_model (option -m)=%d\n", param->vehicle_vanet_vehicular_traffic_model);
	fprintf(fp_1, "vehicle_vanet_acl_measurement_flag (option -h)=%d\n", param->vehicle_vanet_acl_measurement_flag);
	fprintf(fp_1, "vehicle_vanet_edd_and_link_model (option -l)=%d\n", param->vehicle_vanet_edd_and_link_model);
	fprintf(fp_1, "vehicle_vanet_edd_model (option -e)=%d\n", param->vehicle_vanet_edd_model);
	fprintf(fp_1, "vehicle_vanet_tbd_edd_computation_type (option -c)=%d\n", param->vehicle_vanet_tbd_edd_computation_type);
	fprintf(fp_1, "vehicle_vanet_edge_delay_model (option -d)=%d\n", param->vehicle_vanet_edge_delay_model);
	fprintf(fp_1, "vehicle_vanet_intersection_forwarding_type (option -f)=%d\n", param->vehicle_vanet_intersection_forwarding_type);
	fprintf(fp_1, "vehicle_maximum_number (option -n)=%d\n", param->vehicle_maximum_number);
	fprintf(fp_1, "vehicle_packet_generating_entity_number (option -g)=%d\n", param->vehicle_packet_generating_entity_number);
	fprintf(fp_1, "vehicle_vanet_stationary_vehicle_flag (option -o)=%d\n", param->vehicle_vanet_stationary_vehicle_flag);
	fprintf(fp_1, "vehicle_interarrival_time (option -z)=%.1f\n", (float)param->vehicle_interarrival_time);
	fprintf(fp_1, "vehicle_think_time (option -H)=%.4f\n", (float)param->vehicle_think_time);
	fprintf(fp_1, "vehicle_speed_distribution (option -q)=%d where EQUAL=1, UNIFORM=2, and NORMAL=3\n", param->vehicle_speed_distribution);
	fprintf(fp_1, "vehicle_path_length_distribution (option -V)=%d where EQUAL=1 and NORMAL=3\n", param->vehicle_path_length_distribution);
	fprintf(fp_1, "communication_packet_delivery_probability_threshold (option -E)=%.4f\n", (float)param->communication_packet_delivery_probability_threshold);
	fprintf(fp_1, "communication_packet_reverse_traversal_hop_distance_threshold (option -J)=%d\n", param->communication_packet_reverse_traversal_hop_distance_threshold);
	fprintf(fp_1, "communication_range (option -r)=%.1f\n", (float)param->communication_range);
	fprintf(fp_1, "communication_packet_interarrival_time (option -i)=%.1f\n", (float)param->communication_packet_interarrival_time);
	fprintf(fp_1, "communication_packet_ttl_override_flag (option -B)=%d\n", (int)param->communication_packet_ttl_override_flag);
	fprintf(fp_1, "communication_packet_ttl (option -t)=%.1f\n", (float)param->communication_packet_ttl);
	fprintf(fp_1, "communication_packet_hop_limit (option -H)=%d\n", param->communication_packet_hop_limit);
	fprintf(fp_1, "communication_packet_maximum_number (option -j)=%.0f\n", (float)param->communication_packet_maximum_number);
	fprintf(fp_1, "communication_multiple_AP_flag (option -M)=%d\n", (int)param->communication_multiple_AP_flag);
	fprintf(fp_1, "communication_AP_maximum_number (option -N)=%.0f\n", (float)param->communication_AP_maximum_number);
	fprintf(fp_1, "generated_packet_number=%d\n", packet_delivery_statistics.generated_packet_number);
	fprintf(fp_1, "generated_packet_copy_number=%d\n", packet_delivery_statistics.generated_packet_copy_number);
	fprintf(fp_1, "delivered_packet_number=%d\n", packet_delivery_statistics.delivered_packet_number);
	fprintf(fp_1, "discarded_packet_number=%d\n", packet_delivery_statistics.discarded_packet_number);
	fprintf(fp_1, "packet_delivery_ratio=%f\n", (float)packet_delivery_statistics.packet_delivery_ratio);
	fprintf(fp_1, "mean_expected_delivery_delay=%f\n", (float)packet_delivery_statistics.mean_expected_delivery_delay);
	fprintf(fp_1, "mean_actual_delivery_delay=%f\n", (float)packet_delivery_statistics.mean_actual_delivery_delay);
	fprintf(fp_1, "ratio_of_two_delivery_delays=%f\n", (float)packet_delivery_statistics.ratio_of_two_delivery_delays);
	fprintf(fp_1, "mean_expected_delivery_delay_standard_deviation=%f\n", (float)packet_delivery_statistics.mean_expected_delivery_delay_standard_deviation);
	fprintf(fp_1, "mean_expected_packet_transmission_number=%u\n", packet_delivery_statistics.mean_expected_packet_transmission_number);
	fprintf(fp_1, "mean_actual_packet_transmission_number=%u\n", packet_delivery_statistics.mean_actual_packet_transmission_number);

	/** store summary into output file 1 */
	//fprintf(fp_2, "%d\t%d\t%d\t%d\t%d\t%d\t%f\t%f\t%f", seed, param->vehicle_vanet_edd_model, param->vehicle_maximum_number, packet_delivery_statistics.generated_packet_number, packet_delivery_statistics.delivered_packet_number, packet_delivery_statistics.discarded_packet_number, (float)packet_delivery_statistics.mean_expected_delivery_delay, (float)packet_delivery_statistics.mean_actual_delivery_delay, (float)packet_delivery_statistics.ratio_of_two_delivery_delays);


#ifdef __DEBUG_INTERACTIVE_MODE__
	/** display summary */
	printf("\n/*** simulation results ***/\n");
	
	printf("data_forwarding_type (option -F)=%s\n", get_data_forwarding_mode_name(param->data_forwarding_mode));
	printf("vanet_forwarding_scheme (option -f)=%s\n", get_vanet_forwarding_scheme_name(param->vanet_forwarding_scheme));

	printf("tpd_encounter_graph_source_routing_flag (option -S)=%d\n", param->tpd_encounter_graph_source_routing_flag);
	printf("tpd_encounter_graph_optimization_flag (option -O)=%d\n", param->tpd_encounter_graph_optimization_flag);
	printf("tpd_encounter_probability_threshold=%.2f\n", param->tpd_encounter_probability_threshold);
	printf("tpd_delivery_probability_threshold=%.2f\n", param->tpd_delivery_probability_threshold);

	printf("seed=%d\n", seed);
	printf("simulation_time=%.1f [sec]\n", param->simulation_time);
	printf("graph_node_number (option -k)=%d\n", param->graph_node_number);
	printf("graph_file_name=%s\n", param->graph_file_name);
    printf("data_forwarding_two_way_forwarding_flag (option -K)=%d\n", param->data_forwarding_two_way_forwarding_flag);
    printf("data_forwarding_multiple_target_point_flag (option -W)=%d\n", param->data_forwarding_multiple_target_point_flag);
    printf("data_forwarding_maximum_target_point_number=%d\n", param->data_forwarding_maximum_target_point_number);
    printf("forwarding_probability_and_statistics_flag=%d\n", (int)param->forwarding_probability_and_statistics_flag);
    printf("packet_delay_measurement_flag (option -X)=%d\n", (int)param->packet_delay_measurement_flag);
    printf("packet_delay_measurement_target_point (option -Y)=%d\n", param->packet_delay_measurement_target_point);
    printf("packet_delay_measurement_time (option -Z)=%.1f\n", param->packet_delay_measurement_time);
	printf("vehicle_vanet_target_point_selection_type=%d\n", param->vehicle_vanet_target_point_selection_type);
	printf("vehicle_vanet_target_point_computation_method (option -C)=%d\n", param->vehicle_vanet_target_point_computation_method);
	printf("vehicle_vanet_target_point_search_space_type (option -P)=%d\n", param->vehicle_vanet_target_point_search_space_type);
	printf("vehicle_vanet_target_point_recomputation_interval_denominator (option -I)=%.1f\n", param->vehicle_vanet_target_point_recomputation_interval_denominator);
	printf("vehicle_vanet_vehicle_trajectory_type (option -T)=%d\n", param->vehicle_vanet_vehicle_trajectory_type);
	printf("vehicle_vanet_vehicle_trajectory_length_type (option -D)=%d\n", param->vehicle_vanet_vehicle_trajectory_length_type);
	printf("vehicle_vanet_vehicular_traffic_model (option -m)=%d\n", param->vehicle_vanet_vehicular_traffic_model);
	printf("vehicle_vanet_acl_measurement_flag (option -h)=%d\n", param->vehicle_vanet_acl_measurement_flag);
	printf("vehicle_vanet_edd_and_link_model (option -l)=%d\n", param->vehicle_vanet_edd_and_link_model);
	printf("vehicle_vanet_edd_model (option -e)=%d\n", param->vehicle_vanet_edd_model);
	printf("vehicle_vanet_tbd_edd_computation_type (option -c)=%d\n", param->vehicle_vanet_tbd_edd_computation_type);
	printf("vehicle_vanet_edge_delay_model (option -d)=%d\n", param->vehicle_vanet_edge_delay_model);
	printf("vehicle_vanet_intersection_forwarding_type (option -f)=%d\n", param->vehicle_vanet_intersection_forwarding_type);
	printf("vehicle_maximum_number (option -n)=%d\n", param->vehicle_maximum_number);
	printf("vehicle_packet_generating_entity_number (option -g)=%d\n", param->vehicle_packet_generating_entity_number);
	printf("vehicle_vanet_stationary_vehicle_flag (option -o)=%d\n", param->vehicle_vanet_stationary_vehicle_flag);
    printf("vehicle_interarrival_time (option -z)=%.1f\n", (float)param->vehicle_interarrival_time);
	printf("vehicle_think_time (option -H)=%.4f\n", (float)param->vehicle_think_time);
	printf("vehicle_speed_distribution (option -q)=%d where EQUAL=1, UNIFORM=2, and NORMAL=3\n", param->vehicle_speed_distribution);
	printf("vehicle_path_length_distribution (option -V)=%d where EQUAL=1 and NORMAL=3\n", param->vehicle_path_length_distribution);
	printf("communication_packet_delivery_probability_threshold (option -E)=%.4f\n", (float)param->communication_packet_delivery_probability_threshold);
	printf("communication_packet_reverse_traversal_hop_distance_threshold (option -J)=%d\n", param->communication_packet_reverse_traversal_hop_distance_threshold);
	printf("communication_range (option -r)=%.1f\n", (float)param->communication_range);
	printf("communication_packet_interarrival_time (option -i)=%.1f\n", (float)param->communication_packet_interarrival_time);
	printf("communication_packet_ttl_override_flag (option -B)=%d\n", (int)param->communication_packet_ttl_override_flag);
	printf("communication_packet_ttl (option -t)=%.1f\n", (float)param->communication_packet_ttl);
	printf("communication_packet_hop_limit (option -H)=%d\n", param->communication_packet_hop_limit);
	printf("communication_packet_maximum_number (option -j)=%.0f\n", (float)param->communication_packet_maximum_number);
	printf("communication_multiple_AP_flag (option -M)=%d\n", (int)param->communication_multiple_AP_flag);
	printf("communication_AP_maximum_number (option -N)=%.0f\n", (float)param->communication_AP_maximum_number);
	printf("generated_packet_number=%d\n", packet_delivery_statistics.generated_packet_number);
	printf("generated_packet_copy_number=%d\n", packet_delivery_statistics.generated_packet_copy_number);
	printf("delivered_packet_number=%d\n", packet_delivery_statistics.delivered_packet_number);
	printf("discarded_packet_number=%d\n", packet_delivery_statistics.discarded_packet_number);
	printf("packet_delivery_ratio=%f\n", (float)packet_delivery_statistics.packet_delivery_ratio);
	printf("mean_expected_delivery_delay=%f\n", (float)packet_delivery_statistics.mean_expected_delivery_delay);
	printf("mean_actual_delivery_delay=%f\n", (float)packet_delivery_statistics.mean_actual_delivery_delay);
	printf("ratio_of_two_delivery_delays=%f\n", (float)packet_delivery_statistics.ratio_of_two_delivery_delays);
	printf("mean_expected_delivery_delay_standard_deviation=%f\n", (float)packet_delivery_statistics.mean_expected_delivery_delay_standard_deviation);
	printf("mean_expected_packet_transmission_number=%u\n", packet_delivery_statistics.mean_expected_packet_transmission_number);
	printf("mean_actual_packet_transmission_number=%u\n", packet_delivery_statistics.mean_actual_packet_transmission_number);
#endif

	/** store VANET performance evaluation result into excel file called output*.xls */
	store_vanet_evaluation_result_into_file(fp_2, param, seed, &packet_delivery_statistics, Gr);
        /*********************************************************************************/

        /* store the forwarding probability and statistics into the file pointed by fp_1 */
        //strcpy(tail_node_buf, "14");
        //strcpy(head_node_buf, "9");
        //store_forwarding_probability_and_statisticis_into_file(fp_1, Gr, Gr_size, &DEr, tail_node_buf, head_node_buf);

        /****************************************/

	/** release dynamic memory for data structures */
	Free_Traffic_Table(&src_table_for_Gr); //release the memory occupied by the traffic source table in real graph Gr
	Free_Traffic_Table(&dst_table_for_Gr); //release the memory occupied by the traffic destination table in real graph Gr
	Free_Traffic_Table(&ap_table_for_Gr); //release the memory occupied by the traffic AP table in real graph Gr
	Free_Traffic_Table(&sn_table_for_Gr); //release the memory occupied by the traffic SN table in real graph Gr

	Free_Path_Table(&path_table); //release the memory occupied by the path table
	Free_Schedule_Table(&sched_table); //release the memory occupied by the schedule table

        /** destroy the data structures in vanet information table in param, such as road network graph Gr, the movement shortest path matrices, the EDD shortest path matrices, etc.; Note that these data structures can be reallocated memory, so the original pointers may not point to the actual memory. */
	destroy_vanet_information_table_in_parameter(param); 

    /* reset variables for real graph Gr */
	Gr = NULL; 
	Gr_size = 0;

	/* reset matrices for movement in Gr */
	Dr_move = NULL;
	Mr_move = NULL;
	matrix_size_for_movement_in_Gr = 0;

	/* reset matrices for EDD in Gr */
	Dr_edd = NULL;
	Mr_edd = NULL;
	Sr_edd = NULL;
	matrix_size_for_edd_in_Gr = 0;

	/* reset matrices for EDC in Gr */
	Wr_edc = NULL;
	Dr_edc = NULL;
	Mr_edc = NULL;
	Sr_edc = NULL;
	matrix_size_for_edc_in_Gr = 0;

	/* destroy queues */
	DestroyQueue((queue_t*) &Er); //destory edge queue Er
	DestroyQueue((queue_t*) &DEr); //destory directional edge queue DEr
	DestroyQueue((queue_t*) &DVQ); //destory destination vehicle queue DVQ
	DestroyQueue((queue_t*) &APQ); //destory access point queue APQ
	DestroyQueue((queue_t*) &SNQ); //destory stationary node queue SNQ
	//DestroyQueue((queue_t*) &FTQ); //destory forwarding table queue FTQ
	DestroyQueue((queue_t*) &GPQ); //destory global packet queue GPQ
	DestroyQueue((queue_t*) &TPQ); //destory target point queue TPQ

	/* destroy the set of road network graphs and the set of the corresponding directional edge queues */
	DestroyRoadNetworkGraphSet_And_DirectionalEdgeQueueSet(Gr_set_number, Gr_set, Gr_set_size, DEr_set);

	/* free the memory for statistics for traffic arrivals each source */
	//printf("free its\n");
	free(interarrival_time_sum); interarrival_time_sum = NULL;
	//printf("free an\n");
	free(arrival_number); arrival_number = NULL;
	//printf("free pat\n");
	free(previous_arrival_time); previous_arrival_time = NULL;
	//printf("free ait\n");
	free(average_interarrival_time); average_interarrival_time = NULL;
	//printf("free vl\n");
	/* release dynamic memory for vehicle_list */
	free_vehicle_list();


	/* delete the AP by freeing the memory allocated to AP only when data_forwarding_mode is download */
	//if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
	//  AP_Delete(&AP);

	/* close output_file_1 */
	fclose(fp_1);

	/* close output_file_2 */
	fclose(fp_2);

#ifdef __LOG_LEVEL_PATH_LIST__
	/* close pathlist log file */
	close_path_list_file();
#endif

#if defined(__LOG_LEVEL_VANET_PACKET_DROP__) || defined(__LOG_LEVEL_VANET_PACKET_AP_ARRIVAL__) || defined(__LOG_LEVEL_VANET_PACKET_DESTINATION_VEHICLE_ARRIVAL__)
	/* close the file pointer for VANET log file */
	close_vanet_file();
#endif

#if defined(__LOG_LEVEL_VANET_PACKET_CARRIER_TRACE__) || defined(__LOG_LEVEL_VANET_PACKET_CARRIER_TRACE_FOR_STATIONARY_NODE__)
	/* close the file pointer for VANET packet carrier trace log file */
	close_vanet_packet_carrier_trace_file();
#endif

	/** close trace files for observing system behavior */
#ifdef __LOG_LEVEL_TRACE_OF_VEHICLE_CONVOY_LENGTH__
	close_trace_file(fp_trace_file_of_vehicle_convoy_length);
	fp_trace_file_of_vehicle_convoy_length = NULL;
#endif


	return 0;
}


/** File: util.h
	Description: specify the utility functions
	Date: 07/25/2006
	Maker: Jaehoon Jeong, jjeong@cs.umn.edu
*/

#ifndef __UTIL_H__
#define __UTIL_H__

#include "common.h"
#include "schedule.h"
#include "vehicle-model.h"

/* Macro functions */
#define MAX(a,b) ((a>=b) ? a : b)
#define MIN(a,b) ((a<=b) ? a : b)

double MAX_In_Double(double a, double b);
//((a>=b) ? a : b)
double MIN_In_Double(double a, double b); 
//((a<=b) ? a : b)

int MAX_In_Integer(int a, int b);
int MIN_In_Integer(int a, int b);

/* function declaration */

real dist_func(int type, ...); //distribution function, such as expntl(), normal(), or uniform()

real delay_func(struct parameter *param, int type, ...); //return delay according to type of component, such as think-time

void init_vehicle_list(); 
//initialize vehicle_list by let vehicle_list.next and vehicle_list.prev pointing to itself

void vehicle_insert(struct struct_vehicle* vehicle); //insert a vehicle node into vehicle_list

boolean vehicle_delete(int id); //delete a vehicle node corresponding to id from vehicle_list

void free_vehicle_list(); //free vehicle list along with the allocated memory

struct struct_vehicle* vehicle_search(int id); //search a vehicle node corresponding to id from vehicle_list

boolean isdetected(struct struct_sensor* sensor_list, int sensor_number, double x, double y); //check if the vehicle with its position (x, y) is detected by an active sensor

double estimate_energy_consumption(double time, double sensing_range, double energy_consumption_rate); //estimate the amount of energy consumption to consider sensor's working time

double estimate_working_time(double energy, double sensing_range, double energy_consumption_rate); //estimate the available working time to consider the residual energy to schedule sensing

void initialize_density(struct struct_sensor* sensor_list, int sensor_number, struct parameter* param); //initialize sensing & communication density with number of one-hop neighbors

void reduce_density(struct struct_sensor* sensor, struct struct_sensor* sensor_list, int sensor_number, struct parameter* param); //reduce sensing and communication density of neighbors of the dead sensor by 1

double adjust_sensing_range(double com_density, double sen_density, struct parameter* param); //adjust a sensor's sensing range with its sensing density

void log_sensornet(struct struct_sensor* sensor_list, int sensor_number, double time, struct parameter* param); //log sensor network with sensing ranges

//void log_track(double current_time, double refresh_time, struct struct_vehicle *vehicle, struct struct_sensor *sensor_list, int total_sensor_number, int *S_ID, double *S_X, double *S_Y, int *p_live_sensor_number);
//log sensor network with sensing ranges

//void close_track_file(); //close tracking log file

int count_neighbor_sensors(double x, double y, struct struct_sensor* sensor_list, int sensor_number, double range);
//count the number of neighbor sensors for the vehicle where range is either sensing range or communication range.

double degree2radian(double degree); //convert angle in degree into angle in radian

double km_hour2m_sec(double speed); //convert speed in [km/hour] into speed in [m/sec]

int update_live_sensor_list(struct struct_sensor *sensor, int *S_ID, double *S_X, double *S_Y, int *p_live_sensor_number);
//update live-sensor-list S_ID, S_X, and S_Y with dying sensor

void assert_memory(void *memory); //assert the memory allocation

void open_localization_file(char *filename);
//open a file pointer for localization log file 

int log_localization(char *sensor_id, double current_time, int vehicle_id, int event, struct struct_sensor *sensor_list);
//log vehicle detections on the linear sensor network

void close_localization_file();
//close localization log file

void open_path_list_file(char *filename);
//open a file pointer for path-list log file 

int log_path_list(char* src, char* dst, double current_time, int vehicle_id, struct_path_node* path_list);
//log the path list

void close_path_list_file();
//close path_list log file

float mean_integer(int i, int j);
//return the mean between i and j

int sum_integer(int i, int j);
//return the sum of integers between i and j

int sum_vector(int *V, int i, int j);
//return the sum of vector values between indices i and j

void open_surveillance_file(char *filename);
//open a file pointer for surveillance log file 

int log_surveillance_for_sensor(int sensor_id, double current_time, STATE event, struct_sensor_table *S);
//log the status of sensor on the road network

int log_surveillance_for_vehicle(int vehicle_id, double current_time, STATE event, struct_coordinate2_t *pos, MOVE_TYPE move_type, edge_queue_t *E);
//log the status of vehicle on the road network

int log_surveillance_for_detection(int sensor_id, double current_time, int vehicle_id, double movement_time, STATE event, struct_sensor_table *S);
//log vehicle detections for surveillance on the road network

void close_surveillance_file();
//close surveillance log file

void open_vanet_file(char *filename);
//open a file pointer for VANET log file 

int log_vanet(VANET_LOG_TYPE type, double current_time, packet_queue_node_t *p, packet_delivery_statistics_t *packet_delivery_stat);
//log a VANET event into VANET logging file

void close_vanet_file();
//close VANET log file

void open_vanet_packet_carrier_trace_file(char *filename);
//open a file pointer for VANET packet carrier trace log file

int log_vanet_packet_carrier_trace(VANET_LOG_TYPE log_type, double current_time, packet_queue_node_t *packet);
//log a VANET event into VANET logging file

int log_vanet_packet_carrier_trace_for_upload(VANET_LOG_TYPE log_type, double current_time, packet_queue_node_t *packet);
//log a VANET event into VANET logging file for upload forwarding mode 

int log_vanet_packet_carrier_trace_for_download(VANET_LOG_TYPE log_type, double current_time, packet_queue_node_t *packet);
//log a VANET event into VANET logging file for download forwarding mode 

int log_vanet_packet_carrier_trace_for_v2v(VANET_LOG_TYPE log_type, double current_time, packet_queue_node_t *packet);
//log a VANET event into VANET logging file for V2V forwarding mode 

void close_vanet_packet_carrier_trace_file();
//close VANET packet carrier trace log file

boolean is_vehicle_detected_for_eager_update(double current_time, struct_vehicle_t *vehicle, edge_queue_t *E, parameter_t *param, struct_sensor_table *S);
//check whether the vehicle is detected by some sensor(s) or not and return the queue of detection sensors in the eager update mode in sensor scheduling. We need to release the memory occupied by vehicle's detection queue Q outside this function.

boolean is_vehicle_detected_for_eager_update_and_step_path(double current_time, struct_vehicle_t *vehicle, edge_queue_t *E, parameter_t *param, struct_sensor_table *S, struct_traffic_table *protection_set);
//check whether the vehicle is detected by some sensor(s) or not and return the queue of detection sensors in the eager update mode in sensor scheduling along with STEP_PATH.

boolean is_vehicle_detected_for_lazy_update(double current_time, struct_vehicle_t *vehicle, edge_queue_t *E, parameter_t *param, struct_sensor_table *S);
//check whether the vehicle is detected by some sensor(s) or not and return the queue of detection sensors in the lazy update mode in sensor scheduling. We need to release the memory occupied by vehicle's detection queue Q outside this function.

boolean is_vehicle_detected_for_lazy_update_and_step_path(double current_time, struct_vehicle_t *vehicle, edge_queue_t *E, parameter_t *param, struct_sensor_table *S, struct_traffic_table *protection_set);
//check whether the vehicle is detected by some sensor(s) or not and return the queue of detection sensors in the lazy update mode in sensor scheduling along with STEP_PATH.

double euclidean_distance1(double x1, double x2);
//return the Euclidean distance between two points in one-dimension

double euclidean_distance2(struct_coordinate1_t *p1, struct_coordinate1_t *p2);
//return the Euclidean distance between two points in two-dimensional space, i.e., plane

boolean is_vehicle_within_communication_range_of_point(parameter_t *param, struct_vehicle_t *vehicle, struct_coordinate1_t *point);
//check whether the vehicle is within the communication range of point

char* get_distribution_type_name(distribution_type_t type);
//return the distribution type name in string

char* get_scan_type_name(sensor_scan_type_t scan_type);
//return the scan type name in string

char* get_sensing_hole_handling_algorithm_name(hole_handling_algorithm_t sensing_hole_handling_algorithm);
//return the sensing hole handling algorithm name in string

boolean is_sensing_hole(sensor_queue_node_t *pSensorNode, schedule_table_node_t *pTableNode, double *left_hole_offset, double *right_hole_offset);
//check if this sensor's death creates a sensing hole

boolean is_initial_sensing_hole(int order, sensor_queue_node_t *pSensorNode, schedule_table_node_t *pTableNode, double *left_hole_offset, double *right_hole_offset);
//check whether or not there is a sensing hole segment to the left of the sensor, but the last sensor also checks whether or not there is a righ hole segment

void show_status_of_sensors_on_path(FILE *fp, edge_queue_t *E, struct_sensor_table *S, struct_path_node *path_list);
//show live sensors and dead sensors on a path in road graph along with the list of vertices on the path from source to destination

int factorial(int n);
//return the n! (i.e., n factorial)

int permutation(int n, int r);
//return the number of all possible cases in the permutation nPr

int combination(int n, int r);
//return the number of all possible cases in the combination nCr

boolean is_there_breach_path_based_on_breach_path_matrix(struct_graph_node *Gv, int Gv_size, int ***Dv_breach, int ***Mv_breach, int *matrix_size_for_breach_in_Gv, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, char *breach_path_src, char *breach_path_dst);
//check whether there is a breach path between an arbitrary pair of entrance point and protection point, considering all possible breach paths based on breach path matrix

boolean is_there_breach_path_based_on_shortest_path_checking(struct_path_table *path_table, struct_traffic_table *src_table_for_Gr, struct_traffic_table *dst_table_for_Gr, edge_queue_t *Er, char *breach_path_src, char *breach_path_dst);
//check whether there is a breach path between an arbitrary pair of entrance point and protection point by checking whether there is the shortest path between entrance point and protection point with no live sensor.

void store_sensor_location_into_file(edge_queue_t *Er, char* filename);
//store sensor location in edge queue into a file called filename

struct_vehicle_t* register_vehicle(int vehicle_id, char *traffic_source, double arrival_time, struct_traffic_table *dst_table_for_Gr, struct_traffic_table *ap_table_for_Gr, struct parameter *param, struct_path_table *path_table, struct_graph_node *Gr, int Gr_size, edge_queue_t *Er, double **Dr_move, int **Mr_move);
//register a new vehicle to vehicle list

void update_vehicle_trajectory(struct_vehicle_t* vehicle, double arrival_time, struct_traffic_table *dst_table_for_Gr, struct_traffic_table *ap_table_for_Gr, struct parameter *param, struct_path_table *path_table, struct_graph_node *Gr, int Gr_size, edge_queue_t *Er, double **Dr_move, int **Mr_move);
//update the vehicle's trajectory given the source node in the real graph

void register_all_vehicle_movement(parameter_t *param, double registration_time, struct_graph_node *G);
/* register all vehicles' movement into the corresponding vehicle_movement_list of the directional edge 
   pointed by a graph node in G where each vehicle is moving */

void register_vehicle_movement(parameter_t *param, struct_vehicle_t* vehicle, double registration_time, struct_graph_node *G);
//register the vehicle's movement into the vehicle_movement_list of the directional edge pointed by a graph node in G

void delete_vehicle_movement(parameter_t *param, struct_vehicle_t* vehicle, double departure_time, struct_graph_node *G);
//delete the vehicle's movement from the vehicle_movement_list of the directional edge pointed by a graph node in G with vehicle id

void process_acl_convoy_for_vehicle_arrival(parameter_t *param, struct_vehicle_t *vehicle, double registration_time, directional_edge_queue_node_t *edge);
//process the vehicle convoy for a new vehicle arrival into the directional edge in order to compute the ACL for the directional edge

void process_acl_convoy_for_vehicle_departure(parameter_t *param, struct_vehicle_t *vehicle, double departure_time, directional_edge_queue_node_t *edge);
//process the vehicle convoy for a new vehicle departure from the directional edge in order to compute the ACL for the directional edge

void update_all_vehicle_edd(double update_time, parameter_t *param, struct_graph_node *G, int G_size, struct_traffic_table *ap_table);
//update all vehicles' EDDs using each vehicle's offset in directional edge along with real graph G and AP table 

void update_all_vehicle_edd_and_edd_sd(double update_time, parameter_t *param, struct_graph_node *G, int G_size, struct_traffic_table *ap_table);
//update all vehicles' EDDs and EDD_SDs using each vehicle's offset in directional edge along with real graph G and AP table 

double get_average_length_of_shortest_paths(double** D, int n, struct_traffic_table *src_table, struct_traffic_table *dst_table);
//compute the average length of the shortest paths between the arbitrary pair of source and destination

boolean adjust_sensor_energy_budget(double current_time, struct_sensor_t *sensor_info);
//adjust the sensor energy budget by the energy consumed by the number of duty cycles from the last restart time to current_time

STATE get_sensor_state(double current_time, struct_sensor_t *sensor_info);
//get the sensor's state at current_time along with its scheduling and remaining energy budget

double get_sensing_circle_meeting_time(double t_arrive, double t_depart, struct_sensor_t *sensor_info, STATE *state);
//determine the time point when the vehicle meets the starting time of the first working period during the movement between t_arrive and t_depart. If there is a working period overlapped during the movement time, return the time point; otherwise retirn -1.

double get_sensor_lifetime(struct_sensor_t *sensor_info, parameter_t *param);
//get sensor lifetime based on its energy budget along with its schedule starting from the sleeping time corresponding to the movement time over the shortest path

void update_all_sensors_state(double current_time, struct_sensor_table *S, parameter_t *param);
//update sensors' states with their remaining energy and work schedule at the current time

void update_sensor_state_with_energy_budget_and_work_schedule(double current_time, struct_sensor_t *sensor_info, parameter_t *param);
//update the sensor's state along with the sensor energy budget by the energy consumed by the number of duty cycles from the last restart time to current_time and with the updated sensing work schedule; this function is called to show the sensor information when a vehicle escapes from the sensor network without detection.

void store_evaluation_result_into_file(FILE *fp_1, FILE *fp_2, parameter_t *param, unsigned int seed, double sensor_network_lifetime, double average_detection_time, int number_of_detected_vehicles);
//store performance evaluation result into a file pointed by fp_1 and fp_2

void store_vanet_evaluation_result_into_file(FILE *fp, parameter_t *param, unsigned int seed, packet_delivery_statistics_t *packet_delivery_statistics, struct_graph_node *G);
//store VANET performance evaluation result into a file pointed by fp

/** conversion function betweem mile unit system and meter unit system */
double convert_mile_to_meter(double mile); //convert mile to meter
double convert_meter_to_mile(double meter); //convert meter to mile
double convert_mile_to_km(double mile); //convert mile to kilometer
double convert_km_to_mile(double meter); //convert kilometer to mile
double convert_mile_per_hour_to_km_per_hour(double speed_in_mile_per_hour); //convert the speed for mile per hour to the speed for kilometer(km) per hour
double convert_km_per_hour_to_mile_per_hour(double speed_in_km_per_hour); //convert the speed for kilometer per hour to the speed for km per hour
double convert_mile_per_hour_to_meter_per_sec(double speed_in_mile_per_hour); //convert the speed for mile per hour to the speed for meter per sec
double convert_meter_per_sec_to_mile_per_hour(double speed_in_meter_per_sec); //convert the speed for meter per sec to the speed for mile per hour
double convert_feet_to_meter(double feet); //convert feet to meter
double convert_meter_to_feet(double meter); //convert meter to feet

double convert_km_per_hour_to_meter_per_sec(double speed_in_km_per_hour); //convert the speed for km per hour to the speed for meter per sec
double convert_meter_per_sec_to_km_per_hour(double speed_in_meter_per_sec); //convert the speed for meter per sec to the speed for km per hour

/** functions for trace for observing system behavior **/
FILE* open_trace_file(char *filename); //open a file pointer for trace file
void log_trace_file(FILE *fp, TRACE_TYPE type, ...); //write an event into the trace file corresponding to the trace file type
void close_trace_file(FILE *fp); //close the file pointer for trace file

/** utility functions */
char* right_index(char *s, char c);
//returns a pointer to the last occurrence of the character c in the string s

double dist(double x1, double x2);
//Euclidean distance between x1 and x2

char *itoa(int i, char* a);
//convert integer i into character string a in the decimal format

/** Convoy Operations */
void convoy_all_vehicle_join(parameter_t *param, double join_time, struct_graph_node *G, int G_size, packet_delivery_statistics_t *packet_delivery_stat);
//let all vehicles join their own convoy moving on the directional edge pointed by a graph node in G where each vehicle is moving

void convoy_construct_convoys_in_graph(parameter_t *param, double join_time, struct_graph_node *G, int G_size, packet_delivery_statistics_t *packet_delivery_stat);
//construct convoys for each directional edge in graph G using the sorted vehicle movement queues

void convoy_construct_convoys_in_directional_edge(parameter_t *param, double join_time, vehicle_movement_queue_t *VQ, convoy_queue_t *CQ, packet_delivery_statistics_t *packet_delivery_stat);
//construct convoys moving on the directional edge <v_i,v_j> into convoy queue CQ along with vehicle_movement_list VQ

convoy_queue_node_t* convoy_construct(struct_vehicle_t *vehicle, double join_time);
//construct an atomic convoy only with the vehicle on the directional edge and register the convoy with the convoy queue of the directional edge

convoy_queue_node_t* convoy_add_vehicle(parameter_t *param, convoy_queue_node_t *pConvoyNode, struct_vehicle_t *vehicle, double join_time, packet_delivery_statistics_t *packet_delivery_stat);
//add the vehicle to the current convoy pointed by pConvoyNode and let the vehicle pass its packets to the convoy head

boolean convoy_join(parameter_t *param, struct_vehicle_t *vehicle, double join_time, packet_delivery_statistics_t *packet_delivery_stat);
//join vehicle into the convoy preceding the vehicle on the directional edge if the vehicle is within the communication range of the convoy

boolean convoy_fast_join(parameter_t *param, double join_time, struct_vehicle_t *vehicle, convoy_queue_node_t *dst_convoy);
//join vehicle into the dst_convoy close to vehicle on the directional edge

boolean convoy_leave(parameter_t *param, struct_vehicle_t *vehicle, double leave_time);
//let vehicle leave from the convoy including the vehicle on the directional edge if the vehicle reaches the directional edge's head or is out of the communication range of the convoy

boolean convoy_update(parameter_t *param, struct_vehicle_t *vehicle, double update_time);
//update the convoys close to the vehicle with the convoy operations, such as convoy_construct, convoy_split, and convoy_merge.

boolean convoy_merge(parameter_t *param, double merge_time, convoy_queue_node_t *src_convoy, convoy_queue_node_t *dst_convoy);
//let src_convoy merge into dst_convoy

boolean convoy_split(parameter_t *param, double split_time, convoy_queue_node_t *src_convoy, struct_vehicle_t *vehicle);
//let src_convoy split into dst_convoy_1 and dst_convoy_2 by using vehicle as the boundary of these two convoys; vehicle is the head of dst_convoy_2


/** operations for stationary vehicle */
void set_vehicle_trajectory(struct_vehicle_t *vehicle, double arrival_time, int *trajectory, int trajectory_size, struct_graph_node *Gr, edge_queue_t *Er);
//set up the vehicle's trajectory

void set_vehicle_current_position(parameter_t *param, struct_vehicle_t *vehicle, double current_time, int *trajectory, int trajectory_size, struct_graph_node *Gr, edge_queue_t *Er);
//set up the vehicle's current position in the road network graph Gr along with the deregistration of the previous movement and the registration of the current movement

/** Geometric functions */
double degree2radian(double degree); //convert the angle in degree into the angle in radian

double radian2degree(double radian); //convert the angle in radian into the angle in degree

void get_position_on_linear_curve(double v, double t, struct_coordinate1_t *p1, struct_coordinate1_t *p2, struct_coordinate1_t *p);
//get the position on the linear curve between two points p1 and p2 for the object moving with speed v after time t, returning the Euclidean coordinate p of vehicle after moving time t, starting from p1 towards p2

void get_position_on_linear_curve_for_offset(double offset, struct_coordinate1_t *p1, struct_coordinate1_t *p2, struct_coordinate1_t *p);
//get the position on the linear curve between two points p1 and p2 such that the position is offset away from p1, returning the current Euclidean coordinate p of vehicle moving from p1 to p2

void get_position_update_vector(struct_vehicle_t *vehicle, struct_graph_node *G, int G_size);
//get the position update vector p (i.e., delta-x and delta-y per time t) on the linear curve between two points p1 and p2 for the mobile vehicle moving with speed v per unit time, starting from p1 towards p2

/** TBR operations */
boolean does_vehicle_convoy_have_packet(struct_vehicle_t *vehicle, parameter_t *param);
//check whether the vehicle's convoy (i.e., convoy leader) or the vehicle has packets to forward according to vehicle_vanet_forwarding_type

boolean does_vehicle_have_packet(struct_vehicle_t *vehicle);
//check whether the vehicle has packets to forward

void replace_vehicle_trajectory_with_mobility_list(double arrival_time, parameter_t *param, struct_vehicle_t *vehicle, destination_vehicle_queue_node_t *pQueueNode, struct_graph_node *Gr, int Gr_size);
//replace vehicle's current trajectory with pQueueNode's mobility_list as a new trajectory

void update_vehicle_trajectory_with_mobility_list(double arrival_time, parameter_t *param, struct_vehicle_t *vehicle, destination_vehicle_queue_node_t *pQueueNode, struct_graph_node *Gr, int Gr_size);
//set vehicle's current trajectory with pQueueNode's mobility_list as a new vehicle trajectory along with mobility type of either MOBILITY_OPEN or MOBILITY_CLOSED

void set_vehicle_speed(parameter_t *param, struct_vehicle_t *vehicle);
//set up vehicle speed according to the speed distribution along with vehicle minimum speed and vehicle maximum speed

/** logging functions */
void store_forwarding_probability_and_statisticis_into_file(FILE *fp, struct_graph_node *G, int G_size, directional_edge_queue_t *Q, char *tail_node, char *head_node);
//store the forwarding probability and statistics into the file pointed by fp_1

boolean is_destination_vehicle(parameter_t *param, struct_vehicle_t *vehicle);
//check whether vehicle is a destination vehicle or not

/** combination function */
void combinations(int v[], int start, int n, int k, int maxk);
/* perform combinations function by enumerating all the possible cases of combination nCk 
 * where n is the number of items and k is the wanted number in the combination
 */

int test_combinations(int n, int k);
/* test the combination function called combinations 
 * such that nCk is the number of all the possible cases
 */

void combinations_for_target_points(int v[], int w[], int start, int n, int k, int maxk, double p[], double d[], parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, target_point_queue_t *global_TPQ);
/* evaluate the combinations for target points by enumerating all the possible cases of combination nCk 
 * where n is the number of items and k is the wanted number in the combination.
  Parameters:-
 	w: intersection vector whose element contains the intersection id 
	   that is one of intersections consisting of the destination 
	   vehicle's trajectory
			
 	v: index vector for intersection vector w;
	   for example, v[1] = 1 and w[v[1]] is the first intersection id

	p: delivery probability whose element contains 
	   the E2E delivery delay from AP to target point 
	   where the index i indicates a target point.
	   For example, d[i] is the delivery delay of target point w[i].
			
	d: delivery delay whose element contains 
	   the E2E delivery probability from AP to target point 
	   where the index indicates a target point.
	   For example, p[i] is the delivery probability of target point w[i].
 */

double compute_delivery_probability_along_with_delivery_delay_for_target_point(int target_point_id, parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, forwarding_table_queue_t *FTQ, double *delivery_delay);
//Given a target point, compute the delivery probability along with the delivery delay for the given target point

/** TPD Functions */
char* get_vanet_forwarding_scheme_name(vanet_forwarding_scheme_t scheme);
//return the vanet forwarding scheme in string

char* get_data_forwarding_mode_name(data_forwarding_mode_t type);
//return the data forwarding mode in string

struct struct_vehicle* get_vehicle_list();
//get the pointer to a static global variable vehicle_list

boolean compute_arrival_time_mean_and_standard_deviation_for_path_node(double arrival_time,
		parameter_t *param,
		struct_path_node *path_list);
//compute the mean and standard deviation of the arrival time for each path node along path_list of a vehicle with speed and speed_standard_deviation by param's vehicle speed

boolean compute_arrival_time_mean_and_standard_deviation_for_path_node_by_vehicle_actual_speed(double arrival_time,
		parameter_t *param,
		struct_path_node *path_list,
		struct_vehicle_t *vehicle);
//compute the mean and standard deviation of the arrival time for each path node along path_list of a vehicle with speed and speed_standard_deviation by vehicle's actual speed

boolean show_trajectory_and_arrival_time_for_all_vehicles();
//show the vehicle trajectory along with the arrival time per path node along the vehicle trajectory for all the vehicles in vehicle_list

boolean show_trajectory_and_arrival_time_for_vehicle(int vid, 
		struct_path_node *path_list); 
//show the vehicle trajectory along with the arrival time per path node along the vehicle trajectory with vehicle id (vid) and vehicle trajectory (path_list)

boolean store_trajectory_and_arrival_time_for_all_vehicles();
//store the vehicle trajectory along with the arrival time per path node along the vehicle trajectory for all the vehicles in vehicle_list

boolean store_trajectory_and_arrival_time_for_vehicle(FILE *fp,
		int vid, 
		struct_path_node *path_list); 
//store the vehicle trajectory along with the arrival time per path node along the vehicle trajectory with vehicle id (vid) and vehicle trajectory (path_list)

#endif

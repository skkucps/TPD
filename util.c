/** File: util.c
	Description: implement the utility functions
	Date: 07/25/2006
	Maker: Jaehoon Jeong, jjeong@cs.umn.edu
*/

#include "stdafx.h"
#include "util.h"
//#include "matlab-operation.h"
#include "shortest-path.h"
#include "vehicle-model.h"
#include "schedule.h"
#include "random-path.h"
#include "queue.h"
#include "all-pairs-shortest-paths.h"
#include "vadd.h"
#include "tpd.h"

#include <math.h> //sin(), cos(), atan(), etc

#include "gsl-util.h" //GSL_Vanet_Compute_TravelTime_And_Deviation()

static struct struct_vehicle vehicle_list; /* head node for vehicle list */
static FILE* fp_track_circle = NULL; /* file for logging target tracking based on circle */
static FILE* fp_track_contour = NULL; /* file for logging target tracking based on contour */
static FILE* fp_localization = NULL; /* file for logging the localization simulation */
static FILE* fp_path_list = NULL; /* file for logging the path lists */
static FILE* fp_surveillance = NULL; /* file for logging the road surveillance simulation */
static FILE* fp_vanet = NULL; /* file for logging the VANET simulation */
static FILE* fp_vanet_packet_carrier_trace = NULL; /* file for logging the VANET packet carrier trace */

real dist_func(int type, ...)
{ //function pointer to distribution function, such as expntl(), uniform(), normal(), erlang() or hyperx()
	real result = 0;
	va_list ap;
	real arg1, arg2;

	switch(type)
	{
		case EQUAL:
			va_start(ap, type); /* startup */
			arg1 = va_arg(ap, real); /* get the next argument */
			result = arg1; //return the same value contained by arg1
			break;

		case EXPONENTIAL:
			va_start(ap, type); /* startup */
			arg1 = va_arg(ap, real); /* get the next argument */
			result = expntl(arg1); //arg1 is mean inter-arrival time x
			break;

		case UNIFORM:
			va_start(ap, type); /* startup */
			arg1 = va_arg(ap, real); /* get the next argument */
			arg2 = va_arg(ap, real); /* get the next argument */
			result = uniform(arg1, arg2); //arg1 is lower bound and arg2 upper bound
			break;

		case NORMAL:
			va_start(ap, type); /* startup */
			arg1 = va_arg(ap, real); /* get the next argument */
			arg2 = va_arg(ap, real); /* get the next argument */
			result = normal(arg1, arg2); //arg1 is mean x and arg2 standard deviation s
			break;

		case ERLANG:
			va_start(ap, type); /* startup */
			arg1 = va_arg(ap, real); /* get the next argument */
			arg2 = va_arg(ap, real); /* get the next argument */
			result = erlang(arg1, arg2); //arg1 is mean x and arg2 standard deviation s
			break;

		case HYPERX:
			va_start(ap, type); /* startup */
			arg1 = va_arg(ap, real); /* get the next argument */
			arg2 = va_arg(ap, real); /* get the next argument */
			result = hyperx(arg1, arg2); //arg1 is mean x and arg2 standard deviation s
			break;

		default:
			printf("Error: distribution %d is not supported!\n", type);
			exit(1);
	} //end of switch

	va_end(ap); /* cleanup */

	return result;
}

real delay_func(struct parameter *param, int type, ...)
{ //return delay according to type of component, such as think-time, cross-traffic, network, gateway or disk
	va_list ap;
	real arg1, arg2;
	real result = 0;

	switch(type) //switch-1
	{
		case DELAY_SENSOR:
			switch(param->sensor_think_time_distribution)
			{
				case EXPONENTIAL:
					result = dist_func(param->sensor_think_time_distribution, param->sensor_think_time);
					break;

				case UNIFORM:
                                       //arg1 = param->sensor_think_time - param->sensor_think_time_standard_deviation;
                                       //arg2 = param->sensor_think_time + param->sensor_think_time_standard_deviation;
                                       /* @[11/26/2009] Update the definition of the uniform interval as below */
                                        arg1 = 0;
                                        arg2 = param->sensor_think_time;

					result = dist_func(param->sensor_think_time_distribution, arg1, arg2);
					break;

				case NORMAL:
				case ERLANG:
				case HYPERX:
					result = dist_func(param->sensor_think_time_distribution, param->sensor_think_time, param->sensor_think_time_standard_deviation);
					break;

				case EQUAL:
					result = param->sensor_think_time;
					break;

				default:
					printf("Error: distribution %d is not supported!\n", type);
				exit(1);
			}
			break;
			
		case DELAY_VEHICLE_INTERARRIVAL_TIME:
			switch(param->vehicle_interarrival_time_distribution)
			{ //@ 10/11/2006: I replace vehicle_think_time with vehicle_interarrival_time for delay_func
				case EXPONENTIAL:
					//result = dist_func(param->vehicle_interarrival_time_distribution, param->vehicle_interarrival_time);
					while(1)
					{
						result = dist_func(param->vehicle_interarrival_time_distribution, param->vehicle_interarrival_time);
						if(result <= param->vehicle_maximum_interarrival_time)
							break;
					}
					break;

				case UNIFORM:
                                       //arg1 = param->vehicle_interarrival_time - param->vehicle_interarrival_time_standard_deviation;
                                        //arg2 = param->vehicle_interarrival_time + param->vehicle_interarrival_time_standard_deviation;
                                       /* @[11/26/2009] Update the definition of the uniform interval as below */
					arg1 = 0;
					arg2 = param->vehicle_interarrival_time;
					result = dist_func(param->vehicle_interarrival_time_distribution, arg1, arg2);
					break;

				case NORMAL:
				case ERLANG:
				case HYPERX:
					result = dist_func(param->vehicle_interarrival_time_distribution, param->vehicle_interarrival_time, param->vehicle_interarrival_time_standard_deviation);
					break;

				case EQUAL:
					result = param->vehicle_interarrival_time;
					break;

				default:
					printf("Error: distribution %d is not supported!\n", type);
				exit(1);
			}
			break;

		case DELAY_VEHICLE_SPEED:
			switch(param->vehicle_speed_distribution)
			{ //@ 10/11/2006: I replace vehicle_think_time with vehicle_interarrival_time for delay_func
				case EXPONENTIAL:
					result = dist_func(param->vehicle_speed_distribution, param->vehicle_speed);
					break;

				case UNIFORM:
					arg1 = param->vehicle_speed - param->vehicle_speed_standard_deviation;
					arg2 = param->vehicle_speed + param->vehicle_speed_standard_deviation;
					result = dist_func(param->vehicle_speed_distribution, arg1, arg2);
					break;

				case NORMAL:
				case ERLANG:
				case HYPERX:
					result = dist_func(param->vehicle_speed_distribution, param->vehicle_speed, param->vehicle_speed_standard_deviation);
					break;

				case EQUAL:
					result = param->vehicle_speed;
					break;

				default:
					printf("Error: distribution %d is not supported!\n", type);
				exit(1);
			}
			break;

		case DELAY_VEHICLE_THINK_TIME:
			switch(param->vehicle_think_time_distribution)
			{ //@ 10/11/2006: I replace vehicle_think_time with vehicle_interarrival_time for delay_func
				case EXPONENTIAL:
					result = dist_func(param->vehicle_think_time_distribution, param->vehicle_think_time);
					break;

				case UNIFORM:
                                        //arg1 = param->vehicle_think_time - param->vehicle_think_time_standard_deviation;
					//arg2 = param->vehicle_think_time + param->vehicle_think_time_standard_deviation;
                                       /* @[11/26/2009] Update the definition of the uniform interval as below */
                                        arg1 = 0;
					arg2 = param->vehicle_think_time;
					result = dist_func(param->vehicle_think_time_distribution, arg1, arg2);
					break;

				case NORMAL:
				case ERLANG:
				case HYPERX:
					result = dist_func(param->vehicle_think_time_distribution, param->vehicle_think_time, param->vehicle_think_time_standard_deviation);
					break;

				case EQUAL:
					result = param->vehicle_think_time;
					break;

				default:
					printf("Error: distribution %d is not supported!\n", type);
				exit(1);
			}
			break;

		case DELAY_PACKET_INTERARRIVAL_TIME:
			switch(param->communication_packet_interarrival_time_distribution)
			{
				case EXPONENTIAL:
					//result = dist_func(param->communication_packet_interarrival_time_distribution, param->communication_packet_interarrival_time);
					while(1)
					{
						result = dist_func(param->communication_packet_interarrival_time_distribution, param->communication_packet_interarrival_time);
						if(result <= param->communication_packet_maximum_interarrival_time)
							break;
					}
					break;

				case UNIFORM:
                                        //arg1 = param->communication_packet_interarrival_time - param->communication_packet_interarrival_time_standard_deviation;
                                        //arg2 = param->communication_packet_interarrival_time + param->communication_packet_interarrival_time_standard_deviation;
                                       /* @[11/26/2009] Update the definition of the uniform interval as below */
                                        arg1 = 0;
					arg2 = param->communication_packet_interarrival_time;
					result = dist_func(param->communication_packet_interarrival_time_distribution, arg1, arg2);
					break;

				case NORMAL:
				case ERLANG:
				case HYPERX:
					result = dist_func(param->communication_packet_interarrival_time_distribution, param->communication_packet_interarrival_time, param->communication_packet_interarrival_time_standard_deviation);
					break;

				case EQUAL:
					result = param->communication_packet_interarrival_time;
					break;

				default:
					printf("Error: distribution %d is not supported!\n", type);
				exit(1);
			}
			break;
			
		default:	
			printf("Error: type %d is not supported!\n", type);
			exit(1);
			
	}; //end of switch-1
	
	va_end(ap); /* cleanup */
        
	if (result < 0.0) result = 0.0;
	return result;
}


void init_vehicle_list()
{ //initialize vehicle_list by let vehicle_list.next and vehicle_list.prev pointing to itself
	vehicle_list.next = &vehicle_list;
	vehicle_list.prev = &vehicle_list;
}

void vehicle_insert(struct struct_vehicle* vehicle)
{ //insert a vehicle node into vehicle_list
	vehicle_list.prev->next = vehicle;
	vehicle->prev = vehicle_list.prev;
	vehicle->next = &vehicle_list;
	vehicle_list.prev = vehicle;
}


boolean vehicle_delete(int id)
{ //delete a vehicle node corresponding to id from vehicle_list
	struct struct_vehicle* ptr;

	for(ptr = vehicle_list.next; ptr != &vehicle_list; ptr = ptr->next)
	{
		if(id == ptr->id)
		{
			ptr->prev->next = ptr->next;
			ptr->next->prev = ptr->prev;

			/* release the memory occupied by the list for the shortest path from source to destination */
			Free_Path_List(ptr->path_list);

			/* release the memory allocated to sensor queue */
			if(ptr->sensor_queue->size > 0)
			{
			  DestroyQueue((queue_t*)ptr->sensor_queue); //free the memory allocated to the sensor queue nodes of sensor queue
			  free(ptr->sensor_queue); //free the memory allocated to sensor queue
			}

			/* release the memory allocated to packet queue */
			if(ptr->packet_queue->size > 0)
			{
			  DestroyQueue((queue_t*)ptr->packet_queue); //free the memory allocated to the packet queue nodes of packet queue
			  free(ptr->packet_queue); //free the memory allocated to packet queue
			}

			/* free the memory of a predicted encounter graph */
			TPD_Free_Predicted_Encounter_Graph(ptr);

			free(ptr); //free the memory allocated to vehicle node
			return TRUE;
		}
	}

	return FALSE;
}

void free_vehicle_list()
{ //free vehicle list along with the allocated memory
	struct struct_vehicle* ptr; //pointer to vehicle node
	struct struct_vehicle* q; //pointer to the deleted vehicle node

	ptr = vehicle_list.next;
	while(ptr != &vehicle_list)
	{
	  q = ptr;

	  /* move ptr to the next vehicle node */
	  ptr = ptr->next;

	  /* release the memory occupied by the list for the shortest path from source to destination */
	  Free_Path_List(q->path_list);

/* release the memory allocated to sensor queue */
	  if(q->sensor_queue->size > 0)
	  {
	    DestroyQueue((queue_t*)q->sensor_queue); //free the memory allocated to the sensor queue nodes of sensor queue
	    free(q->sensor_queue); //free the memory allocated to sensor queue
	  }

	  /* release the memory allocated to packet queue Q */
	  if(q->packet_queue->size > 0)
	  {
	    DestroyQueue((queue_t*)q->packet_queue); //free the memory allocated to the packet queue nodes of packet queue
	    free(q->packet_queue); //free the memory allocated to packet queue
	  }

	  /* free the memory of a predicted encounter graph */
	  TPD_Free_Predicted_Encounter_Graph(q);

	  free(q); //free the memory allocated to the vehicle node
	}

	init_vehicle_list(); //initialize vehicle_list to point to itself since there is no actual vehicle node
}

struct struct_vehicle* vehicle_search(int id)
{ //search a vehicle node corresponding to id from vehicle_list
	struct struct_vehicle* ptr;

	if(&vehicle_list == vehicle_list.next)
		return NULL; //there is no vehicle node in the vehicle list

	for(ptr = vehicle_list.next; ptr != &vehicle_list; ptr = ptr->next)
	{
		if(id == ptr->id)
		{
			return ptr;
		}
	}

	return NULL;
}

boolean isdetected(struct struct_sensor* sensor_list, int sensor_number, double x, double y)
{ //check if the vehicle with its position (x, y) is detected by an active sensor 
	boolean result;
	int i; //index
	double d; //distance

	result = FALSE;
	for(i = 0; i < sensor_number; i++)
	{
		if(sensor_list[i].state == SENSOR_SENSE)
		{
			d = sqrt(pow((x - sensor_list[i].pos.x), 2) + pow((y - sensor_list[i].pos.y), 2));
			if(d <= sensor_list[i].sensing_range)
			{
				result = TRUE;
				break;
			}
		}
	}

	return result;
}

double estimate_energy_consumption(double time, double sensing_range, double energy_consumption_rate)
{ //estimate the amount of energy consumption to consider sensor's working time
	double energy_consumption;
	
	/* Energy consumption model for sensing:
	   P(r) = a*r^n where energy consumption rate a, sensing radius r, and power exponent n.
	   E(t,r) = P(r)*t where sensing duration t. */

	/* E = P*t where power P = V*C and duration t */

	energy_consumption = time*pow(sensing_range, 2)*energy_consumption_rate; //sensing area is square
	//energy_consumption = time*pow(sensing_range, 3)*energy_consumption_rate; //sensing area is cubic

	return energy_consumption;
}

double estimate_working_time(double energy, double sensing_range, double energy_consumption_rate)
{ //estimate the available working time to consider the residual energy to schedule sensing
	double time;
	
	time = energy/(pow(sensing_range, 2)*energy_consumption_rate); //sensing area is square
	//time = energy/(pow(sensing_range, 3)*energy_consumption_rate); //sensing area is cubic

	return time;
}

void initialize_density(struct struct_sensor* sensor_list, int sensor_number, struct parameter* param)
{ //initialize sensing density with number of one-hop neighbors
	int i, j; //index
	double d; //distance

	for(i = 0; i < sensor_number; i++) //for-1
	{
		sensor_list[i].sen_density = 0; //initialize sensing density with 0
		sensor_list[i].com_density = 0; //initialize communication density with 0
		for(j = 0; j < sensor_number; j++) //for-2
		{
			if(j == i) //skip the same sensor
				continue;

			d = sqrt(pow((sensor_list[j].pos.x - sensor_list[i].pos.x), 2) + pow((sensor_list[j].pos.y - sensor_list[i].pos.y), 2));
			if(d <= param->sensor_sensing_range*2)
				sensor_list[i].com_density++;	

			if(d <= param->sensor_sensing_range)
				sensor_list[i].sen_density++;	
		} //end of for-2
	} //end of for-1
}
 
void reduce_density(struct struct_sensor* sensor, struct struct_sensor* sensor_list, int sensor_number, struct parameter* param)
{ //reduce sensing & communication density of neighbors of the dead sensor by 1
	int i; //index
	double d; //distance

	for(i = 0; i < sensor_number; i++) //for-1
	{
		if(sensor->id == sensor_list[i].id)
			continue;

		d = sqrt(pow((sensor->pos.x - sensor_list[i].pos.x), 2) + pow((sensor->pos.y - sensor_list[i].pos.y), 2));
		if(d <= param->sensor_sensing_range*2)
			sensor_list[i].com_density--;	

		if(d <= param->sensor_sensing_range)
			sensor_list[i].sen_density--;	
	}
}

double adjust_sensing_range(double com_density, double sen_density, struct parameter* param)
{ //adjust a sensor's sensing range with its sensing density
	double range; //sensing range

	/* reflect communication density */
	range = param->sensor_sensing_range/(log10(com_density + 1) + 1);
	//range = param->sensor_sensing_range/sqrt(density + 1);
	//range = param->sensor_sensing_range/(sqrt(density) + 1);
	//range = param->sensor_sensing_range/(density + 1);

	/* reflect sensing density */
	range = range/(log10(sen_density + 1) + 1);

	return range;
}

void log_sensornet(struct struct_sensor* sensor_list, int sensor_number, double time, struct parameter* param)
{ //log sensor network with sensing ranges
	FILE* fp;
	char filename[BUF_SIZE];
	//char buf[BUF_SIZE];
	int i; //index


	/*
	strcpy(filename, "sensornet_");
	sprintf(buf, "%f", time);
	strcat(filename, buf);
	strcat(filename, ".dat");
	*/
	strcpy(filename, "sensornet.dat");

	/* open log file */
	fp = fopen(filename, "w");
	if(!fp)
	{
		fprintf(stderr, "Error: unable to open file \"%s\"\n", filename);
		exit(1);
	}

	for(i = 0; i < sensor_number; i++)
	{
		/* adjust a sensor's sensing range with its sensing density */
		//sensor_list[i].sensing_range = adjust_sensing_range(sensor_list[i].com_density, sensor_list[i].sen_density, param);
		fprintf(fp, "%f %f %f\n", (float) sensor_list[i].pos.x, (float) sensor_list[i].pos.y, (float) sensor_list[i].sensing_range);
	}

	fclose(fp);
}


int count_neighbor_sensors(double x, double y, struct struct_sensor* sensor_list, int sensor_number, double range)
{ //count the number of neighbor sensors for the vehicle where range is either sensign range or communication range.
 
  int counter = 0;
  double dist; //Manhanttan distance or Euclidean distance
  int i;
  double xx, yy; //sensor's x and y coordinates

  for(i = 0; i < sensor_number; i++)
  {
    xx = sensor_list[i].pos.x;
    yy = sensor_list[i].pos.y;
    dist = sqrt(pow(xx-x,2) + pow(yy-y,2));
    /* check if the neighbor sensor is located within sensing range from the vehicle */
    if(dist <= range)
      counter++;
  }//end of for

  return counter;
}

double km_hour2m_sec(double speed)
{ //convert speed in [km/hour] into speed in [m/sec]
	double converted_speed;

	converted_speed = speed * 1000/3600;
	return converted_speed;
}

int update_live_sensor_list(struct struct_sensor *sensor, int *S_ID, double *S_X, double *S_Y, int *p_live_sensor_number)
{ //update live-sensor-list S_ID, S_X, and S_Y with dying sensor
	int i; //loop index
	int result = 1;
	
	for(i = 0; i < *p_live_sensor_number; i++)
	{
		if(S_ID[i] == sensor->id)
		{
			S_ID[i] = S_ID[*p_live_sensor_number - 1];
			S_X[i] = S_X[*p_live_sensor_number - 1];
			S_Y[i] = S_Y[*p_live_sensor_number - 1];
			(*p_live_sensor_number)--;
			return 0;
		}
	}

	return 1;
}

void assert_memory(void *memory)
{ //assert the memory allocation
  //assert(memory != NULL);
  double current_time = 0; //current time in simulation
  
	if(memory == NULL)
	{
	        current_time = smpl_time();

		printf("[time=%.2f] assert_memory(): allocated memory is NULL\n", current_time);
		exit(1);
	}	
}

void open_localization_file(char *filename)
{ //open a file pointer for localization log file 
	char filename_buf[BUF_SIZE]; //log file for localization

	if(filename == NULL)
	{
		filename = filename_buf;
		strcpy(filename, "result\\localization");
		strcat(filename, ".txt");
	}

	/* open log file */
	fp_localization = fopen(filename, "w");
	if(!fp_localization)
	{
		fprintf(stderr, "Error: unable to open file \"%s\"\n", filename);
		exit(1);
	}
}

int log_localization(char *sensor_id, double current_time, int vehicle_id, int event, struct struct_sensor *sensor_list)
{ //log vehicle detections on the linear sensor network
	static int seq = 0; //sequence number
	int id;
	double timestamp;
	int increment; //number of new logged entries: 0: detection error, 1: normal detection, and 2: duplicate detection
	double detection_missing;
    double duplicate_detection;

	id = atoi(sensor_id);
	timestamp = current_time + sensor_list[id-1].time_sync_error;

	if(event == VEHICLE_CHECK && (sensor_list[id-1].detection_missing_probability > 0)) //if-1: determine whether it registers vehicle detection timestamp(s) into the log file along with duplicate detection or not
	{
		detection_missing = uniform(0.0, 1.0);
		if(detection_missing < sensor_list[id-1].detection_missing_probability) //if-2: detection missing
		{
			increment = 0;
#ifdef __DEBUG__
			printf("DETECTION MISSING: %s %f %d %d\n", sensor_id, (float) timestamp, event, vehicle_id);
#endif
		} //end of if-2
		else if(sensor_list[id-1].duplicate_detection_probability > 0) //else-if-2: determine whether it registers duplicate vehicle detection timestamps into the log file or not
		{
			duplicate_detection = uniform(0.0, 1.0);
			if(duplicate_detection < sensor_list[id-1].duplicate_detection_probability) //if-3: duplicate detection
			{
				increment = 2;
				fprintf(fp_localization, "%s %f %d %d\n", sensor_id, (float) timestamp, event, vehicle_id);
				fprintf(fp_localization, "%s %f %d %d\n", sensor_id, (float) (timestamp + DUPLICATE_DETECTION_INTERVAL), event, vehicle_id);

#ifdef __DEBUG__
				printf("DUPLICATE DETECTION 1: %s %f %d %d\n", sensor_id, (float) timestamp, event, vehicle_id);
				printf("DUPLICATE DETECTION 2: %s %f %d %d\n", sensor_id, (float) (timestamp + DUPLICATE_DETECTION_INTERVAL), event, vehicle_id);
#endif
			} //end of if-3
			else //else-3: normal detection
			{
				increment = 1;
				fprintf(fp_localization, "%s %f %d %d\n", sensor_id, (float) timestamp, event, vehicle_id);

#ifdef __DEBUG__
				printf("%s %f %d %d\n", sensor_id, (float) timestamp, event, vehicle_id);
#endif
			} //end of else-3
		} //end of else-if-2
		else //else-2: normal detection
		{
			increment = 1;
			fprintf(fp_localization, "%s %f %d %d\n", sensor_id, (float) timestamp, event, vehicle_id);

#ifdef __DEBUG__
			printf("%s %f %d %d\n", sensor_id, (float) timestamp, event, vehicle_id);
#endif
		} //end of else-2
	} //end of if-1
	else if(event == VEHICLE_CHECK && (sensor_list[id-1].duplicate_detection_probability > 0)) //else-if-1: determine whether it registers vehicle detection timestamp(s) into the log file or not
	{
		duplicate_detection = uniform(0.0, 1.0);
		if(duplicate_detection < sensor_list[id-1].duplicate_detection_probability) //if-4: duplicate detection
		{
			increment = 2;
			fprintf(fp_localization, "%s %f %d %d\n", sensor_id, (float) timestamp, event, vehicle_id);
			fprintf(fp_localization, "%s %f %d %d\n", sensor_id, (float) (timestamp + DUPLICATE_DETECTION_INTERVAL), event, vehicle_id);

#ifdef __DEBUG__
			printf("DUPLICATE DETECTION 1: %s %f %d %d\n", sensor_id, (float) timestamp, event, vehicle_id);
			printf("DUPLICATE DETECTION 2: %s %f %d %d\n", sensor_id, (float) (timestamp + DUPLICATE_DETECTION_INTERVAL), event, vehicle_id);
#endif
		} //end of if-4
		else //else-4: normal detection
		{
			increment = 1;
			fprintf(fp_localization, "%s %f %d %d\n", sensor_id, (float) timestamp, event, vehicle_id);

#ifdef __DEBUG__
			printf("%s %f %d %d\n", sensor_id, (float) timestamp, event, vehicle_id);
#endif
		} //end of else-4
	} //end of else-if-1
	else //else-1: logging other than vehicle detection at the state VEHICLE_CHECK
	{
		increment = 1;
		fprintf(fp_localization, "%s %f %d %d\n", sensor_id, (float) timestamp, event, vehicle_id);

#ifdef __DEBUG__
		printf("%s %f %d %d\n", sensor_id, (float) timestamp, event, vehicle_id);
#endif
	} //end of else-1

	//return ++seq;
	return increment;
}

void close_localization_file()
{ //close localization log file
	fclose(fp_localization);
}

void open_path_list_file(char *filename)
{ //open a file pointer for path-list log file 
	char filename_buf[BUF_SIZE]; //log file for path lists
 
	if(filename == NULL)
	{
		filename = filename_buf;
		strcpy(filename, "result\\path-list");
		strcat(filename, ".txt");
	}

    /* open log file */
    fp_path_list = fopen(filename, "w");
    if(!fp_path_list)
    {
       fprintf(stderr, "Error: unable to open file \"%s\"\n", filename);
       exit(1);
    }
}

int log_path_list(char* src, char* dst, double current_time, int vehicle_id, struct_path_node* path_list)
{ //log the path list
	struct_path_node* node = NULL; //node pointer
	//static int seq = 0; //sequence number
	//char msg[MSG_BUF_SIZE]; //message buffer
	char *msg = NULL; //pointer message buffer
	const int number_string_length = NUMBER_STRING_LEN; //length of number string
	int msg_len; //msg length

	msg_len = sizeof(char)*((int)path_list->weight)*number_string_length;
	msg = (char*) calloc(msg_len, sizeof(char));
	assert_memory(msg);

	sprintf(msg, "%f %d (%s=>%s): ", (float) current_time, vehicle_id, src, dst);

	/* get the path list */
	node = path_list->next;
	do
	{
		if(strcmp(node->vertex, dst) != 0)
		{
			strcat(msg, node->vertex);
			strcat(msg, "->");
		}
		else
		{
			strcat(msg, dst);
			break;
		}

		node = node->next;
	} while(1);

	fprintf(fp_path_list, "%s\n", msg);

#ifdef __DEBUG__
	printf("%s\n", msg);
#endif

	free(msg); //release the dynamic memory allocated to msg

	return 0;
	//return ++seq;
}

void close_path_list_file()
{ //close path_list log file
	fclose(fp_path_list);
}

float mean_integer(int i, int j)
{ //return the mean between integers i and j
	float result;
	int n;
	
	if(i > j)
	{
		printf("Error: mean(): i(=%d) > j(=%d)\n", i, j);
		exit(1);
	}
	
	n = j - i + 1;
	result = (float)sum_integer(i, j)/n;

	return result; 
}

int sum_integer(int i, int j)
{ //return the sum of integers between i and j
	int result = 0;
	int n;
	int k;
	int sum1, sum2;

	if(i > j)
	{
		printf("Error: mean(): i(=%d) > j(=%d)\n", i, j);
		exit(1);
	}
	else if(i > 0)
	{
		sum1 = j*(j+1)/2;
		sum2 = i*(i+1)/2;
		result = sum1 - sum2 + i;
	}
	else
	{
		n = j - i + 1;
		for(k = 0; k < n; k++)
		{
			result += i+k;
		}
	}

	return result;
}

int sum_vector(int *V, int i, int j)
{ //return the sum of vector values between indices i and j
	int result = 0;
	int n;
	int k;

	if(i > j)
	{
		printf("Error: mean(): i(=%d) > j(=%d)\n", i, j);
		exit(1);
	}

	n = j - i + 1;
	for(k = 0; k < n; k++)
	{
		result += V[i+k];
	}

	return result;
}

/** functions for road surveillance **/
void open_surveillance_file(char *filename)
{ //open a file pointer for surveillance log file 
	char filename_buf[BUF_SIZE]; //log file for surveillance

	if(filename == NULL)
	{
		filename = filename_buf;
#ifdef _LINUX_
		strcpy(filename, "result/surveillance");
#else
		strcpy(filename, "result\\surveillance");
#endif
		strcat(filename, ".txt");
	}

	/* open log file */
	fp_surveillance = fopen(filename, "w");
	if(!fp_surveillance)
	{
		fprintf(stderr, "Error: unable to open file \"%s\"\n", filename);
		exit(1);
	}
}

int log_surveillance_for_sensor(int sensor_id, double current_time, STATE event, struct_sensor_table *S)
{ //log the status of sensor on the road network
	int id  = sensor_id; //ID of the sensor detecting a vehicle of vehicle ID
	double timestamp; //timestamp indicating the sensor's system time
	double energy_budget = S->list[id-1]->info.energy; //remaining energy budget
	int increment = 1; //number of new logged entries

	timestamp = current_time + S->list[id-1]->info.time_sync_error;

	fprintf(fp_surveillance, "%f %d %d %f %f\n", (float) current_time, event, sensor_id, (float) timestamp, (float) energy_budget);

#ifdef __DEBUG__
	printf("%f %d %d %f %f\n", (float) current_time, event, sensor_id, (float) timestamp, (float) energy_budget);
#endif

	return increment;
}

int log_surveillance_for_vehicle(int vehicle_id, double current_time, STATE event, struct_coordinate2_t *pos, MOVE_TYPE move_type, edge_queue_t *E)
{ //log the status of vehicle on the road network
	int id  = vehicle_id; //vehicle ID
	int eid = pos->eid;
	double offset = pos->offset;
	edge_queue_node_t *pEdgeNode = NULL;
	char *tail_node = NULL;
	char *head_node = NULL;
	int increment = 1; //number of new logged entries

	pEdgeNode = GetEdgeNodeByEID(E, eid);
	tail_node = pEdgeNode->tail_node;
	head_node = pEdgeNode->head_node;


	fprintf(fp_surveillance, "%f %d %d %d %s %s %f %d\n", (float) current_time, event, vehicle_id, eid, tail_node, head_node, (float) offset, move_type);

#ifdef __DEBUG__
	printf("%f %d %d %d %s %s %f %d\n", (float) current_time, event, vehicle_id, eid, tail_node, head_node, (float) offset, move_type);
#endif

	return increment;
}

int log_surveillance_for_detection(int sensor_id, double current_time, int vehicle_id, double movement_time, STATE event, struct_sensor_table *S)
{ //log vehicle detections for surveillance on the road network
	//static int seq = 0; //sequence number
	int id  = sensor_id; //ID of the sensor detecting a vehicle of vehicle ID
	double timestamp; //timestamp indicating the sensor's system time
	int increment; //number of new logged entries: 0: detection error, 1: normal detection, and 2: duplicate detection
	double detection_missing;
    double duplicate_detection;

	timestamp = current_time + S->list[id-1]->info.time_sync_error;

	if(event == VEHICLE_CHECK && (S->list[id-1]->info.detection_missing_probability > 0)) //if-1: determine whether it registers vehicle detection timestamp(s) into the log file along with duplicate detection or not
	{
		detection_missing = uniform(0.0, 1.0);
		if(detection_missing < S->list[id-1]->info.detection_missing_probability) //if-2: detection missing
		{
			increment = 0;
#ifdef __DEBUG__
			printf("DETECTION MISSING: %f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) timestamp, vehicle_id, (float) movement_time);
#endif
		} //end of if-2
		else if(S->list[id-1]->info.duplicate_detection_probability > 0) //else-if-2: determine whether it registers duplicate vehicle detection timestamps into the log file or not
		{
			duplicate_detection = uniform(0.0, 1.0);
			if(duplicate_detection < S->list[id-1]->info.duplicate_detection_probability) //if-3: duplicate detection
			{
				increment = 2;
				fprintf(fp_surveillance, "%f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) timestamp, vehicle_id, (float) movement_time);
				fprintf(fp_surveillance, "%f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) (timestamp + DUPLICATE_DETECTION_INTERVAL), vehicle_id, (float) movement_time);

#ifdef __DEBUG__
				printf("DUPLICATE DETECTION 1: %f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) timestamp, vehicle_id, (float) movement_time);
				printf("DUPLICATE DETECTION 2: %f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) (timestamp + DUPLICATE_DETECTION_INTERVAL), vehicle_id, (float) movement_time);
#endif
			} //end of if-3
			else //else-3: normal detection
			{
				increment = 1;
				fprintf(fp_surveillance, "%f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) timestamp, vehicle_id, (float) movement_time);

#ifdef __DEBUG__
				printf("%f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) timestamp, vehicle_id, (float) movement_time);
#endif
			} //end of else-3
		} //end of else-if-2
		else //else-2: normal detection
		{
			increment = 1;
			fprintf(fp_surveillance, "%f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) timestamp, vehicle_id, (float) movement_time);

#ifdef __DEBUG__
			printf("%f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) timestamp, vehicle_id, (float) movement_time);
#endif
		} //end of else-2
	} //end of if-1
	else if(event == VEHICLE_CHECK && (S->list[id-1]->info.duplicate_detection_probability > 0)) //else-if-1: determine whether it registers vehicle detection timestamp(s) into the log file or not
	{
		duplicate_detection = uniform(0.0, 1.0);
		if(duplicate_detection < S->list[id-1]->info.duplicate_detection_probability) //if-4: duplicate detection
		{
			increment = 2;
			fprintf(fp_surveillance, "%f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) timestamp, vehicle_id, (float) movement_time);
			fprintf(fp_surveillance, "%f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) (timestamp + DUPLICATE_DETECTION_INTERVAL), vehicle_id, (float) movement_time);

#ifdef __DEBUG__
			printf("DUPLICATE DETECTION 1: %f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) timestamp, vehicle_id, (float) movement_time);
			printf("DUPLICATE DETECTION 2: %f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) (timestamp + DUPLICATE_DETECTION_INTERVAL), vehicle_id, (float) movement_time);
#endif
		} //end of if-4
		else //else-4: normal detection
		{
			increment = 1;
			fprintf(fp_surveillance, "%f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) timestamp, vehicle_id, (float) movement_time);

#ifdef __DEBUG__
			printf("%f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) timestamp, vehicle_id, (float) movement_time);
#endif
		} //end of else-4
	} //end of else-if-1
	else //else-1: logging other than vehicle detection at the state VEHICLE_CHECK
	{
		increment = 1;
		/*
		fprintf(fp_surveillance, "%f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) timestamp, vehicle_id, (float) movement_time);

#ifdef __DEBUG__
		printf("%f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) timestamp, vehicle_id, (float) movement_time);
#endif
		*/

		/*@ log movement times */
		fprintf(fp_surveillance, "%f\n", (float) movement_time);

#ifdef __DEBUG__
		printf("%f %d %d %f %d %f\n", (float) current_time, event, sensor_id, (float) timestamp, vehicle_id, (float) movement_time);
#endif

	} //end of else-1

	//return ++seq;
	return increment;
}

void close_surveillance_file()
{ //close surveillance log file
	fclose(fp_surveillance);
}

/** functions for VANET data forwarding **/
void open_vanet_file(char *filename)
{ //open a file pointer for VANET log file
	char filename_buf[BUF_SIZE]; //log file for VANET

	if(filename == NULL)
	{
		filename = filename_buf;
#ifdef _LINUX_
		strcpy(filename, "output/output");
#else
		strcpy(filename, "output\\output");
#endif
		strcat(filename, ".log");
	}

	/* open log file */
	fp_vanet = fopen(filename, "w");
	if(!fp_vanet)
	{
		fprintf(stderr, "Error: unable to open file \"%s\"\n", filename);
		exit(1);
	}
}

int log_vanet(VANET_LOG_TYPE type, double current_time, packet_queue_node_t *p, packet_delivery_statistics_t *packet_delivery_stat)
{ //log a VANET event into VANET logging file
	int i = 0; //index for for-loop
	int increment = 1; //number of new logged entries
	global_packet_queue_t *Q = p->global_packet->ptr_queue; //pointer to the global packet queue
	int index = 0; //index for packet id queue
	boolean packet_receive_flag = FALSE; //flag to check whether a copy of the packet pointed by p has already been received by the destination vehicle or not
      
	/** @for debugging */
	//if(current_time > 3658)
	//  printf("log_vanet(): current_time=%f\n", (float)current_time);
	/*******************/

	switch(type) //switch-1
	{
	case VANET_LOG_PACKET_AP_ARRIVAL: //log the data forwarding to Internet access point on VANET into VANET logging file
	  p->actual_delivery_delay = p->destination_arrival_time - p->generation_time; //compute packet delivery delay
          printf("delivery_delay %.1f = arrival_time %.1f - generation_time %.1f\n", p->actual_delivery_delay,p->destination_arrival_time,p->generation_time);
	  p->delivery_delay_difference = p->actual_delivery_delay - p->expected_delivery_delay; 
	
#ifdef __LOG_LEVEL_VANET_PACKET_AP_ARRIVAL__
	  /* make a forwarding log into VANET logging file */
	  fprintf(fp_vanet, "%d\t%.2f\t%d\t%d\t%d\t%d\t%d\t%d\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%d\n", type, (float)current_time, p->src_id, p->carry_src_id, p->carry_dst_id, p->dst_id, p->actual_dst_id, p->seq, (float)p->generation_time, (float)p->last_receive_time, (float)p->expected_delivery_delay, (float)p->actual_delivery_delay, (float)p->expected_delivery_delay_standard_deviation, (float)p->delivery_delay_difference, (float)p->ttl, p->size);
#endif

#if defined(__DEBUG_LEVEL_VANET_PACKET_AP_ARRIVAL__) || defined(__DEBUG_PACKET_TRACE__)
	  printf("type=%d, current_time=%.2f, src_id=%d, carry_src_id=%d, carry_dst_id=%d, p->dst_id=%d, p->actual_dst_id=%d, seq=%d, generation_time=%.2f, last_receive_time=%.2f, expected_delivery_delay=%.2f, actual_delivery_delay=%.2f, expected_delivery_delay_standard_deviation=%.2f, delivery_delay_difference=%.2f, ttl=%.2f, size=%d\n\n", type, (float)current_time, p->src_id, p->carry_src_id, p->carry_dst_id, p->dst_id, p->actual_dst_id, p->seq, (float)p->generation_time, (float)p->last_receive_time, (float)p->expected_delivery_delay, (float)p->actual_delivery_delay, (float)p->expected_delivery_delay_standard_deviation, (float)p->delivery_delay_difference, (float)p->ttl, p->size);
#endif

	  /* update packet delivery statistics */
	  packet_delivery_stat->delivered_packet_number++;
          // Taehwan 20140610
	  //printf("########### arrived at AP on %lld\n",p->destination_arrival_time);
	  packet_delivery_stat->expected_delivery_delay_sum += p->expected_delivery_delay;
	  packet_delivery_stat->actual_delivery_delay_sum += p->actual_delivery_delay;
	  packet_delivery_stat->ap_arrival_delay_sum += p->actual_delivery_delay;
	  packet_delivery_stat->expected_delivery_delay_standard_deviation_sum += p->expected_delivery_delay_standard_deviation;
	  packet_delivery_stat->delivery_delay_difference_sum += p->delivery_delay_difference;

	  break;

	case VANET_LOG_PACKET_DESTINATION_VEHICLE_ARRIVAL: //log the data forwarding to a destination vehicle on VANET into VANET logging file
	  /** check whether a copy of the packet pointed by p */
	  if(Q == NULL)
	  {
		printf("log_vanet(): VANET_LOG_PACKET_DESTINATION_VEHICLE_ARRIVAL: global_packet_queue is NULL!\n");
		exit(1);
	  }

	  index = p->id; //index for packet_id_vector
	
	  /* check whether packet id vector needs to be expanded or not */
	  if(index >= Q->physical_vector_size)
	  {
		Reallocate_GlobalPacketQueue_PacketVectors(Q, PACKET_VECTOR_INCREASE_SIZE);
		//increase the memory of packet vectors by vector_increase_size to accommodate more packets
	  }

	  /* check whether the packet corresponding to index is already received or not */
	  if(Q->packet_id_bitmap_vector[index] == FALSE)
	  {
	    /* register the packet delivery performance */
	    p->actual_delivery_delay = p->destination_arrival_time - p->generation_time; //compute packet delivery delay
	    printf("delivery_delay %.1f = arrival_time %.1f - generation_time %.1f\n", p->actual_delivery_delay,p->destination_arrival_time,p->generation_time);
	    p->delivery_delay_difference = p->actual_delivery_delay - p->expected_delivery_delay;

		/* register the packet delivery event into packet vectors */
		Q->packet_id_bitmap_vector[index] = TRUE;
		Q->packet_delivery_delay_vector[index] = p->actual_delivery_delay;

		/* increment actual_vector_size */
		Q->actual_vector_size++;
	  }
	  else
	  {
		return 0;	
	  }
	
#ifdef __LOG_LEVEL_VANET_PACKET_DESTINATION_VEHICLE_ARRIVAL__
	  /* make a forwarding log into VANET logging file */
	  fprintf(fp_vanet, "%d\t%.2f\t%d\t%d\t%d\t%d\t%d\t%d\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%u\t%u\n", type, (float)current_time, p->src_id, p->carry_src_id, p->carry_dst_id, p->src_id, p->dst_id, p->seq, (float)p->generation_time, (float)p->last_receive_time, (float)p->expected_delivery_delay, (float)p->actual_delivery_delay, (float)p->expected_delivery_delay_standard_deviation, (float)p->delivery_delay_difference, (float)p->ttl, p->expected_packet_transmission_number, p->packet_transmission_count);
#endif

#if defined(__DEBUG_LEVEL_VANET_PACKET_DESTINATION_VEHICLE_ARRIVAL__) || defined(__DEBUG_PACKET_TRACE__)
	  printf("type=%d, current_time=%.2f, src_id=%d, carry_src_id=%d, carry_dst_id=%d, p->dst_id=%d, p->actual_dst_id=%d, seq=%d, generation_time=%.2f, last_receive_time=%.2f, expected_delivery_delay=%.2f, actual_delivery_delay=%.2f, expected_delivery_delay_standard_deviation=%.2f, delivery_delay_difference=%.2f, ttl=%.2f, expected_packet_transmission_number, packet_transmission_count=%u\n\n", type, (float)current_time, p->src_id, p->carry_src_id, p->carry_dst_id, p->src_id, p->dst_id, p->seq, (float)p->generation_time, (float)p->last_receive_time, (float)p->expected_delivery_delay, (float)p->actual_delivery_delay, (float)p->expected_delivery_delay_standard_deviation, (float)p->delivery_delay_difference, (float)p->ttl, p->expected_packet_transmission_number, p->packet_transmission_count);
#endif

	  /* update packet delivery statistics */
	  packet_delivery_stat->delivered_packet_number++;
          // Taehwan 20140610
	  //printf("########### arrived at VEHICLE on %ld\n",p->destination_arrival_time);
      //    printf("########### delivery delay is %ld\n",p->expected_delivery_delay);
	  packet_delivery_stat->expected_delivery_delay_sum += p->expected_delivery_delay;
	  packet_delivery_stat->actual_delivery_delay_sum += p->actual_delivery_delay;
          packet_delivery_stat->destination_vehicle_arrival_delay_sum += p->actual_delivery_delay;
	  packet_delivery_stat->expected_delivery_delay_standard_deviation_sum += p->expected_delivery_delay_standard_deviation;
	  packet_delivery_stat->delivery_delay_difference_sum += p->delivery_delay_difference;

          if (packet_delivery_stat->min_delivery_delay_difference == 0 || (int)p->delivery_delay_difference < packet_delivery_stat->min_delivery_delay_difference) 
		packet_delivery_stat->min_delivery_delay_difference = (int)p->delivery_delay_difference; 

	  packet_delivery_stat->expected_packet_transmission_number_sum += p->expected_packet_transmission_number;
          packet_delivery_stat->actual_packet_transmission_number_sum += p->packet_transmission_count;

	  break;

	case VANET_LOG_PACKET_DROP: //log the packet drop due to TTL expiration into VANET logging file

	  index = p->id; //index for packet_id_vector
	
	  /* check whether packet id vector needs to be expanded or not */
	  if(index >= Q->physical_vector_size)
	  {
		Reallocate_GlobalPacketQueue_PacketVectors(Q, PACKET_VECTOR_INCREASE_SIZE);
		//increase the memory of packet vectors by vector_increase_size to accommodate more packets
	  }

	  /* increase packet deletion count */
	  Q->packet_deletion_count_vector[index]++;

	  if(Q->packet_deletion_count_vector[index] < Q->packet_copy_number)
	  {
		return 0;
	  }
		 			
	  p->actual_lifetime = p->last_receive_time - p->generation_time; //compute the packet's lifetime
	  p->delivery_delay_difference = p->actual_lifetime - p->expected_delivery_delay; 

#ifdef __LOG_LEVEL_VANET_PACKET_DROP__
	  /* make a packet drop log into VANET logging file */
	  fprintf(fp_vanet, "%d\t%.2f\t%d\t%d\t%d\t%d\t%d\t%d\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%u\t%u\n", type, (float)current_time, p->src_id, p->carry_src_id, p->carry_dst_id, p->dst_id, p->actual_dst_id, p->seq, (float)p->generation_time, (float)p->last_receive_time, (float)p->expected_delivery_delay, (float)p->actual_lifetime, (float)p->expected_delivery_delay_standard_deviation, (float)p->delivery_delay_difference, (float)p->ttl, p->expected_packet_transmission_number, p->packet_transmission_count);
#endif

#if defined(__DEBUG_LEVEL_VANET_PACKET_DROP__) || defined(__DEBUG_PACKET_TRACE__)
	  printf("type=%d, current_time=%.2f, src_id=%d, carry_src_id=%d, carry_dst_id=%d, p->dst_id=%d, p->actual_dst_id=%d, seq=%d, generation_time=%.2f, last_receive_time=%.2f, expected_delivery_delay=%.2f, actual_lifetime=%.2f, expected_delivery_delay_standard_deviation=%.2f, delivery_delay_difference=%.2f, ttl=%.2f, expected_transmission_number=%u, packet_transmission_count=%u\n\n", type, (float)current_time, p->src_id, p->carry_src_id, p->carry_dst_id, p->dst_id, p->actual_dst_id, p->seq, (float)p->generation_time, (float)p->last_receive_time, (float)p->expected_delivery_delay, (float)p->actual_lifetime, (float)p->expected_delivery_delay_standard_deviation, (float)p->delivery_delay_difference, (float)p->ttl, p->expected_packet_transmission_number, p->packet_transmission_count);
#endif

	  /* update packet delivery statistics */
	  packet_delivery_stat->discarded_packet_number++;
	  packet_delivery_stat->expected_packet_transmission_number_sum += p->expected_packet_transmission_number;
	  packet_delivery_stat->actual_packet_transmission_number_sum += p->packet_transmission_count;

          break;

        default:
	  printf("log_vanet(): Error: type(%d) is not supported yet!", type);
	  exit(1);
	} //end of switch-1

	return increment;
}

void close_vanet_file()
{ //close VANET log file
	fclose(fp_vanet);
}

/** functions for VANET packet carrier trace **/
void open_vanet_packet_carrier_trace_file(char *filename)
{ //open a file pointer for VANET packet carrier trace log file
  char filename_buf[BUF_SIZE]; //log file for VANET

  if(filename == NULL)
  {
    filename = filename_buf;
#ifdef _LINUX_
    strcpy(filename, "output/output");
#else
    strcpy(filename, "output\\output");
#endif
    strcat(filename, ".pct");
  }

  /* open log file */
  fp_vanet_packet_carrier_trace = fopen(filename, "w");
  if(!fp_vanet_packet_carrier_trace)
  {
    fprintf(stderr, "Error: unable to open file \"%s\"\n", filename);
    exit(1);
  }
}


int log_vanet_packet_carrier_trace(VANET_LOG_TYPE log_type, double current_time, packet_queue_node_t *packet)
{ //log a VANET event into VANET logging file according to data forwarding mode
  int increment = 1; //number of new logged entries

  if(packet->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
  {
    increment = log_vanet_packet_carrier_trace_for_upload(log_type, current_time, packet);
  }
  else if(packet->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
  {
    increment = log_vanet_packet_carrier_trace_for_download(log_type, current_time, packet);
  }
  else if(packet->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
  {
    increment = log_vanet_packet_carrier_trace_for_v2v(log_type, current_time, packet);
  }
  else
  {
    printf("log_vanet_packet_carrier_trace(): packet->data_forwarding_mode(%d) is not supported!\n", packet->data_forwarding_mode);
    exit(1);
  }

  return increment;
}

int log_vanet_packet_carrier_trace_for_upload(VANET_LOG_TYPE log_type, double current_time, packet_queue_node_t *packet)
{ //log a VANET event into VANET logging file for upload forwarding mode 
  int increment = 1; //number of new logged entries
  carrier_trace_queue_node_t *pQueueNode = NULL; //pointer to a carrier trace queue node
  int size = packet->carrier_trace.size;
  int i = 0; //index for for-loop

  pQueueNode = &(packet->carrier_trace.head);
  for(i = 0; i < size; i++)
  {
    pQueueNode = pQueueNode->next;

    /* log the packet carrier trace entry into the trace log file */
    /* format: (log_type, packet_source_id, packet_destination_id, carrier_id, node_type, receive_time, tail_node, head_node, offset, x-coordinate, y-coordinate) */
    if(pQueueNode->node_type == VANET_NODE_AP || pQueueNode->node_type == VANET_NODE_VEHICLE || pQueueNode->node_type == VANET_NODE_SNODE)
    {
      if(pQueueNode->node_type == VANET_NODE_SNODE || pQueueNode->node_type == VANET_NODE_AP)
      {
        //fprintf(fp_vanet_packet_carrier_trace, "%d %d %d %d %d %d %d %d %d %.2f %.2f %.2f %d %d %.2f %.2f %.2f\n", packet->id, log_type, packet->src_id, pQueueNode->carry_src_id, pQueueNode->carry_dst_id, packet->dst_id, packet->target_point_id, pQueueNode->target_point_id, pQueueNode->node_type, pQueueNode->EDD, pQueueNode->EDD_SD, pQueueNode->receive_time, packet->target_point_id, packet->target_point_id, 0, pQueueNode->euclidean_pos.x, pQueueNode->euclidean_pos.y);
        fprintf(fp_vanet_packet_carrier_trace, "pid=%d: log_type=%d, [%d=>(%d=>%d)=>%d: packet tp=%d, trace tp=%d], %d, <EDD=%.2f, EDD_SD=%.2f>, receive_time=%.2f, (%d->%d:%.2f), (%.2f,%.2f)\n", packet->id, log_type, packet->src_id, pQueueNode->carry_src_id, pQueueNode->carry_dst_id, packet->dst_id, packet->target_point_id,  pQueueNode->target_point_id, pQueueNode->node_type, pQueueNode->EDD, pQueueNode->EDD_SD, pQueueNode->receive_time, pQueueNode->carry_dst_id, pQueueNode->carry_dst_id, 0.0, pQueueNode->euclidean_pos.x, pQueueNode->euclidean_pos.y);
      }
      else
      {
        //fprintf(fp_vanet_packet_carrier_trace, "%d %d %d %d %d %d %d %d %d %.2f %.2f %.2f %s %s %.2f %.2f %.2f\n", packet->id, log_type, packet->src_id, pQueueNode->carry_src_id, pQueueNode->carry_dst_id, packet->dst_id, packet->target_point_id, pQueueNode->target_point_id, pQueueNode->node_type, pQueueNode->EDD, pQueueNode->EDD_SD, pQueueNode->receive_time, pQueueNode->graph_pos.enode->tail_node, pQueueNode->graph_pos.enode->head_node, pQueueNode->graph_pos.offset, pQueueNode->euclidean_pos.x, pQueueNode->euclidean_pos.y);
        fprintf(fp_vanet_packet_carrier_trace, "pid=%d: log_type=%d, [%d=>(%d=>%d)=>%d: packet tp=%d, trace tp=%d], %d, <EDD=%.2f, EDD_SD=%.2f>, receive_time=%.2f, (%s->%s:%.2f), (%.2f,%.2f)\n", packet->id, log_type, packet->src_id, pQueueNode->carry_src_id, pQueueNode->carry_dst_id, packet->dst_id, packet->target_point_id,  pQueueNode->target_point_id, pQueueNode->node_type, pQueueNode->EDD, pQueueNode->EDD_SD, pQueueNode->receive_time, pQueueNode->graph_pos.enode->tail_node, pQueueNode->graph_pos.enode->head_node, pQueueNode->graph_pos.offset, pQueueNode->euclidean_pos.x, pQueueNode->euclidean_pos.y);
      }
    }
    else
    {
      printf("log_vanet_packet_carrier_trace_for_upload(): node_type(%d) is not supported yet!\n", pQueueNode->node_type);
      exit(1);
    }
  }
  
  return increment;
}

int log_vanet_packet_carrier_trace_for_download(VANET_LOG_TYPE log_type, double current_time, packet_queue_node_t *packet)
{ //log a VANET event into VANET logging file for download forwarding mode 
  int increment = 1; //number of new logged entries
  carrier_trace_queue_node_t *pQueueNode = NULL; //pointer to a carrier trace queue node
  int size = packet->carrier_trace.size;
  int i = 0; //index for for-loop

  pQueueNode = &(packet->carrier_trace.head);
  for(i = 0; i < size; i++) //for
  {
    pQueueNode = pQueueNode->next;

    /* log the packet carrier trace entry into the trace log file */
    /* format: (log_type, packet_source_id, packet_destination_id, carrier_id, node_type, receive_time, tail_node, head_node, offset, x-coordinate, y-coordinate) */
    if(pQueueNode->node_type == VANET_NODE_AP || pQueueNode->node_type == VANET_NODE_VEHICLE || pQueueNode->node_type == VANET_NODE_SNODE) //if-1
    {
      fprintf(fp_vanet_packet_carrier_trace, "pid=%d: log_type=%d, [%d=>(%d=>%d)=>%d: packet tp=%d, trace tp=%d], %d, <EDD=%.2f, EDD_SD=%.2f>, receive_time=%.2f, (%s->%s:%.2f), (%.2f,%.2f) || dst_vehicle: (%s->%s:%.2f), (%.2f,%.2f)\n", packet->id, log_type, packet->src_id, pQueueNode->carry_src_id, pQueueNode->carry_dst_id, packet->dst_id, packet->target_point_id,  pQueueNode->target_point_id, pQueueNode->node_type, pQueueNode->EDD_for_download, pQueueNode->EDD_SD_for_download, pQueueNode->receive_time, pQueueNode->graph_pos.tail_node, pQueueNode->graph_pos.head_node, pQueueNode->graph_pos.offset, pQueueNode->euclidean_pos.x, pQueueNode->euclidean_pos.y, pQueueNode->dst_graph_pos.enode->tail_node, pQueueNode->dst_graph_pos.enode->head_node, pQueueNode->dst_graph_pos.offset, pQueueNode->dst_euclidean_pos.x, pQueueNode->dst_euclidean_pos.y);

#if defined(__DEBUG_LEVEL_VANET_PACKET_CARRIER_TRACE__) || defined(__DEBUG_LEVEL_VANET_PACKET_CARRIER_TRACE_FOR_STATIONARY_NODE__)
     printf("pid=%d: log_type=%d, [%d=>(%d=>%d)=>%d: packet tp=%d, trace tp=%d], %d, <EDD=%.2f, EDD_SD=%.2f>, receive_time=%.2f, (%s->%s:%.2f), (%.2f,%.2f) || dst_vehicle: (%s->%s:%.2f), (%.2f,%.2f)\n", packet->id, log_type, packet->src_id, pQueueNode->carry_src_id, pQueueNode->carry_dst_id, packet->dst_id, packet->target_point_id,  pQueueNode->target_point_id, pQueueNode->node_type, pQueueNode->EDD_for_download, pQueueNode->EDD_SD_for_download, pQueueNode->receive_time, pQueueNode->graph_pos.tail_node, pQueueNode->graph_pos.head_node, pQueueNode->graph_pos.offset, pQueueNode->euclidean_pos.x, pQueueNode->euclidean_pos.y, pQueueNode->dst_graph_pos.enode->tail_node, pQueueNode->dst_graph_pos.enode->head_node, pQueueNode->dst_graph_pos.offset, pQueueNode->dst_euclidean_pos.x, pQueueNode->dst_euclidean_pos.y);
#endif
    } //end of if-1
    else //else-2
    {
      printf("log_vanet_packet_carrier_trace_for_download(): node_type(%d) is not supported yet!\n", pQueueNode->node_type);
      exit(1);
    } //end of else-2
  } //end of for

  fprintf(fp_vanet_packet_carrier_trace, "\n");
  
  return increment;
}

int log_vanet_packet_carrier_trace_for_v2v(VANET_LOG_TYPE log_type, double current_time, packet_queue_node_t *packet)
{ //log a VANET event into VANET logging file for V2V forwarding mode 
  int increment = 1; //number of new logged entries
  carrier_trace_queue_node_t *pQueueNode = NULL; //pointer to a carrier trace queue node
  int size = packet->carrier_trace.size;
  int i = 0; //index for for-loop

  pQueueNode = &(packet->carrier_trace.head);
  for(i = 0; i < size; i++) //for
  {
    pQueueNode = pQueueNode->next;

    /* log the packet carrier trace entry into the trace log file */
    /* format: (log_type, packet_source_id, packet_destination_id, carrier_id, node_type, receive_time, tail_node, head_node, offset, x-coordinate, y-coordinate) */
    if(pQueueNode->node_type == VANET_NODE_AP || pQueueNode->node_type == VANET_NODE_VEHICLE || pQueueNode->node_type == VANET_NODE_SNODE) //if-1
    {
      fprintf(fp_vanet_packet_carrier_trace, "pid=%d: log_type=%d, [%d=>(%d=>%d)=>%d: packet tp=%d, trace tp=%d], %d, <EDD=%.2f, EDD_SD=%.2f>, receive_time=%.2f, (%s->%s:%.2f), (%.2f,%.2f) || dst_vehicle: (%s->%s:%.2f), (%.2f,%.2f)\n", packet->id, log_type, packet->src_id, pQueueNode->carry_src_id, pQueueNode->carry_dst_id, packet->dst_id, packet->target_point_id,  pQueueNode->target_point_id, pQueueNode->node_type, pQueueNode->EDD_for_download, pQueueNode->EDD_SD_for_download, pQueueNode->receive_time, pQueueNode->graph_pos.tail_node, pQueueNode->graph_pos.head_node, pQueueNode->graph_pos.offset, pQueueNode->euclidean_pos.x, pQueueNode->euclidean_pos.y, pQueueNode->dst_graph_pos.enode->tail_node, pQueueNode->dst_graph_pos.enode->head_node, pQueueNode->dst_graph_pos.offset, pQueueNode->dst_euclidean_pos.x, pQueueNode->dst_euclidean_pos.y);

#if defined(__DEBUG_LEVEL_VANET_PACKET_CARRIER_TRACE__) || defined(__DEBUG_LEVEL_VANET_PACKET_CARRIER_TRACE_FOR_STATIONARY_NODE__)
     printf("pid=%d: log_type=%d, [%d=>(%d=>%d)=>%d: packet tp=%d, trace tp=%d], %d, <EDD=%.2f, EDD_SD=%.2f>, receive_time=%.2f, (%s->%s:%.2f), (%.2f,%.2f) || dst_vehicle: (%s->%s:%.2f), (%.2f,%.2f)\n", packet->id, log_type, packet->src_id, pQueueNode->carry_src_id, pQueueNode->carry_dst_id, packet->dst_id, packet->target_point_id,  pQueueNode->target_point_id, pQueueNode->node_type, pQueueNode->EDD_for_download, pQueueNode->EDD_SD_for_download, pQueueNode->receive_time, pQueueNode->graph_pos.tail_node, pQueueNode->graph_pos.head_node, pQueueNode->graph_pos.offset, pQueueNode->euclidean_pos.x, pQueueNode->euclidean_pos.y, pQueueNode->dst_graph_pos.enode->tail_node, pQueueNode->dst_graph_pos.enode->head_node, pQueueNode->dst_graph_pos.offset, pQueueNode->dst_euclidean_pos.x, pQueueNode->dst_euclidean_pos.y);
#endif
    } //end of if-1
    else //else-2
    {
      printf("%s:%d: node_type(%d) is not supported yet!\n", 
			  __FUNCTION__, __LINE__, pQueueNode->node_type);
      exit(1);
    } //end of else-2
  } //end of for

  fprintf(fp_vanet_packet_carrier_trace, "\n");
  
  return increment;
}

void close_vanet_packet_carrier_trace_file()
{ //close VANET packet carrier trace log file
  fclose(fp_vanet_packet_carrier_trace);
}

/*******************************************************/
/** functions for trace for observing system behavior **/
FILE* open_trace_file(char *filename)
{ //open a file pointer for trace file
  FILE *fp = NULL; //file pointer 

  if(filename == NULL)
  {
    printf("open_trace_file(): filename is NULL\n");
    exit(1);
  }

  /* open log file */
  fp = fopen(filename, "w");
  if(!fp)
  {
    printf("open_trace_file(): Error: unable to open file \"%s\"\n", filename);
    exit(1);
  }

  return fp;
}

void log_trace_file(FILE *fp, TRACE_TYPE type, ...)
{ //write an event into the trace file corresponding to the trace file type
  va_list ap;
  double double_arg1 = 0, double_arg2 = 0, double_arg3 = 0, double_arg4 = 0, double_arg5 = 0, double_arg6 = 0;
  int int_arg1 = 0, int_arg2 = 0, int_arg3 = 0;

  va_start(ap, type); /* startup */

  switch(type)
  {
  case TRACE_SENSOR_NUMBER:
    double_arg1 = va_arg(ap, double); /* get the next argument: current time */
    int_arg1 = va_arg(ap, int); /* get the next argument: the number of living sensors */
    fprintf(fp, "%f\t%d\n", (float)double_arg1, int_arg1);
    break;

  case TRACE_HOLE_NUMBER:
    double_arg1 = va_arg(ap, double); /* get the next argument: current time */
    int_arg1 = va_arg(ap, int); /* get the next argument: the accumulated number of sensing holes */
    int_arg2 = va_arg(ap, int); /* get the next argument: the eid in Gr of the new hole */
    double_arg2 = va_arg(ap, int); /* get the next argument: the offset in Gr of the new hole */
    fprintf(fp, "%f\t%d\t%d\t%f\n", (float)double_arg1, int_arg1, int_arg2, (float)double_arg2);
    break;

  case TRACE_SLEEPING_TIME:
    double_arg1 = va_arg(ap, double); /* get the next argument: current time */
    double_arg2 = va_arg(ap, double); /* get the next argument: T_sleep */
    double_arg3 = va_arg(ap, double); /* get the next argument: T_move */
    double_arg4 = va_arg(ap, double); /* get the next argument: T_scan */
    fprintf(fp, "%f\t%f\t%f\t%f\n", (float)double_arg1, (float)double_arg2, (float)double_arg3, (float)double_arg4);
    break;

  case TRACE_VEHICLE_DETECTION_TIME:
    double_arg1 = va_arg(ap, double); /* get the next argument: current time */
    double_arg2 = va_arg(ap, double); /* get the next argument: detection time; movement time from the arrival time to the detected time */
    double_arg3 = va_arg(ap, double); /* get the next argument: arrival time */
    double_arg4 = va_arg(ap, double); /* get the next argument: detected time */
    fprintf(fp, "%f\t%f\t%f\t%f\n", (float)double_arg1, (float)double_arg2, (float)double_arg3, (float)double_arg4);
    break;

  case TRACE_VEHICLE_CONVOY_LENGTH:
    double_arg1 = va_arg(ap, double); /* get the next argument: logging time */     
    double_arg2 = va_arg(ap, double); /* get the next argument: convoy start time */
    double_arg3 = va_arg(ap, double); /* get the next argument: convoy start length */
    double_arg4 = va_arg(ap, double); /* get the next argument: convoy end time */
    double_arg5 = va_arg(ap, double); /* get the next argument: convoy end length */
    double_arg6 = va_arg(ap, double); /* get the next argument: vehicle speed */
    int_arg1 = va_arg(ap, int); /* get the next argument: vehicle_number */
 
    fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f\t%d\n", (float)double_arg1, (float)double_arg2, (float)double_arg3, (float)double_arg4, (float)double_arg5, (float)double_arg6, int_arg1);

    break;

  default:
    printf("log_trace_file(): type(%d) is not supported!\n", type);
    exit(1);
  } //end of switch

  va_end(ap); /* cleanup */
}

void close_trace_file(FILE *fp)
{ //close the file pointer for trace file
  fclose(fp);
}

boolean is_vehicle_detected_for_eager_update(double current_time, struct_vehicle_t *vehicle, edge_queue_t *E, parameter_t *param, struct_sensor_table *S)
{ //check whether the vehicle is detected by some sensor(s) or not and return the queue of detection sensors in the eager update mode in sensor scheduling. We need to release the memory occupied by vehicle's detection queue Q outside this function.
  /** NOTE: We assume that vehicles do not change its direction on one edge. */

	boolean flag = FALSE;
	int num_detection_sensor = 0; //number of detection sensors 
	sensor_queue_node_t node; //sensor queue node
	edge_queue_node_t *pEdgeNode = NULL; //pointer to an edge queue node
	sensor_queue_node_t *pSensorNode = NULL; //pointer to a sensor queue node
	double r = param->sensor_sensing_range; //sensing radius
	double v = vehicle->speed; //speed: [m/s]
	double I_s = -1; //starting time of overlap time interval
	double I_e = -1; //ending time of overlap time interval
	double offset = vehicle->current_pos_in_Gr.offset; //vehicle's offset
	double M_s = vehicle->move_start_time; //starting time of vehicle movement interval
	double M_e = vehicle->move_end_time; //ending time of vehicle movement interval
	double W_s = -1; //starting time of sensor working interval
	double W_e = -1; //ending time of sensor working interval
	double p = -1; //sensor's offset
	double p_left_end = -1; //position of the left-end of sensor's sensing circle
	double p_right_end = -1; //position of the right-end of sensor's sensing circle
	double o_s = -1; //vehicle's offset at I_s
	double o_e = -1; //vehicle's offset at I_e 
	double d_s = -1; //distances between o_s and p
	double d_e = -1; //distances between o_e and p 
	double time_diff = -1; //time differece from M_s to I_s
	double available_working_time = 0; //available working time for sensor's current energy
	double next_working_time = 0; //next working time of sensor during next working period
	int i = 0; //indices for for-loops
	double edge_length = 0; //road segment length corresponding to edge
	double movement_time = 0; //movement time until the vehicle is detected

	/* initialize detection_time for vehicle */
	vehicle->detection_time = INF; //vehicle detection time

	/* initialize detection sensor queue if this queue is used at first, otherwise we need to destory it here */
	if(vehicle->sensor_queue->size <= 0)
		InitQueue((queue_t*)vehicle->sensor_queue, QTYPE_SENSOR);
	else
		DestroyQueue((queue_t*)vehicle->sensor_queue);

	/** get the pointer to the edge node where the vehicle is placed now */
	pEdgeNode = GetEdgeNodeByEID(E, vehicle->current_pos_in_Gr.eid);
	if(pEdgeNode == NULL)
	{
		printf("is_vehicle_detected(): pEdgeNode is NULL\n");
		exit(1);
	}
	
	edge_length = pEdgeNode->weight;
	for(i = 0; i < pEdgeNode->sensor_location_list.size; i++)
	{
		pSensorNode = GetSensorQueueNodeFromLocationQueue(&(pEdgeNode->sensor_location_list), i, S);
		//if(pSensorNode->info.state == SENSOR_SENSE || pSensorNode->info.state == SENSOR_SLEEP) //if-1
		if(pSensorNode->info.state == SENSOR_SENSE || pSensorNode->info.state == SENSOR_ESTIMATE || pSensorNode->info.state == SENSOR_SLEEP) //if-1
		{
			//@For debugging
			//if(current_time >= 5136.5 && vehicle->id == 430 && pSensorNode->order_in_Gr == 149)
			//	printf("senor if is %d\n", pSensorNode->info.id);
			////////////////

			//if(pSensorNode->info.state == SENSOR_SENSE)
			if(pSensorNode->info.state == SENSOR_SENSE || pSensorNode->info.state == SENSOR_ESTIMATE)
			{
				W_s = pSensorNode->info.sensing_start_time;
				W_e = pSensorNode->info.sensing_end_time;
			}
			else if(pSensorNode->info.state == SENSOR_SLEEP)
			{ //we need to check how long the sensor can work for sleeping with the remaining energy
				/* check sensor's available working time for sensing */
				available_working_time = estimate_working_time(pSensorNode->info.energy, pSensorNode->info.sensing_range, pSensorNode->info.energy_consumption_rate);
				next_working_time = MIN(available_working_time, param->sensor_work_time);

				if(next_working_time < param->sensor_work_time) //this sensor cannot work since it has no enough energy to detect vehicle
					continue;

				W_s = pSensorNode->info.sleeping_end_time;
				W_e = pSensorNode->info.sleeping_end_time + next_working_time;
			}

			/* check whether the vehicle's movement time interval [M_s,M_e] is overlapped with the sensor's working time interval [I_s, I_e] or not */
			if(M_s <= W_e && M_e >= W_s) //if-2
			{
				I_s = MAX(M_s, W_s);
				I_e = MIN(M_e, W_e);
			} //end of if-2
			else
				continue;

			p = pSensorNode->info.pos_in_Gr.offset; //position of the sensor at an edge in real graph Gr
			p_left_end = MAX(p-r, 0); //position of the left-end of sensor's sensing circle
			p_right_end = MIN(p+r, edge_length); //position of the right-end of sensor's sensing circle
	
			/*
			if(vehicle->move_type == MOVE_FORWARD)
			{
				time_diff = I_s - M_s;
				//o_s = offset + time_diff*v;
				o_s = MIN(offset + time_diff*v, edge_length);

				time_diff = I_e - M_s;
				//o_e = offset + time_diff*v;
				o_e = MIN(offset + time_diff*v, edge_length);
			}
			else if(vehicle->move_type == MOVE_BACKWARD)
			{
				time_diff = I_s - M_s;
				//o_s = offset - time_diff*v;
				o_s = MAX(offset - time_diff*v, 0);

				time_diff = I_e - M_s;
				//o_e = offset - time_diff*v;
				o_e = MAX(offset - time_diff*v, 0);
			}
			else
			{
				printf("is_vehicle_detected(): vehicle->move_type(%d) is invalid\n", vehicle->move_type);
				exit(1);
			}
			*/
			

			/* time_diff is the differece from M_s to I_s */
			if(vehicle->move_type == MOVE_FORWARD)
			{
				if(M_s == I_s) //since I_s = MAX(M_s, W_s), I_s >= M_s
					time_diff = 0;
				else
					time_diff = I_s - M_s;

				//specify the position of vehicle at time I_s
				o_s = MIN(offset + time_diff*v, edge_length);

				//specify the position of vehicle at time I_e
				time_diff = I_e - I_s;
				o_e = MIN(o_s + time_diff*v, edge_length);
			}
			else if(vehicle->move_type == MOVE_BACKWARD)
			{
				if(M_s == I_s) //since I_s = MAX(M_s, W_s), I_s >= M_s
					time_diff = 0;
				else
					time_diff = I_s - M_s;

				//specify the position of vehicle at time I_s
				o_s = MAX(offset - time_diff*v, 0);

				//specify the position of vehicle at time I_e
				time_diff = I_e - I_s;
				o_e = MAX(o_s - time_diff*v, 0);
			}
			else
			{
				printf("is_vehicle_detected(): vehicle->move_type(%d) is invalid\n", vehicle->move_type);
				exit(1);
			}

			//@For debugging
			/*
			if(time_diff < 0)
			{
				printf("time_diff(%f) is negative!\n", (float)time_diff);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                fgetc(stdin);
#endif
				exit(1);
			}
			*/
			////////////////
			
			/* check whether the vehicle is within the sensor's sensing range during the overlapped time interval [I_s, I_e] */
			d_s = euclidean_distance1(o_s, p);
			d_e = euclidean_distance1(o_e, p);

			/*
			if(d_s <= r)
			{ //vehicle is within the sensing range at I_s
				flag = TRUE;
				movement_time = 0;
			}
			else 
			*/
			if(d_s > r && d_e > r)
			{
				if(o_s <= o_e && (o_s <= p && p <= o_e))       // o_s  (   p   )  o_e
				{//MOVE_FORWARD: Forward movement on the edge  //  v --> 
					flag = TRUE;
					movement_time = (p_left_end - o_s)/v;
				}
				else if(o_s > o_e && (o_e <= p && p <= o_s))   // o_e  (   p   )  o_s
				{//MOVE_BACKWARD: Backward movement on the edge//              <-- v
					flag = TRUE;
					movement_time = (o_s - p_right_end)/v;
				}
			}
			else
			{
				flag = TRUE;

				if(o_s <= o_e) //MOVE_FORWARD
				{
					if(o_s >= p_left_end)  // ( o_s  p   o_e
						movement_time = 0; //    v -->
					else if(o_s < p_left_end) // o_s  (   p   o_e
						movement_time = (p_left_end - o_s)/v;
				}
				else if(o_s > o_e) //MOVE_BACKWARD
				{
					if(o_s <= p_right_end) // o_e   p   o_s )
						movement_time = 0; //              <-- v
					else if(o_s > p_right_end) // o_e   p   )  o_s
						movement_time = (o_s - p_right_end)/v;
				}
			}

			//@For debugging
			/*
			if(movement_time < 0)
			{
				printf("movement_time(%f) is negative!\n", (float)movement_time);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                fgetc(stdin);
#endif
				exit(1);
			}
			*/
			////////////////

			if(flag == TRUE)
			{
				memcpy(&node, pSensorNode, sizeof(node));

				//node.info.recent_detection_time = (I_s + I_e)/2; 
				//we use average of times I_s and I_e considering the detection processing delay.
				//time_diff = I_s - M_s; //time when the vehicle moves from its current offset to the offset corresponding to I_s
				
				time_diff = fabs(o_s-offset)/v; //time taken by the movement from vehicle's offset to o_s
				node.info.recent_detection_time = current_time + time_diff + movement_time;
				//movement_time is the time taken until the vehicle placed at o_s is detected by sensor 
				Enqueue((queue_t*)vehicle->sensor_queue, (queue_node_t*)&node);

				if(vehicle->detection_time > node.info.recent_detection_time) //update detection_time so that it can have the latest detection time in timeline
				{
					vehicle->detection_time = node.info.recent_detection_time;
					vehicle->detecting_sensor = node.info.id; //first detecting sensor's id
				}

				num_detection_sensor++;
				flag = FALSE;
			}
		} //end of if-1
	}
	
	if(num_detection_sensor > 0)
		return TRUE;
	else
		return FALSE;
}

boolean is_vehicle_detected_for_eager_update_and_step_path(double current_time, struct_vehicle_t *vehicle, edge_queue_t *E, parameter_t *param, struct_sensor_table *S, struct_traffic_table *protection_set)
{ //check whether the vehicle is detected by some sensor(s) or not and return the queue of detection sensors in the eager update mode in sensor scheduling along with STEP_PATH. We need to release the memory occupied by vehicle's detection queue Q outside this function.
  /** NOTE: We assume that vehicles do not change its direction on one edge. */

	boolean flag = FALSE;
	int num_detection_sensor = 0; //number of detection sensors 
	sensor_queue_node_t node; //sensor queue node
	edge_queue_node_t *pEdgeNode = NULL; //pointer to an edge queue node
	sensor_queue_node_t *pSensorNode = NULL; //pointer to a sensor queue node
	double r = param->sensor_sensing_range; //sensing radius
	double v = vehicle->speed; //speed: [m/s]
	double I_s = -1; //starting time of overlap time interval
	double I_e = -1; //ending time of overlap time interval
	double offset = 0; //vehicle->current_pos_in_Gr.offset; //vehicle's offset
	double M_s = current_time; //vehicle->move_start_time; //starting time of vehicle movement interval
	double M_e = 0; //vehicle->move_end_time; //ending time of vehicle movement interval
	double W_s = -1; //starting time of sensor working interval
	double W_e = -1; //ending time of sensor working interval
	double p = -1; //sensor's offset
	double p_left_end = -1; //position of the left-end of sensor's sensing circle
	double p_right_end = -1; //position of the right-end of sensor's sensing circle
	double o_s = -1; //vehicle's offset at I_s
	double o_e = -1; //vehicle's offset at I_e 
	double d_s = -1; //distances between o_s and p
	double d_e = -1; //distances between o_e and p 
	double time_diff = -1; //time differece from M_s to I_s
	double available_working_time = 0; //available working time for sensor's current energy
	double next_working_time = 0; //next working time of sensor during next working period
	int i = 0; //indices for for-loops
	double edge_length = 0; //road segment length corresponding to edge
	double movement_time = 0; //movement time until the vehicle is detected
	int eid; //edge id; this indicates the undirectional edge including two directional edges with the same end points of the edge
	char *tail_node = NULL; //pointer to the tail node of an edge
	char *head_node = NULL; //pointer to the head node of an edge
	struct_path_node *ptr; //pointer to the current position vertex on the path
	MOVE_TYPE move_type; //movement type for directional edge = {MOVE_FORWARD, MOVE_BACKWARD}


	/* initialize detection_time for vehicle */
	vehicle->detection_time = INF; //vehicle detection time

	/* initialize detection sensor queue if this queue is used at first, otherwise we need to destory it here */
	if(vehicle->sensor_queue->size <= 0)
		InitQueue((queue_t*)vehicle->sensor_queue, QTYPE_SENSOR);
	else
		DestroyQueue((queue_t*)vehicle->sensor_queue);

	/** check whether the vehicle can be detected by some sensor(s) via its path from the source to the destination (or protection point) */
	ptr = vehicle->path_ptr; //ptr is set to the entrance vertex
	while(!IsProtectionPoint(ptr->vertex, protection_set)) //while
	{
	  tail_node = ptr->vertex;
	  head_node = ptr->next->vertex;
	  //eid = FastGetEdgeID_MoveType(G, tail_node, head_node, &move_type, &edge_length);
	  eid = GetEdgeID_MoveType(E, tail_node, head_node, &move_type, &edge_length);

	  /** set the vehicle's offset and movement_end_time (M_e) on each edge */
	  /* update the vehicle's offset according to move_type */
	  if(move_type == MOVE_FORWARD)
	    offset = 0;
	  else
	    offset = edge_length;

	  /* update movement_end_time (M_e) for the edge */
	  M_e = M_s + edge_length/v;

	  if((M_e - vehicle->move_end_time) > ERROR_TOLERANCE_FOR_REAL_ARITHMETIC) //if M_e is greater than move_end_time arriving at the vehicle's destination, then exit the while loop.
	  //if(M_e > vehicle->move_end_time) //if M_e is greater than move_end_time arriving at the vehicle's destination, then exit the while loop.
	    break;

	  /** get the pointer to the edge node where the vehicle is placed now */
	  pEdgeNode = GetEdgeNodeByEID(E, eid);
	  if(pEdgeNode == NULL)
	  {
	    printf("is_vehicle_detected_for_eager_update_and_step_path(): pEdgeNode is NULL\n");
	    exit(1);
	  }

	  //edge_length = pEdgeNode->weight;
	  for(i = 0; i < pEdgeNode->sensor_location_list.size; i++) //for
	  {
		pSensorNode = GetSensorQueueNodeFromLocationQueue(&(pEdgeNode->sensor_location_list), i, S);
		//if(pSensorNode->info.state == SENSOR_SENSE || pSensorNode->info.state == SENSOR_SLEEP) //if-1
		if(pSensorNode->info.state == SENSOR_SENSE || pSensorNode->info.state == SENSOR_ESTIMATE || pSensorNode->info.state == SENSOR_SLEEP) //if-1
		{
			//@For debugging
			//if(current_time >= 5136.5 && vehicle->id == 430 && pSensorNode->order_in_Gr == 149)
			//	printf("senor if is %d\n", pSensorNode->info.id);
			////////////////

			//if(pSensorNode->info.state == SENSOR_SENSE)
			if(pSensorNode->info.state == SENSOR_SENSE || pSensorNode->info.state == SENSOR_ESTIMATE)
			{
				W_s = pSensorNode->info.sensing_start_time;
				W_e = pSensorNode->info.sensing_end_time;
			}
			else if(pSensorNode->info.state == SENSOR_SLEEP)
			{ //we need to check how long the sensor can work for sleeping with the remaining energy
				/* check sensor's available working time for sensing */
				available_working_time = estimate_working_time(pSensorNode->info.energy, pSensorNode->info.sensing_range, pSensorNode->info.energy_consumption_rate);
				next_working_time = MIN(available_working_time, param->sensor_work_time);

				if(next_working_time < param->sensor_work_time) //this sensor cannot work since it has no enough energy to detect vehicle
					continue;

				W_s = pSensorNode->info.sleeping_end_time;
				W_e = pSensorNode->info.sleeping_end_time + next_working_time;
			}

			/* check whether the vehicle's movement time interval [M_s,M_e] is overlapped with the sensor's working time interval [I_s, I_e] or not */
			if(M_s <= W_e && M_e >= W_s) //if-2
			{
				I_s = MAX(M_s, W_s);
				I_e = MIN(M_e, W_e);
			} //end of if-2
			else
				continue;

			p = pSensorNode->info.pos_in_Gr.offset; //position of the sensor at an edge in real graph Gr
			p_left_end = MAX(p-r, 0); //position of the left-end of sensor's sensing circle
			p_right_end = MIN(p+r, edge_length); //position of the right-end of sensor's sensing circle
	
			/*
			if(move_type == MOVE_FORWARD)
			{
				time_diff = I_s - M_s;
				//o_s = offset + time_diff*v;
				o_s = MIN(offset + time_diff*v, edge_length);

				time_diff = I_e - M_s;
				//o_e = offset + time_diff*v;
				o_e = MIN(offset + time_diff*v, edge_length);
			}
			else if(move_type == MOVE_BACKWARD)
			{
				time_diff = I_s - M_s;
				//o_s = offset - time_diff*v;
				o_s = MAX(offset - time_diff*v, 0);

				time_diff = I_e - M_s;
				//o_e = offset - time_diff*v;
				o_e = MAX(offset - time_diff*v, 0);
			}
			else
			{
				printf("is_vehicle_detected(): vehicle->move_type(%d) is invalid\n", vehicle->move_type);
				exit(1);
			}
			*/
			

			/* time_diff is the differece from M_s to I_s */
			if(move_type == MOVE_FORWARD)
			{
				if(M_s == I_s) //since I_s = MAX(M_s, W_s), I_s >= M_s
					time_diff = 0;
				else
					time_diff = I_s - M_s;

				//specify the position of vehicle at time I_s
				o_s = MIN(offset + time_diff*v, edge_length);

				//specify the position of vehicle at time I_e
				time_diff = I_e - I_s;
				o_e = MIN(o_s + time_diff*v, edge_length);
			}
			else if(move_type == MOVE_BACKWARD)
			{
				if(M_s == I_s) //since I_s = MAX(M_s, W_s), I_s >= M_s
					time_diff = 0;
				else
					time_diff = I_s - M_s;

				//specify the position of vehicle at time I_s
				o_s = MAX(offset - time_diff*v, 0);

				//specify the position of vehicle at time I_e
				time_diff = I_e - I_s;
				o_e = MAX(o_s - time_diff*v, 0);
			}
			else
			{
				printf("is_vehicle_detected(): vehicle->move_type(%d) is invalid\n", vehicle->move_type);
				exit(1);
			}

			//@For debugging
			/*
			if(time_diff < 0)
			{
				printf("time_diff(%f) is negative!\n", (float)time_diff);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                fgetc(stdin);
#endif
				exit(1);
			}
			*/
			////////////////
			
			/* check whether the vehicle is within the sensor's sensing range during the overlapped time interval [I_s, I_e] */
			d_s = euclidean_distance1(o_s, p);
			d_e = euclidean_distance1(o_e, p);

			/*
			if(d_s <= r)
			{ //vehicle is within the sensing range at I_s
				flag = TRUE;
				movement_time = 0;
			}
			else 
			*/
			if(d_s > r && d_e > r)
			{
				if(o_s <= o_e && (o_s <= p && p <= o_e))       // o_s  (   p   )  o_e
				{//MOVE_FORWARD: Forward movement on the edge  //  v --> 
					flag = TRUE;
					movement_time = (p_left_end - o_s)/v;
				}
				else if(o_s > o_e && (o_e <= p && p <= o_s))   // o_e  (   p   )  o_s
				{//MOVE_BACKWARD: Backward movement on the edge//              <-- v
					flag = TRUE;
					movement_time = (o_s - p_right_end)/v;
				}
			}
			else
			{
				flag = TRUE;

				if(o_s <= o_e) //MOVE_FORWARD
				{
					if(o_s >= p_left_end)  // ( o_s  p   o_e
						movement_time = 0; //    v -->
					else if(o_s < p_left_end) // o_s  (   p   o_e
						movement_time = (p_left_end - o_s)/v;
				}
				else if(o_s > o_e) //MOVE_BACKWARD
				{
					if(o_s <= p_right_end) // o_e   p   o_s )
						movement_time = 0; //              <-- v
					else if(o_s > p_right_end) // o_e   p   )  o_s
						movement_time = (o_s - p_right_end)/v;
				}
			}

			//@For debugging
			/*
			if(movement_time < 0)
			{
				printf("movement_time(%f) is negative!\n", (float)movement_time);
#ifdef __DEBUG_INTERACTIVE_MODE__
                                fgetc(stdin);
#endif
				exit(1);
			}
			*/
			////////////////

			if(flag == TRUE)
			{
				memcpy(&node, pSensorNode, sizeof(node));

				//node.info.recent_detection_time = (I_s + I_e)/2; 
				//we use average of times I_s and I_e considering the detection processing delay.
				//time_diff = I_s - M_s; //time when the vehicle moves from its current offset to the offset corresponding to I_s
				
				time_diff = fabs(o_s-offset)/v; //time taken by the movement from vehicle's offset to o_s
				node.info.recent_detection_time = current_time + time_diff + movement_time;
				//movement_time is the time taken until the vehicle placed at o_s is detected by sensor 
				Enqueue((queue_t*)vehicle->sensor_queue, (queue_node_t*)&node);

				if(vehicle->detection_time > node.info.recent_detection_time) //update detection_time so that it can have the latest detection time in timeline
				{
					vehicle->detection_time = node.info.recent_detection_time;
					vehicle->detecting_sensor = node.info.id; //first detecting sensor's id
				}

				num_detection_sensor++;
				flag = FALSE;
			}
		} //end of if-1
	  } //end of for

	  /** if some sensors on the current edge detect the vehicle, then exit the while loop */
	  if(num_detection_sensor > 0)
	    break;

          /** update movement_start_time (M_s) with movement_end_time (M_e) for the previous edge */
	  M_s = M_e;	  

          /** update ptr to next vertex on the path */
	  ptr = ptr->next;
	} //end of while
	
	if(num_detection_sensor > 0)
		return TRUE;
	else
		return FALSE;
}

boolean is_vehicle_detected_for_lazy_update(double current_time, struct_vehicle_t *vehicle, edge_queue_t *E, parameter_t *param, struct_sensor_table *S)
{ //check whether the vehicle is detected by some sensor(s) or not and return the queue of detection sensors in the lazy update mode in sensor scheduling. We need to release the memory occupied by vehicle's detection queue Q outside this function.
  /** NOTE: We assume that vehicles do not change its direction on one edge. */

	int num_detection_sensor = 0; //number of detection sensors 
	sensor_queue_node_t node; //sensor queue node
	edge_queue_node_t *pEdgeNode = NULL; //pointer to an edge queue node
	sensor_queue_node_t *pSensorNode = NULL; //pointer to a sensor queue node
	double r = param->sensor_sensing_range; //sensing radius
	double v = vehicle->speed; //speed: [m/s]
	double offset = vehicle->current_pos_in_Gr.offset; //vehicle's offset
	double M_s = vehicle->move_start_time; //starting time of vehicle movement interval
	double M_e = vehicle->move_end_time; //ending time of vehicle movement interval
	double p = -1; //sensor's offset
	double p_left_end = -1; //position of the left-end of sensor's sensing circle
	double p_right_end = -1; //position of the right-end of sensor's sensing circle
	double time_diff = -1; //time differece from M_s to I_s
	double available_working_time = 0; //available working time for sensor's current energy
	double next_working_time = 0; //next working time of sensor during next working period
	int i = 0; //indices for for-loops
	double edge_length = 0; //road segment length corresponding to edge
	double vehicle_moving_distance = 0; //moving distance from vehicle's offset to the sensing circle
	double vehicle_moving_time = 0 ; //moving time duration from vehicle's offset to the sensing circle
	double t_arrive = 0; //time point when the vehicle arrives at the sensor's sensing circle
	double t_depart = 0; //time point when the vehicle departs at the sensor's sensing circle
	double t_meet = 0; //time point when the vehicle meets the sensor's closest working period within the sensor's sensing circle
	STATE state = STATE_UNKNOWN; //sensor's state

	/* initialize detection_time for vehicle */
	vehicle->detection_time = INF; //vehicle detection time

	/* initialize detection sensor queue if this queue is used at first, otherwise we need to destory it here */
	if(vehicle->sensor_queue->size <= 0)
		InitQueue((queue_t*)vehicle->sensor_queue, QTYPE_SENSOR);
	else
		DestroyQueue((queue_t*)vehicle->sensor_queue);

	/** get the pointer to the edge node where the vehicle is placed now */
	pEdgeNode = GetEdgeNodeByEID(E, vehicle->current_pos_in_Gr.eid);
	if(pEdgeNode == NULL)
	{
		printf("is_vehicle_detected(): pEdgeNode is NULL\n");
		exit(1);
	}
	
	edge_length = pEdgeNode->weight;
	for(i = 0; i < pEdgeNode->sensor_location_list.size; i++)
	{
		pSensorNode = GetSensorQueueNodeFromLocationQueue(&(pEdgeNode->sensor_location_list), i, S);

		p = pSensorNode->info.pos_in_Gr.offset; //position of the sensor at an edge in real graph Gr
		p_left_end = MAX(p-r, 0); //position of the left-end of sensor's sensing circle
		p_right_end = MIN(p+r, edge_length); //position of the right-end of sensor's sensing circle

		/* compute t_arrive that is the time point when the vehicle arrives at the sensor's sensing circle */
	        if(vehicle->move_type == MOVE_FORWARD)
		  vehicle_moving_distance = p_left_end - offset;
		else
		  vehicle_moving_distance = offset - p_right_end;
		
		vehicle_moving_time = vehicle_moving_distance/v;
		t_arrive = M_s + vehicle_moving_time; //time point hitting the sensor's sensing circle

		/* compute t_depart that is the time point when the vehicle departs from the sensor's sensing circle */
		vehicle_moving_distance = p_right_end - p_left_end;
		vehicle_moving_time = vehicle_moving_distance/v;
		t_depart = t_arrive + vehicle_moving_time; //time point leaving from the sensor's sensing circle

		/* determine the time point when the vehicle meets the starting time of the first working period during the movement between t_arrive and t_depart. 
                  If there is a working period overlapped during the movement time, return the time point; otherwise retirn -1. */
		t_meet = get_sensing_circle_meeting_time(t_arrive, t_depart, &(pSensorNode->info), &state);
 
		if(t_meet >= 0) //if-1
		{
		  memcpy(&node, pSensorNode, sizeof(node));

		  node.info.recent_detection_time = t_meet; //time when the vehicle is detected by this sensor
		  Enqueue((queue_t*)vehicle->sensor_queue, (queue_node_t*)&node);

		  if(vehicle->detection_time > node.info.recent_detection_time) //update detection_time so that it can have the latest detection time in timeline
		  {
		    vehicle->detection_time = node.info.recent_detection_time;
		    vehicle->detecting_sensor = node.info.id; //first detecting sensor's id
		  }

		  num_detection_sensor++;
		} //end of if-1
	}
	
	if(num_detection_sensor > 0)
		return TRUE;
	else
		return FALSE;
}

boolean is_vehicle_detected_for_lazy_update_and_step_path(double current_time, struct_vehicle_t *vehicle, edge_queue_t *E, parameter_t *param, struct_sensor_table *S, struct_traffic_table *protection_set)
{ //check whether the vehicle is detected by some sensor(s) or not and return the queue of detection sensors in the lazy update mode in sensor scheduling along with STEP_PATH. We need to release the memory occupied by vehicle's detection queue Q outside this function.
  /** NOTE: We assume that vehicles do not change its direction on one edge. */

	int num_detection_sensor = 0; //number of detection sensors 
	sensor_queue_node_t node; //sensor queue node
	edge_queue_node_t *pEdgeNode = NULL; //pointer to an edge queue node
	sensor_queue_node_t *pSensorNode = NULL; //pointer to a sensor queue node
	double r = param->sensor_sensing_range; //sensing radius
	double v = vehicle->speed; //speed: [m/s]
	double offset = 0; //vehicle->current_pos_in_Gr.offset; //vehicle's offset
	double M_s = current_time; // = vehicle->move_start_time; //starting time of vehicle movement interval
	double M_e = 0; // = vehicle->move_end_time; //ending time of vehicle movement interval
	double p = -1; //sensor's offset
	double p_left_end = -1; //position of the left-end of sensor's sensing circle
	double p_right_end = -1; //position of the right-end of sensor's sensing circle
	double time_diff = -1; //time differece from M_s to I_s
	double available_working_time = 0; //available working time for sensor's current energy
	double next_working_time = 0; //next working time of sensor during next working period
	int i = 0, k = 0; //indices for for-loops
	double edge_length = 0; //road segment length corresponding to edge
	double vehicle_moving_distance = 0; //moving distance from vehicle's offset to the sensing circle
	double vehicle_moving_time = 0 ; //moving time duration from vehicle's offset to the sensing circle
	double t_arrive = 0; //time point when the vehicle arrives at the sensor's sensing circle
	double t_depart = 0; //time point when the vehicle departs at the sensor's sensing circle
	double t_meet = 0; //time point when the vehicle meets the sensor's closest working period within the sensor's sensing circle
	int eid; //edge id; this indicates the undirectional edge including two directional edges with the same end points of the edge
	char *tail_node = NULL; //pointer to the tail node of an edge
	char *head_node = NULL; //pointer to the head node of an edge
	struct_path_node *ptr; //pointer to the current position vertex on the path
	MOVE_TYPE move_type; //movement type for directional edge = {MOVE_FORWARD, MOVE_BACKWARD}
	STATE state = STATE_UNKNOWN; //sensor's state
 
	/* initialize detection_time for vehicle */
	vehicle->detection_time = INF; //vehicle detection time

	/* initialize detection sensor queue if this queue is used at first, otherwise we need to destory it here */
	if(vehicle->sensor_queue->size <= 0)
		InitQueue((queue_t*)vehicle->sensor_queue, QTYPE_SENSOR);
	else
		DestroyQueue((queue_t*)vehicle->sensor_queue);

	/** check whether the vehicle can be detected by some sensor(s) via its path from the source to the destination (or protection point) */
	ptr = vehicle->path_ptr; //ptr is set to the entrance vertex
	while(!IsProtectionPoint(ptr->vertex, protection_set)) //while
	{
	  tail_node = ptr->vertex;
	  head_node = ptr->next->vertex;
	  //eid = FastGetEdgeID_MoveType(G, tail_node, head_node, &move_type, &edge_length);
	  eid = GetEdgeID_MoveType(E, tail_node, head_node, &move_type, &edge_length);

	  /** set the vehicle's offset and movement_end_time (M_e) on each edge */
	  /* update the vehicle's offset according to move_type */
	  if(move_type == MOVE_FORWARD)
	    offset = 0;
	  else
	    offset = edge_length;

	  /* update movement_end_time (M_e) for the edge */
	  M_e = M_s + edge_length/v;
	  
	  if((M_e - vehicle->move_end_time) > ERROR_TOLERANCE_FOR_REAL_ARITHMETIC) //if M_e is greater than move_end_time arriving at the vehicle's destination, then exit the while loop.	  
	  //if(M_e > vehicle->move_end_time) //if M_e is greater than move_end_time arriving at the vehicle's destination, then exit the while loop.
	    break;

	  /** get the pointer to the edge node where the vehicle is placed now */
	  pEdgeNode = GetEdgeNodeByEID(E, eid);
	  if(pEdgeNode == NULL)
	  {
	    printf("is_vehicle_detected_for_lazy_update_and_step_path(): pEdgeNode is NULL\n");
	    exit(1);
	  }
	
	  //edge_length = pEdgeNode->weight;
	  for(i = 0; i < pEdgeNode->sensor_location_list.size; i++) //for
	  {
	    pSensorNode = GetSensorQueueNodeFromLocationQueue(&(pEdgeNode->sensor_location_list), i, S);

	    //@for debugging
	    //if(vehicle->id == 9062 && current_time > 108270 && pSensorNode->info.id == 1231) //== 2271+121)
	    //  printf("vehicle %d is checked\n", vehicle->id);
	    ////////////////

	    p = pSensorNode->info.pos_in_Gr.offset; //position of the sensor at an edge in real graph Gr
	    p_left_end = MAX(p-r, 0); //position of the left-end of sensor's sensing circle
	    p_right_end = MIN(p+r, edge_length); //position of the right-end of sensor's sensing circle

	    /* compute t_arrive that is the time point when the vehicle arrives at the sensor's sensing circle */
	    if(move_type == MOVE_FORWARD)
	      vehicle_moving_distance = p_left_end - offset;
	    else
	      vehicle_moving_distance = offset - p_right_end;
		
	    vehicle_moving_time = vehicle_moving_distance/v;
	    t_arrive = M_s + vehicle_moving_time; //time point hitting the sensor's sensing circle

	    /* compute t_depart that is the time point when the vehicle departs from the sensor's sensing circle */
	    vehicle_moving_distance = p_right_end - p_left_end;
	    vehicle_moving_time = vehicle_moving_distance/v;
	    t_depart = t_arrive + vehicle_moving_time; //time point leaving from the sensor's sensing circle

	    /* determine the time point when the vehicle meets the starting time of the first working period during the movement between t_arrive and t_depart. 
               If there is a working period overlapped during the movement time, return the time point; otherwise retirn -1. */
	    t_meet = get_sensing_circle_meeting_time(t_arrive, t_depart, &(pSensorNode->info), &state);
 
	    if(t_meet >= 0) //if-1
	    {
	      memcpy(&node, pSensorNode, sizeof(node));

	      node.info.recent_detection_time = t_meet; //time when the vehicle is detected by this sensor
	      Enqueue((queue_t*)vehicle->sensor_queue, (queue_node_t*)&node);

	      if(vehicle->detection_time > node.info.recent_detection_time) //update detection_time so that it can have the latest detection time in timeline
	      {
		vehicle->detection_time = node.info.recent_detection_time;
		vehicle->detecting_sensor = node.info.id; //first detecting sensor's id
	      }

	      num_detection_sensor++;
	    } //end of if-1

	  } //end of for

	  /** if some sensors on the current edge detect the vehicle, then exit the while loop */
	  if(num_detection_sensor > 0)
	    break;

	  /** update movement_start_time (M_s) with movement_end_time (M_e) for the previous edge */
	  M_s = M_e;

	  /** update ptr to next vertex on the path */
	  ptr = ptr->next;
	} //end of while
	
	if(num_detection_sensor > 0)
		return TRUE;
	else
		return FALSE;
}

double euclidean_distance1(double x1, double x2)
{ //return the Euclidean distance between two points in one-dimensional space, i.e., line
	double d; //distance
	
	d = fabs(x1 - x2);

	return d;
}

double euclidean_distance2(struct_coordinate1_t *p1, struct_coordinate1_t *p2)
{ //return the Euclidean distance between two points in two-dimensional space, i.e., plane
	double d; //distance
	double delta_x = p1->x - p2->x;
	double delta_y = p1->y - p2->y;
	double sum = 0;

	sum = delta_x*delta_x + delta_y*delta_y;

	d = sqrt(sum);;

	return d;
}

boolean is_vehicle_within_communication_range_of_point(parameter_t *param, struct_vehicle_t *vehicle, struct_coordinate1_t *point)
{ //check whether the vehicle is within the communication range of point
    boolean result = FALSE; 
    double distance = 0; //distance between vehicle and point
    struct_coordinate1_t *p1 = &(vehicle->current_pos); //vehicle's position in the 2D space
    struct_coordinate1_t *p2 = point; //point's position in the 2D space

    distance = euclidean_distance2(p1, p2);

    if(distance <= param->communication_range)
        result = TRUE;

    return result;
}


char* get_distribution_type_name(distribution_type_t type)
{ //return the distribution type name in string
	static char distribution_type_name[NUMBER_OF_DISTRIBUTION_TYPE][BUF_SIZE] = {
	  STRING_FOR_UNKNOWN_DISTRIBUTION, 
	  STRING_FOR_EQUAL,
	  STRING_FOR_UNIFORM, 
	  STRING_FOR_NORMAL, 
	  STRING_FOR_EXPONENTIAL, 
	  STRING_FOR_ERLANG, 
	  STRING_FOR_HYPERX
	};

	return distribution_type_name[type];
}

char* get_scan_type_name(sensor_scan_type_t scan_type)
{ //return the scan type name in string
	static char scan_type_name[NUMBER_OF_SCAN_TYPE][BUF_SIZE] = {
	  STRING_FOR_SCAN_UNKNOWN,
	  STRING_FOR_SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING,
	  STRING_FOR_SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING,	  
	  STRING_FOR_SCAN_NO_USE,
	  STRING_FOR_SCAN_TURN_ON_ALL,
	  STRING_FOR_SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING,
	  STRING_FOR_SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING_AND_WITH_SENSING_HOLE_HANDLING,
	  STRING_FOR_SCAN_NO_USE_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING,
	  STRING_FOR_SCAN_VARIABLE_SPEED_WITH_VARIABLE_SLEEPING_TIME_AND_WITH_SENSING_HOLE_HANDLING,
	  STRING_FOR_SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_VARIABLE_WORKING_TIME,
	  STRING_FOR_SCAN_CONSTANT_SPEED_WITH_NONOPTIMAL_SLEEPING,
	  STRING_FOR_SCAN_VARIABLE_SPEED_WITH_NONOPTIMAL_SLEEPING,
	  STRING_FOR_SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING,
	  STRING_FOR_SCAN_CONSTANT_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING
	};

	return scan_type_name[scan_type];
}

char* get_sensing_hole_handling_algorithm_name(hole_handling_algorithm_t sensing_hole_handling_algorithm)
{ //return the sensing hole handling algorithm name in string
	static char hole_handling_algorithm_name[NUMBER_OF_HOLE_HANDLING_TYPE][BUF_SIZE] = {
		STRING_FOR_HOLE_HANDLING_UNKNOWN,
		STRING_FOR_HOLE_HANDLING_EXHAUSTIVE_SEARCH_ALGORITHM,
		STRING_FOR_HOLE_HANDLING_GREEDY_ALGORITHM_BASED_HEURISTICS,
		STRING_FOR_HOLE_HANDLING_GREEDY_ALGORITHM_BASED_MINIMAL_SPANNING_TREE,
		STRING_FOR_HOLE_HANDLING_RANDOM_LABELING,
		STRING_FOR_HOLE_HANDLING_NO_HANDLING,
		STRING_FOR_HOLE_HANDLING_ALL_ENTRANCE_POINTS,
		STRING_FOR_HOLE_HANDLING_ALL_PROTECTION_POINTS
	};

	return hole_handling_algorithm_name[sensing_hole_handling_algorithm];
}

boolean is_sensing_hole(sensor_queue_node_t *pSensorNode, schedule_table_node_t *pTableNode, double *left_hole_offset, double *right_hole_offset)
{ //check if this sensor's death creates a sensing hole
	boolean result = FALSE;
	double weight = pTableNode->weight; //weight
	sensor_queue_node_t *p = NULL; //pointer to sensor node

	//int order = pSensorNode->order; //order in sensor list => make errors since it accesses a wrong index
	//int order = pSensorNode->order_for_live_sensors; //order for live sensors in sensor list
	int order = pSensorNode->order_in_Gv; //order for sensors in sensor list in the virtual graph

	//double r1 = param->sensor_sensing_range; //sensing radius of sensor
	double r = 0; //sensing radius of neighboring sensor
	//double range = 0; //coverage range that is the sum of r1 and r2
	int i; //index for for-loop
	double offset1 = pSensorNode->info.pos_in_Gv.offset; //offset of sensor in road segment
	double offset2 = -1; //offset of neighboring sensor
	//double left_hole_offset = -1; //offset of left hole
	//double right_hole_offset = -1; //offset of right hole
	double d = -1; //distance between two offsets

	/* check if the left sensing range of the dying sensor is covered by neighbor sensors */
	*left_hole_offset = -1;
	for(i = order - 1; i >= 0; i--)
	{
		p = (sensor_queue_node_t*) GetQueueNode((queue_t*) &(pTableNode->sensor_list), i);

		if(p->info.state == SENSOR_DIE)
			continue;

		r = p->info.sensing_range;
		offset2 = p->info.pos_in_Gv.offset;
		d = fabs(offset2 - offset1);
		
		if(d <= r)
		{
			*left_hole_offset = offset1 + (r-d);
			break;
		}
		else if(d > r)
		{
			*left_hole_offset = offset2 + r;
			break;
		}		
	}

	/* check whether there is no sensor to the left of the dying sensor */
	if(i < 0)
		*left_hole_offset = 0;


	/* check if the right sensing range of the dying sensor is covered by neighbor sensors */
	*right_hole_offset = -1;
	for(i = order + 1; i <= pTableNode->sensor_list.size - 1; i++)
	{
		p = (sensor_queue_node_t*) GetQueueNode((queue_t*) &(pTableNode->sensor_list), i);

		if(p->info.state == SENSOR_DIE)
			continue;

		r = p->info.sensing_range;
		offset2 = p->info.pos_in_Gv.offset;
		d = fabs(offset2 - offset1);
		
		if(d <= r)
		{
			*right_hole_offset = offset1 - (r-d);
			break;
		}
		else if(d > r)
		{
			*right_hole_offset = offset2 - r;
			break;
		}		
	}
	/* check whether there is no sensor to the left of the dying sensor */
	if(i > pTableNode->sensor_list.size - 1)
		*right_hole_offset = weight;

	/* check if there is a sensing hole or not */
	//else if(*left_hole_offset == 0 && *right_hole_offset == weight)
	if((fabs(*left_hole_offset) <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC) && (fabs(*right_hole_offset - weight) <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC))
	//tail_node and head_node become virtual enter/exit nodes
		result = TRUE;
	//else if(*left_hole_offset == 0 && *right_hole_offset > 0)
	else if((fabs(*left_hole_offset) <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC) && (*right_hole_offset > 0 && *right_hole_offset < weight))
	//tail_node becomes virtual enter/exit node and there is one sensing hole
		result = TRUE;
	//else if((*left_hole_offset > 0 && *left_hole_offset < weight) && *right_hole_offset == weight)
	else if((*left_hole_offset > 0 && *left_hole_offset < weight) && (fabs(*right_hole_offset - weight) <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC))
	//there is one sensing hole and head_node becomes virtual enter/exit node
		result = TRUE;
	else if((*left_hole_offset > 0 && *right_hole_offset < weight) && (*left_hole_offset < *right_hole_offset))
	//there are two sensing holes
		result = TRUE;
	else if(*left_hole_offset >= *right_hole_offset)
	//there is no sensing hole even though the sensor dies, 
	//because other sensors can cover the sensor's sensing area.
		result = FALSE;
	else
	{
		result = FALSE;
		printf("is_sensing_hole(): the condition of *left_hole_offset(%f) and *right_hole_offset(%f) \
			is out of what we expected\n", (float) *left_hole_offset, (float) *right_hole_offset);
#ifdef __DEBUG_INTERACTIVE_MODE__
                fgetc(stdin);
#endif
		exit(1);
	}

	return result;
}

boolean is_initial_sensing_hole(int order, sensor_queue_node_t *pSensorNode, schedule_table_node_t *pTableNode, double *left_hole_offset, double *right_hole_offset)
{ //check whether or not there is a sensing hole segment to the left of the sensor, but the last sensor also checks whether or not there is a righ hole segment
	boolean result = FALSE;
	int n = pTableNode->sensor_list.size; //number of the sensors on the edge
	double weight = pTableNode->weight; //weight
	sensor_queue_node_t *p = NULL; //pointer to the just previous sensor node	
	double r1 = pSensorNode->info.sensing_range; //sensing radius of this sensor
	double r2 = 0; //sensing radius of neighboring sensor
	double offset1 = pSensorNode->info.pos_in_Gr.offset; //offset of sensor in road segment
	double offset2 = -1; //offset of neighboring sensor
	
	/** check the existence of left hole segment or right hole segment depending on order */
	if(order == 0) /* check the existence of a left hole segment for the sensor of order 0 */
	{
		if(offset1 - r1 <= 0) //this sensor covers the left segment from the sensor to the tail of the edge
		{
			*left_hole_offset = -1;
			*right_hole_offset = -1;
			result = FALSE;
		}
		else //there is a left sensing hole segment from the sensor to the tail of the edge
		{
			*left_hole_offset = 0;
			*right_hole_offset = offset1 - r1;
			result = TRUE;
		}
	}
	else if(order == n) /* check the existence of right hole segment for the sensor of order n-1 */
	{
		if(offset1 + r1 >= weight) //this sensor covers the right segment from the sensor to the tail of the edge
		{
			*left_hole_offset = -1;
			*right_hole_offset = -1;
			result = FALSE;
		}
		else //there is a right sensing hole segment from the sensor to the head of the edge
		{
			*left_hole_offset = offset1 + r1;
			*right_hole_offset = weight;
			result = TRUE;
		}
	}
	else /* check the existence of left hole segment for the sensor of order */
	{
		//p = (sensor_queue_node_t*) GetQueueNode((queue_t*) &(pTableNode->sensor_list), order);
		p = (sensor_queue_node_t*) GetQueueNode((queue_t*) &(pTableNode->sensor_list), order-1); //p points the a neighboring sensor left to the sensor pointed by pSensorNode with offset
		offset2 = p->info.pos_in_Gr.offset;
		r2 = p->info.sensing_range;

		if(offset1 - r1 <= offset2 + r2) //this sensor's coverage intersects the coverage of the previous sensor placed to the left of this sensor, so there is no sensing hole
		{
			*left_hole_offset = -1;
			*right_hole_offset = -1;
			result = FALSE;
		}
		else //this sensor's coverage does not intersect the coverage of the previous sensor placed to the left of this sensor, so there is a sensing hole segment
		{
			*left_hole_offset = offset2 + r2;
			*right_hole_offset = offset1 - r1;
			result = TRUE;
		}
	}

	return result;
}

void show_status_of_sensors_on_path(FILE *fp, edge_queue_t *E, struct_sensor_table *S, struct_path_node *path_list)
{ //show live sensors and dead sensors on a path in road graph along with the list of vertices on the path from source to destination
	struct_path_node *pPathNode = NULL; //pointer to a path node
	char *u = NULL; //tail of an edge
	char *v = NULL; //head of an edge
	edge_queue_node_t *pEdgeNode = NULL; //pointer to an edge queue node
	location_queue_node_t *pLocationNode = NULL; //pointer to a location queue node
	sensor_queue_node_t *pSensorNode = NULL; //pointer to a sensor queue node
	boolean flip_flag = FALSE; //flag to see whether u is tail and v is head or the reverse
	int i = 0; //index for for-loop
	int count = 0; //counter
	int order = 1; //the order of edge visited by the vehicle

#ifdef __DEBUG_LEVEL_SHOW_STATUS_OF_SENSORS_ON_PATH__
	printf("\n\n### Display of Live Sensors on the Path taken by Escaped Vehicle ###\n");
#endif

	if(fp) fprintf(fp, "\n\n### Display of Live Sensors on the Path taken by Escaped Vehicle ###\n\n");

	pPathNode = path_list->next; //pPathNode points to the entrance node
	while(IsEndOfTravel(path_list, pPathNode) == FALSE) //while
	{
		u = pPathNode->vertex;       //u is the edge's tail 
		v = pPathNode->next->vertex; //v is the edge's head
		pEdgeNode = LookupEdgeQueue(E, u, v, &flip_flag);
		//return the pointer to the edge queue entry corresponding to the edge consisting of vertices u and v

#ifdef __DEBUG_LEVEL_SHOW_STATUS_OF_SENSORS_ON_PATH__
		printf("[%d] edge (%s, %s) of length %f with %d sensors: \n\n", order, u, v, pEdgeNode->weight, pEdgeNode->sensor_location_list.size);
#endif

		if(fp) fprintf(fp, "[%d] edge (%s, %s) of length %f with %d sensors: \n", order, u, v, pEdgeNode->weight, pEdgeNode->sensor_location_list.size);
		order++;

		count = 0;
		pLocationNode = &(pEdgeNode->sensor_location_list.head);
		for(i = 0; i < pEdgeNode->sensor_location_list.size; i++) //for
		{
			pLocationNode = pLocationNode->next;
			pSensorNode = GetSensorQueueNodeFromLocationQueue(&(pEdgeNode->sensor_location_list), i, S);
			if(pSensorNode->info.state == SENSOR_DIE)
			{ //continue;				
#ifdef __DEBUG_LEVEL_SHOW_STATUS_OF_SENSORS_ON_PATH__
			  printf("D: "); //D stands for DEAD
#endif
			  if(fp) fprintf(fp, "D: ");
			}
			else
			{
#ifdef __DEBUG_LEVEL_SHOW_STATUS_OF_SENSORS_ON_PATH__
			  printf("L: "); //L stands for LIVE
#endif
			  if(fp) fprintf(fp, "L: ");
			  count++;
			}
			 
			if(flip_flag == FALSE)
			{
#ifdef __DEBUG_LEVEL_SHOW_STATUS_OF_SENSORS_ON_PATH__
				printf("sensor id %d: offset of edge (%s, %s)=%f, initial energy=%f, current energy=%f, state=%d, sensing_start_time=%f, sensing_end_time=%f, sleeping_interval=%f, relative_sensing_start_time=%f, relative_sensing_end_time=%f, surveillance_restart_time=%f", pSensorNode->info.id, u, v, (float)pSensorNode->info.pos_in_Gr.offset, (float)pSensorNode->info.initial_energy, (float)pSensorNode->info.energy, pSensorNode->info.state, (float)pSensorNode->info.sensing_start_time, (float)pSensorNode->info.sensing_end_time, (float)pSensorNode->info.sleeping_interval, (float)pSensorNode->info.relative_sensing_start_time, (float)pSensorNode->info.relative_sensing_end_time, (float)pSensorNode->info.surveillance_restart_time);
#endif

				if(fp) fprintf(fp, "sensor id %d: offset of edge (%s, %s)=%f, initial energy=%f, current energy=%f, state=%d, sensing_start_time=%f, sensing_end_time=%f, sleeping_interval=%f, relative_sensing_start_time=%f, relative_sensing_end_time=%f, surveillance_restart_time=%f", pSensorNode->info.id, u, v, (float)pSensorNode->info.pos_in_Gr.offset, (float)pSensorNode->info.initial_energy, (float)pSensorNode->info.energy, pSensorNode->info.state, (float)pSensorNode->info.sensing_start_time, (float)pSensorNode->info.sensing_end_time, (float)pSensorNode->info.sleeping_interval, (float)pSensorNode->info.relative_sensing_start_time, (float)pSensorNode->info.relative_sensing_end_time, (float)pSensorNode->info.surveillance_restart_time);
			}
			else
			{
#ifdef __DEBUG_LEVEL_SHOW_STATUS_OF_SENSORS_ON_PATH__
				printf("sensor id %d: offset of edge (%s, %s)=%f, initial energy=%f, current energy=%f, state=%d, sensing_start_time=%f, sensing_end_time=%f, sleeping_interval=%f, relative_sensing_start_time=%f, relative_sensing_end_time=%f, surveillance_restart_time=%f", pSensorNode->info.id, v, u, (float)pSensorNode->info.pos_in_Gr.offset, (float)pSensorNode->info.initial_energy, (float)pSensorNode->info.energy, pSensorNode->info.state, (float)pSensorNode->info.sensing_start_time, (float)pSensorNode->info.sensing_end_time, (float)pSensorNode->info.sleeping_interval, (float)pSensorNode->info.relative_sensing_start_time, (float)pSensorNode->info.relative_sensing_end_time, (float)pSensorNode->info.surveillance_restart_time);
#endif

				if(fp) fprintf(fp, "sensor id %d: offset of edge (%s, %s)=%f, initial energy=%f, current energy=%f, state=%d, sensing_start_time=%f, sensing_end_time=%f, sleeping_interval=%f, relative_sensing_start_time=%f, relative_sensing_end_time=%f, surveillance_restart_time=%f", pSensorNode->info.id, v, u, (float)pSensorNode->info.pos_in_Gr.offset, (float)pSensorNode->info.initial_energy, (float)pSensorNode->info.energy, pSensorNode->info.state, (float)pSensorNode->info.sensing_start_time, (float)pSensorNode->info.sensing_end_time, (float)pSensorNode->info.sleeping_interval, (float)pSensorNode->info.relative_sensing_start_time, (float)pSensorNode->info.relative_sensing_end_time, (float)pSensorNode->info.surveillance_restart_time);
			}

			if(pSensorNode->info.state == SENSOR_DIE)
			{				
#ifdef __DEBUG_LEVEL_SHOW_STATUS_OF_SENSORS_ON_PATH__
			  printf(", death_time=%f\n\n", (float)pSensorNode->info.death_time); //print the sensor's death time
#endif
			  if(fp) fprintf(fp, ", death_time=%f\n\n", (float)pSensorNode->info.death_time);
			}
			else
			{
#ifdef __DEBUG_LEVEL_SHOW_STATUS_OF_SENSORS_ON_PATH__
			  printf("\n\n");
#endif
			  if(fp) fprintf(fp, "\n\n");
			}
		} //end of for
#ifdef __DEBUG_LEVEL_SHOW_STATUS_OF_SENSORS_ON_PATH__
		printf("\t => edge (%s, %s) has %d live sensors\n\n", u, v, pEdgeNode->live_sensor_number);
#endif
		if(fp) fprintf(fp, "\t => edge (%s, %s) has %d live sensors\n\n", u, v, count);

		pPathNode = pPathNode->next;
	} //end of while
}

int factorial(int n)
{ //return the n! (i.e., n factorial)
	int num = 1;
	int i;

	if(n == 0)
		return num;
	else if(n < 0)
	{
		printf("factorial(): n(%d) must be equal or greater than 0\n", n);
		exit(1);
	}

	for(i = n; i >= 1; i--)
	{
		num *= i;
	}

	return num;
}

int permutation(int n, int r)
{ //return the number of all possible cases in the permutation nPr
	int num = 1;
	int i;
	int bound; //bound for permutation

	if(n <= 0)
	{
		printf("permutation(): n(%d) must be greater than 0\n", n);
		exit(1);
	}

	if(r <= 0)
	{
		printf("permutation(): r(%d) must be greater than 0\n", r);
		exit(1);
	}
	else if(r > n)
	{
		printf("permutation(): r(%d) must be equal or less than n(%d)\n", r, n);
		exit(1);
	}

	bound = n - (r - 1);
	for(i = n; i >= bound; i--)
	{
		num *= i;
	}

	return num;
}

int combination(int n, int r)
{ //return the number of all possible cases in the combination nCr
	int num; //number of all possible cases

	num = permutation(n, r)/factorial(r);

	return num;
}

boolean is_there_breach_path_based_on_breach_path_matrix(struct_graph_node *Gv, int Gv_size, int ***Dv_breach, int ***Mv_breach, int *matrix_size_for_breach_in_Gv, struct_traffic_table *src_table_for_Gv, struct_traffic_table *dst_table_for_Gv, char *breach_path_src, char *breach_path_dst)
{ //check whether there is a breach path between an arbitrary pair of entrance point and protection point, considering all possible breach paths based on breach path matrix
  boolean result = FALSE;
  int i, j;
  int src_node_id, dst_node_id;

  Floyd_Warshall_Construct_Matrices_For_Breach(Gv, Gv_size, Dv_breach, Mv_breach, matrix_size_for_breach_in_Gv);
  //construct the shortest path weight matrix D and predecessor matrix M for breach path where no-live-sensor edge has 1 and live-sensor-edge has INF.  

  for(i = 0; i < src_table_for_Gv->number; i++)
  {
    src_node_id = atoi(src_table_for_Gv->list[i].vertex);
    for(j = 0; j < dst_table_for_Gv->number; j++)
    {
      dst_node_id = atoi(dst_table_for_Gv->list[j].vertex);

      if((*Dv_breach)[src_node_id-1][dst_node_id-1] < INF)
      { //there is a breach path between nodes i and j
	result = TRUE;
	sprintf(breach_path_src, "%d", src_node_id);
	sprintf(breach_path_dst, "%d", dst_node_id);
	break;
      }
    }    
  }

  return result;
}

boolean is_there_breach_path_based_on_shortest_path_checking(struct_path_table *path_table, struct_traffic_table *src_table_for_Gr, struct_traffic_table *dst_table_for_Gr, edge_queue_t *Er, char *breach_path_src, char *breach_path_dst)
{ //check whether there is a breach path between an arbitrary pair of entrance point and protection point by checking whether there is the shortest path between entrance point and protection point with no live sensor.
	boolean result = FALSE;
	struct_path_node *path_list = NULL; //list of vertices on the path from entrance point (i.e., source) to protection point (i.e., destination)
	char *traffic_source = NULL; //traffic source
	char *traffic_destination = NULL; //traffic destination
	int src = 0; //integer corresponding to traffic source
	int dst = 0; //integer corresponding to traffic destination
	struct_path_node *path_ptr = NULL; //pointer to the current vertex on the path
	char *tail = NULL; //edge's tail
	char *head = NULL; //edge's head
    int i, j; //indicies for for-loops
	boolean flip_flag; //flag to see whether the order of the tail and head of the returned edge_queue_node_t is the same as with (u,v) or not
	edge_queue_node_t* pEdgeQueueNode = NULL; //pointer to edge queue node
	int live_sensor_number_on_path = 0; //number of live sensors on the path from source to destination
	int path_hop_count = 0; //path hop count which is the number of edges in the path

	for(i = 0; i < src_table_for_Gr->number; i++) //for-1
	{
	        //traffic_source = src_table_for_Gr->list[i];
                traffic_source = src_table_for_Gr->list[i].vertex;
		for(j = 0; j < dst_table_for_Gr->number; j++) //for-2
		{
		        //traffic_destination = dst_table_for_Gr->list[j];
                        traffic_destination = dst_table_for_Gr->list[j].vertex;

			if(strcmp(traffic_source, traffic_destination) == 0)
			//make sure that traffic destination is different from traffic source
				continue; 
	
#ifdef __DEBUG_FOR_BREACH_PATH_IN_DETAIL__
			printf("vehicle path: traffic_source(%s) => traffic_destination(%s)\n", traffic_source, traffic_destination);
#endif

			path_list = Make_Path_List(path_table, traffic_source, traffic_destination, &path_hop_count);
			path_ptr = path_list->next; //path_ptr points to traffic_source
			live_sensor_number_on_path = 0;
			while(1)
			{
				path_ptr = path_ptr->next;
				tail = path_ptr->prev->vertex;	
				head = path_ptr->vertex;

#ifdef __DEBUG_FOR_BREACH_PATH_IN_DETAIL__
				printf("edge: (tail, head) = (%s, %s)\n", tail, head);
#endif

				pEdgeQueueNode = LookupEdgeQueue(Er, tail, head, &flip_flag);
				//return the pointer to edge queue node corresponding to the edge consisting of vertices u and v

				/* increase live_sensor_number_on_path by live_sensor_number on the edge of (tail, head) */
				live_sensor_number_on_path += pEdgeQueueNode->live_sensor_number;

				if(strcmp(head, traffic_destination) == 0)
					break;
			}

			/* check whether there is no live sensor on the path or not */
			if(live_sensor_number_on_path == 0)
			{
				strcpy(breach_path_src, traffic_source);
				strcpy(breach_path_dst, traffic_destination);
				return TRUE;
			}

		} //end of for-2
	} //end of for-1

	return result;
}

void store_sensor_location_into_file(edge_queue_t *Er, char* filename)
{ //store sensor location in edge queue into a file called filename
	FILE *fp; //path-table file
	int i, j;
	char *msg = NULL;
	int msg_len; //msg length
	char buf[NAME_SIZE]; //buffer to contain one double number
	const int number_string_length = NUMBER_STRING_LEN; //length of number string
	edge_queue_node_t *pEdgeNode = NULL; //pointer to edge queue node
	location_queue_node_t *pLocation = NULL; //pointer to sensor location on an edge
	int n; //number of sensors on an edge

	/* open a file for sensor location */
	fp = fopen(filename, "w");
	if(!fp)
	{
		fprintf(stderr, "store_sensor_location_into_file(): Error: unable to open file \"%s\"\n", filename);
		exit(1);
	}

	/* store sensor location for each edge */
	pEdgeNode = &(Er->head);
	for(i = 0; i < Er->size; i++)
	{
		pEdgeNode = pEdgeNode->next;
		pLocation = &(pEdgeNode->sensor_location_list.head);

		/* allocate memory to contain sensor location on an edge */
		n = pEdgeNode->sensor_location_list.size;
		msg_len = sizeof(char)*n*number_string_length;
		msg = (char*) calloc(msg_len, sizeof(char));
		assert_memory(msg);

		for(j = 0; j < pEdgeNode->sensor_location_list.size; j++)
		{
			pLocation = pLocation->next;
			sprintf(buf, "%f\n", pLocation->sensor->info.pos_in_Gr.offset);
			strcat(msg, buf);
		}

		fprintf(fp, msg);
#ifdef __DEBUG_LEVEL_0__
		printf(msg);
#endif
		/* release memory */
		free(msg); //if NUMBER_STRING_LEN is small for numbers, this could make heap error here.
	}

	/* close the sensor location file */
	fclose(fp);
}

struct_vehicle_t* register_vehicle(int vehicle_id, char *traffic_source, double arrival_time, struct_traffic_table *dst_table_for_Gr, struct_traffic_table *ap_table_for_Gr, struct parameter *param, struct_path_table *path_table, struct_graph_node *Gr, int Gr_size, edge_queue_t *Er, double **Dr_move, int **Mr_move)
{ //register a new vehicle to vehicle list
	struct_vehicle_t* vehicle = NULL;
	char traffic_destination_buf[NAME_SIZE]; //traffic destination buffer
	char *traffic_destination = traffic_destination_buf; //traffic destination
	int index; //index for dst table entry
	struct_path_node *path_list = NULL; //list of vertices on the path from source to destination
	struct_path_node *ptr = NULL; //pointer to path node
	int src = 0; //integer corresponding to traffic source
	int dst = 0; //integer corresponding to traffic destination
	//int u, v; //u is tail vertex and v is head vertex of an edge
	char *tail_node = NULL; //pointer to tail node name
	char *head_node = NULL; //pointer to head node name
	struct_graph_node *tail_gnode_in_node_array = NULL; //pointer to the graph node corresponding to the tail node in the node array of Gr
	MOVE_TYPE move_type; //movement type for vehicle
	double edge_length = 0; //the length of the edge
	int path_hop_count; //hop count for the path, i.e., the number of edges
	directional_edge_queue_node_t *ptr_directional_edge_node = NULL; //pointer to the directional edge node in directional edge queue DEr for real graph Gr

	vehicle = (struct_vehicle_t*) calloc(1, sizeof(struct_vehicle_t)); 

	if(vehicle == NULL)
	{
		printf("Error: calloc() cannot allocate memory for vehicle\n");
#ifdef __DEBUG_INTERACTIVE_MODE__
                fgetc(stdin);
#endif
		exit(1);
	}

	/** initialize the vehicle instance **/
	vehicle->id = vehicle_id;

	vehicle->type = VEHICLE_CARRIER;
	vehicle->mobility_type = MOBILITY_HYBRID; //Hybrid Mobility of City Section Mobility model and Manhattan Mobility model
	vehicle->role = VEHICLE_ROLE_NON_AP_VEHICLE; //Non-AP vehicle that is not guranteed to pass AP(s

	//vehicle->state = VEHICLE_CHECK;
	vehicle->state = VEHICLE_ARRIVE;
	vehicle->seq = 0;
	vehicle->arrival_time = arrival_time;
	vehicle->restart_time = arrival_time;

	/* allocate the memory for sensor queue */
	vehicle->sensor_queue = (sensor_queue_t*) calloc(1, sizeof(sensor_queue_t)); 
	assert_memory(vehicle->sensor_queue);
	InitQueue((queue_t*)vehicle->sensor_queue, QTYPE_SENSOR); 

	/* allocate the memory for packet queue */
	vehicle->packet_queue = (packet_queue_t*) calloc(1, sizeof(packet_queue_t)); 
	assert_memory(vehicle->packet_queue);
	InitQueue((queue_t*)vehicle->packet_queue, QTYPE_PACKET); 

	/* allocate the memory of a predicted encounter graph */
	TPD_Allocate_Predicted_Encounter_Graph(param, vehicle);

	/* select vehicle's speed using the speed distribution considering vehicle minimum speed and vehicle maximum speed */
	set_vehicle_speed(param, vehicle);

	/* set vehicle's speed standard deviation to param->vehicle_speed_standard_deviation */
	vehicle->speed_standard_deviation = param->vehicle_speed_standard_deviation;

	/* set vehicle's unit_length_mean_travel_time and unit_length_travel_time_standard_deviation */
#if TPD_ACTUAL_VEHICLE_SPEED_USE_FLAG /* [ */
	GSL_Vanet_Compute_TravelTime_And_Deviation_By_VehicleActualSpeed(param, param->vehicle_unit_length, &(vehicle->unit_length_mean_travel_time), &(vehicle->unit_length_travel_time_standard_deviation), vehicle->speed, vehicle->speed_standard_deviation);
#else
	GSL_Vanet_Compute_TravelTime_And_Deviation(param, param->vehicle_unit_length, &(vehicle->unit_length_mean_travel_time), &(vehicle->unit_length_travel_time_standard_deviation));
#endif

	/* compute the variance of the travel time for the unit length */
	vehicle->unit_length_travel_time_variance = pow(vehicle->unit_length_travel_time_standard_deviation, 2); 
				
	/* choose the destination and the path list with at least minimum hop count according to path length distribution {EQUAL, NORMAL} */
	do
	{
		if(vehicle->role == VEHICLE_ROLE_AP_VEHICLE)
		{
		  if(IsVertexInTrafficTable(ap_table_for_Gr, traffic_source) == FALSE)
		  {
		    index = smpl_random(0, ap_table_for_Gr->number-1);
		    strcpy(traffic_destination, ap_table_for_Gr->list[index].vertex);
		  }
		  else
                  {
		    index = smpl_random(0, dst_table_for_Gr->number-1);
                    strcpy(traffic_destination, dst_table_for_Gr->list[index].vertex);
                  }
		}
		else
                {
		  index = smpl_random(0, dst_table_for_Gr->number-1);
                  strcpy(traffic_destination, dst_table_for_Gr->list[index].vertex);
                }

		if(strcmp(traffic_source, traffic_destination) == 0)
		//make sure that traffic destination is different from traffic source
		  continue; 

	        if(param->vehicle_path_length_distribution == EQUAL)
	        {
				path_list = Make_Path_List(path_table, traffic_source, traffic_destination, &path_hop_count);
	          //path_list = Make_Path_List_Before_The_Closest_Protection_Point(path_table, traffic_source, traffic_destination, dst_table_for_Gr);
	        }
	        else
	        {
				src = atoi(traffic_source);
				dst = atoi(traffic_destination);
				path_list = Random_Path_Make_Path_List(src, dst, Gr, Gr_size, Dr_move, Mr_move, param, &path_hop_count);
		  //path_list = Random_Path_Make_Path_List_Before_The_Closest_Protection_Point(src, dst, Gr, Gr_size, Dr_move, Mr_move, param, dst_table_for_Gr);
	        }

		if(path_hop_count >= param->vehicle_path_minimum_hop_count)
		//let the path_hop_count have at least a minimum hop count
		  break; 
		else
		{
		  /* delete vehicle's path-list */
		  Free_Path_List(path_list);
		  path_list = NULL;
		}
	} while(1);				

	/* set path_list, path_hop_count, path_ptr and path_current_hop */
	vehicle->path_list = path_list;

	/* compute the mean and standard deviation of the arrival time for each path node along path_list */	
#if TPD_ACTUAL_VEHICLE_SPEED_USE_FLAG /* [ */
	compute_arrival_time_mean_and_standard_deviation_for_path_node_by_vehicle_actual_speed(arrival_time, param, path_list, vehicle); //compute the mean and standard deviation of the arrival time for each path node along path_list by vehicle's actual speed
#else	
	compute_arrival_time_mean_and_standard_deviation_for_path_node(arrival_time, param, path_list); //compute the mean and standard deviation of the arrival time for each path node along path_list by param's vehicle speed
#endif /* ] */

	vehicle->path_hop_count = path_hop_count; //set up path_hop_count for path_list
	vehicle->path_ptr = path_list->next;
	vehicle->path_current_hop = 0; //reset path_current_hop to zero
	vehicle->path_current_edge_offset = 0; //set path_current_edge_offset to zero

	/* set the initial position, current position and movement type */
	tail_node = vehicle->path_ptr->vertex;
	head_node = vehicle->path_ptr->next->vertex;
	vehicle->current_pos_in_Gr.eid = vehicle->init_pos_in_Gr.eid = FastGetEdgeID_MoveType(Gr, tail_node, head_node, &move_type, &edge_length, &ptr_directional_edge_node);
	//vehicle->current_pos_in_Gr.eid = vehicle->init_pos_in_Gr.eid = GetEdgeID_MoveType(Er, tail_node, head_node, &move_type, &edge_length);
	vehicle->move_type = move_type;
	vehicle->edge_length = edge_length;
	if(move_type == MOVE_FORWARD)
		vehicle->current_pos_in_Gr.offset = vehicle->init_pos_in_Gr.offset = 0;
	else if(move_type == MOVE_BACKWARD)
		vehicle->current_pos_in_Gr.offset = vehicle->init_pos_in_Gr.offset = edge_length;
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

	/* compute the position update vector to update the vehicle's geometric position by movement time, such as STEP_TIME */
	get_position_update_vector(vehicle, Gr, Gr_size);

	/* set path_length from the source to the closest protection point towards the destination */
	/*
	ptr = vehicle->path_ptr;
	vehicle->path_length = 0;
	while(!IsEndOfTravel(vehicle->path_list, ptr))
	{
	  tail_node = ptr->vertex;
	  head_node = ptr->next->vertex;
	  u = atoi(tail_node) - 1;
	  v = atoi(head_node) - 1;
	  edge_length = Dr_move[u][v];
	  vehicle->path_length += edge_length;
	  ptr = ptr->next;
	}
	*/
		
	/* insert this vehicle into the tail of vehicle_list */
	vehicle_insert(vehicle);

	return vehicle;
}

void update_vehicle_trajectory(struct_vehicle_t* vehicle, double arrival_time, struct_traffic_table *dst_table_for_Gr, struct_traffic_table *ap_table_for_Gr, struct parameter *param, struct_path_table *path_table, struct_graph_node *Gr, int Gr_size, edge_queue_t *Er, double **Dr_move, int **Mr_move)
{ //update the vehicle's trajectory given the source node in the real graph
	char traffic_source_buf[NAME_SIZE]; //traffic source buffer  
	char traffic_destination_buf[NAME_SIZE]; //traffic destination buffer
	char *traffic_source = traffic_source_buf; //traffic source
	char *traffic_destination = traffic_destination_buf; //traffic destination
	int index; //index for dst table entry
	struct_path_node *path_list = NULL; //list of vertices on the path from source to destination
	struct_path_node *ptr = NULL; //pointer to path node
	int src = 0; //integer corresponding to traffic source
	int dst = 0; //integer corresponding to traffic destination
	//int u, v; //u is tail vertex and v is head vertex of an edge
	char *tail_node = NULL; //pointer to tail node name
	char *head_node = NULL; //pointer to head node name
	struct_graph_node *tail_gnode_in_node_array = NULL; //pointer to the graph node corresponding to the tail node in the node array of Gr
	MOVE_TYPE move_type; //movement type for vehicle
	double edge_length = 0; //the length of the edge
	int path_hop_count; //hop count for the path, i.e., the number of edges
	directional_edge_queue_node_t *ptr_directional_edge_node = NULL; //pointer to the directional edge node in directional edge queue DEr for real graph Gr

	/** @for debugging */
	//if(vehicle == 0xd28e540 && arrival_time >= 91.92)
	//  printf("update_vehicle_trajectory(): DEBUGGING!\n");
	/*****/

	/* set vehicle's destination to vehicle's source */
	strcpy(traffic_source, vehicle->path_ptr->vertex);

	/* delete vehicle's path-list */
	Free_Path_List(vehicle->path_list);
	vehicle->path_list = NULL;
	vehicle->path_ptr = NULL;

	/** initialize the vehicle instance **/
	//vehicle->state = VEHICLE_CHECK;
	vehicle->state = VEHICLE_RESTART;
	vehicle->seq = 0;
	vehicle->restart_time = arrival_time;
	//InitQueue((queue_t*) &(vehicle->Q), QTYPE_PACKET);

	/* choose the destination and the path list with at least minimum hop count according to path length distribution {EQUAL, NORMAL} */
	do
	{
		if(vehicle->role == VEHICLE_ROLE_AP_VEHICLE)
		{
		  if(IsVertexInTrafficTable(ap_table_for_Gr, traffic_source) == FALSE)
		  {
		    index = smpl_random(0, ap_table_for_Gr->number-1);
		    strcpy(traffic_destination, ap_table_for_Gr->list[index].vertex);
		  }
		  else
                  {
		    index = smpl_random(0, dst_table_for_Gr->number-1);
                    strcpy(traffic_destination, dst_table_for_Gr->list[index].vertex);
                  }
		}
		else
                {
		  index = smpl_random(0, dst_table_for_Gr->number-1);
                  strcpy(traffic_destination, dst_table_for_Gr->list[index].vertex);
                }

		if(strcmp(traffic_source, traffic_destination) == 0)
		//make sure that traffic destination is different from traffic source
		  continue; 

	        if(param->vehicle_path_length_distribution == EQUAL)
	        {
	          path_list = Make_Path_List(path_table, traffic_source, traffic_destination, &path_hop_count);
	          //path_list = Make_Path_List_Before_The_Closest_Protection_Point(path_table, traffic_source, traffic_destination, dst_table_for_Gr);
	        }
	        else
	        {
		  src = atoi(traffic_source);
		  dst = atoi(traffic_destination);
		  path_list = Random_Path_Make_Path_List(src, dst, Gr, Gr_size, Dr_move, Mr_move, param, &path_hop_count);
		  //path_list = Random_Path_Make_Path_List_Before_The_Closest_Protection_Point(src, dst, Gr, Gr_size, Dr_move, Mr_move, param, dst_table_for_Gr);
	        }

		if(path_hop_count >= param->vehicle_path_minimum_hop_count)
		//let the path_hop_count have at least a minimum hop count
		  break; 
		else
		{
		  /* delete vehicle's path-list */
		  Free_Path_List(path_list);
                  path_list = NULL;
		}
	} while(1);				

	/* set path_list, path_hop_count, path_ptr and path_current_hop */
	vehicle->path_list = path_list;

 /* compute the mean and standard deviation of the arrival time for each path node along path_list */
#if TPD_ACTUAL_VEHICLE_SPEED_USE_FLAG /* [ */
	compute_arrival_time_mean_and_standard_deviation_for_path_node_by_vehicle_actual_speed(arrival_time, param, path_list, vehicle); //compute the mean and standard deviation of the arrival time for each path node along path_list by vehicle's actual speed
#else   
	compute_arrival_time_mean_and_standard_deviation_for_path_node(arrival_time, param, path_list); //compute the mean and standard deviation of the arrival time for each path node along path_list by param's vehicle speed
#endif /* ] */

	vehicle->path_hop_count = path_hop_count; //set up path_hop_count for path_list
	vehicle->path_ptr = path_list->next;
	vehicle->path_current_hop = 0; //reset path_current_hop to zero
	vehicle->path_current_edge_offset = 0; //set path_current_edge_offset to zero

	/* set the initial position, current position and movement type */
	tail_node = vehicle->path_ptr->vertex;
	head_node = vehicle->path_ptr->next->vertex;
	vehicle->current_pos_in_Gr.eid = vehicle->init_pos_in_Gr.eid = FastGetEdgeID_MoveType(Gr, tail_node, head_node, &move_type, &edge_length, &ptr_directional_edge_node);
	vehicle->move_type = move_type;
	vehicle->edge_length = edge_length;
	if(move_type == MOVE_FORWARD)
		vehicle->current_pos_in_Gr.offset = vehicle->init_pos_in_Gr.offset = 0;
	else if(move_type == MOVE_BACKWARD)
		vehicle->current_pos_in_Gr.offset = vehicle->init_pos_in_Gr.offset = edge_length;
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

	/* compute the position update vector to update the vehicle's geometric position by movement time, such as STEP_TIME */
	get_position_update_vector(vehicle, Gr, Gr_size);

	/* set path_length from the source to the closest protection point towards the destination */
	/*
	ptr = vehicle->path_ptr;
	vehicle->path_length = 0;
	while(!IsEndOfTravel(vehicle->path_list, ptr))
	{
	  tail_node = ptr->vertex;
	  head_node = ptr->next->vertex;
	  u = atoi(tail_node) - 1;
	  v = atoi(head_node) - 1;
	  edge_length = Dr_move[u][v];
	  vehicle->path_length += edge_length;
	  ptr = ptr->next;
	}
	*/
}

void register_all_vehicle_movement(parameter_t *param, double registration_time, struct_graph_node *G)
{ /* register all vehicles' movement into the corresponding vehicle_movement_list of the directional edge
     pointed by a graph node in G where each vehicle is moving */
	struct struct_vehicle* ptr;

	if(&vehicle_list == vehicle_list.next)
		return; //there is no vehicle node in the vehicle list

	for(ptr = vehicle_list.next; ptr != &vehicle_list; ptr = ptr->next)
	{
	  if(ptr->state != VEHICLE_ESCAPE) //for the vehicle with state VEHICLE_ESCAPE, we do not register it since it is leaving the road network
	    register_vehicle_movement(param, ptr, registration_time, G); //register vehicle's movement into vehicle_movement_list in the directional edge
	}
}

void update_all_vehicle_edd(double update_time, parameter_t *param, struct_graph_node *G, int G_size, struct_traffic_table *ap_table)
{ //update all vehicles' EDDs using each vehicle's offset in directional edge along with real graph G and AP table 
	struct struct_vehicle* ptr;

	if(&vehicle_list == vehicle_list.next)
		return; //there is no vehicle node in the vehicle list

	for(ptr = vehicle_list.next; ptr != &vehicle_list; ptr = ptr->next)
	{
	  if(ptr->state != VEHICLE_ESCAPE) //for the vehicle with state VEHICLE_ESCAPE, we do not update its EDD since it is leaving the road network
	    VADD_Update_Vehicle_EDD(update_time, param, ptr, G, G_size, ap_table);
	    //compute vehicle's EDD using the vehicle's offset in directional edge along with real graph G and AP table 
	}
}

void update_all_vehicle_edd_and_edd_sd(double update_time, parameter_t *param, struct_graph_node *G, int G_size, struct_traffic_table *ap_table)
{ //update all vehicles' EDDs and EDD_SDs using each vehicle's offset in directional edge along with real graph G and AP table 
	struct struct_vehicle* ptr;

	if(&vehicle_list == vehicle_list.next)
		return; //there is no vehicle node in the vehicle list

	for(ptr = vehicle_list.next; ptr != &vehicle_list; ptr = ptr->next)
	{
	  /**@for debugging */
	  //if(ptr->id == 90)
	  //  printf("at time=%.1f, vehicle(id=%d) is tracked\n", (float)update_time, ptr->id);
	  /******************/

	  if(ptr->state != VEHICLE_ESCAPE) //for the vehicle with state VEHICLE_ESCAPE, we do not update its EDD since it is leaving the road network
	    VADD_Update_Vehicle_EDD_And_EDD_SD(update_time, param, ptr, G, G_size, ap_table);
	    //compute vehicle's EDD and EDD_SD using the vehicle's offset in directional edge along with real graph G and AP table 
	}
}

void register_vehicle_movement(parameter_t *param, struct_vehicle_t* vehicle, double registration_time, struct_graph_node *G)
{ //register the vehicle's movement into the vehicle_movement_list of the directional edge pointed by a graph node in G
  char tail_node[NAME_SIZE];  //tail node of a directional edge
  char head_node[NAME_SIZE];  //head node of a directional edge
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to a directional edge queue node
  vehicle_movement_queue_node_t movement_node; //vehicle movement queue node
  vehicle_movement_queue_node_t *pMoveNode = NULL; //pointer to vehicle movement queue node
  vehicle_movement_queue_t *Q = NULL; //pointer to the vehicle movement queue

  /**@for debugging */
  //if(vehicle->id == 1795 && registration_time >= 3600.0)
  //  printf("at time=%.1f, vehicle(id=%d) is tracked!\n", (float)registration_time, vehicle->id);
  /******************/

  /** initialize movement_node */
  memset(&movement_node, 0, sizeof(movement_node));

  /* check whether flag_vehicle_movement_queue_registration is set to FALSE */
  if(vehicle->flag_vehicle_movement_queue_registration == FALSE) //if-1
  {
    strcpy(tail_node, vehicle->path_ptr->vertex); //tail node for the directional edge where the vehicle is moving
    strcpy(head_node, vehicle->path_ptr->next->vertex); //head node for the directional edge where the vehicle is moving
    pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
    if(pEdgeNode == NULL)
    {
      printf("register_vehicle_movement(): pEdgeNode for <%s,%s> is NULL\n", tail_node, head_node);
      exit(1);
    }

    /* determine vehicle's offset in the directional edge
       NOTE: vehicle->move_type is for the moving direction in the undirectional edge of real graph Gr     
    */
    
    if(vehicle->move_type == MOVE_FORWARD) //if-1.1
    {
      movement_node.offset = vehicle->current_pos_in_Gr.offset;
    } //end of if-1.1
    else if(vehicle->move_type == MOVE_BACKWARD) //else-if-1.2
    {
      movement_node.offset = vehicle->edge_length - vehicle->current_pos_in_Gr.offset;
    } //end of else-if-1.2
    else //else-1.3
    {
      printf("register_vehicle_movement(): vehicle->move_type(%d) is unknown!\n", vehicle->move_type);
      exit(1);
    } //end of else-1.3

    movement_node.vid = vehicle->id;
    movement_node.vnode = vehicle; //pointer to vehicle node
    movement_node.speed = vehicle->speed;
    movement_node.move_type = MOVE_FORWARD;
    movement_node.registration_time = registration_time;
    movement_node.arrival_time = registration_time - movement_node.offset/vehicle->speed; //assume constant vehicle speed
    movement_node.departure_time = registration_time + (vehicle->edge_length - movement_node.offset)/vehicle->speed; //assume constant vehicle speed
    movement_node.order = -1;

    pMoveNode = (vehicle_movement_queue_node_t*)Enqueue((queue_t*)&(pEdgeNode->vehicle_movement_list), (queue_node_t*)&movement_node);

    vehicle->ptr_vehicle_movement_queue_node = pMoveNode; //let vehicle's ptr_vehicle_movement_queue_node to point to the corresponding vehicle movement queue node in the vehicle_movement_list of the edge where the vehicle is moving

    /* set flag_vehicle_movement_queue_registration to TRUE */
    vehicle->flag_vehicle_movement_queue_registration = TRUE;

    /* update vehicle's current position in vehicle's digraph */
	strcpy(vehicle->current_pos_in_digraph.tail_node, tail_node);
	strcpy(vehicle->current_pos_in_digraph.head_node, head_node);
    vehicle->current_pos_in_digraph.offset = movement_node.offset;
    vehicle->current_pos_in_digraph.enode = pEdgeNode;
    vehicle->current_pos_in_digraph.eid = pEdgeNode->eid;

    /* compute edge arrival time and edge departure time */
    vehicle->edge_arrival_time = movement_node.arrival_time; //time when it arrives at an edge
    vehicle->edge_departure_time = movement_node.departure_time; //time when it departs from an edge

    /* perform Average Convoy Length (ACL) measurement from pEdgeNode->tail_node */
    if(param->vehicle_vanet_acl_measurement_flag) //if-2.1
    {
      process_acl_convoy_for_vehicle_arrival(param, vehicle, registration_time, pEdgeNode);
    } //end of if-2.1
  } //end of if-1
  else //else-2
  {
    printf("register_vehicle_movement(): Error: at time=%.1f, vehicle(id=%d) has already been registered in another vehicle movement queue for undirectional edge of eid=%d with move_type=%d\n", (float)registration_time, vehicle->id, vehicle->current_pos_in_Gr.eid, vehicle->move_type);
    exit(1);
  } //end of else-2
}

void delete_vehicle_movement(parameter_t *param, struct_vehicle_t* vehicle, double departure_time, struct_graph_node *G)
{ //delete the vehicle's movement from the vehicle_movement_list of the directional edge pointed by a graph node in G with vehicle id
  char tail_node[NAME_SIZE];  //tail node of a directional edge
  char head_node[NAME_SIZE];  //head node of a directional edge
  directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to a directional edge queue node
  
  /* check whether flag_vehicle_movement_queue_registration is set to TRUE */
  if(vehicle->flag_vehicle_movement_queue_registration)
  {
    strcpy(tail_node, vehicle->path_ptr->vertex); //tail node for the directional edge where the vehicle is moving
    strcpy(head_node, vehicle->path_ptr->next->vertex); //head node for the directional edge where the vehicle is moving
    pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
    if(pEdgeNode == NULL)
    {
      printf("delete_vehicle_movement(): pEdgeNode for <%s,%s> is NULL\n", tail_node, head_node);
      exit(1);
    }

    /* perform Average Convoy Length (ACL) measurement from pEdgeNode->tail_node */
    if(param->vehicle_vanet_acl_measurement_flag)
    {
      process_acl_convoy_for_vehicle_departure(param, vehicle, departure_time, pEdgeNode);
    }

    /* delete vehicle movement queue node corresponding to vehicle */
    DeleteVehicleMovementWithVID(&(pEdgeNode->vehicle_movement_list), vehicle->id);

    /* set vehicle's ptr_vehicle_movement_queue_node to NULL */
    vehicle->ptr_vehicle_movement_queue_node = NULL;

    /* set flag_vehicle_movement_queue_registration to FALSE */
    vehicle->flag_vehicle_movement_queue_registration = FALSE;
  }
}

void process_acl_convoy_for_vehicle_arrival(parameter_t *param, struct_vehicle_t *vehicle, double registration_time, directional_edge_queue_node_t *edge)
{ //process the vehicle convoy for a new vehicle arrival into the directional edge in order to compute the ACL for the directional edge

  /**@ for debugging */
  //if(edge->eid != 1)
  //  return;
  /*******************/

  /* compute the vehicle interarrival time and update the last arrival time */
  if(edge->acl_vehicle_arrival_number == 0) //if-1
  {
    edge->acl_interarrival_time = INF;
  } //end of if-1
  else //else-2
  {
    edge->acl_interarrival_time = registration_time - edge->acl_convoy_tail_arrival_time;
  } //end of else-2

  /**@for debugging */
  //if(vehicle->id == 0x1c && registration_time >= 126)
  //  printf("process_acl_convoy_for_vehicle_arrival(): at time=%.1f, vid=%d", (float)registration_time, vehicle->id);

  /**@ for debugging for the update discrepancy */
  //if((edge->acl_interarrival_time > edge->acl_convoy_threshold) && (edge->acl_interarrival_time - edge->acl_convoy_threshold <= param->vehicle_step_time))
  //  printf("process_acl_convoy_for_vehicle_arrival(): at time=%.1f, vid=%d, difference between interarrival time and threshold=%.1f\n", (float)registration_time, vehicle->id, (float)(edge->acl_interarrival_time - edge->acl_convoy_threshold));

  /******************/

  /* update convoy information according to the interarrival time */
  if(edge->acl_convoy_vehicle_number > 0) //if-3
  { //the case where the convoy already exists

    if((edge->acl_measurement_start_flag == FALSE) && (edge->acl_interarrival_time > edge->acl_convoy_threshold)) //if-3.1
    { //the vehicle is out of the current convoy tail vehicle's communication range
      /* set up the new convoy's information */
      edge->acl_convoy_head_vehicle = vehicle; //set the convoy head vehicle to vehicle
      edge->acl_convoy_head_arrival_time = registration_time; //set the convoy head arrival time to registration time
      edge->acl_convoy_tail_vehicle = vehicle; //set the convoy tail vehicle to vehicle
      edge->acl_convoy_tail_arrival_time = registration_time; //set the convoy tail arrival time to registration time
      edge->acl_convoy_start_time = registration_time;
      edge->acl_convoy_end_time = 0;
      edge->acl_convoy_start_length = 0;
      edge->acl_convoy_end_length = 0;
      edge->acl_convoy_vehicle_number = 1;    
    } //end of if-3.1
    else if((edge->acl_measurement_start_flag == TRUE) && (edge->acl_interarrival_time > edge->acl_convoy_threshold)) //if-3.2
    { //the vehicle is out of the current convoy tail vehicle's communication range

      /* update convoy head and convoy start time and convoy start length during edge->acl_interarrival_time */
      //edge->acl_convoy_head_vehicle = Find_Vehicle_Farthest_Towards_Edge_Head_Node_In_Connected_Network_Component(param, edge->acl_convoy_tail_vehicle);  
      //edge->acl_convoy_head_arrival_time = edge->acl_convoy_head_vehicle->edge_arrival_time; //set the convoy head arrival time to the convoy head's edge arrival time

      /* set the convoy start time */
      //if(edge->acl_convoy_head_vehicle == NULL)
      //  edge->acl_convoy_start_time = edge->acl_convoy_head_departure_time; //the case where the original convoy head departed from the edge and the following didn't arrive at the head of the edge yet  

      //edge->acl_convoy_start_length = edge->acl_convoy_head_vehicle->current_pos_in_digraph.offset - edge->acl_convoy_head_vehicle->speed * (registration_time - edge->acl_convoy_start_time);

      /* compute convoy area for ACL */
      edge->acl_convoy_end_time = edge->acl_convoy_tail_arrival_time + edge->acl_convoy_threshold;
      edge->acl_convoy_duration = edge->acl_convoy_end_time - edge->acl_convoy_start_time;
      edge->acl_convoy_end_length = edge->acl_convoy_start_length + edge->acl_convoy_head_vehicle->speed * edge->acl_convoy_duration;

      /**@for debugging */
      if(edge->acl_convoy_end_length > edge->weight + ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
        printf("process_acl_convoy_for_vehicle_arrival(): at time=%.1f, for vehicle(id=%d), edge->acl_convoy_end_length(%f) > edge->weight(%f)\n", (float)registration_time, vehicle->id, (float)edge->acl_convoy_end_length, (float)edge->weight);
      /******************/

      edge->acl_convoy_rectangle_height = edge->acl_convoy_start_length;
      edge->acl_convoy_triangle_height = edge->acl_convoy_end_length - edge->acl_convoy_rectangle_height;
      edge->acl_convoy_area += edge->acl_convoy_rectangle_height * edge->acl_convoy_duration + edge->acl_convoy_triangle_height * edge->acl_convoy_duration / 2;

      /* update the end of ACL measurement for this update of ACL convoy area */
      edge->acl_measurement_end_time = registration_time;

      /* set up the new convoy's information */
      edge->acl_convoy_head_vehicle = vehicle; //set the convoy head vehicle to vehicle
      edge->acl_convoy_head_arrival_time = registration_time; //set the convoy head arrival time to registration time
      edge->acl_convoy_tail_vehicle = vehicle; //set the convoy tail vehicle to vehicle
      edge->acl_convoy_tail_arrival_time = registration_time; //set the convoy tail arrival time to registration time
      edge->acl_convoy_start_time = registration_time;
      edge->acl_convoy_end_time = 0;
      edge->acl_convoy_start_length = 0;
      edge->acl_convoy_end_length = 0;
      edge->acl_convoy_vehicle_number = 1;    
    } //end of if-3.2
    else if(edge->acl_interarrival_time <= edge->acl_convoy_threshold) //else-3.3
    { //the vehicle becomes a new convoy tail of the current convoy
      edge->acl_convoy_tail_vehicle = vehicle; //set the convoy tail vehicle to vehicle
      edge->acl_convoy_tail_arrival_time = registration_time; //set the convoy tail arrival time to registration time
      edge->acl_convoy_vehicle_number++;
    } //end of else-3.3
  } //end of if-3
  else //else-4
  { //this vehicle becomes the convoy head
    edge->acl_convoy_head_vehicle = vehicle; //set the convoy head vehicle to vehicle
    edge->acl_convoy_head_arrival_time = registration_time; //set the convoy head arrival time to registration time
    edge->acl_convoy_tail_vehicle = vehicle; //set the convoy tail vehicle to vehicle
    edge->acl_convoy_tail_arrival_time = registration_time; //set the convoy tail arrival time to registration time
    edge->acl_convoy_start_time = registration_time;
    edge->acl_convoy_end_time = 0;
    edge->acl_convoy_start_length = 0;
    edge->acl_convoy_end_length = 0;
    edge->acl_convoy_vehicle_number = 1;
  } //end of else-4

  edge->acl_vehicle_arrival_number++; //increase the vehicle arrival number
}

void process_acl_convoy_for_vehicle_departure(parameter_t *param, struct_vehicle_t *vehicle, double departure_time, directional_edge_queue_node_t *edge)
{ //process the vehicle convoy for a new vehicle departure from the directional edge in order to compute the ACL for the directional edge
  vehicle_movement_queue_t *Q = &(edge->vehicle_movement_list); //vehicle movement queue
  double interval = 0; //interval between this departure time and the convoy tail arrival time 
 
  /**@ for debugging */
  //if(edge->eid != 1)
  //  return;

  //if(vehicle->state_time > 72 && vehicle->id == 25)
  //  printf("process_acl_convoy_for_vehicle_departure(): at time=%f\n", vehicle->state_time);

  /*******************/

  /* check whether vehicle is the stationary vehicle with id=1; in this case, we ignore this vehicle's departure since the departure time is zero */
  if((param->vehicle_vanet_stationary_vehicle_flag == TRUE) && (vehicle->id == 1))
    return;
 
  /* if vehicle is convoy head, then update the ACL convoy information */
  if(edge->acl_convoy_vehicle_number == 0 || edge->acl_convoy_head_vehicle == NULL) //if-1
  { //this vehicle has belonged to the diminished convoy; note that in the second condition, acl_convoy_vehicle_number > 0, but there is no following vehicle with less offset than the previous convoy head vehicle

    /* check ACL measurement start flag */
    if(edge->acl_measurement_start_flag == FALSE)
    {
      /* This is the actual start of ACL measurement. */
      edge->acl_measurement_start_time = departure_time;

      /* set the ACL measurement start flag to TRUE */
      edge->acl_measurement_start_flag = TRUE;
    }

    return;
  } //end of if-1
  else if(vehicle->id == edge->acl_convoy_head_vehicle->id) //else-if-2
  {
    /* compute the interval between this departure time and the convoy tail arrival time */
    interval = departure_time - edge->acl_convoy_tail_arrival_time;

    /* process the first departure: set the start time of ACL measurement for the first vehicle's departure: the first departure may be zero for vehicle's id = 0 */
    if(edge->acl_measurement_start_flag == FALSE) //if-2.1
    {
      /* This is the actual start of ACL measurement:
         we do not compute the average area for this first departure. 
      */
      edge->acl_measurement_start_time = departure_time;

      /* set the ACL measurement start flag to TRUE */
      edge->acl_measurement_start_flag = TRUE;
    } //end of if-2.1
    else //else-2.2
    { //process the other departures except for the first one
      ///* compute the interval between this departure time and the convoy tail arrival time */
      //interval = departure_time - edge->acl_convoy_tail_arrival_time;
    
      /* compute convoy end time */
      if(interval > edge->acl_convoy_threshold) //if-2.2.1 
      { //since there is no vehicle arrival after the convoy tail during the interval, the convoy has been disconnected from the edge's tail node
        edge->acl_convoy_end_time = edge->acl_convoy_tail_arrival_time + edge->acl_convoy_threshold;
      } //end of if-2.2.1
      else //else-2.2.2
      { //During the interval, the convoy is still connected to the edge's tail node
        edge->acl_convoy_end_time = departure_time;
      } //end of else-2.2.2

      /* compute convoy area for ACL */
      edge->acl_convoy_duration = edge->acl_convoy_end_time - edge->acl_convoy_start_time;
      edge->acl_convoy_end_length = edge->acl_convoy_start_length + edge->acl_convoy_head_vehicle->speed * edge->acl_convoy_duration;

      /**@for debugging */
      if(edge->acl_convoy_end_length > edge->weight + ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
        printf("process_acl_convoy_for_vehicle_departure(): at time=%.1f, for vehicle(id=%d), edge->acl_convoy_end_length(%f) > edge->weight(%f)\n", (float)departure_time, vehicle->id, (float)edge->acl_convoy_end_length, (float)edge->weight);
      /******************/

      edge->acl_convoy_rectangle_height = edge->acl_convoy_start_length;
      edge->acl_convoy_triangle_height = edge->acl_convoy_end_length - edge->acl_convoy_rectangle_height;
      edge->acl_convoy_area += edge->acl_convoy_rectangle_height * edge->acl_convoy_duration + edge->acl_convoy_triangle_height * edge->acl_convoy_duration / 2;      

      /* update the end of ACL measurement for this update of ACL convoy area */
      edge->acl_measurement_end_time = departure_time;
    } //end of else-2.2

    /* update the convoy information */
    if(interval > edge->acl_convoy_threshold) //if-2.3
    { //this case is that there is no vehicle arrival after the convoy tail during the interval, so the convoy has been disconnected from the edge's tail node
      edge->acl_convoy_head_vehicle = NULL; //set the convoy head vehicle to NULL
      //edge->acl_convoy_head_arrival_time = 0; //set the convoy head arrival time to zero
      edge->acl_convoy_head_departure_time = departure_time; //set the convoy head arrival time to zero
      //edge->acl_convoy_tail_vehicle = NULL; //set the convoy tail vehicle to NULL
      //edge->acl_convoy_tail_arrival_time = 0; //set the convoy tail arrival time to zero
      edge->acl_convoy_start_time = 0;
      edge->acl_convoy_end_time = 0;
      edge->acl_convoy_start_length = 0; //set the convoy start length to zero
      edge->acl_convoy_end_length = 0;
      edge->acl_convoy_vehicle_number = 0;
    } //end of if-2.3
    else //else-2.4
    { //During the interval, the convoy is still connected to the edge's tail node
      
      edge->acl_convoy_head_vehicle = Find_Following_Vehicle_Within_Communication_Range_On_Directional_Edge(param, vehicle, &(edge->acl_convoy_vehicle_number)); //set the convoy head vehicle to the next following vehicle
      //@NOTE: we need to consider that more than one vehicle have the same offset. In this case, we need to adjust acl_convoy_vehicle_number appropriately

      //edge->acl_convoy_head_vehicle = Q->head.next->next->vnode; //set the convoy head vehicle to vehicle
      //@NOTE: we assume that vehicle speed is constant, so for variable speed, we need to modify the above line to select the following vehicle

      /**@for debugging */
      if(edge->acl_convoy_head_vehicle == NULL && edge->acl_convoy_vehicle_number > 1)
      {
	printf("process_acl_convoy_for_vehicle_departure(): edge->acl_convoy_head_vehicle == NULL && edge->acl_convoy_vehicle_number(%d) > 1\n", edge->acl_convoy_vehicle_number);
      }
      /******************/

      if(edge->acl_convoy_head_vehicle != NULL) //if-2.4.1
      {
        edge->acl_convoy_head_arrival_time = edge->acl_convoy_head_vehicle->edge_arrival_time; //set the convoy head arrival time to the convoy head's edge arrival time
	edge->acl_convoy_head_departure_time = departure_time; //set the convoy head departure time to the convoy head's edge departure time as the previous head's departure time
	//edge->acl_convoy_head_departure_time = 0; //set the convoy head departure time to zero in order to indicate that the convoy head has not arrived at the edge head yet
        edge->acl_convoy_start_time = departure_time; //set the convoy's start time to departure_time
        edge->acl_convoy_start_length = (departure_time - edge->acl_convoy_head_vehicle->edge_arrival_time) * edge->acl_convoy_head_vehicle->speed; //set the convoy start length to the new convoy head's offset on the directional edge
        //edge->acl_convoy_start_length = edge->acl_convoy_head_vehicle->current_pos_in_digraph.offset; //set the convoy start length to the new convoy head's offset on the directional edge	

        edge->acl_convoy_end_time = 0;      
        edge->acl_convoy_end_length = 0;
        edge->acl_convoy_vehicle_number--; //decrease the number of convoy vehicles
      } //end of if-2.4.1
      else //else-2.4.2
      {
        //edge->acl_convoy_head_arrival_time = 0; //set the convoy head arrival time to zero
	edge->acl_convoy_head_departure_time = departure_time; //set the convoy head departure time to departure_time
        edge->acl_convoy_start_time = 0;
        edge->acl_convoy_start_length = 0; //set the convoy start length to zero

        edge->acl_convoy_end_time = 0;      
        edge->acl_convoy_end_length = 0;
        edge->acl_convoy_vehicle_number = 0; //set the number of convoy vehicles to zero

      } //end of else-2.4.2
    } //end of else-2.4
  } //end of else-if-2
}

double get_average_length_of_shortest_paths(double** D, int n, struct_traffic_table *src_table, struct_traffic_table *dst_table)
{ //compute the average length of the shortest paths between the arbitrary pair of source and destination

	int u, v; //vertices
	double sum_of_path_lengths = 0; //sum of the shortest path lengths
	int number_of_pairs = 0; //number of pairs of shortest path
	double average_length = 0; //average length of the shortest paths
	int i, j; //indices of for-loops

	for(i = 0; i < src_table->number; i++)
	{
	        //u = atoi(src_table->list[i]);
                u = atoi(src_table->list[i].vertex);
		for(j = 0; j < dst_table->number; j++)
		{
		        //v = atoi(dst_table->list[j]);
                        v = atoi(dst_table->list[j].vertex);

			sum_of_path_lengths += D[u-1][v-1];
			number_of_pairs++;
		}
	}

	/* compute the average length of shortest paths */
	average_length = sum_of_path_lengths/number_of_pairs;

	return average_length;
}

boolean adjust_sensor_energy_budget(double current_time, struct_sensor_t *sensor_info)
{ //adjust the sensor energy budget by the energy consumed by the number of duty cycles from the last restart time to current_time
  boolean result = TRUE; //the result with TRUE means that the sensor is still alive with the energy available for working; otherwise,
                         //it is dead due to energy depletion.
  double D = 0; //duration of surveillance starting from sensor working
  int n = 0; //number of duty cycles
  double t_restart = sensor_info->surveillance_restart_time;
  double t_current = current_time;
  double T_init = sensor_info->initial_sleeping_interval;
  double T_cycle = 0; //the length of one duty cycle
  double T_diff = 0; //portion of the last working time
  double w = sensor_info->sensing_interval; //working time
  double t_first = 0; //starting time point of the first working time
  double t_last = 0; //starting time point of the last whole working time contained in the number of duty cycles
  double T_w = 0; //accumulated working time
  double E_consume = 0; //energy consumed during the surveillance after surveillance_restart_time
  double E_diff = 0 ; //energy difference
  double E_work = 0; //energy needed for one working period
  
  /* check if current_time < t_restart */
  if(current_time < t_restart)
  {
    printf("adjust_sensor_energy_budget(): current_time(%f) < t_restart(%f)\n", current_time, t_restart);
#ifdef __DEBUG_INTERACTIVE_MODE__
    fgetc(stdin);
#endif
    exit(1);
  }

  T_cycle = sensor_info->sensing_interval + sensor_info->sleeping_interval;
  D = t_current - t_restart - T_init; //The duration of D includes the cycles of the working time after reschedule
  /* check whether D is greater than 0 or not; if D < 0, the sensor didn't consume any energy after the last refresh of its schedule at the time t_restart */
  if(D <= 0)
  {
    t_first = t_restart + T_init;
    t_last = t_first;
    T_w = 0; //no accumulated working time
    T_diff = 0; //zero time difference for actual working time
    E_consume = 0; //no energy consumption
  }
  else
  {
    n = (int)floor(D/T_cycle);
    T_diff = MIN(w, D-n*T_cycle);
    //T_diff = MAX(T_diff, 0); //if T_diff is negative, we set T_diff to 0.
  
    t_first = t_restart + T_init;
    t_last = t_first + n*T_cycle;

    T_w = n*w + T_diff; //@NOTE: when a new reschedule is issued during the sensor's working period, we just consider not only the energy consumption before the current working schedule, but also the woring time during this current working period. So we count T_diff as additional energy consumption.

    /* compute the energy consumed during the surveillance after surveillance_restart_time */
    E_consume =  estimate_energy_consumption(T_w, sensor_info->sensing_range, sensor_info->energy_consumption_rate);
  }

  /* compute the remaining energy in sensor */
  E_diff = sensor_info->energy - E_consume;

  /* compute the energy consumed for the working time w */
  E_work = estimate_energy_consumption(w, sensor_info->sensing_range, sensor_info->energy_consumption_rate);

  /* determine whether the sensor's energy budget remains or not */
  if((E_diff - E_work) > -1*ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
  //if(E_diff > E_work)
  {
    sensor_info->energy -= E_consume; 

    //@NOTE that the sensor is alive up to current_time + w.

    /*
    if(sensor_info->energy < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
      result = FALSE;
      sensor_info->energy = 0;
      //sensor_info->state = SENSOR_DIE; //@NOTE that state is set to SENSOR_DIE in PerformSensingSchedule()
    }
    else
      result = TRUE;
    */

    result = TRUE;
  }
  else
  {
    result = FALSE;
    sensor_info->energy = 0; //reset the energy budget to zero since the sensor worked up to its energy budget after the last reschedule
    
    /* set sensor's state to SENSOR_DIE due to energy depletion */
    //sensor_info->state = SENSOR_DIE; //@NOTE that state is set to SENSOR_DIE in PerformSensingSchedule()
  }

  return result;
}

void update_sensor_state_with_energy_budget_and_work_schedule(double current_time, struct_sensor_t *sensor_info, parameter_t *param)
{ //update the sensor's state along with the sensor energy budget by the energy consumed by the number of duty cycles from the last restart time to current_time and with the updated sensing work schedule; this function is called to show the sensor information when a vehicle escapes from the sensor network without detection.
  double D = 0; //duration of surveillance starting from sensor working
  int n = 0; //number of duty cycles
  double t_restart = sensor_info->surveillance_restart_time;
  double t_current = current_time;
  double T_init = sensor_info->initial_sleeping_interval;
  double T_cycle = 0; //the length of one duty cycle
  double T_diff = 0; //portion of the last working time
  double w = sensor_info->sensing_interval; //working time
  double t_first = 0; //starting time point of the first working time
  double t_last = 0; //starting time point of the last whole working time contained in the number of duty cycles
  double T_w = 0; //accumulated working time
  double E_consume = 0; //energy consumed during the surveillance after surveillance_restart_time
  double E_diff = 0 ; //energy difference
  double E_work = 0; //energy needed for one working period
  double delay = 0; //delay
  
  /* check if current_time < t_restart */
  if(current_time < t_restart)
  {
    printf("adjust_sensor_energy_budget(): current_time(%f) < t_restart(%f)\n", current_time, t_restart);
#ifdef __DEBUG_INTERACTIVE_MODE__
    fgetc(stdin);
#endif
    exit(1);
  }

  T_cycle = sensor_info->sensing_interval + sensor_info->sleeping_interval;
  D = t_current - t_restart - T_init; //The duration of D includes the cycles of the working time after reschedule
  /* check whether D is greater than 0 or not; if D < 0, the sensor didn't consume any energy after the last refresh of its schedule at the time t_restart */
  if(D <= 0)
  {
    t_first = t_restart + T_init;
    t_last = t_first;
    T_w = 0; //no accumulated working time
    T_diff = 0; //zero time difference for actual working time
    E_consume = 0; //no energy consumption
  }
  else
  {
    n = (int)floor(D/T_cycle);
    T_diff = MIN(w, D-n*T_cycle);
    //T_diff = MAX(T_diff, 0); //if T_diff is negative, we set T_diff to 0.
  
    t_first = t_restart + T_init;
    t_last = t_first + n*T_cycle;

    T_w = n*w + T_diff; //@NOTE: when a new reschedule is issued during the sensor's working period, we just consider not only the energy consumption before the current working schedule, but also the woring time during this current working period. So we count T_diff as additional energy consumption.

    /* compute the energy consumed during the surveillance after surveillance_restart_time */
    E_consume =  estimate_energy_consumption(T_w, sensor_info->sensing_range, sensor_info->energy_consumption_rate);
  }

  /* compute the remaining energy in sensor */
  E_diff = sensor_info->energy - E_consume;

  /* compute the energy consumed for the working time w */
  E_work = estimate_energy_consumption(w, sensor_info->sensing_range, sensor_info->energy_consumption_rate);

  /* determine whether the sensor's energy budget remains or not */
  if((E_diff - E_work) > -1*ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
  {
    sensor_info->energy -= E_consume;

    /* update the sensor's state */
    if((t_last <= t_current) && (t_current <= t_last+w))
      sensor_info->state = SENSOR_SENSE;
    else
      sensor_info->state = SENSOR_SLEEP;

    /* update the current work schedule */
    sensor_info->sensing_start_time = t_last;
    sensor_info->sensing_end_time = t_last + w;
  }
  else
  {
    sensor_info->energy = 0; //reset the energy budget to zero since the sensor worked up to its energy budget after the last reschedule

    sensor_info->state = SENSOR_DIE;
    delay = get_sensor_lifetime(sensor_info, param);
    sensor_info->death_time = t_restart + delay;

    /* update the last work schedule */
    sensor_info->sensing_start_time = MAX(sensor_info->death_time - w, 0);
    sensor_info->sensing_end_time = sensor_info->death_time;
  }
}

STATE get_sensor_state(double current_time, struct_sensor_t *sensor_info)
{ //get the sensor's state at current_time along with its scheduling and remaining energy budget
  STATE state = STATE_UNKNOWN; //state
  double D = 0; //duration of surveillance starting from sensor working
  int n = 0; //number of duty cycles
  double t_restart = sensor_info->surveillance_restart_time;
  double t_current = current_time;
  double T_init = sensor_info->initial_sleeping_interval;
  double T_cycle = 0; //the length of one duty cycle
  //double T_diff = 0; //portion of the last working time
  double w = sensor_info->sensing_interval; //working time
  double t_first = 0; //starting time point of the first working time
  double t_last = 0; //starting time point of the last whole working time contained in the number of duty cycles
  double T_w = 0; //accumulated working time
  double E_consume = 0; //energy consumed during the surveillance after surveillance_restart_time
  double E_diff = 0 ; //energy difference
  double E_work = 0; //energy needed for one working period

  /* check sensor's state */
  if(sensor_info->state == SENSOR_DIE && sensor_info->death_time <= current_time)
  {
    return sensor_info->state;
  }
  
  /* check if current_time < t_restart */
  if(current_time < t_restart)
  {
    printf("adjust_sensor_energy_budget(): current_time(%f) < t_restart(%f)\n", current_time, t_restart);
#ifdef __DEBUG_INTERACTIVE_MODE__
    fgetc(stdin);
#endif
    exit(1);
  }

  T_cycle = sensor_info->sensing_interval + sensor_info->sleeping_interval;
  D = t_current - t_restart - T_init; //The duration of D includes the cycles of the working time after reschedule
  /* check whether D is greater than 0 or not; if D < 0, the sensor didn't consume any energy after the last refresh of its schedule at the time t_restart */
  if(D <= 0)
  {
    t_first = t_restart + T_init;
    t_last = t_first;
    T_w = 0; //no accumulated working time
    //T_diff = 0; //zero time difference for actual working time
    E_consume = 0; //no energy consumption
  }
  else
  {
    n = (int)floor(D/T_cycle);
    //T_diff = MIN(w, D-n*T_cycle);
    //T_diff = MAX(T_diff, 0); //if T_diff is negative, we set T_diff to 0.
  
    t_first = t_restart + T_init;
    t_last = t_first + n*T_cycle;

    //T_w = n*w + T_diff;
    T_w = n*w; //@NOTE: since a vehicle can enter the sensor's sensing range during the working period, we just consider the energy consumption before the current working schedule. So we don't count T_diff as additional energy consumption.

    /* compute the energy consumed during the surveillance after surveillance_restart_time */
    E_consume =  estimate_energy_consumption(T_w, sensor_info->sensing_range, sensor_info->energy_consumption_rate);
  }

  /* compute the remaining energy in sensor */
  E_diff = sensor_info->energy - E_consume;

  /* compute the energy consumed for the working time w */
  E_work = estimate_energy_consumption(w, sensor_info->sensing_range, sensor_info->energy_consumption_rate);

  /* determine the sensor's state */
  if((E_diff - E_work) > -1*ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
  //if(E_diff > E_work)
  {
    if(t_last <= t_current && t_current <= t_last+w)
    //if(t_last <= t_current && t_current <= t_last+T_diff)
    {
      state = SENSOR_SENSE;
    }
    else
    {
      state = SENSOR_SLEEP;    
    }
  }
  else
  {
    state = SENSOR_DIE;
  }

  return state;
}

double get_sensing_circle_meeting_time(double t_arrive, double t_depart, struct_sensor_t *sensor_info, STATE *state)
{ //determine the time point when the vehicle meets the starting time of the first working period during the movement between t_arrive and t_depart. If there is a working period overlapped during the movement time, return the time point; otherwise retirn -1.
  double t_meet = -1; //time point when the vehicle meets the sensor's closest working period within the sensor's sensing circle
  //STATE state = STATE_UNKNOWN; //state
  double D = 0; //duration of surveillance starting from sensor working
  int n = 0; //number of duty cycles
  double t_restart = sensor_info->surveillance_restart_time;
  double t_current = t_arrive; //vehicle arriving time at sensing circle
  double T_init = sensor_info->initial_sleeping_interval;
  double T_cycle = 0; //the length of one duty cycle
  //double T_diff = 0; //portion of the last working time
  double w = sensor_info->sensing_interval; //working time
  double t_first = 0; //starting time point of the first working time
  double t_last = 0; //starting time point of the last whole working time contained in the number of duty cycles
  double T_w = 0; //accumulated working time
  double E_consume = 0; //energy consumed during the surveillance after surveillance_restart_time
  double E_diff = 0 ; //energy difference
  double E_work = 0; //energy needed for one working period

  /* set sensor's state to STATE_UNKNOWN */
  *state = STATE_UNKNOWN;

  /* check if t_arrive < t_restart */
  if(t_arrive < t_restart)
  {
    printf("adjust_sensor_energy_budget(): t_arrive(%f) < t_restart(%f)\n", t_arrive, t_restart);
#ifdef __DEBUG_INTERACTIVE_MODE__
    fgetc(stdin);
#endif
    exit(1);
  }

  T_cycle = sensor_info->sensing_interval + sensor_info->sleeping_interval;
  D = t_current - t_restart - T_init; //The duration of D includes the cycles of the working time after reschedule
  /* check whether D is greater than 0 or not; if D < 0, the sensor didn't consume any energy after the last refresh of its schedule at the time t_restart */
  if(D <= 0)
  {
    t_first = t_restart + T_init;
    t_last = t_first;
    T_w = 0; //no accumulated working time
    //T_diff = 0; //zero time difference for actual working time
    E_consume = 0; //no energy consumption
  }
  else
  {
    n = (int)floor(D/T_cycle);
    //T_diff = MIN(w, D-n*T_cycle);
  
    t_first = t_restart + T_init;
    t_last = t_first + n*T_cycle;

    //T_w = n*w + T_diff;
    T_w = n*w; //@NOTE: since a vehicle can enter the sensor's sensing range during the working period, we just consider the energy consumption before the current working schedule. So we don't count T_diff as additional energy consumption.

    /* compute the energy consumed during the surveillance after surveillance_restart_time */
    E_consume =  estimate_energy_consumption(T_w, sensor_info->sensing_range, sensor_info->energy_consumption_rate);
  }

  /* compute the remaining energy in sensor */
  E_diff = sensor_info->energy - E_consume;

  /* compute the energy consumed for the working time w */
  E_work = estimate_energy_consumption(w, sensor_info->sensing_range, sensor_info->energy_consumption_rate);

  /* determine the sensor's state */
  if((E_diff - E_work) > -1*ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
  //if(E_diff > E_work)
  {
    /* check whether the vehicle's movement time interval [t_arrive,t_depart] is overlapped with the sensor's working time interval [t_last, t_last+w] or not */
    if((t_arrive <= t_last+w) && (t_depart >= t_last))
    {
      *state = SENSOR_SENSE;
      t_meet = t_current;
    }
    else if(t_depart >= t_last+T_cycle)
    {
      *state = SENSOR_SENSE;
      t_meet = t_last + T_cycle;
    }
    else 
    {
      *state = SENSOR_SLEEP;    
      t_meet = -1;
    }
  }
  else
  {
    *state = SENSOR_DIE;
    t_meet = -1;
  }

  return t_meet;
}

double get_sensor_lifetime(struct_sensor_t *sensor_info, parameter_t *param)
{ //get sensor lifetime based on its energy budget along with its schedule starting from the sleeping time corresponding to the movement time over the shortest path
  double T_avail = 0; //available working time
  double T_life = 0; //sensor's lifetime
  double T_work = sensor_info->sensing_interval; //working time per duty cycle
  double T_init = sensor_info->initial_sleeping_interval; //initial sleeping time
  double T_sleep = sensor_info->sleeping_interval; //sleeping time per duty cycle
  double T_cycle = 0; //time corresponding to one duty cycle
  int n = 0 ; //number of duty cycles
  double T_diff = 0; //time difference
  
  /* estimate the available working time corresponding to the sensor's energy budget */
  T_avail = estimate_working_time(sensor_info->energy, sensor_info->sensing_range, sensor_info->energy_consumption_rate);

  /* compute the actual sensor lifetime according to sensing scheduling */
  switch(param->sensor_scan_type)
  {
    case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING:
    case SCAN_VARIABLE_SPEED_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
    case SCAN_VARIABLE_SPEED_WITH_VARIABLE_SLEEPING_TIME_AND_WITH_SENSING_HOLE_HANDLING:
    case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING:
    case SCAN_VARIABLE_SPEED_WITHOUT_MOVEMENT_SLEEPING_AND_WITH_SENSING_HOLE_HANDLING:
    case SCAN_NO_USE:
    case SCAN_NO_USE_WITH_OPTIMAL_SLEEPING_AND_SENSING_HOLE_HANDLING:
      n = (int)floor(T_avail/T_work);
      T_diff = T_avail - n*T_work;
      if((T_work - T_diff) < ERROR_TOLERANCE_FOR_REAL_ARITHMETIC) //T_diff is very close to T_work, inclease the number n of cycles by one.
	n++;

      T_cycle = T_sleep + T_work;
      if(n >= 2)
      {
	T_life = (T_init + T_work) + (n-1)*T_cycle; //the 1st period + (n-1)*T_cycle
      }
      else if(n == 1)
      {
	T_life = T_init + T_work; //only the 1st period
      }
      else
	T_life = 0;

      break;

    case SCAN_TURN_ON_ALL:
      T_life = T_avail;
      break;

    default:
      printf("get_sensor_lifetime(): sensor_scan_type(%d) is not supported!\n", param->sensor_scan_type);
#ifdef __DEBUG_INTERACTIVE_MODE__
      fgetc(stdin);
#endif
      exit(1);
  }
  
  return T_life;
}

void update_all_sensors_state(double current_time, struct_sensor_table *S, parameter_t *param)
{ //update sensors' states with their remaining energy and work schedule at the current time
  int i; //for-loop index

  for(i = 0; i < S->number; i++)
  {
    if(S->list[i]->info.state == SENSOR_DIE)
      continue;

    //@for debugging
    //if(i == (1260 - 1))
    //  printf("sensor %d is checked\n", i+1);
    ////////////////

    update_sensor_state_with_energy_budget_and_work_schedule(current_time, &(S->list[i]->info), param); //update the sensor's state along with the sensor energy budget by the energy consumed by the number of duty cycles from the last restart time to current_time and with the updated sensing work schedule; this function is called to show the sensor information when a vehicle escapes from the sensor network without detection.
  }
}

void store_evaluation_result_into_file(FILE *fp_1, FILE *fp_2, parameter_t *param, unsigned int seed, double sensor_network_lifetime, double average_detection_time, int number_of_detected_vehicles)
{ //store performance evaluation result into a file pointed by fp_1 and fp_2
  double test_param = 0; //test parameter, such as working time, energy variation, sensor density, vehicle speed, vehicle path length variation, etc.
  int target_algorithm = 0; //compared target algorithm, such as sensing scheduling algorithm, hole labeling algorithm, etc.

  /* set test_param according to evaluation type */
  switch(param->evaluation_type)
  {
  case EVALUATION_WORK_TIME:
    test_param = param->sensor_work_time;
    break;

  case EVALUATION_ENERGY_VARIATION:
    test_param = param->sensor_energy_standard_deviation;
    break;

  case EVALUATION_SENSOR_DENSITY:
    test_param = param->sensor_density;
    break;

  case EVALUATION_SENSOR_DENSITY_VARIATION:
    test_param = param->sensor_density_standard_deviation;
    break;

  case EVALUATION_VEHICLE_SPEED:
    test_param = param->vehicle_speed_in_mile_per_hour;
    break;

  case EVALUATION_PATH_LENGTH_VARIATION:
    test_param = param->vehicle_path_length_standard_deviation_in_mile;
    break;

  default:
    printf("store_evaluation_result_into_file(): evaluation type of %d is not supported!\n", param->evaluation_type);
    exit(1); 
  }

  /* set target_algorithm according to comparison target type */
  switch(param->comparison_target_type)
  {
  case COMPARISON_SCHEDULE_ALGORITHM:
    target_algorithm = param->sensor_scan_type;
    break;

  case COMPARISON_HOLE_LABELING_ALGORITHM:
    target_algorithm = param->sensor_hole_handling_algorithm;
    break;

  case COMPARISON_HOLE_LABELING_MODE:
    target_algorithm = param->sensor_hole_handling_mode;
    break;

  default:
    printf("store_evaluation_result_into_file(): comparison target type of %d is not supported!\n", param->comparison_target_type);
    exit(1); 
  }

  fprintf(fp_1, "\n### performance result after simulation ###\n");
  fprintf(fp_1, "%f\t%u\t%u\t%f\t%f\t%u\n", (float)test_param, seed, target_algorithm, (float)sensor_network_lifetime, (float)average_detection_time, number_of_detected_vehicles);

  fprintf(fp_2, "%f\t%u\t%u\t%f\t%f\t%u",   (float)test_param, seed, target_algorithm, (float)sensor_network_lifetime, (float)average_detection_time, number_of_detected_vehicles);

#ifdef __DEBUG_INTERACTIVE_MODE__
  printf("\n### performance result after simulation ###\n");
  printf("%f\t%u\t%u\t%f\t%f\t%u\n", (float)test_param, seed, target_algorithm, (float)sensor_network_lifetime, (float)average_detection_time, number_of_detected_vehicles);
#endif
}

void store_vanet_evaluation_result_into_file(FILE *fp, parameter_t *param, unsigned int seed, packet_delivery_statistics_t *packet_delivery_statistics, struct_graph_node *G)
{ //store VANET performance evaluation result into a file pointed by fp
  double test_param = 0; //test parameter, such as vehicle interarrival time, the number of vehicles, vehicle speed
  int target_algorithm = 0; //compared target algorithm, such as EDD model, link delay model, etc.
  struct_graph_node *pGraphNode = G[0].next; //pointer to the directional edge of <1,2> for the measurement of average convoy length
  double l = pGraphNode->weight; //road length of the directional edge of <1,2> 
  double v = param->vehicle_speed;
  double expected_average_convoy_length = 0; //expected average convoy length for the directional edge of <1,2>
  double actual_average_convoy_length = 0; //actual average convoy length for the directional edge of <1,2>
  double ratio_of_two_average_convoy_lengths = 0; //ratio of expected average convoy length to actual average convoy length
  double difference_between_two_average_convoy_lengths = 0; //difference between the expected average convoy length and actual average convoy length
  double acl_measurement_time = 0; //ACL measurement time

  /* set test_param according to evaluation type */
  switch(param->evaluation_type)
  {
  case EVALUATION_VEHICLE_MAXIMUM_NUMBER:
    test_param = param->vehicle_maximum_number;
    break;

  case EVALUATION_VEHICLE_PACKET_GENERATING_ENTITY_NUMBER:
    test_param = param->vehicle_packet_generating_entity_number;
    break;

  case EVALUATION_VEHICLE_SPEED:
    test_param = param->vehicle_speed_in_mile_per_hour;
    break;

  case EVALUATION_VEHICLE_SPEED_STANDARD_DEVIATION:
    test_param = param->vehicle_speed_standard_deviation_in_mile_per_hour;
    //test_param = param->vehicle_speed_standard_deviation;
    break;

  case EVALUATION_VEHICLE_INTERARRIVAL_TIME:
    test_param = param->vehicle_interarrival_time;
    break;

  case EVALUATION_COMMUNICATION_PACKET_INTERARRIVAL_TIME:
    test_param = param->communication_packet_interarrival_time;
    break;

  case EVALUATION_COMMUNICATION_PACKET_TTL:
    test_param = param->communication_packet_ttl;
    break;

  case EVALUATION_COMMUNICATION_PACKET_DELIVERY_PROBABILITY_THRESHOLD:
    test_param = param->communication_packet_delivery_probability_threshold;
    break;

  case EVALUATION_COMMUNICATION_AP_MAXIMUM_NUMBER:
    test_param = param->communication_AP_maximum_number;
    break;

  case EVALUATION_COMMUNICATION_SN_MAXIMUM_NUMBER:
    test_param = param->communication_SN_maximum_number;
    break;

  case EVALUATION_SIMULATION_TIME:
    test_param = param->simulation_time;
    break;

  default:
    printf("store_vanet_evaluation_result_into_file(): evaluation type of %d is not supported!\n", param->evaluation_type);
    exit(1); 
  }

  /* set target_algorithm according to comparison target type */
  switch(param->comparison_target_type)
  {
  case COMPARISON_EDD_AND_LINK_MODEL:
    target_algorithm = param->vehicle_vanet_edd_and_link_model;
    break;

  case COMPARISON_EDD_MODEL:
    target_algorithm = param->vehicle_vanet_edd_model;
    break;

  case COMPARISON_EDGE_DELAY_MODEL:
  case COMPARISON_AVERAGE_CONVOY_LENGTH_ESTIMATION_TYPE:
  case COMPARISON_AVERAGE_LINK_DELAY_ESTIMATION_TYPE:
    target_algorithm = param->vehicle_vanet_edge_delay_model;
    break;

  case COMPARISON_TBD_EDD_COMPUTATION_TYPE:
    target_algorithm = param->vehicle_vanet_tbd_edd_computation_type;
    break;

  case COMPARISON_INTERSECTION_FORWARDING_TYPE:
    target_algorithm = param->vehicle_vanet_intersection_forwarding_type;
    break;

  case COMPARISON_TARGET_POINT_SELECTION_TYPE:
    target_algorithm = param->vehicle_vanet_target_point_selection_type;
    break;

  case COMPARISON_TARGET_POINT_COMPUTATION_METHOD:
    target_algorithm = param->vehicle_vanet_target_point_computation_method;
    break;

  case COMPARISON_TARGET_POINT_NUMBER:
    target_algorithm = param->data_forwarding_maximum_target_point_number;
    break;

  default:
    printf("store_vanet_evaluation_result_into_file(): comparison target type of %d is not supported!\n", param->comparison_target_type);
    exit(1); 
  }

  /* store the evaluation results into the file and display the results */
  switch(param->comparison_target_type)
  {
  case COMPARISON_AVERAGE_CONVOY_LENGTH_ESTIMATION_TYPE:
    /* expected average convoy length for the directional edge of <1,2> */
    expected_average_convoy_length = VADD_Compute_Average_Convoy_Length(param, pGraphNode); 

    /* actual average convoy length for the directional edge of <1,2> */
    acl_measurement_time = pGraphNode->ptr_directional_edge_node->acl_measurement_end_time - pGraphNode->ptr_directional_edge_node->acl_measurement_start_time;
    actual_average_convoy_length = pGraphNode->ptr_directional_edge_node->acl_convoy_area / acl_measurement_time;

    /* compute the ratio of two average convoy lengths */
    ratio_of_two_average_convoy_lengths = expected_average_convoy_length/actual_average_convoy_length; //ratio of expected average convoy length to actual average convoy length

    difference_between_two_average_convoy_lengths = expected_average_convoy_length - actual_average_convoy_length; //difference between the expected average convoy length and actual average convoy length
	printf("\n### performance result after simulation estimation###\n");
	fprintf(fp, "%.2f\t%u\t%u\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n",
		(float)test_param,
		seed,
		target_algorithm,
		(float)expected_average_convoy_length,
		(float)actual_average_convoy_length,
		(float)ratio_of_two_average_convoy_lengths,
		(float)packet_delivery_statistics->packet_delivery_ratio,
		(float)difference_between_two_average_convoy_lengths,
		(float)0.0);

#ifdef __DEBUG_INTERACTIVE_MODE__
	printf("%.2f\t%u\t%u\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n",
		(float)test_param,
		seed,
		target_algorithm,
		(float)expected_average_convoy_length,
		(float)actual_average_convoy_length,
		(float)ratio_of_two_average_convoy_lengths,
		(float)packet_delivery_statistics->packet_delivery_ratio,
		(float)difference_between_two_average_convoy_lengths,
		(float)0.0);
#endif
    break;

  default:
    printf("\n### performance result after simulation default###\n");
    fprintf(fp, "%.2f\t%u\t%u\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%u\t%u\n", 
	(float)test_param, 
	seed, 
	target_algorithm, 
	(float)packet_delivery_statistics->mean_expected_delivery_delay, 
	(float)packet_delivery_statistics->mean_actual_delivery_delay, 
	(float)packet_delivery_statistics->ratio_of_two_delivery_delays, 
	(float)packet_delivery_statistics->packet_delivery_ratio, 
	(float)packet_delivery_statistics->mean_expected_delivery_delay_standard_deviation, 
	(float)packet_delivery_statistics->mean_delivery_delay_difference, 
	packet_delivery_statistics->mean_expected_packet_transmission_number, 
	packet_delivery_statistics->mean_actual_packet_transmission_number);

#ifdef __DEBUG_INTERACTIVE_MODE__
    printf("%.2f\t%u\t%u\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%u\t%u\t%d\n", 
	(float)test_param, 
	seed, 
	target_algorithm, 
	(float)packet_delivery_statistics->mean_expected_delivery_delay, 
	(float)packet_delivery_statistics->mean_actual_delivery_delay, 
	(float)packet_delivery_statistics->ratio_of_two_delivery_delays, 
	(float)packet_delivery_statistics->packet_delivery_ratio, 
	(float)packet_delivery_statistics->mean_expected_delivery_delay_standard_deviation, 
	(float)packet_delivery_statistics->mean_delivery_delay_difference, 
	packet_delivery_statistics->mean_expected_packet_transmission_number, 
	packet_delivery_statistics->mean_actual_packet_transmission_number,
        packet_delivery_statistics->min_delivery_delay_difference);
#endif
    break;
  }
}

/** conversion function betweem mile unit system and meter unit system */
double convert_feet_to_meter(double feet)
{ //convert feet to meter
  double meter = 0;

  meter = feet*FEET;

  return meter;
}

double convert_meter_to_feet(double meter)
{ //convert meter to feet
  double feet = 0;

  feet = meter/FEET;

  return feet;
}
/////

double convert_mile_to_meter(double mile)
{ //convert mile to meter
  double meter = 0;

  meter = mile*MILE;

  return meter;
}

double convert_meter_to_mile(double meter)
{ //convert meter to mile
  double mile = 0;
  
  mile = meter/MILE;

  return mile;
}

double convert_mile_to_km(double mile)
{ //convert mile to kilometer
  double km = 0;

  km = mile*MILE/1000;

  return km;
}

double convert_km_to_mile(double km)
{ //convert kilometer to mile
  double mile = 0;
  
  mile = km*1000/MILE;

  return mile;
}
/////

double convert_mile_per_hour_to_km_per_hour(double speed_in_mile_per_hour)
{ //convert the speed for mile per hour to the speed for kilometer(km) per hour
  double speed_in_km_per_hour = 0;

  speed_in_km_per_hour = speed_in_mile_per_hour*MILE/1000;
  return speed_in_km_per_hour;
}

double convert_km_per_hour_to_mile_per_hour(double speed_in_km_per_hour)
{ //convert the speed for kilometer per hour to the speed for km per hour
  double speed_in_mile_per_hour = 0;

  speed_in_mile_per_hour = speed_in_km_per_hour*1000/MILE;

  return speed_in_mile_per_hour;
}
/////

double convert_mile_per_hour_to_meter_per_sec(double speed_in_mile_per_hour)
{ //convert the speed for mile per hour to the speed for meter per sec
  double speed_in_meter_per_sec = 0;

  speed_in_meter_per_sec = speed_in_mile_per_hour*MILE/3600;

  return speed_in_meter_per_sec;
}

double convert_meter_per_sec_to_mile_per_hour(double speed_in_meter_per_sec)
{ //convert the speed for meter per sec to the speed for mile per hour
  double speed_in_mile_per_hour = 0;

  speed_in_mile_per_hour = speed_in_meter_per_sec*3600/MILE;

  return speed_in_mile_per_hour;
}

double convert_km_per_hour_to_meter_per_sec(double speed_in_km_per_hour)
{ //convert the speed for km per hour to the speed for meter per sec
  double speed_in_meter_per_sec = 0;

  speed_in_meter_per_sec = speed_in_km_per_hour*KILO/3600;

  return speed_in_meter_per_sec;
}

double convert_meter_per_sec_to_km_per_hour(double speed_in_meter_per_sec)
{ //convert the speed for meter per sec to the speed for km per hour
  double speed_in_km_per_hour = 0;

  speed_in_km_per_hour = speed_in_meter_per_sec*3600/KILO;

  return speed_in_km_per_hour;
}

/////


/** utility functions */

char* right_index(char *s, char c)
{ //returns a pointer to the last occurrence of the character c in the string s
	char *ptr = NULL; //pointer to a character
	char *ptr_index = NULL; //pointer to the last occurrence of the character c
	int i = 0; //index for for-loop
	int length = 0; //length of string s

	length = (int)strlen(s);
	ptr = s + (length - 1); //let ptr point to the last character of string s
	for(i = 0; i < length; i++)
	{
		if(*(ptr - i) == c)
		{
			ptr_index = ptr - i;
			break;
		}
	}
	
	return ptr_index;
}

double dist(double x1, double x2)
{ //Euclidean distance between x1 and x2
  double distance = 0;

  distance = fabs(x1 - x2);

  return distance;
}

char *itoa(int i, char* a)
{ //convert integer i into character string a in the decimal format
  static char buf[BUF_SIZE]; //buffer for the representation of integer i

  if(a != NULL)
  {
    sprintf(a, "%d", i);
    return a;
  }
  else
  {
    sprintf(buf, "%d", i);
    return buf;
  }
}


/** Convoy Operations */
void convoy_all_vehicle_join(parameter_t *param, double join_time, struct_graph_node *G, int G_size, packet_delivery_statistics_t *packet_delivery_stat)
{ //let all vehicles join their own convoy moving on the directional edge pointed by a graph node in G where each vehicle is moving

	if(&vehicle_list == vehicle_list.next)
	  return; //there is no vehicle node in the vehicle list

	/* sort the vehicle movement queue for each directional edge in graph G */
	SortVehicleMovementQueues_In_Graph(G, G_size);

	/* construct convoys for each directional edge using the sorted vehicle movement queues */
	convoy_construct_convoys_in_graph(param, join_time, G, G_size, packet_delivery_stat);
}

void convoy_construct_convoys_in_graph(parameter_t *param, double join_time, struct_graph_node *G, int G_size, packet_delivery_statistics_t *packet_delivery_stat)
{ //construct convoys for each directional edge in graph G using the sorted vehicle movement queues
        int i = 0, j = 0; //IDs for vertices in graph G
	struct_graph_node *ptr = NULL; //pointer to graph node

	/* make adjacency matrix W with adjacent list G */
	for(i = 0; i < G_size; i++)
	{
		ptr = G[i].next;
		if(ptr == NULL)
			continue;

		while(ptr != NULL)
		{
			j = atoi(ptr->vertex) - 1; 
			//node id starts from 1, but it should decrease by 1 for weight matrix where the id starts from 0.
                        convoy_construct_convoys_in_directional_edge(param, join_time, &(ptr->ptr_directional_edge_node->vehicle_movement_list), &(ptr->ptr_directional_edge_node->convoy_list), packet_delivery_stat);
                        //construct convoys moving on the directional edge <v_i,v_j> along with vehicle_movement_list
			ptr = ptr->next;
		}
	}
}

void convoy_construct_convoys_in_directional_edge(parameter_t *param, double join_time, vehicle_movement_queue_t *VQ, convoy_queue_t *CQ, packet_delivery_statistics_t *packet_delivery_stat)
{ //construct convoys moving on the directional edge <v_i,v_j> into convoy queue CQ along with vehicle_movement_list VQ
  directional_edge_queue_node_t *pEdgeNode = VQ->enode; //pointer to the directional edge node
  double offset1 = 0, offset2 = 0; //offsets in the directional edge
  int i = 0; //index for for-loop
  vehicle_movement_queue_node_t *pMoveNode = NULL; //pointer to a vehicle movement queue node
  int cid = 0; //convoy id
  convoy_queue_node_t *pConvoyNode = NULL; //pointer to a convoy queue node
  double preceding_vehicle_offset = INF; //offset of the preceding vehicle
  double following_vehicle_offset = 0; //offset of the following vehicle
  double distance = INF; //distance between two adjacent vehicles

  /** destroy the convoy queue if the queue size is greater than zero */
  if(CQ->size)
    DestroyQueue((queue_t*)CQ);

  /** construct convoy by traversing the VQ from the rear vehicle movement node with the greatest offset towards the front vehicle movement node with the smallest offset */
  pMoveNode = &(VQ->head);
  for(i = 0; i < VQ->size; i++) //for-1
  {
    pMoveNode = pMoveNode->prev;
    following_vehicle_offset = pMoveNode->vnode->current_pos_in_digraph.offset;
    //following_vehicle_offset = pMoveNode->vnode->ptr_vehicle_movement_queue_node->offset;
    distance = preceding_vehicle_offset - following_vehicle_offset; //distance between two adjacent vehicles on the directional edge
    if(distance < -1*ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    {
      printf("convoy_construct_convoys_in_directional_edge(): distance(=%f) is negative\n", (float)distance);
      exit(1);
    }

    if(distance > param->communication_range) //if-1
    { //if this condition holds, make another convoy
      pConvoyNode = convoy_construct(pMoveNode->vnode, join_time);
    } //end of if-1
    else //end of else-2
    { //in this case, add the vehicle to the current convoy pointed by pConvoyNode
      convoy_add_vehicle(param, pConvoyNode, pMoveNode->vnode, join_time, packet_delivery_stat);
    } //end of else-2
    
    preceding_vehicle_offset = following_vehicle_offset;
  } //end of for-1
}

convoy_queue_node_t* convoy_construct(struct_vehicle_t *vehicle, double join_time)
{ //construct an atomic convoy only with the vehicle on the directional edge and register the convoy with the convoy queue of the directional edge
  directional_edge_queue_node_t *pEdgeNode = vehicle->current_pos_in_digraph.enode; //pointer to the directional edge
  convoy_queue_node_t convoy_node; //convoy queue node
  convoy_queue_node_t *pConvoyNode = NULL; //pointer to convoy queue node
  vehicle_queue_node_t vehicle_node; //vehicle queue node

  /* check whether flag_convoy_registration is set to FALSE */
  if(vehicle->flag_convoy_registration == FALSE) //if-1
  {
    convoy_node.cid = vehicle->id;
    convoy_node.tail_vehicle = vehicle;
    convoy_node.head_vehicle = vehicle;
    convoy_node.leader_vehicle = vehicle;

    pConvoyNode = (convoy_queue_node_t*)Enqueue((queue_t*)&(pEdgeNode->convoy_list), (queue_node_t*)&convoy_node);

    /* initialize vehicle node */
    memset(&vehicle_node, 0, sizeof(vehicle_node));
    vehicle_node.vid = vehicle->id;
    vehicle_node.vnode = vehicle;

    /* enqueue the vehicle node into the convoy's vehicle list */
    Enqueue((queue_t*)&(pConvoyNode->vehicle_list), (queue_node_t*)&vehicle_node);

    /* let vehicle's ptr_convoy_queue_node to point to the corresponding convoy queue node in the convoy_list of the edge where the vehicle is moving */
    vehicle->ptr_convoy_queue_node = pConvoyNode;

    /* set flag_convoy_registration to TRUE */
    vehicle->flag_convoy_registration = TRUE;

    /* set the vehicle's convoy join time to join_time */
    vehicle->convoy_join_time = join_time;
  } //end of if-1
  else //else-1
  {
    printf("convoy_construct(): Error: vehicle(%d) has already been registered in another convoy queue node for undirectional edge of eid=%d with move_type=%d\n", vehicle->id, vehicle->current_pos_in_Gr.eid, vehicle->move_type);
    exit(1);
  } //end of else-1

  return pConvoyNode;
}

convoy_queue_node_t* convoy_add_vehicle(parameter_t *param, convoy_queue_node_t *pConvoyNode, struct_vehicle_t *vehicle, double join_time, packet_delivery_statistics_t *packet_delivery_stat)
{ //add the vehicle to the current convoy pointed by pConvoyNode and let the vehicle pass its packets to the convoy head
  vehicle_queue_node_t vehicle_node; //vehicle queue node
  int discard_count = 0; //count for discarded packets

  /* check whether flag_convoy_registration is set to FALSE */
  if(vehicle->flag_convoy_registration == FALSE) //if-1
  {
    /* initialize vehicle node */
    memset(&vehicle_node, 0, sizeof(vehicle_node));
    vehicle_node.vid = vehicle->id;
    vehicle_node.vnode = vehicle;

    /* enqueue the vehicle node into the convoy's vehicle list */
    Enqueue((queue_t*)&(pConvoyNode->vehicle_list), (queue_node_t*)&vehicle_node);

    /* let vehicle's ptr_convoy_queue_node to point to the corresponding convoy queue node in the convoy_list of the edge where the vehicle is moving */
    vehicle->ptr_convoy_queue_node = pConvoyNode;

    /* set flag_convoy_registration to TRUE */
    vehicle->flag_convoy_registration = TRUE;

    /* set the vehicle's convoy join time to join_time */
    vehicle->convoy_join_time = join_time;

    /* set the convoy's tail vehicle to vehicle */
    pConvoyNode->tail_vehicle = vehicle;
    
    /* According to data_forwarding_mode, update the convoy's leader vehicle if the vehicle's EDD is shorter than the convoy leader's EDD */
    if((param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD) || (param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD))
    {
      if(vehicle->EDD < pConvoyNode->leader_vehicle->EDD)
	  {
        pConvoyNode->leader_vehicle = vehicle;
	  }
	  else
	  {
	    /* pass the vehicle's packets to the convoy head */
		VADD_Forward_Packet_To_Next_Carrier(param, join_time, vehicle, pConvoyNode->head_vehicle, packet_delivery_stat, &discard_count);
	  }
    }
    else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
    {
      if(vehicle->EDD < pConvoyNode->leader_vehicle->EDD)
	  {
        pConvoyNode->leader_vehicle = vehicle;
	  }
    }
  } //end of if-1
  else //else-1
  {
    printf("convoy_add_vehicle(): Error: vehicle(%d) has already been registered in another convoy queue node for undirectional edge of eid=%d with move_type=%d\n", vehicle->id, vehicle->current_pos_in_Gr.eid, vehicle->move_type);
    exit(1);
  } //end of else-1

  return pConvoyNode;
}

boolean convoy_join(parameter_t *param, struct_vehicle_t *vehicle, double join_time, packet_delivery_statistics_t *packet_delivery_stat)
{ //join vehicle into the convoy preceding the vehicle on the directional edge if the vehicle is within the communication range of the convoy
  //char tail_node[NAME_SIZE];  //tail node of a directional edge
  //char head_node[NAME_SIZE];  //head node of a directional edge
  directional_edge_queue_node_t *pEdgeNode = vehicle->current_pos_in_digraph.enode; //pointer to a directional edge queue node
  convoy_queue_node_t *pConvoyNode = NULL; //pointer to convoy queue node
  vehicle_queue_node_t vehicle_node; //vehicle queue node
  boolean flag = FALSE; //flag to indicate whether this vehicle's join creates a new convoy or not
  int discard_count = 0; //count for discarded packets

#ifdef __DEBUG_CONVOY_JOIN__
  printf("convoy_join(): at time %f, vehicle(vid=%d) joins into a convoy on the directional edge(eid=%d)\n", (float)join_time, vehicle->id, pEdgeNode->eid);  
  
#endif

  /**@for debugging */
  // if(vehicle->id == 74 && join_time >= 4807)
  //  printf("convoy_join(): vehicle(id=%d) is traced\n", vehicle->id);
  /******************/

  /* check whether flag_convoy_registration is set to FALSE */
  if(vehicle->flag_convoy_registration == FALSE)
  {
    /*
    strcpy(tail_node, vehicle->path_ptr->vertex); //tail node for the directional edge where the vehicle is moving
    strcpy(head_node, vehicle->path_ptr->next->vertex); //head node for the directional edge where the vehicle is moving
    pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
    if(pEdgeNode == NULL)
    {
      printf("convoy_join(): pEdgeNode for <%s,%s> is NULL\n", tail_node, head_node);
      exit(1);
    }
    */

    /* check whether there exists a convoy for this vehicle to join within the communication range */
    if(pEdgeNode->convoy_list.size == 0)
    { //if there is no convoy, this vehicle creates a new convoy moving on this directional edge
      pConvoyNode = convoy_construct(vehicle, join_time);
      //construct an atomic convoy only with the vehicle on the directional edge and register the convoy with the convoy queue of the directional edge

      flag = TRUE;
    }
    else
    {
      pConvoyNode = GetConvoyClosestToVehicle(param, &(pEdgeNode->convoy_list), vehicle->current_pos_in_digraph.offset); //get the pointer to the convoy node closest to the vehicle within the communication range
      //pConvoyNode = (convoy_queue_node_t*) GetRearQueueNode((queue_t*) &(pEdgeNode->convoy_list));
      //get the pointer to the last enqueued convoy node since it is closest to the vehicle
      //@We need to improve this line later to support variable vehicle speed
      //@ Note: we assume that the vehicle speed is constant; otherwise, we need to search a convoy closest to this vehicle

      if(pConvoyNode != NULL)
      {
        /* initialize vehicle node */
        memset(&vehicle_node, 0, sizeof(vehicle_node));
        vehicle_node.vid = vehicle->id;
        vehicle_node.vnode = vehicle;

        /* enqueue the vehicle node into the convoy's vehicle list */
        Enqueue((queue_t*)&(pConvoyNode->vehicle_list), (queue_node_t*)&vehicle_node);

        /* determine the convoy's new head or new tail */
        if(vehicle->current_pos_in_digraph.offset > pConvoyNode->head_vehicle->current_pos_in_digraph.offset) //the vehicle has greater offset than the convoy head, so update the convoy head with the vehicle.
        {
			if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD || param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
			{
				/* pass the convoy head's packets to the vehicle */
				if(pConvoyNode->head_vehicle->packet_queue->size > 0)
					VADD_Forward_Packet_To_Next_Carrier(param, join_time, pConvoyNode->head_vehicle, vehicle, packet_delivery_stat, &discard_count);
			}

			pConvoyNode->head_vehicle = vehicle; //update the convoy head with the vehicle
			pConvoyNode->cid = vehicle->id; //update the convoy id with the vehicle's id
		}
		else if(vehicle->current_pos_in_digraph.offset <= pConvoyNode->tail_vehicle->current_pos_in_digraph.offset)
		//@Note: = is needed when the head is the only vehicle in the convoy and its offset is zero.
		//else if(vehicle->current_pos_in_digraph.offset < pConvoyNode->tail_vehicle->current_pos_in_digraph.offset)
		{ //the vehicle has less offset than the convoy tail, so update the convoy tail with the vehicle
			if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD || param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
			{
				/* pass the vehicle's packets to the convoy head */
				if(vehicle->packet_queue->size > 0)
					VADD_Forward_Packet_To_Next_Carrier(param, join_time, vehicle, pConvoyNode->head_vehicle, packet_delivery_stat, &discard_count);
			}

			pConvoyNode->tail_vehicle = vehicle;
		}
		else
		{ //the vehicle is moving between the convoy head and the convoy tail
			if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD || param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
			{
				/* pass the vehicle's packets to the convoy head */
				if(vehicle->packet_queue->size > 0)
					VADD_Forward_Packet_To_Next_Carrier(param, join_time, vehicle, pConvoyNode->head_vehicle, packet_delivery_stat, &discard_count);
			}
		}

        /* According to data_forwarding_mode, update the convoy's leader vehicle if the vehicle's EDD is shorter than the convoy leader's EDD */
        if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
        {
          if(vehicle->EDD < pConvoyNode->leader_vehicle->EDD)
            pConvoyNode->leader_vehicle = vehicle;
        }
        else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
        {
          if(vehicle->EDD < pConvoyNode->leader_vehicle->EDD)
            pConvoyNode->leader_vehicle = vehicle;
        }
        else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
        {
          if(vehicle->EDD < pConvoyNode->leader_vehicle->EDD)
            pConvoyNode->leader_vehicle = vehicle;
        }

	/* let vehicle's ptr_convoy_queue_node to point to the corresponding convoy queue node in the convoy_list of the edge where the vehicle is moving */
        vehicle->ptr_convoy_queue_node = pConvoyNode;

        /* set the vehicle's convoy join time to join_time */
        vehicle->convoy_join_time = join_time;

        /* set flag_convoy_registration to TRUE */
        vehicle->flag_convoy_registration = TRUE;

        flag = FALSE;
      }
      else
      { //if there is no convoy to join within communication range, this vehicle creates a new convoy moving on this directional edge
		pConvoyNode = convoy_construct(vehicle, join_time);
	//construct an atomic convoy only with the vehicle on the directional edge and register the convoy with the convoy queue of the directional edge

        flag = TRUE;
      }
    }
  }
  else
  {
    printf("convoy_join(): Error: vehicle(%d) has already been registered in another convoy queue node for undirectional edge of eid=%d with move_type=%d\n", vehicle->id, vehicle->current_pos_in_Gr.eid, vehicle->move_type);
    exit(1);
  }  

  return flag;
}

boolean convoy_fast_join(parameter_t *param, double join_time, struct_vehicle_t *vehicle, convoy_queue_node_t *dst_convoy)
{ //join vehicle into the dst_convoy close to vehicle on the directional edge
  boolean flag = FALSE; //flag to indicate whether this vehicle's join creates a new convoy or not
  vehicle_queue_node_t vehicle_node; //vehicle queue node

#ifdef __DEBUG_CONVOY_FAST_JOIN__
  printf("convoy_fast_join(): at time %f, vehicle(vid=%d) joins into dst_convoy(cid=%d)\n", (float)join_time, vehicle->id, dst_convoy->cid);
#endif
 
  if(dst_convoy == NULL) //if-1
  {
    convoy_construct(vehicle, join_time);
    //construct an atomic convoy only with the vehicle on the directional edge and register the convoy with the convoy queue of the directional edge

    flag = TRUE; //indicate that vehicle performs convoy_construct()
  } //end of if-1
  else //else-2
  {
    /* initialize vehicle node */
    memset(&vehicle_node, 0, sizeof(vehicle_node));
    vehicle_node.vid = vehicle->id;
    vehicle_node.vnode = vehicle;

    /* enqueue the vehicle node into dst_convoy's vehicle list */
    Enqueue((queue_t*)&(dst_convoy->vehicle_list), (queue_node_t*)&vehicle_node);

    /* determine the convoy's new head or new tail */
    if(vehicle->current_pos_in_digraph.offset > dst_convoy->head_vehicle->current_pos_in_digraph.offset) //the vehicle has greater offset than the convoy head, so update the convoy head with the vehicle.
    {
      dst_convoy->head_vehicle = vehicle; //update the convoy head with the vehicle
      dst_convoy->cid = vehicle->id; //update the convoy id with the vehicle's id
    }
    else if(vehicle->current_pos_in_digraph.offset <= dst_convoy->tail_vehicle->current_pos_in_digraph.offset)
    //@Note: = is needed when the head is the only vehicle in the convoy and its offset is zero.
    //else if(vehicle->current_pos_in_digraph.offset < dst_convoy->tail_vehicle->current_pos_in_digraph.offset)
    { //the vehicle has less offset than the convoy tail, so update the convoy tail with the vehicle
      dst_convoy->tail_vehicle = vehicle;
    }

    /* According to data_forwarding_mode, update the convoy's leader vehicle if the vehicle's EDD is shorter than the convoy leader's EDD */
    if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
    {
      if(vehicle->EDD < dst_convoy->leader_vehicle->EDD)
        dst_convoy->leader_vehicle = vehicle;
    }
    else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
    {
      if(vehicle->EDD < dst_convoy->leader_vehicle->EDD)
        dst_convoy->leader_vehicle = vehicle;
    }
    else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
    {
      if(vehicle->EDD < dst_convoy->leader_vehicle->EDD)
        dst_convoy->leader_vehicle = vehicle;
    }

    /* let vehicle's ptr_convoy_queue_node to point to the corresponding convoy queue node in the convoy_list of the edge where the vehicle is moving */
    vehicle->ptr_convoy_queue_node = dst_convoy;

    /* set the vehicle's convoy join time to join_time */
    vehicle->convoy_join_time = join_time;

    /* set flag_convoy_registration to TRUE */
    vehicle->flag_convoy_registration = TRUE;

    flag = FALSE;
  } //end of else-2

  return flag;
}

boolean convoy_leave(parameter_t *param, struct_vehicle_t *vehicle, double leave_time)
//boolean convoy_leave(parameter_t *param, struct_vehicle_t *vehicle, double leave_time, struct_graph_node *G)
{ //let vehicle leave from the convoy including the vehicle on the directional edge if the vehicle reaches the directional edge's head or is out of the communication range of the convoy
  convoy_queue_t *CQ = vehicle->ptr_convoy_queue_node->ptr_queue; //pointer to the convoy queue for the directional edge where vehicle is moving
  //char tail_node[NAME_SIZE];  //tail node of a directional edge
  //char head_node[NAME_SIZE];  //head node of a directional edge
  //directional_edge_queue_node_t *pEdgeNode = NULL; //pointer to a directional edge queue node
  convoy_queue_node_t *pConvoyNode = NULL, *pConvoyNode2 = NULL; //pointer to convoy queue node
  boolean flag = FALSE; //flag to indicate whether this vehicle's leave deletes the convoy or not
  struct_vehicle_t *head_vehicle = NULL; //pointer to a convoy head vehicle
  boolean flag_head_vehicle = FALSE; //flag to check if the vehicle is the convoy head or not
  boolean flag_tail_vehicle = FALSE; //flag to check if the vehicle is the convoy tail or not
  boolean flag_leader_vehicle = FALSE; //flag to check if the vehicle is the convoy leader or not

#ifdef __DEBUG_CONVOY_LEAVE__
  printf("convoy_leave(): at time %f, vehicle(vid=%d) leaves the convoy(cid=%d)\n", (float)leave_time, vehicle->id, vehicle->ptr_convoy_queue_node->cid);
#endif

  /**@for debugging */
  //if(leave_time >= 3691 && vehicle == 0xd2717e0)
  //if(leave_time >= 3691 && vehicle->id == 48)
  //  printf("convoy_leave(): at time %f, vehicle(id=%d) is traced\n", (float)leave_time, vehicle->id);
  /******************/

  /* check whether flag_convoy_registration is set to FALSE */
  if(vehicle->flag_convoy_registration == TRUE) //if-1
  {
    pConvoyNode = vehicle->ptr_convoy_queue_node; //obtain the pointer of the convoy node containing the vehicle

    /**@for debugging */
    //if(leave_time >= 373 && (pConvoyNode->cid == 32 || vehicle->id == 30))
    //  printf("convoy(cid=%d) or vehicle(vid=%d) are tracked\n", pConvoyNode->cid, vehicle->id);
    /******************/

    /* check whether this vehicle is the convoy header or not */
    if(vehicle->id == pConvoyNode->head_vehicle->id)
      flag_head_vehicle = TRUE;
    
    /* check whether this vehicle is the convoy tail or not */
    if(vehicle->id == pConvoyNode->tail_vehicle->id)
      flag_tail_vehicle = TRUE;
    
    /* check whether this vehicle is the convoy leader or not */
    if(vehicle->id == pConvoyNode->leader_vehicle->id)
      flag_leader_vehicle = TRUE;

    /** perform the convoy-leave according to the role of the vehicle, such as head, tail, or leader in the convoy */
    if(pConvoyNode->vehicle_list.size == 1) //sometimes, some vehicle may have greater offset than the convoy head or have less offset than the convoy tail due to the discrete event simulation and a small offset difference between two vehicles    
    { //the convoy has only one vehicle
      if(!(flag_head_vehicle && flag_tail_vehicle && flag_leader_vehicle)) //if-1.2
	//if(pConvoyNode->vehicle_list.size != 1) //if-1.2.1
      {
	printf("convoy_leave(): at time %f, in the convoy(id=%d) of size 1, convoy head(id=%d) must be the same as convoy tail(id=%d) and convoy leader(id=%d)!\n", (float)leave_time, pConvoyNode->cid, pConvoyNode->head_vehicle->id, pConvoyNode->tail_vehicle->id, pConvoyNode->leader_vehicle->id);
        exit(1);
      } //end of if-1.2.1
      else //else-1.2.2
      {
        /* delete the vehicle from its convoy's vehicle list */
        DeleteVehicleWithVID(&(pConvoyNode->vehicle_list), vehicle->id);

        /* get the pointer to the directional edge in order to delete the convoy registered with the directional edge */
	/*
        strcpy(tail_node, vehicle->path_ptr->vertex); //tail node for the directional edge where the vehicle is moving
        strcpy(head_node, vehicle->path_ptr->next->vertex); //head node for the directional edge where the vehicle is moving
	*/

        /*
        pEdgeNode = FastLookupDirectionalEdgeQueue(G, tail_node, head_node);
        if(pEdgeNode == NULL) //if-1.1
        {
          printf("convoy_join(): pEdgeNode for <%s,%s> is NULL\n", tail_node, head_node);
          exit(1);
        } //end of if-1.1
	*/

        /* delete the convoy node since there is no vehicle in this convoy */
	DeleteConvoyWithCID(CQ, pConvoyNode->cid);
	//DeleteConvoyWithCID(&(pEdgeNode->convoy_list), pConvoyNode->cid);

	flag = TRUE; //indicate that the convoy with this vehicle is deleted from the directional edge's convoy list
      } //end of else-1.2.2
    } //end of if-1.2
    else if(flag_head_vehicle) //else if-1.3
    { //if the vehicle is the convoy head, then find a new convoy head
      DeleteVehicleWithVID(&(pConvoyNode->vehicle_list), vehicle->id);

      /** determine the convoy's new head and set it to a vehicle with maximum offset */
      Set_NewConvoyHead(pConvoyNode);

      /* update convoy id with the new convoy head's vid */
      pConvoyNode->cid = pConvoyNode->head_vehicle->id;
      
      /** update a new leader vehicle */
      if(flag_leader_vehicle)
	  Set_NewConvoyLeader(param, pConvoyNode);

      /** update a new leader vehicle
         Note that a head vehicle can be a tail vehicle in the case whether its offset is the same as with other vehicles. */
      if(flag_tail_vehicle)
	  Set_NewConvoyTail(pConvoyNode);
    } //end of else if-1.3
    else if(flag_tail_vehicle) //else if-1.4
    { //if the vehicle is the convoy tail, then find a new convoy tail
      DeleteVehicleWithVID(&(pConvoyNode->vehicle_list), vehicle->id);

      /** determine the convoy's new tail and the convoy's new leader and set them to the convoy's pointers */
      Set_NewConvoyTail(pConvoyNode);

      /* update a new leader vehicle */
      if(flag_leader_vehicle)
	  Set_NewConvoyLeader(param, pConvoyNode);
    } //end of else if-1.4
    else if(flag_leader_vehicle) //else if-1.5
    { //if the vehicle is the convoy leader, then find a new convoy leader
      DeleteVehicleWithVID(&(pConvoyNode->vehicle_list), vehicle->id);

      /** determine the convoy's new leader and and set it to a vehicle with minimum EDD */
      Set_NewConvoyLeader(param, pConvoyNode);
    } //end of else if-1.5
    else //else-1.6
    { //if the vehicle is not the convoy head, then delete the vehicle from vehicle list and perform convoy split if there occurs a network partition within the convoy
      DeleteVehicleWithVID(&(pConvoyNode->vehicle_list), vehicle->id);

      /**@for debugging */
      //printf("convoy_leave(): a vehicle(id=%d) is leaving the convoy(id=%d)!\n", vehicle->id, pConvoyNode->cid);
      /******************/

      /* split the convoy if there occurs a network partition due to the deletion of this vehicle */
      /* 
      if(is_there_network_partition(pConvoyNode))
	convoy_split(param, vehicle, pEdgeNode, pConvoyNode, leave_time, G); //split a convoy into two convoy and let vehicles within the original convoy refer to the new convoys according to their offsets
      */
    } //else-1.6

    /* let vehicle's flag_convoy_registration be FALSE to denote that the vehicle leaves from the current convoy */
    vehicle->flag_convoy_registration = FALSE;
    vehicle->ptr_convoy_queue_node = NULL;
  } //end of if-1

  return flag;
}

boolean convoy_update(parameter_t *param, struct_vehicle_t *vehicle, double update_time)
{ //update the convoys close to the vehicle with the convoy operations, such as convoy_construct, convoy_split, and convoy_merge. 
  boolean flag = FALSE;
  vehicle_movement_queue_t* VQ = vehicle->ptr_vehicle_movement_queue_node->ptr_queue; //pointer to the vehicle movement queue for the directional edge where vehicle is moving
  struct_vehicle_t *left = NULL; //the left-neighboring vehicle for vehicle
  struct_vehicle_t *right = NULL; //the right-neighboring vehicle for vehicle
  int VQ_size = VQ->size; //size of vehicle movement queue
  int order = 0; //ascending order of vehicle in offset, from 0 to size-1
  double r = param->communication_range; //communication range
  double d_left = 0, d_right = 0; //distance between left(i) and i; distance between i and right(i) where i is vehicle, left(i) is the left neighboring vehicle of i and right(i) is the right neighboring vehicle of i.
  int vehicle_cid = vehicle->ptr_convoy_queue_node->cid; //vehicle's convoy id
  int left_cid = 0; //convoy id of the left neighboring vehicle of vehicle
  int right_cid = 0; //convoy id of the left neighboring vehicle of vehicle
  convoy_queue_t *CQ = vehicle->ptr_convoy_queue_node->ptr_queue; //pointer to the convoy queue for the directional edge where vehicle is moving
  convoy_queue_node_t *vehicle_convoy = vehicle->ptr_convoy_queue_node; //pointer to vehicle's convoy queue node
  convoy_queue_node_t *left_convoy = NULL; //pointer to the left vehicle's convoy queue node
  convoy_queue_node_t *right_convoy = NULL; //pointer to the right vehicle's convoy queue node
  /*@ Note: we update vehicle_convoy, left_convoy, right_convoy before using it after performing convoy_merge() or convoy_split() since the pointers of vehicle_convoy, left_convoy and right_convoy might change after these two operations */

  /*@ Note: assume that the vehicle_list is sorted in ascending order of offsets */
  
  /** Operarations 
    - Let r be the communication range.
    - Sort the vehicle movement queue for the directional edge where vehicle (i.e., i) is moving.
    - Let left(i) be the left vehicle of vehicle i and right(i) be the right vehicle of vehicle i.
    - We can obtain the appropriate action for the relationshop among i, left(i) and right(i) using a truth table.

    - if dist(left(i)->offset, i->offset) <= r and dist(right(i)->offset, i->offset) <= r, then
        if left(i)->cid == i->cid and i->cid == right(i)->cid, then
          no action. //there is no change in the convoy
        elseif left(i)->cid == i->cid and i->cid != right(i)->cid, then 
          merge i->convoy into right(i)->convoy.
        elseif left(i)->cid != i->cid and i->cid == right(i)->cid, then 
          merge left(i)->convoy into i->convoy.
        elseif left(i)->cid != i->cid and left(i)->cid == right(i)->cid, then 
          merge i->convoy into right(i)->convoy.
        elseif left(i)->cid != i->cid and right(i)->cid != i->cid and left(i)->cid != right(i)->cid, then
          merge left(i)->convoy and i->convoy into right(i)->convoy.          
	endif
      endif

    - if dist(left(i)->offset, i->offset) <= r and dist(right(i)->offset, i->offset) > r, then
        if left(i)->cid == i->cid and i->cid == right(i)->cid, then
          split right(i)->convoy into i->convoy and right(i)->convoy.
        elseif left(i)->cid == i->cid and i->cid != right(i)->cid, then 
          no action.
        elseif left(i)->cid != i->cid and i->cid == right(i)->cid, then
          split right(i)->convoy into i->convoy and right(i)->convoy. 
          merge left(i)->convoy into i->convoy.
        elseif left(i)->cid != i->cid and left(i)->cid == right(i)->cid, then
	  split right(i)->convoy into left(i)->convoy and right(i)->convoy.  
          merge left(i)->convoy into i->convoy.
        elseif left(i)->cid != i->cid and right(i)->cid != i->cid and left(i)->cid != right(i)->cid, then
          merge left(i)->convoy into i->convoy.          
	endif
      endif

    - if dist(left(i)->offset, i->offset) > r and dist(right(i)->offset, i->offset) <= r, then
        if left(i)->cid == i->cid and i->cid == right(i)->cid, then
          split i->convoy into left(i)->convoy and i->convoy.
        elseif left(i)->cid == i->cid and i->cid != right(i)->cid, then 
          split i->convoy into left(i)->convoy and i->convoy.
          merge i->convoy into right(i)->convoy.
        elseif left(i)->cid != i->cid and i->cid == right(i)->cid, then
          no action.
        elseif left(i)->cid != i->cid and left(i)->cid == right(i)->cid, then
	  split right(i)->convoy into left(i)->convoy and right(i)->convoy.  
          merge i->convoy into right(i)->convoy.
        elseif left(i)->cid != i->cid and right(i)->cid != i->cid and left(i)->cid != right(i)->cid, then
          merge i->convoy into right(i)->convoy.          
	endif     
      endif

    - if dist(left(i)->offset, i->offset) > r and dist(right(i)->offset, i->offset) > r, then
        if left(i)->cid == i->cid and i->cid == right(i)->cid, then
          split right(i)->convoy into left(i)->convoy, i->convoy and right(i)->convoy.
        elseif left(i)->cid == i->cid and i->cid != right(i)->cid, then 
          split i->convoy into left(i)->convoy and i->convoy.
        elseif left(i)->cid != i->cid and i->cid == right(i)->cid, then
          split right(i)->convoy into i->convoy and right(i)->convoy.
        elseif left(i)->cid != i->cid and left(i)->cid == right(i)->cid, then
	  split right(i)->convoy into left(i)->convoy and right(i)->convoy.  
          make a new convoy with vehicle i.
        elseif left(i)->cid != i->cid and right(i)->cid != i->cid and left(i)->cid != right(i)->cid, then
          no action.          
	endif      
      endif
  */

#ifdef __DEBUG_CONVOY_UPDATE__
  printf("convoy_update(): at time %f, vehicle(vid=%d) performs convoy_update in the convoy(cid=%d)\n", (float)update_time, vehicle->id, vehicle_convoy->cid);
#endif

  /**@for debugging */
  //if(update_time >= 4809 && vehicle->id == 74)
  //if(update_time >= 3661 && (vehicle->id == 7 || vehicle->id == 54))
  //  printf("convoy_update(): at %f, vehicle(id=%d) is traced\n", (float)update_time, vehicle->id);
  /******************/

  /** Sort the vehicle movement queue for the directional edge where vehicle (i.e., i) is moving. */
  SortVehicleMovementQueue(VQ);
  //sort vehicle movement queue nodes in ascending order according to offset in the directional edge
  
  /** Obtain the left vehicle and the right vehicle for vehicle */
  order = vehicle->ptr_vehicle_movement_queue_node->order;
  if(VQ_size == 1) //if-1
  {
    return FALSE; //there is nothing to update for convoy
  } //end of if-1
  else //else-2
  { //the case where VQ_size > 1
    if(order == 0) //if-2.1
    {
      left = NULL;
      right = vehicle->ptr_vehicle_movement_queue_node->next->vnode;

      d_left = -1;
      d_right = dist(right->current_pos_in_digraph.offset, vehicle->current_pos_in_digraph.offset);

      left_convoy = NULL;
      right_convoy = right->ptr_convoy_queue_node;

      left_cid = -1;
      right_cid = right_convoy->cid;
    } //end of if-2.1
    else if(order == VQ_size-1) //else-if-2.2
    {
      left = vehicle->ptr_vehicle_movement_queue_node->prev->vnode;
      right = NULL;

      d_left = dist(left->current_pos_in_digraph.offset, vehicle->current_pos_in_digraph.offset);
      d_right = -1;
   
      left_convoy = left->ptr_convoy_queue_node;
      right_convoy = NULL;

      left_cid = left_convoy->cid;
      right_cid = -1;
    } //end of else-if-2.2
    else //else-2.3
    {
      left = vehicle->ptr_vehicle_movement_queue_node->prev->vnode;
      right = vehicle->ptr_vehicle_movement_queue_node->next->vnode;

      d_left = dist(left->current_pos_in_digraph.offset, vehicle->current_pos_in_digraph.offset);
      d_right = dist(right->current_pos_in_digraph.offset, vehicle->current_pos_in_digraph.offset);
   
      left_convoy = left->ptr_convoy_queue_node;
      right_convoy = right->ptr_convoy_queue_node;

      left_cid = left_convoy->cid;
      right_cid = right_convoy->cid;
    } //end of else-2.3
  } //end of else-2

  /** perform convoy_merge(), convoy_split(), convoy_leave() and convoy_fast_join() for vehicle's movement update*/
  if(left != NULL && right != NULL) //if-3
  {
    if(d_left <= r && d_right <= r) //if-3.1
    {
      if(left_cid == vehicle_cid && vehicle_cid == right_cid) //if-3.1.1
      {
	/* no action since there is no change in the convoy */
      } //end of if-3.1.1
      else if(left_cid == vehicle_cid && vehicle_cid != right_cid) //if-3.1.2
      {
	/* merge vehicle_convoy into right_convoy */
        convoy_merge(param, update_time, vehicle_convoy, right_convoy);
      } //end of if-3.1.2
      else if(left_cid != vehicle_cid && vehicle_cid == right_cid) //if-3.1.3
      {
	/* merge left_convoy into vehicle_convoy */
        convoy_merge(param, update_time, left_convoy, vehicle_convoy);
      } //end of if-3.1.3
      else if(left_cid != vehicle_cid && left_cid == right_cid) //if-3.1.4
      {
        /* make vehicle leave its current convoy */
        convoy_leave(param, vehicle, update_time);

	/* join vehicle into right_convoy */
	convoy_fast_join(param, update_time, vehicle, right_convoy);
      } //end of if-3.1.4
      else if(left_cid != vehicle_cid && right_cid != vehicle_cid && left_cid != right_cid) //if-3.1.5
      {
        /* make vehicle leave its current convoy */
        convoy_leave(param, vehicle, update_time);

	/* join vehicle into right_convoy */
	convoy_fast_join(param, update_time, vehicle, right_convoy);

        /* merge left_convoy into right_convoy */
        convoy_merge(param, update_time, left_convoy, right_convoy);
      } //end of if-3.1.5
    } //end of if-3.1
    /*************************************************/
    else if(d_left <= r && d_right > r) //else-if-3.2
    {
      if(left_cid == vehicle_cid && vehicle_cid == right_cid) //if-3.2.1
      {
	/**@for debugging */
        //if(update_time >= 3688 && vehicle->id == 54)
        //printf("convoy_update(): at time %f, vehicle(id=%d) is traced\n", (float)update_time, vehicle->id);
        /******************/

	/* split right_convoy into vehicle_convoy and right_convoy */
	convoy_split(param, update_time, right_convoy, vehicle);
      } //end of if-3.2.1
      else if(left_cid == vehicle_cid && vehicle_cid != right_cid) //if-3.2.2
      {
	/* no action since there is no change in the convoy */
      } //end of if-3.2.2
      else if(left_cid != vehicle_cid && vehicle_cid == right_cid) //if-3.2.3
      {
        /* make vehicle leave its current convoy */
        convoy_leave(param, vehicle, update_time);

	/* join vehicle into left_convoy */
	convoy_fast_join(param, update_time, vehicle, left_convoy);
      } //end of if-3.2.3
      else if(left_cid != vehicle_cid && left_cid == right_cid) //if-3.2.4
      {
        /* make vehicle leave its current convoy */
        convoy_leave(param, vehicle, update_time);

	/* split right_convoy into left_convoy and right_convoy */
	convoy_split(param, update_time, right_convoy, left);

	/* join vehicle into left_convoy */
	left_convoy = left->ptr_convoy_queue_node; //update left_convoy due to the split of right_convoy
	convoy_fast_join(param, update_time, vehicle, left_convoy);
      } //end of if-3.2.4
      else if(left_cid != vehicle_cid && right_cid != vehicle_cid && left_cid != right_cid) //if-3.2.5
      {
        /* make vehicle leave its current convoy */
        convoy_leave(param, vehicle, update_time);

	/* join vehicle into left_convoy */
	convoy_fast_join(param, update_time, vehicle, left_convoy);
      } //end of if-3.2.5
    } //end of else-if-3.2
    else if(d_left > r && d_right <= r) //else-if-3.3
    {
      if(left_cid == vehicle_cid && vehicle_cid == right_cid) //if-3.3.1
      {
	/* split right_convoy into left_convoy and right_convoy */
	convoy_split(param, update_time, right_convoy, left);
      } //end of if-3.3.1
      else if(left_cid == vehicle_cid && vehicle_cid != right_cid) //if-3.3.2
      {
        /* make vehicle leave its current convoy */
        convoy_leave(param, vehicle, update_time);

	/* join vehicle into right_convoy */
	convoy_fast_join(param, update_time, vehicle, right_convoy);
      } //end of if-3.3.2
      else if(left_cid != vehicle_cid && vehicle_cid == right_cid) //if-3.3.3
      {
	/* no action since there is no change in the convoy */
      } //end of if-3.3.3
      else if(left_cid != vehicle_cid && left_cid == right_cid) //if-3.3.4
      {
        /* make vehicle leave its current convoy */
        convoy_leave(param, vehicle, update_time);

	/* split right_convoy into left_convoy and right_convoy */
	convoy_split(param, update_time, right_convoy, left);

	/* join vehicle into right_convoy */
	left_convoy = left->ptr_convoy_queue_node; //update left_convoy due to the split of right_convoy; note that right_convoy is still the same after the convoy_split; actually, we don't need to update left_convoy here since we don't use it in the following convoy_fast_join().

	convoy_fast_join(param, update_time, vehicle, right_convoy);
      } //end of if-3.3.4
      else if(left_cid != vehicle_cid && right_cid != vehicle_cid && left_cid != right_cid) //if-3.3.5
      {
        /* make vehicle leave its current convoy */
        convoy_leave(param, vehicle, update_time);

	/* join vehicle into right_convoy */
	convoy_fast_join(param, update_time, vehicle, right_convoy);
      } //end of if-3.3.5
    } //end of else-if-3.3
    else if(d_left > r && d_right > r) //else-if-3.4
    {
      if(left_cid == vehicle_cid && vehicle_cid == right_cid) //if-3.4.1
      {
	/* split right_convoy into vehicle_convoy and right_convoy */
	convoy_split(param, update_time, right_convoy, vehicle);

	/* split vehicle_convoy into left_convoy and vehicle_convoy */
        vehicle_convoy = vehicle->ptr_convoy_queue_node; //update vehicle_convoy due to the split of right_convoy

	convoy_split(param, update_time, vehicle_convoy, left);
      } //end of if-3.4.1
      else if(left_cid == vehicle_cid && vehicle_cid != right_cid) //if-3.4.2
      {
	/* split vehicle_convoy into left_convoy and vehicle_convoy */
	convoy_split(param, update_time, vehicle_convoy, left);
      } //end of if-3.4.2
      else if(left_cid != vehicle_cid && vehicle_cid == right_cid) //if-3.4.3
      {
	/* split right_convoy into vehicle_convoy and right_convoy */
	convoy_split(param, update_time, right_convoy, vehicle);
      } //end of if-3.4.3
      else if(left_cid != vehicle_cid && left_cid == right_cid) //if-3.4.4
      {
        /* make vehicle leave its current convoy */
        convoy_leave(param, vehicle, update_time);

	/* join vehicle into its own convoy; that is, construct a new convoy */
	convoy_fast_join(param, update_time, vehicle, NULL);

	/* split right_convoy into left_convoy and right_convoy */
	convoy_split(param, update_time, right_convoy, left);
      } //end of if-3.4.4
      else if(left_cid != vehicle_cid && right_cid != vehicle_cid && left_cid != right_cid) //if-3.4.5
      {
	/* no action since there is no change in the convoy */
      } //end of if-3.4.5
    } //end of else-if-3.4
  } //end of if-3
  /*************************************************/
  else if(left != NULL && right == NULL) //else-if-4
  {
    if(d_left <= r) //if-4.1
    {
      if(left_cid == vehicle_cid) //if-4.1.1
      {
	/* no action since there is no change in the convoy */
      } //end of if-4.1.1
      else if(left_cid != vehicle_cid) //if-4.1.2
      {
        /* make vehicle leave its current convoy */
        convoy_leave(param, vehicle, update_time);

	/* join vehicle into left_convoy */
	convoy_fast_join(param, update_time, vehicle, left_convoy);
      } //end of if-4.1.2
    } //end of if-4.1
    if(d_left > r) //if-4.2
    {
      if(left_cid == vehicle_cid) //if-4.2.1
      {
	/* split vehicle_convoy into left_convoy and vehicle_convoy */
	convoy_split(param, update_time, vehicle_convoy, left);
      } //end of if-4.2.1
      else if(left_cid != vehicle_cid) //if-4.2.2
      {
	/* no action since there is no change in the convoy */
      } //end of if-4.2.2
    } //end of if-4.2
  } //end of else-if-4
  /*************************************************/
  else if(left == NULL && right != NULL) //else if-5
  {
    if(d_right <= r) //if-5.1
    {
      if(vehicle_cid == right_cid) //if-5.1.1
      {
	/* no action since there is no change in the convoy */
      } //end of if-5.1.1
      else if(vehicle_cid != right_cid) //if-5.1.2
      {
        /* make vehicle leave its current convoy */
        convoy_leave(param, vehicle, update_time);

	/* join vehicle into right_convoy */
	convoy_fast_join(param, update_time, vehicle, right_convoy);
      } //end of if-5.1.2
    } //end of if-5.1
    if(d_right > r) //if-5.2
    {
      if(vehicle_cid == right_cid) //if-5.2.1
      {
	/* split right_convoy into vehicle_convoy and right_convoy */
	convoy_split(param, update_time, right_convoy, vehicle);
      } //end of if-5.2.1
      else if(vehicle_cid != right_cid) //if-5.2.2
      {
	/* no action since there is no change in the convoy */
      } //end of if-5.2.2
    } //end of if-5.2
  } //end of else-if-5

  /** Update the convoy head, convoy tail, and convoy leader for the convoy containing the vehicle after the vehicle movement update */
   
  /* @Note: update vehicle_convoy since vehicle_convoy can be updated through either convoy_merge() or convoy_split() */
  vehicle_convoy = vehicle->ptr_convoy_queue_node; 

  if((vehicle->id != vehicle_convoy->head_vehicle->id) && (vehicle->current_pos_in_digraph.offset > vehicle_convoy->head_vehicle->current_pos_in_digraph.offset))
  {
    vehicle_convoy->head_vehicle = vehicle;
    vehicle_convoy->cid = vehicle->id;
    //Note that we update the convoy's cid with vehicle's id even though the previous convoy head stays within the convoy
  }

  if((vehicle->id != vehicle_convoy->tail_vehicle->id) && (vehicle->current_pos_in_digraph.offset <= vehicle_convoy->tail_vehicle->current_pos_in_digraph.offset))
  //@Note: = is needed when the head is the only vehicle in the convoy and its offset is zero.
  //if((vehicle->id != vehicle_convoy->tail_vehicle->id) && (vehicle->current_pos_in_digraph.offset < vehicle_convoy->tail_vehicle->current_pos_in_digraph.offset))
    vehicle_convoy->tail_vehicle = vehicle;

  if(vehicle->id != vehicle_convoy->leader_vehicle->id)
  {
    /* According to data_forwarding_mode, update the convoy's leader vehicle if the vehicle's EDD is shorter than the convoy leader's EDD */
    if(param->data_forwarding_mode == DATA_FORWARDING_MODE_UPLOAD)
    {
      if(vehicle->EDD < vehicle_convoy->leader_vehicle->EDD)
        vehicle_convoy->leader_vehicle = vehicle;
    }
    else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_DOWNLOAD)
    {
      if(vehicle->EDD < vehicle_convoy->leader_vehicle->EDD)
        vehicle_convoy->leader_vehicle = vehicle;
    }
    else if(param->data_forwarding_mode == DATA_FORWARDING_MODE_V2V)
    {
      if(vehicle->EDD < vehicle_convoy->leader_vehicle->EDD)
        vehicle_convoy->leader_vehicle = vehicle;
    }
  }

  return flag;
}

boolean convoy_merge(parameter_t *param, double merge_time, convoy_queue_node_t *src_convoy, convoy_queue_node_t *dst_convoy)
{ //let src_convoy merge into dst_convoy; concatenate src_convoy's vehicle_list into dst_convoy's vehicle_list
  boolean flag = FALSE;
  convoy_queue_t *CQ = src_convoy->ptr_queue; //pointer to convoy queue
  int src_size = src_convoy->vehicle_list.size; //size of the vehicle list in src_convoy 
  int dst_size = dst_convoy->vehicle_list.size; //size of the vehicle list in dst_convoy
  vehicle_queue_node_t *src_head = &(src_convoy->vehicle_list.head); //pointer to the head node the vehicle list in src_convoy 
  vehicle_queue_node_t *dst_head = &(dst_convoy->vehicle_list.head); //pointer to the head node the vehicle list in dst_convoy 
  vehicle_queue_node_t *pVehicleNode = NULL; //pointer to vehicle queue node
  int i = 0; //index for for-loop

#ifdef __DEBUG_CONVOY_MERGE__
  printf("convoy_merge(): at time %f, src_convoy(cid=%d) is merged into dst_convoy(cid=%d)\n", (float)merge_time, src_convoy->cid, dst_convoy->cid);
#endif

  /**@for debugging */
  //if(merge_time >= 4808 && src_convoy->cid == 82)
  //  printf("convoy_merge(): at time %f, convoy(cid=%d) is traced\n", (float)merge_time, src_convoy->cid);
  /******************/
 
  /** check the validity of src_convoy and dst_convoy */
  if(src_size == 0 || dst_size == 0)
  {
    printf("convoy_merge(): Error: src_convoy->vehicle_list.size(%d) == 0 or dst_convoy->vehicle_list.size(%d) == 0\n", src_size, dst_size);
    exit(1);
  } 

  /** concatenate src_convoy's vehicle_list into dst_convoy's vehicle_list */
  /* link dst_head's rear node and src_head's front node */
  dst_head->prev->next = src_head->next;
  src_head->next->prev = dst_head->prev;
  
  /* link dst_head and src_head's rear node */
  dst_head->prev = src_head->prev;  
  src_head->prev->next = dst_head;

  /** adjust the pointer to the convoy queue node for each vehicle in src_convoy */
  pVehicleNode = src_head;
  for(i = 0; i < src_size; i++)
  {
    pVehicleNode = pVehicleNode->next;
    pVehicleNode->vnode->ptr_convoy_queue_node = dst_convoy;
    pVehicleNode->vnode->convoy_join_time = merge_time;
  }

  /** update the size of the vehicle list in dst_convoy and src_convoy */
  dst_convoy->vehicle_list.size += src_size;
  src_convoy->vehicle_list.size = 0;

  /* reset src_head->prev and src_head->next to src_head */
  src_head->prev = src_head;
  src_head->next = src_head;

  /** delete src_convoy */
  DeleteConvoyWithCID(CQ, src_convoy->cid);

  /** update the convoy head, tail and leader */  
  /* sort dst_convoy's vehicle list in ascending order of offset */
  SortVehicleQueue(&(dst_convoy->vehicle_list));

  /* set the convoy head to the rear vehicle in vehicle list and update the convoy's id (i.e., cid) with the new head vehicle's id */
  dst_convoy->head_vehicle = dst_convoy->vehicle_list.head.prev->vnode;
  dst_convoy->cid = dst_convoy->head_vehicle->id;
  
  /* set the convoy tail to the front vehicle in vehicle list */
  dst_convoy->tail_vehicle = dst_convoy->vehicle_list.head.next->vnode;

  /* set the convoy leader to the vehicle with minimum EDD in vehicle list */
  Set_NewConvoyLeader(param, dst_convoy); //find a vehicle with minimum EDD as the convoy's new leader in dst_convoy

  return flag;
}

boolean convoy_split(parameter_t *param, double split_time, convoy_queue_node_t *src_convoy, struct_vehicle_t *vehicle)
{ //let src_convoy split into dst_convoy_1 and dst_convoy_2 by using vehicle as the boundary of these two convoys; vehicle is the head of dst_convoy_2
  boolean flag = FALSE;
  convoy_queue_t *CQ = src_convoy->ptr_queue; //pointer to convoy queue
  vehicle_queue_t* VQ = &(src_convoy->vehicle_list); //pointer to vehicle queue
  vehicle_queue_node_t *convoy_1_head = &(src_convoy->vehicle_list.head); //pointer to the head for dst_convoy_1's vehicle list where dst_convoy_1 is the convoy consisting of vehicles preceding vehicle
  vehicle_queue_node_t *convoy_2_head = NULL; //pointer to the head for dst_convoy_2's vehicle list where dst_convoy_2 is the convoy consisting of vehicles following vehicle along with vehicle
  vehicle_queue_node_t *convoy_1_start = NULL; //pointer to the vehicle queue node for the start of the vehicle list of src_convoy after the split
  convoy_queue_node_t convoy_node; //convoy queue node
  convoy_queue_node_t *dst_convoy = NULL; //pointer to the destination convoy separated from src_convoy, having vehicle as the convoy head
  int src_size = src_convoy->vehicle_list.size; //size of the vehicle list in src_convoy
  int dst_size = 0; //size of the vehicle list in dst_convoy
  vehicle_queue_node_t *src_head = &(src_convoy->vehicle_list.head); //pointer to the head node the vehicle list in src_convoy 
  //vehicle_queue_node_t *dst_head = NULL; //pointer to the head node the vehicle list in dst_convoy 
  vehicle_queue_node_t *pVehicleNode = NULL; //pointer to vehicle queue node
  int i = 0; //index for for-loop
  int count = 0; //count for the number of dst_convoy_2

#ifdef __DEBUG_CONVOY_SPLIT__
  printf("convoy_split(): at time %f, convoy(cid=%d) is split for vehicle(vid=%d)\n", (float)split_time, src_convoy->cid, vehicle->id);
#endif

  /**@for debugging */
  //if(split_time >= 4128 && src_convoy->cid == 209)
  //  printf("convoy_split(): at time %f, convoy(cid=%d) is traced\n", (float)split_time, src_convoy->cid);

  //if(split_time >= 4841 && vehicle->id == 54)
  //  printf("convoy_split(): at time %f, vehicle(id=%d) is traced\n", (float)split_time, vehicle->id);

  //return flag;
  /******************/
  
  /** check the validity of src_convoy */
  if(src_size < 2)
  {
    printf("convoy_split(): Error: src_convoy->vehicle_list.size(%d) must be greater than 2\n", src_size);
    exit(1);
  } 

  /** sort vehicle list in ascending order of offset */
  SortVehicleQueue(VQ);

  /** split src_convoy into src_convoy and dst_convoy by using vehicle as the convoy head of dst_convoy */
  /* obtain the pointer to the vehicle queue node corresponding to vehicle */
  pVehicleNode = src_head;
  for(i = 0; i < src_size; i++)
  {
    pVehicleNode = pVehicleNode->next;
    count++; //increment count by one
    if(vehicle->id == pVehicleNode->vnode->id)
      break;
  }

  if(i == src_size)
  {
    printf("convoy_split(): Error: vehicle(id=%d) does not exist in the convoy(cid=%d)\n", vehicle->id, src_convoy->cid);
    exit(1);
  }

  /* let convoy_1_start point to the next node of pVehicleNode */
  convoy_1_start = pVehicleNode->next;

  /**@ Start of the construction of convoy_2 */
 
  /* make a convoy queue node for dst_convoy_2 and enqueue it into convoy queue CQ */
  convoy_node.cid = pVehicleNode->vnode->id; //set the new convoy's id with the head vehicle's id
  convoy_node.tail_vehicle = pVehicleNode->vnode;
  convoy_node.head_vehicle = pVehicleNode->vnode;
  convoy_node.leader_vehicle = pVehicleNode->vnode;
  dst_convoy = (convoy_queue_node_t*)Enqueue((queue_t*)CQ, (queue_node_t*)&convoy_node);

  /* set convoy_2_head to the pointer to the head for dst_convoy_2's vehicle list where dst_convoy_2 is the convoy consisting of vehicles following vehicle along with vehicle */
  convoy_2_head = &(dst_convoy->vehicle_list.head);

  /* set convoy_2_head where convoy_2 is the convoy consisting of vehicles following vehicle along with vehicle */
  convoy_2_head->next = src_head->next;
  convoy_2_head->prev = pVehicleNode;
 
  /* let the first vehicle node and the last vehicle node point to convoy_2_head */
  convoy_2_head->next->prev = convoy_2_head;
  convoy_2_head->prev->next = convoy_2_head;

  /* set dst_convoy->vehicle_list.size to count */
  dst_convoy->vehicle_list.size = count;

  /**@ End of the construction of convoy_2 */
 

  /**@ Start of the construction of convoy_1 */
  /* set convoy_1_head where convoy_1 is the convoy consisting of vehicles preceding vehicle */
  convoy_1_head->next = convoy_1_start; //note that convoy_1_head->next == src_head->nex
  //convoy_1_head->prev = src_head->prev;

  /* let the first vehicle node point to convoy_1_head */
  convoy_1_head->next->prev = convoy_1_head;

  /* set src_convoy->vehicle_list.size to count */
  src_convoy->vehicle_list.size -= count;

  /**@ End of the construction of convoy_1 */


  /** adjust the pointer to the convoy queue node for each vehicle in dst_convoy */
  pVehicleNode = &(dst_convoy->vehicle_list.head);
  for(i = 0; i < count; i++)
  {
    pVehicleNode = pVehicleNode->next;
    pVehicleNode->vnode->ptr_convoy_queue_node = dst_convoy;
    pVehicleNode->vnode->convoy_join_time = split_time;
  }

  /** adjust the convoy tail and convoy leader of src_convoy */
  src_convoy->tail_vehicle = src_convoy->vehicle_list.head.next->vnode; //set convoy tail to the first vehicle node in vehicle list
  Set_NewConvoyLeader(param, src_convoy); //find a vehicle with minimum EDD as the convoy's new leader in src_convoy

  /** adjust the convoy tail and convoy leader of dst_convoy */
  dst_convoy->tail_vehicle = dst_convoy->vehicle_list.head.next->vnode; //set convoy tail to the first vehicle node in vehicle list
  Set_NewConvoyLeader(param, dst_convoy); //find a vehicle with minimum EDD as the convoy's new leader in dst_convoy

#ifdef __DEBUG_CONVOY_SPLIT__
  printf("convoy_split(): at time %f, dst_convoy(cid=%d) is split from for src_convoy(cid=%d)\n", (float)split_time, dst_convoy->cid, src_convoy->cid);
#endif
  
  return flag;
}


/** operations for stationary vehicle */

void set_vehicle_trajectory(struct_vehicle_t *vehicle, double arrival_time, int *trajectory, int trajectory_size, struct_graph_node *Gr, edge_queue_t *Er)
{ //set up the vehicle's trajectory
  struct_path_node *path_list = NULL; //list of vertices on the path from source to destination
  char *tail_node = NULL; //pointer to tail node name
  char *head_node = NULL; //pointer to head node name
  MOVE_TYPE move_type; //movement type for vehicle
  double edge_length = 0; //the length of the edge
  int path_hop_count; //hop count for the path, i.e., the number of edges
  directional_edge_queue_node_t *ptr_directional_edge_node = NULL; //pointer to the directional edge node in directional edge queue DEr for real graph Gr

  /** delete vehicle's path-list */
  Free_Path_List(vehicle->path_list);
  vehicle->path_list = NULL;
  vehicle->path_ptr = NULL;

  /** initialize the vehicle instance **/
  vehicle->state = VEHICLE_STATIONARY_VEHICLE_SEND;
  vehicle->seq = 0;
  vehicle->restart_time = arrival_time;

  /** make the path list for a given trajectory */
  path_list = Make_Path_List_For_Given_Trajectory(trajectory, trajectory_size, Gr, &path_hop_count);

  vehicle->path_list = path_list;
  vehicle->path_hop_count = path_hop_count; //set up path_hop_count for path_list
  vehicle->path_ptr = path_list->next;
  vehicle->path_current_hop = 0; //reset path_current_hop to zero

  /** set the initial position, current position and movement type */
  tail_node = vehicle->path_ptr->vertex;
  head_node = vehicle->path_ptr->next->vertex;
  vehicle->current_pos_in_Gr.eid = vehicle->init_pos_in_Gr.eid = FastGetEdgeID_MoveType(Gr, tail_node, head_node, &move_type, &edge_length, &ptr_directional_edge_node);
  //vehicle->current_pos_in_Gr.eid = vehicle->init_pos_in_Gr.eid = GetEdgeID_MoveType(Er, tail_node, head_node, &move_type, &edge_length);
  vehicle->move_type = move_type;
  vehicle->edge_length = edge_length;
  if(move_type == MOVE_FORWARD)
    vehicle->current_pos_in_Gr.offset = vehicle->init_pos_in_Gr.offset = 0;
  else if(move_type == MOVE_BACKWARD)
    vehicle->current_pos_in_Gr.offset = vehicle->init_pos_in_Gr.offset = edge_length;
  else
  {
    printf("set_vehicle_trajectory(): move_type(%d) is invalid\n", move_type);
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
}

void set_vehicle_current_position(parameter_t *param, struct_vehicle_t *vehicle, double current_time, int *trajectory, int trajectory_size, struct_graph_node *Gr, edge_queue_t *Er)
{ //set up the vehicle's current position in the road network graph Gr along with the deregistration of the previous movement and the registration of the current movement

  char *tail_node = NULL; //pointer to tail node name
  char *head_node = NULL; //pointer to head node name
  MOVE_TYPE move_type; //movement type for vehicle
  double edge_length = 0; //the length of the edge
  directional_edge_queue_node_t *ptr_directional_edge_node = NULL; //pointer to the directional edge node in directional edge queue DEr for real graph Gr

  /** check whether the trajectory is valid or not */
  if(trajectory == NULL)
  {
    printf("set_vehicle_current_position(): Error: trajectory is NULL\n", trajectory);
    exit(1);
  }
  else if(trajectory_size <= 1)
  {
    printf("set_vehicle_current_position(): trajectory_size(=%d) is less than 2\n", trajectory_size);
    exit(1);
  }

  /** set the current position to the beginning of the path with path_current_hop zero and also set movement type according to the direction of the edge */
  vehicle->path_ptr = vehicle->path_list->next; //let path_ptr point to the first node on the path_list
  vehicle->path_current_hop = 0; //reset path_current_hop to zero

  tail_node = vehicle->path_list->next->vertex;
  head_node = vehicle->path_list->next->next->vertex;
  vehicle->current_pos_in_Gr.eid = FastGetEdgeID_MoveType(Gr, tail_node, head_node, &move_type, &edge_length, &ptr_directional_edge_node);
  //vehicle->current_pos_in_Gr.eid = GetEdgeID_MoveType(Er, tail_node, head_node, &move_type, &edge_length);
  vehicle->move_type = move_type;
  vehicle->edge_length = edge_length;
  if(move_type == MOVE_FORWARD)
    vehicle->current_pos_in_Gr.offset = 0;
  else if(move_type == MOVE_BACKWARD)
    vehicle->current_pos_in_Gr.offset = edge_length;
  else
  {
    printf("set_vehicle_current_position(): move_type(%d) is invalid\n", move_type);
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

  /* delete vehicle movement from the previous directional edge where the vehicle is moving */
  delete_vehicle_movement(param, vehicle, current_time, Gr);

  /* register vehicle movement with the current directional edge where the vehicle is moving */
  register_vehicle_movement(param, vehicle, current_time, Gr);
}


/** Geometric functions */

double degree2radian(double degree)
{ //convert the angle in degree into the angle in radian
  double radian = degree/180*PI;

  return radian;
}

double radian2degree(double radian)
{ //convert the angle in radian into the angle in degree
  double degree = radian/PI*180;

  return degree;
}

void get_position_on_linear_curve(double v, double t, struct_coordinate1_t *p1, struct_coordinate1_t *p2, struct_coordinate1_t *p)
{ //get the position on the linear curve between two points p1 and p2 for the mobile target moving with speed v after time t, returning the Euclidean coordinate p of vehicle after moving time t, starting from p1 towards p2
  double *x1 = &(p1->x), *y1 = &(p1->y); //vehicle's starting point p1
  double *x2 = &(p2->x), *y2 = &(p2->y); //vehicle's ending point p2
  double *x = &(p->x), *y = &(p->y); //vehicle's current point p
  double a = 0; //slope
  double b = 0; //intercept
  double theta = 0; //angle corresponding to slope a where theta = atan(a);

  if(fabs(*x1 - *x2) <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
  { //in the case where x1 is very close to x2, we consider x1 to be equal to x2
    if(*y1 < *y2)
      theta = degree2radian(90);
    else
      theta = degree2radian(270);
  }
  else
  { //otherwise, that is, x1 != x2
    a = (*y2 - *y1)/(*x2 - *x1); //compute the slope of the line
    theta = atan(a);

    if(*x1 > *x2)
      theta += degree2radian(180); //we need to add 180 degree to let the target's x-coordinte decrease as it moves
  }

  /* compute x- and y-coordinates of the target */
  *x = *x1 + v*t*cos(theta); //compute the x-coordinate of the mobile target
  *y = *y1 + v*t*sin(theta); //compute the y-coordinate of the mobile target
}

void get_position_on_linear_curve_for_offset(double offset, struct_coordinate1_t *p1, struct_coordinate1_t *p2, struct_coordinate1_t *p)
{ //get the position on the linear curve between two points p1 and p2 such that the position is offset away from p1, returning the current Euclidean coordinate p of vehicle moving from p1 to p2
  double *x1 = &(p1->x), *y1 = &(p1->y); //vehicle's starting point p1
  double *x2 = &(p2->x), *y2 = &(p2->y); //vehicle's ending point p2
  double *x = &(p->x), *y = &(p->y); //vehicle's current point p
  double a = 0; //slope
  double b = 0; //intercept
  double theta = 0; //angle corresponding to slope a where theta = atan(a);

  if(fabs(*x1 - *x2) <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
  { //in the case where x1 is very close to x2, we consider x1 to be equal to x2
    if(*y1 < *y2)
      theta = degree2radian(90);
    else
      theta = degree2radian(270);
  }
  else
  { //otherwise, that is, x1 != x2
    a = (*y2 - *y1)/(*x2 - *x1); //compute the slope of the line
    theta = atan(a);

    if(*x1 > *x2)
      theta += degree2radian(180); //we need to add 180 degree to let the target's x-coordinte decrease as it moves
  }

  /* compute x- and y-coordinates of the point for offset */
  *x = *x1 + offset*cos(theta); //compute the x-coordinate of the point for offset
  *y = *y1 + offset*sin(theta); //compute the y-coordinate of the point for offset
}

void get_position_update_vector(struct_vehicle_t *vehicle, struct_graph_node *G, int G_size)
{ //get the position update vector p (i.e., delta-x and delta-y per time t) on the linear curve between two points p1 and p2 for the mobile vehicle moving with speed v per unit time, starting from p1 towards p2
  double v = vehicle->speed; //vehicle speed
  double t = 1; //update duration: 1 second
  struct_coordinate1_t p1; //the coordinate of the tail node of the directed edge (tail, head) where the vehicle is moving
  struct_coordinate1_t p2; //the coordinate of the head node of the directed edge (tail, head) where the vehicle is moving
  struct_coordinate1_t p;  //the coordinate of the vehicle after time t, starting from p1 towards p2
  struct_coordinate1_t vector;  //the position update vector; vehicle add p.x and p.y to its current x-coordinate and y-coordinate every unit time (e.g., STEP_TIME), respectively
  char *tail = NULL; //pointer to the name of the tail node
  char *head = NULL; //pointer to the name of the head node
  struct_graph_node *tail_gnode_in_node_array = NULL; //pointer to the graph node corresponding to the tail node in the node array of G
  struct_graph_node *head_gnode_in_node_array = NULL; //pointer to the graph node corresponding to the head node in the node array of G
  
  if(IsEndOfTravel(vehicle->path_list, vehicle->path_ptr) == TRUE)
  {
    printf("get_position_update_vector(): vehicle is on the last vertex on its path, so we cannot make position update vector!\n");
    exit(1);
  }

  tail = vehicle->path_ptr->vertex;
  head = vehicle->path_ptr->next->vertex;

  tail_gnode_in_node_array = LookupGraph(G, G_size, tail);
  head_gnode_in_node_array = LookupGraph(G, G_size, head);
  
  /* obtain the coordinates of the tail and head nodes */
  memcpy(&p1, &(tail_gnode_in_node_array->coordinate), sizeof(p1));
  memcpy(&p2, &(head_gnode_in_node_array->coordinate), sizeof(p2));

  /* obtain the position update vector */
  get_position_on_linear_curve(v, t, &p1, &p2, &p);

  vector.x = p.x - p1.x;
  vector.y = p.y - p1.y;

  /* set the values of vector.x and vector.y to zero when they are close to zero */
  if(fabs(vector.x) <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    vector.x = 0;

  if(fabs(vector.y) <= ERROR_TOLERANCE_FOR_REAL_ARITHMETIC)
    vector.y = 0;

  /* copy the position update vector to vehicle->pos_update_vector */
  memcpy(&(vehicle->pos_update_vector), &(vector), sizeof(vector));
}

void test_get_position_on_linear_curve()
{ //test whether get_position_on_linear_curve() works well or not
  double v = 20; //vehicle speed = 20 [meter/sec]
  double t = 10; //moving time = 10 sec
  struct_coordinate1_t p1, p2; //start point p1 and end point p2
  struct_coordinate1_t p; //target's position
  
  /* case 1: p1 = (1000, 500), p2 = (2000, 500) */
  p1.x = 1000; p1.y = 500;
  p2.x = 2000; p2.y = 500;  

  get_position_on_linear_curve(v, t, &p1, &p2, &p);
  printf("case 1: p1=(%.0f,%.0f), p2=(%.0f,%.0f), p=(%.0f,%.0f)\n", p1.x, p1.y, p2.x, p2.y, p.x, p.y);

  /* case 2: p1 = (2000, 500), p2 = (1000, 500) */
  p1.x = 2000; p1.y = 500;
  p2.x = 1000; p2.y = 500;  

  get_position_on_linear_curve(v, t, &p1, &p2, &p);
  printf("case 2: p1=(%.0f,%.0f), p2=(%.0f,%.0f), p=(%.0f,%.0f)\n", p1.x, p1.y, p2.x, p2.y, p.x, p.y);

  /* case 3: p1 = (1000, 500), p2 = (1000, 1000) */
  p1.x = 1000; p1.y = 500;
  p2.x = 1000; p2.y = 1000;  

  get_position_on_linear_curve(v, t, &p1, &p2, &p);
  printf("case 3: p1=(%.0f,%.0f), p2=(%.0f,%.0f), p=(%.0f,%.0f)\n", p1.x, p1.y, p2.x, p2.y, p.x, p.y);

  /* case 4: p1 = (1000, 1000), p2 = (1000, 500) */
  p1.x = 1000; p1.y = 1000;
  p2.x = 1000; p2.y = 500;  

  get_position_on_linear_curve(v, t, &p1, &p2, &p);
  printf("case 4: p1=(%.0f,%.0f), p2=(%.0f,%.0f), p=(%.0f,%.0f)\n", p1.x, p1.y, p2.x, p2.y, p.x, p.y);
  
  ////////////////////////////////////////////////

  /* case 5: p1 = (0, 0), p2 = (1000, 1000) */
  p1.x = 0; p1.y = 0;
  p2.x = 1000; p2.y = 1000;  

  get_position_on_linear_curve(v, t, &p1, &p2, &p);
  printf("case 5: p1=(%.0f,%.0f), p2=(%.0f,%.0f), p=(%.0f,%.0f)\n", p1.x, p1.y, p2.x, p2.y, p.x, p.y);

  /* case 6: p1 = (0, 0), p2 = (-1000, 1000) */
  p1.x = 0; p1.y = 0;
  p2.x = -1000; p2.y = 1000;  

  get_position_on_linear_curve(v, t, &p1, &p2, &p);
  printf("case 6: p1=(%.0f,%.0f), p2=(%.0f,%.0f), p=(%.0f,%.0f)\n", p1.x, p1.y, p2.x, p2.y, p.x, p.y);

  /* case 7: p1 = (0, 0), p2 = (-1000, -1000) */
  p1.x = 0; p1.y = 0;
  p2.x = -1000; p2.y = -1000;  

  get_position_on_linear_curve(v, t, &p1, &p2, &p);
  printf("case 7: p1=(%.0f,%.0f), p2=(%.0f,%.0f), p=(%.0f,%.0f)\n", p1.x, p1.y, p2.x, p2.y, p.x, p.y);

  /* case 8: p1 = (0, 0), p2 = (1000, -1000) */
  p1.x = 0; p1.y = 0;
  p2.x = 1000; p2.y = -1000;  

  get_position_on_linear_curve(v, t, &p1, &p2, &p);
  printf("case 8: p1=(%.0f,%.0f), p2=(%.0f,%.0f), p=(%.0f,%.0f)\n", p1.x, p1.y, p2.x, p2.y, p.x, p.y);

}

/** TBD operations */
boolean does_vehicle_convoy_have_packet(struct_vehicle_t *vehicle, parameter_t *param)
{ //check whether the vehicle's convoy (i.e., convoy leader) or the vehicle has packets to forward according to vehicle_vanet_forwarding_type
  int size = 0; //packet queue size
  
  if(param->vehicle_vanet_forwarding_type == VANET_FORWARDING_BASED_ON_CONVOY)
    size = vehicle->ptr_convoy_queue_node->leader_vehicle->packet_queue->size; 
  else
    size = vehicle->packet_queue->size;

  if(size > 0)
    return TRUE;
  else
    return FALSE;
}

boolean does_vehicle_have_packet(struct_vehicle_t *vehicle)
{ //check whether the vehicle has packets to forward
  int size = 0; //packet queue size
  
  size = vehicle->packet_queue->size;

  if(size > 0)
    return TRUE;
  else
    return FALSE;
}

void replace_vehicle_trajectory_with_mobility_list(double arrival_time, parameter_t *param, struct_vehicle_t *vehicle, destination_vehicle_queue_node_t *pQueueNode, struct_graph_node *Gr, int Gr_size)
{ //replace vehicle's current trajectory with pQueueNode's mobility_list as a new vehicle trajectory

  struct_path_node *path_list = NULL; //list of vertices on the path from source to destination
  int path_hop_count; //hop count for the path, i.e., the number of edges
  char *tail_node = NULL; //pointer to tail node name
  char *head_node = NULL; //pointer to head node name
  struct_graph_node *tail_gnode_in_node_array = NULL; //pointer to the graph node corresponding to the tail node in the node array of Gr
  MOVE_TYPE move_type; //movement type for vehicle
  double edge_length = 0; //the length of the edge
  directional_edge_queue_node_t *ptr_directional_edge_node = NULL; //pointer to the directional edge node in directional edge queue DEr for real graph Gr

  /*@ for debugging */
  //if(vehicle->mobility_type == MOBILITY_STATIONARY)
  //    printf("replace_vehicle_trajectory_with_mobility_list(): vid=%d\n", vehicle->id);
  /******************/

  /* delete vehicle's path-list */
  Free_Path_List(vehicle->path_list);
  vehicle->path_list = NULL;
  vehicle->path_ptr = NULL;

  /* set pQueueNode's mobility list to vehicle's path_list as a new trajectory */
  path_list = Make_Forward_Path_List_With_Mobility_List(&(pQueueNode->mobility_list), Gr, Gr_size, &path_hop_count);
  //make a path list with a given mobility list as a new vehicle trajectory  

  /* set vehicle's path direction to forward direction */
  vehicle->path_direction = VEHICLE_PATH_DIRECTION_FORWARD;

  /* set path_list, path_hop_count, path_ptr and path_current_hop */
  vehicle->path_list = path_list;

  /* compute the mean and standard deviation of the arrival time for each path node along path_list */
#if TPD_ACTUAL_VEHICLE_SPEED_USE_FLAG /* [ */ 
  compute_arrival_time_mean_and_standard_deviation_for_path_node_by_vehicle_actual_speed(arrival_time, param, path_list, vehicle); //compute the mean and standard deviation of the arrival time for each path node along path_list by vehicle's actual speed
#else   
  compute_arrival_time_mean_and_standard_deviation_for_path_node(arrival_time, param, path_list); //compute the mean and standard deviation of the arrival time for each path node along path_list by param's vehicle speed
#endif /* ] */

  vehicle->path_hop_count = path_hop_count; //set up path_hop_count for path_list
  vehicle->path_ptr = path_list->next;
  vehicle->path_current_hop = 0; //reset path_current_hop to zero

  /* set the initial position, current position and movement type */
  tail_node = vehicle->path_ptr->vertex;
  head_node = vehicle->path_ptr->next->vertex;
  vehicle->current_pos_in_Gr.eid = vehicle->init_pos_in_Gr.eid = FastGetEdgeID_MoveType(Gr, tail_node, head_node, &move_type, &edge_length, &ptr_directional_edge_node);
  vehicle->move_type = move_type;
  vehicle->edge_length = edge_length;
  if(move_type == MOVE_FORWARD)
      vehicle->current_pos_in_Gr.offset = vehicle->init_pos_in_Gr.offset = 0;
  else if(move_type == MOVE_BACKWARD)
      vehicle->current_pos_in_Gr.offset = vehicle->init_pos_in_Gr.offset = edge_length;
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

  /* compute the position update vector to update the vehicle's geometric position by movement time, such as STEP_TIME */
  get_position_update_vector(vehicle, Gr, Gr_size);
}

void update_vehicle_trajectory_with_mobility_list(double arrival_time, parameter_t *param, struct_vehicle_t *vehicle, destination_vehicle_queue_node_t *pQueueNode, struct_graph_node *Gr, int Gr_size)
{ //set vehicle's current trajectory with pQueueNode's mobility_list as a new vehicle trajectory along with mobility type of either MOBILITY_OPEN or MOBILITY_CLOSED

  struct_path_node *path_list = NULL; //list of vertices on the path from source to destination
  int path_hop_count; //hop count for the path, i.e., the number of edges
  char *tail_node = NULL; //pointer to tail node name
  char *head_node = NULL; //pointer to head node name
  struct_graph_node *tail_gnode_in_node_array = NULL; //pointer to the graph node corresponding to the tail node in the node array of Gr
  MOVE_TYPE move_type; //movement type for vehicle
  double edge_length = 0; //the length of the edge
  mobility_type_t mobility_type; //mobility type
  directional_edge_queue_node_t *ptr_directional_edge_node = NULL; //pointer to the directional edge node in directional edge queue DEr for real graph Gr

  /* set pQueueNode's mobility list to vehicle's path_list as a new trajectory */
  mobility_type = pQueueNode->mobility_type;
  if(mobility_type == MOBILITY_OPEN)
  {
    /* set path_ptr and path_current_hop */
    vehicle->path_ptr = vehicle->path_list->next;
    vehicle->path_current_hop = 0; //reset path_current_hop to zero

	/* set path_list to vehicle->path_list */
	path_list = vehicle->path_list;

    /* compute the mean and standard deviation of the arrival time for each path node along path_list */
#if TPD_ACTUAL_VEHICLE_SPEED_USE_FLAG /* [ */
    compute_arrival_time_mean_and_standard_deviation_for_path_node_by_vehicle_actual_speed(arrival_time, param, path_list, vehicle); //compute the mean and standard deviation of the arrival time for each path node along path_list by vehicle's actual speed
#else   
    compute_arrival_time_mean_and_standard_deviation_for_path_node(arrival_time, param, path_list); //compute the mean and standard deviation of the arrival time for each path node along path_list by param's vehicle speed
#endif /* ] */
  }
  else //mobility_type == MOBILITY_CLOSED
  {
    /* delete vehicle's path-list */
    Free_Path_List(vehicle->path_list);
    vehicle->path_list = NULL;
    vehicle->path_ptr = NULL;

    /* set path list according to vehicle's current path direction */
    if(vehicle->path_direction == VEHICLE_PATH_DIRECTION_FORWARD)
    {
      path_list = Make_Backward_Path_List_With_Mobility_List(&(pQueueNode->mobility_list), Gr, Gr_size, &path_hop_count);

      /* set vehicle's path direction to backward direction */
      vehicle->path_direction = VEHICLE_PATH_DIRECTION_BACKWARD;
    }
    else //vehicle->path_direction == VEHICLE_PATH_DIRECTION_BACKWARD
    {
      path_list = Make_Forward_Path_List_With_Mobility_List(&(pQueueNode->mobility_list), Gr, Gr_size, &path_hop_count);

      /* set vehicle's path direction to forward direction */
      vehicle->path_direction = VEHICLE_PATH_DIRECTION_FORWARD;
    }

    /* set path_list, path_hop_count, path_ptr and path_current_hop */
    vehicle->path_list = path_list;

	/* compute the mean and standard deviation of the arrival time for each path node along path_list */
#if TPD_ACTUAL_VEHICLE_SPEED_USE_FLAG /* [ */
	compute_arrival_time_mean_and_standard_deviation_for_path_node_by_vehicle_actual_speed(arrival_time, param, path_list, vehicle); //compute the mean and standard deviation of the arrival time for each path node along path_list by vehicle's actual speed
#else   
	compute_arrival_time_mean_and_standard_deviation_for_path_node(arrival_time, param, path_list); //compute the mean and standard deviation of the arrival time for each path node along path_list by param's vehicle speed
#endif /* ] */

    vehicle->path_hop_count = path_hop_count; //set up path_hop_count for path_list
    vehicle->path_ptr = path_list->next;
    vehicle->path_current_hop = 0; //reset path_current_hop to zero
  }

  /* set the initial position, current position and movement type */
  tail_node = vehicle->path_ptr->vertex;
  head_node = vehicle->path_ptr->next->vertex;
  vehicle->current_pos_in_Gr.eid = vehicle->init_pos_in_Gr.eid = FastGetEdgeID_MoveType(Gr, tail_node, head_node, &move_type, &edge_length, &ptr_directional_edge_node);
  vehicle->move_type = move_type;
  vehicle->edge_length = edge_length;
  if(move_type == MOVE_FORWARD)
    vehicle->current_pos_in_Gr.offset = vehicle->init_pos_in_Gr.offset = 0;
  else if(move_type == MOVE_BACKWARD)
    vehicle->current_pos_in_Gr.offset = vehicle->init_pos_in_Gr.offset = edge_length;
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

  /* compute the position update vector to update the vehicle's geometric position by movement time, such as STEP_TIME */
  get_position_update_vector(vehicle, Gr, Gr_size);
}

void set_vehicle_speed(parameter_t *param, struct_vehicle_t *vehicle)
{ //set up vehicle speed according to the speed distribution along with vehicle minimum speed and vehicle maximum speed

  /* select vehicle's speed using the speed distribution considering vehicle minimum speed and vehicle maximum speed */
  do
  {
    vehicle->speed = delay_func(param, DELAY_VEHICLE_SPEED);
		
    //@let vehicle's speed be less than param->vehicle_maximum_speed
    if((vehicle->speed >= param->vehicle_minimum_speed) && 
       (vehicle->speed <= param->vehicle_maximum_speed))
      break;
  } while(1);
}

/** logging functions */
void store_forwarding_probability_and_statisticis_into_file(FILE *fp, struct_graph_node *Gr, int Gr_size, directional_edge_queue_t *DEr, char *tail_node, char *head_node)
{ //store the forwarding probability and statistics into the file pointed by fp_1    
    directional_edge_queue_node_t *pEdgeQueueNode = NULL; //pointer to a directional edge queue node 
    probability_and_statistics_queue_t *Q = NULL; //pointer to a probability and statistics queue
    probability_and_statistics_queue_node_t *pProbQueueNode = NULL; //pointer to a probability and statistics queue node
    conditional_forwarding_probability_queue_node_t *pCondQueueNode = NULL; //pointer to a conditional forwarding probability queue node

    int i = 0; //for-loop index
    struct_graph_node *head_gnode = NULL;

    head_gnode = GetNeighborGraphNode(Gr, Gr_size, tail_node, head_node); 

    pEdgeQueueNode = head_gnode->ptr_directional_edge_node;

    Q = &(pEdgeQueueNode->probability_and_statistics_queue);
    pProbQueueNode = &(Q->head);

    for(i = 0; i < Q->size; i++)
    {
        pProbQueueNode = pProbQueueNode->next;

        fprintf(fp, "[%d] timestamp=%.3f:\n", i+1, pProbQueueNode->timestamp);
        fprintf(fp, " mean_interarrival_time=%.3f\n", pProbQueueNode->mean_interarrival_time);
        fprintf(fp, " lambda=%.3f\n", pProbQueueNode->lambda);
        fprintf(fp, " contact_probability=%.3f\n", pProbQueueNode->contact_probability);
        fprintf(fp, " forwarding_probability=%.3f\n", pProbQueueNode->forwarding_probability);
        fprintf(fp, " branch_probability=%.3f\n", pProbQueueNode->branch_probability);
        fprintf(fp, " average_forwarding_probability=%.3f\n", pProbQueueNode->average_forwarding_probability);
        fprintf(fp, " EDD=%.3f\n\n", pProbQueueNode->EDD);
    }
}

boolean is_destination_vehicle(parameter_t *param, struct_vehicle_t *vehicle)
{ //check whether vehicle is a destination vehicle or not
    boolean result = FALSE;

    if(vehicle->id == param->vanet_table.dst_vnode->id)
    {
        result = TRUE;
    }

    return result;
}

/** combination function */
void combinations(int v[], int start, int n, int k, int maxk)
{ /* perform combinations function by enumerating all the possible cases of combination nCk 
   * where n is the number of items and k is the wanted number in the combination
   */
	int i;

	/* k here counts through positions in the maxk-element v.
	 * If k > maxk, then the v is complete and we can use it.
	 */	
	if(k > maxk)
	{
		/* insert code here to use combinations as you please */
		for(i = 1; i <= maxk; i++)
		{
			printf("%i", v[i]);		
		}
		printf("\n");
		return;
	}

	/* For this k-th element of the v, try all start..n elements in
	 * that position.
	 */
	for(i = start; i <= n; i++)
	{
		v[k] = i;
		
		/* Recursively generate combinations of integers from
		 * i+1..n
		 */
		combinations(v, i+1, n, k+1, maxk);
	}
}

int test_combinations(int n, int k) 
{ /* test the combination function called combinations 
   * such that nCk is the number of all the possible cases
   */
	int v[100];
	
	/* Generate all combinations of n elements taken k at a time,
	 * starting with combinations containing 1 in the first position. 
	 */

	combinations(v, 1, n, 1, k);

	return 0;
}

void combinations_for_target_points(int v[], int w[], int start, int n, int k, int maxk, double p[], double d[], parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, packet_queue_node_t *packet, forwarding_table_queue_t *FTQ, target_point_queue_t *global_TPQ)
{ /* evaluate the combinations for target points by enumerating all the possible cases of combination nCk 
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

	int i;
	target_point_queue_t local_TPQ; //local target point queue
	target_point_queue_node_t qnode; //target point queue node
	target_point_queue_node_t *pQueueNode = NULL; //pointer to a target point queue node
	double delivery_probability = 0; //delivery probability from AP to destination_vehicle
	double delivery_delay = 0; //delivery delay from AP to destination_vehicle
	double P_failure = 0; //accumulated delivery-failure probability for multiple target points
	double P_success = 0; //accumulated delivery-success probability for multiple target points
	double sum_of_conditional_delay_expectations = 0; //sum of conditional expectations of the delivery delay for multiple target points
	double average_delivery_delay = 0; //average delivery delay

	/* k here counts through positions in the maxk-element v.
	 * If k > maxk, then the v is complete and we can use it.
	 */	
	if(k > maxk)
	{
		/* initialize the local target point queue */
		InitQueue((queue_t*) &local_TPQ, QTYPE_TARGET_POINT);

		/* insert code here to use combinations as you please */
		for(i = 1; i <= maxk; i++)
		{
			/* initialize queue node */
			qnode.target_point_id = w[v[i]];

			/* compute delivery probability p[v[i]] and delivery delay for target point w[v[i]] */
			//delivery_probability = compute_delivery_probability_along_with_delivery_delay_for_target_point(
			//		qnode.target_point_id, param, current_time, AP_vertex, 
			//		destination_vehicle, FTQ, &delivery_delay);
			/* get delivery_probability and delivery_delay through the look-up of vectors p[] and d[] */
			delivery_probability = p[v[i]];
			delivery_delay = d[v[i]];

#ifdef __DEBUG_COMBINATIONS_FOR_TARGET_POINTS__
			printf("w[%d]=%d (d=%.2f, p=%.2f) ", v[i], w[v[i]], delivery_delay, delivery_probability);
#endif
			/* @for debugging */
			//if((packet->id == 33) && (qnode.target_point_id == 28 || qnode.target_point_id == 35))
			//	printf("combinations_for_target_points(): target_point=%d: delivery_probability=%.2f, delivery_delay=%.2f\n", 
			//		qnode.target_point_id, (float)delivery_probability, (float)delivery_delay);
			/******************/

			/* enqueue target queue node qnode into target queue local_TPQ */
			qnode.delivery_delay = delivery_delay;
			qnode.delivery_probability = delivery_probability;
			Enqueue((queue_t*)&local_TPQ, (queue_node_t*)&qnode);
		}
#ifdef __DEBUG_COMBINATIONS_FOR_TARGET_POINTS__
		printf("\n");
#endif
		/* compute the accumulated delivery-failure probability P_failure for local_TPQ */
		P_failure = 1; //accumuated delivery-failure probability
		pQueueNode = &(local_TPQ.head);
		for(i = 0; i < local_TPQ.size; i++)
		{
			pQueueNode = pQueueNode->next;


			if(pQueueNode->delivery_probability > 0)
			{ //Only when delivery_probability is greater than 0, update sum_of_conditional_delay_expectations and P_failure; otherwise, do not include the corresponding target point in these computations.

				/* update sum_of_conditional_delay_expectations */
				sum_of_conditional_delay_expectations += pQueueNode->delivery_delay * (P_failure * pQueueNode->delivery_probability);

				/* update P_failure */
				P_failure *= (1 - pQueueNode->delivery_probability);
			}
		}

		/* compute the accumulated delivery-success probability P_success for local_TPQ */
		P_success = 1 - P_failure;

		/* check whether local_TPQ has a better performance than global_TPQ in terms of delivery success probability and delivery delay */
		if((global_TPQ->delivery_success_probability < param->communication_packet_delivery_probability_threshold) && (P_success > global_TPQ->delivery_success_probability))
		{ /*@ In the case where global_TPQ cannot satisfy the required delivery probability and local_TPQ has higher delivery probability, local_TPQ replaces global_TPQ. */

			/* compute the average delivery delay for multiple target points in local_TPQ */
			average_delivery_delay = sum_of_conditional_delay_expectations;

			/* replace the target points in global_TPQ with those in local_TPQ */
			ReplaceTargetPointQueue(global_TPQ, &local_TPQ);

			/* update the minimum average delivery delay in global_TPQ along with the corresponding 
			 * delivery success probability: Note that these assignments must be performed after 
			 * ReplaceTargetPointQueue(); otherwise, the performance metric values are erased.
			 */
			global_TPQ->minimum_average_delivery_delay = average_delivery_delay;
			global_TPQ->delivery_success_probability = P_success;
		}
		else if((global_TPQ->delivery_success_probability >= param->communication_packet_delivery_probability_threshold) && (P_success > param->communication_packet_delivery_probability_threshold))
		{ /*@ In the case where global_TPQ can satisfy the required delivery probability and local_TPQ can also satisfy this probability,  when local_TPQ has a shorter average delivery delay than global_TPQ, local_TPQ replaces global_TPQ. */

			/* compute the average delivery delay for multiple target points in local_TPQ */
			average_delivery_delay = sum_of_conditional_delay_expectations;

			/* compare the local TPQ with the global TPQ in order to determine whether
		 	 * the local TPQ needs to replace the global TPQ in terms of the average delivery delay 
		 	 */
			if(average_delivery_delay < global_TPQ->minimum_average_delivery_delay)
			{
				/* replace the target points in global_TPQ with those in local_TPQ */
				ReplaceTargetPointQueue(global_TPQ, &local_TPQ);

				/* update the minimum average delivery delay in global_TPQ along with the corresponding 
				 * delivery success probability: Note that these assignments must be performed after 
				 * ReplaceTargetPointQueue(); otherwise, the performance metric values are erased.
				 */
				global_TPQ->minimum_average_delivery_delay = average_delivery_delay;
				global_TPQ->delivery_success_probability = P_success;
			}
			else
			{
				/* destroy the local TPQ */
				DestroyQueue((queue_t*) &local_TPQ);
			}
		}
		else
		{
			/* destroy the local TPQ */
			DestroyQueue((queue_t*) &local_TPQ);
		}
		return;
	}

	/* For this k-th element of the v, try all start..n elements in
	 * that position.
	 */
	for(i = start; i <= n; i++)
	{
		v[k] = i;
		
		/* Recursively generate combinations of integers from
		 * i+1..n
		 */
		combinations_for_target_points(v, w, i+1, n, k+1, maxk, p, d, param, current_time, AP_vertex, destination_vehicle, packet, FTQ, global_TPQ);
	}
}

double compute_delivery_probability_along_with_delivery_delay_for_target_point(int target_point_id, parameter_t *param, double current_time, char *AP_vertex, struct_vehicle_t *destination_vehicle, forwarding_table_queue_t *FTQ, double *delivery_delay)
{ //Given a target point, compute the delivery probability along with the delivery delay for the given target point
	double optimization_value = 0; //value for target point optimization
	char target_point[NAME_SIZE]; //target point

	double path_distance = 0; //the distance of vehicle's path from the current position to the destination position
	double path_travel_time = 0; //path travel time
	double path_travel_time_deviation = 0; //path travel time deviation

	double EDD_p = 0; //EDD_p is the packet's expected delivery delay to target point p
	double EDD_SD_p = 0; //EDD_p is the packet's expected delivery delay standard deviation to target point p

	double EAD_p = 0; //EAD_p is the destination vehicle's expected arrival delay to target point p
	double EAD_VAR_p = 0; //EAD_VAR_p is the variance of the destination vehicle's arrival delay to target point p
	double EAD_SD_p = 0; //EAD_SD_p is the standard deviation of the destination vehicle's arrival delay to target point p

	double delivery_probability = 0 ; //delivery probability for two delay distributions (EDD_p, EDD_SD_p) and (EAD_p, EAD_SD_p)

	double max_constraint_value = 0; //maximum constraint value to select a target point in the case where there exists no target point to satisfy the required constraint
	int max_constraint_value_target_point_id = 0; //target point corresponding to the maximum constraint value

	double packet_ttl = param->communication_packet_ttl; //packet's TTL

	/* convert target point's integer value into char string */
    itoa(target_point_id, target_point);

	/* compute the path distance, the path travel time, and the path travel time deviation from the destination vehicle's current position to the target point */
	path_distance = Compute_PathDistance_From_CurrentPosition_To_TargetPoint_With_VehiclePathList_Along_With_PathTravelTime_And_Deviation(param, current_time, destination_vehicle, target_point, &path_travel_time, &path_travel_time_deviation);

    /* compute the mean and standard deviation of the travel time duration 
       @Note: make sure that EAD_p is non-zero for the Gamma distribution */
    if(path_travel_time < 0)
    {
		*delivery_delay = INF; //infinite delivery delay indicates that the packet cannot be delivered to the destination vehicle
		return 0;
    }
    else
    {
        EAD_p = path_travel_time; //param->vehicle_speed should be used since it is average vehicle speed
        EAD_SD_p = path_travel_time_deviation;
    }   

    /** compute EDD_p for a target point p */
    /* compute the EDD and EDD_SD for the target point at the intersection having this AP */
    VADD_Get_EDD_And_EDD_SD_For_TargetPoint_At_Intersection(param, target_point_id, AP_vertex, FTQ, &EDD_p, &EDD_SD_p);

    /** Filtering: check the difference between EDD_p and EAD_p with ten times of EAD_p */
    if(EDD_p > EAD_p)
    //if(EDD_p >= EAD_p*10)
    {
		*delivery_delay = INF; //infinite delivery delay indicates that the packet cannot be delivered to the destination vehicle
		return 0;
    }

    /** Compute the probability that the packet will arrive at the target point earlier than the destination vehicle */
    /* Note that the probability is P[T_i^p <= T_i^v] = \int_{0}^{\infy} \int_{0}^{y} {f(x)g(y)}\,dx\,dy */

    /* compute the delay probability and delay delay for the given target point */ 
    *delivery_delay = Compute_TargetPoint_OptimizationValue(param, target_point_id, EDD_p, EDD_SD_p, EAD_p, EAD_SD_p, packet_ttl, &max_constraint_value, &max_constraint_value_target_point_id); //compute the optimization value for an intersection i that is an optimal target point candidate given the the packet delivery delay distribution (EDD_p, EDD_SD_p) and the destination vehicle arrival delay distribution (EAD_p, EAD_SD_p).

	/* set delivery probability */
	delivery_probability = max_constraint_value;

	/* return delivery delay */
	return delivery_probability;
}

/* Note: These macros for MAX and MIN do not work correctly, so I replace them with normal functions */
double MAX_In_Double(double a, double b)
{
	return ((a>=b) ? a : b);
}

double MIN_In_Double(double a, double b)
{
	return ((a<=b) ? a : b);
}

int MAX_In_Integer(int a, int b)
{
	return ((a>=b) ? a : b);
}

int MIN_In_Integer(int a, int b)
{
	return ((a<=b) ? a : b);
}

/** TPD Functions */
char* get_vanet_forwarding_scheme_name(vanet_forwarding_scheme_t scheme)
{ //return the vanet forwarding scheme in string
	static char vanet_forwarding_scheme[NUMBER_OF_VANET_FORWARDING_SCHEME][BUF_SIZE] = {
	STRING_FOR_VANET_FORWARDING_UNKNOWN,
	STRING_FOR_VANET_FORWARDING_VADD,
	STRING_FOR_VANET_FORWARDING_TBD,
	STRING_FOR_VANET_FORWARDING_TPD,
	STRING_FOR_VANET_FORWARDING_EPIDEMIC,
	STRING_FOR_VANET_FORWARDING_TSF,
	STRING_FOR_VANET_FORWARDING_TMA
	};
		
	return vanet_forwarding_scheme[scheme];
}

char* get_data_forwarding_mode_name(data_forwarding_mode_t type)
{ //return the data forwarding mode in string
	static char data_forwarding_mode[NUMBER_OF_DATA_FORWARDING_MODE][BUF_SIZE] = {
		STRING_FOR_DATA_FORWARDING_MODE_UNKNOWN,
		STRING_FOR_DATA_FORWARDING_MODE_DOWNLOAD,
		STRING_FOR_DATA_FORWARDING_MODE_UPLOAD,
		STRING_FOR_DATA_FORWARDING_MODE_V2V
	};

	return data_forwarding_mode[type];
}

struct struct_vehicle* get_vehicle_list()
{ //get the pointer to a static global variable vehicle_list
	return &vehicle_list;
}

boolean compute_arrival_time_mean_and_standard_deviation_for_path_node(double arrival_time,
		parameter_t *param,
		struct_path_node *path_list)
{ //compute the mean and standard deviation of the arrival time for each path node along path_list of a vehicle with speed and speed_standard_deviation by param's vehicle speed
	double speed = param->vehicle_speed; //average vehicle speed
	double speed_standard_deviation = param->vehicle_speed_standard_deviation; //vehicle speed standard deviation

	double unit_length = param->vehicle_unit_length; //unit length (i.e., 1 meter) to compute the mean and variance of the travel time with
	double unit_length_travel_time = param->vehicle_unit_length_mean_travel_time; //travel time for the unit length
	double unit_length_travel_time_variance = param->vehicle_unit_length_travel_time_variance; //travel time variance for the unit length

	struct_path_node *path_ptr = NULL; //pointer to the tail vertex of the current edge along the path of vehicle
	int path_list_size = (int)path_list->weight; //the number of intersections constructing the path for path_list

	double edge_travel_time = 0; //the travel time for an edge
	double edge_travel_time_standard_deviation = 0; //the standard deviation of the travel time for an edge
	double edge_travel_time_variance = 0; //the variance of the travel time for an edge

#if 0 /* [ */
	double edge_travel_time2 = 0; //the travel time for an edge
	double edge_travel_time_variance2= 0; //the variance of the travel time for an edge
#endif /* ] */

	double edge_arrivel_time = 0; //the arrival time at an edge
	double edge_length = 0; //edge length
	double subpath_travel_time = 0; //the travel time for a subpath from the starting path node to the current path node
	double subpath_travel_time_variance = 0; //the variance of the travel time for a subpath from the starting path node to the current path node

#if 0 /* [ */
	double edge_travel_time2 = 0; //the travel time for an edge
	double subpath_travel_time2= 0; //the travel time for a subpath from the starting path node to the current path node
#endif /* ]*/

	int i = 0; //for-loop index

	/* check the validity of speed variables */
	if((speed < VEHICLE_ZERO_SPEED) || 
			(speed_standard_deviation < VEHICLE_ZERO_SPEED))
	{
		return FALSE;
	}

	/* check the validity of path_list_size */
	if(path_list_size == 0)
	{
		printf("%s:%d path_list_size is zero!\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}


	for(path_ptr = path_list->next; path_ptr != path_list;)
	{
		/* compute the travel metrics for the next intersection */
		edge_length = path_ptr->weight;

#if 1 /* [ */
		//Note that the following two estimated measures of edge_travel_time and edge_travel_time_variance are almost the same as those of edge_travel_time and edge_travel_time_variance computed by Gaussian distribution for vehicle speed.
		edge_travel_time = edge_length*unit_length_travel_time;
		edge_travel_time_variance = edge_length*edge_length*unit_length_travel_time_variance;
		edge_travel_time_standard_deviation = sqrt(edge_travel_time_variance);
#else
		GSL_Vanet_Compute_TravelTime_And_Deviation(param, edge_length, &edge_travel_time, &edge_travel_time_standard_deviation); 
		edge_travel_time_variance = pow(edge_travel_time_standard_deviation, 2); //compute the variance of the travel time for the unit length
#endif /* ] */

		/* count the intersection waiting time that is the waiting time of vehicle passing through an intersection area */
		edge_travel_time += param->vehicle_mean_think_time;
		edge_travel_time_variance += param->vehicle_think_time_variance;

		subpath_travel_time += edge_travel_time;
		subpath_travel_time_variance += edge_travel_time_variance;

		/* set the travel metrics for the current intersection for path_ptr */
		path_ptr->expected_arrival_time = arrival_time + subpath_travel_time;
		path_ptr->edge_travel_delay = edge_travel_time;
		path_ptr->edge_travel_delay_standard_deviation = edge_travel_time_standard_deviation;
		path_ptr->travel_time = subpath_travel_time;
		path_ptr->travel_time_standard_deviation = sqrt(subpath_travel_time_variance);

		path_ptr = path_ptr->next;
	}

	return TRUE;
}

boolean compute_arrival_time_mean_and_standard_deviation_for_path_node_by_vehicle_actual_speed(double arrival_time,
		parameter_t *param,
		struct_path_node *path_list,
		struct_vehicle_t *vehicle)
{ //compute the mean and standard deviation of the arrival time for each path node along path_list of a vehicle with speed and speed_standard_deviation by vehicle's actual speed
	double speed = vehicle->speed; //average vehicle speed
	double speed_standard_deviation = vehicle->speed_standard_deviation; //vehicle speed standard deviation

	double unit_length = param->vehicle_unit_length; //unit length (i.e., 1 meter) to compute the mean and variance of the travel time with
	double unit_length_travel_time = vehicle->unit_length_mean_travel_time; //travel time for the unit length
	double unit_length_travel_time_variance = vehicle->unit_length_travel_time_variance; //travel time variance for the unit length

	struct_path_node *path_ptr = NULL; //pointer to the tail vertex of the current edge along the path of vehicle
	int path_list_size = (int)path_list->weight; //the number of intersections constructing the path for path_list

	double edge_travel_time = 0; //the travel time for an edge
	double edge_travel_time_standard_deviation = 0; //the standard deviation of the travel time for an edge
	double edge_travel_time_variance = 0; //the variance of the travel time for an edge

#if 0 /* [ */
	double edge_travel_time2 = 0; //the travel time for an edge
	double edge_travel_time_variance2= 0; //the variance of the travel time for an edge
#endif /* ] */

	double edge_arrivel_time = 0; //the arrival time at an edge
	double edge_length = 0; //edge length
	double subpath_travel_time = 0; //the travel time for a subpath from the starting path node to the current path node
	double subpath_travel_time_variance = 0; //the variance of the travel time for a subpath from the starting path node to the current path node

#if 0 /* [ */
	double edge_travel_time2 = 0; //the travel time for an edge
	double subpath_travel_time2= 0; //the travel time for a subpath from the starting path node to the current path node
#endif /* ]*/

	int i = 0; //for-loop index

	/* check the validity of speed variables */
	if((speed < VEHICLE_ZERO_SPEED) || 
			(speed_standard_deviation < VEHICLE_ZERO_SPEED))
	{
		return FALSE;
	}

	/* check the validity of path_list_size */
	if(path_list_size == 0)
	{
		printf("%s:%d path_list_size is zero!\n",
				__FUNCTION__, __LINE__);
		exit(1);
	}


	for(path_ptr = path_list->next; path_ptr != path_list;)
	{
		/* compute the travel metrics for the next intersection */
		edge_length = path_ptr->weight;

#if 1 /* [ */
		//Note that the following two estimated measures of edge_travel_time and edge_travel_time_variance are almost the same as those of edge_travel_time and edge_travel_time_variance computed by Gaussian distribution for vehicle speed.
		edge_travel_time = edge_length*unit_length_travel_time;
		edge_travel_time_variance = edge_length*edge_length*unit_length_travel_time_variance;
		edge_travel_time_standard_deviation = sqrt(edge_travel_time_variance);
#else
		GSL_Vanet_Compute_TravelTime_And_Deviation_By_VehicleActualSpeed(param, edge_length, &edge_travel_time, &edge_travel_time_standard_deviation, vehicle->speed, vehicle->speed_standard_deviation); 
		edge_travel_time_variance = pow(edge_travel_time_standard_deviation, 2); //compute the variance of the travel time for the unit length
#endif /* ] */

		/* count the intersection waiting time that is the waiting time of vehicle passing through an intersection area */
		edge_travel_time += param->vehicle_mean_think_time;
		edge_travel_time_variance += param->vehicle_think_time_variance;

		subpath_travel_time += edge_travel_time;
		subpath_travel_time_variance += edge_travel_time_variance;

		/* set the travel metrics for the current intersection for path_ptr */
		path_ptr->expected_arrival_time = arrival_time + subpath_travel_time;
		path_ptr->edge_travel_delay = edge_travel_time;
		path_ptr->edge_travel_delay_standard_deviation = edge_travel_time_standard_deviation;
		path_ptr->travel_time = subpath_travel_time;
		path_ptr->travel_time_standard_deviation = sqrt(subpath_travel_time_variance);

		path_ptr = path_ptr->next;
	}

	return TRUE;
}

boolean show_trajectory_and_arrival_time_for_all_vehicles()
{ //show the vehicle trajectory along with the arrival time per path node along the vehicle trajectory for all the vehicles in vehicle_list

	struct struct_vehicle *local_vehicle_list = get_vehicle_list();
	struct struct_vehicle *vehicle = NULL;

	vehicle = local_vehicle_list->next;
	while(vehicle != local_vehicle_list)
	{
		show_trajectory_and_arrival_time_for_vehicle(vehicle->id,
				vehicle->path_list); //show the vehicle trajectory along with the arrival time per path node along the vehicle trajectory

		vehicle = vehicle->next;
	}

	return TRUE;
}

boolean show_trajectory_and_arrival_time_for_vehicle(int vid, 
		struct_path_node *path_list) 
{ //show the vehicle trajectory along with the arrival time per path node along the vehicle trajectory with vehicle id (vid) and vehicle trajectory (path_list)

	struct_path_node *path_ptr = NULL; //pointer to the tail vertex of the current edge along the path of vehicle
	int intersection = 0; //intersection in road network
	double arrival_time = 0; //expected arrival time

	printf("\nvid=%d: ", vid);
	for(path_ptr = path_list->next; path_ptr != path_list;)
	{
		intersection = atoi(path_ptr->vertex);
		arrival_time = path_ptr->expected_arrival_time;

		if(path_ptr->next != path_list)
			printf("%d(%.0f)->", intersection, arrival_time);
		else
			printf("%d(%.0f)\n", intersection, arrival_time);

		path_ptr = path_ptr->next;
	}

	return TRUE;
}

boolean store_trajectory_and_arrival_time_for_all_vehicles()
{ //store the vehicle trajectory along with the arrival time per path node along the vehicle trajectory for all the vehicles in vehicle_list

	struct struct_vehicle *local_vehicle_list = get_vehicle_list();
	struct struct_vehicle *vehicle = NULL;
	FILE *fp = NULL; //file pointer to the file to store vehicle trajectory information

	fp = fopen(TPD_TRAJECTORY_FILE_NAME, "w"); //open file pointer fp for trajectory file
	if(!fp)
	{
		printf("%s:%d: unable to open file \"%s\"\n", 
				__FUNCTION__, __LINE__,
				TPD_TRAJECTORY_FILE_NAME);
		exit(1);
	}

	vehicle = local_vehicle_list->next;
	while(vehicle != local_vehicle_list)
	{
		store_trajectory_and_arrival_time_for_vehicle(fp, vehicle->id,
				vehicle->path_list); //show the vehicle trajectory along with the arrival time per path node along the vehicle trajectory

		vehicle = vehicle->next;
	}

	fclose(fp); //close fp

	return TRUE;
}

boolean store_trajectory_and_arrival_time_for_vehicle(FILE *fp,
		int vid, 
		struct_path_node *path_list) 
{ //store the vehicle trajectory along with the arrival time per path node along the vehicle trajectory with vehicle id (vid) and vehicle trajectory (path_list)

	struct_path_node *path_ptr = NULL; //pointer to the tail vertex of the current edge along the path of vehicle
	int intersection = 0; //intersection in road network
	double arrival_time = 0; //expected arrival time

	fprintf(fp, "\nvid=%d: ", vid);
	for(path_ptr = path_list->next; path_ptr != path_list;)
	{
		intersection = atoi(path_ptr->vertex);
		arrival_time = path_ptr->expected_arrival_time;

		if(path_ptr->next != path_list)
			fprintf(fp, "%d(%.0f)->", intersection, arrival_time);
		else
			fprintf(fp, "%d(%.0f)\n", intersection, arrival_time);

		path_ptr = path_ptr->next;
	}

	return TRUE;
}

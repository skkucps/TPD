/** File: sensor-model.h
	Description: specify the macro constants, structures, and enum types for sensor model.
	Date: 09/19/2007
	Maker: Jaehoon Jeong, jjeong@cs.umn.edu
*/

#ifndef __SENSOR_MODEL_H__
#define __SENSOR_MODEL_H__

#include "common.h"
#include "queue.h"

/* sensor structure */
typedef struct struct_sensor 
{
	int id;   /* sensor id */
	SENSOR_TYPE type; /* sensor type = {INTERSECTION_NODE, NONINTERSECTION_NODE} */
	STATE state; /* sensor state */
	STATE state_before_reschedule; /* sensor state before reschedule */
	double reschedule_time; /* reschedule time is when sensor has changed its sensing schedule due to the sensing hole handling */
	boolean reschedule_flag; /* flag indicating that sensor is rescheduled, so it performs just the state of SENSOR_RESCHEDULE.
				    When this flag is set to on and sensor enters others states other than SENSOR_BORN, the sensor exits 
				    the state without any action. This flag is set to off when the sensor enters SENSOR_RESCHEDULE. */

	/* The following three timestamps and three flags are used to prevent the same event with the same time from being performed more times than once */
	double time_for_state_sensor_estimate; /* time when the sensor will process the state of SENSOR_ESTIMATE */
	double time_for_state_sensor_sense;    /* time when the sensor will process the state of SENSOR_SENSE */
	double time_for_state_sensor_sleep;    /* time when the sensor will process the state of SENSOR_SLEEP */
	double time_for_state_sensor_die;      /* time when the sensor will process the state of SENSOR_DIE */

	boolean flag_for_state_sensor_estimate; /* flag used to indicate that the sensor has performed SENSOR_ESTIMATE after reschedule.
					   	   TRUE indicates that the sensor has processed the event with time time_for_state_sensor_estimate */
	boolean flag_for_state_sensor_sense;    /* flag used to indicate that the sensor has performed SENSOR_SENSE after reschedule. 
					           TRUE indicates that the sensor has processed the event with time time_for_state_sensor_sense */
	boolean flag_for_state_sensor_sleep;    /* flag used to indicate that the sensor has performed SENSOR_SLEEP after reschedule.
					           TRUE indicates that the sensor has processed the event with time time_for_state_sensor_sleep */
	boolean flag_for_state_sensor_die;      /* flag used to indicate that the sensor has performed SENSOR_DIE after reschedule.
					           TRUE indicates that the sensor has processed the event with time time_for_state_sensor_die */

	int seq;  /* sequence number */
	double initial_energy; /* initial energy */
	double energy; /* remaining energy */
	double energy_consumption_rate; /* energy_consumption_rate */
	double warm_up_time; /* warm-up time for sensing devices */
	double turn_on_energy_consumption; /* turn_on_energy_consumption */
	double sensing_range;   /* sensing range */
	double birth_time;   /* birth time is when sensor starts working initially */
	double death_time;   /* death time is when sensor stops sensing due to complete energy consumption */
        double surveillance_start_time; /* surveillance start time starting from the sleeping corresponding to the movement time on the shortest path */
        double surveillance_restart_time; /* surveillance restart time starting from the virtual scanning after rescheduling */
	double sensing_start_time;   /* time when sensor starts sensing in duty cycle */
	double sensing_start_time_for_reschedule;   /* time when sensor starts sensing in duty cycle to compute the energy consumption before reschedule */
	double sensing_end_time;   /* time when sensor stops sensing in duty cycle */
	double sensing_interval;   /* sensing interval in duty cycle */
        double relative_sensing_start_time; /* relative sensing start time in one working period starting from 0. */
        double relative_sensing_end_time;   /* relative sensing end time in one working period starting from 0. */
	double sleeping_start_time; /* time is when sensor starts sleeping in duty cycle */
	double sleeping_end_time;   /* time is when sensor stops sleeping in duty cycle */
	double sleeping_interval; /* sleeping interval in duty cycle = sleeping time + scanning time + alpha */
        double initial_sleeping_interval; /* initial sleeping interval corresponding to the vehicle movement after the surveillance start/restart time */
	struct_coordinate1_t pos; /* current position in the Catesian coordinate system */
	struct_coordinate2_t pos_in_Gr; /* current position in the graph coordinate system (i.e., edge id and relative location) in real graph Gr */
	struct_coordinate2_t pos_in_Gv; /* current position in the graph coordinate system (i.e., edge id and relative location) in virtual graph Gv */
	double com_density; /* communication density */
	double sen_density; /* sensing density */
	double time_sync_error; /* time synchronization error */
        double detection_missing_probability; /* sensor detection missing probability */
	double duplicate_detection_probability; /* sensor duplicate detection probability */
	double recent_detection_time; /* recent detection time */

	/* statistics */
	int duty_cycle_number; /* the number of duty cycles (i.e. sleeping and sensing) that the sensor performed  */
} struct_sensor_t;

#endif

/**
 *  File: mobility.c
    Description: initialize the mobility of destination vehicle(s) with mobility_file_name
    Date: 07/10/2009
    Update Date: 07/10/2009
    Maker: Jaehoon Jeong
*/

#include "stdafx.h"
#include "queue.h"
#include "mobility.h" //structures and related constants for destination vehicle mobility

void init_mobility(char* mobility_file_name, boolean vehicle_id_override_flag, int new_simulation_node_id, destination_vehicle_queue_t* DVQ)
{ //initialize the mobility of destination vehicles with destination vehicle queue DVQ and update the vehicle id with the id starting from new_simulation_node_id according to vehicle_id_override_flag
  FILE *fp; //file pointer
  char input_buf[BUFFER_SIZE]; //input buffer
  size_t input_buf_size; //input buffer size
  char *token; //pointer to a token
  destination_vehicle_queue_node_t queue_node; //destination vehicle queue node
  destination_vehicle_queue_node_t* pQueueNode = NULL; //pointer to a destination vehicle queue node  
  mobility_queue_node_t mobility_queue_node; //mobility queue node
  int destination_vehicle_id = 0; //destination vehicle id
  mobility_type_t mobility_type = MOBILITY_UNKNOWN; //mobility type
  int intersection_id = 0; //intersection id
  int new_destination_vid = new_simulation_node_id; //new destination vehicle id starting from new_simulation_node_id
  /* initialize destination vehicle queue DVQ */
  InitQueue((queue_t*) DVQ, QTYPE_DESTINATION_VEHICLE);

  /* open mobility configuration file called mobility_file_name */
  fp = fopen(mobility_file_name, "r");
  if(!fp)
  {
    printf("init_mobility(): Error: unable to open file \"%s\"\n", mobility_file_name);
    exit(1);
  }

  while(fgets(input_buf, sizeof(input_buf), fp) != NULL) //while-1
  {
    input_buf_size = strlen(input_buf);

    token = (char*) strtok(input_buf, "\n\r");

    if(token == NULL)
    {
      printf("init_mobility(): Error: the key in the mobility configuration file is null!\n");
      exit(1);
    }
		
    if((token[0] == '\n') || (token[0] == '\r') || (token[0] == ' ') || (token[0] == '\t')) //this line is white space
      continue;
    else if(token[0] == '#') //this is comment line
      continue;

    /** obtain a destination vehicle's trajectory for its mobility */
    /* initialize destination queue node */
    memset(&queue_node, 0, sizeof(queue_node));

    /* set vehicle id */
    token = (char*) strtok(input_buf, ":"); //remove ':'
    if(token == NULL)
    {
      printf("Error: vehicle id is omitted!\n");
      exit(1);
    }
    destination_vehicle_id = atoi(token);
    queue_node.vid = destination_vehicle_id;

    /* override vehicle id if vehicle_id_override_flag is set */
    if(vehicle_id_override_flag)
    {
        queue_node.vid = new_destination_vid;
        new_destination_vid++; //increase new_destination_vid by one for the new destination vehicle        
    }

    /* mobility type */
    token = (char*) strtok(NULL, ":"); //remove ':'
    if(token == NULL)
    {
      printf("Error: mobility type is omitted!\n");
      exit(1);
    }
    mobility_type = (mobility_type_t)atoi(token);
    queue_node.mobility_type = mobility_type;
    
    /* vehicle trajectory */
/*     token = (char*) strtok(NULL, "\n\t"); //remove '\n' */
/*     if(token == NULL) */
/*     { */
/*       printf("Error: mobility type is omitted!\n"); */
/*       exit(1); */
/*     } */

    /* insert queue_node into destination vehicle queue */
    pQueueNode = (destination_vehicle_queue_node_t*) Enqueue((queue_t*) DVQ, (queue_node_t*) &queue_node);

    /** obtain the list of neighbor nodes */
    do //do-while-1.1
    {
      /* find an intersection id */
      token = (char*) strtok(NULL, ", \t\n\r");
      if(token == NULL)
	break;

      /* initialize mobility queue node */
      memset(&mobility_queue_node, 0, sizeof(mobility_queue_node));

      intersection_id = atoi(token);
      mobility_queue_node.intersection_id = intersection_id;

      /* insert queue_node into destination vehicle queue */
      Enqueue((queue_t*) &(pQueueNode->mobility_list), (queue_node_t*) &mobility_queue_node);      
    } while(1); //end of do-while-1.1

  } //end of while-1
}

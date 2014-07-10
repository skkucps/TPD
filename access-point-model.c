/** File: access-point-model.c
	Description: implement the operations for Internet access point model.
	Date: 02/24/2009
	Maker: Jaehoon Jeong, jjeong@cs.umn.edu
*/

#include "access-point-model.h"
#include "shortest-path.h"
#include "util.h" //assert_memory()

void AP_Init(struct_access_point_t *AP, parameter_t *param, struct_graph_node* Gr, int Gr_size, char *ap_vertex, int ap_id)
{ //initialize the AP by setting the location in the road network with G and also by setting AP's vertex and id name with ap_vertex and ap_id, respectively 
  int id = atoi(ap_vertex); //intersection id where the AP is placed
  struct_graph_node* pGraphNode = &(Gr[id-1]); //pointer to the graph node for the intersection where the AP is placed
  double delay = 0; //delay for the next event schedule

  //memset(AP, 0, sizeof(*AP)); //@[07/13/09], this memset invalidates the valid pointers in AP already set up through Enqueue()

  AP->id = ap_id;
  strcpy(AP->vertex, ap_vertex);
  memcpy(&(AP->coordinate), &(pGraphNode->coordinate), sizeof(struct_coordinate1_t));
  AP->gnode = pGraphNode;

  /* let AP point to the original road network graph Gr */
  AP->Gr = Gr;
  AP->Gr_size = Gr_size;

  /* initialize the packet queue along with the memory allocation for the packet queue */
  AP->packet_queue = (packet_queue_t*) calloc(1, sizeof(packet_queue_t)); 
  assert_memory(AP->packet_queue);
  InitQueue((queue_t*)AP->packet_queue, QTYPE_PACKET); 

  /** create an augmented graph Ga, edge queue Ea, and directional edge queue DEa for data forwarding */
  /* create an augmented graph Ga including a target point for the data forwarding towards a destination vehicle */
  AP->Ga = Make_Forwarding_Graph(Gr, Gr_size, &(AP->Ga_size)); //make a new forwarding graph for data forwarding used by Access Point (AP) based on the road network graph Gr

  /** construct edge queue AP->Ea and associate it with graph AP->Ga */
  /* construct an edge queue Ea containing the information of edges in real graph Ga. */
  ConstructEdgeQueue(&(AP->Ea), AP->Ga, AP->Ga_size, param);

  /* associate each pair of two adjacent graph nodes (i.e., physical edge) with the corresponding edge entry in Ea */
  AssociateGraphEdgeWithEdgeEntry(AP->Ga, AP->Ga_size, &(AP->Ea));

  /** construct directional edge queue AP->DEa and associate it with graph AP->Ga */
  /* construct a directional edge queue DEa containing the information of directional edges in augmented graph Ga */
  ConstructDirectionalEdgeQueue(param, &(AP->DEa), AP->Ga, AP->Ga_size);

  /* associate each pair of two adjacent graph nodes (i.e., physical edge) with the corresponding directional edge entry in DEa */
  AssociateGraphEdgeWithDirectionalEdgeEntry(AP->Ga, AP->Ga_size, &(AP->DEa));

  /** construct target point table with ap_vertex */
  /* initialize traffic table */
  InitTrafficTable(&(AP->ap_table_for_target_point));

  /* add a target point ap_vertex to the target point table */
  AddTrafficTableEntry(&(AP->ap_table_for_target_point), ap_vertex);
}

void AP_Delete(struct_access_point_t *AP)
{ //delete the AP by freeing the memory allocated to AP

  /* release the memory allocated to packet queue */
  if(AP->packet_queue != NULL)
  { 
    /* delete packet queue */
    DestroyQueue((queue_t*)AP->packet_queue); //free the memory allocated to the packet queue nodes of packet queue
     
    /* free the memory allocated to packet queue */
    free(AP->packet_queue);
  }

  /* delete the augmented graph Ga, edge queue Ea, and directional edge queue DEa for data forwarding */
  Free_Graph(AP->Ga, AP->Ga_size); //release the memory allocated to the augmented graph Ga
  AP->Ga = NULL; 
  AP->Ga_size = 0;

  DestroyQueue((queue_t*) &(AP->Ea)); //destory edge queue Ea
  DestroyQueue((queue_t*) &(AP->DEa)); //destory directional edge queue DEa

  Free_Traffic_Table(&(AP->ap_table_for_target_point)); //release the memory occupied by the traffic AP table in real graph Gr  
}


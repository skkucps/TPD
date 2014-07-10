##!/usr/bin/perl 
## File: vanet_common.pm
## Description: common file for perl script for VANET simulation running
## Maker: Jaehoon Paul Jeong
## Date: Oct., 02, 2008
## Update: July 24, 2010

#use Switch; #include Switch

#@debug option
$DEBUG = 0; #1: display debugging messages and 0: suppress the messages
#$DEBUG = 1; #1: display debugging messages and 0: suppress the messages
#$DEBUG_1 = 1; #1: display debugging messages and 0: suppress the messages

#$DEBUG_SD = 1; #for debugging the computation of standard deviation

#@constants #
#@comparison targets
$CONST_COMPARISON_EDD_AND_LINK_MODEL = 1;
$CONST_COMPARISON_EDD_MODEL = 2;
$CONST_COMPARISON_TBD_EDD_COMPUTATION_TYPE = 3;
$CONST_COMPARISON_EDGE_DELAY_MODEL = 4;
$CONST_COMPARISON_INTERSECTION_FORWARDING_TYPE = 5;
$CONST_COMPARISON_AVERAGE_CONVOY_LENGTH_ESTIMATION_TYPE = 6; #estimation type for the Average Convoy Length (ACL) on a finite road segment
$CONST_COMPARISON_AVERAGE_LINK_DELAY_ESTIMATION_TYPE = 7; #estimation type for Average Link Delay (ALD) on a finite road segment
$CONST_COMPARISON_TARGET_POINT_SELECTION_TYPE = 8; #target point selection type
$CONST_COMPARISON_TARGET_POINT_COMPUTATION_METHOD = 9; #target point computation method
$CONST_COMPARISON_TARGET_POINT_NUMBER = 10; #target point number to send packet copies

#@evaluation types
$CONST_EVALUATION_VEHICLE_MAXIMUM_NUMBER = 1;
$CONST_EVALUATION_VEHICLE_PACKET_GENERATING_ENTITY_NUMBER = 2;
$CONST_EVALUATION_VEHICLE_SPEED = 3;
$CONST_EVALUATION_VEHICLE_SPEED_STANDARD_DEVIATION = 4;
$CONST_EVALUATION_VEHICLE_INTERARRIVAL_TIME = 5;
$CONST_EVALUATION_COMMUNICATION_PACKET_INTERARRIVAL_TIME = 6; #1/packet_sending_rate
$CONST_EVALUATION_COMMUNICATION_PACKET_TTL = 7; #packet time-to-live (TTL), that is, packet lifetime
$CONST_EVALUATION_COMMUNICATION_PACKET_DELIVERY_PROBABILITY_THRESHOLD = 8; #packet delivery probability threshold
$CONST_EVALUATION_COMMUNICATION_AP_MAXIMUM_NUMBER = 9; #maximum number of APs
$CONST_EVALUATION_SIMULATION_TIME = 10; #simulation time
$CONST_EVALUATION_VEHICLE_AP_PASSING_ENTITY_PERCENTAGE = 11; #vehicle AP passing entity percentage
$CONST_EVALUATION_COMMUNICATION_SN_MAXIMUM_NUMBER = 12; #maximum number of Stationary Nodes (SNs)

#@distribution types
$CONST_DISTRIBUTION_EQUAL = 1;
$CONST_DISTRIBUTION_UNIFORM = 2;
$CONST_DISTRIBUTION_NORMAL = 3;
$CONST_DISTRIBUTION_EXPONENTIAL = 4;

#@Data Forwarding Mode
$CONST_DATA_FORWARDING_MODE_DOWNLOAD = 1;
$CONST_DATA_FORWARDING_MODE_UPLOAD = 2;

#@EDD-Link model types
$CONST_TBD_EDD_AND_TBD_LINK = 1;
$CONST_TBD_EDD_AND_VADD_LINK = 2;
$CONST_VADD_EDD_AND_VADD_LINK = 3;
$CONST_VADD_EDD_AND_TBD_LINK = 4;

#@EDD model types
$CONST_EDD_MODEL_PER_VEHICLE = 1;
$CONST_EDD_MODEL_PER_INTERSECTION = 2;
$CONST_EDD_MODEL_HYBRID = 3;

#@TBD EDD computation types
$CONST_TBD_EDD_COMPUTATION_TYPE_FULL_PATH = 1;
$CONST_TBD_EDD_COMPUTATION_TYPE_PARTIAL_PATH = 2;
$CONST_TBD_EDD_COMPUTATION_TYPE_ONE_HOP_AHEAD = 3;

#@Edge delay model types
$CONST_EDGE_DELAY_MODEL_TBD_FOR_FINITE_ROAD = 1;
$CONST_EDGE_DELAY_MODEL_TBD_FOR_INFINITE_ROAD = 2;
$CONST_EDGE_DELAY_MODEL_VADD = 3;
$CONST_EDGE_DELAY_MODEL_TSF = 4;

#@Intersection Forwarding types
$CONST_VANET_INTERSECTION_FORWARDING_DEFERRED_FORWARDING = 1;
$CONST_VANET_INTERSECTION_FORWARDING_EAGER_FORWARDING = 2;

#@Vehicular traffic model types
$CONST_VANET_VEHICULAR_TRAFFIC_CLOSED_NETWORK = 1;
$CONST_VANET_VEHICULAR_TRAFFIC_OPEN_NETWORK = 2;

#@Target point selection types
$CONST_VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT = 1; #optimal target point selection based on the packet trajectory with stationary nodes

$CONST_VANET_TARGET_POINT_SELECTION_STATIC_TARGET_POINT = 2; #Static target point selection
$CONST_VANET_TARGET_POINT_SELECTION_CONVERGENT_TARGET_POINT = 3; #Convergent target point selection
$CONST_VANET_TARGET_POINT_SELECTION_REVERSE_PATH_TARGET_POINT = 4; #reverse target point selection that the target point is determined as an intersection from the reverse path on the destination vehicle's trajectory
$CONST_VANET_TARGET_POINT_SELECTION_PROGRESS_TARGET_POINT = 5; #PROGRESS target point selection
$CONST_VANET_TARGET_POINT_SELECTION_ADAPTIVE_TARGET_POINT = 6; #Adaptive target point selection
$CONST_VANET_TARGET_POINT_SELECTION_DYNAMIC_TARGET_POINT = 7; #Dynamic target point selection

#@Target point computation methods
$CONST_VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_PARTIALLY_DYNAMIC_FORWARDING = 1; #target point computation method for an optimal target point, based on the packet trajectory with the partially dynamic forwarding through stationary nodes
$CONST_VANET_TARGET_POINT_COMPUTATION_METHOD_END_INTERSECTION = 2; #target point computation method to choose the destination's end intersection in the vehicle trajectory as target point
$CONST_VANET_TARGET_POINT_COMPUTATION_METHOD_RANDOM_INTERSECTION = 3; #target point computation method to choose a random intersection among the intersections on the destination vehicle trajectory

$CONST_VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_STATIC_FORWARDING = 4; #target point computation method for an optimal target point, based on the packet trajectory with the static forwarding through stationary nodes
$CONST_VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_FULLY_DYNAMIC_FORWARDING = 5; #target point computation method for an optimal target point, based on the packet trajectory with the fully dynamic forwarding through stationary nodes
$CONST_VANET_TARGET_POINT_COMPUTATION_METHOD_OPTIMAL_INTERSECTION = 6; #target point computation method for an optimal target point 
$CONST_VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_EARLIEST_ARRIVING_INTERSECTION = 7; #target point computation method for a target point to choose an intersection where the packet arrives at the intersection earlier than the destination vehicle with the smallest EDD
$CONST_VANET_TARGET_POINT_COMPUTATION_METHOD_HEADING_INTERSECTION = 8; #target point computation method to choose the destination's heading intersection as target point

#@Vehicle trajectory length types
$CONST_VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE = 1; #infinite vehicle trajectory length
$CONST_VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE = 2; #finite vehicle trajectory length

#@default values for parameters @#
$default_data_forwarding_mode = $CONST_DATA_FORWARDING_MODE_DOWNLOAD; #data forwarding mode is download from AP to vehicle

#@ graph-configuration file
#$default_road_network_size = 9; #road network with 9 intersections
#$default_road_network_size = 36; #road network with 36 intersections
$default_road_network_size = 49; #road network with 49 intersections

#@@For Upload Mode
#$default_vehicle_maximum_number = 100;

#@@For Download Mode
#@for the delivery probability threshold
#$default_vehicle_maximum_number = 50;

#@for the multiple APs
#$default_vehicle_maximum_number = 250;

#@for the multiple SNs
$default_vehicle_maximum_number = 100;

#@for the other parameters
#$default_vehicle_maximum_number = 250;
#######################################

#@@For Upload Mode
#$default_vehicle_packet_generating_entity_number = 10;

#@@For Download Mode
$default_vehicle_packet_generating_entity_number = 0;

$default_vehicle_AP_passing_entity_percentage=0;

$default_vehicle_speed = 40; #unit is MPH.
#$default_vehicle_speed_standard_deviation = 7; #unit is MPH.
$default_vehicle_speed_standard_deviation = 5; #unit is MPH.
#$default_vehicle_speed_standard_deviation = 3.5; #unit is MPH.

$default_communication_packet_interarrival_time = 5; #unit is seconds.
#$default_communication_packet_interarrival_time = 1; #unit is seconds.

$default_communication_packet_ttl = 0; #unit is seconds.

#@@For Upload Mode
#$default_communication_packet_ttl_for_delivery_delay = 50000; #TTL for delivery delay comparison; unit is seconds.
#$default_communication_packet_ttl_for_delivery_ratio = 3600; #TTL for delivery ratio comparison; unit is seconds.

#@@For Download Mode
$default_communication_packet_ttl_for_delivery_delay = 2100; #TTL for delivery delay comparison; unit is seconds.
$default_communication_packet_ttl_for_delivery_ratio = 2100; #TTL for delivery ratio comparison; unit is seconds.

#@@For Upload Mode
#$default_communication_packet_maximum_number = 50000; #the number of packets generated by APs or vehicles

#@@For Download Mode
$default_communication_packet_maximum_number = 2000; #the number of packets generated by APs or vehicles

$default_communication_packet_delivery_probability_threshold = 0.95; #packet delivery probability threshold
#$default_communication_packet_delivery_probability_threshold = 0.9; #packet delivery probability threshold

$default_communication_AP_maximum_number = 1; #the number of APs

#@@For Upload Mode
#$default_simulation_time = 3600*11; #unit is seconds; note that after 1 hour, packets are usually generated

#@@For Download Mode
$default_simulation_time = 3600*6; #unit is seconds; note that after 1 hour, packets are usually generated

#@flag to control parameters
#$default_distribution_for_vehicle_speed = $CONST_DISTRIBUTION_EQUAL;
$default_distribution_for_vehicle_speed = $CONST_DISTRIBUTION_NORMAL;

$default_vehicle_vanet_vehicular_traffic_model = $CONST_VANET_VEHICULAR_TRAFFIC_CLOSED_NETWORK;

$default_edd_and_link_model = $CONST_VADD_EDD_AND_TBD_LINK;
#$default_edd_and_link_model = $CONST_TBD_EDD_AND_TBD_LINK;

$default_edd_model = $CONST_EDD_MODEL_PER_INTERSECTION;
#$default_edd_model = $CONST_EDD_MODEL_PER_VEHICLE;

$default_tbd_edd_computation_type = $CONST_TBD_EDD_COMPUTATION_TYPE_PARTIAL_PATH;

#@set the edge delay model to the TSF link delay model

#@@For Upload Mode
#$default_edge_delay_model = $CONST_EDGE_DELAY_MODEL_TBD_FOR_FINITE_ROAD;

#@@For Download Mode
$default_edge_delay_model = $CONST_EDGE_DELAY_MODEL_TSF;

$default_intersection_forwarding_type = $CONST_VANET_INTERSECTION_FORWARDING_EAGER_FORWARDING; 
$default_acl_estimation_type = $CONST_EDGE_DELAY_MODEL_TBD;

$default_target_point_selection_type = $CONST_VANET_TARGET_POINT_SELECTION_PACKET_TRAJECTORY_TARGET_POINT;
#$default_target_point_selection_type = $CONST_VANET_TARGET_POINT_SELECTION_PROGRESS_TARGET_POINT;
#$default_target_point_selection_type = $CONST_VANET_TARGET_POINT_SELECTION_ADAPTIVE_TARGET_POINT;

$default_target_point_computation_method = $CONST_VANET_TARGET_POINT_COMPUTATION_METHOD_PACKET_TRAJECTORY_PARTIALLY_DYNAMIC_FORWARDING;
#$default_target_point_computation_method = $CONST_VANET_TARGET_POINT_COMPUTATION_METHOD_OPTIMAL_INTERSECTION;

#$default_vehicle_trajectory_length_type =$CONST_VANET_VEHICLE_TRAJECTORY_LENGTH_INFINITE;
$default_vehicle_trajectory_length_type =$CONST_VANET_VEHICLE_TRAJECTORY_LENGTH_FINITE;

$default_multiple_APs_flag = 0; #flag to indicate whether the graph configuration for multiple APs is used or not

#@the configuration structure of parameter: (base, number_of_values, step, default_value, current_value)

#* Comparison target types
@EDD_AND_LINK_MODEL = (1, 4, 1, 1, 0); #EDD-Link model
@EDD_MODEL = (1, 2, 1, 1, 0); #EDD model
@TBD_EDD_COMPUTATION_TYPE = (1, 3, 1, 1, 0); #TBD EDD computation type
@EDGE_DELAY_MODEL = (1, 3, 1, 1, 0); #Edge delay model
@INTERSECTION_FORWARDING_TYPE = (1, 2, 1, 1, 0); #Intersection Forwarding type
#@TARGET_POINT_SELECTION_TYPE = (1, 3, 1, 1, 0); #Target point selection type
@TARGET_POINT_SELECTION_TYPE = (1, 2, 1, 1, 0); #Target point selection type
@TARGET_POINT_COMPUTATION_METHOD = (1, 3, 1, 1, 0); #Target point computation method
@TARGET_POINT_NUMBER = (1, 3, 1, 1, 0); #Target point computation method

#* Evaluation type parameters
#@SEEDS = (1, 5, 1, 1, 0); #seed

#@for TMC'11 Version
#@SEEDS = (1, 10, 1, 1, 0); #seed

#@for Multi-target-point Data Forwarding
#@SEEDS = (1, 20, 1, 1, 0); #seed

#@@For Download Mode: For Multiple APs
#@SEEDS = (1, 50, 1, 1, 0); #seed
#@SEEDS = (1, 20, 1, 1, 0); #seed

#@@For Download Mode: For Multiple SNs
@SEEDS = (1, 50, 1, 1, 0); #seed
#@SEEDS = (1, 20, 1, 1, 0); #seed

#@@For Upload Mode
#@VEHICLE_MAXIMUM_NUMBER_1 = (20, 10, 20, 100, 0); #vehicle maximum number set 1 for low traffic density

#@@For Upload Mode
@VEHICLE_MAXIMUM_NUMBER_2 = (200, 10, 100, 200, 0); #vehicle maximum number set 2 for high traffic density

#@@For Download Mode for TMC'11
@VEHICLE_MAXIMUM_NUMBER_1 = (50, 10, 50, 100, 0); #vehicle maximum number set 1 for low traffic density

#@@For Download Mode and Multi-target-point Data Forwarding
#@VEHICLE_MAXIMUM_NUMBER_1 = (30, 12, 10, 100, 0); #vehicle maximum number set 2 for extremely low traffic density


@VEHICLE_MAXIMUM_NUMBER = @VEHICLE_MAXIMUM_NUMBER_1; #vehicle maximum number set
#@VEHICLE_MAXIMUM_NUMBER = @VEHICLE_MAXIMUM_NUMBER_2; #vehicle maximum number set
@VEHICLE_PACKET_GENERATING_ENTITY_NUMBER = (10, 10, 10, 10, 0); #number of packet generating vehicles
@VEHICLE_SPEED = (20, 9, 5, 40, 0); #vehicle speed: unit is mile per hour [mph]
#@VEHICLE_SPEED_STANDARD_DEVIATION = (0, 11, 1, 0, 0); #vehicle speed standard deviation: unit is mile per hour [mph]
@VEHICLE_SPEED_STANDARD_DEVIATION = (1, 10, 1, 0, 0); #vehicle speed standard deviation: unit is mile per hour [mph]
@VEHICLE_INTERARRIVAL_TIME = (2, 10, 2, 0, 0); #vehicle interarrival time: unit is seconds

@COMMUNICATION_PACKET_INTERARRIVAL_TIME = (1, 10, 1, 5, 0); #packet interarrival time
@COMMUNICATION_PACKET_TTL = (1000, 10, 1000, 10000, 0); #packet TTL

#@COMMUNICATION_AP_MAXIMUM_NUMBER = (1, 9, 1, 1, 0); #the number of APs
#@COMMUNICATION_AP_MAXIMUM_NUMBER = (1, 16, 1, 1, 0); #the number of APs
@COMMUNICATION_AP_MAXIMUM_NUMBER = (1, 13, 1, 1, 0); #the number of APs

@COMMUNICATION_SN_MAXIMUM_NUMBER = (31, 10, 2, 1, 0); #the number of APs

@COMMUNICATION_PACKET_DELIVERY_PROBABILITY_THRESHOLD = (0.55, 9, 0.05, 5, 0); #packet delivery probability threshold

@SIMULATION_TIME = (3600*2, 10, 3600, 3600*5, 0); #simulation time
@VEHICLE_AP_PASSING_ENTITY_PERCENTAGE = (0, 11, 10, 0, 0); #vehicle AP passing entity percentage

#@the configuration structure of test parameter that has a value at the horizontal axis: (base, number_of_values, value_list, default_value, current_value)
@VALUE_ARRAY = ();
@TEST_PARAM = (0, 0, [@VALUE_ARRAY], 0, 0);

##############################################

local $default_comparison_target = 1; #EDD-Link model
$comparison_target = 0; #performance comparison target

local $default_evaluation_type = 1; #vehicle maximum number
$evaluation_type = 0; #simulation evaluation type

##############################################

#@select the graph configuration for the number of APs
print("Select the multiple_APs_flag [0: A single AP, 1: Multiple APs] (default: 0): ");
$multiple_APs_flag = <STDIN>;
chop($multiple_APs_flag);

if($multiple_APs_flag eq "")
{
    $default_multiple_APs_flag = 0;
}
elsif($multiple_APs_flag == 0 || $multiple_APs_flag == 1)
{
    $default_multiple_APs_flag = $multiple_APs_flag;
}
else
{
    print("multiple_APs_flag of $multiple_APs_flag is not supported!\n");
    exit;
}

#@select the vehicular traffic density
print("Select the vehicular traffic density [1: Light Traffic Density, 2: Heavy Traffic Density] (default: 1): ");
$vehicular_traffic_density = <STDIN>;
chop($vehicular_traffic_density);

if($vehicular_traffic_density eq "")
{
    @VEHICLE_MAXIMUM_NUMBER = @VEHICLE_MAXIMUM_NUMBER_1; #vehicle maximum number set for light traffic density
}
elsif($vehicular_traffic_density == 1)
{
    @VEHICLE_MAXIMUM_NUMBER = @VEHICLE_MAXIMUM_NUMBER_1; #vehicle maximum number set for light traffic density
}
elsif($vehicular_traffic_density == 2)
{
    @VEHICLE_MAXIMUM_NUMBER = @VEHICLE_MAXIMUM_NUMBER_2; #vehicle maximum number set for heavy traffic density
}
else
{
    print("vehicular_traffic_density of $vehicular_traffic_density is not supported!\n");
    exit;
}

#@select performance evaluation objective
print("Select the performance evaluation objective [1: Delivery Delay, 2: Delivery Ratio] (default: 1): ");
$evaluation_objective = <STDIN>;
chop($evaluation_objective);

if($evaluation_objective eq "")
{
    $default_communication_packet_ttl = $default_communication_packet_ttl_for_delivery_delay;
}
elsif($evaluation_objective < 1 || $evaluation_ojective > 2)
{
    print("evaluation objective of $evaluation_objective is not supported!\n");
    exit;
}
elsif($evaluation_objective == 1)
{
    $default_communication_packet_ttl = $default_communication_packet_ttl_for_delivery_delay;
}
elsif($evaluation_objective == 2)
{
    $default_communication_packet_ttl = $default_communication_packet_ttl_for_delivery_ratio;
}

#@select comparison target
print("Put the comparison target [1: EDD-Link model, 2: EDD model, 3: TBD's EDD computation type, 4: Edge delay model, 5: Intersection Forwarding type, 6: Average Convoy Length estimation type for one road segment, 7: Average Link Delay estimation for one road segment, 8: Target Point Selection type, 9: Target Point Computation Method, 10: Target Point Number] (default: 1): ");
$comparison_target = <STDIN>;
chop($comparison_target);

if($comparison_target eq "")
{
    $comparison_target = $default_comparison_target;
}
elsif($comparison_target < 1 || $comparison_target > 10)
{
    print("comparison target of $comparison_target is not supported!\n");
    exit;
}
elsif($comparison_target == $CONST_COMPARISON_AVERAGE_LINK_DELAY_ESTIMATION_TYPE)
{
    #let the vehicle speed be determined as either constant or variable
    print("Select the vehicle speed distribution option [1: Constant Speed and 2: Variable Speed] (default: 1): ");
    $vehicle_speed_distribution_option = <STDIN>;
    chop($vehicle_speed_distribution_option);

    if($vehicle_speed_distribution_option eq "")
    {
	$vehicle_speed_distribution_option = 1;
	$default_distribution_for_vehicle_speed = $CONST_DISTRIBUTION_EQUAL;
    }
    elsif($vehicle_speed_distribution_option < 1 || $vehicle_speed_distribution_option > 2)
    {
	print("vehicle_speed_distribution_option of $vehicle_speed_distribution_option is not supported!\n");
	exit;
    }
    elsif($vehicle_speed_distribution_option == 1)
    {
	$default_distribution_for_vehicle_speed = $CONST_DISTRIBUTION_EQUAL;
    }
    elsif($vehicle_speed_distribution_option == 2)
    {
	$default_distribution_for_vehicle_speed = $CONST_DISTRIBUTION_NORMAL;
    }

    print("\nvehicle_speed_distribution_option is $vehicle_speed_distribution_option\n");
}

print("\ncomparison target is $comparison_target\n\n");

#@select evaluation type
print("\nPut the evaluation type [1: vehicle maximum number, 2: vehicle packet generating entity number, 3: vehicle speed, 4: vehicle speed standard deviation, 5: vehicle interarrival time, 6: communication packet interarrival time, 7: communication packet TTL, 8: communication packet delivery probability threshold, 9: AP maximum number, 10: simulation time, 11: AP passing vehicle percentage, 12: SN maximum number] (default: 1): ");
$evaluation_type = <STDIN>;
chop($evaluation_type); 
#chop() removes and returns the last character from the given string of $evaluation_type; in this line, the removed character is '\n'.

if($evaluation_type eq "")
{
    $evaluation_type = $default_evaluation_type;
}
elsif($evaluation_type < 1 || $evaluation_type > 12)
{
    print("evaluation type of $evaluation_type is not supported!\n");
    exit;
}

print("\nevaluation type is $evaluation_type\n\n");


#@configuration array consisting of three sub-arrays with comparison target
if($comparison_target == $CONST_COMPARISON_EDD_AND_LINK_MODEL) #EDD-Link model
{
    @Config = ([@SEEDS], [@TEST_PARAM], [@EDD_AND_LINK_MODEL]);
}
elsif($comparison_target == $CONST_COMPARISON_EDD_MODEL) #EDD model
{
    @Config = ([@SEEDS], [@TEST_PARAM], [@EDD_MODEL]);
}
elsif($comparison_target == $CONST_COMPARISON_TBD_EDD_COMPUTATION_TYPE) #TBD's EDD computation type
{
    @Config = ([@SEEDS], [@TEST_PARAM], [@TBD_EDD_COMPUTATION_TYPE]);
}
elsif($comparison_target == $CONST_COMPARISON_EDGE_DELAY_MODEL) #Edge delay model
{
    @Config = ([@SEEDS], [@TEST_PARAM], [@EDGE_DELAY_MODEL]);
}
elsif($comparison_target == $CONST_COMPARISON_INTERSECTION_FORWARDING_TYPE) #Intersection Forwarding type
{
    @Config = ([@SEEDS], [@TEST_PARAM], [@INTERSECTION_FORWARDING_TYPE]);
}
elsif($comparison_target == $CONST_COMPARISON_AVERAGE_CONVOY_LENGTH_ESTIMATION_TYPE) #Average Convoy Length Estimation Type
{
    @Config = ([@SEEDS], [@TEST_PARAM], [@EDGE_DELAY_MODEL]);
}
elsif($comparison_target == $CONST_COMPARISON_AVERAGE_LINK_DELAY_ESTIMATION_TYPE) #Average Link Delay Estimation Type
{
    @Config = ([@SEEDS], [@TEST_PARAM], [@EDGE_DELAY_MODEL]);
}
elsif($comparison_target == $CONST_COMPARISON_TARGET_POINT_SELECTION_TYPE) #Target Point Selection Type
{
    @Config = ([@SEEDS], [@TEST_PARAM], [@TARGET_POINT_SELECTION_TYPE]);
}
elsif($comparison_target == $CONST_COMPARISON_TARGET_POINT_COMPUTATION_METHOD) #Target Point Computation Method
{
    @Config = ([@SEEDS], [@TEST_PARAM], [@TARGET_POINT_COMPUTATION_METHOD]);
}
elsif($comparison_target == $CONST_COMPARISON_TARGET_POINT_NUMBER) #Target Point Number
{
    @Config = ([@SEEDS], [@TEST_PARAM], [@TARGET_POINT_NUMBER]);
}

#@set test parameters with Config[1][2] according to $simulation_type
if($evaluation_type == $CONST_EVALUATION_VEHICLE_MAXIMUM_NUMBER) #if-1: working time
{
    for($i = 0; $i < $VEHICLE_MAXIMUM_NUMBER[1]; $i++)
    {
	$test_param = $VEHICLE_MAXIMUM_NUMBER[0] + $VEHICLE_MAXIMUM_NUMBER[2]*$i;
	push(@{$Config[1][2]}, $test_param);
	$Config[1][1]++;
    }

    #set $Config[1][2] with @VEHICLE_MAXIMUM_NUMBER_1
    #for($i = 0; $i < $VEHICLE_MAXIMUM_NUMBER_1[1]; $i++)
    #{
	#$test_param = $VEHICLE_MAXIMUM_NUMBER_1[0] + $VEHICLE_MAXIMUM_NUMBER_1[2]*$i;
	#push(@{$Config[1][2]}, $test_param);
	#$Config[1][1]++;
    #}

    #set $Config[1][2] with VEHICLE_MAXIMUM_NUMBER_2
    #for($i = 0; $i < $VEHICLE_MAXIMUM_NUMBER_2[1]; $i++)
    #{
	#$test_param = $VEHICLE_MAXIMUM_NUMBER_2[0] + $VEHICLE_MAXIMUM_NUMBER_2[2]*$i;
	#push(@{$Config[1][2]}, $test_param);
	#$Config[1][1]++;
    #}
}#end of if-1
elsif($evaluation_type == $CONST_EVALUATION_VEHICLE_PACKET_GENERATING_ENTITY_NUMBER) #if-2: vehicle packet generating entity number
{
    #set $Config[1][2] with VEHICLE_PACKET_GENERATING_ENTITY_NUMBER
    for($i = 0; $i < $VEHICLE_PACKET_GENERATING_ENTITY_NUMBER[1]; $i++)
    {
	$test_param = $VEHICLE_PACKET_GENERATING_ENTITY_NUMBER[0] + $VEHICLE_PACKET_GENERATING_ENTITY_NUMBER[2]*$i;
	push(@{$Config[1][2]}, $test_param);
	$Config[1][1]++;
    }
} #end of if-2
elsif($evaluation_type == $CONST_EVALUATION_VEHICLE_SPEED) #if-3: vehicle speed
{
    #set $Config[1][2] with VEHICLE_SPEED
    for($i = 0; $i < $VEHICLE_SPEED[1]; $i++)
    {
	$test_param = $VEHICLE_SPEED[0] + $VEHICLE_SPEED[2]*$i;
	push(@{$Config[1][2]}, $test_param);
	$Config[1][1]++;
    }
} #end of if-3
elsif($evaluation_type == $CONST_EVALUATION_VEHICLE_SPEED_STANDARD_DEVIATION) #if-4: vehicle speed standard deviation
{
    #set $Config[1][2] with VEHICLE_SPEED_STANDARD_DEVIATION
    for($i = 0; $i < $VEHICLE_SPEED_STANDARD_DEVIATION[1]; $i++)
    {
	$test_param = $VEHICLE_SPEED_STANDARD_DEVIATION[0] + $VEHICLE_SPEED_STANDARD_DEVIATION[2]*$i;
	push(@{$Config[1][2]}, $test_param);
	$Config[1][1]++;
    }
} #end of if-4
elsif($evaluation_type == $CONST_EVALUATION_VEHICLE_INTERARRIVAL_TIME) #if-5: vehicle interarrival time
{
    #set $Config[1][2] with VEHICLE_INTERARRIVAL_TIME
    for($i = 0; $i < $VEHICLE_INTERARRIVAL_TIME[1]; $i++)
    {
	$test_param = $VEHICLE_INTERARRIVAL_TIME[0] + $VEHICLE_INTERARRIVAL_TIME[2]*$i;
	push(@{$Config[1][2]}, $test_param);
	$Config[1][1]++;
    }
} #end of if-5
elsif($evaluation_type == $CONST_EVALUATION_COMMUNICATION_PACKET_INTERARRIVAL_TIME) #if-6: communication packet interarrival time (i.e., 1/sending_rate)
{
    #set $Config[1][2] with COMMUNICATION_PACKET_INTERARRIVAL_TIME
    for($i = 0; $i < $COMMUNICATION_PACKET_INTERARRIVAL_TIME[1]; $i++)
    {
	$test_param = $COMMUNICATION_PACKET_INTERARRIVAL_TIME[0] + $COMMUNICATION_PACKET_INTERARRIVAL_TIME[2]*$i;
	push(@{$Config[1][2]}, $test_param);
	$Config[1][1]++;
    }
} #end of if-6
elsif($evaluation_type == $CONST_EVALUATION_COMMUNICATION_PACKET_TTL) #if-7: communication packet TTL
{
    #set $Config[1][2] with COMMUNICATION_PACKET_TTL
    for($i = 0; $i < $COMMUNICATION_PACKET_TTL[1]; $i++)
    {
	$test_param = $COMMUNICATION_PACKET_TTL[0] + $COMMUNICATION_PACKET_TTL[2]*$i;
	push(@{$Config[1][2]}, $test_param);
	$Config[1][1]++;
    }
} #end of if-7
elsif($evaluation_type == $CONST_EVALUATION_COMMUNICATION_PACKET_DELIVERY_PROBABILITY_THRESHOLD) #if-8: communication packet delivery probability threshold
{
    #set $Config[1][2] with COMMUNICATION_PACKET_DELIVERY_PROBABILITY_THRESHOLD
    for($i = 0; $i < $COMMUNICATION_PACKET_DELIVERY_PROBABILITY_THRESHOLD[1]; $i++)
    {
	$test_param = $COMMUNICATION_PACKET_DELIVERY_PROBABILITY_THRESHOLD[0] + $COMMUNICATION_PACKET_DELIVERY_PROBABILITY_THRESHOLD[2]*$i;
	push(@{$Config[1][2]}, $test_param);
	$Config[1][1]++;
    }
} #end of if-8
elsif($evaluation_type == $CONST_EVALUATION_COMMUNICATION_AP_MAXIMUM_NUMBER) #if-9: communication AP maximum number
{
    #set $Config[1][2] with COMMUNICATION_AP_MAXIMUM_NUMBER
    for($i = 0; $i < $COMMUNICATION_AP_MAXIMUM_NUMBER[1]; $i++)
    {
	$test_param = $COMMUNICATION_AP_MAXIMUM_NUMBER[0] + $COMMUNICATION_AP_MAXIMUM_NUMBER[2]*$i;
	push(@{$Config[1][2]}, $test_param);
	$Config[1][1]++;
    }
} #end of if-9
elsif($evaluation_type == $CONST_EVALUATION_SIMULATION_TIME) #if-10: simulation time
{
    #set $Config[1][2] with SIMULATION_TIME 
    for($i = 0; $i < $SIMULATION_TIME[1]; $i++)
    {
	$test_param = $SIMULATION_TIME[0] + $SIMULATION_TIME[2]*$i;
	push(@{$Config[1][2]}, $test_param);
	$Config[1][1]++;
    }
} #end of case-10
elsif($evaluation_type == $CONST_EVALUATION_VEHICLE_AP_PASSING_ENTITY_PERCENTAGE) #if-11: vehicle AP passing entity percentage
{
    #set $Config[1][2] with EVALUATION_VEHICLE_AP_PASSING_ENTITY_PERCENTAGE 
    for($i = 0; $i < $VEHICLE_AP_PASSING_ENTITY_PERCENTAGE[1]; $i++)
    {
	$test_param = $VEHICLE_AP_PASSING_ENTITY_PERCENTAGE[0] + $VEHICLE_AP_PASSING_ENTITY_PERCENTAGE[2]*$i;
	push(@{$Config[1][2]}, $test_param);
	$Config[1][1]++;
    }
} #end of case-11
elsif($evaluation_type == $CONST_EVALUATION_COMMUNICATION_SN_MAXIMUM_NUMBER) #if-12: communication SN maximum number
{
    #set $Config[1][2] with COMMUNICATION_SN_MAXIMUM_NUMBER
    for($i = 0; $i < $COMMUNICATION_SN_MAXIMUM_NUMBER[1]; $i++)
    {
	$test_param = $COMMUNICATION_SN_MAXIMUM_NUMBER[0] + $COMMUNICATION_SN_MAXIMUM_NUMBER[2]*$i;
	push(@{$Config[1][2]}, $test_param);
	$Config[1][1]++;
    }
} #end of if-12

#task files split by split program
@task_files = ("xaa", "xab", "xac", "xad", "xae", "xaf", "xag", "xah", "xai", "xaj", "xak", "xal", "xam", "xan", "xao");

#number of measures for performance evaluation, such as expected delivery delay, actual delivery delay, the ratio of two delivery delays, packet delivery ratio, expected delivery delay standard deviation, delivery delay difference between the expected delivery delay and actual delivery delay, expected packet transmission number, and actual packet transmission number; if we add other measures, we need to increase $num_of_measures by the number of the additional measures
$num_of_measures = 8;
#$num_of_measures = 6;

#number of metrics for performance evaluation, such as average and standard deviation
$num_of_metrics = 2;

#range of parameters
$num_of_seeds = $Config[0][1]; #get the number of seeds
$num_of_test_param_values = $Config[1][1]; #get the number of test parameter values
$num_of_comparison_target_types = $Config[2][1]; #get the number of comparison target types

print("\$num_of_seeds=$num_of_seeds, \$num_of_test_param_values=$num_of_test_param_values, \$num_of_comparison_target_types=$num_of_comparison_target_types \n");

#head and tail for script
#$head_for_itasca = "#!/bin/bash -l\n#PBS -l walltime=01:00:00,mem=10gb,nodes=1:ppn=8\n#PBS -m abe\nmodule load intel\nexport OMP_NUM_THREADS=8";
$head_for_itasca = "#!/bin/bash -l\n#PBS -l walltime=02:00:00,mem=10gb,nodes=2:ppn=8\n#PBS -m abe\nmodule load intel\nexport OMP_NUM_THREADS=8";
$tail_for_itasca = "# end of blade portable batch system script\n";

#$head_for_nf = "#!/bin/csh\n#PBS -l nodes=1:ppn=1,mem=100mb,walltime=5:00:00\n# send mail options\n#PBS -m abe \n";
#$tail_for_nf = "# end of blade portable batch system script\n"; 

#$head_for_blade = "#!/bin/bash -l\n#PBS -l walltime=5:00:00,mem=2gb,nodes=1:ppn=1\n# send mail options\n#PBS -m abe \n";
#$tail_for_blade = "# end of blade portable batch system script\n"; 

#$head_for_calhoun = "#!/bin/bash -l\n#PBS -l walltime=5:00:00,mem=4gb,nodes=2:ppn=8\n# send mail options\n#PBS -m abe \nmodule load intel\nmodule load vmpi\n";
$head_for_calhoun = "#!/bin/bash -l\n#PBS -l walltime=5:00:00,mem=4gb,nodes=2:ppn=8\n# send mail options\n#PBS -m abe \n";
$tail_for_calhoun = "# end of calhoun portable batch system script\n"; 

#select the head and tail for the used linux cluster
$head = $head_for_itasca;
$tail = $tail_for_itasca;

#$head = $head_for_blade;
#$tail = $tail_for_blade;

#$head = $head_for_calhoun;
#$tail = $tail_for_calhoun;

#$head = $head_for_nf;
#$tail = $tail_for_nf;

1; #return TRUE value






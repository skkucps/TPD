#!/usr/bin/perl -w
## File: vanet_gentask.pl
## Description: the program generating the perl script for VANET simulation running
## Maker: Jaehoon Paul Jeong
## Date: Oct., 02, 2008

use vanet_common; #include vanet_common.pm

local $param_conf_dir = "./param-configuration"; #directory name for parameter configuration
local $param_conf_file; #parameter configuration file
local $default_param_conf_file_name = "param.conf"; #param conf file for simulation time type
local $default_output_dir = "./output"; #default output file directory
local $output_file_prefix = "output"; #output file prefix
local $distribution_for_vehicle_speed = $default_distribution_for_vehicle_speed; #distribution_for_vehicle_speed
local $edge_delay_model = 0; #link delay model
local $edd_model = 0; #EDD model

#set param conf file to default param.conf
$param_conf_file = $default_param_conf_file_name;

print("\$param_conf_file is $param_conf_file\n");

#clean the previous output files
print("Clean files under the directory output...\n");
unlink <./output/*>;
print("Clean is done!\n");

#generate the script for tasks for TEST_PARAM
open OUTPUT, ">TASK.sh" or die;

#computer the number of tasks that will be submitted to the linux cluster
$num_of_tasks = 0;

for($i = 0; $i < $num_of_seeds; $i++)
{
    $seed = $Config[0][0] + $Config[0][2]*$i;
    $Config[0][4] = $seed; #current seed
    for($j = 0; $j < $num_of_test_param_values; $j++)
    {
		$test_param = $Config[1][2][$j]; #current test-param value
		$Config[1][4] = $test_param; #update Config[1][4] to have the current test-param value
		for($k = 0; $k < $num_of_comparison_target_types; $k++)
		{
			$target_type_value = $Config[2][0] + $Config[2][2]*$k;
			$Config[2][4] = $target_type_value; #current comparison target type's value

			#@make argument list according to test parameter type
			if($evaluation_type == $CONST_EVALUATION_VEHICLE_MAXIMUM_NUMBER) #vehicle maximum number
			{
				print OUTPUT "./vanet -a $comparison_target -b $evaluation_type -s $seed -n $test_param -q $default_distribution_for_vehicle_speed -w $default_vehicle_speed_standard_deviation -t $default_communication_packet_ttl -E $default_communication_packet_delivery_probability_threshold";
			} 
			elsif($evaluation_type == $CONST_EVALUATION_VEHICLE_PACKET_GENERATING_ENTITY_NUMBER) #number of packet generating vehicles
			{
				print OUTPUT "./vanet -a $comparison_target -b $evaluation_type -s $seed -g $test_param -q $default_distribution_for_vehicle_speed -w $default_vehicle_speed_standard_deviation -n $default_vehicle_maximum_number -t $default_communication_packet_ttl -E $default_communication_packet_delivery_probability_threshold";
			}
			elsif($evaluation_type == $CONST_EVALUATION_VEHICLE_SPEED) #vehicle speed
			{
				$distribution_for_vehicle_speed = $CONST_DISTRIBUTION_NORMAL;
				print OUTPUT "./vanet -a $comparison_target -b $evaluation_type -s $seed -q $default_distribution_for_vehicle_speed -v $test_param -w $default_vehicle_speed_standard_deviation -n $default_vehicle_maximum_number -t $default_communication_packet_ttl -E $default_communication_packet_delivery_probability_threshold";
			}
			elsif($evaluation_type == $CONST_EVALUATION_VEHICLE_SPEED_STANDARD_DEVIATION) #vehicle speed standard deviation
			{
				$distribution_for_vehicle_speed = $CONST_DISTRIBUTION_NORMAL;
				print OUTPUT "./vanet -a $comparison_target -b $evaluation_type -s $seed -q $default_distribution_for_vehicle_speed -w $test_param -n $default_vehicle_maximum_number -t $default_communication_packet_ttl -E $default_communication_packet_delivery_probability_threshold";
			}
			elsif($evaluation_type == $CONST_EVALUATION_VEHICLE_INTERARRIVAL_TIME) #vehicle interarrival time
			{
				print OUTPUT "./vanet -a $comparison_target -b $evaluation_type -s $seed -z $test_param -w $default_vehicle_speed_standard_deviation -n $default_vehicle_maximum_number -t $default_communication_packet_ttl -E $default_communication_packet_delivery_probability_threshold";
			}
			elsif($evaluation_type == $CONST_EVALUATION_COMMUNICATION_PACKET_INTERARRIVAL_TIME) #packet interarrival time
			{
				print OUTPUT "./vanet -a $comparison_target -b $evaluation_type -s $seed -i $test_param -q $default_distribution_for_vehicle_speed -w $default_vehicle_speed_standard_deviation -n $default_vehicle_maximum_number -t $default_communication_packet_ttl -E $default_communication_packet_delivery_probability_threshold";
			}
			elsif($evaluation_type == $CONST_EVALUATION_COMMUNICATION_PACKET_TTL) #
			{
				print OUTPUT "./vanet -a $comparison_target -b $evaluation_type -s $seed -t $test_param -q $default_distribution_for_vehicle_speed -w $default_vehicle_speed_standard_deviation -n $default_vehicle_maximum_number -E $default_communication_packet_delivery_probability_threshold";
			}
			elsif($evaluation_type == $CONST_EVALUATION_COMMUNICATION_PACKET_DELIVERY_PROBABILITY_THRESHOLD) #
			{
				print OUTPUT "./vanet -a $comparison_target -b $evaluation_type -s $seed -t $default_communication_packet_ttl -q $default_distribution_for_vehicle_speed -w $default_vehicle_speed_standard_deviation -n $default_vehicle_maximum_number -E $test_param";
			}
			elsif($evaluation_type == $CONST_EVALUATION_COMMUNICATION_AP_MAXIMUM_NUMBER) #
			{
				print OUTPUT "./vanet -a $comparison_target -b $evaluation_type -s $seed -t $default_communication_packet_ttl -q $default_distribution_for_vehicle_speed -w $default_vehicle_speed_standard_deviation -n $default_vehicle_maximum_number -E $default_communication_packet_delivery_probability_threshold -N $test_param";
			}
			elsif($evaluation_type == $CONST_EVALUATION_COMMUNICATION_SN_MAXIMUM_NUMBER) #
			{
				print OUTPUT "./vanet -a $comparison_target -b $evaluation_type -s $seed -t $default_communication_packet_ttl -q $default_distribution_for_vehicle_speed -w $default_vehicle_speed_standard_deviation -n $default_vehicle_maximum_number -E $default_communication_packet_delivery_probability_threshold -U $test_param";
			}
			elsif($evaluation_type == $CONST_EVALUATION_SIMULATION_TIME) #
			{
				print OUTPUT "./vanet -a $comparison_target -b $evaluation_type -s $seed -u $test_param -q $default_distribution_for_vehicle_speed -w $default_vehicle_speed_standard_deviation -n $default_vehicle_maximum_number -t $default_communication_packet_ttl -E $default_communication_packet_delivery_probability_threshold";
			}
			elsif($evaluation_type == $CONST_EVALUATION_VEHICLE_AP_PASSING_ENTITY_PERCENTAGE) #
			{
				print OUTPUT "./vanet -a $comparison_target -b $evaluation_type -s $seed -q $default_distribution_for_vehicle_speed -w $default_vehicle_speed_standard_deviation -n $default_vehicle_maximum_number -t $default_communication_packet_ttl -A $test_param -E $default_communication_packet_delivery_probability_threshold";
			}

            #@add comparison target option to the argument list according to comparison target
			if($comparison_target == $CONST_COMPARISON_EDD_AND_LINK_MODEL) #EDD-Link model
			{
				#update edd model and link delay model according to edd_and_link_model
				if($target_type_value == $CONST_TBD_EDD_AND_TBD_LINK )
				{
					$edd_model = $CONST_EDD_MODEL_PER_VEHICLE;
					$edge_delay_model = $CONST_EDGE_DELAY_MODEL_TBD_FOR_FINITE_ROAD;
				}
				elsif($target_type_value == $CONST_TBD_EDD_AND_VADD_LINK)
				{
					$edd_model = $CONST_EDD_MODEL_PER_VEHICLE;
					$edge_delay_model = $CONST_EDGE_DELAY_MODEL_VADD;
				}
				elsif($target_type_value == $CONST_VADD_EDD_AND_VADD_LINK)
				{
					$edd_model = $CONST_EDD_MODEL_PER_INTERSECTION;
					$edge_delay_model = $CONST_EDGE_DELAY_MODEL_VADD;
				}
				elsif($target_type_value == $CONST_VADD_EDD_AND_TBD_LINK)
				{
					$edd_model = $CONST_EDD_MODEL_PER_INTERSECTION;
					$edge_delay_model = $CONST_EDGE_DELAY_MODEL_TBD_FOR_FINITE_ROAD;
				}

				print OUTPUT " -l $target_type_value -e $edd_model -d $edge_delay_model -c $default_tbd_edd_computation_type -f $default_intersection_forwarding_type";
			}	    
			elsif($comparison_target == $CONST_COMPARISON_EDD_MODEL) #EDD model
			{
				print OUTPUT " -c $default_tbd_edd_computation_type -d $default_edge_delay_model -f $default_intersection_forwarding_type -e $target_type_value";
			} 
			elsif($comparison_target == $CONST_COMPARISON_EDGE_DELAY_MODEL) #Edge delay model
			{
				print OUTPUT " -c $default_tbd_edd_computation_type -d $target_type_value -f $default_intersection_forwarding_type -e $default_edd_model";
			}
			elsif($comparison_target == $CONST_COMPARISON_TBD_EDD_COMPUTATION_TYPE) #TBD's EDD computation type
			{
				print OUTPUT " -c $target_type_value -d $default_edge_delay_model -f $default_intersection_forwarding_type -e $default_edd_model";
			}
			elsif($comparison_target == $CONST_COMPARISON_INTERSECTION_FORWARDING_TYPE) #Intersection Forwarding type
			{
				print OUTPUT " -c $default_tbd_edd_computation_type -d $default_edge_delay_model -f $target_type_value -e $default_edd_model";
			}
			elsif($comparison_target == $CONST_COMPARISON_AVERAGE_CONVOY_LENGTH_ESTIMATION_TYPE) #Average Convoy Length Estimation type
			{
				$default_vehicle_vanet_vehicular_traffic_model = $CONST_VANET_VEHICULAR_TRAFFIC_OPEN_NETWORK;
				print OUTPUT " -c $default_tbd_edd_computation_type -d $target_type_value -f $default_intersection_forwarding_type -e $default_edd_model -m $default_vehicle_vanet_vehicular_traffic_model -k 2 -g 1 -q 1 -h 1";
			}
			elsif($comparison_target == $CONST_COMPARISON_AVERAGE_LINK_DELAY_ESTIMATION_TYPE) #Average Convoy Length Estimation type
			{
				$default_vehicle_vanet_vehicular_traffic_model = $CONST_VANET_VEHICULAR_TRAFFIC_OPEN_NETWORK;
				print OUTPUT " -c $default_tbd_edd_computation_type -d $target_type_value -f $default_intersection_forwarding_type -e $default_edd_model -m $default_vehicle_vanet_vehicular_traffic_model -k 2 -g 1 -q $default_distribution_for_vehicle_speed";
			}
			elsif($comparison_target == $CONST_COMPARISON_TARGET_POINT_SELECTION_TYPE) #Target Point Selection type
			{
				print OUTPUT " -c $default_tbd_edd_computation_type -d $default_edge_delay_model -f $default_intersection_forwarding_type -e $default_edd_model -S $target_type_value -C $default_target_point_computation_method";
			} 
			elsif($comparison_target == $CONST_COMPARISON_TARGET_POINT_COMPUTATION_METHOD) #Target Point Computation method
			{
				print OUTPUT " -c $default_tbd_edd_computation_type -d $default_edge_delay_model -f $default_intersection_forwarding_type -e $default_edd_model -S $default_target_point_selection_type -C $target_type_value";
			} 
			elsif($comparison_target == $CONST_COMPARISON_TARGET_POINT_NUMBER) #Target Point Number
			{
				print OUTPUT " -c $default_tbd_edd_computation_type -d $default_edge_delay_model -f $default_intersection_forwarding_type -e $default_edd_model -S $default_target_point_selection_type -C $default_target_point_computation_method -K 0 -W 1 -O $target_type_value";
			} 

			#add simulation time parameter
			print OUTPUT " -u $default_simulation_time";

            #add multiple_AP_flag
			print OUTPUT " -M $default_multiple_APs_flag";

			#add maximum packet number to generate
			print OUTPUT " -j $default_communication_packet_maximum_number";

			#add road_network_size
			print OUTPUT " -k $default_road_network_size";	    

			#add data forwarding mode
			#print OUTPUT " -F $default_data_forwarding_mode";

			#add vehicle trajectory length type
			#print OUTPUT " -D $default_vehicle_trajectory_length_type";

			$output_file = $output_file_prefix;
			for($h = 0; $h <= $#Config; $h++)
			{
				$output_file = $output_file."#".$Config[$h][4];
            }

			$output_file_1 = $output_file.".txt";
			$output_file_2 = $output_file.".xls";

			print OUTPUT " -p $param_conf_dir/$param_conf_file";
			print OUTPUT " -x $default_output_dir/$output_file_1";
			print OUTPUT " -y $default_output_dir/$output_file_2\n";

			$num_of_tasks++; #increase the number of tasks
		}
    }
}

#print the total number of tasks
print("the total number of tasks is $num_of_tasks\n");

close(OUTPUT);
chmod 0755,, "TASK.sh";


##create submit_task file submitting tasks for linux cluster ##

#number of computing nodes
$num_of_nodes = 10;
#$num_of_nodes = 6;
print("the number of computing nodes is $num_of_nodes\n");

#number of shell lines for each computing node
$num_of_shell_lines_per_node = $num_of_tasks/$num_of_nodes;

#print the task information
print("each file has $num_of_shell_lines_per_node lines\n");

#perform split of tasks
`split -l $num_of_shell_lines_per_node TASK.sh`;

#change the mode of tasks files
`chmod +x x*`;

#generate the file submit_task for submitting tasks to the linux cluster
open SUBMIT_TASK, ">submit_task" or die;

print SUBMIT_TASK "#!/bin/bash\n";

for($i = 0; $i < $num_of_nodes; $i++)
{
    $command = "qsub task-".$task_files[$i]."&\n"; #concatenate task_files[$i] and "&\n"
    print $command;
    print SUBMIT_TASK $command;
}

chmod 0755,, "submit_task";
close(SUBMIT_TASK);

##create each submitted_task file for the linux cluster

$current_dir = `pwd`;
for($i = 0; $i < $num_of_nodes; $i++)
{
    $task_file = $task_files[$i];
    $task_script = "task-".$task_files[$i];
    print("$task_script\n");
    open TASK_SCRIPT, ">$task_script" or die;
    write TASK_SCRIPT;
    chmod 0755,, $task_script;
    close(TASK_SCRIPT);
}

##format for print
format TASK_SCRIPT =
@*
$head
cd @*
$current_dir
@<<<<<<<<
$task_file
@*
$tail
.

#!/usr/bin/perl -w
## File: vanet_compute_cdf_of_delivery_cost.pl
## Description: the program computing the cumulative distribution function of the actual delivery cost for the excel file containing the packet delivery tuples. 
## Maker: Jaehoon Paul Jeong
## Date: May, 10, 2011

use POSIX; #for floor()

#@debug option
$DEBUG = 0; #1: display debugging messages and 0: suppress the messages

#@log tuple type
$LOG_TYPE_PACKET_AP_ARRIVAL = 1; #log type for packet arrival at AP
$LOG_TYPE_PACKET_DESTINATION_VEHICLE_ARRIVAL = 2; #log type for packet arrival at destination vehicle
$LOG_TYPE_PACKET_DROP = 3; #log type for packet drop

#@forwarding type
$FORWARDING_TYPE_TSF = 1; #Trajectory-based Statistical Forwarding (TSF)
$FORWARDING_TYPE_LTP = 2; #Last Trajectory Point (LTP)
$FORWARDING_TYPE_RTP = 3; #Random Trajectory Point (RTP)

#@delivery cost type
$DELIVERY_DELAY_TYPE_ADC = 1; #actual delivery cost (ADC)
$DELIVERY_DELAY_TYPE_EDC = 2; #expected delivery cost (EDC)

#unit number for CDF computattion
my $default_unit_number = 1; #1 transmission
#my $default_unit_number = 1; #
my $unit_number = $default_unit_number;
my $default_maximum_number = 35; #default maximum transmission number 

#number of transmission number steps for the computation of CDF
#my $cost_step_number = floor($default_ttl/$unit_number);
my $cost_step_number = int($default_maximum_number/$unit_number);

#index of maximum delivery cost among the delivery costs
my $maximum_delivery_cost_index = -1;

#@declare arrays for data
my @DELAY_TABLE = (); #table for containing delivery cost: horizontal axis values
my @COUNT_TABLE = (); #table for containing the count of costs
my @CUMULATIVE_COUNT_TABLE = (); #table for containing the cumulative count for CDF
my @CDF_TABLE = (); #table for the CDF of delivery cost: vertical axis values

my $default_output_dir_name = "./output"; #default name of output directory containing the actual delivery cost log file

#default log file name
my $default_log_file_name = "output.log";
my $log_file_name = 0; #log file containing the logging tuples for delivery cost 

#cdf file name
my $cdf_file_name = 0; #CDF file for delivery cost 
my $prefix_of_cdf_file = "stat-cdf";
my $suffix_of_cdf_file = "xls";

#variables for forwarding type
my $name_for_tsf = "-of-tsf"; #file name for TSF
my $name_for_ltp = "-of-ltp"; #file name for LTP
my $name_for_rtp = "-of-rtp"; #file name for RTP
my $default_middle_name_for_forwarding_type = $name_for_tsf;
my $middle_name_for_forwarding_type = 0;
my $default_forwarding_type = 1; #TSF
my $forwarding_type = 0;

#variables for delivery cost type
my $name_for_adc = "-for-adc"; #file name for Actual Delivery Cost (ADC)
my $name_for_edc = "-for-edc"; #file name for Expected Delivery Cost (EDC)
my $default_middle_name_for_delivery_cost_type = $name_for_adc;
my $middle_name_for_delivery_cost_type = 0;
my $default_delivery_cost_type = 1; #actual delivery cost 
my $delivery_ype = 0;

#@local varibles for for-loop
my ($size, $i, $index); #$index is the index of the item corresponding to the delivery cost 

#@initialize tables
$size = $cost_step_number;
for($i = 0; $i < $size; $i++)
{
    $DELAY_TABLE[$i] = $i;
    $COUNT_TABLE[$i] = 0;
    $CUMULATIVE_COUNT_TABLE[$i] = 0;
    $CDF_TABLE[$i] = 0;
}

#@ get the type of the data forwarding scheme generating the log file
print("Put the type of the data forwarding for CDF file [TSF:1, LTP:2 and RTP:3] (default: $default_forwarding_type): ");
$forwarding_type = <STDIN>;
chop($forwarding_type);

if($forwarding_type eq "")
{
    $forwarding_type = $default_forwarding_type;
}

#select the stat file's suffix corresponding to the forwarding type
if($forwarding_type == $FORWARDING_TYPE_TSF)
{
    $middle_name_for_forwarding_type = $default_middle_name_for_forwarding_type;
}
elsif($forwarding_type == $FORWARDING_TYPE_LTP)
{
    $middle_name_for_forwarding_type = $name_for_ltp;
}
elsif($forwarding_type == $FORWARDING_TYPE_RTP)
{
    $middle_name_for_forwarding_type = $name_for_rtp;
}
else
{
    print("forwarding_type($forwarding_type) is not supported!\n");
    exit;
}

#@ get the type of the delivery cost from the log file
print("Put the type of the delivery cost for CDF file [Actual Delivery Cost (ADC):1 and Expected Delivery Cost (EDC):2] (default: $default_delivery_cost_type): ");
$delivery_cost_type = <STDIN>;
chop($delivery_cost_type);

if($delivery_cost_type eq "")
{
    $delivery_cost_type = $default_delivery_cost_type;
}

#select the stat file's suffix corresponding to the delivery cost type
if($delivery_cost_type == $DELIVERY_DELAY_TYPE_ADC)
{
    $middle_name_for_delivery_cost_type = $default_middle_name_for_delivery_cost_type;
}
elsif($delivery_cost_type == $DELIVERY_DELAY_TYPE_EDC)
{
    $middle_name_for_delivery_cost_type = $name_for_edc;
}
else
{
    print("delivery_cost_type($delivery_cost_type) is not supported!\n");
    exit;
}

#@ make cdf file name
$cdf_file_name = $prefix_of_cdf_file.$middle_name_for_forwarding_type.$middle_name_for_delivery_cost_type.".".$suffix_of_cdf_file;

#@ print the cdf file
print("cdf_file_name is $cdf_file_name\n");

#@ print the log file
$log_file_name = $default_log_file_name;
print("log_file_name is $log_file_name\n");

#@ get the name of output directory of simulation data
print("Put the name of the directory containing the log file for the cost (default: $default_output_dir_name): ");
$output_dir = <STDIN>;
chop($output_dir);

if($output_dir eq "")
{
    $output_dir = $default_output_dir_name;
}

#@ get the unit number for CDF creation
print("Put the unit number for CDF creation in seconds (default: $default_unit_number sec): ");
$unit_number = <STDIN>;
chop($unit_number);

if($unit_number eq "")
{
    $unit_number = $default_unit_number;
}

#@open the log file
open LOG_FILE, $output_dir."/".$log_file_name or die;

#@build COUNT_TABLE from log file
$line_num = 0;
while(<LOG_FILE>) #while-1
{
    ($field_0, $field_1, $field_2, $field_3, $field_4, $field_5, $field_6, $field_7, $field_8, $field_9, $field_10, $field_11, $field_12, $field_13, $field_14, $field_15, $field_16) = split("\t");
    
    if($field_0 == $LOG_TYPE_PACKET_DROP)
    {
	#print("Error: there is a packet drop!\n");
	#exit;

	#@we count the dropped packet's cost
	#$DELIVERY_DELAY_TYPE_ADC
	if($delivery_cost_type == $DELIVERY_DELAY_TYPE_ADC)
	{
	    $index = int($field_16/$unit_number);
	}
	else #$DELIVERY_DELAY_TYPE_EDC
	{
	    $index = int($field_15/$unit_number);
	}

	#update $maximum_delivery_cost
	if($maximum_delivery_cost_index < $index)
	{
	    $maximum_delivery_cost_index = $index;
	}

	$COUNT_TABLE[$index]++; #increase the count of the cost corresponding to $index
    }
    else
    {
	#$DELIVERY_DELAY_TYPE_ADC
	if($delivery_cost_type == $DELIVERY_DELAY_TYPE_ADC)
	{
	    $index = int($field_16/$unit_number);
	}
	else #$DELIVERY_DELAY_TYPE_EDC
	{
	    $index = int($field_15/$unit_number);
	}

	#update $maximum_delivery_cost
	if($maximum_delivery_cost_index < $index)
	{
	    $maximum_delivery_cost_index = $index;
	}

	$COUNT_TABLE[$index]++; #increase the count of the cost corresponding to $index
    }

    $line_num++;

#@for debugging
if($DEBUG)
{
    print("$line_num: $field_0, $field_1, $field_2, $field_3, $field_4, $field_5, $field_6, $field_7, $field_8, $field_9, $field_10, $field_11, $field_12, $field_13, $field_14, $field_15, $field_16 => ");
    print("EDC=$field_15 and ADC=$field_16\n");
    print("COUNT_TABLE[$index]=$COUNT_TABLE[$index]\n");
    $input = <STDIN>;
}
###############

} #end of while-1

#@show $line_num
print("\$line_num=$line_num\n");

#@show $maximum_delivery_cost_index
print("\$maximum_delivery_cost_index=$maximum_delivery_cost_index\n");
$input = <STDIN>;

close(LOG_FILE);

#@build COUNT_TABLE from log file
$size = $cost_step_number;
for($i = 0; $i < $size; $i++)
{
    if($i == 0)
    {
	$CUMULATIVE_COUNT_TABLE[$i] = $COUNT_TABLE[$i];
    }
    else
    {
	$CUMULATIVE_COUNT_TABLE[$i] = $CUMULATIVE_COUNT_TABLE[$i-1] + $COUNT_TABLE[$i];
    }

    $CDF_TABLE[$i] = $CUMULATIVE_COUNT_TABLE[$i]/$line_num;
}

#@show CUMULATIVE_COUNT_TABLE[$size-1]
print("\$CUMULATIVE_COUNT_TABLE[$size-1]=$CUMULATIVE_COUNT_TABLE[$size-1]\n");
$input = <STDIN>;

#stor
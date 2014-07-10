#!/usr/bin/perl -w
## File: vanet_compute_cdf_of_delivery_delay.pl
## Description: the program computing the cumulative distribution function of the actual delivery delay for the excel file containing the packet delivery tuples. 
## Maker: Jaehoon Paul Jeong
## Date: Nov., 20, 2008

use POSIX; #for floor()

#@debug option
$DEBUG = 0; #1: display debugging messages and 0: suppress the messages

#@log tuple type
$LOG_TYPE_PACKET_AP_ARRIVAL = 1; #log type for packet arrival at AP
$LOG_TYPE_PACKET_DESTINATION_VEHICLE_ARRIVAL = 2; #log type for packet arrival at destination vehicle
$LOG_TYPE_PACKET_DROP = 3; #log type for packet drop

#@forwarding type
$FORWARDING_TYPE_TBD = 1; #TBD
$FORWARDING_TYPE_VADD = 2; #VADD

#@delivery delay type
$DELIVERY_DELAY_TYPE_ADD = 1; #actual delivery delay (ADD)
$DELIVERY_DELAY_TYPE_EDD = 2; #expected delivery delay (EDD)

#unit time for CDF computattion
#my $default_unit_time = 1; #1 second
#my $default_unit_time = 100; #100 seconds
my $default_unit_time = 200; #200 seconds
my $unit_time = $default_unit_time;
my $default_ttl = 50000; #default TTL for the range of delivery delay
#my $default_maximum_delay = 7000; #default maximum of delivery delay
#my $default_maximum_delay = 10000; #default maximum of delivery delay
my $default_maximum_delay = 6000; #default maximum of delivery delay

#number of time steps for the computation of CDF
#my $delay_step_number = floor($default_ttl/$unit_time);
my $delay_step_number = int($default_maximum_delay/$unit_time);

#index of maximum delivery delay among the delivery delays
my $maximum_delivery_delay_index = -1;

#@declare arrays for data
my @DELAY_TABLE = (); #table for containing delivery delay: horizontal axis values
my @COUNT_TABLE = (); #table for containing the count of delays
my @CUMULATIVE_COUNT_TABLE = (); #table for containing the cumulative count for CDF
my @CDF_TABLE = (); #table for the CDF of delivery delay: vertical axis values

my $default_output_dir_name = "./output"; #default name of output directory containing the actual delivery delay log file

#default log file name
my $default_log_file_name = "output.log";
my $log_file_name = 0; #log file containing the logging tuples for delivery delay

#cdf file name
my $cdf_file_name = 0; #CDF file for delivery delay
my $prefix_of_cdf_file = "stat-cdf";
my $suffix_of_cdf_file = "xls";

#variables for forwarding type
my $name_for_tbd = "-of-tbd"; #file name for TBD
my $name_for_vadd = "-of-vadd"; #file name for VADD
my $default_middle_name_for_forwarding_type = $name_for_tbd;
my $middle_name_for_forwarding_type = 0;
my $default_forwarding_type = 1; #TBD
my $forwarding_type = 0;

#variables for delivery delay type
my $name_for_add = "-for-add"; #file name for Actual Delivery Delay (ADD)
my $name_for_edd = "-for-edd"; #file name for Expected Delivery Delay (EDD)
my $default_middle_name_for_delivery_delay_type = $name_for_add;
my $middle_name_for_delivery_delay_type = 0;
my $default_delivery_delay_type = 1; #actual delivery delay
my $delivery_delay_type = 0;

#@local varibles for for-loop
my ($size, $i, $index); #$index is the index of the item corresponding to the delivery delay

#@initialize tables
$size = $delay_step_number;
for($i = 0; $i < $size; $i++)
{
    $DELAY_TABLE[$i] = $i;
    $COUNT_TABLE[$i] = 0;
    $CUMULATIVE_COUNT_TABLE[$i] = 0;
    $CDF_TABLE[$i] = 0;
}

#@ get the type of the data forwarding scheme generating the log file
print("Put the type of the data forwarding for CDF file [TBD:1 and VADD:2] (default: $default_forwarding_type): ");
$forwarding_type = <STDIN>;
chop($forwarding_type);

if($forwarding_type eq "")
{
    $forwarding_type = $default_forwarding_type;
}

#select the stat file's suffix corresponding to the forwarding type
if($forwarding_type == $FORWARDING_TYPE_TBD)
{
    $middle_name_for_forwarding_type = $default_middle_name_for_forwarding_type;
}
elsif($forwarding_type == $FORWARDING_TYPE_VADD)
{
    $middle_name_for_forwarding_type = $name_for_vadd;
}
else
{
    print("forwarding_type($forwarding_type) is not supported!\n");
    exit;
}

#@ get the type of the delivery delay from the log file
print("Put the type of the delivery delay for CDF file [Actual Delivery Delay (ADD):1 and Expected Delivery Delay (EDD):2] (default: $default_delivery_delay_type): ");
$delivery_delay_type = <STDIN>;
chop($delivery_delay_type);

if($delivery_delay_type eq "")
{
    $delivery_delay_type = $default_delivery_delay_type;
}

#select the stat file's suffix corresponding to the delivery delay type
if($delivery_delay_type == $DELIVERY_DELAY_TYPE_ADD)
{
    $middle_name_for_delivery_delay_type = $default_middle_name_for_delivery_delay_type;
}
elsif($delivery_delay_type == $DELIVERY_DELAY_TYPE_EDD)
{
    $middle_name_for_delivery_delay_type = $name_for_edd;
}
else
{
    print("delivery_delay_type($delivery_delay_type) is not supported!\n");
    exit;
}

#@ make cdf file name
$cdf_file_name = $prefix_of_cdf_file.$middle_name_for_forwarding_type.$middle_name_for_delivery_delay_type.".".$suffix_of_cdf_file;

#@ print the cdf file
print("cdf_file_name is $cdf_file_name\n");

#@ print the log file
$log_file_name = $default_log_file_name;
print("log_file_name is $log_file_name\n");

#@ get the name of output directory of simulation data
print("Put the name of the directory containing the log file for the delay delay (default: $default_output_dir_name): ");
$output_dir = <STDIN>;
chop($output_dir);

if($output_dir eq "")
{
    $output_dir = $default_output_dir_name;
}

#@ get the unit time for CDF creation
print("Put the unit time for CDF creation in seconds (default: $default_unit_time sec): ");
$unit_time = <STDIN>;
chop($unit_time);

if($unit_time eq "")
{
    $unit_time = $default_unit_time;
}

#@open the log file
open LOG_FILE, $output_dir."/".$log_file_name or die;

#@build COUNT_TABLE from log file
$line_num = 0;
while(<LOG_FILE>) #while-1
{
    ($field_0, $field_1, $field_2, $field_3, $field_4, $field_5, $field_6, $field_7, $field_8, $field_9, $field_10, $field_11, $field_12, $field_13, $field_14, $field_15) = split("\t");
    
    if($field_0 == $LOG_TYPE_PACKET_DROP)
    {
	print("Error: there is a packet drop!\n");
	exit;
    }
    else
    {
	if($delivery_delay_type == $DELIVERY_DELAY_TYPE_ADD)
	{
	    $index = int($field_11/$unit_time);
	}
	else #$DELIVERY_DELAY_TYPE_EDD
	{
	    $index = int($field_10/$unit_time);
	}

	#update $maximum_delivery_delay
	if($maximum_delivery_delay_index < $index)
	{
	    $maximum_delivery_delay_index = $index;
	}

	$COUNT_TABLE[$index]++; #increase the count of the delay corresponding to $index
    }

    $line_num++;

#@for debugging
if($DEBUG)
{
    print("$line_num: $field_0, $field_1, $field_2, $field_3, $field_4, $field_5, $field_6, $field_7, $field_8, $field_9, $field_10, $field_11, $field_12, $field_13, $field_14, $field_15 => ");
    print("EDD=$field_10 and ADD=$field_11\n");
    print("COUNT_TABLE[$index]=$COUNT_TABLE[$index]\n");
    $input = <STDIN>;
}
###############

} #end of while-1

#@show $line_num
print("\$line_num=$line_num\n");

#@show $maximum_delivery_delay_index
print("\$maximum_delivery_delay_index=$maximum_delivery_delay_index\n");
$input = <STDIN>;

close(LOG_FILE);

#@build COUNT_TABLE from log file
$size = $delay_step_number;
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

#store the statistics into a file
&vanet_store_cdf_statistics_into_file(@CDF_TABLE, $unit_time, $maximum_delivery_delay_index, $cdf_file_name);

####################################################################################################

#@function definition
sub vanet_store_cdf_statistics_into_file
{
    my $default_stat_dir_name = "./result";
    my ($stat_dir, $stat_file_name, $x, $y);

    #@ get the name of result directory of statistics file
    print("Put the directory name for statistics file (default: ./result): ");
    $stat_dir = <STDIN>;
    chop($stat_dir);

    if($stat_dir eq "")
    {
	$stat_dir = $default_stat_dir_name;
    }
    
    #make stat file name
    $stat_file_name = $stat_dir."/".$cdf_file_name;
    print("stat_file_name is $stat_file_name\n\n");
    #$input = <STDIN>;

    #@store statistics into stat file
    open STAT_FILE, ">".$stat_file_name or die;

    $size = $maximum_delivery_delay_index;
    for($i = 0; $i <= $size; $i++) #for
    {
	$x = $unit_time*$i;
	#print $x;
	$y = $CDF_TABLE[$i];

	print STAT_FILE $x;
	print STAT_FILE "\t$y\n";
    } #end of for

    #close file
    close(STAT_FILE);

    1; #return value
}

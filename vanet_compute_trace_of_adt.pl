#!/usr/bin/perl -w
## File: vsa_compute_trace_of_adt.pl
## Description: the program computing the trace of average detection time for each unit time (such as 10 minutes) for the excel file containing vehicle detection time tuples. 
## Maker: Jaehoon Paul Jeong
## Date: March., 08, 2008

use POSIX; #for floor()

#@debug option
$DEBUG = 0; #1: display debugging messages and 0: suppress the messages

#unit time for moving window for average computattion
my $default_unit_time = 60*60; #60 min = 1 hour
my $unit_time = 0;

#number of time slots for the computation of average detection time
$slot_number = 0;

#@declare arrays for data
my @TIME_ARRAY = ();
my @MOVEMENT_TIME_ARRAY = ();
my @ARRIVAL_TIME_ARRAY = ();
my @DETECTION_TIME_ARRAY = ();
my @TRACE_TABLE = ([@TIME_ARRAY], [@MOVEMENT_TIME_ARRAY], [@DETECTION_TIME_ARRAY], [@DETECTION_TIME_ARRAY]); #table for trace data
my @SLOT_TABLE = (); #table for containing vehicle detection time and movement time;
my @ADT_TABLE = (); #table for average detection time per unit time
my @ADT_SD_TABLE = (); #table for standard deviation of average detection time per unit time

my $default_result_dir_name = "./result"; #default name of result directory containing the vehicle detection time file

#default trace file name
my $default_trace_file_name = "trace-file-of-vehicle-detection-time.xls";
my ($trace_file_name, $stat_dir, $prefix_of_trace_file, $suffix_of_trace_file);

#scheduleing algorithm types
my $name_for_virtual_scan = "-for-virtual-scan"; #file name for virtual scan
my $name_for_duty_cycling = "-for-duty-cycling"; #file name for duty-cycling
my $name_for_always_awake = "-for-always-awake"; #file name for always-awake
my $default_middle_name = $name_for_virtual_scan;
my $middle_name = 0;
my $default_algorithm_type = 1; #virtual scan
my $algorithm_type = 0;

#@ get the type of the scheduling generating the trace file
print("Put the type of the scheduling generating the trace file [virtual scan:1, duty-cycling:2, and always-awake:3] (default: $default_algorithm_type): ");
$algorithm_type = <STDIN>;
chop($algorithm_type);

if($algorithm_type eq "")
{
    $algorithm_type = $default_algorithm_type;
}

#select the stat file's suffix corresponding to the algorithm type
if($algorithm_type == 1)
{
    $middle_name = $default_middle_name;
}
elsif($algorithm_type == 2)
{
    $middle_name = $name_for_duty_cycling;
}
elsif($algorithm_type == 3)
{
    $middle_name = $name_for_always_awake;
}
else
{
    print("algorithm_type($algorithm_type) is not supported!\n");
    exit;
}

#get the prefix and suffix of the trace file
($prefix_of_trace_file, $suffix_of_trace_file) = split(/\./, $default_trace_file_name);

#make trace file name
$trace_file_name = $prefix_of_trace_file.$middle_name.".".$suffix_of_trace_file;
print("trace_file_name is $trace_file_name\n");
$input = <STDIN>;

#@ get the name of the trace file
#print("Put the name for the trace file (default: $default_trace_file_name): ");
#$trace_file = <STDIN>;
#chop($trace_file_name);
#
#if($trace_file_name eq "")
#{
#    $trace_file_name = $default_trace_file_name;
#}
#print("trace_file_name is $trace_file_name\n");


#@ get the name of output directory of simulation data
print("Put the name of the directory containing the trace file for vehicle detection time (default: $default_result_dir_name): ");
$result_dir = <STDIN>;
chop($result_dir);

if($result_dir eq "")
{
    $result_dir = $default_result_dir_name;
}

#@ get the aggregation unit time for average
print("Put the aggregation unit time for average in seconds (default: $default_unit_time sec): ");
$unit_time = <STDIN>;
chop($unit_time);

if($unit_time eq "")
{
    $unit_time = $default_unit_time;
}

#open the trace file
open TRACE_FILE, $result_dir."/".$trace_file_name or die;

$line_num = 0;
$max_detection_time = -1; #maximum detection time
$max_detection_time_index = -1; #index of the item corresponding to the maximum detection time
while(<TRACE_FILE>) #while-1
{
    ($field_0, $field_1, $field_2, $field_3) = split("\t");
    
    #@push $field_0, ..., $field_3 into the table TRACE_TABLE
    push(@{$TRACE_TABLE[0]}, $field_0);
    push(@{$TRACE_TABLE[1]}, $field_1);
    push(@{$TRACE_TABLE[2]}, $field_2);
    push(@{$TRACE_TABLE[3]}, $field_3);

    if($field_0 > $max_detection_time)
    {
	$max_detection_time = $field_0;
	$max_detection_time_index = $line_num;
    }

    $line_num++;

#@for debugging
if($DEBUG)
{
    print("$line_num: $field_0, $field_1, $field_2, $field_3\n");
    #$input = <STDIN>;
}
###############

} #end of while-1

close(TRACE_FILE);

########
if($DEBUG)
{    
    $input = <STDIN>;

    $size = $#{$TRACE_TABLE[0]} + 1; #dereference the reference to the arrary with curly brace
    for($i = 0; $i < $size; $i++)
    {
        print("\$TRACE_TABLE[0][$i] = $TRACE_TABLE[0][$i]\n");
    }
}
########

#number of time slots for the computation of average detection time
$slot_number = floor($max_detection_time/$unit_time);

if($max_detection_time > ($slot_number*$unit_time))
{
    $slot_number++; #increase the slot number;
}

#print the number of slots
print("the number of slots is $slot_number\n");

#construct SLOT TABLE with slot_number
for($i = 0; $i < $slot_number; $i++)
{
    @SLOT_ARRAY = ();
    for($j = 0; $j < 2; $j++)
    {
	@DETECTION_TIME_ARRAY = ();
	@MOVEMENT_TIME_ARRAY = ();

	push @SLOT_ARRAY, [@DETECTION_TIME_ARRAY]; #array for detected time within one time slot
	push @SLOT_ARRAY, [@MOVEMENT_TIME_ARRAY]; #array for movement time within one time slot
    }
    push @SLOT_TABLE, [@SLOT_ARRAY];
}

#distribute the vehicle detection time into appropriate slot in SLOT_TABLE
$size = $#{$TRACE_TABLE[0]} + 1; #dereference the reference to the arrary with curly brace
for($i = 0; $i < $size; $i++)
{
    $index = $TRACE_TABLE[0][$i]/$unit_time;
    
    push(@{$SLOT_TABLE[$index][0]}, $TRACE_TABLE[0][$i]);
    push(@{$SLOT_TABLE[$index][1]}, $TRACE_TABLE[1][$i]);

########
if($DEBUG)
{    
    print("\$TRACE_TABLE[0][$i] = $TRACE_TABLE[0][$i], \$TRACE_TABLE[1][$i] = $TRACE_TABLE[1][$i]\n");
}

}

########
if($DEBUG)
{    
    $input = <STDIN>;
    $size = $#{$SLOT_TABLE[3][0]} + 1; #dereference the reference to the arrary with curly brace
    for($i = 0; $i < $size; $i++)
    {
        print("\$SLOT_TABLE[3][0][$i] = $SLOT_TABLE[3][0][$i]\n");
        print("\$SLOT_TABLE[3][1][$i] = $SLOT_TABLE[3][1][$i]\n\n");
    }
}
########

#compute the average of movement time for average detection time for each slot
&vsa_process_data_for_average_detection_time(@SLOT_TABLE, $slot_number, @ADT_TABLE, @ADT_SD_TABLE);

#store the statistics into a file
&vsa_store_adt_statistics_into_file(@ADT_TABLE, @ADT_SD_TABLE, $slot_number, $unit_time, $middle_name);

####################################################################################################

#@function definition
###############################################
sub vsa_process_data_for_average_detection_time
{
    my ($sum, $num, $i, $j);

    #@compute mean for each measure
    for($i = 0; $i < $slot_number; $i++) #for-1
    {
	$num = $#{$SLOT_TABLE[$i][1]} + 1; #dereference the reference to the arrary with curly brace

        $sum = &get_sum_of_array_items($SLOT_TABLE[$i][1]);

	if($num == 0)
	{
	    print("vsa_process_data_for_average_detection_time(): \$num is 0\n");
	    exit;
	}
	push(@ADT_TABLE, $sum/$num);

#@for debugging
if(1)
#if(@DEBUG)
{
	print("mean of ($i) = $ADT_TABLE[$i]\n");
	#$input = <STDIN>;
}
###############
   } #end of for-1

    #@compute standard deviation for each measure
    for($i = 0; $i < $slot_number; $i++) #for-1
    {
	$mean = $ADT_TABLE[$i];

	$sum = &get_sum_of_mean_error_square($SLOT_TABLE[$i][1], $mean);
	$num = $#{$SLOT_TABLE[$i][1]} + 1; #dereference the reference to the arrary with curly brace
	if($num == 1)
	{
	    print("vsa_process_data_for_average_detection_time(): \$num is 1\n");
	    #exit;
	    push(@ADT_SD_TABLE, $sum/$num);
	}
        else
        {
	    push(@ADT_SD_TABLE, $sum/($num-1));
	}

#@for debugging
if(1)
#if(@DEBUG)
{
	print("standard deviation of ($i) = $ADT_SD_TABLE[$i]\n");
	#$input = <STDIN>;
}
###############
    } #end of for-1

    1; #return value
}


sub get_sum_of
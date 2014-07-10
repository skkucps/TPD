#!/usr/bin/perl -w
## File: vanet_produce_statistics.pm
## Description: the function that process a data file according to composite key value
## (e.g., vehicle interarrival time + link delay type) in order to get statistics and split data files.
## Maker: Jaehoon Paul Jeong
## Date: Oct., 03, 2008
## Update Date: March, 04, 2009

use vanet_common;

sub vanet_produce_statistics
{
#@declare arrays for data
    #local(@INNER_KEY_TABLE); #key table for inner table containing data
    #local(@OUTER_KEY_TABLE); #key table for outer table containing inner tables

    #local(@INNER_DATA_TABLE); #inner data table containing data
    #local(@OUTER_DATA_TABLE); #outer data table containing inner data tables
    #local(@TMP_TABLE); #temporay table for data inserted later
    #local(@MEASURE_TABLE); #temporay table for measure table contents

    #local(@INNER_STAT_TABLE); #inner stat table containing statistics
    #local(@OUTER_STAT_TABLE); #outer stat table containing inner statistics tables

    #local(%INNER_INDEX_TABLE); #index table to return the index in inner data table for the key in inner key table
    #local(%OUTER_INDEX_TABLE); #index table to return the index in outer data table for the key in outer key table
    #########

    @INNER_KEY_TABLE = (); #key table for inner table containing data
    @OUTER_KEY_TABLE = (); #key table for outer table containing inner tables

    @INNER_DATA_TABLE = (); #inner data table containing data
    @OUTER_DATA_TABLE = (); #outer data table containing inner data tables
    @TMP_TABLE = (); #temporay table for data inserted later
    @MEASURE_TABLE = (); #temporay table for measure table contents

    @INNER_STAT_TABLE = (); #inner stat table containing statistics
    @OUTER_STAT_TABLE = (); #outer stat table containing inner statistics tables

    %INNER_INDEX_TABLE = (); #index table to return the index in inner data table for the key in inner key table
    %OUTER_INDEX_TABLE = (); #index table to return the index in outer data table for the key in outer key table
    ####################################
    
    #@initialize INNER_KEY_TABLE and INNER_INDEX_TABLE
    $size_of_inner_table = $Config[1][1]; #e.g., number of vehicle interarrival times
#@for debugging
if($DEBUG_1)
{
    print("size_of_inner_table=$size_of_inner_table\n");
    $input = <STDIN>;
}
###############
    for($i = 0; $i < $size_of_inner_table; $i++)
    {
	$INNER_KEY_TABLE[$i] = $Config[1][2][$i];

	#$key = sprintf("%.06f", $INNER_KEY_TABLE[$i]); #$key is string rather than floating-point number to support the 6 digits after decimal point of the key (i.e., vehicle interarrival time)
	$key = sprintf("%.2f", $INNER_KEY_TABLE[$i]); #$key is string rather than floating-point number to support the 2 digits after decimal point of the key (i.e., vehicle interarrival time)

	#$INNER_INDEX_TABLE{$INNER_KEY_TABLE[$i]} = $i; #associate the key with index $i
	$INNER_INDEX_TABLE{$key} = $i; #associate the key with index $i

#@for debugging
if($DEBUG)
{
        print("$key : $INNER_INDEX_TABLE{$key}\n");
        $input = <STDIN>;
}
###############
    }

    #@initialize OUTER_KEY_TABLE and OUTER_INDEX_TABLE
    $size_of_outer_table = $Config[2][1]; #e.g., number of link delay types
    for($i = 0; $i < $size_of_outer_table; $i++)
    {
	$OUTER_KEY_TABLE[$i] = $Config[2][0] + $i*$Config[2][2];
	$key = sprintf("%d", $OUTER_KEY_TABLE[$i]); #$key is string rather than integer number
	#$OUTER_INDEX_TABLE{$OUTER_KEY_TABLE[$i]} = $i; #associate the key with index $i 
	$OUTER_INDEX_TABLE{$key} = $i; #associate the key with index $i 
	#print("$key : $OUTER_INDEX_TABLE{$key}\n");
    }

    #@initialize OUTER_DATA_TABLE
    for($i = 0; $i < $size_of_outer_table; $i++) #for-1
    {    
	@INNER_DATA_TABLE = (); #NOTE: inner-data-table must be reset to insert a new inner-data-table into outer-data-table
	for($j = 0; $j < $size_of_inner_table; $j++) #for-2
	{
	    @TMP_TABLE = ();
	    for($k = 0; $k < $num_of_measures; $k++) #for-3
	    {
			@MEASURE_TABLE = (); #expected delivery delay, actual delivery delay, the ratio of two delivery delays, packet delivery ratio, expected delivery delay standard deviation, delivery delay difference between the expected delivery delay and actual delivery delay, expected packet transmission number, and actual packet transmission number.
			push @TMP_TABLE, [@MEASURE_TABLE];
	    } #end of for-3

	    push @INNER_DATA_TABLE, [@TMP_TABLE];
	} #end of for-2
    
	push @OUTER_DATA_TABLE, [@INNER_DATA_TABLE];
    } #end of for-1

    #@initialize OUTER_STAT_TABLE
    for($i = 0; $i < $size_of_outer_table; $i++) #for-1
    {    
	@INNER_STAT_TABLE = (); #NOTE: inner-data-table must be reset to insert a new inner-data-table into outer-data-table
	for($j = 0; $j < $size_of_inner_table; $j++) #for-2
	{
	    @TMP_TABLE = ();
	    for($k = 0; $k < $num_of_measures; $k++) #for-3
	    {
			@MEASURE_TABLE = (); #expected delivery delay, actual delivery delay, the ratio of two delivery delays, packet delivery ratio, expected delivery delay standard deviation, delivery delay difference between the expected delivery delay and actual delivery delay, expected packet transmission number, and actual packet transmission number.
			push @TMP_TABLE, [@MEASURE_TABLE];
	    } #end of for-3

	    push @INNER_STAT_TABLE, [@TMP_TABLE];
	} #end of for-2
    
	push @OUTER_STAT_TABLE, [@INNER_STAT_TABLE];
    } #end of for-1

    #@read the lines of data_file and split them into appropriate INNER_DATA_TABLEs with composite key (e.g., link delay type and vehicle interarrival time)

    #@for debugging
    #print("$result_dir./.$data_file\n");
    #$input = <STDIN>;

    open DATA_FILE, $result_dir."/".$data_file or die;
    $line_num = 0;
    while(<DATA_FILE>) #while@OUTER_KEY_TABLE
    {
	    #($field_0, $field_1, $field_2, $field_3, $field_4, $field_5, $field_6, $field_7, $field_8) = split("\t");
	    ($field_0, $field_1, $field_2, $field_3, $field_4, $field_5, $field_6, $field_7, $field_8, $field_9, $field_10) = split("\t");

        #chop($field_6); #removes and returns the last character from the given string of $evaluation_type; in this line, the removed character is '\n'.

#@for debugging
if($DEBUG_1)
{
	#print("$line_num: $field_0, $field_1, $field_2, $field_3, $field_4, $field_5, $field_6, $field_7, $field_8 \n");
	print("$line_num: $field_0, $field_1, $field_2, $field_3, $field_4, $field_5, $field_6, $field_7, $field_8, $field_9, $field_10 \n");
	$input = <STDIN>;
}
###############
	$line_num++;

	#@field_0 is inner-table key and field_2 is outer-table key where @field_0 is test_param and @field_2 is sched_type
	$outer_index = 	$OUTER_INDEX_TABLE{$field_2}; #outer-table index corresponding to outer-table key
	$inner_index = 	$INNER_INDEX_TABLE{$field_0}; #inner-table index corresponding to inner-table key

	#@push $field_3, $field_4, $field_5, $field_6, $field_7, $field_8, $field_9, $field_10 into inner-data-table
        push(@{$OUTER_DATA_TABLE[$outer_index][$inner_index][0]}, $field_3);
        push(@{$OUTER_DATA_TABLE[$outer_index][$inner_index][1]}, $field_4);
        push(@{$OUTER_DATA_TABLE[$outer_index][$inner_index][2]}, $field_5);
        push(@{$OUTER_DATA_TABLE[$outer_index][$inner_index][3]}, $field_6);
        push(@{$OUTER_DATA_TABLE[$outer_index][$inner_index][4]}, $field_7);
        push(@{$OUTER_DATA_TABLE[$outer_index][$inner_index][5]}, $field_8);
        push(@{$OUTER_DATA_TABLE[$outer_index][$inner_index][6]}, $field_9);
        push(@{$OUTER_DATA_TABLE[$outer_index][$inner_index][7]}, $field_10);

	#$index = $#{$OUTER_DATA_TABLE[$outer_index][$inner_index][0]}; #get the last index of the table
	#$index++; #make next available index
	#$OUTER_DATA_TABLE[$outer_index][$inner_index][0][$index] = $field_3;	
    } #end of while

    close(DATA_FILE);

########
if($DEBUG)
{    
    $size = $#{$OUTER_DATA_TABLE[0][0][0]} + 1; #dereference the reference to the arrary with curly brace
    for($i = 0; $i < $size; $i++)
    {
    print("\$OUTER_DATA_TABLE[0][0][0][$i] = $OUTER_DATA_TABLE[0][0][0][$i]\n");
	print("\$OUTER_DATA_TABLE[0][0][1][$i] = $OUTER_DATA_TABLE[0][0][1][$i]\n");
	print("\$OUTER_DATA_TABLE[0][0][2][$i] = $OUTER_DATA_TABLE[0][0][2][$i]\n");
	print("\$OUTER_DATA_TABLE[0][0][3][$i] = $OUTER_DATA_TABLE[0][0][3][$i]\n");
	print("\$OUTER_DATA_TABLE[0][0][4][$i] = $OUTER_DATA_TABLE[0][0][4][$i]\n");
	print("\$OUTER_DATA_TABLE[0][0][5][$i] = $OUTER_DATA_TABLE[0][0][5][$i]\n");
	print("\$OUTER_DATA_TABLE[0][0][5][$i] = $OUTER_DATA_TABLE[0][0][6][$i]\n");
	print("\$OUTER_DATA_TABLE[0][0][5][$i] = $OUTER_DATA_TABLE[0][0][7][$i]\n");
    }
}
########

    #@process statistics
    &vanet_process_data_for_statistics(@OUTER_KEY_TABLE, @OUTER_DATA_TABLE, $size_of_outer_table, $size_of_inner_table, $num_of_measures, @OUTER_STAT_TABLE);

    #@store statistics into a statistics file for gnuplot
    &vanet_store_statistics_into_file(@OUTER_STAT_TABLE, @OUTER_KEY_TABLE, @OUTER_DATA_TABLE, $size_of_outer_table, $size_of_inner_table, $num_of_measures);

    1; #return value

} #end of vanet_produce_statistics 


sub vanet_process_data_for_statistics
{
    my ($sum, $num, $i, $j, $k);

    #@compute mean for each measure
    for($i = 0; $i < $size_of_outer_table; $i++) #for-1
    {
	for($j = 0; $j < $size_of_inner_table; $j++) #for-2
	{
	    for($k = 0; $k < $num_of_measures; $k++) #for-3
	    {
		$sum = &get_sum_of_array_items($OUTER_DATA_TABLE[$i][$j][$k]);
		$num = $#{$OUTER_DATA_TABLE[$i][$j][$k]} + 1;
		if($num == 0)
		{
		    print("vanet_process_data_for_statistics(): \$num is 0\n");
		    exit;
		}
		$OUTER_STAT_TABLE[$i][$j][$k][0] = $sum/$num;

		#@for debugging
if($DEBUG_1)
{
		print("mean of ($i,$j,$k) for \$num=$num is $OUTER_STAT_TABLE[$i][$j][$k][0]\n");
		$input = <STDIN>;
}
		###############
	    } #end of for-3
	} #end of for-2
    } #end of for-1

    #@compute standard deviation for each measure; Note that this standard deviation is computed with the mean values, not with the raw data
    for($i = 0; $i < $size_of_outer_table; $i++) #for-1
    {
	for($j = 0; $j < $size_of_inner_table; $j++) #for-2
	{
	    for($k = 0; $k < $num_of_measures; $k++) #for-3
	    {
		$mean = $OUTER_STAT_TABLE[$i][$j][$k][0];

		$sum = &get_sum_of_mean_error_square($OUTER_DATA_TABLE[$i][$j][$k], $mean);
		$num = $#{$OUTER_DATA_TABLE[$i][$j][$k]} + 1;
		if($num == 1)
		{
		    print("vanet_process_data_for_statistics(): \$num is 1\n");
		    exit;
		}
                $OUTER_STAT_TABLE[$i][$j][$k][1] = sqrt($sum/($num-1));
		#$OUTER_STAT_TABLE[$i][$j][$k][1] = $sum/($num-1);

		#@for debugging
if($DEBUG_SD)
{
		print("standard deviation of ($i,$j,$k) for \$num=$num is $OUTER_STAT_TABLE[$i][$j][$k][1]\n");
		$input = <STDIN>;
}
		###############

	    } #end of for-3
	} #end of for-2
    } #end of for-1

    1; #return value
}


sub get_sum_of_array_items
{
    my $sum = 0;
    my $array_ref = $_[0];
    my $index_bound = 0;
    my $i = 0;

    $index_bound = $#{$array_ref};
    for($i = 0; $i <= $index_bound; $i++)
    {
	$sum += $array_ref->[$i];
    }

    #@for debugging
if($DEBUG)
{
    print("\$sum inside get_sum_of_array_items() = $sum\n");
    $input = <STDIN>;
}
    ###############

    $sum; #return value
}

sub get_sum_of_mean_error_square
{
    my $sum = 0;
    my $array_ref = $_[0];
    my $mean = $_[1];
    my $index_bound = 0;
    my $i = 0;
    my $diff = 0;

    $index_bound = $#{$array_ref};
    for($i = 0; $i <= $index_bound; $i++)
    {
	$diff = $array_ref->[$i] - $mean;
	$sum += $diff * $diff;

    #@for debugging
	if($DEBUG_SD)
        {
	    print("\$array_ref->[$i]=$array_ref->[$i],\$mean=$mean, \$diff=$diff, \$sum=$sum\n");
	    $input = <STDIN>;
	}
    ###############

    }

    #@for debugging
if($DEBUG_SD)
{
    print("\$sum inside get_sum_of_mean_error_square() with \$mean $mean = $sum\n");
    $input = <STDIN>;
}
    ###############

    $sum; #return value
}

sub vanet_store_statistics_into_file
{
    my $default_stat_file_name = "stat.xls";
    my $default_stat_dir_name = "./result";
    my ($stat_file, $stat_dir, $prefix_of_stat_file, $suffix_of_stat_file, $x, $y0, $y1, $y2, $y3, $y4);

    #@ get the name of result directory of statistics file
    print("Put the directory name for statistics file (default: ./result): ");
    $stat_dir = <STDIN>;
    chop($stat_dir);

    if($stat_dir eq "")
    {
	$stat_dir = $default_stat_dir_name;
    }

    #@ get the name of statistics file
    print("Put the name for the statistics file (default: stat-file.xls): ");
    $stat_file = <STDIN>;
    chop($stat_file);

    if($stat_file eq "")
    {
	$stat_file = $default_stat_file_name;
    }
    
    #get the prefix and suffix of the stat_file
    ($prefix_of_stat_file, $suffix_of_stat_file) = split(/\./, $stat_file);

    #print("stat_file is $stat_file, prefix is $prefix_of_stat_file, and suffix is $suffix_of_stat_file\n");
    #$input = <STDIN>;

    #@store statistics into stat file
    open STAT_FILE_0, ">".$stat_dir."/".$prefix_of_stat_file."-estimate.".$suffix_of_stat_file or die;
    open STAT_FILE_1, ">".$stat_dir."/".$prefix_of_stat_file."-measure.".$suffix_of_stat_file or die;
    open STAT_FILE_2, ">".$stat_dir."/".$prefix_of_stat_file."-delay-ratio.".$suffix_of_stat_file or die;
    open STAT_FILE_3, ">".$stat_dir."/".$prefix_of_stat_file."-delivery-ratio.".$suffix_of_stat_file or die;
    open STAT_FILE_4, ">".$stat_dir."/".$prefix_of_stat_file."-difference.".$suffix_of_stat_file or die;
    open STAT_FILE_5, ">".$stat_dir."/".$prefix_of_stat_file."-sd-estimate.".$suffix_of_stat_file or die;    
    open STAT_FILE_6, ">".$stat_dir."/".$prefix_of_stat_file."-sd-measure.".$suffix_of_stat_file or die;    
    open STAT_FILE_7, ">".$stat_dir."/".$prefix_of_stat_file."-cost-estimate.".$suffix_of_stat_file or die;    
    open STAT_FILE_8, ">".$stat_dir."/".$prefix_of_stat_file."-cost-measure.".$suffix_of_stat_file or die;    

    #@STAT_FILE = (STAT_FILE_0, STAT_FILE_1, STAT_FILE_2, STAT_FILE_3);

    for($j = 0; $j < $size_of_inner_table; $j++) #for-1
    {
	$x = $INNER_KEY_TABLE[$j];
	#print $x;
	print STAT_FILE_0 $x;
	print STAT_FILE_1 $x;
	print STAT_FILE_2 $x;	
	print STAT_FILE_3 $x;
	print STAT_FILE_4 $x;
	print STAT_FILE_5 $x;
	print STAT_FILE_6 $x;
	print STAT_FILE_7 $x;
	print STAT_FILE_8 $x;
	for($i = 0; $i < $size_of_outer_table; $i++) #for-2
	{
	    #print "\t$OUTER_STAT_TABLE[$i][$j][0][0]";
	    print STAT_FILE_0 "\t$OUTER_STAT_TABLE[$i][$j][0][0]";
	    print STAT_FILE_1 "\t$OUTER_STAT_TABLE[$i][$j][1][0]"; #mean of actual delivery delay by the measurement
	    print STAT_FILE_2 "\t$OUTER_STAT_TABLE[$i][$j][2][0]";
	    print STAT_FILE_3 "\t$OUTER_STAT_TABLE[$i][$j][3][0]";
	    print STAT_FILE_4 "\t$OUTER_STAT_TABLE[$i][$j][4][0]";
	    print STAT_FILE_5 "\t$OUTER_STAT_TABLE[$i][$j][5][0]";
	    print STAT_FILE_6 "\t$OUTER_STAT_TABLE[$i][$j][1][1]"; #standard deviation of actual delivery delay by the measurement
	    print STAT_FILE_7 "\t$OUTER_STAT_TABLE[$i][$j][6][0]"; #expected packet transmission
	    print STAT_FILE_8 "\t$OUTER_STAT_TABLE[$i][$j][7][0]"; #actual packet transmission
	} #end of for-2
	#print "\n";
	print STAT_FILE_0 "\n";
	print STAT_FILE_1 "\n";
	print STAT_FILE_2 "\n";
	print STAT_FILE_3 "\n";
	print STAT_FILE_4 "\n";
	print STAT_FILE_5 "\n";
	print STAT_FILE_6 "\n";
	print STAT_FILE_7 "\n";
	print STAT_FILE_8 "\n";
    } #end of for-1

    #close files
    close(STAT_FILE_0);
    close(STAT_FILE_1);
    close(STAT_FILE_2);
    close(STAT_FILE_3);
    close(STAT_FILE_4);
    close(STAT_FILE_5);
    close(STAT_FILE_6);
    close(STAT_FILE_7);
    close(STAT_FILE_8);

    1; #return value
}

###############################
#vanet_produce_statistics();

###############################
#@time = localtime(time());
#print("@time\n");
#print("$time[0], $time[1]\n");

#$data_string = "a\tb\tc\td\te\tf\n";
#($work_time, $seed, $sched_type, $lifetime, $ave_detect_time, $num_of_vehicles) = split("\t", $data_string);
#print("$work_time, $seed, $sched_type, $lifetime, $ave_detect_time, $num_of_vehicles\n");

1; #return value

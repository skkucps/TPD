#!/usr/bin/perl -w
## File: vanet_process_data.pl
## Description: the program aggregating the simulation output files of Excel format into one data file 
## and then split the aggregated data file into multiple data files according to composite key value
## (e.g., working time + scanning type).
## Maker: Jaehoon Paul Jeong
## Date: Oct., 03, 2008

use vanet_common; #include vanet_common.pm
use vanet_produce_statistics; #include vanet_produce_statistics.pm

#default output directory name
my $default_output_dir_name = "./output"; #default name of output directory containing raw simulation data files

my $default_result_dir_name = "./result"; #default name of result directory containing the aggregated data file and summary statistics files

#default data file name
my $default_data_file_name = "output-data.xls";

#my ($output_dir, $result_dir, $data_file); #my variables are visible inside this code, not within the function called by this code

#@ get the name of output directory of simulation data
print("Put the name of the directory containing simulation data (default: $default_output_dir_name): ");
$output_dir = <STDIN>;
chop($output_dir);

if($output_dir eq "")
{
    $output_dir = $default_output_dir_name;
}

#@ get the name of result directory of processed data
print("Put the name of the directory for the aggregated data file and summary statistics files (default: $default_result_dir_name): ");
$result_dir = <STDIN>;
chop($result_dir);

if($result_dir eq "")
{
    $result_dir = $default_result_dir_name;
}

#@ get the name of aggregated data file
print("Put the name for the aggregated data file (default: $default_data_file_name): ");
$data_file = <STDIN>;
chop($data_file);

if($data_file eq "")
{
    $data_file = $default_data_file_name;
}

#$data_file = $output_dir."/".$data_file;
print("data_file is $data_file\n");

#clean the previous output files
print("Clean the previous data file $data_file under the directory $result_dir...\n");
unlink <$result_dir/$data_file>;
unlink <$result_dir/*.xls>;
print("Clean is done!\n");

#open an aggregated data file
open DATA_FILE, ">".$result_dir."/".$data_file or die;

#read one line from each .xls and write it into DATA_FILE
$num_of_lines = 0; #number of lines in the aggregated data file
while($output_file = <$output_dir/*.xls>)
{
    print("output file name is $output_file\n");
    open(OUTPUT_FILE, $output_file);
    $data_tuple = <OUTPUT_FILE>;
    #chop($data_tuple);
if($DEBUG)
{
    print("data_tuple is $data_tuple\n");
}
    print DATA_FILE $data_tuple."\n";
    close(OUTPUT_FILE);
    $num_of_lines++;
}

close(DATA_FILE);

#report the number of lines of the aggregated data file
print("the number of lines in $data_file is $num_of_lines\n");

#ask whether to perform data file split or not
#$default_operation_option = "y";

print("Do we perform the data processing for generating statistics summary files? [y/n] \n");
$operation_option = <STDIN>;
chop($operation_option);

if($operation_option eq "")
{
    $operation_option = "y";
}
elsif($operation_option eq "n")
{
    exit;
}
elsif($operation_option ne "y")
{
    print("operation_option($operation_option) is not supported!\n");
    exit;
}

#@Here we process data_file in order to get statistics and split data files according to composite key
&vanet_produce_statistics($result_dir, $data_file, @Config);

1; #return value

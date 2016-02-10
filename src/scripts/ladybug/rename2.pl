#!/usr/bin/perl

use File::Copy;

$filetype = "ppm";

print ("Camera number [0-5]: ");

$camera = <STDIN>;
chomp($camera);

if ((length($camera) != 1) || ($camera < 0) || ($camera > 5)) {
print("error");
break;
}

#rename files
opendir(DIR, ".");
@files = grep(/\.$filetype$/,readdir(DIR));
closedir(DIR);

my $counter = 0;

# first renaming step

foreach $file (@files) {
   $counter_string = "$counter";
   while (length($counter_string) < 6) {
	$counter_string = "0".$counter_string;
   }
	
   $target = "_".$counter_string."camera$camera".".".$filetype;
   move($file,$target);
   $counter = $counter + 1;
}


# second renaming step

opendir(DIR, ".");
@files = grep(/\.$filetype$/,readdir(DIR));
closedir(DIR);

foreach $file (@files) {
   $new_file = substr ($file, 1, length($file));
   move($file,$new_file);
   print("$new_file\n");
}



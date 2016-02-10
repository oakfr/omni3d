#!/usr/bin/perl
#
# ladybug_split_to_6.pl
# 
# Author: Olivier Koch - koch@csail.mit.edu
#
# Date: June-29-2005
#
#
# This program splits a bunch of ladybug images into 6 directories (corresponding to the 6 cameras).
# Copy this script into the same directory as the ppm images
# and run it with no arguments.
#

use Cwd;
use File::Copy;

$filetype = "ppm";


if (mkdir ('cam0') == 0) {
	#dir already exists!
	print("Warning! Some of the cam* directories already exist. Overwrite? (y/n): ");
	$answer = <STDIN>;
	if (!yes($answer)) {
	    print("exiting...\n");
		exit();
	}

	# clean-up
	for ($i=0;$i<6;$i++) {
		$dir_name = "cam".$i;
		remove_dir($dir_name);
	}
}

#create
for ($i=0;$i<6;$i++) {
	$dir_name = "cam".$i;
	mkdir($dir_name);
}

#move files
opendir(DIR, ".");
@files = grep(/\.$filetype$/,readdir(DIR));
closedir(DIR);

foreach $file (@files) {
   @words = split(/\./,$file);
   $string = reverse $words[0];
   $id = substr($string,0,1);
   $target = "cam".$id."/".$file;
   move($file,$target);
}


#################################################
sub yes ()
{
	$txt = @_[0];
	chomp($txt);
    return ($txt eq "y")
}

sub remove_dir ()
{
	$dirname = @_[0];
	if (! -d $dirname) {
		return;
	}

	$current_dir = cwd;

	chdir($dirname);
	unlink <*>;
	chdir($current_dir);
	rmdir($dirname);
}


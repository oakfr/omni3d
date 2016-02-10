#!/usr/bin/perl

use File::Copy;
use Cwd;

$filetype = "ppm";
$select_list = "select.txt";

# clean-up
for ($i=0;$i<6;$i++) {
	$dir_name = "cam".$i."_select";
	remove_dir($dir_name);
}

#create
for ($i=0;$i<6;$i++) {
	$dir_name = "cam".$i."_select";
	mkdir($dir_name);
}

#open select list
open(DATA,$select_list) || die "Cant open $select_list\n";

while (<DATA>) {
  $line = $_;
  $l = substr($line,0,length($line)-2);  

  for ($camera=0;$camera<6;$camera++) {
	
	$str = ladybugFilename($camera,$l);
	$filename_start = "cam".$camera."\\".$str;
	$filename_end = "cam".$camera."_select\\".$str;
	if (!-e $filename_start) {
		print "ERROR: $filename_start does not exist!\n";
	}

	$cmd = "cp \"$filename_start\" \"$filename_end\"";
	qx\$cmd\;
	print ("$filename_start ==> $filename_end\n");
 }
}

 close (DATA);

#######################################################################

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

#######################################################################

sub ladybugFilename ()
{
	$my_camera = @_[0];
	$my_index = @_[1];


	while (length($my_index) < 6) {
		$my_index = "0".$my_index;
	}

	$my_str =  $my_index."camera".$my_camera.".".$filetype;

	return $my_str;
}

#!/usr/bin/perl

use File::Copy;

$filetype = "bmp";

#rename files
opendir(DIR, ".");
@files = grep(/\.$filetype$/,readdir(DIR));
closedir(DIR);

my $counter = 0;
my $cam_counter = 0;

foreach $file (@files) {
   $target = ladybugFilename($cam_counter,$counter);
  # move($file,$target);
   $cam_counter = $cam_counter + 1;
if ($cam_counter == 6) {
	$cam_counter = 0;
	   $counter = $counter + 1;
	}
print("$file ==> $target\n");

}


sub ladybugFilename ()
{
	$my_camera = @_[0];
	$my_index = @_[1];


	while (length($my_index) < 6) {
		$my_index = "0".$my_index;
	}

	$my_str =  $my_index."-cam".$my_camera.".".$filetype;

	return $my_str;
}

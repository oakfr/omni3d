#!/usr/bin/perl

use File::Copy;

$filetype = "bmp";

#rename files
opendir(DIR, ".");
@files = grep(/\.$filetype$/,readdir(DIR));
closedir(DIR);

my($n) = scalar(@files)/6;

print "$n images\n";

my $counter = 0;

foreach $file (@files) {

   if ( $file =~ /(\d+)-cam(\d+)(.)+/ ) {
      #print "$1 -- $2\n";
   }
   $target = "tmp/".ladybugFilename($counter, $2);
   move($file,$target);
   if ( $2 == "5" ) {
	   $counter = $counter + 1;
   }
 print("$file ==> $target\n");

}


sub ladybugFilename ()
{
	$my_index = @_[0];
    $my_camera = @_[1];

	while (length($my_index) < 6) {
		$my_index = "0".$my_index;
	}

	$my_str =  $my_index."-cam".$my_camera.".".$filetype;

	return $my_str;
}

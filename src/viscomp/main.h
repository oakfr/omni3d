#include "basic.h"
#include "viewer/viewer.h"
#include "signature/signatureviewer.h"

LUT _lut; // a lookup table
Fl_Progress *progress_bar; // a progress bar
Viewer viewer; // a viewer for display

std::vector< std::string > dirnames; // list of directories to process




#ifndef WIN32
#include <sys/ioctl.h>
#endif

#include "Xbus.h"

void
xsens_setParameters ( int port );
void 
xsens_writeHeaders();
void
xsens_init ( int port );

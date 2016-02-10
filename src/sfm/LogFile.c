/*
 * LogFile.c : This file contains some routines that are used to 
 * read and write 3D structure files. These 3D structure files
 * are binary files with a very simple format which can be 
 * deduced from these routines.
 */

#include <stdio.h>
#include "SFM.h"

int WriteStructureFile (char *filename,
			int type, int n, int m,
			segment *Segments1, position *Positions1,
			segment *Segments2, position *Positions2)
{
  FILE *TheFile;

  if (!((type == 1) || (type == 2))) return (-1);

  TheFile = fopen (filename, "w");

  if (!TheFile) return (-2);

  fwrite (&type, sizeof(int), 1, TheFile);
  fwrite (&n, sizeof(int), 1, TheFile);
  fwrite (&m, sizeof(int), 1, TheFile);

  fwrite (Segments1,  sizeof(segment),  n, TheFile);
  fwrite (Positions1, sizeof(position), m, TheFile);

  if (type == 2) {
    fwrite (Segments2,  sizeof(segment),  n, TheFile);
    fwrite (Positions2, sizeof(position), m, TheFile);
  }

  fclose (TheFile);
  return (0);
}


int ReadStructureFile (char *filename,
		       int *type, int *n, int *m,
		       segment *Segments1, position *Positions1,
		       segment *Segments2, position *Positions2)
{
  FILE *TheFile;

  TheFile = fopen (filename, "r");

  if (!TheFile) return (-2);

  fread (type, sizeof(int), 1, TheFile);

  if (!((*type == 1) || (*type == 2))) return (-1);

  fread (n, sizeof(int), 1, TheFile);
  fread (m, sizeof(int), 1, TheFile);

  fread (Segments1,  sizeof(segment),  *n, TheFile);
  fread (Positions1, sizeof(position), *m, TheFile);

  if (*type == 2) {
    fread (Segments2,  sizeof(segment),  *n, TheFile);
    fread (Positions2, sizeof(position), *m, TheFile);
  }

  fclose (TheFile);
  return 0;
}

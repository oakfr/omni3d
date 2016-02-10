/*
 * algorithm.h : header file for algorithm.c
 */

void ComputeV (double *v, line TheLine);
void ComputeD (double *d, line TheLine);
void ComputeM (double *m, line TheLine, position ThePosition);
char IndexToAxis (int index);
void UpdateLine_q (double *q, double *step, double *newq);
void UpdatePosition_q (double *q, double *step, double *newq);
double v_error (line l, segment s);
double LineError (line l, segment s);
double R_error (position p, position true_p);
double T_error (position p, position true_p);
int ReconstructEndPoints (segment *TheSegments);
double AverageRerror ();
double AverageTerror ();
double AverageVerror ();
double AverageLineError ();

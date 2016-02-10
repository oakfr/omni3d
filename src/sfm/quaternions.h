/*
 * quaternions.h : Header file for quaternions.c
 */

void VectorToQuaternion (double *w, double *q);
void QuaternionToVector (double *q, double *w);
void NormalizeQuaternion (double *q);
void RotateQuaternion (double *b, double *q, double *a);
void RotateVector (double *b, double *w, double *a);
void RotateAngleVector (double *b, double alpha, double *u, double *a);
void QuaternionConjugate (double *q, double *q_prime);
void QuaternionMultiply (double *result, double *q1, double *q2);
double QuaternionNorm (double *q);
double VectorNorm (double *w);
double QuaternionDistance (double *q1, double *q2);
void PrintQuaternion (double *q);
void RandomQuaternion (double *q, double maxtheta);
int MakeQuaternion (double *q, double angle, char axis);

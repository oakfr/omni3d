/*
 * C.J. Taylor's implementation of the fortran DSIGN function
 */

double d_sign (double *a, double *b)
{
  if (*b < 0.0) {
    if (*a > 0.0) *a = -*a;
  } else {
    if (*a < 0.0) *a = -*a;
  }

  return (*a);
}

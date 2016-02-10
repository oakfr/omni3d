/* daxpy.f -- translated by f2c (version of 23 January 1990  9:30:25).
   You must link the resulting object file with the libraries:
	-lF77 -lI77 -lm -lc   (in that order)
*/

#include "f2c.h"

/* Subroutine */ int daxpy_(n, da, dx, incx, dy, incy)
integer *n;
doublereal *da;
doublereal *dx;
integer *incx;
doublereal *dy;
integer *incy;
{
    /* System generated locals */
    integer i_1;

    /* Local variables */
    static integer i, m, ix, iy, mp1;

    /* Parameter adjustments */
    --dx;
    --dy;

    /* Function Body */

/*     constant times a vector plus a vector. */
/*     uses unrolled loops for increments equal to one. */
/*     jack dongarra, linpack, 3/11/78. */


    if (*n <= 0) {
	return 0;
    }
    if (*da == 0.) {
	return 0;
    }
    if (*incx == 1 && *incy == 1) {
	goto L20;
    }

/*        code for unequal increments or equal increments */
/*          not equal to 1 */

    ix = 1;
    iy = 1;
    if (*incx < 0) {
	ix = (-(*n) + 1) * *incx + 1;
    }
    if (*incy < 0) {
	iy = (-(*n) + 1) * *incy + 1;
    }
    i_1 = *n;
    for (i = 1; i <= i_1; ++i) {
	dy[iy] += *da * dx[ix];
	ix += *incx;
	iy += *incy;
/* L10: */
    }
    return 0;

/*        code for both increments equal to 1 */


/*        clean-up loop */

L20:
    m = *n % 4;
    if (m == 0) {
	goto L40;
    }
    i_1 = m;
    for (i = 1; i <= i_1; ++i) {
	dy[i] += *da * dx[i];
/* L30: */
    }
    if (*n < 4) {
	return 0;
    }
L40:
    mp1 = m + 1;
    i_1 = *n;
    for (i = mp1; i <= i_1; i += 4) {
	dy[i] += *da * dx[i];
	dy[i + 1] += *da * dx[i + 1];
	dy[i + 2] += *da * dx[i + 2];
	dy[i + 3] += *da * dx[i + 3];
/* L50: */
    }
    return 0;
} /* daxpy_ */


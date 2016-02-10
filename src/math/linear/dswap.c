/* dswap.f -- translated by f2c (version of 23 January 1990  9:30:25).
   You must link the resulting object file with the libraries:
	-lF77 -lI77 -lm -lc   (in that order)
*/

#include "f2c.h"

/* Subroutine */ int dswap_(n, dx, incx, dy, incy)
integer *n;
doublereal *dx;
integer *incx;
doublereal *dy;
integer *incy;
{
    /* System generated locals */
    integer i_1;

    /* Local variables */
    static integer i, m;
    static doublereal dtemp;
    static integer ix, iy, mp1;

    /* Parameter adjustments */
    --dx;
    --dy;

    /* Function Body */

/*     interchanges two vectors. */
/*     uses unrolled loops for increments equal one. */
/*     jack dongarra, linpack, 3/11/78. */


    if (*n <= 0) {
	return 0;
    }
    if (*incx == 1 && *incy == 1) {
	goto L20;
    }

/*       code for unequal increments or equal increments not equal */
/*         to 1 */

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
	dtemp = dx[ix];
	dx[ix] = dy[iy];
	dy[iy] = dtemp;
	ix += *incx;
	iy += *incy;
/* L10: */
    }
    return 0;

/*       code for both increments equal to 1 */


/*       clean-up loop */

L20:
    m = *n % 3;
    if (m == 0) {
	goto L40;
    }
    i_1 = m;
    for (i = 1; i <= i_1; ++i) {
	dtemp = dx[i];
	dx[i] = dy[i];
	dy[i] = dtemp;
/* L30: */
    }
    if (*n < 3) {
	return 0;
    }
L40:
    mp1 = m + 1;
    i_1 = *n;
    for (i = mp1; i <= i_1; i += 3) {
	dtemp = dx[i];
	dx[i] = dy[i];
	dy[i] = dtemp;
	dtemp = dx[i + 1];
	dx[i + 1] = dy[i + 1];
	dy[i + 1] = dtemp;
	dtemp = dx[i + 2];
	dx[i + 2] = dy[i + 2];
	dy[i + 2] = dtemp;
/* L50: */
    }
    return 0;
} /* dswap_ */


/* dscal.f -- translated by f2c (version of 23 January 1990  9:30:25).
   You must link the resulting object file with the libraries:
	-lF77 -lI77 -lm -lc   (in that order)
*/

#include "f2c.h"

/* Subroutine */ int dscal_(n, da, dx, incx)
integer *n;
doublereal *da;
doublereal *dx;
integer *incx;
{
    /* System generated locals */
    integer i_1, i_2;

    /* Local variables */
    static integer i, m, nincx, mp1;

    /* Parameter adjustments */
    --dx;

    /* Function Body */

/*     scales a vector by a constant. */
/*     uses unrolled loops for increment equal to one. */
/*     jack dongarra, linpack, 3/11/78. */
/*     modified 3/93 to return if incx .le. 0. */


    if (*n <= 0 || *incx <= 0) {
	return 0;
    }
    if (*incx == 1) {
	goto L20;
    }

/*        code for increment not equal to 1 */

    nincx = *n * *incx;
    i_1 = nincx;
    i_2 = *incx;
    for (i = 1; i_2 < 0 ? i >= i_1 : i <= i_1; i += i_2) {
	dx[i] = *da * dx[i];
/* L10: */
    }
    return 0;

/*        code for increment equal to 1 */


/*        clean-up loop */

L20:
    m = *n % 5;
    if (m == 0) {
	goto L40;
    }
    i_2 = m;
    for (i = 1; i <= i_2; ++i) {
	dx[i] = *da * dx[i];
/* L30: */
    }
    if (*n < 5) {
	return 0;
    }
L40:
    mp1 = m + 1;
    i_2 = *n;
    for (i = mp1; i <= i_2; i += 5) {
	dx[i] = *da * dx[i];
	dx[i + 1] = *da * dx[i + 1];
	dx[i + 2] = *da * dx[i + 2];
	dx[i + 3] = *da * dx[i + 3];
	dx[i + 4] = *da * dx[i + 4];
/* L50: */
    }
    return 0;
} /* dscal_ */


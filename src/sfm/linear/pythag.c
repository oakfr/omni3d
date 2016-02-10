/* pythag.f -- translated by f2c (version of 23 January 1990  9:30:25).
   You must link the resulting object file with the libraries:
	-lF77 -lI77 -lm -lc   (in that order)
*/

#include "f2c.h"

doublereal pythag_(a, b)
doublereal *a, *b;
{
    /* System generated locals */
    doublereal ret_val, d_1, d_2, d_3;

    /* Local variables */
    static doublereal p, r, s, t, u;


/*     finds dsqrt(a**2+b**2) without overflow or destructive underflow */


/* Computing MAX */
    d_1 = abs(*a), d_2 = abs(*b);
    p = max(d_2,d_1);
    if (p == 0.) {
	goto L20;
    }
/* Computing MAX */
    d_2 = abs(*a), d_3 = abs(*b);
/* Computing 2nd power */
    d_1 = min(d_3,d_2) / p;
    r = d_1 * d_1;
L10:
    t = r + 4.;
    if (t == 4.) {
	goto L20;
    }
    s = r / t;
    u = s * 2. + 1.;
    p = u * p;
/* Computing 2nd power */
    d_1 = s / u;
    r = d_1 * d_1 * r;
    goto L10;
L20:
    ret_val = p;
    return ret_val;
} /* pythag_ */


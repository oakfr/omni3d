/* drotg.f -- translated by f2c (version of 23 January 1990  9:30:25).
   You must link the resulting object file with the libraries:
	-lF77 -lI77 -lm -lc   (in that order)
*/

#include "f2c.h"

/* Table of constant values */

static doublereal c_b4 = 1.;

/* Subroutine */ int drotg_(da, db, c, s)
doublereal *da, *db, *c, *s;
{
    /* System generated locals */
    doublereal d_1, d_2;

    /* Builtin functions */
    double sqrt(), d_sign();

    /* Local variables */
    static doublereal r, scale, z, roe;


/*     construct givens plane rotation. */
/*     jack dongarra, linpack, 3/11/78. */
/*                    modified 9/27/86. */


    roe = *db;
    if (abs(*da) > abs(*db)) {
	roe = *da;
    }
    scale = abs(*da) + abs(*db);
    if (scale != 0.) {
	goto L10;
    }
    *c = 1.;
    *s = 0.;
    r = 0.;
    goto L20;
L10:
/* Computing 2nd power */
    d_1 = *da / scale;
/* Computing 2nd power */
    d_2 = *db / scale;
    r = scale * sqrt(d_1 * d_1 + d_2 * d_2);
    r = d_sign(&c_b4, &roe) * r;
    *c = *da / r;
    *s = *db / r;
L20:
    z = *s;
    if (abs(*c) > 0. && abs(*c) <= *s) {
	z = 1. / *c;
    }
    *da = r;
    *db = z;
    return 0;
} /* drotg_ */


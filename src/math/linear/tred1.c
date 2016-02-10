/* tred1.f -- translated by f2c (version of 23 January 1990  9:30:25).
   You must link the resulting object file with the libraries:
	-lF77 -lI77 -lm -lc   (in that order)
*/

#include "f2c.h"

/* Subroutine */ int tred1_(nm, n, a, d, e, e2)
integer *nm, *n;
doublereal *a, *d, *e, *e2;
{
    /* System generated locals */
    integer a_dim1, a_offset, i_1, i_2, i_3;
    doublereal d_1;

    /* Builtin functions */
    double sqrt(), d_sign();

    /* Local variables */
    static doublereal f, g, h;
    static integer i, j, k, l;
    static doublereal scale;
    static integer ii, jp1;

    /* Parameter adjustments */
    a_dim1 = *nm;
    a_offset = a_dim1 + 1;
    a -= a_offset;
    --d;
    --e;
    --e2;

    /* Function Body */


/*     this subroutine is a translation of the algol procedure tred1, */
/*     num. math. 11, 181-195(1968) by martin, reinsch, and wilkinson. */
/*     handbook for auto. comp., vol.ii-linear algebra, 212-226(1971). */

/*     this subroutine reduces a real symmetric matrix */
/*     to a symmetric tridiagonal matrix using */
/*     orthogonal similarity transformations. */

/*     on input */

/*        nm must be set to the row dimension of two-dimensional */
/*          array parameters as declared in the calling program */
/*          dimension statement. */

/*        n is the order of the matrix. */

/*        a contains the real symmetric input matrix.  only the */
/*          lower triangle of the matrix need be supplied. */

/*     on output */

/*        a contains information about the orthogonal trans- */
/*          formations used in the reduction in its strict lower */
/*          triangle.  the full upper triangle of a is unaltered. */

/*        d contains the diagonal elements of the tridiagonal matrix. */

/*        e contains the subdiagonal elements of the tridiagonal */
/*          matrix in its last n-1 positions.  e(1) is set to zero. */

/*        e2 contains the squares of the corresponding elements of e. */
/*          e2 may coincide with e if the squares are not needed. */

/*     questions and comments should be directed to burton s. garbow, */
/*     mathematics and computer science div, argonne national laboratory 
*/

/*     this version dated august 1983. */

/*     ------------------------------------------------------------------ 
*/

    i_1 = *n;
    for (i = 1; i <= i_1; ++i) {
	d[i] = a[*n + i * a_dim1];
	a[*n + i * a_dim1] = a[i + i * a_dim1];
/* L100: */
    }
/*     .......... for i=n step -1 until 1 do -- .......... */
    i_1 = *n;
    for (ii = 1; ii <= i_1; ++ii) {
	i = *n + 1 - ii;
	l = i - 1;
	h = 0.;
	scale = 0.;
	if (l < 1) {
	    goto L130;
	}
/*     .......... scale row (algol tol then not needed) .......... */
	i_2 = l;
	for (k = 1; k <= i_2; ++k) {
/* L120: */
	    scale += (d_1 = d[k], abs(d_1));
	}

	if (scale != 0.) {
	    goto L140;
	}

	i_2 = l;
	for (j = 1; j <= i_2; ++j) {
	    d[j] = a[l + j * a_dim1];
	    a[l + j * a_dim1] = a[i + j * a_dim1];
	    a[i + j * a_dim1] = 0.;
/* L125: */
	}

L130:
	e[i] = 0.;
	e2[i] = 0.;
	goto L300;

L140:
	i_2 = l;
	for (k = 1; k <= i_2; ++k) {
	    d[k] /= scale;
	    h += d[k] * d[k];
/* L150: */
	}

	e2[i] = scale * scale * h;
	f = d[l];
	d_1 = sqrt(h);
	g = -d_sign(&d_1, &f);
	e[i] = scale * g;
	h -= f * g;
	d[l] = f - g;
	if (l == 1) {
	    goto L285;
	}
/*     .......... form a*u .......... */
	i_2 = l;
	for (j = 1; j <= i_2; ++j) {
/* L170: */
	    e[j] = 0.;
	}

	i_2 = l;
	for (j = 1; j <= i_2; ++j) {
	    f = d[j];
	    g = e[j] + a[j + j * a_dim1] * f;
	    jp1 = j + 1;
	    if (l < jp1) {
		goto L220;
	    }

	    i_3 = l;
	    for (k = jp1; k <= i_3; ++k) {
		g += a[k + j * a_dim1] * d[k];
		e[k] += a[k + j * a_dim1] * f;
/* L200: */
	    }

L220:
	    e[j] = g;
/* L240: */
	}
/*     .......... form p .......... */
	f = 0.;

	i_2 = l;
	for (j = 1; j <= i_2; ++j) {
	    e[j] /= h;
	    f += e[j] * d[j];
/* L245: */
	}

	h = f / (h + h);
/*     .......... form q .......... */
	i_2 = l;
	for (j = 1; j <= i_2; ++j) {
/* L250: */
	    e[j] -= h * d[j];
	}
/*     .......... form reduced a .......... */
	i_2 = l;
	for (j = 1; j <= i_2; ++j) {
	    f = d[j];
	    g = e[j];

	    i_3 = l;
	    for (k = j; k <= i_3; ++k) {
/* L260: */
		a[k + j * a_dim1] = a[k + j * a_dim1] - f * e[k] - g * d[k];
	    }

/* L280: */
	}

L285:
	i_2 = l;
	for (j = 1; j <= i_2; ++j) {
	    f = d[j];
	    d[j] = a[l + j * a_dim1];
	    a[l + j * a_dim1] = a[i + j * a_dim1];
	    a[i + j * a_dim1] = f * scale;
/* L290: */
	}

L300:
    ;}

    return 0;
} /* tred1_ */


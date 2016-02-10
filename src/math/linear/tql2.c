/* tql2.f -- translated by f2c (version of 23 January 1990  9:30:25).
   You must link the resulting object file with the libraries:
	-lF77 -lI77 -lm -lc   (in that order)
*/

#include "f2c.h"

/* Table of constant values */

static doublereal c_b10 = 1.;

/* Subroutine */ int tql2_(nm, n, d, e, z, ierr)
integer *nm, *n;
doublereal *d, *e, *z;
integer *ierr;
{
    /* System generated locals */
    integer z_dim1, z_offset, i_1, i_2, i_3;
    doublereal d_1, d_2;

    /* Builtin functions */
    double d_sign();

    /* Local variables */
    static doublereal c, f, g, h;
    static integer i, j, k, l, m;
    static doublereal p, r, s, c2, c3;
    static integer l1, l2;
    static doublereal s2;
    static integer ii;
    extern doublereal pythag_();
    static doublereal dl1, el1;
    static integer mml;
    static doublereal tst1, tst2;

    /* Parameter adjustments */
    --d;
    --e;
    z_dim1 = *nm;
    z_offset = z_dim1 + 1;
    z -= z_offset;

    /* Function Body */


/*     this subroutine is a translation of the algol procedure tql2, */
/*     num. math. 11, 293-306(1968) by bowdler, martin, reinsch, and */
/*     wilkinson. */
/*     handbook for auto. comp., vol.ii-linear algebra, 227-240(1971). */

/*     this subroutine finds the eigenvalues and eigenvectors */
/*     of a symmetric tridiagonal matrix by the ql method. */
/*     the eigenvectors of a full symmetric matrix can also */
/*     be found if  tred2  has been used to reduce this */
/*     full matrix to tridiagonal form. */

/*     on input */

/*        nm must be set to the row dimension of two-dimensional */
/*          array parameters as declared in the calling program */
/*          dimension statement. */

/*        n is the order of the matrix. */

/*        d contains the diagonal elements of the input matrix. */

/*        e contains the subdiagonal elements of the input matrix */
/*          in its last n-1 positions.  e(1) is arbitrary. */

/*        z contains the transformation matrix produced in the */
/*          reduction by  tred2, if performed.  if the eigenvectors */
/*          of the tridiagonal matrix are desired, z must contain */
/*          the identity matrix. */

/*      on output */

/*        d contains the eigenvalues in ascending order.  if an */
/*          error exit is made, the eigenvalues are correct but */
/*          unordered for indices 1,2,...,ierr-1. */

/*        e has been destroyed. */

/*        z contains orthonormal eigenvectors of the symmetric */
/*          tridiagonal (or full) matrix.  if an error exit is made, */
/*          z contains the eigenvectors associated with the stored */
/*          eigenvalues. */

/*        ierr is set to */
/*          zero       for normal return, */
/*          j          if the j-th eigenvalue has not been */
/*                     determined after 30 iterations. */

/*     calls pythag for  dsqrt(a*a + b*b) . */

/*     questions and comments should be directed to burton s. garbow, */
/*     mathematics and computer science div, argonne national laboratory 
*/

/*     this version dated august 1983. */

/*     ------------------------------------------------------------------ 
*/

    *ierr = 0;
    if (*n == 1) {
	goto L1001;
    }

    i_1 = *n;
    for (i = 2; i <= i_1; ++i) {
/* L100: */
	e[i - 1] = e[i];
    }

    f = 0.;
    tst1 = 0.;
    e[*n] = 0.;

    i_1 = *n;
    for (l = 1; l <= i_1; ++l) {
	j = 0;
	h = (d_1 = d[l], abs(d_1)) + (d_2 = e[l], abs(d_2));
	if (tst1 < h) {
	    tst1 = h;
	}
/*     .......... look for small sub-diagonal element .......... */
	i_2 = *n;
	for (m = l; m <= i_2; ++m) {
	    tst2 = tst1 + (d_1 = e[m], abs(d_1));
	    if (tst2 == tst1) {
		goto L120;
	    }
/*     .......... e(n) is always zero, so there is no exit */
/*                through the bottom of the loop .......... */
/* L110: */
	}

L120:
	if (m == l) {
	    goto L220;
	}
L130:
	if (j == 30) {
	    goto L1000;
	}
	++j;
/*     .......... form shift .......... */
	l1 = l + 1;
	l2 = l1 + 1;
	g = d[l];
	p = (d[l1] - g) / (e[l] * 2.);
	r = pythag_(&p, &c_b10);
	d[l] = e[l] / (p + d_sign(&r, &p));
	d[l1] = e[l] * (p + d_sign(&r, &p));
	dl1 = d[l1];
	h = g - d[l];
	if (l2 > *n) {
	    goto L145;
	}

	i_2 = *n;
	for (i = l2; i <= i_2; ++i) {
/* L140: */
	    d[i] -= h;
	}

L145:
	f += h;
/*     .......... ql transformation .......... */
	p = d[m];
	c = 1.;
	c2 = c;
	el1 = e[l1];
	s = 0.;
	mml = m - l;
/*     .......... for i=m-1 step -1 until l do -- .......... */
	i_2 = mml;
	for (ii = 1; ii <= i_2; ++ii) {
	    c3 = c2;
	    c2 = c;
	    s2 = s;
	    i = m - ii;
	    g = c * e[i];
	    h = c * p;
	    r = pythag_(&p, &e[i]);
	    e[i + 1] = s * r;
	    s = e[i] / r;
	    c = p / r;
	    p = c * d[i] - s * g;
	    d[i + 1] = h + s * (c * g + s * d[i]);
/*     .......... form vector .......... */
	    i_3 = *n;
	    for (k = 1; k <= i_3; ++k) {
		h = z[k + (i + 1) * z_dim1];
		z[k + (i + 1) * z_dim1] = s * z[k + i * z_dim1] + c * h;
		z[k + i * z_dim1] = c * z[k + i * z_dim1] - s * h;
/* L180: */
	    }

/* L200: */
	}

	p = -s * s2 * c3 * el1 * e[l] / dl1;
	e[l] = s * p;
	d[l] = c * p;
	tst2 = tst1 + (d_1 = e[l], abs(d_1));
	if (tst2 > tst1) {
	    goto L130;
	}
L220:
	d[l] += f;
/* L240: */
    }
/*     .......... order eigenvalues and eigenvectors .......... */
    i_1 = *n;
    for (ii = 2; ii <= i_1; ++ii) {
	i = ii - 1;
	k = i;
	p = d[i];

	i_2 = *n;
	for (j = ii; j <= i_2; ++j) {
	    if (d[j] >= p) {
		goto L260;
	    }
	    k = j;
	    p = d[j];
L260:
	;}

	if (k == i) {
	    goto L300;
	}
	d[k] = d[i];
	d[i] = p;

	i_2 = *n;
	for (j = 1; j <= i_2; ++j) {
	    p = z[j + i * z_dim1];
	    z[j + i * z_dim1] = z[j + k * z_dim1];
	    z[j + k * z_dim1] = p;
/* L280: */
	}

L300:
    ;}

    goto L1001;
/*     .......... set error -- no convergence to an */
/*                eigenvalue after 30 iterations .......... */
L1000:
    *ierr = l;
L1001:
    return 0;
} /* tql2_ */


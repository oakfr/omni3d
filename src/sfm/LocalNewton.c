/*
 * LocalNewton.c : This file implements a full Newton non-linear minimization
 * scheme based on the routines found in "Numerical Methods for Unconstrained
 * Optimization and Nonlinear Equations" by Dennis and Shnabel (1983).
 * These routines have been extended so that they can also be applied
 * to parameter spaces that are not isomorphic to Rn.
 */

#include <math.h>
//#include <values.h>

/* Defines *******************************************************/

#define MAX_N     500
#define MACHEPS   5.5511151231257839e-17

#define min(a,b) ((a) <= (b) ? (a) : (b))
#define max(a,b) ((a) >= (b) ? (a) : (b))
#define sqr(a)   ((a)*(a))

#pragma warning(disable: 4013)
#pragma warning(disable: 4033)
#pragma warning(disable: 4715)
#pragma warning(disable: 4716)

/*
 * elt calculates the index of an element in a symmetric matrix stored in 
 * row major, lower triangular form.
 */

#define elt(row,col)  ((col) > (row) ? ((((col)*(col) + (col))/2) + (row)) : \
                                       ((((row)*(row) + (row))/2) + (col)))

/* Forward Declarations ******************************************/

/* User supplied routines */

int FN   (double *x, double *f);
int GRAD (double *x, double *g);
int HESS (double *x, double *H);
int UPDATE (double *x, double *step, double *newx);
int REPORT (int itncount, double *x,
	    double fc, double *gc, double *H,
	    int nfev, int njev, int nhev);

int UMDRIVER (int n, int m,
	      int (*FN)(), int (*GRAD)(), int (*HESS)(), int (*UPDATE)(),
	      int (*REPORT)(),
	      double *typx, double typf,
	      double gradtol, double steptol, double maxstep, int itnlimit,
	      double *xc, double *fc, double *gc, 
	      int *nfev, int *njev, int *nhev, int *info,
	      double *L, double *H);

int DOGDRIVER (int n, int m, double *xc, double fc,
	       int (*FN)(), int (*UPDATE)(),
	       double *gc, double *L, double *sN, double *Sx, double *s,
	       double maxstep, double steptol,
	       double *delta, double *x_plus, double *f_plus, 
	       int *maxtaken, int *fev);

int DOGSTEP (int n, double *gc, double *L, double *sN, double *Sx, 
	     double Newtlen, double maxstep, double *delta, 
	     int *firstdog, double *Cauchylen, double *nu, double *sSD, 
	     double *v, double *s, int *Newttaken);

int TRUSREGUP (int n, int m, double *xc, double fc,
	       int (*FN)(), int (*UPDATE)(),
	       double *gc, double *L, double *s, double *Sx, int Newttaken,
	       double maxstep, double steptol,
	       double *delta, int *retcode,
	       double *x_plus_prev, double *s_prev, double *f_plus_prev,
	       double *x_plus, double *f_plus, int *maxtaken, int *fev);

/* Subroutines ***************************************************/

/*
 * UMDRIVER : The Unconstrained Minimization Driver. This module
 * implements a full newton minimization scheme using a Dogleg trust
 * region step scheme.
 *
 * Description of the arguments :
 *
 * int n : input, The dimension of the tangent space.
 *
 * int m : input, The dimension of the parameter space.
 *
 * int FN (double *x, double *f) : input, 
 *  User supplied objective function.
 *
 * GRAD (double *x, double *g) : input,
 *  User supplied function for calculating the Jacobian.
 *
 * HESS (double *x, double *g) : input,
 *  User supplied function for computing the Hessian.
 *
 * UPDATE (double *x, double *step, double *newx) : input,
 *  User supplied function for updating the parameter vector 
 *  with a step vector.
 *
 * REPORT (int itncount, double *x,
 *         double fc, double *gc, double *H,
 *	   int nfev, int njev, int nhev) : input,
 *  User supplied function for logging information about the minimization
 *  process. Set this to a NULL pointer if you don't need it.
 *
 * double typx[n] : An array of +ve values greater than 0 that indicate
 *  the expected magnitudes of the elements of the parameter vector, x.
 *
 * double typf : input, Typical value of the objective function at the minimum.
 *
 * double gradtol : input, The tolerance on the scaled gradient magnitude.
 *
 * double steptol : input, Tolerance on the scaled step size.
 *
 * double maxsstep : input, The maximum size of the scaled step.
 *
 * int itnlimit : input, Maximum number of iterations.
 *
 * double *xc : input-output, This vector should contain the initial estimate
 *  for the parameters on input and will contain the final estimate on 
 *  termination.
 *
 * double *f : output, The final value of the objective function.
 *
 * double *gc : output, The final value of the gradient.
 *
 * int *nfev : output, The number of function evaluations.
 *
 * int *njev : output, The number of Jacobian evaluations.
 *
 * int *nhev : output, The number of Hessian evaluations.
 *
 * int *termcode : output, This variable indicates the type of termination
 *   condition encountered.
 *
 *    termcode = 0 : Bad input parameters.
 *
 *    termocde = 1 : Norm of scaled gradient less than gradtol.
 *
 *    termcode = 2 : Norm of scaled distance between consecutive
 *      steps less than steptol.
 *
 *    termcode = 3 : Last global step failed to locate a point lower than the
 *      current estimate. Either x is a local minimizer or the gradient
 *      routine has a bug.
 *
 *    termcode = 4 : iteration limit exceeded.
 *
 *    termcode = 5 : Five consecutive steps of length maxstep have been taken;
 *      either f(x) is unbounded below or f(x) has a finite asymptote in
 *      some direction.
 *
 * double *L, *H : Working variables These should be arrays of doubles
 *   each array must contain at least n*n elements since they will be used
 *   to store two dimensional arrays.
 *
 */

int UMDRIVER (int n, int m,
	      int (*FN)(), int (*GRAD)(), int (*HESS)(), int (*UPDATE)(),
	      int (*REPORT)(),
	      double *typx, double typf,
	      double gradtol, double steptol, double maxstep, int itnlimit,
	      double *xc, double *fc, double *gc, 
	      int *nfev, int *njev, int *nhev, int *termcode,
	      double *L, double *H)
{
  double Sx[MAX_N], sN[MAX_N], x_plus[MAX_N];
  double f_plus, g_plus[MAX_N], step[MAX_N];
  int i, maxtaken, itncount=0, consecmax=0;
  double delta = -1.0;

  *nfev = *njev = *nhev = 0;

  /* Checking the input parameters */
  if ( (n < 1) || (n > MAX_N) ||
       (m < 1) || (m > MAX_N) ||
       (gradtol < 0.0) || (steptol < 0.0) ||
       (maxstep < 0.0) || (itnlimit < 1) ) {
    *termcode = 0;
    return 0;
  }

  /* Setting up the scale factors */
  for (i=0; i < n; ++i) Sx[i] = 1/typx[i];
  
  itncount = 0;

  FN (xc, fc);    ++(*nfev);
  GRAD (xc, gc);  ++(*njev);
  HESS (xc, H);   ++(*nhev);

  if (REPORT) REPORT (itncount, xc, *fc, gc, H, *nfev, *njev, *nhev);

  *termcode = UMSTOP0 (n, xc, *fc, gc, Sx, typf, gradtol);
  if (*termcode) return;

  while (!*termcode) {
    ++itncount;
    MODELHESS (n, Sx, MACHEPS, H, L);
    CHOLSOLVE (n, gc, L, sN);
    *termcode = DOGDRIVER (n, m, xc, *fc, FN, UPDATE, gc, L, sN, Sx, step, 
			   maxstep, steptol, 
			   &delta, x_plus, &f_plus, &maxtaken, nfev);
    GRAD (x_plus, g_plus);   ++(*njev);
    *termcode = UMSTOP (n, xc, x_plus, f_plus, g_plus,
			Sx, typf, *termcode, gradtol, steptol,
			itncount, itnlimit, maxtaken, &consecmax,
			step);
    
    for (i=0; i < m; ++i) xc[i] = x_plus[i];
    *fc = f_plus;
    for (i=0; i < n; ++i) gc[i] = g_plus[i];
    HESS (xc, H);   ++(*nhev);

    if (REPORT) REPORT (itncount, xc, *fc, gc, H, *nfev, *njev, *nhev);
  }

  return 0;
}

/*
 * MODELHESS : This routine produces an approximation for the Hessian
 * matrix that is safely positive definite.
 */

int MODELHESS (int n, double *Sx, double macheps, double *H, double *L)
{
  int i, j;
  double sqrteps, maxdiag, mindiag, maxposdiag, maxoff;
  double maxadd, sdd, mu, maxoffl, maxev, minev, offrow, temp;

  /* Scale H */

  for (i=0; i < n; ++i)
    for (j=0; j <= i; ++j)
      H[elt(i,j)] /= (Sx[i]*Sx[j]);

  sqrteps = sqrt(macheps);
  
  for (i=1, maxdiag=mindiag=H[0]; i < n; ++i) {
    temp =  H[elt(i,i)];
    maxdiag = max(maxdiag, temp);
    mindiag = min(mindiag, temp);
  }
  
  maxposdiag = max(0, maxdiag);

  if (mindiag <= sqrteps*maxposdiag) {
    mu = 2*(maxposdiag - mindiag)*sqrteps - mindiag;
    maxdiag = maxdiag + mu;
  } else
    mu = 0.0;

  for (i=0, maxoff=H[1]; i < n; ++i)
    for (j=0; j < i; ++j)
      maxoff = max (maxoff, fabs (H[elt(i,j)]));

  if (maxoff*(1 + 2*sqrteps) > maxdiag) {
    mu += (maxoff - maxdiag) + 2*sqrteps * maxoff;
    maxdiag = maxoff * (1 + 2*sqrteps);
  }

  if (maxdiag == 0.0) {
    mu = 1.0;
    maxdiag = 1.0;
  }
  
  if (mu > 0) {
    for (i=0; i < n; ++i)
      H[elt(i,i)] += mu;
  }
  
  maxoffl = sqrt(max(maxdiag, (maxoff/n)));

  CHOLDECOMP (n, H, maxoffl, macheps, L, &maxadd);

  if (maxadd > 0) { /* H wasn't positive definite */
    maxev = minev = H[0];
    for (i=0; i < n; ++i) {
      offrow=0.0;
      for (j=0;   j < i; ++j) offrow += fabs (H[elt(j,i)]);
      for (j=i+1; j < n; ++j) offrow += fabs (H[elt(i,j)]);
      temp = H[elt(i,i)];
      maxev = max(maxev, temp + offrow);
      minev = min(minev, temp - offrow);
    }

    sdd = (maxev - minev)*sqrteps - minev;
    sdd = max(sdd, 0.0);
    mu = min(maxadd, sdd);
    
    for (i=0; i < n; ++i)
      H[elt(i,i)] += mu;

    CHOLDECOMP (n, H, 0.0, macheps, L, &maxadd);
  }

  /* Unscale H and L */

  for (i=0; i < n; ++i)
    for (j=0; j <= i; ++j)
      H[elt(i,j)] *= (Sx[i]*Sx[j]);

  for (i=0; i < n; ++i)
    for (j=0; j <= i; ++j)
      L[elt(i,j)] *= Sx[i];
}

/*
 * CHOLDECOMP : This routine computes the Cholesky decomposition of a
 * symmetric matrix.
 */

int CHOLDECOMP (int n, double *H, double maxoffl, double macheps,
		double *L, double *maxadd)
{
  int i, j, k;
  double minl, minl2, temp, minljj;

  minl = sqrt(sqrt(macheps))*maxoffl;

  if (maxoffl == 0.0) {
    for (i=0, temp=0.0; i < n; ++i)
      temp = max(temp, fabs(H[elt(i,i)]));
    maxoffl = sqrt(temp);
    minl2 = sqrt(macheps) * maxoffl;
  }

  /*
   * maxadd will contain the maximum amount implicitly added to any
   * diagonal element of H in forming L.
   */

  *maxadd = 0.0;

  for (j=0; j < n; ++j) {
    for (i=0, temp=0.0; i < j; ++i)
      temp += sqr(L[elt(j,i)]);

    L[elt(j,j)] = H[elt(j,j)] - temp;

    minljj = 0.0;
    
    for (i=j+1; i < n; ++i) {
      for (k=0, temp=0.0; k < j; ++k)
	temp += L[elt(i,k)] * L[elt(j,k)];
      L[elt(i,j)] = H[elt(j,i)] - temp;
      minljj = max(fabs(L[elt(i,j)]), minljj);
    }

    minljj = max(minljj/maxoffl, minl);

    if (L[elt(j,j)] > sqr(minljj)) /* Normal Chloesky iteration */
      L[elt(j,j)] = sqrt(L[elt(j,j)]);
    else {
      if (minljj < minl2) minljj = minl2;
      *maxadd = max(*maxadd, sqr(minljj) - L[elt(j,j)]);
      L[elt(j,j)] = minljj;
    }

    temp = L[elt(j,j)];
    for (i=j+1; i < n; ++i)
      L[elt(i,j)] /= temp;
  }
}

/*
 * CHOLSOLVE : This function is used to solve the linear system
 * L*Transpose(L)*s = -g. Notice the -ve sign.
 */

int CHOLSOLVE (int n, double *g, double *L, double *s)
{
  int i;

  /* Solve Ly = g */
  LSOLVE (n, g, L, s);
  /* Solve Transpose(L)s = y */
  LTSOLVE (n, s, L, s);

  for (i=0; i < n; ++i) s[i] = -s[i];
}

int LSOLVE (int n, double *b, double *L, double *y)
{
  int i, j;
  double temp;
  
  y[0] = b[0] / L[0];

  for (i=1; i < n; ++i) {
    for (j=0, temp=0.0; j < i; ++j)
      temp += L[elt(i,j)]*y[j];

    y[i] = (b[i] - temp) / L[elt(i,i)];
  }
}

int LTSOLVE (int n, double *y, double *L, double *x)
{
  int i, j;
  double temp;
  
  x[n-1] = y[n-1] / L[elt((n-1), (n-1))];

  for (i=n-2; i > -1 ; --i) {
    for (j=i+1, temp=0.0; j < n; ++j)
      temp += L[elt(j,i)]*x[j];

    x[i] = (y[i] - temp) / L[elt(i,i)];
  }
}

/*
 * DOGDRIVER : This routine finds an x_plus on the double dogleg curve
 * that lies within the trust region redius , delta, it also updates
 * the trust region.
 */

int DOGDRIVER (int n, int m, double *xc, double fc,
	       int (*FN)(), int (*UPDATE)(),
	       double *gc, double *L, double *sN, double *Sx, double *s,
	       double maxstep, double steptol,
	       double *delta, double *x_plus, double *f_plus, 
	       int *maxtaken, int *fev)
{
  int i, retcode=4, firstdog=1, Newttaken;
  double Newtlen, temp, Cauchylen, nu;
  double sSD[MAX_N], v[MAX_N], x_plus_prev[MAX_N], s_prev[MAX_N], f_plus_prev;
  
  /* Calculating the length of the Newton step */
  for (i=0, Newtlen=0.0; i < n; ++i) {
    temp = Sx[i]*sN[i];
    Newtlen += temp*temp;
  }

  Newtlen = sqrt (Newtlen);

  do {
    DOGSTEP (n, gc, L, sN, Sx, Newtlen, maxstep, delta, &firstdog,
	     &Cauchylen, &nu, sSD, v, s, &Newttaken);
    TRUSREGUP (n, m, xc, fc, FN, UPDATE, gc, L, s, Sx, Newttaken, maxstep, 
	       steptol, delta, &retcode, x_plus_prev, s_prev, &f_plus_prev,
	       x_plus, f_plus, maxtaken, fev);
  } while (retcode >= 2);

  return (retcode);
}

/*
 * DOGSTEP : This routine finds the intersection of the polygonal arc
 * with the boundary of the trust region OR it finds the Newton step.
 */

int DOGSTEP (int n, double *gc, double *L, double *sN, double *Sx, 
	     double Newtlen, double maxstep, double *delta, 
	     int *firstdog, double *Cauchylen, double *nu, double *sSD, 
	     double *v, double *s, int *Newttaken)
{
  int i, j;
  double alpha, beta, temp, tempv, lambda;

  if (Newtlen < *delta) { /* s is the Newton step */
    *Newttaken = 1;
    for (i=0; i < n; ++i) s[i] = sN[i];
    *delta = Newtlen;
    return;
  }

  /* Newton step too long, s is on the dogleg curve */

  *Newttaken = 0;


  if (*firstdog) {   /* Calculate the double dogleg curve */
    *firstdog = 0;
    
    for (i=0, alpha=0.0; i < n; ++i) {
      temp = gc[i]/Sx[i];
      alpha += temp*temp;
    }
  
    for (i=0, beta=0.0; i < n; ++i) {
      for (j=i, temp=0.0; j < n; ++j)
	temp += (L[elt(j,i)]*gc[j])/(Sx[j]*Sx[j]);
      beta += temp*temp;
    }

    /* sSD is the Cauchy step in the scaled metric */
    for (i=0; i < n; ++i)
      sSD[i] = -(alpha/beta)*(gc[i]/Sx[i]);

    *Cauchylen = (alpha * sqrt(alpha)) / beta;

    for (i=0, temp=0.0; i < n; ++i)
      temp += gc[i]*sN[i];

    *nu = 0.2 + (0.8*alpha*alpha)/ (beta*fabs(temp));

    for (i=0; i < n; ++i)
      v[i] = (*nu)*Sx[i]*sN[i] - sSD[i];

    if (*delta == -1) {
      *delta = min(*Cauchylen, maxstep);
    }
  }

  if ((*nu * Newtlen) <= *delta) {
    /* Take a partial step in the Newton direction */
    temp = (*delta) / Newtlen;
    for (i=0; i < n; ++i)
      s[i] = temp*sN[i];
    return;
  }

  if (*Cauchylen >= *delta) {
    /* Take a partial step in the steepest descent direction */
    temp = (*delta) / (*Cauchylen);
    for (i=0; i < n; ++i)
      s[i] = temp*sSD[i]/Sx[i];
    return;
  }

  /* 
   * Else calculate a convex combination of sSD and nu*sN that has a scaled
   * length of delta.
   */

  for (i=0, temp=0.0; i < n; ++i)
    temp += v[i]*sSD[i];

  for (i=0, tempv=0.0; i < n; ++i)
    tempv += v[i]*v[i];

  lambda = -temp + sqrt(sqr(temp) - tempv*(sqr(*Cauchylen) - sqr(*delta)));
  lambda /= tempv;

  for (i=0; i < n; ++i)
    s[i] = (sSD[i] + lambda*v[i])/Sx[i];
}

int TRUSREGUP (int n, int m, double *xc, double fc,
	       int (*FN)(), int (*UPDATE)(),
	       double *gc, double *L, double *s, double *Sx, int Newttaken,
	       double maxstep, double steptol,
	       double *delta, int *retcode,
	       double *x_plus_prev, double *s_prev, double *f_plus_prev,
	       double *x_plus, double *f_plus, int *maxtaken, int *fev)
{
  int i, j;
  double alpha, steplen, temp, delta_f, initslope, rellength;
  double delta_f_pred;
  *maxtaken = 0;
  
  /* 
   * alpha is a constant used in the step acceptance test. It can
   * be changed by changing the following line.
   */

  alpha = 1e-4;
  
  /* calculating the norm of the scaled step */
  for (i=0, steplen=0.0; i < n; ++i) {
    temp = Sx[i]*s[i];
    steplen += temp*temp;
  }
  
  steplen = sqrt (steplen);

  /* Update step */
  UPDATE (xc, s, x_plus);

  FN (x_plus, f_plus);  ++(*fev);

  delta_f = *f_plus - fc;

  for (i=0, initslope=0.0; i < n; ++i)
    initslope += gc[i]*s[i];

  if (*retcode != 3) *f_plus_prev = 0.0;
  
  if ((*retcode == 3) && 
      ( (*f_plus >= *f_plus_prev)  || (delta_f > alpha*initslope) ) ) {
    /* reset x_plus to x_plus_prev and terminate global step */
    *retcode = 0;
    for (i=0; i < m; ++i) x_plus[i] = x_plus_prev[i];
    for (i=0; i < n; ++i) s[i] = s_prev[i];
    *f_plus = *f_plus_prev;
    *delta /= 2;
    return;
  }

  if (delta_f >= alpha*initslope) {
    /* f(x_plus) is too large */
    
    for(i=0, rellength=0.0; i < n; ++i) {
      temp = fabs(s[i]) * Sx[i];
      rellength = max (temp, rellength);
    }

    if (rellength < steptol) { 
      /* |x_plus - xc| too small, terminate the step */
      *retcode = 1;
      for (i=0; i < m; ++i) x_plus[i] = xc[i];
      for (i=0; i < n; ++i) s[i] = 0.0;
      return;
    }

    /* reduce delta, continue global step */
    *retcode = 2;
    temp = -(initslope*steplen) / (2*(delta_f - initslope));
    if (temp < 0.1*(*delta)) 
      *delta *= 0.1;
    else if (temp > 0.5*(*delta))
      *delta *= 0.5;
    else
      *delta = temp;
    return;
  }

  /* f(x_plus) sufficiently small */

  /* Calculating the predicted reduction in f */
  for (i=0, delta_f_pred=initslope; i < n; ++i) {
    for (j=i, temp=0.0; j < n; ++j)
      temp += L[elt(j,i)]*s[j];
    delta_f_pred += (temp*temp)/2;
  }

  if ( ( (*retcode != 2) &&
	 (fabs(delta_f_pred - delta_f) <= 0.1*fabs(delta_f)) ) ||
       ( (delta_f <= initslope) && (!Newttaken) &&
	 (*delta <= 0.99*maxstep) ) ) {
    *retcode = 3;
    for (i=0; i < m; ++i) x_plus_prev[i] = x_plus[i];
    for (i=0; i < n; ++i) s_prev[i] = s[i];
    *f_plus_prev = *f_plus;
    *delta = min(2*(*delta), maxstep);
    return;
  } else {
    *retcode = 0;
    if (steplen > 0.99*maxstep)
      *maxtaken = 1;
    if (delta_f >= 0.1*delta_f_pred)
      *delta /=2;
    else if (delta_f <= 0.75*delta_f_pred)
      *delta = min(2*(*delta), maxstep);
    return;
  }
}

/*
 * UMSTOP : This module implements the stopping criterion.
 */

int UMSTOP (int n, double *xc, double *x_plus, double f_plus, double *g_plus,
	    double *Sx, double typf, int retcode,
	    double gradtol, double steptol, int itncount, int itnlimit,
	    int maxtaken, int *consecmax, double *step)
{
  int i;
  double temp1, temp2;

  if (retcode == 1) return (3);

  /* computing the maximum component of the scaled gradient */
  for (i=0, temp2 = 0.0; i < n; ++i) {
    temp1 = fabs(g_plus[i]) / (Sx[i] * (max(fabs(f_plus), typf)));
    temp2 = max(temp1,temp2); 
  }
  
  if (temp2 <= gradtol) return (1);

  /* computing the maximum component of the scaled step */
  for (i=0, temp2 = 0.0; i < n; ++i) {
    temp1 = fabs(step[i]) * Sx[i];
    temp2 = max(temp1,temp2); 
  }

  if (temp2 <= steptol) return (2);
  
  if (itncount >= itnlimit) return (4);

  if (maxtaken) {
    /* A maximum length step was taken */
    ++(*consecmax);
    /* if 5 consecutive large steps were taken then terminate */
    if (*consecmax >= 5) return (5);
  } else
    *consecmax = 0;
 
  return (0);
}

int UMSTOP0 (int n, double *x, double f, double *g, double *Sx,
	     double typf, double gradtol)
{
  int i;
  double temp1, temp2;
 
  /* computing the maximum component of the scaled gradient */
  for (i=0, temp2 = 0.0; i < n; ++i) {
    temp1 = fabs(g[i]) / (Sx[i] * (max(fabs(f), typf)));
    temp2 = max(temp1,temp2); 
  }

  if (temp2 <= 0.001*gradtol)
    return (1);
  else
    return (0);
}

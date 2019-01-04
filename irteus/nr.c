///////////////////////////////////////////////////////////////////////////////
///
/// $Id$
///
/// Copyright (c) 1987- JSK, The University of Tokyo.  All Rights Reserved.
///
/// This software is a collection of EusLisp code for robot applications,
/// which has been developed by the JSK Laboratory for the IRT project.
/// For more information on EusLisp and its application to the robotics,
/// please refer to the following papers.
///
/// Toshihiro Matsui
/// Multithread object-oriented language euslisp for parallel and
///  asynchronous programming in robotics
/// Workshop on Concurrent Object-based Systems,
///  IEEE 6th Symposium on Parallel and Distributed Processing, 1994
///
/// Permission to use this software for educational, research
/// and non-profit purposes, without fee, and without a written
/// agreement is hereby granted to all researchers working on
/// the IRT project at the University of Tokyo, provided that the
/// above copyright notice remains intact.  
///

#include "eus.h"
#include "nr.h"
#include <math.h>

static double sqrarg;
#define SQR(a) ((sqrarg=(a)) == 0.0 ? 0.0 : sqrarg*sqrarg)

static double maxarg1, maxarg2;
#define FMAX(a,b) (maxarg1=(a),maxarg2=(b),(maxarg1) > (maxarg2) ? (maxarg1) : (maxarg2))

static int iminarg1, iminarg2;
#define IMIN(a,b) (iminarg1=(a),iminarg2=(b),(iminarg1) < (iminarg2) ? (iminarg1) : (iminarg2))

#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))

#define NR_END 1
#define FREE_ARG char*

void nrerror(char error_text[])
{
  fprintf(stderr,"Numerical Recipes run-time error...\n");
  fprintf(stderr,"%s\n",error_text);
  fprintf(stderr,"...now existing to system...\n");
}

double *nr_vector(int nl, int nh)
{
  double *v;
  v =(double *)malloc((size_t)((nh-nl+1+NR_END)*sizeof(double)));
  if (!v) {nrerror("allocation failure in nr_vector()"); return (double*)NULL;}
  return v-nl+NR_END;
}

double **nr_matrix(int nrl, int nrh, int ncl, int nch)
{
  int i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
  double **m;
  
  m = (double **)malloc((size_t)((nrow+NR_END)*sizeof(double *)));
  if (!m) {nrerror("allocation failure 1 in nr_matrix()"); return (double**)NULL;}
  m += NR_END;
  m -= nrl;

  m[nrl]=(double *)malloc((size_t)((nrow*ncol+NR_END)*sizeof(double)));
  if (!m[nrl]) {nrerror("allocation failure 2 in nr_matrix()"); return (double**)NULL;}
  m[nrl] += NR_END;
  m[nrl] -= ncl;

  for (i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

  return m;
}

void free_nr_vector(double *v, int nl, int nh)
{
  free((FREE_ARG)(v+nl-NR_END));
}

void free_nr_matrix(double **m, int nrl, int nrh, int ncl, int nch)
{
  free((FREE_ARG)(m[nrl]+ncl-NR_END));
  free((FREE_ARG)(m+nrl-NR_END));
}

void lubksb(double **a, int n, int *indx, double b[]) {
  int i,ii=0,ip,j;
  double sum;

  for (i=1;i<=n;i++) {
    ip=indx[i];
    sum=b[ip];
    b[ip]=b[i];
    if (ii)
      for (j=ii;j<=i-1;j++) sum -= a[i][j]*b[j];
    else if (sum) ii=i;
    b[i]=sum;
  }
  for (i=n;i>=1;i--) {
    sum=b[i];
    for (j=i+1;j<=n;j++) sum -= a[i][j]*b[j];
    b[i]=sum/a[i][i];
  }
}

int ludcmp(double **a, int n, int *indx, double *d) {
  int i,imax,j,k;
  double big,dum,sum,temp;
  double *vv;

  vv=nr_vector(1,n);
  *d=1.0;
  for (i=1;i<=n;i++) {
    big=0.0;
    for (j=1;j<=n;j++)
      if ((temp=fabs(a[i][j])) > big) big=temp;
    if (big == 0.0) { free_nr_vector(vv,1,n); return -1; }
    vv[i]=1.0/big;
  }
  for (j=1;j<=n;j++) {
    for (i=1;i<j;i++) {
      sum=a[i][j];
      for (k=1;k<i;k++) sum -= a[i][k]*a[k][j];
      a[i][j]=sum;
    }
    big=0.0;
    for (i=j;i<=n;i++) {
      sum=a[i][j];
      for (k=1;k<j;k++)
	sum -= a[i][k]*a[k][j];
      a[i][j]=sum;
      if ( (dum=vv[i]*fabs(sum)) >= big) {
	big=dum;
	imax=i;
      }
    }
    if (j != imax) {
      for (k=1;k<=n;k++) {
	dum=a[imax][k];
	a[imax][k]=a[j][k];
	a[j][k]=dum;
      }
      *d = -(*d);
      vv[imax]=vv[j];
    }
    indx[j]=imax;
    if (a[j][j] == 0.0) a[j][j]=TINY;
    if (j != n) {
      dum=1.0/(a[j][j]);
      for (i=j+1;i<=n;i++) a[i][j] *= dum;
    }
  }
  free_nr_vector(vv,1,n);
  return 0;
}

int svdsolve(double **a, int m, int n, double *b, double *x)
{
  int j;
  double **v, *w, wmax, wmin;
  v = nr_matrix(1,n,1,n);
  w = nr_vector(1,n);
  if ( svdcmp(a,m,n,w,v) < 0 ) {
    free_nr_vector(w,1,n);
    free_nr_matrix(v,1,n,1,n);
    return -1;
  }
  wmax = 0.0;
  for (j=1; j<=n; j++) if (w[j] > wmax) wmax = w[j];
  wmin = wmax*1.0e-6;
  for (j=1; j<=n; j++) if (w[j] < wmin) w[j] = 0.0;
  svbksb(a,w,v,m,n,b,x);
  free_nr_vector(w,1,n);
  free_nr_matrix(v,1,n,1,n);
  return 1;
}

void svbksb(double **u, double w[], double **v, int m, int n, double b[], double x[])
{
  int jj,j,i;
  double s, *tmp;

  tmp = nr_vector(1,n);
  for (j=1;j<=n;j++){
    s=0.0;
    if (w[j]){
      for (i=1;i<=m;i++) s += u[i][j]*b[i];
      s/= w[j];
    }
    tmp[j] = s;
  }
  for (j=1;j<=n;j++){
    s=0.0;
    for (jj=1;jj<=n;jj++) s+=v[j][jj]*tmp[jj];
    x[j]=s;
  }
  free_nr_vector(tmp,1,n);
}

int svdcmp(double **a, int m, int n, double w[], double **v)
{
  double pythag(double a, double b);
  int flag,i,its,j,jj,k,l,nm;
  double anorm,c,f,g,h,s,scale,x,y,z,*rv1;
  
  rv1=nr_vector(1,n);
  g=scale=anorm=0.0;
  for (i=1;i<=n;i++){
    l=i+1;
    rv1[i]=scale*g;
    g=s=scale=0.0;
    if (i<=m){
      for (k=i;k<=m;k++) scale += fabs(a[k][i]);
      if (scale) {
	for (k=i;k<=m;k++){
	  a[k][i] /= scale;
	  s+=a[k][i]*a[k][i];
	}
	f=a[i][i];
	g = -SIGN(sqrt(s), f);
	h=f*g-s;
	a[i][i]=f-g;
	for (j=l;j<=n;j++){
	  for (s=0.0,k=i;k<=m;k++) s+=a[k][i]*a[k][j];
	  f = s/h;
	  for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
	}
	for (k=i;k<=m;k++) a[k][i] *= scale;
      }
    }
    w[i]=scale*g;
    g=s=scale=0.0;
    if (i<=m && i!=n){
      for (k=l;k<=n;k++) scale += fabs(a[i][k]);
      if (scale){
	for (k=l;k<=n;k++){
	  a[i][k] /= scale;
	  s += a[i][k]*a[i][k];
	}
	f = a[i][l];
	g = -SIGN(sqrt(s), f);
	h=f*g-s;
	a[i][l]=f-g;
	for (k=l;k<=n;k++) rv1[k]=a[i][k]/h;
	for (j=l;j<=m;j++){
	  for (s=0.0,k=l;k<=n;k++) s+=a[j][k]*a[i][k];
	  for (k=l;k<=n;k++) a[j][k] += s*rv1[k];
	}
	for (k=l;k<=n;k++) a[i][k] *= scale;
      }
    }
    anorm=FMAX(anorm, (fabs(w[i])+fabs(rv1[i])));
  }
  for (i=n;i>=1;i--){
    if (i<n){
      if (g){
	for (j=l;j<=n;j++) v[j][i]=(a[i][j]/a[i][l])/g;
	for (j=l;j<=n;j++){
	  for (s=0.0,k=l;k<=n;k++) s+=a[i][k]*v[k][j];
	  for (k=l;k<=n;k++) v[k][j] += s*v[k][i];
	}
      }
      for (j=l;j<=n;j++) v[i][j]=v[j][i]=0.0;
    }
    v[i][i]=1.0;
    g=rv1[i];
    l=i;
  }
  for (i=IMIN(m,n);i>=1;i--){
    l=i+1;
    g=w[i];
    for (j=l;j<=n;j++) a[i][j]=0.0;
    if (g){
      g=1.0/g;
      for (j=l;j<=n;j++){
	for (s=0.0,k=l;k<=m;k++) s+= a[k][i]*a[k][j];
	f = (s/a[i][i])*g;
	for (k=i;k<=m;k++) a[k][j]+=f*a[k][i];
      }
      for (j=i;j<=m;j++) a[j][i] *= g;
    }else for (j=i;j<=m;j++) a[j][i]=0.0;
    ++a[i][i];
  }
  for (k=n;k>=1;k--){
    for (its=1;its<=30;its++){
      flag =1;
      for (l=k;l>=1;l--){
	nm=l-1;
	if ((eusfloat_t)(fabs(rv1[l])+anorm) == (eusfloat_t)anorm){
	  flag=0;
	  break;
	}
	if ((eusfloat_t)(fabs(w[nm])+anorm)==(eusfloat_t)anorm) break;
      }
      if (flag){
	c=0.0;
	s=1.0;
	for (i=l;i<=k;i++){
	  f=s*rv1[i];
	  rv1[i]=c*rv1[i];
	  if ((eusfloat_t)(fabs(f)+anorm)==(eusfloat_t)anorm) break;
	  g=w[i];
	  h=pythag(f,g);
	  w[i]=h;
	  h=1.0/h;
	  c=g*h;
	  s = -f*h;
	  for (j=1;j<=m;j++){
	    y=a[j][nm];
	    z=a[j][i];
	    a[j][nm]=y*c+z*s;
	    a[j][i]=z*c-y*s;
	  }
	}
      }
      z=w[k];
      if (l==k){
	if (z<0.0){
	  w[k] = -z;
	  for (j=1;j<=n;j++) v[j][k] = -v[j][k];
	}
	break;
      }
      if (its == 30) {nrerror("no convergence in 30 svdcmp iterations"); return -1;}
      x=w[l];
      nm=k-1;
      y=w[nm];
      g=rv1[nm];
      h=rv1[k];
      f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
      g=pythag(f, 1.0);
      f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
      c=s=1.0;
      for (j=l;j<=nm;j++){
	i=j+1;
	g=rv1[i];
	y=w[i];
	h=s*g;
	g=c*g;
	z=pythag(f,h);
	rv1[j]=z;
	c=f/z;
	s=h/z;
	f=x*c+g*s;
	g=g*c-x*s;
	h=y*s;
	y*=c;
	for (jj=1;jj<=n;jj++){
	  x=v[jj][j];
	  z=v[jj][i];
	  v[jj][j]=x*c+z*s;
	  v[jj][i]=z*c-x*s;
	}
	z=pythag(f,h);
	w[j]=z;
	if (z) {
	  z=1.0/z;
	  c=f*z;
	  s=h*z;
	}
	f=c*g+s*y;
	x=c*y-s*g;
	for (jj=1;jj<=m;jj++){
	  y=a[jj][j];
	  z=a[jj][i];
	  a[jj][j]=y*c+z*s;
	  a[jj][i]=z*c-y*s;
	}
      }
      rv1[l]=0.0;
      rv1[k]=f;
      w[k]=x;
    }
  }
  free_nr_vector(rv1,1,n);
  return 1;
}

void tred2(double **a, int n, double d[], double e[])
{
  int l,k,j,i;
  double scale,hh,h,g,f;
  for (i=n;i>=2;i--) {
    l=i-1;
    h=scale=0.0;
    if (l > 1) {
      for (k=1;k<=l;k++)
	scale += fabs(a[i][k]);
      if (scale == 0.0) // Skip transformation.
	e[i]=a[i][l];
      else {
	for (k=1;k<=l;k++) {
	  a[i][k] /= scale; // Use scaled a's for transformation.
	  h += a[i][k]*a[i][k]; // Form mu in h.
	}
	f=a[i][l];
	g=(f >= 0.0 ? -sqrt(h) : sqrt(h));
	e[i]=scale*g;
	h -= f*g; // Now h is equation (11.2.4).
	a[i][l]=f-g; // Store u in the ith row of a.
	f=0.0;
	for (j=1;j<=l;j++) {
	  /* Next statement can be omitted if eigenvectors not wanted */
	  a[j][i]=a[i][j]/h; // Store u/H in ith column of a.
	  g=0.0; // Form an element of A u in g.
	  for (k=1;k<=j;k++)
	    g += a[j][k]*a[i][k];
	  for (k=j+1;k<=l;k++)
	    g += a[k][j]*a[i][k];
	  e[j]=g/h; // Form element of p in temporarily unused element of e.
	  f += e[j]*a[i][j];
	}
	hh=f/(h+h); // Form K, equation (11.2.11).
	for (j=1;j<=l;j++) { // Form q and store in e overwriting p.
	  f=a[i][j];
	  e[j]=g=e[j]-hh*f;
	  for (k=1;k<=j;k++) // Reduce a, equation (11.2.13).
	    a[j][k] -= (f*e[k]+g*a[i][k]);
	}
      }
    } else
      e[i]=a[i][l];
    d[i]=h;
  }
  /* Next statement can be omitted if eigenvectors not wanted */
  d[1]=0.0;
  e[1]=0.0;
  /* Contents of this loop can be omitted if eigenvectors not
     wanted except for statement d[i]=a[i][i]; */
  for (i=1;i<=n;i++) { // Begin accumulation of transformation matrices.
    l=i-1;
    if (d[i]) { // This block skipped when i=1.
      for (j=1;j<=l;j++) {
	g=0.0;
	for (k=1;k<=l;k++) // Use u and u/H stored in a to form PQ.
	  g += a[i][k]*a[k][j];
	for (k=1;k<=l;k++)
	  a[k][j] -= g*a[k][i];
      }
    }
    d[i]=a[i][i]; // This statement remains.
    a[i][i]=1.0;  // Reset row and column of a to identity  matrix for next iteration
    for (j=1;j<=l;j++) a[j][i]=a[i][j]=0.0;
  }
}

int tqli(double d[], double e[], int n, double **z)
{
  double pythag(double a, double b);
  int m,l,iter,i,k;
  double s,r,p,g,f,dd,c,b;

  for (i=2;i<=n;i++) e[i-1]=e[i]; // Convenient to renumber the elements of e. 
  e[n]=0.0;
  for (l=1;l<=n;l++) {
    iter=0;
    do {
      for (m=l;m<=n-1;m++) { // Look for a single small subdiagonal element to split the matrix.
	dd=fabs(d[m])+fabs(d[m+1]);
	if ((double)(fabs(e[m])+dd) == dd) break;
      }
      if (m != l) {
	if (iter++ == 30) {nrerror("Too many iterations in tqli"); return -1;}
	g=(d[l+1]-d[l])/(2.0*e[l]); // Form shift.
	r=pythag(g,1.0);
	g=d[m]-d[l]+e[l]/(g+SIGN(r,g)); // This is dm . ks.
	s=c=1.0;
	p=0.0;
	for (i=m-1;i>=l;i--) { // A plane rotation as in the original QL, followed by Givens rotations to restore tridiagonal form.
	  f=s*e[i];
	  b=c*e[i];
	  e[i+1]=(r=pythag(f,g));
	  if (r == 0.0) { // Recover from underflow.
	    d[i+1] -= p;
	    e[m]=0.0;
	    break;
	  }
	  s=f/r;
	  c=g/r;
	  g=d[i+1]-p;
	  r=(d[i]-g)*s+2.0*c*b;
	  d[i+1]=g+(p=s*r);
	  g=c*r-b;
	  /* Next loop can be omitted if eigenvectors not wanted*/
	  for (k=1;k<=n;k++) { // Form eigenvectors.
	    f=z[k][i+1];
	    z[k][i+1]=s*z[k][i]+c*f;
	    z[k][i]=c*z[k][i]-s*f;
	  }
	}
	if (r == 0.0 && i >= l) continue;
	d[l] -= p;
	e[l]=g;
	e[m]=0.0;
      }
    } while (m != l);
  }
  return 1;
}

double pythag(double a, double b)
{
  double absa, absb;
  absa=fabs(a);
  absb=fabs(b);
  if (absa > absb) return absa*sqrt(1.0+SQR(absb/absa));
  else return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+SQR(absa/absb)));
}


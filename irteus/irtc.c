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
#include <math.h>
extern pointer ___irtc();
static register_irtc()
{ add_module_initializer("___irtc", ___irtc);}

#define colsize(p) (intval(p->c.ary.dim[1]))
#define rowsize(p) (intval(p->c.ary.dim[0]))
#define ismatrix(p) ((isarray(p) && \
                      p->c.ary.rank==makeint(2) && \
                      elmtypeof(p->c.ary.entity)==ELM_FLOAT))

/* copy from nr.c */
static float sqrarg;
#define SQR(a) ((sqrarg=(a)) == 0.0 ? 0.0 : sqrarg*sqrarg)

static float maxarg1, maxarg2;
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

float *nr_vector(long nl, long nh)
{
  float *v;
  v =(float *)malloc((size_t)((nh-nl+1+NR_END)*sizeof(float)));
  if (!v) {nrerror("allocation failure in nr_vector()"); return (float*)NULL;}
  return v-nl+NR_END;
}

float **nr_matrix(long nrl, long nrh, long ncl, long nch)
{
  long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
  float **m;
  
  m = (float **)malloc((size_t)((nrow+NR_END)*sizeof(float *)));
  if (!m) {nrerror("allocation failure 1 in nr_matrix()"); return (float**)NULL;}
  m += NR_END;
  m -= nrl;

  m[nrl]=(float *)malloc((size_t)((nrow*ncol+NR_END)*sizeof(float)));
  if (!m[nrl]) {nrerror("allocation failure 2 in nr_matrix()"); return (float**)NULL;}
  m[nrl] += NR_END;
  m[nrl] -= ncl;

  for (i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

  return m;
}

void free_nr_vector(float *v, long nl, long nh)
{
  free((FREE_ARG)(v+nl-NR_END));
}

void free_nr_matrix(float **m, long nrl, long nrh, long ncl, long nch)
{
  free((FREE_ARG)(m[nrl]+ncl-NR_END));
  free((FREE_ARG)(m+nrl-NR_END));
}

static copymat(dest,src,size)
pointer dest,src;
register int size;
{ register int i;
  register float_t *rv=dest->c.ary.entity->c.fvec.fv;
  register float_t *mv=src->c.ary.entity->c.fvec.fv;
  size=size*size;
  for (i=0; i<size; i++) rv[i]=mv[i]; }


#define TINY 1.0e-20
void lubksb(float **a, int n, int *indx, float b[]);
int  ludcmp(float **a, int n, int *indx, float *d);

void lubksb(float **a, int n, int *indx, float b[]) {
  int i,ii=0,ip,j;
  float sum;

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

int ludcmp(float **a, int n, int *indx, float *d) {
  int i,imax,j,k;
  float big,dum,sum,temp;
  float *vv;

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

int svdsolve(float **a, int m, int n, float *b, float *x);
void svbksb(float **u, float *w, float **v, int m, int n, float *b, float *x);
int svdcmp(float **a, int m, int n, float *w, float **v);

int svdsolve(float **a, int m, int n, float *b, float *x)
{
  int j;
  float **v, *w, wmax, wmin;
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

void svbksb(float **u, float w[], float **v, int m, int n, float b[], float x[])
{
  int jj,j,i;
  float s, *tmp;

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

int svdcmp(float **a, int m, int n, float w[], float **v)
{
  float pythag(float a, float b);
  int flag,i,its,j,jj,k,l,nm;
  float anorm,c,f,g,h,s,scale,x,y,z,*rv1;
  
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
	if ((float)(fabs(rv1[l])+anorm) == anorm){
	  flag=0;
	  break;
	}
	if ((float)(fabs(w[nm])+anorm)==anorm) break;
      }
      if (flag){
	c=0.0;
	s=1.0;
	for (i=l;i<=k;i++){
	  f=s*rv1[i];
	  rv1[i]=c*rv1[i];
	  if ((float)(fabs(f)+anorm)==anorm) break;
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

float pythag(float a, float b)
{
  float absa, absb;
  absa=fabs(a);
  absb=fabs(b);
  if (absa > absb) return absa*sqrt(1.0+SQR(absb/absa));
  else return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+SQR(absa/absb)));
}

#define SWAP(g,h) {y=(g);(g)=(h);(h)=y;}

/*
 *
 */

pointer SV_SOLVE(ctx,n,argv)
register context *ctx;
int n;
pointer argv[];
/* (SV_SOLVE mat vec &optional ret) */
{ 
  pointer a,b,x;
  float **aa, *bb, *xx;
  int i, j, s;

  ckarg2(2,3);
  a=argv[0];  b=argv[1];
  if (!ismatrix(a)) error(E_NOVECTOR);
  s=colsize(a);
  if (!isfltvector(b)) error(E_NOVECTOR);
  if (s!=vecsize(b)) error(E_VECSIZE);
  if (n==3) {
    x=argv[2];
    if (!isvector(x)) error(E_NOVECTOR);
    if (s!=vecsize(x)) error(E_VECSIZE); }
  else  x=(pointer)makefvector(s);

  aa = nr_matrix(1,s,1,s);
  bb = nr_vector(1,s);
  xx = nr_vector(1,s);
  for (i = 0; i < s; i++){
    for (j = 0; j < s; j++){
      aa[j+1][i+1]=a->c.ary.entity->c.fvec.fv[j*s+i];
    }
  }
  for (i = 0; i < s; i++){
    bb[i+1] = b->c.fvec.fv[i];
  }
  if ( svdsolve(aa, s, s, bb, xx) < 0 ) {
    return NIL;
  }
  
  for (i = 0; i < s; i++){
    x->c.fvec.fv[i] = xx[i+1];
  }

  free_nr_matrix(aa,1,s,1,s);
  free_nr_vector(bb,1,s);
  free_nr_vector(xx,1,s);

  return(x);}

pointer SV_DECOMPOSE(ctx,n,argv)
register context *ctx;
int n;
pointer argv[];
/* (SV_DECOMPOSE mat) */
{ 
  pointer a,ru,rv,rw, rr;
  float **u, **v, *w, y;
  int c, r, i, j, *idx, k, pc=0;;

  ckarg(1);
  a=argv[0];
  if (!ismatrix(a)) error(E_NOVECTOR);
  c = colsize(a);
  r = rowsize(a);

  u = nr_matrix(1,r,1,c);
  v = nr_matrix(1,c,1,c);
  w = nr_vector(1,c);
  for (i = 0; i < c; i++){
    for (j = 0; j < r; j++){
      u[j+1][i+1]=a->c.ary.entity->c.fvec.fv[j*c+i];
    }
  }
  if ( svdcmp(u, r, c, w, v) < 0 ) {
    free_nr_matrix(u,1,r,1,c);
    free_nr_matrix(v,1,c,1,c);
    free_nr_vector(w,1,c);
    return NIL;
  }

  ru = makematrix(ctx,r,c); vpush(ru); pc++;
  rw = makefvector(c);      vpush(rw); pc++;
  rv = makematrix(ctx,c,c); vpush(rv); pc++;

  idx = malloc(sizeof(int)*(c+1));

  for (i = 0; i < c; i++){ idx[i+1] = i+1 ;}
  for (i = 0; i < c; i++) {
    for (j = i+1; j < c; j++) {
      if ( w[i+1] < w[j+1] ) {
	SWAP(w[i+1], w[j+1]);
	k = idx[i+1]; idx[i+1] = idx[j+1]; idx[j+1] = k;
      }
    }
  }
  
  for (i = 0; i < c; i++){
    for (j = 0; j < r; j++){
      ru->c.ary.entity->c.fvec.fv[j*c+i] = u[j+1][idx[i+1]];
    }
  }
  for (i = 0; i < c; i++){
    rw->c.fvec.fv[i] = w[i+1];
  }
  for (i = 0; i < c; i++){
    for (j = 0; j < c; j++){
      rv->c.ary.entity->c.fvec.fv[j*c+i] = v[j+1][idx[i+1]];
    }
  }

  free_nr_matrix(u,1,r,1,c);
  free_nr_matrix(v,1,c,1,c);
  free_nr_vector(w,1,c);

  free(idx);
  
  while(pc-->0) vpop();
  return(cons(ctx,ru,cons(ctx,rw,(cons(ctx,rv,NIL)))));}
  
/*
 *
 */

pointer LU_SOLVE2(ctx,n,argv) /* re-definition */
register context *ctx;
int n;
pointer argv[];
/* (LU-SOLVE mat perm bvector [result]) */
{ pointer a,p,b,x;
  int i, j, s;
  float **aa, *cols;
  int *indx;

  ckarg2(3,4);
  a=argv[0];  p=argv[1]; b=argv[2];
  if (!ismatrix(a)) error(E_NOVECTOR);
  s=colsize(a);
  if (!isvector(p) || !isfltvector(b)) error(E_NOVECTOR);
  if (s!=vecsize(p) || s!=vecsize(b)) error(E_VECSIZE);
  if (n==4) {
    x=argv[3];
    if (!isvector(x)) error(E_NOVECTOR);
    if (s!=vecsize(x)) error(E_VECSIZE); }
  else  x=(pointer)makefvector(s);

  aa = nr_matrix(1,s,1,s);
  cols = nr_vector(1,s);
  indx = malloc(sizeof(int)*(s+1));
  for (i=0; i<s; i++)
    for (j=0; j<s; j++)
      aa[i+1][j+1]=a->c.ary.entity->c.fvec.fv[i*s+j];
  for (i=0; i<s; i++) indx[i+1] = intval(p->c.vec.v[i]);
  for (i=0; i<s; i++) cols[i+1] = b->c.fvec.fv[i];
  lubksb(aa,s,indx,cols);
  for (i=0; i<s; i++) x->c.fvec.fv[i] = cols[i+1];
  
  free_nr_matrix(aa,1,s,1,s);
  free_nr_vector(cols,1,s);
  free(indx);

  return(x);}

pointer LU_DECOMPOSE2(ctx,n,argv) /* re-definition */
register context *ctx;
int n;
pointer argv[];
/* (LU-DECOMPOSE mat [result] [tmp-vector]) */
{ pointer a,result,pv;
  float **aa, d;
  int i, j, s, stat, *indx;

  ckarg2(1,3);
  a=argv[0];
  if (!ismatrix(a)) error(E_NOVECTOR);
  s=colsize(a);
  if (s!=rowsize(a)) error(E_VECSIZE);
  if (n==1) result=a;
  else {
    result=argv[1];
    if (!ismatrix(result)) error(E_NOVECTOR);
    if (s!=colsize(result)) error(E_VECSIZE);
    copymat(result,a,s); 
  }
  if (n==3) {
    pv=argv[2];
    if (!isvector(pv)) error(E_NOVECTOR);
    if (s!=vecsize(pv)) error(E_VECSIZE);
  }else{
    pv=makevector(C_VECTOR,s);
  }

  aa = nr_matrix(1,s,1,s);
  indx = malloc(sizeof(int)*(s+1));

  for (i=0; i<s; i++)
    for (j=0; j<s; j++)
      aa[i+1][j+1]=a->c.ary.entity->c.fvec.fv[i*s+j];
  stat = ludcmp(aa, s, indx, &d);
  for (i=0; i<s; i++) pv->c.vec.v[i]=makeint(indx[i+1]);
  for (i=0; i<s; i++)
    for (j=0; j<s; j++)
      result->c.ary.entity->c.fvec.fv[i*s+j] = aa[i+1][j+1];

  free_nr_matrix(aa,1,s,1,s);
  free(indx);

  if (stat<0) return(NIL);
  else return(pv);}

pointer MATRIX_DETERMINANT(ctx,n,argv)
register context *ctx;
int n;
pointer argv[];
{ pointer a,result;
  numunion nu;
  float **aa, d;
  int i, j, s, stat, *indx;

  ckarg2(1,2);
  a=argv[0];
  if (!ismatrix(a)) error(E_NOVECTOR);
  s=colsize(a);
  if (s!=rowsize(a)) error(E_VECSIZE);
  if (n==1) result=a;
  else {
    result=argv[1];
    if (!ismatrix(result)) error(E_NOVECTOR);
    if (s!=colsize(result)) error(E_VECSIZE);
    copymat(result,a,s); 
  }

  aa = nr_matrix(1,s,1,s);
  indx = malloc(sizeof(int)*(s+1));

  for (i=0; i<s; i++)
    for (j=0; j<s; j++)
      aa[i+1][j+1]=a->c.ary.entity->c.fvec.fv[i*s+j];
  stat = ludcmp(aa, s, indx, &d);
  for (i=0; i<s; i++) d = d*aa[i+1][i+1];
  if ( ((-1 * TINY) <= d) && (d <= TINY) ) d = 0.0;

  free_nr_matrix(aa,1,s,1,s);
  free(indx);

  if (stat<0) return(makeflt(0.0));
  else return(makeflt(d));}

pointer PSEUDO_INVERSE2(ctx,n,argv)
register context *ctx;
int n;
pointer argv[];
{ pointer a,result;
  numunion nu;
  float **u, **v, *w, y;
  int c, r, i, j, k, *idx;

  ckarg2(1,2);
  a=argv[0];
  if (!ismatrix(a)) error(E_NOVECTOR);
  c=colsize(a);
  r=rowsize(a);
  if (n==1) {
    result=makematrix(ctx,r,c); vpush(result);
  }else {
    result=argv[1];
    if (!ismatrix(result)) error(E_NOVECTOR);
    if (r!=colsize(result)||c!=rowsize(result)) error(E_VECSIZE);
  }

  u = nr_matrix(1,r,1,c);
  v = nr_matrix(1,c,1,c);
  w = nr_vector(1,c);
  for (i = 0; i < c; i++){
    for (j = 0; j < r; j++){
      u[j+1][i+1]=a->c.ary.entity->c.fvec.fv[j*c+i];
    }
  }
  if ( svdcmp(u, r, c, w, v) < 0 ) {
    nrerror("svdcmp() returns error"); 
    free_nr_matrix(u,1,r,1,c);
    free_nr_matrix(v,1,c,1,c);
    free_nr_vector(w,1,c);
    return NIL;
  }
  idx = malloc(sizeof(int)*(c+1));

  for (i = 0; i < c; i++){ idx[i+1] = i+1 ;}
  for (i = 0; i < c; i++) {
    for (j = i+1; j < c; j++) {
      if ( w[i+1] < w[j+1] ) {
	SWAP(w[i+1], w[j+1]);
	k = idx[i+1]; idx[i+1] = idx[j+1]; idx[j+1] = k;
      }
    }
  }
  
  // A* = v w ut
  for (i=1;i<=c;i++) {
    if (w[i]>0.0001) w[i] = 1.0/w[i];
  }
  for (i=0;i<c;i++) {
    for (j=0;j<r;j++) {
      result->c.ary.entity->c.fvec.fv[(i)*r+(j)]=0.0;
      for (k=0;k<c;k++) {
	result->c.ary.entity->c.fvec.fv[(i)*r+(j)]+=
	  v[(i+1)][idx[(k+1)]]*w[(k+1)]*u[(j+1)][idx[(k+1)]];
      }
    }
  }

  free_nr_matrix(u,1,r,1,c);
  free_nr_matrix(v,1,c,1,c);
  free_nr_vector(w,1,c);

  free(idx);

  vpop(); // vpush(result)
  return(result);}

/*
 *
 */

int matrix2quaternion(float *c, float *q){
  float q02, q12, q22, q32;
  q02 = (1 + c[0*3+0] + c[1*3+1] + c[2*3+2]) / 4;
  q12 = (1 + c[0*3+0] - c[1*3+1] - c[2*3+2]) / 4;
  q22 = (1 - c[0*3+0] + c[1*3+1] - c[2*3+2]) / 4;
  q32 = (1 - c[0*3+0] - c[1*3+1] + c[2*3+2]) / 4;

  if ( (q02 >= q12) && (q02 >= q22) && (q02 >= q32) ) {
    q[0] = sqrt(q02);
    q[1] = (c[2*3+1] - c[1*3+2]) / (4 * q[0]);
    q[2] = (c[0*3+2] - c[2*3+0]) / (4 * q[0]);
    q[3] = (c[1*3+0] - c[0*3+1]) / (4 * q[0]);
  } else if ( (q12 >= q02) && (q12 >= q22) && (q12 >= q32) ) {
    q[1] = sqrt(q12);
    q[0] = (c[2*3+1] - c[1*3+2]) / (4 * q[1]);
    q[2] = (c[0*3+1] + c[1*3+0]) / (4 * q[1]);
    q[3] = (c[0*3+2] + c[2*3+0]) / (4 * q[1]);
  } else if ( (q22 >= q02) && (q22 >= q12) && (q22 >= q32) ) {
    q[2] = sqrt(q22);
    q[0] = (c[0*3+2] - c[2*3+0]) / (4 * q[2]);
    q[1] = (c[0*3+1] + c[1*3+0]) / (4 * q[2]);
    q[3] = (c[1*3+2] + c[2*3+1]) / (4 * q[2]);
  } else if ( (q32 >= q02) && (q32 >= q12) && (q32 >= q22) ) {
    q[3] = sqrt(q32);
    q[0] = (c[1*3+0] - c[0*3+1]) / (4 * q[3]);
    q[1] = (c[0*3+2] + c[2*3+0]) / (4 * q[3]);
    q[2] = (c[1*3+2] + c[2*3+1]) / (4 * q[3]);
  } else {
    fprintf(stderr, ";; matrix2quaternion\n");
    exit(1);
  }
}

int quaternion2matrix(float *q, float *c){
  float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
  // (+ (* q0 q0) (* q1 q1) (- (* q2 q2)) (- (* q3 q3)))
  c[0*3+0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
  // (* 2 (- (* q1 q2) (* q0 q3)))
  c[0*3+1] = 2 * (q1*q2 - q0*q3);
  // (* 2 (+ (* q1 q3) (* q0 q2)))
  c[0*3+2] = 2 * (q1*q3 + q0*q2);
  // (* 2 (+ (* q1 q2) (* q0 q3)))
  c[1*3+0] = 2 * (q1*q2 + q0*q3);
  // (+ (* q0 q0) (- (* q1 q1)) (* q2 q2) (- (* q3 q3)))
  c[1*3+1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
  // (* 2 (- (* q2 q3) (* q0 q1)))
  c[1*3+2] = 2 * (q2*q3 - q0*q1);
  // (* 2 (- (* q1 q3) (* q0 q2)))
  c[2*3+0] = 2 * (q1*q3 - q0*q2);
  // (* 2 (+ (* q2 q3) (* q0 q1)))
  c[2*3+1] = 2 * (q2*q3 + q0*q1);
  // (+ (* q0 q0) (- (* q1 q1)) (- (* q2 q2)) (* q3 q3))
  c[2*3+2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
}


int quaternion_multiply(float *q1, float *q2, float *q3){
  float q10 = q1[0], q11 = q1[1], q12 = q1[2], q13 = q1[3];
  float q20 = q2[0], q21 = q2[1], q22 = q2[2], q23 = q2[3];
  // (+ (* q10 q20) (- (* q11 q21)) (- (* q12 q22)) (- (* q13 q23)))
  q3[0] = q10*q20 - q11*q21 - q12*q22 - q13*q23;
  // (+ (* q10 q21)    (* q11 q20)     (* q12 q23)  (- (* q13 q22)))
  q3[1] = q10*q21 + q11*q20 + q12*q23 - q13*q22;
  // (+ (* q10 q22) (- (* q11 q23))    (* q12 q20)     (* q13 q21))
  q3[2] = q10*q22 - q11*q23 + q12*q20 + q13*q21;
  // (+ (* q10 q23)    (* q11 q22)  (- (* q12 q21))    (* q13 q20))
  q3[3] = q10*q23 + q11*q22 - q12*q21 + q13*q20;
}

pointer MATTIMES3(ctx,n,argv)
     register context *ctx;
     int n;
     register pointer *argv;
{
  register int i;
  register pointer p,result;
  float *c1,*c2,*c3;
  float q1[4], q2[4], q3[4], q;
  
  ckarg2(2,3);
  c1 = argv[0]->c.ary.entity->c.fvec.fv;
  c2 = argv[1]->c.ary.entity->c.fvec.fv;
  if (n==3) result = argv[2];
  else result = makematrix(ctx,3,3);
  c3 = result->c.ary.entity->c.fvec.fv;

  /*
     (setf c3 (quaternion2matrix 
	       (normalize-vector (quaternion*
				  (matrix2quaternion c1) 
				  (matrix2quaternion c2)))))
  */
  matrix2quaternion(c1, q1);
  matrix2quaternion(c2, q2);
  quaternion_multiply(q1, q2, q3);
  //noromalize-vector
  q = sqrt(q3[0]*q3[0]+q3[1]*q3[1]+q3[2]*q3[2]+q3[3]*q3[3]);
  q3[0] /= q; q3[1] /= q; q3[2] /= q; q3[3] /= q;
  quaternion2matrix(q3, c3);

  return(result);
}

pointer MATPLUS(ctx,n,argv)
     register context *ctx;
     int n;
     register pointer *argv;
{
  register int i, j, row, col;
  register pointer p,result;
  float *c1,*c2,*c3;
  
  ckarg2(2,3);
  if (!ismatrix(argv[0]) || !ismatrix(argv[1])) error(E_NOVECTOR);
  c1 = argv[0]->c.ary.entity->c.fvec.fv;
  c2 = argv[1]->c.ary.entity->c.fvec.fv;
  row = rowsize(argv[0]); col = colsize(argv[0]); 
  if (!((row==rowsize(argv[1])) && (col==colsize(argv[1]))) )
    error(E_VECINDEX);
  if (n==3) {
    if (!((row==rowsize(argv[2])) &&
	  (col==colsize(argv[2])))) error(E_VECINDEX);
    result = argv[2];
  } else {
    result = makematrix(ctx,row,col);
  }
  c3 = result->c.ary.entity->c.fvec.fv;

  for (i = 0; i< row; i++ ) {
    for (j = 0; j< col; j++ ) {
      c3[i*col+j] = c1[i*col+j] + c2[i*col+j];
    }
  }

  return(result);
}

pointer MATMINUS(ctx,n,argv)
     register context *ctx;
     int n;
     register pointer *argv;
{
  register int i, j, row, col;
  register pointer p,result;
  float *c1,*c2,*c3;
  
  ckarg2(2,3);
  if (!ismatrix(argv[0]) || !ismatrix(argv[1])) error(E_NOVECTOR);
  c1 = argv[0]->c.ary.entity->c.fvec.fv;
  c2 = argv[1]->c.ary.entity->c.fvec.fv;
  row = rowsize(argv[0]); col = colsize(argv[0]); 
  if (!((row==rowsize(argv[1])) && (col==colsize(argv[1]))) )
    error(E_VECINDEX);
  if (n==3) {
    if (!((row==rowsize(argv[2])) &&
	  (col==colsize(argv[2])))) error(E_VECINDEX);
    result = argv[2];
  } else {
    result = makematrix(ctx,row,col);
  }
  c3 = result->c.ary.entity->c.fvec.fv;

  for (i = 0; i< row; i++ ) {
    for (j = 0; j< col; j++ ) {
      c3[i*col+j] = c1[i*col+j] - c2[i*col+j];
    }
  }

  return(result);
}

pointer ___irtc(ctx,n,argv, env)
register context *ctx;
int n;
pointer argv[];
pointer env;
{
  pointer mod=argv[0];
  defun(ctx,"ROTM3*",mod,MATTIMES3);
  defun(ctx,"M+",mod,MATPLUS);
  defun(ctx,"M-",mod,MATMINUS);
  defun(ctx,"SV-SOLVE",mod,SV_SOLVE);
  defun(ctx,"SV-DECOMPOSE",mod,SV_DECOMPOSE);
  defun(ctx,"LU-SOLVE2",mod,LU_SOLVE2);
  defun(ctx,"LU-DECOMPOSE2",mod,LU_DECOMPOSE2);
  defun(ctx,"MATRIX-DETERMINANT",mod,MATRIX_DETERMINANT);
  defun(ctx,"PSEUDO-INVERSE2",mod,PSEUDO_INVERSE2);
}

///////////////////////////////////////////////////////////////////////////////
///
/// $Id$
///
/// $Log$
/// Revision 1.6  2009-08-07 11:22:38  k-okada
/// add pseudo-inverse2, use array-dimensions
///
/// Revision 1.5  2009/03/02 12:12:49  k-okada
/// lu-decompose2 accepts LU-DECOMPOSE2 mat [result] [tmp-vector]
///
/// Revision 1.4  2009/02/17 02:04:48  k-okada
/// fix typo on copyright
///
/// Revision 1.3  2008/11/11 11:10:25  k-okada
/// error handling when normalize-vector #f(0 0 0), again
///
/// Revision 1.2  2008/11/11 03:01:18  k-okada
/// error handling when normalize-vector #f(0 0 0) -> 0, add VNORMALIZE in irtc.c remove defun normalize-vector from irtmath.l
///
/// Revision 1.1  2008/09/18 18:11:00  k-okada
/// add irteus
///
///
///

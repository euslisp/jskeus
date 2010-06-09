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
extern pointer ___irtc();
static register_irtc()
{ add_module_initializer("___irtc", ___irtc);}

#define colsize(p) (intval(p->c.ary.dim[1]))
#define rowsize(p) (intval(p->c.ary.dim[0]))
#define ismatrix(p) ((isarray(p) && \
                      p->c.ary.rank==makeint(2) && \
                      elmtypeof(p->c.ary.entity)==ELM_FLOAT))

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
  eusfloat_t **aa, *bb, *xx;
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
  eusfloat_t **u, **v, *w, y;
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
  eusfloat_t **aa, *cols;
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
  eusfloat_t **aa, d;
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
  eusfloat_t **aa, d;
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
  eusfloat_t **u, **v, *w, y;
  int c, r, i, j, k, *idx;

  ckarg2(1,2);
  a=argv[0];
  if (!ismatrix(a)) error(E_NOVECTOR);
  c=colsize(a);
  r=rowsize(a);
  if (n==1) {
    result=makematrix(ctx,c,r); vpush(result);
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

int matrix2quaternion(eusfloat_t *c, eusfloat_t *q){
  eusfloat_t q02, q12, q22, q32;
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
    fprintf(stderr, ";; matrix2quaternion q02=%f,q12=%f,q22=%f,q32=%f\n",
	    q02,q12,q22,q32);
    error(E_USER,(pointer)";; matrix2quaternion\n");
  }
}

int quaternion2matrix(eusfloat_t *q, eusfloat_t *c){
  eusfloat_t q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
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


int quaternion_multiply(eusfloat_t *q1, eusfloat_t *q2, eusfloat_t *q3){
  eusfloat_t q10 = q1[0], q11 = q1[1], q12 = q1[2], q13 = q1[3];
  eusfloat_t q20 = q2[0], q21 = q2[1], q22 = q2[2], q23 = q2[3];
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
  eusfloat_t *c1,*c2,*c3;
  eusfloat_t q1[4], q2[4], q3[4], q;
  
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
  eusfloat_t *c1,*c2,*c3;
  
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
  eusfloat_t *c1,*c2,*c3;
  
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
/// Revision 1.11  2010-03-13 05:49:12  k-okada
/// split nr code from irtc.c to nr.c
///
/// Revision 1.10  2010/02/03 07:36:06  k-okada
/// float_t->eusfloat_t, integer_t->eusinteger_t
///
/// Revision 1.9  2010/02/02 09:50:35  k-okada
/// fix for 64bit eus float->float_t
///
/// Revision 1.8  2009/11/08 04:08:09  inaba
/// change exit to error for continuing debug of NaN in matrix2quaternion of irtc.c
///
/// Revision 1.7  2009/08/13 16:43:37  fujimoto
/// fix pseudo-inverse2
///
/// Revision 1.6  2009/08/07 11:22:38  k-okada
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

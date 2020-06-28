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
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///
/// * Redistributions of source code must retain the above copyright notice,
///   this list of conditions and the following disclaimer.
/// * Redistributions in binary form must reproduce the above copyright notice,
///   this list of conditions and the following disclaimer in the documentation
///   and/or other materials provided with the distribution.
/// * Neither the name of JSK Robotics Laboratory, The University of Tokyo
///   (JSK) nor the names of its contributors may be used to endorse or promote
///   products derived from this software without specific prior written
///   permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
/// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
/// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
/// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
/// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
/// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
/// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
/// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
/// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
/// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///

#ifndef _NR_H_
#define _NR_H_

static double sqrarg;
#define SQR(a) ((sqrarg=(a)) == 0.0 ? 0.0 : sqrarg*sqrarg)

static double maxarg1, maxarg2;
#define FMAX(a,b) (maxarg1=(a),maxarg2=(b),(maxarg1) > (maxarg2) ? (maxarg1) : (maxarg2))

static int iminarg1, iminarg2;
#define IMIN(a,b) (iminarg1=(a),iminarg2=(b),(iminarg1) < (iminarg2) ? (iminarg1) : (iminarg2))

#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define SWAP(g,h) {y=(g);(g)=(h);(h)=y;}

#define NR_END 1
#define FREE_ARG char*

void nrerror(char error_text[]);
double *nr_vector(int nl, int nh);
double **nr_matrix(int nrl, int nrh, int ncl, int nch);
void free_nr_vector(double *v, int nl, int nh);
void free_nr_matrix(double **m, int nrl, int nrh, int ncl, int nch);

#define TINY 1.0e-20
void lubksb(double **a, int n, int *indx, double b[]);
int  ludcmp(double **a, int n, int *indx, double *d);

int svdsolve(double **a, int m, int n, double *b, double *x);
void svbksb(double **u, double *w, double **v, int m, int n, double *b, double *x);
int svdcmp(double **a, int m, int n, double *w, double **v);

void tred2(double **a, int n, double d[], double e[]);
int tqli(double d[], double e[], int n, double **z);

double pythag(double a, double b);

static void copymat(dest,src,size)
pointer dest,src;
register int size;
{ register int i;
  register eusfloat_t *rv=dest->c.ary.entity->c.fvec.fv;
  register eusfloat_t *mv=src->c.ary.entity->c.fvec.fv;
  size=size*size;
  for (i=0; i<size; i++) rv[i]=mv[i]; }

#endif //_NR_H_

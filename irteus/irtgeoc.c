#pragma init (register_irtgeoc)
#include "eus.h"
#include <math.h>

extern pointer ___irtgeoc();
static void register_irtgeoc()
{ add_module_initializer("___irtgeoc", ___irtgeoc);}

//
//
#define colsize(p) (intval(p->c.ary.dim[1]))
#define rowsize(p) (intval(p->c.ary.dim[0]))
#define ismatrix(p) ((isarray(p) && \
                      p->c.ary.rank==makeint(2) && \
                      elmtypeof(p->c.ary.entity)==ELM_FLOAT))

pointer C_COORDS_TRNSFORM_VECTOR(ctx,n,argv)
     register context *ctx;
     int n;
     register pointer argv[];
{
  numunion nu;
  register pointer result;
  eusfloat_t *pos, *rot, *mat, *ret;
  int inversep = 0, fill = 0;
  int srcsize, dstsize;
  int i,j;

  //   0,   1,      2,        3,         4
  // pos, rot, matrix, (matrix), (inverse)

  ckarg2(3,5);
  if ( (!isfltvector(argv[0])) || (!ismatrix(argv[1])) || (!ismatrix(argv[2]))) error(E_TYPEMISMATCH);
  pos = argv[0]->c.fvec.fv;
  rot = argv[1]->c.ary.entity->c.fvec.fv;
  mat = argv[2]->c.ary.entity->c.fvec.fv;
  if (n==5) {
    if(!ismatrix(argv[3])) error(E_TYPEMISMATCH);
    result = argv[3];
    inversep = 1;
  } else if (n==4) {
    if(ismatrix(argv[3])) {
      result = argv[3];
    } else {
      result = makematrix(ctx,rowsize(argv[2]), colsize(argv[2]));
      inversep = 1;
      fill = 1;
    }
  } else { // n == 3
    result = makematrix(ctx,rowsize(argv[2]), colsize(argv[2]));
    fill = 1;
  }
  ret = result->c.ary.entity->c.fvec.fv;

  srcsize = colsize(argv[2]);
  dstsize = colsize(result);
  if ((srcsize < 3) && (dstsize < 3)) error(E_TYPEMISMATCH);
  if (inversep) {
    for(i=0;i<rowsize(result);i++){
      eusfloat_t x = mat[i*srcsize+0] - pos[0],
        y = mat[i*srcsize+1] - pos[1],
        z = mat[i*srcsize+2] - pos[2];
      ret[i*dstsize+0] = rot[0]*x+rot[3]*y+rot[6]*z;
      ret[i*dstsize+1] = rot[1]*x+rot[4]*y+rot[7]*z;
      ret[i*dstsize+2] = rot[2]*x+rot[5]*y+rot[8]*z;
      if(fill) {
        for(j=3;j<dstsize;j++) {
          ret[i*dstsize + j] = mat[i*srcsize + j];
        }
      }
    }
  } else {
    for(i=0;i<rowsize(result);i++){
      eusfloat_t x = mat[i*srcsize+0], y = mat[i*srcsize+1], z = mat[i*srcsize+2];
      ret[i*dstsize+0] = rot[0]*x+rot[1]*y+rot[2]*z+pos[0];
      ret[i*dstsize+1] = rot[3]*x+rot[4]*y+rot[5]*z+pos[1];
      ret[i*dstsize+2] = rot[6]*x+rot[7]*y+rot[8]*z+pos[2];
      if(fill) {
        for(j=3;j<dstsize;j++) {
          ret[i*dstsize + j] = mat[i*srcsize + j];
        }
      }
    }
  }

  return(result);
}

pointer C_MATRIX_ROW(ctx,n,argv)
     register context *ctx;
     int n;
     register pointer argv[];
{
  numunion nu;
  register pointer result;
  register eusfloat_t *mat, *ret;
  register eusinteger_t pos,cols,i;
  int setp = 0;

  //      0,       1,        2,      3
  // matrix, row_num, (vector), (set?)
  // return float-vector
  // if vector is set, elements in vector is over written by row-vector
  // if vector and set? are set, value of vector is copied to matrix-row

  ckarg2(2,4);
  //if ( (!ismatrix(argv[0])) || (!isint(argv[1])) ) error(E_TYPEMISMATCH); //no check
  mat = argv[0]->c.ary.entity->c.fvec.fv;
  cols = colsize(argv[0]);
  pos = cols*intval(argv[1]);

  if (n==4) {
    //if (!(isfltvector(argv[2]))) error(E_TYPEMISMATCH); //no check
    result = argv[2];
    setp=1;
  } else if (n==3) {
    //if (!(isfltvector(argv[2]))) error(E_TYPEMISMATCH); //no check
    result = argv[2];
  } else { // n == 2
    result = makefvector(cols);
  }
  ret = result->c.fvec.fv;

  if(setp) {
    mat += pos;
    for(i=0;i<cols;i++) {
      *mat++ = *ret++;
    }
  } else {
    mat += pos;
    for(i=0;i<cols;i++) {
      *ret++ = *mat++;
    }
  }

  return(result);
}

// utility for using matrix as vector-array
static pointer VECTOR_ARRAY_MEAN(ctx,n,argv)
     register context *ctx;
     int n;
     register pointer *argv;
{
  int i,j,size,dim,pc=0;
  eusfloat_t *m, *fv;
  pointer mat;

  //            0,         1
  // matrix-array, (average)

  ckarg2(1,2);
  if(! ismatrix(argv[0])) {
    error(E_NOVECTOR);
  }
  m = argv[0]->c.ary.entity->c.fvec.fv;
  size = rowsize(argv[0]);
  dim = colsize(argv[0]);

  if(n>1 && isfltvector(argv[1])) {
    mat = argv[1];
  } else {
    mat = makevector(C_FLTVECTOR, dim); vpush(mat); pc++;
  }
  fv = mat->c.fvec.fv;

  for(i=0;i<size;i++) {
    for(j=0;j<dim;j++) {
      fv[j] += *m++;
    }
  }

  for(j=0;j<dim;j++) {
    fv[j] /= size;
  }

  while(pc-->0) vpop();
  return mat;
}

static pointer VECTOR_ARRAY_VARIANCE(ctx,n,argv)
     register context *ctx;
     int n;
     register pointer *argv;
{
  int i,j,size,dim,pc=0, free_ave=1;
  eusfloat_t *m, *fv, *ave;
  pointer mat, amat;

  //            0,          1,         2
  // matrix-array, (variance), (average)

  ckarg2(1,3);
  if(! ismatrix(argv[0])) {
    error(E_NOVECTOR);
  }

  size = rowsize(argv[0]);
  dim = colsize(argv[0]);

  if(n>1 && isfltvector(argv[1])) {
    mat = argv[1];
  } else {
    mat = makevector(C_FLTVECTOR, dim); vpush(mat); pc++;
  }
  fv = mat->c.fvec.fv;

  if(n>2 && isfltvector(argv[2])) {
    amat = argv[2];
    ave =  amat->c.fvec.fv;
    free_ave=0;
  } else {
    ave = (eusfloat_t *) malloc(sizeof(eusfloat_t)*dim);
  }

  for(i=0;i<dim;i++) {
    fv[i] = 0.0;
    ave[i] = 0.0;
  }

  m = argv[0]->c.ary.entity->c.fvec.fv;
  for(i=0;i<size;i++) {
    for(j=0;j<dim;j++) {
      ave[j] += *m++;
    }
  }
  for(j=0;j<dim;j++) {
    ave[j] /= size;
  }

  m = argv[0]->c.ary.entity->c.fvec.fv;
  for(i=0;i<size;i++) {
    for(j=0;j<dim;j++) {
      fv[j] += pow((*m++ - ave[j]), 2);
    }
  }
  for(j=0;j<dim;j++) {
    fv[j] /= size;
  }

  if(free_ave) free(ave);

  while(pc-->0) vpop();
  return mat;
}
static pointer VECTOR_ARRAY_MAX_MIN(ctx,n,argv)
     register context *ctx;
     int n;
     register pointer *argv;
{
  int i,j,size,dim,pc=0;
  eusfloat_t *m, *fvmin, *fvmax;
  pointer fmax, fmin, ret;

  //            0,            1,            2
  // matrix-array, (max-vector), (min-vector)

  ckarg2(1,3);
  if(! ismatrix(argv[0])) {
    error(E_NOVECTOR);
  }
  m = argv[0]->c.ary.entity->c.fvec.fv;
  size = rowsize(argv[0]);
  dim = colsize(argv[0]);

  if(n == 1 && isfltvector(argv[1])) {
    fmax = argv[1];
    fmin = makevector(C_FLTVECTOR, dim); vpush(fmin); pc++;
  } else if(n > 2 && isfltvector(argv[1]) && isfltvector(argv[2])) {
    fmax = argv[1];
    fmin = argv[2];
  } else {
    fmax = makevector(C_FLTVECTOR, dim); vpush(fmax); pc++;
    fmin = makevector(C_FLTVECTOR, dim); vpush(fmin); pc++;
  }
  fvmax = fmax->c.fvec.fv;
  fvmin = fmin->c.fvec.fv;

  // set initial value
  for(j=0;j<dim;j++) {
    eusfloat_t val = *m++;
    fvmax[j] = val;
    fvmin[j] = val;
  }
  for(i=1;i<size;i++) {
    for(j=0;j<dim;j++) {
      eusfloat_t val = *m++;
      if (val > fvmax[j])
        fvmax[j] = val;
      if (val < fvmin[j])
        fvmin[j] = val;
    }
  }

  ret=cons(ctx, fmin, NIL);
  vpush(ret);
  ret=cons(ctx, fmax, ret);
  vpop();
  while(pc-->0) vpop();
  return ret;
}

static pointer FVECTOR_REPLACE(ctx,n,argv)
     register context *ctx;
     int n;
     register pointer *argv;
{
  register int i,count;
  register eusfloat_t *src, *dest;
  eusinteger_t ss,ds,se,de;
  numunion nu;

  //       0,       1,           2,         3,           4,         5
  // dst_vec, src_vec, (start_src), (end_src), (start_dst), (end_dst)

  ckarg2(2,6);
  if (!isfltvector(argv[0])) error(E_NOVECTOR);
  if (!isfltvector(argv[1])) error(E_NOVECTOR);

  dest = argv[0]->c.fvec.fv;
  src  = argv[1]->c.fvec.fv;

  ds = (n==2) ? 0 : ckintval(argv[2]);
  de = (n<=3) ? vecsize(argv[0]) : ckintval(argv[3]);
  ss = (n<=4) ? 0 : ckintval(argv[4]);
  se = (n<=5) ? vecsize(argv[1]) : ckintval(argv[5]);

  count = min(de-ds, se-ss);
  dest += ds;
  src += ss;

  for(i = 0; i < count; i++) {
    *dest++ = *src++;
  }

  return argv[0];
}
//
//
pointer C_ISNAN (ctx,n,argv)
     register context *ctx;
     int n;
     register pointer argv[];
{
  ckarg(1);

  if ( isflt(argv[0]) ) {
    numunion nu;
    eusfloat_t f = fltval(argv[0]);
    if(isnan(f)) return T;
    return NIL;
  } else {
    return NIL;
  }
}

#include "defun.h" // redefine defun for update defun() API
pointer ___irtgeoc(ctx,n, argv, env)
     register context *ctx;int n;pointer *argv;pointer env;
{
  defun(ctx,"C-COORDS-TRANSFORM-VECTOR",argv[0],C_COORDS_TRNSFORM_VECTOR,NULL);
  defun(ctx,"C-MATRIX-ROW",argv[0],C_MATRIX_ROW,NULL);
  defun(ctx,"VECTOR-ARRAY-MEAN",argv[0],VECTOR_ARRAY_MEAN,NULL);
  defun(ctx,"VECTOR-ARRAY-VARIANCE",argv[0],VECTOR_ARRAY_VARIANCE,NULL);
  defun(ctx,"VECTOR-ARRAY-MAX-MIN",argv[0],VECTOR_ARRAY_MAX_MIN,NULL);
  defun(ctx,"FVECTOR-REPLACE", argv[0], FVECTOR_REPLACE,NULL);

  defun(ctx,"C-ISNAN", argv[0], C_ISNAN,NULL);
}

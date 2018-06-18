#pragma init (register_cvoxel)
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <setjmp.h>
#include <errno.h>
#include <sys/time.h>

#include "eus.h"

extern pointer ___ckdtree();
static register_ckdtree()
  { add_module_initializer("___ckdtree", ___ckdtree);}

extern void *kdtree_init (eusfloat_t *points, int nPts, int dim);
extern void kdtree_k_search (eusinteger_t kdTree, eusfloat_t* points, int k, eusfloat_t eps, eusfloat_t *ret[], eusinteger_t *ret_idx);
extern int  kdtree_fr_search (eusinteger_t kdTree, eusfloat_t* points, int k, eusfloat_t r, eusfloat_t eps, eusfloat_t *ret[], eusinteger_t *ret_idx[]);
extern void kdtree_quit (eusinteger_t kdtree);
extern void kdtree_k_search_matrix (eusinteger_t kdTree, eusfloat_t* points, int n, int k, eusfloat_t eps, eusfloat_t* ret[], eusinteger_t* ret_idx, eusfloat_t* dlst);

pointer C_KDTREE_INIT (ctx,n,argv)
     register context *ctx;
     int n;
     register pointer argv[];
{
  eusfloat_t *points;
  int n_points, dim;
  points = argv[0]->c.fvec.fv;

  n_points = ckintval(argv[1]);
  dim = ckintval(argv[2]);

  return makeint((eusinteger_t)(kdtree_init(points, n_points, dim)));
}
// copy from jskgeoc.c
#define colsize(p) (intval(p->c.ary.dim[1]))
#define rowsize(p) (intval(p->c.ary.dim[0]))
#define ismatrix(p) ((isarray(p) && \
                      p->c.ary.rank==makeint(2) && \
                      elmtypeof(p->c.ary.entity)==ELM_FLOAT))
pointer C_KDTREE_K_SEARCH_MATRIX (ctx,n,argv)
     register context *ctx;
     int n;
     register pointer argv[];
{
  numunion nu;
  eusinteger_t kdtree = ckintval(argv[0]);
  eusfloat_t* points;
  int k = ckintval(argv[2]);
  int rownum;
  eusfloat_t eps = ckfltval(argv[3]);
  eusfloat_t *ret;
  eusinteger_t *idx = NULL;
  eusfloat_t *dlst = NULL;
  /* addr mat k eps (index-vector) (distance-vector) */

  ckarg2(4,6);
  if(n > 4) {
    if (isintvector(argv[4])) {
      idx = argv[4]->c.ivec.iv;
    } else {
      idx = (eusinteger_t *)ckintval(argv[4]);
    }
  }
  if(n > 5) {
    if (isfltvector(argv[5])) {
      dlst = argv[5]->c.fvec.fv;
    } else {
      dlst = (eusfloat_t *)ckintval(argv[5]);
    }
  }

  if(!ismatrix(argv[1])) error(E_TYPEMISMATCH);
  //colsize(argv[1]); // have to check dimension
  rownum = rowsize(argv[1]);
  points = argv[1]->c.ary.entity->c.fvec.fv;

  kdtree_k_search_matrix(kdtree, points, rownum, k, eps, &ret, idx, dlst);

  return NIL;
}

pointer C_KDTREE_K_SEARCH_MATRIX_RAW (ctx,n,argv)
     register context *ctx;
     int n;
     register pointer argv[];
{
  numunion nu;
  eusfloat_t *spts, *dpts;
  int i,j,k = ckintval(argv[2]);
  int snum, dnum, col;
  eusfloat_t eps = ckfltval(argv[3]);
  eusfloat_t least_min = 1.0e300;
  eusinteger_t *idx = NULL;
  eusfloat_t *dlst = NULL;

  /* mat mat k eps (index-vector) (distance-vector) */
  ckarg2(4,6);
  if(n > 4) {
    if (isintvector(argv[4])) {
      idx = argv[4]->c.ivec.iv;
    } else {
      idx = (eusinteger_t *)ckintval(argv[4]);
    }
  }
  if(n > 5) {
    if (isfltvector(argv[5])) {
      dlst = argv[5]->c.fvec.fv;
    } else {
      dlst = (eusfloat_t *)ckintval(argv[5]);
    }
  }

  if(!ismatrix(argv[0])) error(E_TYPEMISMATCH);
  if(!ismatrix(argv[1])) error(E_TYPEMISMATCH);

  dnum = rowsize(argv[0]);
  dpts = argv[0]->c.ary.entity->c.fvec.fv;
  col = colsize(argv[0]);

  snum = rowsize(argv[1]);
  spts = argv[1]->c.ary.entity->c.fvec.fv;
  if(col != colsize(argv[1])) error(E_TYPEMISMATCH);

  eusfloat_t *stmp, *dtmp;
  for(i = 0; i < snum; i++) {
    eusfloat_t dmin = 1.0e300;
    eusinteger_t id = -1;
    stmp = spts + i * col;

    for(j = 0; j < dnum; j++) {
      dtmp = dpts + j * col;
      eusfloat_t dist = 0.0, d;

      for(k = 0; k < col; k++) {
        d = stmp[k] - ( *dtmp++ );
        dist += d * d;
      }

      if(dist < dmin) {
        dmin = dist;
        id = j;
      }
    }
    if(idx != NULL) idx[i] = id;
    {
      eusfloat_t tmp = sqrt(dmin);
      if(dlst != NULL) dlst[i] = tmp;
      if(tmp < least_min) least_min = tmp;
    }
  }
  return makeflt(least_min);
}

pointer C_KDTREE_K_SEARCH (ctx,n,argv)
     register context *ctx;
     int n;
     register pointer argv[];
{
  numunion nu; int i, pc = 0;
  eusinteger_t kdtree = ckintval(argv[0]);
  eusfloat_t* point = argv[1]->c.fvec.fv;
  int k = ckintval(argv[2]);
  eusfloat_t eps = ckfltval(argv[3]);
  eusfloat_t *ret;
  eusinteger_t *idx = NULL;
  if(n > 3) {
    if (isintvector(argv[4])) {
      idx = argv[4]->c.ivec.iv;
    } else {
      idx = (eusinteger_t *)ckintval(argv[4]);
    }
  }

  kdtree_k_search(kdtree, point, k, eps, &ret, idx);

  int dim = vecsize(argv[1]);
  pointer mat = makematrix(ctx,k,dim); vpush(mat); pc++;
  for (i = 0; i < k*dim; i++){
    mat->c.ary.entity->c.fvec.fv[i] = ret[i];
  }
  free(ret);
  while(pc-->0) vpop();
  return mat;
}

pointer C_KDTREE_FR_SEARCH (ctx,n,argv)
     register context *ctx;
     int n;
     register pointer argv[];
{
  numunion nu; int i, pc = 0;
  eusinteger_t kdtree = ckintval(argv[0]);
  eusfloat_t* point = argv[1]->c.fvec.fv;
  int k = ckintval(argv[2]);
  eusfloat_t radius = ckfltval(argv[3]);
  eusfloat_t eps = ckfltval(argv[4]);
  eusfloat_t *ret;
  eusinteger_t *idx = NULL;
  eusinteger_t use_idx = 0;
  if(n > 4) {
    use_idx = intval(argv[5]);
  }

  k = kdtree_fr_search(kdtree, point, k, radius, eps, &ret, &idx);

  if ( k <= 0 ) { return NIL;}

  pointer mat;
  if(!use_idx) {
    int dim = vecsize(argv[1]);
    mat = makematrix(ctx,k,dim); vpush(mat); pc++;
    for (i = 0; i < k*dim; i++){
      mat->c.ary.entity->c.fvec.fv[i] = ret[i];
    }
  } else {
    mat = makevector(C_INTVECTOR, k); vpush(mat); pc++;
    for(i = 0; i < k; i++) {
      mat->c.ivec.iv[i] = idx[i];
    }
  }
  free(ret);
  free(idx);

  while(pc-->0) vpop();
  return mat;
}

pointer C_KDTREE_QUIT (ctx,n,argv)
     register context *ctx;
     int n;
     register pointer argv[];
{
  eusinteger_t kdtree = intval(argv[0]);
  kdtree_quit(kdtree);

  return NIL;
}

#if 0
// for speed up calculating distance between point cloud
pointer C_KDTREE_SEARCH_AND_CALC_DISTANCE (ctx,n,argv)
     register context *ctx;
     int n;
     register pointer argv[];
{
  // src->model points, dst->measured points
  //           0        1           2        3           4   5         6        7           8
  // kdtree_addr, src-mat, src-normal, dst-mat, dst-normal, vp(dst), idx-vec, dist-vec, point_step
  numunion nu;
  int i;
  eusinteger_t kdtree = ckintval(argv[0]);
  int k = 1;
  int rownum;
  eusfloat_t eps = 4.0e-7;//single-float-epsilon
  eusfloat_t *ret;
  eusinteger_t *idx = NULL;
  eusfloat_t *dlst = NULL;
  int point_step = 3;
  int pnum = 0;
  eusfloat_t *points;
  eusfloat_t *normals;
  eusfloat_t *spts;
  eusfloat_t *dpts;
  eusfloat_t *dnormals;
  eusfloat_t *vp;

  ckarg2(8,9);
  if (isintvector(argv[6])) {
    idx = argv[6]->c.ivec.iv;
  } else {
    idx = ckintval(argv[6]);
  }
  if (isfltvector(argv[7])) {
    dlst = argv[7]->c.fvec.fv;
  } else {
    dlst = ckintval(argv[7]);
  }
  if(n > 8) {
    point_step = ckintval(argv[8]);
  }

  if(!ismatrix(argv[1])) error(E_TYPEMISMATCH);
  //colsize(argv[1]); // have to check dimension
  rownum = rowsize(argv[1]);
  points = argv[1]->c.ary.entity->c.fvec.fv;
  normals = argv[2]->c.ary.entity->c.fvec.fv;
  vp = argv[5]->c.fvec.fv;
  spts = (eusfloat_t *)malloc(sizeof(eusfloat_t) * rownum * point_step);

  // make new points
  for(i=0;i<rownum;i++) {
    eusfloat_t vx = vp[0] - points[i*point_step + 0];
    eusfloat_t vy = vp[1] - points[i*point_step + 1];
    eusfloat_t vz = vp[2] - points[i*point_step + 2];
    eusfloat_t nx = normals[i*3 + 0];
    eusfloat_t ny = normals[i*3 + 1];
    eusfloat_t nz = normals[i*3 + 2];
    idx[i] = -1;
    if (nx * vx  + ny * vy + nz * vz > 0) {
      memcpy(&(spts[pnum*point_step]), &(points[i*point_step]), sizeof(eusfloat_t)*point_step);
      pnum++;
    }
  }

  kdtree_k_search_matrix(kdtree, spts, pnum, k, eps, &ret, idx, dlst);

  dpts = argv[3]->c.ary.entity->c.fvec.fv;
  dnormals = argv[4]->c.ary.entity->c.fvec.fv;
  // recalc_dist
  for(i=0;i<pnum;i++) {
    int j = idx[i];
    eusfloat_t sx = spts[i*point_step + 0];
    eusfloat_t sy = spts[i*point_step + 1];
    eusfloat_t sz = spts[i*point_step + 2];
    eusfloat_t dx = dpts[j*point_step + 0];
    eusfloat_t dy = dpts[j*point_step + 1];
    eusfloat_t dz = dpts[j*point_step + 2];
    eusfloat_t nx = dnormals[j*3 + 0];
    eusfloat_t ny = dnormals[j*3 + 1];
    eusfloat_t nz = dnormals[j*3 + 2];
    dlst[i] = abs((sx - dx) * nx + (sy - dy) * ny + (sz - dz) * nz) * dlst[i];
  }
  free(spts);
  return makeint(pnum);
}
#endif
// for speed up calculating distance between point cloud
pointer C_KDTREE_SEARCH_AND_CALC_DISTANCE (ctx,n,argv)
     register context *ctx;
     int n;
     register pointer argv[];
{
  // src -> measured points, dest -> model points
  //           0        1        2           3        4         5           6
  // kdtree_addr, src-mat, dst-mat, dst-normal, idx-vec, dist-vec, point_step
  numunion nu;
  int i;
  eusinteger_t kdtree = ckintval(argv[0]);
  int k = 1;
  int rownum;
  eusfloat_t eps = 4.0e-7;//single-float-epsilon
  eusfloat_t *ret;
  eusinteger_t *idx = NULL;
  eusfloat_t *dlst = NULL;
  int point_step = 3;

  eusfloat_t *points;
  eusfloat_t *normals;
  eusfloat_t *spts;
  eusfloat_t *dpts;
  eusfloat_t *dnormals;
  eusfloat_t *vp;

  ckarg2(6,7);
  if (isintvector(argv[4])) {
    idx = argv[4]->c.ivec.iv;
  } else {
    idx = (eusinteger_t *)ckintval(argv[4]);
  }
  if (isfltvector(argv[5])) {
    dlst = argv[5]->c.fvec.fv;
  } else {
    dlst = (eusfloat_t *)ckintval(argv[5]);
  }
  if(n > 6) {
    point_step = ckintval(argv[6]);
  }

  if(!ismatrix(argv[1])) error(E_TYPEMISMATCH);
  if(!ismatrix(argv[2])) error(E_TYPEMISMATCH);
  if(!ismatrix(argv[3])) error(E_TYPEMISMATCH);
  //colsize(argv[1]); // have to check dimension
  rownum = rowsize(argv[1]);
  points = argv[1]->c.ary.entity->c.fvec.fv;

  kdtree_k_search_matrix(kdtree, points, rownum, k, eps, &ret, idx, dlst);

  dpts = argv[2]->c.ary.entity->c.fvec.fv;
  dnormals = argv[3]->c.ary.entity->c.fvec.fv;
  // recalc_dist
  for(i=0;i<rownum;i++) {
    int j = idx[i];
    eusfloat_t sx = points[i*point_step + 0];
    eusfloat_t sy = points[i*point_step + 1];
    eusfloat_t sz = points[i*point_step + 2];
    eusfloat_t dx = dpts[j*point_step + 0];
    eusfloat_t dy = dpts[j*point_step + 1];
    eusfloat_t dz = dpts[j*point_step + 2];
    eusfloat_t nx = dnormals[j*3 + 0];
    eusfloat_t ny = dnormals[j*3 + 1];
    eusfloat_t nz = dnormals[j*3 + 2];

    dlst[i] *= fabs(((sx - dx) * nx) + ((sy - dy) * ny) + ((sz - dz) * nz));
  }

  return NIL;
}

pointer ___ckdtree(ctx, n, argv, env)
register context *ctx; int n; pointer *argv; pointer env;
{
 defun(ctx,"C-KDTREE-INIT", argv[0],C_KDTREE_INIT);
 defun(ctx,"C-KDTREE-K-SEARCH", argv[0],C_KDTREE_K_SEARCH);
 defun(ctx,"C-KDTREE-K-SEARCH-MATRIX", argv[0],C_KDTREE_K_SEARCH_MATRIX);
 defun(ctx,"C-KDTREE-K-SEARCH-MATRIX-RAW", argv[0],C_KDTREE_K_SEARCH_MATRIX_RAW);
 defun(ctx,"C-KDTREE-FR-SEARCH", argv[0],C_KDTREE_FR_SEARCH);
 defun(ctx,"C-KDTREE-QUIT", argv[0], C_KDTREE_QUIT);
 defun(ctx,"C-KDTREE-K-DISTANCE-POINTS", argv[0], C_KDTREE_SEARCH_AND_CALC_DISTANCE);
}

/*
 * $Log$
 * Revision 1.1  2010-02-16 11:09:35  k-okada
 * add
 *
 * Revision 1.3  2010/02/16 11:08:11  k-okada
 * add log
 *
 * Revision 1.2  2010/02/16 11:05:36  k-okada
 * add log
 *
 * 1.2 add log
 * 1.1 add kdtree.c, CANN.C
 */

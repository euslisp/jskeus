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
// author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

#pragma init (register_irtglc)

#include "eus.h"

extern pointer ___irtglc();
static void register_irtglc()
{ add_module_initializer("___irtglc", ___irtglc);}

#define colsize(p) (intval(p->c.ary.dim[1]))
#define rowsize(p) (intval(p->c.ary.dim[0]))
#define isimage(p) ((isarray(p) && \
                     p->c.ary.rank==makeint(2) && \
                     (elmtypeof(p->c.ary.entity)==ELM_CHAR || \
                      elmtypeof(p->c.ary.entity)==ELM_BYTE)))

pointer CTRANSPOSE_IMAGE_ROWS(ctx,n,argv)
     register context *ctx;
     register int n;
     register pointer argv[];
/* (height step src-entity &optional dst-entity) */
{
  int h,step,y;
  char *src, *dst, *buf;
  
  ckarg2(3,4);
  h=ckintval(argv[0]); step=ckintval(argv[1]);
  if (isstring(argv[2])) src=(char *)argv[2]->c.str.chars;
  else src=(char*)bigintval(argv[2]);

  if (n==3) {
    buf=malloc(sizeof(char)*step);
    for(y = 0; y < h/2; ++y) {
      memcpy(buf, src + (h-y-1)*step, step);
      memcpy(src + (h-y-1)*step, src + y*step, step);
      memcpy(src + y*step, buf, step);
    }
    free(buf);
    return((pointer)src);
  } else {
    if (isstring(argv[3])) dst=(char *)argv[3]->c.str.chars;
    else dst=(char*)bigintval(argv[3]);
    for(y = 0; y < h; ++y) {
      memcpy(dst + y*step, src + (h-y-1)*step, step);
    }
    return((pointer)dst);}}

#include "defun.h" // redefine defun for update defun() API
pointer ___irtglc(ctx,n,argv,env)
     register context *ctx;
     int n;
     pointer argv[];
     pointer env;
{
  pointer mod=argv[0];
  defun(ctx,"CTRANSPOSE-IMAGE-ROWS",mod,CTRANSPOSE_IMAGE_ROWS,NULL);
}


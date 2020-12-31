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


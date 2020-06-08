///////////////////////////////////////////////////////////////////////////////
///
/// $Id$
///
/// Copyright (c) 1987- JSK, The University of Tokyo.  All Rights Reserved.
///
/// This software is a collection of EusLisp code for robot applications,
/// which has been developed by the JSK Laboratory for the IRT project.
/// For more information on EusLisp and it's application to the robotics,
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

#pragma init (register_eusbullet)

#include "eus.h"

extern pointer ___eusbullet();
static void register_eusbullet()
{ add_module_initializer("___eusbullet", ___eusbullet);}

extern eusinteger_t C_BT_MakeSphereModel(eusfloat_t r);
extern eusinteger_t C_BT_MakeBoxModel(eusfloat_t xsize, eusfloat_t ysize, eusfloat_t zsize);
extern eusinteger_t C_BT_MakeCylinderModel(eusfloat_t radius, eusfloat_t height);
extern eusinteger_t C_BT_MakeCapsuleModel(eusfloat_t radius, eusfloat_t height);
extern eusinteger_t C_BT_MakeMeshModel(eusfloat_t *verticesPoints, eusinteger_t numVertices);
extern eusinteger_t C_BT_CalcCollisionDistance(eusinteger_t modelAddrA, eusinteger_t modelAddrB,
                                                   eusfloat_t *posA, eusfloat_t *quatA, eusfloat_t *posB, eusfloat_t *quatB,
                                                   eusfloat_t *dist, eusfloat_t *dir, eusfloat_t *pA, eusfloat_t *pB);
extern eusinteger_t C_BT_SetMargin(eusinteger_t modelAddr, eusfloat_t margin);

pointer BTMAKESPHEREMODEL(register context *ctx, int n, register pointer *argv)
{
  numunion nu;
  eusfloat_t r = ckfltval(argv[0]);
  eusinteger_t addr = C_BT_MakeSphereModel(r);
  return makeint(addr);
}

pointer BTMAKEBOXMODEL(register context *ctx, int n, register pointer *argv)
{
  numunion nu;
  eusfloat_t xsize = ckfltval(argv[0]);
  eusfloat_t ysize = ckfltval(argv[1]);
  eusfloat_t zsize = ckfltval(argv[2]);
  eusinteger_t addr = C_BT_MakeBoxModel(xsize, ysize, zsize);
  return makeint(addr);
}

pointer BTMAKECYLINDERMODEL(register context *ctx, int n, register pointer *argv)
{
  numunion nu;
  eusfloat_t radius = ckfltval(argv[0]);
  eusfloat_t height = ckfltval(argv[1]);
  eusinteger_t addr = C_BT_MakeCylinderModel(radius, height);
  return makeint(addr);
}

pointer BTMAKECAPSULEMODEL(register context *ctx, int n, register pointer *argv)
{
  numunion nu;
  eusfloat_t radius = ckfltval(argv[0]);
  eusfloat_t height = ckfltval(argv[1]);
  eusinteger_t addr = C_BT_MakeCapsuleModel(radius, height);
  return makeint(addr);
}

pointer BTMAKEMESHMODEL(register context *ctx, int n, register pointer *argv)
{
  numunion nu;
  eusfloat_t *verticesPoints = argv[0]->c.fvec.fv;
  eusinteger_t numVertices = ckintval(argv[1]);
  eusinteger_t addr = C_BT_MakeMeshModel(verticesPoints, numVertices);
  return makeint(addr);
}

pointer BTCALCCOLLISIONDISTANCE(register context *ctx, int n, register pointer *argv)
{
  eusinteger_t modelAddrA = intval(argv[0]);
  eusinteger_t modelAddrB = intval(argv[1]);
  eusfloat_t *posA = argv[2]->c.fvec.fv;
  eusfloat_t *quatA = argv[3]->c.fvec.fv;
  eusfloat_t *posB = argv[4]->c.fvec.fv;
  eusfloat_t *quatB = argv[5]->c.fvec.fv;
  eusfloat_t *dist = argv[6]->c.fvec.fv;
  eusfloat_t *dir = argv[7]->c.fvec.fv;
  eusfloat_t *pA = argv[8]->c.fvec.fv;
  eusfloat_t *pB = argv[9]->c.fvec.fv;
  eusinteger_t ret = C_BT_CalcCollisionDistance(modelAddrA, modelAddrB, posA, quatA, posB, quatB, dist, dir, pA, pB);
  return makeint(ret);
}

pointer BTSETMARGIN(register context *ctx, int n, register pointer *argv)
{
  numunion nu;
  eusinteger_t modelAddr = intval(argv[0]);
  eusfloat_t margin = ckfltval(argv[1]);
  eusinteger_t ret =  C_BT_SetMargin(modelAddr, margin);
  return makeint(ret);
}

#include "defun.h" // redefine defun for update defun() API
pointer ___eusbullet(register context *ctx, int n, register pointer *argv)
{
    pointer mod=argv[0];
    defun(ctx, "BTMAKESPHEREMODEL", mod, BTMAKESPHEREMODEL, NULL);
    defun(ctx, "BTMAKEBOXMODEL", mod, BTMAKEBOXMODEL, NULL);
    defun(ctx, "BTMAKECYLINDERMODEL", mod, BTMAKECYLINDERMODEL, NULL);
    defun(ctx, "BTMAKECAPSULEMODEL", mod, BTMAKECAPSULEMODEL, NULL);
    defun(ctx, "BTMAKEMESHMODEL", mod, BTMAKEMESHMODEL, NULL);
    defun(ctx, "BTCALCCOLLISIONDISTANCE", mod, BTCALCCOLLISIONDISTANCE, NULL);
    defun(ctx, "BTSETMARGIN", mod, BTSETMARGIN, NULL);

    pointer ALGO_BULLET;
    ALGO_BULLET=defconst(ctx,"*COLLISION-ALGORITHM-BULLET*",ALGO_BULLET,userpkg);
#if HAVE_BULLET
    ALGO_BULLET->c.sym.speval=defkeyword(ctx,"BULLET");
#else
    ALGO_BULLET->c.sym.speval=NIL;
#endif
}

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

// for eus.h
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <setjmp.h>
#include <errno.h>
#include <sstream>

#define class   eus_class
#define throw   eus_throw
#define export  eus_export
#define vector  eus_vector
#define string  eus_string
#include <eus.h> // include eus.h just for eusfloat_t ...
#undef class
#undef throw
#undef export
#undef vector
#undef string

#if HAVE_BULLET
#include <btBulletCollisionCommon.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkCollisionDescription.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btComputeGjkEpaPenetration.h>
#include <LinearMath/btGeometryUtil.h>
#endif

#if HAVE_BULLET
#define CALL_WITH_BULLET_CHECK(X) X
#else
#define CALL_WITH_BULLET_CHECK(X) fprintf(stderr, "jskeus is compiled without bullet, so you can not use function %s. Please install bullet >= 2.83.\n", __PRETTY_FUNCTION__); return -1;
#endif


#if HAVE_BULLET

// The btDistanceInfo class is copied from https://github.com/bulletphysics/bullet3/blob/master/test/collision/btDistanceInfo.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2014 Erwin Coumans http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.  In
no event will the authors be held liable for any damages arising from the use of
this software.  Permission is granted to anyone to use this software for any
purpose, including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim
that you wrote the original software. If you use this software in a product, an
acknowledgment in the product documentation would be appreciated but is not
required.

2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.

3. This notice may not be removed or altered from any source distribution.
*/
struct btDistanceInfo
{
  btVector3 m_pointOnA;
  btVector3 m_pointOnB;
  btVector3 m_normalBtoA;
  btScalar m_distance;
};

// The ConvexWrap class is copied from https://github.com/bulletphysics/bullet3/blob/master/test/collision/main.cpp
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2014 Erwin Coumans http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.  In
no event will the authors be held liable for any damages arising from the use of
this software.  Permission is granted to anyone to use this software for any
purpose, including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim
that you wrote the original software. If you use this software in a product, an
acknowledgment in the product documentation would be appreciated but is not
required.

2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.

3. This notice may not be removed or altered from any source distribution.
*/
///Original author: Erwin Coumans, October 2014
///Initial version of this low-level GJK/EPA/MPR convex-convex collision test
///You can provide your own support function in combination with the template functions
///See btComputeGjkEpaSphereSphereCollision below for an example
///Todo: the test needs proper coverage and using a convex hull point cloud
///Also the GJK, EPA and MPR should be improved, both quality and performance
struct ConvexWrap
{
  btConvexShape* m_convex;
  btTransform m_worldTrans;
  inline btScalar getMargin() const
  {
    return m_convex->getMargin();
  }
  inline btVector3 getObjectCenterInWorld() const
  {
    return m_worldTrans.getOrigin();
  }
  inline const btTransform& getWorldTransform() const
  {
    return m_worldTrans;
  }
  inline btVector3 getLocalSupportWithMargin(const btVector3& dir) const
  {
    return m_convex->localGetSupportingVertex(dir);
  }
  inline btVector3 getLocalSupportWithoutMargin(const btVector3& dir) const
  {
    return m_convex->localGetSupportingVertexWithoutMargin(dir);
  }
};

long BT_MakeSphereModel(double radius)
{
  return (long)(new btSphereShape(radius));
};

long BT_MakeBoxModel(double xsize, double ysize, double zsize)
{
  return (long)(new btBoxShape(0.5*btVector3(xsize, ysize, zsize)));
};

long BT_MakeCylinderModel(double radius, double height)
{
  return (long)(new btCylinderShapeZ(btVector3(radius, radius, 0.5*height)));
};

long BT_MakeCapsuleModel(double radius, double height)
{
  return (long)(new btCapsuleShapeZ(radius, 0.5*height));
};

long BT_MakeMeshModel(double *verticesPoints, long numVertices)
{
  btConvexHullShape* pshape = new btConvexHullShape();
  if (numVertices == 0) {
    fprintf(stderr, "BT_MakeMeshModel with numVertices == 0\n");
  }
#define SHRINK_FOR_MARGIN false
  if (SHRINK_FOR_MARGIN) {
    // Shrink vertices for default margin CONVEX_DISTANCE_MARGIN,
    // which should be nonzero positive for fast computation of penetration distance.
    // ref: https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=2358#p9411
    // But sometimes, this doesn't work well (vertices become empty), so currently disabled.
    btAlignedObjectArray<btVector3> vertices;
    for (int i = 0; i < 3 * numVertices; i += 3) {
      vertices.push_back(btVector3(verticesPoints[i], verticesPoints[i+1], verticesPoints[i+2]));
    }
    btAlignedObjectArray<btVector3> planes;
    btGeometryUtil::getPlaneEquationsFromVertices(vertices, planes);
    int sz = planes.size();
    for (int i = 0 ; i < sz ; i++) {
      planes[i][3] += CONVEX_DISTANCE_MARGIN;
    }
    vertices.clear();
    btGeometryUtil::getVerticesFromPlaneEquations(planes, vertices);
    sz = vertices.size();
    for (int i = 0 ; i < sz ; i++) {
      pshape->addPoint(vertices[i]);
    }
  } else {
    for (int i = 0; i < 3 * numVertices; i += 3) {
      pshape->addPoint(btVector3(verticesPoints[i], verticesPoints[i+1], verticesPoints[i+2]));
    }
  }
  return (long)pshape;
};

long BT_CalcCollisionDistance(long modelAddrA, long modelAddrB,
                           double *posA, double *quatA, double *posB, double *quatB,
                           double *dist, double *dir, double *pA, double *pB)
{
  ConvexWrap a, b;
  a.m_convex = ((btConvexShape *)modelAddrA);
  a.m_worldTrans.setOrigin(btVector3(posA[0], posA[1], posA[2]));
  a.m_worldTrans.setRotation(btQuaternion(quatA[1], quatA[2], quatA[3], quatA[0])); // w is first element in euslisp
  b.m_convex = ((btConvexShape *)modelAddrB);
  b.m_worldTrans.setOrigin(btVector3(posB[0], posB[1], posB[2]));
  b.m_worldTrans.setRotation(btQuaternion(quatB[1], quatB[2], quatB[3], quatB[0])); // w is first element in euslisp
  // The origin of euslisp cylinder model is located on bottom, so local translation of half height is necessary
  if(btCylinderShapeZ* cly = dynamic_cast<btCylinderShapeZ*>(a.m_convex)) {
    btVector3 heightOffset(btVector3(0, 0, cly->getHalfExtentsWithMargin().getZ()));
    a.m_worldTrans.setOrigin(a.m_worldTrans.getOrigin() + a.m_worldTrans.getBasis() * heightOffset);
  }
  if(btCylinderShapeZ* cly = dynamic_cast<btCylinderShapeZ*>(b.m_convex)) {
    btVector3 heightOffset(btVector3(0, 0, cly->getHalfExtentsWithMargin().getZ()));
    b.m_worldTrans.setOrigin(b.m_worldTrans.getOrigin() + b.m_worldTrans.getBasis() * heightOffset);
  }

  btGjkCollisionDescription colDesc;
  btVoronoiSimplexSolver simplexSolver;
  btDistanceInfo distInfo;
  int res = -1;
  simplexSolver.reset();
  res = btComputeGjkEpaPenetration(a, b, colDesc, simplexSolver, &distInfo);

  // The result of btComputeGjkEpaPenetration is offseted by CONVEX_DISTANCE_MARGIN.
  // Although the offset is considered internally in primitive shapes, not considered in convex hull shape.
  // So, the result is modified manually.
  if(dynamic_cast<btConvexHullShape*>((btConvexShape *)modelAddrA)) {
    distInfo.m_distance += CONVEX_DISTANCE_MARGIN;
    distInfo.m_pointOnA += CONVEX_DISTANCE_MARGIN * distInfo.m_normalBtoA;
  }
  if(dynamic_cast<btConvexHullShape*>((btConvexShape *)modelAddrB)) {
    distInfo.m_distance += CONVEX_DISTANCE_MARGIN;
    distInfo.m_pointOnB += - CONVEX_DISTANCE_MARGIN * distInfo.m_normalBtoA;
  }

  *dist = distInfo.m_distance;
  for (int i = 0; i < 3; i++) {
    dir[i] = distInfo.m_normalBtoA[i];
    pA[i] = distInfo.m_pointOnA[i];
    pB[i] = distInfo.m_pointOnB[i];
  }

  return res;
};

long BT_SetMargin(long modelAddr, double margin)
{
  // shape are shrinked for CONVEX_DISTANCE_MARGIN, so CONVEX_DISTANCE_MARGIN is added to margin
  ((btConvexShape *)modelAddr)->setMargin(CONVEX_DISTANCE_MARGIN+margin);
  return 0;
};
#endif

extern "C"
{
  eusinteger_t C_BT_MakeSphereModel(eusfloat_t r)
  {
    CALL_WITH_BULLET_CHECK(return BT_MakeSphereModel(r);)
  }

  eusinteger_t C_BT_MakeBoxModel(eusfloat_t xsize, eusfloat_t ysize, eusfloat_t zsize)
  {
    CALL_WITH_BULLET_CHECK(return BT_MakeBoxModel(xsize, ysize, zsize);)
  }

  eusinteger_t C_BT_MakeCylinderModel(eusfloat_t radius, eusfloat_t height)
  {
    CALL_WITH_BULLET_CHECK(return BT_MakeCylinderModel(radius, height);)
  }

  eusinteger_t C_BT_MakeCapsuleModel(eusfloat_t radius, eusfloat_t height)
  {
    CALL_WITH_BULLET_CHECK(return BT_MakeCapsuleModel(radius, height);)
  }

  eusinteger_t C_BT_MakeMeshModel(eusfloat_t *verticesPoints, eusinteger_t numVertices)
  {
    #if HAVE_BULLET
    double _verticesPoints[3*numVertices];
    for (int i = 0; i < 3 * numVertices; i++ ) { _verticesPoints[i] = verticesPoints[i]; }
    #endif
    CALL_WITH_BULLET_CHECK(return BT_MakeMeshModel(_verticesPoints, numVertices);)
  }

  eusinteger_t C_BT_CalcCollisionDistance(eusinteger_t modelAddrA, eusinteger_t modelAddrB,
                                         eusfloat_t *posA, eusfloat_t *quatA, eusfloat_t *posB, eusfloat_t *quatB,
                                         eusfloat_t *dist, eusfloat_t *dir, eusfloat_t *pA, eusfloat_t *pB)
  {
    #if HAVE_BULLET
    double _posA[3], _quatA[4], _posB[3], _quatB[4];
    double _dist[1], _dir[3], _pA[3], _pB[3];
    eusinteger_t ret;
    for (int i = 0; i < 3; i++ ) {_posA[i] = posA[i]; _posB[i] = posB[i]; }
    for (int i = 0; i < 4; i++ ) {_quatA[i] = quatA[i]; _quatB[i] = quatB[i]; }
    _dist[0] = dist[0];
    for (int i = 0; i < 3; i++ ) {_dir[i] = dir[i]; _pA[i] = pA[i]; _pB[i] = pB[i];}
    ret = BT_CalcCollisionDistance(modelAddrA, modelAddrB,
                                _posA, _quatA, _posB, _quatB,
                                _dist, _dir, _pA, _pB);
    dist[0] = _dist[0];
    for (int i = 0; i < 3; i++ ) {dir[i] = _dir[i]; pA[i] = _pA[i]; pB[i] = _pB[i];}
    #endif
    CALL_WITH_BULLET_CHECK(return ret;)
  }

  eusinteger_t C_BT_SetMargin(eusinteger_t modelAddr, eusfloat_t margin)
  {
    CALL_WITH_BULLET_CHECK(return BT_SetMargin(modelAddr, margin);)
  }
}

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

#include "PQP/src/PQP.h"
#include "PQP/src/MatVec.h"

#include <stdio.h>
extern "C"{
    
PQP_Model *PQP_MakeModel()
{
    return new PQP_Model();
}

void PQP_DeleteModel(PQP_Model *m)
{
    delete(m);
}

int PQP_BeginModel(PQP_Model* m)
{
    return m->BeginModel();
}

int PQP_EndModel(PQP_Model* m)
{
    return m->EndModel();
}

int PQP_AddTri(PQP_Model *m, double p1[], double p2[], double p3[],
		   int flag)
{
    return m->AddTri(p1, p2, p3, flag);
}

int PQP_Collide(double R1[3][3], double T1[3], PQP_Model *PQP_Model1,
		double R2[3][3], double T2[3], PQP_Model *PQP_Model2,
		int flag)
{
    PQP_CollideResult cres;
    PQP_Collide(&cres, R1, T1, PQP_Model1, R2, T2, PQP_Model2, flag);
    return cres.NumPairs();
}

double PQP_Distance(double R1[3][3], double T1[3], PQP_Model *PQP_Model1,
		    double R2[3][3], double T2[3], PQP_Model *PQP_Model2,
		    PQP_REAL *P1, PQP_REAL *P2, int qsize)
{
    PQP_REAL V1[3], V2[3];
    PQP_DistanceResult dres;
    PQP_Distance(&dres, R1, T1, PQP_Model1, R2, T2, PQP_Model2, 0.0, 0.0, qsize);
    VcV(V1, dres.P1());
    VcV(V2, dres.P2());
    MxVpV(P1, R1, V1, T1);
    MxVpV(P2, R2, V2, T2);
    return (double)(dres.Distance());
}
}

///////////////////////////////////////////////////////////////////////////////
///
/// $Id$
///
/// $Log$
/// Revision 1.2  2009-02-17 02:04:48  k-okada
/// fix typo on copyright
///
/// Revision 1.1  2008/09/18 18:11:00  k-okada
/// add irteus
///
///
///

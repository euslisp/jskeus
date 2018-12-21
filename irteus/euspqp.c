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

#pragma init (register_euspqp)

#include "eus.h"

extern pointer ___euspqp();
static void register_euspqp()
{ add_module_initializer("___euspqp", ___euspqp);}

extern eusinteger_t PQP_MakeModel();
extern void PQP_DeleteModel(eusinteger_t m);
extern int PQP_BeginModel(eusinteger_t m);
extern int PQP_EndModel(eusinteger_t m);
extern int PQP_AddTri(eusinteger_t m, double p1[], double p2[], double p3[],
                      int flag);
extern int PQP_Collide(double R1[3][3], double T1[3], eusinteger_t PQP_Model1,
                       double R2[3][3], double T2[3], eusinteger_t PQP_Model2,
                       int flag);
extern double PQP_Distance(double R1[3][3], double T1[3], eusinteger_t* PQP_Model1,
                           double R2[3][3], double T2[3], eusinteger_t* PQP_Model2,
                           double *P1, double *P2, int qsize);

pointer PQPMAKEMODEL(register context *ctx, int n, register pointer *argv)
{
    eusinteger_t addr = PQP_MakeModel();
    return makeint(addr);
}

pointer PQPDELETEMODEL(register context *ctx, int n, register pointer *argv)
{
    eusinteger_t m = intval(argv[0]);
    PQP_DeleteModel(m);
    return(NIL);
}

pointer PQPBEGINMODEL(register context *ctx, int n, register pointer *argv)
{
    eusinteger_t m = intval(argv[0]);
    PQP_BeginModel(m);
    return(NIL);
}

pointer PQPENDMODEL(register context *ctx, int n, register pointer *argv)
{
    eusinteger_t m = intval(argv[0]);
    PQP_EndModel(m);
    return(NIL);
}

pointer PQPADDTRI(register context *ctx, int n, register pointer *argv)
{
    eusinteger_t m = intval(argv[0]);
    eusfloat_t *fv1=argv[1]->c.fvec.fv;
    eusfloat_t *fv2=argv[2]->c.fvec.fv;
    eusfloat_t *fv3=argv[3]->c.fvec.fv;
    eusinteger_t id = ((eusinteger_t)argv[4])>>2;
    double dv1[3], dv2[3], dv3[3];
    dv1[0] = fv1[0]; dv1[1] = fv1[1]; dv1[2] = fv1[2]; 
    dv2[0] = fv2[0]; dv2[1] = fv2[1]; dv2[2] = fv2[2]; 
    dv3[0] = fv3[0]; dv3[1] = fv3[1]; dv3[2] = fv3[2];
    #if 0
    printf("#f(%f %f %f), #f(%f %f %f), #f(%f %f %f), %d\n",
	   dv1[0], dv1[1], dv1[2], 
	   dv2[0], dv2[1], dv2[2], 
	   dv3[0], dv3[1], dv3[2], id);
    #endif
    PQP_AddTri(m, dv1, dv2, dv3, id);
    return(NIL);
}

pointer PQPCOLLIDE(register context *ctx, int n, register pointer *argv)
{
    eusfloat_t *fr1=argv[0]->c.ary.entity->c.fvec.fv;
    eusfloat_t *ft1=argv[1]->c.fvec.fv;
    eusinteger_t m1=intval(argv[2]);
    eusfloat_t *fr2=argv[3]->c.ary.entity->c.fvec.fv;
    eusfloat_t *ft2=argv[4]->c.fvec.fv;
    eusinteger_t m2=intval(argv[5]);
    eusinteger_t flag=((eusinteger_t)argv[6])>>2;
    double dr1[3][3], dr2[3][3], dt1[3], dt2[3];

    /*printf("addr1=0x%x, addr2=0x%x, flag=%d\n", m1>>2, m2>>2, flag);*/
    
    dr1[0][0] = fr1[0]; dr1[0][1] = fr1[1]; dr1[0][2] = fr1[2]; 
    dr1[1][0] = fr1[3]; dr1[1][1] = fr1[4]; dr1[1][2] = fr1[5]; 
    dr1[2][0] = fr1[6]; dr1[2][1] = fr1[7]; dr1[2][2] = fr1[8];
#if 0
    printf("%f %f %f\n %f %f %f\n %f %f %f\n\n",
	   dr1[0][0], dr1[0][1], dr1[0][2], 
	   dr1[1][0], dr1[1][1], dr1[1][2], 
	   dr1[2][0], dr1[2][1], dr1[2][2]);
#endif    
    dt1[0] = ft1[0]; dt1[1] = ft1[1]; dt1[2] = ft1[2];

    /*printf("%f %f %f\n", dt1[0], dt1[1], dt1[2]);*/
    
    dr2[0][0] = fr2[0]; dr2[0][1] = fr2[1]; dr2[0][2] = fr2[2]; 
    dr2[1][0] = fr2[3]; dr2[1][1] = fr2[4]; dr2[1][2] = fr2[5]; 
    dr2[2][0] = fr2[6]; dr2[2][1] = fr2[7]; dr2[2][2] = fr2[8]; 

    dt2[0] = ft2[0]; dt2[1] = ft2[1]; dt2[2] = ft2[2]; 

    return makeint((eusfloat_t)(PQP_Collide(dr1, dt1, m1, dr2, dt2, m2, flag)));
}

pointer PQPDISTANCE(register context *ctx, int n, register pointer *argv)
{
    eusfloat_t *fr1=argv[0]->c.ary.entity->c.fvec.fv;
    eusfloat_t *ft1=argv[1]->c.fvec.fv;
    eusinteger_t m1=intval(argv[2]);
    eusfloat_t *fr2=argv[3]->c.ary.entity->c.fvec.fv;
    eusfloat_t *ft2=argv[4]->c.fvec.fv;
    eusinteger_t m2=intval(argv[5]);
    eusfloat_t *p1=argv[6]->c.fvec.fv;
    eusfloat_t *p2=argv[7]->c.fvec.fv;
    double dr1[3][3], dr2[3][3], dt1[3], dt2[3];
    int qsize = 2;
    numunion nu;

    if (n == 9) qsize = intval(argv[8]);

    /*printf("addr1=0x%x, addr2=0x%x, flag=%d\n", m1>>2, m2>>2, flag);*/
    
    dr1[0][0] = fr1[0]; dr1[0][1] = fr1[1]; dr1[0][2] = fr1[2]; 
    dr1[1][0] = fr1[3]; dr1[1][1] = fr1[4]; dr1[1][2] = fr1[5]; 
    dr1[2][0] = fr1[6]; dr1[2][1] = fr1[7]; dr1[2][2] = fr1[8];
#if 0
    printf("%f %f %f\n %f %f %f\n %f %f %f\n\n",
	   dr1[0][0], dr1[0][1], dr1[0][2], 
	   dr1[1][0], dr1[1][1], dr1[1][2], 
	   dr1[2][0], dr1[2][1], dr1[2][2]);
#endif    
    dt1[0] = ft1[0]; dt1[1] = ft1[1]; dt1[2] = ft1[2];

    /*printf("%f %f %f\n", dt1[0], dt1[1], dt1[2]);*/
    
    dr2[0][0] = fr2[0]; dr2[0][1] = fr2[1]; dr2[0][2] = fr2[2]; 
    dr2[1][0] = fr2[3]; dr2[1][1] = fr2[4]; dr2[1][2] = fr2[5]; 
    dr2[2][0] = fr2[6]; dr2[2][1] = fr2[7]; dr2[2][2] = fr2[8]; 

    dt2[0] = ft2[0]; dt2[1] = ft2[1]; dt2[2] = ft2[2]; 

    double distance, P1[3], P2[3];
    distance = PQP_Distance(dr1, dt1, (void *)(m1), dr2, dt2, (void *)(m2), P1, P2, qsize);
    p1[0]=P1[0]; p1[1]=P1[1]; p1[2]=P1[2];
    p2[0]=P2[0]; p2[1]=P2[1]; p2[2]=P2[2];
    return (makeflt(distance));
}

#include "defun.h" // redefine defun for update defun() API
pointer ___euspqp(register context *ctx, int n, register pointer *argv)
{
    pointer mod=argv[0];
    defun(ctx, "PQPMAKEMODEL", mod, PQPMAKEMODEL, NULL);
    defun(ctx, "PQPDELETEMODEL", mod, PQPDELETEMODEL, NULL);
    defun(ctx, "PQPBEGINMODEL", mod, PQPBEGINMODEL, NULL);
    defun(ctx, "PQPENDMODEL", mod, PQPENDMODEL, NULL);
    defun(ctx, "PQPADDTRI", mod, PQPADDTRI, NULL);
    defun(ctx, "PQPCOLLIDE", mod, PQPCOLLIDE, NULL);
    defun(ctx, "PQPDISTANCE", mod, PQPDISTANCE, NULL);
}

///////////////////////////////////////////////////////////////////////////////
///
/// $Id$
///
/// $Log$
/// Revision 1.6  2010-02-03 07:36:06  k-okada
/// float_t->eusfloat_t, integer_t->eusinteger_t
///
/// Revision 1.5  2010/02/02 09:50:35  k-okada
/// fix for 64bit eus float->float_t
///
/// Revision 1.4  2009/08/25 01:26:12  k-okada
/// fix pointer generated by makeint must use intval not >>2
///
/// Revision 1.3  2009/02/17 02:04:48  k-okada
/// fix typo on copyright
///
/// Revision 1.2  2009/02/17 02:03:54  k-okada
/// fix file encoding
///
/// Revision 1.1  2008/09/18 18:11:00  k-okada
/// add irteus
///
///
///

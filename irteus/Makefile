###############################################################################
###
### $Id$
###
### Copyright (c) 1987- JSK, The University of Tokyo.  All Rights Reserved.
###
### This software is a collection of EusLisp code for robot applications,
### which has been developed by the JSK Laboratory for the IRT project.
### For more information on EusLisp and its application to the robotics,
### please refer to the following papers.
###
### Toshihiro Matsui
### Multithread object-oriented language euslisp for parallel and
###  asynchronous programming in robotics
### Workshop on Concurrent Object-based Systems,
###  IEEE 6th Symposium on Parallel and Distributed Processing, 1994
###
### Permission to use this software for educational, research
### and non-profit purposes, without fee, and without a written
### agreement is hereby granted to all researchers working on
### the IRT project at the University of Tokyo, provided that the
### above copyright notice remains intact.  
###

include ./Makefile.$(ARCHDIR)

# directory where this package install object file to
IRTEUSDIR=$(EUSDIR)
INSTALLBINDIR=$(IRTEUSDIR)/$(ARCHDIR)/bin
INSTALLOBJDIR=$(IRTEUSDIR)/$(ARCHDIR)/obj
INSTALLLIBDIR=$(IRTEUSDIR)/$(ARCHDIR)/lib

# common
WFLAGS= #-Wall

BINDIR=$(EUSDIR)/$(ARCHDIR)/bin
OBJDIR=$(EUSDIR)/$(ARCHDIR)/obj
LIBDIR=$(EUSDIR)/$(ARCHDIR)/lib

PQPLIBDIR=PQP/$(ARCHDIR)
PQPLIB=-L$(PQPLIBDIR) -lPQP-static

MODULES.L=irt_modules.l
EUSLIB_MODULES.L=$(addprefix $(EUSDIR)/lib/,$(MODULES.L))

IRTEUS=irtmath irtutil irtgraph pgsql time
IRTEUSG=irtgeo pqp irtscene irtmodel irtsensor irtdyna irtrobot irtbvh irtcollada irtpointcloud
IRTEUSX=irtx
IRTEUSIMG=irtimage eusjpeg png
IRTEUSGL=irtgl irtglrgb irtviewer
NR=nr

LIBIRTEUS=$(INSTALLLIBDIR)/$(LPFX)irteus.$(LSFX)
LIBIRTEUSG=$(INSTALLLIBDIR)/$(LPFX)irteusg.$(LSFX)
LIBIRTEUSX=$(INSTALLLIBDIR)/$(LPFX)irteusx.$(LSFX)
LIBIRTEUSIMG=$(INSTALLLIBDIR)/$(LPFX)irteusimg.$(LSFX)
LIBIRTEUSGL=$(INSTALLLIBDIR)/$(LPFX)irteusgl.$(LSFX)
LIBNR=$(INSTALLLIBDIR)/$(LPFX)nr.$(LSFX)

IRTEUS_OSFX=$(addsuffix .$(OSFX),$(IRTEUS))
IRTEUS_LSFX=$(addsuffix .$(LSFX),$(IRTEUS))
IRTEUSOBJS=$(addprefix $(INSTALLOBJDIR)/,$(IRTEUS_OSFX))
IRTEUSDLLS=$(addprefix $(INSTALLOBJDIR)/,$(IRTEUS_LSFX))
IRTEUS_C=$(addprefix $(INSTALLOBJDIR)/,$(addsuffix .c,$(IRTEUS)))
IRTEUS_H=$(addprefix $(INSTALLOBJDIR)/,$(addsuffix .h,$(IRTEUS)))
IRTEUS_L=$(addsuffix .l,$(IRTEUS))
IRTEUS_L:=$(subst pgsql.l,$(EUSDIR)/lib/llib/pgsql.l,$(IRTEUS_L))
IRTEUS_L:=$(subst time.l,$(EUSDIR)/lib/llib/time.l,$(IRTEUS_L))

IRTEUSG_OSFX=$(addsuffix .$(OSFX),$(IRTEUSG))
IRTEUSG_LSFX=$(addsuffix .$(LSFX),$(IRTEUSG))
IRTEUSGOBJS=$(addprefix $(INSTALLOBJDIR)/,$(IRTEUSG_OSFX))
IRTEUSGDLLS=$(addprefix $(INSTALLOBJDIR)/,$(IRTEUSG_LSFX))
IRTEUSG_C=$(addprefix $(INSTALLOBJDIR)/,$(addsuffix .c,$(IRTEUSG)))
IRTEUSG_H=$(addprefix $(INSTALLOBJDIR)/,$(addsuffix .h,$(IRTEUSG)))
IRTEUSG_L=$(addsuffix .l,$(IRTEUSG))

IRTEUSX_OSFX=$(addsuffix .$(OSFX),$(IRTEUSX))
IRTEUSX_LSFX=$(addsuffix .$(LSFX),$(IRTEUSX))
IRTEUSXOBJS=$(addprefix $(INSTALLOBJDIR)/,$(IRTEUSX_OSFX))
IRTEUSXDLLS=$(addprefix $(INSTALLOBJDIR)/,$(IRTEUSX_LSFX))
IRTEUSX_C=$(addprefix $(INSTALLOBJDIR)/,$(addsuffix .c,$(IRTEUSX)))
IRTEUSX_H=$(addprefix $(INSTALLOBJDIR)/,$(addsuffix .h,$(IRTEUSX)))
IRTEUSX_L=$(addsuffix .l,$(IRTEUSX))

IRTEUSIMG_OSFX=$(addsuffix .$(OSFX),$(IRTEUSIMG))
IRTEUSIMG_LSFX=$(addsuffix .$(LSFX),$(IRTEUSIMG))
IRTEUSIMGOBJS=$(addprefix $(INSTALLOBJDIR)/,$(IRTEUSIMG_OSFX))
IRTEUSIMGDLLS=$(addprefix $(INSTALLOBJDIR)/,$(IRTEUSIMG_LSFX))
IRTEUSIMG_C=$(addprefix $(INSTALLOBJDIR)/,$(addsuffix .c,$(IRTEUSIMG)))
IRTEUSIMG_H=$(addprefix $(INSTALLOBJDIR)/,$(addsuffix .h,$(IRTEUSIMG)))
IRTEUSIMG_L=$(patsubst eusjpeg.l,$(EUSDIR)/lisp/image/jpeg/eusjpeg.l,$(addsuffix .l,$(IRTEUSIMG)))

IRTEUSGL_OSFX=$(addsuffix .$(OSFX),$(IRTEUSGL))
IRTEUSGL_LSFX=$(addsuffix .$(LSFX),$(IRTEUSGL))
IRTEUSGLOBJS=$(addprefix $(INSTALLOBJDIR)/,$(IRTEUSGL_OSFX))
IRTEUSGLDLLS=$(addprefix $(INSTALLOBJDIR)/,$(IRTEUSGL_LSFX))
IRTEUSGL_C=$(addprefix $(INSTALLOBJDIR)/,$(addsuffix .c,$(IRTEUSGL)))
IRTEUSGL_H=$(addprefix $(INSTALLOBJDIR)/,$(addsuffix .h,$(IRTEUSGL)))
IRTEUSGL_L=$(addsuffix .l,$(IRTEUSGL))

IRTCOBJECTS=$(INSTALLOBJDIR)/irtc.$(OSFX) $(INSTALLOBJDIR)/irtgeoc.$(OSFX)
IRTGCOBJECTS=$(INSTALLOBJDIR)/CPQP.$(OSFX) $(INSTALLOBJDIR)/euspqp.$(OSFX)
IRTIMGCOBJECTS=$(INSTALLOBJDIR)/euspng.$(OSFX)
IRTGLCOBJECTS=$(INSTALLOBJDIR)/irtglc.$(OSFX)
NROBJECTS=$(INSTALLOBJDIR)/nr.$(OSFX)

all: defun.h $(LIBNR) $(LIBIRTEUS) $(LIBIRTEUSG) $(LIBIRTEUSX) $(LIBIRTEUSIMG) $(LIBIRTEUSGL) make-link install-irtext.l

.PHONY: defun.h
defun.h:
	echo '#include "eus.h"\n#undef defun\npointer TEST();\nvoid test(void) {register context *ctx; pointer mod; defun(ctx,"TEST",mod,TEST,NULL);}' > test.c
	echo "// redefine defun for update defun() API () https://github.com/euslisp/EusLisp/pull/116" > defun.h
	echo "#undef defun" >> defun.h
	$(CC) $(CFLAGS) $(WFLAGS) -c test.c $(OBJOPT) test.o || echo "#define defun(a, b, c, d, e) defun(a, b, c, d) // for EusLisp < 9.24" >> defun.h
	cat defun.h

$(LIBIRTEUS): $(IRTEUSOBJS) $(IRTCOBJECTS) $(LIBNR)
	$(LD) $(SOFLAGS) $(OUTOPT)$(LIBIRTEUS) $(IRTEUSOBJS) \
		$(IRTCOBJECTS) $(IMPLIB)

$(LIBIRTEUSG): $(IRTEUSGOBJS) $(IRTGCOBJECTS) $(LIBNR) PQP/$(ARCHDIR)/libPQP-static.a
	$(LD) $(SOFLAGS) $(OUTOPT)$(LIBIRTEUSG) $(IRTEUSGOBJS) \
		$(IRTGCOBJECTS) $(IMPLIB) $(PQPLIB)

$(LIBIRTEUSX): $(IRTEUSXOBJS) $(LIBNR)
	$(LD) $(SOFLAGS) $(OUTOPT)$(LIBIRTEUSX) $(IRTEUSXOBJS) $(IMPLIB)

$(LIBIRTEUSIMG): $(INSTALLLIBDIR)/jpegmemcd.$(LSFX) $(IRTEUSIMGOBJS) $(IRTIMGCOBJECTS) $(LIBNR)
	$(LD) $(SOFLAGS) $(OUTOPT)$(LIBIRTEUSIMG) $(IRTEUSIMGOBJS) $(IRTIMGCOBJECTS) $(IMPLIB) $(IMPLIBIMG)

$(LIBIRTEUSGL): $(IRTEUSGLOBJS) $(IRTGLCOBJECTS) $(LIBNR)
	$(LD) $(SOFLAGS) $(OUTOPT)$(LIBIRTEUSGL) $(IRTEUSGLOBJS) $(IRTGLCOBJECTS) $(IMPLIB)

$(IRTEUSOBJS): $(INSTALLOBJDIR)/compile_irt.log
$(IRTEUSOBJS) $(INSTALLOBJDIR)/compile_irt.log: $(IRTEUS_L)
	$(BINDIR)/eus2$(ESFX) "(setq *objdir* \"$(INSTALLOBJDIR)/\")" < ./compile_irt.l > $(INSTALLOBJDIR)/compile_irt.log
	-rm -f $(IRTEUS_C) $(IRTEUS_H)

$(IRTEUSGOBJS): $(INSTALLOBJDIR)/compile_irtg.log
$(IRTEUSGOBJS) $(INSTALLOBJDIR)/compile_irtg.log: $(IRTEUSG_L)
	$(BINDIR)/eusgl$(ESFX) "(setq *objdir* \"$(INSTALLOBJDIR)/\")" < ./compile_irtg.l > $(INSTALLOBJDIR)/compile_irtg.log
	-rm -f $(IRTEUSG_C) $(IRTEUSG_H)

$(IRTEUSXOBJS): $(INSTALLOBJDIR)/compile_irtx.log
$(IRTEUSXOBJS) $(INSTALLOBJDIR)/compile_irtx.log: $(IRTEUSX_L)
	$(BINDIR)/eusx$(ESFX) "(setq *objdir* \"$(INSTALLOBJDIR)/\")" < ./compile_irtx.l > $(INSTALLOBJDIR)/compile_irtx.log
	-rm -f $(IRTEUSX_C) $(IRTEUSX_H)

$(IRTEUSIMGOBJS): $(INSTALLOBJDIR)/compile_irtimg.log
$(IRTEUSIMGOBJS) $(INSTALLOBJDIR)/compile_irtimg.log: $(IRTEUSIMG_L)
	$(BINDIR)/eusx$(ESFX) "(setq *objdir* \"$(INSTALLOBJDIR)/\")" < ./compile_irtimg.l > $(INSTALLOBJDIR)/compile_irtimg.log
	-rm -f $(IRTEUSIMG_C) $(IRTEUSIMG_H)

$(IRTEUSGLOBJS): $(INSTALLOBJDIR)/compile_irtgl.log
$(IRTEUSGLOBJS) $(INSTALLOBJDIR)/compile_irtgl.log: $(IRTEUSGL_L)
	$(BINDIR)/eusgl$(ESFX) "(setq *objdir* \"$(INSTALLOBJDIR)/\")" < ./compile_irtgl.l > $(INSTALLOBJDIR)/compile_irtgl.log
	-rm -f $(IRTEUSGL_C) $(IRTEUSGL_H)

PQP/$(ARCHDIR)/libPQP-static.a:
	make -C PQP

$(LIBNR): $(NROBJECTS)
	$(LD) $(SOFLAGS) $(OUTOPT)$(LIBNR) $(NROBJECTS)

BINS=$(BINDIR)/irteus$(ESFX) $(BINDIR)/irteusgl$(ESFX)
make-link: $(BINS) $(EUSDIR)/irteus
$(BINS):
	(cd $(INSTALLBINDIR); ln -sf eus$(ESFX) irteus$(ESFX))
	(cd $(INSTALLBINDIR); ln -sf eusgl$(ESFX) irteusgl$(ESFX))
$(EUSDIR)/irteus:
	(cd $(IRTEUSDIR); ln -sf ../irteus irteus)

install-irtext.l: $(IRTEUSDIR)/lib/irtext.l
$(IRTEUSDIR)/lib/irtext.l: irtext.l
	-cp -f irtext.l $(IRTEUSDIR)/lib

clean:
	-rm -f $(IRTCOBJECTS) $(IRTGCOBJECTS)
	-rm -f $(LIBIRTEUS) $(LIBIRTEUSG) $(LIBIRTEUSGL) $(LIBIRTEUSX) $(LIBIRTEUSIMG)
	-rm -f $(IRTEUSOBJS) $(IRTEUSDLLS) $(IRTEUS_C) $(IRTEUS_H)
	-rm -f $(IRTEUSGOBJS) $(IRTEUSGDLLS) $(IRTEUSG_C) $(IRTEUSG_H)
	-rm -f $(IRTEUSXOBJS) $(IRTEUSXDLLS) $(IRTEUSX_C) $(IRTEUSX_H)
	-rm -f $(IRTEUSIMGOBJS) $(IRTEUSIMGDLLS) $(IRTEUSIMG_C) $(IRTEUSIMG_H)
	-rm -f $(IRTEUSGLOBJS) $(IRTEUSGLDLLS) $(IRTEUSGL_C) $(IRTEUSGL_H)
	-rm -f $(INSTALLLIBDIR)/jpegmemcd.$(LSFX) $(LIBNR) $(NROBJECTS)
	-rm -f *~ *.stackdump
	-rm -f $(EUSDIR)/irteus
	-rm -f $(INSTALLOBJDIR)/compile_*.log
	chmod a-x Makefile* *.l *.c
	(cd PQP;make clean)
	(cd $(EUSDIR)/lisp/image/jpeg/; make clean)
	-rm -f $(EUSDIR)/lib/llib/pgsql.c $(EUSDIR)/lib/llib/pgsql.h
	-rm -f $(EUSDIR)/lib/llib/time.c $(EUSDIR)/lib/llib/time.h

$(INSTALLOBJDIR)/irtmath.$(OSFX): irtmath.l
$(INSTALLOBJDIR)/irtutil.$(OSFX): irtutil.l
$(INSTALLOBJDIR)/irtgraph.$(OSFX): irtgraph.l
$(INSTALLOBJDIR)/irtgeo.$(OSFX): irtgeo.l
$(INSTALLOBJDIR)/irtscene.$(OSFX): irtscene.l
$(INSTALLOBJDIR)/irtmodel.$(OSFX): irtmodel.l
$(INSTALLOBJDIR)/irtdyna.$(OSFX): irtdyna.l
$(INSTALLOBJDIR)/irtcollada.$(OSFX): irtcollada.l
$(INSTALLOBJDIR)/irtsensor.$(OSFX): irtsensor.l
$(INSTALLOBJDIR)/irtpointcloud.$(OSFX): irtpointcloud.l
$(INSTALLOBJDIR)/irtrobot.$(OSFX): irtrobot.l
$(INSTALLOBJDIR)/irtbvh.$(OSFX): irtbvh.l
$(INSTALLOBJDIR)/irtx.$(OSFX): irtx.l
$(INSTALLOBJDIR)/irtgl.$(OSFX): irtgl.l
$(INSTALLOBJDIR)/irtglrgb.$(OSFX): irtglrgb.l
$(INSTALLOBJDIR)/irtviewer.$(OSFX): irtviewer.l
$(INSTALLOBJDIR)/irtimage.$(OSFX): irtimage.l
$(INSTALLOBJDIR)/pqp.$(OSFX): pqp.l
$(INSTALLOBJDIR)/png.$(OSFX): png.l
$(INSTALLOBJDIR)/pgsql.$(OSFX): $(EUSDIR)/lib/llib/pgsql.l
$(INSTALLOBJDIR)/time.$(OSFX): $(EUSDIR)/lib/llib/time.l
$(INSTALLOBJDIR)/eusjpeg.$(OSFX): $(EUSDIR)/lisp/image/jpeg/eusjpeg.l

$(INSTALLOBJDIR)/irtc.$(OSFX): irtc.c $(filter-out $(INSTALLOBJDIR)/irtc.$(OSFX),$(IRTEUSOBJS) $(IRTEUSGOBJS) $(IRTGCOBJECTS) PQP/$(ARCHDIR)/libPQP-static.a $(IRTEUSXOBJS) $(INSTALLLIBDIR)/jpegmemcd.$(LSFX) $(IRTEUSIMGOBJS) $(IRTIMGCOBJECTS) $(IRTEUSGLOBJS) $(IRTGLCOBJECTS)) defun.h
	$(CC) $(CFLAGS) $(WFLAGS) -c irtc.c $(OBJOPT)$(INSTALLOBJDIR)/irtc.$(OSFX)
$(INSTALLOBJDIR)/irtglc.$(OSFX): irtglc.c $(filter-out $(INSTALLOBJDIR)/irtglc.$(OSFX),$(IRTEUSOBJS) $(IRTEUSGOBJS) $(IRTGCOBJECTS) PQP/$(ARCHDIR)/libPQP-static.a $(IRTEUSXOBJS) $(INSTALLLIBDIR)/jpegmemcd.$(LSFX) $(IRTEUSIMGOBJS) $(IRTIMGCOBJECTS) $(IRTEUSGLOBJS) $(IRTGLCOBJECTS)) defun.h
	$(CC) $(CFLAGS) $(WFLAGS) -c irtglc.c $(OBJOPT)$(INSTALLOBJDIR)/irtglc.$(OSFX)
$(INSTALLOBJDIR)/irtgeoc.$(OSFX): irtgeoc.c $(filter-out $(INSTALLOBJDIR)/irtgeoc.$(OSFX), $(IRTEUSOBJS) $(IRTEUSGOBJS) $(IRTGCOBJECTS) PQP/$(ARCHDIR)/libPQP-static.a $(IRTEUSXOBJS) $(INSTALLLIBDIR)/jpegmemcd.$(LSFX) $(IRTEUSIMGOBJS) $(IRTIMGCOBJECTS) $(IRTEUSGLOBJS) $(IRTGLCOBJECTS)) defun.h
	$(CC) $(CFLAGS) $(WFLAGS) -c irtgeoc.c $(OBJOPT)$(INSTALLOBJDIR)/irtgeoc.$(OSFX)
$(INSTALLOBJDIR)/CPQP.$(OSFX): CPQP.C defun.h
	$(CXX) $(CXXFLAGS) -c CPQP.C $(OBJOPT)$(INSTALLOBJDIR)/CPQP.$(OSFX)
$(INSTALLOBJDIR)/euspqp.$(OSFX): euspqp.c defun.h
	$(CC) $(CFLAGS) $(WFLAGS) -c euspqp.c $(OBJOPT)$(INSTALLOBJDIR)/euspqp.$(OSFX)
$(INSTALLOBJDIR)/euspng.$(OSFX): euspng.c defun.h
	$(CC) $(CFLAGS) $(WFLAGS) -c euspng.c $(OBJOPT)$(INSTALLOBJDIR)/euspng.$(OSFX)
$(INSTALLOBJDIR)/nr.$(OSFX): nr.c defun.h
	$(CC) $(CFLAGS) $(WFLAGS) -c nr.c $(OBJOPT)$(INSTALLOBJDIR)/nr.$(OSFX)

$(INSTALLLIBDIR)/jpegmemcd.$(LSFX): $(EUSDIR)/lisp/image/jpeg/jpegmemcd.c
	(cd $(EUSDIR)/lisp/image/jpeg/;make ARCH=$(ARCHDIR) LIBDIR=$(INSTALLLIBDIR) OBJDIR=$(INSTALLOBJDIR))

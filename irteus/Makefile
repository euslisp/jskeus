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

# common
BINDIR=$(EUSDIR)/$(ARCHDIR)/bin
OBJDIR=$(EUSDIR)/$(ARCHDIR)/obj
LIBDIR=$(EUSDIR)/$(ARCHDIR)/lib

PQPLIBDIR=PQP/$(ARCHDIR)
PQPLIB=-L$(PQPLIBDIR) -lPQP-static

MODULES.L=irt_modules.l
EUSLIB_MODULES.L=$(addprefix $(EUSDIR)/lib/,$(MODULES.L))

IRTEUS=irtmath irtutil pgsql
IRTEUSG=irtgeo pqp irtmodel irtdyna irtrobot
IRTEUSX=irtx
IRTEUSIMG=irtimage eusjpeg png
IRTEUSGL=irtgl irtglrgb irtviewer
NR=nr

LIBIRTEUS=$(LIBDIR)/$(LPFX)irteus.$(LSFX)
LIBIRTEUSG=$(LIBDIR)/$(LPFX)irteusg.$(LSFX)
LIBIRTEUSX=$(LIBDIR)/$(LPFX)irteusx.$(LSFX)
LIBIRTEUSIMG=$(LIBDIR)/$(LPFX)irteusimg.$(LSFX)
LIBIRTEUSGL=$(LIBDIR)/$(LPFX)irteusgl.$(LSFX)
LIBNR=$(LIBDIR)/$(LPFX)nr.$(LSFX)

IRTEUS_OSFX=$(addsuffix .$(OSFX),$(IRTEUS))
IRTEUS_LSFX=$(addsuffix .$(LSFX),$(IRTEUS))
IRTEUSOBJS=$(addprefix $(OBJDIR)/,$(IRTEUS_OSFX))
IRTEUSDLLS=$(addprefix $(OBJDIR)/,$(IRTEUS_LSFX))
IRTEUS_C=$(addsuffix .c,$(IRTEUS))
IRTEUS_H=$(addsuffix .h,$(IRTEUS))

IRTEUSG_OSFX=$(addsuffix .$(OSFX),$(IRTEUSG))
IRTEUSG_LSFX=$(addsuffix .$(LSFX),$(IRTEUSG))
IRTEUSGOBJS=$(addprefix $(OBJDIR)/,$(IRTEUSG_OSFX))
IRTEUSGDLLS=$(addprefix $(OBJDIR)/,$(IRTEUSG_LSFX))
IRTEUSG_C=$(addsuffix .c,$(IRTEUSG))
IRTEUSG_H=$(addsuffix .h,$(IRTEUSG))

IRTEUSX_OSFX=$(addsuffix .$(OSFX),$(IRTEUSX))
IRTEUSX_LSFX=$(addsuffix .$(LSFX),$(IRTEUSX))
IRTEUSXOBJS=$(addprefix $(OBJDIR)/,$(IRTEUSX_OSFX))
IRTEUSXDLLS=$(addprefix $(OBJDIR)/,$(IRTEUSX_LSFX))
IRTEUSX_C=$(addsuffix .c,$(IRTEUSX))
IRTEUSX_H=$(addsuffix .h,$(IRTEUSX))

IRTEUSIMG_OSFX=$(addsuffix .$(OSFX),$(IRTEUSIMG))
IRTEUSIMG_LSFX=$(addsuffix .$(LSFX),$(IRTEUSIMG))
IRTEUSIMGOBJS=$(addprefix $(OBJDIR)/,$(IRTEUSIMG_OSFX))
IRTEUSIMGDLLS=$(addprefix $(OBJDIR)/,$(IRTEUSIMG_LSFX))
IRTEUSIMG_C=$(addsuffix .c,$(IRTEUSIMG))
IRTEUSIMG_H=$(addsuffix .h,$(IRTEUSIMG))

IRTEUSGL_OSFX=$(addsuffix .$(OSFX),$(IRTEUSGL))
IRTEUSGL_LSFX=$(addsuffix .$(LSFX),$(IRTEUSGL))
IRTEUSGLOBJS=$(addprefix $(OBJDIR)/,$(IRTEUSGL_OSFX))
IRTEUSGLDLLS=$(addprefix $(OBJDIR)/,$(IRTEUSGL_LSFX))
IRTEUSGL_C=$(addsuffix .c,$(IRTEUSGL))
IRTEUSGL_H=$(addsuffix .h,$(IRTEUSGL))

IRTCOBJECTS=$(OBJDIR)/irtc.$(OSFX) 
IRTGCOBJECTS=$(OBJDIR)/CPQP.$(OSFX) $(OBJDIR)/euspqp.$(OSFX)
IRTIMGCOBJECTS=$(OBJDIR)/euspng.$(OSFX)
NROBJECTS=$(OBJDIR)/nr.$(OSFX)

all: $(LIBNR) $(LIBIRTEUS) $(LIBIRTEUSG) $(LIBIRTEUSX) $(LIBIRTEUSIMG) $(LIBIRTEUSGL) make-link install-irtext.l

$(LIBIRTEUS): $(IRTEUSOBJS) $(IRTCOBJECTS)
	$(LD) $(SOFLAGS) $(OUTOPT)$(LIBIRTEUS) $(IRTEUSOBJS) \
		$(IRTCOBJECTS) $(IMPLIB)

$(LIBIRTEUSG): $(IRTEUSGOBJS) $(IRTGCOBJECTS) PQP/$(ARCHDIR)/libPQP-static.a
	$(LD) $(SOFLAGS) $(OUTOPT)$(LIBIRTEUSG) $(IRTEUSGOBJS) \
		$(IRTGCOBJECTS) $(IMPLIB) $(PQPLIB)

$(LIBIRTEUSX): $(IRTEUSXOBJS)
	$(LD) $(SOFLAGS) $(OUTOPT)$(LIBIRTEUSX) $(IRTEUSXOBJS) $(IMPLIB)

$(LIBIRTEUSIMG): $(IRTEUSIMGOBJS) $(IRTIMGCOBJECTS) $(LIBDIR)/jpegmemcd.$(LSFX)
	$(LD) $(SOFLAGS) $(OUTOPT)$(LIBIRTEUSIMG) $(IRTEUSIMGOBJS) $(IRTIMGCOBJECTS) $(IMPLIB) $(IMPLIBIMG)

$(LIBIRTEUSGL): $(IRTEUSGLOBJS)
	$(LD) $(SOFLAGS) $(OUTOPT)$(LIBIRTEUSGL) $(IRTEUSGLOBJS) $(IMPLIB)

$(IRTEUSOBJS):
	$(BINDIR)/eus2$(ESFX) < ./compile_irt.l
	-rm -f $(IRTEUS_C) $(IRTEUS_H)

$(IRTEUSGOBJS):
	$(BINDIR)/eusg$(ESFX) < ./compile_irtg.l
	-rm -f $(IRTEUSG_C) $(IRTEUSG_H)

$(IRTEUSXOBJS) :
	$(BINDIR)/eusx$(ESFX) < ./compile_irtx.l
	-rm -f $(IRTEUSX_C) $(IRTEUSX_H)

$(IRTEUSIMGOBJS) :
	$(BINDIR)/eusx$(ESFX) < ./compile_irtimg.l
	-rm -f $(IRTEUSIMG_C) $(IRTEUSIMG_H)

$(IRTEUSGLOBJS):
	$(BINDIR)/eusgl$(ESFX) < ./compile_irtgl.l
	-rm -f $(IRTEUSGL_C) $(IRTEUSGL_H)

PQP/$(ARCHDIR)/libPQP-static.a:
	make -C PQP

$(LIBNR): $(NROBJECTS)
	$(LD) $(SOFLAGS) $(OUTOPT)$(LIBNR) $(NROBJECTS)

BINS=$(BINDIR)/irteus$(ESFX) $(BINDIR)/irteusgl$(ESFX)
make-link: $(BINS)
$(BINS):
	ln -sf eus$(ESFX) $(BINDIR)/irteus$(ESFX)
	ln -sf eusgl$(ESFX) $(BINDIR)/irteusgl$(ESFX)

install-irtext.l: $(EUSDIR)/lib/irtext.l
$(EUSDIR)/lib/irtext.l: irtext.l
	-cp -f irtext.l $(EUSDIR)/lib

clean:
	-rm -f $(IRTCOBJECTS) $(IRTGCOBJECTS)
	-rm -f $(LIBIRTEUS) $(LIBIRTEUSG) $(LIBIRTEUSGL) $(LIBIRTEUSX) $(LIBIRTEUSIMG)
	-rm -f $(IRTEUSOBJS) $(IRTEUSDLLS) $(IRTEUS_C) $(IRTEUS_H)
	-rm -f $(IRTEUSGOBJS) $(IRTEUSGDLLS) $(IRTEUSG_C) $(IRTEUSG_H)
	-rm -f $(IRTEUSXOBJS) $(IRTEUSXDLLS) $(IRTEUSX_C) $(IRTEUSX_H)
	-rm -f $(IRTEUSIMGOBJS) $(IRTEUSIMGDLLS) $(IRTEUSIMG_C) $(IRTEUSIMG_H)
	-rm -f $(IRTEUSGLOBJS) $(IRTEUSGLDLLS) $(IRTEUSGL_C) $(IRTEUSGL_H)
	-rm -f $(LIBDIR)/jpegmemcd.$(LSFX) $(LIBNR) $(NROBJECTS)
	-rm -f *~ *.stackdump
	chmod a-x Makefile* *.l *.c
	(cd PQP;make clean)
	(cd $(EUSDIR)/lisp/image/jpeg/; rm -f *.o)

$(OBJDIR)/irtmath.$(OSFX): irtmath.l
$(OBJDIR)/irtutil.$(OSFX): irtutil.l
$(OBJDIR)/irtgeo.$(OSFX): irtgeo.l
$(OBJDIR)/irtmodel.$(OSFX): irtmodel.l
$(OBJDIR)/irtdyna.$(OSFX): irtdyna.l
$(OBJDIR)/irtrobot.$(OSFX): irtrobot.l
$(OBJDIR)/irtx.$(OSFX): irtx.l
$(OBJDIR)/irtgl.$(OSFX): irtgl.l
$(OBJDIR)/irtglrgb.$(OSFX): irtglrgb.l
$(OBJDIR)/irtviewer.$(OSFX): irtviewer.l
$(OBJDIR)/irtimage.$(OSFX): irtimage.l
$(OBJDIR)/pqp.$(OSFX): pqp.l
$(OBJDIR)/png.$(OSFX): png.l
$(OBJDIR)/pgsql.$(OSFX): $(EUSDIR)/lib/llib/pgsql.l
$(OBJDIR)/eusjpeg.$(OSFX): $(EUSDIR)/lisp/image/jpeg/eusjpeg.l

$(OBJDIR)/irtc.$(OSFX): irtc.c
	$(CC) $(CFLAGS) -c irtc.c $(OBJOPT)$(OBJDIR)/irtc.$(OSFX)
$(OBJDIR)/CPQP.$(OSFX): CPQP.C
	$(CXX) $(CXXFLAGS) -c CPQP.C $(OBJOPT)$(OBJDIR)/CPQP.$(OSFX)
$(OBJDIR)/euspqp.$(OSFX): euspqp.c
	$(CC) $(CFLAGS) -c euspqp.c $(OBJOPT)$(OBJDIR)/euspqp.$(OSFX)
$(OBJDIR)/euspng.$(OSFX): euspng.c
	$(CC) $(CFLAGS) -c euspng.c $(OBJOPT)$(OBJDIR)/euspng.$(OSFX)
$(OBJDIR)/nr.$(OSFX): nr.c
	$(CC) $(CFLAGS) -c nr.c $(OBJOPT)$(OBJDIR)/nr.$(OSFX)

$(LIBDIR)/jpegmemcd.$(LSFX): $(EUSDIR)/lisp/image/jpeg/jpegmemcd.c
	(cd $(EUSDIR)/lisp/image/jpeg/;make)

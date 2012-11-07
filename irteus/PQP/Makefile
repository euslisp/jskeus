include Makefile.$(ARCHDIR)

PQPDIR		= src

CFLAGS		= -I. -I.. -D$(ARCHDIR)
LDFLAGS		= -L. -L..

.SUFFIXES: .cpp

SRCS		= PQP.cpp BV.cpp Build.cpp TriDist.cpp

OBJECTS		= $(ARCHDIR)/PQP.o $(ARCHDIR)/BV.o \
			$(ARCHDIR)/Build.o $(ARCHDIR)/TriDist.o 

CLEAN		= $(ARCHDIR)/$(LPFX)PQP.$(LSFX) $(OBJECTS) *.$(LSFX) \
		$(ARCHDIR)/*.o *~ $(ARCHDIR)/$(LPFX)PQP-static.a

default: directory library

directory:
	if [ ! -e $(ARCHDIR) ]; then mkdir -p $(ARCHDIR); fi

install: $(EUSDIR)/$(ARCHDIR)/lib/$(LPFX)PQP.$(LSFX)

$(EUSDIR)/$(ARCHDIR)/lib/$(LPFX)PQP.$(LSFX): $(ARCHDIR)/$(LPFX)PQP.$(LSFX)
	cp $(ARCHDIR)/$(LPFX)PQP.$(LSFX) $(EUSDIR)/$(ARCHDIR)/lib

library: $(ARCHDIR)/$(LPFX)PQP.$(LSFX) \
	$(ARCHDIR)/$(LPFX)PQP-static.a

$(ARCHDIR)/$(LPFX)PQP.$(LSFX): $(OBJECTS)
	$(LD) $(SOFLAGS) $(OUTOPT)$(ARCHDIR)/$(LPFX)PQP.$(LSFX) $(OBJECTS)

$(ARCHDIR)/$(LPFX)PQP-static.a: $(OBJECTS)
	$(AR) $(ARFLAGS) $@ $(OBJECTS)

$(OBJECTS):
	$(CC) $(CFLAGS) -DCOMPILE_LIB -c $(PQPDIR)/$(*F).cpp $(OBJOPT)$@

clean:
	rm -f $(CLEAN)

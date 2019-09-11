include Makefile.$(ARCHDIR)

CFLAGS		= -I. -I.. -D$(ARCHDIR) -D$(MACHINE) `pkg-config bullet --cflags`
LDFLAGS		= -L. -L.. `pkg-config bullet --libs`

.SUFFIXES: .cpp

SRCS		= BulletCollision.cpp

OBJECTS		= $(ARCHDIR)/BulletCollision.o

CLEAN		= $(ARCHDIR)/$(LPFX)BULLET.$(LSFX) $(OBJECTS) *.$(LSFX)

default: directory library

directory:
	if [ ! -e $(ARCHDIR) ]; then mkdir -p $(ARCHDIR); fi

install: $(EUSDIR)/$(ARCHDIR)/lib/$(LPFX)BULLET.$(LSFX)

$(EUSDIR)/$(ARCHDIR)/lib/$(LPFX)BULLET.$(LSFX): $(ARCHDIR)/$(LPFX)BULLET.$(LSFX)
	cp $(ARCHDIR)/$(LPFX)BULLET.$(LSFX) $(EUSDIR)/$(ARCHDIR)/lib

library: $(ARCHDIR)/$(LPFX)BULLET.$(LSFX)

$(ARCHDIR)/$(LPFX)BULLET.$(LSFX): $(OBJECTS)
	$(LD) $(SOFLAGS) $(OUTOPT)$(ARCHDIR)/$(LPFX)BULLET.$(LSFX) $(OBJECTS) $(LDFLAGS)

$(OBJECTS): $(SRCS)
	$(CC) $(CFLAGS) -DCOMPILE_LIB -c $(*F).cpp $(OBJOPT)$@

clean:
	rm -f $(CLEAN)

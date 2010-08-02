all: eus-installed irteus-installed show-env

SVN_EUSURL=https://euslisp.svn.sourceforge.net/svnroot/euslisp/trunk/EusLisp
SVN_IRTEUSURL=https://jskeus.svn.sourceforge.net/svnroot/jskeus/trunk/irteus

EUSC_PATCH=eus.c_CUSTUM_EUSDIR.patch

MACHINE=$(shell uname -m)
OS=$(shell uname -s)
ifeq ($(OS),Linux)
 ifeq ($(MACHINE),x86_64)
  export ARCHDIR=Linux64
  export MAKEFILE=Makefile.Linux64
 else
  export ARCHDIR=Linux
  export MAKEFILE=Makefile.Linux.thread
 endif
endif
ifeq ($(OS),Cygwin)
 export ARCHDIR=Cygwin
 export MAKEFILE=Makefile.Cygwin
endif

export EUSDIR=$(shell pwd)/eus

show-env:
	@echo ";;\n;; environment variable for euslisp\n;;"
	@echo "export EUSDIR=$(EUSDIR)"
	@echo "export ARCHDIR=$(ARCHDIR)"
	@echo "export PATH=\$$EUSDIR/\$$ARCHDIR/bin:\$$PATH"
	@echo "export LD_LIBRARY_PATHPATH=\$$EUSDIR/\$$ARCHDIR/bin:\$$LD_LIBRARY_PATH"

eus:
	svn co -N $(SVN_EUSURL) eus

eus-update:
	cd eus; svn up lisp
	cd eus; svn up -N lib; svn up -N lib/llib
	touch eus-update

eus-installed: eus eus-update
	cd eus/lisp && ln -sf $(MAKEFILE) Makefile && make eus0 eus1 eus2 eusg eusx eusgl eus
	touch eus-installed

irteus:
	svn co $(SVN_IRTEUSURL) irteus

irteus-update:
	cd irteus; svn up
	touch irteus-update

irteus-installed: irteus irteus-update
	cd irteus; make
	touch irteus-installed

clean:
	-rm *-installed *-update
	if [ -e irteus ]; then cd irteus; make clean ; fi
	if [ -e eus/lisp ]; then cd eus/lisp; make clean ; fi

wipe: clean
	-rm -fr eus irteus
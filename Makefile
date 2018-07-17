all: eus-installed irteus-installed manuals bashrc.eus

GIT_EUSURL ?= http://github.com/euslisp/EusLisp
GIT_EUSBRANCH ?= master

EUSC_PATCH=eus.c_CUSTUM_EUSDIR.patch

GCC_MACHINE=$(shell gcc -dumpmachine)
$(info "-- GCC_MACHINE = ${GCC_MACHINE}")
ifneq ($(GCC_MACHINE),)
 ifeq ($(GCC_MACHINE),x86_64-linux-gnu)
   MACHINE=x86_64
 endif
 ifeq ($(GCC_MACHINE),i686-linux-gnu)
   MACHINE=x86
 endif
endif
ifeq ($(MACHINE),)
 MACHINE=$(shell uname -m)
endif
OS=$(shell uname -s | sed 's/[^A-Za-z1-9].*//')
$(info "-- MACHINE = ${MACHINE}")
$(info "--      OS = ${OS}")
ifeq ($(OS),Linux)
 ifeq ($(MACHINE),x86_64)
  export ARCHDIR=Linux64
  export MAKEFILE=Makefile.Linux64
 else
 ifneq (, $(findstring armv,$(MACHINE)))
  export ARCHDIR=LinuxARM
  export MAKEFILE=Makefile.LinuxARM
 else ifneq (, $(findstring aarch,$(MACHINE)))
  export ARCHDIR=LinuxARM
  export MAKEFILE=Makefile.LinuxARM
 else
  export ARCHDIR=Linux
  export MAKEFILE=Makefile.Linux.thread
 endif
 endif
endif
ifeq ($(OS),CYGWIN)
 export ARCHDIR=Cygwin
 export MAKEFILE=Makefile.Cygwin
endif
ifeq ($(OS),Darwin)
 export ARCHDIR=Darwin
 export MAKEFILE=Makefile.Darwin
endif

export EUSDIR=$(shell pwd)/eus

manuals: manual.pdf jmanual.pdf
manual.pdf:
	ln -sf eus/doc/latex/manual.pdf manual.pdf

jmanual.pdf:
	ln -sf eus/doc/jlatex/jmanual.pdf  jmanual.pdf

bashrc.eus:
	@bash -c 'echo -e "#\n\
# bashrc.eus : environment variable for euslisp \n#\n\
export EUSDIR=$(EUSDIR) \n\
export ARCHDIR=$(ARCHDIR) \n\
export PATH=\$$EUSDIR/\$$ARCHDIR/bin:\$$EUSDIR/\$$ARCHDIR/lib:\$$PATH \n\
export LD_LIBRARY_PATH=\$$EUSDIR/\$$ARCHDIR/lib:\$$EUSDIR/\$$ARCHDIR/bin:\$$LD_LIBRARY_PATH \n\
"' > bashrc.eus
	@bash -c 'echo -e "\e[1;32m;; generating bashrc.eus ...\n;;\e[m"'
	@bash -c 'echo -e "\e[1;32m;;   mv bashrc.eus ~/bashrc.eus\e[m"'
	@bash -c 'echo -e "\e[1;32m;;   echo \"source ~/bashrc.eus\" >> ~/.bashrc\e[m"'
	@bash -c 'echo -e "\e[1;32m;;   source ~/.bashrc\e[m"'
	@cat bashrc.eus

eus:
	COUNT=10; while [ $$COUNT -gt 0 -a ! -e eus ] ; do echo $$COUNT; sleep 1; GIT_SSL_NO_VERIFY=true git clone --depth 10 $(GIT_EUSURL) -b $(GIT_EUSBRANCH) eus; COUNT=`expr $$COUNT - 1`; done; #

eus-installed: eus
	cd eus/lisp && ln -sf $(MAKEFILE) Makefile && $(MAKE) eus0 eus1 eus2 eusg eusx eusgl eus

irteus-installed: eus-installed
	cd irteus; $(MAKE)

clean:
	-rm -f bashrc.eus manual.pdf jmanual.pdf
	if [ -e irteus ]; then cd irteus; make clean ; fi
	if [ -e eus/lisp ]; then cd eus/lisp; make clean ; fi

wipe: clean
	-rm -fr eus irteus


doc:
	cd doc; make

.PHONY: doc

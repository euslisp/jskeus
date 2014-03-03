all: eus-installed irteus-installed manuals bashrc.eus

GIT_EUSURL=http://github.com/euslisp/EusLisp

EUSC_PATCH=eus.c_CUSTUM_EUSDIR.patch

MACHINE=$(shell uname -m)
OS=$(shell uname -s | sed 's/[^A-Za-z1-9].*//')
$(info "-- MACHINE = ${MACHINE}")
$(info "--      OS = ${OS}")
ifeq ($(OS),Linux)
 ifeq ($(MACHINE),x86_64)
  export ARCHDIR=Linux64
  export MAKEFILE=Makefile.Linux64
 else
 ifeq ($(MACHINE),armv6l)
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
export LD_LIBRARY_PATH=\$$EUSDIR/\$$ARCHDIR/bin:\$$LD_LIBRARY_PATH \n\
"' > bashrc.eus
	@bash -c 'echo -e "\e[1;32m;; generating bashrc.eus ...\n;;\e[m"'
	@bash -c 'echo -e "\e[1;32m;;   mv bashrc.eus ~/bashrc.eus\e[m"'
	@bash -c 'echo -e "\e[1;32m;;   echo \"source ~/bashrc.eus\" >> ~/.bashrc\e[m"'
	@bash -c 'echo -e "\e[1;32m;;   source ~/.bashrc\e[m"'
	@cat bashrc.eus

eus:
	git clone --depth 10 $(GIT_EUSURL) eus

eus-installed: eus
	cd eus/lisp && ln -sf $(MAKEFILE) Makefile && make eus0 eus1 eus2 eusg eusx eusgl eus

irteus-installed:
	cd irteus; make

clean:
	-rm -f bashrc.eus manual.pdf jmanual.pdf
	if [ -e irteus ]; then cd irteus; make clean ; fi
	if [ -e eus/lisp ]; then cd eus/lisp; make clean ; fi

wipe: clean
	-rm -fr eus irteus


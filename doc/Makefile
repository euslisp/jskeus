TMPDIR := $(shell mktemp -d)
FIGDIR := ${TMPDIR}/fig

# IMAGE_FILES := $(shell find ${FIGDIR} -name "*.(jpg|png)" | sed s/jpg/bb/g)

# BB_FILES = $(patsubst %.jpg,%.bb,$(IMAGE_FILES))

# .SUFFIXES: .txt .tex .bbl .bb .jpg .JPG .png .PNG .bmp .BMP .pdf

# .jpg.bb:
# 	ebb -b $<

all: pdf
	rm -fr ${TMPDIR}

copy_eus_tex:
	-mkdir -p ${TMPDIR}/fig
	cp ../eus/doc/jlatex/*.tex ${TMPDIR}
	cp ../eus/doc/jlatex/*.sty ${TMPDIR}
	cp -r ../eus/doc/jlatex/fig/* ./fig/* ${TMPDIR}/fig/
	cp ../eus/doc/latex/*.tex ${TMPDIR}
	cp ../eus/doc/latex/*.sty ${TMPDIR}
	cp -r ../eus/doc/latex/fig/* ./fig/* ${TMPDIR}/fig/
	(cd ${TMPDIR};for x in fig/*.ps;  do ps2ps $$x $$x.new; mv $$x.new $$x; done) ### ???? we need to run ps2ps to pass gs??
	(cd ${TMPDIR};for x in fig/*.jpg; do extractbb $$x; done) # workaround for https://github.com/backtracking/bibtex2html/issues/9
	cp ./*latex/*.tex ${TMPDIR}
	platex --version | grep utf8 || nkf --in-place -e ${TMPDIR}/*.tex
	for x in irtrobot irtmodel irtsensor irtscene irtdyna irtgeo irtgl irtutil irtviewer irtx irtmath irtbvh irtcollada irtgraph gnuplotlib irtimage irtpointcloud png irtcollision pqp bullet; do\
		irteusgl ../eus/lib/llib/documentation.l "(make-document \"../irteus/$$x.l\" \"${TMPDIR}/$$x-func.tex\")" "(exit)"; \
        done

current.tex:
	echo "% CURRENT_TAG" > ${TMPDIR}/current.tex
	echo "\\\\newcommand{\\\\eusversion}[0]{"`cd ../eus; git describe --tags $$(git rev-list --tags --max-count=1) | sed s/EusLisp-//`"}" >> ${TMPDIR}/current.tex
	echo "\\\\newcommand{\\\\irteusversion}[0]{"`git describe --tags $$(git rev-list --tags --max-count=1)`"}" >> ${TMPDIR}/current.tex
	cat ${TMPDIR}/current.tex

toc: copy_eus_tex current.tex
	(cd ${TMPDIR}; makeindex jmanual.tex)
	(cd ${TMPDIR}; makeindex manual.tex)


dvi: toc
	(cd ${TMPDIR}; platex jmanual.tex)
	(cd ${TMPDIR}; platex manual.tex)
	cp ${TMPDIR}/j*.dvi ./jlatex
	cp ${TMPDIR}/[!j]*.dvi ./latex

pdf: dvi
	(cd ${TMPDIR}; dvipdfmx jmanual.dvi)
	(cd ${TMPDIR}; dvipdfmx manual.dvi)
	cp ${TMPDIR}/j*.pdf ./jlatex
	cp ${TMPDIR}/[!j]*.pdf ./latex
	cp ${TMPDIR}/*.pdf .


clean:
	rm -f *~ *.aux *.idx *.log *.toc *.out *.ind *.ilg

distclean: clean
	rm -fr *.dvi *.pdf

html: dvi
	mkdir -p ${TMPDIR}/html
	(cd ${TMPDIR}; TRANSPARENT_COLOR="#ffffff" charset="UTF-8" latex2html -dir ${TMPDIR}/html -local_icons -split +3 -auto_prefix -init_file ${PWD}/../eus/doc/latex/.latex2html-init -iso_language JP -address "This document was generated using the LaTeX2HTML translator on `date` from <a href=\"http://github.com/euslisp/jskeus.git\">jskeus</a> version <a href=\"http://github.com/euslisp/jskeus/commit/`git -C ${PWD} rev-parse --verify HEAD`\">`git log -1  --oneline`</a>" -html_version="4.0,unicode" jmanual)
	(cd ${TMPDIR}; TRANSPARENT_COLOR="#ffffff" charset="UTF-8" latex2html -dir ${TMPDIR}/html -local_icons -split +3 -auto_prefix -init_file ${PWD}/../eus/doc/latex/.latex2html-init -iso_language JP -address "This document was generated using the LaTeX2HTML translator on `date` from <a href=\"http://github.com/euslisp/jskeus.git\">jskeus</a> version <a href=\"http://github.com/euslisp/jskeus/commit/`git -C ${PWD} rev-parse --verify HEAD`\">`git log -1  --oneline`</a>" -html_version="4.0,unicode" manual)
	(cd ${TMPDIR}/html; for imgfile in jmanual-img*.png; do pngtopnm $$imgfile > ${TMPDIR}/$$imgfile.pnm; pnmtopng -transparent white ${TMPDIR}/$$imgfile.pnm > $$imgfile || echo "Failed to convert $$imgfile"; done)
	(cd ${TMPDIR}/html; for imgfile in manual-img*.png; do pngtopnm $$imgfile > ${TMPDIR}/$$imgfile.pnm; pnmtopng -transparent white ${TMPDIR}/$$imgfile.pnm > $$imgfile || echo "Failed to convert $$imgfile"; done)
	(cd ${TMPDIR}/html; sed -i 's@</TITLE>@</TITLE>\n<META HTTP-EQUIV="Content-Type" CONTENT="text/html; charset=UTF-8">@' j*.html index.html)
	cp -r ${TMPDIR}/html ./

latex.py:
	wget https://raw.githubusercontent.com/jobh/latex.py/master/latex.py

rst: current.tex copy_eus_tex latex.py
	cp latex.py ../eus/doc/latex/preamble.tex ${TMPDIR}
	for file in ${TMPDIR}/*.tex; do \
	   name=`basename $$file .tex`; \
	   if [ \"$$name\" == \"test\" -o \"$$name\" == \"jmanual\" ]; then continue; fi; \
	   echo \"Converting $$name.tex to $$name.rst\"; \
	   (cd ${TMPDIR}; python3 latex.py -2 -L preamble.tex $$name.tex > ${TMPDIR}/tmp_$$name.tex); \
	   pandoc ${TMPDIR}/tmp_$$name.tex -o $$name.rst -V documentclass=ltjarticle --pdf-engine=lualatex; \
	   sed -i 's@.. figure:: \(.*\).ps@.. figure:: \1.png@g' $$name.rst;  # use png for figures \
	done
	for file in ../eus/doc/jlatex/fig/*.ps; do \
	   name=`basename $$file .ps`; \
	   echo \"Converting $$name.ps to $$name.png\";  ### ???? we need to run ps2ps to pass gs?? does not happy with pstopnm \
	   pstopnm -stdout ../eus/doc/jlatex/fig/$$name.ps | pnmflip -rotate270 | pnmtopng - > fig/$$name.png; \
	done

	cp ./jlatex/jmanual.tex ${TMPDIR}/jmanual.tex
	sed -i 's@\\input{\(.*\)}@..include:: \1\n@g' ${TMPDIR}/jmanual.tex     ### convert \input in tex to bypass pandoc EXCEPT current
	sed -i '/..include:: current/r ${TMPDIR}/current.tex' ${TMPDIR}/jmanual.tex  ### convert \input in tex to bypass pandoc EXCEPT current
	sed -i 's@\\part{\(.*\)}@.. toctree:: \1@' ${TMPDIR}/jmanual.tex  # convert \part in tex to bypass pandoc
	sed -i ':a;/^[^%].*\\\\$$/{N;s/\\\\\n//;ba}' ${TMPDIR}/jmanual.tex # concatinate title/author multi lines
	sed -i '/^\\vspace{10mm}$$/d' ${TMPDIR}/jmanual.tex # remove vspace{10mm} within title to pass pandoc
	pandoc --wrap=none -s ${TMPDIR}/jmanual.tex -o jmanual.rst -V documentclass=ltjarticle --pdf-engine=lualatex
	pandoc --wrap=none -s ${TMPDIR}/manual.tex -o manual.rst -V documentclass=ltjarticle --pdf-engine=lualatex
	sed -i 's@..include:: \(.*\)@   \1@' jmanual.rst # restore ..include for rst
	sed -i 's@.. toctree:: \(.*\)@.. toctree::\n   :maxdepth: 2\n   :caption: \1@' jmanual.rst  # restore ..toctree for rst
	sed -i 's@\*\* \*\*@\*\*\n\*\*@g' jmanual.rst # add newline after ** ** in title
	sed -i 's@** -\(.*\)$$@**\n\n-\1\n@' jmanual.rst # add newline before -ロボットモデリングの拡張 in title
	sed -i -ne '3!{p;d;};h;n;p;g;p' jmanual.rst  ## swap 2nd and 3rd line
	sed -i -ne '4!{p;d;};h;n;:1;7!{N;b1};G;h;n;p;g;p' jmanual.rst  ## swap lines https://www.unix.com/shell-programming-and-scripting/85230-swapping-switching-2-lines-using-sed.html
	sed -i -ne '2h;2!H;$${g;s/\n/ /;p}' jmanual.rst  ## concatenate EusLisp and Reference Manual (https://unix.stackexchange.com/questions/188264/want-to-substitute-only-first-occurence-with-sed)
	sed -i 's/^:Author:/:Authors: -/' jmanual.rst # use :authors: instead of :author:
	sed -ie '/^:Authors:/ {p;s/.*//;N; s/\n/\ \ \ \ \ \ \ \ \ \ -\ /;}' jmanual.rst # add authors after :authors:
	head -n 20 jmanual.rst

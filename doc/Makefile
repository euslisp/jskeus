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
	exit
	-mkdir -p ${TMPDIR}/fig
	cp ../eus/doc/jlatex/*.tex ${TMPDIR}
	cp ../eus/doc/jlatex/*.sty ${TMPDIR}
	cp -r ../eus/doc/jlatex/fig/* ./fig/* ${TMPDIR}/fig/
	for x in ${TMPDIR}/fig/*.jpg; do ebb -b $$x; extractbb $$x;done
	cp *.tex ${TMPDIR}
	platex --version | grep utf8 || nkf --in-place -e ${TMPDIR}/*.tex
	for x in irtrobot irtmodel irtsensor irtscene irtdyna irtgeo irtgl irtutil irtviewer irtx irtmath irtbvh irtcollada irtgraph irtimage irtpointcloud png pqp; do\
		irteusgl ../eus/lib/llib/documentation.l "(make-document \"../irteus/$$x.l\" \"${TMPDIR}/$$x-func.tex\")" "(exit)"; \
        done

toc: copy_eus_tex
	(cd ${TMPDIR}; makeindex jmanual.tex)

dvi: toc
	(cd ${TMPDIR}; platex jmanual.tex)
	(cd ${TMPDIR}; platex jmanual.tex)
	cp ${TMPDIR}/*.dvi .

pdf: dvi
	(cd ${TMPDIR}; dvipdfmx jmanual.dvi)
	cp ${TMPDIR}/*.pdf .


clean:
	rm -f *~ *.aux *.idx *.log *.toc *.out *.ind *.ilg

distclean: clean
	rm -f *.dvi *.pdf

html: copy_eus_tex
	nkf --in-place -u ${TMPDIR}/*.tex
	mkdir ${TMPDIR}/html
	sed -i 's@^\(\\\usepackage.*{hyperref}\)@%\1@' ${TMPDIR}/jmanual.tex
	sed -i 's@\\\documentclass\[\]{jarticle}@\\\documentclass\[\]{article}@' ${TMPDIR}/jmanual.tex
	sed -i 's@^\\\ifnum 42146@%\\\ifnum 42146@' ${TMPDIR}/jmanual.tex
	(cd ${TMPDIR}; charset="UTF-8" latex2html -dir ${TMPDIR}/html -local_icons -auto_prefix -iso_language JP jmanual)
	(cd ${TMPDIR}/html; sed -i 's@</TITLE>@</TITLE>\n<META HTTP-EQUIV="Content-Type" CONTENT="text/html; charset=UTF-8">@' j*.html index.html)
	sed -i 's@%\(\\\usepackage.*{hyperref}\)@\1@' ${TMPDIR}/jmanual.tex
	sed -i 's@\\\documentclass\[\]{article}@\\\documentclass\[\]{jarticle}@' ${TMPDIR}/jmanual.tex
	sed -i 's@^%\\\ifnum 42146@\\\ifnum 42146@' ${TMPDIR}/jmanual.tex
	cp -r ${TMPDIR}/html ./
	rm -fr ${TMPDIR}


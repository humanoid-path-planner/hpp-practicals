SOURCES = \
	exercise-1.asciidoc \
	exercise-2.asciidoc

HTML_FILES=$(SOURCES:%.asciidoc=%.html)

all: $(HTML_FILES)

%.html: %.asciidoc
	asciidoc --filter latex -a toc -a icons -o $@ $<

clean:
	rm -f $(HTML_FILES)

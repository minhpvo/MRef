include ../Makefile.rules

IFLAGS = $(BASEINCPATH) \
	 $(OPENCVINCPATH) \
	 $(LASINCPATH) \
	 $(OPENMESHINCPATH) \
	 $(BOOSTINCPATH) \
	$(EIGENINCPATH)	

LFLAGS =  $(BASELIBPATH) \
	  $(LASLIBPATH) \
	  $(OPENCVLIBPATH) $(LIBOCVHIGHGUI) $(LIBOCVCALIB3D) $(LIBOCVIMGPROC) $(LIBOCOBJDETECT)  $(LIBOCVCORE) $(LIBOCVEXTRA)\
	$(OPENMESHLIBPATH) $(BOOSTFILESYSTEMLIBPATHSTAT) -lprstimer

OBJECTS = TfwIO.o

.PHONY: all
all: lib exe
lib: $(OBJECTS) tfwutil.a
exe: 

TfwIO.o: TfwIO.h TfwIO.cpp
	@echo ${cc_green} Compile TfwIO.cpp ... ${cc_nocolor}
	${CC} ${CFLAGS} -c TfwIO.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

test_tfwio.o: test_tfwio.cpp
	@echo ${cc_green} Compile test_tfwio.cpp ... ${cc_nocolor}
	${CC} ${CFLAGS} -c test_tfwio.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

test_tfwio: test_tfwio.o ${OBJECTS}
	@echo ${cc_green}Link test_tfwio...${cc_nocolor}
	${CC} ${CCFLAGS} -o test_tfwio test_tfwio.o ${OBJECTS} ${LFLAGS}
	#@mv test_tfwio ./test
	@echo ${cc_green}...Done ${cc_nocolor}

tfwutil.a : ${OBJECTS}
	@echo ${cc_green} Archive...${cc_nocolor}
	ar rcs libtfwutil.a ${OBJECTS}
	@mv libtfwutil.a ${LIBPATH}
	@echo ${cc_green}...Done${cc_nocolor}


.PHONY: clean
clean:
	@rm -rvf *.o




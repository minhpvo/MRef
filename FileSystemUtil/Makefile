include ../Makefile.rules

IFLAGS = $(BASEINCPATH) \
	 $(OPENCVINCPATH) \
	 $(LASINCPATH) \
	 $(OPENMESHINCPATH) \
	 $(BOOSTINCPATH)

LFLAGS =  $(BASELIBPATH) \
	  $(LASLIBPATH) \
	  $(OPENCVLIBPATH) $(LIBOCVHIGHGUI) $(LIBOCVCALIB3D) $(LIBOCVIMGPROC) $(LIBOCOBJDETECT)  $(LIBOCVCORE) $(LIBOCVEXTRA)\
	$(OPENMESHLIBPATH) $(BOOSTFILESYSTEMLIBPATH)  

OBJECTS = FDUtil.o

.PHONY: all
all: lib exe
lib: $(OBJECTS) libfdutil.a
exe: 

FDUtil.o: FDUtil.h FDUtil.cpp
	@echo ${cc_green} Compile FDUtil.cpp ... ${cc_nocolor}
	${CC} ${CFLAGS} -c FDUtil.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

libfdutil.a: ${OBJECTS}
	@echo ${cc_green} Archive...${cc_nocolor}
	ar rcs libfdutil.a ${OBJECTS}
	@mv libfdutil.a ${LIBPATH}
	@echo ${cc_green}...Done${cc_nocolor}

.PHONY: clean
clean:
	@rm -rvf *.o




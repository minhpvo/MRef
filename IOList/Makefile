include ../Makefile.rules


IFLAGS = $(BASEINCPATH) \
	 $(OPENCVINCPATH) \
	 $(EIGENINCPATH) \
	 $(EMBREEINCPATH) \
	 $(OPENMESHINCPATH) \
	 $(BOOSTINCPATH)

LFLAGS =  $(BASELIBPATH) -lprstimer -lmeshutil \
	  $(EMBREELIBPATH) \
	  $(OPENCVLIBPATH) $(LIBOCVHIGHGUI) $(LIBOCVCALIB3D) $(LIBOCVIMGPROC) $(LIBOCOBJDETECT)  $(LIBOCVCORE) $(LIBOCVEXTRA)\
	  $(EMBREELIBPATH) \
	  $(OPENMESHLIBPATH) \
	  $(BOOSTFILESYSTEMLIBPATH)


OBJECTS = IOList.o

.PHONY: all
all: lib exe
lib: $(OBJECTS) libiolist.a
exe: 

IOList.o: IOList.h IOList.cpp
	@echo ${cc_green} Compile IOList.cpp ... ${cc_nocolor}
	${CC} ${CFLAGS} -c IOList.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}


libiolist.a : ${OBJECTS}
	@echo ${cc_green} Archive...${cc_nocolor}
	ar rcs libiolist.a ${OBJECTS}
	@mv libiolist.a ${LIBPATH}
	@echo ${cc_green}...Done${cc_nocolor}


.PHONY: clean
clean:
	@rm -rvf *.o



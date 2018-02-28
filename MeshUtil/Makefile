include ../Makefile.rules

IFLAGS = $(BASEINCPATH) \
	 $(OPENCVINCPATH) \
	 $(OPENMESHINCPATH) \
	 $(BOOSTINCPATH) \
	 $(EIGENINCPATH)

LFLAGS =  $(BASELIBPATH) -lprstimer -lmeshmetadata -liolist \
	  $(OPENCVLIBPATH) $(LIBOCVHIGHGUI) $(LIBOCVCALIB3D) $(LIBOCVIMGPROC) $(LIBOCOBJDETECT)  $(LIBOCVCORE) $(LIBOCVEXTRA)\
	$(OPENMESHLIBPATH) $(BOOSTFILESYSTEMLIBPATH) 

OBJECTS = MeshDivide.o MeshSimplify.o MeshFilter.o MeshGeom.o MeshIO.o MeshConv.o

.PHONY: all
all: lib exe
lib: $(OBJECTS) meshutil.a
exe: 

MeshDivide.o: MeshDivide.h MeshDivide.cpp
	@echo ${cc_green} Compile MeshDivide.cpp ... ${cc_nocolor}
	${CC} ${CFLAGS} -c MeshDivide.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

MeshSimplify.o: MeshSimplify.h MeshSimplify.cpp
	@echo ${cc_green} Compile MeshSimplify.cpp ... ${cc_nocolor}
	${CC} ${CFLAGS} -c MeshSimplify.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

MeshFilter.o: MeshFilter.h MeshFilter.cpp
	@echo ${cc_green} Compile MeshFilter.cpp ... ${cc_nocolor}
	${CC} ${CFLAGS} -c MeshFilter.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

MeshGeom.o: MeshGeom.h MeshGeom.cpp
	@echo ${cc_green} Compile MeshGeom.cpp ... ${cc_nocolor}
	${CC} ${CFLAGS} -c MeshGeom.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

MeshConv.o: MeshConv.h MeshConv.cpp
	@echo ${cc_green} Compile MeshConv.cpp ... ${cc_nocolor}
	${CC} ${CFLAGS} -c MeshConv.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

MeshIO.o: MeshIO.h MeshIO.cpp
	@echo ${cc_green} Compile MeshIO.cpp ... ${cc_nocolor}
	${CC} ${CFLAGS} -c MeshIO.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

meshutil.a : ${OBJECTS}
	@echo ${cc_green} Archive...${cc_nocolor}
	ar rcs libmeshutil.a ${OBJECTS}
	@mv libmeshutil.a ${LIBPATH}
	@echo ${cc_green}...Done${cc_nocolor}


.PHONY: clean
clean:
	@rm -rvf *.o




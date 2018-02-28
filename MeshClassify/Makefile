include ../Makefile.rules


IFLAGS = $(BASEINCPATH) \
	 $(OPENCVINCPATH) \
	 $(EIGENINCPATH) \
	 $(EMBREEINCPATH)\
	 $(OPENMESHINCPATH) \
	$(BOOSTINCPATH)

LFLAGS =  $(BASELIBPATH) -lgraphmrf -liolist -lprsori -llikelihoodimage -lraytracer -lprstimer -lmeshutil -lmeshmetadata  \
	  $(EMBREELIBPATH) \
	  $(OPENCVLIBPATH) $(LIBOCVHIGHGUI) $(LIBOCVCALIB3D) $(LIBOCVIMGPROC) $(LIBOCOBJDETECT)  $(LIBOCVCORE) $(LIBOCVEXTRA)\
	  $(EMBREELIBPATH) \
	  $(OPENMESHLIBPATH) \
	$(BOOSTFILESYSTEMLIBPATH)


OBJECTS = MeshMRF.o

all: lib exe
lib: $(OBJECTS)  libmeshmrf.a
exe: 

.PHONY:  all

MeshMRF.o: MeshMRF.h MeshMRF.cpp
	@echo ${cc_green} Compile MeshMRF.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c MeshMRF.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

libmeshmrf.a: ${OBJECTS}
	@echo ${cc_green} Archive...${cc_nocolor}
	ar rcs libmeshmrf.a ${OBJECTS}
	@mv libmeshmrf.a ${LIBPATH}
	@echo ${cc_green}...Done${cc_nocolor}


.PHONY: clean
clean:
	@rm -rvf *.o



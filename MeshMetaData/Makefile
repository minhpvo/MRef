include ../Makefile.rules


IFLAGS = $(BASEINCPATH) \
	 $(OPENCVINCPATH) \
	 $(EIGENINCPATH) \
	 $(EMBREEINCPATH)\
	 $(OPENMESHINCPATH) \
	$(BOOSTINCPATH)

LFLAGS =  $(BASELIBPATH) -liolist -lprstimer -lmeshutil \
	  $(EMBREELIBPATH) \
	  $(OPENCVLIBPATH) $(LIBOCVHIGHGUI) $(LIBOCVCALIB3D) $(LIBOCVIMGPROC) $(LIBOCOBJDETECT)  $(LIBOCVCORE) $(LIBOCVEXTRA)\
	  $(EMBREELIBPATH) \
	  $(OPENMESHLIBPATH) \
	$(BOOSTFILESYSTEMLIBPATHSTAT)


OBJECTS = MeshMetaData.o

all: lib exe
lib: $(OBJECTS)  libmeshmetadata.a
exe: 

.PHONY:  all

MeshMetaData.o: MeshMetaData.h MeshMetaData.cpp
	@echo ${cc_green} Compile MeshmetaData.cpp ... ${cc_nocolor}
	${CC} ${CFLAGS} -c MeshMetaData.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

libmeshmetadata.a: ${OBJECTS}
	@echo ${cc_green} Archive...${cc_nocolor}
	ar rcs libmeshmetadata.a ${OBJECTS}
	@mv libmeshmetadata.a ${LIBPATH}
	@echo ${cc_green}...Done${cc_nocolor}

.PHONY: clean
clean:
	@rm -rvf *.o



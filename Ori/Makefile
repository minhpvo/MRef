include ../Makefile.rules


IFLAGS = $(BASEINCPATH) \
	 $(OPENCVINCPATH) \
	 $(EIGENINCPATH) \
	 $(EMBREEINCPATH)\
	 $(OPENMESHINCPATH) \
	$(BOOSTINCPATH)

LFLAGS =  $(BASELIBPATH) -lprstimer -lmeshutil \
	  $(EMBREELIBPATH) \
	  $(OPENCVLIBPATH) $(LIBOCVHIGHGUI) $(LIBOCVCALIB3D) $(LIBOCVIMGPROC) $(LIBOCOBJDETECT)  $(LIBOCVCORE) $(LIBOCVEXTRA)\
	  $(EMBREELIBPATH) \
	  $(OPENMESHLIBPATH) \
	$(BOOSTFILESYSTEMLIBPATHSTAT)


OBJECTS = Orientation.o OriGeom.o

all: lib exe
lib: $(OBJECTS) test_ori.o libprsori.a
exe: test_ori

.PHONY:  all

OriGeom.o: OriGeom.h OriGeom.cpp
	@echo ${cc_green} Compile OriGeom.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c OriGeom.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

Orientation.o: Orientation.h Orientation.cpp
	@echo ${cc_green} Compile Orientation.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c Orientation.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

test_ori.o:  test_ori.cpp
	@echo ${cc_green} Compile test_ori.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c test_ori.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

test_ori: test_ori.o ${OBJECTS}
	@echo ${cc_green}Link test_ori...${cc_nocolor}
	${CC} ${CCFLAGS} -o test_ori test_ori.o ${OBJECTS} ${LFLAGS}
	#@mv test_ori ./tests
	@echo ${cc_green}...Done ${cc_nocolor}

libprsori.a: ${OBJECTS}
	@echo ${cc_green} Archive...${cc_nocolor}
	ar rcs libprsori.a ${OBJECTS}
	@mv libprsori.a ${LIBPATH}
	@echo ${cc_green}...Done${cc_nocolor}


.PHONY: clean
clean:
	@rm -rvf *.o



include ../Makefile.rules


IFLAGS = $(BASEINCPATH) \
	 $(OPENCVINCPATH) \
	 $(EIGENINCPATH) \
	 $(EMBREEINCPATH)\
	 $(OPENMESHINCPATH) \
	$(BOOSTINCPATH)

LFLAGS =  $(BASELIBPATH) -lprstimer -lmeshutil -lprsori \
	  $(EMBREELIBPATH) \
	  $(OPENCVLIBPATH) $(LIBOCVHIGHGUI) $(LIBOCVCALIB3D) $(LIBOCVIMGPROC) $(LIBOCOBJDETECT)  $(LIBOCVCORE) $(LIBOCVEXTRA)\
	  $(EMBREELIBPATH) \
	  $(OPENMESHLIBPATH) \
	$(BOOSTFILESYSTEMLIBPATH)


OBJECTS = RayTracer.o

all: lib exe
lib: $(OBJECTS) libraytracer.a
exe: 

.PHONY:  all

RayTracer.o: RayTracer.h RayTracer.cpp
	@echo ${cc_green} Compile RayTracer.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c RayTracer.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

test_tracer.o: test_tracer.cpp
	@echo ${cc_green}Compile test_tracer.cpp...${cc_nocolor}
	${CC} ${CFLAGS} -c test_tracer.cpp ${IFLAGS}
	@echo ${cc_green}... Done ${cc_nocolor}

test_tracer: test_tracer.o ${OBJECTS}
	@echo ${cc_green}Link test_tracer...${cc_nocolor}
	${CC} ${CCFLAGS} -o test_tracer test_tracer.o ${OBJECTS} ${LFLAGS}
	@echo ${cc_green}...Done ${cc_nocolor}

libraytracer.a : ${OBJECTS}
	@echo ${cc_green} Archive...${cc_nocolor}
	ar rcs libraytracer.a ${OBJECTS}
	@mv libraytracer.a ${LIBPATH}
	@echo ${cc_green}...Done${cc_nocolor}


.PHONY: clean
clean:
	@rm -rvf *.o



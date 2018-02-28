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


OBJECTS = ControlRefine.o ControlWriter.o ControlReader.o 

all: lib exe
lib: $(OBJECTS) libcontrol.a
exe: test_control

.PHONY:  all

ControlRefine.o: ControlRefine.h ControlRefine.cpp
	@echo ${cc_green} Compile ControlRefine.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c ControlRefine.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

ControlReader.o: ControlReader.h ControlReader.cpp
	@echo ${cc_green} Compile ControlReader.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c ControlReader.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

ControlWriter.o: ControlWriter.h ControlWriter.cpp
	@echo ${cc_green} Compile ControlWriter.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c ControlWriter.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

test_control.o:  test_control.cpp
	@echo ${cc_green} Compile test_control.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c test_control.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

test_control: test_control.o ${OBJECTS}
	@echo ${cc_green}Link test_control...${cc_nocolor}
	${CC} ${CCFLAGS} -o test_control test_control.o ${OBJECTS} ${LFLAGS}
	#@mv test_ori ./tests
	@echo ${cc_green}...Done ${cc_nocolor}

libcontrol.a: ${OBJECTS}
	@echo ${cc_green} Archive...${cc_nocolor}
	ar rcs libcontrol.a ${OBJECTS}
	@mv libcontrol.a ../lib
	@echo ${cc_green}...Done${cc_nocolor}


.PHONY: clean
clean:
	@rm -rvf *.o



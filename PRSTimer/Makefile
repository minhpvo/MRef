include ../Makefile.rules


IFLAGS = $(BASEINCPATH) \
	 $(OPENCVINCPATH) \

LFLAGS =  $(BASELIBPATH) \
	  $(OPENCVLIBPATH) $(LIBOCVHIGHGUI) $(LIBOCVCALIB3D) $(LIBOCVIMGPROC) $(LIBOCOBJDETECT)  $(LIBOCVCORE) $(LIBOCVEXTRA)\

OBJECTS = PRSTimer.o

all: lib exe 
lib: $(OBJECTS) prstimer.a
exe:

.PHONY:  all

PRSTimer.o: PRSTimer.h PRSTimer.cpp
	@echo ${cc_green} Compile PRSTimer.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c PRSTimer.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

prstimer.a : ${OBJECTS}
	@echo ${cc_green} Archive...${cc_nocolor}
	ar rcs libprstimer.a ${OBJECTS}
	@mv libprstimer.a ../lib/ 
	@echo ${cc_green}...Done${cc_nocolor}


.PHONY: clean
clean:
	@rm -rvf *.o




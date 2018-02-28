
include ../Makefile.rules


IFLAGS = $(BASEINCPATH) \
	 $(OPENCVINCPATH) \

LFLAGS =  $(BASELIBPATH) \
	  $(OPENCVLIBPATH) $(LIBOCVHIGHGUI) $(LIBOCVCALIB3D) $(LIBOCVIMGPROC) $(LIBOCOBJDETECT)  $(LIBOCVCORE) $(LIBOCVEXTRA)\
-lprstimer

OBJECTS = Convolution.o

.PHONY: all
all: lib exe
lib: $(OBJECTS) convolution.a
exe: 


Convolution.o: Convolution.h Convolution.cpp
	@echo ${cc_green} Compile Convolution.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c Convolution.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

convolution.a : ${OBJECTS}
	@echo ${cc_green} Archive...${cc_nocolor}
	ar rcs libconvolution.a ${OBJECTS}
	@mv libconvolution.a ${LIBPATH}
	@echo ${cc_green}...Done${cc_nocolor}

.PHONY: clean
clean:
	@rm -rvf *.o




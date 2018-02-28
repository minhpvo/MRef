
include ../Makefile.rules


IFLAGS = $(BASEINCPATH) \
	 $(OPENCVINCPATH) \

LFLAGS =  $(BASELIBPATH) \
	  $(OPENCVLIBPATH) $(LIBOCVHIGHGUI) $(LIBOCVCALIB3D) $(LIBOCVIMGPROC) $(LIBOCOBJDETECT)  $(LIBOCVCORE) $(LIBOCVEXTRA)\

OBJECTS = graph.o lbp_graph.o icm_graph.o

all: lib exe 
lib: $(OBJECTS) libgraphmrf.a
exe:

.PHONY:  all

graph.o: graph.h graph.cpp
	@echo ${cc_green} Compile graph.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c graph.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

lbp_graph.o: lbp_graph.h lbp_graph.cpp
	@echo ${cc_green} Compile lbp_graph.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c lbp_graph.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

icm_graph.o: icm_graph.h icm_graph.cpp
	@echo ${cc_green} Compile icm_graph.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c icm_graph.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

libgraphmrf.a : ${OBJECTS}
	@echo ${cc_green} Archive...${cc_nocolor}
	ar rcs libgraphmrf.a ${OBJECTS}
	@mv libgraphmrf.a ${LIBPATH}
	@echo ${cc_green}...Done${cc_nocolor}


.PHONY: clean
clean:
	@rm -rvf *.o




include ../Makefile.rules


IFLAGS = $(BASEINCPATH) \
	 $(OPENCVINCPATH) \
	 $(EIGENINCPATH) \
	 $(EMBREEINCPATH)\
	 $(OPENMESHINCPATH) \
	 $(BOOSTINCPATH)

LFLAGS =  $(BASELIBPATH) -lprstimer -lconvolution -liolist -lmeshmrf -lgraphmrf -lprsori -llikelihoodimage -lraytracer -lmeshutil  -lmeshmetadata -lfdutil -lcontrol \
	  $(EMBREELIBPATH) \
	  $(OPENCVLIBPATH) $(LIBOCVHIGHGUI) $(LIBOCVCALIB3D) $(LIBOCVIMGPROC) $(LIBOCOBJDETECT)  $(LIBOCVCORE) $(LIBOCVEXTRA)\
	  $(BOOSTFILESYSTEMLIBPATH) \
	  $(EMBREELIBPATH)   \
	  $(OPENMESHLIBPATH)


OBJECTS = DataCopy.o  GradThinPlate.o DMDI2.o GradCalcStereo.o MeshScaler.o ViewSelector.o MeshRefine.o GradStraightEdges.o
# LabelVertFaceConvertion.o Mesh.o MeshConvertion.o

all: lib exe
lib: $(OBJECTS) libgradcalcstereo.a
test_grad: test_grad.o test_grad
exe: Consistency.o Consistency

.PHONY:  all

DataCopy.o: DataCopy.h DataCopy.cpp
	@echo ${cc_green} Compile DataCopy.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c DataCopy.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

MeshScaler.o: MeshScaler.h MeshScaler.cpp
	@echo ${cc_green} Compile MeshScaler.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c MeshScaler.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

MeshRefine.o: MeshRefine.h MeshRefine.cpp
	@echo ${cc_green} Compile MeshRefine.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c MeshRefine.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

LabelVertFaceConvertion.o: LabelVertFaceConvertion.h LabelVertFaceConvertion.cpp
	@echo ${cc_green} Compile LabelVertFaceConvertion.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c LabelVertFaceConvertion.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

GradStraightEdges.o: GradStraightEdges.h GradStraightEdges.cpp
	@echo ${cc_green} Compile GradStraightEdges.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c GradStraightEdges.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

ViewSelector.o: ViewSelector.h ViewSelector.cpp
	@echo ${cc_green} Compile ViewSelector.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c ViewSelector.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

GradThinPlate.o: GradThinPlate.h GradThinPlate.cpp
	@echo ${cc_green} Compile GradThinPlate.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c GradThinPlate.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

GradSemantic.o: GradSemantic.h GradSemantic.cpp
	@echo ${cc_green} Compile GradSemantic.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c GradSemantic.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

MeshConvertion.o: MeshConvertion.h MeshConvertion.cpp
	@echo ${cc_green} Compile MeshConvertion.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c MeshConvertion.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

DMDI2.o: DMDI2.h DMDI2.cpp
	@echo ${cc_green} Compile DMDI2.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c DMDI2.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

GradCalcStereo.o: GradCalcStereo.h GradCalcStereo.cpp
	@echo ${cc_green} Compile GradCalcStereo.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c GradCalcStereo.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

SemDataGrad.o: SemDataGrad.h SemDataGrad.cpp
	@echo ${cc_green} Compile SemDataGrad.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c SemDataGrad.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

Mesh.o: Mesh.h Mesh.cpp
	@echo ${cc_green} Compile Mesh.cpp ...  ${cc_nocolor}
	${CC} ${CFLAGS} -c Mesh.cpp ${IFLAGS}
	@echo ${cc_green}... DONE${cc_nocolor}

test_grad.o: test_grad.cpp
	@echo ${cc_green}Compile test_grad.cpp...${cc_nocolor}
	${CC} ${CFLAGS} -c test_grad.cpp ${IFLAGS}
	@echo ${cc_green}... Done ${cc_nocolor}

test_grad: test_grad.o ${OBJECTS}
	@echo ${cc_green}Link test_grad...${cc_nocolor}
	${CC} ${CCFLAGS} -o meshrefine test_grad.o ${OBJECTS} ${LFLAGS}
	@mv meshrefine ../bin
	@echo ${cc_green}...Done ${cc_nocolor}

Consistency.o: Consistency.cpp
	@echo ${cc_green}Compile Consistency.cpp...${cc_nocolor}
	${CC} ${CFLAGS} -c Consistency.cpp ${IFLAGS}
	@echo ${cc_green}... Done ${cc_nocolor}

Consistency: Consistency.o ${OBJECTS}
	@echo ${cc_green}Link Consistency...${cc_nocolor}
	${CC} ${CCFLAGS} -o consistency Consistency.o ${OBJECTS} ${LFLAGS}
	@mv consistency ../bin
	@echo ${cc_green}...Done ${cc_nocolor}

libgradcalcstereo.a : ${OBJECTS}
	@echo ${cc_green} Archive...${cc_nocolor}
	ar rcs libcalcgradstereo.a ${OBJECTS}
	@mv libcalcgradstereo.a ${LIBPATH}
	@echo ${cc_green}...Done${cc_nocolor}


.PHONY: clean
clean:
	@rm -rvf *.o



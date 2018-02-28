include ./Makefile.rules

TARGETS =  	IOList \
		PRSTimer \
		MeshMetaData \
		FileSystemUtil \
		MeshUtil \
		Convolution \
		PRSTimer \
		Ori \
		GraphMRF \
		LikelihoodImage \
		RayTracer \
		MeshRefine \
		MeshClassify \
		TfwUtil \
		MeshMetaData \
		ControlFile

.PHONY: all lib exe $(TARGETS)

all: dir lib exe
lib: ${TARGETS}
lib: MODE = lib
exe: ${TARGETS}
exe: MODE = exe

dir:
	@mkdir -p ./lib
	@mkdir -p ./bin

TfwUtil:
	@echo ${cc_blue} Building TfwUtil...${cc_nocolor}
	@make -C ./TfwUtil $(MODE)
	@echo ${cc_blue} ...Done${cc_nocolor}
	@echo

MeshUtil:
	@echo ${cc_blue} Building MeshUtil...${cc_nocolor}
	@make -C ./MeshUtil $(MODE)
	@echo ${cc_blue} ...Done${cc_nocolor}
	@echo

MeshMetaData:
	@echo ${cc_blue} Building MeshMetaData...${cc_nocolor}
	@make -C ./MeshMetaData $(MODE) 
	@echo ${cc_blue} ...Done${cc_nocolor}
	@echo

FileSystemUtil:
	@echo ${cc_blue} Building FileSystemUtil...${cc_nocolor}
	@make -C ./FileSystemUtil $(MODE)
	@echo ${cc_blue} ...Done${cc_nocolor}
	@echo

PRSTimer:
	@echo ${cc_blue} Building PRSTimer...${cc_nocolor}
	@make -C ./PRSTimer $(MODE)
	@echo ${cc_blue} ...Done${cc_nocolor}
	@echo

Ori:
	@echo ${cc_blue} Building Ori...${cc_nocolor}
	@make -C ./Ori $(MODE)
	@echo ${cc_blue} ...Done${cc_nocolor}
	@echo

GraphMRF:
	@echo ${cc_blue} Building GraphMRF...${cc_nocolor}
	@make -C ./GraphMRF $(MODE)
	@echo ${cc_blue} ...Done${cc_nocolor}
	@echo

Convolution:
	@echo ${cc_blue} Building Convolution...${cc_nocolor}
	@make -C ./Convolution $(MODE)
	@echo ${cc_blue} ...Done${cc_nocolor}
	@echo

IOList:
	@echo ${cc_blue} Building IOList...${cc_nocolor}
	@make -C ./IOList ${MODE} 
	@echo ${cc_blue} ...Done${cc_nocolor}
	@echo

LikelihoodImage:
	@echo ${cc_blue} Building LikelihoodImage...${cc_nocolor}
	@make -C ./LikelihoodImage ${MODE}
	@echo ${cc_blue} ...Done${cc_nocolor}
	@echo

RayTracer:
	@echo ${cc_blue} Building RayTracer...${cc_nocolor}
	@make -C ./RayTracer ${MODE}
	@echo ${cc_blue} ...Done${cc_nocolor}
	@echo

MeshClassify:
	@echo ${cc_blue} Building MeshClassify...${cc_nocolor}
	@make -C ./MeshClassify ${MODE}
	@echo ${cc_blue} ...Done${cc_nocolor}
	@echo

MeshRefine:
	@echo ${cc_blue} Building MeshRefine...${cc_nocolor}
	@make -C ./MeshRefine ${MODE}
	@echo ${cc_blue} ...Done${cc_nocolor}
	@echo

ControlFile:
	@echo ${cc_blue} Building ControlFile...${cc_nocolor}
	@make -C ./ControlFile ${MODE}
	@echo ${cc_blue} ...Done${cc_nocolor}
	@echo


#.PHONY: clean
clean:
	@make -C ./MeshUtil clean
	@make -C ./PRSTimer clean
	@make -C ./Ori clean
	@make -C ./GraphMRF clean
	@make -C ./IOList clean
	@make -C ./LikelihoodImage clean
	@make -C ./RayTracer clean
	@make -C ./MeshClassify clean
	@make -C ./TfwUtil clean
	@make -C ./MeshMetaData clean
	@make -C ./FileSystemUtil clean
	@make -C ./Convolution clean
	@make -C ./MeshRefine clean
	@make -C ./ControlFile clean

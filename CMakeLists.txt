cmake_minimum_required(VERSION 2.8)
project( SemanticMeshRef )

if (CMAKE_BUILD_TYPE STREQUAL Debug)
    set(BUILD_POSTFIX "_d")
else(CMAKE_BUILD_TYPE STREQUAL Debug)
    set(BUILD_POSTFIX "")
endif(CMAKE_BUILD_TYPE STREQUAL Debug)

option(OPENMP_ENABLED "Whether to enable OpenMP" ON)
option(LTO_ENABLED "Whether to enable link-time optimization" ON)

if(OPENMP_ENABLED)
    find_package(OpenMP QUIET)
endif()

find_package( OpenCV REQUIRED )

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
include_directories(${Boost_LIBRARY_DIR})

find_package(OpenMesh REQUIRED)
include_directories(${OpenMesh_INCLUDE_DIR})
include_directories(${OpenMesh_LIBRARY_DIR})


IF(WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /bigobj /openmp") 
	
	INCLUDE_DIRECTORIES(ThirdParty/vlfeat)
    link_directories(ThirdParty/vlfeat/bin/win64)
	
	add_definitions(-DUSESIFTGPU)
	add_definitions(-DNOMINMAX)
	add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES)	
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)	
	
	include_directories(Audio)
	include_directories(Geometry)
	include_directories(ImgPro)
	include_directories(TrackMatch)
	include_directories(Ulti)
	include_directories(Drivers)
	include_directories(Vis)

	add_subdirectory(Audio)
	add_subdirectory(Geometry)
	add_subdirectory(ImgPro)
	add_subdirectory(TrackMatch)
	add_subdirectory(Ulti)
	add_subdirectory(Drivers)
	add_subdirectory(Vis)
	
	add_executable( SemanticMeshRef main.cpp)
	
	target_link_libraries( SemanticMeshRef ${OpenCV_LIBS}  ${CERES_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARY} ${GLUT_LIBRARY} vl.lib 
		Audio Geometry ImgPro TrackMatch Ulti Drivers Vis)
		
	set_target_properties(Audio PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
	set_target_properties(Audio PROPERTIES OUTPUT_NAME Audio${BUILD_POSTFIX})
	set_target_properties(Geometry PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
	set_target_properties(Geometry PROPERTIES OUTPUT_NAME Geometry${BUILD_POSTFIX})
	set_target_properties(ImgPro PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
	set_target_properties(ImgPro PROPERTIES OUTPUT_NAME ImgPro${BUILD_POSTFIX})
	set_target_properties(TrackMatch PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
	set_target_properties(TrackMatch PROPERTIES OUTPUT_NAME TrackMatch${BUILD_POSTFIX})
	set_target_properties(Ulti PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
	set_target_properties(Ulti PROPERTIES OUTPUT_NAME Ulti${BUILD_POSTFIX})
	set_target_properties(Drivers PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
	set_target_properties(Drivers PROPERTIES OUTPUT_NAME Drivers${BUILD_POSTFIX})
	set_target_properties(Vis PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
	set_target_properties(Vis PROPERTIES OUTPUT_NAME Vis${BUILD_POSTFIX})
		
elseif(UNIX) 
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -fpermissive -fopenmp -std=c++11 -O4")
	
    INCLUDE_DIRECTORIES(/home/mpvo/Research/DevSoft/SemanticMeshRef/ThirdParty/vlfeat)
    link_directories(/home/mpvo/Research/DevSoft/SemanticMeshRef/ThirdParty/vlfeat/bin/glnxa64)
	
	add_definitions(-DSCHEDULER)
	add_definitions(-DUSESIFTGPU)
	add_definitions(-DEC2=1)
	
	add_executable( SemanticMeshRef 	main.cpp
							Audio/AudioPro.cpp
							ImgPro/ImagePro.cpp
							Geometry/Geometry1.cpp
							Geometry/Geometry2.cpp
							TrackMatch/FeatureEst.cpp
							TrackMatch/MatchingTracking.cpp
							Ulti/MiscAlgo.cpp
							Ulti/DataIO.cpp
							Ulti/MathUlti.cpp
							Ulti/GeneralUlti.cpp
							Drivers/Drivers.cpp
							Vis/Visualization.cpp)
							 
	target_link_libraries( SemanticMeshRef ${OpenCV_LIBS}  ${CERES_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARY} ${GLUT_LIBRARY} libvl.so -ldl)
ENDIF()

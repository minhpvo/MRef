add_library(MeshRefine DataCopy.cpp DMDI2.cpp GradCalcStereo.cpp GradStraightEdges.cpp GradThinPlate.cpp MeshRefine.cpp MeshScaler.cpp ViewSelector.cpp)

set_target_properties(MeshRefine PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set_target_properties(MeshRefine PROPERTIES OUTPUT_NAME MeshRefine${BUILD_POSTFIX})

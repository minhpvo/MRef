add_library(MeshClassify MeshMRF.cpp)

set_target_properties(MeshClassify PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set_target_properties(MeshClassify PROPERTIES OUTPUT_NAME MeshClassify${BUILD_POSTFIX})

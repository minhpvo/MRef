add_library(MeshUtil MeshConv.cpp MeshDivide.cpp MeshFilter.cpp MeshGeom.cpp MeshIO.cpp MeshSimplify.cpp)

set_target_properties(MeshUtil PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set_target_properties(MeshUtil PROPERTIES OUTPUT_NAME MeshUtil${BUILD_POSTFIX})

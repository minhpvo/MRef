add_library(MeshMetaData MeshMetaData.cpp)

set_target_properties(MeshMetaData PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set_target_properties(MeshMetaData PROPERTIES OUTPUT_NAME MeshMetaData${BUILD_POSTFIX})

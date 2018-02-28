add_library(Ori Orientation.cpp OriGeom.cpp)

set_target_properties(Ori PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set_target_properties(Ori PROPERTIES OUTPUT_NAME Ori${BUILD_POSTFIX})

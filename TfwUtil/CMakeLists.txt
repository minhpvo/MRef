add_library(TfwUtil TfwIO.cpp)

set_target_properties(TfwUtil PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set_target_properties(TfwUtil PROPERTIES OUTPUT_NAME TfwUtil${BUILD_POSTFIX})

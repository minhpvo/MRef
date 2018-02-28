add_library(Convolution Convolution.cpp)

set_target_properties(Convolution PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set_target_properties(Convolution PROPERTIES OUTPUT_NAME Convolution${BUILD_POSTFIX})

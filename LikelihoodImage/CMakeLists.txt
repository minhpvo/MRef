add_library(LikelihoodImage LikelihoodImage.cpp)

set_target_properties(LikelihoodImage PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set_target_properties(LikelihoodImage PROPERTIES OUTPUT_NAME LikelihoodImage${BUILD_POSTFIX})

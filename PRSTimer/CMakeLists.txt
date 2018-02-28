add_library(PRSTimer PRSTimer.cpp)

set_target_properties(PRSTimer PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set_target_properties(PRSTimer PROPERTIES OUTPUT_NAME PRSTimer${BUILD_POSTFIX})

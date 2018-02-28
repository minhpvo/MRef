add_library(RayTracer RayTracer.cpp)

set_target_properties(RayTracer PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set_target_properties(RayTracer PROPERTIES OUTPUT_NAME RayTracer${BUILD_POSTFIX})

add_library(GraphMRF graph.cpp icm_graph.cpp lbp_graph.cpp)

set_target_properties(GraphMRF PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set_target_properties(GraphMRF PROPERTIES OUTPUT_NAME GraphMRF${BUILD_POSTFIX})

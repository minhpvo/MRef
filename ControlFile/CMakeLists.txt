add_library(ControlFile ControlReader.cpp ControlRefine.cpp ControlWriter.cpp)

set_target_properties(ControlFile PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set_target_properties(ControlFile PROPERTIES OUTPUT_NAME ControlFile${BUILD_POSTFIX})

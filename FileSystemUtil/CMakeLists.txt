add_library(FileSystemUtil FDUtil.cpp)

set_target_properties(FileSystemUtil PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set_target_properties(FileSystemUtil PROPERTIES OUTPUT_NAME FileSystemUtil${BUILD_POSTFIX})

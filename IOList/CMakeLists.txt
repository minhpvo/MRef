add_library(IOList IOList.cpp)

set_target_properties(IOList PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set_target_properties(IOList PROPERTIES OUTPUT_NAME IOList${BUILD_POSTFIX})

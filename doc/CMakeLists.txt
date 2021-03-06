
set(BUILD_DOCS FALSE CACHE BOOL "Build documentation")
if(NOT BUILD_DOCS)
if(NOT BUILD_ONLY_DOCS)
    return()
endif()
endif()

message(STATUS "Building docs. Use target: make docs")
find_package(Doxygen REQUIRED)
find_package(Sphinx REQUIRED)


# Find all the public headers
file(GLOB_RECURSE FARM_NG_PUBLIC_HEADERS ${PROJECT_SOURCE_DIR}/modules/*/cpp/**.h)

#This will be the main output of our command
set(DOXYGEN_INDEX_FILE ${CMAKE_CURRENT_SOURCE_DIR}/html/index.html)

set(DOXYGEN_INPUT_DIR "")
foreach(_module ${_modules})
 set(DOXYGEN_INPUT_DIR "${DOXYGEN_INPUT_DIR} ${PROJECT_SOURCE_DIR}/modules/${_module}/cpp/farm_ng/${_module}")
endforeach()

set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/doxygen)
set(DOXYGEN_INDEX_FILE ${DOXYGEN_OUTPUT_DIR}/html/index.html)
set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

#Replace variables inside @@ with the current values
configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR}) #Doxygen won't create this for us
add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
                   DEPENDS ${FARM_NG_PUBLIC_HEADERS}
                   COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
                   MAIN_DEPENDENCY ${DOXYFILE_OUT} ${DOXYFILE_IN}
                   COMMENT "Generating docs")

add_custom_target(Doxygen DEPENDS ${DOXYGEN_INDEX_FILE})
set_target_properties(Doxygen PROPERTIES EXCLUDE_FROM_ALL TRUE)



set(SPHINX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR})
set(SPHINX_BUILD ${CMAKE_CURRENT_BINARY_DIR}/sphinx)

add_custom_target(docs
                  COMMAND
                  ${SPHINX_EXECUTABLE} -b html
		  # Tell Breathe where to find the Doxygen output
                  -Dbreathe_projects.farm_ng=${DOXYGEN_OUTPUT_DIR}/xml
                  ${SPHINX_SOURCE} ${SPHINX_BUILD}
                  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                  COMMENT "Generating documentation with Sphinx"
		  DEPENDS ${DOXYGEN_INDEX_FILE})
set_target_properties(docs PROPERTIES EXCLUDE_FROM_ALL TRUE)

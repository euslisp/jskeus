# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jskeus)

find_package(euslisp REQUIRED)
message(STATUS "ARCHDIR=${ARCHDIR}")
message(STATUS "EUSDIR=${EUSDIR}")

execute_process(COMMAND grep version ${PROJECT_SOURCE_DIR}/package.xml
                COMMAND sed -e s/[^0-9.]//g
                OUTPUT_VARIABLE jskeus_VERSION
                OUTPUT_STRIP_TRAILING_WHITESPACE)
# build irteus on install command
set(INSTALLDIR \${DESTDIR}${CMAKE_INSTALL_PREFIX}/share/euslisp/jskeus/eus/)
add_custom_command(OUTPUT ${INSTALLDIR}
  COMMAND cmake -E make_directory ${INSTALLDIR}/lib
  COMMAND cmake -E make_directory ${INSTALLDIR}/${ARCHDIR}/bin
  COMMAND cmake -E make_directory ${INSTALLDIR}/${ARCHDIR}/obj
  COMMAND cmake -E make_directory ${INSTALLDIR}/${ARCHDIR}/lib)
add_custom_target(compile_jskeus
  DEPENDS ${INSTALLDIR}
  COMMAND export EUSDIR=${EUSDIR} DISPLAY="" lt_cv_sys_lib_dlsearch_path_spec=${lt_cv_sys_lib_dlsearch_path_spec} && \$\(MAKE\) -j1 -l1 -C ${PROJECT_SOURCE_DIR}/irteus -f Makefile ARCHDIR=${ARCHDIR} EUSDIR=${EUSDIR} IRTEUSDIR=${INSTALLDIR} 'SVNVERSION=\\"${jskeus_VERSION}\\"' VERBOSE=1)

install(CODE "execute_process(COMMAND \"${CMAKE_COMMAND}\" --build \"${CMAKE_BINARY_DIR}\" --target compile_jskeus RESULT_VARIABLE result_variable OUTPUT_VARIABLE output_variable)
if(NOT \${result_variable} EQUAL 0)
  message(FATAL_ERROR \"\${output_variable}\")
endif()")

install(DIRECTORY doc
   DESTINATION share/jskeus/
   USE_SOURCE_PERMISSIONS)
install(DIRECTORY irteus
   DESTINATION share/euslisp/jskeus/
   USE_SOURCE_PERMISSIONS)
install(FILES package.xml DESTINATION share/jskeus/)

add_custom_target(test) # dummy target for catkin test

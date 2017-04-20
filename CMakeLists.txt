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
message(STATUS "euslisp_VERSION : ${euslisp_VERSION}")
message(STATUS "jskeus_VERSION  : ${jskeus_VERSION}")

# build irteus
# if EUSDIR is write protected use share as target directory
execute_process(COMMAND test -w ${EUSDIR} RESULT_VARIABLE eusdir_WRITABLE)
if(${eusdir_WRITABLE} EQUAL 0)
  message(STATUS "eusdir_WRITABLE : TRUE")
  set(INSTALLDIR ${EUSDIR}) # install to EUSDIR
else()
  message(STATUS "eusdir_WRITABLE : FALSE")
  set(INSTALLDIR $ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/euslisp) # install to prefix/eusdir
  message(STATUS "INSTALLDIR ${INSTALLDIR}")
  execute_process(
    COMMAND cmake -E make_directory ${INSTALLDIR}
    COMMAND cmake -E copy_directory ${EUSDIR}/lib ${INSTALLDIR}/lib
    COMMAND cmake -E copy_directory ${EUSDIR}/${ARCHDIR} ${INSTALLDIR}/${ARCHDIR})
endif()
add_custom_target(compile_jskeus ALL
  COMMAND export EUSDIR=${EUSDIR} DISPLAY="" lt_cv_sys_lib_dlsearch_path_spec=${lt_cv_sys_lib_dlsearch_path_spec} && \$\(MAKE\) -j1 -l1 -C ${PROJECT_SOURCE_DIR}/irteus -f Makefile ARCHDIR=${ARCHDIR} EUSDIR=${EUSDIR} IRTEUSDIR=${INSTALLDIR} 'SVNVERSION=\\"${jskeus_VERSION}\\"' VERBOSE=1
  COMMAND cmake -E remove -f ${INSTALLDIR}/irteus
  COMMAND cmake -E create_symlink ${PROJECT_SOURCE_DIR}/irteus ${INSTALLDIR}/irteus
)

# catkin
find_package(catkin)
catkin_package()
if(${eusdir_WRITABLE} EQUAL 0) # EUSDIR IS WRITABLE
else()                         # EUSDIR IS NOT WRITABLE
  catkin_add_env_hooks(99.jskeus SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
endif()

# install
install(DIRECTORY doc irteus
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
install(CODE "execute_process(COMMAND cmake -E remove -f ${INSTALLDIR}/irteus)")
install(CODE "execute_process(COMMAND cmake -E create_symlink \${DESTDIR}${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/irteus ${INSTALLDIR}/irteus)")

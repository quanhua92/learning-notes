cmake_minimum_required(VERSION 2.8)

project(learn_cmake)

MACRO(SUBDIRLIST result curdir)
 FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
 SET(dirlist "")
 FOREACH (child ${children})
   IF (IS_DIRECTORY ${curdir}/${child})
     LIST(APPEND dirlist ${child})
   ENDIF()
 ENDFOREACH()
 SET(${result} ${dirlist})
ENDMACRO()

# find the subdirectories in src
SUBDIRLIST(SRC_DIRS ${PROJECT_SOURCE_DIR}/src)

# add subdirectories
FOREACH (subdir ${SRC_DIRS})
 message(STATUS "adding subdir: " ${subdir})
 add_subdirectory(src/${subdir})
ENDFOREACH()

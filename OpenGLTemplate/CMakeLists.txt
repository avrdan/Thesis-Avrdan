set(CMAKE_BUILD_TYPE Debug)
project(OpenGLTemplate)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
file(GLOB_RECURSE RES_FILES *.frag *.vert *.glsl *.cpp *.h)
add_executable(${PROJECT_NAME} ${SRC_LIST} ${RES_FILES})


set(INCLUDE_GFLW_DIRS "opt/local/include/GLFW")
set(INCLUDE_GLM_DIRS "opt/local/include/glm")
set(LIB_DIRS "opt/local/lib")

find_package(OpenGL REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES})

#glew
find_package( GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${GLEW_LIBRARIES})

#glfw
find_package( GLFW REQUIRED )
include_directories(${GLFW_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${GLFW_LIBRARY})

#glm
include_directories(${GLM_INCLUDE_DIR})

#add_subdirectory("../${PROJECT_NAME}/common")
#target_link_libraries(${PROJECT_NAME} common)
#include_directories( ${SRC_LIST})
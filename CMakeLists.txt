cmake_minimum_required(VERSION 3.16.3)

set(PROJECT_NAME MeshOptimization)

project(${PROJECT_NAME})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

set (MO_SOURCE_FILES
     code/main.cpp
     code/common.h
     code/common.cpp
     code/allocator.h
     code/allocator.cpp
     code/string.h
     code/string.cpp
     code/threadpool.h
     code/threadpool.cpp
     code/random.h
     code/random.cpp
     code/vector.h
     code/vector.cpp
     code/matrix.h
     code/matrix.cpp
     code/aabb.h
     code/aabb.cpp
     code/bvh.h
     code/bvh.cpp
     code/geometry_algorithm.h
     code/geometry_algorithm.cpp
     code/filebuffer.h
     code/filebuffer.cpp
     code/obj.h
     code/obj.cpp
     code/mesh.h
     code/mesh.cpp
     code/mesh_test.h
     code/mesh_test.cpp
     code/meshopt.h
     code/meshopt.cpp
     code/meshopt_internal.h
     code/meshopt_common.cpp
     code/meshopt_projection.cpp
     code/meshopt_localfit.cpp
     code/meshopt_globalfit.cpp
     code/meshopt_simpliciesfit.cpp
     )

# AMPL boolean variable
if (WIN32)
# If you don't want to use AMPL, then just set it as FALSE
# I currently turn off AMPL solver here
set(USE_AMPL_SOLVER FALSE) 
else()
set(USE_AMPL_SOLVER FALSE)
endif()

if (USE_AMPL_SOLVER)
list(APPEND ${MO_SOURCE_FILES}
     code/ampl_test.h
     code/ampl_test.cpp
     code/ampl_solver.h
     code/ampl_solver.cpp)
endif()

source_group(src FILES ${MO_SOURCE_FILES})

# tinyobjloader
set(TINYOBJLOADER_PATH dependency/tinyobjloader)
set(TINYOBJLOADER_FILES ${TINYOBJLOADER_PATH}/tiny_obj_loader.h)
source_group(lib\\tinyobjloader FILES ${TINYOBJLOADER_FILES})

# ampl
if (USE_AMPL_SOLVER)
set(AMPL_PATH dependency/ampl)
file(GLOB_RECURSE AMPL_FILES ${AMPL_PATH}/include/ampl/*.h)
source_group(lib\\ampl FILES ${AMPL_FILES})
else()
set(AMPL_PATH "")
endif()

add_executable(${PROJECT_NAME} 
               ${MO_SOURCE_FILES}
               ${TINYOBJLOADER_FILES}
               ${AMPL_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC ${TINYOBJLOADER_PATH})

if (USE_AMPL_SOLVER)
target_compile_definitions(${PROJECT_NAME} PUBLIC USE_AMPL_SOLVER)
target_include_directories(${PROJECT_NAME} PUBLIC ${AMPL_PATH}/include)
target_link_libraries(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/${AMPL_PATH}/lib/ampl-2.0.12.lib)
endif()

set(ADDITIONAL_OUTPUT_PATH "")
if (WIN32 AND MSVC)
    set(ADDITIONAL_OUTPUT_PATH "$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>")

set_property(TARGET ${PROJECT_NAME}
                APPEND
                PROPERTY COMPILE_OPTIONS /utf-8)
set_property(TARGET ${PROJECT_NAME}
                APPEND
                PROPERTY VS_DEBUGGER_WORKING_DIRECTORY
                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${ADDITIONAL_OUTPUT_PATH}")
endif()

if (USE_AMPL_SOLVER)
install(FILES ${AMPL_PATH}/lib/ampl-2.0.12.dll DESTINATION  "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${ADDITIONAL_OUTPUT_PATH}")
endif()


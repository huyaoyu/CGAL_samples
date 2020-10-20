cmake_minimum_required(VERSION 3.17)
project(CGALBlog)

set(CMAKE_CXX_STANDARD 17)

# Boost.
FIND_LIBRARY( LIB_BOOST_FILESYSTEM NAMES boost_filesystem )
FIND_LIBRARY( LIB_BOOST_SYSTEM NAMES boost_system )
message(STATUS "LIB_BOOST_FILESYSTEM=${LIB_BOOST_FILESYSTEM}")
message(STATUS "LIB_BOOST_SYSTEM=${LIB_BOOST_SYSTEM}")
set( LINKED_BOOST_LIBRARIES ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} )

# CGAL.
set(CGAL_DIR $ENV{CGAL_DIR})
find_package(CGAL REQUIRED COMPONENTS Qt5 OPTIONAL_COMPONENTS Core )
message(STATUS "CGAL_LIBRARIES=${CGAL_LIBRARIES}")

if(CGAL_Qt5_FOUND)
    add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
endif()

include_directories( includes )

set(EXE_READ_WRITE_PLY_POINT_CLOUD ReadWritePLYPointCloud)
add_executable(EXE_READ_WRITE_PLY_POINT_CLOUD ReadWritePLYPointCloud.cpp)
target_link_libraries(EXE_READ_WRITE_PLY_POINT_CLOUD
        ${LINKED_BOOST_LIBRARIES}
        ${CGAL_LIBRARIES} CGAL_Qt5)

set(EXE_READ_WRITE_PLY_MESH ReadPLYMesh)
add_executable(EXE_READ_WRITE_PLY_MESH ReadWritePLYMesh.cpp)
target_link_libraries(EXE_READ_WRITE_PLY_MESH
        ${LINKED_BOOST_LIBRARIES}
        ${CGAL_LIBRARIES} CGAL_Qt5)

set(EXE_SURFACE_NORMAL SurfaceNormal)
add_executable(EXE_SURFACE_NORMAL SurfaceNormal.cpp)
target_link_libraries(EXE_SURFACE_NORMAL
        ${LINKED_BOOST_LIBRARIES}
        ${CGAL_LIBRARIES} CGAL_Qt5)

set(EXE_REPAIR Repair)
add_executable(EXE_REPAIR Repair.cpp)
target_link_libraries(EXE_REPAIR
        ${LINKED_BOOST_LIBRARIES}
        ${CGAL_LIBRARIES} CGAL_Qt5)
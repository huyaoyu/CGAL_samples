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

# Eigen.
find_package(Eigen3 REQUIRED NO_MODULE)

include_directories( includes )

set(EXE_READ_WRITE_PLY_POINT_CLOUD ReadWritePLYPointCloud)
add_executable(${EXE_READ_WRITE_PLY_POINT_CLOUD} ReadWritePLYPointCloud.cpp)
target_link_libraries(${EXE_READ_WRITE_PLY_POINT_CLOUD}
        ${LINKED_BOOST_LIBRARIES}
        ${CGAL_LIBRARIES} CGAL_Qt5)

set(EXE_READ_WRITE_PLY_MESH ReadPLYMesh)
add_executable(${EXE_READ_WRITE_PLY_MESH} ReadWritePLYMesh.cpp)
target_link_libraries(${EXE_READ_WRITE_PLY_MESH}
        ${LINKED_BOOST_LIBRARIES}
        ${CGAL_LIBRARIES} CGAL_Qt5)

set(EXE_SURFACE_NORMAL SurfaceNormal)
add_executable(${EXE_SURFACE_NORMAL} SurfaceNormal.cpp)
target_link_libraries(${EXE_SURFACE_NORMAL}
        ${LINKED_BOOST_LIBRARIES}
        ${CGAL_LIBRARIES} CGAL_Qt5)

set(EXE_REPAIR Repair)
add_executable(${EXE_REPAIR} Repair.cpp)
target_link_libraries(${EXE_REPAIR}
        ${LINKED_BOOST_LIBRARIES}
        ${CGAL_LIBRARIES} CGAL_Qt5)

set(EXE_AFS AFSReconstruction)
add_executable(${EXE_AFS} AFSReconstruction.cpp)
target_link_libraries(${EXE_AFS}
        ${LINKED_BOOST_LIBRARIES}
        ${CGAL_LIBRARIES} CGAL_Qt5)

set(EXE_ISOTROPIC_REMESHING IsoTropicRemeshing)
add_executable(${EXE_ISOTROPIC_REMESHING} IsotropicRemeshing.cpp)
target_link_libraries(${EXE_ISOTROPIC_REMESHING}
        ${LINKED_BOOST_LIBRARIES}
        ${CGAL_LIBRARIES} CGAL_Qt5)

set(EXE_HOLE_FILLING HoleFilling)
add_executable(${EXE_HOLE_FILLING} HoleFilling.cpp)
target_link_libraries(${EXE_HOLE_FILLING}
        ${LINKED_BOOST_LIBRARIES}
        ${CGAL_LIBRARIES} CGAL_Qt5)

set(EXE_RAY_SHOOTING RayShooting)
add_executable(${EXE_RAY_SHOOTING} RayShooting.cpp)
target_link_libraries(${EXE_RAY_SHOOTING}
        ${LINKED_BOOST_LIBRARIES}
        ${CGAL_LIBRARIES} CGAL_Qt5
        Eigen3::Eigen)

set(EXE_PC_SHAPE_DETECTION PCShapeDetection)
add_executable(${EXE_PC_SHAPE_DETECTION} PointCloudShapeDetection.cpp)
target_link_libraries(${EXE_PC_SHAPE_DETECTION}
        ${LINKED_BOOST_LIBRARIES}
        ${CGAL_LIBRARIES} CGAL_Qt5)